# -*- coding: utf-8 -*-
"""
autonomous_mode.py — Controlador autónomo principal.

Estados internos:
  LANE_FOLLOWING   → seguimiento normal del carril.
  APPROACHING_STOP → señal STOP detectada, frenado progresivo.
  STOPPED_WAIT     → parado frente a la señal, contando 5 s.
  RESUMING         → aceleración gradual tras la pausa.
  PARKING          → delega en ParkingManeuver.
  OBSTACLE_HOLD    → obstáculo inesperado, parado hasta que desaparece.

Reglas TMR 2026 implementadas:
  * Velocidad modulada: aceleración en rectas, reducción en curvas.
  * Señal STOP: frenar a ≤30 cm, esperar 5 s, reanudar.
  * LED parpadeante durante la pausa de STOP.
  * Estacionamiento en batería con cinemática Ackermann.
"""

import time
from enum import Enum, auto
from typing import Optional

import RPi.GPIO as GPIO

from config import (
    SPEED_STRAIGHT, SPEED_CURVE, SPEED_APPROACH,
    STOP_BRAKE_START_MM, STOP_TARGET_MM, STOP_TOLERANCE_MM,
    STOP_WAIT_SEC, STOP_LED_BLINK_HZ,
    EMERGENCY_STOP_MM,
    STEER_KP, STEER_KI, STEER_KD,
    VEL_STOP_KP, VEL_STOP_KI, VEL_STOP_KD,
    SERVO_CENTER_ANGLE,
    PIN_LED_STOP, PIN_LED_STATUS,
)
from control.pid_controller import PIDController
from vision.lane_detector import LaneData
from vision.object_detector import ObjectDetector
from autonomy.parking_maneuver import ParkingManeuver, ParkingState


class AutoState(Enum):
    LANE_FOLLOWING   = auto()
    APPROACHING_STOP = auto()
    STOPPED_WAIT     = auto()
    RESUMING         = auto()
    PARKING          = auto()
    OBSTACLE_HOLD    = auto()


class AutonomousController:
    """
    Controlador autónomo completo para el TMR 2026.

    Responsabilidades:
      - Mantener el coche centrado en el carril (PID de dirección).
      - Ajustar velocidad según curvatura.
      - Ejecutar la secuencia de STOP.
      - Delegar el estacionamiento en ParkingManeuver.
    """

    def __init__(self, motor, steering):
        self.motor    = motor
        self.steering = steering

        # PIDs
        self._steer_pid = PIDController(
            kp=STEER_KP, ki=STEER_KI, kd=STEER_KD,
            setpoint=0.0,
            output_limits=(-(SERVO_CENTER_ANGLE - 45), (SERVO_CENTER_ANGLE - 45)),
            integral_limits=(-20.0, 20.0),
        )

        self._stop_pid = PIDController(
            kp=VEL_STOP_KP, ki=VEL_STOP_KI, kd=VEL_STOP_KD,
            setpoint=STOP_TARGET_MM,
            output_limits=(0.0, SPEED_APPROACH),
            integral_limits=(-10.0, 10.0),
        )

        # Sub-FSM de estacionamiento
        self._parking = ParkingManeuver(gap_side="right")

        self._state = AutoState.LANE_FOLLOWING
        self._stop_wait_start: float = 0.0
        self._resume_start: float    = 0.0
        self._led_last_toggle: float = 0.0
        self._led_on: bool           = False

        # GPIOs para LEDs (ya configurados en main.py)
        self._pin_led_stop   = PIN_LED_STOP
        self._pin_led_status = PIN_LED_STATUS

    # ----------------------------------------------------------
    # API pública
    # ----------------------------------------------------------
    def activate(self):
        """Prepara el controlador para comenzar a conducir."""
        self._state = AutoState.LANE_FOLLOWING
        self._steer_pid.reset()
        self._stop_pid.reset()
        self._set_led(self._pin_led_status, True)
        self._set_led(self._pin_led_stop,   False)

    def deactivate(self):
        """Para todo al salir del modo autónomo."""
        self.motor.brake()
        self.steering.center()
        self._set_led(self._pin_led_status, False)
        self._set_led(self._pin_led_stop,   False)

    def trigger_parking(self):
        """Activa el reto de estacionamiento (llamar desde la FSM principal)."""
        if self._state == AutoState.LANE_FOLLOWING:
            self._parking.reset()
            self._parking.start()
            self._state = AutoState.PARKING
            print("[AUTO] Reto de estacionamiento activado.")

    def update(
        self,
        lane: LaneData,
        obj_result: ObjectDetector.AnalysisResult,
        tof_mm: Optional[float],
        dt: float,
    ):
        """
        Ciclo de control principal — llamar una vez por iteración del bucle.

        Parameters
        ----------
        lane       : LaneData         resultado del detector de carril
        obj_result : AnalysisResult   resultado del detector de objetos
        tof_mm     : float | None     distancia ToF en mm
        dt         : float            tiempo desde el último ciclo (s)
        """

        # ── Emergencia global: obstáculo muy cercano (FUERA de reversa) ──
        if (self._state not in (AutoState.PARKING,)
                and tof_mm is not None
                and tof_mm < EMERGENCY_STOP_MM):
            self._transition(AutoState.OBSTACLE_HOLD)

        match self._state:

            case AutoState.LANE_FOLLOWING:
                self._do_lane_following(lane, obj_result, tof_mm, dt)

            case AutoState.APPROACHING_STOP:
                self._do_approaching_stop(lane, tof_mm, dt)

            case AutoState.STOPPED_WAIT:
                self._do_stopped_wait()

            case AutoState.RESUMING:
                self._do_resuming(lane, dt)

            case AutoState.PARKING:
                park_state = self._parking.update(tof_mm, self.motor, self.steering)
                if park_state == ParkingState.PARKED:
                    self._transition(AutoState.LANE_FOLLOWING)
                elif park_state == ParkingState.ABORTED:
                    self._transition(AutoState.LANE_FOLLOWING)

            case AutoState.OBSTACLE_HOLD:
                self.motor.brake()
                if tof_mm is None or tof_mm >= EMERGENCY_STOP_MM + 50:
                    # El obstáculo se retiró — retomar
                    self._transition(AutoState.LANE_FOLLOWING)

    # ----------------------------------------------------------
    # Sub-estados de control
    # ----------------------------------------------------------
    def _do_lane_following(self, lane, obj_result, tof_mm, dt):
        """Seguimiento normal del carril con velocidad modulada."""

        # ── Verificar señal STOP ──
        if (obj_result.stop_sign_detected
                and obj_result.stop_sign_distance_mm is not None
                and obj_result.stop_sign_distance_mm < STOP_BRAKE_START_MM):
            self._transition(AutoState.APPROACHING_STOP)
            return

        # ── Verificar semáforo ──
        if (obj_result.traffic_light is not None
                and obj_result.traffic_light.color == "red"):
            self.motor.brake()
            self._apply_steering(lane, dt)
            return

        # ── Velocidad según curvatura ──
        speed = lane.suggested_speed

        # ── Dirección ──
        self._apply_steering(lane, dt)
        self.motor.set_throttle(speed)

    def _do_approaching_stop(self, lane, tof_mm, dt):
        """Frenado progresivo PID hasta STOP_TARGET_MM."""

        # Actualizar fuente de distancia: preferir ToF cuando esté en rango útil
        dist = tof_mm if (tof_mm is not None and tof_mm < 800) else None

        if dist is None:
            # Sin lectura de ToF — frenado abierto conservador
            self.motor.set_throttle(SPEED_APPROACH * 0.5)
            self._apply_steering(lane, dt)
            return

        error = dist - STOP_TARGET_MM   # + cuando aún lejos, − cuando pasado

        if error <= STOP_TOLERANCE_MM:
            # Llegamos al punto de parada
            self.motor.brake()
            self._transition(AutoState.STOPPED_WAIT)
            return

        # Velocidad proporcional a la distancia restante
        speed = self._stop_pid.compute(dist, dt)
        self.motor.set_throttle(max(speed, 5.0))  # mínimo 5% para vencer inercia
        self._apply_steering(lane, dt)

    def _do_stopped_wait(self):
        """Pausa de 5 s frente al STOP, con LED parpadeante."""
        self.motor.brake()
        self.steering.center()

        # Parpadeo LED
        now = time.monotonic()
        if now - self._led_last_toggle >= (1.0 / (2 * STOP_LED_BLINK_HZ)):
            self._led_on = not self._led_on
            self._set_led(self._pin_led_stop, self._led_on)
            self._led_last_toggle = now

        if now - self._stop_wait_start >= STOP_WAIT_SEC:
            self._set_led(self._pin_led_stop, False)
            self._transition(AutoState.RESUMING)
            print("[AUTO] Reanudando marcha tras STOP.")

    def _do_resuming(self, lane, dt):
        """Aceleración gradual tras la pausa del STOP."""
        RAMP_TIME = 1.5  # segundos para alcanzar velocidad normal
        elapsed   = time.monotonic() - self._resume_start
        t         = min(elapsed / RAMP_TIME, 1.0)
        speed     = SPEED_CURVE + t * (SPEED_STRAIGHT - SPEED_CURVE)

        self._apply_steering(lane, dt)
        self.motor.set_throttle(speed)

        if t >= 1.0:
            self._transition(AutoState.LANE_FOLLOWING)

    # ----------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------
    def _apply_steering(self, lane: LaneData, dt: float):
        """
        Aplica el PID de dirección.
        error_px positivo → coche a la izquierda → corrección a la derecha (ángulo >90°).
        """
        correction = self._steer_pid.compute(lane.error_px, dt)
        servo_angle = SERVO_CENTER_ANGLE + correction
        self.steering.set_angle(servo_angle)

    def _transition(self, new_state: AutoState):
        """Cambia de estado y registra marcas de tiempo relevantes."""
        print(f"[AUTO] {self._state.name} → {new_state.name}")
        self._state = new_state

        if new_state == AutoState.STOPPED_WAIT:
            self._stop_wait_start = time.monotonic()
            self._steer_pid.reset()

        elif new_state == AutoState.RESUMING:
            self._resume_start = time.monotonic()
            self._stop_pid.reset()

        elif new_state == AutoState.LANE_FOLLOWING:
            self._steer_pid.reset()

    @staticmethod
    def _set_led(pin: int, state: bool):
        try:
            GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)
        except Exception:
            pass   # Si el GPIO no está configurado, no crashear

    @property
    def current_state(self) -> AutoState:
        return self._state
