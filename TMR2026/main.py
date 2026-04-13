# -*- coding: utf-8 -*-
"""
main.py — Punto de entrada del sistema TMR 2026.

Máquina de estados principal:
  STANDBY      → Esperando conexión del mando Bluetooth.
  MANUAL       → Control manual con joystick (seguro por defecto).
  VISION_TEST  → Motores bloqueados, debug de visión en terminal.
  AUTONOMOUS   → Control total por IA + sensores.

Arquitectura de hilos:
  ┌─ Main thread ──────────────────────────────────────────────────┐
  │  FSM principal, lectura de gamepad, comandos a motor/servo.    │
  └────────────────────────────────────────────────────────────────┘
  ┌─ Camera thread (daemon) ───────────────────────────────────────┐
  │  Captura Picamera2, IMX500 NPU devuelve detecciones en meta.   │
  │  LaneDetector en CPU (~8 ms por frame).  Publica CameraFrame.  │
  └────────────────────────────────────────────────────────────────┘
  ┌─ ToF thread (daemon) ──────────────────────────────────────────┐
  │  VL53L0X a 50 Hz.  Actualiza distancia en DistanceSensor.      │
  └────────────────────────────────────────────────────────────────┘
"""

import sys
import time
import signal
import threading

import RPi.GPIO as GPIO

# ── Módulos del sistema ──
from config import (
    PIN_LED_STOP, PIN_LED_STATUS,
    SERVO_CENTER_ANGLE,
    BTN_BACK_TO_MANUAL, BTN_VISION_TEST, BTN_AUTONOMOUS,
)
from hardware.motor_driver   import MotorDriver
from hardware.steering_driver import SteeringDriver
from hardware.distance_sensor import DistanceSensor
from hardware.camera_manager  import CameraManager
from control.gamepad_reader   import GamepadReader
from vision.lane_detector     import LaneDetector, LaneData
from vision.object_detector   import ObjectDetector
from autonomy.autonomous_mode import AutonomousController


# ══════════════════════════════════════════════════════════════════
# Estado del vehículo
# ══════════════════════════════════════════════════════════════════
class VehicleMode:
    STANDBY    = "STANDBY"
    MANUAL     = "MANUAL"
    VISION_TEST = "VISION_TEST"
    AUTONOMOUS = "AUTONOMOUS"


# ══════════════════════════════════════════════════════════════════
# Sistema principal
# ══════════════════════════════════════════════════════════════════
class CarritoTMR:

    LOOP_HZ = 50   # frecuencia del bucle principal (20 ms por ciclo)

    def __init__(self):
        # ── GPIO global ──
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self._setup_leds()

        # ── Hardware ──
        print("[INIT] Inicializando hardware...")
        self.motor    = MotorDriver()
        self.steering = SteeringDriver()
        self.sensor   = DistanceSensor()
        self.camera   = CameraManager()
        self.gamepad  = GamepadReader()

        # ── Vision ──
        self.lane_detector = LaneDetector(debug=False)
        self.obj_detector  = ObjectDetector()

        # ── Controlador autónomo ──
        self.autonomous = AutonomousController(self.motor, self.steering)

        # ── Estado ──
        self._mode    = VehicleMode.STANDBY
        self._running = True
        self._dt      = 1.0 / self.LOOP_HZ
        self._last_loop_time = time.monotonic()

        # ── Señal de apagado limpio ──
        signal.signal(signal.SIGINT,  self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

        print("[INIT] Sistema listo.")

    # ----------------------------------------------------------
    # Arranque
    # ----------------------------------------------------------
    def run(self):
        """Arranca todos los subsistemas y ejecuta el bucle principal."""
        print("[RUN] Iniciando subsistemas...")
        self.sensor.start()
        self.camera.start()
        self.gamepad.start()
        self.motor.enable()

        print("[RUN] Esperando conexión del mando (Bluetooth)...")
        self._set_led(PIN_LED_STATUS, True)

        try:
            self._main_loop()
        finally:
            self._shutdown()

    # ----------------------------------------------------------
    # Bucle principal
    # ----------------------------------------------------------
    def _main_loop(self):
        while self._running:
            now = time.monotonic()
            self._dt = now - self._last_loop_time
            self._last_loop_time = now

            gp    = self.gamepad.state
            tof   = self.sensor.distance_mm
            frame_data = self.camera.get_latest_frame()

            # ── Procesar visión (CPU — rápido) ──
            lane = LaneData(0, 0, False, 0, 0)
            obj_result = ObjectDetector.AnalysisResult()

            if frame_data is not None:
                lane = self.lane_detector.process(frame_data.image)
                obj_result = self.obj_detector.analyze(
                    frame_data.detections, frame_data.image, tof
                )

            # ── Transiciones de modo por botones ──
            self._handle_mode_transitions(gp)

            # ── Ejecutar lógica del modo activo ──
            match self._mode:

                case VehicleMode.STANDBY:
                    self._mode_standby(gp)

                case VehicleMode.MANUAL:
                    self._mode_manual(gp)

                case VehicleMode.VISION_TEST:
                    self._mode_vision_test(lane, obj_result, tof)

                case VehicleMode.AUTONOMOUS:
                    self._mode_autonomous(lane, obj_result, tof)

            # ── Sincronizar frecuencia del bucle ──
            elapsed = time.monotonic() - now
            sleep_time = (1.0 / self.LOOP_HZ) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ----------------------------------------------------------
    # Gestión de modos
    # ----------------------------------------------------------
    def _handle_mode_transitions(self, gp):
        """
        Detecta flancos de botón y cambia de modo.
        Los cambios de modo SIEMPRE detienen el motor por seguridad.
        """
        if not gp.connected:
            if self._mode != VehicleMode.STANDBY:
                print("[FSM] Mando desconectado → STANDBY")
                self._safe_stop()
                self._change_mode(VehicleMode.STANDBY)
            return

        if self.gamepad.consume_button(BTN_BACK_TO_MANUAL):
            if self._mode != VehicleMode.MANUAL:
                self._safe_stop()
                self._change_mode(VehicleMode.MANUAL)

        elif self.gamepad.consume_button(BTN_VISION_TEST):
            if self._mode != VehicleMode.VISION_TEST:
                self._safe_stop()
                self._change_mode(VehicleMode.VISION_TEST)

        elif self.gamepad.consume_button(BTN_AUTONOMOUS):
            if self._mode != VehicleMode.AUTONOMOUS:
                self._safe_stop()
                self._change_mode(VehicleMode.AUTONOMOUS)
                self.autonomous.activate()

    def _change_mode(self, new_mode: str):
        print(f"[FSM] {self._mode} → {new_mode}")
        self._mode = new_mode

    # ----------------------------------------------------------
    # STANDBY
    # ----------------------------------------------------------
    def _mode_standby(self, gp):
        """Espera activa hasta que el mando se conecte."""
        if gp.connected:
            print("[FSM] Mando conectado. Modo MANUAL activado.")
            self._change_mode(VehicleMode.MANUAL)
        else:
            # Parpadeo lento para indicar que espera
            if int(time.monotonic() * 2) % 2 == 0:
                self._set_led(PIN_LED_STATUS, True)
            else:
                self._set_led(PIN_LED_STATUS, False)

    # ----------------------------------------------------------
    # MANUAL
    # ----------------------------------------------------------
    def _mode_manual(self, gp):
        """
        Control manual directo del mando.

        Acelerador  : Gatillo R2 → motor adelante  (0-100%)
        Freno       : Gatillo L2 → freno           (0-100%)
        Dirección   : Joystick derecho X → servo   (45°-135°)
        """
        # ── Motor ──
        if gp.brake > 0.05:
            # L2 presionado → freno / reversa suave
            self.motor.set_throttle(-gp.brake * 60)   # máx 60% en reversa manual
        elif gp.throttle > 0.05:
            self.motor.set_throttle(gp.throttle * 100)
        else:
            self.motor.brake()

        # ── Dirección ──
        # steer en [-1, 1]: -1 = izquierda (45°), +1 = derecha (135°)
        servo_range = SERVO_CENTER_ANGLE - 45  # = 45°
        servo_angle = SERVO_CENTER_ANGLE + gp.steer * servo_range
        self.steering.set_angle(servo_angle)

    # ----------------------------------------------------------
    # VISION TEST
    # ----------------------------------------------------------
    def _mode_vision_test(self, lane, obj_result, tof_mm):
        """
        Motores fijos en 0.  Solo imprime resultados de visión para debug.
        Ideal para calibrar la cámara sin mover el coche.
        """
        self._safe_stop()

        print(
            f"\r[VISION] "
            f"Error carril: {lane.error_px:+6.1f}px | "
            f"Curva: {'SI' if lane.is_curve else 'NO'} | "
            f"ToF: {tof_mm or '---':>5} mm | "
            f"STOP: {'SÍ' if obj_result.stop_sign_detected else 'no':>2} "
            f"({obj_result.stop_sign_distance_mm or '---'} mm) | "
            f"Semáforo: {obj_result.traffic_light.color if obj_result.traffic_light else '---'}",
            end="", flush=True,
        )

    # ----------------------------------------------------------
    # AUTONOMOUS
    # ----------------------------------------------------------
    def _mode_autonomous(self, lane, obj_result, tof_mm):
        """Delega completamente en AutonomousController."""
        self.autonomous.update(lane, obj_result, tof_mm, self._dt)

    # ----------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------
    def _safe_stop(self):
        self.motor.brake()
        self.steering.center()
        if self._mode == VehicleMode.AUTONOMOUS:
            self.autonomous.deactivate()

    def _setup_leds(self):
        for pin in (PIN_LED_STOP, PIN_LED_STATUS):
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

    @staticmethod
    def _set_led(pin: int, state: bool):
        GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)

    # ----------------------------------------------------------
    # Apagado limpio
    # ----------------------------------------------------------
    def _handle_signal(self, signum, frame):
        print(f"\n[SYS] Señal {signum} recibida → apagando...")
        self._running = False

    def _shutdown(self):
        print("[SYS] Apagando subsistemas...")
        self._safe_stop()
        self.gamepad.stop()
        self.sensor.stop()
        self.camera.stop()
        self.motor.cleanup()

        # LEDs apagados
        for pin in (PIN_LED_STOP, PIN_LED_STATUS):
            GPIO.output(pin, GPIO.LOW)

        GPIO.cleanup()
        print("[SYS] Sistema apagado correctamente.")


# ══════════════════════════════════════════════════════════════════
# Punto de entrada
# ══════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    carro = CarritoTMR()
    carro.run()
