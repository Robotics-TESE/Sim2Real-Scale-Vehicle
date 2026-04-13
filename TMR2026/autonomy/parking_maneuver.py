# -*- coding: utf-8 -*-
"""
parking_maneuver.py — Sub-máquina de estados para Estacionamiento en Batería.

Reto TMR 2026:
  Espacio de 60 cm entre dos coches estáticos.
  El coche debe entrar perpendicularmente en reversa (estacionamiento en batería).

Cinemática Ackermann usada:
  Radio de giro: R = L / tan(δ)  (modelo bicicleta)
  Donde:
    L = WHEELBASE (distancia entre ejes)
    δ = ángulo de dirección central (servo − 90°)

Maniobra planificada (desde posición alineada con el espacio):
  ┌─ SEARCHING ─────────────────────────────────────────────────────┐
  │  Avanza despacio.  VL53L0X lateral detecta un hueco ≥ 60 cm.   │
  └──────────────────────────────────────────────────────────────────┘
        ↓  gap detectado
  ┌─ POSITIONING ────────────────────────────────────────────────────┐
  │  Avanza un poco más para alinear el eje trasero con el hueco.   │
  └──────────────────────────────────────────────────────────────────┘
        ↓  tiempo de avance completado
  ┌─ REVERSING_LOCK ──────────────────────────────────────────────────┐
  │  Reversa con máximo giro Ackermann hacia el hueco.               │
  │  La trayectoria curva mete la parte trasera en el espacio.       │
  └──────────────────────────────────────────────────────────────────┘
        ↓  tiempo de arco completado
  ┌─ REVERSING_STRAIGHT ──────────────────────────────────────────────┐
  │  Endereza ruedas y continúa en reversa hasta centrarse.          │
  └──────────────────────────────────────────────────────────────────┘
        ↓  tiempo completado
  ┌─ PARKED ──────────────────────────────────────────────────────────┐
  │  Motor stop.  Señal de completado al controlador principal.      │
  └──────────────────────────────────────────────────────────────────┘
"""

import math
import time
from enum import Enum, auto

from config import (
    WHEELBASE, TRACK_WIDTH, MAX_STEERING_ANGLE_DEG,
    SERVO_CENTER_ANGLE, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
    PARK_SEARCH_SPEED, PARK_MANEUVER_SPEED,
    PARK_MIN_GAP_MM, PARK_TARGET_GAP_MM,
    PARK_OVERSHOOT_SEC, PARK_REVERSE_LOCK_SEC,
    PARK_REVERSE_STRAIGHT_SEC,
    PARK_GAP_CAMERA_MIN_SEC,
    EMERGENCY_STOP_MM,
)


class ParkingState(Enum):
    IDLE             = auto()
    SEARCHING        = auto()
    POSITIONING      = auto()
    REVERSING_LOCK   = auto()
    REVERSING_STRAIGHT = auto()
    PARKED           = auto()
    ABORTED          = auto()


class ParkingManeuver:
    """
    Sub-FSM de estacionamiento en batería.

    Parámetros geométricos (todos en config.py):
      PARK_OVERSHOOT_SEC       : tiempo de avance extra tras detectar el hueco
      PARK_REVERSE_LOCK_SEC    : tiempo de reversa con giro máximo
      PARK_REVERSE_STRAIGHT_SEC: tiempo de reversa derecho

    Estos tiempos se calibran en pista.  Reemplazar con encoder/odometría
    si el coche los tiene.
    """

    # Dirección del hueco: "left" o "right" (a qué lado está el espacio)
    def __init__(self, gap_side: str = "right"):
        self.gap_side = gap_side  # lado donde está el espacio de estacionamiento
        self._state     = ParkingState.IDLE
        self._phase_start: float    = 0.0
        self._gap_detected_at: float = 0.0
        self._gap_open_since: float  = 0.0   # cuando dejó de haber AUTO en zona lateral

        # Para decidir el ángulo de giro máximo
        if gap_side == "right":
            self._lock_angle = SERVO_MAX_ANGLE    # giro máximo derecha
            self._straight_angle = SERVO_CENTER_ANGLE
        else:
            self._lock_angle = SERVO_MIN_ANGLE     # giro máximo izquierda
            self._straight_angle = SERVO_CENTER_ANGLE

        # Precalcular radio de giro en la maniobra (informativo)
        self._R_turn = self._calc_turning_radius()

    # ----------------------------------------------------------
    # API pública
    # ----------------------------------------------------------
    @property
    def state(self) -> ParkingState:
        return self._state

    @property
    def is_active(self) -> bool:
        return self._state not in (ParkingState.IDLE,
                                   ParkingState.PARKED,
                                   ParkingState.ABORTED)

    @property
    def is_complete(self) -> bool:
        return self._state == ParkingState.PARKED

    def start(self):
        """Inicia la búsqueda del espacio de estacionamiento."""
        self._state = ParkingState.SEARCHING
        self._phase_start = time.monotonic()
        print("[PARKING] Iniciando búsqueda de espacio...")

    def abort(self):
        """Aborta la maniobra y regresa al estado IDLE."""
        self._state = ParkingState.ABORTED
        print("[PARKING] Maniobra abortada.")

    def reset(self):
        self._state = ParkingState.IDLE

    def update(
        self,
        tof_distance_mm: float | None,
        motor,
        steering,
        obj_result=None,
    ) -> ParkingState:
        """
        Actualiza la FSM.  Debe llamarse en cada ciclo del bucle principal.

        Parameters
        ----------
        tof_distance_mm : float | None
            Lectura del VL53L0X (frontal o lateral según montaje).
        motor : MotorDriver
        steering : SteeringDriver

        Returns
        -------
        ParkingState actual
        """
        now = time.monotonic()
        elapsed = now - self._phase_start

        # ── Seguridad global: obstáculo frontal durante reversa ──
        if (self._state in (ParkingState.REVERSING_LOCK,
                             ParkingState.REVERSING_STRAIGHT)
                and tof_distance_mm is not None
                and tof_distance_mm < EMERGENCY_STOP_MM):
            motor.stop()
            steering.center()
            self.abort()
            print("[PARKING] EMERGENCIA: obstáculo durante reversa.")
            return self._state

        match self._state:

            case ParkingState.SEARCHING:
                # Avanza despacio buscando el hueco.
                # Prioridad: cámara (obj_result) > ToF (si disponible).
                motor.set_throttle(PARK_SEARCH_SPEED)
                steering.center()

                gap_open = self._detect_gap(tof_distance_mm, obj_result, now)

                if gap_open:
                    self._gap_detected_at = now
                    self._transition(ParkingState.POSITIONING)
                    print(f"[PARKING] Hueco detectado. Posicionando...")

            case ParkingState.POSITIONING:
                # Avanza un poco más para alinear el eje trasero
                motor.set_throttle(PARK_SEARCH_SPEED)
                steering.center()

                if elapsed >= PARK_OVERSHOOT_SEC:
                    self._transition(ParkingState.REVERSING_LOCK)
                    print("[PARKING] Posición lista. Iniciando reversa con giro...")

            case ParkingState.REVERSING_LOCK:
                # Reversa con giro máximo hacia el hueco
                motor.set_throttle(-PARK_MANEUVER_SPEED)
                steering.set_angle(self._lock_angle)

                arc_time = self._estimate_arc_time()
                if elapsed >= arc_time:
                    self._transition(ParkingState.REVERSING_STRAIGHT)
                    print("[PARKING] Arco completado. Enderezando...")

            case ParkingState.REVERSING_STRAIGHT:
                # Endereza y termina de entrar
                motor.set_throttle(-PARK_MANEUVER_SPEED)
                steering.center()

                if elapsed >= PARK_REVERSE_STRAIGHT_SEC:
                    motor.stop()
                    steering.center()
                    self._state = ParkingState.PARKED
                    print("[PARKING] ¡Estacionamiento completado!")

            case ParkingState.PARKED | ParkingState.IDLE | ParkingState.ABORTED:
                pass  # estado terminal, nada que hacer

        return self._state

    # ----------------------------------------------------------
    # Cálculo de arco Ackermann
    # ----------------------------------------------------------
    def _calc_turning_radius(self) -> float:
        """
        Radio de giro en la fase REVERSING_LOCK usando el ángulo máximo.
        R = L / tan(δ)
        """
        delta = abs(self._lock_angle - SERVO_CENTER_ANGLE)
        delta_rad = math.radians(delta)
        if delta_rad < 0.01:
            return float("inf")
        return WHEELBASE / math.tan(delta_rad)

    def _estimate_arc_time(self) -> float:
        """
        Estima el tiempo necesario para rotar 90° en el arco Ackermann
        a la velocidad de maniobra.  Solo es una guía — el tiempo exacto
        se calibra en PARK_REVERSE_LOCK_SEC (config.py).

        Longitud del arco para 90°: s = (π/2) * R
        Velocidad lineal aproximada (map 18% PWM → ~0.25 m/s en escala 1:10).
        """
        SPEED_MS_APPROX = 0.20  # m/s a PARK_MANEUVER_SPEED — calibrar
        arc_length = (math.pi / 2) * self._R_turn
        estimated = arc_length / SPEED_MS_APPROX if SPEED_MS_APPROX > 0 else PARK_REVERSE_LOCK_SEC
        # Usar el valor de config como límite superior de seguridad
        return min(estimated, PARK_REVERSE_LOCK_SEC)

    # ----------------------------------------------------------
    # Detección de hueco
    # ----------------------------------------------------------
    def _detect_gap(self, tof_mm, obj_result, now: float) -> bool:
        """
        Combina cámara y ToF para decidir si hay un hueco de estacionamiento.

        Lógica:
        - Si hay obj_result → usar cámara como fuente principal.
          El hueco existe cuando NO hay AUTO en la zona lateral derecha
          durante al menos PARK_GAP_CAMERA_MIN_SEC segundos.
        - Si no hay obj_result → caer a ToF (comportamiento original).
        """
        if obj_result is not None:
            return self._detect_gap_camera(obj_result, now)

        # Fallback a ToF
        gap_open = (tof_mm is None or tof_mm >= PARK_MIN_GAP_MM)
        return gap_open and (now - self._phase_start) > 0.3

    def _detect_gap_camera(self, obj_result, now: float) -> bool:
        """
        Detecta el hueco cuando el espacio lateral derecho está despejado.

        Mientras el primer auto delimitador estaba en el lado derecho y
        ahora ya no hay AUTO en esa zona = inicio del hueco.
        Requiere PARK_GAP_CAMERA_MIN_SEC segundos consecutivos sin AUTO lateral
        para confirmar (evita falsos positivos por frames ruidosos).
        """
        lateral_clear = not obj_result.car_in_park_zone

        if lateral_clear:
            if self._gap_open_since == 0.0:
                self._gap_open_since = now   # empieza a contar
            gap_secs = now - self._gap_open_since
            return gap_secs >= PARK_GAP_CAMERA_MIN_SEC
        else:
            self._gap_open_since = 0.0   # resetear si vuelve a aparecer un auto
            return False

    def _transition(self, new_state: ParkingState):
        self._state = new_state
        self._phase_start = time.monotonic()
        self._gap_open_since = 0.0   # reset al cambiar de fase
