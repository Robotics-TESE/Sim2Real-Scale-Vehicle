# -*- coding: utf-8 -*-
"""
steering_driver.py — Servo MG90s en PCA9685 Canal 0.

Bus I²C alternativo: SDA=GPIO0, SCL=GPIO1
    i2c = busio.I2C(board.D1, board.D0)

Geometría Ackermann implementada:
  El coche RC tiene ambas ruedas delanteras ligadas mecánicamente
  a un único servo. Usamos el ángulo central del modelo de bicicleta
  y lo convertimos a pulso PWM del servo.

  Radio de giro:  R = Wheelbase / tan(δ)
  Ángulo rueda interior: arctan(L / (R − W/2))
  Ángulo rueda exterior: arctan(L / (R + W/2))

  Como el servo mueve ambas ruedas a la vez usamos el ángulo central δ
  directamente (simplificación válida para escala 1:10 y velocidades bajas).
"""

import math
import adafruit_pca9685
from adafruit_motor import servo as adafruit_servo
from adafruit_extended_bus import ExtendedI2C

from config import (
    PCA9685_PWM_FREQ,
    SERVO_CHANNEL,
    SERVO_MIN_PULSE_US,
    SERVO_MAX_PULSE_US,
    SERVO_CENTER_ANGLE,
    SERVO_MIN_ANGLE,
    SERVO_MAX_ANGLE,
    WHEELBASE,
    TRACK_WIDTH,
    MAX_STEERING_ANGLE_DEG,
)

try:
    from config import STEERING_INVERTED
except ImportError:
    STEERING_INVERTED = False


class SteeringDriver:
    """Control de dirección con geometría Ackermann sobre PCA9685."""

    def __init__(self):
        # Bus i2c-3 (GPIO 0=SDA, GPIO 1=SCL) creado por dtoverlay
        i2c = ExtendedI2C(3)
        pca = adafruit_pca9685.PCA9685(i2c)
        pca.frequency = PCA9685_PWM_FREQ

        self._servo = adafruit_servo.Servo(
            pca.channels[SERVO_CHANNEL],
            min_pulse=SERVO_MIN_PULSE_US,
            max_pulse=SERVO_MAX_PULSE_US,
        )
        self._current_angle = SERVO_CENTER_ANGLE
        self.center()

    # ----------------------------------------------------------
    # API pública
    # ----------------------------------------------------------
    def center(self):
        """Ruedas al frente (centro geométrico)."""
        self._servo.angle = SERVO_CENTER_ANGLE
        self._current_angle = SERVO_CENTER_ANGLE

    def set_angle(self, angle_deg: float):
        """
        Mueve el servo al ángulo indicado.

        Parameters
        ----------
        angle_deg : float
            Rango [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE].
            90° = recto, <90° = izquierda, >90° = derecha.

        Si `STEERING_INVERTED=True` el ángulo lógico se voltea al escribir al
        servo, pero `current_angle` retorna el valor lógico esperado por el
        consumidor (signals/PID/etc.).
        """
        angle_deg = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, float(angle_deg)))
        physical = (2.0 * SERVO_CENTER_ANGLE - angle_deg) if STEERING_INVERTED else angle_deg
        self._servo.angle = physical
        self._current_angle = angle_deg

    def steer_from_error(self, lane_error_px: float, kp: float = 0.09) -> float:
        """
        Convierte el error de carril (píxeles) en ángulo de servo.
        Usa control proporcional simple — el PID lo llama externamente
        y pasa la corrección ya calculada como `steer_delta_deg`.

        Retorna el ángulo aplicado.
        """
        # Una corrección de 1° por cada ~11 px de error a Kp=0.09
        delta = kp * lane_error_px
        angle = SERVO_CENTER_ANGLE + delta
        self.set_angle(angle)
        return self._current_angle

    def set_steering_angle(self, steering_deg: float):
        """
        Interfaz Ackermann: recibe el ángulo de dirección CENTRAL
        (+ = derecha, − = izquierda) en grados y lo convierte a
        posición de servo.

        La corrección Ackermann exacta para cada rueda se calcula
        internamente para referencia, aunque el servo físico aplica
        solo el ángulo central.
        """
        steering_deg = max(
            -MAX_STEERING_ANGLE_DEG,
            min(MAX_STEERING_ANGLE_DEG, steering_deg)
        )

        # Ángulos individuales Ackermann (informativos / logging)
        inner, outer = self._ackermann_angles(steering_deg)

        # Servo aplica el ángulo central
        servo_angle = SERVO_CENTER_ANGLE + steering_deg
        self.set_angle(servo_angle)
        return inner, outer

    # ----------------------------------------------------------
    # Geometría Ackermann
    # ----------------------------------------------------------
    @staticmethod
    def _ackermann_angles(
        center_deg: float,
    ) -> tuple[float, float]:
        """
        Calcula los ángulos individuales de rueda (modelo Ackermann puro).

        Returns
        -------
        (inner_deg, outer_deg)
            inner = rueda del lado del giro (más cerrada)
            outer = rueda opuesta al giro (más abierta)
        """
        if abs(center_deg) < 0.1:
            return 0.0, 0.0

        delta = math.radians(center_deg)
        R = WHEELBASE / math.tan(delta)  # radio de giro del eje central

        sign = math.copysign(1, center_deg)
        R_inner = abs(R) - sign * TRACK_WIDTH / 2
        R_outer = abs(R) + sign * TRACK_WIDTH / 2

        # Evitar división por cero
        inner_deg = math.degrees(math.atan(WHEELBASE / R_inner)) * sign if R_inner != 0 else 90.0 * sign
        outer_deg = math.degrees(math.atan(WHEELBASE / R_outer)) * sign if R_outer != 0 else 0.0

        return inner_deg, outer_deg

    @staticmethod
    def turning_radius(steering_deg: float) -> float:
        """Radio de giro en metros para un ángulo central (modelo bicicleta)."""
        if abs(steering_deg) < 0.1:
            return float("inf")
        return WHEELBASE / math.tan(math.radians(abs(steering_deg)))

    @property
    def current_angle(self) -> float:
        return self._current_angle

    @property
    def steering_deviation(self) -> float:
        """Desvío del centro en grados (+ derecha, − izquierda)."""
        return self._current_angle - SERVO_CENTER_ANGLE
