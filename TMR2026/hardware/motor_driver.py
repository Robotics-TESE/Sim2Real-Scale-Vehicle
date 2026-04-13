# -*- coding: utf-8 -*-
"""
motor_driver.py — Control del puente H IBT-2.

Cableado real del coche:
  RPWM = GPIO 18 (Pin 12)  — PWM avance
  LPWM = GPIO 13 (Pin 33)  — PWM reversa
  R_EN + L_EN → 3.3V fijo  — siempre habilitado (sin GPIO de enable)

Con R_EN y L_EN en 3.3V, el puente está siempre activo.
El control de dirección y velocidad se hace solo con RPWM y LPWM:
  Avance  → RPWM=%duty, LPWM=0
  Reversa → RPWM=0,     LPWM=%duty
  Freno   → RPWM=100,   LPWM=100  (freno eléctrico / cortocircuito regenerativo)
  Stop    → RPWM=0,     LPWM=0    (rueda libre)
"""

import RPi.GPIO as GPIO
from config import PIN_MOTOR_RPWM, PIN_MOTOR_LPWM, MOTOR_PWM_FREQ


class MotorDriver:
    """Interfaz de alto nivel para el IBT-2 con enable permanente."""

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(PIN_MOTOR_RPWM, GPIO.OUT)
        GPIO.setup(PIN_MOTOR_LPWM, GPIO.OUT)

        self._pwm_fwd = GPIO.PWM(PIN_MOTOR_RPWM, MOTOR_PWM_FREQ)
        self._pwm_rev = GPIO.PWM(PIN_MOTOR_LPWM, MOTOR_PWM_FREQ)
        self._pwm_fwd.start(0)
        self._pwm_rev.start(0)

        self._current_duty = 0.0

    # ----------------------------------------------------------
    # API pública
    # ----------------------------------------------------------
    def enable(self):
        """No-op — el enable está en hardware (3.3V fijo)."""
        pass

    def disable(self):
        """Rueda libre — corta ambos PWM."""
        self._pwm_fwd.ChangeDutyCycle(0)
        self._pwm_rev.ChangeDutyCycle(0)
        self._current_duty = 0.0

    def set_throttle(self, duty: float):
        """
        Aplica potencia al motor.

        Parameters
        ----------
        duty : float
            [-100, 100]  >0 = avance, <0 = reversa, 0 = freno
        """
        duty = max(-100.0, min(100.0, duty))
        self._current_duty = duty

        if duty > 0:
            self._pwm_fwd.ChangeDutyCycle(duty)
            self._pwm_rev.ChangeDutyCycle(0)
        elif duty < 0:
            self._pwm_fwd.ChangeDutyCycle(0)
            self._pwm_rev.ChangeDutyCycle(-duty)
        else:
            self.brake()

    def brake(self):
        """Freno eléctrico — ambas entradas al 100 %."""
        self._pwm_fwd.ChangeDutyCycle(100)
        self._pwm_rev.ChangeDutyCycle(100)
        self._current_duty = 0.0

    def stop(self):
        """Alias de brake."""
        self.brake()

    @property
    def duty(self) -> float:
        return self._current_duty

    def cleanup(self):
        self.disable()
        self._pwm_fwd.stop()
        self._pwm_rev.stop()
