# -*- coding: utf-8 -*-
"""
motor_driver.py — Control del puente H IBT-2.

Pinout fijado en config.py (BCM):
  EN   = 24   (habilitar driver)
  RPWM = 18   (PWM avance)
  LPWM = 13   (PWM reversa)

El IBT-2 tiene dos entradas independientes:
  Avance  → EN=HIGH, RPWM=%duty, LPWM=0
  Reversa → EN=HIGH, RPWM=0,    LPWM=%duty
  Freno   → EN=HIGH, RPWM=100,  LPWM=100  (freno eléctrico)
  Costa   → EN=LOW  (rueda libre)
"""

import RPi.GPIO as GPIO
from config import PIN_MOTOR_EN, PIN_MOTOR_RPWM, PIN_MOTOR_LPWM, MOTOR_PWM_FREQ


class MotorDriver:
    """Interfaz de alto nivel para el IBT-2."""

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in (PIN_MOTOR_EN, PIN_MOTOR_RPWM, PIN_MOTOR_LPWM):
            GPIO.setup(pin, GPIO.OUT)

        GPIO.output(PIN_MOTOR_EN, GPIO.LOW)   # deshabilitado al inicio

        self._pwm_fwd = GPIO.PWM(PIN_MOTOR_RPWM, MOTOR_PWM_FREQ)
        self._pwm_rev = GPIO.PWM(PIN_MOTOR_LPWM, MOTOR_PWM_FREQ)
        self._pwm_fwd.start(0)
        self._pwm_rev.start(0)

        self._current_duty = 0.0   # + avance, - reversa
        self._enabled = False

    # ----------------------------------------------------------
    # API pública
    # ----------------------------------------------------------
    def enable(self):
        GPIO.output(PIN_MOTOR_EN, GPIO.HIGH)
        self._enabled = True

    def disable(self):
        """Costa — sin frenado activo."""
        GPIO.output(PIN_MOTOR_EN, GPIO.LOW)
        self._pwm_fwd.ChangeDutyCycle(0)
        self._pwm_rev.ChangeDutyCycle(0)
        self._current_duty = 0.0
        self._enabled = False

    def set_throttle(self, duty: float):
        """
        Aplica un nivel de potencia al motor.

        Parameters
        ----------
        duty : float
            Rango [-100, 100].
            > 0  → avance
            < 0  → reversa
            = 0  → freno suave (corta PWM pero EN sigue HIGH)
        """
        duty = max(-100.0, min(100.0, duty))
        self._current_duty = duty

        if not self._enabled:
            self.enable()

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
        if not self._enabled:
            self.enable()
        self._pwm_fwd.ChangeDutyCycle(100)
        self._pwm_rev.ChangeDutyCycle(100)
        self._current_duty = 0.0

    def stop(self):
        """Alias de freno para el bucle principal."""
        self.brake()

    @property
    def duty(self) -> float:
        return self._current_duty

    def cleanup(self):
        self.disable()
        self._pwm_fwd.stop()
        self._pwm_rev.stop()
        # GPIO.cleanup() lo llama main.py para no interferir con otros módulos
