"""Brake light for the TMR 2026 vehicle.

A single LED that turns on while the car is braking or stopped (FRENADO /
ESPERA states of the FSM). It does not blink -- it stays solid.

Default pin (BCM): 16.
Backend: `lgpio` (chip 4, Pi 5) with an `RPi.GPIO` fallback. If no backend
is available it becomes a no-op without breaking the rest of the system.
"""

from typing import Optional


class BrakeLight:
    """
    Brake-light controller.

    Usage::

        brake = BrakeLight(pin=16)
        brake.on()    # when entering FRENADO or ESPERA
        brake.off()   # when leaving
        brake.close()
    """

    def __init__(self, pin: int = 16):
        self._pin     = pin
        self._is_on   = False
        self._backend: Optional[str] = None
        self._handle:  Optional[int] = None
        self._setup_gpio()

    def _setup_gpio(self) -> None:
        try:
            import lgpio
            self._lgpio  = lgpio
            self._handle = lgpio.gpiochip_open(4)
            lgpio.gpio_claim_output(self._handle, self._pin, 0)
            self._backend = "lgpio"
            print(f"[BRAKE] lgpio OK - pin={self._pin}")
            return
        except Exception as e:
            last_err = e

        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self._pin, GPIO.OUT, initial=GPIO.LOW)
            self._GPIO    = GPIO
            self._backend = "RPi.GPIO"
            print(f"[BRAKE] RPi.GPIO OK - pin={self._pin}")
            return
        except Exception as e:
            last_err = e

        print(f"[BRAKE] No GPIO - brake light disabled ({last_err})")
        self._backend = None

    def _write(self, value: int) -> None:
        if self._backend == "lgpio":
            self._lgpio.gpio_write(self._handle, self._pin, value)
        elif self._backend == "RPi.GPIO":
            self._GPIO.output(self._pin, value)

    def on(self) -> None:
        if not self._is_on:
            self._write(1)
            self._is_on = True

    def off(self) -> None:
        if self._is_on:
            self._write(0)
            self._is_on = False

    @property
    def is_on(self) -> bool:
        return self._is_on

    def close(self) -> None:
        try:
            if self._backend == "lgpio" and self._handle is not None:
                self._lgpio.gpio_write(self._handle, self._pin, 0)
                self._lgpio.gpiochip_close(self._handle)
            elif self._backend == "RPi.GPIO":
                self._GPIO.output(self._pin, 0)
        except Exception:
            pass
        self._is_on   = False
        self._backend = None
