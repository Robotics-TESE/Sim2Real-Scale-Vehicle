"""Turn signals and hazards for the TMR 2026 vehicle.

One LED per side (left / right). Hazard = both blinking together. The blink
is driven by `time.monotonic()` -- NEVER `sleep()`, NEVER a thread. The main
loop calls `tick()` at 50 Hz; the blink happens between calls.

Modes:
  OFF    -> both LEDs off
  LEFT   -> only the left one blinks
  RIGHT  -> only the right one blinks
  HAZARD -> both blink in phase

Default pins (BCM): 19 (left) / 20 (right). They match
`vision_config.yaml:gpio.led_turn_left / led_turn_right` to reuse the wiring
of the `vision_module.py` test script.

Backend: tries `lgpio` (chip 4, Pi 5) and falls back to `RPi.GPIO`. If no
backend is available the module becomes a no-op -- the rest of the program
keeps running without turn signals.
"""

from enum import Enum, auto
import time
from typing import Optional


class SignalMode(Enum):
    OFF    = auto()
    LEFT   = auto()
    RIGHT  = auto()
    HAZARD = auto()


class TurnSignals:
    """
    Turn-signal / hazard controller with software blinking.

    Usage::

        signals = TurnSignals(pin_left=19, pin_right=20, blink_hz=2.0)
        signals.set_mode(SignalMode.LEFT)
        while running:
            signals.tick()      # every iteration of the main loop
        signals.close()
    """

    def __init__(
        self,
        pin_left:  int   = 19,
        pin_right: int   = 20,
        blink_hz:  float = 2.0,
    ):
        self._pin_l = pin_left
        self._pin_r = pin_right
        self._half_period = 0.5 / blink_hz if blink_hz > 0 else 0.25

        self._mode        = SignalMode.OFF
        self._last_toggle = time.monotonic()
        self._blink_on    = False

        self._backend: Optional[str] = None
        self._handle:  Optional[int] = None
        self._setup_gpio()

    def _setup_gpio(self) -> None:
        try:
            import lgpio
            self._lgpio  = lgpio
            self._handle = lgpio.gpiochip_open(4)
            lgpio.gpio_claim_output(self._handle, self._pin_l, 0)
            lgpio.gpio_claim_output(self._handle, self._pin_r, 0)
            self._backend = "lgpio"
            print(f"[SIGNALS] lgpio OK - left={self._pin_l}  right={self._pin_r}")
            return
        except Exception as e:
            last_err = e

        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self._pin_l, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self._pin_r, GPIO.OUT, initial=GPIO.LOW)
            self._GPIO    = GPIO
            self._backend = "RPi.GPIO"
            print(f"[SIGNALS] RPi.GPIO OK - left={self._pin_l}  right={self._pin_r}")
            return
        except Exception as e:
            last_err = e

        print(f"[SIGNALS] No GPIO - turn signals disabled ({last_err})")
        self._backend = None

    def _write(self, pin: int, value: int) -> None:
        if self._backend == "lgpio":
            self._lgpio.gpio_write(self._handle, pin, value)
        elif self._backend == "RPi.GPIO":
            self._GPIO.output(pin, value)

    def set_mode(self, mode: SignalMode) -> None:
        """Change the mode. Does nothing if it is the same."""
        if mode == self._mode:
            return
        self._mode        = mode
        self._last_toggle = time.monotonic()
        self._blink_on    = True
        self._apply()

    @property
    def mode(self) -> SignalMode:
        return self._mode

    def tick(self) -> None:
        """
        Advance the blink according to `time.monotonic()`.
        Call on every iteration of the main loop (50 Hz recommended).
        Non-blocking.
        """
        if self._mode == SignalMode.OFF:
            if self._blink_on:
                self._blink_on = False
                self._apply()
            return

        now = time.monotonic()
        if now - self._last_toggle >= self._half_period:
            self._blink_on    = not self._blink_on
            self._last_toggle = now
            self._apply()

    def _apply(self) -> None:
        """Write the current state to both LEDs based on mode + blink_on."""
        left  = 0
        right = 0
        if self._blink_on:
            if self._mode in (SignalMode.LEFT,  SignalMode.HAZARD):
                left  = 1
            if self._mode in (SignalMode.RIGHT, SignalMode.HAZARD):
                right = 1
        self._write(self._pin_l, left)
        self._write(self._pin_r, right)

    def close(self) -> None:
        """Turn LEDs off and release GPIO."""
        try:
            if self._backend == "lgpio" and self._handle is not None:
                self._lgpio.gpio_write(self._handle, self._pin_l, 0)
                self._lgpio.gpio_write(self._handle, self._pin_r, 0)
                self._lgpio.gpiochip_close(self._handle)
            elif self._backend == "RPi.GPIO":
                self._GPIO.output(self._pin_l, 0)
                self._GPIO.output(self._pin_r, 0)
        except Exception:
            pass
        self._backend = None
