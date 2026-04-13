# -*- coding: utf-8 -*-
"""
distance_sensor.py — Dos VL53L0X en I²C bus 4 (GPIO 22/23).

Cableado real:
  SDA → GPIO 23 (Pin 16)
  SCL → GPIO 22 (Pin 15)
  XSHUT delantero → GPIO 17 (Pin 11)
  XSHUT trasero   → GPIO 27 (Pin 13)

Dos sensores en el mismo bus I²C requieren direcciones distintas.
Secuencia de inicialización:
  1. Ambos XSHUT en LOW  → ambos sensores apagados
  2. XSHUT delantero HIGH → solo el delantero arranca en 0x29
  3. Cambiar dirección del delantero a 0x30
  4. XSHUT trasero HIGH   → el trasero arranca en 0x29 (default)
  5. Resultado: delantero=0x30, trasero=0x29
"""

import threading
import time

import board
import busio
import RPi.GPIO as GPIO
import adafruit_vl53l0x

from config import (
    PIN_TOF_XSHUT_FRONT,
    PIN_TOF_XSHUT_REAR,
    TOF_ADDR_FRONT,
    TOF_ADDR_REAR,
    TOF_TIMING_BUDGET_US,
    TOF_MAX_RANGE_MM,
    TOF_POLL_INTERVAL_S,
)


class DistanceSensor:
    """
    Gestiona dos VL53L0X (delantero y trasero) en hilo dedicado.

    Uso:
        sensor = DistanceSensor()
        sensor.start()
        mm_front = sensor.front_mm
        mm_rear  = sensor.rear_mm
        sensor.stop()
    """

    def __init__(self):
        # Configurar pines XSHUT como salidas
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_TOF_XSHUT_FRONT, GPIO.OUT)
        GPIO.setup(PIN_TOF_XSHUT_REAR,  GPIO.OUT)

        # Apagar ambos sensores
        GPIO.output(PIN_TOF_XSHUT_FRONT, GPIO.LOW)
        GPIO.output(PIN_TOF_XSHUT_REAR,  GPIO.LOW)
        time.sleep(0.1)

        # Bus I²C 4 — GPIO 22 (SCL) / GPIO 23 (SDA)
        self._i2c = busio.I2C(board.D22, board.D23)

        # Inicializar sensor delantero en 0x30
        GPIO.output(PIN_TOF_XSHUT_FRONT, GPIO.HIGH)
        time.sleep(0.1)
        self._front = adafruit_vl53l0x.VL53L0X(self._i2c)
        self._front.set_address(TOF_ADDR_FRONT)   # cambiar a 0x30
        self._front.measurement_timing_budget = TOF_TIMING_BUDGET_US

        # Inicializar sensor trasero en 0x29 (default)
        GPIO.output(PIN_TOF_XSHUT_REAR, GPIO.HIGH)
        time.sleep(0.1)
        self._rear = adafruit_vl53l0x.VL53L0X(self._i2c)
        # El trasero queda en 0x29 — no se cambia
        self._rear.measurement_timing_budget = TOF_TIMING_BUDGET_US

        # Estado compartido
        self._front_mm: float | None = None
        self._rear_mm:  float | None = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    # ----------------------------------------------------------
    # Control del hilo
    # ----------------------------------------------------------
    def start(self):
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._poll_loop,
            name="ToF-Sensors",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    # ----------------------------------------------------------
    # API pública (thread-safe)
    # ----------------------------------------------------------
    @property
    def front_mm(self) -> float | None:
        """Distancia frontal en mm. None si fuera de rango."""
        with self._lock:
            return self._front_mm

    @property
    def rear_mm(self) -> float | None:
        """Distancia trasera en mm. None si fuera de rango."""
        with self._lock:
            return self._rear_mm

    @property
    def distance_mm(self) -> float | None:
        """Alias del sensor frontal (interfaz compatible con el resto del código)."""
        return self.front_mm

    def is_obstacle_near(self, threshold_mm: float) -> bool:
        d = self.front_mm
        return d is not None and d < threshold_mm

    # ----------------------------------------------------------
    # Hilo de lectura
    # ----------------------------------------------------------
    def _poll_loop(self):
        while not self._stop_event.is_set():
            try:
                raw_f = self._front.range
                raw_r = self._rear.range
                front = raw_f if raw_f < TOF_MAX_RANGE_MM else None
                rear  = raw_r if raw_r < TOF_MAX_RANGE_MM else None
            except Exception:
                front = None
                rear  = None

            with self._lock:
                self._front_mm = front
                self._rear_mm  = rear

            time.sleep(TOF_POLL_INTERVAL_S)
