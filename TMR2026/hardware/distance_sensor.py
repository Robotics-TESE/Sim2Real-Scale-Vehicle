# -*- coding: utf-8 -*-
"""
distance_sensor.py — VL53L0X en el bus I²C alternativo.

Comparte el bus con el PCA9685.  El lock de I²C lo gestiona
el módulo de nivel superior (main.py) o cada driver por separado
usando threading.Lock() si es necesario.

El sensor corre en modo continuo y en un hilo dedicado
actualiza la lectura cada TOF_POLL_INTERVAL_S segundos.
"""

import threading
import time
import board
import busio
import adafruit_vl53l0x

from config import TOF_TIMING_BUDGET_US, TOF_MAX_RANGE_MM, TOF_POLL_INTERVAL_S


class DistanceSensor:
    """
    Wrapper thread-safe del VL53L0X.

    Uso:
        sensor = DistanceSensor()
        sensor.start()
        mm = sensor.distance_mm   # lectura más reciente
        sensor.stop()
    """

    def __init__(self):
        # El I²C alternativo es compartido — lo recibimos o lo creamos aquí.
        # Si steering_driver.py ya lo inicializó, reutilizar la misma instancia
        # pasándola como argumento o usando un singleton (ver i2c_bus.py si
        # se requiere en futuras versiones).  Por ahora crea su propia conexión.
        i2c = busio.I2C(board.D1, board.D0)
        self._sensor = adafruit_vl53l0x.VL53L0X(i2c)
        self._sensor.measurement_timing_budget = TOF_TIMING_BUDGET_US

        self._distance_mm: float | None = None
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
            name="ToF-Sensor",
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
    def distance_mm(self) -> float | None:
        """Última lectura en mm, o None si el sensor está fuera de rango."""
        with self._lock:
            return self._distance_mm

    @property
    def distance_cm(self) -> float | None:
        mm = self.distance_mm
        return mm / 10.0 if mm is not None else None

    def is_obstacle_near(self, threshold_mm: float) -> bool:
        """True si hay un obstáculo dentro del umbral especificado."""
        d = self.distance_mm
        return d is not None and d < threshold_mm

    # ----------------------------------------------------------
    # Hilo de lectura
    # ----------------------------------------------------------
    def _poll_loop(self):
        while not self._stop_event.is_set():
            try:
                raw = self._sensor.range  # mm
                value = raw if raw < TOF_MAX_RANGE_MM else None
            except Exception:
                value = None

            with self._lock:
                self._distance_mm = value

            time.sleep(TOF_POLL_INTERVAL_S)
