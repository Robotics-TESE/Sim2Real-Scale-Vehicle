# -*- coding: utf-8 -*-
"""
main.py — Sistema TMR 2026.

Botones del mando:
  A / Cruz      → Manual  (conducción directa)
  B / Círculo   → Visión  (cámara encendida, motores OFF — para depurar)
  X / Cuadrado  → Autónomo (carril + STOP + crucero peatonal)
  Y / Triángulo → Estacionamiento en batería

Gatillos:
  R2 → acelerar
  L2 → frenar / reversa suave

Joystick derecho X → dirección
"""

import sys
import time
import signal

import RPi.GPIO as GPIO

from config import (
    PIN_LED_STOP, PIN_LED_STATUS,
    SERVO_CENTER_ANGLE,
    BTN_MANUAL, BTN_VISION, BTN_AUTONOMOUS, BTN_PARKING,
)
from hardware.motor_driver    import MotorDriver
from hardware.steering_driver import SteeringDriver
from hardware.distance_sensor import DistanceSensor
from hardware.camera_manager  import CameraManager
from control.gamepad_reader   import GamepadReader
from vision.lane_detector     import LaneDetector, LaneData
from vision.object_detector   import ObjectDetector
from autonomy.autonomous_mode import AutonomousController


class VehicleMode:
    STANDBY    = "STANDBY"
    MANUAL     = "MANUAL"
    VISION     = "VISION"
    AUTONOMOUS = "AUTONOMOUS"
    PARKING    = "PARKING"


class CarritoTMR:

    LOOP_HZ = 50

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self._setup_leds()

        print("[INIT] Inicializando hardware...")
        self.motor    = MotorDriver()
        self.steering = SteeringDriver()
        self.sensor   = DistanceSensor()   # opcional — no crashea si no está
        self.camera   = CameraManager()
        self.gamepad  = GamepadReader()

        self.lane_detector = LaneDetector(debug=False)
        self.obj_detector  = ObjectDetector()
        self.autonomous    = AutonomousController(self.motor, self.steering)

        self._mode    = VehicleMode.STANDBY
        self._running = True
        self._dt      = 1.0 / self.LOOP_HZ
        self._last_t  = time.monotonic()

        signal.signal(signal.SIGINT,  self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

        print("[INIT] Listo. Esperando mando Bluetooth...")

    # ----------------------------------------------------------
    def run(self):
        self.sensor.start()
        self.camera.start()
        self.gamepad.start()

        try:
            self._main_loop()
        finally:
            self._shutdown()

    # ----------------------------------------------------------
    def _main_loop(self):
        while self._running:
            now = time.monotonic()
            self._dt  = now - self._last_t
            self._last_t = now

            gp         = self.gamepad.state
            tof        = self.sensor.distance_mm     # None si no hay sensor
            frame_data = self.camera.get_latest_frame()

            # Visión (CPU ligero — ~8 ms)
            lane = LaneData(0, 0, False, 0, SERVO_CENTER_ANGLE)
            obj  = ObjectDetector.AnalysisResult()
            if frame_data is not None:
                lane = self.lane_detector.process(frame_data.image)
                obj  = self.obj_detector.analyze(
                    frame_data.detections, frame_data.image, tof)

            # Transiciones de modo
            self._handle_mode_transitions(gp)

            # Ejecutar modo activo
            match self._mode:
                case VehicleMode.STANDBY:
                    self._standby(gp)
                case VehicleMode.MANUAL:
                    self._manual(gp)
                case VehicleMode.VISION:
                    self._vision(lane, obj, tof)
                case VehicleMode.AUTONOMOUS:
                    self.autonomous.update(lane, obj, tof, self._dt)
                case VehicleMode.PARKING:
                    self.autonomous.update(lane, obj, tof, self._dt)

            # Mantener frecuencia
            elapsed = time.monotonic() - now
            wait = (1.0 / self.LOOP_HZ) - elapsed
            if wait > 0:
                time.sleep(wait)

    # ----------------------------------------------------------
    def _handle_mode_transitions(self, gp):
        if not gp.connected:
            if self._mode != VehicleMode.STANDBY:
                print("[FSM] Mando desconectado → STANDBY")
                self._safe_stop()
                self._mode = VehicleMode.STANDBY
            return

        if self.gamepad.consume_button(BTN_MANUAL):
            self._safe_stop()
            self._change_mode(VehicleMode.MANUAL)

        elif self.gamepad.consume_button(BTN_VISION):
            self._safe_stop()
            self._change_mode(VehicleMode.VISION)

        elif self.gamepad.consume_button(BTN_AUTONOMOUS):
            self._safe_stop()
            self._change_mode(VehicleMode.AUTONOMOUS)
            self.autonomous.activate()

        elif self.gamepad.consume_button(BTN_PARKING):
            if self._mode == VehicleMode.AUTONOMOUS:
                self.autonomous.trigger_parking()
                print("[FSM] Estacionamiento activado")
            else:
                self._safe_stop()
                self._change_mode(VehicleMode.AUTONOMOUS)
                self.autonomous.activate()
                self.autonomous.trigger_parking()

    def _change_mode(self, new_mode: str):
        print(f"[FSM] {self._mode} → {new_mode}")
        self._mode = new_mode

    # ----------------------------------------------------------
    # Modos
    # ----------------------------------------------------------
    def _standby(self, gp):
        """Espera el mando. Parpadeo lento del LED de estado."""
        if gp.connected:
            print("[FSM] Mando conectado → MANUAL")
            self._change_mode(VehicleMode.MANUAL)
            self._set_led(PIN_LED_STATUS, True)
        else:
            self._set_led(PIN_LED_STATUS, int(time.monotonic() * 2) % 2 == 0)

    def _manual(self, gp):
        """
        Control directo del mando.
        R2 → acelerar  |  L2 → freno/reversa  |  Stick derecho → dirección
        """
        if gp.brake > 0.05:
            self.motor.set_throttle(-gp.brake * 60)
        elif gp.throttle > 0.05:
            self.motor.set_throttle(gp.throttle * 100)
        else:
            self.motor.brake()

        rango = SERVO_CENTER_ANGLE - 45   # 45°
        self.steering.set_angle(SERVO_CENTER_ANGLE + gp.steer * rango)

    def _vision(self, lane, obj, tof_mm):
        """
        Motores OFF. Imprime en terminal lo que ve la cámara.
        Ideal para calibrar detecciones antes de correr autónomo.
        """
        self._safe_stop()

        stop_info = "no"
        if obj.stop_sign_detected:
            d = obj.stop_sign_distance_mm
            stop_info = f"SÍ  {d:.0f}mm" if d else "SÍ  ?mm"

        semaforo = "---"
        if obj.traffic_light:
            semaforo = obj.traffic_light.color.upper()

        print(
            f"\r[VIS] "
            f"Error:{lane.error_px:+6.1f}px | "
            f"Curva:{'SI' if lane.is_curve else 'no'} | "
            f"Crucero:{'SI' if lane.crosswalk_detected else 'no'} | "
            f"ToF:{str(tof_mm or '---'):>5}mm | "
            f"STOP:{stop_info:<12} | "
            f"Semáforo:{semaforo}",
            end="", flush=True,
        )

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

    def _set_led(self, pin: int, state):
        GPIO.output(pin, GPIO.HIGH if bool(state) else GPIO.LOW)

    def _handle_signal(self, signum, frame):
        print(f"\n[SYS] Señal {signum} → apagando...")
        self._running = False

    def _shutdown(self):
        print("[SYS] Apagando...")
        self._safe_stop()
        self.gamepad.stop()
        self.sensor.stop()
        self.camera.stop()
        self.motor.cleanup()
        for pin in (PIN_LED_STOP, PIN_LED_STATUS):
            GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup()
        print("[SYS] Listo.")


# ----------------------------------------------------------
if __name__ == "__main__":
    CarritoTMR().run()
