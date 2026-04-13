# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Hardware Target

Raspberry Pi with:
- CSI camera (Picamera2) — refactored system; USB webcam — original system
- VL53L0X ToF distance sensors via I2C (original) or serial UART `/dev/ttyUSB0` CSV format `"dist_delante,dist_atras"` (refactored)
- GPIO PWM motor control (PIN 18 = PWM, PIN 23 = DIR)
- GPIO LEDs: rojo=17, amarillo=27, verde=22 (BCM mode)
- Optional: pygame joystick (button 2 = activate, button 0 = emergency stop)

## Running the System

```bash
# Refactored system (Raspberry Pi CSI camera) — recommended
python main_refactored.py

# Original system (USB webcam + I2C VL53L0X)
python AUTO_YOLO/main.py
```

Controls: `Q` = quit, `P` = toggle parking mode, `R` = reset system state.

## Installing Dependencies

No `requirements.txt` exists. Install manually:

```bash
pip install ultralytics opencv-python numpy matplotlib pyserial pygame
# Raspberry Pi specific:
pip install picamera2 RPi.GPIO adafruit-circuitpython-vl53l0x
```

## Architecture

The codebase has two parallel implementations that co-exist:

### Refactored System (CAMARA / CONTROL / STATE_MACHINE)
Used by `main_refactored.py`. Clean modular design:

- **`CAMARA/`** — All perception. Each detector exposes `detectar_*(frame)` and `dibujar_*(frame, result)`.
  - `YOLO/objeto/` — Object detection (stop, yield, speed_limit signs) via YOLOv8
  - `YOLO/semaforo/` — Traffic light color classification (HSV-based)
  - `YOLO/señal/` — Traffic sign detection
  - `lane_detection/lane_center/` — Lane center via Canny + Hough + polyfit
  - `lane_detection/lane_curvature/` — Road curvature calculation

- **`PERCEPTION_OUTPUT/perception_output.py`** — Central data structure aggregating all sensor/vision data. Fields: `distancia_obj`, `tipo_obj`, `lane_error`, `semaforo`, `interseccion`. Passed as dict via `obtener_estado_percepcion()`.

- **`STATE_MACHINE/state_machine.py`** — Evaluates `PerceptionOutput` dict and returns one of 12 states (constants at top of file). Priority order: EMERGENCIA > semáforos > intersecciones > obstáculos > RUTA_LIBRE. `accion()` returns `(setpoint_velocidad, estado_str)`.

- **`CONTROL/vehicle_controller.py`** — `VehicleController` with:
  - `controlar_velocidad(setpoint)` — PID velocity loop (Kp=0.6, Ki=0.2, Kd=0.05)
  - `controlar_direccion_simple(lane_error, frame_width)` — proportional steering, both wheels same angle, clipped to ±30°. Formula: `angle = 0.5 * (lane_error / (frame_width/2))`
  - `maniobra_estacionamiento_paralelo(...)` — 4-phase parallel parking (search→align→backup→adjust)

### Original System (AUTO_YOLO/)
Used by `AUTO_YOLO/main.py`. Flat structure, tighter coupling:

- `detector/detector.py` — Combined YOLO inference + distance estimation + semaphore color detection
- `vision/lane_detector.py` — Lane detection returning center X coordinate
- `pid/pid_controller.py` — Classic PID
- `feedback/adaptive_pid.py` — Adjusts gains dynamically: `Kp = clip(0.4 + 0.02*|error|, max=3.0)`
- `motor/pwm_controller.py` — Smooth PWM ramp (max 3% step per tick)
- `fsm/state_machine.py` — Hierarchical FSM: SensorFSM → StopFSM → SemaforoFSM (evaluated in reverse priority)

## YOLO Models

Models are located at:
- `AUTO_YOLO/weights/best.pt` — custom trained object detector
- `best.pt` (root) — latest custom model
- `yolov8n.pt` — pretrained YOLOv8 nano

Traffic light model trained on 7 classes: `green, red, yellow, left, right, straight, stop` (see `traffic_lights/data.yaml`).

## Key Design Decisions

**Simple steering over Ackerman**: Both front wheels rotate at identical angles. Ackerman geometry (`STATE_MACHINE/state_machine.py:get_steering_ackerman`) exists but is not used in the main loop — `controlar_direccion_simple` is the active path.

**Known issue**: `main_refactored.py` line 189 hardcodes `espacio_parking_detectado = True`, causing parking mode to activate automatically whenever `lane_error < 20`. Parking requires real lateral sensors to work correctly.

**Distance units**: ToF serial data arrives in mm, converted to cm (`dist_delante / 10`) before storing in `PerceptionOutput.distancia_obj`. FSM thresholds are in cm: EMERGENCIA < 15cm, STOP_CERCA < 25cm, STOP_LEJOS < 50cm.
