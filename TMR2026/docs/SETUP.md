# Setup — Sim2Real Scale Vehicle (TMR 2026)

## 1. Install system dependencies

```bash
sudo apt update && sudo apt install -y \
  python3-picamera2 python3-libcamera imx500-all \
  python3-pygame python3-rpi.gpio \
  bluetooth bluez python3-dbus \
  python3-smbus2

pip3 install --break-system-packages \
  adafruit-circuitpython-pca9685 \
  adafruit-circuitpython-motor \
  adafruit-circuitpython-vl53l0x \
  adafruit-blinka \
  opencv-python-headless
```

## 2. Enable the alternative I²C bus (GPIO 0 / GPIO 1)

Append to the end of `/boot/firmware/config.txt`:

```
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=0,i2c_gpio_scl=1,i2c_gpio_delay_us=2
```

Reboot and verify: `ls /dev/i2c-*` -> `/dev/i2c-3` should appear.

## 3. Load the YOLOv8 model into the IMX500

```bash
# Check the available models
ls /usr/share/imx500-models/

# The expected file is:
# /usr/share/imx500-models/imx500_network_yolov8n_pp.rpk
# If it has a different name, update IMX500_MODEL_PATH in config.py
```

## 4. Configure the gamepad Bluetooth

```bash
bluetoothctl
  power on
  agent on
  scan on
  # Put the gamepad in pairing mode (hold PS/Xbox + Share)
  pair XX:XX:XX:XX:XX:XX
  trust XX:XX:XX:XX:XX:XX
  connect XX:XX:XX:XX:XX:XX
  exit
```

For it to connect automatically when the Pi powers on, the gamepad must be in
the "trusted" device list.

## 5. Install the systemd service

```bash
# Copy the file to the systemd directory
sudo cp /home/pi/carrito_tmr/systemd/carrito_tmr.service \
        /etc/systemd/system/

# Reload, enable and start
sudo systemctl daemon-reload
sudo systemctl enable carrito_tmr
sudo systemctl start  carrito_tmr

# Watch the logs in real time
journalctl -u carrito_tmr -f
```

## 6. Useful commands during operation

```bash
# Check status
sudo systemctl status carrito_tmr

# Stop manually (useful for debugging)
sudo systemctl stop carrito_tmr

# Run manually without the service (to see prints in the terminal)
cd /home/pi/carrito_tmr
python3 main.py

# Vision debug only, without starting the service
python3 -c "
from hardware.camera_manager import CameraManager
from vision.lane_detector import LaneDetector
import time, cv2
cam = CameraManager(); cam.start(); ld = LaneDetector(debug=True)
while True:
    f = cam.get_latest_frame()
    if f:
        data = ld.process(f.image)
        cv2.imwrite('/tmp/debug.jpg', data.debug_image)
        print(data)
    time.sleep(0.1)
"
```

## 7. On-track calibration

| Parameter | File | What to adjust |
|-----------|------|----------------|
| `CAMERA_FOCAL_LENGTH_PX` | config.py | Measure with a chessboard or a STOP sign at a known distance |
| `STEER_KP/KI/KD` | config.py | Increase Kp until it oscillates, then halve it |
| `PARK_OVERSHOOT_SEC` | config.py | Time how long it takes to pass the 60 cm gap |
| `PARK_REVERSE_LOCK_SEC` | config.py | Adjust until the arc is ~90 deg |
| `SPEED_STRAIGHT / SPEED_CURVE` | config.py | Max speed without motion-blurred images |
| `threshold` in LaneDetector | lane_detector.py | Adjust for the track lighting |

## 8. Project structure

```
TMR2026/
├── main.py                   ← Entry point (Raspberry Pi, 50 Hz loop)
├── main_simulator.py         ← Digital twin (Unity / Sim2Real)
├── config.py                 ← ALL parameters
├── requirements.txt
├── docs/                     ← SETUP, Sim2Real, calibration, deliveries
├── hardware/
│   ├── motor.py              ← IBT-2 soft-start (ACTIVE; RPWM=18, LPWM=13)
│   ├── steering_driver.py    ← PCA9685 + MG90s + Ackermann geometry
│   ├── distance_sensor.py    ← VL53L0X dedicated thread
│   ├── signals.py            ← Turn signals / hazard (2 Hz)
│   └── brake_light.py        ← Brake light
├── control/
│   ├── fsm.py                ← Driving FSM (5 states)
│   ├── parking_fsm.py        ← Battery parking
│   └── pid_controller.py     ← Generic anti-windup PID
├── vision/
│   ├── camera_stream.py      ← Picamera2 RGB→BGR (thread)
│   ├── lane_pipeline.py      ← BEV + HSV + sliding windows (ACTIVE)
│   └── sign_detector.py      ← YOLOv8n + color fallback (ACTIVE)
├── autonomy/                 ← Alternative implementations (not wired in)
├── tests/                    ← pytest (FSM, signals, hysteresis)
├── tools/                    ← test_camera.py, capture, YOLO evaluation
└── systemd/
    └── carrito_tmr.service   ← Auto-start on boot
```
