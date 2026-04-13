# -*- coding: utf-8 -*-
"""
config.py — Parámetros globales del sistema TMR 2026.
Todos los valores físicos, pines y ganancias PID viven aquí.
NO importar hardware real desde este módulo.
"""

# ============================================================
# HARDWARE PINS  (RPi.GPIO, numeración BCM)
# ============================================================
# --- IBT-2 H-Bridge ---
PIN_MOTOR_EN   = 24   # Enable (HIGH = habilitado)
PIN_MOTOR_RPWM = 18   # Forward / acelerar
PIN_MOTOR_LPWM = 13   # Reverse  / frenar
MOTOR_PWM_FREQ = 1000 # Hz — IBT-2 acepta hasta 25 kHz

# --- LEDs status (opcionales, conectar a GND por resistencia 220 Ω) ---
PIN_LED_STOP   = 25   # Parpadea durante parada en STOP
PIN_LED_STATUS = 26   # Estado general del sistema

# ============================================================
# I²C ALTERNATIVO  (GPIO 0 = SDA, GPIO 1 = SCL)
# Uso: busio.I2C(board.D1, board.D0)
# Devices en este bus: PCA9685 + VL53L0X
# ============================================================

# --- PCA9685 ---
PCA9685_I2C_ADDR   = 0x40   # Dirección por defecto
PCA9685_PWM_FREQ   = 50     # Hz (estándar servo analógico)
SERVO_CHANNEL      = 0      # Canal MG90s en la placa

# --- Servo MG90s ---
SERVO_MIN_PULSE_US  = 500   # µs → ~0°
SERVO_MAX_PULSE_US  = 2500  # µs → ~180°
SERVO_CENTER_ANGLE  = 90.0  # grados — ruedas al frente
SERVO_MIN_ANGLE     = 45.0  # grados — giro máximo izquierda
SERVO_MAX_ANGLE     = 135.0 # grados — giro máximo derecha

# ============================================================
# PI AI CAMERA  (Sony IMX500 — aceleración NPU on-chip)
# ============================================================
# Instalar modelos: sudo apt install imx500-all
IMX500_MODEL_PATH = "/usr/share/imx500-models/imx500_network_yolov8n_pp.rpk"

CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
CAMERA_FPS    = 30

# Umbral de confianza para aceptar detecciones del NPU
DETECTION_CONFIDENCE = 0.50

# Nombres de clases COCO que interesan al TMR
# (deben coincidir con el modelo cargado en el IMX500)
CLASSES_OF_INTEREST = {
    "stop sign"    : "STOP",
    "traffic light": "SEMAFORO",
    "person"       : "PERSONA",
    "car"          : "AUTO",
}

# ============================================================
# DIMENSIONES DEL VEHÍCULO  (metros, escala 1:10)
# ============================================================
WHEELBASE    = 0.258   # Distancia entre ejes
TRACK_WIDTH  = 0.172   # Ancho entre ruedas delanteras
CAR_LENGTH   = 0.420   # Largo total
CAR_WIDTH    = 0.200   # Ancho total

MAX_STEERING_ANGLE_DEG = 35.0  # Límite físico del servo en grados desde centro

# ============================================================
# VL53L0X  (sensor ToF frontal)
# ============================================================
TOF_TIMING_BUDGET_US = 20_000  # 20 ms — balance velocidad/precisión
TOF_MAX_RANGE_MM     = 1_200   # mm — fuera de rango = None
TOF_POLL_INTERVAL_S  = 0.020   # 50 Hz lectura del sensor

# ============================================================
# GANANCIAS PID
# ============================================================
# Steering (error de carril → ángulo servo)
STEER_KP = 0.09
STEER_KI = 0.002
STEER_KD = 0.025

# Velocidad (aproximación a señal STOP)
VEL_STOP_KP = 0.035   # salida en % PWM por mm de error
VEL_STOP_KI = 0.001
VEL_STOP_KD = 0.008

# ============================================================
# MODO AUTÓNOMO — VELOCIDADES Y UMBRALES
# ============================================================
SPEED_STRAIGHT   = 65   # % PWM en rectas
SPEED_CURVE      = 38   # % PWM en curvas
SPEED_APPROACH   = 28   # % PWM al aproximarse a señal

# Umbral de curvatura para reducir velocidad (radianes del error de perspectiva)
CURVE_THRESHOLD_RAD = 0.30

# Umbral de error de carril para detectar "carril perdido"
LANE_LOST_THRESHOLD_PX = 280

# ============================================================
# COMPORTAMIENTO SEÑAL STOP
# ============================================================
STOP_BRAKE_START_MM  = 700   # mm — empieza frenado progresivo
STOP_TARGET_MM       = 270   # mm — distancia final de parada (≤30 cm regla TMR)
STOP_TOLERANCE_MM    = 30    # mm — ventana de aceptación
STOP_WAIT_SEC        = 5.0   # segundos de pausa obligatoria
STOP_LED_BLINK_HZ    = 2.0   # frecuencia de parpadeo LED

# Altura real de la señal STOP del TMR (para estimación de distancia con bbox)
STOP_SIGN_REAL_HEIGHT_M = 0.18   # metros
CAMERA_FOCAL_LENGTH_PX  = 490.0  # píxeles — calibrar con tablero si es posible

# ============================================================
# EMERGENCIA
# ============================================================
EMERGENCY_STOP_MM = 120  # mm — parada de emergencia por obstáculo frontal

# ============================================================
# ESTACIONAMIENTO EN BATERÍA
# ============================================================
PARK_SEARCH_SPEED  = 22   # % PWM durante búsqueda
PARK_MANEUVER_SPEED = 18  # % PWM durante maniobra
PARK_MIN_GAP_MM    = 520  # mm — mínimo hueco para considerar la plaza válida
PARK_TARGET_GAP_MM = 600  # mm — ancho nominal del espacio

# Tiempos calibrados de la maniobra (ajustar en pista)
PARK_OVERSHOOT_SEC    = 1.2  # avanzar tras detectar el inicio del hueco
PARK_REVERSE_LOCK_SEC = 2.5  # reversa con giro completo
PARK_REVERSE_STRAIGHT_SEC = 1.0  # reversa derecho para centrar

# ============================================================
# GAMEPAD  (mapeo Xbox / PS4 genérico vía pygame)
# ============================================================
BTN_BACK_TO_MANUAL = 0   # A (Xbox) / Cruz  (PS4)
BTN_VISION_TEST    = 1   # B (Xbox) / Círculo (PS4)
BTN_AUTONOMOUS     = 2   # X (Xbox) / Cuadrado (PS4)

AXIS_STEER    = 3   # Joystick derecho X
AXIS_THROTTLE = 5   # Gatillo R2 (−1 = soltado, +1 = fondo)
AXIS_BRAKE    = 4   # Gatillo L2

JOYSTICK_DEADBAND = 0.08
TRIGGER_DEADBAND  = 0.05

# ============================================================
# RUTAS
# ============================================================
import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR  = os.path.join(BASE_DIR, "logs")
