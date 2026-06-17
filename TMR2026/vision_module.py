"""
vision_module.py — Módulo de visión producción TMR 2026.

Características
---------------
• Dual stream Picamera2: main BGR888 1280×720 (overlay NPU) + lores YUV420 640×480 (carril)
• YOLO11n en NPU IMX500 on-chip — cero inferencia CPU; auto-fallback a EfficientDet Lite0
• Plano Y del stream lores — extracción directa [:H, :W], sin cvtColor (~2 ms)
• Detección de carril por histograma de columnas con PID (Kp/Ki/Kd desde YAML)
• FSM de 9 estados: INIT→CALIBRATING→LANE_FOLLOWING→SIGN_APPROACH→SIGN_BRAKING
  →STOPPED_AT_SIGN→RESUMING→END_OF_TRACK→TELEOP_OVERRIDE
• GPIO hazard (parpadeo 2 Hz) + direccionales + botón teleop físico — lgpio chip 4 (Pi 5)
• Teleop UDP JSON en puerto 5005
• Vigilante FPS: warn < 25, reset < 15 FPS
• Todos los parámetros tunables en vision_config.yaml — recarga sin recompilar

Instalación
-----------
  sudo apt install imx500-all python3-picamera2
  pip install opencv-python-headless numpy pyyaml

Uso
---
  vm = VisionModule()
  vm.start()
  s = vm.get_state()          # VisionState thread-safe
  y = vm.get_latest_y_plane() # ndarray grayscale 640×480 para lane detector externo
  vm.stop()
"""

from __future__ import annotations

import json
import logging
import os
import socket
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

log = logging.getLogger("tmr.vision")


def _load_cfg() -> dict:
    """Carga vision_config.yaml junto a este archivo. Devuelve {} si falla."""
    path = Path(__file__).parent / "vision_config.yaml"
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        log.info("Configuración cargada: %s", path)
        return data
    except Exception as exc:
        log.warning("vision_config.yaml no disponible (%s) — usando defaults", exc)
        return {}

_CFG = _load_cfg()


def _cv(section: str, key: str, default):
    """Acceso seguro a parámetro de configuración con fallback a default."""
    try:
        return _CFG[section][key]
    except (KeyError, TypeError):
        return default


_MAIN_W        = _cv("camera", "main_w", 1280)
_MAIN_H        = _cv("camera", "main_h", 720)
_LORES_W       = _cv("camera", "lores_w", 640)
_LORES_H       = _cv("camera", "lores_h", 480)
_FPS           = _cv("camera", "fps", 30)
_AWB_WARMUP_S  = _cv("camera", "awb_warmup_s", 1.5)
_BUF_COUNT     = _cv("camera", "buffer_count", 4)
_MODEL_CANDS: list[str] = _cv("camera", "model_candidates", [
    "/usr/share/imx500-models/imx500_network_yolo11n_pp.rpk",
    "/usr/share/imx500-models/imx500_network_yolov8n_pp.rpk",
    "/usr/share/imx500-models/imx500_network_efficientdet_lite0_pp.rpk",
])
_MODEL_PATH = next((m for m in _MODEL_CANDS if os.path.exists(m)), _MODEL_CANDS[-1])

_ROI_Y0        = _cv("lane", "roi_y_start", 240)
_ROI_Y1        = _cv("lane", "roi_y_end",   480)
_THRESH        = _cv("lane", "threshold", 200)
_MORPH_K       = _cv("lane", "morph_kernel_size", 3)
_MIN_PEAK      = _cv("lane", "min_peak_pixels", 15)
_LOST_LIMIT    = _cv("lane", "lost_frames_limit", 8)
_lane_pid_cfg  = _cv("lane", "pid", {})
_PID_KP        = _lane_pid_cfg.get("kp", 0.8)
_PID_KI        = _lane_pid_cfg.get("ki", 0.05)
_PID_KD        = _lane_pid_cfg.get("kd", 0.15)
_PID_MAX_I     = _lane_pid_cfg.get("max_integral", 30.0)

_SIGN_CONF     = _cv("signs", "confidence_threshold", 0.55)
_SIGN_MIN_AREA = _cv("signs", "min_bbox_area_px2", 600)
_SIGN_REAL_H   = _cv("signs", "sign_real_height_m", 0.10)
_FOCAL_PX      = _cv("signs", "focal_length_px", 1550)
_HYST_F        = _cv("signs", "hysteresis_frames", 3)
_IOU_THRESH    = _cv("signs", "iou_threshold", 0.50)
_APPROACH_M    = _cv("signs", "approach_dist_m", 0.50)
_BRAKE_M       = _cv("signs", "brake_dist_m", 0.30)
_SIGN_ROI_Y    = _cv("signs", "sign_roi_y_end", 200)

_STOP_WAIT_S   = _cv("fsm", "stop_wait_s", 5.0)
_APPROACH_F    = _cv("fsm", "speed_approach_factor", 0.60)
_RESUME_RAMP_S = _cv("fsm", "resume_ramp_s", 1.5)

_GPIO_HAZ_L    = _cv("gpio", "led_hazard_left",  5)
_GPIO_HAZ_R    = _cv("gpio", "led_hazard_right", 6)
_GPIO_TURN_L   = _cv("gpio", "led_turn_left",   19)
_GPIO_TURN_R   = _cv("gpio", "led_turn_right",  20)
_GPIO_BTN      = _cv("gpio", "btn_teleop",      21)
_HAZ_HZ        = _cv("gpio", "hazard_blink_hz", 2.0)

_UDP_PORT      = _cv("teleop", "udp_port", 5005)
_UDP_HOST      = _cv("teleop", "udp_host", "0.0.0.0")

_FPS_WARN      = _cv("performance", "fps_warn_threshold",  25)
_FPS_RESET     = _cv("performance", "fps_reset_threshold", 15)
_FPS_WIN_S     = _cv("performance", "fps_check_window_s",  2.0)

_COCO: dict[str, str] = {
    "stop sign":     "STOP",
    "traffic light": "SEMAFORO",
    "person":        "PERSONA",
    "car":           "AUTO",
}


class VisionFSMState(Enum):
    INIT             = auto()
    CALIBRATING      = auto()
    LANE_FOLLOWING   = auto()
    SIGN_APPROACH    = auto()
    SIGN_BRAKING     = auto()
    STOPPED_AT_SIGN  = auto()
    RESUMING         = auto()
    END_OF_TRACK     = auto()
    TELEOP_OVERRIDE  = auto()


@dataclass
class LaneResult:
    """Resultado del detector de carril por histograma."""
    error:      float
    confidence: float
    left_x:     Optional[int] = None
    right_x:    Optional[int] = None
    center_x:   Optional[int] = None


@dataclass
class DetectionResult:
    """Detección confirmada del NPU."""
    label:      str
    class_id:   int
    confidence: float
    x1: int; y1: int; x2: int; y2: int

    @property
    def width(self)  -> int: return self.x2 - self.x1
    @property
    def height(self) -> int: return self.y2 - self.y1
    @property
    def area(self)   -> int: return self.width * self.height
    @property
    def cx(self)     -> int: return (self.x1 + self.x2) // 2
    @property
    def cy(self)     -> int: return (self.y1 + self.y2) // 2


@dataclass
class VisionState:
    """
    Estado semántico completo publicado por VisionModule cada frame.

    Inmutable una vez publicado — no mutar después de asignarlo al estado.
    Thread-safe: reemplazado atómicamente bajo lock, el objeto en sí no cambia.
    """
    lane_error:       float = 0.0
    lane_confidence:  float = 0.0
    steer_correction: float = 0.0

    stop_detected:    bool           = False
    stop_distance_mm: Optional[float] = None
    stop_bbox:        Optional[tuple] = None

    traffic_light_color: str   = "unknown"
    traffic_light_conf:  float = 0.0

    person_detected:  bool           = False
    car_detected:     bool           = False
    car_bbox:         Optional[tuple] = None
    car_in_lane:      bool           = False
    car_in_park_zone: bool           = False

    fsm_state:    VisionFSMState = VisionFSMState.INIT
    speed_factor: float          = 1.0

    fps:             float = 0.0
    frame_id:        int   = 0
    timestamp:       float = field(default_factory=time.monotonic)
    raw_detections:  list  = field(default_factory=list)


class VisionModule:
    """
    Módulo de visión TMR 2026 — IMX500 NPU + histograma de carril + FSM 9 estados.

    Uso típico::

        vm = VisionModule()
        vm.start()
        while True:
            s = vm.get_state()
            servo.set_angle(90 + s.steer_correction)
            motor.set_throttle(BASE_PWM * s.speed_factor)
        vm.stop()
    """

    def __init__(self, display_overlay: bool = False) -> None:
        self._display_overlay = display_overlay
        self._model_path = _MODEL_PATH

        self._imx500 = None
        self._picam2 = None

        self._state      = VisionState()
        self._state_lock = threading.Lock()

        self._latest_frame:   Optional[np.ndarray] = None
        self._latest_y_plane: Optional[np.ndarray] = None
        self._frame_lock   = threading.Lock()
        self._y_lock       = threading.Lock()

        self._stop_event = threading.Event()
        self._cap_thread: Optional[threading.Thread] = None

        self._fsm             = VisionFSMState.INIT
        self._stopped_since   = 0.0
        self._resume_start    = 0.0
        self._lost_frames     = 0

        self._pid_integral   = 0.0
        self._pid_prev_err   = 0.0
        self._pid_prev_t     = time.monotonic()

        self._sign_hyst: dict[str, dict] = {
            v: {"bbox": None, "count": 0, "conf": 0.0}
            for v in _COCO.values()
        }

        self._class_map: dict[int, str] = {}

        self._fps_ts:       list[float] = []
        self._fps_low_since = 0.0

        self._cap_fails = 0

        self._gpio_h:    Optional[int] = None
        self._gpio_ok    = False
        self._haz_event  = threading.Event()
        self._haz_thread: Optional[threading.Thread] = None

        self._teleop_active = False
        self._teleop_sock:   Optional[socket.socket] = None
        self._teleop_thread: Optional[threading.Thread] = None

        log.info("VisionModule creado — modelo: %s", self._model_path)


    def start(self) -> None:
        """Inicializa GPIO, UDP, cámara y arranca el hilo de captura."""
        self._setup_gpio()
        self._start_teleop_udp()
        self._init_camera()
        self._stop_event.clear()
        self._fsm = VisionFSMState.LANE_FOLLOWING
        self._cap_thread = threading.Thread(
            target=self._capture_loop,
            name="VisionCapture",
            daemon=True,
        )
        self._cap_thread.start()
        log.info("VisionModule iniciado — hilo de captura activo.")

    def stop(self) -> None:
        """Detiene el hilo de captura y libera todos los recursos."""
        self._stop_event.set()
        if self._cap_thread:
            self._cap_thread.join(timeout=4.0)
        self.hazard_lights_off()
        if self._teleop_sock:
            try: self._teleop_sock.close()
            except Exception: pass
        if self._picam2 is not None:
            try: self._picam2.stop()
            except Exception: pass
        self._release_gpio()
        log.info("VisionModule detenido.")


    def get_state(self) -> VisionState:
        """Retorna el VisionState más reciente. Nunca bloquea más de µs."""
        with self._state_lock:
            return self._state

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Frame BGR (1280×720) del stream principal. None hasta el primer frame."""
        with self._frame_lock:
            return self._latest_frame

    def get_latest_y_plane(self) -> Optional[np.ndarray]:
        """
        Plano Y (640×480, uint8, grayscale) del stream lores.

        Extraído directamente del array YUV420 sin cvtColor.
        Útil para un detector de carril externo adicional.
        """
        with self._y_lock:
            return self._latest_y_plane

    def recalibrate_lighting(self) -> None:
        """Re-calibra AE/AWB en vivo. Bloquea al invocador ~awb_warmup_s."""
        if self._picam2 is None:
            return
        log.info("Recalibrando AE/AWB...")
        self._picam2.set_controls({"AeEnable": True, "AwbEnable": True})
        time.sleep(_AWB_WARMUP_S)
        self._lock_ae_awb()
        log.info("Recalibración completada.")

    def teleop_release(self) -> None:
        """Cancela el modo teleop y vuelve al control autónomo."""
        self._teleop_active = False
        log.info("Teleop liberado — retorno a modo autónomo.")

    def hazard_lights_on(self) -> None:
        """Activa intermitentes de peligro (parpadeo a _HAZ_HZ)."""
        if not self._gpio_ok:
            return
        if self._haz_thread and self._haz_thread.is_alive():
            return
        self._haz_event.clear()
        self._haz_thread = threading.Thread(
            target=self._hazard_blink_loop,
            name="HazardBlink",
            daemon=True,
        )
        self._haz_thread.start()

    def hazard_lights_off(self) -> None:
        """Apaga intermitentes de peligro."""
        self._haz_event.set()
        if self._gpio_ok and self._gpio_h is not None:
            try:
                import lgpio
                lgpio.gpio_write(self._gpio_h, _GPIO_HAZ_L, 0)
                lgpio.gpio_write(self._gpio_h, _GPIO_HAZ_R, 0)
            except Exception:
                pass

    def set_turn_signal(self, direction: Optional[str]) -> None:
        """Activa direccional. direction: 'left' | 'right' | None (apagar)."""
        if not self._gpio_ok or self._gpio_h is None:
            return
        try:
            import lgpio
            lgpio.gpio_write(self._gpio_h, _GPIO_TURN_L,
                             1 if direction == "left"  else 0)
            lgpio.gpio_write(self._gpio_h, _GPIO_TURN_R,
                             1 if direction == "right" else 0)
        except Exception:
            pass


    def _init_camera(self) -> None:
        from picamera2 import Picamera2
        from picamera2.devices.imx500 import IMX500

        self._imx500 = IMX500(self._model_path)
        self._picam2 = Picamera2(self._imx500.camera_num)

        cfg = self._picam2.create_preview_configuration(
            main={
                "format": "BGR888",
                "size":   (_MAIN_W, _MAIN_H),
            },
            lores={
                "format": "YUV420",
                "size":   (_LORES_W, _LORES_H),
            },
            controls={
                "FrameDurationLimits": (33333, 33333),
                "AeEnable":      True,
                "AwbEnable":     True,
                "AwbMode":       4,
                "Contrast":      1.5,
                "Saturation":    1.8,
                "Sharpness":     4.0,
                "NoiseReductionMode": 2,
            },
            buffer_count=_BUF_COUNT,
        )
        self._picam2.configure(cfg)

        if self._display_overlay:
            self._picam2.pre_callback = self._overlay_callback

        self._picam2.start()

        log.info("Estabilizando AE/AWB (%.1f s)...", _AWB_WARMUP_S)
        time.sleep(_AWB_WARMUP_S)
        self._lock_ae_awb()

        try:
            self._picam2.set_controls({"AfMode": 2, "AfSpeed": 1})
            log.debug("Autoenfoque continuo activado.")
        except Exception:
            log.debug("Autoenfoque no disponible — enfoque fijo.")

        self._build_class_map()
        log.info("Cámara lista: main=%d×%d  lores=%d×%d  @%d FPS",
                 _MAIN_W, _MAIN_H, _LORES_W, _LORES_H, _FPS)

    def _lock_ae_awb(self) -> None:
        """Lee metadatos actuales y fija exposición + balance de blancos."""
        try:
            meta   = self._picam2.capture_metadata()
            exp    = meta.get("ExposureTime")
            gain   = meta.get("AnalogueGain")
            cgains = meta.get("ColourGains")

            ctrl: dict = {"AeEnable": False}
            if exp    is not None: ctrl["ExposureTime"] = exp
            if gain   is not None: ctrl["AnalogueGain"] = gain
            if cgains is not None:
                ctrl["AwbEnable"]   = False
                ctrl["ColourGains"] = tuple(cgains)

            self._picam2.set_controls(ctrl)
            log.info("AE/AWB bloqueados — exp=%s µs  gain=%.2f  cgains=%s",
                     exp, gain or 0.0, cgains)
        except Exception as exc:
            log.warning("No se pudo bloquear AE/AWB: %s", exc)

    def _build_class_map(self) -> None:
        """Construye {class_id → etiqueta TMR} desde los intrínsecos del modelo."""
        try:
            intr = self._imx500.network_intrinsics
            if intr and intr.labels:
                for idx, lbl in enumerate(intr.labels):
                    low = lbl.strip().lower()
                    for coco, friendly in _COCO.items():
                        if coco in low:
                            self._class_map[idx] = friendly
                            break
            if self._class_map:
                log.info("Clases mapeadas: %s", self._class_map)
                print(f"[VISION] Clases mapeadas: {self._class_map}")
            else:
                log.warning("Mapa de clases vacío — modelo sin etiquetas COCO conocidas")
                if intr and intr.labels:
                    print("[VISION] Primeras 10 etiquetas del modelo:", intr.labels[:10])
        except Exception as exc:
            log.warning("No se pudo construir mapa de clases: %s", exc)


    def _capture_loop(self) -> None:
        """
        Bucle principal: captura → plano Y → detección carril → NPU → FSM → publish.

        Todo el trabajo de estado mutable de la FSM ocurre en este único hilo;
        get_state() solo necesita un lock de lectura ligero.
        """
        dbg_count = 0
        log.info("Hilo de captura arrancado.")

        while not self._stop_event.is_set():

            if self._gpio_ok:
                self._poll_teleop_button()

            try:
                req       = self._picam2.capture_request()
                main_arr  = req.make_array("main")
                lores_arr = req.make_array("lores")
                meta      = req.get_metadata()
                req.release()
            except Exception as exc:
                self._cap_fails += 1
                log.warning("Fallo captura %d/3: %s", self._cap_fails, exc)
                if self._cap_fails >= 3:
                    log.error("Reconectando cámara por fallos consecutivos...")
                    self._reconnect()
                time.sleep(0.05)
                continue

            self._cap_fails = 0

            with self._frame_lock:
                self._latest_frame = main_arr

            y_plane = lores_arr[:_LORES_H, :]
            with self._y_lock:
                self._latest_y_plane = y_plane

            lane = self._detect_lane(y_plane)

            raw_dets  = self._parse_npu(meta, main_arr.shape)
            sign_dets = self._filter_signs(raw_dets)

            dbg_count += 1
            if dbg_count % 60 == 0 and raw_dets:
                print("[VISION] NPU raw:", ", ".join(
                    f"{d.label}({d.confidence:.2f})" for d in raw_dets))

            stop_det = next((d for d in sign_dets if d.label == "STOP"), None)
            dist_mm: Optional[float] = None
            if stop_det is not None and stop_det.height >= 5:
                dist_mm = (_SIGN_REAL_H * _FOCAL_PX / stop_det.height) * 1000.0

            new_state = self._build_state(lane, sign_dets, dist_mm, raw_dets)
            self._update_fps(new_state)

            with self._state_lock:
                self._state = new_state


    def _detect_lane(self, y_plane: np.ndarray) -> LaneResult:
        """
        Pipeline de detección de carril sobre el plano Y (640×480, uint8).

        1. ROI inferior [_ROI_Y0 : _ROI_Y1] — excluye zona de señales de tráfico.
        2. cv2.threshold → máscara binaria (líneas blancas brillantes).
        3. MORPH_OPEN 3×3 — elimina reflejos y ruido puntual.
        4. Histograma de columnas → picos izq/der → centro del carril.

        Tiempo típico: ~2 ms en Pi 5 a 640×480.
        """
        roi = y_plane[_ROI_Y0:_ROI_Y1, :]

        _, binary = cv2.threshold(roi, _THRESH, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (_MORPH_K, _MORPH_K))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        histogram = np.sum(binary, axis=0).astype(np.int32)

        W   = histogram.shape[0]
        mid = W // 2

        lp = int(np.argmax(histogram[:mid]))
        rp = int(np.argmax(histogram[mid:])) + mid

        lv = int(histogram[lp])
        rv = int(histogram[rp])

        l_ok = lv >= _MIN_PEAK
        r_ok = rv >= _MIN_PEAK

        if l_ok and r_ok:
            cx         = (lp + rp) // 2
            confidence = 1.0
        elif l_ok:
            cx         = lp + 160
            confidence = 0.5
        elif r_ok:
            cx         = rp - 160
            confidence = 0.5
        else:
            return LaneResult(error=0.0, confidence=0.0)

        error = float(cx - mid)
        return LaneResult(
            error=error, confidence=confidence,
            left_x=lp if l_ok else None,
            right_x=rp if r_ok else None,
            center_x=cx,
        )


    def _parse_npu(
        self, metadata: dict, img_shape: tuple
    ) -> list[DetectionResult]:
        """
        Extrae detecciones del tensor de salida del IMX500.

        Soporta dos formatos:
        - YOLO11n _pp (3 outputs): boxes(x1y1x2y2), scores, classes — normalizado [0-1]
        - EfficientDet / TF (4 outputs): boxes(ymxnymxn), classes, scores, count
        """
        np_out = self._imx500.get_outputs(metadata, add_batch=True)
        if np_out is None:
            return []

        ih, iw = img_shape[:2]

        try:
            if len(np_out) >= 4:
                boxes   = np_out[0][0]
                classes = np_out[1][0]
                scores  = np_out[2][0]
                count   = int(np_out[3][0])
                tf_fmt  = True
            else:
                boxes   = np_out[0][0]
                scores  = np_out[1][0]
                classes = np_out[2][0]
                count   = len(scores)
                tf_fmt  = False
        except (IndexError, TypeError):
            return []

        results: list[DetectionResult] = []
        for i in range(min(count, len(scores))):
            score  = float(scores[i])
            cls_id = int(classes[i])

            if score < _SIGN_CONF:
                continue
            label = self._class_map.get(cls_id)
            if label is None:
                continue

            b = boxes[i]
            if tf_fmt:
                y1 = int(b[0]*ih); x1 = int(b[1]*iw)
                y2 = int(b[2]*ih); x2 = int(b[3]*iw)
            else:
                x1 = int(b[0]*iw); y1 = int(b[1]*ih)
                x2 = int(b[2]*iw); y2 = int(b[3]*ih)

            x1, x2 = sorted([max(0, x1), min(iw-1, x2)])
            y1, y2 = sorted([max(0, y1), min(ih-1, y2)])

            results.append(DetectionResult(
                label=label, class_id=cls_id, confidence=score,
                x1=x1, y1=y1, x2=x2, y2=y2,
            ))
        return results


    def _filter_signs(
        self, raw: list[DetectionResult]
    ) -> list[DetectionResult]:
        """
        Filtra señales de tráfico aplicando en orden:
        1. ROI superior (cy < _SIGN_ROI_Y) para STOP y SEMAFORO.
        2. Área mínima > _SIGN_MIN_AREA px².
        3. Histéresis de _HYST_F frames con IoU > _IOU_THRESH.
        """
        cands: dict[str, DetectionResult] = {}
        for d in raw:
            if d.label in ("STOP", "SEMAFORO") and d.cy > _SIGN_ROI_Y:
                continue
            if d.area < _SIGN_MIN_AREA:
                continue
            if d.label not in cands or d.confidence > cands[d.label].confidence:
                cands[d.label] = d

        confirmed: list[DetectionResult] = []

        for label, h in self._sign_hyst.items():
            det = cands.get(label)
            if det is not None:
                if h["bbox"] is not None and h["count"] > 0:
                    iou = _iou(*h["bbox"], det.x1, det.y1, det.x2, det.y2)
                    h["count"] = h["count"] + 1 if iou >= _IOU_THRESH else 1
                else:
                    h["count"] = 1
                h["bbox"] = (det.x1, det.y1, det.x2, det.y2)
                h["conf"] = det.confidence
            else:
                h["count"] = max(0, h["count"] - 1)
                if h["count"] == 0:
                    h["bbox"] = None

            if h["count"] >= _HYST_F and det is not None:
                confirmed.append(det)

        return confirmed


    def _build_state(
        self,
        lane:      LaneResult,
        sign_dets: list[DetectionResult],
        dist_mm:   Optional[float],
        raw_dets:  list[DetectionResult],
    ) -> VisionState:
        """Combina los resultados del frame en un VisionState y avanza la FSM."""
        s = VisionState(raw_detections=raw_dets)

        s.lane_error      = lane.error
        s.lane_confidence = lane.confidence
        s.steer_correction = self._pid_step(lane.error)

        if lane.confidence < 0.15:
            self._lost_frames += 1
        else:
            self._lost_frames = 0

        for det in sign_dets:
            if det.label == "STOP":
                s.stop_detected    = True
                s.stop_bbox        = (det.x1, det.y1, det.x2, det.y2)
                s.stop_distance_mm = dist_mm
            elif det.label == "SEMAFORO":
                frm = self._latest_frame
                if frm is not None:
                    color, conf = _classify_light(frm, det)
                    s.traffic_light_color = color
                    s.traffic_light_conf  = conf
            elif det.label == "PERSONA":
                s.person_detected = True
            elif det.label == "AUTO":
                s.car_detected = True
                s.car_bbox     = (det.x1, det.y1, det.x2, det.y2)
                cx = det.cx
                s.car_in_lane    = (abs(cx - _MAIN_W / 2) < _MAIN_W * 0.35
                                    and det.area >= 2500)
                s.car_in_park_zone = cx > _MAIN_W * 0.55

        s.fsm_state, s.speed_factor = self._step_fsm(s, dist_mm)

        if self._gpio_ok:
            if s.fsm_state in (VisionFSMState.STOPPED_AT_SIGN,
                               VisionFSMState.END_OF_TRACK):
                self.hazard_lights_on()
            elif s.fsm_state != VisionFSMState.TELEOP_OVERRIDE:
                self.hazard_lights_off()

        return s


    def _pid_step(self, error: float) -> float:
        """Un paso del PID proporcional-integral-derivativo del carril."""
        now = time.monotonic()
        dt  = max(now - self._pid_prev_t, 1e-3)
        self._pid_prev_t = now

        self._pid_integral += error * dt
        self._pid_integral  = max(-_PID_MAX_I, min(_PID_MAX_I, self._pid_integral))

        deriv = (error - self._pid_prev_err) / dt
        self._pid_prev_err = error

        return _PID_KP * error + _PID_KI * self._pid_integral + _PID_KD * deriv


    def _step_fsm(
        self,
        s:       VisionState,
        dist_mm: Optional[float],
    ) -> tuple[VisionFSMState, float]:
        """
        Ejecuta un paso de la FSM y devuelve (nuevo_estado, factor_velocidad [0-1]).

        Prioridades (de mayor a menor):
          1. Teleop override (UDP o botón físico)
          2. Fin de pista   (carril perdido ≥ _LOST_LIMIT frames)
          3. Transiciones de señal STOP
          4. Seguimiento de carril normal
        """
        now = time.monotonic()

        if self._teleop_active:
            if self._fsm != VisionFSMState.TELEOP_OVERRIDE:
                log.info("FSM → TELEOP_OVERRIDE")
                self._fsm = VisionFSMState.TELEOP_OVERRIDE
            return VisionFSMState.TELEOP_OVERRIDE, 1.0

        if self._lost_frames >= _LOST_LIMIT:
            if self._fsm not in (VisionFSMState.END_OF_TRACK,
                                 VisionFSMState.STOPPED_AT_SIGN):
                log.warning("FSM → END_OF_TRACK (carril perdido %d frames)",
                            self._lost_frames)
                self._fsm = VisionFSMState.END_OF_TRACK
            return VisionFSMState.END_OF_TRACK, 0.0

        if self._fsm == VisionFSMState.END_OF_TRACK:
            log.info("FSM END_OF_TRACK → LANE_FOLLOWING (carril recuperado)")
            self._fsm = VisionFSMState.LANE_FOLLOWING

        if self._fsm == VisionFSMState.TELEOP_OVERRIDE:
            log.info("FSM TELEOP_OVERRIDE → LANE_FOLLOWING")
            self._fsm = VisionFSMState.LANE_FOLLOWING

        match self._fsm:

            case VisionFSMState.CALIBRATING:
                self._fsm = VisionFSMState.LANE_FOLLOWING
                return VisionFSMState.LANE_FOLLOWING, 1.0

            case VisionFSMState.LANE_FOLLOWING:
                if s.stop_detected and dist_mm is not None:
                    dist_m = dist_mm / 1000.0
                    if dist_m <= _APPROACH_M:
                        log.info("FSM LANE_FOLLOWING→SIGN_APPROACH  dist=%.0f mm", dist_mm)
                        self._fsm = VisionFSMState.SIGN_APPROACH
                        return VisionFSMState.SIGN_APPROACH, _APPROACH_F
                return VisionFSMState.LANE_FOLLOWING, 1.0

            case VisionFSMState.SIGN_APPROACH:
                if not s.stop_detected:
                    log.info("FSM SIGN_APPROACH→LANE_FOLLOWING (señal perdida)")
                    self._fsm = VisionFSMState.LANE_FOLLOWING
                    return VisionFSMState.LANE_FOLLOWING, 1.0
                if dist_mm is not None and dist_mm / 1000.0 <= _BRAKE_M:
                    log.info("FSM SIGN_APPROACH→SIGN_BRAKING  dist=%.0f mm", dist_mm)
                    self._fsm = VisionFSMState.SIGN_BRAKING
                    return VisionFSMState.SIGN_BRAKING, 0.3
                return VisionFSMState.SIGN_APPROACH, _APPROACH_F

            case VisionFSMState.SIGN_BRAKING:
                if dist_mm is None:
                    return VisionFSMState.SIGN_BRAKING, 0.1
                target_mm = _BRAKE_M * 1000.0 * 0.5
                if dist_mm <= target_mm:
                    log.info("FSM SIGN_BRAKING→STOPPED_AT_SIGN  dist=%.0f mm", dist_mm)
                    self._fsm        = VisionFSMState.STOPPED_AT_SIGN
                    self._stopped_since = now
                    return VisionFSMState.STOPPED_AT_SIGN, 0.0
                factor = max(0.05, min(0.3,
                             dist_mm / (_APPROACH_M * 1000.0) * 0.3))
                return VisionFSMState.SIGN_BRAKING, factor

            case VisionFSMState.STOPPED_AT_SIGN:
                elapsed = now - self._stopped_since
                if elapsed >= _STOP_WAIT_S:
                    log.info("FSM STOPPED_AT_SIGN→RESUMING  espera=%.2f s", elapsed)
                    self._fsm        = VisionFSMState.RESUMING
                    self._resume_start = now
                return VisionFSMState.STOPPED_AT_SIGN, 0.0

            case VisionFSMState.RESUMING:
                t = min((now - self._resume_start) / _RESUME_RAMP_S, 1.0)
                if t >= 1.0:
                    log.info("FSM RESUMING→LANE_FOLLOWING")
                    self._fsm = VisionFSMState.LANE_FOLLOWING
                    for h in self._sign_hyst.values():
                        h["count"] = 0; h["bbox"] = None
                    return VisionFSMState.LANE_FOLLOWING, 1.0
                return VisionFSMState.RESUMING, 0.3 + t * 0.7

            case _:
                return self._fsm, 1.0


    def _setup_gpio(self) -> None:
        h = None
        try:
            import lgpio
            h = lgpio.gpiochip_open(4)
            for pin in (_GPIO_HAZ_L, _GPIO_HAZ_R, _GPIO_TURN_L, _GPIO_TURN_R):
                lgpio.gpio_claim_output(h, pin, 0, 0)
            lgpio.gpio_claim_input(h, _GPIO_BTN, lgpio.SET_PULL_UP)
            self._gpio_h  = h
            self._gpio_ok = True
            log.info("GPIO listo — chip 4, pines: haz=%d/%d turn=%d/%d btn=%d",
                     _GPIO_HAZ_L, _GPIO_HAZ_R, _GPIO_TURN_L, _GPIO_TURN_R, _GPIO_BTN)
        except Exception as exc:
            log.warning("GPIO no disponible: %s", exc)
            self._gpio_ok = False
            if h is not None:
                try:
                    import lgpio; lgpio.gpiochip_close(h)
                except Exception: pass

    def _release_gpio(self) -> None:
        if self._gpio_h is not None:
            try:
                import lgpio
                lgpio.gpiochip_close(self._gpio_h)
            except Exception: pass
            self._gpio_h  = None
            self._gpio_ok = False

    def _poll_teleop_button(self) -> None:
        """Lee el botón físico (activo en LOW) desde el hilo de captura."""
        try:
            import lgpio
            val = lgpio.gpio_read(self._gpio_h, _GPIO_BTN)
            if val == 0 and not self._teleop_active:
                log.info("Botón teleop presionado → TELEOP_OVERRIDE")
                self._teleop_active = True
            elif val == 1 and self._teleop_active:
                log.info("Botón teleop liberado → modo autónomo")
                self._teleop_active = False
        except Exception:
            pass

    def _hazard_blink_loop(self) -> None:
        """Hilo: alterna hazard LEDs a _HAZ_HZ Hz hasta que _haz_event se activa."""
        if not self._gpio_ok or self._gpio_h is None:
            return
        try:
            import lgpio
            half = 0.5 / _HAZ_HZ
            state = 0
            while not self._haz_event.is_set():
                state ^= 1
                lgpio.gpio_write(self._gpio_h, _GPIO_HAZ_L, state)
                lgpio.gpio_write(self._gpio_h, _GPIO_HAZ_R, state)
                time.sleep(half)
        except Exception as exc:
            log.warning("hazard_blink_loop: %s", exc)
        finally:
            try:
                import lgpio
                lgpio.gpio_write(self._gpio_h, _GPIO_HAZ_L, 0)
                lgpio.gpio_write(self._gpio_h, _GPIO_HAZ_R, 0)
            except Exception: pass


    def _start_teleop_udp(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((_UDP_HOST, _UDP_PORT))
            sock.settimeout(0.5)
            self._teleop_sock = sock
            self._teleop_thread = threading.Thread(
                target=self._teleop_udp_loop,
                name="TeleopUDP",
                daemon=True,
            )
            self._teleop_thread.start()
            log.info("Teleop UDP en %s:%d", _UDP_HOST, _UDP_PORT)
        except Exception as exc:
            log.warning("No se pudo iniciar teleop UDP: %s", exc)

    def _teleop_udp_loop(self) -> None:
        """
        Escucha comandos UDP JSON.

        Protocolo:
          {"cmd":"teleop", "turn":"left"|"right"|null}  → activa teleop
          {"cmd":"auto"}                                 → vuelve a autónomo
        """
        while not self._stop_event.is_set():
            try:
                data, _ = self._teleop_sock.recvfrom(256)
                cmd = json.loads(data.decode("utf-8", errors="replace"))
                if cmd.get("cmd") == "teleop":
                    self._teleop_active = True
                    self.set_turn_signal(cmd.get("turn"))
                    log.info("Teleop UDP activado (turn=%s)", cmd.get("turn"))
                elif cmd.get("cmd") == "auto":
                    self._teleop_active = False
                    self.set_turn_signal(None)
                    log.info("Teleop UDP: retorno a autónomo")
            except socket.timeout:
                pass
            except Exception as exc:
                if not self._stop_event.is_set():
                    log.debug("Teleop UDP recv error: %s", exc)


    def _update_fps(self, s: VisionState) -> None:
        now = time.monotonic()
        self._fps_ts.append(now)
        cutoff = now - _FPS_WIN_S
        while self._fps_ts and self._fps_ts[0] < cutoff:
            self._fps_ts.pop(0)

        fps = len(self._fps_ts) / _FPS_WIN_S
        s.fps       = fps
        s.frame_id  = self._state.frame_id + 1
        s.timestamp = now

        if fps < _FPS_RESET_THRESH:
            if self._fps_low_since == 0.0:
                self._fps_low_since = now
            elif now - self._fps_low_since > _FPS_WIN_S * 2:
                log.error("FPS crítico %.1f — forzando reset de cámara", fps)
                self._fps_low_since = now
                threading.Thread(target=self._reconnect, daemon=True).start()
        elif fps < _FPS_WARN:
            if self._fps_low_since == 0.0:
                self._fps_low_since = now
            elif now - self._fps_low_since > _FPS_WIN_S:
                log.warning("FPS bajo: %.1f (warn=%d reset=%d)",
                            fps, _FPS_WARN, _FPS_RESET_THRESH)
                self._fps_low_since = now
        else:
            self._fps_low_since = 0.0


    def _overlay_callback(self, request) -> None:
        """Dibuja bboxes sobre el buffer DMA. Invocado por Picamera2 pre-entrega."""
        from picamera2 import MappedArray
        _C = {"STOP": (0,0,255), "SEMAFORO": (0,200,255),
              "PERSONA": (255,100,0), "AUTO": (200,0,200)}
        with self._state_lock:
            dets = list(self._state.raw_detections)
        with MappedArray(request, "main") as m:
            for d in dets:
                c = _C.get(d.label, (180,180,180))
                cv2.rectangle(m.array, (d.x1, d.y1), (d.x2, d.y2), c, 2)
                cv2.putText(m.array, f"{d.label} {d.confidence:.0%}",
                            (d.x1, max(d.y1-6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv2.LINE_AA)


    def _reconnect(self) -> None:
        log.warning("Reconectando cámara...")
        try: self._picam2.stop()
        except Exception: pass
        time.sleep(1.0)
        try:
            self._init_camera()
            self._cap_fails = 0
            log.info("Reconexión de cámara exitosa.")
        except Exception as exc:
            log.error("Reconexión fallida: %s", exc)


def _iou(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2) -> float:
    """Intersection over Union entre dos bboxes."""
    ix1 = max(ax1, bx1); iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2); iy2 = min(ay2, by2)
    iw = max(0, ix2-ix1); ih_ = max(0, iy2-iy1)
    inter = iw * ih_
    area_a = max(0, ax2-ax1) * max(0, ay2-ay1)
    area_b = max(0, bx2-bx1) * max(0, by2-by1)
    union  = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def _classify_light(
    frame: np.ndarray, det: "DetectionResult"
) -> tuple[str, float]:
    """Clasifica color de semáforo en el ROI del bbox por máscaras HSV."""
    pad = 4
    x1 = max(0, det.x1+pad);             y1 = max(0, det.y1+pad)
    x2 = min(frame.shape[1]-1, det.x2-pad)
    y2 = min(frame.shape[0]-1, det.y2-pad)
    if x2 <= x1 or y2 <= y1:
        return "unknown", 0.0

    roi = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    ranges = {
        "red":    [((0,100,100),(10,255,255)), ((155,100,100),(180,255,255))],
        "yellow": [((20,100,100),(35,255,255))],
        "green":  [((45,60,60),(85,255,255))],
    }
    scores: dict[str, int] = {}
    for color, rngs in ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in rngs:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        scores[color] = int(np.sum(mask > 0))

    best  = max(scores, key=scores.__getitem__)
    total = roi.shape[0] * roi.shape[1]
    conf  = scores[best] / total if total > 0 else 0.0
    return (best, conf) if conf >= 0.05 else ("unknown", 0.0)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)8s] %(name)s — %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 60)
    print("  VisionModule TMR 2026 — Demo de consola")
    print("  Ctrl+C para salir")
    print("=" * 60)

    vm = VisionModule(display_overlay=False)
    vm.start()

    try:
        while True:
            s = vm.get_state()
            parts = [
                f"FPS:{s.fps:4.1f}",
                f"#{s.frame_id:6d}",
                f"FSM:{s.fsm_state.name:<18}",
                f"spd:{s.speed_factor:.2f}",
                f"lane_err:{s.lane_error:+6.1f}px",
                f"conf:{s.lane_confidence:.2f}",
            ]
            if s.stop_detected:
                d = f"{s.stop_distance_mm:.0f}" if s.stop_distance_mm else "?"
                parts.append(f"STOP:{d}mm")
            if s.traffic_light_color != "unknown":
                parts.append(f"LUZ:{s.traffic_light_color.upper()}")
            if s.car_in_lane:
                parts.append("AUTO[CARRIL]")
            if s.person_detected:
                parts.append("PERSONA")
            print("\r" + "  ".join(parts) + "   ", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nDeteniendo...")
    finally:
        vm.stop()
        print("Listo.")
