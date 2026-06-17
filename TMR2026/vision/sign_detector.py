"""Traffic-sign detection with YOLOv8n (separate thread).

Runs in a daemon thread at ~8-12 FPS on the Pi 5 CPU.
The control thread never waits for the detector -- it consumes the latest
available result.

Expected classes in the model (indices adjustable in SIGN_CLASSES):
  0 -> stop_sign
  1 -> crosswalk

Default model: weights/tmr_signs.pt (trained for TMR signs).
Fallback:      weights/yolov8n.pt   (COCO model -- uses stop sign and person).
"""

import os
import threading
import time
from typing import Optional

import cv2
import numpy as np

try:
    from config import STOP_SIGN_REAL_HEIGHT_M, CAMERA_FOCAL_LENGTH_PX
except ImportError:
    STOP_SIGN_REAL_HEIGHT_M = 0.04
    CAMERA_FOCAL_LENGTH_PX  = 490.0

SIGN_REAL_HEIGHT_M = {
    "stop":     STOP_SIGN_REAL_HEIGHT_M,
    "red":      0.06,
    "green":    0.06,
    "yellow":   0.06,
    "left":     0.05,
    "right":    0.05,
    "straight": 0.05,
}


_RED_HSV_LO_1 = np.array([  0, 100,  60])
_RED_HSV_HI_1 = np.array([ 12, 255, 255])
_RED_HSV_LO_2 = np.array([165, 100,  60])
_RED_HSV_HI_2 = np.array([179, 255, 255])
_PURPLE_HSV_LO = np.array([120,  60,  40])
_PURPLE_HSV_HI = np.array([160, 255, 255])

_COLOR_MIN_AREA = 1500
_COLOR_FILL_RATIO_MIN = 0.55
_COLOR_ASPECT_MIN = 0.65
_COLOR_ASPECT_MAX = 1.50


def _detect_red_blob(frame_bgr: np.ndarray):
    """
    Return (x1, y1, x2, y2, area) of the largest red/purple contour,
    or None if nothing plausible is found.

    Applied filters:
      - area >= _COLOR_MIN_AREA   -> rejects distant objects
      - aspect in [0.65, 1.50]    -> rejects elongated boxes
      - fill_ratio >= 0.55        -> rejects hollow / irregular blobs
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, _RED_HSV_LO_1,    _RED_HSV_HI_1)
    m2 = cv2.inRange(hsv, _RED_HSV_LO_2,    _RED_HSV_HI_2)
    m3 = cv2.inRange(hsv, _PURPLE_HSV_LO,   _PURPLE_HSV_HI)
    mask = cv2.bitwise_or(cv2.bitwise_or(m1, m2), m3)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_area = 0
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        bbox_area = w * h
        if bbox_area < _COLOR_MIN_AREA:
            continue
        aspect = w / max(1.0, h)
        if not (_COLOR_ASPECT_MIN <= aspect <= _COLOR_ASPECT_MAX):
            continue
        contour_area = cv2.contourArea(c)
        fill_ratio = contour_area / max(1.0, bbox_area)
        if fill_ratio < _COLOR_FILL_RATIO_MIN:
            continue
        if bbox_area > best_area:
            best_area = bbox_area
            best = (x, y, x + w, y + h, bbox_area)
    return best


class Detection:
    """A confirmed sign detection."""
    __slots__ = ("label", "confidence", "x1", "y1", "x2", "y2", "distance_m")

    def __init__(self, label: str, confidence: float,
                 x1: int, y1: int, x2: int, y2: int,
                 distance_m: Optional[float] = None):
        self.label      = label
        self.confidence = confidence
        self.x1 = x1; self.y1 = y1
        self.x2 = x2; self.y2 = y2
        self.distance_m = distance_m

    @property
    def area(self) -> int:
        return (self.x2 - self.x1) * (self.y2 - self.y1)

    @property
    def cx(self) -> int:
        return (self.x1 + self.x2) // 2

    @property
    def height_px(self) -> int:
        return max(1, self.y2 - self.y1)


class SignDetector:
    """
    STOP and crosswalk sign detector with YOLOv8n.

    Usage::

        sd = SignDetector("weights/tmr_signs.pt", conf=0.55, imgsz=320)
        sd.start()
        sd.update_frame(frame)          # call on every camera frame
        dets = sd.get_detections()      # non-blocking, returns latest list
        sd.stop()
    """

    SIGN_CLASSES = {"stop_sign", "stop sign", "crosswalk", "cross walk"}

    MAX_HZ = 15.0

    HYSTERESIS_FRAMES = 3

    def __init__(
        self,
        model_path: str  = "weights/tmr_signs.pt",
        conf:       float = 0.55,
        imgsz:      int   = 320,
        hysteresis_frames: int = HYSTERESIS_FRAMES,
    ):
        self._conf   = conf
        self._imgsz  = imgsz
        self._hysteresis = max(1, hysteresis_frames)
        self._model  = None

        self._frame:      Optional[np.ndarray] = None
        self._frame_lock  = threading.Lock()
        self._results:    list[Detection] = []
        self._result_lock = threading.Lock()

        self._consecutive: dict[str, int] = {}
        self._last_raw:    dict[str, Detection] = {}

        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._model_path = model_path
        self._load_model()


    def start(self) -> None:
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._detect_loop,
            name="SignDetector",
            daemon=True,
        )
        self._thread.start()
        print(f"[YOLO] Detection thread started (imgsz={self._imgsz}, conf={self._conf})")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)


    def update_frame(self, frame: np.ndarray) -> None:
        """Provide a new frame to the detector. Non-blocking."""
        with self._frame_lock:
            self._frame = frame

    def get_detections(self) -> list[Detection]:
        """Return the most recent detection list. Non-blocking."""
        with self._result_lock:
            return list(self._results)

    def has_sign(self, label: str) -> bool:
        """True if the label is in the current detections."""
        return any(d.label == label for d in self.get_detections())

    def has_any_sign(self) -> bool:
        """True if any relevant sign is detected."""
        return len(self.get_detections()) > 0

    def closest_sign(self, label: Optional[str] = None) -> Optional[Detection]:
        """
        Return the detection with the smallest `distance_m` (the closest one).
        Filters by label if given. None if there is nothing.
        """
        dets = self.get_detections()
        if label is not None:
            dets = [d for d in dets if d.label == label]
        dets = [d for d in dets if d.distance_m is not None]
        if not dets:
            return None
        return min(dets, key=lambda d: d.distance_m)


    def _resolve_model_path(self) -> str:
        """
        Prefer the exported NCNN version (`<model>_ncnn_model/`) if it exists
        next to the `.pt`. On the Pi 5 (ARM CPU) NCNN is 3-4x faster than
        PyTorch at the same accuracy. It is generated with
        `tools/export_model.py` and committed to the repo -- if missing, the
        plain `.pt` is used.
        """
        path = self._model_path
        if path.endswith(".pt"):
            ncnn_dir = path[:-3] + "_ncnn_model"
            if os.path.isdir(ncnn_dir):
                return ncnn_dir
        return path

    def _load_model(self) -> None:
        path = self._resolve_model_path()
        candidates = [path]
        if path != self._model_path:
            candidates.append(self._model_path)

        for cand in candidates:
            try:
                from ultralytics import YOLO
                model = YOLO(cand, task="detect")
                dummy = np.zeros((self._imgsz, self._imgsz, 3), dtype=np.uint8)
                model(dummy, imgsz=self._imgsz, conf=self._conf, verbose=False)
                backend = "NCNN" if cand.endswith("_ncnn_model") else "PyTorch"
                print(f"[YOLO] Model loaded ({backend}): {cand}")
                self._model = model
                return
            except Exception as e:
                print(f"[YOLO] Could not load {cand} ({e}).")

        print("[YOLO] Falling back to COLOR (red) STOP detector.")
        self._model = None


    def _detect_loop(self) -> None:
        min_interval = 1.0 / self.MAX_HZ

        while not self._stop_event.is_set():
            t0 = time.monotonic()

            with self._frame_lock:
                frame = self._frame

            if frame is None:
                time.sleep(0.05)
                continue

            raw_dets = []
            if self._model is not None:
                try:
                    results = self._model(
                        frame,
                        imgsz=self._imgsz,
                        conf=self._conf,
                        verbose=False,
                    )
                    raw_dets = self._parse_results(results, frame.shape)
                except Exception as e:
                    print(f"[YOLO] Inference error: {e}")
                    raw_dets = []

            has_yolo_stop = any(d.label == "stop_sign" for d in raw_dets)
            if not has_yolo_stop:
                blob = _detect_red_blob(frame)
                if blob is not None:
                    x1, y1, x2, y2, area = blob
                    height_px = max(1, y2 - y1)
                    distance_m = (STOP_SIGN_REAL_HEIGHT_M
                                  * CAMERA_FOCAL_LENGTH_PX) / height_px
                    raw_dets.append(Detection(
                        "stop_sign", 0.55, x1, y1, x2, y2,
                        distance_m=distance_m,
                    ))

            confirmed = self._apply_hysteresis(raw_dets)

            with self._result_lock:
                self._results = confirmed

            elapsed = time.monotonic() - t0
            sleep   = max(0.0, min_interval - elapsed)
            time.sleep(sleep)

    def _apply_hysteresis(self, raw_dets: list[Detection]) -> list[Detection]:
        """
        Temporal filter: a detection is only confirmed (and published) when
        it appears with the same label in `self._hysteresis` consecutive
        frames.

        - Keeps the per-label counter in `self._consecutive`.
        - Keeps the last raw detection per label in `self._last_raw` (using
          the largest-area one if several appear in the frame -- usually the
          closest, which is the one the FSM cares about).
        """
        seen_this_frame: dict[str, Detection] = {}
        for d in raw_dets:
            prev = seen_this_frame.get(d.label)
            if prev is None or d.area > prev.area:
                seen_this_frame[d.label] = d

        for label in list(self._consecutive.keys()):
            if label not in seen_this_frame:
                self._consecutive[label] = 0

        for label, det in seen_this_frame.items():
            self._consecutive[label] = self._consecutive.get(label, 0) + 1
            self._last_raw[label]    = det

        confirmed: list[Detection] = []
        for label, count in self._consecutive.items():
            if count >= self._hysteresis and label in self._last_raw:
                confirmed.append(self._last_raw[label])
        return confirmed

    def _parse_results(self, results, img_shape) -> list[Detection]:
        ih, iw = img_shape[:2]
        dets: list[Detection] = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf   = float(box.conf[0])
                label  = (self._model.names.get(cls_id, str(cls_id))
                          .lower().replace(" ", "_"))

                if label not in SIGN_REAL_HEIGHT_M:
                    continue

                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])

                if x2 <= 1 and y2 <= 1:
                    x1 = int(x1 * iw); y1 = int(y1 * ih)
                    x2 = int(x2 * iw); y2 = int(y2 * ih)

                area = (x2 - x1) * (y2 - y1)
                if area < 150:
                    continue

                normalized = "stop_sign" if label == "stop" else label

                height_px  = max(1, y2 - y1)
                real_h     = SIGN_REAL_HEIGHT_M[label]
                distance_m = (real_h * CAMERA_FOCAL_LENGTH_PX) / height_px

                dets.append(Detection(
                    normalized, conf, x1, y1, x2, y2,
                    distance_m=distance_m,
                ))

        return dets
