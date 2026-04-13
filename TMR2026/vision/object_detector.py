# -*- coding: utf-8 -*-
"""
object_detector.py — Post-procesado de las detecciones del IMX500.

Las detecciones ya vienen resueltas por el NPU en camera_manager.py.
Este módulo provee lógica de alto nivel:
  - Filtrar detecciones por clase y umbral.
  - Calcular distancia estimada a señales STOP.
  - Detectar el color de un semáforo usando la región del bbox.
  - Proveer información estructurada al controlador autónomo.
"""

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from hardware.camera_manager import Detection
from config import (
    CAMERA_WIDTH, CAMERA_HEIGHT,
    STOP_SIGN_REAL_HEIGHT_M, CAMERA_FOCAL_LENGTH_PX,
    DETECTION_CONFIDENCE,
    OVERTAKE_LANE_RATIO, OVERTAKE_MIN_BBOX_AREA, OVERTAKE_TRIGGER_Y_MIN,
    PARK_GAP_CAMERA_ZONE,
)


@dataclass
class TrafficLightState:
    color: str           # "red", "yellow", "green", "unknown"
    confidence: float    # [0, 1]


class ObjectDetector:
    """
    Interpreta las detecciones del IMX500 y devuelve eventos semánticos
    para el controlador autónomo.

    Uso:
        od = ObjectDetector()
        result = od.analyze(detections, frame)
        if result.stop_distance_mm:
            ...
    """

    @dataclass
    class AnalysisResult:
        stop_sign_detected: bool = False
        stop_sign_distance_mm: Optional[float] = None
        stop_sign_bbox: Optional[tuple] = None    # (x1,y1,x2,y2)

        traffic_light: Optional["TrafficLightState"] = None

        person_detected: bool = False
        car_detected: bool = False
        car_bbox: Optional[tuple] = None          # bbox del auto más grande/cercano
        car_in_lane: bool = False                 # True si el auto bloquea nuestro carril
        car_in_park_zone: bool = False            # True si hay auto en zona lateral (parking)
        closest_object_mm: Optional[float] = None

    # ----------------------------------------------------------
    def analyze(
        self,
        detections: list[Detection],
        frame: np.ndarray,
        tof_distance_mm: Optional[float] = None,
    ) -> "ObjectDetector.AnalysisResult":
        """
        Analiza la lista de detecciones y el frame opcional para extraer
        información semántica.

        Parameters
        ----------
        detections : list[Detection]
            Detecciones del NPU del IMX500.
        frame : np.ndarray
            Frame BGR original (para clasificación de color de semáforo).
        tof_distance_mm : float | None
            Lectura actual del VL53L0X (para confirmar distancia de STOP).
        """
        result = self.AnalysisResult()

        for det in detections:
            if det.confidence < DETECTION_CONFIDENCE:
                continue

            if det.label == "STOP":
                result.stop_sign_detected = True
                result.stop_sign_bbox = (det.x1, det.y1, det.x2, det.y2)

                # Distancia por bbox (si disponible)
                dist_bbox = det.estimated_distance_m()
                if dist_bbox is not None:
                    dist_mm_bbox = dist_bbox * 1000

                    # Si el ToF está disponible y en rango útil, preferirlo
                    if tof_distance_mm is not None and tof_distance_mm < 1200:
                        result.stop_sign_distance_mm = tof_distance_mm
                    else:
                        result.stop_sign_distance_mm = dist_mm_bbox

            elif det.label == "SEMAFORO":
                result.traffic_light = self._classify_traffic_light(frame, det)

            elif det.label == "PERSONA":
                result.person_detected = True

            elif det.label == "AUTO":
                result.car_detected = True
                bbox = (det.x1, det.y1, det.x2, det.y2)
                area = det.width * det.height
                # Guardar el auto más grande (más cercano)
                if result.car_bbox is None:
                    result.car_bbox = bbox
                else:
                    prev = result.car_bbox
                    prev_area = (prev[2]-prev[0]) * (prev[3]-prev[1])
                    if area > prev_area:
                        result.car_bbox = bbox

        # Distancia al objeto más cercano (para emergencia)
        if tof_distance_mm is not None:
            result.closest_object_mm = tof_distance_mm

        # ── Clasificar posición del auto ──────────────────────────
        if result.car_bbox is not None:
            x1, y1, x2, y2 = result.car_bbox
            cx   = (x1 + x2) / 2
            area = (x2 - x1) * (y2 - y1)
            frame_cx = CAMERA_WIDTH / 2

            # Obstáculo en carril: cerca del centro y suficientemente grande
            result.car_in_lane = (
                abs(cx - frame_cx) < CAMERA_WIDTH * OVERTAKE_LANE_RATIO
                and area >= OVERTAKE_MIN_BBOX_AREA
                and y2 >= OVERTAKE_TRIGGER_Y_MIN
            )

            # Auto en zona lateral derecha (detección de espacio para parking)
            result.car_in_park_zone = cx > CAMERA_WIDTH * PARK_GAP_CAMERA_ZONE

        return result

    # ----------------------------------------------------------
    # Clasificación de semáforos por HSV en el bbox
    # ----------------------------------------------------------
    def _classify_traffic_light(
        self, frame: np.ndarray, det: Detection
    ) -> TrafficLightState:
        """
        Clasifica el color del semáforo usando la región del bbox.

        Estrategia:
          1. Recortar el bbox con margen pequeño.
          2. Convertir a HSV.
          3. Calcular máscara para rojo, amarillo y verde.
          4. El color con mayor número de píxeles detectados gana.
        """
        pad = 4
        x1 = max(0, det.x1 + pad)
        y1 = max(0, det.y1 + pad)
        x2 = min(frame.shape[1] - 1, det.x2 - pad)
        y2 = min(frame.shape[0] - 1, det.y2 - pad)

        if x2 <= x1 or y2 <= y1:
            return TrafficLightState("unknown", 0.0)

        roi = frame[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Rangos HSV (calibrados para luz de interior/pista cerrada)
        ranges = {
            "red":    [((0, 100, 100), (10, 255, 255)),
                       ((160, 100, 100), (180, 255, 255))],
            "yellow": [((20, 100, 100), (35, 255, 255))],
            "green":  [((45, 60, 60), (85, 255, 255))],
        }

        scores: dict[str, int] = {}
        for color, range_list in ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (lo, hi) in range_list:
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            scores[color] = int(np.sum(mask > 0))

        best_color = max(scores, key=scores.get)
        total_px   = roi.shape[0] * roi.shape[1]
        confidence = scores[best_color] / total_px if total_px > 0 else 0.0

        # Umbral mínimo para no reportar colores fantasma
        if confidence < 0.05:
            best_color = "unknown"
            confidence = 0.0

        return TrafficLightState(color=best_color, confidence=confidence)
