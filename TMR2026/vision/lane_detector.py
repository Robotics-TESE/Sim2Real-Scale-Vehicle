# -*- coding: utf-8 -*-
"""
lane_detector.py — Detección de carril con OpenCV.

Pista TMR: Carpeta NEGRA con líneas BLANCAS.

Algoritmo:
  1. ROI: franja inferior del frame (donde están las líneas más cercanas).
  2. Escala de grises → blur → umbral binario alto (blanco sobre negro).
  3. Ventana deslizante horizontal para detectar línea izquierda y derecha.
  4. Centro del carril = media de ambas líneas.
  5. Error de carril = centro_carril − centro_imagen  (px).
  6. Curvatura estimada = diferencia de error entre banda cercana y lejana.
  7. Velocidad sugerida basada en la curvatura.

Salida: LaneData con error_px, curvature_rad, is_curve, confidence.
"""

from dataclasses import dataclass
import math
import cv2
import numpy as np

from config import (
    CAMERA_WIDTH, CAMERA_HEIGHT,
    CURVE_THRESHOLD_RAD,
    SPEED_STRAIGHT, SPEED_CURVE,
    LANE_LOST_THRESHOLD_PX,
)


@dataclass
class LaneData:
    error_px: float        # + = coche a la izquierda del carril, − = a la derecha
    curvature_rad: float   # curvatura estimada (abs)
    is_curve: bool         # True si la curvatura supera el umbral
    confidence: float      # [0, 1] — qué tan buena es la detección
    suggested_speed: float # % PWM sugerido según curvatura
    debug_image: np.ndarray | None = None  # frame anotado (solo en modo test)


class LaneDetector:
    """
    Detector de carril por visión para pista negra con líneas blancas.

    Parámetros calibrables:
      roi_top_ratio   — fracción del alto desde arriba donde empieza el ROI
      threshold       — valor mínimo de gris para considerar "blanco"
      n_windows       — número de ventanas horizontales en el ROI
    """

    def __init__(
        self,
        roi_top_ratio: float = 0.55,
        roi_near_ratio: float = 0.80,
        threshold: int = 160,
        n_windows: int = 6,
        debug: bool = False,
    ):
        self.roi_top_ratio  = roi_top_ratio   # ROI empieza aquí (ej. 55% del alto)
        self.roi_near_ratio = roi_near_ratio  # banda cercana (ej. 80%)
        self.threshold      = threshold
        self.n_windows      = n_windows
        self.debug          = debug

        self._W = CAMERA_WIDTH
        self._H = CAMERA_HEIGHT
        self._mid = self._W // 2

    # ----------------------------------------------------------
    # API pública
    # ----------------------------------------------------------
    def process(self, frame: np.ndarray) -> LaneData:
        """
        Procesa un frame BGR y devuelve LaneData.

        Parameters
        ----------
        frame : np.ndarray
            Frame BGR888 de la cámara (640×480).
        """
        gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(blurred, self.threshold, 255, cv2.THRESH_BINARY)

        roi_top  = int(self._H * self.roi_top_ratio)
        roi_near = int(self._H * self.roi_near_ratio)

        # Extraer bandas: lejana (arriba del ROI) y cercana (abajo del ROI)
        far_band  = binary[roi_top  : roi_near, :]
        near_band = binary[roi_near :          , :]

        cx_far,  conf_far  = self._find_lane_center(far_band)
        cx_near, conf_near = self._find_lane_center(near_band)

        confidence = (conf_far + conf_near) / 2.0

        # Error: positivo = coche a la izquierda del carril (debe girar derecha)
        error_px = cx_near - self._mid

        # Curvatura como el cambio de eje x entre banda cercana y lejana
        far_height  = roi_near - roi_top
        near_height = self._H - roi_near
        dy = far_height + near_height / 2  # distancia vertical aproximada en px
        curvature_rad = math.atan2(abs(cx_near - cx_far), dy) if dy > 0 else 0.0

        is_curve = curvature_rad > CURVE_THRESHOLD_RAD

        # Velocidad sugerida: lineal entre SPEED_CURVE y SPEED_STRAIGHT
        t = min(curvature_rad / CURVE_THRESHOLD_RAD, 1.0) if CURVE_THRESHOLD_RAD > 0 else 0
        suggested_speed = SPEED_STRAIGHT * (1 - t) + SPEED_CURVE * t

        # Si el carril está muy perdido, reducir confianza y velocidad
        if abs(error_px) > LANE_LOST_THRESHOLD_PX:
            confidence = 0.0
            suggested_speed = SPEED_CURVE

        debug_img = None
        if self.debug:
            debug_img = self._draw_debug(
                frame, binary, roi_top, roi_near,
                cx_near, cx_far, error_px
            )

        return LaneData(
            error_px       = error_px,
            curvature_rad  = curvature_rad,
            is_curve       = is_curve,
            confidence     = confidence,
            suggested_speed = suggested_speed,
            debug_image    = debug_img,
        )

    # ----------------------------------------------------------
    # Detección de centro de carril en una banda
    # ----------------------------------------------------------
    def _find_lane_center(self, band: np.ndarray) -> tuple[float, float]:
        """
        Localiza la línea izquierda y derecha en `band` usando ventana
        deslizante por columnas y devuelve (centro_px, confianza).

        Si solo se detecta una línea, estima la otra en base al ancho
        típico del carril (la mitad de la imagen = todo el carril visible).
        """
        if band.size == 0:
            return float(self._mid), 0.0

        # Histograma de columnas en la banda
        col_sum = np.sum(band, axis=0).astype(np.float32)

        mid = self._W // 2
        left_half  = col_sum[:mid]
        right_half = col_sum[mid:]

        left_peak  = int(np.argmax(left_half))              if left_half.max()  > 0 else None
        right_peak = int(np.argmax(right_half)) + mid       if right_half.max() > 0 else None

        if left_peak is not None and right_peak is not None:
            center     = (left_peak + right_peak) / 2.0
            confidence = min(
                left_half[left_peak]  / (band.shape[0] * 255),
                right_half[right_peak - mid] / (band.shape[0] * 255)
            )
            confidence = min(confidence * 5.0, 1.0)  # normalizar
        elif left_peak is not None:
            # Solo línea izquierda detectada — estimar derecha
            center = left_peak + self._W * 0.35  # ancho típico del carril
            confidence = 0.4
        elif right_peak is not None:
            center = right_peak - self._W * 0.35
            confidence = 0.4
        else:
            center = float(self._mid)
            confidence = 0.0

        return center, confidence

    # ----------------------------------------------------------
    # Visualización de debug
    # ----------------------------------------------------------
    def _draw_debug(
        self, frame, binary, roi_top, roi_near,
        cx_near, cx_far, error_px
    ) -> np.ndarray:
        vis = frame.copy()

        # ROI boundaries
        cv2.line(vis, (0, roi_top),  (self._W, roi_top),  (0, 200, 200), 1)
        cv2.line(vis, (0, roi_near), (self._W, roi_near), (0, 200, 200), 1)

        # Línea de centro
        cv2.line(vis, (self._mid, roi_top), (self._mid, self._H), (200, 200, 0), 1)

        # Centro detectado
        cy_near = int((self._H + roi_near) / 2)
        cy_far  = int((roi_top + roi_near) / 2)
        cv2.circle(vis, (int(cx_near), cy_near), 6, (0, 255, 0), -1)
        cv2.circle(vis, (int(cx_far),  cy_far),  6, (255, 165, 0), -1)

        cv2.putText(vis, f"Error: {error_px:.1f}px", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        return vis
