"""
Módulo de Detección de Fichas (Gato y Ratón) mediante Visión por Computadora.
Soporta segmentación en espacio de color HSV, calibración interactiva,
filtrado morfológico y estabilización temporal (debounce) para evitar falsos positivos con manos.
"""

import json
import os
from collections import deque
from typing import Dict, List, Optional, Tuple
import cv2
import numpy as np
from game_ai import EMPTY, GATO, RATON


class PieceDetector:
    def __init__(self, config_file: str = "piece_config.json", history_len: int = 8, min_stable_frames: int = 6):
        self.config_file = config_file
        self.history_len = history_len
        self.min_stable_frames = min_stable_frames

        # Rangos HSV por defecto:
        # RATÓN: Azul (por defecto)
        # GATO: Rojo (por defecto, abarca ambos extremos del canal H en HSV)
        self.color_ranges: Dict[str, dict] = {
            RATON: {
                "name": "Ratón (Azul)",
                "lower": [95, 70, 50],
                "upper": [135, 255, 255],
                "lower2": None,
                "upper2": None
            },
            GATO: {
                "name": "Gato (Rojo)",
                "lower": [0, 80, 50],
                "upper": [10, 255, 255],
                "lower2": [168, 80, 50],
                "upper2": [180, 255, 255]
            }
        }

        # Historial de detecciones para estabilización temporal por casilla (3x3)
        self.history = [[deque(maxlen=self.history_len) for _ in range(3)] for _ in range(3)]
        self.stable_board = [[EMPTY for _ in range(3)] for _ in range(3)]

        self.load_config()

    def load_config(self) -> bool:
        """Carga rangos de color HSV personalizados desde archivo JSON si existe."""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, "r") as f:
                    data = json.load(f)
                    for k in [RATON, GATO]:
                        if k in data:
                            self.color_ranges[k].update(data[k])
                    return True
            except Exception as e:
                print(f"[PieceDetector] Error al cargar configuración: {e}")
        return False

    def save_config(self) -> bool:
        """Guarda la configuración de rangos de color en archivo JSON."""
        try:
            with open(self.config_file, "w") as f:
                json.dump(self.color_ranges, f, indent=4)
            return True
        except Exception as e:
            print(f"[PieceDetector] Error al guardar configuración: {e}")
            return False

    def calibrate_color_at_point(self, hsv_image: np.ndarray, x: int, y: int, piece_type: str, radius: int = 8):
        """
        Calibra el rango de color para 'RATON' o 'GATO' tomando una muestra alrededor de (x, y).
        """
        h, w = hsv_image.shape[:2]
        x1, x2 = max(0, x - radius), min(w, x + radius)
        y1, y2 = max(0, y - radius), min(h, y + radius)
        patch = hsv_image[y1:y2, x1:x2]

        if patch.size == 0:
            return

        mean_h = np.median(patch[:, :, 0])
        mean_s = np.median(patch[:, :, 1])
        mean_v = np.median(patch[:, :, 2])

        # Margen adaptativo
        h_margin = 16
        s_min = max(40, int(mean_s * 0.55))
        v_min = max(40, int(mean_v * 0.55))

        lower = [max(0, int(mean_h - h_margin)), s_min, v_min]
        upper = [min(180, int(mean_h + h_margin)), 255, 255]

        self.color_ranges[piece_type]["lower"] = lower
        self.color_ranges[piece_type]["upper"] = upper
        self.color_ranges[piece_type]["lower2"] = None
        self.color_ranges[piece_type]["upper2"] = None

        print(f"[PieceDetector] Calibrado {self.color_ranges[piece_type]['name']} HSV centro ({mean_h:.1f}, {mean_s:.1f}, {mean_v:.1f})")
        self.save_config()

    def get_color_mask(self, hsv_roi: np.ndarray, piece_type: str) -> np.ndarray:
        """Genera máscara binaria para el tipo de ficha especificado."""
        cfg = self.color_ranges[piece_type]
        lower1 = np.array(cfg["lower"], dtype=np.uint8)
        upper1 = np.array(cfg["upper"], dtype=np.uint8)
        mask = cv2.inRange(hsv_roi, lower1, upper1)

        # Si tiene segundo rango (común en color rojo en extremos de 0 y 180)
        if cfg.get("lower2") is not None and cfg.get("upper2") is not None:
            lower2 = np.array(cfg["lower2"], dtype=np.uint8)
            upper2 = np.array(cfg["upper2"], dtype=np.uint8)
            mask2 = cv2.inRange(hsv_roi, lower2, upper2)
            mask = cv2.bitwise_or(mask, mask2)

        # Limpieza morfológica para reducir ruido
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
        return mask

    def detect_cell_piece(self, cell_bgr: np.ndarray) -> Tuple[str, float]:
        """
        Determina si una casilla contiene una ficha de RATÓN, GATO o está VACÍA.
        Retorna (tipo_ficha, confianza/porcentaje).
        """
        if cell_bgr.size == 0:
            return EMPTY, 0.0

        hsv = cv2.cvtColor(cell_bgr, cv2.COLOR_BGR2HSV)
        total_pixels = cell_bgr.shape[0] * cell_bgr.shape[1]

        mask_raton = self.get_color_mask(hsv, RATON)
        mask_gato = self.get_color_mask(hsv, GATO)

        count_raton = cv2.countNonZero(mask_raton)
        count_gato = cv2.countNonZero(mask_gato)

        pct_raton = (count_raton / total_pixels) * 100.0
        pct_gato = (count_gato / total_pixels) * 100.0

        # Umbral mínimo de ocupación para considerar que hay una ficha física (ej. > 8% del área de la celda)
        min_threshold_pct = 7.5

        if pct_raton >= min_threshold_pct and pct_raton > pct_gato * 1.3:
            return RATON, pct_raton
        elif pct_gato >= min_threshold_pct and pct_gato > pct_raton * 1.3:
            return GATO, pct_gato
        else:
            return EMPTY, 0.0

    def update_and_detect_board(self, warped_board: np.ndarray) -> Tuple[List[List[str]], List[List[str]]]:
        """
        Procesa el tablero rectificado 3x3 y actualiza la detección temporal.
        
        Retorna:
            (raw_board, stable_board):
            - raw_board: detección instantánea del frame actual.
            - stable_board: detección suavizada temporalmente resistente al movimiento de manos.
        """
        raw_board = [[EMPTY for _ in range(3)] for _ in range(3)]
        cell_size = warped_board.shape[0] // 3
        margin = int(cell_size * 0.16)

        for r in range(3):
            for c in range(3):
                y1 = r * cell_size + margin
                y2 = (r + 1) * cell_size - margin
                x1 = c * cell_size + margin
                x2 = (c + 1) * cell_size - margin

                cell_roi = warped_board[y1:y2, x1:x2]
                piece, _ = self.detect_cell_piece(cell_roi)
                raw_board[r][c] = piece

                # Agregar al buffer temporal
                self.history[r][c].append(piece)

                # Verificar si un estado domina el buffer de estabilidad
                counts = {}
                for val in self.history[r][c]:
                    counts[val] = counts.get(val, 0) + 1

                for val, count in counts.items():
                    if count >= self.min_stable_frames:
                        self.stable_board[r][c] = val
                        break

        return raw_board, self.stable_board

    def reset_history(self):
        """Reinicia el buffer temporal."""
        for r in range(3):
            for c in range(3):
                self.history[r][c].clear()
                self.stable_board[r][c] = EMPTY
