"""
Módulo de Detección y Calibración del Tablero de Gato y Ratón.
Maneja la detección automática por contornos, calibración manual de 4 esquinas,
transformación de perspectiva (homografía) y mapeo de celdas 3x3.
"""

import json
import os
import cv2
import numpy as np
from typing import List, Tuple, Optional


def order_points(pts: np.ndarray) -> np.ndarray:
    """
    Ordena 4 puntos en el orden: [Top-Left, Top-Right, Bottom-Right, Bottom-Left].
    """
    rect = np.zeros((4, 2), dtype="float32")
    pts = pts.reshape(4, 2)

    # Top-Left tiene la suma más pequeña (x + y), Bottom-Right tiene la suma más grande
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    # Top-Right tiene la menor diferencia (y - x), Bottom-Left tiene la mayor diferencia
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    return rect


class BoardDetector:
    def __init__(self, warped_size: int = 450, config_file: str = "board_config.json"):
        self.warped_size = warped_size
        self.config_file = config_file
        self.corners: Optional[np.ndarray] = None  # 4 esquinas en imagen original
        self.dst_corners = np.array([
            [0, 0],
            [self.warped_size - 1, 0],
            [self.warped_size - 1, self.warped_size - 1],
            [0, self.warped_size - 1]
        ], dtype="float32")
        
        self.M: Optional[np.ndarray] = None      # Matriz perspectiva hacia warped
        self.M_inv: Optional[np.ndarray] = None  # Matriz perspectiva inversa hacia original

        self.load_config()

    def load_config(self) -> bool:
        """Carga las esquinas calibradas si existe archivo de configuración."""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, "r") as f:
                    data = json.load(f)
                    if "corners" in data and len(data["corners"]) == 4:
                        self.set_corners(np.array(data["corners"], dtype="float32"))
                        return True
            except Exception as e:
                print(f"[BoardDetector] Error al cargar configuración: {e}")
        return False

    def save_config(self) -> bool:
        """Guarda las 4 esquinas calibradas en archivo JSON."""
        if self.corners is not None:
            try:
                with open(self.config_file, "w") as f:
                    json.dump({"corners": self.corners.tolist()}, f, indent=4)
                return True
            except Exception as e:
                print(f"[BoardDetector] Error al guardar configuración: {e}")
        return False

    def set_corners(self, corners: np.ndarray):
        """Asigna y ordena las 4 esquinas del tablero calculando las matrices de homografía."""
        self.corners = order_points(corners)
        self.M = cv2.getPerspectiveTransform(self.corners, self.dst_corners)
        self.M_inv = cv2.getPerspectiveTransform(self.dst_corners, self.corners)

    def auto_detect_board(self, frame: np.ndarray) -> bool:
        """
        Intenta detectar automáticamente el contorno cuadrangular del tablero en el frame.
        Retorna True si encontró un cuadrilátero válido con aspecto cuadrado.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Detección de bordes adaptativa
        thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
        )
        
        # Operaciones morfológicas para cerrar líneas
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False

        # Ordenar contornos por área descendente
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        frame_area = frame.shape[0] * frame.shape[1]

        for cnt in contours[:5]:
            area = cv2.contourArea(cnt)
            # El tablero debe ocupar entre el 10% y el 85% del área total
            if area < frame_area * 0.08 or area > frame_area * 0.85:
                continue

            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)

            if len(approx) == 4 and cv2.isContourConvex(approx):
                self.set_corners(approx.reshape(4, 2))
                return True

        return False

    def set_default_centered_roi(self, width: int, height: int, box_size: int = 360):
        """Genera un ROI centrado por defecto si no hay calibración previa."""
        cx, cy = width // 2, height // 2
        half = box_size // 2
        default_pts = np.array([
            [cx - half, cy - half],
            [cx + half, cy - half],
            [cx + half, cy + half],
            [cx - half, cy + half]
        ], dtype="float32")
        self.set_corners(default_pts)

    def get_warped_board(self, frame: np.ndarray) -> Optional[np.ndarray]:
        """Aplica la transformación de perspectiva para obtener la vista superior plana 450x450."""
        if self.M is None:
            return None
        return cv2.warpPerspective(frame, self.M, (self.warped_size, self.warped_size))

    def get_cell_roi(self, warped: np.ndarray, row: int, col: int, margin_pct: float = 0.15) -> np.ndarray:
        """
        Extrae la región de interés (ROI) de una casilla (fila, columna) con un margen interno
        para ignorar líneas o bordes divisorios.
        """
        cell_size = self.warped_size // 3
        y1 = int(row * cell_size + cell_size * margin_pct)
        y2 = int((row + 1) * cell_size - cell_size * margin_pct)
        x1 = int(col * cell_size + cell_size * margin_pct)
        x2 = int((col + 1) * cell_size - cell_size * margin_pct)
        return warped[y1:y2, x1:x2]

    def map_warped_point_to_frame(self, x: float, y: float) -> Tuple[int, int]:
        """Transforma coordenadas de la vista corregida (warped) al espacio de la cámara original."""
        if self.M_inv is None:
            return int(x), int(y)
        pt = np.array([[[x, y]]], dtype="float32")
        transformed = cv2.perspectiveTransform(pt, self.M_inv)
        return int(transformed[0][0][0]), int(transformed[0][0][1])

    def get_cell_quad_in_frame(self, row: int, col: int) -> np.ndarray:
        """Obtiene las 4 esquinas de una casilla proyectadas sobre el frame original."""
        cell_size = self.warped_size // 3
        pts = np.array([
            [col * cell_size, row * cell_size],
            [(col + 1) * cell_size, row * cell_size],
            [(col + 1) * cell_size, (row + 1) * cell_size],
            [col * cell_size, (row + 1) * cell_size]
        ], dtype="float32").reshape(-1, 1, 2)

        if self.M_inv is not None:
            transformed = cv2.perspectiveTransform(pts, self.M_inv)
            return transformed.astype(int)
        return pts.astype(int)

    def draw_board_overlay(self, frame: np.ndarray, color=(0, 255, 0), thickness=2):
        """Dibuja el perímetro del tablero y la cuadrícula 3x3 en el frame de la cámara."""
        if self.corners is None:
            return

        pts = self.corners.astype(int)
        cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=thickness)

        # Dibujar líneas internas de la cuadrícula proyectadas
        for i in range(1, 3):
            # Línea vertical i
            p_top = self.map_warped_point_to_frame(i * (self.warped_size // 3), 0)
            p_bot = self.map_warped_point_to_frame(i * (self.warped_size // 3), self.warped_size)
            cv2.line(frame, p_top, p_bot, color, thickness=1, lineType=cv2.LINE_AA)

            # Línea horizontal i
            p_left = self.map_warped_point_to_frame(0, i * (self.warped_size // 3))
            p_right = self.map_warped_point_to_frame(self.warped_size, i * (self.warped_size // 3))
            cv2.line(frame, p_left, p_right, color, thickness=1, lineType=cv2.LINE_AA)
