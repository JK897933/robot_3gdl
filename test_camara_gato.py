"""
Test independiente de Cámara y Lógica del Juego de Gato (sin conexión al robot).
Permite probar la detección HSV de piezas ('X' roja / 'O' azul) en el tablero 3x3,
el filtro de estabilidad temporal y la respuesta de la IA (Minimax) sin usar el ESP32.
"""

import os
import sys
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import cv2
import numpy as np
from PIL import Image, ImageTk

from board_detector import BoardDetector
from piece_detector import PieceDetector
from game_ai import EMPTY, GATO, RATON


def check_winner(board, player):
    lines = [
        (0, 1, 2), (3, 4, 5), (6, 7, 8),
        (0, 3, 6), (1, 4, 7), (2, 5, 8),
        (0, 4, 8), (2, 4, 6),
    ]
    return any(board[a] == player and board[b] == player and board[c] == player for a, b, c in lines)


def minimax_best_move(board):
    """Misma lógica de decisión del robot en gui_robot_gato.py."""
    for i in range(9):
        if board[i] == 0:
            board[i] = 2
            if check_winner(board, 2):
                board[i] = 0
                return i
            board[i] = 0

    for i in range(9):
        if board[i] == 0:
            board[i] = 1
            if check_winner(board, 1):
                board[i] = 0
                return i
            board[i] = 0

    if board[4] == 0:
        return 4
    for i in (0, 2, 6, 8):
        if board[i] == 0:
            return i
    for i in range(9):
        if board[i] == 0:
            return i
    return None


class CameraGatoTester(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Test Cámara + Lógica Gato (Sin Robot)")
        self.geometry("1120x680")
        self.configure(bg="#0f172a")

        self.cap = None
        self.camera_running = False
        self.camera_thread = None
        self.current_frame = None
        self.photo_img = None

        self.board = [0] * 9
        self.game_active = True
        self.turn = "HUMANO"
        self.last_detected_board = [0] * 9
        self.detection_stability_count = 0

        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.board_detector = BoardDetector(warped_size=450, config_file=os.path.join(base_dir, "board_config.json"))
        self.piece_detector = PieceDetector(config_file=os.path.join(base_dir, "piece_config.json"))

        self.var_auto_vision = tk.BooleanVar(value=True)
        self.var_flip = tk.BooleanVar(value=True)
        self.var_px_thresh = tk.IntVar(value=400)

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(30, self._update_canvas)

    def _build_ui(self):
        top = tk.Frame(self, bg="#1e293b", padx=12, pady=8)
        top.pack(fill=tk.X)

        tk.Label(
            top, text="TEST DE CÁMARA + LÓGICA DEL GATO (SIN ROBOT)",
            bg="#1e293b", fg="#38bdf8", font=("Segoe UI", 12, "bold")
        ).pack(side=tk.LEFT)

        tk.Label(top, text="Cámara:", bg="#1e293b", fg="#cbd5e1", font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(20, 4))
        self.combo_cam = ttk.Combobox(top, values=["Cam 0", "Cam 1", "Cam 2", "Cam 3"], state="readonly", width=8)
        self.combo_cam.current(0)
        self.combo_cam.pack(side=tk.LEFT, padx=(0, 8))

        self.btn_camera = tk.Button(
            top, text="Iniciar Video", bg="#059669", fg="#ffffff",
            font=("Segoe UI", 9, "bold"), padx=12, pady=4, bd=0, cursor="hand2",
            command=self.toggle_camera
        )
        self.btn_camera.pack(side=tk.LEFT, padx=4)

        tk.Checkbutton(
            top, text="Auto-Detectar Turno", variable=self.var_auto_vision,
            bg="#1e293b", fg="#e2e8f0", selectcolor="#0f172a", activebackground="#1e293b",
            font=("Segoe UI", 9)
        ).pack(side=tk.LEFT, padx=10)

        tk.Checkbutton(
            top, text="Espejo (Flip)", variable=self.var_flip,
            bg="#1e293b", fg="#e2e8f0", selectcolor="#0f172a", activebackground="#1e293b",
            font=("Segoe UI", 9)
        ).pack(side=tk.LEFT, padx=6)

        tk.Label(top, text="Umbral px:", bg="#1e293b", fg="#cbd5e1", font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(10, 4))
        tk.Scale(
            top, from_=100, to=2000, orient=tk.HORIZONTAL, variable=self.var_px_thresh,
            bg="#1e293b", fg="#f8fafc", highlightthickness=0, length=120
        ).pack(side=tk.LEFT)

        body = tk.Frame(self, bg="#0f172a", padx=12, pady=12)
        body.pack(fill=tk.BOTH, expand=True)

        # Panel izquierdo: Video
        left = tk.Frame(body, bg="#1e293b", padx=8, pady=8)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        self.canvas_cam = tk.Canvas(left, bg="#020617", highlightthickness=0)
        self.canvas_cam.pack(fill=tk.BOTH, expand=True)

        self.lbl_vision_info = tk.Label(
            left, text="Visión: Cámara detenida. Muestra una ficha ROJA ('X') en una casilla para probar.",
            bg="#1e293b", fg="#94a3b8", font=("Consolas", 9), anchor="w"
        )
        self.lbl_vision_info.pack(fill=tk.X, pady=(6, 0))

        # Panel derecho: Tablero virtual y log de lógica
        right = tk.Frame(body, bg="#1e293b", padx=12, pady=12, width=340)
        right.pack(side=tk.RIGHT, fill=tk.Y)
        right.pack_propagate(False)

        self.lbl_turn = tk.Label(
            right, text="● TU TURNO (X - Rojo)", bg="#1e293b", fg="#22c55e",
            font=("Segoe UI", 12, "bold")
        )
        self.lbl_turn.pack(pady=(0, 10))

        grid_frame = tk.Frame(right, bg="#0f172a", padx=6, pady=6)
        grid_frame.pack()

        self.cell_buttons = []
        for idx in range(9):
            r, c = divmod(idx, 3)
            btn = tk.Button(
                grid_frame, text=f"{idx+1}", width=6, height=3,
                bg="#1e293b", fg="#64748b", font=("Segoe UI", 14, "bold"),
                bd=1, relief=tk.SOLID, cursor="hand2",
                command=lambda i=idx: self.on_human_move(i, source="CLIC")
            )
            btn.grid(row=r, column=c, padx=4, pady=4)
            self.cell_buttons.append(btn)

        tk.Button(
            right, text="Reiniciar Partida", bg="#2563eb", fg="#ffffff",
            font=("Segoe UI", 10, "bold"), pady=6, bd=0, cursor="hand2",
            command=self.reset_game
        ).pack(fill=tk.X, pady=(12, 10))

        tk.Label(
            right, text="Registro de Lógica del Robot:", bg="#1e293b", fg="#cbd5e1",
            font=("Segoe UI", 9, "bold"), anchor="w"
        ).pack(fill=tk.X)

        self.txt_log = tk.Text(
            right, bg="#020617", fg="#e2e8f0", font=("Consolas", 9),
            height=12, state=tk.DISABLED, wrap=tk.WORD, bd=0, padx=6, pady=6
        )
        self.txt_log.pack(fill=tk.BOTH, expand=True, pady=(4, 0))
        self._log("Listo. Inicia el video o haz clic en una casilla para probar la lógica.")

    def _log(self, msg):
        self.txt_log.config(state=tk.NORMAL)
        self.txt_log.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.txt_log.see(tk.END)
        self.txt_log.config(state=tk.DISABLED)

    def toggle_camera(self):
        if not self.camera_running:
            idx = int(self.combo_cam.get().split()[-1])
            self.cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY)
            if not self.cap or not self.cap.isOpened():
                messagebox.showerror("Error", f"No se pudo abrir la Cámara {idx}.")
                return
            self.camera_running = True
            self.btn_camera.config(text="Detener Video", bg="#dc2626")
            self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
            self.camera_thread.start()
            self._log(f"Cámara {idx} iniciada.")
        else:
            self.camera_running = False
            if self.cap:
                self.cap.release()
                self.cap = None
            self.btn_camera.config(text="Iniciar Video", bg="#059669")
            self.lbl_vision_info.config(text="Visión: Cámara detenida.")
            self._log("Cámara detenida.")

    def _camera_loop(self):
        while self.camera_running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.03)
                continue

            if self.var_flip.get():
                frame = cv2.flip(frame, 1)

            h, w, _ = frame.shape
            if self.board_detector.corners is None:
                self.board_detector.set_default_centered_roi(w, h, box_size=min(int(w * 0.62), h - 40))

            warped = self.board_detector.get_warped_board(frame)
            if warped is not None:
                _, stable_board = self.piece_detector.update_and_detect_board(warped)
            else:
                stable_board = [[EMPTY for _ in range(3)] for _ in range(3)]

            self.board_detector.draw_board_overlay(frame, color=(0, 245, 212), thickness=2)

            detected = [0] * 9
            for r in range(3):
                for c in range(3):
                    idx = r * 3 + c
                    piece = stable_board[r][c]
                    quad = self.board_detector.get_cell_quad_in_frame(r, c)
                    cx = int(np.mean(quad[:, 0, 0]))
                    cy = int(np.mean(quad[:, 0, 1]))
                    if piece == GATO:
                        detected[idx] = 1
                        cv2.putText(frame, "X", (cx - 12, cy + 12), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                    elif piece == RATON:
                        detected[idx] = 2
                        cv2.putText(frame, "O", (cx - 12, cy + 12), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 200, 0), 3)
                    else:
                        cv2.putText(frame, f"{idx+1}", (cx - 6, cy + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

            if self.var_auto_vision.get() and self.game_active and self.turn == "HUMANO":
                for k in range(9):
                    if detected[k] == 1 and self.board[k] == 0:
                        self.after(0, lambda pos=k: self.on_human_move(pos, source="CÁMARA"))
                        break

            self.current_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            time.sleep(0.03)

    def _update_canvas(self):
        if self.camera_running and self.current_frame is not None:
            cw = self.canvas_cam.winfo_width()
            ch = self.canvas_cam.winfo_height()
            if cw > 50 and ch > 50:
                img = Image.fromarray(self.current_frame).resize((cw, ch), Image.Resampling.BILINEAR)
                self.photo_img = ImageTk.PhotoImage(image=img)
                self.canvas_cam.create_image(0, 0, image=self.photo_img, anchor=tk.NW)
        self.after(30, self._update_canvas)

    def on_human_move(self, idx, source="CLIC"):
        if not self.game_active or self.board[idx] != 0 or self.turn != "HUMANO":
            return

        self.board[idx] = 1
        self.cell_buttons[idx].config(text="X", fg="#ef4444", bg="#1e293b")
        self._log(f"Humano ({source}) jugó en Casilla {idx+1}.")

        if check_winner(self.board, 1):
            self.game_active = False
            self.lbl_turn.config(text="★ ¡GANASTE! (X)", fg="#22c55e")
            self._log("Resultado: Ganó el humano.")
            return

        if 0 not in self.board:
            self.game_active = False
            self.lbl_turn.config(text="● EMPATE", fg="#f59e0b")
            self._log("Resultado: Empate.")
            return

        self.turn = "ROBOT"
        self.lbl_turn.config(text="● PENSANDO ROBOT (O)...", fg="#38bdf8")
        self.after(350, self._robot_turn)

    def _robot_turn(self):
        if not self.game_active:
            return
        move = minimax_best_move(self.board)
        if move is None:
            return

        self.board[move] = 2
        self.cell_buttons[move].config(text="O", fg="#38bdf8", bg="#0f172a")
        self._log(f"IA Minimax eligió Casilla {move+1} -> Robot se movería a Casilla {move+1}.")

        if check_winner(self.board, 2):
            self.game_active = False
            self.lbl_turn.config(text="● GANÓ EL ROBOT (O)", fg="#ef4444")
            self._log("Resultado: Ganó el robot.")
            return

        if 0 not in self.board:
            self.game_active = False
            self.lbl_turn.config(text="● EMPATE", fg="#f59e0b")
            self._log("Resultado: Empate.")
            return

        self.turn = "HUMANO"
        self.lbl_turn.config(text="● TU TURNO (X - Rojo)", fg="#22c55e")

    def reset_game(self):
        self.board = [0] * 9
        self.game_active = True
        self.turn = "HUMANO"
        self.last_detected_board = [0] * 9
        self.detection_stability_count = 0

        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.board_detector = BoardDetector(warped_size=450, config_file=os.path.join(base_dir, "board_config.json"))
        self.piece_detector = PieceDetector(config_file=os.path.join(base_dir, "piece_config.json"))
        for idx, btn in enumerate(self.cell_buttons):
            btn.config(text=f"{idx+1}", fg="#64748b", bg="#1e293b")
        self.lbl_turn.config(text="● TU TURNO (X - Rojo)", fg="#22c55e")
        self._log("Partida reiniciada.")

    def _on_close(self):
        self.camera_running = False
        if self.cap:
            self.cap.release()
        self.destroy()


if __name__ == "__main__":
    app = CameraGatoTester()
    app.mainloop()
