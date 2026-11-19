"""
Test independiente de Cámara y Lógica del Juego de Gato (sin conexión al robot).
Permite probar la detección HSV de piezas ('X' roja / 'O' azul) en el tablero 3x3,
el filtro de estabilidad temporal y la respuesta de la IA (Minimax) sin usar el ESP32.
"""

import sys
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import cv2
import numpy as np
from PIL import Image, ImageTk


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
            size = min(int(w * 0.62), h - 40)
            rx1 = (w - size) // 2
            ry1 = (h - size) // 2
            rx2 = rx1 + size
            ry2 = ry1 + size

            cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (0, 245, 212), 2)
            cv2.putText(frame, "TABLERO 3x3", (rx1 + 6, max(20, ry1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 245, 212), 2)

            cell_size = size // 3
            detected = [0] * 9
            thresh = self.var_px_thresh.get()

            for r in range(3):
                for c in range(3):
                    idx = r * 3 + c
                    cx1 = rx1 + c * cell_size
                    cy1 = ry1 + r * cell_size
                    cx2 = cx1 + cell_size
                    cy2 = cy1 + cell_size
                    cv2.rectangle(frame, (cx1, cy1), (cx2, cy2), (80, 80, 80), 1)

                    roi = frame[cy1 + 8:cy2 - 8, cx1 + 8:cx2 - 8]
                    red_px = 0
                    blue_px = 0
                    if roi.size > 0:
                        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                        m_red1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
                        m_red2 = cv2.inRange(hsv, np.array([170, 70, 50]), np.array([180, 255, 255]))
                        red_px = cv2.countNonZero(m_red1 | m_red2)

                        m_blue = cv2.inRange(hsv, np.array([90, 70, 50]), np.array([130, 255, 255]))
                        blue_px = cv2.countNonZero(m_blue)

                    if red_px > thresh:
                        detected[idx] = 1
                        cv2.putText(frame, f"X ({red_px})", (cx1 + 10, cy1 + cell_size // 2),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                    elif blue_px > thresh:
                        detected[idx] = 2
                        cv2.putText(frame, f"O ({blue_px})", (cx1 + 10, cy1 + cell_size // 2),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 200, 0), 2)
                    else:
                        cv2.putText(frame, f"{idx+1} R:{red_px} B:{blue_px}", (cx1 + 6, cy1 + 18),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180, 180, 180), 1)

            if self.var_auto_vision.get() and self.game_active and self.turn == "HUMANO":
                if detected == self.last_detected_board:
                    self.detection_stability_count += 1
                    if self.detection_stability_count == 7:
                        for k in range(9):
                            if detected[k] == 1 and self.board[k] == 0:
                                self.after(0, lambda pos=k: self.on_human_move(pos, source="CÁMARA"))
                                break
                else:
                    self.last_detected_board = list(detected)
                    self.detection_stability_count = 0

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
