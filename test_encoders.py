"""
================================================================================
PRUEBA Y DIAGNÓSTICO EXCLUSIVO DE ENCODERS MAGNÉTICOS AS5600 (12-BIT I2C)
PROYECTO: ROBOT 3 GDL JUGADOR DE GATO | PLATAFORMA: ESP32-S3
================================================================================
Este programa permite probar únicamente los 3 sensores AS5600 manteniendo intacto
el código de control de motores y cinemática del Robot 3 GDL:
- Al conectar, envía automáticamente el comando STOP para liberar los motores
  (ENA deshabilitado) y permitir girar los encoders libremente con la mano.
- Monitorea en tiempo real el estado del bus I2C de cada encoder:
  * Encoder 1 (Base):   SDA = GPIO 1  | SCL = GPIO 2  (con auto-inversión SDA/SCL)
  * Encoder 2 (Hombro): SDA = GPIO 8  | SCL = GPIO 9
  * Encoder 3 (Codo):   SDA = GPIO 10 | SCL = GPIO 11
- Muestra ángulo calibrado (0.00° a 360.00°), lectura cruda 12-bit (0 a 4095),
  ganancia AGC (0 a 255), detección de imán, velocidad manual (RPM y °/s),
  conteo de vueltas acumuladas y prueba de cobertura angular (0° a 360°).
================================================================================
"""

import math
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


class AS5600EncoderTesterApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Prueba de Encoders AS5600 (Modo Solo Sensores / Ejes Libres) | Robot 3 GDL ESP32-S3")
        self.geometry("1280x840")
        self.minsize(1020, 700)
        self.configure(bg="#05070c")

        self.num_encoders = 3
        self.pkt_seq = 500

        # Configuración visual y de pines por cada Encoder AS5600
        self.encoder_configs = [
            {
                "id": 1,
                "name": "ENCODER 1 (J1: BASE / YAW)",
                "pins": "Bus I2C #1  •  SDA: GPIO 1  |  SCL: GPIO 2  |  Addr: 0x36",
                "accent": "#38bdf8",
                "accent_dark": "#0284c7",
                "bg_card": "#0b1322",
                "border": "#1e3a5f",
            },
            {
                "id": 2,
                "name": "ENCODER 2 (J2: HOMBRO / PITCH)",
                "pins": "Bus I2C #2  •  SDA: GPIO 8  |  SCL: GPIO 9  |  Addr: 0x36",
                "accent": "#34d399",
                "accent_dark": "#059669",
                "bg_card": "#091918",
                "border": "#134e4a",
            },
            {
                "id": 3,
                "name": "ENCODER 3 (J3: CODO / PITCH)",
                "pins": "Bus I2C #3  •  SDA: GPIO 10 |  SCL: GPIO 11 |  Addr: 0x36",
                "accent": "#c084fc",
                "accent_dark": "#7c3aed",
                "bg_card": "#141026",
                "border": "#4c1d95",
            },
        ]

        # Estado en tiempo real de cada encoder
        self.encoders = [
            {
                "id": i + 1,
                "angle_deg": 0.0,
                "prev_angle_deg": None,
                "raw_value": 0,
                "agc": 0,
                "magnet_ok": False,
                "i2c_online": False,
                "last_rx_time": 0.0,
                "rpm_manual": 0.0,
                "deg_per_sec": 0.0,
                "turns": 0,
                "total_angle_deg": 0.0,
                "min_angle": None,
                "max_angle": None,
                "sectors_visited": set(),  # 12 sectores de 30° para verificar vuelta completa
                "rx_count": 0,
            }
            for i in range(self.num_encoders)
        ]

        self.serial_port = None
        self.running = False
        self.reader_thread = None

        self.canvases = []
        self.widgets = []

        self.setup_ui()
        self.refresh_ports()
        self.animate()
        # Auto-conectar si se detectó un puerto USB del ESP32-S3
        self.after(200, self.auto_connect_if_usb)

    def auto_connect_if_usb(self):
        if not serial or self.running:
            return
        for p in serial.tools.list_ports.comports():
            hwid = (p.hwid or "").upper()
            desc = (p.description or "").upper()
            if "303A" in hwid or ("USB" in desc and "BLUETOOTH" not in desc):
                self.combo_ports.set(p.device)
                self.toggle_connection()
                break

    # ==========================================================
    # PROTOCOLO SERIAL CON CHECKSUM XOR
    # ==========================================================
    def calculate_checksum(self, text):
        cs = 0
        for ch in text:
            cs ^= ord(ch)
        return cs

    def build_packet(self, encoder_id, cmd, p1=0.0, p2=0.0, p3=0.0):
        self.pkt_seq = (self.pkt_seq + 1) % 10000
        payload = f"PKT,{self.pkt_seq},{encoder_id},{cmd},{p1:.2f},{p2:.2f},{p3:.2f}"
        cs = self.calculate_checksum(payload)
        return f"${payload}*{cs:02X}\r\n"

    # ==========================================================
    # CONSTRUCCIÓN DE LA INTERFAZ GRÁFICA
    # ==========================================================
    def setup_ui(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TCombobox", fieldbackground="#11141c", background="#1b202c", foreground="#ffffff")
        style.map("TCombobox", fieldbackground=[("readonly", "#11141c")])

        # 1. BARRA SUPERIOR
        top_bar = tk.Frame(self, bg="#0b0e14", padx=14, pady=8, highlightthickness=1, highlightbackground="#1e293b")
        top_bar.pack(fill=tk.X, padx=12, pady=(8, 4))

        tk.Label(top_bar, text="🔌 PUERTO:", bg="#0b0e14", fg="#94a3b8", font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 4))
        self.combo_ports = ttk.Combobox(top_bar, width=12, state="readonly")
        self.combo_ports.pack(side=tk.LEFT, padx=3)

        btn_refresh = tk.Button(
            top_bar, text="🔄", bg="#1e293b", fg="#cbd5e1", activebackground="#334155",
            activeforeground="#ffffff", relief=tk.FLAT, font=("Segoe UI", 9), bd=0, padx=6, pady=2,
            cursor="hand2", command=self.refresh_ports
        )
        btn_refresh.pack(side=tk.LEFT, padx=2)

        tk.Label(top_bar, text="BAUD:", bg="#0b0e14", fg="#94a3b8", font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(10, 4))
        self.combo_baud = ttk.Combobox(top_bar, width=8, state="readonly", values=["115200", "9600", "230400"])
        self.combo_baud.set("115200")
        self.combo_baud.pack(side=tk.LEFT, padx=3)

        self.btn_connect = tk.Button(
            top_bar, text="CONECTAR", bg="#2563eb", fg="#ffffff", activebackground="#1d4ed8",
            activeforeground="#ffffff", font=("Segoe UI", 9, "bold"), padx=14, pady=3,
            relief=tk.FLAT, bd=0, cursor="hand2", command=self.toggle_connection
        )
        self.btn_connect.pack(side=tk.LEFT, padx=(10, 8))

        btn_zero_all = tk.Button(
            top_bar, text="🎯 FIJAR CERO A TODOS", bg="#1e293b", fg="#38bdf8",
            activebackground="#334155", activeforeground="#ffffff",
            font=("Segoe UI", 8, "bold"), padx=10, pady=3, relief=tk.FLAT, bd=0,
            cursor="hand2", command=self.cmd_zero_all
        )
        btn_zero_all.pack(side=tk.LEFT, padx=4)

        btn_reset_zero_all = tk.Button(
            top_bar, text="↺ BORRAR OFFSET CERO", bg="#1e293b", fg="#cbd5e1",
            activebackground="#334155", activeforeground="#ffffff",
            font=("Segoe UI", 8, "bold"), padx=10, pady=3, relief=tk.FLAT, bd=0,
            cursor="hand2", command=self.cmd_reset_zero_all
        )
        btn_reset_zero_all.pack(side=tk.LEFT, padx=4)

        btn_reset_stats = tk.Button(
            top_bar, text="🧹 LIMPIAR PRUEBA 360°", bg="#1e293b", fg="#fbbf24",
            activebackground="#334155", activeforeground="#ffffff",
            font=("Segoe UI", 8, "bold"), padx=10, pady=3, relief=tk.FLAT, bd=0,
            cursor="hand2", command=self.reset_all_test_stats
        )
        btn_reset_stats.pack(side=tk.LEFT, padx=4)

        btn_free_motors = tk.Button(
            top_bar, text="🔓 LIBERAR MOTORES (STOP)", bg="#065f46", fg="#a7f3d0",
            activebackground="#047857", activeforeground="#ffffff",
            font=("Segoe UI", 8, "bold"), padx=10, pady=3, relief=tk.FLAT, bd=0,
            cursor="hand2", command=self.cmd_release_motors
        )
        btn_free_motors.pack(side=tk.RIGHT, padx=4)

        self.lbl_status_badge = tk.Label(
            top_bar, text="● DESCONECTADO", bg="#0b0e14", fg="#ef4444",
            font=("Segoe UI", 9, "bold")
        )
        self.lbl_status_badge.pack(side=tk.RIGHT, padx=10)

        # 2. CONTENEDOR CENTRAL: 3 TARJETAS DE DIAGNÓSTICO DE ENCODERS
        center_frame = tk.Frame(self, bg="#05070c")
        center_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=4)

        for i in range(self.num_encoders):
            cfg = self.encoder_configs[i]
            card = tk.Frame(
                center_frame, bg=cfg["bg_card"],
                highlightthickness=1, highlightbackground=cfg["border"], padx=10, pady=8
            )
            card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)

            # Encabezado del Encoder
            head = tk.Frame(card, bg=cfg["bg_card"])
            head.pack(fill=tk.X, pady=(0, 4))

            title_col = tk.Frame(head, bg=cfg["bg_card"])
            title_col.pack(side=tk.LEFT, fill=tk.X, expand=True)

            tk.Label(
                title_col, text=cfg["name"], bg=cfg["bg_card"], fg=cfg["accent"],
                font=("Segoe UI", 10, "bold")
            ).pack(anchor=tk.W)

            tk.Label(
                title_col, text=cfg["pins"], bg=cfg["bg_card"], fg="#94a3b8",
                font=("Consolas", 8)
            ).pack(anchor=tk.W)

            badge_i2c = tk.Label(
                head, text="● SIN SEÑAL I2C", bg="#1e293b", fg="#f87171",
                font=("Segoe UI", 7, "bold"), padx=7, pady=2
            )
            badge_i2c.pack(side=tk.RIGHT)

            # Canvas del Dial Angular + Sectores de Cobertura 360°
            canvas = tk.Canvas(
                card, bg="#070a10", highlightthickness=1,
                highlightbackground="#162032"
            )
            canvas.pack(fill=tk.BOTH, expand=True, pady=4)
            canvas.bind("<Configure>", lambda e, idx=i: self.draw_encoder_gauge(idx))
            self.canvases.append(canvas)

            # Panel de Métricas Detalladas del Sensor AS5600
            metrics = tk.Frame(card, bg="#070a10", padx=8, pady=6, highlightthickness=1, highlightbackground="#162032")
            metrics.pack(fill=tk.X, pady=4)

            # Fila 1: Valor Crudo (12 bits) y Voltaje / Porcentaje equivalente
            r1 = tk.Frame(metrics, bg="#070a10")
            r1.pack(fill=tk.X, pady=1)
            tk.Label(r1, text="RAW (12-bit):", bg="#070a10", fg="#94a3b8", font=("Consolas", 8)).pack(side=tk.LEFT)
            lbl_raw = tk.Label(r1, text="0 / 4095 (0.0%)", bg="#070a10", fg="#f8fafc", font=("Consolas", 8, "bold"))
            lbl_raw.pack(side=tk.RIGHT)

            # Fila 2: Estado del Imán y Ganancia AGC
            r2 = tk.Frame(metrics, bg="#070a10")
            r2.pack(fill=tk.X, pady=1)
            tk.Label(r2, text="Estado Imán / AGC:", bg="#070a10", fg="#94a3b8", font=("Consolas", 8)).pack(side=tk.LEFT)
            lbl_mag = tk.Label(r2, text="SIN IMÁN  |  AGC: 0/255", bg="#070a10", fg="#f87171", font=("Consolas", 8, "bold"))
            lbl_mag.pack(side=tk.RIGHT)

            # Fila 3: Velocidad Manual y Vueltas Acumuladas
            r3 = tk.Frame(metrics, bg="#070a10")
            r3.pack(fill=tk.X, pady=1)
            tk.Label(r3, text="Giro Manual / Vueltas:", bg="#070a10", fg="#94a3b8", font=("Consolas", 8)).pack(side=tk.LEFT)
            lbl_turns = tk.Label(r3, text="0.0 °/s (0.0 RPM)  |  Vueltas: 0", bg="#070a10", fg=cfg["accent"], font=("Consolas", 8, "bold"))
            lbl_turns.pack(side=tk.RIGHT)

            # Fila 4: Cobertura de Prueba 360° (Sectores visitados al girar a mano)
            r4 = tk.Frame(metrics, bg="#070a10")
            r4.pack(fill=tk.X, pady=1)
            tk.Label(r4, text="Cobertura Giro 360°:", bg="#070a10", fg="#94a3b8", font=("Consolas", 8)).pack(side=tk.LEFT)
            lbl_cov = tk.Label(r4, text="0 / 12 sectores (0%)", bg="#070a10", fg="#cbd5e1", font=("Consolas", 8, "bold"))
            lbl_cov.pack(side=tk.RIGHT)

            # Botones de Calibración Individual del Encoder
            btn_row = tk.Frame(card, bg=cfg["bg_card"])
            btn_row.pack(fill=tk.X, pady=(2, 0))

            btn_zero = tk.Button(
                btn_row, text="🎯 Fijar 0.0° Aquí", bg="#1e293b", fg="#f8fafc",
                activebackground=cfg["accent_dark"], activeforeground="#ffffff",
                font=("Segoe UI", 8, "bold"), relief=tk.FLAT, bd=0, padx=8, pady=3,
                cursor="hand2", command=lambda idx=i: self.cmd_zero_single(idx)
            )
            btn_zero.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))

            btn_rst_zero = tk.Button(
                btn_row, text="↺ Ángulo Absoluto", bg="#1e293b", fg="#cbd5e1",
                activebackground="#334155", activeforeground="#ffffff",
                font=("Segoe UI", 8), relief=tk.FLAT, bd=0, padx=8, pady=3,
                cursor="hand2", command=lambda idx=i: self.cmd_reset_zero_single(idx)
            )
            btn_rst_zero.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

            btn_clr = tk.Button(
                btn_row, text="🧹 Reiniciar Conteo", bg="#1e293b", fg="#fbbf24",
                activebackground="#334155", activeforeground="#ffffff",
                font=("Segoe UI", 8), relief=tk.FLAT, bd=0, padx=8, pady=3,
                cursor="hand2", command=lambda idx=i: self.reset_single_test_stats(idx)
            )
            btn_clr.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))

            self.widgets.append({
                "badge_i2c": badge_i2c,
                "lbl_raw": lbl_raw,
                "lbl_mag": lbl_mag,
                "lbl_turns": lbl_turns,
                "lbl_cov": lbl_cov,
            })

        # 3. BARRA INFERIOR: MONITOR DE TRAMAS CRUDAS Y GUÍA DE DIAGNÓSTICO
        bottom_bar = tk.Frame(self, bg="#0b0e14", padx=14, pady=6, highlightthickness=1, highlightbackground="#1e293b")
        bottom_bar.pack(fill=tk.X, padx=12, pady=(2, 8))

        tk.Label(
            bottom_bar, text="📡 ÚLTIMA TRAMA SERIAL:", bg="#0b0e14",
            fg="#38bdf8", font=("Segoe UI", 8, "bold")
        ).pack(side=tk.LEFT, padx=(0, 6))

        self.lbl_last_raw_line = tk.Label(
            bottom_bar, text="Esperando conexión serial con el ESP32-S3...",
            bg="#0b0e14", fg="#94a3b8", font=("Consolas", 8)
        )
        self.lbl_last_raw_line.pack(side=tk.LEFT)

        tk.Label(
            bottom_bar,
            text="💡 Gira cada imán manualmente para rellenar el anillo verde de cobertura 360°",
            bg="#0b0e14", fg="#64748b", font=("Segoe UI", 8, "italic")
        ).pack(side=tk.RIGHT)

    # ==========================================================
    # DIBUJO DEL INSTRUMENTO CIRCULAR DEL ENCODER
    # ==========================================================
    def draw_encoder_gauge(self, idx):
        if idx >= len(self.canvases):
            return

        canvas = self.canvases[idx]
        canvas.delete("all")

        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w < 100 or h < 100:
            return

        enc = self.encoders[idx]
        cfg = self.encoder_configs[idx]
        deg = enc["angle_deg"]

        cx = w / 2
        cy = (h / 2) - 18
        radius = max(45, min(w, h - 55) * 0.39)

        # 1. Anillo exterior de cobertura de prueba (12 sectores de 30°)
        cov_r = radius + 12
        cov_box = [cx - cov_r, cy - cov_r, cx + cov_r, cy + cov_r]
        for s in range(12):
            start_tk = 90 - ((s + 1) * 30)
            visited = s in enc["sectors_visited"]
            col = "#10b981" if visited else "#1e293b"
            width_arc = 5 if visited else 3
            canvas.create_arc(
                cov_box, start=start_tk + 1, extent=28,
                style=tk.ARC, outline=col, width=width_arc
            )

        # 2. Círculo principal graduado 0° - 360° completo
        dial_box = [cx - radius, cy - radius, cx + radius, cy + radius]
        canvas.create_oval(dial_box, outline="#334155", width=2, fill="#090d16")

        # Graduaciones cada 15° (mayores cada 45°)
        for a in range(0, 360, 15):
            rad = math.radians(90.0 - a)
            is_major = (a % 45 == 0)
            t_len = 10 if is_major else 5
            x1 = cx + (radius - 2) * math.cos(rad)
            y1 = cy - (radius - 2) * math.sin(rad)
            x2 = cx + (radius - 2 - t_len) * math.cos(rad)
            y2 = cy - (radius - 2 - t_len) * math.sin(rad)

            canvas.create_line(
                x1, y1, x2, y2,
                fill="#e2e8f0" if is_major else "#475569",
                width=2.0 if is_major else 1.0
            )

            if is_major:
                tx = cx + (radius - t_len - 13) * math.cos(rad)
                ty = cy - (radius - t_len - 13) * math.sin(rad)
                canvas.create_text(
                    tx, ty, text=f"{a}°", fill="#94a3b8",
                    font=("Segoe UI", 8, "bold")
                )

        # 3. Aguja del Encoder
        needle_rad = math.radians(90.0 - deg)
        nx = cx + (radius - 8) * math.cos(needle_rad)
        ny = cy - (radius - 8) * math.sin(needle_rad)

        needle_col = cfg["accent"] if enc["i2c_online"] else "#475569"
        canvas.create_line(cx + 1, cy + 1, nx + 1, ny + 1, fill="#020408", width=4.0)
        canvas.create_line(cx, cy, nx, ny, fill=needle_col, width=3.0, capstyle=tk.ROUND)

        # Punto en la punta de la aguja
        canvas.create_oval(nx - 4, ny - 4, nx + 4, ny + 4, fill=needle_col, outline="#ffffff")

        # Pivote central
        canvas.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill="#ffffff", outline="#0f172a", width=2)

        # 4. Lectura Digital Grande debajo del dial
        y_text = cy + radius + 24
        canvas.create_text(
            cx, y_text,
            text=f"{deg:06.2f}°" if enc["i2c_online"] else "---.--°",
            fill="#ffffff" if enc["i2c_online"] else "#64748b",
            font=("Consolas", 18, "bold")
        )

        y_sub = y_text + 19
        canvas.create_text(
            cx, y_sub,
            text=f"Acumulado: {enc['total_angle_deg']:+.1f}°  |  Tramas OK: {enc['rx_count']}",
            fill=cfg["accent"],
            font=("Consolas", 8, "bold")
        )

    # ==========================================================
    # ACCIONES Y COMANDOS DE PRUEBA
    # ==========================================================
    def cmd_release_motors(self):
        pkt = self.build_packet(0, "STOP")
        self.send_serial_cmd(pkt)

    def cmd_zero_single(self, idx):
        pkt = self.build_packet(idx + 1, "ZERO")
        self.send_serial_cmd(pkt)
        self.reset_single_test_stats(idx)

    def cmd_zero_all(self):
        pkt = self.build_packet(0, "ZERO")
        self.send_serial_cmd(pkt)
        self.reset_all_test_stats()

    def cmd_reset_zero_single(self, idx):
        pkt = self.build_packet(idx + 1, "RESET_ZERO")
        self.send_serial_cmd(pkt)

    def cmd_reset_zero_all(self):
        pkt = self.build_packet(0, "RESET_ZERO")
        self.send_serial_cmd(pkt)

    def reset_single_test_stats(self, idx):
        enc = self.encoders[idx]
        enc["turns"] = 0
        enc["total_angle_deg"] = 0.0
        enc["prev_angle_deg"] = enc["angle_deg"]
        enc["sectors_visited"].clear()
        self.draw_encoder_gauge(idx)

    def reset_all_test_stats(self):
        for i in range(self.num_encoders):
            self.reset_single_test_stats(i)

    # ==========================================================
    # GESTIÓN DE PUERTO SERIAL
    # ==========================================================
    def send_serial_cmd(self, cmd_str):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(cmd_str.encode("utf-8"))
            except (serial.SerialException, OSError, PermissionError):
                self.handle_serial_disconnect()

    def handle_serial_disconnect(self):
        self.running = False
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
        except Exception:
            pass
        self.btn_connect.config(text="CONECTAR", bg="#2563eb")
        self.lbl_status_badge.config(text="● DESCONECTADO", fg="#ef4444")

    def refresh_ports(self):
        if not serial:
            return
        comports = list(serial.tools.list_ports.comports())
        ports = [p.device for p in comports]
        if ports:
            self.combo_ports["values"] = ports
            # Priorizar puerto USB del ESP32 (VID 303A o descripción USB) sobre puertos Bluetooth
            preferred = ports[0]
            for p in comports:
                desc = (p.description or "").upper()
                hwid = (p.hwid or "").upper()
                if "303A" in hwid or ("USB" in desc and "BLUETOOTH" not in desc):
                    preferred = p.device
                    break
            self.combo_ports.set(preferred)
        else:
            self.combo_ports["values"] = ["Sin puertos"]
            self.combo_ports.set("Sin puertos")

    def toggle_connection(self):
        if not serial:
            messagebox.showerror("Error", "pyserial no está instalado. Ejecuta: pip install pyserial")
            return

        if not self.running:
            p = self.combo_ports.get()
            if not p or p == "Sin puertos":
                messagebox.showwarning("Atención", "Selecciona un puerto COM válido.")
                return

            baud = int(self.combo_baud.get())
            try:
                self.serial_port = serial.Serial(
                    port=p,
                    baudrate=baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.1,
                )
                self.running = True
                self.btn_connect.config(text="DESCONECTAR", bg="#dc2626")
                self.lbl_status_badge.config(text="● EN LÍNEA (MODO SOLO ENCODERS)", fg="#22c55e")

                # Iniciar hilo lector y enviar STOP automático para dejar los ejes libres
                self.reader_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
                self.reader_thread.start()
                self.after(150, self.cmd_release_motors)
            except Exception as e:
                messagebox.showerror("Error de Conexión", f"No se pudo abrir {p}:\n{e}")
        else:
            self.handle_serial_disconnect()

    def serial_read_loop(self):
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                line = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                # Tramas del firmware: $TEL,<ID>,<CALIB_DEG>,<RAW>,<SPEED_HZ>,<ERROR>,<STATE>,<AGC>,<MAG_STAT>*<CS>
                if line.startswith("$") and "*" in line:
                    star_idx = line.rfind("*")
                    payload = line[1:star_idx]
                    cs_str = line[star_idx + 1:]
                    try:
                        received_cs = int(cs_str, 16)
                        if self.calculate_checksum(payload) == received_cs:
                            parts = payload.split(",")
                            if parts[0] == "TEL" and len(parts) >= 6:
                                m_id = int(parts[1])
                                if 1 <= m_id <= self.num_encoders:
                                    now = time.time()
                                    enc = self.encoders[m_id - 1]
                                    new_deg = float(parts[2])
                                    raw_val = int(parts[3])
                                    agc_val = int(parts[7]) if len(parts) > 7 else enc["agc"]
                                    mag_ok = (parts[8] == "1") if len(parts) > 8 else enc["magnet_ok"]

                                    # Cálculo de vueltas acumuladas y velocidad manual de giro
                                    if enc["prev_angle_deg"] is not None and enc["last_rx_time"] > 0:
                                        dt = now - enc["last_rx_time"]
                                        delta = new_deg - enc["prev_angle_deg"]
                                        if delta > 180.0:
                                            delta -= 360.0
                                            enc["turns"] -= 1
                                        elif delta < -180.0:
                                            delta += 360.0
                                            enc["turns"] += 1

                                        enc["total_angle_deg"] += delta
                                        if dt > 0.005:
                                            inst_dps = delta / dt
                                            enc["deg_per_sec"] = 0.6 * enc["deg_per_sec"] + 0.4 * inst_dps
                                            enc["rpm_manual"] = enc["deg_per_sec"] / 6.0

                                    enc["prev_angle_deg"] = new_deg
                                    enc["angle_deg"] = new_deg
                                    enc["raw_value"] = raw_val
                                    enc["agc"] = agc_val
                                    enc["magnet_ok"] = mag_ok
                                    enc["i2c_online"] = True
                                    enc["last_rx_time"] = now
                                    enc["rx_count"] += 1

                                    # Registrar sector de 30° visitado (0..11)
                                    sector = int((new_deg % 360.0) // 30.0)
                                    enc["sectors_visited"].add(sector)

                                    self.lbl_last_raw_line.config(text=line)
                    except ValueError:
                        pass
            except (serial.SerialException, OSError, PermissionError):
                self.handle_serial_disconnect()
                break
            except Exception:
                time.sleep(0.01)

    # ==========================================================
    # ACTUALIZACIÓN VISUAL EN TIEMPO REAL (~25 FPS)
    # ==========================================================
    def animate(self):
        now = time.time()
        for i in range(self.num_encoders):
            enc = self.encoders[i]
            w = self.widgets[i]

            # Verificar si el encoder sigue respondiendo por I2C (timeout 0.6 s)
            if now - enc["last_rx_time"] > 0.6:
                enc["i2c_online"] = False
                enc["deg_per_sec"] = 0.0
                enc["rpm_manual"] = 0.0

            if enc["i2c_online"]:
                w["badge_i2c"].config(text="● I2C CONECTADO OK", bg="#14532d", fg="#4ade80")
            else:
                w["badge_i2c"].config(text="● SIN RESPUESTA I2C", bg="#450a0a", fg="#f87171")

            pct_raw = (enc["raw_value"] / 4095.0) * 100.0
            w["lbl_raw"].config(text=f"{enc['raw_value']:4d} / 4095 ({pct_raw:5.1f}%)")

            if enc["i2c_online"] and enc["magnet_ok"]:
                w["lbl_mag"].config(
                    text=f"IMÁN ÓPTIMO  |  AGC: {enc['agc']:3d}/255",
                    fg="#4ade80"
                )
            elif enc["i2c_online"]:
                w["lbl_mag"].config(
                    text=f"SIN IMÁN / DÉBIL  |  AGC: {enc['agc']:3d}/255",
                    fg="#fbbf24"
                )
            else:
                w["lbl_mag"].config(text="SIN DATOS I2C", fg="#f87171")

            w["lbl_turns"].config(
                text=f"{enc['deg_per_sec']:+6.1f} °/s ({enc['rpm_manual']:+5.1f} RPM)  |  Vueltas: {enc['turns']:+d}"
            )

            cov_count = len(enc["sectors_visited"])
            cov_pct = int((cov_count / 12.0) * 100)
            cov_col = "#4ade80" if cov_count == 12 else ("#fbbf24" if cov_count > 0 else "#94a3b8")
            cov_txt = "✔ PRUEBA 360° COMPLETA" if cov_count == 12 else f"{cov_count:2d} / 12 sectores ({cov_pct}%)"
            w["lbl_cov"].config(text=cov_txt, fg=cov_col)

            self.draw_encoder_gauge(i)

        self.after(40, self.animate)


if __name__ == "__main__":
    app = AS5600EncoderTesterApp()
    app.mainloop()
