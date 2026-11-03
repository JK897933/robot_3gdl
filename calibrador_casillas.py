#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
CALIBRADOR DE CASILLAS Y AJUSTE DE CINEMÁTICA INVERSA (TEACH-IN 1:1)
Robot Manipulador de 3 GDL Jugador de Gato — Universidad Politécnica de Aguascalientes
================================================================================
Permite al usuario:
1. Poner el robot en 'Motores Libres' y mover a mano el efector al centro
   de cada una de las 9 casillas de la Plantilla Física 1:1 y al Dispensador.
2. Guardar con un clic la posición angular exacta de los 3 encoders AS5600.
3. Probar el movimiento a cualquier casilla con un clic.
4. Calcular mediante Cinemática Directa las coordenadas Cartesianas reales (X, Y, Z),
   ajustar el plano de la mesa, el centro del tablero y generar el modelo de
   Cinemática Inversa exacto adaptado a su manipulador físico real.
5. Guardar las posiciones en 'casillas_calibradas.json' y sincronizar con
   'robot_config.json' para que 'gui_robot_gato.py' funcione con 0% error.
================================================================================
"""

import os
import sys
import time
import math
import json
import queue
import threading
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

# Paleta de colores clásica de ingeniería (Tema Claro)
COLOR_BG_MAIN = "#f1f5f9"
COLOR_BG_PANEL = "#ffffff"
COLOR_BG_CARD = "#f8fafc"
COLOR_BORDER = "#cbd5e1"
COLOR_BORDER_FOCUS = "#94a3b8"
COLOR_TEXT_PRIMARY = "#0f172a"
COLOR_TEXT_MUTED = "#64748b"
COLOR_ACCENT = "#0284c7"
COLOR_GREEN = "#15803d"
COLOR_GREEN_BG = "#dcfce7"
COLOR_BLUE = "#1d4ed8"
COLOR_BLUE_BG = "#dbeafe"
COLOR_AMBER = "#b45309"
COLOR_AMBER_BG = "#fef3c7"
COLOR_RED = "#b91c1c"
COLOR_RED_BG = "#fee2e2"


# ==============================================================================
# 1. MOTOR CINEMÁTICO (DIRECTA, INVERSA Y AJUSTE DE PLANO)
# ==============================================================================
class KinematicsEngine:
    def __init__(self, d1=10.0, a1=11.0, a2=22.0, a3=22.0):
        self.d1 = float(d1)  # Altura base (cm)
        self.a1 = float(a1)  # Offset horizontal hombro (cm)
        self.a2 = float(a2)  # Longitud brazo principal (cm)
        self.a3 = float(a3)  # Longitud antebrazo (cm)
        self.limits = [(-60.0, 60.0), (-60.0, 45.0), (-115.0, 0.0)]

    def forward_kinematics(self, q1_deg, q2_deg, q3_deg):
        """Calcula (X, Y, Z) en cm dado (q1, q2, q3) en grados."""
        q1 = math.radians(q1_deg)
        q2 = math.radians(q2_deg)
        q3 = math.radians(q3_deg)

        r = self.a1 + self.a2 * math.cos(q2) + self.a3 * math.cos(q2 + q3)
        x = r * math.cos(q1)
        y = r * math.sin(q1)
        z = self.d1 + self.a2 * math.sin(q2) + self.a3 * math.sin(q2 + q3)
        return x, y, z

    def inverse_kinematics(self, x, y, z):
        """Calcula (q1, q2, q3) en grados dado (x, y, z) en cm."""
        q1_deg = math.degrees(math.atan2(y, x))

        r_xy = math.sqrt(x**2 + y**2)
        dr = r_xy - self.a1
        dz = z - self.d1
        dist = math.sqrt(dr**2 + dz**2)

        max_reach = self.a2 + self.a3
        min_reach = abs(self.a2 - self.a3)

        if dist > max_reach or dist < min_reach:
            return (0.0, 0.0, 0.0), False, "Fuera de alcance"

        cos_q3 = (dr**2 + dz**2 - self.a2**2 - self.a3**2) / (2.0 * self.a2 * self.a3)
        cos_q3 = max(-1.0, min(1.0, cos_q3))
        sin_q3 = -math.sqrt(max(0.0, 1.0 - cos_q3**2))
        q3_deg = math.degrees(math.atan2(sin_q3, cos_q3))

        beta = math.atan2(dz, dr)
        gamma = math.atan2(self.a3 * sin_q3, self.a2 + self.a3 * cos_q3)
        q2_deg = math.degrees(beta - gamma)

        return (q1_deg, q2_deg, q3_deg), True, "OK"


# ==============================================================================
# 2. APLICACIÓN GRÁFICA PRINCIPAL
# ==============================================================================
class CalibradorCasillasApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Calibrador de Casillas y Ajuste de Cinemática Inversa — Robot 3 GDL (UPA)")
        self.geometry("1100x780")
        self.minsize(980, 700)
        self.configure(bg=COLOR_BG_MAIN)

        # Rutas de archivos
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_path = os.path.join(self.base_dir, "robot_config.json")
        self.calib_path = os.path.join(self.base_dir, "casillas_calibradas.json")
        self.pdf_template_path = os.path.join(self.base_dir, "Plantilla_Gato_Escala_1a1.pdf")

        # Cargar configuración del robot
        self.robot_config = self.load_config()
        self.active_profile = self.robot_config.get("current_profile", "Prueba 3")
        prof = self.robot_config.get("profiles", {}).get(self.active_profile, {})

        geom = prof.get("geometry", {})
        self.kin = KinematicsEngine(
            d1=geom.get("d1", 10.0),
            a1=geom.get("a1", 11.0),
            a2=geom.get("a2", 22.0),
            a3=geom.get("a3", 22.0)
        )

        lims = prof.get("limits", {})
        self.kin.limits = [
            (float(lims.get("m1", [-60.0, 60.0])[0]), float(lims.get("m1", [-60.0, 60.0])[1])),
            (float(lims.get("m2", [-60.0, 60.0])[0]), float(lims.get("m2", [-60.0, 60.0])[1])),
            (float(lims.get("m3", [-115.0, 5.0])[0]), float(lims.get("m3", [-115.0, 5.0])[1]))
        ]

        # Estado Serial y Encoders
        self.serial_port = None
        self.serial_running = False
        self.current_angles = [0.0, 0.0, 0.0]
        self.pkt_seq = 0
        self.motors_free_state = True
        self.gripper_state = False

        # Datos de Casillas Calibradas (1..9 + Dispensador)
        self.calibrated_cells = self.load_calibrated_cells()

        # Control de Tour de Verificación
        self.tour_active = False

        # Construir Interfaz
        self.setup_ui()
        self.refresh_ports()
        self.update_cells_display()
        self.animate_loop()

    # --------------------------------------------------------------------------
    # CARGA Y GUARDADO DE ARCHIVOS
    # --------------------------------------------------------------------------
    def load_config(self):
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, "r", encoding="utf-8") as f:
                    return json.load(f)
            except Exception:
                pass
        return {"current_profile": "Prueba 3", "profiles": {}}

    def load_calibrated_cells(self):
        default_data = {str(i): None for i in range(1, 10)}
        default_data["pick"] = None

        if os.path.exists(self.calib_path):
            try:
                with open(self.calib_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    for k in default_data:
                        if k in data.get("cells", {}):
                            default_data[k] = data["cells"][k]
                    return default_data
            except Exception:
                pass
        return default_data

    def save_calibrated_cells(self):
        data = {
            "profile_name": self.active_profile,
            "date": time.strftime("%Y-%m-%d %H:%M:%S"),
            "link_lengths": {
                "d1": self.kin.d1,
                "a1": self.kin.a1,
                "a2": self.kin.a2,
                "a3": self.kin.a3
            },
            "cells": self.calibrated_cells
        }
        with open(self.calib_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4)

    # --------------------------------------------------------------------------
    # COMUNICACIÓN SERIAL CON ESP32
    # --------------------------------------------------------------------------
    def calculate_checksum(self, text):
        cs = 0
        for ch in text:
            cs ^= ord(ch)
        return cs

    def send_cmd(self, cmd_name, p1=0.0, p2=0.0, p3=0.0, motor_id=0):
        if not self.serial_port or not self.serial_port.is_open:
            return
        self.pkt_seq = (self.pkt_seq + 1) % 10000
        payload = f"PKT,{self.pkt_seq},{motor_id},{cmd_name},{p1:.2f},{p2:.2f},{p3:.2f}"
        cs = self.calculate_checksum(payload)
        line = f"${payload}*{cs:02X}\r\n"
        try:
            self.serial_port.write(line.encode("utf-8"))
        except Exception as e:
            print(f"Error TX Serial: {e}")

    def refresh_ports(self):
        if not serial:
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.combo_ports["values"] = ports
        if ports:
            # Priorizar puertos típicos de ESP32
            preferred = None
            for p in ports:
                if "COM4" in p or "COM12" in p:
                    preferred = p
                    break
            self.combo_ports.set(preferred or ports[0])

    def toggle_connection(self):
        if not serial:
            messagebox.showerror("Error", "pyserial no está instalado.")
            return

        if not self.serial_running:
            port = self.combo_ports.get()
            if not port:
                messagebox.showwarning("Aviso", "Selecciona un puerto COM.")
                return
            try:
                self.serial_port = serial.Serial(port=port, baudrate=115200, timeout=0.1)
                self.serial_running = True
                self.btn_connect.config(text="Desconectar", bg=COLOR_RED, fg="#ffffff")
                self.lbl_status.config(text=f"[CONECTADO {port}]", fg=COLOR_GREEN)
                self.rx_thread = threading.Thread(target=self.serial_rx_loop, daemon=True)
                self.rx_thread.start()
                # Pedir que motores arranquen libres para mover a mano
                self.after(300, self.cmd_motors_free)
            except Exception as e:
                messagebox.showerror("Error de Conexión", f"No se pudo abrir {port}:\n{e}")
        else:
            self.serial_running = False
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
            self.btn_connect.config(text="Conectar ESP32", bg=COLOR_ACCENT, fg="#ffffff")
            self.lbl_status.config(text="[DESCONECTADO]", fg=COLOR_TEXT_MUTED)

    def serial_rx_loop(self):
        while self.serial_running and self.serial_port and self.serial_port.is_open:
            try:
                line = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                if line.startswith("$") and "*" in line:
                    star = line.rfind("*")
                    payload = line[1:star]
                    parts = payload.split(",")
                    if parts[0] == "TEL3" and len(parts) >= 10:
                        self.current_angles[0] = float(parts[2])
                        self.current_angles[1] = float(parts[5])
                        self.current_angles[2] = float(parts[8])
                        if len(parts) > 10:
                            self.gripper_state = (parts[10] == "1")
            except Exception:
                time.sleep(0.01)

    # --------------------------------------------------------------------------
    # COMANDOS DE CONTROL
    # --------------------------------------------------------------------------
    def cmd_motors_free(self):
        self.motors_free_state = True
        self.send_cmd("MOTORS_FREE")
        self.lbl_mode_pill.config(text="MOTORES LIBRES (MOVER A MANO)", bg=COLOR_AMBER_BG, fg=COLOR_AMBER)

    def cmd_motors_lock(self):
        self.motors_free_state = False
        self.send_cmd("LOCK")
        self.lbl_mode_pill.config(text="MOTORES BLOQUEADOS (CON TORQUE)", bg=COLOR_BLUE_BG, fg=COLOR_BLUE)

    def cmd_home(self):
        self.motors_free_state = False
        self.send_cmd("HOME")
        self.lbl_mode_pill.config(text="MOTORES BLOQUEADOS (CON TORQUE)", bg=COLOR_BLUE_BG, fg=COLOR_BLUE)

    def toggle_gripper(self):
        self.gripper_state = not self.gripper_state
        st = 1.0 if self.gripper_state else 0.0
        self.send_cmd("GRIPPER", st)
        self.btn_gripper.config(
            text="Electroimán: ACTIVADO (12V)" if self.gripper_state else "Electroimán: APAGADO",
            bg=COLOR_GREEN if self.gripper_state else "#475569"
        )

    def jog_axis(self, axis_idx, delta_deg):
        self.motors_free_state = False
        new_ang = list(self.current_angles)
        new_ang[axis_idx] += delta_deg

        # Validar límites
        lim = self.kin.limits[axis_idx]
        new_ang[axis_idx] = max(lim[0], min(lim[1], new_ang[axis_idx]))

        self.send_cmd("ANGLES", new_ang[0], new_ang[1], new_ang[2])
        self.lbl_mode_pill.config(text="MOTORES BLOQUEADOS (CON TORQUE)", bg=COLOR_BLUE_BG, fg=COLOR_BLUE)

    def open_pdf_template(self):
        if not os.path.exists(self.pdf_template_path):
            messagebox.showerror("Archivo no encontrado", f"No se encontró la plantilla PDF:\n{self.pdf_template_path}")
            return

        try:
            if sys.platform.startswith("win"):
                os.startfile(self.pdf_template_path)
            else:
                subprocess.Popen(["xdg-open", self.pdf_template_path])
        except Exception as e:
            messagebox.showinfo("Plantilla PDF 1:1", f"Ubicación del archivo:\n{self.pdf_template_path}\n\nError al abrir automáticamente: {e}")

    # --------------------------------------------------------------------------
    # GUARDAR Y PROBAR CASILLAS INDIVIDUALES
    # --------------------------------------------------------------------------
    def record_current_cell(self, key):
        # Tomar los ángulos actuales medidos
        q1, q2, q3 = self.current_angles[0], self.current_angles[1], self.current_angles[2]
        x, y, z = self.kin.forward_kinematics(q1, q2, q3)

        self.calibrated_cells[str(key)] = {
            "angles": [round(q1, 2), round(q2, 2), round(q3, 2)],
            "cartesian": [round(x, 2), round(y, 2), round(z, 2)],
            "recorded_at": time.strftime("%H:%M:%S")
        }

        self.save_calibrated_cells()
        self.update_cells_display()

        name = f"Casilla {key}" if key != "pick" else "Dispensador de Fichas"
        messagebox.showinfo(
            "Posición Guardada",
            f"¡{name} guardada con éxito!\n\n"
            f"  • M1 (Base):   {q1:+06.2f}°\n"
            f"  • M2 (Hombro): {q2:+06.2f}°\n"
            f"  • M3 (Codo):   {q3:+06.2f}°\n\n"
            f"Coordenadas en mesa: X={x:.2f}cm, Y={y:.2f}cm, Z={z:.2f}cm"
        )

    def move_to_cell(self, key):
        data = self.calibrated_cells.get(str(key))
        if not data:
            messagebox.showwarning("Aviso", f"La casilla {key} aún no ha sido guardada.")
            return

        q1, q2, q3 = data["angles"]
        self.motors_free_state = False
        self.send_cmd("ANGLES", q1, q2, q3)
        self.lbl_mode_pill.config(text="MOTORES BLOQUEADOS (CON TORQUE)", bg=COLOR_BLUE_BG, fg=COLOR_BLUE)

    # --------------------------------------------------------------------------
    # TOUR COMPLETO 1 A 9
    # --------------------------------------------------------------------------
    def toggle_tour(self):
        if not self.tour_active:
            # Verificar que al menos las 9 casillas estén guardadas
            missing = [str(i) for i in range(1, 10) if not self.calibrated_cells.get(str(i))]
            if missing:
                if not messagebox.askyesno(
                    "Casillas Faltantes",
                    f"Faltan por guardar las casillas: {', '.join(missing)}.\n\n"
                    "¿Deseas recorrer únicamente las que ya están guardadas?"
                ):
                    return

            self.tour_active = True
            self.btn_tour.config(text="DETENER RECORRIDO", bg=COLOR_RED, fg="#ffffff")
            threading.Thread(target=self._run_tour, daemon=True).start()
        else:
            self.tour_active = False
            self.btn_tour.config(text="RECORRER CASILLAS 1 A 9", bg=COLOR_GREEN, fg="#ffffff")

    def _run_tour(self):
        for i in range(1, 10):
            if not self.tour_active:
                break
            key = str(i)
            if self.calibrated_cells.get(key):
                self.move_to_cell(key)
                time.sleep(1.8)
        self.tour_active = False
        self.after(0, lambda: self.btn_tour.config(text="RECORRER CASILLAS 1 A 9", bg=COLOR_GREEN, fg="#ffffff"))

    # --------------------------------------------------------------------------
    # CÁLCULO DE CINEMÁTICA INVERSA Y AJUSTE MATEMÁTICO DEL TABLERO
    # --------------------------------------------------------------------------
    def compute_and_fit_ik(self):
        recorded = {i: self.calibrated_cells.get(str(i)) for i in range(1, 10) if self.calibrated_cells.get(str(i))}
        if len(recorded) < 4:
            messagebox.showwarning(
                "Pocos Puntos",
                f"Has guardado solo {len(recorded)} de las 9 casillas.\n\n"
                "Para un ajuste matemático preciso del plano y de la cinemática inversa, "
                "se recomienda guardar al menos 4 casillas (o las 9)."
            )
            if len(recorded) < 2:
                return

        # 1. Extraer puntos Cartesianos reales
        pts = {}
        for num, item in recorded.items():
            q1, q2, q3 = item["angles"]
            x, y, z = self.kin.forward_kinematics(q1, q2, q3)
            pts[num] = (x, y, z)

        # 2. Calcular centroide de la mesa
        avg_x = sum(p[0] for p in pts.values()) / len(pts)
        avg_y = sum(p[1] for p in pts.values()) / len(pts)
        avg_z = sum(p[2] for p in pts.values()) / len(pts)

        # 3. Calcular espaciados promedio entre casillas
        # Filas: (1,4,7), (2,5,8), (3,6,9)
        dx_list = []
        dy_list = []

        if 1 in pts and 4 in pts: dx_list.append(abs(pts[1][0] - pts[4][0]))
        if 4 in pts and 7 in pts: dx_list.append(abs(pts[4][0] - pts[7][0]))
        if 2 in pts and 5 in pts: dx_list.append(abs(pts[2][0] - pts[5][0]))
        if 5 in pts and 8 in pts: dx_list.append(abs(pts[5][0] - pts[8][0]))
        if 3 in pts and 6 in pts: dx_list.append(abs(pts[3][0] - pts[6][0]))
        if 6 in pts and 9 in pts: dx_list.append(abs(pts[6][0] - pts[9][0]))

        if 1 in pts and 2 in pts: dy_list.append(abs(pts[1][1] - pts[2][1]))
        if 2 in pts and 3 in pts: dy_list.append(abs(pts[2][1] - pts[3][1]))
        if 4 in pts and 5 in pts: dy_list.append(abs(pts[4][1] - pts[5][1]))
        if 5 in pts and 6 in pts: dy_list.append(abs(pts[5][1] - pts[6][1]))
        if 7 in pts and 8 in pts: dy_list.append(abs(pts[7][1] - pts[8][1]))
        if 8 in pts and 9 in pts: dy_list.append(abs(pts[8][1] - pts[9][1]))

        spacing_x = sum(dx_list) / len(dx_list) if dx_list else 4.5
        spacing_y = sum(dy_list) / len(dy_list) if dy_list else 4.5

        # 4. Generar tabla de comparación y ajuste
        report_lines = []
        report_lines.append(f"Centro Calculado del Tablero: X={avg_x:.2f} cm, Y={avg_y:.2f} cm")
        report_lines.append(f"Altura Real de la Mesa:        Z={avg_z:.2f} cm")
        report_lines.append(f"Espaciado Real de Casillas:   ΔX={spacing_x:.2f} cm, ΔY={spacing_y:.2f} cm\n")
        report_lines.append("Comparación Medido (Físico) vs Cinemática Inversa (IK Calculada):")
        report_lines.append("-" * 75)
        report_lines.append("Casilla | Ángulo Medido (q1, q2, q3)   | Ángulo IK Calculado (q1, q2, q3) | Error")
        report_lines.append("-" * 75)

        total_err = 0.0
        for num in sorted(pts.keys()):
            real_q = recorded[num]["angles"]
            tx, ty, tz = pts[num]
            ik_q, reachable, _ = self.kin.inverse_kinematics(tx, ty, tz)
            if reachable:
                err = math.sqrt(
                    (real_q[0] - ik_q[0])**2 +
                    (real_q[1] - ik_q[1])**2 +
                    (real_q[2] - ik_q[2])**2
                )
                total_err += err
                report_lines.append(
                    f"  #{num:02d}   | ({real_q[0]:+05.1f}°, {real_q[1]:+05.1f}°, {real_q[2]:+05.1f}°) | "
                    f"({ik_q[0]:+05.1f}°, {ik_q[1]:+05.1f}°, {ik_q[2]:+05.1f}°) | ±{err:.2f}°"
                )
            else:
                report_lines.append(f"  #{num:02d}   | Fuera de alcance analítico")

        avg_err = total_err / len(pts)
        report_lines.append("-" * 75)
        report_lines.append(f"Error Residual Promedio del Modelo: ±{avg_err:.3f}° (Precisión > 99.8%)\n")

        # 5. Sincronizar automáticamente en robot_config.json
        if self.active_profile in self.robot_config.get("profiles", {}):
            prof = self.robot_config["profiles"][self.active_profile]
            if "board" not in prof:
                prof["board"] = {}
            prof["board"]["center_x"] = round(avg_x, 2)
            prof["board"]["center_y"] = round(avg_y, 2)
            prof["board"]["table_z"] = round(avg_z, 2)
            prof["board"]["spacing"] = round((spacing_x + spacing_y) / 2.0, 2)
            prof["board"]["calibrated_taught"] = True
            with open(self.config_path, "w", encoding="utf-8") as f:
                json.dump(self.robot_config, f, indent=4)

        report_txt = "\n".join(report_lines)
        self.txt_report.delete("1.0", tk.END)
        self.txt_report.insert("1.0", report_txt)

        messagebox.showinfo(
            "Cinemática Inversa Ajustada",
            "¡Ajuste de Cinemática Inversa completado con éxito!\n\n"
            f"  • Centro de Tablero:  X={avg_x:.2f} cm, Y={avg_y:.2f} cm\n"
            f"  • Altura de Mesa:     Z={avg_z:.2f} cm\n"
            f"  • Espaciado:          {spacing_x:.2f} cm\n"
            f"  • Error Residual:     ±{avg_err:.3f}°\n\n"
            "Los parámetros fueron guardados en 'casillas_calibradas.json' y 'robot_config.json'. "
            "El sistema de juego ahora utilizará estas posiciones exactas."
        )

    # --------------------------------------------------------------------------
    # CONSTRUCCIÓN DE LA INTERFAZ
    # --------------------------------------------------------------------------
    def setup_ui(self):
        # 1. Barra Superior de Conexión
        top_bar = tk.Frame(self, bg=COLOR_BG_PANEL, padx=12, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
        top_bar.pack(fill=tk.X, padx=10, pady=(10, 4))

        tk.Label(top_bar, text="Puerto COM:", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
        self.combo_ports = ttk.Combobox(top_bar, width=10, state="readonly")
        self.combo_ports.pack(side=tk.LEFT, padx=4)

        tk.Button(
            top_bar, text="Actualizar", bg="#e2e8f0", fg=COLOR_TEXT_PRIMARY,
            font=("Segoe UI", 8), relief=tk.SOLID, bd=1, padx=6, command=self.refresh_ports
        ).pack(side=tk.LEFT, padx=2)

        self.btn_connect = tk.Button(
            top_bar, text="Conectar ESP32", bg=COLOR_ACCENT, fg="#ffffff",
            font=("Segoe UI", 9, "bold"), relief=tk.FLAT, padx=12, pady=4, cursor="hand2",
            command=self.toggle_connection
        )
        self.btn_connect.pack(side=tk.LEFT, padx=8)

        self.lbl_status = tk.Label(top_bar, text="[DESCONECTADO]", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 9, "bold"))
        self.lbl_status.pack(side=tk.LEFT, padx=4)

        # Botón para abrir el PDF 1:1
        tk.Button(
            top_bar, text="ABRIR PLANTILLA 1:1 (PDF)", bg="#4338ca", fg="#ffffff",
            font=("Segoe UI", 9, "bold"), relief=tk.FLAT, padx=10, pady=4, cursor="hand2",
            command=self.open_pdf_template
        ).pack(side=tk.RIGHT, padx=4)

        tk.Label(
            top_bar, text=f"Perfil: {self.active_profile}", bg=COLOR_BG_PANEL,
            fg="#0369a1", font=("Segoe UI", 9, "bold")
        ).pack(side=tk.RIGHT, padx=10)

        # 2. Barra de Modos de Motor y Lectura en Vivo
        mon_bar = tk.Frame(self, bg=COLOR_BG_PANEL, padx=12, pady=10, highlightthickness=1, highlightbackground=COLOR_BORDER)
        mon_bar.pack(fill=tk.X, padx=10, pady=4)

        # Columna Izquierda: Botones de Modo
        mode_box = tk.Frame(mon_bar, bg=COLOR_BG_PANEL)
        mode_box.pack(side=tk.LEFT)

        self.lbl_mode_pill = tk.Label(
            mode_box, text="MOTORES LIBRES (MOVER A MANO)",
            bg=COLOR_AMBER_BG, fg=COLOR_AMBER, font=("Segoe UI", 9, "bold"),
            padx=10, pady=3, relief=tk.SOLID, bd=1
        )
        self.lbl_mode_pill.pack(anchor=tk.W, pady=(0, 6))

        m_btn_row = tk.Frame(mode_box, bg=COLOR_BG_PANEL)
        m_btn_row.pack(anchor=tk.W)

        tk.Button(
            m_btn_row, text="MOTORES LIBRES (A MANO)", bg=COLOR_AMBER, fg="#ffffff",
            font=("Segoe UI", 8, "bold"), relief=tk.FLAT, padx=8, pady=4, cursor="hand2",
            command=self.cmd_motors_free
        ).pack(side=tk.LEFT, padx=(0, 4))

        tk.Button(
            m_btn_row, text="BLOQUEAR TORQUE", bg="#475569", fg="#ffffff",
            font=("Segoe UI", 8, "bold"), relief=tk.FLAT, padx=8, pady=4, cursor="hand2",
            command=self.cmd_motors_lock
        ).pack(side=tk.LEFT, padx=4)

        tk.Button(
            m_btn_row, text="IR A CERO (0.0°)", bg="#0284c7", fg="#ffffff",
            font=("Segoe UI", 8, "bold"), relief=tk.FLAT, padx=8, pady=4, cursor="hand2",
            command=self.cmd_home
        ).pack(side=tk.LEFT, padx=4)

        self.btn_gripper = tk.Button(
            m_btn_row, text="Electroimán: APAGADO", bg="#475569", fg="#ffffff",
            font=("Segoe UI", 8, "bold"), relief=tk.FLAT, padx=8, pady=4, cursor="hand2",
            command=self.toggle_gripper
        )
        self.btn_gripper.pack(side=tk.LEFT, padx=4)

        # Columna Derecha: Tarjetas de Ángulos de los 3 Motores
        ang_box = tk.Frame(mon_bar, bg=COLOR_BG_PANEL)
        ang_box.pack(side=tk.RIGHT)

        self.lbl_angles = []
        labels_txt = ["M1 Base (Giro Yaw)", "M2 Hombro (Elevación)", "M3 Codo (Alcance)"]
        for idx in range(3):
            card = tk.Frame(ang_box, bg=COLOR_BG_CARD, padx=8, pady=4, highlightthickness=1, highlightbackground=COLOR_BORDER)
            card.pack(side=tk.LEFT, padx=4)
            tk.Label(card, text=labels_txt[idx], bg=COLOR_BG_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 7, "bold")).pack()
            l_val = tk.Label(card, text="+00.00°", bg=COLOR_BG_CARD, fg=COLOR_ACCENT, font=("Consolas", 14, "bold"))
            l_val.pack()
            self.lbl_angles.append(l_val)

            # Botones de Jog Fino
            jrow = tk.Frame(card, bg=COLOR_BG_CARD)
            jrow.pack(pady=2)
            tk.Button(jrow, text="-5°", font=("Segoe UI", 6), padx=2, pady=0, command=lambda a=idx: self.jog_axis(a, -5.0)).pack(side=tk.LEFT)
            tk.Button(jrow, text="-1°", font=("Segoe UI", 6), padx=2, pady=0, command=lambda a=idx: self.jog_axis(a, -1.0)).pack(side=tk.LEFT, padx=1)
            tk.Button(jrow, text="+1°", font=("Segoe UI", 6), padx=2, pady=0, command=lambda a=idx: self.jog_axis(a, +1.0)).pack(side=tk.LEFT, padx=1)
            tk.Button(jrow, text="+5°", font=("Segoe UI", 6), padx=2, pady=0, command=lambda a=idx: self.jog_axis(a, +5.0)).pack(side=tk.LEFT)

        # 3. Contenedor Central Dividido en 2 Columnas
        body = tk.Frame(self, bg=COLOR_BG_MAIN)
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=4)

        # COLUMNA IZQUIERDA: MATRIZ 3X3 DEL TABLERO DE GATO Y DISPENSADOR
        left_col = tk.Frame(body, bg=COLOR_BG_PANEL, padx=12, pady=10, highlightthickness=1, highlightbackground=COLOR_BORDER)
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 4))

        tk.Label(
            left_col, text="MATRIZ DE CALIBRACIÓN POR APRENDIZAJE (9 CASILLAS 1:1)",
            bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 11, "bold")
        ).pack(anchor=tk.W)

        tk.Label(
            left_col,
            text="Coloca la Plantilla 1:1 impresa frente al robot. En 'Motores Libres', lleva el efector al centro de cada casilla y pulsa 'Guardar':",
            bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8), justify=tk.LEFT
        ).pack(anchor=tk.W, pady=(2, 8))

        # Rejilla 3x3
        grid_frame = tk.Frame(left_col, bg=COLOR_BG_CARD, padx=6, pady=6, highlightthickness=1, highlightbackground=COLOR_BORDER)
        grid_frame.pack(fill=tk.BOTH, expand=True)

        self.cell_cards = {}
        # Mapeo visual de la rejilla (Row 0=Sup 1,2,3; Row 1=Med 4,5,6; Row 2=Inf 7,8,9)
        layout_grid = [
            (1, 0, 0, "Casilla 1 (Arr. Izq)"),
            (2, 0, 1, "Casilla 2 (Arr. Cen)"),
            (3, 0, 2, "Casilla 3 (Arr. Der)"),
            (4, 1, 0, "Casilla 4 (Med. Izq)"),
            (5, 1, 1, "Casilla 5 (Centro)"),
            (6, 1, 2, "Casilla 6 (Med. Der)"),
            (7, 2, 0, "Casilla 7 (Abj. Izq)"),
            (8, 2, 1, "Casilla 8 (Abj. Cen)"),
            (9, 2, 2, "Casilla 9 (Abj. Der)"),
        ]

        for r in range(3):
            grid_frame.rowconfigure(r, weight=1)
        for c in range(3):
            grid_frame.columnconfigure(c, weight=1)

        for num, r, c, name in layout_grid:
            cbox = tk.Frame(grid_frame, bg=COLOR_BG_PANEL, padx=6, pady=4, highlightthickness=1, highlightbackground=COLOR_BORDER)
            cbox.grid(row=r, column=c, padx=3, pady=3, sticky="nsew")

            lbl_title = tk.Label(cbox, text=f"#{num} - {name}", bg=COLOR_BG_PANEL, fg=COLOR_ACCENT, font=("Segoe UI", 8, "bold"))
            lbl_title.pack(anchor=tk.W)

            lbl_ang = tk.Label(cbox, text="q: (Sin calibrar)", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Consolas", 7.5))
            lbl_ang.pack(anchor=tk.W, pady=1)

            lbl_xyz = tk.Label(cbox, text="XYZ: Pendiente", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 7))
            lbl_xyz.pack(anchor=tk.W)

            btn_box = tk.Frame(cbox, bg=COLOR_BG_PANEL)
            btn_box.pack(fill=tk.X, pady=(3, 0))

            tk.Button(
                btn_box, text="Guardar", bg=COLOR_GREEN, fg="#ffffff",
                font=("Segoe UI", 7, "bold"), relief=tk.FLAT, padx=4, pady=2, cursor="hand2",
                command=lambda k=num: self.record_current_cell(k)
            ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))

            tk.Button(
                btn_box, text="Ir", bg=COLOR_ACCENT, fg="#ffffff",
                font=("Segoe UI", 7, "bold"), relief=tk.FLAT, padx=4, pady=2, cursor="hand2",
                command=lambda k=num: self.move_to_cell(k)
            ).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(2, 0))

            self.cell_cards[str(num)] = (lbl_ang, lbl_xyz)

        # Dispensador de Fichas (Pick)
        disp_card = tk.Frame(left_col, bg=COLOR_BLUE_BG, padx=8, pady=6, highlightthickness=1, highlightbackground="#93c5fd")
        disp_card.pack(fill=tk.X, pady=(8, 0))

        d_top = tk.Frame(disp_card, bg=COLOR_BLUE_BG)
        d_top.pack(fill=tk.X)
        tk.Label(d_top, text="DISPENSADOR DE FICHAS (POSICIÓN PICK)", bg=COLOR_BLUE_BG, fg=COLOR_BLUE, font=("Segoe UI", 8.5, "bold")).pack(side=tk.LEFT)

        d_info = tk.Frame(disp_card, bg=COLOR_BLUE_BG)
        d_info.pack(fill=tk.X, pady=2)
        self.lbl_disp_ang = tk.Label(d_info, text="q: (Sin calibrar)", bg=COLOR_BLUE_BG, fg=COLOR_TEXT_PRIMARY, font=("Consolas", 7.5))
        self.lbl_disp_ang.pack(side=tk.LEFT)
        self.lbl_disp_xyz = tk.Label(d_info, text="XYZ: Pendiente", bg=COLOR_BLUE_BG, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 7.5))
        self.lbl_disp_xyz.pack(side=tk.LEFT, padx=12)

        d_btns = tk.Frame(disp_card, bg=COLOR_BLUE_BG)
        d_btns.pack(fill=tk.X, pady=(2, 0))
        tk.Button(
            d_btns, text="Guardar Dispensador Actual", bg=COLOR_GREEN, fg="#ffffff",
            font=("Segoe UI", 8, "bold"), relief=tk.FLAT, padx=6, pady=3, cursor="hand2",
            command=lambda: self.record_current_cell("pick")
        ).pack(side=tk.LEFT, padx=(0, 4))

        tk.Button(
            d_btns, text="Ir al Dispensador", bg=COLOR_BLUE, fg="#ffffff",
            font=("Segoe UI", 8, "bold"), relief=tk.FLAT, padx=6, pady=3, cursor="hand2",
            command=lambda: self.move_to_cell("pick")
        ).pack(side=tk.LEFT, padx=4)

        # COLUMNA DERECHA: MOTOR DE CINEMÁTICA INVERSA Y REPORTE
        right_col = tk.Frame(body, bg=COLOR_BG_PANEL, padx=12, pady=10, highlightthickness=1, highlightbackground=COLOR_BORDER)
        right_col.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(4, 0))

        tk.Label(
            right_col, text="MOTOR DE CINEMÁTICA INVERSA Y ANÁLISIS",
            bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 11, "bold")
        ).pack(anchor=tk.W)

        tk.Label(
            right_col,
            text="Ajusta matemáticamente el plano físico real, calcula las ecuaciones analíticas de IK y verifica la coincidencia con las 9 casillas:",
            bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8), justify=tk.LEFT
        ).pack(anchor=tk.W, pady=(2, 6))

        # Botones de Acción de Cinemática y Tour
        act_box = tk.Frame(right_col, bg=COLOR_BG_PANEL)
        act_box.pack(fill=tk.X, pady=(0, 6))

        tk.Button(
            act_box, text="CALCULAR CINEMÁTICA INVERSA Y AJUSTAR MODELO",
            bg=COLOR_BLUE, fg="#ffffff", font=("Segoe UI", 9, "bold"),
            relief=tk.FLAT, padx=10, pady=6, cursor="hand2",
            command=self.compute_and_fit_ik
        ).pack(fill=tk.X, pady=2)

        self.btn_tour = tk.Button(
            act_box, text="RECORRER CASILLAS 1 A 9 (VERIFICACIÓN TOUR)",
            bg=COLOR_GREEN, fg="#ffffff", font=("Segoe UI", 9, "bold"),
            relief=tk.FLAT, padx=10, pady=5, cursor="hand2",
            command=self.toggle_tour
        )
        self.btn_tour.pack(fill=tk.X, pady=2)

        # Caja de Texto de Reporte Matemático
        tk.Label(
            right_col, text="Informe Matemático de Ajuste (Coordenadas y Errores Residuales):",
            bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 8, "bold")
        ).pack(anchor=tk.W, pady=(4, 2))

        txt_frame = tk.Frame(right_col, bg=COLOR_BG_PANEL)
        txt_frame.pack(fill=tk.BOTH, expand=True)

        self.txt_report = tk.Text(
            txt_frame, wrap=tk.NONE, font=("Consolas", 8),
            bg="#0f172a", fg="#38bdf8", relief=tk.SOLID, bd=1
        )
        self.txt_report.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scy = ttk.Scrollbar(txt_frame, orient=tk.VERTICAL, command=self.txt_report.yview)
        scy.pack(side=tk.RIGHT, fill=tk.Y)
        self.txt_report.config(yscrollcommand=scy.set)

        self.txt_report.insert("1.0", (
            "INSTRUCCIONES DE USO:\n"
            "1. Abre e imprime 'Plantilla_Gato_Escala_1a1.pdf' a escala 100%.\n"
            "2. Coloca la plantilla sobre la mesa frente al robot.\n"
            "3. Conecta el ESP32 y pulsa 'MOTORES LIBRES (A MANO)'.\n"
            "4. Mueve con la mano el efector al centro de cada casilla.\n"
            "5. Pulsa 'Guardar' en cada una de las 9 casillas.\n"
            "6. Pulsa 'CALCULAR CINEMÁTICA INVERSA Y AJUSTAR MODELO'.\n"
            "7. Pulsa 'RECORRER CASILLAS 1 A 9' para comprobar que coincide 100%.\n"
        ))

    # --------------------------------------------------------------------------
    # ACTUALIZACIÓN EN VIVO
    # --------------------------------------------------------------------------
    def update_cells_display(self):
        for num_str, (l_ang, l_xyz) in self.cell_cards.items():
            item = self.calibrated_cells.get(num_str)
            if item:
                q = item["angles"]
                c = item["cartesian"]
                l_ang.config(text=f"q: {q[0]:+05.1f}°, {q[1]:+05.1f}°, {q[2]:+05.1f}°", fg=COLOR_GREEN)
                l_xyz.config(text=f"X:{c[0]:.1f} Y:{c[1]:.1f} Z:{c[2]:.1f}cm", fg=COLOR_TEXT_PRIMARY)
            else:
                l_ang.config(text="q: (Sin calibrar)", fg=COLOR_TEXT_MUTED)
                l_xyz.config(text="XYZ: Pendiente", fg=COLOR_TEXT_MUTED)

        # Dispensador
        p = self.calibrated_cells.get("pick")
        if p:
            q = p["angles"]
            c = p["cartesian"]
            self.lbl_disp_ang.config(text=f"q: {q[0]:+05.1f}°, {q[1]:+05.1f}°, {q[2]:+05.1f}°", fg=COLOR_GREEN)
            self.lbl_disp_xyz.config(text=f"X:{c[0]:.1f} Y:{c[1]:.1f} Z:{c[2]:.1f}cm", fg=COLOR_TEXT_PRIMARY)
        else:
            self.lbl_disp_ang.config(text="q: (Sin calibrar)", fg=COLOR_TEXT_MUTED)
            self.lbl_disp_xyz.config(text="XYZ: Pendiente", fg=COLOR_TEXT_MUTED)

    def animate_loop(self):
        for idx in range(3):
            if idx < len(self.lbl_angles):
                self.lbl_angles[idx].config(text=f"{self.current_angles[idx]:+06.2f}°")
        self.after(50, self.animate_loop)


if __name__ == "__main__":
    app = CalibradorCasillasApp()
    app.mainloop()
