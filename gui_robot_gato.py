"""
================================================================================
UNIVERSIDAD POLITÉCNICA DE AGUASCALIENTES - INGENIERÍA EN MECATRÓNICA
SISTEMA DE CONTROL, SIMULACIÓN 3D Y VISIÓN ARTIFICIAL DEL ROBOT 3 GDL (MK2)
JUGADOR DE GATO (TRES EN RAYA / TIC-TAC-TOE)
================================================================================

ÍNDICE GENERAL TEMÁTICO DEL CÓDIGO FUENTE:
--------------------------------------------------------------------------------
• TEMA 1: PARÁMETROS FÍSICOS, LIBRERÍAS Y CONSTANTES DEL SISTEMA
  - Importaciones principales (Tkinter, PIL, OpenCV, PySerial, etc.)
  - Constantes geométricas, mecánicas y paleta de colores de la UI.

• TEMA 2: CINEMÁTICA DEL ROBOT MK2 (DIRECTA E INVERSA - FK / IK)
  - Parámetros Denavit-Hartenberg / Geometría Antropomórfica (d1, a1, a2, a3).
  - 2.1 Cinemática Directa (FK): Cálculo analítico de (X, Y, Z) a partir de (q1, q2, q3).
  - 2.2 Cinemática Inversa (IK): >>> [PARTE CLAVE] <<<
        Cálculo trigonométrico exhaustivo paso a paso para obtener los ángulos
        articulares (q1, q2, q3) a partir de coordenadas cartesianas (X, Y, Z):
        * Paso 1: Ángulo de la Base q1 (plano XY).
        * Paso 2: Reducción radial al plano del brazo (dr, dz).
        * Paso 3: Validación del espacio de trabajo (singularidades y límites de alcance).
        * Paso 4: Ángulo del Codo q3 por Ley de Cosenos (configuración codo arriba).
        * Paso 5: Ángulo del Hombro q2 por descomposición geométrica (beta - gamma).
        * Paso 6: Validación de límites físicos seguros de los servomotores/pasos.

• TEMA 3: DINÁMICA DE LAGRANGE-EULER EN TIEMPO REAL
  - Cálculo de torques estáticos de gravedad (tau1, tau2, tau3) en N*m.
  - Alerta de sobrecarga estructural y torque en articulación del hombro.

• TEMA 4: PROTOCOLO DE COMUNICACIÓN SERIAL CON ESP32
  - Cálculo de Checksum XOR de 8 bits.
  - Trama robusta de comandos enmarcados: $PKT,seq,ack,CMD,val1,val2,val3*CS\r\n
  - Despacho de comandos articulares, parada de emergencia, gripper y jugadas.
  - Hilo de lectura en segundo plano con parseo dual: telemetría $TEL3 y texto ASCII.

• TEMA 5: VISIÓN ARTIFICIAL Y PROCESAMIENTO DE IMÁGENES (OPENCV)
  - Captura de video en tiempo real desde WebCam.
  - Detección de ROI del tablero central 3x3 y de los dispensadores laterales.
  - Segmentación por color en espacio HSV (Fichas "X" del humano y "O" del robot).
  - Filtro de estabilidad temporal para reconocimiento automático de jugadas humanas.

• TEMA 6: INTELIGENCIA ARTIFICIAL Y LÓGICA DE JUEGO (MINIMAX)
  - Evaluación de condiciones de victoria en las 8 líneas del tablero.
  - Algoritmo de decisión Minimax optimizado (Ganar -> Bloquear -> Centro -> Esquinas).
  - Control de turnos, marcadores acumulados y reinicio de partida.

• TEMA 7: CINEMÁTICA OPERACIONAL Y SECUENCIA PICK & PLACE
  - Validación de IK con retroalimentación de Cinemática Directa (Error Euclidiano).
  - Máquina de estados automatizada para tomar ficha, elevar a PSS, colocar y volver a HOME.
  - Sincronización multi-hilo segura entre la GUI y la cinemática del robot.

• TEMA 8: SIMULACIÓN GRÁFICA 3D INTERACTIVA Y CÁMARA ORBITAL
  - Proyección tridimensional en Canvas Tkinter con transformaciones yaw/pitch/zoom.
  - Vistas rápidas (Isométrica, Planta, Perfil) y manipulación orbital con el ratón.
  - Renderizado de componentes: suelo cuadriculado, dispensadores, tablero, fichas y brazo MK2.
  - Mini radar bidimensional (planta 2D) en tiempo real.

• TEMA 9: INSTRUMENTACIÓN VIRTUAL ANALÓGICA (VELOCÍMETROS / GAUGES)
  - Indicadores semicirculares tipo reloj industrial por cada articulación (J1, J2, J3).
  - Arcos de estado (Normal, Precaución, Límite), aguja analógica y lectura angular.

• TEMA 10: CONSTRUCCIÓN DE LA INTERFAZ GRÁFICA (GUI TKINTER)
  - Diseño moderno "Dark Theme" estilo centro de control mecatrónico.
  - Panel Izquierdo: Visión artificial y tablero táctil interactivo.
  - Panel Central: Entradas numéricas de IK, accesos directos y Canvas 3D.
  - Panel Derecho: Instrumentación virtual, monitoreo de torques y botones de control.

• TEMA 11: CICLO DE ANIMACIÓN PRINCIPAL Y PUNTO DE ENTRADA
  - Bucle a 25 FPS para interpolación continua, telemetría y actualización visual.
  - Ejecución de la aplicación principal.
================================================================================
"""

import os
import json
import sys
import math
import re
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk

# Módulos opcionales con degradación elegante
try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


# ==============================================================================
# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ TEMA 1: PARÁMETROS FÍSICOS, LIBRERÍAS Y CONSTANTES DEL SISTEMA             ║
# ╚════════════════════════════════════════════════════════════════════════════╝
# ==============================================================================

# ==============================================================================
# PALETA DE COLORES CLÁSICA PROFESIONAL (ESTILO INGENIERÍA / LABVIEW / MATLAB)
# ==============================================================================
COLOR_BG_MAIN        = "#f1f5f9"  # Fondo general de escritorio (gris neutro claro)
COLOR_BG_PANEL       = "#ffffff"  # Fondo de paneles secundarios (blanco puro)
COLOR_BG_CARD        = "#f8fafc"  # Fondo de tarjetas y marcos de agrupación
COLOR_BORDER         = "#cbd5e1"  # Bordes nítidos clásicos de 1px
COLOR_BORDER_FOCUS   = "#94a3b8"  # Bordes de elementos destacados
COLOR_ACCENT         = "#1e40af"  # Azul clásico institucional para títulos
COLOR_CYAN_ACCENT    = "#1e40af"  # Azul clásico para títulos
COLOR_GREEN_SUCCESS  = "#15803d"  # Verde industrial sobrio
COLOR_AMBER_WARNING  = "#b45309"  # Ámbar industrial para advertencias
COLOR_RED_DANGER     = "#b91c1c"  # Rojo de seguridad para paros y límites
COLOR_TEXT_PRIMARY   = "#0f172a"  # Texto principal negro carbón de alto contraste
COLOR_TEXT_MUTED     = "#475569"  # Texto secundario gris técnico legible

# Constantes de reducción y calibración cinemática
GEAR_RATIO_CYCLOIDAL = 31.0               # Reducción cicloidal 31:1 en cada articulación
STEPS_PER_DEG_1_16   = 275.5556           # Micropasos por grado en resolución 1/16
STEPS_PER_DEG_1_8    = 137.7778           # Micropasos por grado en resolución 1/8
NOMINAL_MAX_ARM_RPM  = 0.97               # Velocidad máxima angular del brazo (~5.82 °/s)


# ==============================================================================
# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ TEMA 2: CINEMÁTICA DEL ROBOT MK2 (DIRECTA E INVERSA - FK / IK)             ║
# ╚════════════════════════════════════════════════════════════════════════════╝
# ==============================================================================
class RobotMK2Kinematics:
    """
    ============================================================================
    MODELO CINEMÁTICO COMPLETO DEL ROBOT ANTROPOMÓRFICO MK2 (RRR):
    ============================================================================
    Geometría dimensional del manipulador (medidas exactas de SolidWorks):
      - Eslabón 1 (Base rotatoria J1):  d1 = 10.0 cm  (Elevación vertical)
                                        a1 = 11.0 cm  (Offset horizontal del hombro)
      - Eslabón 2 (Hombro J2):          a2 = 32.922 cm (Longitud del brazo superior)
      - Eslabón 3 (Codo J3):            a3 = 32.934 cm (Longitud del antebrazo con efector)

    Alcance espacial:
      - Alcance máximo horizontal (brazo extendido): a1 + a2 + a3 ≈ 76.85 cm
      - Radio de trabajo del hombro-codo: (a2 + a3) = 65.856 cm
      - Zona ciega interna (singularidad de radio mínimo): |a2 - a3| ≈ 0.012 cm

    Límites mecánicos de rotación angular segura:
      - Articulación J1 (Base - Yaw):    [-90.0°, +90.0°]
      - Articulación J2 (Hombro - Pitch):[  0.0°, 115.0°]
      - Articulación J3 (Codo - Pitch):  [-160.0°,   0.0°]
    ============================================================================
    """

    def __init__(self):
        # Dimensiones en centímetros para cinemática cartesiana
        self.d1 = 10.0      # cm: Altura del eje de la base al eje del hombro
        self.a1 = 11.0      # cm: Desplazamiento horizontal desde el centro de giro
        self.a2 = 32.922    # cm: Eslabón 2 (Brazo - 329.22 mm)
        self.a3 = 32.934    # cm: Eslabón 3 (Antebrazo - 329.34 mm)

        # Dimensiones en metros y masas para cálculo dinámico de Lagrange
        self.d1_m = self.d1 / 100.0
        self.a1_m = self.a1 / 100.0
        self.a2_m = self.a2 / 100.0
        self.a3_m = self.a3 / 100.0
        self.m1 = 0.50      # kg: Masa estimada del conjunto base
        self.m2 = 0.40      # kg: Masa del brazo superior
        self.m3 = 0.30      # kg: Masa del antebrazo + efector final
        self.g = 9.81       # m/s^2: Aceleración de la gravedad terrestre

        # Rango admisible de movimiento angular seguro (grados - Reductor 31:1)
        # Límites del robot:
        # M1 (Base):   [-180.0°, +180.0°] (Giro libre completo 360°)
        # M2 (Hombro): [-90.0°,  +90.0°]
        # M3 (Codo):   [-90.0°,  +90.0°]
        self.limits = [
            (-180.0, 180.0), # J1 (Base - Yaw): Giro completo 360° [-180°, +180°]
            (-90.0, 90.0),   # J2 (Hombro - Pitch): [-90°, +90°]
            (-90.0, 90.0)    # J3 (Codo - Pitch): [-90°, +90°]
        ]
        self.nut_collision_margin_deg = 2.0  # Margen de alerta temprana de colisión

    def check_m2_m3_collision(self, q2_deg, q3_deg):
        """
        Verificación de límites articulares.
        Con rango completo [-90°, +90°] se permite total libertad de movimiento.
        Retorna:
          (colliding: bool, warning: bool, message: str)
        """
        lim_m2 = self.limits[1]
        lim_m3 = self.limits[2]
        if q3_deg < lim_m3[0] or q3_deg > lim_m3[1]:
            return True, True, f"M3 fuera de rango [{lim_m3[0]:.0f}°, {lim_m3[1]:.0f}°]"
        if q2_deg < lim_m2[0] or q2_deg > lim_m2[1]:
            return True, True, f"M2 fuera de rango [{lim_m2[0]:.0f}°, {lim_m2[1]:.0f}°]"
        return False, False, "OK"

    # --------------------------------------------------------------------------
    # 2.1 CINEMÁTICA DIRECTA (FK - FORWARD KINEMATICS) [MECANISMO PARALELOGRAMO]
    # --------------------------------------------------------------------------
    def forward_kinematics(self, q1_deg, q2_deg, q3_deg):
        """
        Calcula la posición espacial cartesiana (X, Y, Z) en centímetros
        del efector final a partir de los ángulos articulares dados en grados.

        MODELO CINEMÁTICO DE PARALELOGRAMO (DESACOPLADO):
          En el robot MK2 físico, el motor M3 está montado en la base giratoria
          y acciona el antebrazo mediante una varilla de empuje/tracción que forma
          un paralelogramo mecánico de 4 barras. Por tanto, la inclinación del
          antebrazo (a3) respecto al plano horizontal NO depende del ángulo del hombro (q2),
          sino directamente del ángulo de M3 (q3 ∈ [0.0°, 45.0°] hacia abajo):

          1. Radio horizontal proyectado en el plano radial del brazo:
             r = a1 + a2*cos(q2) + a3*cos(q3)
          2. Proyección sobre los ejes coordenados del mundo:
             X = r * cos(q1)
             Y = r * sin(q1)
             Z = d1 + a2*sin(q2) - a3*sin(q3)

        Retorna:
          (x, y, z) en cm con respecto al centro de la base en Z=0.
        """
        q1 = math.radians(q1_deg)
        q2 = math.radians(q2_deg)
        q3 = math.radians(q3_deg)

        r = self.a1 + self.a2 * math.cos(q2) + self.a3 * math.cos(q3)
        x = r * math.cos(q1)
        y = r * math.sin(q1)
        z = self.d1 + self.a2 * math.sin(q2) - self.a3 * math.sin(q3)
        return x, y, z

    # --------------------------------------------------------------------------
    # 2.2 CINEMÁTICA INVERSA (IK - INVERSE KINEMATICS) >>> [MECANISMO PARALELOGRAMO] <<<
    # --------------------------------------------------------------------------
    def inverse_kinematics(self, x, y, z, elbow_up=True):
        """
        ========================================================================
        >>> [CINEMÁTICA INVERSA EXACTA: PARALELOGRAMO DESACOPLADO MK2] <<<
        ========================================================================
        Dado un punto objetivo cartesiano (x, y, z) en cm, calcula la terna de
        ángulos articulares (q1, q2, q3) en grados necesarios para posicionar
        el efector final exactamente en ese punto.

        DIAGRAMA GEOMÉTRICO DEL PARALELOGRAMO:
                     Z ^
                       |              (X, Y, Z) Efector Final
                       |                 O (P3)
                       |                /
                       |         a3    /  (Antebrazo = 32.934 cm, ángulo -q3 a horizontal)
                       |              /
                       |             O (P2: Codo, altura Z2 = d1 + a2*sin(q2))
                       |            /
                       |     a2    /   (Brazo = 32.922 cm, ángulo q2 a horizontal)
                       |          /
                       |         O (P1: Hombro, altura Z1 = d1)
                       |        /
                       |  a1   /       (Offset horizontal base = 11.0 cm)
                       |      /
                       |=====O (P0: Base giratoria, ángulo q1)
                       |     |
                       |     | d1      (Altura vertical base = 10.0 cm)
                       |     |
                       +-----+--------------------------> R (Plano radial)
                      (0,0,0)

        DESARROLLO MATEMÁTICO:
        ------------------------------------------------------------------------
        1. Base: q1 = atan2(y, x)
        2. Proyección radial: dr = sqrt(x^2 + y^2) - a1, dz = z - d1, dist = sqrt(dr^2 + dz^2)
        3. Triángulo de eslabones:
           cos(Δθ) = (dist^2 - a2^2 - a3^2) / (2 * a2 * a3)
           cos(ψ)  = (dist^2 + a2^2 - a3^2) / (2 * a2 * dist)
           β       = atan2(dz, dr)
        4. Ramas cinemáticas:
           Rama 1 (Codo Arriba): θ2 = β + ψ, θ3 = θ2 - Δθ  => q2 = θ2, q3 = -θ3
           Rama 2 (Codo Abajo):  θ2 = β - ψ, θ3 = θ2 + Δθ  => q2 = θ2, q3 = -θ3
        5. Selección inteligente: Elige la solución cuyos ángulos respeten estrictamente
           los límites físicos del manipulador:
             M1: [-60.0°, +60.0°]
             M2: [-60.0°, +60.0°]
             M3: [  0.0°, +45.0°]
        ========================================================================
        """
        # --- PASO 1: Rotación de la Base (Giro en plano XY) ---
        q1_rad = math.atan2(y, x)
        q1_deg = math.degrees(q1_rad)

        # --- PASO 2: Desplazamiento relativo al eje del hombro ---
        r_xy = math.sqrt(x**2 + y**2)
        dr = r_xy - self.a1
        dz = z - self.d1

        dist_sq = dr**2 + dz**2
        dist = math.sqrt(dist_sq)

        # --- PASO 3: Verificación y adaptación suave de frontera de alcance ---
        max_reach = self.a2 + self.a3
        min_reach = abs(self.a2 - self.a3)

        if abs(dz) >= max_reach:
            return (0.0, 0.0, 0.0), False, (
                f"Altura Z={z:.1f}cm fuera de alcance vertical (Max={self.d1 + max_reach:.1f}cm, Min={self.d1 - max_reach:.1f}cm)"
            )

        # Proyección inteligente de envolvente radial:
        # Si el punto cartesiano (x, y) tiene un radio que excede el alcance horizontal máximo a esa altura z,
        # se acota dr al radio geométricamente alcanzable en esa dirección angular (q1).
        # Esto permite mover Y (izquierda/derecha) y X libremente sin bloqueos por frontera de alcance.
        max_dr = math.sqrt(max(0.0, max_reach**2 - dz**2))
        if dr > max_dr:
            dr = max_dr
            r_xy = dr + self.a1
            x = r_xy * math.cos(q1_rad)
            y = r_xy * math.sin(q1_rad)
            dist_sq = dr**2 + dz**2
            dist = math.sqrt(dist_sq)

        if dist < min_reach:
            return (0.0, 0.0, 0.0), False, (
                f"Punto en zona ciega interna (Dist={dist:.1f}cm < Min={min_reach:.1f}cm)"
            )

        # --- PASO 4: Solución trigonométrica desacoplada ---
        cos_dth = (dist_sq - self.a2**2 - self.a3**2) / (2.0 * self.a2 * self.a3)
        cos_dth = max(-1.0, min(1.0, cos_dth))
        dth = math.acos(cos_dth)

        cos_psi = (dist_sq + self.a2**2 - self.a3**2) / (2.0 * self.a2 * dist)
        cos_psi = max(-1.0, min(1.0, cos_psi))
        psi = math.acos(cos_psi)

        beta = math.atan2(dz, dr)

        # Solución 1 (Codo Arriba):
        t2_u = beta + psi
        t3_u = t2_u - dth
        sol_up = (q1_deg, math.degrees(t2_u), math.degrees(-t3_u))

        # Solución 2 (Codo Abajo):
        t2_d = beta - psi
        t3_d = t2_d + dth
        sol_down = (q1_deg, math.degrees(t2_d), math.degrees(-t3_d))

        # --- PASO 5: Filtrado de límites mecánicos exactos ---
        # M1: [-60.0°, +60.0°]
        # M2: [-60.0°, +60.0°]
        # M3: [  0.0°, +45.0°]
        lim_m1 = self.limits[0]
        lim_m2 = self.limits[1]
        lim_m3 = self.limits[2]

        def check_lims(sol):
            j1, j2, j3 = sol
            return (lim_m1[0] - 0.1 <= j1 <= lim_m1[1] + 0.1 and
                    lim_m2[0] - 0.1 <= j2 <= lim_m2[1] + 0.1 and
                    lim_m3[0] - 0.1 <= j3 <= lim_m3[1] + 0.1)

        v_up = check_lims(sol_up)
        v_down = check_lims(sol_down)

        if elbow_up and v_up:
            best = sol_up
        elif not elbow_up and v_down:
            best = sol_down
        elif v_up:
            best = sol_up
        elif v_down:
            best = sol_down
        else:
            best = sol_up if elbow_up else sol_down
            j1, j2, j3 = best
            return (j1, j2, j3), False, (
                f"Límites excedidos: M1={j1:.1f}°, M2={j2:.1f}°, M3={j3:.1f}° "
                f"(Límites M1:[{lim_m1[0]:.0f},{lim_m1[1]:.0f}] M2:[{lim_m2[0]:.0f},{lim_m2[1]:.0f}] M3:[{lim_m3[0]:.0f},{lim_m3[1]:.0f}])"
            )

        return best, True, "OK"


# ==============================================================================
# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ TEMA 3: DINÁMICA DE LAGRANGE-EULER EN TIEMPO REAL                          ║
# ╚════════════════════════════════════════════════════════════════════════════╝
# ==============================================================================
    def calculate_lagrange_torques(self, q1_deg, q2_deg, q3_deg):
        """
        Calcula los torques estáticos de gravedad (N*m) según el modelo de
        Lagrange-Euler para manipulador con mecanismo de paralelogramo desacoplado:

        Formulación analítica del vector de gravedad G(q):
          - tau1 = 0.0 N*m (Eje colineal con gravedad).
          - tau2 = (m2/2 + m3) * g * a2_m * cos(q2)
          - tau3 = (m3/2) * g * a3_m * cos(q3)

        Retorna:
          (tau1, tau2, tau3) en Newtons-metro.
        """
        q2 = math.radians(q2_deg)
        q3 = math.radians(q3_deg)

        tau1 = 0.0
        tau2 = (self.m2 * 0.5 + self.m3) * self.g * self.a2_m * math.cos(q2)
        tau3 = self.m3 * 0.5 * self.g * self.a3_m * math.cos(q3)

        return tau1, tau2, tau3


# ==============================================================================
# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ CLASE PRINCIPAL DEL CONTROLADOR Y APLICACIÓN DE INTERFAZ GRÁFICA           ║
# ╚════════════════════════════════════════════════════════════════════════════╝
# ==============================================================================
class Robot3GDLGatoVisionController(tk.Tk):
    """
    Controlador general del sistema: integra comunicación serie, visión
    artificial, inteligencia artificial, modelo cinemático y GUI interactiva.
    """

    def __init__(self):
        super().__init__()

        # Configuración básica de la ventana principal adaptada a la resolución de pantalla
        self.title("Controlador Cinemático & Jugador de Gato Robot MK2 (3 GDL) | UPA")
        screen_w = self.winfo_screenwidth()
        screen_h = self.winfo_screenheight()
        init_w = min(1400, max(1080, screen_w - 40))
        init_h = min(900, max(640, screen_h - 60))
        self.geometry(f"{init_w}x{init_h}")
        self.minsize(980, 560)
        try:
            self.state('zoomed')
        except Exception:
            pass
        self.configure(bg=COLOR_BG_MAIN)

        # Instancia del modelo cinemático analítico
        self.kin = RobotMK2Kinematics()

        # Comunicación Serial con ESP32 y control de tasa en tiempo real
        self.serial_port = None
        self.running = False
        self.reader_thread = None
        self.pkt_seq = 200
        self._last_slider_tx_time = 0.0
        self._slider_tx_pending = False
        self.lbl_slider_min = []
        self.lbl_slider_max = []
        self.preset_btn_list = []

        # Estado cinemático del manipulador
        self.current_angles = [0.0, 0.0, 0.0]  # [J1, J2, J3] actuales (reales o simulados)
        self.target_angles  = [0.0, 0.0, 0.0]  # [J1, J2, J3] consignas objetivo
        self.speeds_hz      = [0, 0, 0]        # Frecuencias de micropasos por articulación
        self.gripper_state  = False            # False: Desactivado / True: Sujetando ficha

        # Estado de conexión y salud de los 3 encoders AS5600
        self.encoder_status = [
            {"connected": False, "magnet": False, "raw": 0, "msg": "Esperando..."}
            for _ in range(3)
        ]
        self.encoder_badge_labels = []
        self.encoder_raw_labels = []

        # Sistema Gestor de Posición Cero y Home de Trabajo Natural:
        # Home de reposo óptimo en el centro del espacio de trabajo del MK2:
        # X=55.0cm, Y=0.0cm, Z=10.0cm (articulaciones hombro ~48.1°, codo ~48.1°)
        self.HOME_X = 55.0
        self.HOME_Y = 0.0
        self.HOME_Z = 10.0
        self.zero_modal_shown = True
        self.zero_calibrated = True
        self.motors_free_state = False

        # Sistema de Coordenadas: Relativo a Cero Home (0.0, 0.0, 0.0)
        # Inicia exactamente en (0.0, 0.0, 0.0) referenciado al actuador físico
        self.coord_mode_relative = True
        self.cart_x = 0.0
        self.cart_y = 0.0
        self.cart_z = 0.0
        self.last_ik_error_pos = 0.0

        # Disposición espacial de las 9 casillas del tablero de Gato y Dispensador
        # Calculadas automáticamente en la zona segura libre de colisión de tuerca M2-M3
        self.board_center_x = 48.0
        self.recalculate_board_cells()

        # Estado del Juego de Gato (0=Vacío, 1=Humano 'X', 2=Robot 'O')
        self.board = [0] * 9
        self.game_active = True
        self.turn = "HUMANO"
        self.human_score = 0
        self.robot_score = 0
        self.draws_score = 0

        # Visión Artificial por WebCam (OpenCV)
        self.cap = None
        self.camera_running = False
        self.camera_thread = None
        self.current_frame = None
        self.last_detected_board = [0] * 9
        self.detection_stability_count = 0

        # Variables de la Cámara 3D Interactiva & Animación
        # Ajustado a la perspectiva física real del usuario en su mesa (laptop a la izq, robot a la der)
        self.cam_yaw = math.radians(-70.0)     # Ángulo horizontal POV usuario (-70°)
        self.cam_pitch = math.radians(22.0)    # Inclinación visual natural (22°)
        self.cam_zoom = 1.05                   # Zoom óptimo
        self.cam_last_x = 0
        self.cam_last_y = 0
        self.is_robot_busy = False             # Bloqueo durante trayectoria de pick & place
        self.ui_queue = queue.Queue()          # Cola segura para sincronizar eventos con Tkinter

        # Secuencia automatizada de Casillas 1 a 9
        self.tour_active = False
        self.active_tour_cell = None
        self.cell_buttons = []

        # Configuración persistente del robot y perfiles guardados
        self.config_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_config.json")
        self.robot_config = self.load_robot_config_file()
        self.active_profile_name = self.robot_config.get("current_profile", "Robot MK2 Gato 31:1 (Predeterminado)")

        # Construcción de la interfaz y arranque del bucle
        self.setup_ui()
        self.refresh_ports()
        self.apply_robot_profile(self.active_profile_name, silent=True)
        self.adapt_cells_to_current_limits(show_msg=False)
        self.animate()
        self.after(200, self.auto_connect_if_usb)
        # Lanzamiento del Asistente de Inicio solo si no está calibrado
        if not self.zero_calibrated:
            self.after(350, self.show_startup_robot_wizard)

    def auto_connect_if_usb(self):
        """Conecta automáticamente si detecta un puerto USB nativo del ESP32-S3."""
        if not serial or self.running:
            return
        for p in serial.tools.list_ports.comports():
            hwid = (p.hwid or "").upper()
            desc = (p.description or "").upper()
            if "303A" in hwid or ("USB" in desc and "BLUETOOTH" not in desc):
                self.combo_ports.set(p.device)
                self.toggle_connection()
                break


# ==============================================================================
# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ TEMA 4: PROTOCOLO DE COMUNICACIÓN SERIAL CON ESP32                         ║
# ╚════════════════════════════════════════════════════════════════════════════╝
# ==============================================================================

    def calculate_checksum(self, text):
        """Calcula el Checksum XOR de 8 bits sobre la cadena de payload."""
        cs = 0
        for ch in text:
            cs ^= ord(ch)
        return cs

    def send_angles_command(self, q1, q2, q3):
        """Transmite una consigna articular al firmware del ESP32 con protocolo $PKT."""
        if getattr(self, 'motors_free_state', False):
            # En modo MOTORES LIBRES, NUNCA transmitir consignas que reactiven torque o causen tics
            return
        # REGLA DE OPERACIÓN (LÍMITES CONFIGURADOS):
        l1_min, l1_max = self.kin.limits[0]
        l2_min, l2_max = self.kin.limits[1]
        l3_min, l3_max = self.kin.limits[2]
        q1 = max(l1_min, min(l1_max, float(q1)))
        q2 = max(l2_min, min(l2_max, float(q2)))
        q3 = max(l3_min, min(l3_max, float(q3)))
        cmd_q1 = float(q1)
        cmd_q2 = float(q2)
        cmd_q3 = float(q3)
        self.pkt_seq = (self.pkt_seq + 1) % 10000
        payload = f"PKT,{self.pkt_seq},0,ANGLES,{cmd_q1:.2f},{cmd_q2:.2f},{cmd_q3:.2f}"
        cs = self.calculate_checksum(payload)
        self.send_serial_raw(f"${payload}*{cs:02X}\r\n")

    def cmd_stop(self):
        """Envía comando de Paro de Emergencia inmediato al robot."""
        self.pkt_seq = (self.pkt_seq + 1) % 10000
        payload = f"PKT,{self.pkt_seq},0,STOP,0.0,0.0,0.0"
        cs = self.calculate_checksum(payload)
        self.send_serial_raw(f"${payload}*{cs:02X}\r\n")

    def toggle_gripper(self):
        """Conmuta el estado del Gripper / Electroimán (ON/OFF) y envía comando al ESP32."""
        self.gripper_state = not self.gripper_state
        st = 1.0 if self.gripper_state else 0.0
        self.pkt_seq = (self.pkt_seq + 1) % 10000
        payload = f"PKT,{self.pkt_seq},0,GRIPPER,{st:.2f},0.0,0.0"
        cs = self.calculate_checksum(payload)
        self.send_serial_raw(f"${payload}*{cs:02X}\r\n")
        self.update_gripper_button()

    def send_serial_raw(self, text):
        """Escribe una cadena de texto en bytes por el puerto serial de manera protegida."""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(text.encode('utf-8'))
            except Exception as e:
                print(f"Error escribiendo al puerto serial: {e}")
                self.handle_serial_disconnect()

    def refresh_ports(self):
        """Escanea los puertos serie COM disponibles priorizando el puerto USB del ESP32-S3 (COM12)."""
        if not serial:
            return
        comports = list(serial.tools.list_ports.comports())
        if not comports:
            self.combo_ports['values'] = []
            return

        usb_ports = []
        other_ports = []
        preferred = None

        for p in comports:
            desc = (p.description or "").upper()
            hwid = (p.hwid or "").upper()
            is_bt = ("BLUETOOTH" in desc or "BTHENUM" in hwid or "VÍNCULO" in desc or "VINCULO" in desc)
            is_esp32 = ("303A" in hwid or "ESP32" in desc or "JTAG" in desc)

            if is_esp32:
                preferred = p.device
                usb_ports.insert(0, p.device)
            elif not is_bt and ("USB" in desc or "UART" in desc or "CH340" in desc or "CP210" in desc):
                usb_ports.append(p.device)
            else:
                other_ports.append(p.device)

        final_ports = usb_ports + other_ports
        if not preferred and usb_ports:
            preferred = usb_ports[0]
        elif not preferred and final_ports:
            preferred = final_ports[0]

        self.combo_ports['values'] = final_ports
        if preferred:
            self.combo_ports.set(preferred)

    def toggle_connection(self):
        """Abre o cierra la conexión serial con el microcontrolador ESP32."""
        if not serial:
            return
        if not self.running:
            p = self.combo_ports.get()
            if not p:
                return
            try:
                self.serial_port = serial.Serial(
                    port=p,
                    baudrate=115200,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.1
                )
                self.running = True
                self.btn_connect.config(text="Desconectar", bg=COLOR_RED_DANGER)
                self.lbl_status.config(text="● CONECTADO", fg=COLOR_GREEN_SUCCESS)
                self.reader_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
                self.reader_thread.start()
                self.after(350, self.query_esp32_limits)
                # Si el ESP32 se desconectó y se vuelve a conectar o el cero ya estaba calibrado:
                # Regresar automáticamente a la posición cero guardada (0.0°, 0.0°, 0.0°)
                if self.zero_calibrated:
                    self.after(600, self.cmd_load_game)
                elif not self.zero_modal_shown:
                    self.after(650, self.show_zero_calibration_modal)
            except Exception as e:
                messagebox.showerror("Error de Conexión", f"No se pudo conectar al puerto {p}:\n{e}")
        else:
            self.handle_serial_disconnect()

    def query_esp32_limits(self):
        """Consulta los límites angulares configurados en la memoria NVS del ESP32."""
        if self.running and self.serial_port and self.serial_port.is_open:
            self.pkt_seq = (self.pkt_seq + 1) % 10000
            payload = f"PKT,{self.pkt_seq},0,GET_LIMITS,0.0,0.0,0.0"
            cs = self.calculate_checksum(payload)
            self.send_serial_raw(f"${payload}*{cs:02X}\r\n")

    def handle_serial_disconnect(self):
        """Maneja el cierre seguro de la conexión serie y actualiza la interfaz gráfica."""
        self.running = False
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
        except Exception:
            pass
        self.btn_connect.config(text="Conectar ESP32", bg="#0284c7")
        self.lbl_status.config(text="● DESCONECTADO", fg="#64748b")

    def serial_read_loop(self):
        """
        Hilo en segundo plano para recepción continua de telemetría desde el ESP32:
        - Decodificación de tramas robustas $TEL3 con Checksum XOR.
        - Compatibilidad de respaldo con texto ASCII plano por expresiones regulares.
        """
        re_j1 = re.compile(r"J1\(Base\):\s*([-\d.]+)°", re.IGNORECASE)
        re_j2 = re.compile(r"J2\(Hombro\):\s*([-\d.]+)°", re.IGNORECASE)
        re_j3 = re.compile(r"J3\(Codo\):\s*([-\d.]+)°", re.IGNORECASE)

        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # 1. Trama enmarcada $TEL3 con Checksum XOR
                if line.startswith('$') and '*' in line:
                    star_idx = line.rfind('*')
                    payload = line[1:star_idx]
                    cs_str = line[star_idx+1:]
                    try:
                        received_cs = int(cs_str, 16)
                        if received_cs == 0 or self.calculate_checksum(payload) == received_cs:
                            parts = payload.split(',')
                            if parts[0] == "TEL3" and len(parts) >= 10:
                                self.current_angles[0] = float(parts[2])
                                self.speeds_hz[0]      = int(parts[3])
                                self.current_angles[1] = float(parts[5])
                                self.speeds_hz[1]      = int(parts[6])
                                self.current_angles[2] = float(parts[8])
                                self.speeds_hz[2]      = int(parts[9])
                                if len(parts) > 10:
                                    self.gripper_state = (parts[10] == '1')
                                if self.motors_free_state:
                                    self.target_angles[0] = self.current_angles[0]
                                    self.target_angles[1] = self.current_angles[1]
                                    self.target_angles[2] = self.current_angles[2]
                                    def _sync_sliders_free():
                                        if hasattr(self, 'sliders_ax') and len(self.sliders_ax) >= 3:
                                            self._suppress_slider_tx = True
                                            try:
                                                for k in range(3):
                                                    self.sliders_ax[k].set(self.current_angles[k])
                                            finally:
                                                self._suppress_slider_tx = False
                                    self.dispatch_ui(_sync_sliders_free)
                                continue

                            # Protocolo Anti-Choque: Alarma de Atasco por Encoder AS5600 inmóvil
                            if parts[0] == "ALARM" and len(parts) >= 3:
                                if parts[1] == "STALL":
                                    m_str = parts[2].replace("M", "")
                                    m_id = int(m_str) if m_str.isdigit() else 0
                                    t_deg = float(parts[3]) if len(parts) > 3 else 0.0
                                    c_deg = float(parts[4]) if len(parts) > 4 else 0.0
                                    self.dispatch_ui(lambda mi=m_id, td=t_deg, cd=c_deg: self.handle_stall_collision_alarm(mi, td, cd))
                                continue

                            # Trama de telemetría de Encoders: $ENC,1,<valid>,<md>,<raw>,2,...
                            if parts[0] == "ENC" and len(parts) >= 13:
                                for m_idx in range(3):
                                    offset = 1 + m_idx * 4
                                    self.encoder_status[m_idx]["connected"] = (parts[offset + 1] == "1")
                                    self.encoder_status[m_idx]["magnet"]    = (parts[offset + 2] == "1")
                                    self.encoder_status[m_idx]["raw"]       = int(parts[offset + 3])
                                continue

                            # Trama individual de Encoder: $TEL,<ID>,<CALIB_DEG>,<RAW>,<SPEED_HZ>,<ERROR>,<STATE>,<AGC>,<MAG_STAT>
                            if parts[0] == "TEL" and len(parts) >= 6:
                                m_idx = int(parts[1]) - 1
                                if 0 <= m_idx < 3:
                                    raw_deg = float(parts[2])
                                    self.current_angles[m_idx] = raw_deg
                                    self.encoder_status[m_idx]["raw"] = int(parts[3])
                                    self.speeds_hz[m_idx] = int(parts[4])
                                    self.encoder_status[m_idx]["connected"] = True
                                    if len(parts) > 8:
                                        self.encoder_status[m_idx]["magnet"] = (parts[8] == "1")
                                continue

                            # Respuesta de diagnóstico de Encoders: $ACK,<seq>,ENC_STATUS,...
                            if len(parts) >= 3 and parts[2] == "ENC_STATUS":
                                for seg in parts[3:]:
                                    if ":" in seg:
                                        sub = seg.split(":")
                                        if len(sub) >= 4:
                                            m_idx = int(sub[0]) - 1
                                            if 0 <= m_idx < 3:
                                                self.encoder_status[m_idx]["connected"] = (sub[1] == "1")
                                                self.encoder_status[m_idx]["magnet"]    = (sub[2] == "1")
                                                self.encoder_status[m_idx]["raw"]       = int(sub[3])
                                continue

                            # Sincronización de Límites de Seguridad: $ACK,<seq>,LIMITS,1:<min>:<max>,...
                            if len(parts) >= 3 and parts[2] == "LIMITS":
                                for seg in parts[3:]:
                                    if ":" in seg:
                                        sub = seg.split(":")
                                        if len(sub) >= 3:
                                            m_idx = int(sub[0]) - 1
                                            if 0 <= m_idx < 3:
                                                lmin_val = float(sub[1])
                                                lmax_val = float(sub[2])
                                                # Asegurar límites amplios para no restringir el movimiento
                                                if m_idx == 0:
                                                    lmin_val = -180.0
                                                    lmax_val = 180.0
                                                elif m_idx == 1:
                                                    lmin_val = -90.0
                                                    lmax_val = 90.0
                                                elif m_idx == 2:
                                                    lmin_val = -90.0
                                                    lmax_val = 90.0
                                                self.kin.limits[m_idx] = (lmin_val, lmax_val)
                                                self.gauge_ranges[m_idx] = (lmin_val, lmax_val)
                                                if m_idx < len(self.sliders_ax):
                                                    self.dispatch_ui(lambda a=m_idx, mn=lmin_val, mx=lmax_val: self.sliders_ax[a].config(from_=mn, to=mx))
                                                if m_idx < len(self.ent_lim_min):
                                                    self.dispatch_ui(lambda a=m_idx, mn=lmin_val: (self.ent_lim_min[a].delete(0, tk.END), self.ent_lim_min[a].insert(0, f"{mn:.1f}")))
                                                if m_idx < len(self.ent_lim_max):
                                                    self.dispatch_ui(lambda a=m_idx, mx=lmax_val: (self.ent_lim_max[a].delete(0, tk.END), self.ent_lim_max[a].insert(0, f"{mx:.1f}")))
                                continue

                            # Respuestas de Calibración de Posición Cero:
                            if len(parts) >= 3 and ("SAVE_GAME" in parts[2] or "LOAD_GAME" in parts[2] or "MOTORS_" in parts[2]):
                                if "SAVE_GAME" in parts[2] or "LOAD_GAME" in parts[2]:
                                    self.zero_calibrated = True
                                    self.motors_free_state = False
                                elif "MOTORS_FREE" in parts[2]:
                                    self.motors_free_state = True
                                elif "MOTORS_LOCK" in parts[2]:
                                    self.motors_free_state = False
                                self.update_zero_bar_ui()
                                continue
                    except ValueError:
                        pass

                # 2. Respaldo por texto ASCII plano y Diagnóstico de Encoders
                if "ALERTA ANTI-CHOQUE" in line or ("ALARM" in line and "STALL" in line):
                    m_match = re.search(r"M([123])", line)
                    m_id = int(m_match.group(1)) if m_match else 0
                    self.dispatch_ui(lambda mi=m_id: self.handle_stall_collision_alarm(mi, 0.0, 0.0))

                re_enc_log = re.compile(r"M([123])\([^)]+\):.*?Raw:\s*(\d+).*?MD:([01])", re.IGNORECASE)
                m_enc = re_enc_log.search(line)
                if m_enc:
                    m_idx = int(m_enc.group(1)) - 1
                    raw_val = int(m_enc.group(2))
                    md_val = (m_enc.group(3) == "1")
                    self.encoder_status[m_idx]["connected"] = True
                    self.encoder_status[m_idx]["magnet"] = md_val
                    self.encoder_status[m_idx]["raw"] = raw_val

                re_boot_ok = re.compile(r"Encoder Motor ([123]).*?OK", re.IGNORECASE)
                mb_ok = re_boot_ok.search(line)
                if mb_ok:
                    self.encoder_status[int(mb_ok.group(1)) - 1]["connected"] = True

                re_boot_fail = re.compile(r"Encoder Motor ([123]).*?NO responde", re.IGNORECASE)
                mb_fail = re_boot_fail.search(line)
                if mb_fail:
                    self.encoder_status[int(mb_fail.group(1)) - 1]["connected"] = False
                m1 = re_j1.search(line)
                if m1:
                    self.current_angles[0] = float(m1.group(1))
                m2 = re_j2.search(line)
                if m2:
                    self.current_angles[1] = float(m2.group(1))
                m3 = re_j3.search(line)
                if m3:
                    self.current_angles[2] = float(m3.group(1))

            except (serial.SerialException, OSError):
                self.handle_serial_disconnect()
                break
            except Exception:
                time.sleep(0.01)


# ==============================================================================
# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ TEMA 5: VISIÓN ARTIFICIAL Y PROCESAMIENTO DE IMÁGENES (OPENCV)             ║
# ╚════════════════════════════════════════════════════════════════════════════╝
# ==============================================================================

    def toggle_camera(self):
        """Inicia o detiene la captura y análisis de video de la WebCam."""
        if not cv2:
            messagebox.showerror("Error", "OpenCV no está disponible en este entorno.")
            return

        if not self.camera_running:
            idx = int(self.combo_cam.get().split()[-1])
            self.cap = cv2.VideoCapture(
                idx, cv2.CAP_DSHOW if sys.platform.startswith('win') else cv2.CAP_ANY
            )
            if not self.cap or not self.cap.isOpened():
                messagebox.showerror("Error de Cámara", f"No se pudo abrir la Cámara {idx}.")
                return

            self.camera_running = True
            self.btn_camera.config(text="Detener VIDEO", bg=COLOR_RED_DANGER)
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
        else:
            self.camera_running = False
            if self.cap:
                self.cap.release()
            self.btn_camera.config(text="Iniciar Video", bg="#059669")
            self.lbl_vision_info.config(text="Visión: WebCam detenida.")

    def camera_loop(self):
        """
        Hilo en segundo plano para captura de fotogramas, segmentación HSV,
        detección de regiones de interés (tablero y dispensadores) y
        reconocimiento automático de tiradas del jugador humano.
        """
        while self.camera_running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.03)
                continue

            frame = cv2.flip(frame, 1)  # Efecto espejo para mayor ergonomía visual
            h, w, _ = frame.shape

            # Dimensiones del tablero central 3x3 y dispensadores
            size = min(int(w * 0.58), h - 60)
            rx1 = int((w - size) / 2)
            ry1 = int((h - size) / 2)
            rx2 = rx1 + size
            ry2 = ry1 + size

            # Borde del tablero central
            cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (0, 245, 212), 2)
            cv2.putText(frame, "TABLERO 3x3", (rx1 + 8, ry1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 245, 212), 1)

            # Dispensador izquierdo: Fichas 'O' (Robot / Círculos Azules)
            dw = max(55, min(int(rx1 - 25), 85))
            dh = int(size * 0.70)
            dy1 = ry1 + (size - dh) // 2
            dy2 = dy1 + dh
            dl_x1 = max(10, rx1 - dw - 12)
            dl_x2 = dl_x1 + dw

            cv2.rectangle(frame, (dl_x1, dy1), (dl_x2, dy2), (255, 200, 0), 2)
            cv2.putText(frame, "DISP. 'O'", (dl_x1 + 4, dy1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1)
            cv2.circle(frame, (dl_x1 + dw // 2, dy1 + dh // 2), int(dw * 0.28), (255, 200, 0), 2)

            # Dispensador derecho: Fichas 'X' (Humano / Cruces Rojas)
            dr_x1 = rx2 + 12
            dr_x2 = min(w - 10, dr_x1 + dw)

            cv2.rectangle(frame, (dr_x1, dy1), (dr_x2, dy2), (0, 0, 255), 2)
            cv2.putText(frame, "DISP. 'X'", (dr_x1 + 4, dy1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1)
            dx_c = dr_x1 + (dr_x2 - dr_x1) // 2
            dy_c = dy1 + dh // 2
            arm_s = int(dw * 0.22)
            cv2.line(frame, (dx_c - arm_s, dy_c - arm_s), (dx_c + arm_s, dy_c + arm_s), (0, 0, 255), 2)
            cv2.line(frame, (dx_c - arm_s, dy_c + arm_s), (dx_c + arm_s, dy_c - arm_s), (0, 0, 255), 2)

            cell_size = size // 3
            detected_cells = [0] * 9

            for i in range(3):
                for j in range(3):
                    idx = i * 3 + j
                    cx1 = rx1 + j * cell_size
                    cy1 = ry1 + i * cell_size
                    cx2 = cx1 + cell_size
                    cy2 = cy1 + cell_size

                    cv2.rectangle(frame, (cx1, cy1), (cx2, cy2), (60, 60, 60), 1)
                    roi = frame[cy1 + 8:cy2 - 8, cx1 + 8:cx2 - 8]
                    if roi.size > 0:
                        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                        # Segmentación de Rojo ('X' Humano)
                        m_red1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
                        m_red2 = cv2.inRange(hsv, np.array([170, 70, 50]), np.array([180, 255, 255]))
                        red_px = cv2.countNonZero(m_red1 | m_red2)
                        # Segmentación de Azul/Cyan ('O' Robot)
                        m_blue = cv2.inRange(hsv, np.array([90, 70, 50]), np.array([130, 255, 255]))
                        blue_px = cv2.countNonZero(m_blue)

                        if red_px > 400:
                            detected_cells[idx] = 1
                            cv2.putText(frame, "X", (cx1 + cell_size//3, cy1 + 2*cell_size//3),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
                        elif blue_px > 400:
                            detected_cells[idx] = 2
                            cv2.putText(frame, "O", (cx1 + cell_size//3, cy1 + 2*cell_size//3),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 200, 0), 3)
                        else:
                            cv2.putText(frame, f"{idx+1}", (cx1 + 8, cy1 + 18),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)

            # Filtro de estabilidad temporal para detección automática de jugada humana
            if self.chk_auto_vision_var.get() and self.game_active and self.turn == "HUMANO":
                if detected_cells == self.last_detected_board:
                    self.detection_stability_count += 1
                    if self.detection_stability_count == 7:
                        for k in range(9):
                            if detected_cells[k] == 1 and self.board[k] == 0:
                                self.after(0, lambda pos=k: self.on_cell_clicked(pos))
                                break
                else:
                    self.last_detected_board = list(detected_cells)
                    self.detection_stability_count = 0

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.current_frame = rgb
            time.sleep(0.03)

    def update_video_canvas(self):
        """Actualiza el widget Canvas de Tkinter con el fotograma RGB actual."""
        if self.camera_running and self.current_frame is not None:
            try:
                cw = self.canvas_cam.winfo_width()
                ch = self.canvas_cam.winfo_height()
                if cw > 50 and ch > 50:
                    img = Image.fromarray(self.current_frame)
                    img = img.resize((cw, ch), Image.Resampling.BILINEAR)
                    self.photo_img = ImageTk.PhotoImage(image=img)
                    self.canvas_cam.create_image(0, 0, image=self.photo_img, anchor=tk.NW)
            except Exception:
                pass


# ==============================================================================
# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ TEMA 6: INTELIGENCIA ARTIFICIAL Y LÓGICA DE JUEGO (MINIMAX)                 ║
# ╚════════════════════════════════════════════════════════════════════════════╝
# ==============================================================================

    def on_cell_clicked(self, idx):
        """Maneja el clic de una casilla por parte del jugador humano ('X')."""
        if not self.game_active or self.board[idx] != 0 or self.turn != "HUMANO" or self.is_robot_busy:
            return

        self.board[idx] = 1
        self.cell_buttons[idx].config(text="X", fg=COLOR_RED_DANGER, bg="#1e293b")

        if self.check_winner(1):
            self.game_active = False
            self.human_score += 1
            self.update_scores("¡Ganaste la partida!")
            return

        if 0 not in self.board:
            self.game_active = False
            self.draws_score += 1
            self.update_scores("¡Empate técnico!")
            return

        self.turn = "ROBOT"
        self.lbl_status.config(text="● TURNO DEL ROBOT...", fg=COLOR_CYAN_ACCENT)
        self.after(250, self.robot_play)

    def trigger_robot_play(self):
        """Fuerza manualmente la ejecución del turno de la IA del robot."""
        if not self.is_robot_busy and self.game_active:
            self.turn = "ROBOT"
            self.lbl_status.config(text="● TURNO DEL ROBOT...", fg=COLOR_CYAN_ACCENT)
            self.after(50, self.robot_play)

    def robot_play(self):
        """Calcula la mejor jugada con Minimax e inicia la trayectoria Pick & Place en un hilo."""
        if not self.game_active or self.is_robot_busy:
            return
        move = self.minimax_best_move()
        if move is not None:
            # Se ejecuta en un hilo secundario para mantener la interfaz 100% responsiva
            threading.Thread(target=self.run_robot_animation_turn, args=(move,), daemon=True).start()

    def minimax_best_move(self):
        """
        Algoritmo Minimax optimizado para estrategia invencible del Gato:
          1. Victoria inmediata: Si puede ganar en este turno, tira allí.
          2. Bloqueo defensivo: Si el humano puede ganar en el siguiente turno, bloquea.
          3. Casillas estratégicas de alta probabilidad:
             - Centro (Casilla 5)
             - Esquinas (Casillas 1, 3, 7, 9)
             - Laterales restantes
        """
        # 1. Ganar si existe jugada ganadora
        for i in range(9):
            if self.board[i] == 0:
                self.board[i] = 2
                if self.check_winner(2):
                    self.board[i] = 0
                    return i
                self.board[i] = 0

        # 2. Bloquear victoria humana
        for i in range(9):
            if self.board[i] == 0:
                self.board[i] = 1
                if self.check_winner(1):
                    self.board[i] = 0
                    return i
                self.board[i] = 0

        # 3. Posiciones estratégicas prioritarias
        if self.board[4] == 0:
            return 4
        for i in [0, 2, 6, 8]:
            if self.board[i] == 0:
                return i
        for i in range(9):
            if self.board[i] == 0:
                return i
        return None

    def check_winner(self, p):
        """Evalúa si el jugador 'p' (1=Humano, 2=Robot) completó alguna de las 8 líneas."""
        lines = [
            (0, 1, 2), (3, 4, 5), (6, 7, 8),  # 3 Filas horizontales
            (0, 3, 6), (1, 4, 7), (2, 5, 8),  # 3 Columnas verticales
            (0, 4, 8), (2, 4, 6)              # 2 Diagonales principales
        ]
        return any(self.board[a] == p and self.board[b] == p and self.board[c] == p for a, b, c in lines)

    def reset_game(self):
        """Limpia el tablero, reactiva el turno humano y regresa el robot a posición HOME."""
        if self.is_robot_busy:
            return
        self.board = [0] * 9
        self.game_active = True
        self.turn = "HUMANO"
        for b in self.cell_buttons:
            b.config(text=" ", bg=COLOR_BG_CARD, fg="#ffffff")
        self.lbl_status.config(text="● TU TURNO (X)", fg=COLOR_GREEN_SUCCESS)
        self.cmd_home()

    def update_scores(self, msg):
        """Actualiza el marcador en pantalla y muestra un cuadro de diálogo informativo."""
        self.lbl_scores.config(
            text=f"Humano (X): {self.human_score}   |   Robot (O): {self.robot_score}   |   Empates: {self.draws_score}"
        )
        messagebox.showinfo("Fin de Partida", msg)


# ==============================================================================
# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ TEMA 7: CINEMÁTICA OPERACIONAL Y SECUENCIA PICK & PLACE DEL ROBOT          ║
# ╚════════════════════════════════════════════════════════════════════════════╝
# ==============================================================================

    def dispatch_ui(self, func, *args, **kwargs):
        """Encola una función para ejecución segura en el hilo principal de Tkinter."""
        self.ui_queue.put((func, args, kwargs))

    def set_status_text(self, text, fg=COLOR_CYAN_ACCENT):
        """Actualiza el mensaje de estado de la barra superior de manera thread-safe."""
        self.dispatch_ui(lambda: self.lbl_status.config(text=text, fg=fg))

    def update_cartesian_entries(self, x_abs, y_abs, z_abs):
        """Actualiza los campos de texto cartesianos respetando el modo relativo a Home (0,0,0) o absoluto."""
        self.cart_x = x_abs
        self.cart_y = y_abs
        self.cart_z = z_abs
        if not (hasattr(self, 'ent_x') and hasattr(self, 'ent_y') and hasattr(self, 'ent_z')):
            return
        if getattr(self, 'coord_mode_relative', True):
            disp_x = x_abs - self.HOME_X
            disp_y = y_abs - self.HOME_Y
            disp_z = z_abs - self.HOME_Z
        else:
            disp_x, disp_y, disp_z = x_abs, y_abs, z_abs

        try:
            self.ent_x.delete(0, tk.END)
            self.ent_x.insert(0, f"{disp_x:.1f}")
            self.ent_y.delete(0, tk.END)
            self.ent_y.insert(0, f"{disp_y:.1f}")
            self.ent_z.delete(0, tk.END)
            self.ent_z.insert(0, f"{disp_z:.1f}")
        except Exception:
            pass

    def set_coord_mode(self, relative=True):
        """Alterna entre modo relativo a Cero Home (0,0,0) y absoluto con respecto a la base."""
        self.coord_mode_relative = relative
        xfk, yfk, zfk = self.kin.forward_kinematics(*self.target_angles)
        self.update_cartesian_entries(xfk, yfk, zfk)
        if hasattr(self, 'lbl_ik_status'):
            if relative:
                rx = xfk - self.HOME_X
                ry = yfk - self.HOME_Y
                rz = zfk - self.HOME_Z
                self.lbl_ik_status.config(
                    text=f"Modo: Relativo Cero (0,0,0) | Pos: ({rx:+.1f}, {ry:+.1f}, {rz:+.1f}) cm (Mesa Z=-39.1)",
                    fg=COLOR_GREEN_SUCCESS
                )
            else:
                self.lbl_ik_status.config(
                    text=f"Modo: Absoluto Base | Pos: ({xfk:.1f}, {yfk:.1f}, {zfk:.1f}) cm (Mesa Z=-29.1)",
                    fg=COLOR_GREEN_SUCCESS
                )

    # --------------------------------------------------------------------------
    # ACCIONAMIENTO DE CINEMÁTICA INVERSA DESDE ENTRADAS CARTESIANAS (X, Y, Z)
    # --------------------------------------------------------------------------
    def cmd_solve_ik_and_move(self):
        """
        Lee las coordenadas X, Y, Z ingresadas por el usuario en la interfaz,
        las valida y solicita el movimiento mediante Cinemática Inversa.
        Soporta modo relativo donde Home sobre la caja es (0, 0, 0) y modo absoluto.
        """
        try:
            ux = float(self.ent_x.get())
            uy = float(self.ent_y.get())
            uz = float(self.ent_z.get())
        except ValueError:
            if hasattr(self, 'lbl_ik_status'):
                self.lbl_ik_status.config(text="Estado IK: [ERROR] Coordenadas X, Y, Z numéricas inválidas", fg=COLOR_RED_DANGER)
            return

        is_rel = getattr(self, 'coord_mode_relative', True)
        if is_rel:
            x_abs = self.HOME_X + ux
            y_abs = self.HOME_Y + uy
            z_abs = self.HOME_Z + uz
        else:
            x_abs, y_abs, z_abs = ux, uy, uz

        self.cmd_goto_xyz(x_abs, y_abs, z_abs)

    def jog_cartesian(self, dx=0.0, dy=0.0, dz=0.0):
        """
        Desplaza el efector final de forma incremental y suave en el espacio cartesiano.
        dz > 0: Subir
        dz < 0: Bajar hacia el tablero / mesa
        dx: Adelante (+) / Atrás (-)
        dy: Derecha (+) / Izquierda (-)
        """
        xfk, yfk, zfk = self.kin.forward_kinematics(*self.target_angles)
        nx = xfk + dx
        ny = yfk + dy
        nz = zfk + dz
        self.cmd_goto_xyz(nx, ny, nz)

    # --------------------------------------------------------------------------
    # VALIDACIÓN, CÁLCULO DE ERROR Y EJECUCIÓN DE CINEMÁTICA INVERSA
    # --------------------------------------------------------------------------
    def cmd_goto_xyz(self, x, y, z):
        """
        ========================================================================
        >>> EJECUCIÓN OPERACIONAL DE CINEMÁTICA INVERSA (IK) <<<
        ========================================================================
        1. Invoca `self.kin.inverse_kinematics(x, y, z, elbow_up=True)`.
        2. Si no es alcanzable o viola límites articulares, notifica el error.
        3. Realiza la comprobación cruzada mediante Cinemática Directa (FK):
             (xfk, yfk, zfk) = self.kin.forward_kinematics(*angles)
        4. Calcula el Error Euclidiano Residual de posicionamiento:
             err = sqrt((xfk - x)^2 + (yfk - y)^2 + (zfk - z)^2)
        5. Actualiza los valores objetivos de la articulación y los sliders.
        6. Transmite la consigna de movimiento a los motores.
        ========================================================================
        """
        self.cart_x = x
        self.cart_y = y
        self.cart_z = z

        # 1. Resolución analítica de la Cinemática Inversa
        angles, reachable, msg = self.kin.inverse_kinematics(x, y, z, elbow_up=True)
        if not reachable:
            def _warn():
                try:
                    self.lbl_ik_status.config(text=f"Estado IK: [NO ALCANZABLE] {msg}", fg=COLOR_RED_DANGER)
                except Exception:
                    pass
            self.dispatch_ui(_warn)
            return

        # 2. Validación de consistencia con Cinemática Directa (FK)
        xfk, yfk, zfk = self.kin.forward_kinematics(*angles)
        err = math.sqrt((xfk - x)**2 + (yfk - y)**2 + (zfk - z)**2)
        self.last_ik_error_pos = err

        # 3. Asignación de consignas a las articulaciones
        if getattr(self, 'motors_free_state', False):
            self.motors_free_state = False
            self.update_zero_bar_ui()

        for i in range(3):
            self.target_angles[i] = angles[i]

        # 4. Actualización sincronizada de la GUI
        def _update_ui():
            try:
                self.update_cartesian_entries(xfk, yfk, zfk)
                if getattr(self, 'coord_mode_relative', True):
                    rx = xfk - self.HOME_X
                    ry = yfk - self.HOME_Y
                    rz = zfk - self.HOME_Z
                    lbl_txt = f"Estado IK: ALCANZABLE | Rel Cero: ({rx:+.1f}, {ry:+.1f}, {rz:+.1f}) cm | FK: ({xfk:.1f}, {yfk:.1f}, {zfk:.1f})"
                else:
                    lbl_txt = f"Estado IK: ALCANZABLE | Pos: ({xfk:.1f}, {yfk:.1f}, {zfk:.1f}) cm | Err: {err:.4e}"
                self.lbl_ik_status.config(text=lbl_txt, fg=COLOR_GREEN_SUCCESS)
                for i in range(3):
                    self.sliders_ax[i].set(angles[i])
            except Exception:
                pass
        self.dispatch_ui(_update_ui)

        # 5. Transmisión del paquete angular al ESP32
        self.send_angles_command(angles[0], angles[1], angles[2])

    def cmd_goto_angles(self, q1, q2, q3):
        """Mueve el robot directamente a consignas angulares (q1, q2, q3) en grados respetando cinemática."""
        if getattr(self, 'motors_free_state', False):
            self.motors_free_state = False
            self.update_zero_bar_ui()
        for i, val in enumerate([q1, q2, q3]):
            self.target_angles[i] = float(val)
        xfk, yfk, zfk = self.kin.forward_kinematics(q1, q2, q3)
        self.cart_x, self.cart_y, self.cart_z = xfk, yfk, zfk
        def _update_ui():
            try:
                self.update_cartesian_entries(xfk, yfk, zfk)
                self.lbl_ik_status.config(
                    text=f"Estado IK: CONSIGNAS ({q1:.1f}°, {q2:.1f}°, {q3:.1f}°) | Pos: ({xfk:.1f}, {yfk:.1f}, {zfk:.1f}) cm",
                    fg=COLOR_GREEN_SUCCESS
                )
                for i in range(3):
                    self.sliders_ax[i].set(self.target_angles[i])
            except Exception:
                pass
        self.dispatch_ui(_update_ui)
        self.send_angles_command(q1, q2, q3)
        self.dispatch_ui(_update_ui)
        self.send_angles_command(q1, q2, q3)

    def cmd_goto_cell(self, cell_idx):
        """
        Ejecuta el movimiento hacia una de las 9 casillas del gato usando Cinemática Inversa o Calibración Teach-In:
        - Modo 'place': Colocar a nivel de tablero (Z=3.0 cm o ángulos calibrados físicamente).
        - Modo 'hover': Aproximación aérea a 5 cm sobre el tablero (Z=8.0 cm).
        - Modo 'pick_place': Secuencia completa automática tomando ficha del dispensador.
        """
        if cell_idx < 0 or cell_idx >= 9:
            return

        cx, cy, cz = self.cell_coords[cell_idx]
        mode = self.cell_action_mode.get() if hasattr(self, 'cell_action_mode') else "place"

        if mode == "pick_place":
            if not self.is_robot_busy:
                self.set_status_text(f" Ejecutando Pick & Place en Casilla {cell_idx+1}...", fg=COLOR_CYAN_ACCENT)
                threading.Thread(target=self.run_robot_animation_turn, args=(cell_idx,), daemon=True).start()
        elif mode == "hover":
            target_z = cz + 6.0
            angles, reachable, msg = self.kin.inverse_kinematics(cx, cy, target_z, elbow_up=True)
            if not reachable:
                self.lbl_ik_status.config(text=f"Casilla {cell_idx+1} [Hover]: {msg}", fg=COLOR_RED_DANGER)
                return
            if hasattr(self, 'lbl_cell_angles_info'):
                self.lbl_cell_angles_info.config(
                    text=f"Casilla {cell_idx+1} [HOVER Z=8cm]: J1={angles[0]:.2f}°, J2={angles[1]:.2f}°, J3={angles[2]:.2f}°",
                    fg=COLOR_CYAN_ACCENT
                )
            self.cmd_goto_xyz(cx, cy, target_z)
            if self.running and self.serial_port and self.serial_port.is_open:
                self.pkt_seq = (self.pkt_seq + 1) % 10000
                payload = f"PKT,{self.pkt_seq},0,HOVER_CELL,{cell_idx+1},0.0,0.0"
                cs = self.calculate_checksum(payload)
                self.send_serial_raw(f"${payload}*{cs:02X}\r\n")
        else: # "place"
            # Prioridad 1: Si existe calibración directa de ángulos guardados para esta casilla física
            calib_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "casillas_calibradas.json")
            taught_angles = None
            if os.path.exists(calib_file):
                try:
                    with open(calib_file, "r", encoding="utf-8") as f:
                        c_cell = json.load(f).get("cells", {}).get(str(cell_idx + 1))
                        if c_cell and "angles" in c_cell:
                            taught_angles = tuple(c_cell["angles"])
                except Exception:
                    pass

            if taught_angles:
                angles = taught_angles
                reachable = True
                msg = "OK (Ángulos Calibrados 1:1)"
            else:
                angles, reachable, msg = self.kin.inverse_kinematics(cx, cy, cz, elbow_up=True)

            if not reachable:
                self.lbl_ik_status.config(text=f"Casilla {cell_idx+1} [Tablero]: {msg}", fg=COLOR_RED_DANGER)
                return
            if hasattr(self, 'lbl_cell_angles_info'):
                info_txt = f"Casilla {cell_idx+1} [TABLERO 1:1]: J1={angles[0]:.2f}°, J2={angles[1]:.2f}°, J3={angles[2]:.2f}°" if taught_angles else f"Casilla {cell_idx+1} [TABLERO Z={cz:.1f}cm]: J1={angles[0]:.2f}°, J2={angles[1]:.2f}°, J3={angles[2]:.2f}°"
                self.lbl_cell_angles_info.config(
                    text=info_txt,
                    fg=COLOR_GREEN_SUCCESS
                )

            if taught_angles:
                self.cmd_goto_angles(angles[0], angles[1], angles[2])
            else:
                self.cmd_goto_xyz(cx, cy, cz)

            if self.running and self.serial_port and self.serial_port.is_open:
                self.pkt_seq = (self.pkt_seq + 1) % 10000
                payload = f"PKT,{self.pkt_seq},0,CELL,{cell_idx+1},0.0,0.0"
                cs = self.calculate_checksum(payload)
                self.send_serial_raw(f"${payload}*{cs:02X}\r\n")

    def recalculate_board_cells(self, center_x=None):
        """
        Calcula las coordenadas de las 9 casillas del tablero de Gato y del dispensador
        en una zona de alcance óptima 100% libre de colisiones mecánicas con la tuerca M2-M3.
        Mantiene fielmente el Point of View (POV) del usuario en la mesa:
          - Fila 1 (Lejana): center_x + 4.5 cm
          - Fila 2 (Centro): center_x cm
          - Fila 3 (Cercana): center_x - 4.5 cm
          - Columna 1 (Izquierda / Laptop): +4.5 cm
          - Columna 2 (Centro): 0.0 cm
          - Columna 3 (Derecha): -4.5 cm
        """
        # 1. Si existe calibración física por aprendizaje (Teach-In 1:1), cargarla con prioridad absoluta
        calib_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "casillas_calibradas.json")
        if os.path.exists(calib_file):
            try:
                with open(calib_file, "r", encoding="utf-8") as f:
                    cdata = json.load(f).get("cells", {})
                new_coords = []
                for i in range(1, 10):
                    item = cdata.get(str(i))
                    if item and "cartesian" in item:
                        new_coords.append(tuple(item["cartesian"]))
                if len(new_coords) == 9:
                    self.cell_coords = new_coords
                    self.board_center_x = self.cell_coords[4][0]
                    p_item = cdata.get("pick")
                    if p_item and "cartesian" in p_item:
                        self.pick_coord = tuple(p_item["cartesian"])
                    return
            except Exception:
                pass

        if center_x is None:
            # Selecciona la distancia óptima según la longitud de los eslabones:
            center_x = 53.0 if getattr(self.kin, 'a2', 32.9) > 28.0 else 42.0

        self.board_center_x = center_x
        d = 4.5
        self.cell_coords = [
            (center_x + d,  d, 3.0),   # Casilla 1 (Fila 1, Col 1 - Lejana Izq)
            (center_x + d,  0.0, 3.0), # Casilla 2 (Fila 1, Col 2 - Lejana Centro)
            (center_x + d, -d, 3.0),   # Casilla 3 (Fila 1, Col 3 - Lejana Der)
            (center_x,      d, 3.0),   # Casilla 4 (Fila 2, Col 1 - Centro Izq)
            (center_x,      0.0, 3.0), # Casilla 5 (Fila 2, Col 2 - Centro Tablero)
            (center_x,     -d, 3.0),   # Casilla 6 (Fila 2, Col 3 - Centro Der)
            (center_x - d,  d, 3.0),   # Casilla 7 (Fila 3, Col 1 - Cercana Izq)
            (center_x - d,  0.0, 3.0), # Casilla 8 (Fila 3, Col 2 - Cercana Centro)
            (center_x - d, -d, 3.0),   # Casilla 9 (Fila 3, Col 3 - Cercana Der)
        ]
        if getattr(self.kin, 'a2', 32.9) > 28.0:
            self.pick_coord = (center_x - 8.0, -18.0, 6.0)
        else:
            self.pick_coord = (center_x - 6.0, -15.0, 6.0)

    def adapt_cells_to_current_limits(self, show_msg=False):
        """
        Calcula y adapta automáticamente la posición del tablero (X centro)
        y las 9 casillas para que se ajusten estrictamente a los límites angulares
        configurados actualmente en el robot (M1, M2, M3), evitando sobre-carrera
        y colisiones con la tuerca articular.
        """
        z_eval = 3.0
        d = 4.5
        max_reach = self.kin.a1 + self.kin.a2 + self.kin.a3
        min_scan = int(max(18.0, self.kin.a1 + 6.0) * 10)
        max_scan = int((max_reach - 8.0) * 10)

        pref_cx = 53.0 if getattr(self.kin, 'a2', 32.9) > 28.0 else 42.0

        best_cx = None
        best_score = -1e9
        best_margin = -1e9

        for cx_10 in range(min_scan, max_scan, 5):
            cx = cx_10 / 10.0
            test_cells = [
                (cx + d,  d, z_eval),
                (cx + d,  0.0, z_eval),
                (cx + d, -d, z_eval),
                (cx,      d, z_eval),
                (cx,      0.0, z_eval),
                (cx,     -d, z_eval),
                (cx - d,  d, z_eval),
                (cx - d,  0.0, z_eval),
                (cx - d, -d, z_eval),
            ]
            all_valid = True
            min_margin = 1e9
            for tx, ty, tz in test_cells:
                sol, reach, _ = self.kin.inverse_kinematics(tx, ty, tz)
                if not reach:
                    all_valid = False
                    break
                for j in range(3):
                    m_low = sol[j] - self.kin.limits[j][0]
                    m_high = self.kin.limits[j][1] - sol[j]
                    min_margin = min(min_margin, m_low, m_high)

            if all_valid and min_margin > 0.0:
                dist_penalty = abs(cx - pref_cx) * 0.4
                score = min_margin - dist_penalty
                if score > best_score:
                    best_score = score
                    best_cx = cx
                    best_margin = min_margin

        if best_cx is not None:
            self.recalculate_board_cells(best_cx)
            if hasattr(self, 'lbl_cell_angles_info'):
                self.lbl_cell_angles_info.config(
                    text=f"9 Casillas adaptadas a límites: Centro X={best_cx:.1f} cm | Margen mín: +{best_margin:.1f}°",
                    fg=COLOR_GREEN_SUCCESS
                )
            if show_msg:
                messagebox.showinfo(
                    "Casillas Adaptadas a Límites",
                    f"Se reajustó la posición del tablero de Gato a los límites actuales:\n\n"
                    f"  • Centro del Tablero: X = {best_cx:.1f} cm\n"
                    f"  • Margen de Seguridad Articular: +{best_margin:.1f}°\n"
                    f"  • Rango M1 (Base):   [{self.kin.limits[0][0]:.1f}°, {self.kin.limits[0][1]:.1f}°]\n"
                    f"  • Rango M2 (Hombro): [{self.kin.limits[1][0]:.1f}°, {self.kin.limits[1][1]:.1f}°]\n"
                    f"  • Rango M3 (Codo):   [{self.kin.limits[2][0]:.1f}°, {self.kin.limits[2][1]:.1f}°]\n\n"
                    f"Las 9 casillas son 100% alcanzables sin colisión de tuerca ni sobre-carrera."
                )
            return True
        else:
            if show_msg:
                messagebox.showwarning(
                    "Límites Demasiado Restrictivos",
                    "No fue posible ubicar las 9 casillas dentro de los límites actuales.\n\n"
                    "Por favor amplíe el rango de M2 o M3 en 'Límites de Seguridad Configurables'."
                )
            return False

    def start_cells_tour_sequence(self):
        """Inicia el recorrido automatizado secuencial de las casillas 1 a 9."""
        if getattr(self, 'tour_active', False):
            return

        # Adaptar previamente las casillas a los límites para garantizar movimiento seguro
        self.adapt_cells_to_current_limits(show_msg=False)

        self.tour_active = True
        if hasattr(self, 'btn_tour_start'):
            self.btn_tour_start.config(text=" RECORRIENDO...", bg="#0369a1", state=tk.DISABLED)
        if hasattr(self, 'btn_tour_stop'):
            self.btn_tour_stop.config(bg=COLOR_RED_DANGER, state=tk.NORMAL)

        threading.Thread(target=self._run_cells_tour_worker, daemon=True).start()

    def stop_cells_tour_sequence(self):
        """Detiene de forma inmediata el recorrido de casillas."""
        self.tour_active = False
        self._reset_tour_ui_state("Secuencia detenida por el usuario.")

    def _reset_tour_ui_state(self, status_msg=None):
        def _update():
            self.active_tour_cell = None
            if hasattr(self, 'btn_tour_start'):
                self.btn_tour_start.config(text="Recorrer 1 a 9 (TOUR)", bg="#0284c7", state=tk.NORMAL)
            if hasattr(self, 'btn_tour_stop'):
                self.btn_tour_stop.config(bg="#475569", state=tk.DISABLED)
            if hasattr(self, 'cell_buttons'):
                for b in self.cell_buttons:
                    try:
                        b.config(bg="#1e293b", fg=COLOR_CYAN_ACCENT)
                    except Exception:
                        pass
            if status_msg and hasattr(self, 'lbl_cell_angles_info'):
                self.lbl_cell_angles_info.config(text=status_msg, fg=COLOR_CYAN_ACCENT)
        self.dispatch_ui(_update)

    def _run_cells_tour_worker(self):
        """Hilo de fondo que recorre secuencialmente las casillas 1 a 9."""
        try:
            try:
                pause_sec = float(self.ent_tour_pause.get()) if hasattr(self, 'ent_tour_pause') else 1.5
                pause_sec = max(0.5, min(5.0, pause_sec))
            except (ValueError, TypeError):
                pause_sec = 1.5

            for cell_idx in range(9):
                if not getattr(self, 'tour_active', False):
                    break

                self.active_tour_cell = cell_idx
                cx, cy, cz = self.cell_coords[cell_idx]
                mode = self.cell_action_mode.get() if hasattr(self, 'cell_action_mode') else "place"
                target_z = (cz + 6.0) if mode == "hover" else cz

                angles, reachable, msg = self.kin.inverse_kinematics(cx, cy, target_z, elbow_up=True)

                def _highlight(idx=cell_idx, ang=angles, ok=reachable, x=cx, y=cy, z=target_z):
                    if hasattr(self, 'cell_buttons') and idx < len(self.cell_buttons):
                        for j, b in enumerate(self.cell_buttons):
                            b.config(bg="#0284c7" if j == idx else "#1e293b", fg="#ffffff" if j == idx else COLOR_CYAN_ACCENT)
                    if hasattr(self, 'lbl_cell_angles_info'):
                        if ok:
                            self.lbl_cell_angles_info.config(
                                text=f" [Casilla {idx+1}/9]: J1={ang[0]:+.1f}°, J2={ang[1]:.1f}°, J3={ang[2]:.1f}° (X={x:.1f}, Y={y:+.1f}, Z={z:.1f}) | EN LÍMITES",
                                fg=COLOR_GREEN_SUCCESS
                            )
                        else:
                            self.lbl_cell_angles_info.config(
                                text=f"[Casilla {idx+1}/9]: Fuera de límites - {msg}",
                                fg=COLOR_RED_DANGER
                            )
                self.dispatch_ui(_highlight)

                # Mover el robot a la casilla
                self.cmd_goto_cell(cell_idx)

                # Pausa con chequeo rápido de cancelación
                steps = int(pause_sec * 20)
                for _ in range(steps):
                    if not getattr(self, 'tour_active', False):
                        break
                    time.sleep(0.05)

            if getattr(self, 'tour_active', False):
                self._reset_tour_ui_state(" Secuencia 1 a 9 completada exitosamente. Todas las posiciones verificadas dentro de límites.")
            else:
                self._reset_tour_ui_state("Secuencia detenida.")
        except Exception as e:
            print(f"Error en worker de secuencia de casillas: {e}")
            self._reset_tour_ui_state("Error durante la secuencia.")
        finally:
            self.tour_active = False

    def apply_user_limits(self, show_msg=False):
        """Aplica los límites angulares configurados por el usuario a la cinemática, sliders, gauges y firmware."""
        new_limits = []
        for i in range(3):
            try:
                v1 = float(self.ent_lim_min[i].get())
                v2 = float(self.ent_lim_max[i].get())
                if v1 == v2:
                    messagebox.showerror("Error en Límites", f"Eje M{i+1}: Los límites no pueden ser iguales ({v1}°).")
                    return
                # Ordenamiento automático inteligente: permite ingresar 0 y -60 en cualquier orden
                lmin = min(v1, v2)
                lmax = max(v1, v2)

                # Regla de rangos admisibles:
                # M1: [-180.0, 180.0] (La base puede girar hasta 360° si el usuario lo requiere)
                # M2: [-60.0, 60.0]
                # M3: [  0.0, 85.0] (Amplitud completa para alcanzar casillas y bajar a mesa)
                if i == 0:
                    lmin = max(-180.0, lmin)
                    lmax = min(180.0, lmax)
                elif i == 1:
                    lmin = max(-60.0, lmin)
                    lmax = min(60.0, lmax)
                elif i == 2:
                    lmin = max(0.0, lmin)
                    lmax = min(90.0, lmax)

                # Sincronizar las cajas de texto en pantalla con el orden correcto
                self.ent_lim_min[i].delete(0, tk.END)
                self.ent_lim_min[i].insert(0, f"{lmin:.1f}")
                self.ent_lim_max[i].delete(0, tk.END)
                self.ent_lim_max[i].insert(0, f"{lmax:.1f}")

                new_limits.append((lmin, lmax))
            except (ValueError, IndexError):
                messagebox.showerror("Error en Límites", f"Eje M{i+1}: Valores numéricos no válidos.")
                return

        # Actualizar cinemática analítica interna
        self.kin.limits = new_limits
        self.gauge_ranges = [lim for lim in new_limits]

        # Actualizar sliders visuales, etiquetas de rango y botones de presets
        self._suppress_slider_tx = True
        try:
            for i in range(3):
                rmin, rmax = new_limits[i]
                if hasattr(self, 'sliders_ax') and i < len(self.sliders_ax):
                    self.sliders_ax[i].config(from_=rmin, to=rmax)
                    cur_val = self.sliders_ax[i].get()
                    clamped = max(rmin, min(rmax, cur_val))
                    if clamped != cur_val:
                        self.sliders_ax[i].set(clamped)
                        self.target_angles[i] = clamped

                if hasattr(self, 'lbl_slider_min') and i < len(self.lbl_slider_min):
                    self.lbl_slider_min[i].config(text=f"{rmin:+.0f}°")
                if hasattr(self, 'lbl_slider_max') and i < len(self.lbl_slider_max):
                    self.lbl_slider_max[i].config(text=f"{rmax:+.0f}°")

                # Actualizar presets dinámicos adaptados a los límites
                self.update_joint_presets_ui(i, rmin, rmax)

                # Redibujar gauge de inmediato
                if hasattr(self, 'canvas_gauges') and i < len(self.canvas_gauges):
                    self.draw_single_gauge(i)
        finally:
            self._suppress_slider_tx = False

        # Guardar en robot_config.json
        if hasattr(self, 'robot_config') and "profiles" in self.robot_config:
            p_name = getattr(self, 'active_profile_name', 'Robot MK2 Gato 31:1 (Predeterminado)')
            if p_name in self.robot_config["profiles"]:
                self.robot_config["profiles"][p_name]["limits"] = {
                    "m1": [new_limits[0][0], new_limits[0][1]],
                    "m2": [new_limits[1][0], new_limits[1][1]],
                    "m3": [new_limits[2][0], new_limits[2][1]],
                }
                self.save_robot_config_file(self.robot_config)

        # Enviar comandos de límites al firmware ESP32
        if self.running and self.serial_port and self.serial_port.is_open:
            for i in range(3):
                lmin, lmax = new_limits[i]
                self.pkt_seq = (self.pkt_seq + 1) % 10000
                payload = f"PKT,{self.pkt_seq},{i+1},LIMITS,{lmin:.1f},{lmax:.1f},0.0"
                cs = self.calculate_checksum(payload)
                self.send_serial_raw(f"${payload}*{cs:02X}\r\n")

        # Adaptar automáticamente las 9 casillas del tablero a los nuevos límites
        self.adapt_cells_to_current_limits(show_msg=False)

        # Feedback visual ágil sin bloquear la pantalla con ventanas emergentes
        if hasattr(self, 'lbl_nut_status'):
            self.lbl_nut_status.config(text=f"ACTIVA (Min Codo ≥ {new_limits[2][0]:.0f}°)")
        if hasattr(self, 'lbl_limits_status'):
            self.lbl_limits_status.config(text="Límites aplicados al Robot, Sliders y Flash", fg="#34d399")
            self.after(3000, lambda: self.lbl_limits_status.config(text="● Límites activos", fg="#94a3b8") if hasattr(self, 'lbl_limits_status') else None)
        elif show_msg:
            messagebox.showinfo(
                "Límites Actualizados",
                f"Límites seguros guardados en NVS y aplicados con éxito:\n\n"
                f"  • M1 (Base):   [{new_limits[0][0]:.1f}°, {new_limits[0][1]:.1f}°]\n"
                f"  • M2 (Hombro): [{new_limits[1][0]:.1f}°, {new_limits[1][1]:.1f}°]\n"
                f"  • M3 (Codo):   [{new_limits[2][0]:.1f}°, {new_limits[2][1]:.1f}°]\n\n"
                f"Manipulador y firmware 100% protegidos contra sobre-carrera."
            )

    def update_joint_presets_ui(self, ax, rmin, rmax):
        """Regenera los 5 botones de presets rápidos adaptados exactamente a los límites configurados."""
        if not hasattr(self, 'preset_btn_list') or ax >= len(self.preset_btn_list):
            return
        step = (rmax - rmin) / 4.0
        raw_vals = [rmin, rmin + step, rmin + 2 * step, rmin + 3 * step, rmax]
        vals = []
        for v in raw_vals:
            if abs(v) < max(0.6, step * 0.3):
                vals.append(0.0)
            else:
                vals.append(round(v, 1) if abs(v) < 10 else round(v))
        vals[0] = rmin
        vals[-1] = rmax

        btns = self.preset_btn_list[ax]
        for b_idx, btn in enumerate(btns):
            if b_idx < len(vals):
                val_deg = vals[b_idx]
                txt = f"{val_deg:+.0f}°" if val_deg != 0 else "0°"
                btn.config(
                    text=txt,
                    command=lambda a=ax, d=val_deg: self.set_joint_preset(a, d)
                )

    def cmd_home(self):
        """Envía el robot a su posición HOME (55.0, 0.0, 10.0 cm | Relativo 0,0,0)."""
        self.motors_free_state = False
        self.cmd_goto_xyz(self.HOME_X, self.HOME_Y, self.HOME_Z)

    def on_joint_slider(self, ax, val):
        """
        Callback cuando el usuario manipula manualmente un slider articular:
        Aplica Cinemática Directa (FK), valida la protección anticolisión de la tuerca M2-M3
        y envía consignas al robot a 30 Hz para garantizar respuesta inmediata en tiempo real.
        """
        if getattr(self, '_suppress_slider_tx', False):
            return
        try:
            deg = float(val)

            # Acotar estrictamente según los límites de la articulación (sin trabas)
            lmin = min(self.kin.limits[ax])
            lmax = max(self.kin.limits[ax])
            deg = max(lmin, min(lmax, deg))

            self.target_angles[ax] = deg
            # Si el usuario manipula un slider, sale del modo libre
            if self.motors_free_state:
                self.motors_free_state = False
                self.update_zero_bar_ui()
            # Recálculo de posición cartesiana por Cinemática Directa (FK)
            xfk, yfk, zfk = self.kin.forward_kinematics(*self.target_angles)
            self.update_cartesian_entries(xfk, yfk, zfk)

            # Despacho en tiempo real a ~30 Hz (33 ms)
            now = time.time()
            if (now - self._last_slider_tx_time) >= 0.033:
                self._last_slider_tx_time = now
                self._slider_tx_pending = False
                self.send_angles_command(self.target_angles[0], self.target_angles[1], self.target_angles[2])
            elif not self._slider_tx_pending:
                self._slider_tx_pending = True
                self.after(33, self._flush_slider_tx)
        except (ValueError, TypeError):
            pass

    def _flush_slider_tx(self):
        """Envía el paquete pendiente acumulado del slider tras cumplirse el intervalo de 33 ms."""
        self._slider_tx_pending = False
        self._last_slider_tx_time = time.time()
        self.send_angles_command(self.target_angles[0], self.target_angles[1], self.target_angles[2])

    def _on_slider_release(self, ax, event=None):
        """Al soltar el slider con el ratón, envía inmediatamente la consigna final definitiva."""
        self._slider_tx_pending = False
        self._last_slider_tx_time = time.time()
        self.send_angles_command(self.target_angles[0], self.target_angles[1], self.target_angles[2])

    def set_joint_preset(self, ax, deg):
        """Ajusta una articulación al ángulo objetivo respetando estrictamente los límites anticolisión."""
        try:
            if getattr(self, 'motors_free_state', False):
                self.motors_free_state = False
                self.update_zero_bar_ui()
            val = float(deg)
            # Protección estricta contra sobre-carrera y colisión de tuerca
            lim_min, lim_max = self.kin.limits[ax]
            val = max(lim_min, min(lim_max, val))
            self.target_angles[ax] = val
            if hasattr(self, 'sliders_ax') and ax < len(self.sliders_ax):
                self.sliders_ax[ax].set(val)
            xfk, yfk, zfk = self.kin.forward_kinematics(*self.target_angles)
            self.update_cartesian_entries(xfk, yfk, zfk)
            self.send_angles_command(self.target_angles[0], self.target_angles[1], self.target_angles[2])
        except Exception:
            pass

    def cmd_all_presets(self, deg):
        """Envía el mismo ángulo de reductor a los 3 motores simultáneamente."""
        for ax in range(3):
            val = float(deg)
            lim_min, lim_max = self.kin.limits[ax]
            val = max(lim_min, min(lim_max, val))
            self.target_angles[ax] = val
            if hasattr(self, 'sliders_ax') and ax < len(self.sliders_ax):
                self.sliders_ax[ax].set(val)
        try:
            xfk, yfk, zfk = self.kin.forward_kinematics(*self.target_angles)
            self.update_cartesian_entries(xfk, yfk, zfk)
        except Exception:
            pass
        self.send_angles_command(self.target_angles[0], self.target_angles[1], self.target_angles[2])

    # --------------------------------------------------------------------------
    # SECUENCIA AUTOMATIZADA PICK & PLACE (TRAYECTORIAS CINEMÁTICAS)
    # --------------------------------------------------------------------------
    def run_robot_animation_turn(self, move):
        """
        Máquina de estados finitos que ejecuta la secuencia completa Pick & Place
        para colocar una ficha 'O' del robot en la casilla seleccionada por la IA:
          1. Desplazamiento sobre el dispensador de fichas (aproximación vertical).
          2. Descenso hasta la ficha (Pick).
          3. Activación del efector final (Gripper ON).
          4. Elevación a Posición Segura de Tránsito (PSS).
          5. Desplazamiento sobre la casilla objetivo (aproximación vertical).
          6. Descenso a la altura del tablero y liberación de la ficha (Place / Gripper OFF).
          7. Despeje vertical sobre la casilla.
          8. Retorno a posición HOME.
          9. Evaluación del estado de la partida (Victoria / Empate / Cambio de turno).
        """
        self.is_robot_busy = True
        try:
            # 1. Aproximación hacia el Dispensador de Fichas (Pick)
            self.set_status_text("Perfil: Moviendo al dispensador...", fg=COLOR_CYAN_ACCENT)
            px, py, pz = self.pick_coord
            self.cmd_goto_xyz(px, py, pz + 5.0)
            self.wait_motion_complete()

            # 2. Descenso y toma de ficha
            self.cmd_goto_xyz(px, py, pz)
            self.wait_motion_complete()

            # 3. Activación del Gripper / Sujeción
            self.gripper_state = True
            self.update_gripper_button()
            time.sleep(0.35)

            # 4. Elevación segura hacia Posición de Tránsito (PSS)
            self.set_status_text("Perfil: Elevando ficha hacia PSS...", fg=COLOR_CYAN_ACCENT)
            self.cmd_goto_xyz(22.0, -5.0, 14.0)
            self.wait_motion_complete()

            # 5. Aproximación vertical sobre la casilla de destino
            self.set_status_text(f"Perfil: Colocando en Casilla {move+1}...", fg=COLOR_CYAN_ACCENT)
            cx, cy, cz = self.cell_coords[move]
            self.cmd_goto_xyz(cx, cy, cz + 5.0)
            self.wait_motion_complete()

            # 6. Descenso y colocación de la ficha
            self.cmd_goto_xyz(cx, cy, cz)
            self.wait_motion_complete()

            # 7. Liberar ficha y registrar jugada en el tablero
            self.gripper_state = False
            self.update_gripper_button()
            self.board[move] = 2
            self.dispatch_ui(lambda: self.cell_buttons[move].config(text="O", fg=COLOR_GREEN_SUCCESS, bg="#1e293b"))
            time.sleep(0.3)

            # 8. Retiro vertical para no colisionar con fichas vecinas
            self.cmd_goto_xyz(cx, cy, cz + 5.0)
            self.wait_motion_complete()

            # 9. Transmisión serial de la jugada al ESP32 si está conectado
            if self.running and self.serial_port and self.serial_port.is_open:
                self.pkt_seq = (self.pkt_seq + 1) % 10000
                payload = f"PKT,{self.pkt_seq},0,PLAY_CELL,{float(move+1):.2f},0.0,0.0"
                cs = self.calculate_checksum(payload)
                self.send_serial_raw(f"${payload}*{cs:02X}\r\n")

            # 10. Retorno a posición segura HOME
            self.set_status_text("Perfil: Regresando a HOME...", fg=COLOR_CYAN_ACCENT)
            self.cmd_home()
            self.wait_motion_complete()

            # 11. Evaluación de condiciones finales de la partida
            if self.check_winner(2):
                self.game_active = False
                self.robot_score += 1
                self.dispatch_ui(lambda: self.update_scores(" ¡El Robot ha ganado la partida!"))
                return

            if 0 not in self.board:
                self.game_active = False
                self.draws_score += 1
                self.dispatch_ui(lambda: self.update_scores("¡Empate técnico!"))
                return

            self.turn = "HUMANO"
            self.set_status_text("● TU TURNO (X)", fg=COLOR_GREEN_SUCCESS)

        finally:
            self.is_robot_busy = False

    def wait_motion_complete(self, timeout=3.2):
        """Espera a que los ángulos articulares alcancen las consignas objetivo."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            max_diff = max(abs(self.target_angles[i] - self.current_angles[i]) for i in range(3))
            if max_diff < 1.2:
                break
            time.sleep(0.04)
        time.sleep(0.08)

    def update_gripper_button(self):
        """Refresca la apariencia y color del botón del Gripper en la GUI."""
        if hasattr(self, 'btn_gripper'):
            self.dispatch_ui(lambda: self.btn_gripper.config(
                text="Herramienta: ON" if self.gripper_state else "Herramienta: OFF",
                bg="#059669" if self.gripper_state else "#334155"
            ))


# ==============================================================================
# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ TEMA 8: SIMULACIÓN GRÁFICA 3D INTERACTIVA Y CÁMARA ORBITAL                 ║
# ╚════════════════════════════════════════════════════════════════════════════╝
# ==============================================================================

    def project_3d(self, x, y, z, cx, cy, scale):
        """
        Proyección 3D interactiva en perspectiva/isométrica con cámara orbital:
          - Eje X: Eje frontal hacia el tablero de juego (+X hacia el frente).
          - Eje Y: Eje lateral (+Y izquierda, -Y derecha / dispensador).
          - Eje Z: Eje vertical (+Z hacia arriba).

        Transformaciones aplicadas:
          1. Rotación de azimut (yaw) sobre el eje Z.
          2. Inclinación de elevación (pitch) sobre el plano visual.
          3. Escalamiento y traslación al origen de pantalla (cx, cy).
        """
        xr = x * math.cos(self.cam_yaw) - y * math.sin(self.cam_yaw)
        yr = x * math.sin(self.cam_yaw) + y * math.cos(self.cam_yaw)

        sin_p = math.sin(self.cam_pitch)
        cos_p = math.cos(self.cam_pitch)

        u = cx + xr * scale
        v = cy - (yr * sin_p + z * cos_p) * scale
        return u, v

    # Presets de cámara rápida
    def view_user_pov(self):
        """Ajusta la cámara 3D para coincidir exactamente con la perspectiva del usuario frente a la mesa."""
        self.cam_yaw = math.radians(-70.0)
        self.cam_pitch = math.radians(22.0)
        self.cam_zoom = 1.05

    def view_isometric(self):
        """Ajusta la cámara a la perspectiva isométrica estándar (40°, 24°)."""
        self.cam_yaw = math.radians(40.0)
        self.cam_pitch = math.radians(24.0)
        self.cam_zoom = 1.0

    def view_top(self):
        """Ajusta la cámara a la vista de planta superior (0°, 82°)."""
        self.cam_yaw = math.radians(0.0)
        self.cam_pitch = math.radians(82.0)
        self.cam_zoom = 1.0

    def view_side(self):
        """Ajusta la cámara a la vista de perfil lateral (90°, 12°)."""
        self.cam_yaw = math.radians(90.0)
        self.cam_pitch = math.radians(12.0)
        self.cam_zoom = 1.0

    # Eventos de ratón para manipulación orbital 3D
    def on_sim_mouse_down(self, event):
        self.cam_last_x = event.x
        self.cam_last_y = event.y

    def on_sim_mouse_drag(self, event):
        dx = event.x - self.cam_last_x
        dy = event.y - self.cam_last_y
        self.cam_last_x = event.x
        self.cam_last_y = event.y

        self.cam_yaw += dx * 0.009
        self.cam_pitch -= dy * 0.009
        self.cam_pitch = max(math.radians(6.0), min(math.radians(85.0), self.cam_pitch))

    def on_sim_mouse_wheel(self, event):
        if event.delta > 0:
            self.cam_zoom = min(2.5, self.cam_zoom * 1.1)
        else:
            self.cam_zoom = max(0.4, self.cam_zoom / 1.1)

    def draw_simulation(self):
        """
        Dibuja la escena completa 3D sobre el Canvas de Tkinter:
          1. Cuadrícula del suelo (Z = 0) y sistema de coordenadas cartesiano.
          2. Dispensadores cilíndricos de fichas (O a la izquierda, X a la derecha).
          3. Tablero elevado 3x3 y fichas volumétricas colocadas.
          4. Robot antropomórfico MK2 con eslabones proporcionales y articulaciones.
          5. Indicadores HUD con coordenadas del efector y mini radar 2D de planta.
        """
        cv = self.canvas_sim
        cv.delete("all")
        w = cv.winfo_width()
        h = cv.winfo_height()
        if w < 60 or h < 60:
            return

        cx = w * 0.35
        cy = h * 0.72
        scale = min(w / 96.0, (h - 40) / 72.0) * self.cam_zoom

        q1_deg = self.current_angles[0]
        q2_deg = self.current_angles[1]
        q3_deg = self.current_angles[2]

        q1 = math.radians(q1_deg)
        q2 = math.radians(q2_deg)
        q3 = math.radians(q3_deg)

        # ----------------------------------------------------------------------
        # 1. CUADRÍCULA DEL SUELO (Z = 0) Y EJES COORDENADOS
        # ----------------------------------------------------------------------
        for gx in range(-15, 75, 10):
            u1, v1 = self.project_3d(gx, -30, 0.0, cx, cy, scale)
            u2, v2 = self.project_3d(gx,  30, 0.0, cx, cy, scale)
            cv.create_line(u1, v1, u2, v2, fill="#e2e8f0", width=1)
        for gy in range(-30, 35, 10):
            u1, v1 = self.project_3d(-15, gy, 0.0, cx, cy, scale)
            u2, v2 = self.project_3d( 70, gy, 0.0, cx, cy, scale)
            cv.create_line(u1, v1, u2, v2, fill="#e2e8f0", width=1)

        # Ejes coordenados principales (Origen X=Rojo, Y=Verde, Z=Cian)
        u0, v0 = self.project_3d(0, 0, 0, cx, cy, scale)
        ux, vx = self.project_3d(7, 0, 0, cx, cy, scale)
        uy, vy = self.project_3d(0, 7, 0, cx, cy, scale)
        uz, vz = self.project_3d(0, 0, 7, cx, cy, scale)
        cv.create_line(u0, v0, ux, vx, fill=COLOR_RED_DANGER, width=2, arrow=tk.LAST)
        cv.create_line(u0, v0, uy, vy, fill=COLOR_GREEN_SUCCESS, width=2, arrow=tk.LAST)
        cv.create_line(u0, v0, uz, vz, fill=COLOR_CYAN_ACCENT, width=2, arrow=tk.LAST)
        cv.create_text(ux + 6, vx, text="X", fill=COLOR_RED_DANGER, font=("Segoe UI", 6, "bold"))
        cv.create_text(uy + 6, vy, text="Y", fill=COLOR_GREEN_SUCCESS, font=("Segoe UI", 6, "bold"))
        cv.create_text(uz, vz - 6, text="Z", fill=COLOR_CYAN_ACCENT, font=("Segoe UI", 6, "bold"))

        # ----------------------------------------------------------------------
        # 2. DISPENSADORES DE FICHAS 3D (Cilindros 'O' y 'X')
        # ----------------------------------------------------------------------
        r_disp = 2.4
        dz_disp = 6.0

        # Dispensador Izquierdo: 'O' (Robot / Fichas Azules / Y = -14 cm)
        dx_o, dy_o = 18.0, -14.0
        poly_o_bot, poly_o_top = [], []
        for i in range(12):
            th = 2.0 * math.pi * i / 12
            bx = dx_o + r_disp * math.cos(th)
            by = dy_o + r_disp * math.sin(th)
            u_b, v_b = self.project_3d(bx, by, 0.0, cx, cy, scale)
            u_t, v_t = self.project_3d(bx, by, dz_disp, cx, cy, scale)
            poly_o_bot.extend([u_b, v_b])
            poly_o_top.extend([u_t, v_t])
        cv.create_polygon(poly_o_bot, fill="#e0f2fe", outline="#0284c7")
        cv.create_polygon(poly_o_top, fill="#bae6fd", outline="#0284c7", width=1.5)
        ud_o, vd_o = self.project_3d(dx_o, dy_o, dz_disp, cx, cy, scale)
        cv.create_text(ud_o, vd_o - 8, text="DISP. 'O'", fill="#0369a1", font=("Segoe UI", 6, "bold"))

        # Dispensador Derecho: 'X' (Humano / Fichas Rojas / Y = +14 cm)
        dx_x, dy_x = 18.0, 14.0
        poly_x_bot, poly_x_top = [], []
        for i in range(12):
            th = 2.0 * math.pi * i / 12
            bx = dx_x + r_disp * math.cos(th)
            by = dy_x + r_disp * math.sin(th)
            u_b, v_b = self.project_3d(bx, by, 0.0, cx, cy, scale)
            u_t, v_t = self.project_3d(bx, by, dz_disp, cx, cy, scale)
            poly_x_bot.extend([u_b, v_b])
            poly_x_top.extend([u_t, v_t])
        cv.create_polygon(poly_x_bot, fill="#fee2e2", outline="#dc2626")
        cv.create_polygon(poly_x_top, fill="#fecaca", outline="#dc2626", width=1.5)
        ud_x, vd_x = self.project_3d(dx_x, dy_x, dz_disp, cx, cy, scale)
        cv.create_text(ud_x, vd_x - 8, text="DISP. 'X'", fill="#b91c1c", font=("Segoe UI", 6, "bold"))

        # ----------------------------------------------------------------------
        # 3. TABLERO DE GATO 3X3 ELEVADO Y FICHAS TRIDIMENSIONALES
        # ----------------------------------------------------------------------
        all_x = [c[0] for c in self.cell_coords]
        all_y = [c[1] for c in self.cell_coords]
        b_x1, b_x2 = min(all_x) - 2.5, max(all_x) + 2.5
        b_y1, b_y2 = min(all_y) - 2.5, max(all_y) + 2.5
        p_board_top = []
        for px, py in [(b_x1, b_y1), (b_x2, b_y1), (b_x2, b_y2), (b_x1, b_y2)]:
            u, v = self.project_3d(px, py, 0.8, cx, cy, scale)
            p_board_top.extend([u, v])
        cv.create_polygon(p_board_top, fill="#f8fafc", outline="#cbd5e1", width=2)

        # 9 Casillas del tablero con representación de fichas
        for idx, (cx_c, cy_c, cz_c) in enumerate(self.cell_coords):
            s_c = 1.9
            p_c = []
            for dxi, dyi in [(-s_c, -s_c), (s_c, -s_c), (s_c, s_c), (-s_c, s_c)]:
                u, v = self.project_3d(cx_c + dxi, cy_c + dyi, 0.9, cx, cy, scale)
                p_c.extend([u, v])

            # Resaltado especial durante la secuencia de verificación 1 a 9
            is_active_tour = (getattr(self, 'active_tour_cell', None) == idx)
            fill_c = "#dbeafe" if is_active_tour else "#ffffff"
            outline_c = "#1e40af" if is_active_tour else "#cbd5e1"
            width_c = 2.5 if is_active_tour else 1
            cv.create_polygon(p_c, fill=fill_c, outline=outline_c, width=width_c)

            val = self.board[idx]
            cu, cv_pos = self.project_3d(cx_c, cy_c, 1.0, cx, cy, scale)
            if is_active_tour:
                # Indicador flotante 3D sobre la casilla en verificación
                cu_t, cv_t = self.project_3d(cx_c, cy_c, 3.8, cx, cy, scale)
                cv.create_text(cu_t, cv_t, text=f"CASILLA {idx+1}", fill=COLOR_CYAN_ACCENT, font=("Segoe UI", 8, "bold"))
            if val == 0:
                cv.create_text(cu, cv_pos, text=f"{idx+1}", fill="#0f172a", font=("Segoe UI", 7, "bold"))
            elif val == 1:
                # Ficha Humana 'X' (Cilindro Rojo)
                p_puck = []
                for pt in range(10):
                    th = 2.0 * math.pi * pt / 10
                    u_p, v_p = self.project_3d(cx_c + 1.4 * math.cos(th), cy_c + 1.4 * math.sin(th), 2.2, cx, cy, scale)
                    p_puck.extend([u_p, v_p])
                cv.create_polygon(p_puck, fill="#dc2626", outline=COLOR_RED_DANGER, width=1.5)
                cu_top, cv_top = self.project_3d(cx_c, cy_c, 2.3, cx, cy, scale)
                cv.create_text(cu_top, cv_top, text="X", fill="#ffffff", font=("Segoe UI", 9, "bold"))
            elif val == 2:
                # Ficha Robot 'O' (Cilindro Verde)
                p_puck = []
                for pt in range(10):
                    th = 2.0 * math.pi * pt / 10
                    u_p, v_p = self.project_3d(cx_c + 1.4 * math.cos(th), cy_c + 1.4 * math.sin(th), 2.2, cx, cy, scale)
                    p_puck.extend([u_p, v_p])
                cv.create_polygon(p_puck, fill="#15803d", outline=COLOR_GREEN_SUCCESS, width=1.5)
                cu_top, cv_top = self.project_3d(cx_c, cy_c, 2.3, cx, cy, scale)
                cv.create_text(cu_top, cv_top, text="O", fill="#ffffff", font=("Segoe UI", 9, "bold"))

        # ----------------------------------------------------------------------
        # 4. ROBOT ANTROPOMÓRFICO MK2 DE 3 GDL (CINEMÁTICA Y VOLUMEN)
        # ----------------------------------------------------------------------
        # Base plato giratorio (turntable) que rota con q1
        r_base = 7.0
        poly_base_bot = []
        poly_base_top = []
        for i in range(16):
            th = 2.0 * math.pi * i / 16
            bx = r_base * math.cos(th)
            by = r_base * math.sin(th)
            u_b, v_b = self.project_3d(bx, by, 0.0, cx, cy, scale)
            u_t, v_t = self.project_3d(bx, by, 2.0, cx, cy, scale)
            poly_base_bot.extend([u_b, v_b])
            poly_base_top.extend([u_t, v_t])
        cv.create_polygon(poly_base_bot, fill="#cbd5e1", outline="#94a3b8")
        cv.create_polygon(poly_base_top, fill="#e2e8f0", outline="#1e40af", width=1.5)

        # Indicador vectorial de la rotación de la base (dirección q1)
        ptr_x = 5.6 * math.cos(q1)
        ptr_y = 5.6 * math.sin(q1)
        u_p0, v_p0 = self.project_3d(0, 0, 2.0, cx, cy, scale)
        u_p1, v_p1 = self.project_3d(ptr_x, ptr_y, 2.0, cx, cy, scale)
        cv.create_line(u_p0, v_p0, u_p1, v_p1, fill=COLOR_CYAN_ACCENT, width=3, arrow=tk.LAST)

        # Columna vertical de la base (d1 = 10.0 cm)
        u_col_b, v_col_b = self.project_3d(0, 0, 2.0, cx, cy, scale)
        u_col_t, v_col_t = self.project_3d(0, 0, self.kin.d1, cx, cy, scale)
        cv.create_line(u_col_b, v_col_b, u_col_t, v_col_t, fill="#94a3b8", width=14, capstyle=tk.ROUND)
        cv.create_line(u_col_b, v_col_b, u_col_t, v_col_t, fill="#0284c7", width=10, capstyle=tk.ROUND)

        # Cálculo de las posiciones espaciales de cada articulación
        p0 = (0.0, 0.0, self.kin.d1)
        p1 = (self.kin.a1 * math.cos(q1), self.kin.a1 * math.sin(q1), self.kin.d1)
        r_elbow = self.kin.a1 + self.kin.a2 * math.cos(q2)
        p2 = (r_elbow * math.cos(q1), r_elbow * math.sin(q1), self.kin.d1 + self.kin.a2 * math.sin(q2))
        r_ee = r_elbow + self.kin.a3 * math.cos(q3)
        p3 = (r_ee * math.cos(q1), r_ee * math.sin(q1), p2[2] - self.kin.a3 * math.sin(q3))

        u0_j, v0_j = self.project_3d(*p0, cx, cy, scale)
        u1_j, v1_j = self.project_3d(*p1, cx, cy, scale)
        u2_j, v2_j = self.project_3d(*p2, cx, cy, scale)
        u3_j, v3_j = self.project_3d(*p3, cx, cy, scale)

        # Eslabón 1: Offset horizontal de la base (a1 = 11cm, gira con q1)
        cv.create_line(u0_j, v0_j, u1_j, v1_j, fill="#0369a1", width=9, capstyle=tk.ROUND)
        cv.create_line(u0_j, v0_j, u1_j, v1_j, fill=COLOR_CYAN_ACCENT, width=5, capstyle=tk.ROUND)
        cv.create_oval(u0_j - 5, v0_j - 5, u0_j + 5, v0_j + 5, fill=COLOR_CYAN_ACCENT, outline="#ffffff", width=1)

        # Varilla paralela de tracción/empuje (Mecanismo Paralelogramo 4 Barras MK2)
        rod_h = 4.2  # Altura de los balancines/cuernos articulares
        p_crank_base = (
            (self.kin.a1 - rod_h * math.sin(q3)) * math.cos(q1),
            (self.kin.a1 - rod_h * math.sin(q3)) * math.sin(q1),
            self.kin.d1 + rod_h * math.cos(q3)
        )
        p_crank_elbow = (
            (r_elbow - rod_h * math.sin(q3)) * math.cos(q1),
            (r_elbow - rod_h * math.sin(q3)) * math.sin(q1),
            p2[2] + rod_h * math.cos(q3)
        )
        ur_b, vr_b = self.project_3d(*p_crank_base, cx, cy, scale)
        ur_e, vr_e = self.project_3d(*p_crank_elbow, cx, cy, scale)
        # Balancines de accionamiento M3 (base y codo)
        cv.create_line(u1_j, v1_j, ur_b, vr_b, fill="#334155", width=4, capstyle=tk.ROUND)
        cv.create_line(u2_j, v2_j, ur_e, vr_e, fill="#334155", width=4, capstyle=tk.ROUND)
        # Varilla negra paralela (guarda sincronía de antebrazo)
        cv.create_line(ur_b, vr_b, ur_e, vr_e, fill="#0f172a", width=3, capstyle=tk.ROUND)

        # Eslabón 2: Brazo Superior (a2 = 32.922 cm)
        cv.create_line(u1_j, v1_j, u2_j, v2_j, fill="#065f46", width=10, capstyle=tk.ROUND)
        cv.create_line(u1_j, v1_j, u2_j, v2_j, fill=COLOR_GREEN_SUCCESS, width=6, capstyle=tk.ROUND)
        cv.create_oval(u1_j - 6, v1_j - 6, u1_j + 6, v1_j + 6, fill="#ffffff", outline=COLOR_GREEN_SUCCESS, width=2)

        # Eslabón 3: Antebrazo con orientación desacoplada (a3 = 32.934 cm)
        cv.create_line(u2_j, v2_j, u3_j, v3_j, fill="#9a3412", width=9, capstyle=tk.ROUND)
        cv.create_line(u2_j, v2_j, u3_j, v3_j, fill=COLOR_AMBER_WARNING, width=5, capstyle=tk.ROUND)
        cv.create_oval(u2_j - 5, v2_j - 5, u2_j + 5, v2_j + 5, fill="#ffffff", outline=COLOR_AMBER_WARNING, width=2)

        # Marcador visual de la Tuerca Articular M2-M3 (Protección Anticolisión)
        is_col, is_warn, _ = self.kin.check_m2_m3_collision(q2_deg, q3_deg)
        if is_col:
            cv.create_oval(u2_j - 10, v2_j - 10, u2_j + 10, v2_j + 10, fill="", outline=COLOR_RED_DANGER, width=3)
            cv.create_oval(u2_j - 6, v2_j - 6, u2_j + 6, v2_j + 6, fill=COLOR_RED_DANGER, outline="#ffffff", width=1.5)
            cv.create_text(u2_j, v2_j - 16, text="[COLISIÓN TUERCA]", fill=COLOR_RED_DANGER, font=("Segoe UI", 7, "bold"))
        elif is_warn:
            cv.create_oval(u2_j - 9, v2_j - 9, u2_j + 9, v2_j + 9, fill="", outline=COLOR_AMBER_WARNING, width=2)
            cv.create_oval(u2_j - 5, v2_j - 5, u2_j + 5, v2_j + 5, fill=COLOR_AMBER_WARNING, outline="#ffffff", width=1.5)
            cv.create_text(u2_j, v2_j - 16, text="[TUERCA PRÓXIMA]", fill=COLOR_AMBER_WARNING, font=("Segoe UI", 7, "bold"))
        else:
            cv.create_oval(u2_j - 4, v2_j - 4, u2_j + 4, v2_j + 4, fill="#475569", outline="#94a3b8", width=1.5)

        # Efector Final y Dedos del Gripper
        cv.create_oval(u3_j - 4, v3_j - 4, u3_j + 4, v3_j + 4, fill="#ffffff", outline=COLOR_BG_MAIN, width=1)
        g_len = 10
        g_spread = 8 if not self.gripper_state else 4
        cv.create_line(u3_j, v3_j, u3_j + g_len, v3_j - g_spread, fill="#94a3b8", width=2.5)
        cv.create_line(u3_j, v3_j, u3_j + g_len, v3_j + g_spread, fill="#94a3b8", width=2.5)

        # Si el Gripper está cerrado transportando una ficha:
        if self.gripper_state:
            cv.create_oval(u3_j + 5 - 5, v3_j - 5, u3_j + 5 + 5, v3_j + 5, fill=COLOR_GREEN_SUCCESS, outline="#ffffff", width=1.5)
            cv.create_text(u3_j + 5, v3_j, text="O", fill="#ffffff", font=("Segoe UI", 6, "bold"))

        # ----------------------------------------------------------------------
        # 5. OVERLAYS Y HUD INFORMATIVO EN PANTALLA
        # ----------------------------------------------------------------------
        cv.create_text(
            12, 14,
            text=f"ROBOT MK2 3D | CÁMARA ({math.degrees(self.cam_yaw):.0f}°, {math.degrees(self.cam_pitch):.0f}°) | ARRASTRA EL RATÓN PARA ROTAR",
            fill="#0f172a", font=("Segoe UI", 8, "bold"), anchor=tk.W
        )

        # Banner de estado anticolisión en tiempo real
        if is_col:
            cv.create_rectangle(12, 28, 410, 48, fill="#7f1d1d", outline=COLOR_RED_DANGER, width=1)
            cv.create_text(16, 38, text=f"[ANTICOLISIÓN]: Codo frenado para evitar choque de tuerca ({q3_deg:+.1f}°)",
                           fill="#ffffff", font=("Segoe UI", 7, "bold"), anchor=tk.W)
        elif is_warn:
            cv.create_rectangle(12, 28, 410, 48, fill="#78350f", outline=COLOR_AMBER_WARNING, width=1)
            cv.create_text(16, 38, text=f"[ADVERTENCIA]: Tuerca a {q3_deg - self.kin.limits[2][0]:.1f}° del choque con M2 ({q3_deg:+.1f}°)",
                           fill="#fef08a", font=("Segoe UI", 7, "bold"), anchor=tk.W)

        # Banner de estado de la secuencia de recorrido 1 a 9
        if getattr(self, 'tour_active', False) and getattr(self, 'active_tour_cell', None) is not None:
            c_curr = self.active_tour_cell + 1
            cv.create_rectangle(12, 52, 430, 72, fill="#0c4a6e", outline=COLOR_CYAN_ACCENT, width=1)
            cv.create_text(16, 62, text=f"Secuencia en curso: Mostrando Casilla {c_curr} de 9 (Posición adaptada a límites)",
                           fill="#ffffff", font=("Segoe UI", 7, "bold"), anchor=tk.W)
        if getattr(self, 'coord_mode_relative', True):
            rx = p3[0] - self.HOME_X
            ry = p3[1] - self.HOME_Y
            rz = p3[2] - self.HOME_Z
            hud_coord_txt = f"Efector [Rel 0,0,0]: X={rx:+.1f} cm | Y={ry:+.1f} cm | Z={rz:+.1f} cm (Mesa Z=-39.1)  •  q1={q1_deg:+.1f}° q2={q2_deg:+.1f}° q3={q3_deg:+.1f}°"
        else:
            hud_coord_txt = f"Efector [Abs Base]: X={p3[0]:.1f} cm | Y={p3[1]:.1f} cm | Z={p3[2]:.1f} cm  •  q1={q1_deg:+.1f}° q2={q2_deg:+.1f}° q3={q3_deg:+.1f}°"
        cv.create_text(
            12, h - 14,
            text=hud_coord_txt,
            fill="#0f172a", font=("Consolas", 8, "bold"), anchor=tk.W
        )

        # Mini Radar Superior (Planta 2D) en la esquina superior derecha
        pip_w = 90
        pip_h = 90
        pip_x = w - pip_w - 10
        pip_y = 10
        cv.create_rectangle(pip_x, pip_y, pip_x + pip_w, pip_y + pip_h, fill="#ffffff", outline="#cbd5e1", width=1)
        cv.create_text(pip_x + 4, pip_y + 8, text="RADAR 2D", fill="#64748b", font=("Segoe UI", 6, "bold"), anchor=tk.W)

        pip_cx = pip_x + pip_w * 0.35
        pip_cy = pip_y + pip_h * 0.50
        pip_sc = pip_w / 75.0

        # Casillas en radar
        for idx, (cx_c, cy_c, cz_c) in enumerate(self.cell_coords):
            rc_x = pip_cx + cx_c * pip_sc
            rc_y = pip_cy - cy_c * pip_sc
            v_c = self.board[idx]
            c_col = COLOR_RED_DANGER if v_c == 1 else (COLOR_GREEN_SUCCESS if v_c == 2 else "#1e293b")
            cv.create_rectangle(rc_x - 2, rc_y - 2, rc_x + 2, rc_y + 2, fill=c_col, outline="")

        # Brazo proyectado en el mini radar
        ree_x = pip_cx + p3[0] * pip_sc
        ree_y = pip_cy - p3[1] * pip_sc
        cv.create_line(pip_cx, pip_cy, ree_x, ree_y, fill=COLOR_CYAN_ACCENT, width=1.5)
        cv.create_oval(pip_cx - 2, pip_cy - 2, pip_cx + 2, pip_cy + 2, fill="#0284c7", outline="")
        cv.create_oval(ree_x - 2, ree_y - 2, ree_x + 2, ree_y + 2, fill=COLOR_AMBER_WARNING, outline="")


# ==============================================================================
# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ TEMA 9: INSTRUMENTACIÓN VIRTUAL ANALÓGICA (VELOCÍMETROS / GAUGES)          ║
# ╚════════════════════════════════════════════════════════════════════════════╝
# ==============================================================================

    def draw_single_gauge(self, ax):
        """
        Dibuja un velocímetro semicircular analógico (180°) para la articulación ax:
          - Arco tricolor de tres segmentos (Verde: rango óptimo, Ámbar: advertencia, Rojo: límite).
          - Aguja analógica dinámica en tiempo real que apunta al ángulo medido.
          - Pivote central blanco y graduaciones en los extremos.
        """
        cv = self.canvas_gauges[ax]
        cv.delete("all")
        w = cv.winfo_width()
        h = cv.winfo_height()
        if w < 30 or h < 25:
            return

        cx = w / 2
        cy = h - 8
        radius = max(18.0, min(w / 2.3, h - 14.0))
        arc_thickness = 5

        box = [cx - radius, cy - radius, cx + radius, cy + radius]

        # 1. Segmento Verde
        cv.create_arc(box, start=120, extent=60, style=tk.ARC, outline=COLOR_GREEN_SUCCESS, width=arc_thickness)
        # 2. Segmento Ámbar
        cv.create_arc(box, start=60, extent=60, style=tk.ARC, outline=COLOR_AMBER_WARNING, width=arc_thickness)
        # 3. Segmento Rojo
        cv.create_arc(box, start=0, extent=60, style=tk.ARC, outline=COLOR_RED_DANGER, width=arc_thickness)

        # Mapeo normalizado del ángulo actual (0° a 360° en reductor)
        rmin, rmax = self.gauge_ranges[ax] if hasattr(self, 'gauge_ranges') else (-60.0, 60.0)
        ang = self.current_angles[ax]
        norm = (ang - rmin) / (rmax - rmin) if rmax > rmin else 0.0
        norm = max(0.0, min(1.0, norm))

        deg_canvas = 180.0 - (norm * 180.0)
        rad = math.radians(deg_canvas)

        # Aguja analógica roja
        needle_len = radius * 0.88
        nx = cx + needle_len * math.cos(rad)
        ny = cy - needle_len * math.sin(rad)
        cv.create_line(cx, cy, nx, ny, fill=COLOR_RED_DANGER, width=2.2, capstyle=tk.ROUND)

        # Pivote central
        cv.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill="#ffffff", outline=COLOR_BG_MAIN, width=1)

        # Textos de límites angulares y centro
        cv.create_text(cx - radius + 2, cy - 5, text=f"{rmin:.0f}°", fill="#475569", font=("Segoe UI", 6, "bold"))
        cv.create_text(cx + radius - 2, cy - 5, text=f"{rmax:.0f}°", fill="#475569", font=("Segoe UI", 6, "bold"))
        mid_val = (rmin + rmax) / 2.0
        cv.create_text(cx, cy - radius - 2, text=f"{mid_val:.0f}°", fill="#475569", font=("Segoe UI", 6, "bold"))


# ==============================================================================
# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ TEMA 10: CONSTRUCCIÓN DE LA INTERFAZ GRÁFICA (GUI TKINTER)                 ║
# ╚════════════════════════════════════════════════════════════════════════════╝
# ==============================================================================

    def setup_ui(self):
        """Construye todos los widgets, marcos, botones y paneles de la GUI."""
        style = ttk.Style(self)
        style.theme_use('clam')
        style.configure("TCombobox", fieldbackground="#ffffff", background="#e2e8f0", foreground="#0f172a")
        style.map("TCombobox", fieldbackground=[("readonly", "#ffffff")])

        # ----------------------------------------------------------------------
        # BARRA SUPERIOR: CONEXIÓN SERIAL & CONTROL WEBCAM
        # ----------------------------------------------------------------------
        top_bar = tk.Frame(self, bg=COLOR_BG_PANEL, padx=16, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
        top_bar.pack(fill=tk.X, padx=12, pady=(10, 6))

        tk.Label(top_bar, text="PUERTO COM:", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 4))
        self.combo_ports = ttk.Combobox(top_bar, width=11, state="readonly")
        self.combo_ports.pack(side=tk.LEFT, padx=3)

        btn_refresh = tk.Button(
            top_bar, text="Actualizar", bg="#e2e8f0", fg="#0f172a", activebackground="#cbd5e1",
            font=("Segoe UI", 8), bd=0, padx=6, relief=tk.FLAT, cursor="hand2", command=self.refresh_ports
        )
        btn_refresh.pack(side=tk.LEFT, padx=3)

        self.btn_connect = tk.Button(
            top_bar, text="Conectar ESP32", bg="#1e40af", fg="#ffffff", activebackground="#1d4ed8",
            font=("Segoe UI", 9, "bold"), padx=12, pady=2, bd=0, relief=tk.FLAT, cursor="hand2", command=self.toggle_connection
        )
        self.btn_connect.pack(side=tk.LEFT, padx=(8, 10))

        # Botón para abrir la Plantilla de Posición del Gato
        btn_template = tk.Button(
            top_bar, text="Plantilla Tablero", bg="#475569", fg="#ffffff", activebackground="#334155",
            font=("Segoe UI", 9, "bold"), padx=10, pady=2, bd=0, relief=tk.FLAT, cursor="hand2", command=self.show_board_position_template_modal
        )
        btn_template.pack(side=tk.LEFT, padx=(0, 6))

        # Botón para Calibrador de Casillas (Teach-In 1:1)
        btn_calib = tk.Button(
            top_bar, text="Calibrador 1:1", bg="#15803d", fg="#ffffff", activebackground="#166534",
            font=("Segoe UI", 9, "bold"), padx=10, pady=2, bd=0, relief=tk.FLAT, cursor="hand2", command=self.launch_calibrador_casillas
        )
        btn_calib.pack(side=tk.LEFT, padx=(0, 6))

        # Botón para cambiar o configurar características del Robot
        self.btn_robot_config = tk.Button(
            top_bar, text=f"Perfil: {self.active_profile_name[:18]}", bg="#0284c7", fg="#ffffff", activebackground="#0369a1",
            font=("Segoe UI", 9, "bold"), padx=10, pady=2, bd=0, relief=tk.FLAT, cursor="hand2", command=self.show_startup_robot_wizard
        )
        self.btn_robot_config.pack(side=tk.LEFT, padx=(0, 12))

        # Control WebCam
        tk.Label(top_bar, text="Cámara Web:", bg=COLOR_BG_PANEL, fg=COLOR_CYAN_ACCENT, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(6, 4))
        self.combo_cam = ttk.Combobox(top_bar, width=9, state="readonly", values=["Cámara 0", "Cámara 1", "Cámara 2"])
        self.combo_cam.set("Cámara 0")
        self.combo_cam.pack(side=tk.LEFT, padx=3)

        self.btn_camera = tk.Button(
            top_bar, text="Iniciar Video", bg="#15803d", fg="#ffffff", activebackground="#166534",
            font=("Segoe UI", 9, "bold"), padx=10, pady=2, bd=0, relief=tk.FLAT, cursor="hand2", command=self.toggle_camera
        )
        self.btn_camera.pack(side=tk.LEFT, padx=4)

        self.chk_auto_vision_var = tk.BooleanVar(value=True)
        chk_vis = tk.Checkbutton(
            top_bar, text="Detección Automática", variable=self.chk_auto_vision_var,
            bg=COLOR_BG_PANEL, fg="#0f172a", selectcolor="#ffffff", activebackground=COLOR_BG_PANEL,
            activeforeground=COLOR_GREEN_SUCCESS, font=("Segoe UI", 8, "bold")
        )
        chk_vis.pack(side=tk.LEFT, padx=10)

        self.lbl_status = tk.Label(top_bar, text="● DESCONECTADO", bg=COLOR_BG_PANEL, fg="#64748b", font=("Segoe UI", 9, "bold"))
        self.lbl_status.pack(side=tk.RIGHT, padx=4)

        # ----------------------------------------------------------------------
        # BARRA DE CONTROL DE POSICIÓN CERO (CALIBRACIÓN DE ORIGEN 0.0°)
        # ----------------------------------------------------------------------
        self.zero_bar = tk.Frame(self, bg=COLOR_BG_PANEL, padx=16, pady=5, highlightthickness=1, highlightbackground=COLOR_BORDER)
        self.zero_bar.pack(fill=tk.X, padx=12, pady=(0, 6))

        tk.Label(
            self.zero_bar, text="POSICIÓN DE ORIGEN (CERO):",
            bg=COLOR_BG_PANEL, fg=COLOR_ACCENT, font=("Segoe UI", 9, "bold")
        ).pack(side=tk.LEFT, padx=(0, 8))

        self.lbl_zero_status = tk.Label(
            self.zero_bar, text="Motores Libres (A mano)",
            bg="#fef3c7", fg="#92400e", relief=tk.SOLID, bd=1, font=("Segoe UI", 8, "bold"), padx=8, pady=2
        )
        self.lbl_zero_status.pack(side=tk.LEFT, padx=(0, 10))

        self.btn_save_game = tk.Button(
            self.zero_bar, text="Guardar Cero (0.0°)", bg="#15803d", fg="#ffffff",
            activebackground="#047857", font=("Segoe UI", 8, "bold"), padx=10, pady=2, bd=0,
            cursor="hand2", command=self.cmd_save_game
        )
        self.btn_save_game.pack(side=tk.LEFT, padx=3)

        self.btn_load_game = tk.Button(
            self.zero_bar, text="Cargar Cero (0.0°)", bg="#1e40af", fg="#ffffff",
            activebackground="#0369a1", font=("Segoe UI", 8, "bold"), padx=10, pady=2, bd=0,
            cursor="hand2", command=self.cmd_load_game
        )
        self.btn_load_game.pack(side=tk.LEFT, padx=3)

        self.btn_toggle_motors = tk.Button(
            self.zero_bar, text="Bloquear Motores", bg="#475569", fg="#ffffff",
            activebackground="#475569", font=("Segoe UI", 8, "bold"), padx=10, pady=2, bd=0,
            cursor="hand2", command=self.toggle_motors_lock_free
        )
        self.btn_toggle_motors.pack(side=tk.LEFT, padx=3)

        self.btn_zero_wizard = tk.Button(
            self.zero_bar, text="Asistente Cero", bg="#0284c7", fg="#ffffff",
            activebackground="#6d28d9", font=("Segoe UI", 8, "bold"), padx=10, pady=2, bd=0,
            cursor="hand2", command=self.show_zero_calibration_modal
        )
        self.btn_zero_wizard.pack(side=tk.LEFT, padx=(10, 4))

        self.lbl_zero_quick_angles = tk.Label(
            self.zero_bar, text="M1: +0.00° | M2: +0.00° | M3: +0.00°",
            bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Consolas", 9, "bold")
        )
        self.lbl_zero_quick_angles.pack(side=tk.RIGHT, padx=4)

        # Banner de Alerta Crítica del Protocolo Anti-Choque / Atasco por Encoder AS5600
        self.frame_stall_banner = tk.Frame(self, bg="#7f1d1d", padx=14, pady=6, highlightthickness=1, highlightbackground="#ef4444")
        self.lbl_stall_banner = tk.Label(
            self.frame_stall_banner,
            text="[ALERTA ANTI-CHOQUE] Motor bloqueado sin movimiento en encoder.",
            bg="#7f1d1d", fg="#ffffff", font=("Segoe UI", 9, "bold")
        )
        self.lbl_stall_banner.pack(side=tk.LEFT)
        self.btn_dismiss_stall = tk.Button(
            self.frame_stall_banner, text="Despejar Alerta", bg="#b91c1c", fg="#ffffff",
            activebackground="#991b1b", font=("Segoe UI", 8, "bold"), padx=10, pady=2, bd=0,
            cursor="hand2", command=lambda: self.frame_stall_banner.pack_forget()
        )
        self.btn_dismiss_stall.pack(side=tk.RIGHT)

        # ----------------------------------------------------------------------
        # CUERPO PRINCIPAL CON 3 PANELES (IZQUIERDA - CENTRO - DERECHA)
        # ----------------------------------------------------------------------
        self.main_frame = tk.Frame(self, bg=COLOR_BG_MAIN)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 8))
        main_frame = self.main_frame

        # ======================================================================
        # PANEL IZQUIERDO: WEBCAM + TABLERO INTERACTIVO DEL GATO
        # ======================================================================
        p_left = tk.Frame(main_frame, bg=COLOR_BG_PANEL, padx=12, pady=10, highlightthickness=1, highlightbackground=COLOR_BORDER, width=410)
        p_left.pack(side=tk.LEFT, fill=tk.BOTH, padx=(0, 6))
        p_left.pack_propagate(False)

        tk.Label(p_left, text="Visión Artificial (WebCam)", bg=COLOR_BG_PANEL, fg=COLOR_CYAN_ACCENT, font=("Segoe UI", 10, "bold")).pack(anchor=tk.W)

        self.canvas_cam = tk.Canvas(p_left, bg="#05070a", width=386, height=270, highlightthickness=1, highlightbackground=COLOR_BORDER_FOCUS)
        self.canvas_cam.pack(pady=6)

        self.lbl_vision_info = tk.Label(p_left, text="Visión: En espera...", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8))
        self.lbl_vision_info.pack(anchor=tk.W, pady=(0, 6))

        tk.Label(p_left, text="Tablero de Juego (3x3)", bg=COLOR_BG_PANEL, fg=COLOR_GREEN_SUCCESS, font=("Segoe UI", 10, "bold")).pack(anchor=tk.W, pady=(4, 2))

        # Marcador de resultados
        score_box = tk.Frame(p_left, bg=COLOR_BG_CARD, padx=8, pady=4, highlightthickness=1, highlightbackground=COLOR_BORDER_FOCUS)
        score_box.pack(fill=tk.X, pady=4)
        self.lbl_scores = tk.Label(
            score_box, text="Humano (X): 0   |   Robot (O): 0   |   Empates: 0",
            bg=COLOR_BG_CARD, fg="#0f172a", font=("Segoe UI", 8, "bold")
        )
        self.lbl_scores.pack()

        # Grilla 3x3 de botones del tablero
        grid_frame = tk.Frame(p_left, bg=COLOR_BORDER_FOCUS, padx=3, pady=3)
        grid_frame.pack(pady=4)

        self.cell_buttons = []
        for i in range(9):
            btn = tk.Button(
                grid_frame, text=" ", bg="#ffffff", fg="#0f172a", activebackground="#e2e8f0", font=("Segoe UI", 18, "bold"), width=3, height=1, relief=tk.SOLID, bd=1, cursor="hand2",
                command=lambda idx=i: self.on_cell_clicked(idx)
            )
            btn.grid(row=i//3, column=i%3, padx=2, pady=2)
            self.cell_buttons.append(btn)

        btn_reset = tk.Button(
            p_left, text="Reiniciar Partida", bg="#e2e8f0", fg="#0f172a", activebackground="#cbd5e1", font=("Segoe UI", 8, "bold"), pady=4, relief=tk.SOLID, bd=1, cursor="hand2", command=self.reset_game
        )
        btn_reset.pack(fill=tk.X, pady=(6, 2))

        btn_sim_robot = tk.Button(
            p_left, text="Turno del Robot", bg="#1e40af", fg="#ffffff", activebackground="#1d4ed8",
            font=("Segoe UI", 8, "bold"), pady=4, bd=0, relief=tk.FLAT, cursor="hand2", command=self.trigger_robot_play
        )
        btn_sim_robot.pack(fill=tk.X, pady=(0, 2))

        btn_open_tpl_left = tk.Button(
            p_left, text="Verificar Plantilla 2D", bg="#475569", fg="#ffffff", activebackground="#334155",
            font=("Segoe UI", 8, "bold"), pady=4, bd=0, relief=tk.FLAT, cursor="hand2", command=self.show_board_position_template_modal
        )
        btn_open_tpl_left.pack(fill=tk.X, pady=(0, 0))

        # ======================================================================
        # PANEL CENTRAL: CINEMÁTICA CARTESIANA (IK) Y SIMULADOR 3D
        # ======================================================================
        p_center = tk.Frame(main_frame, bg=COLOR_BG_PANEL, padx=12, pady=10, highlightthickness=1, highlightbackground=COLOR_BORDER)
        p_center.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=3)

        tk.Label(p_center, text="Control Cartesiano (Cinemática Inversa MK2)", bg=COLOR_BG_PANEL, fg=COLOR_AMBER_WARNING, font=("Segoe UI", 10, "bold")).pack(anchor=tk.W)

        # Caja de entrada de coordenadas cartesianas (X, Y, Z) y Control de Movimiento
        ik_input_box = tk.Frame(p_center, bg=COLOR_BG_CARD, padx=10, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER_FOCUS)
        ik_input_box.pack(fill=tk.X, pady=6)

        # Fila 1: Selector de Modo de Coordenadas (Relativo Cero Home / Absoluto Base)
        r_mode = tk.Frame(ik_input_box, bg=COLOR_BG_CARD)
        r_mode.pack(fill=tk.X, pady=(0, 4))
        self.var_coord_mode = tk.StringVar(value="rel")
        tk.Label(r_mode, text="Sistema:", bg=COLOR_BG_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=(0, 4))
        rb_rel = tk.Radiobutton(
            r_mode, text="Relativo Cero Home (0,0,0)", variable=self.var_coord_mode, value="rel",
            bg=COLOR_BG_CARD, fg="#0f172a", selectcolor="#e2e8f0", activebackground=COLOR_BG_CARD,
            font=("Segoe UI", 8, "bold"), command=lambda: self.set_coord_mode(True)
        )
        rb_rel.pack(side=tk.LEFT, padx=(0, 8))
        rb_abs = tk.Radiobutton(
            r_mode, text="Absoluto Base (Mesa Z=-29.1)", variable=self.var_coord_mode, value="abs",
            bg=COLOR_BG_CARD, fg="#64748b", selectcolor="#e2e8f0", activebackground=COLOR_BG_CARD,
            font=("Segoe UI", 8), command=lambda: self.set_coord_mode(False)
        )
        rb_abs.pack(side=tk.LEFT)

        # Fila 2: Inputs numéricos X, Y, Z y botón de cálculo IK
        r_inputs = tk.Frame(ik_input_box, bg=COLOR_BG_CARD)
        r_inputs.pack(fill=tk.X, pady=(2, 4))

        tk.Label(r_inputs, text="Destino:", bg=COLOR_BG_CARD, fg="#0284c7", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=(0, 4))
        tk.Label(r_inputs, text="X (cm):", bg=COLOR_BG_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 2))
        self.ent_x = tk.Entry(r_inputs, width=6, bg="#ffffff", fg="#0f172a", font=("Consolas", 10, "bold"), bd=1, relief=tk.SOLID, insertbackground="#0f172a")
        self.ent_x.insert(0, f"{self.cart_x:.1f}")
        self.ent_x.pack(side=tk.LEFT, padx=(0, 10))

        tk.Label(r_inputs, text="Y (cm):", bg=COLOR_BG_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 2))
        self.ent_y = tk.Entry(r_inputs, width=6, bg="#ffffff", fg="#0f172a", font=("Consolas", 10, "bold"), bd=1, relief=tk.SOLID, insertbackground="#0f172a")
        self.ent_y.insert(0, f"{self.cart_y:.1f}")
        self.ent_y.pack(side=tk.LEFT, padx=(0, 10))

        tk.Label(r_inputs, text="Z (cm):", bg=COLOR_BG_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 2))
        self.ent_z = tk.Entry(r_inputs, width=6, bg="#ffffff", fg="#0f172a", font=("Consolas", 10, "bold"), bd=1, relief=tk.SOLID, insertbackground="#0f172a")
        self.ent_z.insert(0, f"{self.cart_z:.1f}")
        self.ent_z.pack(side=tk.LEFT, padx=(0, 12))

        btn_ik_go = tk.Button(
            r_inputs, text="Calcular IK y Mover", bg="#1e40af", fg="#ffffff", activebackground="#1d4ed8",
            font=("Segoe UI", 8, "bold"), padx=10, pady=3, bd=0, relief=tk.FLAT, cursor="hand2", command=self.cmd_solve_ik_and_move
        )
        btn_ik_go.pack(side=tk.RIGHT)

        # Fila 3: Botones de Paso Rápido Jog (Subir / Bajar / Movimiento Cartesiano)
        r_jog = tk.Frame(ik_input_box, bg=COLOR_BG_CARD)
        r_jog.pack(fill=tk.X, pady=(3, 2))

        tk.Label(r_jog, text="Paso Z:", bg=COLOR_BG_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=(0, 3))
        btn_z_up5 = tk.Button(r_jog, text="▲ +5cm", bg="#0284c7", fg="#ffffff", activebackground="#0369a1", font=("Segoe UI", 7, "bold"), padx=4, pady=1, bd=0, relief=tk.FLAT, cursor="hand2", command=lambda: self.jog_cartesian(dz=5.0))
        btn_z_up5.pack(side=tk.LEFT, padx=1)
        btn_z_up1 = tk.Button(r_jog, text="▲ +1cm", bg="#0284c7", fg="#ffffff", activebackground="#0369a1", font=("Segoe UI", 7, "bold"), padx=4, pady=1, bd=0, relief=tk.FLAT, cursor="hand2", command=lambda: self.jog_cartesian(dz=1.0))
        btn_z_up1.pack(side=tk.LEFT, padx=1)
        btn_z_dn1 = tk.Button(r_jog, text="▼ -1cm", bg="#d97706", fg="#ffffff", activebackground="#b45309", font=("Segoe UI", 7, "bold"), padx=4, pady=1, bd=0, relief=tk.FLAT, cursor="hand2", command=lambda: self.jog_cartesian(dz=-1.0))
        btn_z_dn1.pack(side=tk.LEFT, padx=1)
        btn_z_dn5 = tk.Button(r_jog, text="▼ -5cm", bg="#d97706", fg="#ffffff", activebackground="#b45309", font=("Segoe UI", 7, "bold"), padx=4, pady=1, bd=0, relief=tk.FLAT, cursor="hand2", command=lambda: self.jog_cartesian(dz=-5.0))
        btn_z_dn5.pack(side=tk.LEFT, padx=(1, 6))

        tk.Label(r_jog, text="Paso X:", bg=COLOR_BG_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=(0, 2))
        btn_x_b5 = tk.Button(r_jog, text="X -5", bg="#475569", fg="#ffffff", activebackground="#334155", font=("Segoe UI", 7, "bold"), padx=3, pady=1, bd=0, relief=tk.FLAT, cursor="hand2", command=lambda: self.jog_cartesian(dx=-5.0))
        btn_x_b5.pack(side=tk.LEFT, padx=1)
        btn_x_back = tk.Button(r_jog, text="X -2", bg="#475569", fg="#ffffff", activebackground="#334155", font=("Segoe UI", 7, "bold"), padx=3, pady=1, bd=0, relief=tk.FLAT, cursor="hand2", command=lambda: self.jog_cartesian(dx=-2.0))
        btn_x_back.pack(side=tk.LEFT, padx=1)
        btn_x_fwd = tk.Button(r_jog, text="X +2", bg="#475569", fg="#ffffff", activebackground="#334155", font=("Segoe UI", 7, "bold"), padx=3, pady=1, bd=0, relief=tk.FLAT, cursor="hand2", command=lambda: self.jog_cartesian(dx=2.0))
        btn_x_fwd.pack(side=tk.LEFT, padx=1)
        btn_x_f5 = tk.Button(r_jog, text="X +5", bg="#475569", fg="#ffffff", activebackground="#334155", font=("Segoe UI", 7, "bold"), padx=3, pady=1, bd=0, relief=tk.FLAT, cursor="hand2", command=lambda: self.jog_cartesian(dx=5.0))
        btn_x_f5.pack(side=tk.LEFT, padx=(1, 4))

        btn_zero_fast = tk.Button(r_jog, text="⟲ Cero (0,0,0)", bg="#15803d", fg="#ffffff", activebackground="#166534", font=("Segoe UI", 7, "bold"), padx=6, pady=1, bd=0, relief=tk.FLAT, cursor="hand2", command=self.cmd_home)
        btn_zero_fast.pack(side=tk.RIGHT)

        # Fila 3b: Paso Jog Eje Y (Giro Izquierda / Derecha Incremental)
        r_jog_y = tk.Frame(ik_input_box, bg=COLOR_BG_CARD)
        r_jog_y.pack(fill=tk.X, pady=(2, 2))

        tk.Label(r_jog_y, text="Paso Y (Incremental):", bg=COLOR_BG_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=(0, 3))
        for dy_val in [-20.0, -10.0, -5.0, -2.0]:
            b = tk.Button(
                r_jog_y, text=f"Y {dy_val:+.0f}", bg="#475569" if abs(dy_val) <= 2 else "#3b82f6",
                fg="#ffffff", activebackground="#1d4ed8", font=("Segoe UI", 7, "bold"),
                padx=3, pady=1, bd=0, relief=tk.FLAT, cursor="hand2",
                command=lambda d=dy_val: self.jog_cartesian(dy=d)
            )
            b.pack(side=tk.LEFT, padx=1)
        for dy_val in [2.0, 5.0, 10.0, 20.0]:
            b = tk.Button(
                r_jog_y, text=f"Y +{dy_val:.0f}", bg="#475569" if abs(dy_val) <= 2 else "#3b82f6",
                fg="#ffffff", activebackground="#1d4ed8", font=("Segoe UI", 7, "bold"),
                padx=3, pady=1, bd=0, relief=tk.FLAT, cursor="hand2",
                command=lambda d=dy_val: self.jog_cartesian(dy=d)
            )
            b.pack(side=tk.LEFT, padx=1)

        # Fila 4: Diagnóstico IK / Error Posicional
        r_diag = tk.Frame(ik_input_box, bg=COLOR_BG_CARD)
        r_diag.pack(fill=tk.X, pady=(6, 0))

        self.lbl_ik_status = tk.Label(
            r_diag, text="Estado IK: Listo | Cero Home (0, 0, 0) calibrado sobre caja",
            bg=COLOR_BG_CARD, fg=COLOR_GREEN_SUCCESS, font=("Consolas", 8)
        )
        self.lbl_ik_status.pack(side=tk.LEFT)

        # Botones de accesos rápidos a posiciones clave: Exclusivamente HOME (Cero calibrado)
        quick_frame = tk.Frame(p_center, bg=COLOR_BG_PANEL)
        quick_frame.pack(fill=tk.X, pady=4)

        tk.Label(quick_frame, text="POSICIÓN CLAVE:", bg=COLOR_BG_PANEL, fg="#475569", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=(0, 6))

        btn_p_home = tk.Button(
            quick_frame, text="HOME (Cero 0.0°)", bg="#1e40af", fg="#ffffff", activebackground="#1d4ed8",
            font=("Segoe UI", 8, "bold"), padx=12, pady=3, relief=tk.FLAT, cursor="hand2", command=self.cmd_home
        )
        btn_p_home.pack(side=tk.LEFT, padx=2)

        # Caja interactiva para las 9 Casillas del Tablero de Gato (Cinemática Inversa MK2)
        cell_box = tk.Frame(p_center, bg=COLOR_BG_CARD, padx=8, pady=5, highlightthickness=1, highlightbackground=COLOR_BORDER_FOCUS)
        cell_box.pack(fill=tk.X, pady=(2, 6))

        r_cbox_top = tk.Frame(cell_box, bg=COLOR_BG_CARD)
        r_cbox_top.pack(fill=tk.X)
        tk.Label(r_cbox_top, text="Casillas del Tablero (1 a 9)", bg=COLOR_BG_CARD, fg=COLOR_GREEN_SUCCESS, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)

        btn_tpl_top = tk.Button(
            r_cbox_top, text="Plantilla 2D", bg="#475569", fg="#ffffff", activebackground="#334155",
            font=("Segoe UI", 7, "bold"), bd=0, padx=6, pady=1, cursor="hand2", command=self.show_board_position_template_modal
        )
        btn_tpl_top.pack(side=tk.LEFT, padx=(8, 0))

        btn_calib_top = tk.Button(
            r_cbox_top, text="Calibrador 1:1", bg="#15803d", fg="#ffffff", activebackground="#166534",
            font=("Segoe UI", 7, "bold"), bd=0, padx=6, pady=1, cursor="hand2", command=self.launch_calibrador_casillas
        )
        btn_calib_top.pack(side=tk.LEFT, padx=(4, 0))

        # Selector de Modo de Acción para las 9 Casillas
        self.cell_action_mode = tk.StringVar(value="place")
        rb_place = tk.Radiobutton(r_cbox_top, text="Tablero (Z=3cm)", variable=self.cell_action_mode, value="place",
                                  bg=COLOR_BG_CARD, fg="#0f172a", selectcolor="#ffffff", activebackground=COLOR_BG_CARD,
                                  activeforeground=COLOR_GREEN_SUCCESS, font=("Segoe UI", 7, "bold"))
        rb_place.pack(side=tk.RIGHT, padx=2)
        rb_hover = tk.Radiobutton(r_cbox_top, text="Hover (Z=8cm)", variable=self.cell_action_mode, value="hover",
                                  bg=COLOR_BG_CARD, fg="#0f172a", selectcolor="#ffffff", activebackground=COLOR_BG_CARD,
                                  activeforeground=COLOR_CYAN_ACCENT, font=("Segoe UI", 7, "bold"))
        rb_hover.pack(side=tk.RIGHT, padx=2)
        rb_pnp = tk.Radiobutton(r_cbox_top, text="Pick & Place", variable=self.cell_action_mode, value="pick_place",
                                bg=COLOR_BG_CARD, fg="#0f172a", selectcolor="#ffffff", activebackground=COLOR_BG_CARD,
                                activeforeground=COLOR_AMBER_WARNING, font=("Segoe UI", 7, "bold"))
        rb_pnp.pack(side=tk.RIGHT, padx=2)

        # Botones de las 9 casillas distribuidos equitativamente
        r_cbox_btns = tk.Frame(cell_box, bg=COLOR_BG_CARD)
        r_cbox_btns.pack(fill=tk.X, pady=(4, 2))
        self.cell_buttons = []
        for c_idx in range(9):
            btn_c = tk.Button(
                r_cbox_btns, text=f"Casilla {c_idx+1}", bg="#ffffff", fg="#1e40af", activebackground="#1e40af",
                activeforeground="#ffffff", relief=tk.SOLID, bd=1, font=("Segoe UI", 8, "bold"), padx=2, pady=3,
                cursor="hand2", command=lambda idx=c_idx: self.cmd_goto_cell(idx)
            )
            btn_c.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)
            self.cell_buttons.append(btn_c)

        # Barra de Control de Secuencia Automatizada (1 a 9) y Adaptación a Límites
        r_tour_bar = tk.Frame(cell_box, bg=COLOR_BG_CARD)
        r_tour_bar.pack(fill=tk.X, pady=(4, 2))

        self.btn_tour_start = tk.Button(
            r_tour_bar, text="Recorrer 1 a 9", bg="#15803d", fg="#ffffff",
            activebackground="#0369a1", font=("Segoe UI", 8, "bold"), bd=0, padx=8, pady=2,
            cursor="hand2", command=self.start_cells_tour_sequence
        )
        self.btn_tour_start.pack(side=tk.LEFT, padx=(0, 4))

        self.btn_tour_stop = tk.Button(
            r_tour_bar, text="Detener", bg="#b91c1c", fg="#ffffff",
            activebackground="#334155", font=("Segoe UI", 8, "bold"), bd=0, padx=8, pady=2,
            cursor="hand2", state=tk.DISABLED, command=self.stop_cells_tour_sequence
        )
        self.btn_tour_stop.pack(side=tk.LEFT, padx=3)

        btn_adapt = tk.Button(
            r_tour_bar, text="Adaptar a Límites", bg="#0284c7", fg="#ffffff",
            activebackground="#0d9488", font=("Segoe UI", 8, "bold"), bd=0, padx=8, pady=2,
            cursor="hand2", command=lambda: self.adapt_cells_to_current_limits(show_msg=True)
        )
        btn_adapt.pack(side=tk.LEFT, padx=3)

        tk.Label(r_tour_bar, text="Pausa:", bg=COLOR_BG_CARD, fg="#94a3b8", font=("Segoe UI", 7)).pack(side=tk.LEFT, padx=(6, 2))
        self.ent_tour_pause = tk.Entry(r_tour_bar, width=4, bg="#ffffff", fg="#0f172a", font=("Consolas", 8, "bold"), bd=1, relief=tk.SOLID)
        self.ent_tour_pause.insert(0, "1.5")
        self.ent_tour_pause.pack(side=tk.LEFT)
        tk.Label(r_tour_bar, text="s", bg=COLOR_BG_CARD, fg="#64748b", font=("Segoe UI", 7)).pack(side=tk.LEFT, padx=(2, 0))

        self.lbl_cell_angles_info = tk.Label(
            cell_box,
            text="Haga clic en una casilla o use 'Recorrer 1 a 9' para ver cada postura adaptada a los límites.",
            bg=COLOR_BG_CARD, fg=COLOR_TEXT_MUTED, font=("Consolas", 8)
        )
        self.lbl_cell_angles_info.pack(anchor=tk.W, pady=(2, 0))

        # Encabezado del Canvas 3D con presets de cámara orbital
        sim_header = tk.Frame(p_center, bg=COLOR_BG_PANEL)
        sim_header.pack(fill=tk.X, pady=(6, 2))
        tk.Label(sim_header, text="Simulación 3D del Manipulador MK2", bg=COLOR_BG_PANEL, fg=COLOR_CYAN_ACCENT, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)

        btn_v_user = tk.Button(sim_header, text="Vista Usuario (POV)", bg="#1e40af", fg="#ffffff", font=("Segoe UI", 7, "bold"), bd=0, padx=6, pady=1, cursor="hand2", command=self.view_user_pov)
        btn_v_user.pack(side=tk.RIGHT, padx=2)
        btn_v_iso = tk.Button(sim_header, text="Isométrica", bg="#e2e8f0", fg="#0f172a", relief=tk.SOLID, bd=1, font=("Segoe UI", 7), padx=6, pady=1, cursor="hand2", command=self.view_isometric)
        btn_v_iso.pack(side=tk.RIGHT, padx=2)
        btn_v_top = tk.Button(sim_header, text="Planta", bg="#e2e8f0", fg="#0f172a", relief=tk.SOLID, bd=1, font=("Segoe UI", 7), padx=6, pady=1, cursor="hand2", command=self.view_top)
        btn_v_top.pack(side=tk.RIGHT, padx=2)
        btn_v_side = tk.Button(sim_header, text="Perfil", bg="#e2e8f0", fg="#0f172a", relief=tk.SOLID, bd=1, font=("Segoe UI", 7), padx=6, pady=1, cursor="hand2", command=self.view_side)
        btn_v_side.pack(side=tk.RIGHT, padx=2)

        self.canvas_sim = tk.Canvas(p_center, bg="#ffffff", height=340, highlightthickness=1, highlightbackground=COLOR_BORDER, cursor="fleur")
        self.canvas_sim.pack(fill=tk.BOTH, expand=True)

        # Enlace de eventos de ratón para rotación orbital y zoom
        self.canvas_sim.bind("<ButtonPress-1>", self.on_sim_mouse_down)
        self.canvas_sim.bind("<B1-Motion>", self.on_sim_mouse_drag)
        self.canvas_sim.bind("<MouseWheel>", self.on_sim_mouse_wheel)

        # ======================================================================
        # PANEL DERECHO: VELOCÍMETROS ANALÓGICOS & MONITOREO DINÁMICO (SCROLLABLE)
        # ======================================================================
        p_right = tk.Frame(main_frame, bg=COLOR_BG_PANEL, padx=4, pady=4, highlightthickness=1, highlightbackground=COLOR_BORDER, width=420)
        p_right.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(6, 0))
        p_right.pack_propagate(False)

        header_frame = tk.Frame(p_right, bg=COLOR_BG_PANEL)
        header_frame.pack(fill=tk.X, pady=(0, 3), padx=2)
        tk.Label(header_frame, text="Monitores Articulares (AS5600)", bg=COLOR_BG_PANEL, fg=COLOR_CYAN_ACCENT, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
        btn_test_enc = tk.Button(
            header_frame, text="Diagnóstico Encoders", bg="#475569", fg="#ffffff",
            activebackground="#0d9488", font=("Segoe UI", 7, "bold"), bd=0, padx=6, pady=2,
            cursor="hand2", command=self.show_encoder_diagnostics_modal
        )
        btn_test_enc.pack(side=tk.RIGHT)

        # Contenedor con barra de desplazamiento vertical integrada (Garantiza acceso completo a M1, M2 y M3)
        canvas_p = tk.Canvas(p_right, bg=COLOR_BG_PANEL, highlightthickness=0)
        scrollbar_p = ttk.Scrollbar(p_right, orient="vertical", command=canvas_p.yview)
        canvas_p.configure(yscrollcommand=scrollbar_p.set)
        scrollbar_p.pack(side=tk.RIGHT, fill=tk.Y)
        canvas_p.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        p_inner = tk.Frame(canvas_p, bg=COLOR_BG_PANEL)
        window_p = canvas_p.create_window((0, 0), window=p_inner, anchor="nw")

        def _on_inner_config(e):
            canvas_p.configure(scrollregion=canvas_p.bbox("all"))

        def _on_canvas_config(e):
            canvas_p.itemconfig(window_p, width=e.width)

        p_inner.bind("<Configure>", _on_inner_config)
        canvas_p.bind("<Configure>", _on_canvas_config)

        def _on_mousewheel_right(event):
            try:
                w_str = str(event.widget)
                if w_str.startswith(str(p_right)):
                    canvas_p.yview_scroll(int(-1 * (event.delta / 120)), "units")
            except Exception:
                pass

        self.bind_all("<MouseWheel>", _on_mousewheel_right, add="+")

        # Rangos seguros de instrumentación (Reductor 31:1 con límites de seguridad mecánica)
        self.gauge_ranges = [lim for lim in self.kin.limits]
        self.canvas_gauges = []
        self.gauge_labels = []
        self.sliders_ax = []
        self.lbl_slider_min = []
        self.lbl_slider_max = []
        self.preset_btn_list = []
        self.lbl_turns_nema = []

        gauge_titles = [
            "M1: BASE (YAW - 31:1)",
            "M2: HOMBRO (PITCH - 31:1)",
            "M3: CODO (PITCH - 31:1)"
        ]

        # Panel de Límites de Seguridad Configurables
        lim_box = tk.Frame(p_inner, bg=COLOR_BG_CARD, padx=6, pady=4, highlightthickness=1, highlightbackground=COLOR_BORDER_FOCUS)
        lim_box.pack(fill=tk.X, pady=(0, 3))

        r_lim_header = tk.Frame(lim_box, bg=COLOR_BG_CARD)
        r_lim_header.pack(fill=tk.X)
        tk.Label(r_lim_header, text="Límites de Seguridad Articular", bg=COLOR_BG_CARD, fg=COLOR_AMBER_WARNING, font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT)

        self.lbl_limits_status = tk.Label(r_lim_header, text="● Límites activos", bg=COLOR_BG_CARD, fg="#15803d", font=("Segoe UI", 7, "bold"))
        self.lbl_limits_status.pack(side=tk.RIGHT)

        self.ent_lim_min = []
        self.ent_lim_max = []
        lim_names = ["M1 Base:", "M2 Hombro:", "M3 Codo:"]
        for m_i in range(3):
            r_lim = tk.Frame(lim_box, bg=COLOR_BG_CARD)
            r_lim.pack(fill=tk.X, pady=1)
            tk.Label(r_lim, text=lim_names[m_i], width=11, anchor=tk.W, bg=COLOR_BG_CARD, fg="#0f172a", font=("Consolas", 8, "bold")).pack(side=tk.LEFT)
            tk.Label(r_lim, text="Min:", bg=COLOR_BG_CARD, fg="#64748b", font=("Segoe UI", 7)).pack(side=tk.LEFT)
            e_min = tk.Entry(r_lim, width=6, bg="#ffffff", fg="#0f172a", font=("Consolas", 8, "bold"), bd=1, relief=tk.SOLID)
            e_min.insert(0, f"{self.kin.limits[m_i][0]:.1f}")
            e_min.pack(side=tk.LEFT, padx=(2, 6))
            self.ent_lim_min.append(e_min)

            tk.Label(r_lim, text="Max:", bg=COLOR_BG_CARD, fg="#64748b", font=("Segoe UI", 7)).pack(side=tk.LEFT)
            e_max = tk.Entry(r_lim, width=6, bg="#ffffff", fg="#0f172a", font=("Consolas", 8, "bold"), bd=1, relief=tk.SOLID)
            e_max.insert(0, f"{self.kin.limits[m_i][1]:.1f}")
            e_max.pack(side=tk.LEFT, padx=2)
            self.ent_lim_max.append(e_max)

        r_nut = tk.Frame(lim_box, bg="#f1f5f9", padx=4, pady=2, highlightthickness=1, highlightbackground=COLOR_BORDER)
        r_nut.pack(fill=tk.X, pady=(2, 2))
        tk.Label(r_nut, text="Anticolisión M2-M3 (Tuerca):", bg="#f1f5f9", fg="#b45309", font=("Segoe UI", 7, "bold")).pack(side=tk.LEFT)
        self.lbl_nut_status = tk.Label(r_nut, text=f"ACTIVA (Min Codo ≥ {self.kin.limits[2][0]:.0f}°)", bg="#f1f5f9", fg="#15803d", font=("Consolas", 7, "bold"))
        self.lbl_nut_status.pack(side=tk.RIGHT)

        btn_apply_limits = tk.Button(
            lim_box, text="Aplicar Límites a NVS", bg="#1e40af", fg="#ffffff",
            activebackground="#0d9488", font=("Segoe UI", 8, "bold"), pady=2, bd=0, relief=tk.FLAT, cursor="hand2",
            command=self.apply_user_limits
        )
        btn_apply_limits.pack(fill=tk.X, pady=(3, 1))

        # Presets seguros acordes al rango exacto de cada articulación
        joint_presets = [
            [-60, -30, 0, 30, 60],     # M1: Base [-60°, 60°]
            [-60, -30, 0, 30, 60],     # M2: Hombro [-60°, 60°]
            [0, 20, 40, 60, 80]        # M3: Codo [0°, 85°]
        ]

        for ax in range(3):
            g_box = tk.Frame(p_inner, bg=COLOR_BG_CARD, padx=6, pady=3, highlightthickness=1, highlightbackground=COLOR_BORDER_FOCUS)
            g_box.pack(fill=tk.X, pady=2)

            r_title = tk.Frame(g_box, bg=COLOR_BG_CARD)
            r_title.pack(fill=tk.X)
            tk.Label(r_title, text=gauge_titles[ax], bg=COLOR_BG_CARD, fg="#0f172a", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT)

            lbl_deg = tk.Label(r_title, text="0.0°", bg=COLOR_BG_CARD, fg=COLOR_CYAN_ACCENT, font=("Consolas", 9, "bold"))
            lbl_deg.pack(side=tk.RIGHT)
            self.gauge_labels.append(lbl_deg)

            # Telemetría unificada en una sola línea compacta
            r_telem = tk.Frame(g_box, bg=COLOR_BG_CARD)
            r_telem.pack(fill=tk.X, pady=(1, 2))
            lbl_badge = tk.Label(
                r_telem, text="● ENCODER...",
                bg="#dcfce7", fg="#15803d", relief=tk.SOLID, bd=1, font=("Segoe UI", 7, "bold"), padx=4, pady=0
            )
            lbl_badge.pack(side=tk.LEFT)
            self.encoder_badge_labels.append(lbl_badge)

            lbl_turns = tk.Label(
                r_telem, text="NEMA: 0.00 / 31 revs",
                bg=COLOR_BG_CARD, fg="#475569", font=("Consolas", 7, "bold"), padx=4
            )
            lbl_turns.pack(side=tk.LEFT)
            self.lbl_turns_nema.append(lbl_turns)

            lbl_raw = tk.Label(r_telem, text="Raw: ----", bg=COLOR_BG_CARD, fg="#64748b", font=("Consolas", 7))
            lbl_raw.pack(side=tk.RIGHT)
            self.encoder_raw_labels.append(lbl_raw)

            # Tacómetro Analógico Compacto (44px)
            cv_g = tk.Canvas(g_box, bg="#ffffff", height=44, highlightthickness=0)
            cv_g.pack(fill=tk.X, pady=(0, 2))
            self.canvas_gauges.append(cv_g)

            # Fila de Slider: [Min] ==== Slider ==== [Max]
            rmin, rmax = self.kin.limits[ax]
            r_slider = tk.Frame(g_box, bg=COLOR_BG_CARD)
            r_slider.pack(fill=tk.X, pady=1)

            lbl_smin = tk.Label(r_slider, text=f"{rmin:+.0f}°", width=5, bg=COLOR_BG_CARD, fg="#475569", font=("Consolas", 7))
            lbl_smin.pack(side=tk.LEFT)
            self.lbl_slider_min.append(lbl_smin)

            slider = tk.Scale(
                r_slider, from_=rmin, to=rmax, resolution=0.5, orient=tk.HORIZONTAL,
                bg=COLOR_BG_CARD, fg=COLOR_TEXT_PRIMARY, activebackground="#1e40af", highlightthickness=0,
                troughcolor="#e2e8f0", font=("Segoe UI", 7), showvalue=False, bd=0, sliderlength=24,
                command=lambda val, a=ax: self.on_joint_slider(a, val)
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
            self.sliders_ax.append(slider)

            lbl_smax = tk.Label(r_slider, text=f"{rmax:+.0f}°", width=5, bg=COLOR_BG_CARD, fg="#475569", font=("Consolas", 7))
            lbl_smax.pack(side=tk.RIGHT)
            self.lbl_slider_max.append(lbl_smax)

            # Fila de Presets rápidos dinámicos
            p_presets = tk.Frame(g_box, bg=COLOR_BG_CARD)
            p_presets.pack(fill=tk.X, pady=(2, 1))
            ax_preset_btns = []
            for deg_preset in joint_presets[ax]:
                btn_p = tk.Button(
                    p_presets, text=f"{deg_preset:+d}°" if deg_preset != 0 else "0°", bg="#ffffff", fg="#0f172a",
                    activebackground="#1e40af", activeforeground="#ffffff", relief=tk.SOLID, bd=1,
                    font=("Segoe UI", 7, "bold"), padx=2, pady=1, cursor="hand2",
                    command=lambda a=ax, d=deg_preset: self.set_joint_preset(a, d)
                )
                btn_p.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)
                ax_preset_btns.append(btn_p)
            self.preset_btn_list.append(ax_preset_btns)

        # Panel de Dinámica Lagrange-Euler (Torques en tiempo real)
        dyn_box = tk.Frame(p_inner, bg=COLOR_BG_CARD, padx=8, pady=4, highlightthickness=1, highlightbackground=COLOR_BORDER_FOCUS)
        dyn_box.pack(fill=tk.X, pady=(4, 2))

        tk.Label(dyn_box, text="Torques Lagrange-Euler G(q)", bg=COLOR_BG_CARD, fg=COLOR_AMBER_WARNING, font=("Segoe UI", 8, "bold")).pack(anchor=tk.W)

        self.lbl_torques = tk.Label(
            dyn_box,
            text="τ1 (Base): 0.000 N·m  |  τ2 (Hombro): 0.000 N·m  |  τ3 (Codo): 0.000 N·m",
            bg=COLOR_BG_CARD, fg="#0f172a", font=("Consolas", 7), justify=tk.LEFT
        )
        self.lbl_torques.pack(anchor=tk.W, pady=(1, 1))

        self.lbl_torque_warning = tk.Label(dyn_box, text="● Carga dentro del rango nominal", bg=COLOR_BG_CARD, fg=COLOR_GREEN_SUCCESS, font=("Segoe UI", 7))
        self.lbl_torque_warning.pack(anchor=tk.W)

        # Botones de Acción Rápida (Gripper y Paro)
        act_box = tk.Frame(p_inner, bg=COLOR_BG_PANEL)
        act_box.pack(fill=tk.X, pady=(4, 6))

        self.btn_gripper = tk.Button(
            act_box, text="Herramienta: OFF", bg="#475569", fg="#ffffff", activebackground="#334155",
            font=("Segoe UI", 8, "bold"), pady=3, bd=0, relief=tk.FLAT, cursor="hand2", command=self.toggle_gripper
        )
        self.btn_gripper.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        btn_estop = tk.Button(
            act_box, text="PARO DE EMERGENCIA", bg=COLOR_RED_DANGER, fg="#ffffff", activebackground="#b91c1c",
            font=("Segoe UI", 8, "bold"), pady=3, bd=0, relief=tk.FLAT, cursor="hand2", command=self.cmd_stop
        )
        btn_estop.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=2)


# ==============================================================================
# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ TEMA 11: CICLO DE ANIMACIÓN PRINCIPAL Y PUNTO DE ENTRADA                   ║
# ╚════════════════════════════════════════════════════════════════════════════╝
# ==============================================================================

    def animate(self):
        """
        Bucle recurrente de animación y telemetría a 25 FPS (~40 ms):
          1. Despacha tareas de interfaz seguras encoladas en `ui_queue`.
          2. Interpola suavemente el movimiento angular en modo simulación virtual.
          3. Dibuja los velocímetros semicirculares por articulación.
          4. Renderiza la escena 3D y el feed de la WebCam.
          5. Recalcula los torques dinámicos de Lagrange-Euler y verifica sobrecargas.
        """
        # 1. Despacho seguro de callbacks GUI encolados
        while not self.ui_queue.empty():
            try:
                fn, a_args, a_kwargs = self.ui_queue.get_nowait()
                fn(*a_args, **a_kwargs)
            except Exception:
                break

        # 2. Interpolación suave de ángulos en modo simulación
        if not self.running:
            speed_deg = 2.4  # Grados por fotograma (~60 °/s)
            for ax in range(3):
                diff = self.target_angles[ax] - self.current_angles[ax]
                if abs(diff) > 0.05:
                    step = min(abs(diff), speed_deg)
                    self.current_angles[ax] += math.copysign(step, diff)
                    self.sliders_ax[ax].set(self.current_angles[ax])
                else:
                    self.current_angles[ax] = self.target_angles[ax]

        # 3. Actualización visual de instrumentos analógicos
        for ax in range(3):
            self.draw_single_gauge(ax)
            self.gauge_labels[ax].config(text=f"{self.current_angles[ax]:06.2f}°")
            if hasattr(self, 'lbl_turns_nema') and ax < len(self.lbl_turns_nema):
                turns_equiv = (self.current_angles[ax] * 31.0) / 360.0
                self.lbl_turns_nema[ax].config(text=f"NEMA 17: {turns_equiv:5.2f} / 31.0 revs")
            if ax < len(self.encoder_badge_labels):
                st = self.encoder_status[ax]
                if not self.running:
                    self.encoder_badge_labels[ax].config(
                        text="● NO SERIAL", bg="#1e293b", fg="#64748b"
                    )
                    self.encoder_raw_labels[ax].config(text="Raw: ----", fg="#64748b")
                else:
                    if st["connected"] and st["magnet"]:
                        self.encoder_badge_labels[ax].config(
                            text="● ENCODER OK (IMÁN DETECTADO)", bg="#dcfce7", fg="#15803d", relief=tk.SOLID, bd=1
                        )
                        self.encoder_raw_labels[ax].config(text=f"Raw: {st['raw']:04d}", fg="#0f172a")
                    elif st["connected"] and not st["magnet"]:
                        self.encoder_badge_labels[ax].config(
                            text="▲ I2C OK (SIN IMÁN)", bg="#fef3c7", fg="#92400e", relief=tk.SOLID, bd=1
                        )
                        self.encoder_raw_labels[ax].config(text=f"Raw: {st['raw']:04d}", fg="#f59e0b")
                    else:
                        self.encoder_badge_labels[ax].config(
                            text="● DESCONECTADO (ERROR I2C)", bg="#fee2e2", fg="#991b1b", relief=tk.SOLID, bd=1
                        )
                        self.encoder_raw_labels[ax].config(text="Raw: N/A", fg="#ef4444")

        # 4. Redibujado de la escena 3D interactiva y cámara
        self.draw_simulation()
        self.update_video_canvas()

        # 5. Cálculo y monitoreo en tiempo real de torques de gravedad
        t1, t2, t3 = self.kin.calculate_lagrange_torques(
            self.current_angles[0], self.current_angles[1], self.current_angles[2]
        )
        self.lbl_torques.config(
            text=f"τ1 (Base):   {t1:+.3f} N·m\nτ2 (Hombro): {t2:+.3f} N·m\nτ3 (Codo):   {t3:+.3f} N·m"
        )
        if hasattr(self, 'lbl_zero_quick_angles'):
            self.lbl_zero_quick_angles.config(
                text=f"M1: {self.current_angles[0]:+06.2f}° | M2: {self.current_angles[1]:+06.2f}° | M3: {self.current_angles[2]:+06.2f}°"
            )
        if abs(t2) > 1.8:
            self.lbl_torque_warning.config(text="Alerta: Carga de Hombro alta (>1.8 N·m)", fg=COLOR_RED_DANGER)
        else:
            self.lbl_torque_warning.config(text="● Carga dentro del rango nominal", fg=COLOR_GREEN_SUCCESS)

        # Planificar siguiente fotograma a 40 ms (25 FPS)
        self.after(40, self.animate)


    def cmd_zero_encoder(self, motor_id=0):
        """Fija el cero (tara) del encoder seleccionado (1..3 o 0 para todos)."""
        self.pkt_seq = (self.pkt_seq + 1) % 10000
        payload = f"PKT,{self.pkt_seq},{motor_id},ZERO,0.0,0.0,0.0"
        cs = self.calculate_checksum(payload)
        self.send_serial_raw(f"${payload}*{cs:02X}\r\n")
        if motor_id == 0:
            self.send_serial_raw("ZERO\n")
            for i in range(3):
                self.current_angles[i] = 0.0
                self.target_angles[i] = 0.0
        elif 1 <= motor_id <= 3:
            self.current_angles[motor_id - 1] = 0.0
            self.target_angles[motor_id - 1] = 0.0

    def cmd_reset_zero_encoder(self, motor_id=0):
        """Restaura el ángulo absoluto sin offset del encoder seleccionado."""
        self.pkt_seq = (self.pkt_seq + 1) % 10000
        payload = f"PKT,{self.pkt_seq},{motor_id},RESET_ZERO,0.0,0.0,0.0"
        cs = self.calculate_checksum(payload)
        self.send_serial_raw(f"${payload}*{cs:02X}\r\n")

    # ==========================================================================
    # SISTEMA GESTOR DE PERFILES Y ASISTENTE DE INICIO DEL ROBOT
    # ==========================================================================
    def load_robot_config_file(self):
        """Carga el archivo de configuración persistente robot_config.json con tolerancia a fallos."""
        default_cfg = {
            "current_profile": "Robot MK2 Gato 31:1 (Predeterminado)",
            "profiles": {
                "Robot MK2 Gato 31:1 (Predeterminado)": {
                    "name": "Robot MK2 Gato 31:1 (Predeterminado)",
                    "description": "Configuración estándar SolidWorks con reductores cicloidales 31:1 y electroimán en GPIO 7",
                    "date": "2026-10-30",
                    "geometry": {"d1": 10.0, "a1": 11.0, "a2": 32.922, "a3": 32.934},
                    "reduction": {"m1": 31.0, "m2": 31.0, "m3": 31.0},
                    "limits": {"m1": [-60.0, 60.0], "m2": [-60.0, 60.0], "m3": [0.0, 45.0]},
                    "tool": {"type": "Electroimán 12V (GPIO 7)", "pick_z": 6.0, "place_z": 3.0, "hover_z": 8.0},
                    "dynamics": {"speed_mode": "Normal (2400 Hz)", "max_speed_hz": 2400, "accel_hz_s": 2500},
                    "zero_calibrated": True
                }
            }
        }
        if not os.path.exists(self.config_file_path):
            self.save_robot_config_file(default_cfg)
            return default_cfg
        try:
            with open(self.config_file_path, "r", encoding="utf-8") as f:
                data = json.load(f)
                if "profiles" not in data or not data["profiles"]:
                    return default_cfg
                return data
        except Exception as e:
            print(f"Error leyendo robot_config.json: {e}")
            return default_cfg

    def save_robot_config_file(self, config_data=None):
        """Guarda los perfiles y el perfil activo en robot_config.json."""
        if config_data is None:
            config_data = getattr(self, "robot_config", {})
        try:
            with open(self.config_file_path, "w", encoding="utf-8") as f:
                json.dump(config_data, f, indent=4, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"Error guardando robot_config.json: {e}")
            return False

    def apply_robot_profile(self, profile_name, silent=False):
        """Aplica las características del perfil seleccionado a la cinemática, sliders, GUI y firmware."""
        profiles = self.robot_config.get("profiles", {})
        if profile_name not in profiles:
            return False
        p = profiles[profile_name]
        self.active_profile_name = profile_name
        self.robot_config["current_profile"] = profile_name
        self.save_robot_config_file(self.robot_config)

        # 1. Actualizar Geometría en Cinemática Analítica
        geom = p.get("geometry", {})
        d1 = float(geom.get("d1", 10.0))
        a1 = float(geom.get("a1", 11.0))
        a2 = float(geom.get("a2", 32.922))
        a3 = float(geom.get("a3", 32.934))
        self.kin.d1 = d1; self.kin.d1_m = d1 / 100.0
        self.kin.a1 = a1; self.kin.a1_m = a1 / 100.0
        self.kin.a2 = a2; self.kin.a2_m = a2 / 100.0
        self.kin.a3 = a3; self.kin.a3_m = a3 / 100.0

        # 2. Actualizar Límites Angulares de Seguridad con Protección Anticolisión M2-M3
        lims = p.get("limits", {})
        raw_m1 = lims.get("m1", [-60.0, 60.0])
        l_m1 = (min(float(raw_m1[0]), float(raw_m1[1])), max(float(raw_m1[0]), float(raw_m1[1])))

        raw_m2 = lims.get("m2", [-60.0, 60.0])
        l_m2 = (min(float(raw_m2[0]), float(raw_m2[1])), max(float(raw_m2[0]), float(raw_m2[1])))

        raw_m3 = lims.get("m3", [-90.0, 0.0])
        v1_3, v2_3 = float(raw_m3[0]), float(raw_m3[1])
        # REGLA ESTRICTA DE OPERACIÓN: M3 nunca hacia atrás (max <= 0.0°). Solo Home (0°) y adelante.
        l_m3 = (min(v1_3, v2_3), min(0.0, max(v1_3, v2_3)))
        self.kin.limits = [l_m1, l_m2, l_m3]
        self.gauge_ranges = [l_m1, l_m2, l_m3]

        # Actualizar casillas y dispensador según la geometría del perfil
        self.recalculate_board_cells()

        if hasattr(self, 'sliders_ax') and len(self.sliders_ax) >= 3:
            for idx, lim in enumerate([l_m1, l_m2, l_m3]):
                self.sliders_ax[idx].config(from_=lim[0], to=lim[1])
        if hasattr(self, 'ent_lim_min') and len(self.ent_lim_min) >= 3:
            for idx, lim in enumerate([l_m1, l_m2, l_m3]):
                self.ent_lim_min[idx].delete(0, tk.END); self.ent_lim_min[idx].insert(0, f"{lim[0]:.1f}")
                self.ent_lim_max[idx].delete(0, tk.END); self.ent_lim_max[idx].insert(0, f"{lim[1]:.1f}")

        # 3. Transmitir Límites al Firmware ESP32 si está conectado
        if self.running and self.serial_port and self.serial_port.is_open:
            for idx, lim in enumerate([l_m1, l_m2, l_m3]):
                self.pkt_seq = (self.pkt_seq + 1) % 10000
                payload = f"PKT,{self.pkt_seq},{idx+1},LIMITS,{lim[0]:.1f},{lim[1]:.1f},0.0"
                cs = self.calculate_checksum(payload)
                self.send_serial_raw(f"${payload}*{cs:02X}\r\n")

        # 4. Actualizar Botón de Perfil en la barra superior
        if hasattr(self, 'btn_robot_config'):
            self.btn_robot_config.config(text=f"Perfil: {profile_name[:16]}")

        # 5. Estado de Calibración de Posición Cero Persistente
        self.zero_calibrated = bool(p.get("zero_calibrated", True))
        self.motors_free_state = not self.zero_calibrated
        self.update_zero_bar_ui()

        if not silent:
            messagebox.showinfo("Perfil Aplicado", f"Se cargó con éxito el perfil de robot:\n\n'{profile_name}'\n\nGeometría, límites y parámetros actualizados correctamente.")
        return True

    def show_startup_robot_wizard(self, initial_view="welcome"):
        """
        Asistente de Inicio Interactivo:
          - Paso 1: Pregunta si desea 'Cargar Robot Guardado' o 'Configurar Robot Nuevo'.
          - Paso 2: Permite configurar y guardar las características completas del manipulador:
            * Posición Cero (Calibración de Origen y encoders AS5600 en vivo).
            * Geometría de eslabones (d1, a1, a2, a3).
            * Relación de reducción cicloidal (31:1).
            * Límites de seguridad mecánica.
            * Efector final y alturas del tablero.
            * Dinámica y velocidad máxima.
        """
        modal = tk.Toplevel(self)
        modal.title("Asistente de Inicio — Selección y Características del Manipulador (3 GDL)")
        modal.geometry("940x720")
        modal.minsize(860, 640)
        modal.configure(bg=COLOR_BG_MAIN)
        modal.transient(self)

        modal_running = [True]
        def on_modal_close():
            modal_running[0] = False
            modal.destroy()
        modal.protocol("WM_DELETE_WINDOW", on_modal_close)

        view_container = tk.Frame(modal, bg=COLOR_BG_MAIN)
        view_container.pack(fill=tk.BOTH, expand=True)

        def clear_view():
            for w in view_container.winfo_children():
                w.destroy()

        # ======================================================================
        # VISTA 1: BIENVENIDA — CARGAR ROBOT vs NUEVO ROBOT
        # ======================================================================
        def render_welcome_view():
            clear_view()

            # Encabezado principal
            hdr = tk.Frame(view_container, bg=COLOR_BG_PANEL, padx=16, pady=14, highlightthickness=1, highlightbackground=COLOR_BORDER)
            hdr.pack(fill=tk.X, padx=16, pady=(14, 10))

            tk.Label(
                hdr, text="BIENVENIDO AL SISTEMA DE CONTROL CINEMÁTICO ROBOT 3 GDL",
                bg=COLOR_BG_PANEL, fg=COLOR_ACCENT, font=("Segoe UI", 13, "bold")
            ).pack(anchor=tk.W)

            tk.Label(
                hdr,
                text="Para comenzar a operar el brazo robótico, selecciona si deseas cargar un robot previamente guardado\n"
                     "o configurar un robot nuevo con sus características mecánicas, reducciones cicloidales y posición cero.",
                bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 9), justify=tk.LEFT
            ).pack(anchor=tk.W, pady=(6, 0))

            # Contenedor dividido en 2 columnas: CARGAR ROBOT (izq) y NUEVO ROBOT (der)
            body = tk.Frame(view_container, bg=COLOR_BG_MAIN)
            body.pack(fill=tk.BOTH, expand=True, padx=16, pady=6)

            # --- TARJETA 1: CARGAR ROBOT GUARDADO ---
            card_load = tk.Frame(body, bg=COLOR_BG_PANEL, padx=14, pady=12, highlightthickness=1, highlightbackground=COLOR_BORDER)
            card_load.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))

            c_hdr = tk.Frame(card_load, bg=COLOR_BG_PANEL)
            c_hdr.pack(fill=tk.X)
            tk.Label(c_hdr, text="CARGAR ROBOT GUARDADO", bg=COLOR_BG_PANEL, fg=COLOR_CYAN_ACCENT, font=("Segoe UI", 11, "bold")).pack(side=tk.LEFT)
            tk.Label(c_hdr, text="PERFILES EN DISCO", bg="#dbeafe", fg="#1e40af", relief=tk.SOLID, bd=1, font=("Segoe UI", 8, "bold"), padx=6, pady=2).pack(side=tk.RIGHT)

            tk.Label(
                card_load,
                text="Recupera la configuración de eslabones, límites seguros, herramienta y calibración de un robot guardado.",
                bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8), justify=tk.LEFT, wraplength=380
            ).pack(anchor=tk.W, pady=(6, 10))

            tk.Label(card_load, text="Selecciona el Perfil del Robot:", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 8, "bold")).pack(anchor=tk.W)

            profiles = self.robot_config.get("profiles", {})
            profile_names = list(profiles.keys())
            combo_prof = ttk.Combobox(card_load, values=profile_names, state="readonly", font=("Segoe UI", 9))
            if self.active_profile_name in profile_names:
                combo_prof.set(self.active_profile_name)
            elif profile_names:
                combo_prof.set(profile_names[0])
            combo_prof.pack(fill=tk.X, pady=(4, 10))

            # Caja de detalles del perfil seleccionado
            f_specs = tk.Frame(card_load, bg=COLOR_BG_CARD, padx=10, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
            f_specs.pack(fill=tk.BOTH, expand=True, pady=4)

            lbl_spec_title = tk.Label(f_specs, text="Ficha Técnica del Perfil:", bg=COLOR_BG_CARD, fg=COLOR_CYAN_ACCENT, font=("Segoe UI", 8, "bold"))
            lbl_spec_title.pack(anchor=tk.W)

            lbl_spec_details = tk.Label(f_specs, text="", bg=COLOR_BG_CARD, fg=COLOR_TEXT_PRIMARY, font=("Consolas", 8), justify=tk.LEFT)
            lbl_spec_details.pack(anchor=tk.W, pady=4)

            def update_specs_card(*_):
                sel = combo_prof.get()
                p = profiles.get(sel, {})
                geom = p.get("geometry", {})
                lim = p.get("limits", {})
                red = p.get("reduction", {})
                tool = p.get("tool", {})
                dyn = p.get("dynamics", {})

                txt = (
                    f"• Geometría: d1={geom.get('d1', 10)}cm | a1={geom.get('a1', 11)}cm\n"
                    f"             a2={geom.get('a2', 32.9)}cm | a3={geom.get('a3', 32.9)}cm\n"
                    f"• Reducción: M1={red.get('m1', 31)}:1 | M2={red.get('m2', 31)}:1 | M3={red.get('m3', 31)}:1\n"
                    f"• Límites:   M1 {lim.get('m1', [-60,60])}° | M2 {lim.get('m2', [0,95])}°\n"
                    f"             M3 {lim.get('m3', [-165,5])}°\n"
                    f"• Efector:   {tool.get('type', 'Electroimán')} (Z_pick={tool.get('pick_z', 6)}cm)\n"
                    f"• Dinámica:  {dyn.get('speed_mode', 'Normal 2400 Hz')}"
                )
                lbl_spec_details.config(text=txt)

            combo_prof.bind("<<ComboboxSelected>>", update_specs_card)
            update_specs_card()

            def on_load_click():
                sel = combo_prof.get()
                if sel:
                    self.apply_robot_profile(sel)
                    if self.running and self.serial_port and self.serial_port.is_open:
                        self.cmd_load_game()
                    on_modal_close()

            def on_edit_click():
                sel = combo_prof.get()
                p = profiles.get(sel, {})
                render_config_view(profile_data=p)

            btn_box_l = tk.Frame(card_load, bg=COLOR_BG_PANEL)
            btn_box_l.pack(fill=tk.X, pady=(10, 0))

            tk.Button(
                btn_box_l, text="Editar Características", bg="#e2e8f0", fg=COLOR_TEXT_PRIMARY,
                activebackground="#cbd5e1", font=("Segoe UI", 8, "bold"), relief=tk.SOLID, bd=1, padx=10, pady=6,
                cursor="hand2", command=on_edit_click
            ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))

            tk.Button(
                btn_box_l, text="CARGAR Y CONTINUAR", bg="#15803d", fg="#ffffff",
                activebackground="#166534", font=("Segoe UI", 9, "bold"), relief=tk.FLAT, padx=12, pady=6,
                cursor="hand2", command=on_load_click
            ).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(4, 0))

            # --- TARJETA 2: NUEVO ROBOT ---
            card_new = tk.Frame(body, bg=COLOR_BG_PANEL, padx=14, pady=12, highlightthickness=1, highlightbackground=COLOR_BORDER)
            card_new.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(8, 0))

            cnew_hdr = tk.Frame(card_new, bg=COLOR_BG_PANEL)
            cnew_hdr.pack(fill=tk.X)
            tk.Label(cnew_hdr, text="NUEVO ROBOT", bg=COLOR_BG_PANEL, fg=COLOR_GREEN_SUCCESS, font=("Segoe UI", 11, "bold")).pack(side=tk.LEFT)
            tk.Label(cnew_hdr, text="ASISTENTE COMPLETO", bg="#dcfce7", fg="#15803d", relief=tk.SOLID, bd=1, font=("Segoe UI", 8, "bold"), padx=6, pady=2).pack(side=tk.RIGHT)

            tk.Label(
                card_new,
                text="Configura un nuevo manipulador o adapta los parámetros mecánicos y electrónicos a tu prototipo:",
                bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8), justify=tk.LEFT, wraplength=380
            ).pack(anchor=tk.W, pady=(6, 12))

            items_box = tk.Frame(card_new, bg=COLOR_BG_CARD, padx=12, pady=10, highlightthickness=1, highlightbackground=COLOR_BORDER)
            items_box.pack(fill=tk.BOTH, expand=True, pady=4)

            steps = [
                ("Posición Cero (Calibración de Origen 0.0°)", "Calibra el origen 0.0° sin giros descontrolados de 360°."),
                ("Dimensiones de Eslabones", "Define alturas d1, a1, brazo a2 y antebrazo a3 (SolidWorks)."),
                ("Reductores Cicloidales 31:1", "Establece la relación de transmisión de cada articulación."),
                ("Límites Angulares de Seguridad", "Protege al robot contra colisiones y sobre-carrera."),
                ("Efector Final y Alturas de Tablero", "Configura electroimán/pinza y alturas de Pick, Place y Hover."),
                ("Dinámica y Velocidad de Juego", "Ajusta rampas suaves de aceleración y velocidad máxima.")
            ]

            for title, desc in steps:
                s_frame = tk.Frame(items_box, bg=COLOR_BG_CARD)
                s_frame.pack(fill=tk.X, pady=3)
                tk.Label(s_frame, text=title, bg=COLOR_BG_CARD, fg=COLOR_CYAN_ACCENT, font=("Segoe UI", 8, "bold")).pack(anchor=tk.W)
                tk.Label(s_frame, text=desc, bg=COLOR_BG_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 7)).pack(anchor=tk.W)

            tk.Button(
                card_new, text="CONFIGURAR NUEVO ROBOT (PASO A PASO)", bg="#15803d", fg="#ffffff",
                activebackground="#166534", font=("Segoe UI", 9, "bold"), relief=tk.FLAT, padx=14, pady=8,
                cursor="hand2", command=lambda: render_config_view(profile_data=None)
            ).pack(fill=tk.X, pady=(12, 0))

            # Barra inferior con botón cerrar
            btn_bottom = tk.Frame(view_container, bg=COLOR_BG_MAIN)
            btn_bottom.pack(fill=tk.X, padx=16, pady=(6, 14))
            tk.Button(
                btn_bottom, text="Cerrar Asistente", bg="#e2e8f0", fg=COLOR_TEXT_PRIMARY,
                activebackground="#cbd5e1", font=("Segoe UI", 8, "bold"), relief=tk.SOLID, bd=1, padx=14, pady=5,
                cursor="hand2", command=on_modal_close
            ).pack(side=tk.RIGHT)

        # ======================================================================
        # VISTA 2: FORMULARIO COMPLETO DE CARACTERÍSTICAS DEL ROBOT
        # ======================================================================
        def render_config_view(profile_data=None):
            clear_view()

            is_new = (profile_data is None)
            base_data = profile_data or {
                "name": "Mi Robot Gato MK2",
                "geometry": {"d1": 10.0, "a1": 11.0, "a2": 32.922, "a3": 32.934},
                "reduction": {"m1": 31.0, "m2": 31.0, "m3": 31.0},
                "limits": {"m1": [-60.0, 60.0], "m2": [0.0, 95.0], "m3": [-165.0, 5.0]},
                "tool": {"type": "Electroimán 12V (GPIO 7)", "pick_z": 6.0, "place_z": 3.0, "hover_z": 8.0},
                "dynamics": {"speed_mode": "Normal (2400 Hz)", "max_speed_hz": 2400, "accel_hz_s": 2500}
            }

            # Barra superior con nombre de perfil
            top_bar_v2 = tk.Frame(view_container, bg=COLOR_BG_PANEL, padx=14, pady=10, highlightthickness=1, highlightbackground=COLOR_BORDER)
            top_bar_v2.pack(fill=tk.X, padx=14, pady=(12, 6))

            tk.Label(top_bar_v2, text="Nombre del Robot / Perfil:", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 6))
            ent_name = tk.Entry(top_bar_v2, bg="#ffffff", fg=COLOR_TEXT_PRIMARY, insertbackground="#0f172a", font=("Segoe UI", 9, "bold"), width=30, relief=tk.SOLID, bd=1)
            ent_name.insert(0, base_data.get("name", "Mi Robot Gato"))
            ent_name.pack(side=tk.LEFT, padx=(0, 10))

            tk.Label(
                top_bar_v2,
                text=" CONFIGURACIÓN DE NUEVO ROBOT" if is_new else "️ EDITANDO PERFIL DE ROBOT",
                bg="#064e3b" if is_new else "#0c4a6e",
                fg="#a7f3d0" if is_new else "#bae6fd",
                font=("Segoe UI", 8, "bold"), padx=8, pady=2
            ).pack(side=tk.RIGHT)

            # Contenedor dividido en 2 columnas: Columna 1 (Cero + Límites + Dinámica) | Columna 2 (Geometría + Reductores + Herramienta)
            form_frame = tk.Frame(view_container, bg=COLOR_BG_MAIN)
            form_frame.pack(fill=tk.BOTH, expand=True, padx=14, pady=4)

            col1 = tk.Frame(form_frame, bg=COLOR_BG_MAIN)
            col1.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

            col2 = tk.Frame(form_frame, bg=COLOR_BG_MAIN)
            col2.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))

            # ------------------------------------------------------------------
            # COLUMNA 1: SECCIÓN 1 - POSICIÓN CERO (CALIBRACIÓN DE ORIGEN)
            # ------------------------------------------------------------------
            c_zero = tk.Frame(col1, bg=COLOR_BG_PANEL, padx=10, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
            c_zero.pack(fill=tk.X, pady=(0, 6))

            z_top = tk.Frame(c_zero, bg=COLOR_BG_PANEL)
            z_top.pack(fill=tk.X)
            tk.Label(z_top, text="1. CALIBRACIÓN DE POSICIÓN CERO", bg=COLOR_BG_PANEL, fg=COLOR_CYAN_ACCENT, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
            lbl_conn_stat = tk.Label(
                z_top,
                text="● SERIAL ACTIVO" if self.running else "● SIN CONEXIÓN SERIAL",
                bg="#064e3b" if self.running else "#1e293b",
                fg="#34d399" if self.running else "#94a3b8",
                font=("Segoe UI", 7, "bold"), padx=4, pady=1
            )
            lbl_conn_stat.pack(side=tk.RIGHT)

            tk.Label(
                c_zero,
                text="Evita que M1 gire 360° al arrancar. Coloca el robot a mano o carga el cero guardado en NVS:",
                bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 7), justify=tk.LEFT
            ).pack(anchor=tk.W, pady=(2, 4))

            # Mini tarjetas de ángulos en vivo
            ang_row = tk.Frame(c_zero, bg=COLOR_BG_PANEL)
            ang_row.pack(fill=tk.X, pady=2)
            ang_v2_labels = []
            for i, ax_name in enumerate(["M1 (Base)", "M2 (Hombro)", "M3 (Codo)"]):
                af = tk.Frame(ang_row, bg=COLOR_BG_CARD, padx=4, pady=3, highlightthickness=1, highlightbackground=COLOR_BORDER)
                af.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)
                tk.Label(af, text=ax_name, bg=COLOR_BG_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 7)).pack()
                la = tk.Label(af, text=f"{self.current_angles[i]:+06.2f}°", bg=COLOR_BG_CARD, fg=COLOR_CYAN_ACCENT, font=("Consolas", 10, "bold"))
                la.pack()
                ang_v2_labels.append(la)

            # Botones de Calibración de Posición Cero
            z_btns1 = tk.Frame(c_zero, bg=COLOR_BG_PANEL)
            z_btns1.pack(fill=tk.X, pady=(4, 2))
            tk.Button(
                z_btns1, text="FIJAR CERO ACTUAL", bg="#15803d", fg="#ffffff",
                font=("Segoe UI", 8, "bold"), relief=tk.FLAT, padx=6, pady=4, cursor="hand2",
                command=self.cmd_save_game
            ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
            tk.Button(
                z_btns1, text="Cargar Cero (0.0°) FLASH", bg="#1e40af", fg="#ffffff",
                font=("Segoe UI", 8, "bold"), relief=tk.FLAT, padx=6, pady=4, cursor="hand2",
                command=self.cmd_load_game
            ).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=1)

            z_btns2 = tk.Frame(c_zero, bg=COLOR_BG_PANEL)
            z_btns2.pack(fill=tk.X, pady=(2, 0))
            tk.Button(
                z_btns2, text="MOTORES LIBRES (A MANO)", bg="#b45309", fg="#ffffff",
                font=("Segoe UI", 8, "bold"), relief=tk.FLAT, padx=6, pady=3, cursor="hand2",
                command=self.cmd_motors_free
            ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
            tk.Button(
                z_btns2, text="BLOQUEAR TORQUE", bg="#475569", fg="#ffffff",
                font=("Segoe UI", 8, "bold"), relief=tk.FLAT, padx=6, pady=3, cursor="hand2",
                command=self.cmd_motors_lock
            ).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=1)

            # ------------------------------------------------------------------
            # COLUMNA 1: SECCIÓN 2 - LÍMITES ANGULARES DE SEGURIDAD
            # ------------------------------------------------------------------
            c_lim = tk.Frame(col1, bg=COLOR_BG_PANEL, padx=10, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
            c_lim.pack(fill=tk.X, pady=(0, 6))

            tk.Label(c_lim, text="2. Límites Angulares de Seguridad (°)", bg=COLOR_BG_PANEL, fg=COLOR_GREEN_SUCCESS, font=("Segoe UI", 9, "bold")).pack(anchor=tk.W)
            tk.Label(
                c_lim, text="Acotan el movimiento del reductor para evitar sobre-carrera o roturas:",
                bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 7)
            ).pack(anchor=tk.W, pady=(1, 4))

            lims_in = base_data.get("limits", {})
            ent_lims = []
            for ax_idx, ax_lbl, l_def in [
                (1, "M1 Base:   ", lims_in.get("m1", [-60.0, 60.0])),
                (2, "M2 Hombro: ", lims_in.get("m2", [0.0, 95.0])),
                (3, "M3 Codo:   ", lims_in.get("m3", [-165.0, 5.0]))
            ]:
                row = tk.Frame(c_lim, bg=COLOR_BG_PANEL)
                row.pack(fill=tk.X, pady=2)
                tk.Label(row, text=ax_lbl, bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 8, "bold"), width=11, anchor=tk.W).pack(side=tk.LEFT)
                tk.Label(row, text="Mín:", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 7)).pack(side=tk.LEFT, padx=1)
                e_min = tk.Entry(row, bg="#ffffff", fg=COLOR_TEXT_PRIMARY, font=("Consolas", 8, "bold"), width=7, relief=tk.SOLID, bd=1)
                e_min.insert(0, f"{l_def[0]:.1f}")
                e_min.pack(side=tk.LEFT, padx=2)
                tk.Label(row, text="Máx:", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 7)).pack(side=tk.LEFT, padx=1)
                e_max = tk.Entry(row, bg="#ffffff", fg=COLOR_TEXT_PRIMARY, font=("Consolas", 8, "bold"), width=7, relief=tk.SOLID, bd=1)
                e_max.insert(0, f"{l_def[1]:.1f}")
                e_max.pack(side=tk.LEFT, padx=2)
                tk.Label(row, text="grados", bg=COLOR_BG_PANEL, fg="#64748b", font=("Segoe UI", 7)).pack(side=tk.LEFT, padx=2)
                ent_lims.append((e_min, e_max))

            # ------------------------------------------------------------------
            # COLUMNA 1: SECCIÓN 3 - DINÁMICA Y VELOCIDAD
            # ------------------------------------------------------------------
            c_dyn = tk.Frame(col1, bg=COLOR_BG_PANEL, padx=10, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
            c_dyn.pack(fill=tk.X)

            tk.Label(c_dyn, text="3. DINÁMICA Y VELOCIDAD DE TRASLACIÓN", bg=COLOR_BG_PANEL, fg=COLOR_CYAN_ACCENT, font=("Segoe UI", 9, "bold")).pack(anchor=tk.W)
            dyn_in = base_data.get("dynamics", {})
            combo_speed = ttk.Combobox(
                c_dyn, state="readonly", font=("Segoe UI", 8),
                values=["Seguro / Calibración (1200 Hz)", "Normal / Fluido (2400 Hz)", "Competición / Rápido (3600 Hz)"]
            )
            cur_sp = dyn_in.get("speed_mode", "Normal (2400 Hz)")
            if "1200" in cur_sp: combo_speed.set("Seguro / Calibración (1200 Hz)")
            elif "3600" in cur_sp: combo_speed.set("Competición / Rápido (3600 Hz)")
            else: combo_speed.set("Normal / Fluido (2400 Hz)")
            combo_speed.pack(fill=tk.X, pady=4)
            tk.Label(
                c_dyn, text="Rampas suaves de aceleración (2500 Hz/s) para evitar inercia y rebotes.",
                bg=COLOR_BG_PANEL, fg="#64748b", font=("Segoe UI", 7)
            ).pack(anchor=tk.W)

            # ------------------------------------------------------------------
            # COLUMNA 2: SECCIÓN 4 - GEOMETRÍA Y ESLABONES
            # ------------------------------------------------------------------
            c_geom = tk.Frame(col2, bg=COLOR_BG_PANEL, padx=10, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
            c_geom.pack(fill=tk.X, pady=(0, 6))

            g_top = tk.Frame(c_geom, bg=COLOR_BG_PANEL)
            g_top.pack(fill=tk.X)
            tk.Label(g_top, text="4. GEOMETRÍA DE ESLABONES (cm)", bg=COLOR_BG_PANEL, fg=COLOR_CYAN_ACCENT, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
            tk.Button(
                g_top, text="↺ Reset MK2", bg="#e2e8f0", fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 7, "bold"), relief=tk.SOLID, bd=1, padx=4, pady=1, cursor="hand2",
                command=lambda: (ent_d1.delete(0, tk.END), ent_d1.insert(0, "10.0"),
                                 ent_a1.delete(0, tk.END), ent_a1.insert(0, "11.0"),
                                 ent_a2.delete(0, tk.END), ent_a2.insert(0, "32.922"),
                                 ent_a3.delete(0, tk.END), ent_a3.insert(0, "32.934"))
            ).pack(side=tk.RIGHT)

            geom_in = base_data.get("geometry", {})
            g_grid = tk.Frame(c_geom, bg=COLOR_BG_PANEL)
            g_grid.pack(fill=tk.X, pady=3)

            tk.Label(g_grid, text="d1 (Elevación vertical Base):", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 8)).grid(row=0, column=0, sticky=tk.W, pady=2)
            ent_d1 = tk.Entry(g_grid, bg="#ffffff", fg=COLOR_TEXT_PRIMARY, font=("Consolas", 8, "bold"), width=8, relief=tk.SOLID, bd=1)
            ent_d1.insert(0, str(geom_in.get("d1", 10.0)))
            ent_d1.grid(row=0, column=1, padx=4)
            tk.Label(g_grid, text="cm", bg=COLOR_BG_PANEL, fg="#64748b", font=("Segoe UI", 7)).grid(row=0, column=2, sticky=tk.W)

            tk.Label(g_grid, text="a1 (Offset horizontal Hombro):", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 8)).grid(row=1, column=0, sticky=tk.W, pady=2)
            ent_a1 = tk.Entry(g_grid, bg="#ffffff", fg=COLOR_TEXT_PRIMARY, font=("Consolas", 8, "bold"), width=8, relief=tk.SOLID, bd=1)
            ent_a1.insert(0, str(geom_in.get("a1", 11.0)))
            ent_a1.grid(row=1, column=1, padx=4)
            tk.Label(g_grid, text="cm", bg=COLOR_BG_PANEL, fg="#64748b", font=("Segoe UI", 7)).grid(row=1, column=2, sticky=tk.W)

            tk.Label(g_grid, text="a2 (Longitud Brazo Hombro-Codo):", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 8)).grid(row=2, column=0, sticky=tk.W, pady=2)
            ent_a2 = tk.Entry(g_grid, bg="#ffffff", fg=COLOR_TEXT_PRIMARY, font=("Consolas", 8, "bold"), width=8, relief=tk.SOLID, bd=1)
            ent_a2.insert(0, str(geom_in.get("a2", 32.922)))
            ent_a2.grid(row=2, column=1, padx=4)
            tk.Label(g_grid, text="cm", bg=COLOR_BG_PANEL, fg="#64748b", font=("Segoe UI", 7)).grid(row=2, column=2, sticky=tk.W)

            tk.Label(g_grid, text="a3 (Antebrazo Codo-Efector):", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 8)).grid(row=3, column=0, sticky=tk.W, pady=2)
            ent_a3 = tk.Entry(g_grid, bg="#ffffff", fg=COLOR_TEXT_PRIMARY, font=("Consolas", 8, "bold"), width=8, relief=tk.SOLID, bd=1)
            ent_a3.insert(0, str(geom_in.get("a3", 32.934)))
            ent_a3.grid(row=3, column=1, padx=4)
            tk.Label(g_grid, text="cm", bg=COLOR_BG_PANEL, fg="#64748b", font=("Segoe UI", 7)).grid(row=3, column=2, sticky=tk.W)

            # ------------------------------------------------------------------
            # COLUMNA 2: SECCIÓN 5 - REDUCCIÓN CICLOIDAL
            # ------------------------------------------------------------------
            c_red = tk.Frame(col2, bg=COLOR_BG_PANEL, padx=10, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
            c_red.pack(fill=tk.X, pady=(0, 6))

            tk.Label(c_red, text="5. REDUCTORES CICLOIDALES (RATIO : 1)", bg=COLOR_BG_PANEL, fg=COLOR_AMBER_WARNING, font=("Segoe UI", 9, "bold")).pack(anchor=tk.W)
            red_in = base_data.get("reduction", {})
            r_grid = tk.Frame(c_red, bg=COLOR_BG_PANEL)
            r_grid.pack(fill=tk.X, pady=2)
            ent_r = []
            for idx, r_name, r_val in [
                (0, "M1 Base:", red_in.get("m1", 31.0)),
                (1, "M2 Hombro:", red_in.get("m2", 31.0)),
                (2, "M3 Codo:", red_in.get("m3", 31.0))
            ]:
                rf = tk.Frame(r_grid, bg=COLOR_BG_PANEL)
                rf.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
                tk.Label(rf, text=r_name, bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 7)).pack()
                er = tk.Entry(rf, bg="#ffffff", fg=COLOR_TEXT_PRIMARY, font=("Consolas", 8, "bold"), width=6, relief=tk.SOLID, bd=1)
                er.insert(0, f"{float(r_val):.1f}")
                er.pack()
                ent_r.append(er)
            tk.Label(c_red, text="31 vueltas de rotor NEMA 17 equivalen a 360° en la junta de salida.", bg=COLOR_BG_PANEL, fg="#64748b", font=("Segoe UI", 7)).pack(anchor=tk.W, pady=(2, 0))

            # ------------------------------------------------------------------
            # COLUMNA 2: SECCIÓN 6 - EFECTOR FINAL Y ALTURAS DEL GATO
            # ------------------------------------------------------------------
            c_tool = tk.Frame(col2, bg=COLOR_BG_PANEL, padx=10, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
            c_tool.pack(fill=tk.X)

            tk.Label(c_tool, text="6. Herramienta y Alturas de Trabajo (cm)", bg=COLOR_BG_PANEL, fg=COLOR_GREEN_SUCCESS, font=("Segoe UI", 9, "bold")).pack(anchor=tk.W)
            tool_in = base_data.get("tool", {})

            t_row = tk.Frame(c_tool, bg=COLOR_BG_PANEL)
            t_row.pack(fill=tk.X, pady=2)
            tk.Label(t_row, text="Tipo de Efector:", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 8)).pack(side=tk.LEFT)
            combo_tool = ttk.Combobox(t_row, state="readonly", font=("Segoe UI", 8), width=20,
                                      values=["Electroimán 12V (GPIO 7)", "Pinza / Gripper Servo", "Ventosa de Vacío"])
            cur_t = tool_in.get("type", "Electroimán 12V (GPIO 7)")
            combo_tool.set(cur_t if cur_t in combo_tool['values'] else "Electroimán 12V (GPIO 7)")
            combo_tool.pack(side=tk.LEFT, padx=6)

            h_row = tk.Frame(c_tool, bg=COLOR_BG_PANEL)
            h_row.pack(fill=tk.X, pady=3)
            tk.Label(h_row, text="Z_pick:", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 7)).pack(side=tk.LEFT)
            ent_z_pick = tk.Entry(h_row, bg="#ffffff", fg=COLOR_TEXT_PRIMARY, font=("Consolas", 8), width=5, relief=tk.SOLID, bd=1)
            ent_z_pick.insert(0, str(tool_in.get("pick_z", 6.0)))
            ent_z_pick.pack(side=tk.LEFT, padx=2)

            tk.Label(h_row, text="Z_place:", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 7)).pack(side=tk.LEFT, padx=(4, 0))
            ent_z_place = tk.Entry(h_row, bg="#ffffff", fg=COLOR_TEXT_PRIMARY, font=("Consolas", 8), width=5, relief=tk.SOLID, bd=1)
            ent_z_place.insert(0, str(tool_in.get("place_z", 3.0)))
            ent_z_place.pack(side=tk.LEFT, padx=2)

            tk.Label(h_row, text="Z_hover:", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 7)).pack(side=tk.LEFT, padx=(4, 0))
            ent_z_hover = tk.Entry(h_row, bg="#ffffff", fg=COLOR_TEXT_PRIMARY, font=("Consolas", 8), width=5, relief=tk.SOLID, bd=1)
            ent_z_hover.insert(0, str(tool_in.get("hover_z", 8.0)))
            ent_z_hover.pack(side=tk.LEFT, padx=2)

            # --- BOTONES DE GUARDAR Y VOLVER ---
            btn_bar_v2 = tk.Frame(view_container, bg=COLOR_BG_MAIN)
            btn_bar_v2.pack(fill=tk.X, padx=14, pady=(8, 14))

            tk.Button(
                btn_bar_v2, text="Volver a Selección", bg="#e2e8f0", fg=COLOR_TEXT_PRIMARY,
                font=("Segoe UI", 9, "bold"), relief=tk.SOLID, bd=1, padx=14, pady=7, cursor="hand2", command=render_welcome_view
            ).pack(side=tk.LEFT)

            def on_save_profile_and_enter():
                try:
                    p_name = ent_name.get().strip() or "Robot MK2 Personalizado"
                    new_profile = {
                        "name": p_name,
                        "description": "Perfil configurado desde el Asistente de Inicio",
                        "date": "2026-10-30",
                        "geometry": {
                            "d1": float(ent_d1.get()),
                            "a1": float(ent_a1.get()),
                            "a2": float(ent_a2.get()),
                            "a3": float(ent_a3.get())
                        },
                        "reduction": {
                            "m1": float(ent_r[0].get()),
                            "m2": float(ent_r[1].get()),
                            "m3": float(ent_r[2].get())
                        },
                        "limits": {
                            "m1": [min(float(ent_lims[0][0].get()), float(ent_lims[0][1].get())), max(float(ent_lims[0][0].get()), float(ent_lims[0][1].get()))],
                            "m2": [min(float(ent_lims[1][0].get()), float(ent_lims[1][1].get())), max(float(ent_lims[1][0].get()), float(ent_lims[1][1].get()))],
                            "m3": [min(float(ent_lims[2][0].get()), float(ent_lims[2][1].get())), max(float(ent_lims[2][0].get()), float(ent_lims[2][1].get()))]
                        },
                        "tool": {
                            "type": combo_tool.get(),
                            "pick_z": float(ent_z_pick.get()),
                            "place_z": float(ent_z_place.get()),
                            "hover_z": float(ent_z_hover.get())
                        },
                        "dynamics": {
                            "speed_mode": combo_speed.get(),
                            "max_speed_hz": 3600 if "3600" in combo_speed.get() else (1200 if "1200" in combo_speed.get() else 2400),
                            "accel_hz_s": 2500
                        },
                        "zero_calibrated": self.zero_calibrated
                    }

                    if "profiles" not in self.robot_config:
                        self.robot_config["profiles"] = {}
                    self.robot_config["profiles"][p_name] = new_profile
                    self.robot_config["current_profile"] = p_name
                    self.save_robot_config_file(self.robot_config)
                    self.apply_robot_profile(p_name, silent=True)

                    messagebox.showinfo(
                        "¡Robot Configurado!",
                        f"¡Características guardadas con éxito para el perfil:\n'{p_name}'!\n\n"
                        f"  • Posición Cero: Calibrada / Sincronizada\n"
                        f"  • Eslabones: d1={new_profile['geometry']['d1']}cm, a2={new_profile['geometry']['a2']}cm\n"
                        f"  • Reducción: 31:1 cicloidal\n"
                        f"  • Límites seguros aplicados al manipulador.\n\n"
                        f"Entrando al entorno de juego y control..."
                    )
                    on_modal_close()
                except ValueError as ve:
                    messagebox.showerror("Error en Valores", f"Revisa los campos numéricos ingresados:\n{ve}")

            tk.Button(
                btn_bar_v2, text="Guardar Características y Entrar",
                bg="#15803d", fg="#ffffff", activebackground="#166534",
                font=("Segoe UI", 9, "bold"), relief=tk.FLAT, padx=16, pady=7, cursor="hand2",
                command=on_save_profile_and_enter
            ).pack(side=tk.RIGHT)

            # Bucle de actualización en tiempo real de los ángulos en el formulario
            def update_v2_angles():
                if not modal_running[0] or not modal.winfo_exists():
                    return
                for idx in range(3):
                    if idx < len(ang_v2_labels):
                        ang_v2_labels[idx].config(text=f"{self.current_angles[idx]:+06.2f}°")
                modal.after(60, update_v2_angles)

            update_v2_angles()

        # Iniciar en la vista correspondiente
        if initial_view == "config":
            render_config_view(profile_data=self.robot_config.get("profiles", {}).get(self.active_profile_name))
        else:
            render_welcome_view()


    # ==========================================================================
    # GESTOR DE POSICIÓN CERO (ORIGEN 0.0° - PREVENCIÓN GIRO 360°)
    # ==========================================================================
    def update_zero_bar_ui(self):
        """Actualiza los indicadores y botones de la barra de Posición Cero."""
        def _do_update():
            try:
                if not hasattr(self, 'lbl_zero_status'):
                    return
                if self.motors_free_state:
                    self.lbl_zero_status.config(
                        text="Motores Libres (A mano)",
                        bg="#fef3c7", fg="#92400e", relief=tk.SOLID, bd=1
                    )
                    self.btn_toggle_motors.config(
                        text="Bloquear Motores",
                        bg="#0284c7"
                    )
                else:
                    if self.zero_calibrated:
                        self.lbl_zero_status.config(
                            text="POSICIÓN CERO: OK (0.0°)",
                            bg="#dcfce7", fg="#15803d", relief=tk.SOLID, bd=1
                        )
                    else:
                        self.lbl_zero_status.config(
                            text="MOTORES BLOQUEADOS (CON TORQUE)",
                            bg="#dbeafe", fg="#1e40af", relief=tk.SOLID, bd=1
                        )
                    self.btn_toggle_motors.config(
                        text="MOTORES LIBRES",
                        bg="#334155"
                    )
                if hasattr(self, 'lbl_zero_quick_angles'):
                    self.lbl_zero_quick_angles.config(
                        text=f"M1: {self.current_angles[0]:+06.2f}° | M2: {self.current_angles[1]:+06.2f}° | M3: {self.current_angles[2]:+06.2f}°"
                    )
            except Exception:
                pass
        self.dispatch_ui(_do_update)

    def toggle_motors_lock_free(self):
        """Alterna entre modo libre (sin torque para manipular con la mano) y bloqueo rígido."""
        if self.motors_free_state:
            self.cmd_motors_lock()
        else:
            self.cmd_motors_free()

    def cmd_save_game(self):
        """Guarda la postura física actual de los 3 motores como CERO absoluto en Flash NVS y perfil local."""
        self.pkt_seq = (self.pkt_seq + 1) % 10000
        payload = f"PKT,{self.pkt_seq},0,SAVE_GAME,0.0,0.0,0.0"
        cs = self.calculate_checksum(payload)
        self.send_serial_raw(f"${payload}*{cs:02X}\r\n")
        self.zero_calibrated = True
        self.motors_free_state = False
        if self.active_profile_name in self.robot_config.get("profiles", {}):
            self.robot_config["profiles"][self.active_profile_name]["zero_calibrated"] = True
            self.save_robot_config_file(self.robot_config)
        self.update_zero_bar_ui()
        for i in range(3):
            self.current_angles[i] = 0.0
            self.target_angles[i] = 0.0
            if hasattr(self, 'sliders_ax') and i < len(self.sliders_ax):
                self._suppress_slider_tx = True
                try:
                    self.sliders_ax[i].set(0.0)
                finally:
                    self._suppress_slider_tx = False
            if hasattr(self, 'entries_ax') and i < len(self.entries_ax):
                self.entries_ax[i].delete(0, tk.END)
                self.entries_ax[i].insert(0, "0.0")
        self.HOME_X = 55.0
        self.HOME_Y = 0.0
        self.HOME_Z = 10.0
        self.update_cartesian_entries(55.0, 0.0, 10.0)

    def cmd_load_game(self):
        """Carga el punto cero previamente guardado en Flash NVS y desplaza el robot suavemente a 0.0°."""
        self.pkt_seq = (self.pkt_seq + 1) % 10000
        payload = f"PKT,{self.pkt_seq},0,LOAD_GAME,0.0,0.0,0.0"
        cs = self.calculate_checksum(payload)
        self.send_serial_raw(f"${payload}*{cs:02X}\r\n")
        self.zero_calibrated = True
        self.motors_free_state = False
        self.target_angles = [0.0, 0.0, 0.0]
        for i in range(3):
            if hasattr(self, 'sliders_ax') and i < len(self.sliders_ax):
                self._suppress_slider_tx = True
                try:
                    self.sliders_ax[i].set(0.0)
                finally:
                    self._suppress_slider_tx = False
            if hasattr(self, 'entries_ax') and i < len(self.entries_ax):
                self.entries_ax[i].delete(0, tk.END)
                self.entries_ax[i].insert(0, "0.0")
        self.HOME_X = 55.0
        self.HOME_Y = 0.0
        self.HOME_Z = 10.0
        self.update_cartesian_entries(55.0, 0.0, 10.0)
        self.update_zero_bar_ui()
        # Enviar consigna de regreso al cero calibrado
        self.send_angles_command(0.0, 0.0, 0.0)

    def handle_stall_collision_alarm(self, motor_idx, target_deg=0.0, current_deg=0.0):
        """
        Maneja la detección de colisión o atasco mecánico por el Protocolo Anti-Choque:
        Si se emiten pulsos de movimiento a un motor pero el encoder AS5600 no detecta
        desplazamiento angular (motor atascado, choque con tuerca, obstáculo o fin de carrera),
        el firmware y la GUI detienen inmediatamente todo el movimiento para proteger reductor y driver.
        """
        # 1. Detener cualquier secuencia activa (Tour 1..9, Pick & Place o IA Gato)
        self.tour_active = False
        self.is_robot_busy = False

        # 2. Frenar consignas y sincronizar a la postura actual
        self.target_angles = list(self.current_angles)
        for k in range(3):
            if hasattr(self, 'sliders_ax') and k < len(self.sliders_ax):
                self._suppress_slider_tx = True
                try:
                    self.sliders_ax[k].set(self.current_angles[k])
                finally:
                    self._suppress_slider_tx = False
            if hasattr(self, 'entries_ax') and k < len(self.entries_ax):
                self.entries_ax[k].delete(0, tk.END)
                self.entries_ax[k].insert(0, f"{self.current_angles[k]:.1f}")

        # 3. Mostrar banner de alerta roja en la GUI
        joint_names = {1: "M1 (Base)", 2: "M2 (Hombro)", 3: "M3 (Codo)"}
        name_str = joint_names.get(motor_idx, f"M{motor_idx}")
        msg = (f"[ALERTA ANTI-CHOQUE] {name_str} bloqueado sin movimiento en encoder "
               f"(Objetivo: {target_deg:.1f}°, Medido: {current_deg:.1f}°). Pulsos detenidos de inmediato.")
        if hasattr(self, 'lbl_stall_banner') and hasattr(self, 'frame_stall_banner'):
            self.lbl_stall_banner.config(text=msg)
            try:
                self.frame_stall_banner.pack(fill=tk.X, padx=12, pady=(0, 6), before=self.main_frame)
            except Exception:
                self.frame_stall_banner.pack(fill=tk.X, padx=12, pady=(0, 6))

        print(f"[ANTI-CHOQUE] {msg}")

    def cmd_motors_free(self):
        """Desactiva los drivers TB6600 para mover el robot libremente con las manos."""
        self.pkt_seq = (self.pkt_seq + 1) % 10000
        payload = f"PKT,{self.pkt_seq},0,FREE,0.0,0.0,0.0"
        cs = self.calculate_checksum(payload)
        self.send_serial_raw(f"${payload}*{cs:02X}\r\n")
        self.send_serial_raw("FREE\n")
        self.send_serial_raw("STOP\n")
        self.motors_free_state = True
        self.update_zero_bar_ui()

    def cmd_motors_lock(self):
        """Activa el torque de retención suave en la postura actual sin generar movimiento."""
        self.pkt_seq = (self.pkt_seq + 1) % 10000
        payload = f"PKT,{self.pkt_seq},0,LOCK,0.0,0.0,0.0"
        cs = self.calculate_checksum(payload)
        self.send_serial_raw(f"${payload}*{cs:02X}\r\n")
        self.send_serial_raw("LOCK\n")
        self.motors_free_state = False
        self.update_zero_bar_ui()

    def show_zero_calibration_modal(self):
        """
        Abre la ventana interactiva de Calibración de Posición Cero (0.0°).
        Evita que M1 gire 360° al iniciar y permite acomodar el robot a mano antes de guardar.
        """
        self.zero_modal_shown = True
        modal = tk.Toplevel(self)
        modal.title("Gestor de Posición Cero — Calibración de Origen (Robot 3 GDL)")
        modal.geometry("840x630")
        modal.minsize(760, 560)
        modal.configure(bg=COLOR_BG_MAIN)
        modal.transient(self)

        modal_running = [True]
        def on_modal_close():
            modal_running[0] = False
            modal.destroy()
        modal.protocol("WM_DELETE_WINDOW", on_modal_close)

        # Header de Calibración de Posición Cero
        hdr = tk.Frame(modal, bg=COLOR_BG_PANEL, padx=16, pady=12, highlightthickness=1, highlightbackground=COLOR_BORDER)
        hdr.pack(fill=tk.X, padx=14, pady=(12, 8))

        top_row = tk.Frame(hdr, bg=COLOR_BG_PANEL)
        top_row.pack(fill=tk.X)
        tk.Label(
            top_row, text="GESTOR DE POSICIÓN CERO — CALIBRACIÓN",
            bg=COLOR_BG_PANEL, fg=COLOR_ACCENT, font=("Segoe UI", 13, "bold")
        ).pack(side=tk.LEFT)

        status_pill = tk.Label(
            top_row,
            text="MOTORES LIBRES (SIN TORQUE)" if self.motors_free_state else "MOTORES BLOQUEADOS",
            bg="#78350f" if self.motors_free_state else "#064e3b",
            fg="#fbbf24" if self.motors_free_state else "#34d399",
            font=("Segoe UI", 9, "bold"), padx=10, pady=3
        )
        status_pill.pack(side=tk.RIGHT)

        tk.Label(
            hdr,
            text="Al encender el robot, los 3 motores arrancan en REPOSO (sin torque) para evitar que M1 gire 360°.\n"
                 "Elige una de las siguientes opciones para inicializar tu posición de juego:",
            bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 9), justify=tk.LEFT
        ).pack(anchor=tk.W, pady=(6, 0))

        # Tarjetas de monitoreo en vivo de los 3 encoders AS5600
        cards_frame = tk.Frame(modal, bg=COLOR_BG_MAIN)
        cards_frame.pack(fill=tk.X, padx=14, pady=6)

        ang_cards = []
        joint_names = ["M1 (Base - Giro)", "M2 (Hombro - Elevación)", "M3 (Codo - Alcance)"]
        colors = ["#38bdf8", "#34d399", "#a78bfa"]

        for i in range(3):
            c_box = tk.Frame(cards_frame, bg=COLOR_BG_PANEL, padx=12, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
            c_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)

            tk.Label(c_box, text=joint_names[i], bg=COLOR_BG_PANEL, fg=COLOR_CYAN_ACCENT, font=("Segoe UI", 9, "bold")).pack(anchor=tk.W)
            lbl_ang = tk.Label(c_box, text=f"{self.current_angles[i]:+06.2f}°", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Consolas", 18, "bold"))
            lbl_ang.pack(pady=4)
            lbl_info = tk.Label(c_box, text="Sensor AS5600: Esperando...", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8))
            lbl_info.pack(anchor=tk.W)
            ang_cards.append((lbl_ang, lbl_info))

        # Contenedor de las 3 opciones de juego
        opts_frame = tk.Frame(modal, bg=COLOR_BG_MAIN)
        opts_frame.pack(fill=tk.BOTH, expand=True, padx=14, pady=8)

        # OPCIÓN 1: NUEVA PARTIDA (Guardar Posición Actual como Cero)
        card_new = tk.Frame(opts_frame, bg=COLOR_BG_PANEL, padx=14, pady=10, highlightthickness=1, highlightbackground=COLOR_BORDER)
        card_new.pack(fill=tk.X, pady=4)

        cnew_top = tk.Frame(card_new, bg=COLOR_BG_PANEL)
        cnew_top.pack(fill=tk.X)
        tk.Label(cnew_top, text="GUARDAR POSICIÓN CERO ACTUAL (0.0°)", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 11, "bold")).pack(side=tk.LEFT)
        tk.Label(cnew_top, text="Recomendado para calibrar", bg="#dcfce7", fg="#15803d", font=("Segoe UI", 8, "bold"), padx=6, pady=2).pack(side=tk.RIGHT)

        tk.Label(
            card_new,
            text="Acomoda el robot en su postura inicial (marcas de cero o plantilla). "
                 "Al pulsar aquí, la posición física actual se define como 0.00° en los 3 ejes, "
                 "se almacena permanentemente en la memoria Flash (NVS) del ESP32 y los motores se bloquean suavemente sin girar.",
            bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8), justify=tk.LEFT, wraplength=740
        ).pack(anchor=tk.W, pady=4)

        def on_save_click():
            self.cmd_save_game()
            status_pill.config(text="POSICIÓN CERO: OK (0.0°)", bg="#dcfce7", fg="#15803d", relief=tk.SOLID, bd=1)
            messagebox.showinfo(
                "Posición Cero Guardada",
                "¡Posición CERO guardada con éxito en la memoria Flash del ESP32!\n\n"
                "  • M1 (Base):   0.00°\n"
                "  • M2 (Hombro): 0.00°\n"
                "  • M3 (Codo):   0.00°\n\n"
                "Los motores ahora mantienen torque de retención suave sin giros bruscos."
            )
            on_modal_close()

        tk.Button(
            card_new, text="GUARDAR POSICIÓN ACTUAL COMO CERO (0.0°)",
            bg="#059669", fg="#ffffff", activebackground="#047857",
            font=("Segoe UI", 9, "bold"), padx=14, pady=6, bd=0, cursor="hand2",
            command=on_save_click
        ).pack(anchor=tk.E, pady=(4, 0))

        # OPCIÓN 2: CONTINUAR PARTIDA (Cargar de Memoria Flash)
        card_load = tk.Frame(opts_frame, bg=COLOR_BG_PANEL, padx=14, pady=10, highlightthickness=1, highlightbackground=COLOR_BORDER)
        card_load.pack(fill=tk.X, pady=4)

        cload_top = tk.Frame(card_load, bg=COLOR_BG_PANEL)
        cload_top.pack(fill=tk.X)
        tk.Label(cload_top, text="CARGAR POSICIÓN CERO GUARDADA", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 11, "bold")).pack(side=tk.LEFT)
        tk.Label(cload_top, text="MEMORIA FLASH NVS", bg="#dbeafe", fg="#1e40af", font=("Segoe UI", 8, "bold"), padx=6, pady=2).pack(side=tk.RIGHT)

        tk.Label(
            card_load,
            text="Recupera los ceros calibrados de la Flash NVS del ESP32 y comanda a los 3 motores "
                 "a desplazarse suavemente de regreso a su postura cero de origen (0.00°, 0.00°, 0.00°).",
            bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8), justify=tk.LEFT, wraplength=740
        ).pack(anchor=tk.W, pady=4)

        def on_load_click():
            self.cmd_load_game()
            status_pill.config(text="CERO CARGADO (0.0°)", bg="#dbeafe", fg="#1e40af", relief=tk.SOLID, bd=1)
            messagebox.showinfo(
                "Posición Cero Cargada",
                "Se cargó la calibración de cero desde la memoria Flash NVS.\n\n"
                "El robot se está desplazando suavemente de regreso a su posición cero (0.0°, 0.0°, 0.0°)."
            )
            on_modal_close()

        tk.Button(
            card_load, text="Cargar Cero (0.0°)",
            bg="#0284c7", fg="#ffffff", activebackground="#0369a1",
            font=("Segoe UI", 9, "bold"), padx=14, pady=6, bd=0, cursor="hand2",
            command=on_load_click
        ).pack(anchor=tk.E, pady=(4, 0))

        # OPCIÓN 3: MODO LIBRE (Acomodar Robot con la Mano)
        card_free = tk.Frame(opts_frame, bg=COLOR_BG_PANEL, padx=14, pady=10, highlightthickness=1, highlightbackground=COLOR_BORDER)
        card_free.pack(fill=tk.X, pady=4)

        cfree_top = tk.Frame(card_free, bg=COLOR_BG_PANEL)
        cfree_top.pack(fill=tk.X)
        tk.Label(cfree_top, text="MODO LIBRE (ACOMODAR CON LA MANO)", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 11, "bold")).pack(side=tk.LEFT)
        tk.Label(cfree_top, text="Calibración Manual", bg="#fef3c7", fg="#92400e", font=("Segoe UI", 8, "bold"), padx=6, pady=2).pack(side=tk.RIGHT)

        tk.Label(
            card_free,
            text="Desactiva la señal ENA de los 3 drivers TB6600 para que los ejes queden sin fuerza. "
                 "Puedes mover el robot con la mano con total suavidad mientras observas los ángulos en vivo arriba. "
                 "Una vez colocado en su posición inicial, pulsa 'Guardar Posición Actual como Cero'.",
            bg=COLOR_BG_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8), justify=tk.LEFT, wraplength=740
        ).pack(anchor=tk.W, pady=4)

        def on_free_click():
            self.cmd_motors_free()
            status_pill.config(text="MOTORES LIBRES (SIN TORQUE)", bg="#fef3c7", fg="#92400e", relief=tk.SOLID, bd=1)

        def on_lock_click():
            self.cmd_motors_lock()
            status_pill.config(text="MOTORES BLOQUEADOS", bg="#dcfce7", fg="#15803d", relief=tk.SOLID, bd=1)

        b_row = tk.Frame(card_free, bg=COLOR_BG_PANEL)
        b_row.pack(anchor=tk.E, pady=(4, 0))

        tk.Button(
            b_row, text="LIBERAR MOTORES (SIN FUERZA)",
            bg="#d97706", fg="#ffffff", activebackground="#b45309",
            font=("Segoe UI", 8, "bold"), padx=10, pady=5, bd=0, cursor="hand2",
            command=on_free_click
        ).pack(side=tk.LEFT, padx=3)

        tk.Button(
            b_row, text="BLOQUEAR (CON TORQUE)",
            bg="#334155", fg="#ffffff", activebackground="#475569",
            font=("Segoe UI", 8, "bold"), padx=10, pady=5, bd=0, cursor="hand2",
            command=on_lock_click
        ).pack(side=tk.LEFT, padx=3)

        # Barra Inferior con enlaces rápidos
        btn_bar = tk.Frame(modal, bg=COLOR_BG_MAIN)
        btn_bar.pack(fill=tk.X, padx=14, pady=(6, 12))

        tk.Button(
            btn_bar, text="Abrir Plantilla Gato", bg="#1e40af", fg="#ffffff", relief=tk.FLAT,
            activebackground="#1d4ed8", font=("Segoe UI", 8, "bold"), padx=10, pady=6,
            cursor="hand2", command=self.show_board_position_template_modal
        ).pack(side=tk.LEFT, padx=4)

        tk.Button(
            btn_bar, text="Cerrar Asistente", bg="#e2e8f0", fg=COLOR_TEXT_PRIMARY, relief=tk.SOLID, bd=1,
            activebackground="#cbd5e1", font=("Segoe UI", 9, "bold"), padx=16, pady=6,
            cursor="hand2", command=on_modal_close
        ).pack(side=tk.RIGHT)

        # Bucle de actualización en tiempo real de los ángulos en el modal
        def update_modal_angles():
            if not modal_running[0] or not modal.winfo_exists():
                return
            for i in range(3):
                lbl_ang, lbl_info = ang_cards[i]
                lbl_ang.config(text=f"{self.current_angles[i]:+06.2f}°")
                st = self.encoder_status[i]
                if st["connected"] and st["magnet"]:
                    lbl_info.config(text=f"AS5600: OK (Imán Óptimo) | Raw: {st['raw']:04d}", fg="#34d399")
                elif st["connected"]:
                    lbl_info.config(text=f"AS5600: I2C OK (Sin Imán) | Raw: {st['raw']:04d}", fg="#fbbf24")
                else:
                    lbl_info.config(text="AS5600: Esperando señal...", fg="#64748b")
            modal.after(50, update_modal_angles)

        update_modal_angles()

    def open_standalone_encoder_tester(self):
        """Abre la herramienta gráfica dedicada de prueba 360° de encoders (test_encoders.py)."""
        import subprocess, os
        if self.running:
            self.handle_serial_disconnect()
        script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_encoders.py")
        subprocess.Popen([sys.executable, script_path])

    def cmd_check_encoders(self):
        """Envía solicitud inmediata de prueba de encoders al ESP32."""
        self.pkt_seq = (self.pkt_seq + 1) % 10000
        payload = f"PKT,{self.pkt_seq},0,CHECK_ENC,0.0,0.0,0.0"
        cs = self.calculate_checksum(payload)
        self.send_serial_raw(f"${payload}*{cs:02X}\r\n")
        self.send_serial_raw("CHECK_ENC\r\n")

    def show_encoder_diagnostics_modal(self):
        """Abre una ventana emergente interactiva de diagnóstico detallado para los 3 encoders AS5600."""
        self.cmd_check_encoders()
        
        modal = tk.Toplevel(self)
        modal.title("Diagnóstico de Encoders AS5600 (3 Buses I2C Dedicados)")
        modal.geometry("620x520")
        modal.configure(bg=COLOR_BG_MAIN)
        modal.transient(self)
        modal.grab_set()

        tk.Label(
            modal, text="DIAGNÓSTICO EN TIEMPO REAL: ENCODERS AS5600",
            bg=COLOR_BG_MAIN, fg=COLOR_CYAN_ACCENT, font=("Segoe UI", 12, "bold")
        ).pack(pady=(12, 4))
        
        tk.Label(
            modal, text="Monitoreo independiente de los 3 buses I2C dedicados en ESP32-S3",
            bg=COLOR_BG_MAIN, fg="#94a3b8", font=("Segoe UI", 8)
        ).pack(pady=(0, 10))

        cards_frame = tk.Frame(modal, bg=COLOR_BG_MAIN)
        cards_frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=4)

        motors_meta = [
            {"name": "Motor 1: BASE (Yaw / J1)", "sda": "GPIO 1", "scl": "GPIO 2", "bus": "Bus I2C #1"},
            {"name": "Motor 2: HOMBRO (Pitch / J2)", "sda": "GPIO 8", "scl": "GPIO 9", "bus": "Bus I2C #2"},
            {"name": "Motor 3: CODO (Pitch / J3)", "sda": "GPIO 10", "scl": "GPIO 11", "bus": "Bus I2C #3"}
        ]

        card_widgets = []

        for ax in range(3):
            meta = motors_meta[ax]
            card = tk.Frame(cards_frame, bg=COLOR_BG_CARD, padx=12, pady=10, highlightthickness=1, highlightbackground=COLOR_BORDER_FOCUS)
            card.pack(fill=tk.X, pady=4)

            # Fila superior: Título y Pines
            r_top = tk.Frame(card, bg=COLOR_BG_CARD)
            r_top.pack(fill=tk.X)
            tk.Label(r_top, text=meta["name"], bg=COLOR_BG_CARD, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
            tk.Label(r_top, text=f"{meta['bus']} | SDA: {meta['sda']} | SCL: {meta['scl']}", bg=COLOR_BG_CARD, fg="#38bdf8", font=("Consolas", 8)).pack(side=tk.RIGHT)

            # Fila media: Estado y Raw
            r_mid = tk.Frame(card, bg=COLOR_BG_CARD)
            r_mid.pack(fill=tk.X, pady=(6, 2))
            lbl_st = tk.Label(r_mid, text="Consultando...", bg="#1e293b", fg="#94a3b8", font=("Segoe UI", 8, "bold"), padx=8, pady=3)
            lbl_st.pack(side=tk.LEFT)
            lbl_details = tk.Label(r_mid, text="Ángulo: --.-° | Raw: ----", bg=COLOR_BG_CARD, fg=COLOR_TEXT_PRIMARY, font=("Consolas", 8))
            lbl_details.pack(side=tk.RIGHT)

            # Fila inferior: Consejo o solución
            lbl_hint = tk.Label(card, text="...", bg=COLOR_BG_CARD, fg="#94a3b8", font=("Segoe UI", 7, "italic"), wraplength=560, justify=tk.LEFT)
            lbl_hint.pack(anchor=tk.W, pady=(4, 0))

            card_widgets.append((lbl_st, lbl_details, lbl_hint))

        def refresh_modal():
            if not self.running:
                for lbl_st, lbl_details, lbl_hint in card_widgets:
                    lbl_st.config(text="PUERTO SERIAL CERRADO", bg="#450a0a", fg="#f87171")
                    lbl_details.config(text="Conecta el ESP32 en la ventana principal")
                    lbl_hint.config(text="Selecciona el puerto COM correspondiente y pulsa 'CONECTAR ESP32'.")
                return

            for ax in range(3):
                st = self.encoder_status[ax]
                lbl_st, lbl_details, lbl_hint = card_widgets[ax]
                ang = self.current_angles[ax]
                raw = st["raw"]

                if st["connected"] and st["magnet"]:
                    lbl_st.config(text=" CONECTADO (IMÁN ÓPTIMO)", bg="#dcfce7", fg="#15803d", relief=tk.SOLID, bd=1)
                    lbl_details.config(text=f"Ángulo: {ang:+06.2f}° | Raw: {raw:04d} (12-bit)", fg="#38bdf8")
                    lbl_hint.config(
                        text="● Comunicación I2C estable. El campo magnético está en el rango óptimo (1.0 - 2.5 mm del chip).",
                        fg="#86efac"
                    )
                elif st["connected"] and not st["magnet"]:
                    lbl_st.config(text="I2C RESPONDE | SIN IMÁN", bg="#fef3c7", fg="#92400e", relief=tk.SOLID, bd=1)
                    lbl_details.config(text=f"Ángulo: {ang:+06.2f}° | Raw: {raw:04d}", fg="#fbbf24")
                    lbl_hint.config(
                        text="▲ El chip responde en la dirección 0x36, pero el sensor no detecta el imán diametral de neodimio o está muy lejos/desalineado.",
                        fg="#fde047"
                    )
                else:
                    meta = motors_meta[ax]
                    lbl_st.config(text="● DESCONECTADO (ERROR I2C)", bg="#fee2e2", fg="#991b1b", relief=tk.SOLID, bd=1)
                    lbl_details.config(text="No hay respuesta en bus I2C", fg="#f87171")
                    lbl_hint.config(
                        text=f" El ESP32 no recibe ACK en el bus. Revisa los cables de {meta['sda']} (SDA) y {meta['scl']} (SCL), la alimentación de 3.3V/GND o soldaduras frías en la PCB.",
                        fg="#fca5a5"
                    )

            modal.after(100, refresh_modal)

        btn_bar = tk.Frame(modal, bg=COLOR_BG_MAIN)
        btn_bar.pack(fill=tk.X, padx=16, pady=(6, 14))

        tk.Button(
            btn_bar, text="Actualizar Diagnóstico", bg="#0284c7", fg="#ffffff",
            activebackground="#0369a1", font=("Segoe UI", 8, "bold"), bd=0, padx=10, pady=6,
            cursor="hand2", command=self.cmd_check_encoders
        ).pack(side=tk.LEFT, padx=(0, 4))

        tk.Button(
            btn_bar, text="LIBERAR EJES (STOP)", bg="#065f46", fg="#a7f3d0",
            activebackground="#047857", font=("Segoe UI", 8, "bold"), bd=0, padx=10, pady=6,
            cursor="hand2", command=self.cmd_stop
        ).pack(side=tk.LEFT, padx=4)

        tk.Button(
            btn_bar, text="CERO A TODOS", bg="#1e293b", fg="#38bdf8",
            activebackground="#334155", font=("Segoe UI", 8, "bold"), bd=0, padx=10, pady=6,
            cursor="hand2", command=lambda: self.cmd_zero_encoder(0)
        ).pack(side=tk.LEFT, padx=4)

        tk.Button(
            btn_bar, text=" ABRIR TESTER 360°", bg="#7c3aed", fg="#ffffff",
            activebackground="#6d28d9", font=("Segoe UI", 8, "bold"), bd=0, padx=10, pady=6,
            cursor="hand2", command=lambda: (modal.destroy(), self.open_standalone_encoder_tester())
        ).pack(side=tk.LEFT, padx=4)

        tk.Button(
            btn_bar, text="CERRAR", bg="#334155", fg="#ffffff",
            activebackground="#475569", font=("Segoe UI", 8, "bold"), bd=0, padx=12, pady=6,
            cursor="hand2", command=modal.destroy
        ).pack(side=tk.RIGHT)

        refresh_modal()

    def open_printable_template(self):
        """Abre la plantilla física a escala 1:1 en PDF para imprimir en papel."""
        import os, subprocess
        pdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Plantilla_Gato_Escala_1a1.pdf")
        if os.path.exists(pdf_path):
            try:
                if sys.platform.startswith("win"):
                    os.startfile(pdf_path)
                else:
                    subprocess.Popen(["xdg-open", pdf_path])
                return
            except Exception as e:
                messagebox.showerror("Error", f"No se pudo abrir el PDF:\n{e}")
        else:
            messagebox.showerror("Archivo no encontrado", f"No se encontró el archivo:\n{pdf_path}")

    def launch_calibrador_casillas(self):
        """Lanza la herramienta gráfica dedicada de calibración de casillas y ajuste de cinemática inversa."""
        import subprocess, sys
        calib_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibrador_casillas.py")
        if os.path.exists(calib_script):
            subprocess.Popen([sys.executable, calib_script])
        else:
            messagebox.showerror("Archivo no encontrado", f"No se encontró:\n{calib_script}")

    def show_board_position_template_modal(self):
        """
        Abre una ventana interactiva de alta precisión con la plantilla 2D ortográfica
        del tablero de gato para verificar en tiempo real si el robot está centrado
        y en posición exacta sobre cualquiera de las 9 casillas.
        """
        modal = tk.Toplevel(self)
        modal.title("Plantilla de Verificación de Posición y Calibración - Gato 3x3")
        modal.geometry("960x650")
        modal.minsize(860, 580)
        modal.configure(bg=COLOR_BG_MAIN)
        modal.transient(self)

        selected_cell_idx = tk.IntVar(value=4)  # Casilla 5 (Centro) por defecto
        view_mode_zoom = tk.BooleanVar(value=False)
        modal_running = [True]

        def on_modal_close():
            modal_running[0] = False
            modal.destroy()

        modal.protocol("WM_DELETE_WINDOW", on_modal_close)

        # Encabezado superior del modal
        top_m = tk.Frame(modal, bg=COLOR_BG_PANEL, padx=14, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
        top_m.pack(fill=tk.X, padx=12, pady=(10, 6))

        h_info = tk.Frame(top_m, bg=COLOR_BG_PANEL)
        h_info.pack(side=tk.LEFT)
        tk.Label(h_info, text="PLANTILLA DE CALIBRACIÓN Y VERIFICACIÓN DE POSICIÓN DEL GATO", bg=COLOR_BG_PANEL, fg=COLOR_CYAN_ACCENT, font=("Segoe UI", 11, "bold")).pack(anchor=tk.W)
        tk.Label(h_info, text="Comprobación visual y milimétrica en tiempo real: Compara la posición real de los encoders con el centro de las casillas.", bg=COLOR_BG_PANEL, fg="#94a3b8", font=("Segoe UI", 8)).pack(anchor=tk.W)

        btn_print = tk.Button(
            top_m, text="Imprimir Plantilla 1:1 en Papel", bg="#1e40af", fg="#ffffff",
            activebackground="#0369a1", font=("Segoe UI", 9, "bold"), padx=12, pady=4, bd=0, relief=tk.FLAT,
            cursor="hand2", command=self.open_printable_template
        )
        btn_print.pack(side=tk.RIGHT)

        # Contenedor principal dividido en dos columnas (Canvas 2D izquierda / Diagnóstico derecha)
        body = tk.Frame(modal, bg=COLOR_BG_MAIN)
        body.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 10))

        # --- COLUMNA IZQUIERDA: CANVAS 2D DE LA PLANTILLA ---
        f_left = tk.Frame(body, bg=COLOR_BG_PANEL, padx=8, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
        f_left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6))

        bar_canvas_tools = tk.Frame(f_left, bg=COLOR_BG_PANEL)
        bar_canvas_tools.pack(fill=tk.X, pady=(0, 4))
        tk.Label(bar_canvas_tools, text="PLANTILLA 2D CENITAL (VISTA SUPERIOR DE LA MESA):", bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT)

        chk_zoom = tk.Checkbutton(
            bar_canvas_tools, text="Zoom al Tablero 3x3", variable=view_mode_zoom,
            bg=COLOR_BG_PANEL, fg=COLOR_TEXT_PRIMARY, selectcolor="#ffffff", activebackground=COLOR_BG_PANEL,
            activeforeground=COLOR_CYAN_ACCENT, font=("Segoe UI", 8, "bold")
        )
        chk_zoom.pack(side=tk.RIGHT)

        canvas_tpl = tk.Canvas(f_left, bg="#ffffff", highlightthickness=1, highlightbackground=COLOR_BORDER)
        canvas_tpl.pack(fill=tk.BOTH, expand=True)

        # --- COLUMNA DERECHA: SELECCIÓN Y TELEMETRÍA DE ALINEACIÓN ---
        f_right = tk.Frame(body, bg=COLOR_BG_PANEL, padx=10, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER, width=380)
        f_right.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(6, 0))
        f_right.pack_propagate(False)

        # 1. Tarjeta: Selector de Casilla Objetivo
        card_sel = tk.Frame(f_right, bg=COLOR_BG_CARD, padx=8, pady=6, highlightthickness=1, highlightbackground=COLOR_BORDER_FOCUS)
        card_sel.pack(fill=tk.X, pady=(0, 6))
        tk.Label(card_sel, text="CASILLA A COMPROBAR (1 a 9):", bg=COLOR_BG_CARD, fg=COLOR_AMBER_WARNING, font=("Segoe UI", 8, "bold")).pack(anchor=tk.W)

        grid_btns = tk.Frame(card_sel, bg=COLOR_BG_CARD)
        grid_btns.pack(pady=4)
        c_buttons = []
        for i in range(9):
            btn_ci = tk.Button(
                grid_btns, text=f"C{i+1}", width=4, font=("Segoe UI", 9, "bold"),
                bg="#1e293b" if i != 4 else "#0284c7", fg="#ffffff", activebackground="#0369a1", bd=0, cursor="hand2",
                command=lambda idx=i: on_select_cell(idx)
            )
            btn_ci.grid(row=i//3, column=i%3, padx=2, pady=2)
            c_buttons.append(btn_ci)

        def on_select_cell(idx):
            selected_cell_idx.set(idx)
            for j, b in enumerate(c_buttons):
                b.config(bg="#0284c7" if j == idx else "#1e293b")
            lbl_tgt_info.config(text=f"Objetivo: Casilla {idx+1} | X={self.cell_coords[idx][0]:.1f}, Y={self.cell_coords[idx][1]:.1f}, Z=3.0 cm")

        lbl_tgt_info = tk.Label(card_sel, text="Objetivo: Casilla 5 | X=26.0, Y=0.0, Z=3.0 cm", bg=COLOR_BG_CARD, fg=COLOR_CYAN_ACCENT, font=("Consolas", 8, "bold"))
        lbl_tgt_info.pack(pady=(2, 4))

        # Botón para comandar al robot a la casilla
        f_action_btns = tk.Frame(card_sel, bg=COLOR_BG_CARD)
        f_action_btns.pack(fill=tk.X, pady=2)

        btn_go_place = tk.Button(
            f_action_btns, text="▶ IR (TABLERO Z=3)", bg="#059669", fg="#ffffff", activebackground="#047857",
            font=("Segoe UI", 8, "bold"), bd=0, pady=3, cursor="hand2",
            command=lambda: self.cmd_goto_cell(selected_cell_idx.get())
        )
        btn_go_place.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        btn_go_hover = tk.Button(
            f_action_btns, text="▲ HOVER (Z=8)", bg="#0284c7", fg="#ffffff", activebackground="#0369a1",
            font=("Segoe UI", 8, "bold"), bd=0, pady=3, cursor="hand2",
            command=lambda: (self.cell_action_mode.set("hover"), self.cmd_goto_cell(selected_cell_idx.get()), self.cell_action_mode.set("place"))
        )
        btn_go_hover.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        # 2. Tarjeta: Semáforo y Métricas de Precisión Milimétrica
        card_metrics = tk.Frame(f_right, bg=COLOR_BG_CARD, padx=8, pady=6, highlightthickness=1, highlightbackground=COLOR_BORDER_FOCUS)
        card_metrics.pack(fill=tk.X, pady=(0, 6))

        tk.Label(card_metrics, text="Evaluación de Alineación:", bg=COLOR_BG_CARD, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 8, "bold")).pack(anchor=tk.W)

        lbl_semaforo = tk.Label(
            card_metrics, text="EVALUANDO POSICIÓN...", bg="#1e293b", fg="#94a3b8",
            font=("Segoe UI", 11, "bold"), pady=6
        )
        lbl_semaforo.pack(fill=tk.X, pady=(4, 6))

        lbl_err_radial = tk.Label(card_metrics, text="Desviación Radial: 0.0 mm", bg=COLOR_BG_CARD, fg=COLOR_TEXT_PRIMARY, font=("Consolas", 10, "bold"))
        lbl_err_radial.pack(anchor=tk.W)

        lbl_coord_real = tk.Label(card_metrics, text="Pos. Actual (FK): X=--.-, Y=--.-, Z=--.- cm", bg=COLOR_BG_CARD, fg="#94a3b8", font=("Consolas", 8))
        lbl_coord_real.pack(anchor=tk.W, pady=(2, 0))

        lbl_deltas = tk.Label(card_metrics, text="ΔX: 0.0 mm | ΔY: 0.0 mm | ΔZ: 0.0 mm", bg=COLOR_BG_CARD, fg="#94a3b8", font=("Consolas", 8))
        lbl_deltas.pack(anchor=tk.W)

        lbl_joint_errors = tk.Label(card_metrics, text="Error Articular: J1: 0.0° | J2: 0.0° | J3: 0.0°", bg=COLOR_BG_CARD, fg="#64748b", font=("Consolas", 7))
        lbl_joint_errors.pack(anchor=tk.W, pady=(2, 0))

        # 3. Tarjeta: Controles Rápidos y Ayuda
        card_actions = tk.Frame(f_right, bg=COLOR_BG_CARD, padx=8, pady=6, highlightthickness=1, highlightbackground=COLOR_BORDER_FOCUS)
        card_actions.pack(fill=tk.X)

        tk.Label(card_actions, text="ACCIONES RÁPIDAS:", bg=COLOR_BG_CARD, fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 8, "bold")).pack(anchor=tk.W)

        r_quick = tk.Frame(card_actions, bg=COLOR_BG_CARD)
        r_quick.pack(fill=tk.X, pady=4)

        tk.Button(r_quick, text="Home", bg="#1e40af", fg="#ffffff", font=("Segoe UI", 8, "bold"), relief=tk.FLAT, padx=6, pady=3, cursor="hand2", command=self.cmd_home).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
        tk.Button(r_quick, text="Herramienta", bg="#e2e8f0", fg=COLOR_TEXT_PRIMARY, font=("Segoe UI", 8, "bold"), relief=tk.SOLID, bd=1, padx=6, pady=3, cursor="hand2", command=self.toggle_gripper).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
        tk.Button(r_quick, text="PARO", bg=COLOR_RED_DANGER, fg="#ffffff", font=("Segoe UI", 8, "bold"), bd=0, padx=6, pady=3, cursor="hand2", command=self.cmd_stop).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        # Función recurrente de renderizado de la plantilla 2D a 25 FPS
        def update_template_canvas():
            if not modal_running[0] or not modal.winfo_exists():
                return

            w = canvas_tpl.winfo_width()
            h = canvas_tpl.winfo_height()
            if w < 50 or h < 50:
                modal.after(50, update_template_canvas)
                return

            canvas_tpl.delete("all")

            # Cinemática Directa del robot en tiempo real
            q1, q2, q3 = self.current_angles
            x_real, y_real, z_real = self.kin.forward_kinematics(q1, q2, q3)

            # Casilla objetivo
            c_idx = selected_cell_idx.get()
            x_tgt, y_tgt, z_tgt = self.cell_coords[c_idx]

            # Errores en milímetros
            dx_mm = (x_real - x_tgt) * 10.0
            dy_mm = (y_real - y_tgt) * 10.0
            dz_mm = (z_real - z_tgt) * 10.0
            dr_mm = math.sqrt(dx_mm**2 + dy_mm**2)

            # Actualizar Semáforo y textos
            lbl_coord_real.config(text=f"Pos. Actual (FK): X={x_real:5.1f}, Y={y_real:5.1f}, Z={z_real:5.1f} cm")
            lbl_deltas.config(text=f"ΔX: {dx_mm:+5.1f} mm | ΔY: {dy_mm:+5.1f} mm | ΔZ: {dz_mm:+5.1f} mm")
            lbl_err_radial.config(text=f"Desviación Radial: {dr_mm:5.1f} mm ({dr_mm/10.0:4.2f} cm)")

            # Ángulos consigna esperados
            tgt_angles, _, _ = self.kin.inverse_kinematics(x_tgt, y_tgt, z_tgt)
            dj1 = q1 - tgt_angles[0]
            dj2 = q2 - tgt_angles[1]
            dj3 = q3 - tgt_angles[2]
            lbl_joint_errors.config(text=f"Error Articular: J1:{dj1:+.1f}° | J2:{dj2:+.1f}° | J3:{dj3:+.1f}°")

            if dr_mm <= 8.0:
                lbl_semaforo.config(
                    text=f"EN POSICIÓN (Alineado: {dr_mm:.1f} mm)",
                    bg="#dcfce7", fg="#15803d", relief=tk.SOLID, bd=1
                )
            elif dr_mm <= 25.0:
                lbl_semaforo.config(
                    text=f"Aproximando... ({dr_mm:.1f} mm)",
                    bg="#fef3c7", fg="#92400e", relief=tk.SOLID, bd=1
                )
            else:
                lbl_semaforo.config(
                    text=f"Fuera de Casilla ({dr_mm:.1f} mm)",
                    bg="#450a0a", fg="#f87171"
                )

            # Configuración de proyección 2D
            is_zoom = view_mode_zoom.get()
            if not is_zoom:
                # Vista completa de mesa (X: 0..38 cm, Y: -20..+20 cm)
                scale = min((w - 40) / 40.0, (h - 40) / 40.0)
                cx_proj = w / 2.0
                cy_proj = h - 30.0
                def to_screen(rx, ry):
                    return cx_proj - ry * scale, cy_proj - rx * scale
            else:
                # Vista en zoom del tablero (X: 18..34 cm, Y: -9..+9 cm)
                scale = min((w - 40) / 20.0, (h - 40) / 18.0)
                cx_proj = w / 2.0
                cy_proj = h / 2.0
                def to_screen(rx, ry):
                    return cx_proj - ry * scale, cy_proj - (rx - 26.0) * scale

            # 1. Cuadrícula milimétrica / centimétrica de fondo
            grid_min_x = 0 if not is_zoom else 18
            grid_max_x = 36 if not is_zoom else 34
            grid_min_y = -18 if not is_zoom else -8
            grid_max_y = 18 if not is_zoom else 8

            for gx in range(grid_min_x, grid_max_x + 1, 2):
                p1_u, p1_v = to_screen(gx, grid_min_y)
                p2_u, p2_v = to_screen(gx, grid_max_y)
                canvas_tpl.create_line(p1_u, p1_v, p2_u, p2_v, fill="#f1f5f9" if gx % 10 != 0 else "#cbd5e1", width=1)
            for gy in range(grid_min_y, grid_max_y + 1, 2):
                p1_u, p1_v = to_screen(grid_min_x, gy)
                p2_u, p2_v = to_screen(grid_max_x, gy)
                canvas_tpl.create_line(p1_u, p1_v, p2_u, p2_v, fill="#f1f5f9" if gy % 10 != 0 else "#cbd5e1", width=1)

            # 2. Origen de la Base del Robot (0, 0) y Ejes
            if not is_zoom:
                b_u, b_v = to_screen(0, 0)
                canvas_tpl.create_oval(b_u - 14, b_v - 14, b_u + 14, b_v + 14, fill="#ffffff", outline="#1e40af", width=1.5)
                canvas_tpl.create_oval(b_u - 3, b_v - 3, b_u + 3, b_v + 3, fill="#0284c7")
                canvas_tpl.create_text(b_u, b_v + 10, text="BASE (0,0)", fill="#0284c7", font=("Segoe UI", 6, "bold"))
                # Eje de hombro a1=11cm
                sh_u, sh_v = to_screen(self.kin.a1, 0)
                canvas_tpl.create_line(b_u, b_v, sh_u, sh_v, fill="#f59e0b", width=2, dash=(2, 2))
                canvas_tpl.create_oval(sh_u - 3, sh_v - 3, sh_u + 3, sh_v + 3, fill="#f59e0b")

                # Dispensadores
                disp_o_u, disp_o_v = to_screen(*self.pick_coord[:2])
                canvas_tpl.create_oval(disp_o_u - 10, disp_o_v - 10, disp_o_u + 10, disp_o_v + 10, fill="#0c4a6e", outline="#38bdf8")
                canvas_tpl.create_text(disp_o_u, disp_o_v, text="DISP 'O'", fill="#ffffff", font=("Segoe UI", 6, "bold"))

            # 3. Contorno del Tablero 3x3
            tb_u1, tb_v1 = to_screen(19.25, -6.75)
            tb_u2, tb_v2 = to_screen(32.75, 6.75)
            canvas_tpl.create_rectangle(tb_u1, tb_v1, tb_u2, tb_v2, fill="#f8fafc", outline="#cbd5e1", width=2)

            # Líneas del gato (#)
            for lx in [23.75, 28.25]:
                lu1, lv1 = to_screen(lx, -6.75)
                lu2, lv2 = to_screen(lx, 6.75)
                canvas_tpl.create_line(lu1, lv1, lu2, lv2, fill="#94a3b8", width=1.5)
            for ly in [-2.25, 2.25]:
                lu1, lv1 = to_screen(19.25, ly)
                lu2, lv2 = to_screen(32.75, ly)
                canvas_tpl.create_line(lu1, lv1, lu2, lv2, fill="#94a3b8", width=1.5)

            # 4. Dibujar las 9 Casillas
            for idx, (cell_x, cell_y, _) in enumerate(self.cell_coords):
                cu, cv_pos = to_screen(cell_x, cell_y)
                is_selected = (idx == c_idx)

                # Anillo 15 mm
                r15_px = 1.5 * scale
                canvas_tpl.create_oval(cu - r15_px, cv_pos - r15_px, cu + r15_px, cv_pos + r15_px, fill="", outline="#1e293b", width=1)

                # Anillo 5 mm
                r5_px = 0.5 * scale
                canvas_tpl.create_oval(cu - r5_px, cv_pos - r5_px, cu + r5_px, cv_pos + r5_px, fill="", outline="#334155", width=1)

                # Cruz central
                cr_len = max(4.0, 0.4 * scale)
                canvas_tpl.create_line(cu - cr_len, cv_pos, cu + cr_len, cv_pos, fill="#64748b", width=1)
                canvas_tpl.create_line(cu, cv_pos - cr_len, cu, cv_pos + cr_len, fill="#64748b", width=1)

                # Resplandor cian si es la casilla objetivo seleccionada
                if is_selected:
                    canvas_tpl.create_oval(cu - r15_px - 3, cv_pos - r15_px - 3, cu + r15_px + 3, cv_pos + r15_px + 3, fill="", outline="#38bdf8", width=2)
                    canvas_tpl.create_text(cu, cv_pos - r15_px - 7, text="OBJETIVO", fill="#38bdf8", font=("Segoe UI", 6, "bold"))

                # Etiqueta de la casilla
                t_color = "#38bdf8" if is_selected else "#94a3b8"
                canvas_tpl.create_text(cu, cv_pos + r15_px + 6, text=f"C{idx+1}", fill=t_color, font=("Segoe UI", 7, "bold"))

            # 5. Representación en tiempo real del Brazo y Efector Final
            t_u, t_v = to_screen(x_real, y_real)

            if not is_zoom:
                canvas_tpl.create_line(b_u, b_v, t_u, t_v, fill="#0284c7", width=2, dash=(3, 2))

            tgt_u, tgt_v = to_screen(x_tgt, y_tgt)
            line_color = "#22c55e" if dr_mm <= 8.0 else ("#f59e0b" if dr_mm <= 25.0 else "#ef4444")
            canvas_tpl.create_line(t_u, t_v, tgt_u, tgt_v, fill=line_color, width=1.5, dash=(2, 2))

            # Cursor del Efector Final
            cursor_r = 7
            canvas_tpl.create_oval(t_u - cursor_r, t_v - cursor_r, t_u + cursor_r, t_v + cursor_r, fill="", outline=line_color, width=2)
            canvas_tpl.create_oval(t_u - 2, t_v - 2, t_u + 2, t_v + 2, fill=line_color)
            canvas_tpl.create_line(t_u - 12, t_v, t_u + 12, t_v, fill=line_color, width=1.5)
            canvas_tpl.create_line(t_u, t_v - 12, t_u, t_v + 12, fill=line_color, width=1.5)

            canvas_tpl.create_text(
                t_u + 14, t_v - 10,
                text=f"ROBOT REAL\n({x_real:.1f}, {y_real:.1f})",
                fill="#0f172a", font=("Consolas", 7, "bold"), anchor=tk.W
            )

            modal.after(40, update_template_canvas)

        update_template_canvas()


# ==============================================================================
# PUNTO DE ENTRADA PRINCIPAL (MAIN)
# ==============================================================================
if __name__ == "__main__":
    app = Robot3GDLGatoVisionController()
    app.mainloop()
