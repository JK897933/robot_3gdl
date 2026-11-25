"""
Módulo de Gestión y Detección de Cámaras Web (USB / Integrada).
Maneja la búsqueda automática de cámaras USB, compatibilidad con múltiples backends
en Windows (DirectShow y Media Foundation) y conmutación en caliente (tecla V).
"""

from typing import List, Optional, Tuple
import cv2
import numpy as np


def test_camera_index(index: int) -> Tuple[bool, Optional[str], Tuple[int, int]]:
    """
    Verifica si una cámara responde en el índice dado probando diferentes backends.
    Retorna (disponible, backend_exitoso, (ancho, alto)).
    """
    backends = [
        ("DirectShow", cv2.CAP_DSHOW),
        ("Default/MSMF", cv2.CAP_ANY)
    ]

    for name, backend in backends:
        try:
            cap = cv2.VideoCapture(index, backend)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    cap.release()
                    return True, name, (w, h)
                cap.release()
        except Exception:
            pass

    return False, None, (0, 0)


def list_available_cameras(max_indices: int = 5) -> List[dict]:
    """
    Escanea los puertos de cámara disponibles en la computadora.
    """
    found = []
    print("\n[Cámara] Buscando cámaras conectadas en el sistema...")
    for idx in range(max_indices):
        ok, backend, (w, h) = test_camera_index(idx)
        if ok:
            camera_type = "USB / Externa" if idx > 0 else "Cámara 0 (Principal/Integrada o USB)"
            print(f"  -> Cámara encontrada en Índice [{idx}]: {camera_type} ({w}x{h} vía {backend})")
            found.append({
                "index": idx,
                "backend": backend,
                "width": w,
                "height": h,
                "label": camera_type
            })

    if not found:
        print("[Cámara] No se encontraron cámaras activas en los índices probados.")
    return found


def open_usb_camera(target_index: Optional[int] = None) -> Tuple[Optional[cv2.VideoCapture], int]:
    """
    Abre la cámara USB deseada o detecta automáticamente la mejor opción.
    
    Lógica:
    - Si el usuario pasa un índice específico (ej. 1), intenta abrirlo.
    - Si target_index es None:
        1. Escanea las cámaras disponibles.
        2. Si hay 2 o más cámaras (ej. laptop con cámara interna + webcam USB conectada),
           selecciona preferentemente el índice 1 (la webcam USB).
        3. Si solo hay 1 cámara (ej. PC de escritorio con webcam USB en puerto 0), selecciona el índice 0.
        4. Si falla, intenta con los demás índices disponibles.
    
    Retorna:
        (cap, selected_index)
    """
    available = list_available_cameras()

    # Si se especificó un índice pero no está o queremos forzarlo
    if target_index is not None:
        chosen_idx = target_index
    else:
        if len(available) >= 2:
            # En laptops, el índice 0 suele ser la interna y el índice 1 es la cámara USB
            print("\n[Cámara USB] Se detectaron múltiples cámaras. Seleccionando la cámara USB (Índice 1)...")
            chosen_idx = 1
        elif len(available) == 1:
            chosen_idx = available[0]["index"]
            print(f"\n[Cámara USB] Seleccionando cámara disponible en Índice {chosen_idx}...")
        else:
            print("\n[Aviso] No se detectaron cámaras conectadas por USB. Intentando índice 0 por defecto...")
            chosen_idx = 0

    # Intentar abrir la cámara con DirectShow primero (óptimo para webcams USB en Windows)
    backends_to_try = [cv2.CAP_DSHOW, cv2.CAP_ANY]

    for backend in backends_to_try:
        try:
            cap = cv2.VideoCapture(chosen_idx, backend)
            if cap.isOpened():
                # Configurar resolución recomendada para procesamiento de imagen
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

                ret, frame = cap.read()
                if ret and frame is not None:
                    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    print(f"[Cámara USB] Cámara [{chosen_idx}] conectada con éxito. Resolución: {actual_w}x{actual_h}\n")
                    return cap, chosen_idx
                cap.release()
        except Exception as e:
            print(f"[Error Cámara] Excepción al intentar abrir índice {chosen_idx}: {e}")

    # Si falló la cámara preferida pero hay otras disponibles, intentar con la primera que funcione
    for cam in available:
        alt_idx = cam["index"]
        if alt_idx == chosen_idx:
            continue
        print(f"[Cámara] Intentando con cámara alternativa en Índice [{alt_idx}]...")
        for backend in backends_to_try:
            cap = cv2.VideoCapture(alt_idx, backend)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"[Cámara] Conectado exitosamente a Índice [{alt_idx}]\n")
                    return cap, alt_idx
                cap.release()

    return None, chosen_idx
