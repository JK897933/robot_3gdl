import os
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import letter, landscape
from reportlab.lib.units import mm, inch
from reportlab.lib import colors

def draw_plantilla(filename="Plantilla_Gato_Escala_1a1.pdf"):
    # Hoja Carta Horizontal: 279.4 mm x 215.9 mm (792 pt x 612 pt)
    w_pt, h_pt = landscape(letter)
    c = canvas.Canvas(filename, pagesize=(w_pt, h_pt))
    c.setTitle("Plantilla Física 1:1 - Tablero de Gato Robot 3 GDL")

    # =========================================================================
    # PÁGINA 1: TABLERO FÍSICO 1:1 (CASILLAS 1 A 9 + DISPENSADOR + REGLA)
    # =========================================================================

    # Fondo y Cuadrícula milimétrica tenue
    # Margen útil: 10 mm a cada lado
    x_min, x_max = 10 * mm, w_pt - 10 * mm
    y_min, y_max = 10 * mm, h_pt - 10 * mm

    c.setStrokeColor(colors.HexColor("#f1f5f9"))
    c.setLineWidth(0.3)
    # Líneas cada 5 mm
    cur_x = x_min
    while cur_x <= x_max:
        c.line(cur_x, y_min, cur_x, y_max)
        cur_x += 5 * mm

    cur_y = y_min
    while cur_y <= y_max:
        c.line(x_min, cur_y, x_max, cur_y)
        cur_y += 5 * mm

    # Líneas cada 10 mm (1 cm)
    c.setStrokeColor(colors.HexColor("#e2e8f0"))
    c.setLineWidth(0.6)
    cur_x = x_min
    while cur_x <= x_max:
        c.line(cur_x, y_min, cur_x, y_max)
        cur_x += 10 * mm

    cur_y = y_min
    while cur_y <= y_max:
        c.line(x_min, cur_y, x_max, cur_y)
        cur_y += 10 * mm

    # Marco exterior
    c.setStrokeColor(colors.HexColor("#94a3b8"))
    c.setLineWidth(1.0)
    c.rect(x_min, y_min, x_max - x_min, y_max - y_min)

    # Encabezado
    c.setFillColor(colors.HexColor("#0f172a"))
    c.setFont("Helvetica-Bold", 13)
    c.drawString(15 * mm, h_pt - 18 * mm, "UNIVERSIDAD POLITÉCNICA DE AGUASCALIENTES — ROBÓTICA Y MECATRÓNICA")
    c.setFont("Helvetica", 9)
    c.setFillColor(colors.HexColor("#475569"))
    c.drawString(15 * mm, h_pt - 22.5 * mm, "Plantilla Física Escala 1:1 | Tablero de Juego (Tic-Tac-Toe) para Calibración Cinemática del Robot 3 GDL")

    # REGLA DE VERIFICACIÓN FÍSICA DE 100 mm (10.0 cm)
    rx = w_pt - 115 * mm
    ry = h_pt - 24 * mm
    c.setFillColor(colors.HexColor("#f8fafc"))
    c.setStrokeColor(colors.HexColor("#0f172a"))
    c.setLineWidth(0.8)
    c.rect(rx, ry, 100 * mm, 6 * mm, fill=1, stroke=1)

    for i in range(11):
        x_tick = rx + i * 10 * mm
        c.line(x_tick, ry, x_tick, ry + 6 * mm)
        if i < 10:
            c.line(x_tick + 5 * mm, ry, x_tick + 5 * mm, ry + 3.5 * mm)
            c.setFont("Helvetica", 5.5)
            c.setFillColor(colors.HexColor("#0f172a"))
            c.drawCentredString(x_tick + 5 * mm, ry + 1.2 * mm, str(i))

    c.setFont("Helvetica-Bold", 7)
    c.setFillColor(colors.HexColor("#0f172a"))
    c.drawCentredString(rx + 50 * mm, ry + 7.5 * mm, "REGLA DE CALIBRACIÓN: 100 mm (10.0 cm)")
    c.setFont("Helvetica-Oblique", 6)
    c.setFillColor(colors.HexColor("#64748b"))
    c.drawCentredString(rx + 50 * mm, ry - 3.5 * mm, "Mida con una regla física real. Si mide 10 cm, la impresión está al 100% (1:1).")

    # =========================================================================
    # TABLERO 3X3 DEL GATO
    # Casillas de 42 mm x 42 mm. Separación entre centros: 45 mm (4.5 cm).
    # Tamaño total: 135 mm x 135 mm (13.5 x 13.5 cm).
    # =========================================================================
    board_w = 135 * mm
    board_h = 135 * mm
    bx = (w_pt - board_w) / 2.0 - 15 * mm   # Ligeramente a la izquierda para dejar espacio al dispensador
    by = (h_pt - board_h) / 2.0 - 8 * mm

    # Fondo blanco sólido para el tablero
    c.setFillColor(colors.white)
    c.setStrokeColor(colors.HexColor("#1e293b"))
    c.setLineWidth(1.8)
    c.roundRect(bx - 4 * mm, by - 4 * mm, board_w + 8 * mm, board_h + 8 * mm, 3 * mm, fill=1, stroke=1)

    # Líneas del tablero (#)
    cell_pitch = 45 * mm
    c.setStrokeColor(colors.HexColor("#0f172a"))
    c.setLineWidth(1.8)
    # Horizontales
    c.line(bx, by + cell_pitch, bx + board_w, by + cell_pitch)
    c.line(bx, by + 2 * cell_pitch, bx + board_w, by + 2 * cell_pitch)
    # Verticales
    c.line(bx + cell_pitch, by, bx + cell_pitch, by + board_h)
    c.line(bx + 2 * cell_pitch, by, bx + 2 * cell_pitch, by + board_h)

    # Título en el tablero
    c.setFont("Helvetica-Bold", 10)
    c.setFillColor(colors.HexColor("#0f2942"))
    c.drawCentredString(bx + board_w / 2.0, by + board_h + 6 * mm, "TABLERO 3x3 (TRES EN RAYA)")

    # Indicación de orientación hacia el robot
    c.setFont("Helvetica-Bold", 8)
    c.setFillColor(colors.HexColor("#0369a1"))
    c.drawCentredString(bx + board_w / 2.0, by - 8.5 * mm, "▲ ORIENTACIÓN: FRENTE DEL ROBOT (HACIA LA BASE) ▲")
    c.setFont("Helvetica", 7)
    c.setFillColor(colors.HexColor("#64748b"))
    c.drawCentredString(bx + board_w / 2.0, by - 12 * mm, "Distancia recomendada: Colocar este borde a 20.0 cm del centro de giro de la Base M1")

    # Dibujar las 9 Casillas con Dianas y Cuadros de Fichas
    # Casillas numeradas estándar de Gato:
    # Fila Sup (lejana del robot): Casillas 1, 2, 3 (col=0, 1, 2; row=2 en Y)
    # Fila Med: Casillas 4, 5, 6 (col=0, 1, 2; row=1 en Y)
    # Fila Inf (cercana al robot): Casillas 7, 8, 9 (col=0, 1, 2; row=0 en Y)
    cell_info = [
        (1, 0, 2, "CASILLA 1", "Arr. Izq"),
        (2, 1, 2, "CASILLA 2", "Arr. Cen"),
        (3, 2, 2, "CASILLA 3", "Arr. Der"),
        (4, 0, 1, "CASILLA 4", "Med. Izq"),
        (5, 1, 1, "CASILLA 5", "Centro"),
        (6, 2, 1, "CASILLA 6", "Med. Der"),
        (7, 0, 0, "CASILLA 7", "Abj. Izq"),
        (8, 1, 0, "CASILLA 8", "Abj. Cen"),
        (9, 2, 0, "CASILLA 9", "Abj. Der")
    ]

    for num, col, row, title, desc in cell_info:
        cx = bx + col * cell_pitch + cell_pitch / 2.0
        cy = by + row * cell_pitch + cell_pitch / 2.0

        # Círculo receptor de ficha (diámetro 32 mm)
        c.setStrokeColor(colors.HexColor("#cbd5e1"))
        c.setLineWidth(0.6)
        c.circle(cx, cy, 16 * mm)

        # Diana de alineación central
        c.setStrokeColor(colors.HexColor("#0284c7"))
        c.setLineWidth(0.6)
        c.circle(cx, cy, 6 * mm)
        c.circle(cx, cy, 2.5 * mm)
        c.setFillColor(colors.HexColor("#ef4444"))
        c.circle(cx, cy, 0.8 * mm, fill=1, stroke=0)

        # Cruz de centrado milimétrico
        c.setStrokeColor(colors.HexColor("#0284c7"))
        c.setLineWidth(0.5)
        c.line(cx - 9 * mm, cy, cx + 9 * mm, cy)
        c.line(cx, cy - 9 * mm, cx, cy + 9 * mm)

        # Número y etiqueta
        c.setFillColor(colors.HexColor("#0f172a"))
        c.setFont("Helvetica-Bold", 14)
        c.drawString(cx - 18 * mm, cy + 12 * mm, str(num))

        c.setFont("Helvetica-Bold", 7.5)
        c.setFillColor(colors.HexColor("#0284c7"))
        c.drawCentredString(cx, cy - 11.5 * mm, title)
        c.setFont("Helvetica", 6)
        c.setFillColor(colors.HexColor("#64748b"))
        c.drawCentredString(cx, cy - 14.5 * mm, f"({desc})")

        # Cuadro de notas de ángulos reales para que el usuario anote con lápiz
        c.setStrokeColor(colors.HexColor("#e2e8f0"))
        c.setFillColor(colors.HexColor("#f8fafc"))
        c.setLineWidth(0.4)
        c.rect(cx - 18 * mm, cy - 19.5 * mm, 36 * mm, 4 * mm, fill=1, stroke=1)
        c.setFont("Helvetica", 5.5)
        c.setFillColor(colors.HexColor("#94a3b8"))
        c.drawCentredString(cx, cy - 17.5 * mm, "q1:___°  q2:___°  q3:___°")

    # =========================================================================
    # ZONA DEL DISPENSADOR DE FICHAS (PICK POSITION)
    # A la derecha del tablero
    # =========================================================================
    dx = bx + board_w + 18 * mm
    dy = by + 25 * mm
    dw = 48 * mm
    dh = 75 * mm

    c.setFillColor(colors.HexColor("#f8fafc"))
    c.setStrokeColor(colors.HexColor("#1e40af"))
    c.setLineWidth(1.2)
    c.roundRect(dx, dy, dw, dh, 3 * mm, fill=1, stroke=1)

    c.setFont("Helvetica-Bold", 8.5)
    c.setFillColor(colors.HexColor("#1e40af"))
    c.drawCentredString(dx + dw / 2.0, dy + dh - 7 * mm, "DISPENSADOR DE FICHAS")
    c.setFont("Helvetica-Bold", 7.5)
    c.setFillColor(colors.HexColor("#0284c7"))
    c.drawCentredString(dx + dw / 2.0, dy + dh - 11 * mm, "(POSICIÓN PICK)")

    # Diana de toma de ficha (Pick)
    dcx = dx + dw / 2.0
    dcy = dy + 32 * mm
    c.setStrokeColor(colors.HexColor("#3b82f6"))
    c.setLineWidth(0.8)
    c.circle(dcx, dcy, 16 * mm)
    c.circle(dcx, dcy, 6 * mm)
    c.circle(dcx, dcy, 2 * mm)
    c.setFillColor(colors.HexColor("#16a34a"))
    c.circle(dcx, dcy, 0.8 * mm, fill=1, stroke=0)
    c.setStrokeColor(colors.HexColor("#3b82f6"))
    c.setLineWidth(0.5)
    c.line(dcx - 9 * mm, dcy, dcx + 9 * mm, dcy)
    c.line(dcx, dcy - 9 * mm, dcx, dcy + 9 * mm)

    c.setFont("Helvetica", 6.5)
    c.setFillColor(colors.HexColor("#475569"))
    c.drawCentredString(dcx, dcy - 11 * mm, "Colocar ficha aquí")
    c.drawCentredString(dcx, dcy - 14 * mm, "para recolección")

    # Caja de notas del dispensador
    c.setStrokeColor(colors.HexColor("#cbd5e1"))
    c.setFillColor(colors.white)
    c.setLineWidth(0.4)
    c.rect(dx + 4 * mm, dy + 4 * mm, dw - 8 * mm, 6 * mm, fill=1, stroke=1)
    c.setFont("Helvetica", 5.5)
    c.setFillColor(colors.HexColor("#64748b"))
    c.drawCentredString(dx + dw / 2.0, dy + 6.5 * mm, "Pick q1:___°  q2:___°  q3:___°")

    # Guía de uso rápido al pie
    c.setFont("Helvetica-Bold", 7)
    c.setFillColor(colors.HexColor("#0f172a"))
    c.drawString(15 * mm, 14 * mm, "GUÍA RÁPIDA DE CALIBRACIÓN POR APRENDIZAJE (TEACH-IN):")
    c.setFont("Helvetica", 6.5)
    c.setFillColor(colors.HexColor("#475569"))
    c.drawString(15 * mm, 11 * mm, "1. Imprime este PDF a escala 100% (sin 'Ajustar a página'). Mide la regla superior con una regla escolar: debe marcar exactamente 10 cm.")
    c.drawString(15 * mm, 8 * mm, "2. Fija la plantilla en la mesa frente al robot. En 'calibrar_casillas_ik.py', pulsa 'Motores Libres' y mueve el efector al centro de cada casilla.")
    c.drawString(15 * mm, 5 * mm, "3. Pulsa 'Guardar Casilla N'. El software calculará y ajustará la cinemática inversa exacta adaptada a tu estructura física real.")

    c.showPage()
    c.save()
    print("Plantilla PDF 1:1 generada exitosamente:", filename)

if __name__ == "__main__":
    draw_plantilla()
