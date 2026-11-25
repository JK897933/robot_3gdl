"""
Módulo de Inteligencia Artificial y Reglas de Juego para Gato y Ratón (Tic-Tac-Toe / 3 en Raya)
Implementa algoritmo Minimax con poda Alfa-Beta y detección de victorias.
"""

import math
import random
from typing import List, Tuple, Optional

EMPTY = " "
RATON = "R"   # Ficha Ratón (Típicamente el Jugador humano)
GATO = "G"    # Ficha Gato (Típicamente la Inteligencia Artificial)

WINNING_COMBINATIONS = [
    # Filas
    [(0, 0), (0, 1), (0, 2)],
    [(1, 0), (1, 1), (1, 2)],
    [(2, 0), (2, 1), (2, 2)],
    # Columnas
    [(0, 0), (1, 0), (2, 0)],
    [(0, 1), (1, 1), (2, 1)],
    [(0, 2), (1, 2), (2, 2)],
    # Diagonales
    [(0, 0), (1, 1), (2, 2)],
    [(0, 2), (1, 1), (2, 0)],
]


def create_empty_board() -> List[List[str]]:
    """Crea un tablero 3x3 vacío."""
    return [[EMPTY for _ in range(3)] for _ in range(3)]


def check_winner(board: List[List[str]]) -> Tuple[Optional[str], Optional[List[Tuple[int, int]]]]:
    """
    Verifica si hay un ganador o empate en el tablero actual.
    
    Retorna:
        (ganador, linea_ganadora):
        - ganador: 'R', 'G', 'DRAW' o None (si la partida sigue en curso).
        - linea_ganadora: lista de 3 coordenadas [(r,c), ...] o None.
    """
    for combo in WINNING_COMBINATIONS:
        (r1, c1), (r2, c2), (r3, c3) = combo
        val1 = board[r1][c1]
        val2 = board[r2][c2]
        val3 = board[r3][c3]
        if val1 != EMPTY and val1 == val2 == val3:
            return val1, combo

    # Verificar si quedan casillas vacías
    for r in range(3):
        for c in range(3):
            if board[r][c] == EMPTY:
                return None, None  # Partida en curso

    return "DRAW", None


def get_available_moves(board: List[List[str]]) -> List[Tuple[int, int]]:
    """Retorna una lista de casillas disponibles (r, c)."""
    moves = []
    for r in range(3):
        for c in range(3):
            if board[r][c] == EMPTY:
                moves.append((r, c))
    return moves


def minimax(
    board: List[List[str]],
    depth: int,
    is_maximizing: bool,
    ai_piece: str,
    human_piece: str,
    alpha: float = -math.inf,
    beta: float = math.inf
) -> int:
    """Algoritmo Minimax con poda Alfa-Beta."""
    winner, _ = check_winner(board)
    if winner == ai_piece:
        return 10 - depth
    elif winner == human_piece:
        return depth - 10
    elif winner == "DRAW":
        return 0

    if is_maximizing:
        max_eval = -math.inf
        for r, c in get_available_moves(board):
            board[r][c] = ai_piece
            evaluation = minimax(board, depth + 1, False, ai_piece, human_piece, alpha, beta)
            board[r][c] = EMPTY
            max_eval = max(max_eval, evaluation)
            alpha = max(alpha, evaluation)
            if beta <= alpha:
                break
        return max_eval
    else:
        min_eval = math.inf
        for r, c in get_available_moves(board):
            board[r][c] = human_piece
            evaluation = minimax(board, depth + 1, True, ai_piece, human_piece, alpha, beta)
            board[r][c] = EMPTY
            min_eval = min(min_eval, evaluation)
            beta = min(beta, evaluation)
            if beta <= alpha:
                break
        return min_eval


def get_best_move(
    board: List[List[str]],
    ai_piece: str = GATO,
    human_piece: str = RATON,
    difficulty: str = "hard"
) -> Optional[Tuple[int, int]]:
    """
    Calcula la mejor posición (fila, columna) para la siguiente jugada de la IA.
    
    Dificultades:
    - 'easy': Elige aleatorio el 50% de las veces.
    - 'medium': Elige aleatorio el 20% de las veces.
    - 'hard': Jugada óptima mediante Minimax con Alfa-Beta.
    """
    available_moves = get_available_moves(board)
    if not available_moves:
        return None

    # Ajuste de dificultad con probabilidad de error intencional
    if difficulty == "easy" and random.random() < 0.5:
        return random.choice(available_moves)
    if difficulty == "medium" and random.random() < 0.2:
        return random.choice(available_moves)

    # Heurística inicial rápida: si el centro está libre en el primer movimiento, tomarlo
    if len(available_moves) >= 8 and board[1][1] == EMPTY and random.random() < 0.8:
        return (1, 1)

    best_val = -math.inf
    best_move = random.choice(available_moves)

    for r, c in available_moves:
        board[r][c] = ai_piece
        move_val = minimax(board, 0, False, ai_piece, human_piece, -math.inf, math.inf)
        board[r][c] = EMPTY

        if move_val > best_val:
            best_val = move_val
            best_move = (r, c)

    return best_move
