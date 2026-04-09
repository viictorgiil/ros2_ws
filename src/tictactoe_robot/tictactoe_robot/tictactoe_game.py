"""
tictactoe_game.py
─────────────────
Pure-logic TicTacToe class with Minimax + Alpha-Beta Pruning.
No I/O, no ROS2 — only game state and AI.  Everything else lives in game_node.py.

Board layout (indices 0-8):
    0 | 1 | 2
    ---------
    3 | 4 | 5
    ---------
    6 | 7 | 8
"""

import random


class TicTacToe:
    """
    Encapsulates board state and AI logic.

    Attributes
    ----------
    board        : list[str]   – 9-element list; ' ', 'X', or 'O'
    human_player : str | None  – symbol chosen by the human ('X' or 'O')
    ai_player    : str | None  – symbol used by the AI
    random_prob  : float       – probability [0, 1] that the AI plays randomly
                                 (difficulty knob: 0 = perfect, 1 = random)
    """

    def __init__(self):
        self.board: list[str] = [" "] * 9
        self.human_player: str | None = None
        self.ai_player:    str | None = None
        self.random_prob:  float      = 0.0   # default: Hard (perfect AI)

    # ------------------------------------------------------------------ display

    def board_string(self) -> str:
        """Return a printable representation of the board."""
        b = self.board
        rows = []
        for i in range(0, 9, 3):
            rows.append(f" {b[i]} | {b[i+1]} | {b[i+2]} ")
            if i < 6:
                rows.append("---+---+---")
        return "\n".join(rows)

    # ------------------------------------------------------------------ queries

    def available_moves(self) -> list[int]:
        """Return indices of empty squares."""
        return [i for i, s in enumerate(self.board) if s == " "]

    def is_board_full(self) -> bool:
        return " " not in self.board

    def check_winner(self) -> str | None:
        """Return winning symbol, or None if no winner yet."""
        b = self.board
        lines = [
            (0, 1, 2), (3, 4, 5), (6, 7, 8),   # rows
            (0, 3, 6), (1, 4, 7), (2, 5, 8),   # columns
            (0, 4, 8), (2, 4, 6),               # diagonals
        ]
        for a, c, d in lines:
            if b[a] == b[c] == b[d] != " ":
                return b[a]
        return None

    def game_over(self) -> bool:
        return self.check_winner() is not None or self.is_board_full()

    # ------------------------------------------------------------------ actions

    def make_move(self, position: int, player: str) -> bool:
        """Place *player*'s symbol at *position*.  Returns False if occupied."""
        if self.board[position] == " ":
            self.board[position] = player
            return True
        return False

    def reset(self):
        """Clear the board for a new game."""
        self.board = [" "] * 9

    # ------------------------------------------------------------------ AI

    def minimax_ab(
        self,
        depth: int,
        is_maximizing: bool,
        alpha: float,
        beta: float,
    ) -> float:
        """
        Minimax with Alpha-Beta Pruning.

        Score convention
        ────────────────
        +  (remaining_moves + 1) : AI wins   (higher = faster win)
        -  (remaining_moves + 1) : human wins
        0                        : draw
        """
        winner = self.check_winner()
        if winner == self.ai_player:
            return len(self.available_moves()) + 1
        if winner == self.human_player:
            return -(len(self.available_moves()) + 1)
        if self.is_board_full():
            return 0

        if is_maximizing:
            best = float("-inf")
            for move in self.available_moves():
                self.board[move] = self.ai_player
                score = self.minimax_ab(depth + 1, False, alpha, beta)
                self.board[move] = " "
                best  = max(best, score)
                alpha = max(alpha, best)
                if beta <= alpha:   # beta cut-off
                    break
            return best
        else:
            best = float("inf")
            for move in self.available_moves():
                self.board[move] = self.human_player
                score = self.minimax_ab(depth + 1, True, alpha, beta)
                self.board[move] = " "
                best = min(best, score)
                beta = min(beta, best)
                if beta <= alpha:   # alpha cut-off
                    break
            return best

    def get_best_move(self) -> int:
        """
        Return the index of the best move for the AI.
        If random_prob > 0 there is a chance a random move is returned instead
        (lower difficulty levels).
        """
        if random.random() < self.random_prob:
            return random.choice(self.available_moves())

        best_score = float("-inf")
        best_move  = None
        for move in self.available_moves():
            self.board[move] = self.ai_player
            score = self.minimax_ab(0, False, float("-inf"), float("inf"))
            self.board[move] = " "
            if score > best_score:
                best_score = score
                best_move  = move
        return best_move
