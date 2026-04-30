"""
tictactoe_game.py
─────────────────
Pure-logic TicTacToe class with Minimax + Alpha-Beta Pruning = GAME ENGINE
No I/O, no ROS2 : saves the real board
                  calculates the best play

Board layout (indices 0-8):
    0 | 1 | 2
    ---------
    3 | 4 | 5
    ---------
    6 | 7 | 8
"""

import random #for the random election depending on the difficulty


class TicTacToe: #class instantiated in game_node.py as self._game
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
        self.board: list[str] = [" "] * 9 #it's the actual logic board
        self.human_player: str | None = None #Initialize the human player's symbol as None
        self.ai_player:    str | None = None #the same for AI
        self.random_prob:  float      = 0.0   # default: Hard (perfect AI)

    # ------------------------------------------------------------------ display

    def board_string(self) -> str: #used primarily for debugging
        """Return a printable representation of the board."""
        b = self.board
        rows = []
        for i in range(0, 9, 3):
            rows.append(f" {b[i]} | {b[i+1]} | {b[i+2]} ")
            if i < 6:
                rows.append("---+---+---")
        return "\n".join(rows)

    # ------------------------------------------------------------------ queries

    def available_moves(self) -> list[int]: #returns a list of integers of the vacant positions
        """Return indices of empty squares."""
        return [i for i, s in enumerate(self.board) if s == " "]

    def is_board_full(self) -> bool: #check to see if there are any empty boxes left
        return " " not in self.board

    def check_winner(self) -> str | None:
        """Return winning symbol, or None if no winner yet."""
        b = self.board # short alias to the board
        lines = [
            (0, 1, 2), (3, 4, 5), (6, 7, 8),   # rows
            (0, 3, 6), (1, 4, 7), (2, 5, 8),   # columns
            (0, 4, 8), (2, 4, 6),               # diagonals
        ]
        for a, c, d in lines: #go through the winning combinations one by one
            if b[a] == b[c] == b[d] != " ":
                return b[a] #returns the winning symbol
        return None #if no winning combination is found, there is no winner

    def game_over(self) -> bool: #The game ends if someone has won or the board is full
        return self.check_winner() is not None or self.is_board_full()

    # ------------------------------------------------------------------ actions

    def make_move(self, position: int, player: str) -> bool: #method that modifies the board
        """Place *player*'s symbol at *position*.  Returns False if occupied."""
        if self.board[position] == " ": #checks if the box is empty
            self.board[position] = player #place the piece in that position
            return True
        return False

    def reset(self):
        """Clear the board for a new game."""
        self.board = [" "] * 9 #clear the board again

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
            #If you win when there are still many empty squares left, that means you won early
        if winner == self.human_player:
            return -(len(self.available_moves()) + 1)
            #It's better to lose later than to lose right away
        if self.is_board_full():
            return 0 #tie

        if is_maximizing:
            best = float("-inf")
            #Start by assuming the worst possible value for a maximizer: negative infinity
            #this is because later you want to increase that value
            for move in self.available_moves(): #tries every possible move
                self.board[move] = self.ai_player
                #simulates the AI making a move on that square, 
                #as a temporary simulation within the search tree
                score = self.minimax_ab(depth + 1, False, alpha, beta) #recursive call to simulate the next turn
                self.board[move] = " " #reverses the simulated move (backtracking)
                best  = max(best, score) #saves the highest score found so far
                alpha = max(alpha, best) #saves the highest score found so far
                if beta <= alpha:   # beta cut-off
                    break #The minimizer already has an alternative that's just as good or better elsewhere, 
                          #so this branch won't change the final decision = it cuts down on the search and saves time
            return best
        else:
            best = float("inf") #the worst case for a minimizer: positive infinite, because you want to lower the value
            for move in self.available_moves():
                self.board[move] = self.human_player
                score = self.minimax_ab(depth + 1, True, alpha, beta)
                self.board[move] = " "
                best = min(best, score)
                beta = min(beta, best) #represents the best value the minimizer can guarantee
                if beta <= alpha:   # alpha cut-off
                    break
            return best

    def get_best_move(self) -> int:
        """
        Return the index of the best move for the AI.
        If random_prob > 0 there is a chance a random move is returned instead
        (lower difficulty levels).
        """
        if random.random() < self.random_prob: #the method returns a random number between 0 and 1
            return random.choice(self.available_moves())

        best_score = float("-inf")
        best_move  = None
        for move in self.available_moves(): #test each of the AI's candidate moves
            self.board[move] = self.ai_player #temporarily insert the AI piece
            score = self.minimax_ab(0, False, float("-inf"), float("inf"))
            self.board[move] = " "
            if score > best_score:
                best_score = score
                best_move  = move
        return best_move
