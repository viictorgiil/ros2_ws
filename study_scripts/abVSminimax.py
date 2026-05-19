# Most of this code is based on:
# https://www.datacamp.com/tutorial/minimax-algorithm-for-ai-in-python?dc_referrer=https%3A%2F%2Fwww.google.com%2F  

# Metrics collected per game:
#   - Nodes visited per AI move (and mean across the game)
#   - Computation time per AI move (and mean / total)
#   - Maximum tree depth reached per move
#   - Branches pruned (Alpha-Beta only)
#   - Summary table printed at game end

import random
import time

USE_ALPHA_BETA = True   # True = Alpha-Beta Pruning | False = Minimax

# =============================================================================
# METRICS CONTAINER
# =============================================================================
 
class MoveMetrics:
    """Performance data for a single AI move, to compare the evolution"""
    def __init__(self, move_number: int):
        self.move_number    = move_number
        self.nodes_visited  = 0          # total nodes evaluated
        self.branches_pruned = 0         # alpha/beta cuts (0 if plain minimax)
        self.max_depth      = 0          # deepest recursion reached
        self.elapsed_time   = 0.0        # seconds
 
 
class GameMetrics:
    """Aggregates MoveMetrics across a full game = mean metrics"""
    def __init__(self, algorithm: str):
        self.algorithm   = algorithm
        self.moves: list[MoveMetrics] = []
 
    def record(self, m: MoveMetrics):
        self.moves.append(m)
 
    # ── derived statistics ──────────────────────────────────────────────────
 
    def _vals(self, attr):
        return [getattr(m, attr) for m in self.moves]
 
    def mean(self, attr):
        vals = self._vals(attr)
        return sum(vals) / len(vals) if vals else 0.0
 
    def total(self, attr):
        return sum(self._vals(attr))
 
    def maximum(self, attr):
        vals = self._vals(attr)
        return max(vals) if vals else 0
 
    # ── pretty printer ──────────────────────────────────────────────────────
 
    def print_summary(self):
        sep = "=" * 58
        print(f"\n{sep}")
        print(f"  PERFORMANCE SUMMARY  –  {self.algorithm}")
        print(sep)
        print(f"  {'AI moves played':<30} {len(self.moves):>6}")
        print(f"  {'Total nodes visited':<30} {self.total('nodes_visited'):>6}")
        print(f"  {'Mean nodes / move':<30} {self.mean('nodes_visited'):>9.1f}")
        print(f"  {'Max nodes in single move':<30} {self.maximum('nodes_visited'):>6}")
        print(f"  {'Max tree depth reached':<30} {self.maximum('max_depth'):>6}")
        if self.algorithm == "Alpha-Beta Pruning":
            print(f"  {'Total branches pruned':<30} {self.total('branches_pruned'):>6}")
            print(f"  {'Mean pruned / move':<30} {self.mean('branches_pruned'):>9.1f}")
        print(f"  {'Total AI compute time (s)':<30} {self.total('elapsed_time'):>9.4f}")
        print(f"  {'Mean time / move (s)':<30} {self.mean('elapsed_time'):>9.4f}")
        print(f"  {'Max time single move (s)':<30} {self.maximum('elapsed_time'):>9.4f}")
        print(sep)
 
        # Per-move breakdown
        print(f"\n  {'Move':<6} {'Nodes':>7} {'Pruned':>7} {'Depth':>6} {'Time(s)':>9}")
        print(f"  {'-'*5} {'-'*7} {'-'*7} {'-'*6} {'-'*9}")
        for m in self.moves:
            print(f"  {m.move_number:<6} {m.nodes_visited:>7} "
                  f"{m.branches_pruned:>7} {m.max_depth:>6} "
                  f"{m.elapsed_time:>9.4f}")
        print(sep + "\n")

#---------------------------------------------------------------------------
# STEP 1: DEFINING A TIC TAC TOE CLASS

class TicTacToe:
    def __init__(self, use_alpha_beta: bool = True):
        # Initialize empty board (using ' ' for empty squares)
        self.board = [" " for _ in range(9)]
        # _ is used because we are not interested in the value
        # The result of the list comprehension is a list of 9 blanks (" "), which is assigned to self.board.
        self.human_player = None
        self.ai_player = None

        self.random_prob = 0.0  # by default it will be difficult

        self.use_alpha_beta = use_alpha_beta
 
        algo_name = "Alpha-Beta Pruning" if use_alpha_beta else "Minimax"
        self.metrics = GameMetrics(algo_name)
 
        # Temporary counters reset before every AI call
        self._nodes   = 0
        self._pruned  = 0
        self._depth   = 0

    def print_board(self):
        """Print the current state of the board"""
        #Docstring: A string of text that is placed right after the definition of 
        #a function, method, or class to describe what that function or class does.
        
        for i in range(0, 9, 3):
            print(f"{self.board[i]} | {self.board[i+1]} | {self.board[i+2]}")
            if i < 6:
                print("---+---+---")

    def available_moves(self):
        """Returns list of available moves (indices of empty squares)"""
        return [i for i, spot in enumerate(self.board) if spot == " "]
        
    # to test step 1:
    #   1. Open the route in the terminal where this script is saved
    #   2. Start an interactive Python session by running "python" or "python3"
    #   3. Import the class: "from tic_tac_toe import TicTacToe"
    #   4. Create a TicTacToe instance and try the methods:
    #       game = TicTacToe()
    #       game.print_board()
    #       game.available_moves()

    #---------------------------------------------------------------------------
    #STEP 2: IMPLEMENTING METHODS FOR MAKING A MOVE AND CHECKING BOARD STATES:

    def make_move(self, position, player):
        """Make a move on the board"""
        if self.board[position] == " ":
            self.board[position] = player #player represents just the symbol for now
            return True
        # If empty, places the player’s symbol and returns True
        return False


    def is_board_full(self):
        """Check if the board is full"""
        return " " not in self.board


    def check_winner(self):
        """Check if there's a winner. Returns winner symbol or None"""
        #Maybe for ROS2 we will use a list that will check the spaces:

        # Check rows:
        # It has to check 3 possibilities:
        # Top row (indices 0,1,2), Middle row (indices 3,4,5) and Bottom row (indices 6,7,8)

        for i in range(0, 9, 3):
            if self.board[i] == self.board[i + 1] == self.board[i + 2] != " ":
                return self.board[i]
        
        # Check columns
        # It has to check 3 possibilities:
        # Left column (indices 0,3,6), Middle column (indices 1,4,7) and Right column (indices 2,5,8)
        for i in range(3):
            if self.board[i] == self.board[i + 3] == self.board[i + 6] != " ":
                return self.board[i]
            
        # Check diagonals
        # It has to check 2 possibilities:
        # Top-left to bottom-right (indices 0,4,8) and Top-right to bottom-left (indices 2,4,6)
        if self.board[0] == self.board[4] == self.board[8] != " ":
            return self.board[0]
        if self.board[2] == self.board[4] == self.board[6] != " ":
            return self.board[2]

        return None # base case, there is no winner

    def game_over(self):
        """Check if the game is over"""
        return self.check_winner() is not None or self.is_board_full()
        # either the winner is found or the board is full.

    #---------------------------------------------------------------------------
    # STEP 3: IMPLEMENTING MINIMAX FOR TIC-TAC-TOE


    def minimax(self, depth, is_maximizing):
        """
        Minimax algorithm implementation
        Returns the best score possible for the current board state
        """

        # METRICS
        self._nodes += 1
        if depth > self._depth:
            self._depth = depth

        # depth - Tracks how many levels deep we are in the recursion. Useful for limit search
        # is_maximizing - A boolean flag indicating whether we're currently looking for 
        # the maximum score (AI's turn) or minimum score (human's turn). This alternates 
        # with each recursive call as players take turns.

        # The more empty spaces there are when a win is detected, the sooner it happened, 
        # so the reward is greater. The +1 prevents the score from being 0 if the board is almost full when someone wins:
        winner = self.check_winner()
        if winner == self.ai_player:
            return len(self.available_moves()) + 1
        if winner == self.human_player:
            return -(len(self.available_moves()) + 1)
        if self.is_board_full(): # this is a draw
            return 0
        
        # if it is the maximizing player's turn (AI), we want to maximize the score
        if is_maximizing:
            best_score = float("-inf") #start by setting best_score to negative infinity, so any real score will be better
            for move in self.available_moves():

                self.board[move] = self.ai_player #Place the AI’s symbol on the board temporarily
                score = self.minimax(depth + 1, False) # Recursively call minimax with the next depth and the minimizing player (=change of turn)
                self.board[move] = " " #Undo the temporary move to restore the board state
                # Update the best score
                best_score = max(score, best_score) #Keep track of the highest score seen so far
            
            return best_score
        
        # All of this lets us check every possible move the AI could make until someone wins or the game is tied

        else: # if it is the minimizing player's turn (human), we want to minimize the score
            best_score = float("inf")
            for move in self.available_moves():

                self.board[move] = self.human_player
                score = self.minimax(depth + 1, True) # Recursively call minimax with the next depth and the maximizing player
                self.board[move] = " "
                # Update the best score
                best_score = min(score, best_score)
            return best_score
       

    def minimax_ab(self, depth, is_maximizing, alpha, beta):
        """
        Minimax with Alpha-Beta Pruning.
 
        alpha : best score the maximizer (AI) can already guarantee
        beta  : best score the minimizer (human) can already guarantee
 
        Pruning logic:
          Maximizer: if a branch score >= beta  → the minimizer will never
                     choose this path → prune (beta cut-off).
          Minimizer: if a branch score <= alpha → the maximizer will never
                     choose this path → prune (alpha cut-off).
        """
        self._nodes += 1
        if depth > self._depth:
            self._depth = depth
 
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
                if beta <= alpha:          # beta cut-off
                    self._pruned += 1
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
                if beta <= alpha:          # alpha cut-off
                    self._pruned += 1
                    break
            return best
#---------------------------------------------------------------------------
# STEP 4: FINDING THE BEST MOVE FOR AI UISNG MINIMAX

#Now that we have the algorithm ready, we need another method that uses the minimax method to determine the best move for the AI after each human move:

    def get_best_move(self, move_number: int):
            """Evaluate all root moves and return the best one, recording metrics."""
            self._nodes  = 0
            self._pruned = 0
            self._depth  = 0
    
            best_score = float("-inf")
            best_move  = None
    
            t_start = time.perf_counter()
    
            for move in self.available_moves():
                self.board[move] = self.ai_player
    
                if self.use_alpha_beta:
                    score = self.minimax_ab(0, False, float("-inf"), float("inf"))
                else:
                    score = self.minimax(0, False)
    
                self.board[move] = " "
    
                if score > best_score:
                    best_score = score
                    best_move  = move
    
            elapsed = time.perf_counter() - t_start
    
            # Record metrics for this move
            m = MoveMetrics(move_number)
            m.nodes_visited  = self._nodes
            m.branches_pruned = self._pruned
            m.max_depth      = self._depth
            m.elapsed_time   = elapsed
            self.metrics.record(m)
    
            return best_move
   
#---------------------------------------------------------------------------
# STEP 5: IMPLEMENTING A GAME LOOP

    def play_game(self):
        """Main Game Loop"""
        algo = self.metrics.algorithm
        print(f"\n{'='*48}")
        print(f"  TIC TAC TOE  –  Algorithm: {algo}")
        print(f"{'='*48}\n")
 
        # Difficulty
        print("Choose difficulty:")
        print("  1 – Easy   (25% random moves)")
        print("  2 – Medium (50% random moves)")
        print("  3 – Hard   (optimal AI)")
        diff = input("Select difficulty (1/2/3): ").strip()
        if diff == "1":
            self.random_prob = 0.25
        elif diff == "2":
            self.random_prob = 0.50
        else:
            self.random_prob = 0.0
 
        # Symbol
        choice = input("\nDo you want to be X or O? ").upper().strip()
        while choice not in ("X", "O"):
            choice = input("Invalid. Choose X or O: ").upper().strip()
 
        self.human_player = choice
        self.ai_player    = "O" if choice == "X" else "X"
 
        print(f"\n  You → '{self.human_player}'   AI → '{self.ai_player}'")
        print("\nBoard positions:")
        print(" 0 | 1 | 2")
        print("---+---+---")
        print(" 3 | 4 | 5")
        print("---+---+---")
        print(" 6 | 7 | 8\n")

        ai_turn = False#random.choice([True, False])

        #if self.human_player == "X":
        #    ai_turn = False  # humaan starts
        #else:
        #    ai_turn = True   # AI starts

        move_count = 0

        while not self.game_over():
            
            self.print_board()

            if ai_turn:
                move_count += 1
                print("\nAI's turn...")

                if random.random() < self.random_prob:
                    move = random.choice(self.available_moves())
                else:
                    move = self.get_best_move(move_count)

                self.make_move(move, self.ai_player)
            else:
                  
                while True:
                    try:
                        move = int(input("\nYour turn (0-8): "))
                        if 0 <= move <= 8 and self.make_move(move, self.human_player):
                            break
                        else:
                            print("Invalid move! Try again.")
                    except ValueError:
                        print("Please enter a number between 0 and 8!")

            ai_turn = not ai_turn  # to alternate between players

        # Game over
        self.print_board()
        winner = self.check_winner()
        if winner == self.ai_player:
            print("\nAI wins!")
        elif winner == self.human_player:
            print("\nCongratulations! You win!")
        else:
            print("\nIt's a tie!")

        self.metrics.print_summary()

if __name__ == "__main__":
    game = TicTacToe(use_alpha_beta=USE_ALPHA_BETA)
    game.play_game()