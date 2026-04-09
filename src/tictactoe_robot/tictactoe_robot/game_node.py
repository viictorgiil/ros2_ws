"""
game_node.py
────────────
ROS2 node  'game_node'

Responsabilidades
─────────────────
1. Interfaz de usuario por terminal (UI del juego: dificultad, turno, tablero).
2. Gestionar el estado de la partida usando TicTacToe.
3. Cuando es turno del robot: llamar a Minimax → llamar al servicio
   robot_controller/place_piece → esperar respuesta → continuar.
4. Cuando es turno del humano: pedir por terminal la celda donde jugó
   (en el futuro esto vendrá de Visión por Computador) y registrarla.

Flujo por turno
───────────────
  Human turn:
    • Mostrar tablero
    • Pedir celda (0-8) por terminal
    • Validar que la celda esté libre
    • Registrar movimiento en el tablero lógico

  AI / Robot turn:
    • Mostrar tablero
    • Calcular mejor jugada con Minimax
    • Llamar a  ~/robot_controller/place_piece (symbol, cell)
    • Esperar OK del servicio
    • Registrar movimiento en el tablero lógico

Dependencias
────────────
  - tictactoe_game.py          (misma carpeta / mismo paquete)
  - tictactoe_interfaces/srv/PlacePiece
"""

import random
import threading

import rclpy
from rclpy.node          import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from tictactoe_interfaces.srv import PlacePiece
from tictactoe_robot.tictactoe_game import TicTacToe

import time
from std_msgs.msg import String

# ─────────────────────────────────────────────────────────────── constants

PLACE_PIECE_SERVICE = "/robot_controller/place_piece"

# ─────────────────────────────────────────────────────────────── helpers

SEPARATOR    = "=" * 44
THIN_SEP     = "-" * 44

def _header(text: str):
    print(f"\n{SEPARATOR}")
    print(f"  {text}")
    print(SEPARATOR)

def _section(text: str):
    print(f"\n{THIN_SEP}")
    print(f"  {text}")
    print(THIN_SEP)

def _board_with_indices(game: TicTacToe):
    """Print board alongside cell indices for reference."""
    b = game.board
    ref = ["0","1","2","3","4","5","6","7","8"]
    print("\n  Current board          Cell index reference")
    for row in range(3):
        i = row * 3
        brow = f"  {b[i]:^3}|{b[i+1]:^3}|{b[i+2]:^3}"
        rrow = f"   {ref[i]:^3}|{ref[i+1]:^3}|{ref[i+2]:^3}"
        print(brow + "       " + rrow)
        if row < 2:
            print("  ---+---+---            ---+---+---")
    print()


# ─────────────────────────────────────────────────────────────── node

class GameNode(Node):

    def __init__(self):
        super().__init__("game_node")

        cbg = MutuallyExclusiveCallbackGroup()
        # a mechanism used to ensure that only 
        # one callback within that group is executed at any given time


        self._robot_idle = True

        self.create_subscription(
            String,
            "/robot_controller/robot_status",
            self._status_callback,
            10
        )

        # Service client for moving the robot
        self._place_client = self.create_client(
            PlacePiece,
            PLACE_PIECE_SERVICE,
            callback_group=cbg,
        )

        self._game = TicTacToe()

        self.get_logger().info(
            "GameNode initialised.  Waiting for robot_controller service…"
        )
        ready = self._place_client.wait_for_service(timeout_sec=10.0)
        if not ready:
            self.get_logger().warning(
                "robot_controller service not available!  "
                "Robot moves will be SIMULATED (no hardware)."
            )
        else:
            self.get_logger().info("robot_controller service found. Ready to play!")

        # Start the game in a thread to avoid blocking the executor
        self._game_thread = threading.Thread(target=self._run_game, daemon=True)
        self._game_thread.start()

    # ─────────────────────────────────────────── terminal UI helpers
    
    @staticmethod
    def _ask_difficulty() -> float:
        """Ask difficulty and return the corresponding random_prob value."""
        _section("Select difficulty")
        print("  1  Easy    (25 % random moves)")
        print("  2  Medium  (50 % random moves)")
        print("  3  Hard    (perfect Minimax AI)")
        while True:

            #import sys
            #print(sys.stdin.isatty())
            choice = input("\n  Your choice (1/2/3): ").strip()
            if choice == "1":
                return 0.25
            if choice == "2":
                return 0.50
            if choice == "3":
                return 0.0
            print("  Invalid choice, please enter 1, 2 or 3.")

    @staticmethod
    def _ask_symbol() -> str:
        """Ask the human which symbol they want to use."""
        _section("Choose your symbol")
        while True:
            choice = input("  Do you want to be X or O? ").strip().upper()
            if choice in ("X", "O"):
                return choice
            print("  Invalid choice, please enter X or O.")




    def _ask_human_move(self) -> int:
        """Ask the human for a valid move and return the cell index."""
        while True:
            try:
                cell = int(input("  Your move (0-8): ").strip())
                if 0 <= cell <= 8 and self._game.board[cell] == " ":
                    return cell
                print("  Cell unavailable or out of range.  Try again.")
            except ValueError:
                print("  Please enter a number between 0 and 8.")

    # ─────────────────────────────────────────── robot interface

    def _status_callback(self, msg):
        if msg.data == "IDLE":
            self._robot_idle = True
        else:
            self._robot_idle = False

    def _call_place_piece(self, symbol: str, cell_index: int) -> bool:

        if not self._place_client.service_is_ready():
            print(f"\n  ⚙ [SIMULATION] '{symbol}' → cell {cell_index}")
            return True

        request = PlacePiece.Request()
        request.symbol = symbol
        request.cell_index = cell_index

        print(f"\n  ⚙ Sending move to robot: '{symbol}' → cell {cell_index} …")

        future = self._place_client.call_async(request)

        done_event = threading.Event()
        response = None

        def _callback(fut):
            nonlocal response
            response = fut.result()
            done_event.set()

        future.add_done_callback(_callback)

        # ✅ 1. esperar respuesta del servicio
        done_event.wait()

        if not response.success:
            print(f"  ✗ Robot error: {response.message}")
            return False

        # ✅ 2. esperar a que el robot termine REALMENTE
        print("  ⏳ Waiting for robot to finish movement...")

        self._robot_idle = False
        while not self._robot_idle:
            time.sleep(0.1)

        print("  ✅ Robot finished movement")

        return True

    # ─────────────────────────────────────────── main game loop

    def _run_game(self):
        """Full game loop — runs in a background thread."""

        _header("TIC-TAC-TOE  ×  UR3 CB3")
        print("  Human vs Robot  |  Minimax + Alpha-Beta Pruning")

        # ── setup ────────────────────────────────────────────────────────
        self._game.random_prob  = self._ask_difficulty()
        human_symbol            = self._ask_symbol()
        self._game.human_player = human_symbol
        self._game.ai_player    = "O" if human_symbol == "X" else "X"

        print(
            f"\n  You: '{self._game.human_player}'   "
            f"Robot AI: '{self._game.ai_player}'"
        )

        
        ai_turn = random.choice([True, False])

        _section("Game starts!")
        print("  Cell indices:")
        print("    0 | 1 | 2")
        print("    ---------")
        print("    3 | 4 | 5")
        print("    ---------")
        print("    6 | 7 | 8\n")

        # ── game loop ────────────────────────────────────────────────────
        while not self._game.game_over():

            _board_with_indices(self._game)

            if ai_turn:
                # ── Robot / AI turn ─────────────────────────────────────
                _section(f"Robot's turn  ('{self._game.ai_player}')")
                move = self._game.get_best_move()
                print(f"  Minimax chose cell {move}")

                success = self._call_place_piece(self._game.ai_player, move)
                if not success:
                    print("  ERROR: robot failed to place piece. Aborting game.")
                    return
                self._game.make_move(move, self._game.ai_player)

            else:
                # ── Human turn ──────────────────────────────────────────
                _section(f"Your turn  ('{self._game.human_player}')")
                print(
                    "  [NOTE] In the future your move will be detected automatically\n"
                    "         by the Computer Vision module.  For now, enter it manually."
                )
                move = self._ask_human_move()
                self._game.make_move(move, self._game.human_player)

            ai_turn = not ai_turn

        # ── game over ────────────────────────────────────────────────────
        _header("GAME OVER")
        _board_with_indices(self._game)

        winner = self._game.check_winner()
        if winner == self._game.ai_player:
            print("  🤖  The robot wins!  Better luck next time.\n")
        elif winner == self._game.human_player:
            print("  🎉  You win!  Congratulations!\n")
        else:
            print("  🤝  It's a tie!\n")

        print("  (Shut down with Ctrl-C or restart to play again.)\n")


# ─────────────────────────────────────────────────────────────── main

def main(args=None):
    rclpy.init(args=args)
    node = GameNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
