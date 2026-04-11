"""
game_node.py  ─  versión PyQt6
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from tictactoe_interfaces.srv import PlacePiece
from tictactoe_robot.tictactoe_game import TicTacToe

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QObject, pyqtSignal

from tictactoe_robot.gui.main_window import MainWindow

PLACE_PIECE_SERVICE = "/robot_controller/place_piece"


class RosSignalBridge(QObject):
    """
    Objeto Qt puro que emite señales desde callbacks de ROS.
    Necesario para cruzar el puente entre hilos de forma segura.
    """
    robot_status_changed = pyqtSignal(str)   # "IDLE" / "BUSY" / "MOVING: home"
    move_completed       = pyqtSignal(int)   # cell_index que acaba de jugar el robot
    game_over            = pyqtSignal(str)   # "X" / "O" / "TIE"
    ai_thinking          = pyqtSignal(int)   # cell que ha elegido minimax


class GameNode(Node):

    def __init__(self, bridge: RosSignalBridge):
        super().__init__("game_node")
        self._bridge = bridge
        self._game   = TicTacToe()
        self._robot_idle = True
        self._game_started = False

        cbg = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            String,
            "/robot_controller/robot_status",
            self._status_callback,
            10,
        )

        self._place_client = self.create_client(
            PlacePiece, PLACE_PIECE_SERVICE, callback_group=cbg
        )

        ready = self._place_client.wait_for_service(timeout_sec=5.0)
        if not ready:
            self.get_logger().warning("robot_controller no disponible — modo simulación.")

    # ── llamado desde la GUI cuando el usuario configura el juego ──────

    def start_game(self, human_symbol: str, difficulty: float):
        """La GUI llama a esto cuando el usuario pulsa 'Start'."""
        self._game             = TicTacToe()
        self._game.random_prob  = difficulty
        self._game.human_player = human_symbol
        self._game.ai_player    = "O" if human_symbol == "X" else "X"
        self._game_started      = True

        import random
        self._ai_turn = random.choice([True, False])

        if self._ai_turn:
            threading.Thread(target=self._do_ai_turn, daemon=True).start()

    # ── llamado desde la GUI cuando el humano clica una celda ──────────

    def human_move(self, cell_index: int):
        """La GUI llama a esto cuando el humano clica una celda válida."""
        if not self._game_started or self._ai_turn:
            return
        if self._game.board[cell_index] != " ":
            return

        self._game.make_move(cell_index, self._game.human_player)
        self._bridge.move_completed.emit(cell_index)

        if self._check_game_over():
            return

        self._ai_turn = True
        threading.Thread(target=self._do_ai_turn, daemon=True).start()

    # ── lógica interna ──────────────────────────────────────────────────

    def _do_ai_turn(self):
        move = self._game.get_best_move()
        self._bridge.ai_thinking.emit(move)

        self._call_place_piece(self._game.ai_player, move)

        self._game.make_move(move, self._game.ai_player)
        self._bridge.move_completed.emit(move)

        if not self._check_game_over():
            self._ai_turn = False

    def _check_game_over(self) -> bool:
        if not self._game.game_over():
            return False
        winner = self._game.check_winner()
        result = winner if winner else "TIE"
        self._bridge.game_over.emit(result)
        self._game_started = False
        return True

    def _call_place_piece(self, symbol: str, cell_index: int) -> bool:
        if not self._place_client.service_is_ready():
            import time; time.sleep(1.5)   # simula tiempo de movimiento
            return True

        req = PlacePiece.Request()
        req.symbol     = symbol
        req.cell_index = cell_index

        import threading, time
        done = threading.Event()
        response = None

        def cb(fut):
            nonlocal response
            response = fut.result()
            done.set()

        self._place_client.call_async(req).add_done_callback(cb)
        done.wait()

        if not response.success:
            return False

        self._robot_idle = False
        while not self._robot_idle:
            time.sleep(0.1)
        return True

    def _status_callback(self, msg: String):
        self._robot_idle = msg.data == "IDLE"
        self._bridge.robot_status_changed.emit(msg.data)


# ───────────────────────────────────────────────────── main

def main(args=None):
    rclpy.init(args=args)

    app    = QApplication(sys.argv)
    bridge = RosSignalBridge()
    node   = GameNode(bridge)

    # ROS en hilo secundario
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    ros_thread.start()

    window = MainWindow(bridge, node)
    window.show()

    exit_code = app.exec()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()