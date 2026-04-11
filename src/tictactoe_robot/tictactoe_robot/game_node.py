"""
game_node.py  ─  versión PyQt6 actualizada
──────────────────────────────────────────
Cambios respecto a la versión original:
  • Usa launch_app() de main_window en lugar de construir MainWindow directamente.
  • RosSignalBridge.move_completed ahora lleva solo cell_index (sin cambio en firma),
    pero MainWindow._on_move_completed() infiere el símbolo desde el estado del tablero.
  • La señal de turno humano se gestiona vía robot_status_changed → IDLE.
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

from tictactoe_robot.gui.main_window import launch_app

PLACE_PIECE_SERVICE = "/robot_controller/place_piece"


class RosSignalBridge(QObject):
    robot_status_changed = pyqtSignal(str)   # "IDLE" / "BUSY" / "MOVING: …"
    move_completed       = pyqtSignal(int)   # cell_index completado
    game_over            = pyqtSignal(str)   # "X" / "O" / "TIE"
    ai_thinking          = pyqtSignal(int)   # celda elegida por minimax


class GameNode(Node):

    def __init__(self, bridge: RosSignalBridge):
        super().__init__("game_node")
        self._bridge = bridge
        self._game   = TicTacToe()
        self._robot_idle   = True
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
            self.get_logger().warning(
                "robot_controller no disponible — modo simulación."
            )

    # ── API pública (llamada desde MainWindow) ─────────────────────────

    def start_game(self, human_symbol: str, difficulty: float):
        self._game              = TicTacToe()
        self._game.random_prob  = difficulty
        self._game.human_player = human_symbol
        self._game.ai_player    = "O" if human_symbol == "X" else "X"
        self._game_started      = True

        import random
        self._ai_turn = random.choice([True, False])

        if self._ai_turn:
            # Robot empieza: deshabilitar tablero hasta que termine
            self._bridge.robot_status_changed.emit("BUSY")
            threading.Thread(target=self._do_ai_turn, daemon=True).start()
        else:
            # Humano empieza: emitir IDLE para que BoardWidget habilite turno
            self._bridge.robot_status_changed.emit("IDLE")

    def human_move(self, cell_index: int):
        """BoardWidget llama a esto tras confirmar la jugada."""
        if not self._game_started or self._ai_turn:
            return
        if self._game.board[cell_index] != " ":
            return

        self._game.make_move(cell_index, self._game.human_player)
        # Notificar movimiento completado del humano
        self._bridge.move_completed.emit(cell_index)

        if self._check_game_over():
            return

        self._ai_turn = True
        self._bridge.robot_status_changed.emit("BUSY")
        threading.Thread(target=self._do_ai_turn, daemon=True).start()

    # ── lógica interna ─────────────────────────────────────────────────

    def _do_ai_turn(self):
        move = self._game.get_best_move()
        self._bridge.ai_thinking.emit(move)

        # Mover físicamente — bloquea hasta que el robot termina
        self._call_place_piece(self._game.ai_player, move)

        self._game.make_move(move, self._game.ai_player)

        # Emitir move_completed DESPUÉS de que el movimiento físico ha acabado
        self._bridge.move_completed.emit(move)

        if not self._check_game_over():
            self._ai_turn = False
            # IDLE hace que BoardWidget habilite el turno humano
            self._bridge.robot_status_changed.emit("IDLE")

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
            import time; time.sleep(1.5)
            return True

        req = PlacePiece.Request()
        req.symbol     = symbol
        req.cell_index = cell_index

        import time
        done     = threading.Event()
        response = None

        def cb(fut):
            nonlocal response
            response = fut.result()
            done.set()

        self._place_client.call_async(req).add_done_callback(cb)
        done.wait()

        if not response.success:
            return False

        # Esperar a que el robot informe IDLE
        self._robot_idle = False
        while not self._robot_idle:
            time.sleep(0.1)
        return True

    def _status_callback(self, msg: String):
        self._robot_idle = msg.data == "IDLE"
        self._bridge.robot_status_changed.emit(msg.data)


# ──────────────────────────────────────────────────────────────── main

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

    # 1. Muestra SetupDialog; si se acepta, abre MainWindow y arranca partida
    window = launch_app(bridge, node)
    if window is None:
        # Usuario cerró el diálogo sin iniciar
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    exit_code = app.exec()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()