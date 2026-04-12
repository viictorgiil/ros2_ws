"""
game_node.py  ─  versión con action client y parada de emergencia (v2)
───────────────────────────────────────────────────────────────────────
Diseño de emergencia corregido:

  PROBLEMA ANTERIOR
  ─────────────────
  1. El diálogo aparecía tarde porque emergency_confirmed solo se emitía
     cuando ROS2 procesaba el cancel_callback — lo cual requería que el
     hilo del servidor terminase primero (el sleep no era interrumpible
     desde el executor).
  2. Al reanudar, resume_after_emergency() relanzaba _do_ai_turn() completo
     → doble movimiento porque el pick-and-place ya había terminado.

  SOLUCIÓN
  ─────────
  • El botón STOP emite emergency_confirmed INMEDIATAMENTE en el hilo Qt
    (sin esperar a ROS2). El diálogo aparece al instante.
  • Se introduce _emergency_event (threading.Event) que los hilos de
    movimiento comprueban; cuando se activa, _call_place_piece devuelve
    False y el hilo sale.
  • Se introduce _move_was_completed (bool): se pone a True justo DESPUÉS
    de que _call_place_piece retorna True (movimiento físico terminado).
    emergency_stop() lo lee para saber si el robot ya acabó o no.
  • resume_after_emergency() usa _move_was_completed para decidir:
      - True  → el movimiento ya terminó; solo hay que continuar con
                 el resto de la secuencia (make_move + siguiente turno).
      - False → el movimiento fue interrumpido; relanzar _call_place_piece.
"""

import sys
import threading
import time

import rclpy
from rclpy.node            import Node
from rclpy.action          import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg          import String
from tictactoe_interfaces.action import PlacePiece
from tictactoe_robot.tictactoe_game import TicTacToe

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore    import QObject, pyqtSignal

from tictactoe_robot.gui.main_window import launch_app

PLACE_PIECE_ACTION = "/robot_controller/place_piece"


class RosSignalBridge(QObject):
    robot_status_changed = pyqtSignal(str)
    move_completed       = pyqtSignal(int)
    game_over            = pyqtSignal(str)
    ai_thinking          = pyqtSignal(int)
    human_turn_started   = pyqtSignal()
    robot_placing_human  = pyqtSignal()
    emergency_confirmed  = pyqtSignal()   # emitido INMEDIATAMENTE al pulsar STOP
    reset_completed      = pyqtSignal()


class GameNode(Node):

    def __init__(self, bridge: RosSignalBridge):
        super().__init__("game_node")
        self._bridge = bridge
        self._game   = TicTacToe()
        self._game_started  = False
        self._teleop        = False
        self._ai_turn       = False
        self._last_human_cell: int | None = None

        # ── Estado de emergencia ────────────────────────────────────────
        # _emergency_event: activo → los hilos de movimiento deben parar
        self._emergency_event = threading.Event()
        # _move_was_completed: el _call_place_piece terminó (físico OK).
        self._move_was_completed = False
        # _move_logic_applied: make_move + move_completed.emit ya ejecutados.
        # Necesario para que _continue_after_completed_move no los repita
        # si la emergencia llegó después de que el hilo original los completó.
        self._move_logic_applied = False
        # _pending_symbol / _pending_cell: qué movimiento estaba en curso
        # al pulsar STOP (para poder reanudarlo si no había terminado)
        self._pending_symbol: str = ""
        self._pending_cell_idx: int = -1

        # Goal handle activo (para cancelar en el action server del robot)
        self._active_goal_handle = None
        self._goal_lock          = threading.Lock()

        cbg = ReentrantCallbackGroup()

        self.create_subscription(
            String,
            "/robot_controller/robot_status",
            self._status_callback,
            10,
        )

        self._place_client = ActionClient(
            self, PlacePiece, PLACE_PIECE_ACTION, callback_group=cbg
        )

        ready = self._place_client.wait_for_server(timeout_sec=5.0)
        if not ready:
            self.get_logger().warning(
                "robot_controller no disponible — modo simulación."
            )

    # ── API pública ────────────────────────────────────────────────────

    def start_game(self, human_symbol: str, difficulty: float, teleop: bool = False):
        self._game              = TicTacToe()
        self._game.random_prob  = difficulty
        self._game.human_player = human_symbol
        self._game.ai_player    = "O" if human_symbol == "X" else "X"
        self._game_started      = True
        self._teleop            = teleop
        self._emergency_event.clear()
        self._move_was_completed = False
        self._move_logic_applied = False

        import random
        self._ai_turn = random.choice([True, False])

        if self._ai_turn:
            self._bridge.robot_status_changed.emit("BUSY")
            threading.Thread(target=self._do_ai_turn, daemon=True).start()
        else:
            self._bridge.robot_status_changed.emit("IDLE")
            self._bridge.human_turn_started.emit()

    def human_move(self, cell_index: int):
        if not self._game_started or self._ai_turn:
            return
        if self._game.board[cell_index] != " ":
            return

        if self._teleop:
            threading.Thread(
                target=self._do_human_teleop_move,
                args=(cell_index,),
                daemon=True,
            ).start()
        else:
            self._game.make_move(cell_index, self._game.human_player)
            self._bridge.move_completed.emit(cell_index)

            if self._check_game_over():
                return

            self._ai_turn = True
            self._bridge.robot_status_changed.emit("BUSY")
            threading.Thread(target=self._do_ai_turn, daemon=True).start()

    def emergency_stop(self):
        """
        Parada de emergencia.

        1. Activa _emergency_event → los hilos de movimiento salen en
           el próximo ciclo de comprobación.
        2. Cancela el goal activo en el action server (para parar el UR3).
        3. Emite emergency_confirmed INMEDIATAMENTE → la GUI muestra el
           diálogo sin esperar a que ROS2 confirme la cancelación.
        """
        self.get_logger().warn("⛔ emergency_stop() activado.")
        self._game_started = False
        self._emergency_event.set()

        # Cancelar goal en el servidor (asíncrono, no bloqueamos)
        with self._goal_lock:
            gh = self._active_goal_handle
        if gh is not None:
            gh.cancel_goal_async()

        # Emitir inmediatamente — el diálogo no espera a ROS2
        self._bridge.emergency_confirmed.emit()

        # NO tocar _move_was_completed ni _move_logic_applied aquí:
        # el hilo de movimiento los habrá fijado ya (o no) de forma atómica
        # antes de salir. Modificarlos aquí crearía una race condition.

    def resume_after_emergency(self):
        """
        Reanudar tras emergencia.

        Si _move_was_completed es True: el robot ya terminó el movimiento
        físico antes de que se procesase la emergencia. Solo hay que
        continuar con la lógica de juego (make_move + siguiente turno).

        Si _move_was_completed es False: el movimiento fue interrumpido.
        Hay que relanzar _call_place_piece con el mismo symbol/cell.
        """
        self._game_started = True
        self._emergency_event.clear()
        self.get_logger().info(
            f"▶ Reanudando. move_was_completed={self._move_was_completed} "
            f"move_logic_applied={self._move_logic_applied}"
        )

        if self._move_logic_applied:
            # El hilo original ya hizo make_move + move_completed.emit.
            # Solo hay que continuar con el siguiente turno.
            threading.Thread(
                target=self._continue_after_logic_applied,
                daemon=True,
            ).start()
        elif self._move_was_completed:
            # El movimiento físico terminó pero la lógica de juego no.
            # Aplicar make_move + siguiente turno sin repetir el place.
            threading.Thread(
                target=self._continue_after_completed_move,
                daemon=True,
            ).start()
        else:
            # El movimiento fue interrumpido — repetir _call_place_piece.
            threading.Thread(
                target=self._retry_interrupted_move,
                daemon=True,
            ).start()

    def go_home_and_reset(self):
        threading.Thread(target=self._do_go_home, daemon=True).start()

    # ── lógica de reanudación ──────────────────────────────────────────

    def _continue_after_logic_applied(self):
        """
        El hilo original ya aplicó make_move + emitió move_completed.
        Solo hay que continuar con el siguiente turno.
        """
        if self._ai_turn:
            # Era turno del robot → pasar al humano
            self._ai_turn = False
            self._bridge.robot_status_changed.emit("IDLE")
            self._bridge.human_turn_started.emit()
        else:
            # Era turno teleop (pieza del humano) → turno del robot
            self._ai_turn = True
            self._do_ai_turn()

    def _continue_after_completed_move(self):
        """
        El pick-and-place terminó justo antes de la emergencia.
        Registrar la jugada en el tablero lógico y pasar al siguiente turno.
        La GUI ya no necesita on_move_completed porque la celda ya fue pintada
        por el hilo original (si move_completed se emitió). Si no se emitió,
        hay que emitirlo aquí. Usamos _move_logic_applied para saberlo.
        """
        symbol = self._pending_symbol
        cell   = self._pending_cell_idx

        if self._ai_turn:
            # Era turno del robot: aplicar lógica y continuar al turno humano
            self._game.make_move(cell, self._game.ai_player)
            self._bridge.move_completed.emit(cell)
            self._move_logic_applied = True
            if not self._check_game_over():
                self._ai_turn = False
                self._bridge.robot_status_changed.emit("IDLE")
                self._bridge.human_turn_started.emit()
        else:
            # Era turno teleop (pieza del humano): aplicar lógica y pasar al robot
            self._game.make_move(cell, self._game.human_player)
            self._bridge.move_completed.emit(cell)
            self._move_logic_applied = True
            if not self._check_game_over():
                self._ai_turn = True
                self._do_ai_turn()

    def _retry_interrupted_move(self):
        """
        El pick-and-place fue interrumpido. Repetir el movimiento físico.
        """
        symbol = self._pending_symbol
        cell   = self._pending_cell_idx
        self._move_was_completed = False
        self._move_logic_applied = False
        self._bridge.robot_status_changed.emit("BUSY")

        if not self._ai_turn:
            self._bridge.robot_placing_human.emit()

        ok = self._call_place_piece(symbol, cell)
        if not ok:
            return   # nueva emergencia

        if self._emergency_event.is_set():
            self._move_was_completed = False
            return

        self._move_was_completed = True

        if self._ai_turn:
            self._game.make_move(cell, self._game.ai_player)
            self._bridge.move_completed.emit(cell)
            self._move_logic_applied = True
            if not self._check_game_over():
                self._ai_turn = False
                self._bridge.robot_status_changed.emit("IDLE")
                self._bridge.human_turn_started.emit()
        else:
            self._game.make_move(cell, self._game.human_player)
            self._bridge.move_completed.emit(cell)
            self._move_logic_applied = True
            if not self._check_game_over():
                self._ai_turn = True
                self._do_ai_turn()

    # ── callbacks internos ─────────────────────────────────────────────

    def _feedback_cb(self, feedback_msg):
        phase = feedback_msg.feedback.phase
        self._bridge.robot_status_changed.emit(f"MOVING: {phase}")

    def _status_callback(self, msg: String):
        self._bridge.robot_status_changed.emit(msg.data)

    # ── secuencias de movimiento ───────────────────────────────────────

    def _do_go_home(self):
        self.get_logger().info("🏠 Enviando robot a home…")

        # Limpiar evento de emergencia para que el goal HOME no sea bloqueado
        # por comprobaciones internas de _call_place_piece.
        self._emergency_event.clear()

        if not self._place_client.server_is_ready():
            time.sleep(1.0)
            self._bridge.reset_completed.emit()
            return

        goal = PlacePiece.Goal()
        goal.symbol     = "HOME"
        goal.cell_index = -1

        # El servidor puede estar terminando de procesar la cancelación anterior
        # (_active_goal_handle todavía no es None). Reintentar hasta que acepte.
        for attempt in range(20):  # hasta ~2 segundos
            future = self._place_client.send_goal_async(goal)
            while not future.done():
                time.sleep(0.05)

            gh = future.result()
            if gh.accepted:
                break
            self.get_logger().warning(
                f"Goal HOME rechazado (intento {attempt + 1}/20), reintentando…"
            )
            time.sleep(0.1)
        else:
            self.get_logger().error("Goal HOME rechazado tras todos los intentos.")
            self._bridge.reset_completed.emit()
            return

        result_future = gh.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        self.get_logger().info("🏠 Robot en home.")
        self._bridge.reset_completed.emit()

    def _do_human_teleop_move(self, cell_index: int):
        self._last_human_cell  = cell_index
        self._pending_symbol   = self._game.human_player
        self._pending_cell_idx = cell_index
        self._move_was_completed = False
        self._move_logic_applied = False

        self._bridge.robot_status_changed.emit("BUSY")
        self._bridge.robot_placing_human.emit()

        ok = self._call_place_piece(self._game.human_player, cell_index)
        if not ok:
            return   # emergencia — hilo sale; resume_after_emergency continuará

        if self._emergency_event.is_set():
            self._move_was_completed = False
            return

        self._move_was_completed = True
        self._game.make_move(cell_index, self._game.human_player)
        self._bridge.move_completed.emit(cell_index)
        self._move_logic_applied = True   # ← punto de corte atómico

        if self._check_game_over():
            return

        self._ai_turn = True
        self._do_ai_turn()

    def _do_ai_turn(self):
        move = self._game.get_best_move()
        self._bridge.ai_thinking.emit(move)

        self._pending_symbol   = self._game.ai_player
        self._pending_cell_idx = move
        self._move_was_completed = False
        self._move_logic_applied = False

        ok = self._call_place_piece(self._game.ai_player, move)
        if not ok:
            return   # emergencia

        if self._emergency_event.is_set():
            self._move_was_completed = False
            return

        self._move_was_completed = True

        # Aplicar lógica de juego. Si la emergencia llega entre _move_was_completed=True
        # y _move_logic_applied=True, _continue_after_completed_move la aplicará.
        # Si llega después de _move_logic_applied=True, _continue_after_logic_applied
        # solo continuará con el siguiente turno sin repetir nada.
        self._game.make_move(move, self._game.ai_player)
        self._bridge.move_completed.emit(move)
        self._move_logic_applied = True   # ← punto de corte atómico

        if not self._check_game_over():
            self._ai_turn = False
            self._bridge.robot_status_changed.emit("IDLE")
            self._bridge.human_turn_started.emit()

    def _check_game_over(self) -> bool:
        if not self._game.game_over():
            return False
        winner = self._game.check_winner()
        result = winner if winner else "TIE"
        self._bridge.game_over.emit(result)
        self._game_started = False
        return True

    def _call_place_piece(self, symbol: str, cell_index: int) -> bool:
        """
        Envía goal y espera resultado.
        Comprueba _emergency_event en cada iteración del bucle de espera.
        Retorna True si completado, False si emergencia o error.
        """
        if not self._place_client.server_is_ready():
            # Simulación: sleep interruptible por emergencia
            for _ in range(30):
                if self._emergency_event.is_set():
                    return False
                time.sleep(0.05)
            return not self._emergency_event.is_set()

        goal = PlacePiece.Goal()
        goal.symbol     = symbol
        goal.cell_index = cell_index

        send_future = self._place_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        while not send_future.done():
            if self._emergency_event.is_set():
                # No tenemos goal handle todavía. Esperar a tenerlo en background
                # para mandar el cancel al servidor y que realmente pare.
                def _cancel_when_ready(f):
                    try:
                        _gh = f.result()
                        if _gh and _gh.accepted:
                            _gh.cancel_goal_async()
                    except Exception:
                        pass
                send_future.add_done_callback(_cancel_when_ready)
                return False
            time.sleep(0.05)

        gh = send_future.result()
        if not gh.accepted:
            self.get_logger().error("Goal rechazado.")
            return False

        # Emergencia llegó exactamente mientras se resolvía send_future
        if self._emergency_event.is_set():
            gh.cancel_goal_async()
            return False

        with self._goal_lock:
            self._active_goal_handle = gh

        result_future = gh.get_result_async()
        while not result_future.done():
            if self._emergency_event.is_set():
                # Cancelar en el servidor también
                gh.cancel_goal_async()
                with self._goal_lock:
                    self._active_goal_handle = None
                return False
            time.sleep(0.05)

        with self._goal_lock:
            self._active_goal_handle = None

        result = result_future.result()

        from action_msgs.msg import GoalStatus

        # 🔥 CLAVE ABSOLUTA
        if self._emergency_event.is_set():
            self.get_logger().warn("Resultado ignorado por emergencia.")
            return False

        if result.status == GoalStatus.STATUS_CANCELED:
            return False

        return result.result.success


# ──────────────────────────────────────────────────────────────── main

def main(args=None):
    rclpy.init(args=args)

    app    = QApplication(sys.argv)
    bridge = RosSignalBridge()
    node   = GameNode(bridge)

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # _current_window mantiene la referencia viva — sin esto el GC destruye
    # la ventana nueva creada tras un reinicio antes de que Qt la procese.
    _current_window: list = [None]

    def _on_reset_completed():
        """
        Llamado desde el hilo Qt cuando el robot llega a home.
        Desconecta y destruye la ventana activa, luego relanza SetupDialog.
        Vivir en main() garantiza que la referencia a la nueva ventana
        se guarda correctamente y no es destruida por el GC.
        """
        old = _current_window[0]
        if old is not None:
            old._disconnect_bridge()   # desconectar señales antes de destruir
            old.close()
            old.deleteLater()
            _current_window[0] = None

        new_window = launch_app(bridge, node)
        _current_window[0] = new_window   # mantener referencia viva

    bridge.reset_completed.connect(_on_reset_completed)

    first_window = launch_app(bridge, node)
    if first_window is None:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    _current_window[0] = first_window

    exit_code = app.exec()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()