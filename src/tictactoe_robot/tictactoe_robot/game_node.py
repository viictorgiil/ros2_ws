"""
game_node.py  ─  action-client version with emergency stop (v2)
───────────────────────────────────────────────────────────────────────
Corrected emergency-stop design:

  PREVIOUS PROBLEM
  ────────────────
  1. The dialog appeared late because emergency_confirmed was emitted only
     when ROS 2 processed cancel_callback, which required the server thread
     to finish first.
  2. On resume, resume_after_emergency() relaunched the full _do_ai_turn()
     and repeated a move that had already physically completed.

  SOLUTION
  ────────
  • The STOP button emits emergency_confirmed immediately on the Qt thread.
  • _emergency_event tells movement threads to stop as soon as possible.
  • _move_was_completed becomes True only after the physical movement ends.
  • resume_after_emergency() uses that state to decide whether to continue
    game logic or retry the interrupted physical action.
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
    emergency_confirmed  = pyqtSignal()   # emitted immediately when STOP is pressed
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

        # ── emergency state ────────────────────────────────────────────
        # _emergency_event: active -> movement threads must stop
        self._emergency_event = threading.Event()
        # _move_was_completed: _call_place_piece completed physically.
        self._move_was_completed = False
        # _move_logic_applied: make_move + move_completed.emit already done.
        self._move_logic_applied = False
        # _pending_symbol / _pending_cell: move that was in progress when
        # STOP was pressed so it can be resumed if needed.
        self._pending_symbol: str = ""
        self._pending_cell_idx: int = -1

        # Active goal handle used to cancel the robot action server goal
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
                "robot_controller not available — simulation mode."
            )

    # ── public API ─────────────────────────────────────────────────────

    def start_game(
        self,
        human_symbol: str,
        difficulty: float,
        teleop: bool = False,
        ai_starts: bool | None = None,
    ):
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
        self._ai_turn = random.choice([True, False]) if ai_starts is None else ai_starts

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
        Emergency stop.

        1. Set _emergency_event so movement threads exit on their next check.
        2. Cancel the active action-server goal to stop the UR3.
        3. Emit emergency_confirmed immediately so the GUI can react.
        """
        self.get_logger().warn("⛔ emergency_stop() activated.")
        self._game_started = False
        self._emergency_event.set()

        # Cancel the action-server goal asynchronously
        with self._goal_lock:
            gh = self._active_goal_handle
        if gh is not None:
            gh.cancel_goal_async()

        # Emit immediately; the dialog does not wait for ROS 2
        self._bridge.emergency_confirmed.emit()

        # Do not touch _move_was_completed or _move_logic_applied here.

    def resume_after_emergency(self):
        """
        Resume after emergency.

        If _move_was_completed is True, only continue game logic.
        If False, retry the interrupted physical move.
        """
        self._game_started = True
        self._emergency_event.clear()
        self.get_logger().info(
            f"▶ Resuming. move_was_completed={self._move_was_completed} "
            f"move_logic_applied={self._move_logic_applied}"
        )

        if self._move_logic_applied:
            # The original thread already applied game logic.
            threading.Thread(
                target=self._continue_after_logic_applied,
                daemon=True,
            ).start()
        elif self._move_was_completed:
            # Physical move completed but game logic did not.
            threading.Thread(
                target=self._continue_after_completed_move,
                daemon=True,
            ).start()
        else:
            # The physical move was interrupted.
            threading.Thread(
                target=self._retry_interrupted_move,
                daemon=True,
            ).start()

    def go_home_and_reset(self):
        threading.Thread(target=self._do_go_home, daemon=True).start()

    # ── resume logic ───────────────────────────────────────────────────

    def _continue_after_logic_applied(self):
        """
        The original thread already applied game logic.
        Only continue with the next turn.
        """
        if self._ai_turn:
            # It was the robot turn -> switch to the human
            self._ai_turn = False
            self._bridge.robot_status_changed.emit("IDLE")
            self._bridge.human_turn_started.emit()
        else:
            # It was a teleop human piece -> switch to the robot
            self._ai_turn = True
            self._do_ai_turn()

    def _continue_after_completed_move(self):
        """
        The pick-and-place finished just before the emergency.
        Register the move in game state and continue to the next turn.
        """
        symbol = self._pending_symbol
        cell   = self._pending_cell_idx

        if self._ai_turn:
            # It was the robot turn: apply logic and continue to the human
            self._game.make_move(cell, self._game.ai_player)
            self._bridge.move_completed.emit(cell)
            self._move_logic_applied = True
            if not self._check_game_over():
                self._ai_turn = False
                self._bridge.robot_status_changed.emit("IDLE")
                self._bridge.human_turn_started.emit()
        else:
            # It was a teleop human piece: apply logic and pass to the robot
            self._game.make_move(cell, self._game.human_player)
            self._bridge.move_completed.emit(cell)
            self._move_logic_applied = True
            if not self._check_game_over():
                self._ai_turn = True
                self._do_ai_turn()

    def _retry_interrupted_move(self):
        """
        The pick-and-place was interrupted. Retry the physical move.
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
            return   # new emergency

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

    # ── internal callbacks ─────────────────────────────────────────────

    def _feedback_cb(self, feedback_msg):
        phase = feedback_msg.feedback.phase
        self._bridge.robot_status_changed.emit(f"MOVING: {phase}")

    def _status_callback(self, msg: String):
        self._bridge.robot_status_changed.emit(msg.data)

    # ── movement sequences ─────────────────────────────────────────────

    def _do_go_home(self):
        self.get_logger().info("🏠 Sending robot home...")

        # Clear the emergency event so the HOME goal is not blocked.
        self._emergency_event.clear()

        if not self._place_client.server_is_ready():
            time.sleep(1.0)
            self._bridge.reset_completed.emit()
            return

        goal = PlacePiece.Goal()
        goal.symbol     = "HOME"
        goal.cell_index = -1

        # The server may still be finishing the previous cancellation.
        for attempt in range(20):  # up to ~2 seconds
            future = self._place_client.send_goal_async(goal)
            while not future.done():
                time.sleep(0.05)

            gh = future.result()
            if gh.accepted:
                break
            self.get_logger().warning(
                f"HOME goal rejected (attempt {attempt + 1}/20), retrying..."
            )
            time.sleep(0.1)
        else:
            self.get_logger().error("HOME goal rejected after all attempts.")
            self._bridge.reset_completed.emit()
            return

        result_future = gh.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        self.get_logger().info("🏠 Robot at home.")
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
            return   # emergency; resume_after_emergency will continue

        if self._emergency_event.is_set():
            self._move_was_completed = False
            return

        self._move_was_completed = True
        self._game.make_move(cell_index, self._game.human_player)
        self._bridge.move_completed.emit(cell_index)
        self._move_logic_applied = True

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
            return   # emergency

        if self._emergency_event.is_set():
            self._move_was_completed = False
            return

        self._move_was_completed = True

        # Apply game logic. If emergency arrives between _move_was_completed
        # and _move_logic_applied, resume logic will handle the gap.
        self._game.make_move(move, self._game.ai_player)
        self._bridge.move_completed.emit(move)
        self._move_logic_applied = True

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
        Send a goal and wait for the result.
        Check _emergency_event in each wait loop iteration.
        Return True on success, False on emergency or error.
        """
        if not self._place_client.server_is_ready():
            # Simulation: emergency-interruptible sleep
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
                # We do not have the goal handle yet. Cancel once ready.
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
            self.get_logger().error("Goal rejected.")
            return False

        # Emergency arrived exactly as send_future resolved
        if self._emergency_event.is_set():
            gh.cancel_goal_async()
            return False

        with self._goal_lock:
            self._active_goal_handle = gh

        result_future = gh.get_result_async()
        while not result_future.done():
            if self._emergency_event.is_set():
                # Also cancel on the server side
                gh.cancel_goal_async()
                with self._goal_lock:
                    self._active_goal_handle = None
                return False
            time.sleep(0.05)

        with self._goal_lock:
            self._active_goal_handle = None

        result = result_future.result()

        from action_msgs.msg import GoalStatus

        if self._emergency_event.is_set():
            self.get_logger().warn("Result ignored due to emergency.")
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

    # Keep the current window alive so it is not garbage-collected on reset.
    _current_window: list = [None]

    def _on_reset_completed():
        """
        Called on the Qt thread when the robot reaches home.
        Destroy the active window and relaunch SetupDialog.
        """
        old = _current_window[0]
        if old is not None:
            old._disconnect_bridge()
            old.close()
            old.deleteLater()
            _current_window[0] = None

        new_window = launch_app(bridge, node)
        _current_window[0] = new_window

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
