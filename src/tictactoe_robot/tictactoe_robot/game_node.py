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

import json
import sys
import threading
import time

import rclpy
from rclpy.node            import Node
from rclpy.action          import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg          import String
from sensor_msgs.msg       import Image
from tictactoe_interfaces.action import MovePiece, PlacePiece
from tictactoe_robot.tictactoe_game import TicTacToe

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore    import QObject, pyqtSignal
from PyQt6.QtGui     import QImage

from tictactoe_robot.gui.main_window import launch_app

PLACE_PIECE_ACTION = "/robot_controller/place_piece"
MOVE_PIECE_ACTION = "/robot_controller/move_piece"
ORIGINAL_VIEW_TOPIC = "/tictactoe/vision/original_view"
RECTIFIED_VIEW_TOPIC = "/tictactoe/vision/rectified_view"
VISION_STATE_TOPIC = "/tictactoe/tablero"
GAME_TURN_TOPIC = "/tictactoe/game/turn"


class RosSignalBridge(QObject):
    robot_status_changed = pyqtSignal(str)
    move_completed       = pyqtSignal(int)
    game_over            = pyqtSignal(str)
    ai_thinking          = pyqtSignal(int)
    human_turn_started   = pyqtSignal()
    robot_placing_human  = pyqtSignal()
    emergency_confirmed  = pyqtSignal()   # emitted immediately when STOP is pressed
    reset_completed      = pyqtSignal()
    original_frame_ready  = pyqtSignal(object)
    rectified_frame_ready = pyqtSignal(object)
    vision_warning_changed = pyqtSignal(str)
    vision_provisional_move = pyqtSignal(int)


class GameNode(Node):

    def __init__(self, bridge: RosSignalBridge):
        super().__init__("game_node")
        self._bridge = bridge
        self._game   = TicTacToe()
        self._game_started  = False
        self._teleop        = False
        self._ai_turn       = False
        self._last_human_cell: int | None = None
        self.declare_parameter("require_vision", True)
        self._require_vision: bool = bool(
            self.get_parameter("require_vision").value
        )

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
        self._interrupted_by_vision = False
        self._vision_paused_event = threading.Event()
        self._paused_for_vision = False
        self._paused_game_started = False

        # Active goal handle used to cancel the robot action server goal
        self._active_goal_handle = None
        self._goal_lock          = threading.Lock()
        self._vision_lock = threading.Lock()
        self._vision_state: dict | None = None
        self._vision_received_at = 0.0
        self._vision_baseline_state: dict | None = None
        self._vision_candidate_cell: int | None = None
        self._vision_candidate_since = 0.0
        self._vision_pending_cell: int | None = None
        self._last_vision_warning = ""
        self._startup_sort_declined = False
        self._robot_motion_active = False

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
        self._move_client = ActionClient(
            self, MovePiece, MOVE_PIECE_ACTION, callback_group=cbg
        )

        self.create_subscription(
            Image,
            ORIGINAL_VIEW_TOPIC,
            self._original_frame_callback,
            10,
        )
        self.create_subscription(
            Image,
            RECTIFIED_VIEW_TOPIC,
            self._rectified_frame_callback,
            10,
        )
        self.create_subscription(
            String,
            VISION_STATE_TOPIC,
            self._vision_state_callback,
            10,
        )
        self._turn_pub = self.create_publisher(String, GAME_TURN_TOPIC, 10)
        self.create_timer(0.25, self._vision_tick)

        ready = self._place_client.wait_for_server(timeout_sec=5.0)
        if not ready:
            self.get_logger().warning(
                "robot_controller not available — simulation mode."
            )
        move_ready = self._move_client.wait_for_server(timeout_sec=1.0)
        if not move_ready:
            self.get_logger().warning(
                "robot_controller move_piece not available — dynamic sorting "
                "will be simulated."
            )
        self._publish_turn_state()

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
        self._vision_paused_event.clear()
        self._interrupted_by_vision = False
        self._move_was_completed = False
        self._move_logic_applied = False

        import random
        self._ai_turn = random.choice([True, False]) if ai_starts is None else ai_starts
        self._publish_turn_state()

        if self._ai_turn:
            self._bridge.robot_status_changed.emit("BUSY")
            threading.Thread(target=self._do_ai_turn, daemon=True).start()
        else:
            self._begin_human_turn(wait_for_fresh_vision=False)

    def human_move(self, cell_index: int):
        if not self._game_started or self._ai_turn:
            return
        if self._game.board[cell_index] != " ":
            return
        if self._require_vision and not self._teleop:
            if self._vision_pending_cell != cell_index:
                self._set_vision_warning(
                    "Confirm the move detected by vision before continuing."
                )
                return
            with self._vision_lock:
                state = dict(self._vision_state or {})
            board = state.get("board") or []
            if len(board) != 9 or board[cell_index] != self._game.human_player:
                self._reject_human_vision_move(
                    "Vision no longer sees your piece in the selected cell."
                )
                return
            if not self._current_human_vision_move_is_valid(state, cell_index):
                self._reject_human_vision_move(
                    "The board/storage state changed after vision selected "
                    "your move. Restore the extra movement before confirming."
                )
                return
            self._vision_pending_cell = None
            self._vision_candidate_cell = None
            self._set_vision_warning("")

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
            self._publish_turn_state()
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

    def collect_board_and_reset(self):
        threading.Thread(target=self._do_collect_board_and_home, daemon=True).start()

    # ── resume logic ───────────────────────────────────────────────────

    def _continue_after_logic_applied(self):
        """
        The original thread already applied game logic.
        Only continue with the next turn.
        """
        if self._ai_turn:
            # It was the robot turn -> switch to the human
            self._begin_human_turn(wait_for_fresh_vision=True)
        else:
            # It was a teleop human piece -> switch to the robot
            self._ai_turn = True
            self._publish_turn_state()
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
                self._begin_human_turn(wait_for_fresh_vision=True)
        else:
            # It was a teleop human piece: apply logic and pass to the robot
            self._game.make_move(cell, self._game.human_player)
            self._bridge.move_completed.emit(cell)
            self._move_logic_applied = True
            if not self._check_game_over():
                self._ai_turn = True
                self._publish_turn_state()
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

        self._interrupted_by_vision = False
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
                self._begin_human_turn(wait_for_fresh_vision=True)
        else:
            self._game.make_move(cell, self._game.human_player)
            self._bridge.move_completed.emit(cell)
            self._move_logic_applied = True
            if not self._check_game_over():
                self._ai_turn = True
                self._publish_turn_state()
                self._do_ai_turn()

    # ── internal callbacks ─────────────────────────────────────────────

    def _feedback_cb(self, feedback_msg):
        phase = feedback_msg.feedback.phase
        self._bridge.robot_status_changed.emit(f"MOVING: {phase}")

    def _status_callback(self, msg: String):
        self._bridge.robot_status_changed.emit(msg.data)

    def _image_to_qimage(self, msg: Image) -> QImage | None:
        if msg.width == 0 or msg.height == 0 or not msg.data:
            return None

        if msg.encoding not in ("bgr8", "rgb8"):
            self.get_logger().warning(
                f"Unsupported image encoding '{msg.encoding}', skipping frame."
            )
            return None

        image = QImage(
            bytes(msg.data),
            msg.width,
            msg.height,
            msg.step,
            QImage.Format.Format_BGR888 if msg.encoding == "bgr8" else QImage.Format.Format_RGB888,
        )
        return image.copy()

    def _original_frame_callback(self, msg: Image):
        image = self._image_to_qimage(msg)
        if image is not None:
            self._bridge.original_frame_ready.emit(image)

    def _rectified_frame_callback(self, msg: Image):
        image = self._image_to_qimage(msg)
        if image is not None:
            self._bridge.rectified_frame_ready.emit(image)

    def _vision_state_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Ignoring malformed vision state JSON.")
            return

        state = {
            "board_detected": bool(data.get("board_detected", "board" in data)),
            "board_detection_paused": bool(
                data.get("board_detection_paused", False)
            ),
            "hand_detected": bool(data.get("hand_detected", False)),
            "board": self._normalise_cells(data.get("board", []), 9),
            "storage1": self._normalise_cells(data.get("storage1", []), 5),
            "storage2": self._normalise_cells(data.get("storage2", []), 5),
            "timestamp": float(data.get("timestamp", time.time())),
            "received_at": time.monotonic(),
            "warning": str(data.get("warning", "")),
        }

        with self._vision_lock:
            self._vision_state = state
            self._vision_received_at = state["received_at"]

        if not self._require_vision:
            return

        if state["hand_detected"] and (
            self._robot_motion_active or (self._ai_turn and self._game_started)
        ):
            self._set_vision_warning("Hand detected during robot turn.")
            if not self._emergency_event.is_set():
                self.emergency_stop()
            return

        if state["board_detection_paused"]:
            return

        if not state["board_detected"]:
            self._set_vision_warning("Board not detected. Game paused.")
            self._pause_for_vision_loss()
            return

        if self._paused_for_vision:
            self._resume_after_vision_recovered()

    @staticmethod
    def _normalise_cells(values, expected_len: int) -> list[str]:
        cells: list[str] = []
        for value in list(values)[:expected_len]:
            label = str(value).upper()
            cells.append(label if label in ("X", "O") else " ")
        while len(cells) < expected_len:
            cells.append(" ")
        return cells

    def _set_vision_warning(self, text: str):
        if text == self._last_vision_warning:
            return
        self._last_vision_warning = text
        self._bridge.vision_warning_changed.emit(text)

    def _reject_human_vision_move(self, message: str):
        self._vision_pending_cell = None
        self._vision_candidate_cell = None
        self._vision_candidate_since = 0.0
        self._bridge.vision_provisional_move.emit(-1)
        self._set_vision_warning(message)

    def _current_human_vision_move_is_valid(self, state: dict, cell: int) -> bool:
        baseline = self._vision_baseline_state
        if baseline is None:
            return False

        board = list(state.get("board") or [])
        storage1 = list(state.get("storage1") or [])
        storage2 = list(state.get("storage2") or [])
        base_board = list(baseline.get("board") or self._game.board)
        base_storage1 = list(baseline.get("storage1") or [])
        base_storage2 = list(baseline.get("storage2") or [])

        if len(board) != 9 or len(storage1) != 5 or len(storage2) != 5:
            return False

        new_cells = [
            i for i, (before, after) in enumerate(zip(base_board, board))
            if before == " " and after in ("X", "O")
        ]
        changed_existing = [
            i for i, (before, after) in enumerate(zip(base_board, board))
            if before != " " and after in ("X", "O") and before != after
        ]
        storage_changes = [
            ("storage1", i)
            for i, (before, after) in enumerate(zip(base_storage1, storage1))
            if before != after
        ] + [
            ("storage2", i)
            for i, (before, after) in enumerate(zip(base_storage2, storage2))
            if before != after
        ]
        removed_from_storage = [
            (zone, i)
            for zone, i in storage_changes
            if (
                zone == "storage1"
                and base_storage1[i] == self._game.human_player
                and storage1[i] == " "
            )
            or (
                zone == "storage2"
                and base_storage2[i] == self._game.human_player
                and storage2[i] == " "
            )
        ]

        return (
            new_cells == [cell]
            and not changed_existing
            and board[cell] == self._game.human_player
            and len(removed_from_storage) == len(storage_changes)
            and len(storage_changes) <= 1
        )

    def _capture_human_turn_baseline(self):
        with self._vision_lock:
            state = dict(self._vision_state or {})
        self._vision_baseline_state = {
            # The internal board is authoritative for turn validation. Vision can
            # lag by a frame after the robot places a piece.
            "board": list(self._game.board),
            "storage1": list(state.get("storage1") or []),
            "storage2": list(state.get("storage2") or []),
        }
        self._vision_candidate_cell = None
        self._vision_candidate_since = 0.0
        self._vision_pending_cell = None
        self._bridge.vision_provisional_move.emit(-1)

    def _begin_human_turn(self, wait_for_fresh_vision: bool = False):
        self._ai_turn = False
        after = time.monotonic()
        self._publish_turn_state()

        if wait_for_fresh_vision:
            self._wait_for_fresh_board_vision(after)

        self._bridge.robot_status_changed.emit("IDLE")
        self._capture_human_turn_baseline()
        self._bridge.human_turn_started.emit()

    def _wait_for_fresh_board_vision(self, after: float, timeout: float = 3.0) -> bool:
        if not self._require_vision:
            return True

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            with self._vision_lock:
                state = dict(self._vision_state or {})

            if (
                state
                and state.get("received_at", 0.0) >= after
                and state.get("board_detected", False)
                and not state.get("board_detection_paused", False)
            ):
                return True

            time.sleep(0.05)

        self.get_logger().warning(
            "Timed out waiting for fresh vision baseline after robot move."
        )
        return False

    def _vision_tick(self):
        if (
            not self._require_vision
            or not self._game_started
            or self._ai_turn
            or self._teleop
            or self._paused_for_vision
        ):
            return

        with self._vision_lock:
            state = dict(self._vision_state or {})
        if not state or not state.get("board_detected", False):
            return

        baseline = self._vision_baseline_state
        if baseline is None:
            self._capture_human_turn_baseline()
            return

        board = list(state.get("board") or [])
        if len(board) != 9:
            return
        storage1 = list(state.get("storage1") or [])
        storage2 = list(state.get("storage2") or [])
        if len(storage1) != 5 or len(storage2) != 5:
            return

        base_board = list(baseline.get("board") or self._game.board)
        base_storage1 = list(baseline.get("storage1") or [])
        base_storage2 = list(baseline.get("storage2") or [])
        new_cells = [
            i for i, (before, after) in enumerate(zip(base_board, board))
            if before == " " and after in ("X", "O")
        ]
        changed_existing = [
            i for i, (before, after) in enumerate(zip(base_board, board))
            if before != " " and after in ("X", "O") and before != after
        ]
        storage_changes = [
            ("storage1", i)
            for i, (before, after) in enumerate(zip(base_storage1, storage1))
            if before != after
        ] + [
            ("storage2", i)
            for i, (before, after) in enumerate(zip(base_storage2, storage2))
            if before != after
        ]
        removed_from_storage = [
            (zone, i)
            for zone, i in storage_changes
            if (
                zone == "storage1"
                and base_storage1[i] == self._game.human_player
                and storage1[i] == " "
            )
            or (
                zone == "storage2"
                and base_storage2[i] == self._game.human_player
                and storage2[i] == " "
            )
        ]
        unexpected_storage_changes = [
            change for change in storage_changes if change not in removed_from_storage
        ]

        if not new_cells and not changed_existing and not storage_changes:
            if self._vision_pending_cell is not None:
                self._vision_pending_cell = None
                self._bridge.vision_provisional_move.emit(-1)
            self._set_vision_warning("")
            return

        if storage_changes and not new_cells:
            self._reject_human_vision_move(
                "Piece movement detected in a storage zone. Place exactly one "
                "piece on the board."
            )
            return

        if changed_existing:
            self._reject_human_vision_move(
                "Vision sees a change on an occupied board cell. Restore the board."
            )
            return

        if len(new_cells) > 1:
            self._reject_human_vision_move(
                "More than one new board cell is occupied. Leave exactly one "
                "piece on the board."
            )
            return

        if len(new_cells) == 0:
            self._reject_human_vision_move(
                "No new board piece detected. Place one piece on an empty board cell."
            )
            return

        cell = new_cells[0]
        if board[cell] != self._game.human_player:
            self._reject_human_vision_move(
                f"Wrong piece detected in cell {cell}. You must place "
                f"{self._game.human_player}, not {board[cell]}."
            )
            return

        if unexpected_storage_changes:
            self._reject_human_vision_move(
                "Correct board piece detected, but another storage piece also "
                "changed. Restore the extra movement."
            )
            return

        if storage_changes and len(removed_from_storage) != 1:
            self._reject_human_vision_move(
                f"Use exactly one {self._game.human_player} piece from its "
                "storage zone for this move."
            )
            return

        now = time.monotonic()
        if self._vision_candidate_cell != cell:
            self._vision_candidate_cell = cell
            self._vision_candidate_since = now
            self._set_vision_warning("Hold the piece still for vision confirmation...")
            return

        if now - self._vision_candidate_since < 3.0:
            return

        if self._vision_pending_cell != cell:
            self._vision_pending_cell = cell
            self._bridge.vision_provisional_move.emit(cell)
        self._set_vision_warning(
            f"Vision detected cell {cell}. Confirm the move when ready."
        )

    def _pause_for_vision_loss(self):
        if self._paused_for_vision:
            return
        self._paused_for_vision = True
        self._paused_game_started = self._game_started
        self._game_started = False
        self._publish_turn_state()
        self._vision_paused_event.set()
        with self._goal_lock:
            gh = self._active_goal_handle
        if gh is not None:
            gh.cancel_goal_async()
        self._bridge.robot_status_changed.emit("VISION_PAUSED")

    def _resume_after_vision_recovered(self):
        self._paused_for_vision = False
        self._vision_paused_event.clear()
        if self._paused_game_started:
            self._game_started = True
        self._set_vision_warning("")
        self._publish_turn_state()

        if self._interrupted_by_vision:
            self._interrupted_by_vision = False
            threading.Thread(target=self._retry_interrupted_move, daemon=True).start()
        elif self._game_started and not self._ai_turn:
            self._capture_human_turn_baseline()
            self._bridge.robot_status_changed.emit("IDLE")
            self._bridge.human_turn_started.emit()

    def _startup_vision_status(self) -> tuple[bool, str]:
        if not self._require_vision:
            return True, "Vision disabled."
        with self._vision_lock:
            state = dict(self._vision_state or {})
        if not state:
            return False, "Waiting for vision state..."
        if not state.get("board_detected", False):
            return False, "Board not detected. Make the 5 green dots visible."

        board = list(state.get("board") or [])
        storage1 = list(state.get("storage1") or [])
        storage2 = list(state.get("storage2") or [])
        all_cells = board + storage1 + storage2
        x_count = all_cells.count("X")
        o_count = all_cells.count("O")

        if x_count != 5 or o_count != 5:
            return False, f"Expected 5 X and 5 O pieces, detected X={x_count}, O={o_count}."
        if any(cell != " " for cell in board):
            return False, "Board must be empty before starting the game."
        if any(cell != "X" for cell in storage1):
            return False, "X pieces must be in storage1 before starting."
        if any(cell != "O" for cell in storage2):
            return False, "O pieces must be in storage2 before starting."
        return True, "Vision startup state is valid."

    def wait_for_startup_vision(self) -> bool:
        last_message = ""
        last_log = 0.0
        while rclpy.ok():
            status, message = self._startup_vision_action()
            if status == "ready":
                self.get_logger().info(message)
                return True
            if status == "sort":
                self.get_logger().warning(message)
                if not self._sort_pieces_to_storage(prompt_for_misplaced=True):
                    self.get_logger().warning(
                        "Startup sorting did not complete. Fix the board or "
                        "storage state; waiting for a valid vision state."
                    )
                    time.sleep(1.0)
                    continue
                continue
            now = time.monotonic()
            if message != last_message or now - last_log > 2.0:
                self.get_logger().warning(message)
                last_message = message
                last_log = now
            time.sleep(0.2)
        return False

    def _startup_vision_action(self) -> tuple[str, str]:
        ok, message = self._startup_vision_status()
        if ok:
            return "ready", message
        with self._vision_lock:
            state = dict(self._vision_state or {})
        if not state or not state.get("board_detected", False):
            return "wait", message

        board = list(state.get("board") or [])
        storage1 = list(state.get("storage1") or [])
        storage2 = list(state.get("storage2") or [])
        all_cells = board + storage1 + storage2
        if all_cells.count("X") != 5 or all_cells.count("O") != 5:
            return "wait", message

        if self._startup_sort_declined:
            return "wait", "Startup sorting was declined by the operator."

        if self._is_sortable_vision_state(state):
            return (
                "sort",
                "Initial pieces are present but not sorted. Starting vision "
                "guided sorting before opening the GUI.",
            )
        return "wait", message

    def _is_sortable_vision_state(self, state: dict) -> bool:
        board = list(state.get("board") or [])
        storage1 = list(state.get("storage1") or [])
        storage2 = list(state.get("storage2") or [])
        return (
            any(cell in ("X", "O") for cell in board)
            or any(cell == "O" for cell in storage1)
            or any(cell == "X" for cell in storage2)
        )

    def _sort_pieces_to_storage(self, prompt_for_misplaced: bool = False) -> bool:
        if prompt_for_misplaced and self._has_storage_misplacement():
            try:
                answer = input(
                    "Vision detected pieces in the wrong storage zone. Continue "
                    "with dynamic sorting? [yes/no]: "
                ).strip().lower()
            except EOFError:
                self.get_logger().warning(
                    "Cannot ask for storage-misplacement confirmation because "
                    "stdin is unavailable. Waiting instead of exiting."
                )
                return False
            if answer not in ("y", "yes"):
                self.get_logger().warning("Startup cancelled by operator.")
                self._startup_sort_declined = True
                return False

        self._bridge.robot_status_changed.emit("BUSY")
        for attempt in range(30):
            with self._vision_lock:
                state = dict(self._vision_state or {})
            if self._storage_sort_complete(state):
                self.get_logger().info("Vision-guided sorting complete.")
                self._bridge.robot_status_changed.emit("IDLE")
                return True

            move = self._plan_next_sort_move(state)
            if move is None:
                self.get_logger().error(
                    "No valid dynamic sorting move found for current vision state."
                )
                self._bridge.robot_status_changed.emit("IDLE")
                return False

            source_slot, target_slot = move
            self.get_logger().info(
                f"Sorting move {attempt + 1}: {source_slot} -> {target_slot}"
            )
            self._publish_vision_mode("ROBOT")
            if not self._call_move_piece(source_slot, target_slot):
                self._publish_turn_state()
                self._bridge.robot_status_changed.emit("IDLE")
                return False
            self._publish_turn_state()
            self._wait_for_vision_after_move()

        self.get_logger().error("Sorting did not converge after 30 moves.")
        self._bridge.robot_status_changed.emit("IDLE")
        return False

    def _has_storage_misplacement(self) -> bool:
        with self._vision_lock:
            state = dict(self._vision_state or {})
        return any(cell == "O" for cell in state.get("storage1", [])) or any(
            cell == "X" for cell in state.get("storage2", [])
        )

    @staticmethod
    def _storage_sort_complete(state: dict) -> bool:
        board = list(state.get("board") or [])
        storage1 = list(state.get("storage1") or [])
        storage2 = list(state.get("storage2") or [])
        return (
            len(board) == 9
            and len(storage1) == 5
            and len(storage2) == 5
            and all(cell == " " for cell in board)
            and all(cell == "X" for cell in storage1)
            and all(cell == "O" for cell in storage2)
        )

    def _plan_next_sort_move(self, state: dict) -> tuple[str, str] | None:
        board = list(state.get("board") or [])
        storage1 = list(state.get("storage1") or [])
        storage2 = list(state.get("storage2") or [])
        if len(board) != 9 or len(storage1) != 5 or len(storage2) != 5:
            return None

        empty_board = self._first_empty(board)
        empty_s1 = self._first_empty(storage1)
        empty_s2 = self._first_empty(storage2)

        for i, symbol in enumerate(board):
            if symbol == "X" and empty_s1 is not None:
                return f"board_{i}", f"storage1_{empty_s1}"
            if symbol == "O" and empty_s2 is not None:
                return f"board_{i}", f"storage2_{empty_s2}"

        for i, symbol in enumerate(storage1):
            if symbol == "O" and empty_s2 is not None:
                return f"storage1_{i}", f"storage2_{empty_s2}"
        for i, symbol in enumerate(storage2):
            if symbol == "X" and empty_s1 is not None:
                return f"storage2_{i}", f"storage1_{empty_s1}"

        wrong_s1 = next((i for i, symbol in enumerate(storage1) if symbol == "O"), None)
        wrong_s2 = next((i for i, symbol in enumerate(storage2) if symbol == "X"), None)
        if wrong_s1 is not None and empty_board is not None:
            return f"storage1_{wrong_s1}", f"board_{empty_board}"
        if wrong_s2 is not None and empty_board is not None:
            return f"storage2_{wrong_s2}", f"board_{empty_board}"

        return None

    @staticmethod
    def _first_empty(cells: list[str]) -> int | None:
        return next((i for i, cell in enumerate(cells) if cell == " "), None)

    def _wait_for_vision_after_move(self, timeout: float = 3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.1)

    def _plan_finished_restart_collection(
        self,
        fresh_after: float,
    ) -> list[tuple[str, str, str]] | None:
        deadline = time.monotonic() + 5.0
        state: dict = {}
        while time.monotonic() < deadline and rclpy.ok():
            with self._vision_lock:
                state = dict(self._vision_state or {})
            if (
                state
                and state.get("received_at", 0.0) >= fresh_after
                and state.get("board_detected", False)
                and not state.get("board_detection_paused", False)
            ):
                break
            time.sleep(0.05)

        if not state or state.get("received_at", 0.0) < fresh_after:
            self.get_logger().error(
                "Cannot plan finished-match restart: no fresh vision frame "
                "after entering idle mode."
            )
            return None

        if not state.get("board_detected", False):
            self.get_logger().error(
                "Cannot plan finished-match restart: board is not detected."
            )
            return None

        if state.get("board_detection_paused", False):
            self.get_logger().error(
                "Cannot plan finished-match restart: board detection is paused."
            )
            return None

        board = list(state.get("board") or [])
        storage1 = list(state.get("storage1") or [])
        storage2 = list(state.get("storage2") or [])
        if len(board) != 9 or len(storage1) != 5 or len(storage2) != 5:
            self.get_logger().error(
                "Cannot plan finished-match restart: incomplete vision state."
            )
            return None

        free_targets = {
            "X": [f"storage1_{i}" for i, cell in enumerate(storage1) if cell == " "],
            "O": [f"storage2_{i}" for i, cell in enumerate(storage2) if cell == " "],
        }
        board_pieces = [
            (i, symbol) for i, symbol in enumerate(board) if symbol in ("X", "O")
        ]
        needed = {
            "X": sum(1 for _, symbol in board_pieces if symbol == "X"),
            "O": sum(1 for _, symbol in board_pieces if symbol == "O"),
        }

        for symbol in ("X", "O"):
            if needed[symbol] > len(free_targets[symbol]):
                self.get_logger().error(
                    "Cannot plan finished-match restart: "
                    f"{needed[symbol]} {symbol} pieces on board but only "
                    f"{len(free_targets[symbol])} empty {symbol} storage holes."
                )
                return None

        plan: list[tuple[str, str, str]] = []
        for board_index, symbol in board_pieces:
            target_slot = free_targets[symbol].pop(0)
            plan.append((f"board_{board_index}", target_slot, symbol))

        return plan

    def _get_fresh_collection_state(
        self,
        fresh_after: float,
        timeout: float = 5.0,
    ) -> dict | None:
        deadline = time.monotonic() + timeout
        state: dict = {}
        while time.monotonic() < deadline and rclpy.ok():
            with self._vision_lock:
                state = dict(self._vision_state or {})
            if (
                state
                and state.get("received_at", 0.0) >= fresh_after
                and state.get("board_detected", False)
                and not state.get("board_detection_paused", False)
                and len(state.get("board") or []) == 9
                and len(state.get("storage1") or []) == 5
                and len(state.get("storage2") or []) == 5
            ):
                return state
            time.sleep(0.05)
        return None

    @staticmethod
    def _slot_value(state: dict, slot: str) -> str | None:
        try:
            zone, raw_index = slot.rsplit("_", 1)
            index = int(raw_index)
        except ValueError:
            return None

        cells = state.get(zone)
        if not isinstance(cells, list) or index < 0 or index >= len(cells):
            return None
        value = str(cells[index]).upper()
        return value if value in ("X", "O") else " "

    def _verify_collection_move_before_start(
        self,
        plan: list[tuple[str, str, str]],
        next_index: int,
        fresh_after: float,
    ) -> bool:
        state = self._get_fresh_collection_state(fresh_after)
        if state is None:
            self.get_logger().error(
                "Reset collection stopped: no fresh vision state while robot "
                "is home."
            )
            return False

        for completed_index in range(next_index):
            source_slot, target_slot, symbol = plan[completed_index]
            source_value = self._slot_value(state, source_slot)
            target_value = self._slot_value(state, target_slot)
            if source_value != " " or target_value != symbol:
                self.get_logger().error(
                    "Reset collection stopped: vision does not match completed "
                    f"move {completed_index + 1}. Expected {source_slot}=empty "
                    f"and {target_slot}={symbol}, got {source_slot}="
                    f"{source_value!r}, {target_slot}={target_value!r}."
                )
                return False

        if next_index >= len(plan):
            self.get_logger().info("Verified completed restart collection plan.")
            return True

        source_slot, target_slot, symbol = plan[next_index]
        source_value = self._slot_value(state, source_slot)
        target_value = self._slot_value(state, target_slot)

        if source_value != symbol:
            self.get_logger().error(
                "Reset collection stopped: next source does not contain the "
                f"expected {symbol}. Got {source_slot}={source_value!r}."
            )
            return False

        if target_value != " ":
            self.get_logger().error(
                "Reset collection stopped before movement: target slot "
                f"{target_slot} is not empty according to vision "
                f"({target_value!r})."
            )
            return False

        self.get_logger().info(
            f"Verified collection move {next_index + 1}: "
            f"{source_slot}={symbol}, {target_slot}=empty."
        )
        return True

    def _publish_turn_state(self):
        if not hasattr(self, "_turn_pub"):
            return
        if not self._game_started:
            self._publish_vision_mode("IDLE")
        elif self._ai_turn:
            self._publish_vision_mode("ROBOT")
        else:
            self._publish_vision_mode("HUMAN")

    def _publish_vision_mode(self, mode: str):
        if not hasattr(self, "_turn_pub"):
            return
        msg = String()
        msg.data = mode
        self._turn_pub.publish(msg)

    # ── movement sequences ─────────────────────────────────────────────

    def _do_go_home(self, emit_reset: bool = True):
        self.get_logger().info("🏠 Sending robot home...")

        # Clear the emergency event so the HOME goal is not blocked.
        self._emergency_event.clear()

        if not self._place_client.server_is_ready():
            time.sleep(1.0)
            if emit_reset:
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
            if emit_reset:
                self._bridge.reset_completed.emit()
            return

        result_future = gh.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        self.get_logger().info("🏠 Robot at home.")
        if emit_reset:
            self._bridge.reset_completed.emit()

    def _do_collect_board_and_home(self):
        self.get_logger().info(
            "Finished match restart: collecting board pieces into storage first."
        )
        self._game_started = False
        fresh_after = time.monotonic()
        self._publish_turn_state()
        self._set_vision_warning("Collecting board pieces into storage...")
        collection_ok = True

        if self._require_vision:
            plan = self._plan_finished_restart_collection(fresh_after)
            if plan is None:
                collection_ok = False
                self._set_vision_warning(
                    "Reset collection could not be planned. Sending robot home "
                    "for safety."
                )
            elif not plan:
                self.get_logger().info("No board pieces found for restart collection.")
                self._set_vision_warning("")
            else:
                self.get_logger().info(
                    f"Finished-match collection plan has {len(plan)} fixed moves."
                )
                ok = True
                for move_index, (source_slot, target_slot, symbol) in enumerate(
                    plan, start=1
                ):
                    self._publish_vision_mode("IDLE")
                    fresh_after = time.monotonic()
                    if not self._verify_collection_move_before_start(
                        plan, move_index - 1, fresh_after
                    ):
                        ok = False
                        break

                    self.get_logger().info(
                        "Collection move "
                        f"{move_index}/{len(plan)} ({symbol}): "
                        f"{source_slot} -> {target_slot}"
                    )
                    self._publish_vision_mode("ROBOT")
                    if not self._call_move_piece(source_slot, target_slot, fast=True):
                        ok = False
                        break
                    self._publish_vision_mode("IDLE")

                if ok:
                    self._publish_vision_mode("IDLE")
                    fresh_after = time.monotonic()
                    ok = self._verify_collection_move_before_start(
                        plan, len(plan), fresh_after
                    )
                    if ok:
                        self._set_vision_warning("")
                    else:
                        self._set_vision_warning(
                            "Reset collection finished physically, but final "
                            "vision verification failed."
                        )
                else:
                    self._publish_vision_mode("IDLE")
                    self._set_vision_warning(
                        "Reset collection failed. Sending robot home for safety."
                    )
                collection_ok = ok

        if collection_ok:
            self._game.reset()
            self._do_go_home(emit_reset=True)
        else:
            self._do_go_home(emit_reset=False)

    def _do_human_teleop_move(self, cell_index: int):
        self._last_human_cell  = cell_index
        self._pending_symbol   = self._game.human_player
        self._pending_cell_idx = cell_index
        self._move_was_completed = False
        self._move_logic_applied = False
        self._interrupted_by_vision = False

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
        self._publish_turn_state()
        self._do_ai_turn()

    def _do_ai_turn(self):
        self._publish_turn_state()
        move = self._game.get_best_move()
        self._bridge.ai_thinking.emit(move)

        self._pending_symbol   = self._game.ai_player
        self._pending_cell_idx = move
        self._move_was_completed = False
        self._move_logic_applied = False
        self._interrupted_by_vision = False

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
            self._begin_human_turn(wait_for_fresh_vision=True)

    def _check_game_over(self) -> bool:
        if not self._game.game_over():
            return False
        winner = self._game.check_winner()
        result = winner if winner else "TIE"
        self._bridge.game_over.emit(result)
        self._game_started = False
        self._publish_turn_state()
        return True

    def _call_place_piece(self, symbol: str, cell_index: int) -> bool:
        """
        Send a goal and wait for the result.
        Check _emergency_event in each wait loop iteration.
        Return True on success, False on emergency or error.
        """
        self._robot_motion_active = True
        try:
            return self._call_place_piece_impl(symbol, cell_index)
        finally:
            self._robot_motion_active = False

    def _call_place_piece_impl(self, symbol: str, cell_index: int) -> bool:
        if not self._place_client.server_is_ready():
            # Simulation: emergency-interruptible sleep
            for _ in range(30):
                if self._emergency_event.is_set() or self._vision_paused_event.is_set():
                    if self._vision_paused_event.is_set():
                        self._interrupted_by_vision = True
                    return False
                time.sleep(0.05)
            return not self._emergency_event.is_set() and not self._vision_paused_event.is_set()

        goal = PlacePiece.Goal()
        goal.symbol     = symbol
        goal.cell_index = cell_index

        send_future = self._place_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        while not send_future.done():
            if self._emergency_event.is_set() or self._vision_paused_event.is_set():
                # We do not have the goal handle yet. Cancel once ready.
                def _cancel_when_ready(f):
                    try:
                        _gh = f.result()
                        if _gh and _gh.accepted:
                            _gh.cancel_goal_async()
                    except Exception:
                        pass
                send_future.add_done_callback(_cancel_when_ready)
                if self._vision_paused_event.is_set():
                    self._interrupted_by_vision = True
                return False
            time.sleep(0.05)

        gh = send_future.result()
        if not gh.accepted:
            self.get_logger().error("Goal rejected.")
            return False

        # Emergency arrived exactly as send_future resolved
        if self._emergency_event.is_set() or self._vision_paused_event.is_set():
            gh.cancel_goal_async()
            if self._vision_paused_event.is_set():
                self._interrupted_by_vision = True
            return False

        with self._goal_lock:
            self._active_goal_handle = gh

        result_future = gh.get_result_async()
        while not result_future.done():
            if self._emergency_event.is_set() or self._vision_paused_event.is_set():
                # Also cancel on the server side
                gh.cancel_goal_async()
                with self._goal_lock:
                    self._active_goal_handle = None
                if self._vision_paused_event.is_set():
                    self._interrupted_by_vision = True
                return False
            time.sleep(0.05)

        with self._goal_lock:
            self._active_goal_handle = None

        result = result_future.result()

        from action_msgs.msg import GoalStatus

        if self._emergency_event.is_set() or self._vision_paused_event.is_set():
            self.get_logger().warn("Result ignored due to emergency.")
            if self._vision_paused_event.is_set():
                self._interrupted_by_vision = True
            return False

        if result.status == GoalStatus.STATUS_CANCELED:
            return False

        return result.result.success

    def _call_move_piece(
        self,
        source_slot: str,
        target_slot: str,
        fast: bool = False,
    ) -> bool:
        """
        Send an explicit source-slot to target-slot move.
        Used for startup sorting and reset cleanup.
        """
        self._robot_motion_active = True
        try:
            return self._call_move_piece_impl(source_slot, target_slot, fast)
        finally:
            self._robot_motion_active = False

    def _call_move_piece_impl(
        self,
        source_slot: str,
        target_slot: str,
        fast: bool,
    ) -> bool:
        if not self._move_client.server_is_ready():
            for _ in range(30):
                if self._emergency_event.is_set() or self._vision_paused_event.is_set():
                    return False
                time.sleep(0.05)
            return (
                not self._emergency_event.is_set()
                and not self._vision_paused_event.is_set()
            )

        goal = MovePiece.Goal()
        goal.source_slot = source_slot
        goal.target_slot = target_slot
        goal.fast = fast

        send_future = self._move_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        while not send_future.done():
            if self._emergency_event.is_set() or self._vision_paused_event.is_set():
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
            self.get_logger().error("MovePiece goal rejected.")
            return False

        if self._emergency_event.is_set() or self._vision_paused_event.is_set():
            gh.cancel_goal_async()
            return False

        with self._goal_lock:
            self._active_goal_handle = gh

        result_future = gh.get_result_async()
        while not result_future.done():
            if self._emergency_event.is_set() or self._vision_paused_event.is_set():
                gh.cancel_goal_async()
                with self._goal_lock:
                    self._active_goal_handle = None
                return False
            time.sleep(0.05)

        with self._goal_lock:
            self._active_goal_handle = None

        result = result_future.result()

        from action_msgs.msg import GoalStatus

        if result.status == GoalStatus.STATUS_CANCELED:
            return False
        if not result.result.success:
            self.get_logger().error(result.result.message)
            return False
        return True


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

    if not node.wait_for_startup_vision():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

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
