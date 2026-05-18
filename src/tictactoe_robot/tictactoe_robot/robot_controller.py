"""
robot_controller.py
────────────────────
ROS2 node  'robot_controller'

Operating modes
───────────────
  simulate:=false (lab default)
    → Connect to the real UR3 action server and SetIO service.

  simulate:=true (without robot)
    → Skip all hardware communication.
      Movements are simulated with time.sleep().
      Publishes robot_status exactly as in real mode.

Emergency stop
──────────────
  The PlacePiece goal can be canceled at any time from game_node.py.
  When canceled:
    • the current movement is interrupted between waypoints
    • the gripper state is left unchanged
    • robot_status = "EMERGENCY_STOP" is published
  On resume, the controller continues from the interrupted sequence step.
  On restart, game_node sends the robot back home.
"""

import json
import time
import threading
from pathlib import Path

import rclpy
from rclpy.node             import Node
from rclpy.action           import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups  import ReentrantCallbackGroup

from builtin_interfaces.msg import Duration
from std_msgs.msg           import String
from control_msgs.action    import FollowJointTrajectory
from trajectory_msgs.msg    import JointTrajectoryPoint
from ur_msgs.srv            import SetIO

from tictactoe_interfaces.action import MovePiece, PlacePiece
from ament_index_python.packages import get_package_share_directory


# ─────────────────────────────────────────────────────────────── constants

APPROACH_OFFSET = [-0.007, +0.1311, +0.2004, -0.3277, 0.0, 0.0]
WAYPOINT_TIME   = 2.5
FAST_WAYPOINT_TIME = 1.5
GRIPPER_WAIT    = 1.0
STOCK_COUNT     = 5

UR_ACTION      = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
SETIO_SERVICE  = "/io_and_status_controller/set_io"

UR3_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

SIM_WAYPOINT_TIME = 0.5
FAST_SIM_WAYPOINT_TIME = 0.25
SIM_GRIPPER_TIME  = 0.1
UR_CANCEL_SETTLE_TIMEOUT = 3.0
UR_RESULT_TIMEOUT_MARGIN = 20.0


# ─────────────────────────────────────────────────────────────── helpers

def _apply_approach(joints: list[float]) -> list[float]:
    return [round(j + o, 7) for j, o in zip(joints, APPROACH_OFFSET)]


def _make_point(joints: list[float], t_sec: float) -> JointTrajectoryPoint:
    pt = JointTrajectoryPoint()
    pt.positions       = joints
    pt.velocities      = [0.0] * len(joints)
    pt.time_from_start = Duration(
        sec=int(t_sec), nanosec=int((t_sec % 1) * 1e9)
    )
    return pt


# ─────────────────────────────────────────────────────────────── node

class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller")
        cbg = ReentrantCallbackGroup()

        # ── simulate parameter ─────────────────────────────────────────
        self.declare_parameter("simulate", False)
        self._simulate: bool = self.get_parameter("simulate").value
        mode_str = "SIMULATION 🖥️" if self._simulate else "REAL HARDWARE 🤖"
        self.get_logger().info(f"Operation Mode: {mode_str}")

        # ── Load positions ───────────────────────────────────────────
        try:
            pkg_path       = get_package_share_directory("tictactoe_robot")
            positions_file = Path(pkg_path) / "positions.json"
        except Exception:
            positions_file = Path("positions.json")

        if not positions_file.exists():
            positions_file = Path("positions.json")

        with open(positions_file) as f:
            self.positions: dict[str, list[float]] = json.load(f)

        self.get_logger().info(
            f"{len(self.positions)} positions loaded from {positions_file}"
        )

        # ── piece counter ──────────────────────────────────────────────
        self._stock_index: dict[str, int] = {"X": 1, "O": 1}

        # ── Hardware (only if not added simulate:=true) ────────────────────────────
        if not self._simulate:
            self._ur_client = ActionClient(
                self, FollowJointTrajectory, UR_ACTION, callback_group=cbg
            )
            self.get_logger().info("Waiting for the UR3 action server...")
            self._ur_client.wait_for_server()
            self.get_logger().info("✅ Action server UR3 ready.")

            self._io_client = self.create_client(
                SetIO, SETIO_SERVICE, callback_group=cbg
            )
            self.get_logger().info("Waiting for the gripper SetIO service...")
            self._io_client.wait_for_service()
            self.get_logger().info("✅ SetIO Service ready.")
        else:
            self._ur_client = None
            self._io_client = None
            self.get_logger().info(
                "⚡ Simulation mode enabled — hardware skipped."
            )

        # ── State Publisher ─────────────────────────────────────────
        self._status_pub = self.create_publisher(String, "~/robot_status", 10)
        self._operation_pub = self.create_publisher(
            String,
            "~/operation_status",
            10,
        )
        self._operation_lock = threading.Lock()
        self._mission_counter = 0
        self._current_operation = {
            "mission_id": "IDLE",
            "task": "Idle",
            "status": "IDLE",
            "phase": "idle",
            "target": "-",
            "fast": False,
            "mode": "simulation" if self._simulate else "hardware",
            "motor_power_pct": {joint: 0.0 for joint in UR3_JOINTS},
        }
        self.create_timer(0.5, self._publish_operation_status)

        # ── Action server place_piece ───────────────────────────────────
        self._action_server = ActionServer(
            self,
            PlacePiece,
            "~/place_piece",
            execute_callback=self._execute_place_piece,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cbg,
        )
        self._move_piece_server = ActionServer(
            self,
            MovePiece,
            "~/move_piece",
            execute_callback=self._execute_move_piece,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cbg,
        )

        # Active goal currently executing
        self._active_goal_handle = None
        self._cancel_requested   = threading.Event()
        self._resume_context: dict[str, int | str | bool] | None = None

        self.get_logger().info(
            "RobotController ready. Action servers ~/place_piece and "
            "~/move_piece active."
        )

    # ─────────────────────────────────────── callbacks del action server

    def _goal_callback(self, goal_request):
        """Accept any goal.
        Reject if another goal is active, unless cancellation has already been
        requested and the current execute callback is about to finish.
        """
        if self._active_goal_handle is not None and not self._cancel_requested.is_set():
            self.get_logger().warning("Goal rejected: another move is already active.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Always accept a cancellation request."""
        self.get_logger().info("⛔ Goal cancellation requested (emergency).")
        self._cancel_requested.set()
        return CancelResponse.ACCEPT

    def _execute_place_piece(self, goal_handle):
        """Callback principal del action server."""
        self._active_goal_handle = goal_handle
        self._cancel_requested.clear()

        symbol     = goal_handle.request.symbol.upper()
        cell_index = goal_handle.request.cell_index

        self.get_logger().info(
            f"PlacePiece goal received: symbol='{symbol}' cell={cell_index}"
        )
        mission_id = self._next_mission_id("HOME" if symbol == "HOME" else "PLACE")
        task = (
            "Return robot to HOME"
            if symbol == "HOME"
            else f"Place {symbol} in board_{cell_index}"
        )
        self._set_operation(
            mission_id=mission_id,
            task=task,
            status="BUSY",
            phase="goal_received",
            target="home" if symbol == "HOME" else f"cell_{cell_index}",
            fast=False,
            motor_power_pct=self._motor_power(60.0),
        )

        result = PlacePiece.Result()

        # Special goal: return home after an emergency restart
        if symbol == "HOME":
            reset_stock = cell_index == -2
            hold_piece_for_validation = cell_index == -3
            try:
                self.go_home(
                    reset_stock=reset_stock,
                    hold_piece_for_validation=hold_piece_for_validation,
                )
            except _EmergencyStop:
                self.get_logger().warn("HOME interrupted by emergency.")
                self._publish_status("EMERGENCY_STOP")
                goal_handle.canceled()
                result.success = False
                result.message = "Emergency stop activated during HOME"
                self._active_goal_handle = None
                return result
            except Exception as e:
                self.get_logger().error(f"go_home failed: {e}")
                goal_handle.abort()
                result.success = False
                result.message = str(e)
                self._active_goal_handle = None
                return result
            goal_handle.succeed()
            result.success = True
            result.message = "Robot at home"
            self._active_goal_handle = None
            return result

        if symbol == "RETURN_HELD_HOME":
            reset_stock = cell_index == -2
            try:
                self.return_held_piece_home(reset_stock=reset_stock)
            except _EmergencyStop:
                self.get_logger().warn("Return-held-home interrupted by emergency.")
                self._publish_status("EMERGENCY_STOP")
                goal_handle.canceled()
                result.success = False
                result.message = "Emergency stop activated during held-piece return"
                self._active_goal_handle = None
                return result
            except Exception as e:
                self.get_logger().error(f"return_held_piece_home failed: {e}")
                goal_handle.abort()
                result.success = False
                result.message = str(e)
                self._active_goal_handle = None
                return result
            goal_handle.succeed()
            result.success = True
            result.message = "Held piece returned and robot at home"
            self._active_goal_handle = None
            return result

        if symbol == "CLEAR_RESUME_CONTEXT":
            self._resume_context = None
            self.get_logger().info("Cleared interrupted movement resume context.")
            goal_handle.succeed()
            result.success = True
            result.message = "Resume context cleared"
            self._active_goal_handle = None
            return result

        if symbol.startswith("CONSUME_STOCK_"):
            piece_symbol = symbol.removeprefix("CONSUME_STOCK_").upper()
            source_index = int(cell_index)
            if piece_symbol not in self._stock_index or not 0 <= source_index < STOCK_COUNT:
                goal_handle.abort()
                result.success = False
                result.message = (
                    f"Invalid stock consume request: {piece_symbol} "
                    f"source_index={source_index}"
                )
                self._active_goal_handle = None
                return result

            next_index = source_index + 2  # storage index is 0-based; stock index is 1-based next piece.
            if self._stock_index[piece_symbol] < next_index:
                self._stock_index[piece_symbol] = next_index
            self.get_logger().info(
                f"Marked {piece_symbol} stock source {source_index} consumed; "
                f"next index is {self._stock_index[piece_symbol]}."
            )
            goal_handle.succeed()
            result.success = True
            result.message = "Stock source consumed"
            self._active_goal_handle = None
            return result

        try:
            start_step = 0
            if (
                self._resume_context is not None
                and self._resume_context.get("mode") == "place_piece"
                and self._resume_context.get("symbol") == symbol
                and self._resume_context.get("cell_index") == cell_index
            ):
                start_step = int(self._resume_context["next_step"])
                self.get_logger().info(
                    f"▶ Reanudando pick-and-place desde el paso {start_step}."
                )
            else:
                self._resume_context = None

            self._pick_and_place(symbol, cell_index, goal_handle, start_step)
        except _EmergencyStop:
            self.get_logger().warn("⛔ Pick-and-place interrupted by emergency.")
            self._publish_status("EMERGENCY_STOP")
            goal_handle.canceled()
            result.success = False
            result.message = "Emergency stop activated"
            self._active_goal_handle = None
            return result
        except Exception as e:
            self.get_logger().error(str(e))
            self._resume_context = None
            self._publish_status("IDLE")
            goal_handle.abort()
            result.success = False
            result.message = str(e)
            self._active_goal_handle = None
            return result

        self._publish_status("IDLE")
        goal_handle.succeed()
        result.success = True
        result.message = "Completed movement"
        self._resume_context = None
        self._active_goal_handle = None
        return result

    def _execute_move_piece(self, goal_handle):
        """Move a piece between two explicit physical slots."""
        self._active_goal_handle = goal_handle
        self._cancel_requested.clear()

        source_slot = goal_handle.request.source_slot
        target_slot = goal_handle.request.target_slot
        fast = bool(goal_handle.request.fast)

        self.get_logger().info(
            f"MovePiece goal received: '{source_slot}' -> '{target_slot}' "
            f"(fast={fast})"
        )
        self._set_operation(
            mission_id=self._next_mission_id("MOVE"),
            task=f"Move {source_slot} -> {target_slot}",
            status="BUSY",
            phase="goal_received",
            target=target_slot,
            fast=fast,
            motor_power_pct=self._motor_power(70.0 if fast else 60.0),
        )

        result = MovePiece.Result()
        try:
            start_step = 0
            if (
                self._resume_context is not None
                and self._resume_context.get("mode") == "move_piece"
                and self._resume_context["source_slot"] == source_slot
                and self._resume_context["target_slot"] == target_slot
                and bool(self._resume_context.get("fast", False)) == fast
            ):
                start_step = int(self._resume_context["next_step"])
                self.get_logger().info(
                    f"Resuming explicit move from step {start_step}."
                )
            else:
                self._resume_context = None

            self._move_piece_between_slots(
                source_slot, target_slot, goal_handle, start_step, fast
            )
        except _EmergencyStop:
            self.get_logger().warn("MovePiece interrupted by emergency.")
            self._publish_status("EMERGENCY_STOP")
            goal_handle.canceled()
            result.success = False
            result.message = "Emergency stop activated"
            self._active_goal_handle = None
            return result
        except Exception as e:
            self.get_logger().error(str(e))
            self._resume_context = None
            self._publish_status("IDLE")
            goal_handle.abort()
            result.success = False
            result.message = str(e)
            self._active_goal_handle = None
            return result

        self._publish_status("IDLE")
        goal_handle.succeed()
        result.success = True
        result.message = "Completed explicit move"
        self._resume_context = None
        self._active_goal_handle = None
        return result

    # ─────────────────────────────────────── gripper

    def _mark_context_holding_piece(self, holding: bool):
        if self._resume_context is not None:
            self._resume_context["holding_piece"] = holding

    def _open_gripper(self):
        if self._cancel_requested.is_set():
            raise _EmergencyStop()
        self._set_operation(
            status="GRIPPER",
            phase="open_gripper",
            target="gripper",
            motor_power_pct=self._motor_power(0.0),
        )
        if self._simulate:
            self.get_logger().info("  🤏 [SIM] Gripper: OPEN")
            self._mark_context_holding_piece(False)
            self._interruptible_sleep(SIM_GRIPPER_TIME)
            return

        req = SetIO.Request()
        req.fun   = 1
        req.pin   = 0
        req.state = 0.0
        self._io_client.call_async(req)
        self.get_logger().info("  🤏 Gripper: OPEN")
        self._mark_context_holding_piece(False)
        self._interruptible_sleep(GRIPPER_WAIT)

    def _close_gripper(self):
        if self._cancel_requested.is_set():
            raise _EmergencyStop()
        self._set_operation(
            status="GRIPPER",
            phase="close_gripper",
            target="gripper",
            motor_power_pct=self._motor_power(0.0),
        )
        if self._simulate:
            self.get_logger().info("  🤏 [SIM] Gripper: CLOSED")
            self._mark_context_holding_piece(True)
            self._interruptible_sleep(SIM_GRIPPER_TIME)
            return

        req = SetIO.Request()
        req.fun   = 1
        req.pin   = 0
        req.state = 1.0
        self._io_client.call_async(req)
        self.get_logger().info("  🤏 Gripper: CLOSED")
        self._mark_context_holding_piece(True)
        self._interruptible_sleep(GRIPPER_WAIT)

    def _consume_context_source_after_target_release(self):
        context = self._resume_context
        if not self._context_target_released(context):
            return
        if context.get("source_consumed", False):
            return

        if context.get("mode") == "place_piece":
            symbol = str(context.get("symbol", "")).upper()
            if symbol in self._stock_index:
                try:
                    source_index = int(
                        context.get("stock_index", self._stock_index[symbol])
                    )
                except (TypeError, ValueError):
                    source_index = self._stock_index[symbol]
                if self._stock_index[symbol] <= source_index:
                    self._stock_index[symbol] = source_index + 1

        context["source_consumed"] = True
        self._publish_operation_status()

    def _open_gripper_at_target(self):
        try:
            self._open_gripper()
        except _EmergencyStop:
            self._consume_context_source_after_target_release()
            raise
        self._consume_context_source_after_target_release()

    # ─────────────────────────────────────── movement

    def _move_to(self, joints: list[float], label: str = "", phase: str = "",
                 goal_handle=None, fast: bool = False):
        self.get_logger().info(f"  → Moving to '{label}'…")
        waypoint_time = FAST_WAYPOINT_TIME if fast else WAYPOINT_TIME
        sim_waypoint_time = FAST_SIM_WAYPOINT_TIME if fast else SIM_WAYPOINT_TIME
        speed_percent = 70.0 if fast else 60.0
        self._set_operation(
            status="MOVING",
            phase=phase or label,
            target=label,
            fast=fast,
            waypoint_time_sec=waypoint_time,
            motor_power_pct=self._motor_power(speed_percent),
        )
        self._publish_status(f"MOVING: {label}", update_operation=False)

        # Publish feedback when a goal_handle exists
        if goal_handle is not None:
            if hasattr(goal_handle.request, "source_slot"):
                fb = MovePiece.Feedback()
            else:
                fb = PlacePiece.Feedback()
            fb.phase    = phase or label
            fb.waypoint = 0
            goal_handle.publish_feedback(fb)

        if self._simulate:
            self._interruptible_sleep(sim_waypoint_time)
            self.get_logger().info(f"  ✅ [SIM] Reached '{label}'")
            return

        # ── Real mode ───────────────────────────────────────────────────
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = UR3_JOINTS
        goal.trajectory.points      = [_make_point(joints, waypoint_time)]

        goal_handle_future = self._ur_client.send_goal_async(goal)

        # Wait for acceptance while checking cancellation
        while not goal_handle_future.done():
            if self._cancel_requested.is_set():
                raise _EmergencyStop()
            time.sleep(0.05)

        ur_goal_handle = goal_handle_future.result()
        if not ur_goal_handle.accepted:
            raise RuntimeError(f"Goal rejected for '{label}'")

        result_future = ur_goal_handle.get_result_async()

        # Wait for the result while checking cancellation
        result_started_at = time.monotonic()
        result_timeout = waypoint_time + UR_RESULT_TIMEOUT_MARGIN
        while not result_future.done():
            if self._cancel_requested.is_set():
                # Also cancel the UR3 goal
                self._cancel_ur_goal_and_wait(
                    ur_goal_handle,
                    result_future,
                    label,
                )
                raise _EmergencyStop()
            if time.monotonic() - result_started_at > result_timeout:
                self._cancel_ur_goal_and_wait(
                    ur_goal_handle,
                    result_future,
                    label,
                )
                raise RuntimeError(
                    f"Timed out waiting for trajectory result toward '{label}'"
                )
            time.sleep(0.05)

        result = result_future.result()
        if result.result.error_code != 0:
            raise RuntimeError(
                f"Trayectoria fallida hacia '{label}' "
                f"(error_code={result.result.error_code})"
            )

        self.get_logger().info(f"  ✅ Reached '{label}'")

    def _cancel_ur_goal_and_wait(self, ur_goal_handle, result_future, label: str):
        self.get_logger().warning(
            f"  ⛔ Cancelling UR trajectory toward '{label}' and waiting for settle."
        )
        try:
            cancel_future = ur_goal_handle.cancel_goal_async()
            deadline = time.monotonic() + UR_CANCEL_SETTLE_TIMEOUT
            while not cancel_future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
        except Exception as exc:
            self.get_logger().warning(
                f"  Could not request UR trajectory cancellation cleanly: {exc}"
            )

        deadline = time.monotonic() + UR_CANCEL_SETTLE_TIMEOUT
        while result_future is not None and not result_future.done():
            if time.monotonic() >= deadline:
                self.get_logger().warning(
                    f"  UR trajectory toward '{label}' did not settle before timeout."
                )
                return
            time.sleep(0.05)

    # ─────────────────────────────────────── pick-and-place

    def _pick_and_place(
        self,
        symbol: str,
        cell_index: int,
        goal_handle,
        start_step: int = 0,
    ):
        idx = self._stock_index[symbol]
        if idx > STOCK_COUNT:
            raise RuntimeError(
                f"No pieces left for '{symbol}' (max {STOCK_COUNT})"
            )

        stock_key = (
            f"pick_stock_{idx}" if symbol == "O"
            else f"pick_stock_{idx}_X"
        )
        stock_slot = f"storage2_{idx - 1}" if symbol == "O" else f"storage1_{idx - 1}"
        cell_key = f"cell_{cell_index}"

        stock_joints   = self.positions[stock_key]
        cell_joints    = self.positions[cell_key]
        home_joints    = self.positions["home"]
        stock_approach = _apply_approach(stock_joints)
        cell_approach  = _apply_approach(cell_joints)

        self.get_logger().info(
            f"━━━ Pick-and-place │ symbol='{symbol}' │ "
            f"stock='{stock_key}' │ cell='{cell_key}' ━━━"
        )
        self._publish_status("BUSY")

        steps = [
            lambda: self._move_to(home_joints, "home", "return_home", goal_handle),
            lambda: self._move_to(
                stock_approach, f"{stock_key}_app", "approach_stock", goal_handle
            ),
            self._open_gripper,
            lambda: self._move_to(stock_joints, stock_key, "pick", goal_handle),
            self._close_gripper,
            lambda: self._move_to(
                stock_approach, f"{stock_key}_app", "approach_stock", goal_handle
            ),
            lambda: self._move_to(
                cell_approach, f"{cell_key}_app", "approach_cell", goal_handle
            ),
            lambda: self._move_to(cell_joints, cell_key, "place", goal_handle),
            self._open_gripper_at_target,
            lambda: self._move_to(
                cell_approach, f"{cell_key}_app", "approach_cell", goal_handle
            ),
            lambda: self._move_to(home_joints, "home", "return_home", goal_handle),
        ]

        if start_step < 0 or start_step >= len(steps):
            start_step = 0

        for step_index in range(start_step, len(steps)):
            self._resume_context = {
                "mode": "place_piece",
                "symbol": symbol,
                "cell_index": cell_index,
                "stock_index": idx,
                "source_slot": stock_slot,
                "target_slot": f"board_{cell_index}",
                "next_step": step_index,
            }
            self._check_cancel()
            steps[step_index]()

        self._resume_context = None

        if self._stock_index[symbol] <= idx:
            self._stock_index[symbol] = idx + 1
        self.get_logger().info("━━━ Sequence Completed ━━━")

    def _move_piece_between_slots(
        self,
        source_slot: str,
        target_slot: str,
        goal_handle,
        start_step: int = 0,
        fast: bool = False,
    ):
        source_key = self._slot_to_position_key(source_slot)
        target_key = self._slot_to_position_key(target_slot)

        source_joints = self.positions[source_key]
        target_joints = self.positions[target_key]
        home_joints = self.positions["home"]
        source_approach = _apply_approach(source_joints)
        target_approach = _apply_approach(target_joints)

        self.get_logger().info(
            f"━━━ Dynamic move │ {source_slot}({source_key}) -> "
            f"{target_slot}({target_key}) │ fast={fast} ━━━"
        )
        self._publish_status("BUSY")

        steps = [
            lambda: self._move_to(
                home_joints, "home", "return_home", goal_handle, fast
            ),
            lambda: self._move_to(
                source_approach, f"{source_key}_app", "approach_source",
                goal_handle, fast
            ),
            self._open_gripper,
            lambda: self._move_to(
                source_joints, source_key, "pick", goal_handle, fast
            ),
            self._close_gripper,
            lambda: self._move_to(
                source_approach, f"{source_key}_app", "approach_source",
                goal_handle, fast
            ),
            lambda: self._move_to(
                target_approach, f"{target_key}_app", "approach_target",
                goal_handle, fast
            ),
            lambda: self._move_to(
                target_joints, target_key, "place", goal_handle, fast
            ),
            self._open_gripper_at_target,
            lambda: self._move_to(
                target_approach, f"{target_key}_app", "approach_target",
                goal_handle, fast
            ),
            lambda: self._move_to(
                home_joints, "home", "return_home", goal_handle, fast
            ),
        ]

        if start_step < 0 or start_step >= len(steps):
            start_step = 0

        for step_index in range(start_step, len(steps)):
            self._resume_context = {
                "mode": "move_piece",
                "source_slot": source_slot,
                "target_slot": target_slot,
                "fast": fast,
                "next_step": step_index,
            }
            self._check_cancel()
            steps[step_index]()

        self._resume_context = None
        self.get_logger().info("━━━ Dynamic Move Completed ━━━")

    @staticmethod
    def _slot_to_position_key(slot: str) -> str:
        try:
            zone, raw_index = slot.rsplit("_", 1)
            index = int(raw_index)
        except ValueError as exc:
            raise RuntimeError(f"Invalid slot name '{slot}'") from exc

        if zone == "board" and 0 <= index <= 8:
            return f"cell_{index}"
        if zone == "storage1" and 0 <= index <= 4:
            return f"pick_stock_{index + 1}_X"
        if zone == "storage2" and 0 <= index <= 4:
            return f"pick_stock_{index + 1}"
        raise RuntimeError(f"Unsupported slot name '{slot}'")

    # ─────────────────────────────────────── helpers

    def _check_cancel(self):
        """Raise _EmergencyStop if cancellation has been requested."""
        if self._cancel_requested.is_set():
            raise _EmergencyStop()

    def _interruptible_sleep(self, duration: float, step: float = 0.05):
        """Interruptible sleep that exits if cancellation arrives."""
        elapsed = 0.0
        while elapsed < duration:
            if self._cancel_requested.is_set():
                raise _EmergencyStop()
            time.sleep(step)
            elapsed += step

    def _next_mission_id(self, prefix: str) -> str:
        self._mission_counter += 1
        return f"{prefix}-{self._mission_counter:04d}"

    def _motor_power(self, percent: float) -> dict[str, float]:
        return {joint: float(percent) for joint in UR3_JOINTS}

    def _set_operation(self, **updates):
        with self._operation_lock:
            self._current_operation.update(updates)
        self._publish_operation_status()

    def _context_source_slot(self, context: dict | None) -> str:
        if not context:
            return ""

        mode = context.get("mode")
        if mode == "move_piece":
            return str(context.get("source_slot", ""))

        if mode != "place_piece":
            return ""

        source_slot = str(context.get("source_slot", ""))
        if source_slot:
            return source_slot

        symbol = str(context.get("symbol", "")).upper()
        if symbol not in ("X", "O"):
            return ""

        stock_index = int(
            context.get("stock_index", self._stock_index.get(symbol, 1))
        )
        zone = "storage2" if symbol == "O" else "storage1"
        return f"{zone}_{stock_index - 1}"

    def _context_target_slot(self, context: dict | None) -> str:
        if not context:
            return ""

        mode = context.get("mode")
        if mode == "move_piece":
            return str(context.get("target_slot", ""))

        if mode != "place_piece":
            return ""

        try:
            cell_index = int(context.get("cell_index", -1))
        except (TypeError, ValueError):
            return ""
        if 0 <= cell_index <= 8:
            return f"board_{cell_index}"
        return ""

    def _context_target_released(self, context: dict | None) -> bool:
        if not context:
            return False
        if context.get("mode") not in ("place_piece", "move_piece"):
            return False
        if self._context_may_hold_piece(context):
            return False
        try:
            next_step = int(context.get("next_step", 0))
        except (TypeError, ValueError):
            return False
        # Step 8 is the target open-gripper step in both move sequences.
        return next_step >= 8

    def _publish_operation_status(self):
        if not hasattr(self, "_operation_pub"):
            return
        with self._operation_lock:
            payload = dict(self._current_operation)
        context = self._resume_context
        holding_piece = self._context_may_hold_piece(context) if context else False
        source_slot = self._context_source_slot(context)
        target_slot = self._context_target_slot(context)
        payload["holding_piece"] = holding_piece
        payload["held_source_slot"] = source_slot if holding_piece else ""
        payload["piece_source_slot"] = source_slot
        payload["piece_target_slot"] = target_slot
        payload["target_released"] = self._context_target_released(context)
        payload["timestamp_ns"] = time.time_ns()
        payload["timestamp"] = time.time()

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._operation_pub.publish(msg)

    def _publish_status(self, status: str, update_operation: bool = True):
        msg      = String()
        msg.data = status
        self._status_pub.publish(msg)
        if not update_operation:
            return

        updates = {"status": status}
        status_upper = status.upper()
        if status_upper == "IDLE":
            updates.update(
                {
                    "mission_id": "IDLE",
                    "task": "Idle",
                    "phase": "idle",
                    "target": "-",
                    "fast": False,
                    "motor_power_pct": self._motor_power(0.0),
                }
            )
        elif status_upper == "EMERGENCY_STOP":
            updates.update(
                {
                    "phase": "emergency_stop",
                    "motor_power_pct": self._motor_power(0.0),
                }
            )
        self._set_operation(**updates)

    # ─────────────────────────────────────── go home (for restart)

    def _context_motion_keys(self, context: dict) -> tuple[str | None, str | None]:
        mode = context.get("mode")
        if mode == "place_piece":
            symbol = str(context.get("symbol", "")).upper()
            cell_index = int(context.get("cell_index", -1))
            source_slot = str(context.get("source_slot", ""))
            if source_slot:
                source_key = self._slot_to_position_key(source_slot)
            else:
                stock_index = int(
                    context.get("stock_index", self._stock_index.get(symbol, 1))
                )
                source_key = (
                    f"pick_stock_{stock_index}" if symbol == "O"
                    else f"pick_stock_{stock_index}_X"
                )
            target_key = f"cell_{cell_index}" if 0 <= cell_index <= 8 else None
            return source_key, target_key

        if mode == "move_piece":
            return (
                self._slot_to_position_key(str(context.get("source_slot", ""))),
                self._slot_to_position_key(str(context.get("target_slot", ""))),
            )

        return None, None

    @staticmethod
    def _context_may_hold_piece(context: dict) -> bool:
        if "holding_piece" in context:
            return bool(context.get("holding_piece", False))

        try:
            next_step = int(context.get("next_step", 0))
        except (TypeError, ValueError):
            return False

        # Fallback for old contexts: only after the close-gripper step has
        # completed can we assume a piece may be held.
        return 5 <= next_step <= 8

    @staticmethod
    def _context_near_key(
        context: dict,
        source_key: str | None,
        target_key: str | None,
    ) -> str | None:
        try:
            next_step = int(context.get("next_step", 0))
        except (TypeError, ValueError):
            return None

        if 3 <= next_step <= 5:
            return source_key
        return None

    def _resume_held_piece_from_home_next(self):
        context = self._resume_context
        if not context or not self._context_may_hold_piece(context):
            return
        if context.get("mode") not in ("place_piece", "move_piece"):
            return

        context["next_step"] = 6
        context["holding_piece"] = True
        self.get_logger().info(
            "Held piece is now at HOME; resume will continue from target approach."
        )
        self._publish_operation_status()

    def _recover_interrupted_context_before_home(
        self,
        hold_piece_for_validation: bool = False,
    ) -> bool:
        context = self._resume_context
        if not context:
            return False

        try:
            source_key, target_key = self._context_motion_keys(context)
            near_key = self._context_near_key(context, source_key, target_key)
            holding_piece = self._context_may_hold_piece(context)

            if near_key is not None and near_key in self.positions:
                near_approach = _apply_approach(self.positions[near_key])
                self._move_to(
                    near_approach,
                    f"{near_key}_app",
                    "safe_lift_before_home",
                    goal_handle=None,
                )

            recovery_key = source_key
            if holding_piece and (
                recovery_key is None or recovery_key not in self.positions
            ):
                return False

            if holding_piece:
                if hold_piece_for_validation:
                    self.get_logger().warning(
                        "HOME requested while the gripper may hold a piece; "
                        "keeping it in the gripper for external vision "
                        "validation before it is returned."
                    )
                    return True

                self.get_logger().warning(
                    "HOME requested while the gripper may hold a piece; "
                    "returning it to the source slot before going home."
                )
                recovery_joints = self.positions[recovery_key]
                recovery_approach = _apply_approach(recovery_joints)
                self._move_to(
                    recovery_approach,
                    f"{recovery_key}_app",
                    "recover_approach_source",
                    goal_handle=None,
                )
                self._move_to(
                    recovery_joints,
                    recovery_key,
                    "recover_return_piece",
                    goal_handle=None,
                )
                self._open_gripper()
                self._move_to(
                    recovery_approach,
                    f"{recovery_key}_app",
                    "recover_leave_source",
                    goal_handle=None,
                )
                return True
            return True
        except _EmergencyStop:
            raise
        except Exception as exc:
            self.get_logger().warning(
                f"Could not run interrupted-move recovery before HOME: {exc}"
            )
            return False

    def _return_held_piece_to_source(self, context: dict) -> bool:
        if not self._context_may_hold_piece(context):
            return False

        source_key, _target_key = self._context_motion_keys(context)
        if source_key is None or source_key not in self.positions:
            return False

        self.get_logger().warning(
            "Returning held interrupted piece to its validated source slot."
        )
        source_joints = self.positions[source_key]
        source_approach = _apply_approach(source_joints)
        self._move_to(
            source_approach,
            f"{source_key}_app",
            "return_held_approach_source",
            goal_handle=None,
        )
        self._move_to(
            source_joints,
            source_key,
            "return_held_piece",
            goal_handle=None,
        )
        self._open_gripper()
        self._move_to(
            source_approach,
            f"{source_key}_app",
            "return_held_leave_source",
            goal_handle=None,
        )
        return True

    def return_held_piece_home(self, reset_stock: bool = True):
        home_joints = self.positions["home"]
        self._cancel_requested.clear()
        try:
            context = self._resume_context
            if context and not self._return_held_piece_to_source(context):
                raise RuntimeError(
                    "Cannot return held piece: interrupted source slot is unknown."
                )
            self._resume_context = None
            self._move_to(home_joints, "home", "return_home", goal_handle=None)
            if reset_stock:
                self._stock_index = {"X": 1, "O": 1}
            self._publish_status("IDLE")
        except _EmergencyStop:
            self._publish_status("EMERGENCY_STOP")
            raise
        except Exception as e:
            self.get_logger().error(f"return_held_piece_home failed: {e}")
            self._publish_status("IDLE")
            raise

    def go_home(
        self,
        reset_stock: bool = True,
        hold_piece_for_validation: bool = False,
    ):
        """Move the robot home without a PlacePiece goal."""
        home_joints = self.positions["home"]
        self._cancel_requested.clear()
        try:
            holding_for_validation = (
                hold_piece_for_validation
                and self._resume_context is not None
                and self._context_may_hold_piece(self._resume_context)
            )
            if self._recover_interrupted_context_before_home(
                hold_piece_for_validation=hold_piece_for_validation,
            ):
                # Once recovery has returned/released the piece, the old move
                # must not be replayed if HOME is interrupted afterwards.
                if not holding_for_validation:
                    self._resume_context = None
            self._move_to(home_joints, "home", "return_home", goal_handle=None)
            if holding_for_validation:
                self._resume_held_piece_from_home_next()
            if reset_stock:
                self._stock_index = {"X": 1, "O": 1}
            if not holding_for_validation:
                self._resume_context = None
            self._publish_status("IDLE")
        except _EmergencyStop:
            self._publish_status("EMERGENCY_STOP")
            raise
        except Exception as e:
            self.get_logger().error(f"go_home failed: {e}")
            self._resume_context = None
            self._publish_status("IDLE")
            raise


# ─────────────────────────────────────────────────────────────── exception

class _EmergencyStop(Exception):
    """Internal exception used to interrupt pick-and-place."""


# ─────────────────────────────────────────────────────────────── main

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    # MultiThreadedExecutor is required here. With single-threaded spin(),
    # cancel_callback cannot run while _execute_place_piece is still executing.
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
