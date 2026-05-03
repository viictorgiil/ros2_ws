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
        self._resume_context: dict[str, int | str] | None = None

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

        result = PlacePiece.Result()

        # Special goal: return home after an emergency restart
        if symbol == "HOME":
            try:
                self.go_home()
            except Exception as e:
                self.get_logger().error(f"go_home failed: {e}")
            goal_handle.succeed()
            result.success = True
            result.message = "Robot at home"
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

    def _open_gripper(self):
        if self._cancel_requested.is_set():
            raise _EmergencyStop()
        if self._simulate:
            self.get_logger().info("  🤏 [SIM] Gripper: OPEN")
            self._interruptible_sleep(SIM_GRIPPER_TIME)
            return

        req = SetIO.Request()
        req.fun   = 1
        req.pin   = 0
        req.state = 0.0
        self._io_client.call_async(req)
        self.get_logger().info("  🤏 Gripper: OPEN")
        self._interruptible_sleep(GRIPPER_WAIT)

    def _close_gripper(self):
        if self._cancel_requested.is_set():
            raise _EmergencyStop()
        if self._simulate:
            self.get_logger().info("  🤏 [SIM] Gripper: CLOSED")
            self._interruptible_sleep(SIM_GRIPPER_TIME)
            return

        req = SetIO.Request()
        req.fun   = 1
        req.pin   = 0
        req.state = 1.0
        self._io_client.call_async(req)
        self.get_logger().info("  🤏 Gripper: CLOSED")
        self._interruptible_sleep(GRIPPER_WAIT)

    # ─────────────────────────────────────── movement

    def _move_to(self, joints: list[float], label: str = "", phase: str = "",
                 goal_handle=None, fast: bool = False):
        self.get_logger().info(f"  → Moving to '{label}'…")
        self._publish_status(f"MOVING: {label}")
        waypoint_time = FAST_WAYPOINT_TIME if fast else WAYPOINT_TIME
        sim_waypoint_time = FAST_SIM_WAYPOINT_TIME if fast else SIM_WAYPOINT_TIME

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
        while not result_future.done():
            if self._cancel_requested.is_set():
                # Also cancel the UR3 goal
                ur_goal_handle.cancel_goal_async()
                raise _EmergencyStop()
            time.sleep(0.05)

        result = result_future.result()
        if result.result.error_code != 0:
            raise RuntimeError(
                f"Trayectoria fallida hacia '{label}' "
                f"(error_code={result.result.error_code})"
            )

        self.get_logger().info(f"  ✅ Reached '{label}'")

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
            self._open_gripper,
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
                "next_step": step_index,
            }
            self._check_cancel()
            steps[step_index]()

        self._resume_context = None

        self._stock_index[symbol] += 1
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
            self._open_gripper,
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

    def _publish_status(self, status: str):
        msg      = String()
        msg.data = status
        self._status_pub.publish(msg)

    # ─────────────────────────────────────── go home (for restart)

    def _context_motion_keys(self, context: dict) -> tuple[str | None, str | None]:
        mode = context.get("mode")
        if mode == "place_piece":
            symbol = str(context.get("symbol", "")).upper()
            cell_index = int(context.get("cell_index", -1))
            stock_index = self._stock_index.get(symbol, 1)
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
        try:
            next_step = int(context.get("next_step", 0))
        except (TypeError, ValueError):
            return False

        # Step 4 closes the gripper; step 8 opens it at the target. If HOME is
        # requested in this interval, the safest recovery is to finish placing.
        return 4 <= next_step <= 8

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
        if 6 <= next_step <= 9:
            return target_key
        return None

    def _recover_interrupted_context_before_home(self):
        context = self._resume_context
        if not context:
            return

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

            if holding_piece and target_key is not None and target_key in self.positions:
                self.get_logger().warning(
                    "HOME requested while the gripper may hold a piece; "
                    "placing it at the planned target before going home."
                )
                target_joints = self.positions[target_key]
                target_approach = _apply_approach(target_joints)
                self._move_to(
                    target_approach,
                    f"{target_key}_app",
                    "recover_approach_target",
                    goal_handle=None,
                )
                self._move_to(
                    target_joints,
                    target_key,
                    "recover_place",
                    goal_handle=None,
                )
                self._open_gripper()
                self._move_to(
                    target_approach,
                    f"{target_key}_app",
                    "recover_leave_target",
                    goal_handle=None,
                )
        except Exception as exc:
            self.get_logger().warning(
                f"Could not run interrupted-move recovery before HOME: {exc}"
            )

    def go_home(self):
        """Move the robot home without a PlacePiece goal."""
        home_joints = self.positions["home"]
        self._cancel_requested.clear()
        try:
            self._recover_interrupted_context_before_home()
            self._move_to(home_joints, "home", "return_home", goal_handle=None)
            self._stock_index = {"X": 1, "O": 1}
            self._resume_context = None
            self._publish_status("IDLE")
        except Exception as e:
            self.get_logger().error(f"go_home failed: {e}")
            self._resume_context = None
            self._publish_status("IDLE")


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
