"""
robot_controller.py
────────────────────
ROS2 node  'robot_controller'

Modos de operación
───────────────────
  simulate:=false (por defecto en lab)
    → Conecta con el action server UR3 y el servicio SetIO reales.

  simulate:=true  (sin robot)
    → Omite toda comunicación con hardware.
      Los movimientos se simulan con time.sleep().
      Publica robot_status igual que en modo real.
"""

import json
import time
import threading
from pathlib import Path

import rclpy
from rclpy.node             import Node
from rclpy.action           import ActionClient
from rclpy.callback_groups  import ReentrantCallbackGroup

from builtin_interfaces.msg import Duration
from std_msgs.msg           import String
from control_msgs.action    import FollowJointTrajectory
from trajectory_msgs.msg    import JointTrajectoryPoint
from ur_msgs.srv            import SetIO

from tictactoe_interfaces.srv import PlacePiece
from ament_index_python.packages import get_package_share_directory


# ─────────────────────────────────────────────────────────────── constants

APPROACH_OFFSET = [-0.007, +0.1311, +0.2004, -0.3277, 0.0, 0.0]
WAYPOINT_TIME   = 2.5
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

# Tiempo simulado por waypoint (más corto que el real para pruebas ágiles)
SIM_WAYPOINT_TIME = 0.5
SIM_GRIPPER_TIME  = 0.3


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

        # ── Parámetro simulate ──────────────────────────────────────────
        self.declare_parameter("simulate", False)
        self._simulate: bool = self.get_parameter("simulate").value
        mode_str = "SIMULATION 🖥️" if self._simulate else "REAL HARDWARE 🤖"
        self.get_logger().info(f"Operation Mode: {mode_str}")

        # ── Cargar posiciones ───────────────────────────────────────────
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

        # ── Contador de piezas ──────────────────────────────────────────
        self._stock_index: dict[str, int] = {"X": 1, "O": 1}

        # ── Hardware (solo si no simulamos) ────────────────────────────
        if not self._simulate:
            self._ur_client = ActionClient(
                self, FollowJointTrajectory, UR_ACTION, callback_group=cbg
            )
            self.get_logger().info("Esperando action server del UR3…")
            self._ur_client.wait_for_server()
            self.get_logger().info("✅ Action server UR3 listo.")

            self._io_client = self.create_client(
                SetIO, SETIO_SERVICE, callback_group=cbg
            )
            self.get_logger().info("Esperando servicio SetIO de la pinza…")
            self._io_client.wait_for_service()
            self.get_logger().info("✅ Servicio SetIO listo.")
        else:
            self._ur_client = None
            self._io_client = None
            self.get_logger().info(
                "⚡ Modo simulación activo — hardware omitido."
            )

        # ── Publisher de estado ─────────────────────────────────────────
        self._status_pub = self.create_publisher(String, "~/robot_status", 10)

        # ── Servicio place_piece ────────────────────────────────────────
        self._srv = self.create_service(
            PlacePiece,
            "~/place_piece",
            self._place_piece_callback,
            callback_group=cbg,
        )
        self.get_logger().info(
            "RobotController listo. Servicio ~/place_piece activo."
        )

    # ─────────────────────────────────── gripper

    def _open_gripper(self):
        if self._simulate:
            self.get_logger().info("  🤏 [SIM] Pinza: ABIERTA")
            time.sleep(SIM_GRIPPER_TIME)
            return

        req = SetIO.Request()
        req.fun   = 1
        req.pin   = 0
        req.state = 0.0
        self._io_client.call_async(req)
        self.get_logger().info("  🤏 Pinza: ABIERTA")
        time.sleep(GRIPPER_WAIT)

    def _close_gripper(self):
        if self._simulate:
            self.get_logger().info("  🤏 [SIM] Pinza: CERRADA")
            time.sleep(SIM_GRIPPER_TIME)
            return

        req = SetIO.Request()
        req.fun   = 1
        req.pin   = 0
        req.state = 1.0
        self._io_client.call_async(req)
        self.get_logger().info("  🤏 Pinza: CERRADA")
        time.sleep(GRIPPER_WAIT)

    # ─────────────────────────────────── movimiento

    def _move_to(self, joints: list[float], label: str = ""):
        self.get_logger().info(f"  → Moviendo a '{label}'…")
        self._publish_status(f"MOVING: {label}")

        if self._simulate:
            time.sleep(SIM_WAYPOINT_TIME)
            self.get_logger().info(f"  ✅ [SIM] Llegado a '{label}'")
            return

        # ── Modo real ───────────────────────────────────────────────────
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = UR3_JOINTS
        goal.trajectory.points      = [_make_point(joints, WAYPOINT_TIME)]

        goal_handle_future = self._ur_client.send_goal_async(goal)

        while not goal_handle_future.done():
            time.sleep(0.05)

        goal_handle = goal_handle_future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f"Goal rechazado para '{label}'")

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        result = result_future.result()
        if result.result.error_code != 0:
            raise RuntimeError(
                f"Trayectoria fallida hacia '{label}' "
                f"(error_code={result.result.error_code})"
            )

        self.get_logger().info(f"  ✅ Llegado a '{label}'")

    # ─────────────────────────────────── pick-and-place

    def _pick_and_place(self, symbol: str, cell_index: int):
        idx = self._stock_index[symbol]
        if idx > STOCK_COUNT:
            raise RuntimeError(
                f"¡Sin piezas para '{symbol}'! (máx {STOCK_COUNT})"
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
            f"━━━ Pick-and-place │ símbolo='{symbol}' │ "
            f"stock='{stock_key}' │ celda='{cell_key}' ━━━"
        )
        self._publish_status("BUSY")

        self._move_to(home_joints,    "home")
        self._move_to(stock_approach, f"{stock_key}_approach")
        self._open_gripper()
        self._move_to(stock_joints,   stock_key)
        self._close_gripper()
        self._move_to(stock_approach, f"{stock_key}_approach")
        self._move_to(cell_approach,  f"{cell_key}_approach")
        self._move_to(cell_joints,    cell_key)
        self._open_gripper()
        self._move_to(cell_approach,  f"{cell_key}_approach")
        self._move_to(home_joints,    "home")

        self._stock_index[symbol] += 1
        self.get_logger().info("━━━ Secuencia completada ━━━")
        self._publish_status("IDLE")

    # ─────────────────────────────────── callback servicio

    def _place_piece_callback(self, request, response):
        symbol     = request.symbol.upper()
        cell_index = request.cell_index

        self.get_logger().info(
            f"PlacePiece recibido: símbolo='{symbol}' celda={cell_index}"
        )

        def worker():
            self._publish_status("BUSY")
            try:
                self._pick_and_place(symbol, cell_index)
            except Exception as e:
                self.get_logger().error(str(e))
                self._publish_status("IDLE")

        threading.Thread(target=worker, daemon=True).start()

        response.success = True
        response.message = "Movimiento en ejecución"
        return response

    # ─────────────────────────────────── helpers

    def _publish_status(self, status: str):
        msg      = String()
        msg.data = status
        self._status_pub.publish(msg)


# ─────────────────────────────────────────────────────────────── main

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()