"""
robot_controller.py
────────────────────
ROS2 node  'robot_controller'

Responsibilities
─────────────────
1. Load positions from positions.json (joint angles in radians for UR3 CB3).
2. Expose a service  ~/place_piece  (tictactoe_interfaces/srv/PlacePiece)
   that receives (symbol, cell_index) and executes the complete sequence
   pick → carry → place → home.
3. Publish the robot’s status to ~/robot_status  (std_msgs/String)
   so that game_node knows when the movement has finished.

Movement
──────────
  Use ActionClient with FollowJointTrajectory and wait for actual confirmation
  that the robot has arrived (not an estimated time.sleep()).

Gripper (Robotiq Hand-E connected to the UR3 CB3)
────────────────────────────────────────────
  Use the /io_and_status_controller/set_io service with digital signals,
  just as in your partner’s secuencia_test.py.
  pin=0, state=1.0 → close | state=0.0 → open

Approach offset
──────────────────
  Use APPROACH_OFFSET from position_manager.py (6 joints calibrated with the
  actual robot), rather than a simple delta at the elbow joint.

Pick-and-place sequence by robot movement:
───────────────────────────────────────────────────
  1.  HOME
  2.  Approach to stock    (pick_stock_N + APPROACH_OFFSET)
  3.  Open gripper
  4.  Move down to stock        (exact pick_stock_N)
  5.  Close gripper          → pick up the part
  6.  Stock approach    (pick_stock_N + APPROACH_OFFSET)
  7.  Cell approach  (cell_N + APPROACH_OFFSET)
  8.  Lower to cell      (cell_N exact)
  9.  Open gripper           → releases the
"""

import json
import time
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
from pathlib import Path

import threading


# ─────────────────────────────────────────────────────────────── constants

# Offset de approach: igual que en position_manager.py, calibrado con el robot real.
# Modifícalo aquí si recalibras el approach.
APPROACH_OFFSET = [-0.007, +0.1311, +0.2004, -0.3277, 0.0, 0.0]

# Tiempo de movimiento por waypoint (segundos).
# Auméntalo si el robot va demasiado rápido entre puntos.
WAYPOINT_TIME = 2.5

# Tiempo de espera tras abrir/cerrar la pinza (segundos).
GRIPPER_WAIT = 1.0

# Número de piezas disponibles en el stock (pick_stock_1 … pick_stock_5).
STOCK_COUNT = 5

# Action server del driver UR (comprueba con: ros2 action list)
UR_ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"

# Servicio de I/O del driver UR para controlar la pinza
SETIO_SERVICE = "/io_and_status_controller/set_io"

# Orden estándar de joints del UR3 CB3
UR3_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# ─────────────────────────────────────────────────────────────── helpers

def _apply_approach(joints: list[float]) -> list[float]:
    """
    Devuelve una copia de joints con APPROACH_OFFSET sumado joint a joint.
    Equivalente a get_approach() de position_manager.py.
    """
    return [round(j + o, 7) for j, o in zip(joints, APPROACH_OFFSET)]


def _make_point(joints: list[float], t_sec: float) -> JointTrajectoryPoint:
    """Crea un JointTrajectoryPoint con tiempo relativo t_sec."""
    pt = JointTrajectoryPoint()
    pt.positions      = joints
    pt.velocities     = [0.0] * len(joints)
    pt.time_from_start = Duration(sec=int(t_sec), nanosec=int((t_sec % 1) * 1e9))
    return pt


# ─────────────────────────────────────────────────────────────── node

class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller")
        cbg = ReentrantCallbackGroup()

        # ── Cargar posiciones ───────────────────────────────────────────

        pkg_path = get_package_share_directory('tictactoe_robot')
        positions_file = Path(pkg_path) / "positions.json"

        if not positions_file.exists():
            positions_file = Path("positions.json")   # fallback: directorio de trabajo

        with open(positions_file) as f:
            self.positions: dict[str, list[float]] = json.load(f)

        self.get_logger().info(
            f"Cargadas {len(self.positions)} posiciones desde {positions_file}"
        )

        # ── Contador de piezas por símbolo ──────────────────────────────
        # Índice del siguiente pick_stock disponible (1-based, máx STOCK_COUNT)
        self._stock_index: dict[str, int] = {"X": 1, "O": 1}

        # ── Action client UR3 ───────────────────────────────────────────
        self._ur_client = ActionClient(
            self, FollowJointTrajectory, UR_ACTION, callback_group=cbg
        )
        self.get_logger().info("Esperando action server del UR3…")
        self._ur_client.wait_for_server()
        self.get_logger().info("✅ Action server UR3 listo.")

        # ── Cliente SetIO para la pinza ─────────────────────────────────
        self._io_client = self.create_client(
            SetIO, SETIO_SERVICE, callback_group=cbg
        )
        self.get_logger().info("Esperando servicio SetIO de la pinza…")
        self._io_client.wait_for_service()
        self.get_logger().info("✅ Servicio SetIO listo.")

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

    # ─────────────────────────────────── control de pinza (SetIO)

    def _open_gripper(self):
        req = SetIO.Request()
        req.fun = 1
        req.pin = 0
        req.state = 0.0

        self._io_client.call_async(req)

        self.get_logger().info("  🤏 Pinza: ABIERTA")
        time.sleep(GRIPPER_WAIT)

    def _close_gripper(self):
        req = SetIO.Request()
        req.fun = 1
        req.pin = 0
        req.state = 1.0

        self._io_client.call_async(req)

        self.get_logger().info("  🤏 Pinza: CERRADA")
        time.sleep(GRIPPER_WAIT)

    # ─────────────────────────────────── movimiento (ActionClient)

    def _move_to(self, joints: list[float], label: str = ""):

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = UR3_JOINTS
        goal.trajectory.points = [_make_point(joints, WAYPOINT_TIME)]

        self.get_logger().info(f"  → Moviendo a '{label}'…")
        self._publish_status(f"MOVING: {label}")

        # Enviar goal (NO spin)
        goal_handle_future = self._ur_client.send_goal_async(goal)

        while not goal_handle_future.done():
            time.sleep(0.05)

        goal_handle = goal_handle_future.result()

        if not goal_handle.accepted:
            raise RuntimeError(f"Goal rechazado para '{label}'")

        # Esperar resultado
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
        #self._publish_status("IDLE")

    # ─────────────────────────────────── secuencia pick-and-place

    def _pick_and_place(self, symbol: str, cell_index: int):
        """
        Secuencia completa de pick-and-place para colocar la pieza.
        Publica estado MOVING por waypoint y IDLE al final.
        """

        idx = self._stock_index[symbol]
        if idx > STOCK_COUNT:
            raise RuntimeError(
                f"¡Sin piezas para el símbolo '{symbol}'! (máx {STOCK_COUNT})"
            )

        stock_key = f"pick_stock_{idx}" if symbol == "O" else f"pick_stock_{idx}_X"
        cell_key  = f"cell_{cell_index}"

        stock_joints = self.positions[stock_key]
        cell_joints  = self.positions[cell_key]
        home_joints  = self.positions["home"]

        stock_approach = _apply_approach(stock_joints)
        cell_approach  = _apply_approach(cell_joints)

        self.get_logger().info(
            f"━━━ Pick-and-place │ símbolo='{symbol}' │ stock='{stock_key}' │ celda='{cell_key}' ━━━"
        )

        self._publish_status("BUSY")

        # 1. HOME
        self._move_to(home_joints, "home")

        # 2. Approach al stock
        self._move_to(stock_approach, f"{stock_key}_approach")

        # 3. Abrir pinza
        self._open_gripper()

        # 4. Bajar al stock
        self._move_to(stock_joints, stock_key)

        # 5. Cerrar pinza → coge la pieza
        self._close_gripper()

        # 6. Subir al approach
        self._move_to(stock_approach, f"{stock_key}_approach")

        # 7. Approach de la celda
        self._move_to(cell_approach, f"{cell_key}_approach")

        # 8. Bajar a la celda
        self._move_to(cell_joints, cell_key)

        # 9. Abrir pinza → soltar pieza
        self._open_gripper()

        # 10. Subir al approach
        self._move_to(cell_approach, f"{cell_key}_approach")

        # 11. HOME
        self._move_to(home_joints, "home")

        # Actualizar contador de piezas
        self._stock_index[symbol] += 1

        self.get_logger().info("━━━ Secuencia completada ━━━")

        self._publish_status("IDLE")
        # ─────────────────────────────────── callback del servicio

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

    # ─────────────────────────────────── helper de estado

    def _publish_status(self, status: str):
        msg = String()
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