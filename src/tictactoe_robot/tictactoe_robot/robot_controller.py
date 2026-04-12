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

Parada de emergencia
─────────────────────
  El goal de PlacePiece puede cancelarse en cualquier momento via
  goal_handle.cancel_goal_async() desde game_node.py.
  Cuando se cancela:
    • Se interrumpe el movimiento actual entre waypoints.
    • La pinza se abre (suelta la pieza si la tenía).
    • Se publica robot_status = "EMERGENCY_STOP".
  Si se reanuda (nuevo goal con mismo symbol/cell), el pick-and-place
  comienza desde el principio (home → stock → cell).
  Si se reinicia, game_node envía el robot a home mediante GoHome.action
  (o simplemente un nuevo goal con symbol="" que se ignora).
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

from tictactoe_interfaces.action import PlacePiece
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

SIM_WAYPOINT_TIME = 0.5
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

        # Goal activo en ejecución (para cancelación)
        self._active_goal_handle = None
        self._cancel_requested   = threading.Event()

        self.get_logger().info(
            "RobotController listo. Action server ~/place_piece activo."
        )

    # ─────────────────────────────────────── callbacks del action server

    def _goal_callback(self, goal_request):
        """Acepta cualquier goal.
        Rechaza si hay un goal activo, EXCEPTO si ya se ha solicitado cancelación
        (en ese caso el execute está a punto de terminar y el HOME debe pasar).
        """
        if self._active_goal_handle is not None and not self._cancel_requested.is_set():
            self.get_logger().warning("Goal rechazado: ya hay un movimiento en curso.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Acepta siempre la petición de cancelación."""
        self.get_logger().info("⛔ Cancelación de goal solicitada (emergencia).")
        self._cancel_requested.set()
        return CancelResponse.ACCEPT

    def _execute_place_piece(self, goal_handle):
        """Callback principal del action server."""
        self._active_goal_handle = goal_handle
        self._cancel_requested.clear()

        symbol     = goal_handle.request.symbol.upper()
        cell_index = goal_handle.request.cell_index

        self.get_logger().info(
            f"PlacePiece goal recibido: símbolo='{symbol}' celda={cell_index}"
        )

        result = PlacePiece.Result()

        # Goal especial: volver a home (usado en reinicio tras emergencia)
        if symbol == "HOME":
            try:
                self.go_home()
            except Exception as e:
                self.get_logger().error(f"go_home falló: {e}")
            goal_handle.succeed()
            result.success = True
            result.message = "Robot en home"
            self._active_goal_handle = None
            return result

        try:
            self._pick_and_place(symbol, cell_index, goal_handle)
        except _EmergencyStop:
            self.get_logger().warn("⛔ Pick-and-place interrumpido por emergencia.")
            # Abrir pinza para soltar la pieza, pero SIN comprobar _cancel_requested
            # (ya está activo — _open_gripper lo lanzaría de nuevo).
            self._open_gripper_safe()
            self._publish_status("EMERGENCY_STOP")
            goal_handle.canceled()
            result.success = False
            result.message = "Parada de emergencia activada"
            self._active_goal_handle = None
            return result
        except Exception as e:
            self.get_logger().error(str(e))
            self._publish_status("IDLE")
            goal_handle.abort()
            result.success = False
            result.message = str(e)
            self._active_goal_handle = None
            return result

        self._publish_status("IDLE")
        goal_handle.succeed()
        result.success = True
        result.message = "Movimiento completado"
        self._active_goal_handle = None
        return result

    # ─────────────────────────────────────── gripper

    def _open_gripper_safe(self):
        """Abre la pinza sin comprobar _cancel_requested.
        Usar solo en el handler de emergencia para soltar la pieza."""
        if self._simulate:
            self.get_logger().info("  🤏 [SIM] Pinza: ABIERTA (safe)")
            time.sleep(SIM_GRIPPER_TIME)
            return
        req = SetIO.Request()
        req.fun   = 1
        req.pin   = 0
        req.state = 0.0
        self._io_client.call_async(req)
        self.get_logger().info("  🤏 Pinza: ABIERTA (safe)")
        time.sleep(GRIPPER_WAIT)

    def _open_gripper(self):
        if self._cancel_requested.is_set():
            raise _EmergencyStop()
        if self._simulate:
            self.get_logger().info("  🤏 [SIM] Pinza: ABIERTA")
            self._interruptible_sleep(SIM_GRIPPER_TIME)
            return

        req = SetIO.Request()
        req.fun   = 1
        req.pin   = 0
        req.state = 0.0
        self._io_client.call_async(req)
        self.get_logger().info("  🤏 Pinza: ABIERTA")
        self._interruptible_sleep(GRIPPER_WAIT)

    def _close_gripper(self):
        if self._cancel_requested.is_set():
            raise _EmergencyStop()
        if self._simulate:
            self.get_logger().info("  🤏 [SIM] Pinza: CERRADA")
            self._interruptible_sleep(SIM_GRIPPER_TIME)
            return

        req = SetIO.Request()
        req.fun   = 1
        req.pin   = 0
        req.state = 1.0
        self._io_client.call_async(req)
        self.get_logger().info("  🤏 Pinza: CERRADA")
        self._interruptible_sleep(GRIPPER_WAIT)

    # ─────────────────────────────────────── movimiento

    def _move_to(self, joints: list[float], label: str = "", phase: str = "",
                 goal_handle=None):
        self.get_logger().info(f"  → Moviendo a '{label}'…")
        self._publish_status(f"MOVING: {label}")

        # Publicar feedback si hay goal_handle
        if goal_handle is not None:
            fb = PlacePiece.Feedback()
            fb.phase    = phase or label
            fb.waypoint = 0
            goal_handle.publish_feedback(fb)

        if self._simulate:
            self._interruptible_sleep(SIM_WAYPOINT_TIME)
            self.get_logger().info(f"  ✅ [SIM] Llegado a '{label}'")
            return

        # ── Modo real ───────────────────────────────────────────────────
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = UR3_JOINTS
        goal.trajectory.points      = [_make_point(joints, WAYPOINT_TIME)]

        goal_handle_future = self._ur_client.send_goal_async(goal)

        # Esperar aceptación comprobando cancelación
        while not goal_handle_future.done():
            if self._cancel_requested.is_set():
                raise _EmergencyStop()
            time.sleep(0.05)

        ur_goal_handle = goal_handle_future.result()
        if not ur_goal_handle.accepted:
            raise RuntimeError(f"Goal rechazado para '{label}'")

        result_future = ur_goal_handle.get_result_async()

        # Esperar resultado comprobando cancelación
        while not result_future.done():
            if self._cancel_requested.is_set():
                # Cancelar también el goal del UR3
                ur_goal_handle.cancel_goal_async()
                raise _EmergencyStop()
            time.sleep(0.05)

        result = result_future.result()
        if result.result.error_code != 0:
            raise RuntimeError(
                f"Trayectoria fallida hacia '{label}' "
                f"(error_code={result.result.error_code})"
            )

        self.get_logger().info(f"  ✅ Llegado a '{label}'")

    # ─────────────────────────────────────── pick-and-place

    def _pick_and_place(self, symbol: str, cell_index: int, goal_handle):
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

        # Cada _move_to comprueba _cancel_requested internamente
        self._move_to(home_joints,    "home",             "return_home",   goal_handle)
        self._check_cancel()
        self._move_to(stock_approach, f"{stock_key}_app", "approach_stock", goal_handle)
        self._check_cancel()
        self._open_gripper()
        self._check_cancel()   # punto de cancelación entre movimientos
        self._move_to(stock_joints,   stock_key,          "pick",           goal_handle)
        self._check_cancel()
        self._close_gripper()
        self._check_cancel()
        self._move_to(stock_approach, f"{stock_key}_app", "approach_stock", goal_handle)
        self._check_cancel()
        self._move_to(cell_approach,  f"{cell_key}_app",  "approach_cell",  goal_handle)
        self._check_cancel()
        self._move_to(cell_joints,    cell_key,           "place",          goal_handle)
        self._check_cancel()
        self._open_gripper()
        self._check_cancel()
        self._move_to(cell_approach,  f"{cell_key}_app",  "approach_cell",  goal_handle)
        self._check_cancel()
        self._move_to(home_joints,    "home",             "return_home",    goal_handle)

        self._stock_index[symbol] += 1
        self.get_logger().info("━━━ Secuencia completada ━━━")

    # ─────────────────────────────────────── helpers

    def _check_cancel(self):
        """Lanza _EmergencyStop si se ha solicitado cancelación."""
        if self._cancel_requested.is_set():
            raise _EmergencyStop()

    def _interruptible_sleep(self, duration: float, step: float = 0.05):
        """sleep() que se interrumpe si llega una cancelación."""
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

    # ─────────────────────────────────────── go home (para reinicio)

    def go_home(self):
        """Mover el robot a home sin goal de PlacePiece (llamado en reinicio)."""
        home_joints = self.positions["home"]
        self._cancel_requested.clear()
        try:
            self._move_to(home_joints, "home", "return_home", goal_handle=None)
            self._publish_status("IDLE")
        except Exception as e:
            self.get_logger().error(f"go_home falló: {e}")
            self._publish_status("IDLE")


# ─────────────────────────────────────────────────────────────── excepción

class _EmergencyStop(Exception):
    """Excepción interna para interrumpir pick-and-place."""


# ─────────────────────────────────────────────────────────────── main

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    # MultiThreadedExecutor es OBLIGATORIO: con rclpy.spin (single-thread) el
    # cancel_callback nunca se despacha mientras _execute_place_piece está en
    # curso, porque el único hilo del executor está ocupado en ese callback.
    # Con MultiThreadedExecutor el cancel llega inmediatamente.
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