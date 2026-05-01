"""
launch/tictactoe.launch.py
──────────────────────────
Starts both ROS2 nodes:
  • robot_controller  — manages UR3 motion + Robotiq gripper
  • game_node         — terminal UI + Minimax AI
  • bridge_node       — forwards vision board state from socket to ROS
  • vision_node       — publishes original/rectified camera views

Usage:
    ros2 launch tictactoe_robot tictactoe.launch.py

Optional arguments:
    simulate:=true    → use simulation mode (no real robot required)
    vision:=false     → skip camera/vision nodes

    ros2 launch tictactoe_robot tictactoe.launch.py simulate:=true
"""

from launch                            import LaunchDescription
from launch.actions                    import DeclareLaunchArgument, LogInfo
from launch.conditions                 import IfCondition
from launch.substitutions             import LaunchConfiguration
from launch_ros.actions                import Node


def generate_launch_description():

    sim_arg = DeclareLaunchArgument(
        "simulate",
        default_value="false",
        description="Run without real hardware (service call is skipped by game_node)",
    )
    vision_arg = DeclareLaunchArgument(
        "vision",
        default_value="true",
        description="Start vision camera publisher and vision bridge",
    )

    robot_controller_node = Node(
        package    = "tictactoe_robot",
        executable = "robot_controller",
        name       = "robot_controller",
        output     = "screen",
        parameters = [{"simulate": LaunchConfiguration("simulate")}],
    )

    game_node = Node(
        package    = "tictactoe_robot",
        executable = "game_node",
        name       = "game_node",
        output     = "screen",
        # Allows terminal I/O (stdin) to reach the node
        additional_env = {"PYTHONUNBUFFERED": "1"},
        emulate_tty    = True,
    )

    vision_bridge_node = Node(
        package    = "tictactoe_vision",
        executable = "bridge_node",
        name       = "vision_bridge",
        output     = "screen",
        condition  = IfCondition(LaunchConfiguration("vision")),
    )

    vision_node = Node(
        package    = "tictactoe_vision",
        executable = "vision_node",
        name       = "vision_node",
        output     = "screen",
        emulate_tty = True,
        condition  = IfCondition(LaunchConfiguration("vision")),
    )

    return LaunchDescription([
        sim_arg,
        vision_arg,
        LogInfo(msg="[TicTacToe] Launching robot_controller, game_node, and vision nodes..."),
        robot_controller_node,
        game_node,
        vision_bridge_node,
        vision_node,
    ])
