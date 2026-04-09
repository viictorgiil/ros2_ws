"""
launch/tictactoe.launch.py
──────────────────────────
Starts both ROS2 nodes:
  • robot_controller  — manages UR3 motion + Robotiq gripper
  • game_node         — terminal UI + Minimax AI

Usage:
    ros2 launch tictactoe_robot tictactoe.launch.py

Optional arguments:
    simulate:=true    → use simulation mode (no real robot required)

    ros2 launch tictactoe_robot tictactoe.launch.py simulate:=true
"""

from launch                            import LaunchDescription
from launch.actions                    import DeclareLaunchArgument, LogInfo
from launch.substitutions             import LaunchConfiguration
from launch_ros.actions                import Node


def generate_launch_description():

    sim_arg = DeclareLaunchArgument(
        "simulate",
        default_value="false",
        description="Run without real hardware (service call is skipped by game_node)",
    )

    robot_controller_node = Node(
        package    = "tictactoe_robot",
        executable = "robot_controller",
        name       = "robot_controller",
        output     = "screen",
        parameters = [{"simulate": LaunchConfiguration("simulate")}],
    )

    # game_node = Node(
    #     package    = "tictactoe_robot",
    #     executable = "game_node",
    #     name       = "game_node",
    #     output     = "screen",
    #     # Allows terminal I/O (stdin) to reach the node
    #     additional_env = {"PYTHONUNBUFFERED": "1"},
    #     emulate_tty    = True,
    # )

    return LaunchDescription([
        sim_arg,
        LogInfo(msg="[TicTacToe] Launching robot_controller and game_node…"),
        robot_controller_node#,
        #game_node,
    ])
