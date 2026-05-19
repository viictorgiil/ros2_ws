# Adaptive Human–Robot Tic-Tac-Toe with ROS 2

![Python](https://img.shields.io/badge/Python-3.10-blue)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)
[![UR3 CB3](https://img.shields.io/badge/Robot-UR3%20CB3-blue)](https://cobots.se/shop/universal-robots/ur-robotarmar/ur3-robot/)

This repository contains the ROS 2 workspace developed for the thesis project **“Integrating AI, Robotic Control and Teleoperation for Adaptive Human-Robot Gaming”**.

The system implements a physical human–robot tic-tac-toe game where a human player competes against an AI-controlled robot. It combines computer vision, game intelligence, a PyQt6 graphical interface, ROS 2 communication, and UR3 robotic manipulation to create an interactive adaptive gaming scenario.

## Demo

A video demonstration of the system is available here:

[![Watch the demo](https://img.youtube.com/vi/p3MNIh-Cx2I/maxresdefault.jpg)](https://youtu.be/p3MNIh-Cx2I?si=ELkZtjjsDt4Ja0GT)

> Click the image to open the YouTube demo.

## Main Features

- **Human–robot tic-tac-toe gameplay** using a physical board and physical game pieces.
- **AI opponent** based on Minimax with Alpha-Beta pruning.
- **Adaptive difficulty**, allowing the robot to play optimally or introduce random moves depending on the selected level.
- **ROS 2 architecture** split into robot, vision, and custom interface packages.
- **UR3 robot control** through joint trajectory actions and gripper I/O.
- **Simulation mode** for testing without the physical robot.
- **Computer vision pipeline** using OpenCV, YOLO, and MediaPipe.
- **Board rectification** through marker-based homography.
- **Hand detection** to pause/avoid unsafe interaction while the user is manipulating the board.
- **Emergency stop and recovery flow**, with options to resume or restart after interruption.
- **PyQt6 GUI** for setup, game monitoring, camera views, status messages, and operator interaction.

## Repository Structure

```text
ros2_ws/
├── requirements.txt
├── materials/
│   ├── sdf_models/          # Additional SDF models used during the thesis
│   └── study_scripts/       # Scripts used for experimental studies
└── src/
    ├── tictactoe_interfaces/
    │   ├── action/
    │   │   ├── MovePiece.action
    │   │   └── PlacePiece.action
    │   └── srv/
    │       └── PlacePiece.srv
    ├── tictactoe_robot/
    │   ├── launch/
    │   │   └── tictactoe.launch.py
    │   ├── positions.json
    │   └── tictactoe_robot/
    │       ├── game_node.py
    │       ├── robot_controller.py
    │       ├── tictactoe_game.py
    │       └── gui/
    └── tictactoe_vision/
        └── tictactoe_vision/
            ├── vision.py
            ├── bridge_node.py
            ├── best2.pt
            └── hand_landmarker.task
```

## Packages

### `tictactoe_robot`

Main gameplay and robot-control package.

It includes:

- `game_node.py`: central game coordinator, GUI bridge, turn management, vision validation, emergency stop handling, and action clients for robot motion.
- `robot_controller.py`: ROS 2 action server that controls the UR3 robot and Robotiq gripper, or simulates movements when `simulate:=true`.
- `tictactoe_game.py`: pure tic-tac-toe game engine with Minimax + Alpha-Beta pruning.
- `gui/`: PyQt6 interface for game setup, board display, status monitoring, emergency handling, and restart/shutdown flow.
- `positions.json`: calibrated UR3 joint positions for board cells, stock positions, and home pose.

### `tictactoe_vision`

Computer vision package responsible for detecting the state of the physical board.

It includes:

- Marker detection for board localization.
- Homography-based board rectification.
- YOLO-based classification of board and storage cells.
- MediaPipe hand detection.
- Publication of original and rectified camera views.
- Publication of the board state as a ROS 2 topic.

### `tictactoe_interfaces`

Custom ROS 2 interfaces used by the system.

It defines:

- `PlacePiece.action`: command the robot to place an `X` or `O` in a board cell.
- `MovePiece.action`: move a physical piece between explicit board/storage slots.
- `PlacePiece.srv`: service-style interface for piece placement requests.

## System Overview

```text
             ┌────────────────────┐
             │   PyQt6 Game GUI   │
             └─────────┬──────────┘
                       │
                       ▼
             ┌────────────────────┐
             │     game_node      │
             │ Game state + AI    │
             │ Safety/recovery    │
             └───────┬─────┬──────┘
                     │     │
        Vision state │     │ Action goals
                     │     ▼
                     │ ┌────────────────────┐
                     │ │ robot_controller   │
                     │ │ UR3 + gripper      │
                     │ │ or simulation      │
                     │ └────────────────────┘
                     ▼
             ┌────────────────────┐
             │    vision_node     │
             │ OpenCV + YOLO      │
             │ MediaPipe hands    │
             └────────────────────┘
```

## Main ROS 2 Topics and Actions

### Topics

| Topic | Type | Description |
|---|---|---|
| `/tictactoe/vision/original_view` | `sensor_msgs/Image` | Original camera view. |
| `/tictactoe/vision/rectified_view` | `sensor_msgs/Image` | Rectified board view. |
| `/tictactoe/board` | `std_msgs/String` | JSON-encoded board/storage state from vision. |
| `/tictactoe/game/turn` | `std_msgs/String` | Current game/turn mode. |
| `/robot_controller/robot_status` | `std_msgs/String` | Robot state such as busy, idle, or emergency stop. |
| `/robot_controller/operation_status` | `std_msgs/String` | Detailed robot operation status. |

### Actions

| Action | Interface | Description |
|---|---|---|
| `/robot_controller/place_piece` | `tictactoe_interfaces/action/PlacePiece` | Pick and place a game piece on the board. |
| `/robot_controller/move_piece` | `tictactoe_interfaces/action/MovePiece` | Move a piece between explicit storage/board slots. |

## Requirements

### System dependencies

- ROS 2
- Python 3
- `colcon`
- A working ROS 2 environment sourced in the terminal
- UR robot ROS 2 control stack when running with the real UR3
- A compatible USB camera for the vision node

### Python dependencies

The Python dependencies are listed in `requirements.txt`:

```text
PyQt6
opencv-python
numpy<2
mediapipe
ultralytics
torch
torchvision
```

Install them with:

```bash
pip install -r requirements.txt
```

## Installation

Clone the workspace:

```bash
git clone https://github.com/viictorgiil/ros2_ws.git
cd ros2_ws
```

Install Python dependencies:

```bash
pip install -r requirements.txt
```

Install ROS dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

## Running the System

### Full system with robot and vision

```bash
ros2 launch tictactoe_robot tictactoe.launch.py
```

This starts:

- `robot_controller`
- `game_node`
- `vision_node`

### Simulation mode

Use this mode when the physical UR3 robot is not available:

```bash
ros2 launch tictactoe_robot tictactoe.launch.py simulate:=true
```

### Run without vision

Use this when testing the game/robot logic without the camera pipeline:

```bash
ros2 launch tictactoe_robot tictactoe.launch.py vision:=false
```

### Simulation without vision

```bash
ros2 launch tictactoe_robot tictactoe.launch.py simulate:=true vision:=false
```

## Vision Setup Notes

The vision node expects a camera device similar to:

```text
/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0
```

If a different camera is used, update the camera path inside `tictactoe_vision/vision.py`.

The board detection relies on green markers and a calibrated physical layout. The vision pipeline rectifies the image into three regions:

```text
storage1 | board | storage2
```

The board uses row-major indexing:

```text
0 | 1 | 2
3 | 4 | 5
6 | 7 | 8
```

## Robot Control

The robot controller supports two modes:

- `simulate:=false`: connects to the real UR3 action server and gripper I/O service.
- `simulate:=true`: skips hardware communication and simulates robot movements.

The controller uses calibrated joint positions stored in `positions.json`, including:

- `home`
- `cell_0` to `cell_8`
- `pick_stock_*`
- `pick_stock_*_X`

For real hardware usage, verify the physical board position, gripper configuration, camera calibration, and all joint targets before running the robot.

## Safety and Emergency Stop

The system includes an emergency-stop workflow:

1. The operator presses `STOP` in the GUI.
2. `game_node` cancels the active robot action goal.
3. `robot_controller` interrupts the movement and publishes an emergency state.
4. The GUI asks whether to resume or restart.
5. The system validates the physical board state before continuing when vision is enabled.

This software-level emergency stop is intended as a control-layer safety feature. It does not replace physical emergency-stop hardware or proper lab safety procedures.

## Game AI

The tic-tac-toe AI is implemented in `tictactoe_game.py`.

It uses:

- Minimax search
- Alpha-Beta pruning
- Score preference for faster wins and slower losses
- A random-move probability parameter for adaptive difficulty

The difficulty parameter controls how often the AI chooses a random valid move instead of the optimal move.

## Development Tips

Build only the custom interfaces after editing `.srv` or `.action` files:

```bash
colcon build --packages-select tictactoe_interfaces
source install/setup.bash
```

Build the full workspace after modifying Python nodes:

```bash
colcon build
source install/setup.bash
```

Inspect available ROS 2 interfaces:

```bash
ros2 interface show tictactoe_interfaces/action/PlacePiece
ros2 interface show tictactoe_interfaces/action/MovePiece
ros2 interface show tictactoe_interfaces/srv/PlacePiece
```

Monitor the board state:

```bash
ros2 topic echo /tictactoe/board
```

Monitor robot status:

```bash
ros2 topic echo /robot_controller/robot_status
```

## Materials

The `materials/` folder contains supporting thesis assets:

- `sdf_models/`: SDF files used for 3D object modelling.
- `study_scripts/`: Python scripts used for experimental studies.

## Known Notes

- The project is currently configured for a specific physical setup, including camera path, board marker layout, UR3 joint positions, and storage-cell mapping.
- Before using a different robot, board, camera, or lighting setup, recalibration is required.
- The trained YOLO model file is packaged inside `tictactoe_vision` and is required for piece classification.
- Real-hardware operation should only be performed in a controlled environment with proper supervision.

## Author

**Víctor Gil** and **Jaime Verdú**  
Thesis project repository: [viictorgiil/ros2_ws](https://github.com/viictorgiil/ros2_ws)

## License

No license has been declared yet. Add a `LICENSE` file before distributing or reusing this project publicly.
