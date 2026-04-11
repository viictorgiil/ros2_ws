from PyQt6.QtWidgets import QMainWindow, QTabWidget, QWidget, QVBoxLayout
from tictactoe_robot.gui.board_widget import BoardWidget
from tictactoe_robot.gui.setup_widget import SetupWidget


class MainWindow(QMainWindow):

    def __init__(self, bridge, node):
        super().__init__()
        self._bridge = bridge
        self._node   = node

        self.setWindowTitle("TicTacToe × UR3")
        self.setMinimumSize(900, 650)

        tabs = QTabWidget()
        self.setCentralWidget(tabs)

        # ── Pestaña 1: Juego ────────────────────────────────────────────
        game_tab     = QWidget()
        game_layout  = QVBoxLayout(game_tab)
        self._setup  = SetupWidget()
        self._board  = BoardWidget()
        game_layout.addWidget(self._setup)
        game_layout.addWidget(self._board)
        tabs.addTab(game_tab, "🎮 Juego")

        # ── Pestaña 2: Cámara (placeholder) ────────────────────────────
        camera_tab = QWidget()
        tabs.addTab(camera_tab, "📷 Visión")

        # ── Pestaña 3: Teleoperación (placeholder) ──────────────────────
        teleop_tab = QWidget()
        tabs.addTab(teleop_tab, "🕹️ Teleoperación")

        # ── Pestaña 4: Log ROS ──────────────────────────────────────────
        log_tab = QWidget()
        tabs.addTab(log_tab, "📋 Log")

        # ── Conexión de señales ─────────────────────────────────────────
        self._setup.start_requested.connect(self._on_start)
        self._board.cell_clicked.connect(node.human_move)

        bridge.move_completed.connect(self._board.on_move_completed)
        bridge.robot_status_changed.connect(self._board.on_status_changed)
        bridge.game_over.connect(self._board.on_game_over)
        bridge.ai_thinking.connect(self._board.on_ai_thinking)

    def _on_start(self, symbol: str, difficulty: float):
        self._board.reset(
            human_symbol=symbol,
            ai_symbol="O" if symbol == "X" else "X",
        )
        self._node.start_game(symbol, difficulty)