"""
gui/main_window.py
────────────────────
Startup flow:
  1. SetupDialog (modal).
  2. After confirmation → MainWindow with an active game.

Emergency stop:
  • board.emergency_requested → node.emergency_stop()
  • bridge.emergency_confirmed → _on_emergency_confirmed() → EmergencyDialog
  • EmergencyDialog.resume  → node.resume_after_emergency()
  • EmergencyDialog.restart → node.go_home_and_reset()
                              bridge.reset_completed closes MainWindow
                              and relaunches SetupDialog
"""

import math

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout,
    QDialog, QLabel, QPushButton, QHBoxLayout, QFrame,
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui  import QFont, QColor, QPainter, QPen, QBrush

from tictactoe_robot.gui.board_widget import BoardWidget, CameraPlaceholder
from tictactoe_robot.gui.setup_widget import SetupDialog


# ─────────────────────────────────────── local palette

_BG      = "#11111b"
_SURFACE = "#1e1e2e"
_BORDER  = "#313244"
_TEXT    = "#cdd6f4"
_SUBTEXT = "#6c7086"
_RED     = "#f38ba8"
_GREEN   = "#a6e3a1"
_YELLOW  = "#f9e2af"
_BLUE    = "#1d4ed8"
_TOKEN_RED = "#dc2626"


# ─────────────────────────────────────── EmergencyDialog

class EmergencyDialog(QDialog):
    """
    Modal dialog shown after an emergency stop.
    Options:
      • Resume  → continue the interrupted sequence.
      • Restart → send the robot home and return to SetupDialog.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("⛔ Emergency Stop")
        self.setModal(True)
        self.setMinimumWidth(440)
        self.setWindowFlag(Qt.WindowType.WindowContextHelpButtonHint, False)
        self.setWindowFlag(Qt.WindowType.WindowCloseButtonHint, False)
        self.setStyleSheet(f"background: {_BG};")

        root = QVBoxLayout(self)
        root.setContentsMargins(28, 24, 28, 24)
        root.setSpacing(20)

        # ── icon + title ───────────────────────────────────────────────
        icon_lbl = QLabel("⛔")
        icon_lbl.setFont(QFont("JetBrains Mono", 36))
        icon_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        icon_lbl.setStyleSheet("border: none;")
        root.addWidget(icon_lbl)

        title = QLabel("EMERGENCY STOP")
        title.setFont(QFont("JetBrains Mono", 14, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet(f"color: {_RED}; border: none;")
        root.addWidget(title)

        # ── message ────────────────────────────────────────────────────
        msg = QLabel(
            "The emergency button was pressed.\n"
            "The robot has stopped moving.\n\n"
            "What do you want to do?"
        )
        msg.setFont(QFont("JetBrains Mono", 10))
        msg.setAlignment(Qt.AlignmentFlag.AlignCenter)
        msg.setStyleSheet(f"color: {_TEXT}; border: none;")
        root.addWidget(msg)

        # ── separator ──────────────────────────────────────────────────
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet(f"color: {_BORDER};")
        root.addWidget(line)

        # ── buttons ────────────────────────────────────────────────────
        btn_row = QHBoxLayout()
        btn_row.setSpacing(12)

        self._btn_resume = QPushButton("▶  Resume")
        self._btn_resume.setFont(QFont("JetBrains Mono", 11, QFont.Weight.Bold))
        self._btn_resume.setMinimumHeight(48)
        self._btn_resume.setCursor(Qt.CursorShape.PointingHandCursor)
        self._btn_resume.setStyleSheet(f"""
            QPushButton {{
                background: {_GREEN}22;
                border: 2px solid {_GREEN};
                border-radius: 10px;
                color: {_GREEN};
            }}
            QPushButton:hover   {{ background: {_GREEN}44; }}
            QPushButton:pressed {{ background: {_GREEN}66; }}
        """)

        self._btn_restart = QPushButton("↺  Restart Game")
        self._btn_restart.setFont(QFont("JetBrains Mono", 11, QFont.Weight.Bold))
        self._btn_restart.setMinimumHeight(48)
        self._btn_restart.setCursor(Qt.CursorShape.PointingHandCursor)
        self._btn_restart.setStyleSheet(f"""
            QPushButton {{
                background: {_RED}22;
                border: 2px solid {_RED};
                border-radius: 10px;
                color: {_RED};
            }}
            QPushButton:hover   {{ background: {_RED}44; }}
            QPushButton:pressed {{ background: {_RED}66; }}
        """)

        btn_row.addWidget(self._btn_resume)
        btn_row.addWidget(self._btn_restart)
        root.addLayout(btn_row)

        # ── info note ──────────────────────────────────────────────────
        note = QLabel(
            "Resume: the robot continues from the step where it was interrupted.\n"
            "Restart: the robot returns home and the game is reconfigured."
        )
        note.setFont(QFont("JetBrains Mono", 8))
        note.setAlignment(Qt.AlignmentFlag.AlignCenter)
        note.setStyleSheet(f"color: {_SUBTEXT}; border: none;")
        root.addWidget(note)

        # ── result: "resume" or "restart" ─────────────────────────────
        self.choice: str = ""

        self._btn_resume.clicked.connect(self._on_resume)
        self._btn_restart.clicked.connect(self._on_restart)

    def _on_resume(self):
        self.choice = "resume"
        self.accept()

    def _on_restart(self):
        self.choice = "restart"
        self.accept()

    def reject(self):
        """Block closing via Escape or implicit dialog close paths."""
        return

    def closeEvent(self, event):
        event.ignore()


class MatchFinishedDialog(QDialog):
    """
    Modal dialog shown when the game ends.
    Options:
      • Restart → send the robot home and relaunch setup.
      • Shut Down → exit the Qt app and let ROS 2 shut down cleanly.
    """

    def __init__(self, result: str, human_symbol: str, parent=None):
        super().__init__(parent)
        self.choice: str = ""
        self.setWindowTitle("Match Finished")
        self.setModal(True)
        self.setMinimumWidth(460)
        self.setWindowFlag(Qt.WindowType.WindowContextHelpButtonHint, False)
        self.setWindowFlag(Qt.WindowType.WindowCloseButtonHint, False)
        self.setStyleSheet(f"background: {_BG};")

        if result == "TIE":
            icon, title_text, body_text, accent = "🤝", "Match Tied", "The board is full and the game ended in a tie.", _YELLOW
        elif result == human_symbol:
            icon, title_text, body_text, accent = "🎉", "You Win", "You beat the robot this round.", _GREEN
        else:
            icon, title_text, body_text, accent = "🤖", "Robot Wins", "The robot won this round.", _RED

        root = QVBoxLayout(self)
        root.setContentsMargins(28, 24, 28, 24)
        root.setSpacing(20)

        icon_lbl = QLabel(icon)
        icon_lbl.setFont(QFont("JetBrains Mono", 36))
        icon_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        icon_lbl.setStyleSheet("border: none;")
        root.addWidget(icon_lbl)

        title = QLabel(title_text)
        title.setFont(QFont("JetBrains Mono", 14, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet(f"color: {accent}; border: none;")
        root.addWidget(title)

        msg = QLabel(
            f"{body_text}\n\nDo you want to restart the game or shut everything down?"
        )
        msg.setFont(QFont("JetBrains Mono", 10))
        msg.setAlignment(Qt.AlignmentFlag.AlignCenter)
        msg.setStyleSheet(f"color: {_TEXT}; border: none;")
        root.addWidget(msg)

        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet(f"color: {_BORDER};")
        root.addWidget(line)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(12)

        self._btn_restart = QPushButton("↺  Restart Game")
        self._btn_restart.setFont(QFont("JetBrains Mono", 11, QFont.Weight.Bold))
        self._btn_restart.setMinimumHeight(48)
        self._btn_restart.setCursor(Qt.CursorShape.PointingHandCursor)
        self._btn_restart.setStyleSheet(f"""
            QPushButton {{
                background: {_GREEN}22;
                border: 2px solid {_GREEN};
                border-radius: 10px;
                color: {_GREEN};
            }}
            QPushButton:hover   {{ background: {_GREEN}44; }}
            QPushButton:pressed {{ background: {_GREEN}66; }}
        """)

        self._btn_shutdown = QPushButton("⏻  Shut Down")
        self._btn_shutdown.setFont(QFont("JetBrains Mono", 11, QFont.Weight.Bold))
        self._btn_shutdown.setMinimumHeight(48)
        self._btn_shutdown.setCursor(Qt.CursorShape.PointingHandCursor)
        self._btn_shutdown.setStyleSheet(f"""
            QPushButton {{
                background: {_RED}22;
                border: 2px solid {_RED};
                border-radius: 10px;
                color: {_RED};
            }}
            QPushButton:hover   {{ background: {_RED}44; }}
            QPushButton:pressed {{ background: {_RED}66; }}
        """)

        btn_row.addWidget(self._btn_restart)
        btn_row.addWidget(self._btn_shutdown)
        root.addLayout(btn_row)

        note = QLabel(
            "Restart sends the robot home and opens the setup screen.\n"
            "Shut Down exits the GUI and shuts down the ROS 2 process."
        )
        note.setFont(QFont("JetBrains Mono", 8))
        note.setAlignment(Qt.AlignmentFlag.AlignCenter)
        note.setStyleSheet(f"color: {_SUBTEXT}; border: none;")
        root.addWidget(note)

        self._btn_restart.clicked.connect(self._on_restart)
        self._btn_shutdown.clicked.connect(self._on_shutdown)

    def _on_restart(self):
        self.choice = "restart"
        self.accept()

    def _on_shutdown(self):
        self.choice = "shutdown"
        self.accept()

    def reject(self):
        return

    def closeEvent(self, event):
        event.ignore()


class TokenCoinWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._angle = 0.0
        self._token = "X"
        self.setMinimumSize(240, 240)

    def set_angle(self, angle: float):
        self._angle = angle
        self.update()

    def set_token(self, token: str):
        self._token = token
        self.update()

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        rect = self.rect().adjusted(16, 16, -16, -16)
        angle_rad = math.radians(self._angle)
        scale_x = max(0.12, abs(math.cos(angle_rad)))

        cx = rect.center().x()
        cy = rect.center().y()
        width = rect.width() * scale_x
        height = rect.height() * 0.88

        token = self._token if math.cos(angle_rad) >= 0 else (
            "O" if self._token == "X" else "X"
        )
        token_color = QColor(_BLUE if token == "X" else _TOKEN_RED)

        shadow_rect = rect.adjusted(28, rect.height() - 48, -28, 8)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(0, 0, 0, 60))
        painter.drawEllipse(shadow_rect)

        painter.save()
        painter.translate(cx, cy)
        painter.scale(scale_x, 1.0)
        painter.translate(-cx, -cy)

        coin_rect = rect.adjusted(12, 12, -12, -12)
        painter.setPen(QPen(QColor(_BORDER), 4))
        painter.setBrush(QBrush(QColor(_SURFACE)))
        painter.drawEllipse(coin_rect)

        inner_rect = coin_rect.adjusted(16, 16, -16, -16)
        painter.setPen(QPen(QColor(_YELLOW), 3))
        painter.setBrush(QBrush(QColor(250, 191, 36, 30)))
        painter.drawEllipse(inner_rect)

        if scale_x > 0.22:
            painter.setPen(QPen(token_color, 8))
            token_font = QFont("JetBrains Mono", 58, QFont.Weight.Bold)
            painter.setFont(token_font)
            painter.drawText(coin_rect, Qt.AlignmentFlag.AlignCenter, token)

        painter.restore()


class StarterDialog(QDialog):
    """
    Transitional dialog shown after setup and before the main game window.
    It animates a coin-like spinner and then reveals who starts.
    """

    def __init__(self, human_starts: bool, human_symbol: str, parent=None):
        super().__init__(parent)
        self._human_starts = human_starts
        self._human_symbol = human_symbol
        self._ai_symbol = "O" if human_symbol == "X" else "X"
        self._result_token = self._human_symbol if human_starts else self._ai_symbol
        self._angle = 0.0
        self._flip_step = 34.0
        self.setWindowTitle("Starting Player")
        self.setModal(True)
        self.setMinimumWidth(520)
        self.setWindowFlag(Qt.WindowType.WindowContextHelpButtonHint, False)
        self.setWindowFlag(Qt.WindowType.WindowCloseButtonHint, False)
        self.setStyleSheet(f"background: {_BG};")

        root = QVBoxLayout(self)
        root.setContentsMargins(32, 32, 32, 32)
        root.setSpacing(20)

        self._coin = TokenCoinWidget()
        self._coin.set_token(self._result_token)
        root.addWidget(self._coin, alignment=Qt.AlignmentFlag.AlignCenter)

        self._title_lbl = QLabel("Deciding who starts...")
        self._title_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._title_lbl.setFont(QFont("JetBrains Mono", 16, QFont.Weight.Bold))
        self._title_lbl.setStyleSheet(f"color: {_TEXT}; border: none;")
        root.addWidget(self._title_lbl)

        self._detail_lbl = QLabel("Flipping the start token...")
        self._detail_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._detail_lbl.setFont(QFont("JetBrains Mono", 10))
        self._detail_lbl.setStyleSheet(f"color: {_SUBTEXT}; border: none;")
        root.addWidget(self._detail_lbl)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._advance_flip)

        QTimer.singleShot(120, self._start_animation)

    def _start_animation(self):
        self._timer.start(28)
        QTimer.singleShot(2600, self._show_result)
        QTimer.singleShot(6000, self.accept)

    def _advance_flip(self):
        self._angle = (self._angle + self._flip_step) % 360.0
        self._flip_step = max(8.0, self._flip_step * 0.97)
        self._coin.set_angle(self._angle)

    def _show_result(self):
        self._timer.stop()
        self._coin.set_angle(0.0)
        self._coin.set_token(self._result_token)
        if self._human_starts:
            self._title_lbl.setText("You start")
            self._detail_lbl.setText(
                f"You play {self._human_symbol}, so you make the first move."
            )
        else:
            self._title_lbl.setText("Robot starts")
            self._detail_lbl.setText(
                f"The robot plays {self._ai_symbol} and makes the first move."
            )

    def reject(self):
        return

    def closeEvent(self, event):
        event.ignore()


# ─────────────────────────────────────── MainWindow

class MainWindow(QMainWindow):

    def __init__(self, bridge, node):
        super().__init__()
        self._bridge = bridge
        self._node   = node

        self._human_symbol : str  = "X"
        self._ai_symbol    : str  = "O"
        self._teleop       : bool = False
        self._match_finished_dialog_open = False

        self.setWindowTitle("TicTacToe × UR3")
        self.setMinimumSize(560, 820)

        # ── tabs ───────────────────────────────────────────────────────
        self._tabs = QTabWidget()
        self._tabs.setStyleSheet("""
            QTabWidget::pane { border: none; background: #11111b; }
            QTabBar::tab {
                background: #1e1e2e; color: #6c7086;
                padding: 8px 20px;
                font-family: 'JetBrains Mono'; font-size: 10px;
                border-bottom: 2px solid transparent;
            }
            QTabBar::tab:selected {
                color: #cdd6f4; border-bottom: 2px solid #89b4fa;
                background: #11111b;
            }
            QTabBar::tab:hover { color: #cdd6f4; background: #181825; }
        """)
        self.setCentralWidget(self._tabs)

        # ── game tab ───────────────────────────────────────────────────
        self._board = BoardWidget()
        self._tabs.addTab(self._board, "🎮  Game")

        # ── teleoperation tab (placeholder) ───────────────────────────
        teleop_tab = QWidget()
        teleop_tab.setStyleSheet("background: #11111b;")
        teleop_layout = QVBoxLayout(teleop_tab)
        teleop_layout.setContentsMargins(16, 12, 16, 12)
        teleop_layout.setSpacing(10)

        teleop_header = QLabel("// ORIGINAL VIEW")
        teleop_header.setFont(QFont("JetBrains Mono", 8, QFont.Weight.Bold))
        teleop_header.setStyleSheet(f"color: {_SUBTEXT};")
        teleop_layout.addWidget(teleop_header)

        self._teleop_camera = CameraPlaceholder()
        teleop_layout.addWidget(self._teleop_camera)
        self._tabs.addTab(teleop_tab, "🕹️  Teleoperation")

        # ── log tab (placeholder) ─────────────────────────────────────
        log_tab = QWidget()
        log_tab.setStyleSheet("background: #11111b;")
        self._tabs.addTab(log_tab, "📋  Log")

        # ── bridge signals ─────────────────────────────────────────────
        bridge.move_completed.connect(self._on_move_completed)
        bridge.robot_status_changed.connect(self._board.on_status_changed)
        bridge.game_over.connect(self._board.on_game_over)
        bridge.game_over.connect(self._on_game_over)
        bridge.ai_thinking.connect(self._board.on_ai_thinking)
        bridge.human_turn_started.connect(self._board.on_human_turn_started)
        bridge.robot_placing_human.connect(self._board.on_robot_placing_human)
        bridge.emergency_confirmed.connect(self._on_emergency_confirmed)
        bridge.original_frame_ready.connect(self._on_original_frame)
        bridge.rectified_frame_ready.connect(self._on_rectified_frame)
        # reset_completed is handled in main() to keep the active window alive.

        # ── board signals ──────────────────────────────────────────────
        self._board.move_confirmed.connect(node.human_move)
        self._board.emergency_requested.connect(node.emergency_stop)

    # ── public API ─────────────────────────────────────────────────────

    def start_new_game(
        self,
        symbol: str,
        difficulty: float,
        teleop: bool = False,
        ai_starts: bool | None = None,
    ):
        self._human_symbol = symbol
        self._ai_symbol    = "O" if symbol == "X" else "X"
        self._teleop       = teleop

        self._board.reset(
            human_symbol=self._human_symbol,
            ai_symbol=self._ai_symbol,
            teleop=teleop,
        )
        self._node.start_game(symbol, difficulty, teleop, ai_starts)

    def _on_original_frame(self, image):
        self._teleop_camera.update_frame(image)

    def _on_rectified_frame(self, image):
        self._board.camera.update_frame(image)

    # ── internal slots ─────────────────────────────────────────────────

    def _on_move_completed(self, cell_index: int):
        """Determine the completed move symbol and draw it."""
        cell_btn = self._board._cells[cell_index]
        if cell_btn.is_empty:
            self._board.on_move_completed(cell_index, self._ai_symbol)

    def _on_emergency_confirmed(self):
        """
        Emitted immediately when STOP is pressed.
        Shows the dialog and acts on the operator's choice.
        Runs on the Qt thread through the signal/slot mechanism.
        """
        dlg = EmergencyDialog(self)
        dlg.exec()

        if dlg.choice == "resume":
            self._do_resume()
        elif dlg.choice == "restart":
            self._do_restart()

    def _do_resume(self):
        """
        Resume according to GameNode's stored interrupted-move state.
        """
        self._node.resume_after_emergency()

    def _do_restart(self):
        """
        Restart: send the robot home, close this window, and reopen SetupDialog.
        """
        self._board._status_bar.set_turn("🏠 Returning home...", _YELLOW)
        self._node.go_home_and_reset()

    def _do_shutdown(self):
        app = QApplication.instance()
        if app is not None:
            app.closeAllWindows()
            app.quit()

    def _on_game_over(self, result: str):
        if self._match_finished_dialog_open:
            return
        self._match_finished_dialog_open = True

        dlg = MatchFinishedDialog(result, self._human_symbol, self)
        dlg.exec()
        self._match_finished_dialog_open = False

        if dlg.choice == "restart":
            self._do_restart()
        elif dlg.choice == "shutdown":
            self._do_shutdown()

    def _disconnect_bridge(self):
        """Disconnect all bridge signals from this window."""
        try:
            self._bridge.move_completed.disconnect(self._on_move_completed)
            self._bridge.robot_status_changed.disconnect(self._board.on_status_changed)
            self._bridge.game_over.disconnect(self._board.on_game_over)
            self._bridge.ai_thinking.disconnect(self._board.on_ai_thinking)
            self._bridge.human_turn_started.disconnect(self._board.on_human_turn_started)
            self._bridge.robot_placing_human.disconnect(self._board.on_robot_placing_human)
            self._bridge.emergency_confirmed.disconnect(self._on_emergency_confirmed)
            self._bridge.original_frame_ready.disconnect(self._on_original_frame)
            self._bridge.rectified_frame_ready.disconnect(self._on_rectified_frame)
            # reset_completed is handled in main()
        except RuntimeError:
            pass

    def _on_reset_completed(self):
        """No-op: main() handles reset window lifecycle."""
        pass


# ─────────────────────────────────────── launch_app

def launch_app(bridge, node):
    """
    Show SetupDialog; if accepted, create and show MainWindow.
    Return MainWindow or None if the user closed the dialog.
    """
    setup = SetupDialog()

    result_symbol     : str   = "X"
    result_difficulty : float = 0.0
    result_teleop     : bool  = False

    def _capture(symbol: str, difficulty: float, teleop: bool):
        nonlocal result_symbol, result_difficulty, result_teleop
        result_symbol     = symbol
        result_difficulty = difficulty
        result_teleop     = teleop

    setup.start_requested.connect(_capture)
    accepted = setup.exec()

    if not accepted:
        return None

    import random

    ai_starts = random.choice([True, False])
    starter_dialog = StarterDialog(
        human_starts=not ai_starts,
        human_symbol=result_symbol,
    )
    starter_dialog.exec()

    window = MainWindow(bridge, node)
    window.show()
    window.start_new_game(
        result_symbol,
        result_difficulty,
        result_teleop,
        ai_starts=ai_starts,
    )
    return window
