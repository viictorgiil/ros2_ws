"""
gui/main_window.py
────────────────────
Flujo de arranque:
  1. SetupDialog (modal).
  2. Al confirmar → MainWindow con partida activa.

Parada de emergencia:
  • board.emergency_requested → node.emergency_stop()
  • bridge.emergency_confirmed → _on_emergency_confirmed() → EmergencyDialog
  • EmergencyDialog.resume  → node (el action server reanuda el goal)
                              el robot retoma desde el principio del
                              pick-and-place actual (comportamiento del servidor)
  • EmergencyDialog.restart → node.go_home_and_reset()
                              bridge.reset_completed → cerrar MainWindow
                              y relanzar SetupDialog
"""

from PyQt6.QtWidgets import (
    QMainWindow, QTabWidget, QWidget, QVBoxLayout,
    QDialog, QLabel, QPushButton, QHBoxLayout, QFrame,
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui  import QFont

from tictactoe_robot.gui.board_widget import BoardWidget
from tictactoe_robot.gui.setup_widget import SetupDialog


# ─────────────────────────────────────── paleta local

_BG      = "#11111b"
_SURFACE = "#1e1e2e"
_BORDER  = "#313244"
_TEXT    = "#cdd6f4"
_SUBTEXT = "#6c7086"
_RED     = "#f38ba8"
_GREEN   = "#a6e3a1"
_YELLOW  = "#f9e2af"


# ─────────────────────────────────────── EmergencyDialog

class EmergencyDialog(QDialog):
    """
    Diálogo modal que aparece tras la parada de emergencia.
    Opciones:
      • Reanudar  → el robot continúa el pick-and-place desde el inicio.
      • Reiniciar → el robot va a home y se vuelve al SetupDialog.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("⛔ Parada de emergencia")
        self.setModal(True)
        self.setMinimumWidth(440)
        self.setStyleSheet(f"background: {_BG};")

        root = QVBoxLayout(self)
        root.setContentsMargins(28, 24, 28, 24)
        root.setSpacing(20)

        # ── Icono + título ──────────────────────────────────────────────
        icon_lbl = QLabel("⛔")
        icon_lbl.setFont(QFont("JetBrains Mono", 36))
        icon_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        icon_lbl.setStyleSheet("border: none;")
        root.addWidget(icon_lbl)

        title = QLabel("PARADA DE EMERGENCIA")
        title.setFont(QFont("JetBrains Mono", 14, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet(f"color: {_RED}; border: none;")
        root.addWidget(title)

        # ── Mensaje ─────────────────────────────────────────────────────
        msg = QLabel(
            "El botón de emergencia fue pulsado.\n"
            "El robot ha detenido su movimiento.\n\n"
            "¿Qué deseas hacer?"
        )
        msg.setFont(QFont("JetBrains Mono", 10))
        msg.setAlignment(Qt.AlignmentFlag.AlignCenter)
        msg.setStyleSheet(f"color: {_TEXT}; border: none;")
        root.addWidget(msg)

        # ── Separador ───────────────────────────────────────────────────
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet(f"color: {_BORDER};")
        root.addWidget(line)

        # ── Botones ─────────────────────────────────────────────────────
        btn_row = QHBoxLayout()
        btn_row.setSpacing(12)

        self._btn_resume = QPushButton("▶  Reanudar")
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

        self._btn_restart = QPushButton("↺  Reiniciar partida")
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

        # ── Nota informativa ────────────────────────────────────────────
        note = QLabel(
            "Reanudar: el robot retoma el movimiento desde el inicio del paso actual.\n"
            "Reiniciar: el robot vuelve a home y se reconfigura la partida."
        )
        note.setFont(QFont("JetBrains Mono", 8))
        note.setAlignment(Qt.AlignmentFlag.AlignCenter)
        note.setStyleSheet(f"color: {_SUBTEXT}; border: none;")
        root.addWidget(note)

        # ── Resultado: "resume" o "restart" ────────────────────────────
        self.choice: str = ""

        self._btn_resume.clicked.connect(self._on_resume)
        self._btn_restart.clicked.connect(self._on_restart)

    def _on_resume(self):
        self.choice = "resume"
        self.accept()

    def _on_restart(self):
        self.choice = "restart"
        self.accept()


# ─────────────────────────────────────── MainWindow

class MainWindow(QMainWindow):

    def __init__(self, bridge, node):
        super().__init__()
        self._bridge = bridge
        self._node   = node

        self._human_symbol : str  = "X"
        self._ai_symbol    : str  = "O"
        self._teleop       : bool = False

        self.setWindowTitle("TicTacToe × UR3")
        self.setMinimumSize(560, 820)

        # ── Pestañas ───────────────────────────────────────────────────
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

        # ── Pestaña Juego ──────────────────────────────────────────────
        self._board = BoardWidget()
        self._tabs.addTab(self._board, "🎮  Juego")

        # ── Pestaña Teleoperación (placeholder) ────────────────────────
        teleop_tab = QWidget()
        teleop_tab.setStyleSheet("background: #11111b;")
        self._tabs.addTab(teleop_tab, "🕹️  Teleoperación")

        # ── Pestaña Log (placeholder) ──────────────────────────────────
        log_tab = QWidget()
        log_tab.setStyleSheet("background: #11111b;")
        self._tabs.addTab(log_tab, "📋  Log")

        # ── Señales del bridge ─────────────────────────────────────────
        bridge.move_completed.connect(self._on_move_completed)
        bridge.robot_status_changed.connect(self._board.on_status_changed)
        bridge.game_over.connect(self._board.on_game_over)
        bridge.ai_thinking.connect(self._board.on_ai_thinking)
        bridge.human_turn_started.connect(self._board.on_human_turn_started)
        bridge.robot_placing_human.connect(self._board.on_robot_placing_human)
        bridge.emergency_confirmed.connect(self._on_emergency_confirmed)
        # Nota: reset_completed lo gestiona main() directamente para mantener
        # la referencia a la ventana viva. MainWindow solo se desconecta y cierra.

        # ── Señales del board ──────────────────────────────────────────
        self._board.move_confirmed.connect(node.human_move)
        self._board.emergency_requested.connect(node.emergency_stop)

    # ── API pública ────────────────────────────────────────────────────

    def start_new_game(self, symbol: str, difficulty: float, teleop: bool = False):
        self._human_symbol = symbol
        self._ai_symbol    = "O" if symbol == "X" else "X"
        self._teleop       = teleop

        self._board.reset(
            human_symbol=self._human_symbol,
            ai_symbol=self._ai_symbol,
            teleop=teleop,
        )
        self._node.start_game(symbol, difficulty, teleop)

    # ── Slots internos ─────────────────────────────────────────────────

    def _on_move_completed(self, cell_index: int):
        """Determina el símbolo del movimiento completado y lo dibuja."""
        cell_btn = self._board._cells[cell_index]
        if cell_btn.is_empty:
            self._board.on_move_completed(cell_index, self._ai_symbol)

    def _on_emergency_confirmed(self):
        """
        Emitido INMEDIATAMENTE al pulsar STOP (no espera a ROS2).
        Muestra el diálogo y actúa según la elección del operador.
        Se ejecuta en el hilo Qt gracias al mecanismo signal/slot.
        """
        dlg = EmergencyDialog(self)
        dlg.exec()

        if dlg.choice == "resume":
            self._do_resume()
        elif dlg.choice == "restart":
            self._do_restart()

    def _do_resume(self):
        """
        Reanudar: GameNode sabe si el movimiento terminó o no
        (_move_was_completed) y actúa en consecuencia internamente.
        """
        # Restaurar botón STOP al estado normal
        self._board._status_bar._stop_btn.setText("⛔  STOP")
        self._board._status_bar.show_stop_button(visible=True, enabled=True)
        self._node.resume_after_emergency()

    def _do_restart(self):
        """
        Reiniciar: robot a home → cerrar esta ventana → abrir SetupDialog.
        El robot se moverá a home; cuando llegue, reset_completed se emite.
        """
        self._board._status_bar.set_turn("🏠 Volviendo a home…", _YELLOW)
        self._node.go_home_and_reset()

    def _disconnect_bridge(self):
        """Desconecta todas las señales del bridge de esta ventana."""
        try:
            self._bridge.move_completed.disconnect(self._on_move_completed)
            self._bridge.robot_status_changed.disconnect(self._board.on_status_changed)
            self._bridge.game_over.disconnect(self._board.on_game_over)
            self._bridge.ai_thinking.disconnect(self._board.on_ai_thinking)
            self._bridge.human_turn_started.disconnect(self._board.on_human_turn_started)
            self._bridge.robot_placing_human.disconnect(self._board.on_robot_placing_human)
            self._bridge.emergency_confirmed.disconnect(self._on_emergency_confirmed)
            # reset_completed no se conectó aquí — lo gestiona main()
        except RuntimeError:
            pass

    def _on_reset_completed(self):
        """No-op: main() gestiona el ciclo de vida de ventanas en reset."""
        pass


# ─────────────────────────────────────── launch_app

def launch_app(bridge, node):
    """
    Muestra SetupDialog; si se acepta, crea y muestra MainWindow.
    Retorna la MainWindow o None si el usuario cerró el diálogo.
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

    window = MainWindow(bridge, node)
    window.show()
    window.start_new_game(result_symbol, result_difficulty, result_teleop)
    return window