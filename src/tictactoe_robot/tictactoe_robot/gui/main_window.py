"""
gui/main_window.py
────────────────────
Flujo de arranque:
  1. Se muestra SetupDialog (modal, ventana propia).
  2. Al pulsar Start → SetupDialog se cierra y se abre MainWindow.
  3. MainWindow contiene las pestañas: Juego, Teleoperación, Log.
     (La cámara está integrada dentro de la pestaña Juego.)

Cambios respecto a la versión original:
  • SetupDialog separado del tablero.
  • move_completed conectado a un wrapper que sabe qué símbolo usar.
  • Señal move_confirmed de BoardWidget → GameNode.human_move().
  • robot_status_changed → board.on_status_changed() (gestiona turno humano).
"""

from PyQt6.QtWidgets import QMainWindow, QTabWidget, QWidget, QVBoxLayout
from PyQt6.QtCore    import Qt

from tictactoe_robot.gui.board_widget  import BoardWidget
from tictactoe_robot.gui.setup_widget  import SetupDialog


class MainWindow(QMainWindow):

    def __init__(self, bridge, node):
        super().__init__()
        self._bridge = bridge
        self._node   = node

        # Estado de la partida activa
        self._human_symbol : str = "X"
        self._ai_symbol    : str = "O"

        self.setWindowTitle("TicTacToe × UR3")
        self.setMinimumSize(560, 820)

        # ── Pestañas ───────────────────────────────────────────────────
        self._tabs = QTabWidget()
        self._tabs.setStyleSheet("""
            QTabWidget::pane {
                border: none;
                background: #11111b;
            }
            QTabBar::tab {
                background: #1e1e2e;
                color: #6c7086;
                padding: 8px 20px;
                font-family: 'JetBrains Mono';
                font-size: 10px;
                border-bottom: 2px solid transparent;
            }
            QTabBar::tab:selected {
                color: #cdd6f4;
                border-bottom: 2px solid #89b4fa;
                background: #11111b;
            }
            QTabBar::tab:hover {
                color: #cdd6f4;
                background: #181825;
            }
        """)
        self.setCentralWidget(self._tabs)

        # ── Pestaña 1: Juego (tablero + cámara integrada) ──────────────
        self._board = BoardWidget()
        self._tabs.addTab(self._board, "🎮  Juego")

        # ── Pestaña 2: Teleoperación (placeholder) ─────────────────────
        teleop_tab = QWidget()
        teleop_tab.setStyleSheet("background: #11111b;")
        self._tabs.addTab(teleop_tab, "🕹️  Teleoperación")

        # ── Pestaña 3: Log ROS (placeholder) ───────────────────────────
        log_tab = QWidget()
        log_tab.setStyleSheet("background: #11111b;")
        self._tabs.addTab(log_tab, "📋  Log")

        # ── Conexión de señales del bridge ─────────────────────────────
        bridge.move_completed.connect(self._on_move_completed)
        bridge.robot_status_changed.connect(self._board.on_status_changed)
        bridge.game_over.connect(self._board.on_game_over)
        bridge.ai_thinking.connect(self._board.on_ai_thinking)

        # Confirmar jugada humana → GameNode
        self._board.move_confirmed.connect(node.human_move)

    # ──────────────────────────────────────────────── API pública

    def start_new_game(self, symbol: str, difficulty: float):
        """Llamado desde game_node.main() tras cerrar el SetupDialog."""
        self._human_symbol = symbol
        self._ai_symbol    = "O" if symbol == "X" else "X"

        self._board.reset(
            human_symbol=self._human_symbol,
            ai_symbol=self._ai_symbol,
        )
        self._node.start_game(symbol, difficulty)

        # Si el nodo decide que el AI empieza, deshabilitar tablero;
        # si empieza el humano, enable_human_turn() se llamará desde
        # on_status_changed cuando llegue el primer IDLE — o lo forzamos:
        # el reset() ya deja todo deshabilitado; GameNode emitirá las
        # señales adecuadas según a quién le toca primero.

    # ──────────────────────────────────────────────── slots internos

    def _on_move_completed(self, cell_index: int):
        """
        Bridge emite move_completed(cell_index) sin el símbolo.
        Determinamos qué símbolo corresponde según el turno actual:
          • Si el tablero tiene turno humano bloqueado → acaba de jugar el humano
            (ya se fijó en _on_confirm_clicked, no hacemos nada).
          • Si el tablero tiene turno del robot → on_move_completed dibuja su símbolo.
        Usamos el estado interno de BoardWidget para decidir.
        """
        # El humano ya fijó su símbolo al confirmar; solo procesamos el del robot.
        # Como GameNode alterna turnos y el humano ya dibujó el suyo,
        # si llega move_completed con el tablero en estado "robot turno" → es el robot.
        # Para distinguirlos: GameNode emite move_completed para AMBOS jugadores.
        # Necesitamos saber quién acaba de jugar. La forma más limpia: extender
        # RosSignalBridge con (cell, symbol). Mientras tanto, consultamos la celda:
        #   - Si la celda ya tiene el símbolo humano fijado → fue jugada del humano.
        #   - Si está vacía → es del robot.
        cell_btn = self._board._cells[cell_index]
        if cell_btn.is_empty:
            # Jugada del robot → dibujar su símbolo
            self._board.on_move_completed(cell_index, self._ai_symbol)
        # Si ya tiene símbolo → fue jugada del humano, ya dibujada al confirmar.


# ──────────────────────────────────────────────────────────── main entry

def launch_app(bridge, node):
    """
    Llamar desde game_node.main() en lugar de construir MainWindow directamente.

    Flujo:
        1. Crea y muestra SetupDialog.
        2. Si el usuario confirma → crea MainWindow y arranca la partida.
        3. Devuelve la MainWindow (o None si el usuario cerró el diálogo).
    """
    setup = SetupDialog()

    result_symbol     : str   = "X"
    result_difficulty : float = 0.0

    def _capture(symbol: str, difficulty: float):
        nonlocal result_symbol, result_difficulty
        result_symbol     = symbol
        result_difficulty = difficulty

    setup.start_requested.connect(_capture)
    accepted = setup.exec()   # bloqueante hasta que el usuario pulsa Start o cierra

    if not accepted:
        return None

    window = MainWindow(bridge, node)
    window.show()
    window.start_new_game(result_symbol, result_difficulty)
    return window