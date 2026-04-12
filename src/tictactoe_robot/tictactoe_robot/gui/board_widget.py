"""
gui/board_widget.py
────────────────────
Widget principal de juego. Contiene:
  • RobotStatusBar (arriba) — con botón de emergencia integrado
  • Tablero 3×3 con selección provisional
  • Botón confirmar jugada
  • Feed de cámara (abajo)

Botón de emergencia (⛔ STOP):
  • Modo Normal  → visible SOLO durante el turno del robot.
  • Modo Teleop  → visible SIEMPRE (el robot se mueve en todos los turnos).
  • Al pulsarlo emite la señal emergency_requested del BoardWidget,
    que MainWindow conecta a GameNode.emergency_stop().
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QFrame, QSizePolicy,
)
from PyQt6.QtCore  import pyqtSignal, Qt
from PyQt6.QtGui   import QFont


# ─────────────────────────────────────── paleta

_BG        = "#11111b"
_SURFACE   = "#1e1e2e"
_BORDER    = "#313244"
_TEXT      = "#cdd6f4"
_SUBTEXT   = "#6c7086"
_PINK      = "#f38ba8"
_TEAL      = "#94e2d5"
_BLUE      = "#89b4fa"
_GREEN     = "#a6e3a1"
_YELLOW    = "#f9e2af"
_RED       = "#f38ba8"   # mismo tono que X pero semánticamente "peligro"
_OVERLAY   = "#45475a"

_FONT_CELL    = QFont("JetBrains Mono", 38, QFont.Weight.Bold)
_FONT_STATUS  = QFont("JetBrains Mono", 11, QFont.Weight.Bold)
_FONT_CONFIRM = QFont("JetBrains Mono", 11, QFont.Weight.Bold)
_FONT_STOP    = QFont("JetBrains Mono", 9,  QFont.Weight.Bold)


def _symbol_color(symbol: str, provisional: bool = False) -> str:
    if provisional:
        return _OVERLAY
    return _TEAL if symbol == "X" else _PINK


# ─────────────────────────────────────── CellButton

class CellButton(QPushButton):
    def __init__(self, index: int):
        super().__init__("")
        self.index        = index
        self._locked      = False
        self._symbol      = ""
        self._provisional = False
        self.setFixedSize(130, 130)
        self.setFont(_FONT_CELL)
        self._apply_style()

    def set_symbol(self, symbol: str, provisional: bool = False):
        self._symbol      = symbol
        self._provisional = provisional
        self.setText(symbol)
        self._apply_style()

    def clear(self):
        self._symbol      = ""
        self._provisional = False
        self._locked      = False
        self.setText("")
        self.setEnabled(True)
        self._apply_style()

    def lock(self):
        self._locked = True
        self.setEnabled(False)
        self._apply_style()

    @property
    def symbol(self) -> str:
        return self._symbol

    @property
    def is_empty(self) -> bool:
        return self._symbol == ""

    def _apply_style(self):
        color = _symbol_color(self._symbol, self._provisional) if self._symbol else _TEXT
        self.setStyleSheet(f"""
            QPushButton {{
                border: 2px solid {_BORDER};
                border-radius: 10px;
                background: {_SURFACE};
                color: {color};
            }}
            QPushButton:hover:enabled {{
                background: #2a2a3e;
                border-color: {_BLUE}88;
            }}
            QPushButton:disabled {{
                background: {_SURFACE};
                color: {color};
            }}
        """)


# ─────────────────────────────────────── RobotStatusBar

class RobotStatusBar(QFrame):
    """
    Barra superior con:
      izquierda → dot + "Robot: STATUS"
      centro    → texto de turno
      derecha   → botón ⛔ STOP (condicional)
    """

    emergency_requested = pyqtSignal()   # conectado a GameNode.emergency_stop()

    def __init__(self):
        super().__init__()
        self.setStyleSheet(f"""
            QFrame {{
                background: {_SURFACE};
                border: 1px solid {_BORDER};
                border-radius: 10px;
            }}
        """)
        self.setFixedHeight(52)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 0, 12, 0)
        layout.setSpacing(8)

        # ── dot + robot label ───────────────────────────────────────────
        self._dot = QLabel("●")
        self._dot.setFont(QFont("JetBrains Mono", 14))
        self._dot.setStyleSheet(f"color: {_SUBTEXT}; border: none;")
        layout.addWidget(self._dot)

        self._robot_lbl = QLabel("Robot: —")
        self._robot_lbl.setFont(_FONT_STATUS)
        self._robot_lbl.setStyleSheet(f"color: {_SUBTEXT}; border: none;")
        layout.addWidget(self._robot_lbl)

        layout.addStretch()

        # ── turno ───────────────────────────────────────────────────────
        self._turn_lbl = QLabel("")
        self._turn_lbl.setFont(_FONT_STATUS)
        self._turn_lbl.setStyleSheet(f"color: {_TEXT}; border: none;")
        layout.addWidget(self._turn_lbl)

        layout.addStretch()

        # ── botón emergencia ────────────────────────────────────────────
        self._stop_btn = QPushButton("⛔  STOP")
        self._stop_btn.setFont(_FONT_STOP)
        self._stop_btn.setFixedHeight(34)
        self._stop_btn.setMinimumWidth(80)
        self._stop_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._stop_btn.setVisible(False)   # oculto por defecto
        self._stop_btn.setStyleSheet(f"""
            QPushButton {{
                background: #f38ba822;
                border: 2px solid #f38ba8;
                border-radius: 8px;
                color: #f38ba8;
                letter-spacing: 1px;
            }}
            QPushButton:hover  {{ background: #f38ba844; }}
            QPushButton:pressed {{ background: #f38ba866; }}
            QPushButton:disabled {{
                background: {_SURFACE};
                border-color: {_BORDER};
                color: {_SUBTEXT};
            }}
        """)
        self._stop_btn.clicked.connect(self.emergency_requested)
        layout.addWidget(self._stop_btn)

    # ── API pública ─────────────────────────────────────────────────────

    def set_robot_status(self, status: str):
        status_upper = status.upper()
        if status_upper == "IDLE":
            color, dot_color = _GREEN, _GREEN
        elif status_upper == "BUSY":
            color, dot_color = _YELLOW, _YELLOW
        elif status_upper.startswith("MOVING"):
            color, dot_color = _BLUE, _BLUE
        elif status_upper == "EMERGENCY_STOP":
            color, dot_color = _RED, _RED
        else:
            color, dot_color = _SUBTEXT, _SUBTEXT

        self._dot.setStyleSheet(f"color: {dot_color}; border: none;")
        self._robot_lbl.setStyleSheet(f"color: {color}; border: none;")
        self._robot_lbl.setText(f"Robot: {status}")

    def set_turn(self, text: str, color: str = _TEXT):
        self._turn_lbl.setStyleSheet(f"color: {color}; border: none;")
        self._turn_lbl.setText(text)

    def show_stop_button(self, visible: bool, enabled: bool = True):
        """
        Controla visibilidad y habilitación del botón de emergencia.
          visible=True  → aparece
          enabled=False → aparece en gris (no pulsable, p.ej. tras pulsarlo ya)
        """
        self._stop_btn.setVisible(visible)
        self._stop_btn.setEnabled(enabled)

    def set_stop_button_pressed(self):
        """Deshabilita el botón tras pulsarlo para evitar doble clic."""
        self._stop_btn.setEnabled(False)
        self._stop_btn.setText("⏳  Parando…")


# ─────────────────────────────────────── CameraPlaceholder

class CameraPlaceholder(QFrame):
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(200)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setStyleSheet(f"""
            QFrame {{
                background: #0d0d1a;
                border: 1px solid {_BORDER};
                border-radius: 10px;
            }}
        """)
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._cam_label = QLabel()
        self._cam_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._cam_label.setStyleSheet("border: none; background: transparent;")
        self._cam_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        layout.addWidget(self._cam_label)

        self._placeholder_lbl = QLabel("📷  Cámara no conectada")
        self._placeholder_lbl.setFont(QFont("JetBrains Mono", 10))
        self._placeholder_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._placeholder_lbl.setStyleSheet(f"color: {_SUBTEXT}; border: none;")
        layout.addWidget(self._placeholder_lbl)

    def update_frame(self, pixmap):
        self._placeholder_lbl.hide()
        self._cam_label.setPixmap(
            pixmap.scaled(
                self._cam_label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )


# ─────────────────────────────────────── BoardWidget

class BoardWidget(QWidget):
    """
    Señales públicas:
        move_confirmed(cell_index)   → GameNode.human_move()
        emergency_requested()        → GameNode.emergency_stop()
    """

    move_confirmed    = pyqtSignal(int)
    emergency_requested = pyqtSignal()   # re-emite la del status bar hacia afuera

    def __init__(self):
        super().__init__()
        self.setStyleSheet(f"background: {_BG};")

        self._human_symbol  : str        = "X"
        self._ai_symbol     : str        = "O"
        self._pending_cell  : int | None = None
        self._human_turn    : bool       = False
        self._teleop        : bool       = False   # se fija en reset()
        self._cells         : list[CellButton] = []

        self._build_ui()

        # Re-emitir la señal del status bar como señal propia del widget
        self._status_bar.emergency_requested.connect(self._on_stop_pressed)

    # ─────────────────────────────────────── UI

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 12, 16, 12)
        root.setSpacing(10)

        # ── Barra de estado ───────────────────────────────────────────
        self._status_bar = RobotStatusBar()
        root.addWidget(self._status_bar)

        # ── Tablero ───────────────────────────────────────────────────
        board_frame = QFrame()
        board_frame.setStyleSheet(f"""
            QFrame {{
                background: {_SURFACE};
                border: 1px solid {_BORDER};
                border-radius: 14px;
            }}
        """)
        board_layout_outer = QVBoxLayout(board_frame)
        board_layout_outer.setContentsMargins(16, 16, 16, 16)
        board_layout_outer.setSpacing(12)

        grid = QGridLayout()
        grid.setSpacing(10)
        for i in range(9):
            btn = CellButton(i)
            btn.clicked.connect(lambda _, idx=i: self._on_cell_clicked(idx))
            self._cells.append(btn)
            grid.addWidget(btn, i // 3, i % 3)
        board_layout_outer.addLayout(grid)

        # ── Botón confirmar ───────────────────────────────────────────
        self._confirm_btn = QPushButton("✔  Confirmar jugada")
        self._confirm_btn.setFont(_FONT_CONFIRM)
        self._confirm_btn.setMinimumHeight(44)
        self._confirm_btn.setEnabled(False)
        self._confirm_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._confirm_btn.setStyleSheet(f"""
            QPushButton {{
                background: {_GREEN}18;
                border: 2px solid {_BORDER};
                border-radius: 10px;
                color: {_SUBTEXT};
                letter-spacing: 1px;
            }}
            QPushButton:enabled {{
                background: {_GREEN}28;
                border-color: {_GREEN};
                color: {_GREEN};
            }}
            QPushButton:enabled:hover   {{ background: {_GREEN}44; }}
            QPushButton:enabled:pressed {{ background: {_GREEN}66; }}
        """)
        self._confirm_btn.clicked.connect(self._on_confirm_clicked)
        board_layout_outer.addWidget(self._confirm_btn)

        root.addWidget(board_frame)

        # ── Cámara ────────────────────────────────────────────────────
        cam_header = QLabel("// VISIÓN EN TIEMPO REAL")
        cam_header.setFont(QFont("JetBrains Mono", 8, QFont.Weight.Bold))
        cam_header.setStyleSheet(f"color: {_SUBTEXT};")
        root.addWidget(cam_header)

        self._camera = CameraPlaceholder()
        root.addWidget(self._camera)

    # ─────────────────────────────────────── API pública

    def reset(self, human_symbol: str, ai_symbol: str, teleop: bool = False):
        self._human_symbol = human_symbol
        self._ai_symbol    = ai_symbol
        self._teleop       = teleop
        self._pending_cell = None
        self._human_turn   = False

        for btn in self._cells:
            btn.clear()
            btn.setEnabled(False)

        self._confirm_btn.setEnabled(False)
        self._status_bar.set_robot_status("IDLE")
        self._status_bar.set_turn("Esperando inicio…", _SUBTEXT)

        # En teleop el STOP aparece siempre; en normal arranca oculto
        self._status_bar.show_stop_button(visible=teleop, enabled=True)
        # Restaurar texto del botón por si se usó en partida anterior
        self._status_bar._stop_btn.setText("⛔  STOP")

    def enable_human_turn(self):
        self._human_turn = True
        self._pending_cell = None
        self._confirm_btn.setEnabled(False)
        for btn in self._cells:
            if btn.is_empty:
                btn.setEnabled(True)
        self._status_bar.set_turn("Tu turno ✋", _GREEN)

        # Modo Normal: ocultar STOP durante turno humano
        if not self._teleop:
            self._status_bar.show_stop_button(visible=False)

    def disable_human_turn(self):
        self._human_turn = False
        self._pending_cell = None
        self._confirm_btn.setEnabled(False)
        for btn in self._cells:
            btn.setEnabled(False)

    def on_move_completed(self, cell_index: int, symbol: str):
        btn = self._cells[cell_index]
        btn.set_symbol(symbol, provisional=False)
        btn.lock()

    def on_status_changed(self, status: str):
        self._status_bar.set_robot_status(status)

    def on_human_turn_started(self):
        self.enable_human_turn()

    def on_robot_placing_human(self):
        self._status_bar.set_turn("🦾 Robot colocando tu pieza…", "#fab387")

    def on_game_over(self, result: str):
        self.disable_human_turn()
        self._status_bar.show_stop_button(visible=False)
        if result == "TIE":
            self._status_bar.set_turn("🤝 ¡Empate!", _YELLOW)
        elif result == self._human_symbol:
            self._status_bar.set_turn("🎉 ¡Has ganado!", _GREEN)
        else:
            self._status_bar.set_turn("🤖 ¡Gana el robot!", _PINK)

    def on_ai_thinking(self, cell: int):
        self._status_bar.set_turn(f"🤔 Robot pensando… → celda {cell}", _BLUE)
        # Mostrar STOP durante el turno del robot.
        # Cubre el caso en que el robot mueve primero (el botón aún no se había
        # mostrado porque el humano todavía no había confirmado ninguna jugada).
        if not self._teleop:
            self._status_bar.show_stop_button(visible=True, enabled=True)
            self._status_bar._stop_btn.setText("⛔  STOP")

    @property
    def camera(self) -> CameraPlaceholder:
        return self._camera

    # ─────────────────────────────────────── slots internos

    def _on_cell_clicked(self, idx: int):
        if not self._human_turn:
            return

        if self._pending_cell is not None and self._pending_cell != idx:
            prev = self._cells[self._pending_cell]
            prev.set_symbol("")
            prev.clear()

        self._pending_cell = idx
        self._cells[idx].set_symbol(self._human_symbol, provisional=True)
        self._confirm_btn.setEnabled(True)
        self._status_bar.set_turn(
            f"Celda {idx} seleccionada — ¿confirmar?", _YELLOW
        )

    def _on_confirm_clicked(self):
        if self._pending_cell is None:
            return

        cell = self._pending_cell
        self._pending_cell = None

        btn = self._cells[cell]
        btn.set_symbol(self._human_symbol, provisional=False)
        btn.lock()

        self.disable_human_turn()
        self._status_bar.set_turn("Robot en movimiento…", _BLUE)

        # Modo Normal: mostrar STOP durante turno del robot
        if not self._teleop:
            self._status_bar.show_stop_button(visible=True, enabled=True)
            self._status_bar._stop_btn.setText("⛔  STOP")

        self.move_confirmed.emit(cell)

    def _on_stop_pressed(self):
        """El operador pulsó el botón de emergencia."""
        self._status_bar.set_stop_button_pressed()
        self._status_bar.set_turn("⛔ Parada de emergencia…", _RED)
        self.emergency_requested.emit()