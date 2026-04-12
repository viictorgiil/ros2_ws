"""
gui/board_widget.py
────────────────────
Widget principal de juego. Contiene:
  • Barra de estado del robot  (arriba)
  • Tablero 3×3 con selección provisional del humano
  • Botón de confirmación de jugada
  • Feed de cámara  (abajo, placeholder sustituible por QLabel con imagen ROS)

Flujo del humano:
  1. Clic en celda vacía → se muestra el símbolo en color tenue (provisional).
     Si clica otra celda vacía, el provisional se mueve.
  2. Clic en "Confirmar jugada" → se fija la jugada y el robot empieza su turno.

Flujo del robot:
  • on_move_completed(cell, symbol) dibuja el símbolo del robot cuando el
    movimiento físico ha terminado (no cuando el algoritmo decide).
  • Cuando el robot termina (IDLE) el estado cambia a "Tu turno ✋".
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QFrame, QSizePolicy,
)
from PyQt6.QtCore  import pyqtSignal, Qt
from PyQt6.QtGui   import QFont, QColor


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
_OVERLAY   = "#45475a"   # símbolo provisional (tenue)

_FONT_CELL    = QFont("JetBrains Mono", 38, QFont.Weight.Bold)
_FONT_STATUS  = QFont("JetBrains Mono", 11, QFont.Weight.Bold)
_FONT_CONFIRM = QFont("JetBrains Mono", 11, QFont.Weight.Bold)


def _symbol_color(symbol: str, provisional: bool = False) -> str:
    if provisional:
        return _OVERLAY
    return _PINK if symbol == "X" else _TEAL


# ─────────────────────────────────────── CellButton

class CellButton(QPushButton):
    """Una de las 9 casillas del tablero."""

    def __init__(self, index: int):
        super().__init__("")
        self.index     = index
        self._locked   = False          # True cuando la jugada está confirmada
        self._symbol   = ""
        self._provisional = False

        self.setFixedSize(130, 130)
        self.setFont(_FONT_CELL)
        self._apply_style()

    # ── estado ──────────────────────────────────────

    def set_symbol(self, symbol: str, provisional: bool = False):
        """Dibuja el símbolo. provisional=True → color tenue."""
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
        """Fija la casilla: deshabilita interacción."""
        self._locked = True
        self.setEnabled(False)
        self._apply_style()

    @property
    def symbol(self) -> str:
        return self._symbol

    @property
    def is_empty(self) -> bool:
        return self._symbol == ""

    # ── estilo ──────────────────────────────────────

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
    """Barra superior con estado del robot y turno actual."""

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
        layout.setContentsMargins(16, 0, 16, 0)

        # indicador de punto de estado
        self._dot = QLabel("●")
        self._dot.setFont(QFont("JetBrains Mono", 14))
        self._dot.setStyleSheet(f"color: {_SUBTEXT}; border: none;")
        layout.addWidget(self._dot)

        # texto robot
        self._robot_lbl = QLabel("Robot: —")
        self._robot_lbl.setFont(_FONT_STATUS)
        self._robot_lbl.setStyleSheet(f"color: {_SUBTEXT}; border: none;")
        layout.addWidget(self._robot_lbl)

        layout.addStretch()

        # turno
        self._turn_lbl = QLabel("")
        self._turn_lbl.setFont(_FONT_STATUS)
        self._turn_lbl.setStyleSheet(f"color: {_TEXT}; border: none;")
        layout.addWidget(self._turn_lbl)

    def set_robot_status(self, status: str):
        status_upper = status.upper()
        if status_upper == "IDLE":
            color, dot_color = _GREEN, _GREEN
        elif status_upper == "BUSY":
            color, dot_color = _YELLOW, _YELLOW
        elif status_upper.startswith("MOVING"):
            color, dot_color = _BLUE, _BLUE
        else:
            color, dot_color = _SUBTEXT, _SUBTEXT

        self._dot.setStyleSheet(f"color: {dot_color}; border: none;")
        self._robot_lbl.setStyleSheet(f"color: {color}; border: none;")
        self._robot_lbl.setText(f"Robot: {status}")

    def set_turn(self, text: str, color: str = _TEXT):
        self._turn_lbl.setStyleSheet(f"color: {color}; border: none;")
        self._turn_lbl.setText(text)


# ─────────────────────────────────────── CameraPlaceholder

class CameraPlaceholder(QFrame):
    """
    Placeholder para el feed de cámara.
    Sustituye el QLabel.setPixmap() con la imagen ROS2 convertida.
    Para integrarlo con ROS2:
        self._cam_label.setPixmap(QPixmap.fromImage(qt_image))
    """

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
        """Llamar desde el callback de imagen ROS2."""
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
    Widget principal de juego.

    Señales:
        move_confirmed(cell_index)  → conectar a GameNode.human_move()
    """

    move_confirmed = pyqtSignal(int)   # reemplaza cell_clicked

    def __init__(self):
        super().__init__()
        self.setStyleSheet(f"background: {_BG};")

        self._human_symbol  : str       = "X"
        self._ai_symbol     : str       = "O"
        self._pending_cell  : int | None = None   # celda provisional
        self._human_turn    : bool       = False
        self._cells         : list[CellButton] = []

        self._build_ui()

    # ─────────────────────────────────────── UI

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 12, 16, 12)
        root.setSpacing(10)

        # ── Barra de estado (arriba) ──────────────────────────────
        self._status_bar = RobotStatusBar()
        root.addWidget(self._status_bar)

        # ── Tablero ───────────────────────────────────────────────
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

        # ── Botón confirmar ───────────────────────────────────────
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
            QPushButton:enabled:hover  {{ background: {_GREEN}44; }}
            QPushButton:enabled:pressed {{ background: {_GREEN}66; }}
        """)
        self._confirm_btn.clicked.connect(self._on_confirm_clicked)
        board_layout_outer.addWidget(self._confirm_btn)

        root.addWidget(board_frame)

        # ── Cámara (abajo) ────────────────────────────────────────
        cam_header = QLabel("// VISIÓN EN TIEMPO REAL")
        cam_header.setFont(QFont("JetBrains Mono", 8, QFont.Weight.Bold))
        cam_header.setStyleSheet(f"color: {_SUBTEXT};")
        root.addWidget(cam_header)

        self._camera = CameraPlaceholder()
        root.addWidget(self._camera)

    # ─────────────────────────────────────── API pública

    def reset(self, human_symbol: str, ai_symbol: str):
        self._human_symbol = human_symbol
        self._ai_symbol    = ai_symbol
        self._pending_cell = None
        self._human_turn   = False

        for btn in self._cells:
            btn.clear()
            btn.setEnabled(False)

        self._confirm_btn.setEnabled(False)
        self._status_bar.set_robot_status("IDLE")
        self._status_bar.set_turn("Esperando inicio…", _SUBTEXT)

    def enable_human_turn(self):
        """Habilita la interacción del humano."""
        self._human_turn = True
        self._pending_cell = None
        self._confirm_btn.setEnabled(False)
        for btn in self._cells:
            if btn.is_empty:
                btn.setEnabled(True)
        self._status_bar.set_turn("Tu turno ✋", _GREEN)

    def disable_human_turn(self):
        """Bloquea la interacción del humano."""
        self._human_turn = False
        self._pending_cell = None
        self._confirm_btn.setEnabled(False)
        for btn in self._cells:
            btn.setEnabled(False)

    # Llamado desde MainWindow/GameNode cuando un movimiento termina físicamente
    def on_move_completed(self, cell_index: int, symbol: str):
        """Fija el símbolo (definitivo) en la casilla una vez el robot ha terminado."""
        btn = self._cells[cell_index]
        btn.set_symbol(symbol, provisional=False)
        btn.lock()

    # Llamado desde bridge.robot_status_changed
    def on_status_changed(self, status: str):
        """Actualiza el indicador de estado del robot. NO gestiona el turno."""
        self._status_bar.set_robot_status(status)

    # Llamado desde bridge.human_turn_started (señal explícita)
    def on_human_turn_started(self):
        """El robot ha terminado: habilitar turno del humano."""
        self.enable_human_turn()

    # Llamado desde bridge.robot_placing_human (solo teleop)
    def on_robot_placing_human(self):
        """Teleop: el robot está colocando la pieza del humano."""
        self._status_bar.set_turn("🦾 Robot colocando tu pieza…", "#fab387")

    def on_game_over(self, result: str):
        self.disable_human_turn()
        if result == "TIE":
            self._status_bar.set_turn("🤝 ¡Empate!", _YELLOW)
        elif result == self._human_symbol:
            self._status_bar.set_turn("🎉 ¡Has ganado!", _GREEN)
        else:
            self._status_bar.set_turn("🤖 ¡Gana el robot!", _PINK)

    def on_ai_thinking(self, cell: int):
        self._status_bar.set_turn(f"🤔 Robot pensando… → celda {cell}", _BLUE)

    # Acceso al widget de cámara para actualizar frames desde fuera
    @property
    def camera(self) -> CameraPlaceholder:
        return self._camera

    # ─────────────────────────────────────── slots internos

    def _on_cell_clicked(self, idx: int):
        if not self._human_turn:
            return

        # Si hay provisional en otra casilla, la limpiamos visualmente
        if self._pending_cell is not None and self._pending_cell != idx:
            prev = self._cells[self._pending_cell]
            prev.set_symbol("")   # borra texto
            prev.clear()          # restaura estado vacío / habilitado

        # Marcar la nueva celda como provisional
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

        # Fijar símbolo definitivo (color real)
        btn = self._cells[cell]
        btn.set_symbol(self._human_symbol, provisional=False)
        btn.lock()

        # Deshabilitar tablero hasta que el robot termine
        self.disable_human_turn()
        # Texto genérico: en normal el robot mueve su pieza, en teleop la del humano
        self._status_bar.set_turn("Robot en movimiento…", _BLUE)

        # Notificar al GameNode
        self.move_confirmed.emit(cell)