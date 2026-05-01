"""
gui/board_widget.py
────────────────────
Main game widget. Contains:
  • RobotStatusBar (top) with integrated emergency button
  • 3×3 board with provisional selection
  • Move confirmation button
  • Camera feed placeholder (bottom)

Emergency button (⛔ STOP):
  • Normal mode  → visible only during the robot's turn.
  • Teleop mode  → visible whenever the robot is actively moving.
  • When pressed it emits BoardWidget.emergency_requested,
    which MainWindow connects to GameNode.emergency_stop().
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QFrame, QSizePolicy,
)
from PyQt6.QtCore  import pyqtSignal, Qt
from PyQt6.QtGui   import QFont, QImage, QPixmap


# ─────────────────────────────────────── palette

_BG        = "#11111b"
_SURFACE   = "#1e1e2e"
_BORDER    = "#313244"
_TEXT      = "#cdd6f4"
_SUBTEXT   = "#6c7086"
_PINK      = "#dc2626"
_TEAL      = "#94e2d5"
_BLUE      = "#1e3a8a"
_GREEN     = "#a6e3a1"
_YELLOW    = "#f9e2af"
_RED       = "#dc2626"   # semantic danger color
_OVERLAY   = "#45475a"

_FONT_CELL    = QFont("JetBrains Mono", 38, QFont.Weight.Bold)
_FONT_STATUS  = QFont("JetBrains Mono", 11, QFont.Weight.Bold)
_FONT_CONFIRM = QFont("JetBrains Mono", 11, QFont.Weight.Bold)
_FONT_STOP    = QFont("JetBrains Mono", 9,  QFont.Weight.Bold)


def _symbol_color(symbol: str, provisional: bool = False) -> str:
    if provisional:
        return _OVERLAY
    return _BLUE if symbol == "X" else _RED


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
    Top bar with:
      left   → dot + "Robot: STATUS"
      center → turn text
      right  → conditional ⛔ STOP button
    """

    emergency_requested = pyqtSignal()   # connected to GameNode.emergency_stop()

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

        # ── turn label ──────────────────────────────────────────────────
        self._turn_lbl = QLabel("")
        self._turn_lbl.setFont(_FONT_STATUS)
        self._turn_lbl.setStyleSheet(f"color: {_TEXT}; border: none;")
        layout.addWidget(self._turn_lbl)

        layout.addStretch()

        # ── emergency button ────────────────────────────────────────────
        self._stop_btn = QPushButton("⛔  STOP")
        self._stop_btn.setFont(_FONT_STOP)
        self._stop_btn.setFixedHeight(34)
        self._stop_btn.setMinimumWidth(80)
        self._stop_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._stop_btn.setVisible(False)   # hidden by default
        self._stop_btn.setStyleSheet(f"""
            QPushButton {{
                background: #fecdd333;
                border: 2px solid #ef4444;
                border-radius: 8px;
                color: #ef4444;
                letter-spacing: 1px;
            }}
            QPushButton:hover  {{ background: #fda4af4d; }}
            QPushButton:pressed {{ background: #fb718566; }}
            QPushButton:disabled {{
                background: {_SURFACE};
                border-color: {_BORDER};
                color: {_SUBTEXT};
            }}
        """)
        self._stop_btn.clicked.connect(self.emergency_requested)
        layout.addWidget(self._stop_btn)

    # ── public API ──────────────────────────────────────────────────────

    def set_robot_status(self, status: str):
        status_upper = status.upper()
        if status_upper == "IDLE":
            color, dot_color = _GREEN, _GREEN
        elif status_upper == "BUSY":
            color, dot_color = _YELLOW, _YELLOW
        elif status_upper.startswith("MOVING"):
            color, dot_color = _BLUE, _BLUE
        elif status_upper in ("EMERGENCY_STOP", "VISION_PAUSED"):
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
        Control emergency button visibility and enabled state.
          visible=True  → shown
          enabled=False → shown but disabled
        """
        self._stop_btn.setVisible(visible)
        self._stop_btn.setEnabled(enabled)

    def set_stop_button_pressed(self):
        """Disable the button after it is pressed to prevent double clicks."""
        self._stop_btn.setEnabled(False)
        self._stop_btn.setText("⏳  Stopping...")


# ─────────────────────────────────────── CameraPlaceholder

class CameraPlaceholder(QFrame):
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(200)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._last_pixmap: QPixmap | None = None
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

        self._placeholder_lbl = QLabel("📷  Camera not connected")
        self._placeholder_lbl.setFont(QFont("JetBrains Mono", 10))
        self._placeholder_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._placeholder_lbl.setStyleSheet(f"color: {_SUBTEXT}; border: none;")
        layout.addWidget(self._placeholder_lbl)

    def _refresh_pixmap(self):
        if self._last_pixmap is None:
            self._cam_label.clear()
            self._placeholder_lbl.show()
            return

        self._placeholder_lbl.hide()
        size = self._cam_label.size()
        if size.width() <= 0 or size.height() <= 0:
            return

        self._cam_label.setPixmap(
            self._last_pixmap.scaled(
                size,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

    def update_frame(self, image):
        if isinstance(image, QImage):
            pixmap = QPixmap.fromImage(image)
        else:
            pixmap = image

        self._last_pixmap = pixmap
        self._placeholder_lbl.hide()
        self._refresh_pixmap()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh_pixmap()


# ─────────────────────────────────────── BoardWidget

class BoardWidget(QWidget):
    """
    Public signals:
        move_confirmed(cell_index) → GameNode.human_move()
        emergency_requested()      → GameNode.emergency_stop()
    """

    move_confirmed    = pyqtSignal(int)
    emergency_requested = pyqtSignal()   # re-emits the status bar signal outward

    def __init__(self):
        super().__init__()
        self.setStyleSheet(f"background: {_BG};")

        self._human_symbol  : str        = "X"
        self._ai_symbol     : str        = "O"
        self._pending_cell  : int | None = None
        self._human_turn    : bool       = False
        self._teleop        : bool       = False   # initialized in reset()
        self._cells         : list[CellButton] = []

        self._build_ui()

        # Re-emit the status bar signal as a widget signal
        self._status_bar.emergency_requested.connect(self._on_stop_pressed)

    # ─────────────────────────────────────── UI

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 12, 16, 12)
        root.setSpacing(10)

        # ── status bar ────────────────────────────────────────────────
        self._status_bar = RobotStatusBar()
        root.addWidget(self._status_bar)

        self._vision_warning = QLabel("")
        self._vision_warning.setFont(QFont("JetBrains Mono", 9, QFont.Weight.Bold))
        self._vision_warning.setWordWrap(True)
        self._vision_warning.setVisible(False)
        self._vision_warning.setStyleSheet(f"""
            QLabel {{
                color: {_YELLOW};
                background: {_YELLOW}14;
                border: 1px solid {_YELLOW}66;
                border-radius: 8px;
                padding: 8px 10px;
            }}
        """)
        root.addWidget(self._vision_warning)

        # ── board ─────────────────────────────────────────────────────
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

        # ── confirm button ────────────────────────────────────────────
        self._confirm_btn = QPushButton("✔  Confirm move")
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

        # ── camera ────────────────────────────────────────────────────
        cam_header = QLabel("// LIVE VISION")
        cam_header.setFont(QFont("JetBrains Mono", 8, QFont.Weight.Bold))
        cam_header.setStyleSheet(f"color: {_SUBTEXT};")
        root.addWidget(cam_header)

        self._camera = CameraPlaceholder()
        root.addWidget(self._camera)

    # ─────────────────────────────────────── public API

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
        self._status_bar.set_turn("Waiting to start...", _SUBTEXT)

        self._reset_stop_button()

    def enable_human_turn(self):
        self._human_turn = True
        self._pending_cell = None
        self._confirm_btn.setEnabled(False)
        for btn in self._cells:
            if btn.is_empty:
                btn.setEnabled(self._teleop)
        if self._teleop:
            self._status_bar.set_turn("Your turn ✋", _GREEN)
        else:
            self._status_bar.set_turn("Place your piece on the board", _GREEN)

        # Hide STOP during the human turn
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
        self._status_bar.set_turn("🦾 Robot placing your piece...", "#fab387")
        self._show_active_stop_button()

    def on_game_over(self, result: str):
        self.disable_human_turn()
        self._status_bar.show_stop_button(visible=False)
        if result == "TIE":
            self._status_bar.set_turn("🤝 It's a tie!", _YELLOW)
        elif result == self._human_symbol:
            self._status_bar.set_turn("🎉 You win!", _GREEN)
        else:
            self._status_bar.set_turn("🤖 Robot wins!", _PINK)

    def on_ai_thinking(self, cell: int):
        self._status_bar.set_turn(f"🤔 Robot thinking... → cell {cell}", _BLUE)
        self._show_active_stop_button()

    def set_vision_warning(self, text: str):
        self._vision_warning.setText(text)
        self._vision_warning.setVisible(bool(text))

    def on_vision_provisional_move(self, cell_index: int):
        if self._teleop or not self._human_turn:
            return

        if self._pending_cell is not None:
            prev = self._cells[self._pending_cell]
            if prev.symbol == self._human_symbol:
                prev.clear()
                prev.setEnabled(False)

        self._pending_cell = None
        self._confirm_btn.setEnabled(False)

        if cell_index < 0:
            self._status_bar.set_turn("Place your piece on the board", _GREEN)
            return

        btn = self._cells[cell_index]
        if not btn.is_empty:
            return

        self._pending_cell = cell_index
        btn.set_symbol(self._human_symbol, provisional=True)
        btn.setEnabled(False)
        self._confirm_btn.setEnabled(True)
        self._status_bar.set_turn(
            f"Vision selected cell {cell_index} — confirm?", _YELLOW
        )

    @property
    def camera(self) -> CameraPlaceholder:
        return self._camera

    # ─────────────────────────────────────── internal slots

    def _on_cell_clicked(self, idx: int):
        if not self._human_turn:
            return
        if not self._teleop:
            return

        if self._pending_cell is not None and self._pending_cell != idx:
            prev = self._cells[self._pending_cell]
            prev.set_symbol("")
            prev.clear()

        self._pending_cell = idx
        self._cells[idx].set_symbol(self._human_symbol, provisional=True)
        self._confirm_btn.setEnabled(True)
        self._status_bar.set_turn(
            f"Cell {idx} selected — confirm?", _YELLOW
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
        self._status_bar.set_turn("Robot moving...", _BLUE)
        self._show_active_stop_button()

        self.move_confirmed.emit(cell)

    def _on_stop_pressed(self):
        """Handle the emergency button press."""
        if not self._status_bar._stop_btn.isEnabled():
            return
        self._status_bar.set_stop_button_pressed()
        self._status_bar.set_turn("⛔ Emergency stop...", _RED)
        self.emergency_requested.emit()

    def _reset_stop_button(self):
        self._status_bar._stop_btn.setText("⛔  STOP")
        self._status_bar.show_stop_button(visible=False, enabled=True)

    def _show_active_stop_button(self):
        self._status_bar._stop_btn.setText("⛔  STOP")
        self._status_bar.show_stop_button(visible=True, enabled=True)
