"""
gui/setup_widget.py
────────────────────
Initial game setup window.
Shown on startup; pressing Start closes it and opens MainWindow.

Emitted signal:
    start_requested(symbol: str, difficulty: float, teleop: bool)

Modes:
  • Normal → AI decides and the robot moves only on its own turn.
  • Teleop → the robot executes all pieces (human + AI).
             The difficulty section is hidden because difficulty does
             not need to be exposed in teleop mode.
"""

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QButtonGroup,
    QFrame, QSizePolicy,
)
from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtGui  import QFont


# ─────────────────────────────────────────────────── palette / constants

_BG           = "#11111b"
_SURFACE      = "#1e1e2e"
_SURFACE2     = "#181825"
_BORDER       = "#313244"
_ACCENT_BLUE  = "#1d4ed8"
_ACCENT_PINK  = "#dc2626"
_ACCENT_TEAL  = "#2dd4bf"
_ACCENT_PEACH = "#fab387"   # orange for teleop mode
_ACCENT_GREEN = "#10b981"
_ACCENT_GOLD  = "#0ea5e9"
_ACCENT_VIOLET = "#f59e0b"
_TEXT         = "#cdd6f4"
_SUBTEXT      = "#6c7086"
_GREEN        = "#a6e3a1"

_FONT_TITLE  = QFont("JetBrains Mono", 13, QFont.Weight.Bold)
_FONT_LABEL  = QFont("JetBrains Mono", 9)
_FONT_BUTTON = QFont("JetBrains Mono", 10, QFont.Weight.Bold)
_FONT_BIG    = QFont("JetBrains Mono", 22, QFont.Weight.Bold)
_FONT_SMALL  = QFont("JetBrains Mono", 8)


def _h_line() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setStyleSheet(f"color: {_BORDER};")
    return line


# ────────────────────────────────────────────────────── ToggleButton

class ToggleButton(QPushButton):
    def __init__(
        self,
        label: str,
        sublabel: str = "",
        accent: str = _ACCENT_BLUE,
        fill_alpha: str = "22",
    ):
        super().__init__()
        self._accent = accent
        self._fill_alpha = fill_alpha
        self.setCheckable(True)
        self.setFont(_FONT_BUTTON)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setMinimumHeight(64)
        self.setText(f"{label}\n{sublabel}" if sublabel else label)
        self._refresh_style(False)
        self.toggled.connect(self._refresh_style)

    def _refresh_style(self, checked: bool):
        if checked:
            self.setStyleSheet(f"""
                QPushButton {{
                    background: {self._accent}{self._fill_alpha};
                    border: 2px solid {self._accent};
                    border-radius: 10px;
                    color: {_TEXT};
                    padding: 8px 16px;
                }}
            """)
        else:
            self.setStyleSheet(f"""
                QPushButton {{
                    background: {_SURFACE};
                    border: 2px solid {_BORDER};
                    border-radius: 10px;
                    color: {_SUBTEXT};
                    padding: 8px 16px;
                }}
                QPushButton:hover {{
                    border-color: {self._accent}88;
                    color: {_TEXT};
                    background: {_SURFACE2};
                }}
            """)


# ────────────────────────────────────────────────────── SectionCard

class SectionCard(QFrame):
    def __init__(self, title: str):
        super().__init__()
        self.setStyleSheet(f"""
            QFrame {{
                background: {_SURFACE};
                border: 1px solid {_BORDER};
                border-radius: 12px;
            }}
        """)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(16, 14, 16, 16)
        outer.setSpacing(12)

        title_lbl = QLabel(title)
        title_lbl.setFont(_FONT_TITLE)
        title_lbl.setStyleSheet(f"color: {_SUBTEXT}; border: none;")
        outer.addWidget(title_lbl)
        outer.addWidget(_h_line())

        self.inner = QVBoxLayout()
        self.inner.setSpacing(8)
        outer.addLayout(self.inner)


# ────────────────────────────────────────────────────── SetupDialog

class SetupDialog(QDialog):
    """
    Modal setup dialog.
    Accept → emit start_requested and close.
    """

    # symbol, difficulty, teleop
    start_requested = pyqtSignal(str, float, bool)

    _DIFFICULTIES = {
        "Easy":   0.75,
        "Medium": 0.50,
        "Hard":   0.0,
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("TicTacToe × UR3 — Setup")
        self.setModal(True)
        self.setMinimumWidth(540)
        self.setStyleSheet(f"background: {_BG};")

        self._symbol     : str   = "X"
        self._difficulty : float = 0.0
        self._teleop     : bool  = False

        self._build_ui()

    # ──────────────────────────────────────────────── UI

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(28, 24, 28, 24)
        root.setSpacing(18)

        # Title
        title = QLabel("TICTACTOE  ×  UR3 CB3")
        title.setFont(_FONT_BIG)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet(f"color: {_TEXT};")
        root.addWidget(title)

        subtitle = QLabel("Human  vs  Robot  │  Minimax + Alpha-Beta Pruning")
        subtitle.setFont(_FONT_LABEL)
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)
        subtitle.setStyleSheet(f"color: {_SUBTEXT};")
        root.addWidget(subtitle)

        # ── operation mode ─────────────────────────────────────────────
        mode_card = SectionCard("// MODE")
        mode_row  = QHBoxLayout()
        mode_row.setSpacing(12)

        self._btn_normal = ToggleButton(
            "🤖  Normal",
            "AI moves its own pieces",
            _ACCENT_GREEN,
            "3d",
        )
        self._btn_teleop = ToggleButton(
            "🕹️  Teleop",
            "robot moves all pieces",
            _ACCENT_PEACH,
        )

        self._mode_group = QButtonGroup(self)
        self._mode_group.setExclusive(True)
        self._mode_group.addButton(self._btn_normal)
        self._mode_group.addButton(self._btn_teleop)
        self._btn_normal.setChecked(True)

        mode_row.addWidget(self._btn_normal)
        mode_row.addWidget(self._btn_teleop)
        mode_card.inner.addLayout(mode_row)

        # Teleop explanation note
        self._teleop_note = QLabel(
            "⚠  In teleop mode the robot physically executes ALL pieces —\n"
            "    yours and its own. You choose on the board; the arm performs them."
        )
        self._teleop_note.setFont(_FONT_SMALL)
        self._teleop_note.setStyleSheet(
            f"color: {_ACCENT_PEACH}; border: none; padding-top: 4px;"
        )
        self._teleop_note.setVisible(False)
        mode_card.inner.addWidget(self._teleop_note)
        root.addWidget(mode_card)

        # ── symbol ─────────────────────────────────────────────────────
        sym_card = SectionCard("// CHOOSE YOUR SYMBOL")
        sym_row  = QHBoxLayout()
        sym_row.setSpacing(12)

        self._btn_x = ToggleButton("✕  Play as X", accent=_ACCENT_BLUE, fill_alpha="34")
        self._btn_o = ToggleButton("○  Play as O", accent=_ACCENT_PINK, fill_alpha="34")

        self._sym_group = QButtonGroup(self)
        self._sym_group.setExclusive(True)
        self._sym_group.addButton(self._btn_x)
        self._sym_group.addButton(self._btn_o)
        self._btn_x.setChecked(True)

        sym_row.addWidget(self._btn_x)
        sym_row.addWidget(self._btn_o)
        sym_card.inner.addLayout(sym_row)
        root.addWidget(sym_card)

        # ── difficulty (hidden in teleop) ──────────────────────────────
        self._diff_card = SectionCard("// DIFFICULTY")
        diff_row        = QHBoxLayout()
        diff_row.setSpacing(12)

        self._diff_btns: dict[str, ToggleButton] = {}
        self._diff_group = QButtonGroup(self)
        self._diff_group.setExclusive(True)

        configs = [
            ("Easy",   "75% random moves",   _ACCENT_TEAL),
            ("Medium", "50% random moves",   _ACCENT_GOLD),
            ("Hard",   "perfect Minimax AI", _ACCENT_VIOLET),
        ]
        for name, sub, accent in configs:
            fill_alpha = "3d" if name == "Medium" else "22"
            btn = ToggleButton(name, sub, accent, fill_alpha)
            self._diff_btns[name] = btn
            self._diff_group.addButton(btn)
            diff_row.addWidget(btn)

        self._diff_btns["Hard"].setChecked(True)
        self._diff_card.inner.addLayout(diff_row)
        root.addWidget(self._diff_card)

        # ── dynamic info ───────────────────────────────────────────────
        self._info_lbl = QLabel(self._info_text())
        self._info_lbl.setFont(_FONT_SMALL)
        self._info_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._info_lbl.setStyleSheet(f"color: {_SUBTEXT};")
        root.addWidget(self._info_lbl)

        # ── Start button ───────────────────────────────────────────────
        self._start_btn = QPushButton("▶  START GAME")
        self._start_btn.setFont(QFont("JetBrains Mono", 12, QFont.Weight.Bold))
        self._start_btn.setMinimumHeight(52)
        self._start_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._start_btn.setStyleSheet(f"""
            QPushButton {{
                background: {_GREEN}22;
                border: 2px solid {_GREEN};
                border-radius: 10px;
                color: {_GREEN};
                letter-spacing: 2px;
            }}
            QPushButton:hover  {{ background: {_GREEN}44; }}
            QPushButton:pressed {{ background: {_GREEN}66; }}
        """)
        root.addWidget(self._start_btn)

        # ── connections ─────────────────────────────────────────────────
        self._btn_normal.toggled.connect(self._on_mode_changed)
        self._btn_teleop.toggled.connect(self._on_mode_changed)
        self._btn_x.toggled.connect(self._on_symbol_changed)
        self._btn_o.toggled.connect(self._on_symbol_changed)
        for btn in self._diff_btns.values():
            btn.toggled.connect(self._on_difficulty_changed)
        self._start_btn.clicked.connect(self._on_start_clicked)

    # ──────────────────────────────────────────────── slots

    def _on_mode_changed(self):
        self._teleop = self._btn_teleop.isChecked()
        self._teleop_note.setVisible(self._teleop)
        self.adjustSize()
        self._info_lbl.setText(self._info_text())

    def _on_symbol_changed(self):
        self._symbol = "X" if self._btn_x.isChecked() else "O"
        self._info_lbl.setText(self._info_text())

    def _on_difficulty_changed(self):
        for name, btn in self._diff_btns.items():
            if btn.isChecked():
                self._difficulty = self._DIFFICULTIES[name]
                break
        self._info_lbl.setText(self._info_text())

    def _on_start_clicked(self):
        self.start_requested.emit(self._symbol, self._difficulty, self._teleop)
        self.accept()

    # ──────────────────────────────────────────────── helpers

    def _info_text(self) -> str:
        ai_sym = "O" if self._symbol == "X" else "X"
        diff_name = next(
            (k for k, v in self._DIFFICULTIES.items() if v == self._difficulty),
            "Hard",
        )
        mode_str = "TELEOP" if self._teleop else "Normal"
        return (
            f"You: '{self._symbol}'  │  Robot AI: '{ai_sym}'  │  "
            f"Difficulty: {diff_name}  │  Mode: {mode_str}"
        )
