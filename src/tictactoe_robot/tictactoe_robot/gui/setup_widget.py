"""
gui/setup_widget.py
────────────────────
Panel de configuración inicial del juego.

Emite la señal  start_requested(symbol: str, difficulty: float)
cuando el usuario pulsa "Start Game".

Uso desde MainWindow:
    self._setup.start_requested.connect(self._on_start)

La señal devuelve:
    symbol      → "X" o "O"
    difficulty  → 0.0 (Hard) | 0.5 (Medium) | 0.25 (Easy)
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QButtonGroup,
    QFrame, QSizePolicy,
)
from PyQt6.QtCore  import pyqtSignal, Qt
from PyQt6.QtGui   import QFont, QColor, QPalette


# ─────────────────────────────────────────────────── paleta / constantes

_BG          = "#11111b"   # fondo principal
_SURFACE     = "#1e1e2e"   # tarjetas / paneles
_SURFACE2    = "#181825"   # fondo alternativo
_BORDER      = "#313244"   # bordes inactivos
_ACCENT_BLUE = "#89b4fa"   # azul Catppuccin
_ACCENT_PINK = "#f38ba8"   # rosa / X
_ACCENT_TEAL = "#94e2d5"   # teal / O
_TEXT        = "#cdd6f4"   # texto principal
_SUBTEXT     = "#6c7086"   # texto secundario
_GREEN       = "#a6e3a1"   # botón start

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
    """
    Botón de selección exclusiva con estilo propio.
    Se activa/desactiva visualmente según su estado checkable.
    """

    def __init__(self, label: str, sublabel: str = "", accent: str = _ACCENT_BLUE):
        super().__init__()
        self._accent = accent
        self.setCheckable(True)
        self.setFont(_FONT_BUTTON)
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        self.setMinimumHeight(64)

        # Layout interno: label grande + sublabel pequeño
        self._build_content(label, sublabel)
        self._refresh_style(False)
        self.toggled.connect(self._refresh_style)

    def _build_content(self, label: str, sublabel: str):
        """Usamos texto multilínea con HTML para el sublabel."""
        if sublabel:
            self.setText(f"{label}\n{sublabel}")
        else:
            self.setText(label)

    def _refresh_style(self, checked: bool):
        if checked:
            self.setStyleSheet(f"""
                QPushButton {{
                    background: {self._accent}22;
                    border: 2px solid {self._accent};
                    border-radius: 10px;
                    color: {self._accent};
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
    """Tarjeta con borde y título de sección."""

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

        # El contenido se añade al inner_layout
        self.inner = QVBoxLayout()
        self.inner.setSpacing(8)
        outer.addLayout(self.inner)


# ────────────────────────────────────────────────────── SetupWidget

class SetupWidget(QWidget):
    """
    Panel de configuración del juego.

    Señal emitida al pulsar Start:
        start_requested(symbol: str, difficulty: float)
    """

    start_requested = pyqtSignal(str, float)

    # Mapeo dificultad → random_prob
    _DIFFICULTIES = {
        "Easy":   0.25,
        "Medium": 0.50,
        "Hard":   0.0,
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self._symbol     : str   = "X"
        self._difficulty : float = 0.0

        self.setStyleSheet(f"background: {_BG};")
        self._build_ui()

    # ─────────────────────────────────────────────── UI

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 20, 24, 20)
        root.setSpacing(16)

        # ── Título ─────────────────────────────────────────────────────
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

        # ── Símbolo ────────────────────────────────────────────────────
        sym_card = SectionCard("// CHOOSE YOUR SYMBOL")
        sym_row  = QHBoxLayout()
        sym_row.setSpacing(12)

        self._btn_x = ToggleButton("✕  Play as X", "goes first (usually)", _ACCENT_PINK)
        self._btn_o = ToggleButton("○  Play as O", "goes second", _ACCENT_TEAL)

        self._sym_group = QButtonGroup(self)
        self._sym_group.setExclusive(True)
        self._sym_group.addButton(self._btn_x)
        self._sym_group.addButton(self._btn_o)
        self._btn_x.setChecked(True)

        sym_row.addWidget(self._btn_x)
        sym_row.addWidget(self._btn_o)
        sym_card.inner.addLayout(sym_row)
        root.addWidget(sym_card)

        # ── Dificultad ─────────────────────────────────────────────────
        diff_card = SectionCard("// DIFFICULTY")
        diff_row  = QHBoxLayout()
        diff_row.setSpacing(12)

        self._diff_btns: dict[str, ToggleButton] = {}
        self._diff_group = QButtonGroup(self)
        self._diff_group.setExclusive(True)

        configs = [
            ("Easy",   "25% random moves",    _ACCENT_TEAL),
            ("Medium", "50% random moves",    _ACCENT_BLUE),
            ("Hard",   "perfect Minimax AI",  _ACCENT_PINK),
        ]
        for name, sub, accent in configs:
            btn = ToggleButton(name, sub, accent)
            self._diff_btns[name] = btn
            self._diff_group.addButton(btn)
            diff_row.addWidget(btn)

        self._diff_btns["Hard"].setChecked(True)
        diff_card.inner.addLayout(diff_row)
        root.addWidget(diff_card)

        # ── Info dinámica ──────────────────────────────────────────────
        self._info_lbl = QLabel(self._info_text())
        self._info_lbl.setFont(_FONT_SMALL)
        self._info_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._info_lbl.setStyleSheet(f"color: {_SUBTEXT};")
        root.addWidget(self._info_lbl)

        # ── Botón Start ────────────────────────────────────────────────
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
            QPushButton:hover {{
                background: {_GREEN}44;
            }}
            QPushButton:pressed {{
                background: {_GREEN}66;
            }}
            QPushButton:disabled {{
                background: {_SURFACE};
                border-color: {_BORDER};
                color: {_SUBTEXT};
            }}
        """)
        root.addWidget(self._start_btn)

        # ── Conexiones ─────────────────────────────────────────────────
        self._btn_x.toggled.connect(self._on_symbol_changed)
        self._btn_o.toggled.connect(self._on_symbol_changed)
        for btn in self._diff_btns.values():
            btn.toggled.connect(self._on_difficulty_changed)
        self._start_btn.clicked.connect(self._on_start_clicked)

    # ─────────────────────────────────────────────── slots

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
        self._start_btn.setEnabled(False)
        self._start_btn.setText("⏳  GAME IN PROGRESS…")
        self.start_requested.emit(self._symbol, self._difficulty)

    # ─────────────────────────────────────────────── helpers

    def _info_text(self) -> str:
        diff_name = next(
            (k for k, v in self._DIFFICULTIES.items() if v == self._difficulty),
            "Hard"
        )
        ai_sym = "O" if self._symbol == "X" else "X"
        return (
            f"You: '{self._symbol}'  │  Robot AI: '{ai_sym}'  │  "
            f"Difficulty: {diff_name}"
        )

    def enable_new_game(self):
        """
        Llamar desde MainWindow cuando la partida termina,
        para permitir iniciar una nueva.
        """
        self._start_btn.setEnabled(True)
        self._start_btn.setText("▶  NEW GAME")