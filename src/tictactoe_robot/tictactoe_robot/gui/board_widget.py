from PyQt6.QtWidgets import QWidget, QGridLayout, QPushButton, QLabel, QStatusBar
from PyQt6.QtCore    import pyqtSignal, Qt
from PyQt6.QtGui     import QFont


class CellButton(QPushButton):
    """Uno de los 9 huecos del tablero."""

    def __init__(self, index: int):
        super().__init__("")
        self.index = index
        self.setFixedSize(120, 120)
        self.setFont(QFont("Arial", 40, QFont.Weight.Bold))
        self.setStyleSheet("""
            QPushButton {
                border: 3px solid #555;
                border-radius: 8px;
                background: #1e1e2e;
                color: #cdd6f4;
            }
            QPushButton:hover:enabled {
                background: #313244;
            }
            QPushButton:disabled {
                color: #89b4fa;
            }
        """)


class BoardWidget(QWidget):

    cell_clicked = pyqtSignal(int)   # enviado a GameNode.human_move()

    def __init__(self):
        super().__init__()
        self._human_symbol = "X"
        self._ai_symbol    = "O"
        self._cells: list[CellButton] = []

        layout = QGridLayout(self)
        layout.setSpacing(8)

        for i in range(9):
            btn = CellButton(i)
            btn.clicked.connect(lambda _, idx=i: self.cell_clicked.emit(idx))
            self._cells.append(btn)
            layout.addWidget(btn, i // 3, i % 3)

        self._status = QLabel("Configura el juego y pulsa Start")
        self._status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self._status, 3, 0, 1, 3)

        self._set_cells_enabled(False)

    def reset(self, human_symbol: str, ai_symbol: str):
        self._human_symbol = human_symbol
        self._ai_symbol    = ai_symbol
        for btn in self._cells:
            btn.setText("")
            btn.setEnabled(True)
        self._status.setText("¡Tu turno!" if True else "Turno del robot…")

    def on_move_completed(self, cell_index: int):
        # Aquí necesitarías saber qué símbolo se jugó.
        # Lo más simple: el board_widget consulta al nodo o recibe el símbolo.
        # Por claridad, emite la señal con (cell, symbol) — ajusta según prefieras.
        pass

    def on_status_changed(self, status: str):
        self._status.setText(f"Robot: {status}")
        human_turn = status == "IDLE"
        self._set_cells_enabled(human_turn)

    def on_game_over(self, result: str):
        self._set_cells_enabled(False)
        if result == "TIE":
            self._status.setText("🤝 ¡Empate!")
        elif result == self._human_symbol:
            self._status.setText("🎉 ¡Has ganado!")
        else:
            self._status.setText("🤖 ¡Gana el robot!")

    def on_ai_thinking(self, cell: int):
        self._status.setText(f"🤔 Robot eligiendo celda {cell}…")
        self._set_cells_enabled(False)

    def _set_cells_enabled(self, enabled: bool):
        for btn in self._cells:
            if btn.text() == "":   # solo habilita las vacías
                btn.setEnabled(enabled)