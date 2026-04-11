"""
test_setup_widget.py
─────────────────────
Prueba el SetupWidget de forma aislada, sin ROS2.

Ejecutar:
    python test_setup_widget.py

Requisito:
    pip install PyQt6
"""

import sys
from PyQt6.QtWidgets import QApplication, QMainWindow
from setup_widget import SetupWidget   # mismo directorio


class TestWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SetupWidget — test standalone")
        self.setMinimumWidth(620)

        widget = SetupWidget()
        self.setCentralWidget(widget)

        # Conectar la señal para ver qué emite
        widget.start_requested.connect(self._on_start)

        # Simular fin de partida a los 3 segundos para probar el reset
        from PyQt6.QtCore import QTimer
        QTimer.singleShot(5000, widget.enable_new_game)

    def _on_start(self, symbol: str, difficulty: float):
        print(f"\n✅ start_requested emitida:")
        print(f"   symbol     = '{symbol}'")
        print(f"   difficulty = {difficulty}")
        print(f"   (en 5 s aparecerá el botón NEW GAME)\n")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = TestWindow()
    win.show()
    sys.exit(app.exec())