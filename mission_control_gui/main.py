#!/usr/bin/env python3
"""
Step 2: Waypoint input UI (no ROS yet).

Just the PyQt5 dashboard skeleton:
- 3 rows of labeled X / Y input fields
- A Dispatch button
- A read-only log panel

On Dispatch click, all 6 fields are validated (non-empty, numeric).
If any field fails, a warning dialog is shown instead of proceeding.
If all fields are valid, the parsed (x, y) tuples are printed into
the log panel -- this is where Nav2 dispatch logic will hook in later.

Run with:
    python3 main_window_test.py
"""

import sys

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton,
    QTextEdit, QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox
)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Mission Control GUI - Step 2 (Waypoint Input)')
        self.resize(500, 400)

        self.waypoint_fields = []  # list of (x_field, y_field) QLineEdit pairs

        central = QWidget()
        main_layout = QVBoxLayout()

        # --- Waypoint input grid ---
        grid = QGridLayout()
        for i in range(3):
            label = QLabel(f'Waypoint {i + 1}:')
            x_label = QLabel('X')
            x_field = QLineEdit()
            x_field.setPlaceholderText('e.g. 2.0')
            y_label = QLabel('Y')
            y_field = QLineEdit()
            y_field.setPlaceholderText('e.g. 1.5')

            grid.addWidget(label, i, 0)
            grid.addWidget(x_label, i, 1)
            grid.addWidget(x_field, i, 2)
            grid.addWidget(y_label, i, 3)
            grid.addWidget(y_field, i, 4)

            self.waypoint_fields.append((x_field, y_field))

        main_layout.addLayout(grid)

        # --- Dispatch button ---
        button_row = QHBoxLayout()
        self.dispatch_button = QPushButton('Dispatch')
        self.dispatch_button.clicked.connect(self.on_dispatch_clicked)
        button_row.addStretch()
        button_row.addWidget(self.dispatch_button)
        main_layout.addLayout(button_row)

        # --- Log panel ---
        log_label = QLabel('Mission Log:')
        main_layout.addWidget(log_label)
        self.log_panel = QTextEdit()
        self.log_panel.setReadOnly(True)
        main_layout.addWidget(self.log_panel)

        central.setLayout(main_layout)
        self.setCentralWidget(central)

    def on_dispatch_clicked(self):
        waypoints = []
        errors = []

        for i, (x_field, y_field) in enumerate(self.waypoint_fields):
            x_text = x_field.text().strip()
            y_text = y_field.text().strip()

            if not x_text or not y_text:
                errors.append(f'Waypoint {i + 1}: X and Y must not be empty.')
                continue

            try:
                x_val = float(x_text)
                y_val = float(y_text)
            except ValueError:
                errors.append(f'Waypoint {i + 1}: X and Y must be numeric.')
                continue

            waypoints.append((x_val, y_val))

        if errors:
            QMessageBox.warning(
                self,
                'Invalid Waypoint Input',
                '\n'.join(errors)
            )
            self.log('Dispatch aborted due to invalid input.')
            return

        self.log('All waypoints valid. Parsed waypoints:')
        for i, (x, y) in enumerate(waypoints):
            self.log(f'  Waypoint {i + 1}: x={x:.2f}, y={y:.2f}')
        self.log('(Nav2 dispatch logic will be wired in here in Step 3.)')

    def log(self, message: str):
        self.log_panel.append(message)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
