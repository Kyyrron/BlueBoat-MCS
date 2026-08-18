"""Dark professional theme (QGroundControl-inspired).

Central palette + application stylesheet.  Map/plot colors are exposed as
constants so graphics items and painters share the exact same language.
"""

from __future__ import annotations

from PySide6.QtGui import QColor

# ------------------------------------------------------------------ palette
BG_DARK = "#14181d"
BG_PANEL = "#1c2128"
BG_RAISED = "#242b33"
BORDER = "#333c46"
TEXT = "#d7dde3"
TEXT_DIM = "#8a949e"
ACCENT = "#2f81f7"
ACCENT_DIM = "#1c4e8f"
OK = "#3fb950"
WARN = "#d29922"
ERR = "#f85149"
ESTOP = "#c93030"

# Map entity colors
C_ROBOT = QColor("#2f81f7")
C_ROBOT_TRACK = QColor("#2f81f7")
C_MISSION_PATH = QColor("#3fb950")
C_PINGER = QColor("#f0883e")
C_PINGER_TRACK = QColor("#f0883e")
C_TARGET_LINE = QColor("#e3b341")
C_PREDICTED = QColor("#bc8cff")
C_MANUAL_TARGET = QColor("#ff6b9d")
C_GRID = QColor(255, 255, 255, 22)
C_GRID_MAJOR = QColor(255, 255, 255, 45)
C_MEASURE = QColor("#56d4dd")

STYLESHEET = f"""
QWidget {{
    background-color: {BG_DARK};
    color: {TEXT};
    font-family: "DejaVu Sans", "Segoe UI", sans-serif;
    font-size: 12px;
}}
QMainWindow::separator {{ background: {BORDER}; width: 2px; height: 2px; }}
QFrame#panel {{
    background-color: {BG_PANEL};
    border: 1px solid {BORDER};
    border-radius: 4px;
}}
QLabel#sectionTitle {{
    color: {TEXT_DIM};
    font-weight: bold;
    font-size: 11px;
    letter-spacing: 1px;
    padding: 4px 0px;
}}
QLabel#valueLabel {{ font-family: "DejaVu Sans Mono", monospace; }}
QGroupBox {{
    border: 1px solid {BORDER};
    border-radius: 4px;
    margin-top: 10px;
    background-color: {BG_PANEL};
    font-weight: bold;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 8px; padding: 0 4px;
    color: {TEXT_DIM};
}}
QPushButton {{
    background-color: {BG_RAISED};
    border: 1px solid {BORDER};
    border-radius: 3px;
    padding: 6px 12px;
}}
QPushButton:hover {{ border-color: {ACCENT}; }}
QPushButton:pressed {{ background-color: {ACCENT_DIM}; }}
QPushButton:checked {{ background-color: {ACCENT_DIM}; border-color: {ACCENT}; }}
QPushButton:disabled {{ color: {TEXT_DIM}; background-color: {BG_PANEL}; }}
QPushButton#estopButton {{
    background-color: {ESTOP};
    color: white; font-weight: bold; font-size: 14px;
    border: 2px solid #7a1f1f; border-radius: 4px; padding: 8px 20px;
}}
QPushButton#estopButton:hover {{ background-color: #e04040; }}
QPushButton#launchButton {{ font-weight: bold; }}
QCheckBox {{ spacing: 6px; }}
QCheckBox::indicator {{ width: 14px; height: 14px; }}
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QPlainTextEdit {{
    background-color: {BG_RAISED};
    border: 1px solid {BORDER};
    border-radius: 3px;
    padding: 4px;
    selection-background-color: {ACCENT_DIM};
}}
QComboBox::drop-down {{ border: none; width: 18px; }}
QScrollArea {{ border: none; }}
QScrollBar:vertical {{ background: {BG_PANEL}; width: 10px; }}
QScrollBar::handle:vertical {{ background: {BORDER}; border-radius: 4px; min-height: 24px; }}
QScrollBar:horizontal {{ background: {BG_PANEL}; height: 10px; }}
QScrollBar::handle:horizontal {{ background: {BORDER}; border-radius: 4px; min-width: 24px; }}
QToolTip {{ background-color: {BG_RAISED}; color: {TEXT}; border: 1px solid {BORDER}; }}
QStatusBar {{ background: {BG_PANEL}; border-top: 1px solid {BORDER}; }}
QSplitter::handle {{ background: {BORDER}; }}
QTabWidget::pane {{ border: 1px solid {BORDER}; }}
QTabBar::tab {{
    background: {BG_PANEL}; padding: 5px 12px; border: 1px solid {BORDER};
    border-bottom: none; border-top-left-radius: 3px; border-top-right-radius: 3px;
}}
QTabBar::tab:selected {{ background: {BG_RAISED}; }}
"""
