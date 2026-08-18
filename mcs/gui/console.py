"""Launch console — dedicated view of the launched ROS2 processes' output.

Displays every stdout/stderr line captured from the ``ros2 launch`` child
(streamed through ``SignalBus.launch_output``), exclusively; application logs
go to the status bar and the terminal, never here.

Properties:

* **Order-preserving** — lines arrive through one queued Qt signal from the
  single reader thread, so display order is reception order.
* **Long-running safe** — the document is capped at ``MAX_LINES`` blocks
  (oldest dropped by Qt) and the refilter history at the same bound, keeping
  memory constant across multi-hour experiments.
* **Auto-scroll** — follows the tail only while the scrollbar is at the
  bottom; scrolling up to read freezes the view without pausing capture.
* **Severity coloring** — ``[INFO]`` / ``[WARN]`` / ``[ERROR]``/``[FATAL]``
  tokens of the ros2 launch line format (plus tracebacks) are tinted.
* **Filter toolbox** — categories matching what ``master_control`` actually
  prints (everything / targets / thrust / warnings & errors) plus a free
  keyword field. Filters affect display only; capture is never filtered.
"""

from __future__ import annotations

import html
import re
from collections import deque
from dataclasses import dataclass

from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QComboBox, QHBoxLayout, QLineEdit, QPlainTextEdit, QPushButton,
    QVBoxLayout, QWidget,
)

from mcs.gui import theme

MAX_LINES = 5000

_SEV_COLORS = {
    "error": theme.ERR,
    "warn": theme.WARN,
    "info": theme.TEXT,
    "debug": theme.TEXT_DIM,
}

_RE_SEV = re.compile(r"\[(INFO|WARN|ERROR|FATAL|DEBUG)\]")

# Category filters keyed to master_control's actual logger output:
#   "State: … Target: … Thrust: …", "Manual target coordinates: …",
#   "Pinger coordinates: …", "Computed thrust: …"
_CATEGORIES: dict[str, re.Pattern | None] = {
    "Everything": None,
    "Targets only": re.compile(r"Target|target coordinates|Pinger coordinates",
                               re.IGNORECASE),
    "Thrust only": re.compile(r"Thrust", re.IGNORECASE),
    "Warnings & errors": re.compile(r"\[(WARN|ERROR|FATAL)\]|Traceback|Exception"),
}


@dataclass
class _Entry:
    text: str
    severity: str  # 'info' | 'warn' | 'error' | 'debug'


def _classify(line: str) -> str:
    m = _RE_SEV.search(line)
    if m:
        token = m.group(1)
        return {"WARN": "warn", "ERROR": "error", "FATAL": "error",
                "DEBUG": "debug"}.get(token, "info")
    if "Traceback" in line or "Exception" in line:
        return "error"
    return "info"


class LaunchConsole(QWidget):
    """Console + filter toolbox for the launched ROS2 processes."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._history: deque[_Entry] = deque(maxlen=MAX_LINES)
        self._category: re.Pattern | None = None
        self._keyword: str = ""

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(3)

        # ---- Filter toolbox ------------------------------------------------
        bar = QHBoxLayout()
        bar.setSpacing(4)
        self._combo = QComboBox()
        self._combo.addItems(list(_CATEGORIES))
        self._combo.setToolTip(
            "Category filters matched against master_control's log lines "
            "(State/Target/Thrust, Manual target coordinates, "
            "Pinger coordinates, [WARN]/[ERROR]).")
        self._combo.currentTextChanged.connect(self._on_category)
        bar.addWidget(self._combo, stretch=0)
        self._keyword_edit = QLineEdit()
        self._keyword_edit.setPlaceholderText("keyword filter…")
        self._keyword_edit.setClearButtonEnabled(True)
        self._keyword_edit.textChanged.connect(self._on_keyword)
        bar.addWidget(self._keyword_edit, stretch=1)
        clear = QPushButton("Clear")
        clear.setToolTip("Clear the console display and its history.")
        clear.clicked.connect(self.clear)
        bar.addWidget(clear, stretch=0)
        layout.addLayout(bar)

        # ---- Console -------------------------------------------------------
        self._view = QPlainTextEdit()
        self._view.setReadOnly(True)
        self._view.setMaximumBlockCount(MAX_LINES)  # bounded for long runs
        self._view.setFont(QFont("DejaVu Sans Mono", 8))
        self._view.setStyleSheet(
            f"QPlainTextEdit {{ background: {theme.BG_DARK};"
            f" border: 1px solid {theme.BORDER}; }}")
        self._view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        layout.addWidget(self._view, stretch=1)

    # ------------------------------------------------------------------ feed
    def append_line(self, line: str) -> None:
        """Slot for SignalBus.launch_output (GUI thread, order-preserving)."""
        entry = _Entry(text=line, severity=_classify(line))
        self._history.append(entry)
        if self._passes(entry):
            self._render(entry)

    def clear(self) -> None:
        self._history.clear()
        self._view.clear()

    # ---------------------------------------------------------------- filters
    def _on_category(self, name: str) -> None:
        self._category = _CATEGORIES.get(name)
        self._refilter()

    def _on_keyword(self, text: str) -> None:
        self._keyword = text.strip().lower()
        self._refilter()

    def _passes(self, entry: _Entry) -> bool:
        if self._category is not None and not self._category.search(entry.text):
            return False
        if self._keyword and self._keyword not in entry.text.lower():
            return False
        return True

    def _refilter(self) -> None:
        """Re-render the (bounded) history under the current filters."""
        self._view.clear()
        for entry in self._history:
            if self._passes(entry):
                self._render(entry, force_no_scroll_check=True)
        self._scroll_to_bottom()

    # --------------------------------------------------------------- render
    def _render(self, entry: _Entry, force_no_scroll_check: bool = False) -> None:
        bar = self._view.verticalScrollBar()
        at_bottom = force_no_scroll_check or bar.value() >= bar.maximum() - 2
        color = _SEV_COLORS[entry.severity]
        self._view.appendHtml(
            f'<span style="color:{color}; white-space:pre;">'
            f"{html.escape(entry.text)}</span>")
        if at_bottom:
            self._scroll_to_bottom()

    def _scroll_to_bottom(self) -> None:
        bar = self._view.verticalScrollBar()
        bar.setValue(bar.maximum())
