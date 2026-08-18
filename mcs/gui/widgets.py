"""Small reusable widgets shared across the panels.

* :class:`StatusLed` — green / orange / red / grey indicator dot.
* :class:`InfoGrid` — aligned key/value read-out grid with monospace values.
* :class:`CollapsibleSection` — titled, collapsible container for the panels.
* :class:`RangeSlider` — dual-handle slider used by the mission timeline.
"""

from __future__ import annotations

from PySide6.QtCore import QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QMouseEvent, QPainter, QPen
from PySide6.QtWidgets import (
    QFrame, QGridLayout, QLabel, QSizePolicy, QToolButton, QVBoxLayout, QWidget,
)

from mcs.gui import theme

_LED_COLORS = {
    "ok": QColor(theme.OK),
    "warn": QColor(theme.WARN),
    "stale": QColor(theme.ERR),
    "never": QColor("#555c64"),
}


class StatusLed(QWidget):
    """A small round status indicator."""

    def __init__(self, diameter: int = 10, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._d = diameter
        self._status = "never"
        self.setFixedSize(diameter + 2, diameter + 2)

    def set_status(self, status: str) -> None:
        if status != self._status:
            self._status = status
            self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        color = _LED_COLORS.get(self._status, _LED_COLORS["never"])
        p.setBrush(QBrush(color))
        p.setPen(QPen(color.darker(150), 1))
        p.drawEllipse(1, 1, self._d, self._d)


class InfoGrid(QWidget):
    """Two-column key/value grid; values are monospace and right-aligned."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._grid = QGridLayout(self)
        self._grid.setContentsMargins(2, 2, 2, 2)
        self._grid.setHorizontalSpacing(10)
        self._grid.setVerticalSpacing(2)
        self._values: dict[str, QLabel] = {}

    def add_row(self, key: str, initial: str = "—") -> None:
        row = self._grid.rowCount()
        klabel = QLabel(key)
        klabel.setStyleSheet(f"color: {theme.TEXT_DIM};")
        vlabel = QLabel(initial)
        vlabel.setObjectName("valueLabel")
        vlabel.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        vlabel.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self._grid.addWidget(klabel, row, 0)
        self._grid.addWidget(vlabel, row, 1)
        self._values[key] = vlabel

    def set(self, key: str, value: str, color: str | None = None) -> None:
        label = self._values.get(key)
        if label is None:
            return
        if label.text() != value:
            label.setText(value)
        style = "font-family: 'DejaVu Sans Mono', monospace;"
        if color:
            style += f" color: {color};"
        if label.styleSheet() != style:
            label.setStyleSheet(style)


class CollapsibleSection(QFrame):
    """A titled section with a toggle arrow, used to structure the side panels."""

    def __init__(self, title: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("panel")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 6)
        layout.setSpacing(2)

        self._button = QToolButton()
        self._button.setText(title)
        self._button.setCheckable(True)
        self._button.setChecked(True)
        self._button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self._button.setArrowType(Qt.ArrowType.DownArrow)
        self._button.setStyleSheet(
            f"QToolButton {{ border: none; font-weight: bold; color: {theme.TEXT_DIM};"
            f" letter-spacing: 1px; }}"
        )
        self._button.toggled.connect(self._on_toggled)
        layout.addWidget(self._button)

        self._body = QWidget()
        self._body_layout = QVBoxLayout(self._body)
        self._body_layout.setContentsMargins(0, 0, 0, 0)
        self._body_layout.setSpacing(4)
        layout.addWidget(self._body)

    def add_widget(self, widget: QWidget) -> None:
        self._body_layout.addWidget(widget)

    def _on_toggled(self, checked: bool) -> None:
        self._body.setVisible(checked)
        self._button.setArrowType(
            Qt.ArrowType.DownArrow if checked else Qt.ArrowType.RightArrow
        )


class RangeSlider(QWidget):
    """Horizontal dual-handle slider selecting a [low, high] sub-range.

    Values are floats in [minimum, maximum].  Emits :attr:`range_changed`
    while dragging.  Used by the mission timeline (start / end time).
    """

    range_changed = Signal(float, float)

    _HANDLE_W = 10.0

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._min = 0.0
        self._max = 1.0
        self._low = 0.0
        self._high = 1.0
        self._drag: str | None = None  # 'low' | 'high' | None
        self.setMinimumHeight(24)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    # ------------------------------------------------------------------ API
    def set_maximum(self, maximum: float, keep_high_at_end: bool) -> None:
        """Extend the range; optionally keep the high handle glued to the end."""
        at_end = keep_high_at_end and self._high >= self._max - 1e-9
        self._max = max(maximum, self._min + 1e-6)
        if at_end:
            self._high = self._max
        self._high = min(self._high, self._max)
        self._low = min(self._low, self._high)
        self.update()

    def values(self) -> tuple[float, float]:
        return self._low, self._high

    def set_values(self, low: float, high: float) -> None:
        self._low = max(self._min, min(low, high))
        self._high = min(self._max, max(low, high))
        self.update()
        self.range_changed.emit(self._low, self._high)

    def high_at_end(self) -> bool:
        return self._high >= self._max - 1e-9

    # ------------------------------------------------------------- painting
    def _x_of(self, value: float) -> float:
        w = self.width() - 2 * self._HANDLE_W
        frac = 0.0 if self._max <= self._min else (value - self._min) / (self._max - self._min)
        return self._HANDLE_W + frac * w

    def _value_of(self, x: float) -> float:
        w = self.width() - 2 * self._HANDLE_W
        frac = 0.0 if w <= 0 else (x - self._HANDLE_W) / w
        return self._min + max(0.0, min(1.0, frac)) * (self._max - self._min)

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        cy = self.height() / 2
        # Track
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(theme.BG_RAISED))
        p.drawRoundedRect(QRectF(self._HANDLE_W, cy - 3, self.width() - 2 * self._HANDLE_W, 6), 3, 3)
        # Selected range
        x0, x1 = self._x_of(self._low), self._x_of(self._high)
        p.setBrush(QColor(theme.ACCENT_DIM))
        p.drawRoundedRect(QRectF(x0, cy - 3, max(2.0, x1 - x0), 6), 3, 3)
        # Handles
        for x in (x0, x1):
            p.setBrush(QColor(theme.ACCENT))
            p.setPen(QPen(QColor(theme.BORDER), 1))
            p.drawRoundedRect(QRectF(x - self._HANDLE_W / 2, cy - 8, self._HANDLE_W, 16), 3, 3)

    # ---------------------------------------------------------------- mouse
    def mousePressEvent(self, event: QMouseEvent) -> None:
        x = event.position().x()
        d_low = abs(x - self._x_of(self._low))
        d_high = abs(x - self._x_of(self._high))
        self._drag = "low" if d_low <= d_high else "high"
        self.mouseMoveEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self._drag is None:
            return
        value = self._value_of(event.position().x())
        if self._drag == "low":
            self._low = min(value, self._high)
        else:
            self._high = max(value, self._low)
        self.update()
        self.range_changed.emit(self._low, self._high)

    def mouseReleaseEvent(self, _event: QMouseEvent) -> None:
        self._drag = None
