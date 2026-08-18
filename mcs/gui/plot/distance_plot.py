"""Live robot ↔ target distance plot.

A lightweight custom-painted time-series plot (no plotting dependency).
Renders the windowed distance series from the store's ``target_dist_hist``,
with auto-scaled axes, a hover cursor and the current value highlighted.
The displayed window follows the mission timeline range slider; recording
continues regardless of the window shown.
"""

from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QSizePolicy, QWidget

from mcs.gui import theme
from mcs.models.store import DataStore

_MARGIN_L, _MARGIN_R, _MARGIN_T, _MARGIN_B = 44, 10, 8, 22


class DistancePlot(QWidget):
    """Distance-to-target vs experiment time."""

    def __init__(self, store: DataStore, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._store = store
        self._window: tuple[float, float] = (0.0, 60.0)
        self._live = True
        self._title = "robot ↔ target distance"
        self.setMinimumHeight(160)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_time_window(self, rel_t0: float, rel_t1: float, live: bool) -> None:
        self._window = (rel_t0, rel_t1)
        self._live = live

    def set_title(self, title: str) -> None:
        self._title = title

    def refresh(self) -> None:
        self.update()

    # -------------------------------------------------------------- painting
    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor(theme.BG_PANEL))
        area = QRectF(_MARGIN_L, _MARGIN_T,
                      self.width() - _MARGIN_L - _MARGIN_R,
                      self.height() - _MARGIN_T - _MARGIN_B)
        p.setPen(QPen(QColor(theme.BORDER), 1))
        p.drawRect(area)

        store = self._store
        rel0, rel1 = self._window
        duration = store.recorded_duration()
        if self._live:
            rel1 = max(duration, rel1)
        t0, t1 = store.t0 + rel0, store.t0 + rel1
        ts, vs = store.target_dist_hist.decimated_window(t0, t1, 2000)

        # Title
        p.setPen(QColor(theme.TEXT_DIM))
        p.setFont(QFont("DejaVu Sans", 8))
        p.drawText(QPointF(area.left(), area.top() - 1), self._title)

        if len(ts) < 2 or rel1 - rel0 <= 0:
            p.setPen(QColor(theme.TEXT_DIM))
            p.drawText(area, Qt.AlignmentFlag.AlignCenter, "no target data")
            return

        v = vs[:, 0]
        v_max = max(float(v.max()) * 1.1, 1.0)

        def xmap(t: float) -> float:
            return area.left() + (t - t0) / (t1 - t0) * area.width()

        def ymap(val: float) -> float:
            return area.bottom() - (val / v_max) * area.height()

        # Grid + y labels (1/2/5 progression)
        step = _nice_step(v_max / 4)
        p.setFont(QFont("DejaVu Sans Mono", 7))
        y_val = 0.0
        while y_val <= v_max + 1e-9:
            y = ymap(y_val)
            p.setPen(QPen(QColor(255, 255, 255, 18), 1))
            p.drawLine(QPointF(area.left(), y), QPointF(area.right(), y))
            p.setPen(QColor(theme.TEXT_DIM))
            p.drawText(QRectF(0, y - 7, _MARGIN_L - 6, 14),
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       f"{y_val:.0f}m" if step >= 1 else f"{y_val:.1f}m")
            y_val += step

        # x labels
        for frac in (0.0, 0.5, 1.0):
            t_rel = rel0 + frac * (rel1 - rel0)
            x = area.left() + frac * area.width()
            p.setPen(QColor(theme.TEXT_DIM))
            p.drawText(QRectF(x - 40, area.bottom() + 4, 80, 16),
                       Qt.AlignmentFlag.AlignCenter, _fmt_t(t_rel))

        # Series
        path = QPainterPath()
        path.moveTo(xmap(float(ts[0])), ymap(float(v[0])))
        for i in range(1, len(ts)):
            path.lineTo(xmap(float(ts[i])), ymap(float(v[i])))
        pen = QPen(QColor(theme.ACCENT), 1.6)
        p.setPen(pen)
        p.drawPath(path)

        # Current value dot + label (only when live edge visible)
        if self._live:
            p.setBrush(QColor(theme.C_TARGET_LINE))
            p.setPen(Qt.PenStyle.NoPen)
            x_last, y_last = xmap(float(ts[-1])), ymap(float(v[-1]))
            p.drawEllipse(QPointF(x_last, y_last), 3.5, 3.5)
            p.setPen(QColor(theme.TEXT))
            p.setFont(QFont("DejaVu Sans Mono", 8, QFont.Weight.Bold))
            p.drawText(QPointF(min(x_last + 6, area.right() - 52), y_last - 6),
                       f"{v[-1]:.2f} m")


def _nice_step(raw: float) -> float:
    if raw <= 0:
        return 1.0
    exp = math.floor(math.log10(raw))
    base = raw / 10 ** exp
    for mult in (1.0, 2.0, 5.0, 10.0):
        if base <= mult:
            return mult * 10 ** exp
    return 10 ** (exp + 1)


def _fmt_t(seconds: float) -> str:
    seconds = max(0.0, seconds)
    m, s = divmod(int(seconds), 60)
    return f"{m:d}:{s:02d}"
