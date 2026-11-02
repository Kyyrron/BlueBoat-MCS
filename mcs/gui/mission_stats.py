"""Floating box for mission statistics."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QVBoxLayout, QWidget

from mcs.core.clock import format_clock
from mcs.gui import theme
from mcs.gui.widgets import CollapsibleSection, InfoGrid
from mcs.models.store import DataStore


class FloatingStatsBox(QWidget):
    """Floating widget that displays mission statistics overlay."""

    def __init__(self, store: DataStore, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._store = store

        self.setObjectName("floatingStatsBox")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(theme.floating_box_style("floatingStatsBox"))
        self.setFixedWidth(260)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        # Never stretch vertically: keep the minimum height.
        layout.setSizeConstraint(QVBoxLayout.SizeConstraint.SetFixedSize)

        sec_stats = CollapsibleSection("MISSION STATISTICS")
        self.stats_grid = InfoGrid()
        for key in ("Duration", "Travelled", "Avg speed", "Max speed", "Controller"):
            self.stats_grid.add_row(key)
        sec_stats.add_widget(self.stats_grid)

        layout.addWidget(sec_stats)

    def refresh_stats(self, low: float, high: float, live: bool = False) -> None:
        st = self._store.statistics(low, high)
        g = self.stats_grid
        g.set("Duration", format_clock(st.duration_s))
        g.set("Travelled", f"{st.travelled_m:8.1f} m")
        g.set("Avg speed", f"{st.avg_speed:5.2f} m/s")
        g.set("Max speed", f"{st.max_speed:5.2f} m/s")
        ctrl = self._store.mission.controller_type or "—"
        g.set("Controller", ctrl if self._store.mission.launch_running else "—")
