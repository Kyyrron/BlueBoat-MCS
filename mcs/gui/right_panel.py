"""Right monitoring panel.

Restructured as a vertical splitter so monitoring and the launch console
share the column without crowding each other:

* **Top part** (scrollable stack)
  * **Map tools** — Zoom +, Zoom −, and *Center Robot* (a one-shot recenter;
    the camera stays completely free afterwards — never a follow mode).
  * **Live distance plot** — robot ↔ current target distance vs time.
  * **Mission timeline** — dual-handle range slider selecting the displayed
    time window (map trajectories + plot). With the high handle at the end
    the display is *live*; moving it back freezes the display while
    recording continues underneath.
  * **Mission statistics** — duration, travelled distance, average / max
    speed and active controller over the selected window.
* **Bottom part** — the **launch console** (:class:`~mcs.gui.console.
  LaunchConsole`), dedicated exclusively to the launched ROS2 processes'
  stdout/stderr, with severity coloring and its keyword filter toolbox.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QHBoxLayout, QLabel, QPushButton, QScrollArea, QSplitter, QVBoxLayout,
    QWidget,
)

from mcs.config.settings import AppConfig
from mcs.gui import theme
from mcs.gui.console import LaunchConsole
from mcs.gui.plot.distance_plot import DistancePlot
from mcs.gui.widgets import CollapsibleSection, InfoGrid, RangeSlider
from mcs.models.store import DataStore, TargetMode


class RightPanel(QWidget):
    """Monitoring sidebar: map tools, plot, timeline, statistics, console."""

    #: rel_t0, rel_t1, live
    time_window_changed = Signal(float, float, bool)
    zoom_in_requested = Signal()
    zoom_out_requested = Signal()
    center_robot_requested = Signal()
    clear_paths_requested = Signal()

    def __init__(self, cfg: AppConfig, store: DataStore,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self._store = store
        # Freely resizable via the splitter: a small minimum only, no
        # maximum (a max-width cap prevented widening the panel at will).
        self.setMinimumWidth(260)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.setSpacing(0)
        splitter = QSplitter(Qt.Orientation.Vertical)
        outer.addWidget(splitter)

        # ================================================ top: monitoring stack
        top_scroll = QScrollArea()
        top_scroll.setWidgetResizable(True)
        top = QWidget()
        top_scroll.setWidget(top)
        layout = QVBoxLayout(top)
        layout.setContentsMargins(0, 0, 0, 4)
        layout.setSpacing(6)

        # ---- Map tools ------------------------------------------------------
        sec_tools = CollapsibleSection("MAP TOOLS")
        tools_row = QHBoxLayout()
        tools_row.setSpacing(6)
        zoom_in = QPushButton("Zoom +")
        zoom_in.clicked.connect(self.zoom_in_requested.emit)
        zoom_out = QPushButton("Zoom −")
        zoom_out.clicked.connect(self.zoom_out_requested.emit)
        center = QPushButton("Center Robot")
        center.setToolTip(
            "Recenter the view on the robot once; the camera then remains "
            "completely free (this is not a follow mode).")
        center.clicked.connect(self.center_robot_requested.emit)
        clear_paths = QPushButton("Clear Paths")
        clear_paths.setToolTip("Clear the robot and pinger trails drawn on "
                               "the map (live states are kept).")
        clear_paths.clicked.connect(self.clear_paths_requested.emit)
        for b in (zoom_in, zoom_out, center, clear_paths):
            tools_row.addWidget(b)
        tools_widget = QWidget()
        tools_widget.setLayout(tools_row)
        sec_tools.add_widget(tools_widget)
        layout.addWidget(sec_tools)

        # ---- Plot -----------------------------------------------------------
        sec_plot = CollapsibleSection("LIVE DISTANCE")
        self.plot = DistancePlot(store)
        sec_plot.add_widget(self.plot)
        layout.addWidget(sec_plot, stretch=2)

        # ---- Timeline --------------------------------------------------------
        sec_tl = CollapsibleSection("MISSION TIMELINE")
        self.slider = RangeSlider()
        self.slider.range_changed.connect(self._on_slider)
        sec_tl.add_widget(self.slider)
        row = QHBoxLayout()
        self._window_label = QLabel("0:00 → live")
        self._window_label.setObjectName("valueLabel")
        row.addWidget(self._window_label)
        row.addStretch(1)
        self._live_button = QPushButton("Go Live")
        self._live_button.setToolTip(
            "Snap the window back to the live edge of the recording.")
        self._live_button.clicked.connect(self._go_live)
        row.addWidget(self._live_button)
        row_widget = QWidget()
        row_widget.setLayout(row)
        sec_tl.add_widget(row_widget)
        layout.addWidget(sec_tl)

        layout.addStretch(1)

        splitter.addWidget(top_scroll)

        # ================================================ bottom: launch console
        sec_console = CollapsibleSection("LAUNCH CONSOLE")
        self.console = LaunchConsole()
        self.console.setMinimumHeight(120)
        sec_console.add_widget(self.console)
        splitter.addWidget(sec_console)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([560, 320])

    # ------------------------------------------------------------------ ticks
    def refresh(self) -> None:
        duration = self._store.recorded_duration()
        self.slider.set_maximum(max(duration, 10.0), keep_high_at_end=True)
        low, high = self.slider.values()
        live = self.slider.high_at_end()
        self._emit_window(low, high, live)
        self.plot.set_time_window(low, high, live)
        title = {
            TargetMode.PINGER: "robot ↔ pinger distance",
            TargetMode.MANUAL: "robot ↔ manual target distance",
            TargetMode.PATH: "robot ↔ path target distance",
            TargetMode.NONE: "robot ↔ target distance",
        }[self._store.mission.target_mode]
        self.plot.set_title(title)
        self.plot.refresh()

    # ---------------------------------------------------------------- slider
    def _on_slider(self, low: float, high: float) -> None:
        self._emit_window(low, high, self.slider.high_at_end())

    def _emit_window(self, low: float, high: float, live: bool) -> None:
        end = "live" if live else _fmt_hms(high)
        self._window_label.setText(f"{_fmt_hms(low)} → {end}")
        self._live_button.setEnabled(not live)
        self.time_window_changed.emit(low, high, live)

    def _go_live(self) -> None:
        low, _ = self.slider.values()
        duration = max(self._store.recorded_duration(), 10.0)
        self.slider.set_maximum(duration, keep_high_at_end=False)
        self.slider.set_values(low, duration)


def _fmt_hms(seconds: float) -> str:
    seconds = int(max(0, seconds))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    return f"{h:d}:{m:02d}:{s:02d}" if h else f"{m:d}:{s:02d}"
