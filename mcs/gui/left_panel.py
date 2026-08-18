"""Left information panel.

Sections (all collapsible):
* **Layers** — visibility checkboxes for every map layer.
* **Robot** — live pose, GPS, heading, speed, controller, mission state,
  motor commands, travelled distance, elapsed time.
* **Pinger** — world & robot-frame coordinates, live distance, last update.
* **Target** — robot↔pinger or robot↔path distance depending on mode.
* **ROS diagnostics** — per-topic rate / age / LED status.

The panel is read-only except for the checkboxes; all values are pulled
from the :class:`~mcs.models.store.DataStore` on the shared 10 Hz tick.
"""

from __future__ import annotations

import time

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox, QGridLayout, QHBoxLayout, QLabel, QScrollArea, QVBoxLayout,
    QWidget,
)

from mcs.config.settings import AppConfig
from mcs.gui import theme
from mcs.gui.widgets import CollapsibleSection, InfoGrid, StatusLed
from mcs.models.store import DataStore, TargetMode

_LAYERS: list[tuple[str, str, bool]] = [
    ("satellite", "Satellite map layer", False),
    ("robot_track", "Robot trajectory", True),
    ("mission_path", "Published mission path", True),
    ("pinger", "USBL pinger position", True),
    ("pinger_track", "USBL pinger trajectory", True),
    ("target_line", "Robot → target line", True),
    ("heading", "Robot heading arrow", False),
    ("grid", "World grid", True),
]


class LeftPanel(QWidget):
    """Live-information sidebar."""

    layer_toggled = Signal(str, bool)

    def __init__(self, cfg: AppConfig, store: DataStore,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self._store = store
        # Freely resizable via the splitter: a small minimum only, no
        # maximum (a max-width cap prevented widening the panel at will).
        self.setMinimumWidth(220)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        outer.addWidget(scroll)
        content = QWidget()
        scroll.setWidget(content)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # ---- Layers ---------------------------------------------------------
        sec_layers = CollapsibleSection("LAYERS")
        self._layer_boxes: dict[str, QCheckBox] = {}
        for key, label, default in _LAYERS:
            box = QCheckBox(label)
            box.setChecked(default)
            box.toggled.connect(lambda on, k=key: self.layer_toggled.emit(k, on))
            self._layer_boxes[key] = box
            sec_layers.add_widget(box)
        self._layer_boxes["satellite"].setEnabled(False)
        self._layer_boxes["satellite"].setToolTip(
            "Enabled once the odom ↔ GPS georeference is established "
            "(requires GPS fix and a few metres of motion).")
        layout.addWidget(sec_layers)

        # ---- Robot -----------------------------------------------------------
        sec_robot = CollapsibleSection("ROBOT")
        self.robot_grid = InfoGrid()
        for key in ("World (x, y)", "GPS", "Heading", "Speed", "Controller",
                    "Mission state", "Left motor", "Right motor",
                    "Travelled", "Elapsed"):
            self.robot_grid.add_row(key)
        sec_robot.add_widget(self.robot_grid)
        layout.addWidget(sec_robot)

        # ---- Pinger -----------------------------------------------------------
        sec_pinger = CollapsibleSection("PINGER")
        self.pinger_grid = InfoGrid()
        for key in ("World (x, y)", "Robot frame", "Distance", "Last update"):
            self.pinger_grid.add_row(key)
        sec_pinger.add_widget(self.pinger_grid)
        layout.addWidget(sec_pinger)

        # ---- Target -----------------------------------------------------------
        sec_target = CollapsibleSection("TARGET")
        self.target_grid = InfoGrid()
        for key in ("Mode", "Distance"):
            self.target_grid.add_row(key)
        sec_target.add_widget(self.target_grid)
        layout.addWidget(sec_target)

        # ---- Diagnostics --------------------------------------------------------
        sec_diag = CollapsibleSection("ROS DIAGNOSTICS")
        diag_widget = QWidget()
        self._diag_grid = QGridLayout(diag_widget)
        self._diag_grid.setContentsMargins(2, 2, 2, 2)
        self._diag_grid.setHorizontalSpacing(8)
        self._diag_grid.setVerticalSpacing(3)
        self._diag_rows: dict[str, tuple[StatusLed, QLabel, QLabel]] = {}
        sec_diag.add_widget(diag_widget)
        layout.addWidget(sec_diag)

        layout.addStretch(1)

    # ================================================================ updates
    def refresh(self) -> None:
        """Shared 10 Hz UI tick — pull values from the store."""
        s = self._store
        r = s.robot
        g = self.robot_grid
        if r.has_odom:
            g.set("World (x, y)", f"{r.x:+8.2f}, {r.y:+8.2f} m")
            g.set("Heading", f"{_deg(r.yaw):6.1f}°")
            g.set("Speed", f"{r.speed:5.2f} m/s")
        else:
            g.set("World (x, y)", "no odom")
        if r.lat is not None:
            g.set("GPS", f"{r.lat:.6f}, {r.lon:.6f}")
        else:
            g.set("GPS", "no fix", theme.TEXT_DIM)
        launcher_ctrl = s.mission.controller_type or "(manual)"
        g.set("Controller", launcher_ctrl if s.mission.launch_running else "—")
        g.set("Mission state", *self._mission_state_text())
        g.set("Left motor", f"{r.thrust_left:+6.2f} N")
        g.set("Right motor", f"{r.thrust_right:+6.2f} N")
        g.set("Travelled", f"{r.travelled_m:8.1f} m")
        elapsed = s.mission.elapsed_s()
        g.set("Elapsed", _fmt_hms(elapsed) if elapsed is not None else "—")

        p = s.pinger
        pg = self.pinger_grid
        if p.seen:
            if p.world is not None:
                pg.set("World (x, y)", f"{p.world[0]:+8.2f}, {p.world[1]:+8.2f} m")
            pg.set("Robot frame", f"{p.body[0]:+7.2f}, {p.body[1]:+7.2f} m")
            pg.set("Distance", f"{p.distance_m:6.2f} m" if p.distance_m else "—")
        else:
            pg.set("World (x, y)", "not detected", theme.TEXT_DIM)
            pg.set("Robot frame", "—")
            pg.set("Distance", "—")
        if p.last_raw_update_t is not None:
            age = time.monotonic() - p.last_raw_update_t
            color = theme.OK if age < 3 else (theme.WARN if age < 10 else theme.ERR)
            pg.set("Last update", f"{age:5.1f} s ago", color)
        else:
            pg.set("Last update", "never", theme.TEXT_DIM)

        mode = s.mission.target_mode
        labels = {TargetMode.NONE: ("no active target", theme.TEXT_DIM),
                  TargetMode.PATH: ("path following", theme.OK),
                  TargetMode.PINGER: ("pinger homing", theme.C_PINGER.name()),
                  TargetMode.MANUAL: ("MANUAL TARGET", theme.C_MANUAL_TARGET.name())}
        text, color = labels[mode]
        self.target_grid.set("Mode", text, color)
        d = s.active_target_distance()
        self.target_grid.set("Distance", f"{d:6.2f} m" if d is not None else "—")

    def _mission_state_text(self) -> tuple[str, str]:
        s = self._store
        if not s.mission.launch_running:
            return "idle", theme.TEXT_DIM
        checks = [s.robot.has_odom]
        if not s.mission.simulation:  # Sim_launch.py has no MAVROS/FCU
            checks.append(s.robot.fcu_connected)
        if s.mission.controller_type:
            checks.append(s.robot.controller_ready)
        ready = sum(checks)
        label = "operational" if ready == len(checks) else "starting"
        suffix = " (sim)" if s.mission.simulation else ""
        color = theme.OK if ready == len(checks) else theme.WARN
        return f"{label} ({ready}/{len(checks)}){suffix}", color

    # ------------------------------------------------------------ diagnostics
    def on_topic_stats(self, stats: dict) -> None:
        for topic in sorted(stats):
            if topic not in self._diag_rows:
                self._add_diag_row(topic)
            led, name_label, value_label = self._diag_rows[topic]
            st = stats[topic]
            led.set_status(st["status"])
            rate = st["rate"]
            age = st["age"]
            expected = st["expected"]
            rate_txt = f"{rate:5.1f} Hz" + (f"/{expected:.0f}" if expected else "")
            age_txt = f"{age:5.1f}s" if age is not None else "  n/a"
            value_label.setText(f"{rate_txt}  {age_txt}")
            colors = {"ok": theme.OK, "warn": theme.WARN,
                      "stale": theme.ERR, "never": theme.TEXT_DIM}
            value_label.setStyleSheet(
                f"color: {colors[st['status']]};"
                "font-family: 'DejaVu Sans Mono', monospace; font-size: 10px;")

    def _add_diag_row(self, topic: str) -> None:
        row = len(self._diag_rows)
        led = StatusLed()
        name = QLabel(topic)
        name.setStyleSheet(f"color: {theme.TEXT_DIM}; font-size: 10px;")
        name.setToolTip(topic)
        value = QLabel("—")
        value.setAlignment(Qt.AlignmentFlag.AlignRight)
        self._diag_grid.addWidget(led, row, 0)
        self._diag_grid.addWidget(name, row, 1)
        self._diag_grid.addWidget(value, row, 2)
        self._diag_rows[topic] = (led, name, value)

    def set_satellite_available(self, available: bool) -> None:
        box = self._layer_boxes["satellite"]
        if box.isEnabled() != available:
            box.setEnabled(available)


def _deg(rad: float) -> float:
    import math
    return math.degrees(rad) % 360.0


def _fmt_hms(seconds: float) -> str:
    seconds = int(max(0, seconds))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    return f"{h:d}:{m:02d}:{s:02d}" if h else f"{m:d}:{s:02d}"
