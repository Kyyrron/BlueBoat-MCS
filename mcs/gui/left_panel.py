"""Left information panel.

Sections (all collapsible):
* **Guardian** — the link guardian's box (``mcs/gui/guardian_box.py``),
  hosted at the very top via :meth:`set_header`.
* **Robot** — live pose, GPS, heading, speed, controller, mission state,
  motor commands, battery charge, travelled distance, elapsed time.
* **Pinger** (2026-11-24) — the run's pinger mode and source, the applied
  filter, then the FILTERED estimate as ``/blueboat/pinger/estimate`` reports
  it (status coloured, position, distance, fix age and rate, 95 % radius,
  speed, % down-weighted / set aside over its window), then the LEGACY
  algorithm's position, body vector and distance, and the age of the last
  raw USBL packet. Every number is read from a topic, never derived.
* **Target** — robot↔pinger or robot↔path distance depending on mode.
* **ROS diagnostics** — per-topic rate / age / LED status.
* **RUN** (real-robot run session only) — pinned below the scroll area, so it
  sits under ROS DIAGNOSTICS and never scrolls away: START RUN / STOP RUN and
  the run's status (``mcs/gui/run_panel.py``, hosted via :meth:`set_footer`).

The map-layer checkboxes live in the right column. The panel is read-only;
all values are pulled from the :class:`~mcs.models.store.DataStore` on the
shared 10 Hz tick.
"""

from __future__ import annotations

import math

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGridLayout,
    QLabel,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from mcs.config.settings import AppConfig
from mcs.core import pinger_filter as pf
from mcs.core.clock import format_clock, station_clock
from mcs.gui import theme
from mcs.gui.widgets import CollapsibleSection, InfoGrid, StatusLed
from mcs.models.store import DataStore, TargetMode, finite_float

# Battery row thresholds: below 40 % is a "plan the return" warning, below
# 20 % an error. A row older than this is greyed rather than trusted.
_BATTERY_WARN_FRAC = 0.40
_BATTERY_ERR_FRAC = 0.20
_BATTERY_STALE_S = 15.0

#: Status colours of the filtered pinger estimate.
_STATUS_COLORS = {"tracking": theme.OK, "coasting": theme.WARN, "stale": theme.ERR,
                  "initialising": theme.TEXT_DIM, "no_data": theme.TEXT_DIM}
_MODE_TEXT = {pf.MODE_OFF: "off (path)", pf.MODE_FOLLOW: "follow pinger",
              pf.MODE_LOG: "log only"}
_SOURCE_TEXT = {pf.SOURCE_FILTER: "filtered", pf.SOURCE_LEGACY: "legacy (dragged)"}


class LeftPanel(QWidget):
    """Live-information sidebar."""

    def __init__(self, cfg: AppConfig, store: DataStore,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self._store = store
        self.setMinimumWidth(220)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(4)
        self._outer = outer
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        outer.addWidget(scroll)
        content = QWidget()
        scroll.setWidget(content)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)
        self._content_layout = layout

        # ---- Robot -----------------------------------------------------------
        sec_robot = CollapsibleSection("ROBOT")
        self.robot_grid = InfoGrid()
        for key in ("World (x, y)", "GPS", "Heading", "Speed", "Controller",
                    "Mission state", "Left motor", "Right motor", "Battery",
                    "Travelled", "Elapsed"):
            self.robot_grid.add_row(key)
        sec_robot.add_widget(self.robot_grid)
        layout.addWidget(sec_robot)

        # ---- Pinger -----------------------------------------------------------
        sec_pinger = CollapsibleSection("PINGER")
        self.pinger_grid = InfoGrid()
        for key in ("Mode", "Source", "Filter", "Status", "World (x, y)", "Distance",
                    "Fix age", "Fix rate", "95 % radius", "Speed", "Down-weighted",
                    "Set aside", "Legacy (x, y)", "Legacy body", "Legacy distance",
                    "Last update"):
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
            # Same source as the map glyph (compass first, else the absolute
            # ENU odom yaw) so the two can never disagree on screen.
            hdg = s.robot_true_heading()
            g.set("Heading", f"{_deg(hdg):6.1f}°" if hdg is not None else "—")
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
        g.set("Battery", *_battery_text(r))
        g.set("Travelled", f"{r.travelled_m:8.1f} m")
        elapsed = s.mission.elapsed_s()
        g.set("Elapsed", format_clock(elapsed) if elapsed is not None else "—")

        self._refresh_pinger()
        p = s.pinger
        pg = self.pinger_grid
        if p.last_raw_update_t is not None:
            age = station_clock().now() - p.last_raw_update_t
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

    def _refresh_pinger(self) -> None:
        """The PINGER section above "Last update" (read from the store)."""
        s = self._store
        m = s.mission
        pg = self.pinger_grid
        if not m.launch_running:
            pg.set("Mode", "—", theme.TEXT_DIM)
        elif m.replay:
            pg.set("Mode", "replay", theme.TEXT_DIM)
        elif m.simulation:
            pg.set("Mode", "n/a (simulation)", theme.TEXT_DIM)
        else:
            pg.set("Mode", _MODE_TEXT.get(m.pinger_mode, m.pinger_mode or "—"))

        est = s.pinger_est
        echo = s.pinger_filter.echo
        src = echo.get("pinger_source") or (est.source if est.payload else "")
        pg.set("Source", _SOURCE_TEXT.get(src, "—"))
        if echo:
            text = f"{echo.get('preset', '?')} · {str(echo.get('hash') or '')[:8]}"
            pf_state = s.pinger_filter
            if pf_state.sent_seq is None:
                color = None
            elif pf_state.applied():
                color = theme.OK
            elif pf_state.refusal() is not None:
                text, color = "refused — " + pf_state.refusal(), theme.ERR
            else:
                text, color = text + " (applying…)", theme.WARN
            pg.set("Filter", text, color)
        else:
            pg.set("Filter", "—", theme.TEXT_DIM)

        p = est.payload
        if est.t is None:
            pg.set("Status", "no estimate", theme.TEXT_DIM)
        else:
            age = station_clock().now() - est.t
            stale_s = self._cfg.diagnostics.warn_age_s.get(
                self._cfg.topics.pinger_estimate, 3.0)
            if age > stale_s:
                pg.set("Status", f"no estimate for {age:4.0f} s", theme.ERR)
            else:
                status = est.status
                event = p.get("event")
                pg.set("Status", status + (f" ({event})" if event else ""),
                       _STATUS_COLORS.get(status, theme.TEXT_DIM))
        if est.world is not None:
            pg.set("World (x, y)", f"{est.world[0]:+8.2f}, {est.world[1]:+8.2f} m")
        else:
            pg.set("World (x, y)", "no estimate" if est.t is None else "not initialised",
                   theme.TEXT_DIM)
        rng = finite_float(p.get("range_m"))
        if rng is None and est.source == pf.SOURCE_FILTER and est.body is not None \
                and any(abs(c) > 1e-9 for c in est.body):
            rng = math.hypot(est.body[0], est.body[1])
        pg.set("Distance", f"{rng:6.2f} m" if rng is not None else "—")
        fix_age, fix_rate, r95 = (finite_float(p.get(k)) for k in ("fix_age", "fix_rate", "r95"))
        pg.set("Fix age", f"{fix_age:5.1f} s" if fix_age is not None else "—")
        pg.set("Fix rate", f"{fix_rate:5.2f} /s" if fix_rate is not None else "—")
        pg.set("95 % radius", f"{r95:6.2f} m" if r95 is not None else "—")
        vx, vy = finite_float(p.get("vx")), finite_float(p.get("vy"))
        pg.set("Speed", f"{math.hypot(vx, vy):5.2f} m/s"
               if vx is not None and vy is not None and est.world is not None else "—")
        stats = p.get("stats") if isinstance(p.get("stats"), dict) else {}
        win = finite_float(stats.get("window_s"))
        suffix = f" / {win:.0f} s" if win is not None else ""
        for key, field in (("Down-weighted", "pct_downweighted"),
                           ("Set aside", "pct_set_aside")):
            v = finite_float(stats.get(field))
            pg.set(key, f"{v:5.1f} %{suffix}" if v is not None else "—")

        leg = s.pinger
        if leg.seen:
            if leg.world is not None:
                pg.set("Legacy (x, y)", f"{leg.world[0]:+8.2f}, {leg.world[1]:+8.2f} m")
            pg.set("Legacy body", f"{leg.body[0]:+7.2f}, {leg.body[1]:+7.2f} m")
            pg.set("Legacy distance",
                   f"{leg.distance_m:6.2f} m" if leg.distance_m else "—")
        else:
            pg.set("Legacy (x, y)", "not detected", theme.TEXT_DIM)
            pg.set("Legacy body", "—")
            pg.set("Legacy distance", "—")

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
        if s.session.active and ready == len(checks):
            label = "run active" if s.session.run_active else "idle, no run"
        suffix = " (sim)" if s.mission.simulation else ""
        color = theme.OK if ready == len(checks) else theme.WARN
        return f"{label} ({ready}/{len(checks)}){suffix}", color

    # ------------------------------------------------------------ diagnostics
    def on_topic_stats(self, stats: dict) -> None:
        for topic in sorted(stats):
            if topic not in self._diag_rows:
                self._add_diag_row(topic)
            led, _name_label, value_label = self._diag_rows[topic]
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

    def set_header(self, widget: QWidget) -> None:
        """Put *widget* at the very top of the panel (the GUARDIAN box)."""
        self._content_layout.insertWidget(0, widget)

    def set_footer(self, widget: QWidget) -> None:
        """Pin *widget* below the scroll area (the run session's RUN box)."""
        self._outer.addWidget(widget)


def _battery_text(r) -> tuple[str, str]:
    """Charge fraction (and pack voltage when the FCU reports one).

    ``BatteryState.percentage`` is a 0..1 fraction, hence the x100. The row
    greys out once the stream goes quiet rather than showing a stale charge
    as if it were live -- /mavros/battery only exists on the real boat, so
    in simulation it stays "no data" for the whole run.
    """
    if r.battery_pct is None and r.battery_v is None:
        return "no data", theme.TEXT_DIM
    parts = []
    if r.battery_pct is not None:
        parts.append(f"{r.battery_pct * 100.0:5.1f} %")
    if r.battery_v is not None:
        parts.append(f"{r.battery_v:5.2f} V")
    text = "  ".join(parts)
    if r.battery_t is None or (station_clock().now() - r.battery_t) > _BATTERY_STALE_S:
        return text, theme.TEXT_DIM
    pct = r.battery_pct
    if pct is None:
        return text, theme.OK
    if pct >= _BATTERY_WARN_FRAC:
        return text, theme.OK
    return text, theme.WARN if pct >= _BATTERY_ERR_FRAC else theme.ERR


def _deg(rad: float) -> float:
    return math.degrees(rad) % 360.0
