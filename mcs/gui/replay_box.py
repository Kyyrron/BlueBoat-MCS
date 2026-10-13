"""Floating REPLAY box — the Gazebo replay of a recorded run (v1.1).

Same look and map-view parenting as the SEA STATE box, and the same place
(top-left of the map): a replay has no sea state, so the two never show
together. Visible only while a "From a ros2bag" launch runs.

Read-out from ``/replay/state`` (``DataStore.replay``, a
:class:`~mcs.core.replay.ReplayReadout`): the recorded run, the replay time
against the run's duration, and the status (playing at n×, paused,
finished, waiting for Gazebo, stale). Controls publish on ``/replay/command``
through ``CommandCenter.send_replay_command``: Pause / Resume and the
playback rate. The station never moves the replayed boat itself.
"""

from __future__ import annotations

import time
from pathlib import Path

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from mcs.config.settings import AppConfig
from mcs.core.replay import format_clock
from mcs.gui import theme
from mcs.gui.widgets import CollapsibleSection
from mcs.models.store import DataStore

_BOX_W = 300
_MAX_CHARS = 30


class FloatingReplayBox(QWidget):
    """Floating widget that displays and controls a bag replay."""

    pause_requested = Signal(bool)      # True = pause, False = resume
    rate_requested = Signal(float)

    _ROWS = ("Run", "Time", "Status")

    def __init__(self, cfg: AppConfig, store: DataStore,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self._store = store

        self.setObjectName("floatingReplayBox")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet("""
            QWidget#floatingReplayBox {
                background-color: rgba(45, 45, 45, 230);
                border: 1px solid #555;
                border-radius: 6px;
            }
        """)
        self.setFixedWidth(_BOX_W)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        sec = CollapsibleSection("REPLAY (recorded run)")

        grid_host = QWidget()
        grid = QGridLayout(grid_host)
        grid.setContentsMargins(2, 2, 2, 2)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(4)
        self._values: dict[str, QLabel] = {}
        for row, key in enumerate(self._ROWS):
            k = QLabel(key)
            k.setStyleSheet(f"color: {theme.TEXT_DIM};")
            v = QLabel("—")
            v.setObjectName("valueLabel")
            v.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            grid.addWidget(k, row, 0)
            grid.addWidget(v, row, 1)
            self._values[key] = v
        sec.add_widget(grid_host)

        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(8)
        sec.add_widget(self.progress)

        controls = QWidget()
        row = QHBoxLayout(controls)
        row.setContentsMargins(0, 0, 0, 0)
        self.pause_btn = QPushButton("Pause")
        self.pause_btn.setToolTip("Pause / resume the replay (the boat holds "
                                  "its pose while paused).")
        self.pause_btn.clicked.connect(self._on_pause_clicked)
        self.rate = QComboBox()
        for r in cfg.replay.rates:
            self.rate.addItem(f"{r:g}×", float(r))
        self.rate.setToolTip("Playback rate")
        self.rate.activated.connect(self._on_rate_activated)
        row.addWidget(self.pause_btn, 1)
        row.addWidget(self.rate)
        sec.add_widget(controls)

        layout.addWidget(sec)
        self.setVisible(False)

    # ---------------------------------------------------------------- actions
    def _on_pause_clicked(self) -> None:
        r = self._store.replay
        self.pause_requested.emit(not (r is not None and r.paused))

    def _on_rate_activated(self, index: int) -> None:
        rate = self.rate.itemData(index)
        if rate:
            self.rate_requested.emit(float(rate))

    # ---------------------------------------------------------------- refresh
    def _set(self, key: str, text: str, color: str | None = None) -> None:
        label = self._values[key]
        shown = text if len(text) <= _MAX_CHARS else (
            text[:_MAX_CHARS // 2 - 1] + "…" + text[-(_MAX_CHARS // 2):])
        if label.text() != shown:
            label.setText(shown)
            label.setToolTip(text)
        style = f"color: {color};" if color else ""
        if label.styleSheet() != style:
            label.setStyleSheet(style)

    def refresh(self) -> None:
        """Pulled on the shared UI tick; hidden outside a replay launch."""
        s = self._store
        show = bool(s.mission.replay and s.mission.launch_running)
        self.setVisible(show)
        if not show:
            return
        r = s.replay
        if r is None:
            self._set("Run", "—")
            self._set("Time", "—")
            self._set("Status", "waiting for the replay node…", theme.TEXT_DIM)
            self.progress.setValue(0)
            self.pause_btn.setEnabled(False)
            self.rate.setEnabled(False)
            self.adjustSize()
            return
        age = time.monotonic() - r.received_mono if r.received_mono else 0.0
        run = Path(r.bag).name
        self._set("Run", f"{run} ({r.frame} world)" if r.frame else run)
        self._set("Time", f"{format_clock(r.t)} / {format_clock(r.duration)}")
        if age > self._cfg.replay.state_fresh_s:
            self._set("Status", f"stale ({age:.0f} s)", theme.WARN)
        elif r.finished:
            self._set("Status", r.status_text, theme.TEXT_DIM)
        elif not r.gazebo:
            self._set("Status", r.status_text, theme.WARN)
        else:
            self._set("Status", r.status_text, theme.WARN if r.paused else theme.OK)
        self.progress.setValue(int(r.progress * 1000))
        self.pause_btn.setEnabled(not r.finished)
        self.pause_btn.setText("Resume" if r.paused else "Pause")
        self.rate.setEnabled(not r.finished)
        if not self.rate.view().isVisible():
            idx = self.rate.findData(float(r.rate))
            if idx >= 0 and idx != self.rate.currentIndex():
                self.rate.setCurrentIndex(idx)
        self.adjustSize()
