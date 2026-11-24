"""Plan sea changes over time (the advanced path of the sea-state dialog).

A plan is a list of **segments**: from its start time each segment sets the
whole sea — the same Current and Waves cards as the basic dialog — and the
change into it is blended over its transition time (the simulator always
ramps, it never steps). The first segment starts with the mission; the
last one lasts until the end. A strip at the bottom draws the planned mean
current speed and wave height against time, so "calm, then light chop
after five minutes, then choppy" reads at a glance instead of as a table
of numbers.

Saved plans are the simulator's own schedule files
(``blueboat_sea_schedule/1``, ``hold`` keyframes with a per-keyframe
``ramp_s``) under ``SeaConfig.schedules_dir``; older ``linear`` timelines
open as segments whose change takes the whole gap — the same water.

Headless API: :meth:`timeline`, :meth:`add_segment`, :meth:`select`,
:meth:`remove_selected`; :class:`TimelineStrip` exposes :meth:`series`.
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from PySide6.QtCore import QPointF, QRectF, Qt, QTime
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QTimeEdit,
    QVBoxLayout,
    QWidget,
)

from mcs.config.settings import SeaConfig
from mcs.core.sea import (
    MIN_TRANSITION_S,
    SeaCatalog,
    SeaChoice,
    SeaTimeline,
    Segment,
    clock_text,
    list_schedules,
    read_schedule,
    write_schedule,
)
from mcs.gui import theme
from mcs.gui.dialogs.sea_state_dialog import CurrentCard, WavesCard, _hint


class TimelineStrip(QWidget):
    """Planned mean current speed (blue) and wave height (green) vs time."""

    def __init__(self, catalog: SeaCatalog, parent=None) -> None:
        super().__init__(parent)
        self._cat = catalog
        self._segs: list[Segment] = []
        self._selected = -1
        self.setMinimumHeight(110)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def set_segments(self, segs: list[Segment], selected: int = -1) -> None:
        self._segs = sorted(segs, key=lambda s: s.start_s)
        self._selected = selected
        self.update()

    def span_s(self) -> float:
        if not self._segs:
            return 600.0
        last = self._segs[-1]
        return max(600.0, (last.start_s + last.transition_s) * 1.15 + 60.0)

    def series(self, which: str) -> list[tuple[float, float]]:
        """(t, value) vertices: holds, and linear blends over each
        transition (what the simulator does under ``hold`` + ``ramp_s``)."""
        def val(s: Segment) -> float:
            if which == "current":
                return s.current.speed_mps if s.current.active else 0.0
            return s.waves.height(self._cat) if s.waves.active else 0.0
        pts: list[tuple[float, float]] = []
        prev = None
        for i, s in enumerate(self._segs):
            v = val(s)
            if i == 0:
                pts.append((0.0, v))
            else:
                pts.append((s.start_s, prev))
                pts.append((s.start_s + max(s.transition_s, MIN_TRANSITION_S), v))
            prev = v
        if pts:
            pts.append((self.span_s(), prev))
        return pts

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(self.rect()).adjusted(46, 6, -8, -18)
        p.fillRect(self.rect(), QColor(theme.BG_PANEL))
        span = self.span_s()
        lanes = (("current", "current m/s", QColor(theme.ACCENT)),
                 ("waves", "waves m", QColor(theme.OK)))
        lane_h = r.height() / 2.0
        p.setPen(QColor(theme.TEXT_DIM))
        for k, (which, name, col) in enumerate(lanes):
            top = r.top() + k * lane_h
            lane = QRectF(r.left(), top + 3, r.width(), lane_h - 6)
            p.setPen(QPen(QColor(theme.BORDER), 1))
            p.drawRect(lane)
            pts = self.series(which)
            vmax = max([v for _, v in pts] + [0.05])
            path = QPainterPath()
            for j, (t, v) in enumerate(pts):
                pt = QPointF(lane.left() + lane.width() * t / span,
                             lane.bottom() - lane.height() * 0.9 * v / vmax)
                path.moveTo(pt) if j == 0 else path.lineTo(pt)
            p.setPen(QPen(col, 2))
            p.drawPath(path)
            p.setPen(QColor(theme.TEXT_DIM))
            p.drawText(QRectF(0, top, r.left() - 4, lane_h),
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       name.split()[0])
            p.drawText(QRectF(lane.right() - 120, lane.top() + 1, 116, 14),
                       Qt.AlignmentFlag.AlignRight, f"max {vmax:.2f} {name.split()[1]}")
        for i, s in enumerate(self._segs):
            x = r.left() + r.width() * s.start_s / span
            sel = i == self._selected
            p.setPen(QPen(QColor(theme.WARN if sel else theme.TEXT_DIM), 2 if sel else 1,
                          Qt.PenStyle.SolidLine if sel else Qt.PenStyle.DashLine))
            p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()))
            p.drawText(QPointF(x + 3, r.bottom() + 13), f"{i + 1}")
        p.setPen(QColor(theme.TEXT_DIM))
        p.drawText(QRectF(r.left(), r.bottom() + 2, r.width(), 14),
                   Qt.AlignmentFlag.AlignRight, f"{clock_text(span)}")
        p.end()


class SeaTimelineDialog(QDialog):
    """Edit a plan of segments; :meth:`timeline` is the result."""

    def __init__(self, catalog: SeaCatalog, cfg: SeaConfig,
                 timeline: SeaTimeline | None = None,
                 first: SeaChoice | None = None, seed: int = 7,
                 parent=None) -> None:
        super().__init__(parent)
        self._cat = catalog
        self._cfg = cfg
        self._seed = int(seed)
        self.setWindowTitle("Plan sea changes over time")
        self.setMinimumWidth(900)
        if timeline is not None and timeline.segments:
            self._tl = SeaTimeline([replace(s, current=replace(s.current),
                                            waves=replace(s.waves))
                                    for s in timeline.sorted()], timeline.name)
        else:
            base = first or SeaChoice()
            self._tl = SeaTimeline([Segment(0.0, 60.0, replace(base.current),
                                            replace(base.waves))], "")
        self._sel = 0
        self._loading = False

        root = QVBoxLayout(self)
        root.addWidget(_hint(
            "Split the mission into segments. From its start time each segment "
            "sets the whole sea; the change into it is blended over its "
            "transition time. The first segment starts with the mission and the "
            "last one lasts until it ends. Within every segment the simulator "
            "still adds its own small, natural variation."))

        body = QHBoxLayout()
        left = QVBoxLayout()
        left.addWidget(QLabel("Segments"))
        self._list = QListWidget()
        self._list.setMinimumWidth(320)
        self._list.setWordWrap(True)
        self._list.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._list.currentRowChanged.connect(self.select)
        left.addWidget(self._list, 1)
        lb = QHBoxLayout()
        for text, slot, tip in (
                ("Add", self.add_segment, ("A new segment 5 minutes after the "
                                           "last, starting from the same sea.")),
                ("Duplicate", self._duplicate, "Copy the selected segment."),
                ("Remove", self.remove_selected, "Delete the selected segment.")):
            b = QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            lb.addWidget(b)
        left.addLayout(lb)
        saved = QHBoxLayout()
        self._saved = QComboBox()
        self._saved.addItem("(load a saved plan)", "")
        for p in list_schedules(cfg.schedules_dir):
            self._saved.addItem(p.stem, str(p))
        self._saved.activated.connect(self._load_saved)
        save = QPushButton("Save as…")
        save.clicked.connect(self._save_as)
        saved.addWidget(self._saved, 1)
        saved.addWidget(save)
        left.addLayout(saved)
        body.addLayout(left, 0)

        right = QVBoxLayout()
        when = QFormLayout()
        self._start = QTimeEdit()
        self._start.setDisplayFormat("H:mm:ss")
        self._start.setMaximumTime(QTime(5, 59, 59))
        self._start.setToolTip("When this segment begins, from the start of the mission.")
        self._ramp = QSpinBox()
        self._ramp.setRange(int(MIN_TRANSITION_S), 1800)
        self._ramp.setSuffix(" s")
        self._ramp.setToolTip("How long the change from the previous segment takes.")
        when.addRow("Starts at", self._start)
        when.addRow("Change gradually over", self._ramp)
        right.addLayout(when)
        self._current = CurrentCard(catalog, compact=True)
        self._waves = WavesCard(catalog, compact=True)
        right.addWidget(self._current)
        right.addWidget(self._waves)
        right.addStretch(1)
        body.addLayout(right, 1)
        root.addLayout(body, 1)

        self.strip = TimelineStrip(catalog)
        root.addWidget(self.strip)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Cancel)
        ok = buttons.addButton("Use this plan", QDialogButtonBox.ButtonRole.AcceptRole)
        ok.setDefault(True)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)

        self._start.timeChanged.connect(self._store)
        self._ramp.valueChanged.connect(self._store)
        self._current.changed.connect(self._store)
        self._waves.changed.connect(self._store)
        self._refresh_list()
        self.select(0)

    # ------------------------------------------------------------ model
    def timeline(self) -> SeaTimeline:
        segs = self._tl.sorted()
        if segs:
            segs[0].start_s = 0.0
        return SeaTimeline(segs, self._tl.name)

    def add_segment(self) -> None:
        segs = self._tl.sorted()
        last = segs[-1] if segs else Segment()
        self._tl.segments.append(Segment(last.start_s + 300.0, 60.0,
                                         replace(last.current), replace(last.waves)))
        self._refresh_list()
        self.select(len(self._tl.segments) - 1)

    def _duplicate(self) -> None:
        segs = self._tl.sorted()
        if not (0 <= self._sel < len(segs)):
            return
        s = segs[self._sel]
        later = [x.start_s for x in segs if x.start_s > s.start_s]
        t = (s.start_s + later[0]) / 2.0 if later else s.start_s + 300.0
        self._tl.segments.append(Segment(t, s.transition_s, replace(s.current),
                                         replace(s.waves)))
        self._refresh_list()
        self.select(self._index_of(t))

    def remove_selected(self) -> None:
        segs = self._tl.sorted()
        if len(segs) <= 1 or not (0 <= self._sel < len(segs)):
            return
        self._tl.segments.remove(segs[self._sel])
        self._refresh_list()
        self.select(max(self._sel - 1, 0))

    def _index_of(self, t: float) -> int:
        for i, s in enumerate(self._tl.sorted()):
            if abs(s.start_s - t) < 1e-6:
                return i
        return 0

    # ------------------------------------------------------------ editing
    def select(self, i: int) -> None:
        segs = self._tl.sorted()
        if not (0 <= i < len(segs)):
            return
        self._sel = i
        s = segs[i]
        self._loading = True
        try:
            if self._list.currentRow() != i:
                self._list.setCurrentRow(i)
            self._start.setTime(QTime(0, 0).addSecs(int(round(s.start_s))))
            self._start.setEnabled(i != 0)
            self._ramp.setValue(int(round(max(s.transition_s, MIN_TRANSITION_S))))
            self._ramp.setEnabled(i != 0)
            self._current.set_setting(s.current)
            self._waves.set_setting(s.waves)
        finally:
            self._loading = False
        self.strip.set_segments(segs, i)

    def _store(self, *_args) -> None:
        if self._loading:
            return
        segs = self._tl.sorted()
        if not (0 <= self._sel < len(segs)):
            return
        s = segs[self._sel]
        if self._sel != 0:
            s.start_s = float(QTime(0, 0).secsTo(self._start.time()))
            s.transition_s = float(self._ramp.value())
        s.current = self._current.setting()
        s.waves = self._waves.setting()
        t = s.start_s
        self._refresh_list()
        self._sel = self._index_of(t)
        self._loading = True
        try:
            self._list.setCurrentRow(self._sel)
        finally:
            self._loading = False
        self.strip.set_segments(self._tl.sorted(), self._sel)

    def _refresh_list(self) -> None:
        self._list.blockSignals(True)
        try:
            self._list.clear()
            for i, s in enumerate(self._tl.sorted()):
                head = (f"{i + 1}. from {clock_text(s.start_s)}"
                        + (f" (over {s.transition_s:.0f} s)" if i else " (mission start)"))
                self._list.addItem(f"{head}\n    {s.summary(self._cat)}")
        finally:
            self._list.blockSignals(False)

    # ------------------------------------------------------------ files
    def _load_saved(self, _index: int = 0) -> None:
        path = str(self._saved.currentData() or "")
        doc = read_schedule(path) if path else None
        if doc is None:
            return
        self._tl = SeaTimeline.from_doc(doc, self._cat, Path(path).stem)
        self._refresh_list()
        self.select(0)

    def _save_as(self) -> None:
        name, ok = QInputDialog.getText(self, "Save plan", "Name",
                                        text=self._tl.name or "")
        if not ok or not name.strip():
            return
        safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in name.strip())
        path = Path(self._cfg.schedules_dir).expanduser() / f"{safe}.yaml"
        if path.exists():
            chosen, _ = QFileDialog.getSaveFileName(
                self, "Save plan as", str(path), "YAML (*.yaml)")
            if not chosen:
                return
            path = Path(chosen)
        self._tl.name = path.stem
        write_schedule(path, self.timeline().to_doc(self._seed))
        if self._saved.findData(str(path)) < 0:
            self._saved.addItem(path.stem, str(path))
        self._saved.setCurrentIndex(self._saved.findData(str(path)))
