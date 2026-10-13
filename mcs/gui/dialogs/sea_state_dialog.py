"""Sea-state dialog: the current and the waves the simulated boat will feel.

Shown by the bottom toolbar AFTER the launch and world dialogs, for every
Gazebo launch (empty world or personalized world); and again from the
floating SEA STATE box's "Change…" button during a run (``live=True``),
where OK sends the change on ``/sim/sea_state/command`` and the simulator
blends it in.

Two cards, in plain language (2026-10-08 redesign):

* **Current** — on/off, the *mean* speed and the direction it comes from,
  and how much it varies. The simulator adds the small smooth variation
  itself; nothing here is a time series.
* **Waves** — on/off, a strength (the simulator's levels, or a custom
  height), the direction they come from, the period (automatic or set),
  crest length and passing wakes.

Changes over time are the advanced path: "Plan changes over time…" opens
:class:`~mcs.gui.dialogs.sea_timeline_dialog.SeaTimelineDialog`. Labels and
descriptions come from the simulator's installed table
(:class:`mcs.core.sea.SeaCatalog`); no literature reference is shown here —
those live in the simulator's ``docs/sea_state_model.md``.

Headless API for the smoke test: :meth:`choice`, :meth:`set_choice`,
:meth:`set_timeline`, :meth:`ramp_s` — no dialog is ever ``exec()``'d there.
The cards (:class:`CurrentCard`, :class:`WavesCard`) are reused by the
timeline editor.
"""

from __future__ import annotations

import random

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QRadioButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from mcs.config.settings import SeaConfig
from mcs.core.sea import (
    CUSTOM_WAVES,
    CurrentSetting,
    SeaCatalog,
    SeaChoice,
    SeaTimeline,
    WaveSetting,
    compass_name,
    parse_direction,
)
from mcs.gui import theme

_COMPASS16 = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
              "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]


#: Inside a card, text and plain containers sit on the card's panel colour
#: instead of the application's darker base (inputs keep their own look).
_CARD_QSS = ("QLabel, QRadioButton, QWidget#plain { background: transparent; }")


def _hint(text: str = "") -> QLabel:
    lab = QLabel(text)
    lab.setObjectName("hint")
    lab.setWordWrap(True)
    return lab


class _DirectionPicker(QWidget):
    """Compass combo + degrees spin, kept in sync; value = from-bearing.

    ``verb`` completes the read-out: ``→ pushes the boat toward S``."""

    changed = Signal()

    def __init__(self, verb: str = "toward", parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("plain")
        self._verb = verb
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        self.combo = QComboBox()
        self.combo.addItems(_COMPASS16)
        self.deg = QDoubleSpinBox()
        self.deg.setRange(0.0, 359.9)
        self.deg.setDecimals(0)
        self.deg.setSuffix("°")
        self.deg.setWrapping(True)
        self.toward = _hint("")
        self.toward.setWordWrap(False)
        row.addWidget(self.combo)
        row.addWidget(self.deg)
        row.addWidget(self.toward, 1)
        self.combo.currentTextChanged.connect(self._from_combo)
        self.deg.valueChanged.connect(self._from_deg)
        self._syncing = False
        self.set_value(0.0)
        self.setToolTip("The compass direction it comes FROM, like the wind: "
                        "'from N' pushes the boat south.")

    def value(self) -> float:
        return float(self.deg.value()) % 360.0

    def set_value(self, deg: float) -> None:
        self._syncing = True
        try:
            self.deg.setValue(float(deg) % 360.0)
            self.combo.setCurrentText(compass_name(deg))
        finally:
            self._syncing = False
        self._relabel()

    def _from_combo(self, name: str) -> None:
        if self._syncing:
            return
        d = parse_direction(name)
        if d is not None:
            self._syncing = True
            try:
                self.deg.setValue(d)
            finally:
                self._syncing = False
        self._relabel()
        self.changed.emit()

    def _from_deg(self, value: float) -> None:
        if self._syncing:
            return
        self._syncing = True
        try:
            self.combo.setCurrentText(compass_name(value))
        finally:
            self._syncing = False
        self._relabel()
        self.changed.emit()

    def _relabel(self) -> None:
        self.toward.setText(f"→ {self._verb} {compass_name(self.value() + 180.0)}")


# ================================================================ cards
class CurrentCard(QGroupBox):
    """Current: on/off, mean speed, direction, variation."""

    changed = Signal()

    def __init__(self, catalog: SeaCatalog, compact: bool = False, parent=None) -> None:
        super().__init__("Current — moving water", parent)
        self._cat = catalog
        self.setStyleSheet(_CARD_QSS)
        self.setCheckable(True)
        self.setChecked(False)
        v = QVBoxLayout(self)
        if not compact:
            v.addWidget(_hint(
                "Water flowing through the survey area. It pushes the boat off "
                "its line and speeds it up or slows it down; the boat has to "
                "point partly into it (crab) to stay on track."))
        form = QFormLayout()
        speed_row = QHBoxLayout()
        speed_row.setContentsMargins(0, 0, 0, 0)
        self.speed = QDoubleSpinBox()
        self.speed.setRange(0.01, 1.5)
        self.speed.setDecimals(2)
        self.speed.setSingleStep(0.05)
        self.speed.setSuffix(" m/s")
        self.speed.setToolTip("Average speed of the water.")
        speed_row.addWidget(self.speed)
        self._shortcuts = []
        for label, val in catalog.current_shortcuts()[:3]:
            b = QPushButton(label.split("(")[0].strip())
            b.setToolTip(f"{val:.2f} m/s — {catalog.description('current', catalog.nearest_current(val))}")
            b.clicked.connect(lambda _=False, x=val: self.speed.setValue(x))
            b.setStyleSheet("padding: 3px 8px;")
            speed_row.addWidget(b)
            self._shortcuts.append(b)
        speed_row.addStretch(1)
        form.addRow("Mean speed", speed_row)
        self.direction = _DirectionPicker("pushes the boat toward")
        form.addRow("Comes from", self.direction)
        self.variation = QComboBox()
        for name in catalog.variation:
            self.variation.addItem(catalog.label("variation", name), name)
        self.variation.setToolTip("How much the current wanders around the "
                                  "mean you set.")
        form.addRow("Variation", self.variation)
        v.addLayout(form)
        self._var_help = _hint()
        v.addWidget(self._var_help)
        self._feel = _hint()
        if not compact:
            v.addWidget(self._feel)
        self.speed.valueChanged.connect(self._on_change)
        self.direction.changed.connect(self._on_change)
        self.variation.currentIndexChanged.connect(self._on_change)
        self.toggled.connect(self._on_change)
        self.set_setting(CurrentSetting())

    def setting(self) -> CurrentSetting:
        return CurrentSetting(enabled=self.isChecked(),
                              speed_mps=round(float(self.speed.value()), 4),
                              from_deg=self.direction.value(),
                              variation=str(self.variation.currentData() or "natural"))

    def set_setting(self, s: CurrentSetting) -> None:
        self.blockSignals(True)
        try:
            self.setChecked(bool(s.enabled))
            self.speed.setValue(float(s.speed_mps))
            self.direction.set_value(float(s.from_deg))
            i = self.variation.findData(s.variation)
            self.variation.setCurrentIndex(max(i, 0))
        finally:
            self.blockSignals(False)
        self._refresh()

    def _on_change(self, *_args) -> None:
        self._refresh()
        self.changed.emit()

    def _refresh(self) -> None:
        name = str(self.variation.currentData() or "")
        stats = self._cat.variation_text(name)
        desc = self._cat.description("variation", name)
        self._var_help.setText(
            "Speed and direction above are averages. The simulator adds smooth "
            "changes around them by itself"
            + (f" ({stats}, over a minute or two)" if stats else "")
            + ". " + desc)
        near = self._cat.nearest_current(float(self.speed.value()))
        self._feel.setText(f"What the boat feels: {self._cat.description('current', near)}"
                           if self._cat.description("current", near) else "")


class WavesCard(QGroupBox):
    """Waves: on/off, strength, direction, period, crest length, wakes."""

    changed = Signal()

    def __init__(self, catalog: SeaCatalog, compact: bool = False, parent=None) -> None:
        super().__init__("Waves — boat motion", parent)
        self._cat = catalog
        self.setStyleSheet(_CARD_QSS)
        self.setCheckable(True)
        self.setChecked(False)
        v = QVBoxLayout(self)
        if not compact:
            v.addWidget(_hint(
                "Waves lift and tilt the boat (heave, roll and pitch). The sonar "
                "is fixed to the hull, so its beams sweep the seabed unevenly: "
                "brightness banding, a wavy seabed line, features stretched, "
                "repeated or missed."))
        form = QFormLayout()
        strength = QHBoxLayout()
        self.level = QComboBox()
        for name in catalog.wave_levels():
            self.level.addItem(catalog.label("waves", name), name)
        self.level.addItem("Custom height…", CUSTOM_WAVES)
        self.custom_hs = QDoubleSpinBox()
        self.custom_hs.setRange(0.02, 1.5)
        self.custom_hs.setDecimals(2)
        self.custom_hs.setSingleStep(0.05)
        self.custom_hs.setSuffix(" m")
        self.custom_hs.setToolTip("Typical wave height (the significant height: "
                                  "the average of the highest third of the waves).")
        strength.addWidget(self.level, 1)
        strength.addWidget(self.custom_hs)
        form.addRow("Strength", strength)
        self.direction = _DirectionPicker("travel toward")
        form.addRow("Come from", self.direction)
        period = QHBoxLayout()
        self.period_auto = QRadioButton("Automatic")
        self.period_set = QRadioButton("Set")
        grp = QButtonGroup(self)
        grp.addButton(self.period_auto)
        grp.addButton(self.period_set)
        self.tp = QDoubleSpinBox()
        self.tp.setRange(0.8, 15.0)
        self.tp.setDecimals(1)
        self.tp.setSingleStep(0.5)
        self.tp.setSuffix(" s")
        self._auto_tp = _hint()
        self._auto_tp.setWordWrap(False)
        period.addWidget(self.period_auto)
        period.addWidget(self._auto_tp)
        period.addSpacing(12)
        period.addWidget(self.period_set)
        period.addWidget(self.tp)
        period.addStretch(1)
        self.period_auto.setToolTip("Time between crests, chosen from the height "
                                    "the way the simulator's harbour levels are.")
        self.tp.setToolTip("Time between wave crests: short = choppy, long = swell.")
        form.addRow("Period", period)
        self.crest = QComboBox()
        for name in catalog.crest:
            self.crest.addItem(catalog.label("crest", name), name)
        form.addRow("Pattern", self.crest)
        self.wakes = QComboBox()
        for name in catalog.wakes:
            self.wakes.addItem(catalog.label("wakes", name), name)
        form.addRow("Passing wakes", self.wakes)
        v.addLayout(form)
        self._level_help = _hint()
        v.addWidget(self._level_help)
        self._opt_help = _hint()
        v.addWidget(self._opt_help)
        if not compact:
            v.addWidget(_hint(
                "Generated automatically: waves are irregular like real water — "
                "each differs in height, they come in groups, wakes arrive at "
                "random times. The seed below makes it repeatable."))
        for sig in (self.level.currentIndexChanged, self.custom_hs.valueChanged,
                    self.tp.valueChanged, self.crest.currentIndexChanged,
                    self.wakes.currentIndexChanged, self.period_auto.toggled,
                    self.toggled):
            sig.connect(self._on_change)
        self.direction.changed.connect(self._on_change)
        self.set_setting(WaveSetting())

    def setting(self) -> WaveSetting:
        return WaveSetting(enabled=self.isChecked(),
                           level=str(self.level.currentData() or "smooth"),
                           hs_m=round(float(self.custom_hs.value()), 3),
                           from_deg=self.direction.value(),
                           period_auto=self.period_auto.isChecked(),
                           tp_s=round(float(self.tp.value()), 2),
                           crest=str(self.crest.currentData() or "short"),
                           wakes=str(self.wakes.currentData() or "occasional"))

    def set_setting(self, s: WaveSetting) -> None:
        self.blockSignals(True)
        try:
            self.setChecked(bool(s.enabled))
            i = self.level.findData(s.level)
            self.level.setCurrentIndex(max(i, 0))
            self.custom_hs.setValue(float(s.hs_m))
            self.direction.set_value(float(s.from_deg))
            (self.period_auto if s.period_auto else self.period_set).setChecked(True)
            self.tp.setValue(float(s.tp_s))
            for combo, val in ((self.crest, s.crest), (self.wakes, s.wakes)):
                j = combo.findData(val)
                combo.setCurrentIndex(max(j, 0))
        finally:
            self.blockSignals(False)
        self._refresh()

    def _on_change(self, *_args) -> None:
        self._refresh()
        self.changed.emit()

    def _refresh(self) -> None:
        s = self.setting()
        custom = s.level == CUSTOM_WAVES
        self.custom_hs.setVisible(custom)
        self.tp.setEnabled(not s.period_auto)
        hs = s.height(self._cat)
        tp = (self._cat.auto_tp(hs) if custom
              else self._cat.number("waves", s.level, "tp_s", 0.0))
        self._auto_tp.setText(f"(≈ {tp:.1f} s)" if tp > 0 else "")
        self._level_help.setText(
            f"Waves about {hs:.2f} m high. " + self._cat.description(
                "waves", s.level if not custom else self._nearest_level(hs)))
        self._opt_help.setText(" ".join(t for t in (
            self._cat.description("crest", s.crest),
            self._cat.description("wakes", s.wakes)) if t))

    def _nearest_level(self, hs: float) -> str:
        levels = self._cat.wave_levels()
        if not levels:
            return ""
        return min(levels, key=lambda n: abs(self._cat.wave_hs(n) - hs))


# =============================================================== dialog
class SeaStateDialog(QDialog):
    """Pick (or change) the sea state. ``choice()`` is the result."""

    def __init__(self, catalog: SeaCatalog, cfg: SeaConfig,
                 last: SeaChoice | None = None, live: bool = False,
                 parent=None) -> None:
        super().__init__(parent)
        self._catalog = catalog
        self._cfg = cfg
        self._live = live
        self._timeline: SeaTimeline | None = None
        self.setWindowTitle("Change the sea state" if live
                            else "Launch Mission — Sea state")
        self.setMinimumWidth(620)
        layout = QVBoxLayout(self)

        title = QLabel("WHAT THE SIMULATED BOAT WILL FEEL" if not live
                       else "CHANGE THE SEA DURING THE RUN")
        title.setObjectName("sectionTitle")
        layout.addWidget(title)
        layout.addWidget(_hint(
            ("The change is blended in gradually while the mission runs. "
             if live else "")
            + "Leave both off for still, flat water — a controlled bench run. "
              "Turn them on to survey under deployment conditions."))
        if not catalog.source:
            warn = _hint("The simulator's sea table was not found (build "
                         "blueboat_sss_sim): names only, no descriptions.")
            warn.setStyleSheet(f"color: {theme.WARN};")
            layout.addWidget(warn)

        self.current_card = CurrentCard(catalog)
        self.waves_card = WavesCard(catalog)
        layout.addWidget(self.current_card)
        layout.addWidget(self.waves_card)

        # ---- reproducibility + live blend ----------------------------------
        misc = QFormLayout()
        seed_row = QHBoxLayout()
        self._seed = QSpinBox()
        self._seed.setRange(1, 999_999)
        self._seed.setToolTip("The same seed and the same settings give the "
                              "same sea, wave for wave, on every run.")
        new_seed = QPushButton("New")
        new_seed.setToolTip("Draw another seed: a different but equally "
                            "realistic sea.")
        new_seed.setStyleSheet("padding: 3px 8px;")
        new_seed.clicked.connect(lambda: self._seed.setValue(random.randint(1, 999_999)))
        seed_row.addWidget(self._seed)
        seed_row.addWidget(new_seed)
        seed_row.addWidget(_hint("same seed + same settings = the same sea"), 1)
        misc.addRow("Seed", seed_row)
        self._ramp = QSpinBox()
        self._ramp.setRange(5, 600)
        self._ramp.setSuffix(" s")
        self._ramp.setValue(int(round(cfg.ramp_s)))
        self._ramp.setToolTip("The new sea is blended in over this time — the "
                              "simulator never switches abruptly.")
        if live:
            misc.addRow("Blend in over", self._ramp)
        layout.addLayout(misc)

        # ---- advanced: changes over time ------------------------------------
        adv = QFrame()
        adv.setObjectName("panel")
        adv.setStyleSheet(_CARD_QSS)
        ah = QHBoxLayout(adv)
        ah.setContentsMargins(8, 6, 8, 6)
        self._timeline_label = _hint()
        self._plan_btn = QPushButton("Plan changes over time…")
        self._plan_btn.setToolTip("Advanced: split the mission into segments, "
                                  "e.g. calm → light chop → choppy, or a current "
                                  "that turns.")
        self._clear_btn = QPushButton("Remove plan")
        self._plan_btn.clicked.connect(self._open_timeline)
        self._clear_btn.clicked.connect(lambda: self.set_timeline(None))
        ah.addWidget(self._timeline_label, 1)
        ah.addWidget(self._plan_btn)
        ah.addWidget(self._clear_btn)
        layout.addWidget(adv)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Cancel)
        ok = buttons.addButton("Apply now" if live else "Launch",
                               QDialogButtonBox.ButtonRole.AcceptRole)
        ok.setDefault(True)
        buttons.accepted.connect(self._accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        base = last.copy() if last is not None else SeaChoice()
        if last is None:
            base.current.variation = "natural"
        self.set_choice(base)

    # ------------------------------------------------------------ values
    def choice(self) -> SeaChoice:
        return SeaChoice(current=self.current_card.setting(),
                         waves=self.waves_card.setting(),
                         seed=int(self._seed.value()),
                         timeline=self._timeline,
                         scenario_file=getattr(self, "_scenario_file", ""))

    def set_choice(self, c: SeaChoice) -> None:
        self.current_card.set_setting(c.current)
        self.waves_card.set_setting(c.waves)
        self._seed.setValue(max(int(c.seed), 1))
        self.set_timeline(c.timeline)

    def set_timeline(self, tl: SeaTimeline | None) -> None:
        """A plan replaces the two cards (its first segment is what they
        showed); ``None`` gives them back."""
        self._timeline = tl if (tl is not None and tl.segments) else None
        on = self._timeline is not None
        self.current_card.setEnabled(not on)
        self.waves_card.setEnabled(not on)
        self._clear_btn.setVisible(on)
        if on:
            self._timeline_label.setText(
                f"Using the {self._timeline.summary()} — the cards above are "
                "replaced by it.")
            self._plan_btn.setText("Edit plan…")
        else:
            self._timeline_label.setText(
                "Changes over time: none — the sea stays as set above for the "
                "whole mission.")
            self._plan_btn.setText("Plan changes over time…")

    def timeline(self) -> SeaTimeline | None:
        return self._timeline

    def ramp_s(self) -> float:
        return float(self._ramp.value())

    # ------------------------------------------------------------ slots
    def _open_timeline(self) -> None:
        from mcs.gui.dialogs.sea_timeline_dialog import SeaTimelineDialog
        dlg = SeaTimelineDialog(self._catalog, self._cfg, self._timeline,
                                first=self.choice(), seed=int(self._seed.value()),
                                parent=self)
        if dlg.exec() == dlg.DialogCode.Accepted:
            self.set_timeline(dlg.timeline())

    def _accept(self) -> None:
        # A launch always carries its sea as a scenario file (the simulator's
        # schedule format: plain keys have no preset name to pass); a live
        # change sends the same content as a JSON command instead.
        c = self.choice()
        if not self._live and not c.is_null:
            self._scenario_file = c.write_scenario(self._cfg.schedules_dir)
        self.accept()
