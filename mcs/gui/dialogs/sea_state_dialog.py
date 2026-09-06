"""Sea-state dialog: choose current and waves for a simulated mission, or
change them while one runs.

Shown by the bottom toolbar AFTER the launch and world dialogs, for every
Gazebo launch (empty world or personalized world); and again from the left
panel's "Modify situation…" button during a run (``live=True``), where OK
sends the change on ``/sim/sea_state/command`` and the simulator ramps to
it. The preset table comes from :mod:`mcs.core.sea` (the simulator's
installed ``sea_states.yaml``, never an import); every preset shows its
one-sentence explanation as a tooltip and under the combo.

The **timeline** group turns the choice into a schedule: rows of
(time, current, from, waves, from) saved as ``blueboat_sea_schedule/1``
YAML under ``SeaConfig.schedules_dir`` and passed to the launch as
``sea_schedule:=``; a saved timeline can be reloaded from the combo.

Headless API for the smoke test: :meth:`choice`, :meth:`set_choice`,
:meth:`rows`, :meth:`set_rows` — no dialog is ever ``exec()``'d there.
"""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from mcs.config.settings import SeaConfig
from mcs.core.sea import (
    CUSTOM_WAVES,
    NULL_CURRENT,
    NULL_WAVES,
    SeaCatalog,
    SeaChoice,
    compass_name,
    list_schedules,
    parse_direction,
    read_schedule,
    schedule_doc,
    schedule_rows,
    write_schedule,
)

_COMPASS16 = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
              "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]


class _DirectionPicker(QWidget):
    """Compass combo + degrees spin, kept in sync; value = from-bearing."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        self.combo = QComboBox()
        self.combo.addItems(_COMPASS16)
        self.deg = QDoubleSpinBox()
        self.deg.setRange(0.0, 359.9)
        self.deg.setDecimals(1)
        self.deg.setSuffix("°")
        self.deg.setWrapping(True)
        self.toward = QLabel("")
        self.toward.setObjectName("hint")
        row.addWidget(QLabel("from"))
        row.addWidget(self.combo)
        row.addWidget(self.deg)
        row.addWidget(self.toward)
        self.combo.currentTextChanged.connect(self._from_combo)
        self.deg.valueChanged.connect(self._from_deg)
        self._syncing = False
        self.set_value(0.0)
        self.setToolTip("Compass direction the current / waves come FROM "
                        "(meteorological convention): 'from N' pushes the "
                        "boat southwards.")

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

    def _from_deg(self, value: float) -> None:
        if self._syncing:
            return
        self._syncing = True
        try:
            self.combo.setCurrentText(compass_name(value))
        finally:
            self._syncing = False
        self._relabel()

    def _relabel(self) -> None:
        self.toward.setText(f"→ {compass_name(self.value() + 180.0)}")


class SeaStateDialog(QDialog):
    """Pick (or change) the sea state. ``choice()`` is the result."""

    COLUMNS = ("t [s]", "current", "from [°]", "waves", "from [°]")

    def __init__(self, catalog: SeaCatalog, cfg: SeaConfig,
                 last: SeaChoice | None = None, live: bool = False,
                 parent=None) -> None:
        super().__init__(parent)
        self._catalog = catalog
        self._cfg = cfg
        self._live = live
        self.setWindowTitle("Modify sea situation" if live
                            else "Launch Mission — Sea state")
        self.setMinimumWidth(560)
        layout = QVBoxLayout(self)

        intro = QLabel(
            ("Change the current and the waves the simulated boat feels; "
             "the simulator ramps to the new situation over the ramp time.")
            if live else
            ("Choose the water the simulated boat will feel: a current "
             "(pushes / slows it through the hull hydrodynamics) and waves "
             "(heave, pitch and roll through a hydrostatic wave force). "
             "Presets are harbour-realistic plus one stress level; the "
             "direction is where the current / waves come from. "
             "'No current' + 'Calm' changes nothing."))
        intro.setWordWrap(True)
        layout.addWidget(intro)
        if catalog.source:
            src = QLabel(f"presets: {catalog.source}")
        else:
            src = QLabel("presets: built-in names (simulator table not found — "
                         "build blueboat_sss_sim to get descriptions)")
        src.setObjectName("hint")
        src.setWordWrap(True)
        layout.addWidget(src)

        form = QFormLayout()
        self._current = QComboBox()
        self._waves = QComboBox()
        for name in catalog.current:
            self._current.addItem(catalog.label("current", name), name)
        for name in catalog.waves:
            self._waves.addItem(catalog.label("waves", name), name)
        self._waves.addItem("Custom (Hs / Tp / spectrum / wake groups)…", CUSTOM_WAVES)
        self._current_help = QLabel("")
        self._waves_help = QLabel("")
        for lab in (self._current_help, self._waves_help):
            lab.setObjectName("hint")
            lab.setWordWrap(True)
        self._current_dir = _DirectionPicker()
        self._waves_dir = _DirectionPicker()
        form.addRow("Current", self._current)
        form.addRow("", self._current_help)
        form.addRow("Current direction", self._current_dir)
        form.addRow("Waves", self._waves)
        form.addRow("", self._waves_help)
        form.addRow("Wave direction", self._waves_dir)
        # Custom waves: explicit numbers instead of a preset (carried to the
        # simulator as a one-keyframe schedule / explicit command fields).
        self._custom_box = QGroupBox("Custom waves")
        cf = QFormLayout(self._custom_box)
        self._c_hs = QDoubleSpinBox(); self._c_hs.setRange(0.0, 2.0)
        self._c_hs.setSingleStep(0.05); self._c_hs.setDecimals(2); self._c_hs.setSuffix(" m")
        self._c_hs.setValue(0.20)
        self._c_hs.setToolTip("Significant wave height of the background sea "
                              "(mean of the highest third of the waves).")
        self._c_tp = QDoubleSpinBox(); self._c_tp.setRange(0.5, 20.0)
        self._c_tp.setSingleStep(0.5); self._c_tp.setDecimals(1); self._c_tp.setSuffix(" s")
        self._c_tp.setValue(3.0)
        self._c_tp.setToolTip("Peak period: the time between the dominant "
                              "waves (harbour chop 2-4 s, swell 5-8 s).")
        self._c_gamma = QDoubleSpinBox(); self._c_gamma.setRange(1.0, 7.0)
        self._c_gamma.setSingleStep(0.5); self._c_gamma.setDecimals(1); self._c_gamma.setValue(1.5)
        self._c_gamma.setToolTip("JONSWAP peakedness: 1 = broad Pierson-Moskowitz "
                                 "sea (irregular), 3.3 = narrow North-Sea storm "
                                 "spectrum (regular-looking).")
        self._c_rate = QDoubleSpinBox(); self._c_rate.setRange(0.0, 120.0)
        self._c_rate.setDecimals(0); self._c_rate.setSuffix(" /h"); self._c_rate.setValue(12.0)
        self._c_rate.setToolTip("Random wake / swell groups per hour (Poisson "
                                "arrivals); 0 = none.")
        self._c_ehs = QDoubleSpinBox(); self._c_ehs.setRange(0.0, 2.0)
        self._c_ehs.setSingleStep(0.05); self._c_ehs.setDecimals(2); self._c_ehs.setSuffix(" m")
        self._c_ehs.setValue(0.20)
        self._c_ehs.setToolTip("Height of a wake / swell group at its peak.")
        self._c_etp = QDoubleSpinBox(); self._c_etp.setRange(2.0, 20.0)
        self._c_etp.setSingleStep(0.5); self._c_etp.setDecimals(1); self._c_etp.setSuffix(" s")
        self._c_etp.setValue(6.0)
        self._c_etp.setToolTip("Period of the waves inside a group.")
        cf.addRow("Hs", self._c_hs)
        cf.addRow("Tp", self._c_tp)
        cf.addRow("γ (peakedness)", self._c_gamma)
        cf.addRow("Wake groups", self._c_rate)
        cf.addRow("Group height", self._c_ehs)
        cf.addRow("Group period", self._c_etp)
        self._custom_box.setVisible(False)
        form.addRow(self._custom_box)
        self._seed = QSpinBox()
        self._seed.setRange(0, 999_999)
        self._seed.setToolTip("Seed of the wave phases and current "
                              "fluctuations (0 = the schedule's / 1); the same "
                              "seed reproduces the same sea.")
        form.addRow("Seed", self._seed)
        self._ramp = QDoubleSpinBox()
        self._ramp.setRange(0.0, 600.0)
        self._ramp.setSuffix(" s")
        self._ramp.setValue(float(cfg.ramp_s))
        self._ramp.setToolTip("Seconds over which the simulator blends from "
                              "the current situation to the new one.")
        if live:
            form.addRow("Ramp", self._ramp)
        layout.addLayout(form)
        self._current.currentIndexChanged.connect(self._refresh_help)
        self._waves.currentIndexChanged.connect(self._refresh_help)
        self._waves.currentIndexChanged.connect(
            lambda *_: self._custom_box.setVisible(self.is_custom()))

        # ---- timeline --------------------------------------------------------
        box = QGroupBox("Timeline (optional): let the sea change over the mission")
        box.setCheckable(True)
        box.setChecked(False)
        vb = QVBoxLayout(box)
        top = QHBoxLayout()
        self._saved = QComboBox()
        self._saved.addItem("(new timeline)", "")
        for p in list_schedules(cfg.schedules_dir):
            self._saved.addItem(p.stem, str(p))
        self._saved.currentIndexChanged.connect(self._load_saved)
        top.addWidget(QLabel("Saved"))
        top.addWidget(self._saved, 1)
        self._interp = QComboBox()
        self._interp.addItems(["linear", "hold"])
        self._interp.setToolTip("linear: numbers blend between keyframes; "
                                "hold: each keyframe holds until the next.")
        top.addWidget(QLabel("between rows"))
        top.addWidget(self._interp)
        vb.addLayout(top)
        self._table = QTableWidget(0, len(self.COLUMNS))
        self._table.setHorizontalHeaderLabels(list(self.COLUMNS))
        self._table.horizontalHeader().setStretchLastSection(True)
        self._table.setMinimumHeight(140)
        vb.addWidget(self._table)
        btns = QHBoxLayout()
        add = QPushButton("Add row (current selection)")
        rem = QPushButton("Remove row")
        save = QPushButton("Save as…")
        add.clicked.connect(self._add_row)
        rem.clicked.connect(self._remove_row)
        save.clicked.connect(self._save_as)
        for b in (add, rem, save):
            btns.addWidget(b)
        btns.addStretch(1)
        vb.addLayout(btns)
        self._timeline = box
        layout.addWidget(box)
        self.schedule_file: str = ""

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Cancel)
        ok = buttons.addButton("Apply now" if live else "Launch",
                               QDialogButtonBox.ButtonRole.AcceptRole)
        ok.setDefault(True)
        buttons.accepted.connect(self._accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.set_choice(last or SeaChoice(current=cfg.default_current,
                                          waves=cfg.default_waves))
        self._refresh_help()

    # ------------------------------------------------------------ values
    def is_custom(self) -> bool:
        return str(self._waves.currentData() or "") == CUSTOM_WAVES

    def custom_waves(self) -> dict | None:
        if not self.is_custom():
            return None
        return {
            "hs_m": float(self._c_hs.value()), "tp_s": float(self._c_tp.value()),
            "gamma": float(self._c_gamma.value()),
            "events": {"rate_per_hour": float(self._c_rate.value()),
                       "hs_m": float(self._c_ehs.value()),
                       "tp_s": float(self._c_etp.value())},
        }

    def choice(self) -> SeaChoice:
        custom = self.custom_waves()
        return SeaChoice(
            current=str(self._current.currentData() or NULL_CURRENT),
            current_from_deg=self._current_dir.value(),
            waves=NULL_WAVES if custom else str(self._waves.currentData() or NULL_WAVES),
            waves_from_deg=self._waves_dir.value(),
            schedule_file=self.schedule_file if (self._timeline.isChecked()
                                                 or (custom and not self._live)) else "",
            seed=int(self._seed.value()),
            custom_waves=custom,
        )

    def set_choice(self, c: SeaChoice) -> None:
        i = self._current.findData(c.current)
        self._current.setCurrentIndex(max(i, 0))
        self._current_dir.set_value(c.current_from_deg)
        if c.custom_waves:
            cw = c.custom_waves
            self._waves.setCurrentIndex(self._waves.findData(CUSTOM_WAVES))
            self._c_hs.setValue(float(cw.get("hs_m", 0.2)))
            self._c_tp.setValue(float(cw.get("tp_s", 3.0)))
            self._c_gamma.setValue(float(cw.get("gamma", 1.5)))
            ev = cw.get("events") or {}
            self._c_rate.setValue(float(ev.get("rate_per_hour", 0.0)))
            self._c_ehs.setValue(float(ev.get("hs_m", 0.2)))
            self._c_etp.setValue(float(ev.get("tp_s", 6.0)))
        else:
            j = self._waves.findData(c.waves)
            self._waves.setCurrentIndex(max(j, 0))
        self._custom_box.setVisible(self.is_custom())
        self._waves_dir.set_value(c.waves_from_deg)
        self._seed.setValue(int(c.seed))
        if c.schedule_file:
            k = self._saved.findData(c.schedule_file)
            if k < 0:
                self._saved.addItem(Path(c.schedule_file).stem, c.schedule_file)
                k = self._saved.count() - 1
            self._timeline.setChecked(True)
            self._saved.setCurrentIndex(k)
            self._load_saved()

    def ramp_s(self) -> float:
        return float(self._ramp.value())

    def rows(self) -> list[dict]:
        out = []
        for r in range(self._table.rowCount()):
            def cell(c, row=r):
                it = self._table.item(row, c)
                return it.text() if it is not None else ""
            try:
                out.append({
                    "t_s": float(cell(0) or 0.0),
                    "current": cell(1) or NULL_CURRENT,
                    "current_from_deg": parse_direction(cell(2) or "0") or 0.0,
                    "waves": cell(3) or NULL_WAVES,
                    "waves_from_deg": parse_direction(cell(4) or "0") or 0.0,
                })
            except ValueError:
                continue
        return sorted(out, key=lambda r: r["t_s"])

    def set_rows(self, rows: list[dict]) -> None:
        self._table.setRowCount(0)
        for r in rows:
            self._append_row(r)

    def schedule_document(self, name: str) -> dict:
        return schedule_doc(self.rows(), name, int(self._seed.value()),
                            self._interp.currentText(),
                            custom_waves=self.custom_waves())

    # ------------------------------------------------------------ slots
    def _refresh_help(self) -> None:
        self._current_help.setText(self._catalog.description(
            "current", str(self._current.currentData() or "")))
        self._waves_help.setText(self._catalog.description(
            "waves", str(self._waves.currentData() or "")))
        self._current.setToolTip(self._current_help.text())
        self._waves.setToolTip(self._waves_help.text())

    def _append_row(self, r: dict) -> None:
        n = self._table.rowCount()
        self._table.insertRow(n)
        vals = (f"{float(r['t_s']):.0f}", str(r["current"]),
                f"{float(r['current_from_deg']):.0f}", str(r["waves"]),
                f"{float(r['waves_from_deg']):.0f}")
        for c, v in enumerate(vals):
            item = QTableWidgetItem(v)
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable)
            self._table.setItem(n, c, item)

    def _add_row(self) -> None:
        rows = self.rows()
        t_next = (rows[-1]["t_s"] + 300.0) if rows else 0.0
        c = self.choice()
        self._append_row({"t_s": t_next, "current": c.current,
                          "current_from_deg": c.current_from_deg,
                          "waves": c.waves, "waves_from_deg": c.waves_from_deg})
        self._timeline.setChecked(True)

    def _remove_row(self) -> None:
        r = self._table.currentRow()
        if r >= 0:
            self._table.removeRow(r)

    def _load_saved(self) -> None:
        path = str(self._saved.currentData() or "")
        if not path:
            return
        doc = read_schedule(path)
        if doc is None:
            return
        self.schedule_file = path
        self._interp.setCurrentText(str(doc.get("interpolation", "linear")))
        self._seed.setValue(int(doc.get("seed", 0) or 0))
        self.set_rows(schedule_rows(doc))

    def _save_as(self) -> None:
        rows = self.rows()
        if not rows:
            return
        name, ok = QInputDialog.getText(self, "Save timeline", "Name")
        if not ok or not name.strip():
            return
        safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in name.strip())
        path = Path(self._cfg.schedules_dir).expanduser() / f"{safe}.yaml"
        if path.exists():
            path, _ = QFileDialog.getSaveFileName(
                self, "Save timeline as", str(path), "YAML (*.yaml)")
            if not path:
                return
        write_schedule(path, self.schedule_document(safe))
        self.schedule_file = str(path)
        if self._saved.findData(str(path)) < 0:
            self._saved.addItem(Path(path).stem, str(path))
        self._saved.setCurrentIndex(self._saved.findData(str(path)))

    def _accept(self) -> None:
        # Custom waves at launch travel as a one-keyframe schedule (the
        # launch arguments only name presets); a live change sends the
        # explicit fields directly.
        if self.is_custom() and not self._live and not self._timeline.isChecked():
            rows = [{"t_s": 0.0,
                     "current": str(self._current.currentData() or NULL_CURRENT),
                     "current_from_deg": self._current_dir.value(),
                     "waves": CUSTOM_WAVES,
                     "waves_from_deg": self._waves_dir.value()}]
            path = Path(self._cfg.schedules_dir).expanduser() / "custom_waves.yaml"
            write_schedule(path, schedule_doc(rows, "custom_waves",
                                              int(self._seed.value()),
                                              custom_waves=self.custom_waves()))
            self.schedule_file = str(path)
        # A ticked timeline with unsaved rows is saved under an automatic
        # name so the launch always points at a real file.
        if self._timeline.isChecked() and self.rows() and not self.schedule_file:
            path = (Path(self._cfg.schedules_dir).expanduser()
                    / "timeline_unsaved.yaml")
            write_schedule(path, self.schedule_document("timeline_unsaved"))
            self.schedule_file = str(path)
        if self._timeline.isChecked() and not self.rows():
            self._timeline.setChecked(False)
        self.accept()
