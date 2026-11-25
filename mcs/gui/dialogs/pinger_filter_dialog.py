"""Pinger settings of the Launch and Update Mission dialogs (2026-11-24).

* :class:`PingerForm` — the rows both dialogs share, so they cannot drift
  apart: **Pinger** (Off / Follow pinger (no path) / Log pinger only),
  **Pinger source** (Filtered (default) / Legacy: dragged by the boat between
  fixes) and **Pinger filter** (a preset of the schema, and **Advanced…**).
* :class:`PingerFilterDialog` — the "Advanced…" panel, GENERATED from
  BlueBoat-Control's parameter schema (``config/pinger_filter.yaml``, read as
  data by :mod:`mcs.core.pinger_filter`). Two tabs on the same values:

  - **Field tuning** (opens first): one card per knob of the schema's
    ``field_tuning`` list -- a picture of its effect (the filter run at a low
    value, the default and a high value, ``pinger_tuning/*.png`` beside the
    schema; click to enlarge), what it does, what lowering / raising it does,
    how to set it in the field, and a slider (field-sensible range, log where
    the schema says) with the exact value beside it. A line under each value
    says live which way it is from the preset and what that does.
  - **All parameters (expert)**: one group box per schema group in schema
    order, one row per parameter (spin box with the schema's min / max,
    combo for an enum, check box for a bool), its unit, its description as
    the tooltip. **Reset to preset**, **Load YAML…** / **Save YAML…** (in
  ``PingerConfig.filter_dir``). An invalid set (``y_sign`` / ``z_sign`` 0,
  ``gate_hard < gate_soft``, ``stale_timeout_s < coast_after_s``, or a loaded
  file out of range) disables OK and shows why.

The result is the ``configure``'s block ``{"preset": <name, or custom when
edited>, "params": {every parameter}}``; the station computes nothing from
it -- what Control applied comes back on ``/blueboat/pinger/filter_config``.
"""

from __future__ import annotations

import math
from pathlib import Path

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (
    QAbstractSpinBox,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from mcs.config.settings import AppConfig
from mcs.core import pinger_filter as pf
from mcs.gui import theme

#: The pinger mode choices (Control's pinger_mode), in menu order.
MODE_ITEMS = ((pf.MODE_OFF, "Off"),
              (pf.MODE_FOLLOW, "Follow pinger (no path)"),
              (pf.MODE_LOG, "Log pinger only"))
#: What /blueboat/pinger_coordinates carries (Control's pinger_source).
SOURCE_ITEMS = ((pf.SOURCE_FILTER, "Filtered (default)"),
                (pf.SOURCE_LEGACY, "Legacy: dragged by the boat between fixes"))


def _decimals(*numbers) -> int:
    """Decimals that show every given number exactly (2..6)."""
    d = 2
    for v in numbers:
        try:
            text = f"{float(v):.6f}".rstrip("0")
        except (TypeError, ValueError):
            continue
        d = max(d, len(text.partition(".")[2]))
    return min(d, 6)


def _editor_value(ed: QWidget):
    if isinstance(ed, QCheckBox):
        return ed.isChecked()
    if isinstance(ed, QComboBox):
        return ed.currentText()
    if isinstance(ed, QSpinBox):
        return int(ed.value())
    return float(ed.value())


def _set_editor_value(ed: QWidget, value) -> None:
    if isinstance(ed, QCheckBox):
        ed.setChecked(bool(value))
    elif isinstance(ed, QComboBox):
        ed.setCurrentText(str(value))
    elif isinstance(ed, QSpinBox):
        ed.setValue(int(value))
    else:
        need = _decimals(value)
        if need > ed.decimals():
            lo, hi = ed.minimum(), ed.maximum()
            ed.setDecimals(need)
            ed.setRange(lo, hi)
        ed.setValue(float(value))


def _on_editor_change(ed: QWidget, slot) -> None:
    if isinstance(ed, QCheckBox):
        ed.toggled.connect(slot)
    elif isinstance(ed, QComboBox):
        ed.currentTextChanged.connect(slot)
    else:
        ed.valueChanged.connect(slot)


def _fill_preset_combo(combo: QComboBox, schema, names) -> None:
    """The preset names, each with its schema description as the tooltip."""
    for name in names:
        combo.addItem(name)
        if schema is not None:
            combo.setItemData(combo.count() - 1, schema.preset_description(name),
                              Qt.ItemDataRole.ToolTipRole)


class _Picture(QLabel):
    """A card's picture, scaled to the card; a click shows it full size."""

    WIDTH, HEIGHT = 680, 245

    def __init__(self, path: Path, title: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._path, self._title = path, title
        self.full = QPixmap(str(path))
        if not self.full.isNull():
            small = self.full.scaled(min(self.WIDTH, self.full.width()),
                                     min(self.HEIGHT, self.full.height()),
                                     Qt.AspectRatioMode.KeepAspectRatio,
                                     Qt.TransformationMode.SmoothTransformation)
            self.setPixmap(small)
            self.setFixedSize(small.size())       # never squeezed (a clipped title)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip("Click to enlarge. Made by running the filter itself at a low value, "
                        "the default and a high value (field-level noise).")

    def mousePressEvent(self, event) -> None:
        dlg = QDialog(self)
        dlg.setWindowTitle(self._title)
        lay = QVBoxLayout(dlg)
        big = QLabel()
        big.setPixmap(self.full)
        area = QScrollArea()
        area.setWidget(big)
        lay.addWidget(area)
        dlg.resize(min(self.full.width() + 40, 1400), min(self.full.height() + 60, 900))
        dlg.exec()


class TuningCard(QFrame):
    """One Field-tuning knob: picture, texts, and per parameter a slider and
    the exact value. ``changed(name, value)`` when the operator moves one."""

    changed = Signal(str, object)
    STEPS = 1000

    def __init__(self, schema: pf.FilterSchema, card: dict, make_editor, base_value,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.card = card
        self.title = str(card.get("title", ""))
        self._schema = schema
        self._base_value = base_value             # name -> the preset's value
        self.setFrameShape(QFrame.Shape.StyledPanel)
        lay = QVBoxLayout(self)
        head = QLabel(f"<b>{self.title}</b> &nbsp; <span style='color:{theme.TEXT_DIM}'>"
                      f"{', '.join(card['params'])}</span>")
        head.setStyleSheet("font-size: 13px;")
        lay.addWidget(head)
        path = schema.picture_path(card)
        self.picture = _Picture(path, self.title) if path is not None else None
        if self.picture is not None:
            lay.addWidget(self.picture)
        texts = QLabel(
            f"<p style='margin:0'>{card.get('what', '')}</p>"
            f"<p style='margin:4px 0 0 0'><b>&#9660; Lower:</b> {card.get('lower', '')}</p>"
            f"<p style='margin:2px 0 0 0'><b>&#9650; Higher:</b> {card.get('higher', '')}</p>"
            f"<p style='margin:4px 0 0 0; color:{theme.TEXT_DIM}'><b>In the field:</b> "
            f"{card.get('field', '')}</p>")
        texts.setWordWrap(True)
        texts.setTextFormat(Qt.TextFormat.RichText)
        lay.addWidget(texts)

        #: name -> (editor, slider or None, effect label)
        self.rows: dict[str, tuple[QWidget, QSlider | None, QLabel]] = {}
        form = QFormLayout()
        lay.addLayout(form)
        for name in card["params"]:
            spec = schema.parameters[name]
            editor = make_editor(name, spec)
            editor.setToolTip(str(spec.get("description", "")))
            rng = schema.slider_range(card, name)
            slider = None
            row = QWidget()
            rl = QHBoxLayout(row)
            rl.setContentsMargins(0, 0, 0, 0)
            if rng is not None:
                slider = QSlider(Qt.Orientation.Horizontal)
                slider.setRange(0, self.STEPS)
                slider.setToolTip(f"{rng[0]:g} … {rng[1]:g} {spec.get('unit') or ''}"
                                  + (" (log scale)" if rng[2] else "")
                                  + " — type a value beside it for anything outside")
                slider.valueChanged.connect(lambda pos, n=name: self._from_slider(n, pos))
                rl.addWidget(slider, 1)
            editor.setMinimumWidth(90)
            rl.addWidget(editor, 0 if slider else 1)
            unit = QLabel(str(spec.get("unit") or ""))
            unit.setStyleSheet(f"color: {theme.TEXT_DIM};")
            rl.addWidget(unit)
            effect = QLabel()
            effect.setWordWrap(True)
            effect.setStyleSheet("font-size: 10px;")
            form.addRow(QLabel(name), row)
            form.addRow("", effect)
            _on_editor_change(editor, lambda _v, n=name: self._from_editor(n))
            self.rows[name] = (editor, slider, effect)

    # ------------------------------------------------------------ slider maths
    def _range(self, name):
        return self._schema.slider_range(self.card, name)

    def _to_pos(self, name, value) -> int:
        lo, hi, log = self._range(name)
        v = min(max(float(value), lo), hi)
        f = (math.log(v / lo) / math.log(hi / lo)) if log else (v - lo) / (hi - lo)
        return int(round(f * self.STEPS))

    def _from_pos(self, name, pos: int) -> float:
        lo, hi, log = self._range(name)
        f = pos / self.STEPS
        v = lo * (hi / lo) ** f if log else lo + f * (hi - lo)
        spec = self._schema.parameters[name]
        if spec.get("type") == "int":
            return int(round(v))
        # Two significant digits keep slider values readable (0.02, 1.5, 12).
        return float(f"{v:.2g}") if v < 10 else round(v, 1 if v < 100 else 0)

    # ------------------------------------------------------------ events
    def _from_slider(self, name: str, pos: int) -> None:
        editor = self.rows[name][0]
        _set_editor_value(editor, self._from_pos(name, pos))     # -> _from_editor

    def _from_editor(self, name: str) -> None:
        editor, slider, _ = self.rows[name]
        value = _editor_value(editor)
        if slider is not None:
            slider.blockSignals(True)
            slider.setValue(self._to_pos(name, value))
            slider.blockSignals(False)
        self._show_effect(name)
        self.changed.emit(name, value)

    def show_value(self, name: str, value) -> None:
        """Set from outside (the expert tab, a preset, a file): no signal."""
        editor, slider, _ = self.rows[name]
        if _editor_value(editor) != value:
            editor.blockSignals(True)
            _set_editor_value(editor, value)
            editor.blockSignals(False)
        if slider is not None:
            slider.blockSignals(True)
            slider.setValue(self._to_pos(name, value))
            slider.blockSignals(False)
        self._show_effect(name)

    def _show_effect(self, name: str) -> None:
        editor, _, label = self.rows[name]
        value, base = _editor_value(editor), self._base_value(name)
        try:
            lower, higher = float(value) < float(base), float(value) > float(base)
        except (TypeError, ValueError):
            lower = higher = False
        if lower:
            label.setText(f"▼ below the preset ({base:g}): {self.card.get('lower', '')}")
            label.setStyleSheet(f"font-size: 10px; color: {theme.WARN};")
        elif higher:
            label.setText(f"▲ above the preset ({base:g}): {self.card.get('higher', '')}")
            label.setStyleSheet(f"font-size: 10px; color: {theme.WARN};")
        else:
            label.setText(f"= the preset's value ({base})")
            label.setStyleSheet(f"font-size: 10px; color: {theme.TEXT_DIM};")


class PingerFilterDialog(QDialog):
    """The Advanced panel: every filter parameter, generated from the schema."""

    def __init__(self, cfg: AppConfig, schema: pf.FilterSchema, base_preset: str,
                 values: dict | None = None, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self._schema = schema
        # Editors emit while they are being built (a range that clamps the
        # initial value): nothing is validated until set_values below.
        self._loading = True
        self.setWindowTitle("Pinger filter — tuning")
        self.setMinimumWidth(760)
        self.resize(820, 860)
        layout = QVBoxLayout(self)

        top = QHBoxLayout()
        top.addWidget(QLabel("Preset"))
        self._preset = QComboBox()
        _fill_preset_combo(self._preset, schema, schema.preset_names())
        if base_preset in schema.presets:
            self._preset.setCurrentText(base_preset)
        self._preset.setToolTip("The preset the values start from. Choosing "
                                "another one loads its values.")
        top.addWidget(self._preset, 1)
        self._reset = QPushButton("Reset to preset")
        self._reset.setToolTip("Discard every change: the chosen preset's values.")
        self._reset.clicked.connect(self.reset_to_preset)
        top.addWidget(self._reset)
        layout.addLayout(top)

        self._state = QLabel()
        self._state.setStyleSheet(f"color: {theme.TEXT_DIM}; font-size: 10px;")
        layout.addWidget(self._state)

        self.tabs = QTabWidget()
        layout.addWidget(self.tabs, 1)

        # ---- Field tuning: the knobs worth touching, with their effect --------
        #: title -> card (the smoke test reads them)
        self.cards: dict[str, TuningCard] = {}
        tune_scroll = QScrollArea()
        tune_scroll.setWidgetResizable(True)
        tune_body = QWidget()
        tune_scroll.setWidget(tune_body)
        tune_layout = QVBoxLayout(tune_body)
        intro = QLabel(
            "The knobs worth touching in the field. Each picture is the filter itself "
            "run at a low value, the default and a high value, with the noise measured "
            "on the 2026-10 field days. Start from the preset; change one knob at a "
            "time; everything else is under <i>All parameters</i>.")
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color: {theme.TEXT_DIM};")
        tune_layout.addWidget(intro)
        for card in schema.tuning:
            tc = TuningCard(schema, card, self._make_plain_editor,
                            lambda n: self._schema.preset_values(self.base_preset()).get(n),
                            tune_body)
            tc.changed.connect(self._from_card)
            tune_layout.addWidget(tc)
            self.cards[tc.title] = tc
        if not schema.tuning:
            tune_layout.addWidget(QLabel("This schema has no field-tuning cards: use "
                                         "All parameters."))
        tune_layout.addStretch(1)
        self.tabs.addTab(tune_scroll, "Field tuning")

        # ---- All parameters (expert) -------------------------------------------
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        body = QWidget()
        scroll.setWidget(body)
        body_layout = QVBoxLayout(body)
        body_layout.setContentsMargins(0, 0, 0, 0)
        self.tabs.addTab(scroll, "All parameters (expert)")

        #: name -> editor / unit label / row label (the smoke test reads them)
        self.editors: dict[str, QWidget] = {}
        self.units: dict[str, QLabel] = {}
        self.labels: dict[str, QLabel] = {}
        self.groups: list[str] = []
        for group in schema.group_names():
            names = schema.names_in_group(group)
            if not names:
                continue
            box = QGroupBox(group.replace("&", "&&"))   # "&" is a mnemonic in Qt
            form = QFormLayout(box)
            for name in names:
                spec = schema.parameters[name]
                editor = self._make_editor(name, spec)
                desc = str(spec.get("description", ""))
                editor.setToolTip(desc)
                label = QLabel(name)
                label.setToolTip(desc)
                unit = QLabel(str(spec.get("unit") or ""))
                unit.setStyleSheet(f"color: {theme.TEXT_DIM};")
                unit.setToolTip(desc)
                row = QWidget()
                row_layout = QHBoxLayout(row)
                row_layout.setContentsMargins(0, 0, 0, 0)
                row_layout.addWidget(editor, 1)
                row_layout.addWidget(unit)
                form.addRow(label, row)
                self.editors[name] = editor
                self.units[name] = unit
                self.labels[name] = label
            body_layout.addWidget(box)
            self.groups.append(group)
        body_layout.addStretch(1)

        self._problem = QLabel()
        self._problem.setWordWrap(True)
        self._problem.setStyleSheet(f"color: {theme.ERR};")
        layout.addWidget(self._problem)

        self._buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        load = self._buttons.addButton("Load YAML…", QDialogButtonBox.ButtonRole.ActionRole)
        save = self._buttons.addButton("Save YAML…", QDialogButtonBox.ButtonRole.ActionRole)
        load.clicked.connect(self._on_load_clicked)
        save.clicked.connect(self._on_save_clicked)
        self._save_button = save
        self._buttons.accepted.connect(self.accept)
        self._buttons.rejected.connect(self.reject)
        layout.addWidget(self._buttons)

        self.set_values(values if values is not None
                        else schema.preset_values(self.base_preset()))
        self._preset.currentTextChanged.connect(lambda _t: self.reset_to_preset())

    # ------------------------------------------------------------ editors
    def _make_plain_editor(self, name: str, spec: dict) -> QWidget:
        """An editor for one parameter, ranges from the schema, unconnected."""
        kind = spec.get("type")
        if kind == "bool":
            return QCheckBox()
        if kind == "enum":
            combo = QComboBox()
            combo.addItems([str(c) for c in spec.get("choices") or []])
            return combo
        lo, hi = spec.get("min"), spec.get("max")
        if kind == "int":
            spin = QSpinBox()
            spin.setRange(int(lo) if lo is not None else -(2 ** 31) + 1,
                          int(hi) if hi is not None else 2 ** 31 - 1)
            return spin
        dspin = QDoubleSpinBox()
        preset_vals = [(p.get("values") or {}).get(name)
                       for p in self._schema.presets.values()]
        dspin.setDecimals(_decimals(spec.get("default"), lo, hi, *preset_vals))
        dspin.setRange(float(lo) if lo is not None else -1e9,
                       float(hi) if hi is not None else 1e9)
        dspin.setStepType(QAbstractSpinBox.StepType.AdaptiveDecimalStepType)
        return dspin

    def _make_editor(self, name: str, spec: dict) -> QWidget:
        """The expert tab's editor: every change re-validates (and updates
        the Field-tuning card that shows the same parameter)."""
        editor = self._make_plain_editor(name, spec)
        _on_editor_change(editor, self._revalidate)
        return editor

    def _value(self, name: str):
        return _editor_value(self.editors[name])

    def _set_value(self, name: str, value) -> None:
        _set_editor_value(self.editors[name], value)

    def _from_card(self, name: str, value) -> None:
        """A card moved: the expert editor holds the value (one truth)."""
        if name in self.editors and self._value(name) != value:
            self._set_value(name, value)            # -> _revalidate -> _sync_cards

    def _sync_cards(self, values: dict) -> None:
        for card in self.cards.values():
            for name in card.rows:
                card.show_value(name, values[name])

    # ------------------------------------------------------------- values
    def values(self) -> dict:
        return {name: self._value(name) for name in self.editors}

    def set_values(self, values: dict) -> None:
        """Fill the editors (names the schema does not know are ignored;
        missing names keep their editor's value)."""
        self._loading = True
        try:
            for name, value in values.items():
                if name in self.editors:
                    self._set_value(name, value)
        finally:
            self._loading = False
        self._revalidate()

    def base_preset(self) -> str:
        return self._preset.currentText()

    def problems(self) -> list[str]:
        return pf.validate(self.values(), self._schema)[1]

    def is_custom(self) -> bool:
        return not pf.same_params(self.values(),
                                  self._schema.preset_values(self.base_preset()))

    def result_block(self) -> dict:
        """``{"preset": <base, or custom when edited>, "params": {...}}``."""
        return {"preset": pf.CUSTOM if self.is_custom() else self.base_preset(),
                "params": self.values()}

    def reset_to_preset(self) -> None:
        self.set_values(self._schema.preset_values(self.base_preset()))

    def _revalidate(self, *_args) -> None:
        if self._loading:
            return
        values = self.values()          # read every editor once per change
        self._sync_cards(values)
        problems = pf.validate(values, self._schema)[1]
        self._problem.setText("Not valid: " + "; ".join(problems) if problems else "")
        self._buttons.button(QDialogButtonBox.StandardButton.Ok).setEnabled(not problems)
        self._save_button.setEnabled(not problems)
        base = self._schema.preset_values(self.base_preset())
        if problems:
            self._state.setText("")
        elif not pf.same_params(values, base):
            changed = pf.changed_names(values, base)
            self._state.setText(f"custom — {len(changed)} value(s) differ from "
                                f"{self.base_preset()}: {', '.join(changed)}")
        else:
            self._state.setText(f"= preset {self.base_preset()}")

    # ----------------------------------------------------------- files
    def filter_dir(self) -> Path:
        d = Path(self._cfg.pinger.filter_dir).expanduser()
        d.mkdir(parents=True, exist_ok=True)
        return d

    def load_yaml(self, path: Path) -> str | None:
        """Load a parameter file; the reason it was refused, or None. A
        refused file changes nothing (no value is clamped or guessed)."""
        try:
            preset, values = pf.read_param_file(Path(path), self._schema,
                                                self.base_preset())
        except ValueError as exc:
            return str(exc)
        if preset in self._schema.presets:
            self._preset.blockSignals(True)
            self._preset.setCurrentText(preset)
            self._preset.blockSignals(False)
        self.set_values(values)
        return None

    def save_yaml(self, path: Path) -> Path:
        """Write the current set (``{schema, preset, params}``); ValueError
        when it is not valid."""
        problems = self.problems()
        if problems:
            raise ValueError("; ".join(problems))
        block = self.result_block()
        return pf.write_param_file(Path(path), block["preset"], block["params"])

    def _on_load_clicked(self) -> None:
        name, _ = QFileDialog.getOpenFileName(
            self, "Load pinger filter parameters", str(self.filter_dir()),
            "Parameter files (*.yaml *.yml)")
        if not name:
            return
        problem = self.load_yaml(Path(name))
        if problem:
            QMessageBox.warning(self, "Pinger filter", f"Not loaded: {problem}")

    def _on_save_clicked(self) -> None:
        name, _ = QFileDialog.getSaveFileName(
            self, "Save pinger filter parameters", str(self.filter_dir()),
            "Parameter files (*.yaml *.yml)")
        if not name:
            return
        path = Path(name)
        if path.suffix not in (".yaml", ".yml"):
            path = path.with_suffix(".yaml")
        try:
            self.save_yaml(path)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, "Pinger filter", f"Not saved: {exc}")

    def accept(self) -> None:
        if self.problems():
            return
        super().accept()


class PingerForm(QObject):
    """The pinger rows of the Launch and Update Mission dialogs."""

    mode_changed = Signal(str)

    def __init__(self, cfg: AppConfig, parent: QWidget) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self._parent = parent
        self.schema = pf.load_schema_or_none(cfg.pinger.schema_file,
                                             cfg.pinger.schema_checkout)
        self._custom: dict | None = None

        self.mode = QComboBox()
        for key, label in MODE_ITEMS:
            self.mode.addItem(label, key)
        self.mode.setToolTip(
            "Off: the run follows its path, or with '(no path)' only holds Manual\n"
            "Target / arrow keys; the pinger is still tracked and shown.\n"
            "Follow pinger: the run steers to the pinger (no path; PID or LoS).\n"
            "Log pinger only: as Off, and the pinger is logged in the run's CSV\n"
            "and per-fix log. Source and filter appear when the pinger is not Off.")
        self.mode.currentIndexChanged.connect(
            lambda _i: self.mode_changed.emit(self.mode_value()))

        self.source = QComboBox()
        for key, label in SOURCE_ITEMS:
            self.source.addItem(label, key)
        self.set_source(cfg.pinger.default_source)
        self.source.setToolTip(
            "What /blueboat/pinger_coordinates carries, i.e. what a pinger-\n"
            "following run steers to: the filtered estimate (default), or the\n"
            "legacy vector, dead-reckoned by the boat between USBL fixes.")

        self.preset = QComboBox()
        names = self.schema.preset_names() if self.schema else []
        if not names:
            names = [cfg.pinger.default_preset]
        _fill_preset_combo(self.preset, self.schema, names)
        if cfg.pinger.default_preset in names:
            self.preset.setCurrentText(cfg.pinger.default_preset)
        self.preset.setToolTip("Filter preset (BlueBoat-Control config/pinger_filter.yaml).")
        self.preset.currentTextChanged.connect(self._on_preset_changed)
        self.advanced = QPushButton("Advanced…")
        self.advanced.setEnabled(self.schema is not None)
        self.advanced.setToolTip(
            "Every filter parameter, generated from the schema."
            if self.schema else "The filter schema (pinger_filter.yaml) was not "
            "found: only the preset can be chosen.")
        self.advanced.clicked.connect(self.open_advanced)
        self.filter_row = QWidget()
        row = QHBoxLayout(self.filter_row)
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(self.preset, 1)
        row.addWidget(self.advanced)
        self.note = QLabel()
        self.note.setWordWrap(True)
        self.note.setStyleSheet(f"color: {theme.TEXT_DIM}; font-size: 10px;")
        self._refresh_note()

    # ------------------------------------------------------------- layout
    def add_mode_row(self, form: QFormLayout, label: str = "Pinger") -> None:
        form.addRow(label, self.mode)

    def add_filter_rows(self, form: QFormLayout) -> None:
        form.addRow("Pinger source", self.source)
        form.addRow("Pinger filter", self.filter_row)
        form.addRow("", self.note)

    def set_rows_visible(self, form: QFormLayout, mode: bool, filt: bool) -> None:
        for field, on in ((self.mode, mode), (self.source, filt),
                          (self.filter_row, filt), (self.note, filt)):
            field.setVisible(on)
            label = form.labelForField(field)
            if label is not None:
                label.setVisible(on)

    # ------------------------------------------------------------- values
    def mode_value(self) -> str:
        return str(self.mode.currentData() or pf.MODE_OFF)

    def set_mode(self, mode: str) -> None:
        idx = self.mode.findData(mode)
        if idx >= 0:
            self.mode.setCurrentIndex(idx)

    def source_value(self) -> str:
        return str(self.source.currentData() or pf.SOURCE_FILTER)

    def set_source(self, source: str) -> None:
        idx = self.source.findData(source)
        if idx >= 0:
            self.source.setCurrentIndex(idx)

    def preset_value(self) -> str:
        return self.preset.currentText()

    def is_custom(self) -> bool:
        return self._custom is not None

    def values(self) -> dict | None:
        """The full parameter set (None without a schema)."""
        if self._custom is not None:
            return dict(self._custom)
        return self.schema.preset_values(self.preset_value()) if self.schema else None

    def filter_block(self) -> dict:
        """The configure's ``{"preset", "params"}`` block."""
        return pf.filter_block(self.schema, self.preset_value(), self._custom)

    def restore(self, preset: str, block: dict | None) -> None:
        """Show a previous choice: its base preset, and its values when they
        differ from that preset (``custom``)."""
        if preset and self.preset.findText(preset) >= 0:
            self.preset.blockSignals(True)
            self.preset.setCurrentText(preset)
            self.preset.blockSignals(False)
        self._custom = None
        params = dict((block or {}).get("params") or {})
        if self.schema is not None and params and not pf.same_params(
                params, self.schema.preset_values(self.preset_value())):
            self._custom = params
        self._refresh_note()

    def apply_advanced(self, base_preset: str, block: dict) -> None:
        """Take the Advanced panel's result."""
        self.restore(base_preset, block)

    def open_advanced(self) -> None:
        if self.schema is None:
            return
        dlg = PingerFilterDialog(self._cfg, self.schema, self.preset_value(),
                                 self.values(), self._parent)
        if dlg.exec() == QDialog.DialogCode.Accepted:
            self.apply_advanced(dlg.base_preset(), dlg.result_block())

    def _on_preset_changed(self, _name: str) -> None:
        self._custom = None          # a new preset replaces edited values
        self._refresh_note()

    def _refresh_note(self) -> None:
        if self.schema is None:
            self.note.setText("Filter schema not found (BlueBoat-Control "
                              "config/pinger_filter.yaml): the preset is sent by "
                              "name, Control fills in its values.")
        elif self._custom is not None:
            n = len(pf.changed_names(self._custom,
                                     self.schema.preset_values(self.preset_value())))
            self.note.setText(f"custom — {n} value(s) changed from {self.preset_value()} "
                              "(Advanced…)")
        else:
            self.note.setText(self.schema.preset_description(self.preset_value()))
