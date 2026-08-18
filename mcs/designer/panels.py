"""Mission Pattern Designer — side panels and auto-generated dialogs.

* :class:`MissionTree` — the structured mission view (groups → waypoints)
  with reorder / lock / rename, synchronized with the map selection.
* :class:`PropertiesPanel` — edits the selected waypoint (name, x, y, lock)
  and its **outgoing segment** (interpolation type + parameters, form built
  from the interpolation schema), or a selected group (name, lock, pattern
  parameters with regeneration).
* :class:`PatternLibrary` — one button per registered pattern; parameters
  are collected by :class:`SchemaDialog`, an auto-form built from the
  generator schema (anchor ``x0``/``y0`` rows injected).
* :class:`SchemaDialog` / :func:`parse_latlon` — shared helpers.
"""

from __future__ import annotations

import re

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFormLayout, QGridLayout, QGroupBox, QLabel, QLineEdit, QPushButton,
    QSpinBox, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget,
)

from mcs.designer import interpolation, patterns
from mcs.designer.model import MissionModel, PatternGroup, Waypoint
from mcs.gui import theme

_UID_ROLE = Qt.ItemDataRole.UserRole


# ============================================================== schema forms
def _make_editor(kind: str, default, minimum, maximum):
    if kind == "int":
        w = QSpinBox()
        w.setRange(int(minimum or 0), int(maximum or 10_000))
        w.setValue(int(default))
        return w, lambda: w.value()
    if kind.startswith("choice:"):
        w = QComboBox()
        w.addItems(kind.split(":", 1)[1].split("|"))
        w.setCurrentText(str(default))
        return w, lambda: w.currentText()
    w = QDoubleSpinBox()
    w.setDecimals(3)
    w.setRange(float(minimum if minimum is not None else -1e6),
               float(maximum if maximum is not None else 1e6))
    w.setValue(float(default))
    return w, lambda: w.value()


class SchemaForm(QWidget):
    """A form auto-built from a schema; values() returns the param dict."""

    edited = Signal()

    def __init__(self, schema: list[tuple], values: dict | None = None,
                 parent=None) -> None:
        super().__init__(parent)
        layout = QFormLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self._getters: dict[str, callable] = {}
        values = values or {}
        for key, label, kind, default, minimum, maximum in schema:
            widget, getter = _make_editor(kind, values.get(key, default),
                                          minimum, maximum)
            if hasattr(widget, "valueChanged"):
                widget.valueChanged.connect(lambda *_: self.edited.emit())
            elif hasattr(widget, "currentTextChanged"):
                widget.currentTextChanged.connect(lambda *_: self.edited.emit())
            layout.addRow(label, widget)
            self._getters[key] = getter

    def values(self) -> dict:
        return {k: g() for k, g in self._getters.items()}


class SchemaDialog(QDialog):
    """Modal parameter dialog built from a schema (+ optional anchor rows)."""

    def __init__(self, title: str, schema: list[tuple],
                 values: dict | None = None, anchor: tuple[float, float] | None = None,
                 parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle(title)
        layout = QVBoxLayout(self)
        rows = list(schema)
        if anchor is not None:
            rows = [("x0", "Anchor X (m)", "float", anchor[0], -1e6, 1e6),
                    ("y0", "Anchor Y (m)", "float", anchor[1], -1e6, 1e6)] + rows
        self.form = SchemaForm(rows, values)
        layout.addWidget(self.form)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok
                                   | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def values(self) -> dict:
        return self.form.values()


_LATLON_RE = re.compile(
    r"^\s*(-?\d+(?:\.\d+)?)\s*[,;\s]\s*(-?\d+(?:\.\d+)?)\s*$")


def parse_latlon(text: str) -> tuple[float, float] | None:
    """Parse Google-Maps-style '33.660196, 130.657780' coordinates."""
    m = _LATLON_RE.match(text)
    if not m:
        return None
    lat, lon = float(m.group(1)), float(m.group(2))
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    return lat, lon


# ================================================================= tree panel
class MissionTree(QWidget):
    """Structured mission view with reorder / delete / group controls."""

    selection_changed = Signal()
    #: (verb, payload) — handled by the window, which owns undo
    action = Signal(str, object)

    def __init__(self, model: MissionModel, parent=None) -> None:
        super().__init__(parent)
        self._model = model
        self._syncing = False
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Mission", "🔒"])
        self.tree.setColumnWidth(0, 190)
        self.tree.setSelectionMode(QTreeWidget.SelectionMode.ExtendedSelection)
        self.tree.itemSelectionChanged.connect(self._on_selection)
        self.tree.itemChanged.connect(self._on_item_changed)
        layout.addWidget(self.tree, stretch=1)

        grid = QGridLayout()
        buttons = [
            ("▲ Up", lambda: self.action.emit("move", -1)),
            ("▼ Down", lambda: self.action.emit("move", +1)),
            ("Duplicate", lambda: self.action.emit("duplicate", None)),
            ("Delete", lambda: self.action.emit("delete", None)),
            ("Group", lambda: self.action.emit("group", None)),
            ("Explode", lambda: self.action.emit("ungroup", None)),
            ("Lock", lambda: self.action.emit("lock", True)),
            ("Unlock", lambda: self.action.emit("lock", False)),
        ]
        for i, (label, slot) in enumerate(buttons):
            b = QPushButton(label)
            b.clicked.connect(slot)
            grid.addWidget(b, i // 4, i % 4)
        layout.addLayout(grid)

    # ----------------------------------------------------------------- sync
    def rebuild(self) -> None:
        self._syncing = True
        self.tree.clear()
        for item in self._model.items:
            if isinstance(item, PatternGroup):
                node = QTreeWidgetItem(
                    [f"Pattern : {item.name}" if item.pattern != "group"
                     else f"Group : {item.name}", "🔒" if item.locked else ""])
                node.setData(0, _UID_ROLE, item.uid)
                node.setFlags(node.flags() | Qt.ItemFlag.ItemIsEditable)
                for w in item.children:
                    child = QTreeWidgetItem([w.name, "🔒" if w.locked else ""])
                    child.setData(0, _UID_ROLE, w.uid)
                    child.setFlags(child.flags() | Qt.ItemFlag.ItemIsEditable)
                    node.addChild(child)
                self.tree.addTopLevelItem(node)
                node.setExpanded(True)
            else:
                node = QTreeWidgetItem([item.name, "🔒" if item.locked else ""])
                node.setData(0, _UID_ROLE, item.uid)
                node.setFlags(node.flags() | Qt.ItemFlag.ItemIsEditable)
                self.tree.addTopLevelItem(node)
        self._syncing = False

    def selected_uids(self) -> set[int]:
        return {it.data(0, _UID_ROLE) for it in self.tree.selectedItems()}

    def select_uids(self, uids: set[int]) -> None:
        self._syncing = True
        for i in range(self.tree.topLevelItemCount()):
            top = self.tree.topLevelItem(i)
            top.setSelected(top.data(0, _UID_ROLE) in uids)
            for j in range(top.childCount()):
                child = top.child(j)
                child.setSelected(child.data(0, _UID_ROLE) in uids)
        self._syncing = False

    def _on_selection(self) -> None:
        if not self._syncing:
            self.selection_changed.emit()

    def _on_item_changed(self, item: QTreeWidgetItem, column: int) -> None:
        if self._syncing or column != 0:
            return
        uid = item.data(0, _UID_ROLE)
        name = item.text(0).split(" : ", 1)[-1].strip()
        self.action.emit("rename", (uid, name))


# ============================================================ properties panel
class PropertiesPanel(QWidget):
    """Edits the current single selection (waypoint / group)."""

    #: (verb, payload) — same routing as the tree
    action = Signal(str, object)

    def __init__(self, model: MissionModel, parent=None) -> None:
        super().__init__(parent)
        self._model = model
        self._uid: int | None = None
        self._pos_spins: dict[str, QDoubleSpinBox] = {}
        self._gps_label: QLabel | None = None
        #: callable returning the active GeoFit or None (set by the window)
        self.fit_provider = None
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._placeholder = QLabel("Select one item to edit its properties.")
        self._placeholder.setStyleSheet(f"color: {theme.TEXT_DIM};")
        self._layout.addWidget(self._placeholder)
        self._body: QWidget | None = None

    def refresh_values(self) -> None:
        """Live update of the X/Y read-outs while the waypoint is dragged.

        Values are pushed with signals blocked, so refreshing can neither
        loop back into a ``set_pos`` action nor steal keyboard focus (the
        form is NOT rebuilt — only the two spinboxes are updated)."""
        if self._uid is None or not self._pos_spins:
            return
        wp = self._model.waypoint(self._uid)
        if wp is None:
            return
        for axis, spin in self._pos_spins.items():
            value = getattr(wp, axis)
            if abs(spin.value() - value) > 1e-9 and not spin.hasFocus():
                spin.blockSignals(True)
                spin.setValue(value)
                spin.blockSignals(False)
        if self._gps_label is not None:
            fit = self.fit_provider() if self.fit_provider else None
            if fit is not None:
                lat, lon = fit.world_to_latlon(wp.x, wp.y)
                self._gps_label.setText(f"{lat:.6f}, {lon:.6f}")
            else:
                self._gps_label.setText("— (no georeference)")

    def show_selection(self, uids: set[int]) -> None:
        if self._body is not None:
            self._body.deleteLater()
            self._body = None
        self._uid = None
        self._pos_spins = {}
        self._gps_label = None
        single = self._model.item(next(iter(uids))) if len(uids) == 1 else None
        self._placeholder.setVisible(single is None)
        if single is None:
            return
        self._uid = single.uid
        self._body = (self._waypoint_form(single)
                      if isinstance(single, Waypoint)
                      else self._group_form(single))
        self._layout.addWidget(self._body)

    # -------------------------------------------------------------- builders
    def _waypoint_form(self, wp: Waypoint) -> QWidget:
        box = QGroupBox(f"Waypoint · {wp.name}")
        form = QFormLayout(box)
        name = QLineEdit(wp.name)
        name.editingFinished.connect(
            lambda: self.action.emit("rename", (wp.uid, name.text())))
        form.addRow("Name", name)
        for axis in ("x", "y"):
            spin = QDoubleSpinBox()
            spin.setRange(-1e6, 1e6)
            spin.setDecimals(3)
            spin.setValue(getattr(wp, axis))
            spin.valueChanged.connect(
                lambda v, a=axis: self.action.emit("set_pos", (wp.uid, a, v)))
            form.addRow(axis.upper() + " (m)", spin)
            self._pos_spins[axis] = spin
        locked = QCheckBox("Locked")
        locked.setChecked(wp.locked)
        locked.toggled.connect(
            lambda on: self.action.emit("lock_one", (wp.uid, on)))
        form.addRow("", locked)

        # Read-only GPS readout (live) when a georeference is available.
        self._gps_label = QLabel("—")
        self._gps_label.setStyleSheet(f"color: {theme.TEXT_DIM};")
        form.addRow("GPS", self._gps_label)

        seg_box = QGroupBox("Segment → next waypoint")
        seg_layout = QVBoxLayout(seg_box)
        combo = QComboBox()
        for key, interp in interpolation.REGISTRY.items():
            combo.addItem(interp.label, key)
        idx = combo.findData(wp.seg_out.kind)
        combo.setCurrentIndex(max(idx, 0))
        seg_layout.addWidget(combo)
        speed_row = QFormLayout()
        speed = QDoubleSpinBox()
        speed.setRange(0.0, 10.0)
        speed.setDecimals(2)
        speed.setSingleStep(0.1)
        speed.setSpecialValueText("mission speed")   # shown at 0.0
        speed.setValue(wp.seg_out.speed)
        speed.setToolTip("Cruise speed on this segment; 'mission speed' (0) "
                         "uses the mission-wide setting.")
        speed_row.addRow("Speed (m/s)", speed)
        seg_layout.addLayout(speed_row)
        interp = interpolation.REGISTRY.get(wp.seg_out.kind,
                                            interpolation.REGISTRY["straight"])
        params_form = SchemaForm(interp.schema,
                                 {**interp.defaults(), **wp.seg_out.params})
        seg_layout.addWidget(params_form)

        def commit() -> None:
            self.action.emit("segment",
                             (wp.uid, combo.currentData(), params_form.values(),
                              speed.value()))

        params_form.edited.connect(commit)
        speed.valueChanged.connect(lambda *_: commit())
        combo.currentIndexChanged.connect(
            lambda *_: self.action.emit("segment",
                                        (wp.uid, combo.currentData(), {},
                                         speed.value())))
        form.addRow(seg_box)
        return box

    def _group_form(self, group: PatternGroup) -> QWidget:
        box = QGroupBox(f"Pattern · {group.name}"
                        if group.pattern != "group" else f"Group · {group.name}")
        layout = QVBoxLayout(box)
        name = QLineEdit(group.name)
        name.editingFinished.connect(
            lambda: self.action.emit("rename", (group.uid, name.text())))
        layout.addWidget(name)
        locked = QCheckBox("Locked")
        locked.setChecked(group.locked)
        locked.toggled.connect(
            lambda on: self.action.emit("lock_one", (group.uid, on)))
        layout.addWidget(locked)
        if group.pattern in patterns.REGISTRY:
            edit = QPushButton("Edit pattern parameters…")
            edit.clicked.connect(
                lambda: self.action.emit("edit_pattern", group.uid))
            layout.addWidget(edit)
        explode = QPushButton("Explode into waypoints")
        explode.clicked.connect(
            lambda: self.action.emit("ungroup_one", group.uid))
        layout.addWidget(explode)
        return box


# ============================================================ pattern library
class PatternLibrary(QWidget):
    """One button per registered survey pattern."""

    pattern_requested = Signal(str)   # pattern key

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        for i, (key, pattern) in enumerate(patterns.REGISTRY.items()):
            b = QPushButton(pattern.label)
            b.clicked.connect(lambda _=False, k=key: self.pattern_requested.emit(k))
            grid.addWidget(b, i // 2, i % 2)
