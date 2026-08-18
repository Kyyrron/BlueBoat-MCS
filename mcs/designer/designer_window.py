"""Mission Pattern Designer — main window.

A small application inside the Mission Control Station: file toolbar
(New / Open / Save / Save As / Duplicate / Rename / Delete), editing toolbar
(add-waypoints mode, align / distribute, copy / paste / duplicate-offset,
undo / redo), the interactive :class:`~mcs.designer.designer_map.
DesignerMapView`, and a right column with the mission tree, the properties
panel, the pattern library and the mission settings (speed / loop /
comment). Live robot / pinger overlays and the satellite layer come from
the main station's :class:`~mcs.models.store.DataStore` when available; a
manual **Set GPS Origin** (Google-Maps format) provides georeferencing
otherwise.

Undo/redo is snapshot-based (see :mod:`mcs.designer.model`); every mutating
entry point calls :meth:`_push_undo` first.
"""

from __future__ import annotations

import logging
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (
    QCheckBox, QDialog, QDialogButtonBox, QDoubleSpinBox, QFormLayout,
    QHBoxLayout, QInputDialog, QLabel, QLineEdit, QListWidget, QMainWindow,
    QMessageBox, QPushButton, QScrollArea, QSplitter, QToolBar, QVBoxLayout,
    QWidget,
)

from mcs.config.settings import AppConfig
from mcs.core.geo import GeoFit
from mcs.designer import io_yaml, patterns
from mcs.designer.designer_map import DesignerMapView, EditMode
from mcs.designer.model import MissionModel, PatternGroup, Waypoint
from mcs.designer.panels import (
    MissionTree, PatternLibrary, PropertiesPanel, SchemaDialog, parse_latlon,
)
from mcs.designer.sampling import sample_mission
from mcs.gui import theme
from mcs.gui.widgets import CollapsibleSection

_LOG = logging.getLogger(__name__)


class DesignerWindow(QMainWindow):
    """The Survey Pattern editor window (non-modal)."""

    def __init__(self, cfg: AppConfig, store=None, parent=None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self._store = store            # main DataStore | None (overlays, geo)
        self._dir = Path(cfg.designer.trajectories_dir)
        self.setWindowTitle("Survey Pattern Designer")
        self.resize(1250, 800)

        self.model = MissionModel()
        self.model.speed = cfg.designer.default_speed_mps
        self._undo: list[dict] = []
        self._redo: list[dict] = []
        self._clipboard: list[dict] = []
        self._dirty = False
        self._manual_fit: GeoFit | None = None

        # ---- Central map + right column -----------------------------------
        self.map = DesignerMapView(cfg, self.model)
        right = self._build_right_column()
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self.map)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 1)
        splitter.setSizes([880, 360])
        self.setCentralWidget(splitter)

        self._build_toolbar()
        self._stats = QLabel("")
        self.statusBar().addPermanentWidget(self._stats)

        # ---- Wiring -----------------------------------------------------------
        self.model.structure_changed.connect(self._on_structure_changed)
        self.model.changed.connect(self._schedule_resample)
        self.model.changed.connect(self.props.refresh_values)
        self.map.selection_changed.connect(self._on_map_selection)
        self.map.edit_started.connect(self._push_undo)
        self.map.point_added.connect(self._on_point_added)
        self.tree.selection_changed.connect(self._on_tree_selection)
        self.tree.action.connect(self._on_action)
        self.props.action.connect(self._on_action)
        self.library.pattern_requested.connect(self._on_pattern_requested)
        self.props.fit_provider = self._active_fit

        self._resample_timer = QTimer(self)
        self._resample_timer.setSingleShot(True)
        self._resample_timer.setInterval(60)
        self._resample_timer.timeout.connect(self._resample)

        self._overlay_timer = QTimer(self)
        self._overlay_timer.timeout.connect(self._refresh_overlays)
        self._overlay_timer.start(250)

        self._on_structure_changed()
        self._update_geo_fit(center=True)

    # ================================================================ layout
    def _build_right_column(self) -> QWidget:
        column = QWidget()
        column.setMinimumWidth(330)
        outer = QVBoxLayout(column)
        outer.setContentsMargins(4, 4, 4, 4)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        outer.addWidget(scroll)
        inner = QWidget()
        scroll.setWidget(inner)
        layout = QVBoxLayout(inner)
        layout.setSpacing(6)

        sec_tree = CollapsibleSection("MISSION")
        self.tree = MissionTree(self.model)
        self.tree.setMinimumHeight(220)
        sec_tree.add_widget(self.tree)
        layout.addWidget(sec_tree, stretch=2)

        sec_props = CollapsibleSection("PROPERTIES")
        self.props = PropertiesPanel(self.model)
        sec_props.add_widget(self.props)
        layout.addWidget(sec_props)

        sec_lib = CollapsibleSection("PATTERN LIBRARY")
        self.library = PatternLibrary()
        sec_lib.add_widget(self.library)
        layout.addWidget(sec_lib)

        sec_settings = CollapsibleSection("MISSION SETTINGS")
        settings = QWidget()
        form = QFormLayout(settings)
        form.setContentsMargins(0, 0, 0, 0)
        self._speed = QDoubleSpinBox()
        self._speed.setRange(0.05, 10.0)
        self._speed.setSingleStep(0.1)
        self._speed.setValue(self.model.speed)
        self._speed.valueChanged.connect(self._on_speed)
        form.addRow("Cruise speed (m/s)", self._speed)
        self._loop = QCheckBox("Loop mission (close last → first)")
        self._loop.toggled.connect(self._on_loop)
        form.addRow("", self._loop)
        self._comment = QLineEdit()
        self._comment.setPlaceholderText("comment (stored in metadata)")
        self._comment.editingFinished.connect(self._on_comment)
        form.addRow("Comment", self._comment)
        sec_settings.add_widget(settings)
        layout.addWidget(sec_settings)
        layout.addStretch(1)
        return column

    def _build_toolbar(self) -> None:
        def act(bar: QToolBar, text: str, slot, shortcut: str | None = None,
                checkable: bool = False) -> QAction:
            action = QAction(text, self)
            action.triggered.connect(slot)
            if shortcut:
                action.setShortcut(QKeySequence(shortcut))
            action.setCheckable(checkable)
            bar.addAction(action)
            return action

        files = QToolBar("File")
        files.setMovable(False)
        self.addToolBar(files)
        act(files, "New", self._file_new, "Ctrl+N")
        act(files, "Open…", self._file_open, "Ctrl+O")
        act(files, "Save", self._file_save, "Ctrl+S")
        act(files, "Save As…", self._file_save_as, "Ctrl+Shift+S")
        files.addSeparator()
        act(files, "Set GPS Origin…", self._set_gps_origin)
        files.addSeparator()
        self._sat_box = QCheckBox("Satellite")
        self._sat_box.setEnabled(False)
        self._sat_box.toggled.connect(self.map.tiles.set_enabled)
        files.addWidget(self._sat_box)
        self._grid_box = QCheckBox("Grid")
        self._grid_box.setChecked(True)
        self._grid_box.toggled.connect(self._on_grid)
        files.addWidget(self._grid_box)
        self._robot_box = QCheckBox("Robot")
        self._robot_box.setChecked(True)
        files.addWidget(self._robot_box)
        self._pinger_box = QCheckBox("Pinger")
        self._pinger_box.setChecked(True)
        files.addWidget(self._pinger_box)

        edit = QToolBar("Edit")
        edit.setMovable(False)
        self.addToolBar(edit)
        self._add_action = act(edit, "✚ Add Waypoints", self._toggle_add,
                               "A", checkable=True)
        edit.addSeparator()
        act(edit, "Align ─", lambda: self._simple_edit(
            lambda: self.model.align(self._selection(), "y")), "Ctrl+Shift+H")
        act(edit, "Align │", lambda: self._simple_edit(
            lambda: self.model.align(self._selection(), "x")), "Ctrl+Shift+V")
        act(edit, "Distribute", lambda: self._simple_edit(
            lambda: self.model.distribute(self._selection())))
        edit.addSeparator()
        act(edit, "Copy", self._copy, "Ctrl+C")
        act(edit, "Paste", self._paste, "Ctrl+V")
        act(edit, "Duplicate+Offset", lambda: self._on_action("duplicate", None),
            "Ctrl+D")
        act(edit, "Delete", lambda: self._on_action("delete", None), "Del")
        edit.addSeparator()
        act(edit, "Undo", self._undo_op, "Ctrl+Z")
        act(edit, "Redo", self._redo_op, "Ctrl+Y")
        edit.addSeparator()
        act(edit, "Center Pattern", self._center_pattern, "F")
        act(edit, "Zoom +", lambda: self.map.zoom_in(), "+")
        act(edit, "Zoom −", lambda: self.map.zoom_out(), "-")
        edit.addSeparator()
        align_act = act(edit, "Align to Start", self._align_to_start)
        align_act.setToolTip(
            "Rigid-transform the mission so it starts at world (0,0) with "
            "its first tangent along +x — i.e. at the boat, moving forward, "
            "every launch (the world frame is zeroed at launch).")

    # ---------------------------------------------------- start alignment
    def _start_misalignment(self) -> tuple[tuple[float, float], float] | None:
        """(origin, initial tangent angle) if the mission does not start at
        (0,0) heading +x within tolerance, else None."""
        samples = sample_mission(self.model, self._cfg.designer.sample_ds_m)
        if samples.empty or len(samples.xy) < 2:
            return None
        import math as _math
        p0 = (float(samples.xy[0][0]), float(samples.xy[0][1]))
        d = samples.xy[1] - samples.xy[0]
        angle = _math.atan2(float(d[1]), float(d[0]))
        if _math.hypot(*p0) <= 0.05 and abs(angle) <= _math.radians(2.0):
            return None
        return p0, angle

    def _align_to_start(self) -> None:
        mis = self._start_misalignment()
        if mis is None:
            self.statusBar().showMessage(
                "Mission already starts at (0,0) along +x.", 4000)
            return
        self._push_undo()
        self.model.align_to_start(*mis)
        self.map.sync_positions()
        self.statusBar().showMessage(
            "Mission aligned: starts at the boat, first motion forward.", 5000)

    def _maybe_offer_alignment(self) -> None:
        """Before saving a NON-GPS mission that does not start at the boat,
        offer the alignment. GPS-anchored missions are geographically fixed
        and are never realigned (the boat turns toward them instead)."""
        if self._active_fit() is not None:
            return
        mis = self._start_misalignment()
        if mis is None:
            return
        answer = QMessageBox.question(
            self, "Align mission to robot start?",
            "Every launch zeroes the world frame at the boat (origin = boat "
            "position, +x = boat heading). This mission does not start at "
            "(0,0) along +x, so the robot would first cut across to it.\n\n"
            "Align the mission so it starts at the boat and begins by "
            "moving forward?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if answer == QMessageBox.StandardButton.Yes:
            self._push_undo()
            self.model.align_to_start(*mis)
            self.map.sync_positions()

    def _center_pattern(self) -> None:
        """Frame the selection (or, without one, the whole mission) on screen."""
        uids = self._selection()
        wps = [w for w in self.model.flatten() if w.uid in uids] or \
            self.model.flatten()
        if not wps:
            self.statusBar().showMessage("Nothing to center — the mission is "
                                         "empty.", 4000)
            return
        self.map.center_on_bounds([w.x for w in wps], [w.y for w in wps])

    # ================================================================ undo
    def _push_undo(self) -> None:
        self._undo.append(self.model.to_dict())
        if len(self._undo) > self._cfg.designer.undo_depth:
            self._undo.pop(0)
        self._redo.clear()
        self._dirty = True
        self._update_title()

    def _undo_op(self) -> None:
        if not self._undo:
            return
        self._redo.append(self.model.to_dict())
        self.model.from_dict(self._undo.pop())

    def _redo_op(self) -> None:
        if not self._redo:
            return
        self._undo.append(self.model.to_dict())
        self.model.from_dict(self._redo.pop())

    # ============================================================ selection
    def _selection(self) -> set[int]:
        return self.map.selected_uids() | self.tree.selected_uids()

    def _on_map_selection(self) -> None:
        uids = self.map.selected_uids()
        self.tree.select_uids(uids)
        self.props.show_selection(uids)

    def _on_tree_selection(self) -> None:
        uids = self.tree.selected_uids()
        # Selecting a group in the tree selects its waypoints on the map.
        map_uids = set(uids)
        for uid in uids:
            item = self.model.item(uid)
            if isinstance(item, PatternGroup):
                map_uids |= {w.uid for w in item.children}
        self.map.select_uids(map_uids)
        self.props.show_selection(uids)

    # ============================================================== actions
    def _on_action(self, verb: str, payload) -> None:  # noqa: C901 - router
        selection = self._selection()
        if verb == "move" and selection:
            self._push_undo()
            for uid in selection:
                self.model.move_item(uid, payload)
        elif verb == "duplicate" and selection:
            self._push_undo()
            new = self.model.duplicate(selection, offset=2.0)
            self.map.select_uids(set(new))
        elif verb == "delete" and selection:
            self._push_undo()
            self.model.remove(selection)
        elif verb == "group" and selection:
            self._push_undo()
            self.model.group_selection(selection)
        elif verb == "ungroup":
            self._push_undo()
            for uid in list(selection):
                self.model.ungroup(uid)
        elif verb == "ungroup_one":
            self._push_undo()
            self.model.ungroup(payload)
        elif verb == "lock" and selection:
            self._push_undo()
            self.model.set_locked(selection, bool(payload))
            self.map.sync_positions()
            self.tree.rebuild()
        elif verb == "lock_one":
            uid, on = payload
            self._push_undo()
            self.model.set_locked({uid}, on)
            self.map.sync_positions()
            self.tree.rebuild()
        elif verb == "rename":
            uid, name = payload
            self._push_undo()
            self.model.rename(uid, name)
        elif verb == "set_pos":
            uid, axis, value = payload
            wp = self.model.waypoint(uid)
            if wp is not None and not self.model.effective_locked(wp):
                self._push_undo()
                setattr(wp, axis, float(value))
                self.model.changed.emit()
                self.map.sync_positions()
        elif verb == "segment":
            uid, kind, params, speed = payload
            wp = self.model.waypoint(uid)
            if wp is not None:
                self._push_undo()
                changed_kind = wp.seg_out.kind != kind
                wp.seg_out.kind = kind
                wp.seg_out.params = dict(params)
                wp.seg_out.speed = float(speed)
                self.model.changed.emit()
                if changed_kind:
                    self.props.show_selection({uid})
                    self.props.refresh_values()
        elif verb == "edit_pattern":
            self._edit_pattern(payload)

    def _simple_edit(self, fn) -> None:
        self._push_undo()
        fn()
        self.map.sync_positions()

    def _copy(self) -> None:
        order = [w for w in self.model.flatten() if w.uid in self._selection()]
        self._clipboard = [w.to_dict() for w in order]

    def _paste(self) -> None:
        if not self._clipboard:
            return
        self._push_undo()
        new_uids = []
        for d in self._clipboard:
            wp = self.model.add_waypoint(d["x"] + 2.0, d["y"] + 2.0,
                                         name=d.get("name", "") + " copy")
            wp.seg_out = Waypoint.from_dict(d).seg_out
            new_uids.append(wp.uid)
        self.map.select_uids(set(new_uids))

    # ================================================================ editing
    def _toggle_add(self, checked: bool) -> None:
        self.map.set_mode(EditMode.ADD if checked else EditMode.SELECT)
        if checked:
            self.statusBar().showMessage(
                "Add Waypoints: click to add · Shift = axis from previous · "
                "Ctrl = fixed-distance (grid step) · right-click to finish.",
                8000)

    def _on_point_added(self, x: float, y: float) -> None:
        self._push_undo()
        self.model.add_waypoint(x, y)

    def _on_pattern_requested(self, key: str) -> None:
        pattern = patterns.REGISTRY[key]
        center = self.map.mapToScene(self.map.viewport().rect().center())
        dialog = SchemaDialog(pattern.label, pattern.schema,
                              anchor=(round(center.x(), 1), round(center.y(), 1)),
                              parent=self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        params = dialog.values()
        self._push_undo()
        self.model.add_group(pattern.label, key, params,
                             pattern.generate(params))

    def _edit_pattern(self, uid: int) -> None:
        group = self.model.item(uid)
        if not isinstance(group, PatternGroup) \
                or group.pattern not in patterns.REGISTRY:
            return
        pattern = patterns.REGISTRY[group.pattern]
        anchor = (group.params.get("x0", 0.0), group.params.get("y0", 0.0))
        dialog = SchemaDialog(pattern.label, pattern.schema,
                              values=group.params, anchor=anchor, parent=self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        params = dialog.values()
        self._push_undo()
        self.model.regenerate_group(uid, params, pattern.generate(params))

    # =============================================================== refresh
    def _on_structure_changed(self) -> None:
        self.map.rebuild_items()
        self.tree.rebuild()
        self._speed.blockSignals(True)
        self._speed.setValue(self.model.speed)
        self._speed.blockSignals(False)
        self._loop.blockSignals(True)
        self._loop.setChecked(self.model.loop)
        self._loop.blockSignals(False)
        if self._comment.text() != self.model.comment:
            self._comment.setText(self.model.comment)
        self._schedule_resample()
        self._update_title()

    def _schedule_resample(self) -> None:
        self._resample_timer.start()

    def _resample(self) -> None:
        samples = sample_mission(self.model, self._cfg.designer.sample_ds_m)
        self.map.update_preview(samples)
        self.map.refresh_labels()
        n = len(self.model.flatten())
        self._stats.setText(
            f"{n} waypoints · {samples.length_m:.1f} m · "
            f"{samples.duration_s:.0f} s @ {self.model.speed:.2f} m/s")

    def _on_speed(self, value: float) -> None:
        self.model.speed = float(value)
        self._dirty = True
        self._schedule_resample()
        self._update_title()

    def _on_loop(self, on: bool) -> None:
        self._push_undo()
        self.model.loop = on
        self.model.changed.emit()

    def _on_comment(self) -> None:
        self.model.comment = self._comment.text()
        self._dirty = True

    def _on_grid(self, on: bool) -> None:
        self.map.grid_visible = on
        self.map.viewport().update()

    # =============================================================== overlays
    def _refresh_overlays(self) -> None:
        robot = pinger = None
        if self._store is not None:
            r = self._store.robot
            if r.has_odom:
                robot = (r.x, r.y, r.yaw)
            pinger = self._store.pinger.world
        self.map.refresh_overlays(robot, pinger,
                                  self._robot_box.isChecked(),
                                  self._pinger_box.isChecked())

    def _active_fit(self):
        """The georeference the design frame is expressed in: the station's
        live fit when the robot is connected and calibrated, else the manual
        GPS origin, else None."""
        if self._store is not None and self._store.geo.is_valid:
            return self._store.geo.fit
        return self._manual_fit

    def _update_geo_fit(self, center: bool = False) -> None:
        fit = self._active_fit()
        self.map.set_geo_fit(fit)
        self._sat_box.setEnabled(fit is not None)
        if fit is None:
            self._sat_box.setChecked(False)
        if center:
            if self._store is not None and self._store.robot.has_odom:
                self.map.centerOn(self._store.robot.x, self._store.robot.y)
            else:
                self.map.centerOn(0.0, 0.0)

    def _set_gps_origin(self) -> None:
        text, ok = QInputDialog.getText(
            self, "Set GPS Origin",
            "GPS coordinates of the world origin (Google Maps format):\n"
            "example: 33.660196, 130.657780")
        if not ok:
            return
        latlon = parse_latlon(text)
        if latlon is None:
            QMessageBox.warning(self, "Set GPS Origin",
                                "Could not parse coordinates. Expected "
                                "'lat, lon' e.g. 33.660196, 130.657780")
            return
        # World (0,0) := the entered GPS point; axes aligned with east/north.
        self._manual_fit = GeoFit(theta=0.0, tx=0.0, ty=0.0,
                                  lat0=latlon[0], lon0=latlon[1],
                                  rms_m=0.0, n_pairs=0)
        self._update_geo_fit()
        self._sat_box.setChecked(True)   # imagery is what the origin is for
        self.map.centerOn(0.0, 0.0)
        self.statusBar().showMessage(
            f"GPS origin set: {latlon[0]:.6f}, {latlon[1]:.6f} — satellite "
            "layer available.", 6000)

    # ================================================================== files
    def _update_title(self) -> None:
        name = self.model.name or "untitled"
        star = " *" if self._dirty else ""
        self.setWindowTitle(f"Survey Pattern Designer — {name}{star}")

    def _confirm_discard(self) -> bool:
        if not self._dirty:
            return True
        answer = QMessageBox.question(
            self, "Unsaved changes",
            "The current mission has unsaved changes. Discard them?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel)
        return answer == QMessageBox.StandardButton.Yes

    def _file_new(self) -> None:
        if not self._confirm_discard():
            return
        self.model.from_dict({"speed": self._cfg.designer.default_speed_mps,
                              "items": []})
        self._undo.clear()
        self._redo.clear()
        self._dirty = False
        self._update_title()

    def _file_save(self) -> None:
        if not self.model.name:
            self._file_save_as()
            return
        self._maybe_offer_alignment()
        self._write(self.model.name)

    def _file_save_as(self) -> None:
        name, ok = QInputDialog.getText(self, "Save As",
                                        "Mission name (letters, digits, _ -):")
        if not ok or not name:
            return
        if not io_yaml.valid_name(name):
            QMessageBox.warning(self, "Save As", "Invalid name.")
            return
        if io_yaml.runtime_path(self._dir, name).exists():
            answer = QMessageBox.question(
                self, "Overwrite", f"'{name}' already exists. Overwrite?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if answer != QMessageBox.StandardButton.Yes:
                return
        self._maybe_offer_alignment()
        self._write(name)

    def _write(self, name: str) -> None:
        wps = self.model.flatten()
        if not wps:
            QMessageBox.warning(self, "Save", "The mission has no waypoints.")
            return
        samples = sample_mission(self.model, self._cfg.designer.sample_ds_m)
        self.model.name = name
        # Embed the GPS anchor: lat/lon of the design-frame origin + its
        # rotation vs east/north. This is what links every waypoint to real
        # GPS coordinates and lets the station deploy the mission into the
        # robot's per-run world frame (docs/08).
        fit = self._active_fit()
        anchor = None
        if fit is not None:
            import math as _math
            lat0, lon0 = fit.world_to_latlon(0.0, 0.0)
            anchor = {"lat0": lat0, "lon0": lon0,
                      "theta_deg": _math.degrees(fit.theta)}
        path = io_yaml.save_mission(self._dir, name, self.model, samples,
                                    geo_anchor=anchor)
        self._dirty = False
        self._update_title()
        anchored = " · GPS-anchored" if anchor is not None else ""
        self.statusBar().showMessage(
            f"Saved {path} ({len(samples.t)} samples, "
            f"{samples.length_m:.1f} m{anchored}) — available in Launch "
            "Mission → custom paths.", 8000)

    def _file_open(self) -> None:
        if not self._confirm_discard():
            return
        dialog = LibraryDialog(self._dir, self)
        if dialog.exec() == QDialog.DialogCode.Accepted and dialog.selected:
            anchor = io_yaml.load_mission(self._dir, dialog.selected, self.model)
            if anchor is not None:
                # The GPS origin the mission was designed with is remembered:
                # restore it so satellite imagery and GPS readouts are
                # immediately available for further editing.
                self._manual_fit = GeoFit(
                    theta=__import__("math").radians(
                        float(anchor.get("theta_deg", 0.0))),
                    tx=0.0, ty=0.0, lat0=float(anchor["lat0"]),
                    lon0=float(anchor["lon0"]), rms_m=0.0, n_pairs=0)
                self._update_geo_fit()
                self._sat_box.setChecked(True)
            self._undo.clear()
            self._redo.clear()
            self._dirty = False
            self._update_title()

    def closeEvent(self, event) -> None:
        if self._confirm_discard():
            self._overlay_timer.stop()
            self._resample_timer.stop()
            event.accept()
        else:
            event.ignore()


class LibraryDialog(QDialog):
    """Mission library: open / duplicate / rename / delete saved missions."""

    def __init__(self, directory: Path, parent=None) -> None:
        super().__init__(parent)
        self._dir = directory
        self.selected: str | None = None
        self.setWindowTitle("Mission library")
        self.setMinimumSize(380, 380)
        layout = QVBoxLayout(self)
        self._list = QListWidget()
        self._list.itemDoubleClicked.connect(lambda _: self.accept())
        layout.addWidget(self._list)

        row = QHBoxLayout()
        for label, slot in (("Duplicate", self._duplicate),
                            ("Rename", self._rename), ("Delete", self._delete)):
            b = QPushButton(label)
            b.clicked.connect(slot)
            row.addWidget(b)
        layout.addLayout(row)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Open
                                   | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        self._refresh()

    def accept(self) -> None:
        item = self._list.currentItem()
        self.selected = item.text() if item else None
        if self.selected:
            super().accept()

    def _refresh(self) -> None:
        self._list.clear()
        self._list.addItems(io_yaml.list_missions(self._dir))

    def _current(self) -> str | None:
        item = self._list.currentItem()
        return item.text() if item else None

    def _duplicate(self) -> None:
        src = self._current()
        if not src:
            return
        dst, ok = QInputDialog.getText(self, "Duplicate", "New name:",
                                       text=src + "_copy")
        if ok and dst and io_yaml.valid_name(dst):
            io_yaml.duplicate_mission(self._dir, src, dst)
            self._refresh()

    def _rename(self) -> None:
        src = self._current()
        if not src:
            return
        dst, ok = QInputDialog.getText(self, "Rename", "New name:", text=src)
        if ok and dst and io_yaml.valid_name(dst) and dst != src:
            io_yaml.rename_mission(self._dir, src, dst)
            self._refresh()

    def _delete(self) -> None:
        name = self._current()
        if not name:
            return
        answer = QMessageBox.question(
            self, "Delete", f"Delete mission '{name}' (runtime + metadata)?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if answer == QMessageBox.StandardButton.Yes:
            io_yaml.delete_mission(self._dir, name)
            self._refresh()
