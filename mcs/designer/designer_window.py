"""Survey Pattern Designer — main window.

A small application inside the Mission Control Station: file toolbar
(New / Open / Save / Save As / Duplicate / Rename / Delete), editing toolbar
(add-waypoints mode, align / distribute, copy / paste / duplicate-offset,
undo / redo), the interactive :class:`~mcs.designer.designer_map.
DesignerMapView`, and a right column with the mission tree, the properties
panel, the pattern library and the mission settings (speed / loop /
comment).

Georeferencing is **always explicit**: **Set GPS Origin** takes typed
coordinates, a Gazebo world's anchor, or a one-off snapshot of the robot's
current fix, and opening a mission restores its own ``geo_anchor``. The
station's live georeference is never adopted, and no live robot or pinger
is drawn -- see :meth:`DesignerWindow._active_fit` for what that used to
cost.

Undo/redo is snapshot-based (see :mod:`mcs.designer.model`); every mutating
entry point calls :meth:`_push_undo` first.
"""

from __future__ import annotations

import logging
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (
    QCheckBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QMenu,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSplitter,
    QToolBar,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from mcs.config.settings import AppConfig
from mcs.core import worlds
from mcs.core.geo import GeoFit
from mcs.designer import io_yaml, patterns
from mcs.designer.designer_map import DesignerMapView, EditMode
from mcs.designer.model import MissionModel, PatternGroup, Waypoint
from mcs.designer.panels import (
    MissionTree,
    PatternLibrary,
    PropertiesPanel,
    SchemaDialog,
    parse_latlon,
)
from mcs.designer.sampling import sample_mission, start_misalignment
from mcs.gui.widgets import CollapsibleSection

_LOG = logging.getLogger(__name__)


class DesignerWindow(QMainWindow):
    """The Survey Pattern editor window (non-modal).

    Deliberately **independent of live telemetry**. The editor's georeference
    is whatever anchor the operator set or the opened file carries, and the
    only thing the station's :class:`~mcs.models.store.DataStore` is used for
    is :meth:`_set_gps_origin_from_robot`, a one-off snapshot of the boat's
    current fix. Nothing here reacts to a GPS lock arriving, changing or being
    lost, so a mission drawn or edited at any moment is the same mission.
    """

    def __init__(self, cfg: AppConfig, store=None, parent=None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        # Read once, by the "From the Robot's Current Position" action only.
        self._store = store            # main DataStore | None
        self._dir = Path(cfg.designer.trajectories_dir)
        self.setWindowTitle("Survey Pattern Designer")
        self.resize(1250, 800)

        self.model = MissionModel()
        self.model.speed = cfg.designer.default_speed_mps
        self._undo: list[dict] = []
        self._redo: list[dict] = []
        self._clipboard: list[dict] = []
        self._dirty = False
        # The design anchor and where it came from. Always set through
        # _set_anchor; never derived from live telemetry (see _active_fit).
        self._manual_fit: GeoFit | None = None
        self._anchor_source = ""
        # Gazebo world the anchor was taken from (Set GPS Origin ▸ From a
        # Gazebo World): (world dir, parsed metadata). Transient window
        # state — never on the model, never in undo, never in the saved
        # runtime YAML; saving with it set copies the world folder for the
        # new path (see _write).
        self._world_ref: tuple[Path, dict] | None = None

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
        self._anchor_label = QLabel("")
        self.statusBar().addPermanentWidget(self._anchor_label)
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
        gps_btn = QToolButton()
        gps_btn.setText("Set GPS Origin…")
        gps_btn.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        gps_menu = QMenu(gps_btn)
        gps_menu.addAction("Enter Coordinates…", self._set_gps_origin)
        gps_menu.addAction("From a Gazebo World…",
                           self._set_gps_origin_from_world)
        gps_menu.addAction("From the Robot's Current Position…",
                           self._set_gps_origin_from_robot)
        gps_btn.setMenu(gps_menu)
        files.addWidget(gps_btn)
        files.addSeparator()
        self._sat_box = QCheckBox("Satellite")
        self._sat_box.setEnabled(False)
        self._sat_box.toggled.connect(self.map.tiles.set_enabled)
        files.addWidget(self._sat_box)
        self._grid_box = QCheckBox("Grid")
        self._grid_box.setChecked(True)
        self._grid_box.toggled.connect(self._on_grid)
        files.addWidget(self._grid_box)

        edit = QToolBar("Edit")
        edit.setMovable(False)
        self.addToolBar(edit)
        act(edit, "✚ Add Waypoints", self._toggle_add, "A", checkable=True)
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
            "its first tangent along +x. The world frame is local ENU "
            "(origin = launch point, +x = EAST), so an aligned mission "
            "starts at the boat's launch position heading east — mainly "
            "useful for simulation; real missions should be GPS-anchored.")

    # ---------------------------------------------------- start alignment
    def _start_misalignment(self) -> tuple[tuple[float, float], float] | None:
        """(origin, initial tangent angle) if the mission does not start at
        (0,0) heading +x within tolerance, else None.

        Shares its threshold and its maths with the launch dialog's badge
        (see :func:`mcs.designer.sampling.start_misalignment`), so the two
        can never disagree about the same mission."""
        samples = sample_mission(self.model, self._cfg.designer.sample_ds_m)
        return start_misalignment(samples.xy,
                                  self._cfg.designer.start_align_tol_m,
                                  self._cfg.designer.start_align_tol_deg)

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
            "Mission aligned: starts at the launch point, first motion "
            "east (+x).", 5000)

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
            "The world frame is local ENU: origin = the boat's launch "
            "position, +x = EAST. This mission does not start at (0,0) "
            "along +x, so the robot would first cut across to it.\n\n"
            "Align the mission so it starts at the launch point, heading "
            "east? (For real-water missions, prefer a GPS anchor instead.)",
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
    def _on_action(self, verb: str, payload) -> None:  # action router
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

    # ============================================================ georeference
    def _active_fit(self):
        """The design frame's georeference: a translation-free GeoFit whose
        ``(lat0, lon0)`` is the GPS of the design origin, so that a design
        point ``(x, y)`` maps to GPS exactly as :func:`io_yaml.deploy_mission`
        will map it (design frame == local ENU about the anchor).

        It is **only ever** the anchor an operator set explicitly, or the one
        carried by the file being edited -- see :meth:`_set_anchor`. The
        station's live georeference is deliberately NOT consulted: it used to
        be preferred here whenever the app held a GPS lock, which silently
        replaced the mission's own anchor with the boat's launch point. Every
        geographic thing in the editor (tiles, per-waypoint GPS read-outs, the
        world-limits rectangle) then translated by the distance between the
        two, and saving re-wrote ``geo_anchor`` from it, corrupting the file
        on disk. ``None`` means an un-anchored mission, whose design frame is
        the robot's local-ENU world frame by convention (see Align to Start).
        """
        return self._manual_fit

    def _set_anchor(self, fit, source: str) -> None:
        """Adopt *fit* as the design anchor, recording where it came from.

        The single entry point for all four sources (a typed origin, a Gazebo
        world, the robot's current fix, an opened mission), so the read-out and
        the map can never disagree about which anchor is in force."""
        self._manual_fit = fit
        self._anchor_source = source
        self._update_geo_fit()

    def _update_geo_fit(self, center: bool = False) -> None:
        fit = self._active_fit()
        self.map.set_geo_fit(fit)
        self._sat_box.setEnabled(fit is not None)
        if fit is None:
            self._sat_box.setChecked(False)
        self._update_anchor_label(fit)
        if center:
            self.map.centerOn(0.0, 0.0)

    def _update_anchor_label(self, fit) -> None:
        if fit is None:
            self._anchor_label.setText(
                "Anchor: none — design frame = the boat's launch point")
            self._anchor_label.setToolTip(
                "This mission is not GPS-anchored: its coordinates are metres "
                "in the robot's local-ENU world frame, whose origin is wherever "
                "the boat is launched. Set a GPS origin to pin it to the ground.")
            return
        self._anchor_label.setText(
            f"Anchor: {fit.lat0:.6f}, {fit.lon0:.6f} ({self._anchor_source})")
        self._anchor_label.setToolTip(
            "GPS of design (0, 0). Saved into the mission as geo_anchor and "
            "used to deploy it into whatever world frame the boat comes up "
            "with. It never follows the station's live GPS.")

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
        # Design (0,0) := the entered GPS point; axes aligned with east/north.
        self._set_anchor(GeoFit(tx=0.0, ty=0.0,
                                lat0=latlon[0], lon0=latlon[1],
                                rms_m=0.0, n_pairs=0), "typed")
        self._sat_box.setChecked(True)   # imagery is what the origin is for
        self.map.centerOn(0.0, 0.0)
        self.statusBar().showMessage(
            f"GPS origin set: {latlon[0]:.6f}, {latlon[1]:.6f} — satellite "
            "layer available.", 6000)

    def _set_gps_origin_from_robot(self) -> None:
        """Snapshot the boat's CURRENT GPS fix as the design origin, once.

        This is the only place the designer reads live telemetry. The anchor
        it produces is then an ordinary fixed property of the mission -- it
        never re-latches, and a later fix does not move the design frame under
        the operator (which is exactly the failure this window used to have).

        The boat's current position is used rather than its world origin
        because it needs only a raw fix, not a converged georeference, and
        "design (0, 0) = where the boat is now" is what the operator means.
        Which point is chosen does not change where the mission executes: an
        anchored mission is deployed through GPS either way.
        """
        robot = self._store.robot if self._store is not None else None
        if robot is None or robot.lat is None or robot.lon is None:
            QMessageBox.warning(
                self, "From the Robot's Current Position",
                "No GPS fix from the robot.\n\nThe station needs a "
                "/mavros/global_position/global fix before its position can "
                "be used as a design origin. Enter coordinates manually "
                "instead, or wait for a fix.")
            return
        lat, lon = float(robot.lat), float(robot.lon)
        self._clear_world_ref()
        self._set_anchor(GeoFit(tx=0.0, ty=0.0, lat0=lat, lon0=lon,
                                rms_m=0.0, n_pairs=0), "robot")
        self._sat_box.setChecked(True)
        self.map.centerOn(0.0, 0.0)
        self.statusBar().showMessage(
            f"GPS origin set from the robot: {lat:.6f}, {lon:.6f} — this is a "
            "one-off snapshot; the design frame will not follow the boat.",
            8000)

    def _set_gps_origin_from_world(self) -> None:
        """Anchor the design frame on a generated Gazebo world.

        The world's ``geo_anchor`` becomes the design origin (design frame
        == the world's local frame) and its limit rectangle is drawn on the
        map as a read-only reference. Saving then copies the world folder
        for the new path (see :meth:`_write`). A world selected here is
        deliberately kept even if the origin is later retyped — the copy's
        provenance offset is recomputed from the anchor actually saved.
        """
        meta_file, _ = QFileDialog.getOpenFileName(
            self, "Select a world (metadata.yaml)",
            self._cfg.launch.worlds_root, "World metadata (metadata.yaml)")
        if not meta_file:
            return
        mp = Path(meta_file)
        meta = worlds.read_world_meta(mp) \
            if mp.name == worlds.METADATA_NAME else None
        if meta is None:
            QMessageBox.warning(
                self, "From a Gazebo World",
                "Not a world metadata.yaml (expected format "
                "'blueboat_world_meta/1' with geo_anchor and limits).")
            return
        anchor = meta["geo_anchor"]
        self._world_ref = (mp.parent, meta)
        self.map.set_world_limits(
            (meta.get("limits") or {}).get("corners_gps"),
            f"{mp.parent.parent.name}/{mp.parent.name}")
        # Design (0,0) := the world's origin; axes aligned with east/north.
        self._set_anchor(GeoFit(tx=0.0, ty=0.0,
                                lat0=float(anchor["lat0"]),
                                lon0=float(anchor["lon0"]),
                                rms_m=0.0, n_pairs=0),
                         f"world: {mp.parent.name}")
        self._sat_box.setChecked(True)
        self.map.centerOn(0.0, 0.0)
        self.statusBar().showMessage(
            f"GPS origin set from world '{mp.parent.name}' — design frame == "
            "world frame; saving copies the world for the new path.", 8000)

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

    def _clear_world_ref(self) -> None:
        self._world_ref = None
        self.map.set_world_limits(None)

    def _file_new(self) -> None:
        if not self._confirm_discard():
            return
        self.model.from_dict({"speed": self._cfg.designer.default_speed_mps,
                              "items": []})
        self._clear_world_ref()
        # The anchor belongs to the mission, so a new one starts un-anchored
        # rather than silently inheriting the previous mission's origin.
        self._set_anchor(None, "")
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
        # Embed the GPS anchor: lat/lon of the design-frame origin. The
        # design frame is local ENU, so theta_deg is always 0 (kept for
        # legacy readers). This is what links every waypoint to real GPS
        # coordinates and lets the station deploy the mission into the
        # robot's per-run world frame (docs/05_trajectory_format.md).
        fit = self._active_fit()
        anchor = None
        if fit is not None:
            lat0, lon0 = fit.world_to_latlon(0.0, 0.0)
            anchor = {"lat0": lat0, "lon0": lon0, "theta_deg": 0.0}
        path = io_yaml.save_mission(self._dir, name, self.model, samples,
                                    geo_anchor=anchor)
        # A path designed "from a Gazebo World" gets that world copied under
        # its own name (~/worlds/<name>/<world>/), so the launch flow finds
        # it grouped with this path. The runtime YAML itself carries no
        # world data; an already-existing copy is left untouched.
        world_note = ""
        if self._world_ref is not None and anchor is not None:
            world_note = " · " + worlds.duplicate_world_for_path(
                self._world_ref[0], Path(self._cfg.launch.worlds_root),
                name, path, anchor)
        self._dirty = False
        self._update_title()
        anchored = " · GPS-anchored" if anchor is not None else ""
        self.statusBar().showMessage(
            f"Saved {path} ({len(samples.t)} samples, "
            f"{samples.length_m:.1f} m{anchored}) — available in Launch "
            f"Mission → custom paths.{world_note}", 8000)

    def _file_open(self) -> None:
        if not self._confirm_discard():
            return
        dialog = LibraryDialog(self._dir, self)
        if dialog.exec() == QDialog.DialogCode.Accepted and dialog.selected:
            self._clear_world_ref()
            anchor = io_yaml.load_mission(self._dir, dialog.selected, self.model)
            # Whatever the file declares wins, including "nothing": an
            # un-anchored mission must not pick up the previous file's origin
            # and be saved as GPS-anchored to a place it was never drawn at.
            self._set_anchor(None, "")
            if anchor is not None:
                # The GPS origin the mission was designed with is remembered:
                # restore it so satellite imagery and GPS readouts are
                # immediately available for further editing.
                self._set_anchor(GeoFit(
                    tx=0.0, ty=0.0, lat0=float(anchor["lat0"]),
                    lon0=float(anchor["lon0"]), rms_m=0.0, n_pairs=0),
                    "mission")
                self._sat_box.setChecked(True)
                if float(anchor.get("theta_deg", 0.0)) != 0.0:
                    # Legacy anchor: points are in a rotated design frame.
                    # Deployment still honours the rotation; the editor's
                    # imagery/read-outs assume ENU and are approximate here.
                    # Re-saving writes theta_deg 0 with the points AS SHOWN.
                    self.statusBar().showMessage(
                        "Legacy GPS anchor (theta_deg != 0): imagery and GPS "
                        "read-outs assume an ENU design frame — verify before "
                        "re-saving.", 12000)
            self._undo.clear()
            self._redo.clear()
            self._dirty = False
            self._update_title()

    def closeEvent(self, event) -> None:
        if self._confirm_discard():
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
