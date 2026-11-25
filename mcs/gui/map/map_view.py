"""The interactive mission map — GPS-frame-only.

A ``QGraphicsView`` over a scene whose coordinates are **local east/north
metres about the latched GPS origin** ``(lat0, lon0)`` (+y flipped in the view
transform so north is up).  There is exactly ONE scene frame and it never
changes: north-up, east-right, view never rotates (only the robot glyph
rotates, to its true heading).  Every world-frame quantity (odom pose, trails,
targets, mission paths) is placed through the pure translation
``GeoFit.world_to_enu`` (``EN = world + t``); every mouse position read back
out of the view goes through the exact inverse.  ``/blueboat/odom`` is local
ENU on both the real boat and the simulator, so no rotation exists anywhere in
this pipeline — see ``GPS_MAP_ARCHITECTURE.md``.

**Nothing is drawn before the frame is anchored**: on real water — and in a
Gazebo run of a GPS-anchored mission, where the station synthesises the GPS
feed itself (``mission.gps_simulated``) — the map stays empty (with a notice)
until the first fixes establish the translation, then behaves fully
geographically, satellite tiles included. Only a sim WITHOUT simulated GPS
(non-anchored mission) uses the identity anchor and draws immediately, tiles
off — there is no GPS to place them with.

Provides:

* smooth wheel zoom anchored under the cursor, drag panning;
* toggleable layers: satellite, robot trajectory, mission path, the pinger
  (2026-11-24: FILTERED estimate position + track + 95 % ellipse, straight
  from ``/blueboat/pinger/estimate``; LEGACY position + track; raw fixes
  coloured by the filter's decision) with a small pinger legend,
  robot↔target line, metric grid;
* a click inspector (world + GPS coordinates + live distance to robot);
* the Distance Tool (two-click measurement, ported from the SSS viewer);
* the Manual Target tool (publishes ``/blueboat/manual_target``, highlights
  the point, draws the approximated LoS future path and pops a
  "Manual Target Reached" banner).

The view never talks to ROS directly: it emits :attr:`target_clicked` and
reads the :class:`~mcs.models.store.DataStore` on refresh ticks.
"""

from __future__ import annotations

import math
from enum import Enum, auto

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QGraphicsLineItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QLabel,
)

from mcs.config.settings import AppConfig
from mcs.core.geo import latlon_to_local_en
from mcs.core.los_predictor import predict_los_path
from mcs.gui import theme
from mcs.gui.map.legend import PingerLegend
from mcs.gui.map.map_items import (
    CrosshairItem,
    EllipseItem,
    FixPointsItem,
    MarkerItem,
    MissionPathItem,
    PolylineItem,
    RobotItem,
    StartMarkerItem,
    TargetLineItem,
    draw_grid,
    draw_north_indicator,
    draw_scale_bar,
)
from mcs.gui.map.tile_layer import TileLayer
from mcs.models.store import DataStore


class MapMode(Enum):
    NORMAL = auto()
    MANUAL_TARGET = auto()
    MEASURE = auto()


class MapView(QGraphicsView):
    """Central interactive map widget."""

    #: world x, y of a click while in MANUAL_TARGET mode
    target_clicked = Signal(float, float)
    #: formatted text describing the last inspected point ('' clears it)
    point_inspected = Signal(str)

    def __init__(self, cfg: AppConfig, store: DataStore, parent=None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self._store = store
        self._mode = MapMode.NORMAL
        self._window: tuple[float, float] | None = None  # timeline (rel t0, t1)
        self._did_initial_center = False

        # ---- Scene & view behaviour --------------------------------------
        self._scene = QGraphicsScene(self)
        self._scene.setSceneRect(-5e4, -5e4, 1e5, 1e5)
        self.setScene(self._scene)
        self.setRenderHints(QPainter.RenderHint.Antialiasing
                            | QPainter.RenderHint.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.MinimalViewportUpdate)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setBackgroundBrush(QColor(theme.BG_DARK))
        self.scale(20.0, -20.0)  # ~20 px/m initially, y-up (north-up)
        # QGroundControl-style rendering: the map is ALWAYS north-up and
        # never rotates. The scene IS the local east/north (ENU) frame about
        # the latched GPS origin — every world-frame quantity is placed
        # through the pure translation GeoFit.world_to_enu(), tiles are
        # axis-aligned, and only the robot glyph rotates (to its TRUE
        # heading). There is no second regime and no scene switch.

        # ---- Layers --------------------------------------------------------
        self.tiles = TileLayer(self._scene, cfg.map)
        self.robot_track = PolylineItem(theme.C_ROBOT_TRACK, 2.0, z=20)
        self.pinger_track = PolylineItem(theme.C_PINGER_TRACK, 1.6,
                                         Qt.PenStyle.DotLine, z=19)
        self.mission_path = MissionPathItem()
        self.start_marker = StartMarkerItem()   # circled "S": where the run begins
        self.predicted_path = PolylineItem(theme.C_PREDICTED, 1.8,
                                           Qt.PenStyle.DashLine, z=32)
        self.target_line = TargetLineItem()
        self.robot_item = RobotItem()
        # The pinger (2026-11-24): the LEGACY algorithm's position and trail
        # (orange, as always), the FILTERED estimate's (purple) with its 95 %
        # ellipse, and the raw fixes coloured by the filter's decision.
        self.pinger_marker = MarkerItem(theme.C_PINGER, 6.0, "legacy", z=42)
        self.pinger_filtered_marker = MarkerItem(theme.C_PINGER_FILTERED, 7.0,
                                                 "pinger", z=43)
        self.pinger_filtered_track = PolylineItem(theme.C_PINGER_FILTERED_TRACK, 2.0,
                                                  Qt.PenStyle.SolidLine, z=19.5)
        self.pinger_ellipse = EllipseItem(theme.C_PINGER_ELLIPSE, z=18)
        self.pinger_fixes = FixPointsItem(z=17)
        self.manual_marker = CrosshairItem(theme.C_MANUAL_TARGET)
        self.click_marker = MarkerItem(QColor(theme.C_MEASURE), 4.0, "", z=41)
        # (2026-11-25) The run's safety way back (sent to the link guardian at
        # START RUN), dashed orange, its last point marked LOITER. Shown with the
        # mission-path layer; placed from lat/lon in the scene's east/north.
        self.safety_path = PolylineItem(theme.C_SAFETY, 2.0, Qt.PenStyle.DashLine, z=21)
        self.safety_end = MarkerItem(theme.C_SAFETY, 5.0, "LOITER", z=41)
        self._scene.addItem(self.safety_path)
        self._scene.addItem(self.safety_end)
        self.safety_path.setVisible(False)
        self.safety_end.setVisible(False)
        # Change keys of the overlays rebuilt only when their input changes.
        self._safety_drawn = None
        self._filtered_track_key = None
        self._fixes_key = None
        for item in (self.robot_track, self.pinger_track, self.mission_path,
                     self.start_marker,
                     self.predicted_path, self.target_line, self.robot_item,
                     self.pinger_marker, self.pinger_filtered_marker,
                     self.pinger_filtered_track, self.pinger_ellipse,
                     self.pinger_fixes, self.manual_marker, self.click_marker):
            self._scene.addItem(item)
        self.pinger_marker.setVisible(False)
        self.pinger_filtered_marker.setVisible(False)
        self.pinger_ellipse.setVisible(False)
        self.start_marker.setVisible(False)
        # Checkbox *intent* per layer; the refresh() gate may hide any item
        # while the map frame is not anchored, so actual visibility is
        # re-derived from these each tick.
        self._pinger_layer_enabled = True          # legacy marker
        self._robot_track_enabled = True
        self._pinger_track_enabled = True          # legacy trail
        self._pinger_filtered_enabled = True
        self._pinger_filtered_track_enabled = True
        self._pinger_ellipse_enabled = False
        self._pinger_fixes_enabled = False
        self._pinger_legend_enabled = True
        self._mission_path_enabled = True
        self._target_line_enabled = True
        self.manual_marker.setVisible(False)
        self.click_marker.setVisible(False)
        self._grid_visible = True

        # Measure tool state
        self._measure_start: QPointF | None = None
        self._measure_line = QGraphicsLineItem()
        pen = QPen(theme.C_MEASURE, 1.5, Qt.PenStyle.DashLine)
        pen.setCosmetic(True)
        self._measure_line.setPen(pen)
        self._measure_line.setZValue(60)
        self._measure_text = QGraphicsSimpleTextItem()
        self._measure_text.setBrush(QColor(theme.C_MEASURE))
        self._measure_text.setFont(QFont("DejaVu Sans Mono", 9))
        self._measure_text.setFlag(
            self._measure_text.GraphicsItemFlag.ItemIgnoresTransformations)
        self._measure_text.setZValue(61)
        self._scene.addItem(self._measure_line)
        self._scene.addItem(self._measure_text)
        self._measure_line.setVisible(False)
        self._measure_text.setVisible(False)

        # "waiting for GPS" notice — shown while the map frame is not
        # anchored yet (real water, before the first GPS fixes). Nothing is
        # drawn on the map until it disappears.
        self._waiting_label = QLabel(
            "Waiting for GPS fix — the map anchors to the first fixes", self)
        self._waiting_label.setStyleSheet(
            f"background: {theme.WARN}; color: black; font-weight: bold;"
            "padding: 6px 16px; border-radius: 4px;")
        self._waiting_label.hide()

        # "Manual Target Reached" banner
        self._banner = QLabel("Manual Target Reached", self)
        self._banner.setStyleSheet(
            f"background: {theme.OK}; color: black; font-weight: bold;"
            "padding: 6px 16px; border-radius: 4px;")
        self._banner.hide()
        self._banner_timer = QTimer(self)
        self._banner_timer.setSingleShot(True)
        self._banner_timer.timeout.connect(self._banner.hide)
        self._reached_announced = False

        # The pinger legend (bottom-left, above the scale bar).
        self.legend = PingerLegend(self)

    # ================================================================= modes
    def set_mode(self, mode: MapMode) -> None:
        if self._mode is MapMode.MEASURE and mode is not MapMode.MEASURE:
            self._clear_measurement()  # a frozen measurement must not outlive the tool
        self._mode = mode
        drag = (QGraphicsView.DragMode.ScrollHandDrag
                if mode is MapMode.NORMAL else QGraphicsView.DragMode.NoDrag)
        self.setDragMode(drag)
        cursor = Qt.CursorShape.CrossCursor if mode is not MapMode.NORMAL \
            else Qt.CursorShape.OpenHandCursor
        self.viewport().setCursor(cursor)
        if mode is not MapMode.MEASURE:
            self._measure_start = None

    def _clear_measurement(self) -> None:
        self._measure_start = None
        self._measure_line.setVisible(False)
        self._measure_text.setVisible(False)
        self.point_inspected.emit("")

    @property
    def mode(self) -> MapMode:
        return self._mode

    def clear_manual_target(self) -> None:
        self.manual_marker.setVisible(False)
        self.predicted_path.setVisible(False)
        self._reached_announced = False

    def show_manual_target(self, x: float, y: float) -> None:
        """``x, y`` are WORLD metres (what was published); refresh() re-places
        the crosshair from the stored world value every tick."""
        self.manual_marker.set_world_pos(*self._to_scene(x, y))
        self.manual_marker.setVisible(True)
        self.predicted_path.setVisible(True)
        self._reached_announced = False

    def set_time_window(self, rel_t0: float, rel_t1: float, live: bool) -> None:
        self._window = None if live else (rel_t0, rel_t1)

    # ============================================================ visibility
    def set_layer_visible(self, layer: str, visible: bool) -> None:
        def _intent(attr, item):
            def apply(v: bool) -> None:
                setattr(self, attr, v)
                item.setVisible(v and self._store.map_frame_ready())
            return apply

        mapping = {
            "satellite": self.tiles.set_enabled,
            "robot_track": _intent("_robot_track_enabled", self.robot_track),
            "mission_path": _intent("_mission_path_enabled", self.mission_path),
            "pinger": self._set_pinger_layer_enabled,
            "pinger_track": _intent("_pinger_track_enabled", self.pinger_track),
            "pinger_filtered": self._data_gated("_pinger_filtered_enabled",
                                                self.pinger_filtered_marker),
            "pinger_filtered_track": _intent("_pinger_filtered_track_enabled",
                                             self.pinger_filtered_track),
            "pinger_ellipse": self._data_gated("_pinger_ellipse_enabled",
                                               self.pinger_ellipse),
            "pinger_fixes": _intent("_pinger_fixes_enabled", self.pinger_fixes),
            "pinger_legend": self._set_legend_enabled,
            "target_line": _intent("_target_line_enabled", self.target_line),
            "grid": self._set_grid_visible,
        }
        fn = mapping.get(layer)
        if fn:
            fn(visible)

    def _set_pinger_layer_enabled(self, enabled: bool) -> None:
        """The checkbox records *intent* only. The marker is actually shown
        by refresh() as ``enabled AND a pinger position has been published``
        — so re-enabling the layer before any /blueboat/pinger_coordinates
        message can never conjure a ghost dot at the (0, 0) default."""
        self._pinger_layer_enabled = enabled
        if not enabled:
            self.pinger_marker.setVisible(False)

    def _data_gated(self, attr: str, item):
        """Checkbox intent for an item refresh() shows only with data (like
        the legacy marker): switching the layer on never conjures a ghost."""
        def apply(v: bool) -> None:
            setattr(self, attr, v)
            if not v:
                item.setVisible(False)
        return apply

    def _set_legend_enabled(self, enabled: bool) -> None:
        self._pinger_legend_enabled = enabled
        if not enabled:
            self.legend.hide()

    def pinger_layers(self) -> dict[str, bool]:
        """Checkbox intent of every pinger layer (the legend's lines)."""
        return {"pinger_filtered": self._pinger_filtered_enabled,
                "pinger_filtered_track": self._pinger_filtered_track_enabled,
                "pinger_ellipse": self._pinger_ellipse_enabled,
                "pinger_fixes": self._pinger_fixes_enabled,
                "pinger": self._pinger_layer_enabled,
                "pinger_track": self._pinger_track_enabled}

    def _set_grid_visible(self, visible: bool) -> None:
        self._grid_visible = visible
        self.viewport().update()

    def _to_scene(self, wx: float, wy: float) -> tuple[float, float]:
        """World-frame point -> scene (local east/north metres).

        Pure translation ``EN = world + t``. In a non-GPS sim there is no
        fit: the sim world is already ENU, so the identity is the correct
        (and exact) anchor. With simulated GPS a real fit exists."""
        fit = self._store.geo.fit
        if fit is not None:
            return fit.world_to_enu(wx, wy)
        return wx, wy

    def _to_world(self, sx: float, sy: float) -> tuple[float, float]:
        """Scene (east/north metres) -> world frame: the exact inverse of
        :meth:`_to_scene`, under the identical test.

        Every mouse position arrives in scene coordinates. Anything read
        back out of the view — the published manual target above all, but
        also the inspector, the measure read-out and GPS conversion — must
        come back through here."""
        fit = self._store.geo.fit
        if fit is not None:
            return fit.enu_to_world(sx, sy)
        return sx, sy

    def _refresh_safety_path(self, store) -> None:
        pts = store.session.safety_way_back
        fit = store.geo.fit
        show = bool(self._mission_path_enabled and fit is not None and len(pts) >= 2)
        self.safety_path.setVisible(show)
        self.safety_end.setVisible(show)
        if not show:
            return
        # The route changes only at START / STOP RUN (a new list each time)
        # and lat0 / lon0 are latched: project it once, not every tick.
        key = (fit.lat0, fit.lon0)
        if self._safety_drawn is not None and self._safety_drawn[0] is pts \
                and self._safety_drawn[1] == key:
            return
        en = np.array([latlon_to_local_en(lat, lon, fit.lat0, fit.lon0)
                       for lat, lon in pts])
        self.safety_path.set_points(en)
        self.safety_end.set_world_pos(float(en[-1, 0]), float(en[-1, 1]))
        self._safety_drawn = (pts, key)

    def _scene_points(self, xy) -> np.ndarray:
        fit = self._store.geo.fit
        if fit is None or len(xy) == 0:
            return xy
        out = np.empty_like(xy[:, 0:2])
        out[:, 0] = xy[:, 0] + fit.tx
        out[:, 1] = xy[:, 1] + fit.ty
        return out

    def _scene_heading(self, world_yaw: float) -> float:
        """Glyph heading in scene (ENU) axes.

        ``robot_true_heading()`` — compass first, else the odom yaw, which is
        absolute ENU on both the real boat and the simulator — IS the scene
        heading; no correction is applied."""
        true_h = self._store.robot_true_heading()
        return true_h if true_h is not None else world_yaw

    # ================================================================ refresh
    def refresh(self) -> None:
        """Called at the UI tick (10 Hz): pull the store, update items.

        The single gate: nothing is drawn until the map frame is anchored
        (``store.map_frame_ready()`` — GPS translation on real water and in
        GPS-simulated runs, immediately in a non-GPS sim). Sequencing at
        anchor time is thereby fix -> tiles -> glyph -> overlays, all in the
        first ready tick."""
        store = self._store
        robot = store.robot

        ready = store.map_frame_ready()
        self._waiting_label.setVisible(not ready)
        if not ready:
            self._waiting_label.adjustSize()
            self._waiting_label.move(
                (self.width() - self._waiting_label.width()) // 2, 12)
            for item in (self.robot_item, self.robot_track, self.pinger_track,
                         self.mission_path, self.start_marker, self.predicted_path,
                         self.target_line, self.pinger_marker,
                         self.pinger_filtered_marker, self.pinger_filtered_track,
                         self.pinger_ellipse, self.pinger_fixes,
                         self.manual_marker, self.click_marker,
                         self.safety_path, self.safety_end):
                item.setVisible(False)
            self.legend.hide()
            self.tiles.update_view(
                None,
                self.mapToScene(self.viewport().rect()).boundingRect(),
                self._px_per_m(),
            )
            return
        # Layer-visibility intent is owned by the checkboxes; restore what
        # the gate hid (data-gated items are re-decided below each tick).
        self.robot_item.setVisible(True)
        self.robot_track.setVisible(self._robot_track_enabled)
        self.pinger_track.setVisible(self._pinger_track_enabled)
        self.pinger_filtered_track.setVisible(self._pinger_filtered_track_enabled)
        self.pinger_fixes.setVisible(self._pinger_fixes_enabled)
        self.mission_path.setVisible(self._mission_path_enabled)
        self.target_line.setVisible(self._target_line_enabled)

        # Time window (absolute)
        if self._window is None:
            t0, t1 = -math.inf, math.inf
        else:
            t0 = store.t0 + self._window[0]
            t1 = store.t0 + self._window[1]

        max_pts = self._cfg.map.trajectory_max_points_drawn
        _, xy = store.robot_track.decimated_window(t0, t1, max_pts)
        self.robot_track.set_points(self._scene_points(xy[:, 0:2])
                                    if len(xy) else np.empty((0, 2)))
        _, pxy = store.pinger_track.decimated_window(t0, t1, max_pts)
        self.pinger_track.set_points(self._scene_points(pxy)
                                     if len(pxy) else np.empty((0, 2)))
        # The filtered track and the raw fixes change only at the fix rate:
        # rebuilt when their length, the window or the anchor changes, and
        # the fixes only while their layer is on.
        fit = store.geo.fit
        anchor = (fit.tx, fit.ty) if fit is not None else None
        key = (len(store.pinger_est_track), self._window, anchor)
        if key != self._filtered_track_key:
            self._filtered_track_key = key
            _, fxy = store.pinger_est_track.decimated_window(t0, t1, max_pts)
            self.pinger_filtered_track.set_points(self._scene_points(fxy)
                                                  if len(fxy) else np.empty((0, 2)))
        pad_m = 8.0 / max(self._px_per_m(), 1e-6)
        key = (len(store.pinger_fixes), self._window, anchor, pad_m)
        if self._pinger_fixes_enabled and key != self._fixes_key:
            self._fixes_key = key
            _, raw = store.pinger_fixes.decimated_window(t0, t1, max_pts)
            self.pinger_fixes.set_points(
                self._scene_points(raw[:, 0:2]) if len(raw) else np.empty((0, 2)),
                raw[:, 2] if len(raw) else [],
                pad_m=pad_m)

        if robot.has_odom:
            sx, sy = self._to_scene(robot.x, robot.y)
            # Map never rotates; the glyph rotates to its true heading.
            self.robot_item.set_pose(sx, sy, self._scene_heading(robot.yaw))
            if not self._did_initial_center:
                self.centerOn(sx, sy)
                self._did_initial_center = True

        has_pinger = self._store.pinger.world is not None
        self.pinger_marker.setVisible(self._pinger_layer_enabled and has_pinger)
        if has_pinger:
            self.pinger_marker.set_world_pos(*self._to_scene(*self._store.pinger.world))
        # The filtered estimate: its own world x, y and ellipse, placed in
        # the scene like every world quantity (_to_scene), never re-derived.
        est = store.pinger_est
        has_est = est.world is not None
        self.pinger_filtered_marker.setVisible(self._pinger_filtered_enabled and has_est)
        if has_est:
            ex, ey = self._to_scene(*est.world)
            self.pinger_filtered_marker.set_world_pos(ex, ey)
            if est.ellipse is not None:
                self.pinger_ellipse.set_ellipse(ex, ey, *est.ellipse)
        self.pinger_ellipse.setVisible(self._pinger_ellipse_enabled and has_est
                                       and est.ellipse is not None)
        self._refresh_legend(has_pinger or has_est or len(store.pinger_fixes) > 0
                             or len(store.pinger_track) > 0
                             or len(store.pinger_est_track) > 0)

        if store.mission_path is not None:
            self.mission_path.set_points(self._scene_points(store.mission_path[:, 0:2]))
            self.start_marker.set_world_pos(*self._to_scene(
                float(store.mission_path[0, 0]), float(store.mission_path[0, 1])))
        else:
            self.mission_path.set_points(np.empty((0, 2)))
        self.start_marker.setVisible(self._mission_path_enabled
                                     and store.mission_path is not None)
        self._refresh_safety_path(store)

        target = store.active_target_world()
        if target is not None and robot.has_odom:
            rsx, rsy = self._to_scene(robot.x, robot.y)
            tsx, tsy = self._to_scene(*target)
            self.target_line.set_endpoints(rsx, rsy, tsx, tsy)
        else:
            self.target_line.setLine(0, 0, 0, 0)

        self._refresh_manual_target()
        # Tiles need a GPS anchor; in a non-GPS sim fit is None and they
        # stay off. With simulated GPS the anchor is real and tiles show
        # the imagery of the location the mission was planned at.
        self.tiles.update_view(
            store.geo.fit if store.geo.is_valid else None,
            self.mapToScene(self.viewport().rect()).boundingRect(),
            self._px_per_m(),
        )
        if self._grid_visible:
            self.viewport().update()

    def _refresh_legend(self, has_data: bool) -> None:
        """The pinger legend: bottom-left above the scale bar, while some
        pinger data exists and at least one pinger layer is on."""
        lines = self.legend.set_layers(self.pinger_layers())
        show = self._pinger_legend_enabled and has_data and lines
        self.legend.setVisible(show)
        if show:
            pos = (8, max(0, self.height() - self.legend.height() - 34))
            if (self.legend.x(), self.legend.y()) != pos:
                self.legend.move(*pos)
            self.legend.raise_()

    def _refresh_manual_target(self) -> None:
        store = self._store
        mt = store.mission.manual_target
        if mt is None or not store.robot.has_odom:
            self.manual_marker.setVisible(False)
            self.predicted_path.setPath(self.predicted_path.path().__class__())
            return
        # Re-placed every tick from the stored WORLD value, so the crosshair
        # tracks the live anchor and survives having been hidden by the
        # pre-anchor gate.
        self.manual_marker.set_world_pos(*self._to_scene(*mt))
        self.manual_marker.setVisible(True)
        self.predicted_path.setVisible(True)
        pts = predict_los_path(
            (store.robot.x, store.robot.y, store.robot.yaw), mt, self._cfg.los)
        self.predicted_path.set_points(self._scene_points(np.asarray(pts))
                                       if len(pts) else np.empty((0, 2)))
        d = math.hypot(mt[0] - store.robot.x, mt[1] - store.robot.y)
        if d <= self._cfg.los.reached_distance_m and not self._reached_announced:
            self._reached_announced = True
            self._show_banner()

    def _show_banner(self) -> None:
        self._banner.adjustSize()
        self._banner.move((self.width() - self._banner.width()) // 2, 12)
        self._banner.show()
        self._banner.raise_()
        self._banner_timer.start(4000)

    # ============================================================== painting
    def drawBackground(self, painter: QPainter, rect: QRectF) -> None:
        super().drawBackground(painter, rect)
        if not self._grid_visible:
            return
        # The scene is axis-aligned ENU (north-up, always), so a normal
        # scene-space grid is already screen-aligned.
        self._grid_spacing = draw_grid(painter, rect, self._px_per_m(),
                                       high_contrast=self.tiles.enabled)

    def drawForeground(self, painter: QPainter, rect: QRectF) -> None:
        super().drawForeground(painter, rect)
        if self._grid_visible:
            draw_scale_bar(painter, self.viewport().height(), self._px_per_m(),
                           getattr(self, "_grid_spacing", 0.0))
        draw_north_indicator(painter, self.viewport().width())

    def _px_per_m(self) -> float:
        # The view is never rotated (north-up fixed), so the horizontal
        # scale magnitude is the pixels-per-metre.
        return abs(self.transform().m11())

    @property
    def north_up(self) -> bool:
        """Always True: the scene is ENU by construction and never rotates."""
        return True

    # ============================================================= map tools
    def zoom_in(self) -> None:
        self._zoom_by(1.25)

    def zoom_out(self) -> None:
        self._zoom_by(1 / 1.25)

    def _zoom_by(self, factor: float) -> None:
        """Button zoom: anchored on the view center (wheel zoom stays on cursor)."""
        px = self._px_per_m() * factor
        if not (0.05 <= px <= 2000.0):
            return
        previous = self.transformationAnchor()
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.scale(factor, factor)
        self.setTransformationAnchor(previous)

    def center_on_robot(self) -> None:
        """One-shot recenter on the robot's current position.

        Deliberately *not* a follow mode: the camera is repositioned exactly
        once and then remains completely free — subsequent robot motion never
        moves the view. (The only other automatic centering is the identical
        one-shot performed on the very first odometry sample.)
        """
        if self._store.robot.has_odom:
            self.centerOn(*self._to_scene(self._store.robot.x,
                                          self._store.robot.y))
            # A deliberate center also satisfies (and consumes) the startup
            # one-shot, so no automatic recenter can ever move the camera
            # after the operator has positioned it.
            self._did_initial_center = True

    # ================================================================= input
    def wheelEvent(self, event) -> None:
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        px = self._px_per_m() * factor
        if 0.05 <= px <= 2000.0:
            self.scale(factor, factor)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            scene_pt = self.mapToScene(event.position().toPoint())
            if self._mode is MapMode.MANUAL_TARGET:
                if not self._store.map_frame_ready():
                    # Refuse: without the GPS anchor a click cannot be
                    # converted to the robot's world frame.
                    self.point_inspected.emit(
                        "manual target unavailable — waiting for GPS fix")
                    return
                # target_clicked carries WORLD: /blueboat/manual_target is a
                # world-frame topic and master_control does its own frame
                # conversion, so the scene point is inverted here, once, at
                # the view boundary (pure translation, EN - t).
                self.target_clicked.emit(*self._to_world(scene_pt.x(), scene_pt.y()))
                return
            if self._mode is MapMode.MEASURE:
                self._handle_measure_click(scene_pt)
                return
            self._inspect_point(scene_pt)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._mode is MapMode.MEASURE and self._measure_start is not None:
            scene_pt = self.mapToScene(event.position().toPoint())
            self._update_measure(scene_pt, final=False)
        super().mouseMoveEvent(event)

    # ------------------------------------------------------------- inspector
    def _inspect_point(self, scene_pt: QPointF) -> None:
        """Marker placement is scene-space; every number reported is world."""
        store = self._store
        self.click_marker.set_world_pos(scene_pt.x(), scene_pt.y())
        self.click_marker.setVisible(True)
        x, y = self._to_world(scene_pt.x(), scene_pt.y())
        parts = [f"world ({x:+.2f}, {y:+.2f}) m"]
        if store.geo.is_valid and store.geo.fit is not None:
            lat, lon = store.geo.fit.world_to_latlon(x, y)
            parts.append(f"GPS {lat:.6f}°, {lon:.6f}°")
        elif store.mission.simulation and not store.mission.gps_simulated:
            parts.append("GPS n/a (simulation)")
        else:
            parts.append("GPS n/a (waiting for fix)")
        if store.robot.has_odom:
            d = math.hypot(x - store.robot.x, y - store.robot.y)
            parts.append(f"robot ↔ point {d:.2f} m")
        self.point_inspected.emit("   |   ".join(parts))

    # ---------------------------------------------------------- measure tool
    def _handle_measure_click(self, scene_pt: QPointF) -> None:
        if self._measure_start is None:
            self._measure_start = scene_pt
            self._measure_line.setVisible(True)
            self._measure_text.setVisible(True)
            self._update_measure(scene_pt, final=False)
        else:
            self._update_measure(scene_pt, final=True)
            self._measure_start = None

    def _update_measure(self, scene_pt: QPointF, final: bool) -> None:
        """The line and its label live in scene coordinates; the reported
        endpoints are world. The distance stays a scene-space computation on
        purpose — world<->scene is a rigid transform, so it is identical
        either way, and computing it here cannot drift from what is drawn."""
        assert self._measure_start is not None
        a, b = self._measure_start, scene_pt
        self._measure_line.setLine(a.x(), a.y(), b.x(), b.y())
        d = math.hypot(b.x() - a.x(), b.y() - a.y())
        self._measure_text.setText(f"{d:.2f} m")
        self._measure_text.setPos((a.x() + b.x()) / 2, (a.y() + b.y()) / 2)
        ax, ay = self._to_world(a.x(), a.y())
        bx, by = self._to_world(b.x(), b.y())
        text = (f"measure: {d:.2f} m   |   A ({ax:+.2f}, {ay:+.2f})"
                f"   B ({bx:+.2f}, {by:+.2f})")
        self.point_inspected.emit(text)
