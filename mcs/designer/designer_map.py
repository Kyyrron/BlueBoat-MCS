"""Mission Pattern Designer — interactive editing map.

Reuses the station's map building blocks (adaptive grid, satellite
:class:`~mcs.gui.map.tile_layer.TileLayer`, robot/pinger glyphs, theme) and
adds CAD-style waypoint editing:

* click-to-add mode with **fixed-distance creation** (Ctrl snaps the
  distance from the previous waypoint to the grid step) and axis
  constraint (Shift);
* drag editing with temporary constraints — **Shift** = horizontal /
  vertical relative to the drag origin, **Ctrl** = snap to grid,
  default snap-to-waypoint within a pixel radius (**Alt** disables);
* rubber-band multi-selection, middle-button panning, wheel zoom;
* live preview: interpolation curves, travel-direction chevrons, waypoint
  numbering, START / END markers.

The view owns no mission logic: it reads/writes the
:class:`~mcs.designer.model.MissionModel` and emits editing intents.
"""

from __future__ import annotations

import math
from enum import Enum, auto

import numpy as np
from PySide6.QtCore import QLineF, QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QGuiApplication, QPainter, QPen
from PySide6.QtWidgets import (
    QGraphicsEllipseItem, QGraphicsItem, QGraphicsItemGroup, QGraphicsScene,
    QGraphicsSimpleTextItem, QGraphicsView,
)

from mcs.config.settings import AppConfig
from mcs.designer.model import MissionModel, Waypoint
from mcs.designer.sampling import SampledMission
from mcs.gui import theme
from mcs.gui.map.map_items import (
    MarkerItem, PolylineItem, RobotItem, draw_grid, draw_scale_bar)
from mcs.gui.map.tile_layer import TileLayer

C_WAYPOINT = QColor("#e3b341")
C_WAYPOINT_SEL = QColor("#2f81f7")
C_WAYPOINT_LOCK = QColor("#8a949e")
C_PREVIEW = QColor("#3fb950")


class EditMode(Enum):
    SELECT = auto()
    ADD = auto()


class WaypointItem(QGraphicsEllipseItem):
    """Constant-pixel-size, draggable waypoint handle."""

    R = 7.0

    def __init__(self, uid: int, host: "DesignerMapView") -> None:
        super().__init__(-self.R, -self.R, 2 * self.R, 2 * self.R)
        self.uid = uid
        self._host = host
        self.setFlags(QGraphicsItem.GraphicsItemFlag.ItemIsMovable
                      | QGraphicsItem.GraphicsItemFlag.ItemIsSelectable
                      | QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges
                      | QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self.setZValue(60)
        self.label = QGraphicsSimpleTextItem("", self)
        self.label.setBrush(QBrush(QColor(theme.TEXT)))
        self.label.setFont(QFont("DejaVu Sans", 8))
        self.label.setPos(self.R + 3, -self.R - 2)
        self.set_locked(False)

    def set_locked(self, locked: bool) -> None:
        self._locked = locked
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, not locked)
        self._restyle()

    def _restyle(self) -> None:
        color = (C_WAYPOINT_LOCK if self._locked
                 else C_WAYPOINT_SEL if self.isSelected() else C_WAYPOINT)
        self.setBrush(QBrush(color))
        self.setPen(QPen(QColor("white"), 1.4))

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionChange \
                and not self._host.syncing:
            return self._host.constrain_move(self, value)
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged \
                and not self._host.syncing:
            self._host.on_item_moved(self)
        if change == QGraphicsItem.GraphicsItemChange.ItemSelectedHasChanged:
            self._restyle()
        return super().itemChange(change, value)


class DesignerMapView(QGraphicsView):
    """The designer's central map."""

    selection_changed = Signal()
    edit_started = Signal()        # push an undo snapshot before a drag
    point_added = Signal(float, float)

    def __init__(self, cfg: AppConfig, model: MissionModel, parent=None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self._model = model
        self.syncing = False
        self._mode = EditMode.SELECT
        self._drag_origin: dict[int, QPointF] = {}
        self._panning: QPointF | None = None

        self._scene = QGraphicsScene(self)
        self._scene.setSceneRect(-5e4, -5e4, 1e5, 1e5)
        self.setScene(self._scene)
        self.setRenderHints(QPainter.RenderHint.Antialiasing
                            | QPainter.RenderHint.SmoothPixmapTransform)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.DragMode.RubberBandDrag)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setBackgroundBrush(QColor(theme.BG_DARK))
        self.scale(15.0, -15.0)
        self.grid_visible = True

        # Layers
        self.tiles = TileLayer(self._scene, cfg.map)
        self.geo_fit = None                     # GeoFit | None (set by window)
        self.preview = PolylineItem(C_PREVIEW, 2.0, z=20)
        self._scene.addItem(self.preview)
        self._decor = QGraphicsItemGroup()      # chevrons + START/END
        self._decor.setZValue(25)
        self._scene.addItem(self._decor)
        self.robot_item = RobotItem()
        self.robot_item.setVisible(False)
        self._scene.addItem(self.robot_item)
        self.pinger_marker = MarkerItem(theme.C_PINGER, 6.0, "pinger", z=42)
        self.pinger_marker.setVisible(False)
        self._scene.addItem(self.pinger_marker)

        self._wp_items: dict[int, WaypointItem] = {}
        self._scene.selectionChanged.connect(self._on_scene_selection)

    def _on_scene_selection(self) -> None:
        if not self.syncing:
            self.selection_changed.emit()

    # ================================================================= modes
    def set_mode(self, mode: EditMode) -> None:
        self._mode = mode
        self.setDragMode(QGraphicsView.DragMode.RubberBandDrag
                         if mode is EditMode.SELECT
                         else QGraphicsView.DragMode.NoDrag)
        self.viewport().setCursor(
            Qt.CursorShape.CrossCursor if mode is EditMode.ADD
            else Qt.CursorShape.ArrowCursor)

    @property
    def mode(self) -> EditMode:
        return self._mode

    # ============================================================ scene sync
    def rebuild_items(self) -> None:
        """Structure changed: recreate waypoint handles."""
        self.syncing = True
        for item in self._wp_items.values():
            self._scene.removeItem(item)
        self._wp_items.clear()
        for wp in self._model.flatten():
            item = WaypointItem(wp.uid, self)
            item.setPos(wp.x, wp.y)
            item.set_locked(self._model.effective_locked(wp))
            self._scene.addItem(item)
            self._wp_items[wp.uid] = item
        self.syncing = False
        self.refresh_labels()

    def sync_positions(self) -> None:
        """Model geometry changed programmatically: move handles."""
        self.syncing = True
        for wp in self._model.flatten():
            item = self._wp_items.get(wp.uid)
            if item is not None:
                item.setPos(wp.x, wp.y)
                item.set_locked(self._model.effective_locked(wp))
        self.syncing = False
        self.refresh_labels()

    def refresh_labels(self) -> None:
        for i, wp in enumerate(self._model.flatten(), start=1):
            item = self._wp_items.get(wp.uid)
            if item is not None:
                item.label.setText(f"{i} · {wp.name}" if wp.name else str(i))

    def selected_uids(self) -> set[int]:
        return {it.uid for it in self._scene.selectedItems()
                if isinstance(it, WaypointItem)}

    def select_uids(self, uids: set[int]) -> None:
        self.syncing = True
        for uid, item in self._wp_items.items():
            item.setSelected(uid in uids)
        self.syncing = False

    # ============================================================== preview
    def update_preview(self, samples: SampledMission) -> None:
        self.preview.set_points(samples.xy if not samples.empty
                                else np.empty((0, 2)))
        for child in list(self._decor.childItems()):
            self._decor.removeFromGroup(child)
            self._scene.removeItem(child)
        if samples.empty or len(samples.xy) < 2:
            return
        # Travel-direction chevrons (world-sized, cheap)
        step = self._cfg.designer.preview_arrow_every_m
        next_s = step
        s = 0.0
        pen = QPen(C_PREVIEW, 0)
        pen.setCosmetic(True)
        pen.setWidthF(1.6)
        for i in range(1, len(samples.xy)):
            seg = samples.xy[i] - samples.xy[i - 1]
            ds = float(np.hypot(*seg))
            s += ds
            if s >= next_s and ds > 1e-9:
                next_s += step
                x, y = samples.xy[i]
                ang = math.atan2(seg[1], seg[0])
                size = 0.8
                for side in (2.5, -2.5):
                    a = ang + math.pi - side * 0.35
                    line = self._scene.addLine(
                        QLineF(x, y, x + size * math.cos(a),
                               y + size * math.sin(a)), pen)
                    self._decor.addToGroup(line)
        start = MarkerItem(QColor(theme.OK), 6.0, "START", z=55)
        start.set_world_pos(*samples.xy[0])
        end = MarkerItem(QColor(theme.ERR), 6.0, "END", z=55)
        end.set_world_pos(*samples.xy[-1])
        for m in (start, end):
            self._scene.addItem(m)
            self._decor.addToGroup(m)

    # ============================================================ constraints
    def constrain_move(self, item: WaypointItem, pos: QPointF) -> QPointF:
        mods = QGuiApplication.keyboardModifiers()
        origin = self._drag_origin.get(item.uid)
        x, y = pos.x(), pos.y()
        if origin is not None and mods & Qt.KeyboardModifier.ShiftModifier:
            if abs(x - origin.x()) >= abs(y - origin.y()):
                y = origin.y()
            else:
                x = origin.x()
        if mods & Qt.KeyboardModifier.ControlModifier:
            g = self._cfg.designer.grid_snap_m
            x, y = round(x / g) * g, round(y / g) * g
        elif not mods & Qt.KeyboardModifier.AltModifier:
            snapped = self._snap_to_waypoint(x, y, exclude=item.uid)
            if snapped is not None:
                x, y = snapped
        return QPointF(x, y)

    def _snap_to_waypoint(self, x: float, y: float,
                          exclude: int) -> tuple[float, float] | None:
        radius_m = self._cfg.designer.waypoint_snap_px / max(
            abs(self.transform().m11()), 1e-9)
        best, best_d = None, radius_m
        for wp in self._model.flatten():
            if wp.uid == exclude:
                continue
            d = math.hypot(wp.x - x, wp.y - y)
            if d < best_d:
                best, best_d = (wp.x, wp.y), d
        return best

    def on_item_moved(self, item: WaypointItem) -> None:
        wp = self._model.waypoint(item.uid)
        if wp is not None:
            wp.x, wp.y = item.pos().x(), item.pos().y()
            self._model.changed.emit()

    # ================================================================ painting
    def drawBackground(self, painter: QPainter, rect: QRectF) -> None:
        super().drawBackground(painter, rect)
        if self.grid_visible:
            self._grid_spacing = draw_grid(
                painter, rect, abs(self.transform().m11()),
                high_contrast=self.tiles.enabled)

    def drawForeground(self, painter: QPainter, rect: QRectF) -> None:
        super().drawForeground(painter, rect)
        if self.grid_visible:
            draw_scale_bar(painter, self.viewport().width(),
                           self.viewport().height(),
                           abs(self.transform().m11()),
                           getattr(self, "_grid_spacing", 0.0))

    # ---------------------------------------------------------------- framing
    def center_on_bounds(self, xs: list[float], ys: list[float],
                         margin_m: float = 5.0) -> None:
        """Frame the given world extents, preserving the y-up flip.

        (fitInView is avoided on purpose: it may normalise the scale signs
        and silently undo the y-flip of this view.)
        """
        if not xs:
            return
        w = max(max(xs) - min(xs), 1.0) + 2 * margin_m
        h = max(max(ys) - min(ys), 1.0) + 2 * margin_m
        cx = (max(xs) + min(xs)) / 2.0
        cy = (max(ys) + min(ys)) / 2.0
        s = min(self.viewport().width() / w, self.viewport().height() / h)
        s = max(0.05, min(s, 200.0))
        transform = self.transform()
        transform.setMatrix(s, 0, 0, 0, -s, 0,
                            transform.m31(), transform.m32(), 1.0)
        self.setTransform(transform)
        self.centerOn(cx, cy)
        self._update_tiles()

    def zoom_in(self) -> None:
        self._zoom_button(1.25)

    def zoom_out(self) -> None:
        self._zoom_button(1 / 1.25)

    def _zoom_button(self, factor: float) -> None:
        """Toolbar zoom: anchored on the view centre (wheel zoom stays
        anchored under the cursor)."""
        px = abs(self.transform().m11()) * factor
        if not (0.05 <= px <= 2000.0):
            return
        centre = self.mapToScene(self.viewport().rect().center())
        self.scale(factor, factor)
        self.centerOn(centre)
        self._update_tiles()

    # ================================================================== input
    def wheelEvent(self, event) -> None:
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        px = abs(self.transform().m11()) * factor
        if 0.05 <= px <= 2000.0:
            self.scale(factor, factor)
        self._update_tiles()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            world = self.mapToScene(event.position().toPoint())
            if self._mode is EditMode.ADD:
                self._add_point(world)
                return
            # SELECT: snapshot origins for constraint + undo before a drag
            if self.itemAt(event.position().toPoint()) is not None:
                self.edit_started.emit()
                self._drag_origin = {
                    it.uid: it.pos() for it in self._scene.selectedItems()
                    if isinstance(it, WaypointItem)}
                under = self.itemAt(event.position().toPoint())
                if isinstance(under, WaypointItem):
                    self._drag_origin.setdefault(under.uid, under.pos())
        if event.button() == Qt.MouseButton.RightButton \
                and self._mode is EditMode.ADD:
            self.set_mode(EditMode.SELECT)
            return
        if event.button() == Qt.MouseButton.MiddleButton or event.button() == Qt.MouseButton.RightButton:
            self._panning = event.position()
            self.viewport().setCursor(Qt.CursorShape.ClosedHandCursor)
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._panning is not None:
            # 1:1 panning via scrollbar deltas. The previous
            # transform-translate approach compounded with the
            # AnchorUnderMouse policy, making panning move far faster than
            # the cursor.
            delta = event.position() - self._panning
            self._panning = event.position()
            h = self.horizontalScrollBar()
            v = self.verticalScrollBar()
            h.setValue(h.value() - int(round(delta.x())))
            v.setValue(v.value() - int(round(delta.y())))
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.MiddleButton or event.button() == Qt.MouseButton.RightButton and self._mode is not EditMode.ADD:
            self._panning = None
            self.viewport().setCursor(Qt.CursorShape.ArrowCursor)
            self._update_tiles()
            return
        self._drag_origin = {}
        super().mouseReleaseEvent(event)

    def _add_point(self, world: QPointF) -> None:
        mods = QGuiApplication.keyboardModifiers()
        x, y = world.x(), world.y()
        wps = self._model.flatten()
        if wps:
            last = wps[-1]
            dx, dy = x - last.x, y - last.y
            if mods & Qt.KeyboardModifier.ShiftModifier:
                if abs(dx) >= abs(dy):
                    y = last.y
                    dy = 0.0
                else:
                    x = last.x
                    dx = 0.0
            if mods & Qt.KeyboardModifier.ControlModifier:
                # Fixed-distance creation: snap range from the previous
                # waypoint to a multiple of the grid step.
                g = self._cfg.designer.grid_snap_m
                d = math.hypot(dx, dy)
                if d > 1e-9:
                    d_snap = max(g, round(d / g) * g)
                    x = last.x + dx / d * d_snap
                    y = last.y + dy / d * d_snap
        self.point_added.emit(x, y)

    # ================================================================== tiles
    def set_geo_fit(self, fit) -> None:
        self.geo_fit = fit
        self._update_tiles()

    def _update_tiles(self) -> None:
        self.tiles.update_view(
            self.geo_fit,
            self.mapToScene(self.viewport().rect()).boundingRect(),
            abs(self.transform().m11()))

    def refresh_overlays(self, robot=None, pinger=None,
                         show_robot=True, show_pinger=True) -> None:
        if robot is not None and show_robot:
            self.robot_item.set_pose(*robot)
            self.robot_item.setVisible(True)
        else:
            self.robot_item.setVisible(False)
        if pinger is not None and show_pinger:
            self.pinger_marker.set_world_pos(*pinger)
            self.pinger_marker.setVisible(True)
        else:
            self.pinger_marker.setVisible(False)
        self._update_tiles()
