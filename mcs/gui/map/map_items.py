"""Graphics items composing the mission map.

All items live directly in ROS world coordinates (metres, x east-ish,
y north-ish per the local frame of ``robot_interface.py``).  The view
applies a y-flip so +y points up on screen.  Items that must keep constant
*pixel* size regardless of zoom (markers, the robot glyph) use
``ItemIgnoresTransformations`` around a world-anchored origin.

Z-order (back to front): tiles(-100) < grid(view background) < mission path
< trajectories < target line < prediction < markers < robot.
"""

from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import QLineF, QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import (
    QGraphicsEllipseItem, QGraphicsItem, QGraphicsItemGroup, QGraphicsLineItem,
    QGraphicsPathItem, QGraphicsPolygonItem, QGraphicsSimpleTextItem,
)

from mcs.gui import theme


def _cosmetic_pen(color: QColor, width: float, style=Qt.PenStyle.SolidLine) -> QPen:
    pen = QPen(color, width, style, Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin)
    pen.setCosmetic(True)  # constant pixel width at any zoom
    return pen


class PolylineItem(QGraphicsPathItem):
    """Efficient polyline rebuilt from an (n, 2) numpy array."""

    def __init__(self, color: QColor, width: float = 2.0,
                 style=Qt.PenStyle.SolidLine, z: float = 0.0) -> None:
        super().__init__()
        self.setPen(_cosmetic_pen(color, width, style))
        self.setZValue(z)

    def set_points(self, xy: np.ndarray) -> None:
        path = QPainterPath()
        if len(xy) >= 2:
            path.moveTo(xy[0, 0], xy[0, 1])
            for i in range(1, len(xy)):
                path.lineTo(xy[i, 0], xy[i, 1])
        self.setPath(path)


class RobotItem(QGraphicsItemGroup):
    """Boat glyph (constant pixel size) + world-scaled heading arrow.

    Yaw convention: identical to the stack (``master_control`` /
    ``bridge_node`` both take yaw from ``R.from_quat(...).as_euler('xyz')``:
    CCW-positive about +z in the y-up world frame). The *heading arrow* is a
    plain world-coordinate line, so the view's y-flip orients it correctly
    for free. The *glyph*, however, uses ``ItemIgnoresTransformations`` and
    is therefore rotated in **device** space, where +y points down and Qt
    rotations are clockwise-positive — a world yaw θ must be applied as
    ``setRotation(-degrees(θ))`` there, otherwise the glyph renders mirrored
    about the x-axis (the original "arrow not properly oriented" defect).
    """

    def __init__(self) -> None:
        super().__init__()
        self.setZValue(50)
        # Constant-pixel-size hull glyph, drawn pointing +x, then rotated.
        self._glyph = QGraphicsPolygonItem(QPolygonF([
            QPointF(14, 0), QPointF(-8, 7), QPointF(-4, 0), QPointF(-8, -7),
        ]))
        self._glyph.setBrush(QBrush(theme.C_ROBOT))
        self._glyph.setPen(QPen(QColor("white"), 1.2))
        self._glyph.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self.addToGroup(self._glyph)
        # Heading arrow: world-scaled SOLID white line ahead of the boat
        # (solid, so it can never be confused with the dashed target line).
        self._arrow = QGraphicsLineItem()
        self._arrow.setPen(_cosmetic_pen(QColor("white"), 1.8))
        self.addToGroup(self._arrow)
        self._arrow_visible = False

    def set_pose(self, x: float, y: float, heading: float) -> None:
        """Place the glyph at scene (x, y) pointing at scene ``heading``.

        The hosting view is always north-up and never rotates, so the scene
        is axis-aligned: the caller passes coordinates and heading already in
        scene axes (ENU true heading once georeferenced, raw world yaw
        before). The glyph ignores view transforms and is rotated in
        **device** space (y-down, CW-positive), hence ``-degrees(heading)``;
        the arrow is a plain scene line and picks up the view's y-flip for
        free. This is the whole orientation contract — there is no separate
        view-rotation term any more."""
        self._glyph.setPos(x, y)
        self._glyph.setRotation(-math.degrees(heading))
        length = 4.0  # metres of look-ahead
        self._arrow.setLine(QLineF(
            x, y, x + length * math.cos(heading), y + length * math.sin(heading)))
        self._arrow.setVisible(self._arrow_visible)

    def set_heading_visible(self, visible: bool) -> None:
        self._arrow_visible = visible
        self._arrow.setVisible(visible)


class MarkerItem(QGraphicsItemGroup):
    """Constant-pixel-size circular marker with an optional text label."""

    def __init__(self, color: QColor, radius_px: float = 6.0,
                 label: str = "", z: float = 40.0) -> None:
        super().__init__()
        self.setZValue(z)
        self._dot = QGraphicsEllipseItem(-radius_px, -radius_px,
                                         2 * radius_px, 2 * radius_px)
        self._dot.setBrush(QBrush(color))
        self._dot.setPen(QPen(QColor("white"), 1.2))
        self._dot.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self.addToGroup(self._dot)
        self._text = QGraphicsSimpleTextItem(label, self._dot)
        self._text.setBrush(QBrush(QColor(theme.TEXT)))
        self._text.setPos(radius_px + 3, -radius_px)

    def set_world_pos(self, x: float, y: float) -> None:
        self._dot.setPos(x, y)

    def set_label(self, text: str) -> None:
        self._text.setText(text)


class CrosshairItem(QGraphicsItemGroup):
    """Constant-pixel-size crosshair marking the manual target."""

    def __init__(self, color: QColor, z: float = 45.0) -> None:
        super().__init__()
        self.setZValue(z)
        self._group = QGraphicsItemGroup()
        pen = QPen(color, 2)
        s = 9.0
        for line in (QLineF(-s, 0, s, 0), QLineF(0, -s, 0, s)):
            item = QGraphicsLineItem(line, self._group)
            item.setPen(pen)
        ring = QGraphicsEllipseItem(-s * 0.7, -s * 0.7, s * 1.4, s * 1.4, self._group)
        ring.setPen(pen)
        self._group.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self.addToGroup(self._group)

    def set_world_pos(self, x: float, y: float) -> None:
        self._group.setPos(x, y)


class TargetLineItem(QGraphicsLineItem):
    """Thin straight line between the robot and the current target."""

    def __init__(self) -> None:
        super().__init__()
        self.setPen(_cosmetic_pen(theme.C_TARGET_LINE, 1.0, Qt.PenStyle.DashLine))
        self.setZValue(30)

    def set_endpoints(self, x0: float, y0: float, x1: float, y1: float) -> None:
        self.setLine(QLineF(x0, y0, x1, y1))


class MissionPathItem(PolylineItem):
    """The published/requested mission path with small waypoint ticks."""

    def __init__(self) -> None:
        super().__init__(theme.C_MISSION_PATH, 2.0, Qt.PenStyle.SolidLine, z=10)
        self.setOpacity(0.9)


def draw_north_indicator(painter, viewport_w: int, north_up: bool) -> None:
    """Small compass hint: 'N ^' when the view is north-up, else a world-up
    notice so the operator knows geographic orientation is not yet known."""
    painter.save()
    painter.resetTransform()
    painter.setFont(QFont("DejaVu Sans", 9, QFont.Weight.Bold))
    if north_up:
        painter.setPen(QColor(230, 235, 240))
        painter.drawText(QPointF(viewport_w - 34.0, 20.0), "N")
        painter.setPen(_cosmetic_pen(QColor(230, 235, 240), 2.0))
        painter.drawLine(QLineF(viewport_w - 22.0, 22.0,
                                viewport_w - 22.0, 8.0))
        painter.drawLine(QLineF(viewport_w - 26.0, 12.0,
                                viewport_w - 22.0, 8.0))
        painter.drawLine(QLineF(viewport_w - 18.0, 12.0,
                                viewport_w - 22.0, 8.0))
    else:
        painter.setPen(QColor(150, 158, 168))
        painter.drawText(QPointF(viewport_w - 150.0, 20.0),
                         "world-up (north unknown)")
    painter.restore()


def draw_scale_bar(painter, viewport_w: int, viewport_h: int,
                   px_per_m: float, spacing_m: float) -> None:
    """Draw the grid-scale indicator (bar + label) in device coordinates.

    Called from a view's ``drawForeground`` with the grid spacing chosen by
    :func:`draw_grid`, so the bar length always equals exactly one grid cell.
    """
    if spacing_m <= 0 or px_per_m <= 0:
        return
    bar_px = spacing_m * px_per_m
    x0, y0 = 16.0, viewport_h - 16.0
    painter.save()
    painter.resetTransform()  # scene -> device coordinates
    pen = QPen(QColor(230, 235, 240), 2)
    painter.setPen(pen)
    painter.drawLine(QLineF(x0, y0, x0 + bar_px, y0))
    for x in (x0, x0 + bar_px):
        painter.drawLine(QLineF(x, y0 - 4, x, y0 + 4))
    label = f"{spacing_m:g} m"
    painter.setFont(QFont("DejaVu Sans Mono", 9))
    painter.setPen(QColor(230, 235, 240))
    painter.drawText(QPointF(x0 + bar_px + 8, y0 + 4), label)
    painter.restore()


def draw_grid(painter, rect: QRectF, px_per_m: float,
              high_contrast: bool = False) -> float:
    """Draw an adaptive metric grid in view background coordinates.

    Returns the chosen grid spacing in metres (for the scale indicator).
    Spacing follows a 1/2/5 decade progression targeting >= ~60 px cells.
    ``high_contrast`` is used over satellite imagery: lines are drawn with a
    dark halo underneath a brighter stroke so the grid stays legible on any
    background.
    """
    target_px = 60.0
    raw = target_px / max(px_per_m, 1e-9)
    exp = math.floor(math.log10(max(raw, 1e-9)))
    base = raw / (10 ** exp)
    for mult in (1.0, 2.0, 5.0, 10.0):
        if base <= mult:
            spacing = mult * 10 ** exp
            break
    else:  # pragma: no cover
        spacing = 10 ** (exp + 1)

    if high_contrast:
        passes = [
            (_cosmetic_pen(QColor(0, 0, 0, 140), 2.6),
             _cosmetic_pen(QColor(0, 0, 0, 170), 3.0)),
            (_cosmetic_pen(QColor(255, 255, 255, 150), 1.1),
             _cosmetic_pen(QColor(255, 255, 255, 220), 1.4)),
        ]
    else:
        passes = [(_cosmetic_pen(theme.C_GRID, 1.0),
                   _cosmetic_pen(theme.C_GRID_MAJOR, 1.0))]

    for pen_minor, pen_major in passes:
        x0 = math.floor(rect.left() / spacing) * spacing
        y0 = math.floor(rect.top() / spacing) * spacing
        i = 0
        x = x0
        while x <= rect.right():
            painter.setPen(pen_major if abs((x / spacing) % 5) < 1e-6 else pen_minor)
            painter.drawLine(QLineF(x, rect.top(), x, rect.bottom()))
            x += spacing
            i += 1
            if i > 400:
                break
        i = 0
        y = y0
        while y <= rect.bottom():
            painter.setPen(pen_major if abs((y / spacing) % 5) < 1e-6 else pen_minor)
            painter.drawLine(QLineF(rect.left(), y, rect.right(), y))
            y += spacing
            i += 1
            if i > 400:
                break
    return spacing
