"""Survey pattern generators.

Each generator turns a parameter dict into an ordered list of ``(x, y)``
waypoints anchored at ``(x0, y0)``. Generators are registered in
:data:`REGISTRY`; the editor auto-builds their parameter dialogs from
``schema`` (same row format as the interpolation schemas, plus ``x0`` /
``y0`` anchor rows injected automatically by the dialog).

Coverage note: the lawnmower is a rectangle-area boustrophedon coverage
generator; polygon-area coverage planners, Dubins-constrained turns and
fully automatic survey generation are documented as future work in the
handover document.
"""

from __future__ import annotations

import math

Point = tuple[float, float]


def _rot(points: list[Point], x0: float, y0: float, deg: float) -> list[Point]:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [(x0 + c * x - s * y, y0 + s * x + c * y) for x, y in points]


class Pattern:
    key = "base"
    label = "Base"
    schema: list[tuple] = []

    def generate(self, p: dict) -> list[Point]:
        raise NotImplementedError


class Lawnmower(Pattern):
    key = "lawnmower"
    label = "Lawnmower"
    schema = [("width", "Area width (m)", "float", 20.0, 1.0, 2000.0),
              ("height", "Area height (m)", "float", 12.0, 1.0, 2000.0),
              ("spacing", "Track spacing (m)", "float", 3.0, 0.2, 500.0),
              ("orientation_deg", "Orientation (°)", "float", 0.0, -180.0, 180.0),
              ("start_corner", "Starting corner", "choice:SW|SE|NW|NE", "SW", None, None)]

    def generate(self, p: dict) -> list[Point]:
        w, h = float(p["width"]), float(p["height"])
        spacing = max(float(p["spacing"]), 0.01)
        corner = str(p.get("start_corner", "SW"))
        rows = int(math.floor(h / spacing)) + 1
        pts: list[Point] = []
        for i in range(rows):
            y = min(i * spacing, h)
            left_to_right = (i % 2 == 0)
            xs = (0.0, w) if left_to_right else (w, 0.0)
            pts.append((xs[0], y))
            pts.append((xs[1], y))
        # Mirror for the requested starting corner (base is SW).
        if corner in ("SE", "NE"):
            pts = [(w - x, y) for x, y in pts]
        if corner in ("NW", "NE"):
            pts = [(x, h - y) for x, y in pts]
        return _rot(pts, float(p["x0"]), float(p["y0"]),
                    float(p.get("orientation_deg", 0.0)))


class Circle(Pattern):
    key = "circle"
    label = "Circle"
    schema = [("radius", "Radius (m)", "float", 6.0, 0.2, 1000.0),
              ("n_points", "Number of points", "int", 12, 3, 360),
              ("direction", "Direction", "choice:CCW|CW", "CCW", None, None),
              ("start_angle_deg", "Start angle (°)", "float", 0.0, -180.0, 180.0)]

    def generate(self, p: dict) -> list[Point]:
        r, n = float(p["radius"]), int(p["n_points"])
        sign = 1.0 if str(p.get("direction", "CCW")) == "CCW" else -1.0
        a0 = math.radians(float(p.get("start_angle_deg", 0.0)))
        pts = [(r * math.cos(a0 + sign * 2 * math.pi * i / n),
                r * math.sin(a0 + sign * 2 * math.pi * i / n))
               for i in range(n + 1)]  # closed
        return [(float(p["x0"]) + x, float(p["y0"]) + y) for x, y in pts]


class Rectangle(Pattern):
    key = "rectangle"
    label = "Rectangle"
    schema = [("width", "Width (m)", "float", 12.0, 0.2, 2000.0),
              ("height", "Height (m)", "float", 8.0, 0.2, 2000.0),
              ("orientation_deg", "Orientation (°)", "float", 0.0, -180.0, 180.0)]

    def generate(self, p: dict) -> list[Point]:
        w, h = float(p["width"]), float(p["height"])
        pts = [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h), (0.0, 0.0)]
        return _rot(pts, float(p["x0"]), float(p["y0"]),
                    float(p.get("orientation_deg", 0.0)))


class Square(Rectangle):
    key = "square"
    label = "Square"
    schema = [("side", "Side (m)", "float", 10.0, 0.2, 2000.0),
              ("orientation_deg", "Orientation (°)", "float", 0.0, -180.0, 180.0)]

    def generate(self, p: dict) -> list[Point]:
        q = dict(p)
        q["width"] = q["height"] = float(p["side"])
        return super().generate(q)


class FigureEight(Pattern):
    key = "figure_eight"
    label = "Figure Eight"
    schema = [("radius", "Lobe radius (m)", "float", 5.0, 0.2, 1000.0),
              ("n_points", "Number of points", "int", 24, 8, 720)]

    def generate(self, p: dict) -> list[Point]:
        r, n = float(p["radius"]), int(p["n_points"])
        pts = []
        for i in range(n + 1):
            t = 2 * math.pi * i / n
            pts.append((r * math.sin(t), r * math.sin(t) * math.cos(t)))
        return [(float(p["x0"]) + x, float(p["y0"]) + y) for x, y in pts]


class Spiral(Pattern):
    key = "spiral"
    label = "Spiral (Archimedean)"
    schema = [("turns", "Turns", "float", 3.0, 0.5, 30.0),
              ("spacing", "Spacing per turn (m)", "float", 2.0, 0.1, 200.0),
              ("points_per_turn", "Points per turn", "int", 16, 6, 128)]

    def generate(self, p: dict) -> list[Point]:
        turns = float(p["turns"])
        b = float(p["spacing"]) / (2 * math.pi)
        n = int(turns * int(p["points_per_turn"]))
        pts = []
        for i in range(n + 1):
            t = 2 * math.pi * turns * i / max(n, 1)
            pts.append((b * t * math.cos(t), b * t * math.sin(t)))
        return [(float(p["x0"]) + x, float(p["y0"]) + y) for x, y in pts]


class ExpandingSquare(Pattern):
    key = "expanding_square"
    label = "Expanding Square"
    schema = [("initial_leg", "Initial leg (m)", "float", 2.0, 0.2, 500.0),
              ("increment", "Increment per leg (m)", "float", 2.0, 0.0, 500.0),
              ("legs", "Number of legs", "int", 8, 2, 64)]

    def generate(self, p: dict) -> list[Point]:
        x = y = 0.0
        pts: list[Point] = [(x, y)]
        heading = 0.0
        leg = float(p["initial_leg"])
        for i in range(int(p["legs"])):
            x += leg * math.cos(heading)
            y += leg * math.sin(heading)
            pts.append((x, y))
            heading += math.pi / 2.0
            if i % 2 == 1:
                leg += float(p["increment"])
        return [(float(p["x0"]) + px, float(p["y0"]) + py) for px, py in pts]


class StationKeeping(Pattern):
    key = "station_keeping"
    label = "Station Keeping"
    schema = []

    def generate(self, p: dict) -> list[Point]:
        return [(float(p["x0"]), float(p["y0"]))]


class Polygon(Pattern):
    key = "polygon"
    label = "Custom Polygon (regular)"
    schema = [("n_sides", "Sides", "int", 5, 3, 24),
              ("radius", "Circumradius (m)", "float", 6.0, 0.2, 1000.0),
              ("orientation_deg", "Orientation (°)", "float", 0.0, -180.0, 180.0)]

    def generate(self, p: dict) -> list[Point]:
        n, r = int(p["n_sides"]), float(p["radius"])
        pts = [(r * math.cos(2 * math.pi * i / n),
                r * math.sin(2 * math.pi * i / n)) for i in range(n)]
        pts.append(pts[0])
        return _rot(pts, float(p["x0"]), float(p["y0"]),
                    float(p.get("orientation_deg", 0.0)))


REGISTRY: dict[str, Pattern] = {
    cls.key: cls() for cls in (
        Lawnmower, Circle, Rectangle, Square, FigureEight, Spiral,
        ExpandingSquare, StationKeeping, Polygon)
}
