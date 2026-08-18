"""Segment interpolation models.

Every model turns one segment (waypoint A → waypoint B, with the previous
and next waypoints as optional context) into a dense polyline. Models are
registered in :data:`REGISTRY`; adding a new one is: subclass, define
``key`` / ``label`` / ``schema``, implement :meth:`sample`, append to the
registry. The editor auto-builds parameter forms from ``schema`` and the
exporter never needs to know model internals — interpolation is entirely an
editor-side concern (the runtime YAML contains only sampled points).

``schema`` rows: ``(param_key, label, kind, default, minimum, maximum)``
with ``kind`` in ``{"float", "int", "choice:<a|b|c>"}``.
"""

from __future__ import annotations

import math

import numpy as np

Point = tuple[float, float]


def _n_for(a: Point, b: Point, ds: float, minimum: int = 2) -> int:
    return max(minimum, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / ds)) + 1)


class Interpolation:
    key = "base"
    label = "Base"
    schema: list[tuple] = []

    def defaults(self) -> dict:
        return {row[0]: row[3] for row in self.schema}

    def sample(self, prev: Point | None, a: Point, b: Point,
               nxt: Point | None, params: dict, ds: float) -> np.ndarray:
        """Return an (n, 2) polyline from *a* (inclusive) to *b* (exclusive)."""
        raise NotImplementedError


class Straight(Interpolation):
    key = "straight"
    label = "Straight line"
    schema = []

    def sample(self, prev, a, b, nxt, params, ds):
        n = _n_for(a, b, ds)
        u = np.linspace(0.0, 1.0, n)[:-1]
        return np.column_stack([a[0] + u * (b[0] - a[0]),
                                a[1] + u * (b[1] - a[1])])


class Sinusoidal(Interpolation):
    key = "sine"
    label = "Sinusoidal"
    schema = [("amplitude", "Amplitude (m)", "float", 1.0, 0.0, 50.0),
              ("cycles", "Cycles", "float", 2.0, 0.25, 20.0)]

    def sample(self, prev, a, b, nxt, params, ds):
        amp = float(params.get("amplitude", 1.0))
        cyc = float(params.get("cycles", 2.0))
        n = _n_for(a, b, ds, minimum=16)
        u = np.linspace(0.0, 1.0, n)[:-1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < 1e-9:
            return np.array([[a[0], a[1]]])
        px, py = -dy / length, dx / length          # unit perpendicular
        off = amp * np.sin(2.0 * math.pi * cyc * u)
        return np.column_stack([a[0] + u * dx + off * px,
                                a[1] + u * dy + off * py])


class Arc(Interpolation):
    key = "arc"
    label = "Circular arc"
    schema = [("deflection_deg", "Deflection (°, signed)", "float", 90.0, -350.0, 350.0)]

    def sample(self, prev, a, b, nxt, params, ds):
        defl = math.radians(float(params.get("deflection_deg", 90.0)))
        chord = math.hypot(b[0] - a[0], b[1] - a[1])
        if chord < 1e-9 or abs(defl) < math.radians(1.0):
            return Straight().sample(prev, a, b, nxt, {}, ds)
        radius = chord / (2.0 * math.sin(abs(defl) / 2.0))
        mx, my = (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0
        # unit perpendicular to the chord; sign of deflection picks the side
        px, py = -(b[1] - a[1]) / chord, (b[0] - a[0]) / chord
        h = math.sqrt(max(radius * radius - (chord / 2.0) ** 2, 0.0))
        side = 1.0 if defl > 0 else -1.0
        cx, cy = mx - side * h * px, my - side * h * py
        a0 = math.atan2(a[1] - cy, a[0] - cx)
        a1 = math.atan2(b[1] - cy, b[0] - cx)
        sweep = a1 - a0
        while side > 0 and sweep <= 0:
            sweep += 2 * math.pi
        while side < 0 and sweep >= 0:
            sweep -= 2 * math.pi
        n = max(12, int(abs(sweep) * radius / ds) + 1)
        t = a0 + np.linspace(0.0, sweep, n)[:-1]
        return np.column_stack([cx + radius * np.cos(t),
                                cy + radius * np.sin(t)])


class CatmullRom(Interpolation):
    key = "spline"
    label = "Spline (Catmull-Rom)"
    schema = [("tension", "Tension", "float", 0.5, 0.0, 1.0)]

    def sample(self, prev, a, b, nxt, params, ds):
        tension = float(params.get("tension", 0.5))
        p0 = np.asarray(prev if prev is not None else a, dtype=float)
        p1, p2 = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        p3 = np.asarray(nxt if nxt is not None else b, dtype=float)
        m1 = tension * (p2 - p0)
        m2 = tension * (p3 - p1)
        n = _n_for(a, b, ds, minimum=16)
        u = np.linspace(0.0, 1.0, n)[:-1][:, None]
        h00 = 2 * u**3 - 3 * u**2 + 1
        h10 = u**3 - 2 * u**2 + u
        h01 = -2 * u**3 + 3 * u**2
        h11 = u**3 - u**2
        return h00 * p1 + h10 * m1 + h01 * p2 + h11 * m2


class Bezier(Interpolation):
    """Cubic Bézier; control points are defined relative to the chord
    (distance as a fraction of chord length, angle relative to the chord
    direction), so the shape survives moving either endpoint."""

    key = "bezier"
    label = "Bézier (cubic)"
    schema = [("c1_frac", "Ctrl 1 distance (× chord)", "float", 0.33, 0.0, 2.0),
              ("c1_angle_deg", "Ctrl 1 angle (°)", "float", 30.0, -180.0, 180.0),
              ("c2_frac", "Ctrl 2 distance (× chord)", "float", 0.33, 0.0, 2.0),
              ("c2_angle_deg", "Ctrl 2 angle (°)", "float", -30.0, -180.0, 180.0)]

    def sample(self, prev, a, b, nxt, params, ds):
        p0 = np.asarray(a, dtype=float)
        p3 = np.asarray(b, dtype=float)
        chord = p3 - p0
        length = float(np.hypot(*chord))
        if length < 1e-9:
            return np.array([[a[0], a[1]]])
        theta = math.atan2(chord[1], chord[0])

        def ctrl(origin: np.ndarray, frac_key: str, ang_key: str,
                 base_angle: float) -> np.ndarray:
            d = float(params.get(frac_key, 0.33)) * length
            ang = base_angle + math.radians(float(params.get(ang_key, 0.0)))
            return origin + d * np.array([math.cos(ang), math.sin(ang)])

        p1 = ctrl(p0, "c1_frac", "c1_angle_deg", theta)
        p2 = ctrl(p3, "c2_frac", "c2_angle_deg", theta + math.pi)
        n = _n_for(a, b, ds, minimum=16)
        u = np.linspace(0.0, 1.0, n)[:-1][:, None]
        return ((1 - u) ** 3 * p0 + 3 * (1 - u) ** 2 * u * p1
                + 3 * (1 - u) * u**2 * p2 + u**3 * p3)


REGISTRY: dict[str, Interpolation] = {
    cls.key: cls() for cls in (Straight, Sinusoidal, Arc, CatmullRom, Bezier)
}
