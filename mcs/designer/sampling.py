"""Mission → time-parameterized samples.

The navigation pipeline evaluates trajectories at arbitrary times
(``path_generation.single_pose(t)``), so the exported artefact is a dense
list of ``[t, x, y, yaw]`` rows: each segment is sampled by its
interpolation model at spatial resolution ``ds``, arc length is accumulated,
and time is assigned as ``t = s / speed``. Yaw is the travel direction
(finite differences). The runtime loader only ever linearly interpolates
between rows — every interpolation model stays editor-side.
"""

from __future__ import annotations

import math

from dataclasses import dataclass

import numpy as np

from mcs.designer.interpolation import REGISTRY
from mcs.designer.model import MissionModel


@dataclass
class SampledMission:
    t: np.ndarray        # (n,)
    xy: np.ndarray       # (n, 2)
    yaw: np.ndarray      # (n,)
    length_m: float
    duration_s: float

    @property
    def empty(self) -> bool:
        return len(self.t) == 0


def sample_mission(model: MissionModel, ds: float = 0.25) -> SampledMission:
    wps = model.flatten()
    if not wps:
        return SampledMission(np.empty(0), np.empty((0, 2)), np.empty(0), 0.0, 0.0)
    if len(wps) == 1:
        w = wps[0]
        return SampledMission(np.array([0.0]), np.array([[w.x, w.y]]),
                              np.array([0.0]), 0.0, 0.0)

    mission_speed = max(float(model.speed), 1e-3)
    pairs = list(zip(wps[:-1], wps[1:]))
    if model.loop:
        pairs.append((wps[-1], wps[0]))

    xy_pts: list[tuple[float, float]] = [(wps[0].x, wps[0].y)]
    t_pts: list[float] = [0.0]
    length = 0.0

    for i, (a, b) in enumerate(pairs):
        prev = (wps[i - 1].x, wps[i - 1].y) if i > 0 else None
        nxt_wp = pairs[i + 1][1] if i + 1 < len(pairs) else None
        nxt = (nxt_wp.x, nxt_wp.y) if nxt_wp is not None else None
        interp = REGISTRY.get(a.seg_out.kind, REGISTRY["straight"])
        seg = np.asarray(interp.sample(prev, (a.x, a.y), (b.x, b.y), nxt,
                                       a.seg_out.params, ds), dtype=float)
        # Per-segment speed: 0 (or negative) means "mission cruise speed".
        speed = a.seg_out.speed if a.seg_out.speed > 1e-6 else mission_speed
        # Walk the segment polyline (a inclusive .. b exclusive) plus its
        # closing point b, accumulating time at THIS segment's speed.
        pts = list(map(tuple, seg)) + [(b.x, b.y)]
        for p in pts:
            dx = p[0] - xy_pts[-1][0]
            dy = p[1] - xy_pts[-1][1]
            step = math.hypot(dx, dy)
            if step <= 1e-9:
                continue
            length += step
            t_pts.append(t_pts[-1] + step / speed)
            xy_pts.append(p)

    xy = np.asarray(xy_pts, dtype=float)
    t = np.asarray(t_pts, dtype=float)
    if len(xy) < 2:
        return SampledMission(np.array([0.0]), xy[:1], np.array([0.0]), 0.0, 0.0)
    d = np.diff(xy, axis=0)
    yaw = np.arctan2(d[:, 1], d[:, 0])
    yaw = np.concatenate([yaw, yaw[-1:]])
    return SampledMission(t=t, xy=xy, yaw=yaw,
                          length_m=float(length), duration_s=float(t[-1]))
