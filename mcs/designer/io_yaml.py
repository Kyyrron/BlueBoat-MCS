"""Mission library I/O.

Two files per mission, in ``designer.trajectories_dir``:

* ``<name>.yaml`` — the **runtime** trajectory: only what execution needs
  (format tag, speed, loop, time-stamped ``[t, x, y, yaw]`` samples plus
  informative length/duration). ``loop`` is the model's ``repeat`` option
  ("Repeat forever"); a closing leg ("Return to start", ``close_path``) is
  already in the samples. Consumed by the control stack's
  ``BlueBoat-Control/blueboat_control/src/_custom_libraries/yaml_trajectory.py``;
  documented in ``docs/05_trajectory_format.md``.
* ``<name>.meta.yaml`` — **editor** metadata: the full designer model
  (groups, locks, segment interpolation settings, comments). The runtime
  never reads it; without it a runtime file can still be re-imported as
  plain waypoints (decimated), so nothing is ever lost beyond styling.

The read-only helpers the launch dialogs call once per saved mission
(:func:`read_geo_anchor`, :func:`read_first_point`,
:func:`read_start_misalignment`, :func:`read_path_ending`) parse through libyaml when it is installed and
memoize the parsed file by ``(mtime, size)``, so a dialog that badges every
saved path costs one ``stat`` per file once the files have been read.
"""

from __future__ import annotations

import datetime
import math
import os
import re
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import yaml

from mcs.designer.sampling import SampledMission, start_misalignment

if TYPE_CHECKING:  # annotation only -- see mcs/designer/sampling.py
    from mcs.designer.model import MissionModel

FORMAT = "blueboat_trajectory/1"
SAFETY_FORMAT = "blueboat_safety_way_back/1"
SAFETY_MAX_POINTS = 200          # link_guardian.lua's SAFE_MAX
_NAME_RE = re.compile(r"^[A-Za-z0-9_\-]+$")

# libyaml's loader when the build has it (~9x faster on the dense sample files);
# same constructors as SafeLoader, so the parsed data is identical.
_SAFE_LOADER = getattr(yaml, "CSafeLoader", yaml.SafeLoader)

# path -> ((mtime_ns, size), parsed data). Read-only for callers.
_DATA_CACHE: dict[str, tuple[tuple[int, int], dict]] = {}


def _read_data(yaml_path: Path) -> dict:
    """Parsed content of a runtime file, memoized until the file changes.

    Raises ``OSError`` / ``yaml.YAMLError`` like a direct parse and caches only
    successes. The returned dict is shared: callers must not mutate it."""
    stat = yaml_path.stat()
    stamp = (stat.st_mtime_ns, stat.st_size)
    key = str(yaml_path)
    hit = _DATA_CACHE.get(key)
    if hit is not None and hit[0] == stamp:
        return hit[1]
    loaded = yaml.load(yaml_path.read_text(), Loader=_SAFE_LOADER) or {}
    data = loaded if isinstance(loaded, dict) else {}
    _DATA_CACHE[key] = (stamp, data)
    return data


def valid_name(name: str) -> bool:
    return bool(_NAME_RE.match(name))


def runtime_path(directory: Path, name: str) -> Path:
    return directory / f"{name}.yaml"


def meta_path(directory: Path, name: str) -> Path:
    return directory / f"{name}.meta.yaml"


def list_missions(directory: Path) -> list[str]:
    if not directory.exists():
        return []
    return sorted(p.stem for p in directory.glob("*.yaml")
                  if not p.name.endswith(".meta.yaml"))


def save_mission(directory: Path, name: str, model: MissionModel,
                 samples: SampledMission,
                 geo_anchor: dict | None = None) -> Path:
    # geo_anchor (optional) georeferences the design frame:
    # {lat0, lon0, theta_deg} = GPS of the design-frame origin and its
    # rotation relative to local east/north. The design frame IS local ENU
    # now, so new anchors always carry theta_deg: 0.0 — the field remains
    # for legacy files, which deploy_mission still honours. With an anchor
    # present every waypoint is linked to real-world GPS, and the station
    # deploys the mission into the robot's CURRENT world frame at run time
    # (see deploy_mission and docs/05_trajectory_format.md).
    directory.mkdir(parents=True, exist_ok=True)
    points = [[round(float(t), 3), round(float(x), 4), round(float(y), 4),
               round(float(psi), 5)]
              for t, (x, y), psi in zip(samples.t, samples.xy, samples.yaw)]
    runtime = {
        "format": FORMAT,
        "name": name,
        "generator": "mission-pattern-designer/1.0",
        # Local wall-clock is deliberate: operators read this field.
        "created": datetime.datetime.now().isoformat(  # noqa: DTZ005
            timespec="seconds"),
        "frame": "world",
        "speed": float(model.speed),
        # Repeat forever (the control stack wraps t modulo duration). Only
        # model.repeat drives it -- never the closing leg (close_path).
        "loop": bool(model.repeat),
        "length_m": round(samples.length_m, 3),
        "duration_s": round(samples.duration_s, 3),
        "points": points,
    }
    if geo_anchor is not None:
        runtime["geo_anchor"] = {
            "lat0": float(geo_anchor["lat0"]),
            "lon0": float(geo_anchor["lon0"]),
            "theta_deg": float(geo_anchor.get("theta_deg", 0.0)),
        }
        safety = safety_way_back_latlon(model.safety_way_back, runtime["geo_anchor"])
        if safety is not None:
            # Lat/lon, so it needs no transform when the file is deployed: the
            # station hands it to the link guardian at START RUN (README of
            # link_guardian/). Control, the GCS and the simulator ignore the key.
            runtime["safety_way_back"] = {"format": SAFETY_FORMAT, "points": safety}
    path = runtime_path(directory, name)
    path.write_text(yaml.safe_dump(runtime, sort_keys=False,
                                   default_flow_style=None))
    meta = {"format": "blueboat_trajectory_meta/1", "model": model.to_dict()}
    meta_path(directory, name).write_text(yaml.safe_dump(meta, sort_keys=False))
    return path


def design_to_latlon(points, geo_anchor: dict) -> list[tuple[float, float]]:
    """Design-frame (x, y) points -> (lat, lon) through a mission's
    ``geo_anchor``: design -> EN about the anchor origin (identity for
    ``theta_deg == 0``; a legacy anchor recorded design = R(theta) @ EN),
    then EN -> lat/lon. The one copy of this convention."""
    from mcs.core.geo import local_en_to_latlon as _en_to_ll

    theta = math.radians(float(geo_anchor.get("theta_deg", 0.0)))
    c, s = math.cos(theta), math.sin(theta)
    lat0, lon0 = float(geo_anchor["lat0"]), float(geo_anchor["lon0"])
    return [_en_to_ll(c * float(x) + s * float(y), -s * float(x) + c * float(y),
                      lat0, lon0)
            for x, y in points]


def safety_way_back_latlon(points, geo_anchor: dict) -> list[list[float]] | None:
    """The design-frame safety way back as [[lat, lon], ...] through the mission's
    anchor, or None when it has fewer than 2 points."""
    if len(points) < 2:
        return None
    return [[round(lat, 8), round(lon, 8)]
            for lat, lon in design_to_latlon(points, geo_anchor)]


def read_safety_way_back(yaml_path: Path) -> list[tuple[float, float]] | None:
    """The safety way back of a GPS-anchored runtime file, [(lat, lon), ...], or
    None when the file has none (or an unusable one)."""
    try:
        data = _read_data(yaml_path)
    except (OSError, yaml.YAMLError):
        return None
    block = data.get("safety_way_back")
    if not isinstance(block, dict) or block.get("format") != SAFETY_FORMAT:
        return None
    try:
        pts = [(float(p[0]), float(p[1])) for p in block.get("points", [])]
    except (TypeError, ValueError, IndexError):
        return None
    if not 2 <= len(pts) <= SAFETY_MAX_POINTS:
        return None
    return pts


def read_geo_anchor(yaml_path: Path) -> dict | None:
    """Return the geo_anchor block of a runtime file, if any."""
    try:
        data = _read_data(yaml_path)
    except OSError:
        return None
    anchor = data.get("geo_anchor")
    if isinstance(anchor, dict) and "lat0" in anchor and "lon0" in anchor:
        return dict(anchor)
    return None


def read_first_point(yaml_path: Path) -> tuple[float, float] | None:
    """(x, y) of the first sample of a runtime file, or None.

    Used to place the simulated GPS receiver relative to the mission start.
    Never raises: an unreadable, non-YAML or malformed file answers None.
    """
    try:
        data = _read_data(yaml_path)
    except (OSError, yaml.YAMLError):
        return None
    points = data.get("points") or []
    if not points:
        return None
    first = points[0]
    try:
        if len(first) < 3:
            return None
        return float(first[1]), float(first[2])
    except (TypeError, ValueError):
        return None


def read_start_misalignment(yaml_path: Path, tol_m: float, tol_deg: float
                            ) -> tuple[tuple[float, float], float] | None:
    """Start misalignment of a runtime file, read from its own samples.

    The launch dialog only has the runtime ``<name>.yaml`` on disk, not the
    designer model. ``points`` IS the sampled polyline save_mission wrote,
    so the first two rows carry exactly the geometry the designer checks;
    the shared predicate lives in :func:`sampling.start_misalignment`.

    Never raises: an unreadable, non-YAML or malformed file answers "no
    misalignment to report" so it cannot break the launch dialog.
    """
    try:
        data = _read_data(yaml_path)
    except (OSError, yaml.YAMLError):
        return None
    try:
        pts = np.asarray(data.get("points", []), dtype=float)
    except (TypeError, ValueError):
        return None
    if pts.ndim != 2 or pts.shape[0] < 2 or pts.shape[1] < 3:
        return None
    return start_misalignment(pts[:, 1:3], tol_m, tol_deg)


def read_path_ending(yaml_path: Path, tol_m: float = 0.05
                     ) -> tuple[bool, bool] | None:
    """``(loop, returns_to_start)`` of a runtime file, read-only.

    ``loop`` is the file's runtime flag (true: the control stack repeats the
    path forever; false: the boat holds the final pose when it ends);
    ``returns_to_start`` says whether its last sample lies within *tol_m* of
    its first. Never raises: an unreadable or malformed file answers None,
    so it cannot break a launch dialog."""
    try:
        data = _read_data(yaml_path)
    except (OSError, yaml.YAMLError):
        return None
    try:
        pts = np.asarray(data.get("points", []), dtype=float)
    except (TypeError, ValueError):
        return None
    if pts.ndim != 2 or pts.shape[0] < 1 or pts.shape[1] < 3:
        return None
    returns = (pts.shape[0] >= 2 and math.hypot(float(pts[-1, 1] - pts[0, 1]),
                                                float(pts[-1, 2] - pts[0, 2])) <= tol_m)
    return bool(data.get("loop", False)), bool(returns)


def deploy_mission(src: Path, current_fit, dst: Path) -> Path:
    """Convert a GPS-anchored mission into the CURRENT run's world frame.

    The robot's world origin is created wherever robot_interface starts, so
    it differs every run; a GPS-anchored mission must not inherit that
    offset. The world frame is local ENU on both ends, so each sample is
    mapped design-frame -> GPS (via the file's own geo_anchor) -> today's
    world frame (via the station's live translation fit). New anchors are
    always written with ``theta_deg: 0`` (the design frame IS ENU); a legacy
    anchor with a non-zero ``theta_deg`` is honoured by rotating its points
    into ENU (and its yaws by ``-theta_a``) before translating. The result
    is written to *dst* -- the file path_generation was pointed at and is
    watching for (it holds position until the file appears).
    """
    import math as _math

    # A usable translation fit is all deployment needs: no rotation is
    # estimated (the odom frame is local ENU), so no vehicle motion is
    # required. The watcher retries until fixes have anchored the frame.
    if current_fit is None:
        raise ValueError("no georeference yet (waiting for GPS fixes)")

    data = yaml.safe_load(src.read_text()) or {}
    anchor = data.get("geo_anchor")
    if not anchor:
        raise ValueError(f"{src} has no geo_anchor")
    theta_a = _math.radians(float(anchor.get("theta_deg", 0.0)))
    points = data.get("points", [])
    latlon = design_to_latlon([(x, y) for _t, x, y, _yaw in points], anchor)
    out_points = []
    for (t_s, _x, _y, yaw), (lat, lon) in zip(points, latlon, strict=True):
        wx, wy = current_fit.latlon_to_world(lat, lon)
        yaw2 = _math.atan2(_math.sin(float(yaw) - theta_a),
                           _math.cos(float(yaw) - theta_a))
        out_points.append([round(float(t_s), 3), round(wx, 4),
                           round(wy, 4), round(yaw2, 5)])
    deployed = dict(data)
    deployed.pop("geo_anchor", None)
    deployed["deployed_from"] = str(src)
    deployed["deployed_fit_rms_m"] = round(float(current_fit.rms_m), 3)
    deployed["points"] = out_points
    _atomic_write_yaml(dst, deployed)
    return dst


def place_mission(src: Path, x: float, y: float, yaw: float, dst: Path) -> Path:
    """Place a NON-anchored mission at the boat, for the start of a run.

    A run session (real-robot launch) fixes the world frame at connection,
    long before a run starts, so a design-frame mission can no longer simply
    start "where the boat was at launch": the boat has been driven since. The
    whole path is therefore moved RIGIDLY so its first sample sits on the
    boat's current world position (x, y) and its first heading on the boat's
    current world heading ``yaw`` (odom yaw, radians ENU) -- the path starts
    at the boat and runs out ahead of it. Shape, spacing and timing are
    untouched: every sample is rotated by ``yaw - yaw_first`` about the first
    sample, then translated; every yaw gains the same angle.

    Written atomically to *dst* -- the file path_generation watches -- with
    ``deployed_from`` and ``placed_at`` provenance. A GPS-anchored mission is
    refused: it is geographically fixed and goes through deploy_mission.
    """
    data = yaml.safe_load(src.read_text()) or {}
    if data.get("geo_anchor"):
        raise ValueError(f"{src.name} is GPS-anchored: deploy it, do not place it")
    points = data.get("points") or []
    if not points:
        raise ValueError(f"{src.name} has no points")
    _t0, x0, y0, yaw0 = (float(v) for v in points[0][:4])
    dpsi = float(yaw) - yaw0
    c, s = math.cos(dpsi), math.sin(dpsi)
    out_points = []
    for t_s, px, py, pyaw in (row[:4] for row in points):
        dx, dy = float(px) - x0, float(py) - y0
        wyaw = float(pyaw) + dpsi
        out_points.append([round(float(t_s), 3),
                           round(float(x) + c * dx - s * dy, 4),
                           round(float(y) + s * dx + c * dy, 4),
                           round(math.atan2(math.sin(wyaw), math.cos(wyaw)), 5)])
    placed = dict(data)
    placed["deployed_from"] = str(src)
    placed["placed_at"] = {"x": round(float(x), 4), "y": round(float(y), 4),
                           "yaw": round(float(yaw), 5)}
    placed["points"] = out_points
    _atomic_write_yaml(dst, placed)
    return dst


def _atomic_write_yaml(dst: Path, data: dict) -> None:
    """Write a runtime trajectory so a reader never sees half a file.

    path_generation reloads its watched file on the next request after the
    mtime changes, and in a run session that request can come within one
    control tick of the write: write a sibling, then os.replace() it in.
    """
    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = dst.with_name(f".{dst.name}.tmp-{os.getpid()}")
    tmp.write_text(yaml.safe_dump(data, sort_keys=False, default_flow_style=None))
    os.replace(tmp, dst)


def deployed_path(directory: Path, name: str) -> Path:
    return directory / ".deployed" / f"{name}.yaml"


def load_mission(directory: Path, name: str, model: MissionModel) -> dict | None:
    """Load into *model* (metadata when present, else re-imported samples).

    Returns the mission's ``geo_anchor`` dict, if any, so the caller can
    restore the GPS origin the mission was designed with."""
    anchor = read_geo_anchor(runtime_path(directory, name))
    mp = meta_path(directory, name)
    if mp.exists():
        meta = yaml.safe_load(mp.read_text()) or {}
        model.from_dict(meta.get("model", {}))
        model.name = name
        return anchor
    rp = runtime_path(directory, name)
    data = yaml.safe_load(rp.read_text()) or {}
    pts = np.asarray(data.get("points", []), dtype=float)
    # Re-imported samples already hold any closing leg: only the runtime
    # repeat flag carries over.
    model.from_dict({"name": name, "speed": data.get("speed", 0.5),
                     "close_path": False, "repeat": bool(data.get("loop", False)),
                     "items": []})
    if len(pts):
        keep = _decimate(pts[:, 1:3], min_step=5.0)
        for i in keep:
            model.add_waypoint(float(pts[i, 1]), float(pts[i, 2]))
    model.name = name
    return anchor


def _decimate(xy: np.ndarray, min_step: float) -> list[int]:
    keep = [0]
    for i in range(1, len(xy)):
        if np.hypot(*(xy[i] - xy[keep[-1]])) >= min_step or i == len(xy) - 1:
            keep.append(i)
    return keep


def delete_mission(directory: Path, name: str) -> None:
    for p in (runtime_path(directory, name), meta_path(directory, name)):
        p.unlink(missing_ok=True)


def rename_mission(directory: Path, old: str, new: str) -> None:
    for old_p, new_p in ((runtime_path(directory, old), runtime_path(directory, new)),
                         (meta_path(directory, old), meta_path(directory, new))):
        if old_p.exists():
            old_p.rename(new_p)


def duplicate_mission(directory: Path, src: str, dst: str) -> None:
    for src_p, dst_p in ((runtime_path(directory, src), runtime_path(directory, dst)),
                         (meta_path(directory, src), meta_path(directory, dst))):
        if src_p.exists():
            dst_p.write_bytes(src_p.read_bytes())
