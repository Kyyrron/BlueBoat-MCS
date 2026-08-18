"""Mission library I/O.

Two files per mission, in ``designer.trajectories_dir``:

* ``<name>.yaml`` — the **runtime** trajectory: only what execution needs
  (format tag, speed, loop, time-stamped ``[t, x, y, yaw]`` samples plus
  informative length/duration). Consumed by
  ``integration/yaml_trajectory.py`` on the robot side; documented in
  ``docs/08_trajectory_format.md``.
* ``<name>.meta.yaml`` — **editor** metadata: the full designer model
  (groups, locks, segment interpolation settings, comments). The runtime
  never reads it; without it a runtime file can still be re-imported as
  plain waypoints (decimated), so nothing is ever lost beyond styling.
"""

from __future__ import annotations

import datetime
import re
from pathlib import Path

import numpy as np
import yaml

from mcs.designer.model import MissionModel
from mcs.designer.sampling import SampledMission

FORMAT = "blueboat_trajectory/1"
_NAME_RE = re.compile(r"^[A-Za-z0-9_\-]+$")


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
    # rotation relative to local east/north. With an anchor present every
    # waypoint is linked to real-world GPS, and the station deploys the
    # mission into the robot's CURRENT world frame at run time
    # (see deploy_mission and docs/08).
    directory.mkdir(parents=True, exist_ok=True)
    points = [[round(float(t), 3), round(float(x), 4), round(float(y), 4),
               round(float(psi), 5)]
              for t, (x, y), psi in zip(samples.t, samples.xy, samples.yaw)]
    runtime = {
        "format": FORMAT,
        "name": name,
        "generator": "mission-pattern-designer/1.0",
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "frame": "world",
        "speed": float(model.speed),
        "loop": bool(model.loop),
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
    path = runtime_path(directory, name)
    path.write_text(yaml.safe_dump(runtime, sort_keys=False,
                                   default_flow_style=None))
    meta = {"format": "blueboat_trajectory_meta/1", "model": model.to_dict()}
    meta_path(directory, name).write_text(yaml.safe_dump(meta, sort_keys=False))
    return path


def read_geo_anchor(yaml_path: Path) -> dict | None:
    """Return the geo_anchor block of a runtime file, if any."""
    try:
        data = yaml.safe_load(yaml_path.read_text()) or {}
    except OSError:
        return None
    anchor = data.get("geo_anchor")
    if isinstance(anchor, dict) and "lat0" in anchor and "lon0" in anchor:
        return anchor
    return None


def deploy_mission(src: Path, current_fit, dst: Path) -> Path:
    """Convert a GPS-anchored mission into the CURRENT run's world frame.

    The robot's world origin is created wherever robot_interface starts, so
    it differs every run; a GPS-anchored mission must not inherit that
    offset. Each sample is mapped design-frame -> GPS (via the file's own
    geo_anchor) -> today's world frame (via the station's live odom<->GPS
    fit), and yaw is rotated by the net frame rotation. The result is
    written to *dst* -- the file path_generation was pointed at and is
    watching for (it holds position until the file appears).
    """
    import math as _math

    from mcs.core.geo import GeoFit as _GeoFit

    # The current fit's rotation must be trustworthy before deployment: a
    # translation-only fit (heading_aligned False, theta placeholder 0) would
    # rotate the whole mission wrong. The watcher retries until this holds.
    if not getattr(current_fit, "heading_aligned", True):
        raise ValueError("georeference heading not aligned yet "
                         "(needs more vehicle motion)")

    data = yaml.safe_load(src.read_text()) or {}
    anchor = data.get("geo_anchor")
    if not anchor:
        raise ValueError(f"{src} has no geo_anchor")
    theta_a = _math.radians(float(anchor.get("theta_deg", 0.0)))
    fit_a = _GeoFit(theta=theta_a, tx=0.0, ty=0.0,
                    lat0=float(anchor["lat0"]), lon0=float(anchor["lon0"]),
                    rms_m=0.0, n_pairs=0)
    dtheta = current_fit.theta - theta_a
    out_points = []
    for t_s, x, y, yaw in data.get("points", []):
        lat, lon = fit_a.world_to_latlon(float(x), float(y))
        wx, wy = current_fit.latlon_to_world(lat, lon)
        yaw2 = _math.atan2(_math.sin(float(yaw) + dtheta),
                           _math.cos(float(yaw) + dtheta))
        out_points.append([round(float(t_s), 3), round(wx, 4),
                           round(wy, 4), round(yaw2, 5)])
    deployed = dict(data)
    deployed.pop("geo_anchor", None)
    deployed["deployed_from"] = str(src)
    deployed["deployed_fit_rms_m"] = round(float(current_fit.rms_m), 3)
    deployed["points"] = out_points
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(yaml.safe_dump(deployed, sort_keys=False,
                                  default_flow_style=None))
    return dst


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
    model.from_dict({"name": name, "speed": data.get("speed", 0.5),
                     "loop": data.get("loop", False), "items": []})
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
