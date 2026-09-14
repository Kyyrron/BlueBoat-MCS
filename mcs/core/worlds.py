"""Generated Gazebo world folders (BlueBoat-SSS-Sim) — listing, filtering,
duplication.

The simulator's World Builder writes one immutable folder per world under
``~/worlds/<path_name>/<world_name>/`` whose ``metadata.yaml``
(``format: blueboat_world_meta/1``) is the cross-module contract: it carries
the GPS ``geo_anchor`` (world (0, 0), ``theta_deg`` always 0), the
``limits`` rectangle both as world-local metres (``limits.local``) and as
four GPS corners, and the provenance ``source_path`` block naming the MCS
trajectory the world was built from.

Deliberately a local reimplementation of the simulator's enumeration
(``blueboat_sss_sim.worldgen.world_folder.list_worlds``), not an import:
modules integrate by serving identical interfaces, never by importing a
neighbour's package (project rule CM-3 — the simulator's ``builder/paths.py``
mirrors this module's launch-dialog enumeration in exactly the same way).

Every reader here follows :func:`mcs.designer.io_yaml.read_geo_anchor`'s
contract: an unreadable, non-YAML or malformed file answers ``None`` / an
empty list, never an exception — a broken world folder must not break the
launch flow or the designer.
"""

from __future__ import annotations

import math
import shutil
from pathlib import Path

import numpy as np
import yaml

from mcs.core.geo import EARTH_RADIUS_M, latlon_to_local_en

METADATA_FORMAT = "blueboat_world_meta/1"
METADATA_NAME = "metadata.yaml"


def read_world_meta(meta_file: Path) -> dict | None:
    """The parsed ``metadata.yaml`` of a world folder, or None.

    Valid only when the format tag matches, ``geo_anchor`` carries
    ``lat0``/``lon0`` and ``limits.local`` carries all four bounds — the
    fields every consumer below relies on.
    """
    try:
        meta = yaml.safe_load(meta_file.read_text()) or {}
    except (OSError, yaml.YAMLError):
        return None
    if not isinstance(meta, dict) or meta.get("format") != METADATA_FORMAT:
        return None
    anchor = meta.get("geo_anchor")
    if not isinstance(anchor, dict) or "lat0" not in anchor \
            or "lon0" not in anchor:
        return None
    local = (meta.get("limits") or {}).get("local")
    if not isinstance(local, dict) or not all(
            k in local for k in ("x_min", "y_min", "x_max", "y_max")):
        return None
    return meta


def list_worlds(worlds_root: Path) -> list[dict]:
    """Every valid world folder under *worlds_root*, newest first.

    Entries are ``{"path_name", "world_name", "dir", "meta"}`` with the
    names taken from the directory layout (the on-disk truth), not from the
    YAML. Unreadable or foreign folders are skipped silently.
    """
    if not worlds_root.exists():
        return []
    out = []
    for meta_file in sorted(worlds_root.glob(f"*/*/{METADATA_NAME}")):
        meta = read_world_meta(meta_file)
        if meta is None:
            continue
        out.append({"path_name": meta_file.parent.parent.name,
                    "world_name": meta_file.parent.name,
                    "dir": meta_file.parent,
                    "meta": meta})
    out.sort(key=lambda w: str(w["meta"].get("created", "")), reverse=True)
    return out


def mission_latlon_points(mission_yaml: Path) -> list[tuple[float, float]] | None:
    """The GPS positions of every sample of an anchored mission, or None.

    Samples are mapped design -> EN with exactly ``deploy_mission``'s
    convention (a legacy anchor with non-zero ``theta_deg`` recorded
    design = R(theta) @ EN, so EN = R(-theta) @ design), then EN -> lat/lon
    about the mission's own anchor. A file without ``geo_anchor`` or
    ``points`` answers None.
    """
    try:
        data = yaml.safe_load(mission_yaml.read_text()) or {}
    except (OSError, yaml.YAMLError):
        return None
    anchor = data.get("geo_anchor")
    if not isinstance(anchor, dict) or "lat0" not in anchor \
            or "lon0" not in anchor:
        return None
    try:
        pts = np.asarray(data.get("points") or [], dtype=float)
    except (TypeError, ValueError):
        return None
    if pts.ndim != 2 or pts.shape[0] == 0 or pts.shape[1] < 3:
        return None
    theta = math.radians(float(anchor.get("theta_deg", 0.0)))
    c, s = math.cos(theta), math.sin(theta)
    x, y = pts[:, 1], pts[:, 2]
    e = c * x + s * y
    n = -s * x + c * y
    lat0, lon0 = float(anchor["lat0"]), float(anchor["lon0"])
    lat = lat0 + np.degrees(n / EARTH_RADIUS_M)
    lon = lon0 + np.degrees(e / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return list(zip(lat.tolist(), lon.tolist()))


def world_contains_any(meta: dict, latlon_pts) -> bool:
    """True when at least one GPS point falls inside the world's limits.

    Points are projected into the world's local frame about its own anchor
    and tested against the ``limits.local`` rectangle — one point inside is
    enough (the deliberate cross-module rule: a path may be launched in any
    world it merely touches).
    """
    if not latlon_pts:
        return False
    anchor = meta["geo_anchor"]
    lat0, lon0 = float(anchor["lat0"]), float(anchor["lon0"])
    pts = np.asarray(latlon_pts, dtype=float)
    e = np.radians(pts[:, 1] - lon0) * EARTH_RADIUS_M * math.cos(
        math.radians(lat0))
    n = np.radians(pts[:, 0] - lat0) * EARTH_RADIUS_M
    lim = meta["limits"]["local"]
    inside = ((e >= float(lim["x_min"])) & (e <= float(lim["x_max"]))
              & (n >= float(lim["y_min"])) & (n <= float(lim["y_max"])))
    return bool(inside.any())


def eligible_worlds(worlds_root: Path, mission_yaml: Path) -> list[dict]:
    """The worlds a mission can be launched in: any world whose limits
    contain at least one of the mission's GPS points. Empty when the
    mission is unreadable or unanchored."""
    latlon = mission_latlon_points(mission_yaml)
    if latlon is None:
        return []
    return [w for w in list_worlds(worlds_root)
            if world_contains_any(w["meta"], latlon)]


def _patch_yaml(path: Path, mutate) -> None:
    data = yaml.safe_load(path.read_text()) or {}
    mutate(data)
    path.write_text(yaml.safe_dump(data, sort_keys=False,
                                   default_flow_style=None))


def duplicate_world_for_path(src_world_dir: Path, worlds_root: Path,
                             path_name: str, runtime_yaml: Path,
                             path_anchor: dict) -> str:
    """Copy a world folder for a newly saved path; return a status note.

    Target is ``worlds_root/<path_name>/<world_name>``. An existing target
    is never touched (write-once, CM-7) — re-saving a mission is a no-op
    here. After the copy, the provenance fields that named the ORIGINAL
    source path are re-pointed at the new one: ``metadata.yaml``'s
    ``source_path`` (name, file, design_offset) and ``builder_state.yaml``'s
    ``path.name``/``path.file``. The cached ``path.points``/``geo_anchor``
    and ``source_path_world.yaml`` stay untouched — they define the world's
    frame re-basing and still describe the geometry the folder was built
    from. Never raises; a failed copy is removed and reported in the note.
    """
    dst = worlds_root / path_name / src_world_dir.name
    if dst.exists():
        return (f"world '{src_world_dir.name}' already exists under "
                f"{path_name} — not copied")
    try:
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(src_world_dir, dst)
        # Saved anchors always carry theta_deg 0 (the design frame IS local
        # ENU), so the new design_offset — the world origin expressed in the
        # new path's design frame (world = design − design_offset) — is a
        # plain EN offset between the two anchors: [0, 0] when the anchor
        # was taken from the world itself.
        w_meta = read_world_meta(dst / METADATA_NAME)
        if w_meta is None:
            raise ValueError("copied metadata.yaml unreadable")
        w_anchor = w_meta["geo_anchor"]
        ox, oy = latlon_to_local_en(
            float(w_anchor["lat0"]), float(w_anchor["lon0"]),
            float(path_anchor["lat0"]), float(path_anchor["lon0"]))

        def _meta(d: dict) -> None:
            d["source_path"] = {"name": path_name, "file": str(runtime_yaml),
                                "design_offset": [round(float(ox), 3),
                                                  round(float(oy), 3)]}

        _patch_yaml(dst / METADATA_NAME, _meta)

        state_file = dst / "builder_state.yaml"
        if state_file.exists():
            def _state(d: dict) -> None:
                p = d.get("path")
                if isinstance(p, dict):
                    p["name"] = path_name
                    p["file"] = str(runtime_yaml)

            _patch_yaml(state_file, _state)
    except Exception as exc:  # noqa: BLE001 — any failure must not break save
        shutil.rmtree(dst, ignore_errors=True)
        return f"world copy FAILED ({exc}) — {src_world_dir.name} not copied"
    shown = (f"~/{dst.relative_to(Path.home())}"
             if dst.is_relative_to(Path.home()) else str(dst))
    return f"world '{src_world_dir.name}' copied to {shown}"
