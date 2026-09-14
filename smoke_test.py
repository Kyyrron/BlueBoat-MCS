"""Offline smoke test: imports every module, instantiates the full window with
QT_QPA_PLATFORM=offscreen, feeds synthetic telemetry through the SignalBus and
exercises store/geo/predictor/designer logic plus the N1/N2/N4/N8 guarantees.

Runs identically whether or not rclpy is importable: with a sourced ROS the
window brings up a real bridge node, without one it degrades to GUI-only. The
blocks that assert what was published substitute their own node stub, so the
result never depends on which of the two it is.
"""
import math
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from mcs.config.settings import AppConfig
from mcs.core.geo import GeoReferencer
from mcs.core.los_predictor import predict_los_path
from mcs.core.series import TimeSeries

cfg = AppConfig()

# --- TimeSeries ---
ts = TimeSeries(dim=2)
for i in range(10000):
    ts.append(i * 0.1, (i, -i))
a, b = ts.window(10.0, 20.0)
assert len(a) == 101, len(a)
a, b = ts.decimated_window(0, 1000, 100)
assert len(a) <= 101
assert ts.last()[0] == 9999 * 0.1
print("TimeSeries ok")

# --- GeoReferencer: translation-only anchor, EN = world + t ---
# /blueboat/odom is local ENU (axes East/North, origin = launch point), so the
# only unknown between world and GPS is the translation t = EN(world origin).
# No motion is required: a stationary boat's fixes anchor the frame too.
geo = GeoReferencer(cfg.geo)
lat0, lon0 = 43.10, 5.90
# The boat launches at world (0,0) and GPS starts reporting once it has
# reached world (37, -23): the referencer latches its projection origin at
# that FIRST fix, so its t = EN(world origin) = (-37, +23) in its own frame
# — a non-trivial anchor, as in the field (odom starts before GPS).
_t_true = (-37.0, 23.0)
from mcs.core.geo import local_en_to_latlon

t = 0.0
for i in range(300):
    t += 1.0
    x, y = 37.0 + 0.05 * i, -23.0 + 0.03 * i      # world pose (= true EN
    lat, lon = local_en_to_latlon(x, y, lat0, lon0)  # about the launch point)
    geo.add_pair(t, x, y, lat, lon)
assert geo.fit is not None, "geo fit missing"
assert geo.is_valid, f"geo rms {geo.fit.rms_m}"
assert abs(geo.fit.tx - _t_true[0]) < 1e-3 and abs(geo.fit.ty - _t_true[1]) < 1e-3, \
    (geo.fit.tx, geo.fit.ty)
for _p in ((0.0, 0.0), (13.5, -7.25), (-40.0, 60.0)):
    _rt = geo.fit.latlon_to_world(*geo.fit.world_to_latlon(*_p))
    assert math.hypot(_rt[0] - _p[0], _rt[1] - _p[1]) < 1e-6, (_p, _rt)
# world_to_latlon must reproduce the true GPS of a world point.
_lat_w, _lon_w = geo.fit.world_to_latlon(50.0, -15.0)
_lat_t, _lon_t = local_en_to_latlon(50.0, -15.0, lat0, lon0)
assert abs(_lat_w - _lat_t) < 1e-8 and abs(_lon_w - _lon_t) < 1e-8

# Anchors within a couple of fixes, boat stationary — no motion needed.
_g_still = GeoReferencer(cfg.geo)
for _i in range(cfg.geo.min_pairs):
    _g_still.add_pair(float(_i), 7.0, -4.0,
                      *local_en_to_latlon(7.0, -4.0, lat0, lon0))
assert _g_still.is_valid and _g_still.fit.n_pairs == cfg.geo.min_pairs
assert abs(_g_still.fit.tx + 7.0) < 1e-6 and abs(_g_still.fit.ty - 4.0) < 1e-6

# Robustness: one GPS glitch must not drag the median anchor nor flip
# is_valid; a *sustained* odom/GPS inconsistency must flag the fit invalid.
_g2 = GeoReferencer(cfg.geo)
t2 = 0.0
for _i in range(50):
    t2 += 1.0
    _e, _n = 100.0 + 0.1 * _i, -50.0 + 0.05 * _i
    _g2.add_pair(t2, _e - 100.0, _n + 50.0, *local_en_to_latlon(_e, _n, lat0, lon0))
_g2.add_pair(t2 + 1.0, 5.0, 2.5, *local_en_to_latlon(400.0, 300.0, lat0, lon0))
for _i in range(50, 55):
    t2 += cfg.geo.refit_period_s + 1.0
    _e, _n = 100.0 + 0.1 * _i, -50.0 + 0.05 * _i
    _g2.add_pair(t2, _e - 100.0, _n + 50.0, *local_en_to_latlon(_e, _n, lat0, lon0))
assert _g2.is_valid, f"glitch broke the fit (rms {_g2.fit.rms_m})"
assert abs(_g2.fit.tx) < 0.5 and abs(_g2.fit.ty) < 0.5, (_g2.fit.tx, _g2.fit.ty)

import random as _random

_g3 = GeoReferencer(cfg.geo)
_rng = _random.Random(7)
t3 = 0.0
for _i in range(40):
    t3 += cfg.geo.refit_period_s + 0.1
    _e, _n = _rng.uniform(-200, 200), _rng.uniform(-200, 200)
    _g3.add_pair(t3, 0.0, 0.0, *local_en_to_latlon(_e, _n, lat0, lon0))
assert _g3.fit is not None and not _g3.is_valid, f"rms {_g3.fit.rms_m}"
print(f"GeoReferencer ok (t=({geo.fit.tx:.2f}, {geo.fit.ty:.2f}) m, "
      f"rms={geo.fit.rms_m:.3f} m)")

# --- Simulated GPS receiver: pure world -> lat/lon translation model ---
# The exact inverse of what GeoReferencer estimates back from the fixes.
from mcs.core.geo import latlon_to_local_en
from mcs.core.sim_gps import SimGpsModel

_sg = SimGpsModel(43.15, 5.95, 0.0)
_lt, _ln = _sg.fix_for(123.4, -56.7)      # first call latches the origin...
assert _lt == 43.15 and _ln == 5.95
for _dx, _dy in ((3.0, 4.0), (-20.5, 12.25), (0.0, -7.0)):
    _lt, _ln = _sg.fix_for(123.4 + _dx, -56.7 + _dy)
    _want = local_en_to_latlon(_dx, _dy, 43.15, 5.95)
    assert abs(_lt - _want[0]) < 1e-12 and abs(_ln - _want[1]) < 1e-12
# ...at first CALL, not at construction.
_sg2 = SimGpsModel(43.15, 5.95, 0.0)
assert _sg2.fix_for(1000.0, 1000.0) == (43.15, 5.95)
# Seeded noise: reproducible per seed, differs across seeds, sane magnitude.
_na = SimGpsModel(43.15, 5.95, 0.4, seed=42)
_nb = SimGpsModel(43.15, 5.95, 0.4, seed=42)
_nc = SimGpsModel(43.15, 5.95, 0.4, seed=43)
_seq_a = [_na.fix_for(0.1 * _i, 0.05 * _i) for _i in range(20)]
_seq_b = [_nb.fix_for(0.1 * _i, 0.05 * _i) for _i in range(20)]
_seq_c = [_nc.fix_for(0.1 * _i, 0.05 * _i) for _i in range(20)]
assert _seq_a == _seq_b and _seq_a != _seq_c
for _i, (_lt, _ln) in enumerate(_seq_a):
    _e, _n = latlon_to_local_en(_lt, _ln, 43.15, 5.95)
    assert math.hypot(_e - 0.1 * _i, _n - 0.05 * _i) < 5 * 0.4 * math.sqrt(2)
print("sim gps model ok")

# --- LoS predictor converges to target ---
pts = predict_los_path((0, 0, 0), (20, 10), cfg.los)
end = pts[-1]
assert math.hypot(end[0] - 20, end[1] - 10) <= cfg.los.reached_distance_m * 1.5, end
print(f"LoS predictor ok ({len(pts)} pts, end={end})")

# --- Start alignment: the launch dialog's badge, checked at file level ---
# Deliberately placed BEFORE any mcs.gui import: the designer helpers the
# launch dialog calls must stay free of the GUI package (and of Qt widgets),
# which is only assertable while nothing has pulled mcs.gui in yet.
app = QApplication(sys.argv)

import shutil
import tempfile
from pathlib import Path

import yaml as _yaml

from mcs.designer import io_yaml
from mcs.designer.model import MissionModel
from mcs.designer.sampling import sample_mission, start_misalignment

assert not any(m.startswith("mcs.gui") for m in sys.modules), \
    "mcs.designer must not import mcs.gui"

_tol = (cfg.designer.start_align_tol_m, cfg.designer.start_align_tol_deg)
_dir = Path(tempfile.mkdtemp())
try:
    # A mission that starts away from the origin, heading +y: the boat would
    # cut across to it, so both the model and the saved file must say so.
    _m = MissionModel()
    _m.from_dict({"name": "legacy", "speed": 0.6, "loop": False, "items": []})
    for _x, _y in ((12.0, -7.0), (12.0, 3.0), (22.0, 3.0)):
        _m.add_waypoint(_x, _y)
    _s = sample_mission(_m, cfg.designer.sample_ds_m)
    _rt = io_yaml.save_mission(_dir, "legacy", _m, _s)

    _model_side = start_misalignment(_s.xy, *_tol)
    _disk_side = io_yaml.read_start_misalignment(_rt, *_tol)
    assert _model_side is not None and _disk_side is not None
    # The designer and the launch dialog must never disagree about one file.
    assert math.hypot(_model_side[0][0] - _disk_side[0][0],
                      _model_side[0][1] - _disk_side[0][1]) < 1e-3, \
        (_model_side, _disk_side)
    assert abs(_model_side[1] - _disk_side[1]) < 1e-4

    # After Align to Start both report "aligned", and the rigid transform
    # leaves the path geometry alone.
    _m.align_to_start(*_model_side)
    _s2 = sample_mission(_m, cfg.designer.sample_ds_m)
    _rt = io_yaml.save_mission(_dir, "legacy", _m, _s2)
    assert start_misalignment(_s2.xy, *_tol) is None
    assert io_yaml.read_start_misalignment(_rt, *_tol) is None
    assert abs(_s2.length_m - _s.length_m) < 1e-6

    # Malformed / short files degrade to "no badge" — they must never raise
    # out of the launch dialog's combo construction.
    for _n, _raw in (("one", "points: [[0.0, 5.0, 5.0, 0.0]]"),
                     ("none", "speed: 0.5"),
                     ("empty", "points: []"),
                     ("ragged", "points: [[0.0, 1.0], [1.0, 2.0]]"),
                     ("garbage", "points: [[[unclosed\n\t: :")):
        _p = _dir / f"{_n}.yaml"
        _p.write_text(_raw)
        assert io_yaml.read_start_misalignment(_p, *_tol) is None, _n
    assert io_yaml.read_start_misalignment(_dir / "absent.yaml", *_tol) is None

    # GPS-anchored missions are exempt: geographically fixed, never realigned.
    _g = _dir / "anchored.yaml"
    _g.write_text(_yaml.safe_dump(
        {"format": io_yaml.FORMAT, "speed": 0.5, "loop": False,
         "geo_anchor": {"lat0": 43.1, "lon0": 5.9, "theta_deg": 0.0},
         "points": [[0.0, 12.0, -7.0, 0.0], [1.0, 13.0, -7.0, 0.0]]}))
    assert io_yaml.read_geo_anchor(_g) is not None
finally:
    shutil.rmtree(_dir, ignore_errors=True)
print("start alignment ok")

# --- Designer core: the Qt-free layer and its two extension registries ---
# model/sampling/interpolation/patterns/io_yaml are deliberately widget-free so
# they can be exercised without the designer UI; nothing did until now. Still
# before the mcs.gui import, so the layering assertion above covers this too.
import numpy as _np

from mcs.designer import interpolation as _interp_mod
from mcs.designer import patterns as _pat_mod
from mcs.designer.model import SegmentSpec

_dsn_dir = Path(tempfile.mkdtemp())
_saved_interp = dict(_interp_mod.REGISTRY)
_saved_pat = dict(_pat_mod.REGISTRY)
try:
    # Every stock generator must produce a usable path from its OWN schema
    # defaults: the schema is what the editor builds its forms from, so a key
    # the generator reads but the schema omits is a crash in the dialog.
    for _key, _gen in _pat_mod.REGISTRY.items():
        _params = {_row[0]: _row[3] for _row in _gen.schema}
        _params.update({"x0": 3.0, "y0": -1.5})
        _pts = _gen.generate(_params)
        assert len(_pts) >= 1, _key
        assert all(len(_q) == 2 and all(map(math.isfinite, _q)) for _q in _pts), _key

    # Same for every interpolation model, over a non-degenerate segment.
    for _key, _ip in _interp_mod.REGISTRY.items():
        _seg = _ip.sample((-4.0, 0.0), (0.0, 0.0), (10.0, 4.0), (14.0, 4.0),
                          _ip.defaults(), 0.25)
        _seg = _np.asarray(_seg)
        assert _seg.ndim == 2 and _seg.shape[1] == 2 and len(_seg) >= 1, _key
        assert bool(_np.isfinite(_seg).all()), _key
        assert _ip.defaults() == {_row[0]: _row[3] for _row in _ip.schema}, _key

    # A mission mixing a pattern group with hand-placed waypoints, and using
    # a non-straight interpolation on one segment.
    _dsn = MissionModel()
    _dsn.from_dict({"name": "designer", "speed": 0.8, "loop": False, "items": []})
    _dsn.add_group("lawn", "lawnmower",
                   {"width": 12.0, "height": 6.0, "spacing": 3.0,
                    "orientation_deg": 0.0, "start_corner": "SW",
                    "x0": 0.0, "y0": 0.0},
                   _pat_mod.REGISTRY["lawnmower"].generate(
                       {"width": 12.0, "height": 6.0, "spacing": 3.0,
                        "orientation_deg": 0.0, "start_corner": "SW",
                        "x0": 0.0, "y0": 0.0}))
    _dsn.add_waypoint(20.0, 10.0)
    _dsn.add_waypoint(30.0, 10.0)
    _flat_wps = _dsn.flatten()
    assert len(_flat_wps) > 3
    _flat_wps[-2].seg_out = SegmentSpec(kind="arc",
                                        params={"deflection_deg": 80.0})

    _dsn_s = sample_mission(_dsn, cfg.designer.sample_ds_m)
    assert len(_dsn_s.t) > len(_flat_wps)
    assert bool((_np.diff(_dsn_s.t) > 0).all()), "sample times must increase"
    assert bool(_np.isfinite(_dsn_s.yaw).all())
    # With every segment at cruise speed, time is arc length over speed.
    assert abs(_dsn_s.duration_s - _dsn_s.length_m / _dsn.speed) < 1e-6, _dsn_s
    # The arc segment is genuinely curved: a straight chord would be shorter.
    _straight = SegmentSpec(kind="straight")
    _arc_spec = _flat_wps[-2].seg_out
    _flat_wps[-2].seg_out = _straight
    _dsn_straight = sample_mission(_dsn, cfg.designer.sample_ds_m)
    assert _dsn_s.length_m > _dsn_straight.length_m + 1.0, \
        (_dsn_s.length_m, _dsn_straight.length_m)
    _flat_wps[-2].seg_out = _arc_spec

    # Per-segment speed overrides the cruise speed for that segment only.
    _flat_wps[0].seg_out = SegmentSpec(kind="straight", speed=_dsn.speed * 4.0)
    _dsn_fast = sample_mission(_dsn, cfg.designer.sample_ds_m)
    assert _dsn_fast.duration_s < _dsn_s.duration_s
    assert abs(_dsn_fast.length_m - _dsn_s.length_m) < 1e-9  # geometry unchanged
    _flat_wps[0].seg_out = SegmentSpec(kind="straight")

    # Save -> load -> resample must reproduce the mission exactly (the editor
    # model round-trips through <name>.meta.yaml; the runtime file carries the
    # samples the robot actually executes).
    _dsn_rt = io_yaml.save_mission(_dsn_dir, "designer", _dsn, _dsn_s)
    _dsn_back = MissionModel()
    io_yaml.load_mission(_dsn_dir, "designer", _dsn_back)
    _dsn_s2 = sample_mission(_dsn_back, cfg.designer.sample_ds_m)
    assert len(_dsn_s2.t) == len(_dsn_s.t)
    assert abs(_dsn_s2.length_m - _dsn_s.length_m) < 1e-9
    assert abs(_dsn_s2.duration_s - _dsn_s.duration_s) < 1e-9
    _dsn_raw = _yaml.safe_load(_dsn_rt.read_text())
    assert _dsn_raw["format"] == io_yaml.FORMAT
    assert len(_dsn_raw["points"]) == len(_dsn_s.t)
    assert abs(_dsn_raw["length_m"] - _dsn_s.length_m) < 5e-4
    assert abs(_dsn_raw["duration_s"] - _dsn_s.duration_s) < 5e-4
    assert "geo_anchor" not in _dsn_raw
    assert io_yaml.list_missions(_dsn_dir) == ["designer"]

    # Extension contract: subclass + register is the whole story, for both
    # registries. The sampler must route through a newly registered model.
    class _TestJog(_interp_mod.Interpolation):
        key = "_test_jog"
        label = "Test jog"
        schema = [("offset", "Offset (m)", "float", 3.0, 0.0, 10.0)]

        def sample(self, prev, a, b, nxt, params, ds):
            off = float(params.get("offset", 3.0))
            mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0 + off)
            return _np.asarray([[a[0], a[1]], [mid[0], mid[1]]])

    class _TestDot(_pat_mod.Pattern):
        key = "_test_dot"
        label = "Test dot"
        schema = [("count", "Count", "int", 4, 1, 10)]

        def generate(self, p):
            n = int(p["count"])
            return [(float(p["x0"]) + i, float(p["y0"])) for i in range(n)]

    _interp_mod.REGISTRY[_TestJog.key] = _TestJog()
    _pat_mod.REGISTRY[_TestDot.key] = _TestDot()
    assert _interp_mod.REGISTRY[_TestJog.key].defaults() == {"offset": 3.0}
    assert len(_pat_mod.REGISTRY[_TestDot.key].generate(
        {"count": 4, "x0": 0.0, "y0": 0.0})) == 4

    _ext = MissionModel()
    _ext.from_dict({"name": "ext", "speed": 1.0, "loop": False, "items": []})
    _ext.add_waypoint(0.0, 0.0)
    _ext.add_waypoint(10.0, 0.0)
    _ext.flatten()[0].seg_out = SegmentSpec(kind=_TestJog.key,
                                            params={"offset": 4.0})
    _ext_s = sample_mission(_ext, cfg.designer.sample_ds_m)
    # 4 m off a 10 m chord: a straight segment could never be this long.
    assert _ext_s.length_m > 12.0, _ext_s.length_m
    assert bool((_np.abs(_ext_s.xy[:, 1]) > 1e-9).any()), "new model not used"
finally:
    _interp_mod.REGISTRY.clear(); _interp_mod.REGISTRY.update(_saved_interp)
    _pat_mod.REGISTRY.clear(); _pat_mod.REGISTRY.update(_saved_pat)
    shutil.rmtree(_dsn_dir, ignore_errors=True)
print("designer ok")

# --- N8: deploy_mission needs a geo_anchor and a usable translation fit ---
# The world frame is local ENU, so deployment is a pure translation: a plain
# translation fit (no motion, no rotation) is the HAPPY path now. The guard
# still refuses a missing fit or a source without an anchor, writing nothing.
_dep_dir = Path(tempfile.mkdtemp())
try:
    _dep_m = MissionModel()
    _dep_m.from_dict({"name": "anchored", "speed": 0.5, "loop": False,
                      "items": []})
    for _x, _y in ((0.0, 0.0), (10.0, 0.0), (10.0, 8.0)):
        _dep_m.add_waypoint(_x, _y)
    _dep_s = sample_mission(_dep_m, cfg.designer.sample_ds_m)
    # Anchor the design 120 m east / 80 m north of the launch point, so the
    # deployment visibly relocates the mission (a launch-point anchor would
    # deploy to the identity and prove nothing).
    _anch_lat, _anch_lon = local_en_to_latlon(120.0, 80.0, lat0, lon0)
    _dep_src = io_yaml.save_mission(
        _dep_dir, "anchored", _dep_m, _dep_s,
        geo_anchor={"lat0": _anch_lat, "lon0": _anch_lon, "theta_deg": 0.0})
    _dep_n = len(_yaml.safe_load(_dep_src.read_text())["points"])

    _dep_dst = io_yaml.deployed_path(_dep_dir, "anchored")
    assert _dep_dst == _dep_dir / ".deployed" / "anchored.yaml", _dep_dst

    # No fit at all -> refuse, and write nothing.
    try:
        io_yaml.deploy_mission(_dep_src, None, _dep_dst)
        raise AssertionError("deploy_mission accepted a missing fit")
    except ValueError:
        pass
    assert not _dep_dst.exists(), "guard must precede every write"

    # A source without a geo_anchor is not a deployable mission.
    _dep_plain = io_yaml.save_mission(_dep_dir, "plain", _dep_m, _dep_s)
    try:
        io_yaml.deploy_mission(_dep_plain, geo.fit,
                               io_yaml.deployed_path(_dep_dir, "plain"))
        raise AssertionError("deploy_mission accepted a file with no geo_anchor")
    except ValueError:
        pass

    # Translation fit -> deploy into THIS run's world frame.
    io_yaml.deploy_mission(_dep_src, geo.fit, _dep_dst)
    _dep_out = _yaml.safe_load(_dep_dst.read_text())
    assert "geo_anchor" not in _dep_out, "deployed file must not stay anchored"
    assert _dep_out["deployed_from"] == str(_dep_src)
    assert abs(_dep_out["deployed_fit_rms_m"] - round(geo.fit.rms_m, 3)) < 1e-9
    assert len(_dep_out["points"]) == _dep_n
    assert _dep_out["speed"] == _dep_m.speed and _dep_out["loop"] is False
    # Non-vacuous: the anchor sits 120/80 m from the launch point, so the
    # deployed coordinates cannot be the design-frame ones.
    _dep_in_pts = _yaml.safe_load(_dep_src.read_text())["points"]
    assert max(math.hypot(_o[1] - _i[1], _o[2] - _i[2])
               for _i, _o in zip(_dep_in_pts, _dep_out["points"])) > 1.0
    # ...they are exactly what the design->GPS->world chain produces, and
    # yaw passes through untouched (theta_deg == 0: design frame IS ENU).
    for _i, _o in zip(_dep_in_pts, _dep_out["points"]):
        _wx, _wy = geo.fit.latlon_to_world(
            *local_en_to_latlon(_i[1], _i[2], _anch_lat, _anch_lon))
        assert math.hypot(_o[1] - _wx, _o[2] - _wy) < 1e-3, (_i, _o)
        assert abs(_o[3] - round(float(_i[3]), 5)) < 1e-6, (_i, _o)

    # Legacy anchor (theta_deg != 0): points rotated into ENU, yaw by -theta.
    _leg_src = io_yaml.save_mission(
        _dep_dir, "legacy_anchor", _dep_m, _dep_s,
        geo_anchor={"lat0": _anch_lat, "lon0": _anch_lon, "theta_deg": 30.0})
    _leg_dst = io_yaml.deployed_path(_dep_dir, "legacy_anchor")
    io_yaml.deploy_mission(_leg_src, geo.fit, _leg_dst)
    _leg_out = _yaml.safe_load(_leg_dst.read_text())
    _th = math.radians(30.0)
    _c30, _s30 = math.cos(_th), math.sin(_th)
    for _i, _o in zip(_yaml.safe_load(_leg_src.read_text())["points"],
                      _leg_out["points"]):
        _e = _c30 * _i[1] + _s30 * _i[2]
        _n = -_s30 * _i[1] + _c30 * _i[2]
        _wx, _wy = geo.fit.latlon_to_world(
            *local_en_to_latlon(_e, _n, _anch_lat, _anch_lon))
        assert math.hypot(_o[1] - _wx, _o[2] - _wy) < 1e-3, (_i, _o)
        _dy = math.atan2(math.sin(_i[3] - _th - _o[3]),
                         math.cos(_i[3] - _th - _o[3]))
        assert abs(_dy) < 1e-4, (_i, _o)
finally:
    shutil.rmtree(_dep_dir, ignore_errors=True)
print("deploy guard ok")

# --- Gazebo world folders: listing, containment filter, duplication ---
# mcs.core.worlds mirrors the simulator's ~/worlds/<path>/<world>/ contract
# (CM-3: reimplemented, never imported). Still before the mcs.gui import,
# so the module is provably Qt-free.
from mcs.core import worlds as _worlds

assert not any(m.startswith("mcs.gui") for m in sys.modules), \
    "mcs.core.worlds must not import mcs.gui"

_w_root = Path(tempfile.mkdtemp(prefix="mcs_smoke_worlds_"))
try:
    _w_lat, _w_lon = 43.30, 5.70          # world anchor == mission anchor

    def _mk_world(path_name, world_name, meta):
        d = _w_root / path_name / world_name
        d.mkdir(parents=True)
        (d / "metadata.yaml").write_text(_yaml.safe_dump(
            meta, sort_keys=False, default_flow_style=None))
        return d

    _lim50 = {"local": {"x_min": -50.0, "y_min": -50.0,
                        "x_max": 50.0, "y_max": 50.0},
              "corners_gps": [
                  list(local_en_to_latlon(-50, -50, _w_lat, _w_lon)),
                  list(local_en_to_latlon(50, -50, _w_lat, _w_lon)),
                  list(local_en_to_latlon(50, 50, _w_lat, _w_lon)),
                  list(local_en_to_latlon(-50, 50, _w_lat, _w_lon))]}
    _w_in = _mk_world("pA", "w_in", {
        "format": "blueboat_world_meta/1", "world_name": "w_in",
        "created": "2026-09-01T10:00:00",
        "source_path": {"name": "pA", "file": "/orig/pA.yaml",
                        "design_offset": [1.0, 2.0]},
        "geo_anchor": {"lat0": _w_lat, "lon0": _w_lon, "theta_deg": 0.0},
        "limits": _lim50, "objects": [{"object_id": 1, "type": "tire"}]})
    # Same limits but anchored 10 km east: the mission cannot touch it.
    _far_lat, _far_lon = local_en_to_latlon(10_000.0, 0.0, _w_lat, _w_lon)
    _mk_world("pB", "w_out", {
        "format": "blueboat_world_meta/1", "world_name": "w_out",
        "created": "2026-09-02T10:00:00",
        "geo_anchor": {"lat0": _far_lat, "lon0": _far_lon, "theta_deg": 0.0},
        "limits": _lim50, "objects": []})
    (_w_root / "pC" / "w_bad").mkdir(parents=True)
    (_w_root / "pC" / "w_bad" / "metadata.yaml").write_text("{{{not yaml")
    _mk_world("pC2", "w_wrong", {"format": "something_else/9",
                                 "geo_anchor": {"lat0": 0, "lon0": 0},
                                 "limits": _lim50})
    # Extra world-folder payload for the duplication checks below.
    (_w_in / "world.sdf").write_text("<sdf><uri>seabed.stl</uri></sdf>")
    (_w_in / "seabed.stl").write_bytes(b"\x00solid\x00")
    (_w_in / "source_path_world.yaml").write_text(
        "format: blueboat_trajectory/1\nname: pA\npoints: [[0, 0, 0, 0]]\n")
    _bs_points = [[0.0, 1.0, 2.0, 0.0], [1.0, 3.0, 4.0, 0.0]]
    (_w_in / "builder_state.yaml").write_text(_yaml.safe_dump(
        {"format": "blueboat_world_builder_state/1",
         "path": {"name": "pA", "file": "/orig/pA.yaml",
                  "geo_anchor": {"lat0": _w_lat, "lon0": _w_lon,
                                 "theta_deg": 0.0},
                  "points": _bs_points},
         "world_name": "w_in"}, sort_keys=False))

    assert _worlds.read_world_meta(
        _w_root / "pC" / "w_bad" / "metadata.yaml") is None
    assert _worlds.read_world_meta(
        _w_root / "pC2" / "w_wrong" / "metadata.yaml") is None
    _wl = _worlds.list_worlds(_w_root)
    assert [(w_["path_name"], w_["world_name"]) for w_ in _wl] == \
        [("pB", "w_out"), ("pA", "w_in")], _wl     # newest first, dir names
    assert _worlds.list_worlds(_w_root / "absent") == []

    # Containment filter: mission design points (0..10, 0..8) about the
    # world's own anchor fall inside w_in's ±50 m limits and 10 km outside
    # w_out's. One point inside is enough by contract.
    _wm_dir = Path(tempfile.mkdtemp(prefix="mcs_smoke_wmission_"))
    try:
        _wm = MissionModel()
        _wm.from_dict({"name": "wm", "speed": 0.5, "loop": False,
                       "items": []})
        for _x, _y in ((0.0, 0.0), (10.0, 0.0), (10.0, 8.0)):
            _wm.add_waypoint(_x, _y)
        _wm_s = sample_mission(_wm, cfg.designer.sample_ds_m)
        _wm_src = io_yaml.save_mission(
            _wm_dir, "wm", _wm, _wm_s,
            geo_anchor={"lat0": _w_lat, "lon0": _w_lon, "theta_deg": 0.0})
        _el = _worlds.eligible_worlds(_w_root, _wm_src)
        assert [w_["world_name"] for w_ in _el] == ["w_in"], _el
        # Unanchored mission / unreadable file -> no worlds, never a raise.
        _wm_plain = io_yaml.save_mission(_wm_dir, "wm_plain", _wm, _wm_s)
        assert _worlds.eligible_worlds(_w_root, _wm_plain) == []
        assert _worlds.eligible_worlds(_w_root,
                                       _wm_dir / "absent.yaml") == []

        # Legacy theta anchor: the containment must honour deploy_mission's
        # design->EN rotation. Points near (55, 0) sit OUTSIDE the ±50 m box
        # unrotated, INSIDE it once rotated by theta = 45°.
        _th_m = MissionModel()
        _th_m.from_dict({"name": "th", "speed": 0.5, "loop": False,
                         "items": []})
        for _x, _y in ((55.0, 0.0), (56.0, 0.0)):
            _th_m.add_waypoint(_x, _y)
        _th_s = sample_mission(_th_m, cfg.designer.sample_ds_m)
        _th_rot = io_yaml.save_mission(
            _wm_dir, "th_rot", _th_m, _th_s,
            geo_anchor={"lat0": _w_lat, "lon0": _w_lon, "theta_deg": 45.0})
        _th_flat = io_yaml.save_mission(
            _wm_dir, "th_flat", _th_m, _th_s,
            geo_anchor={"lat0": _w_lat, "lon0": _w_lon, "theta_deg": 0.0})
        assert [w_["world_name"] for w_ in
                _worlds.eligible_worlds(_w_root, _th_rot)] == ["w_in"]
        assert _worlds.eligible_worlds(_w_root, _th_flat) == []

        # Duplication: copy for a NEW path anchored 100 m E / 40 m S of the
        # world origin -> design_offset = EN of the world origin in the new
        # path's frame = (-100, +40).
        _np_lat, _np_lon = local_en_to_latlon(100.0, -40.0, _w_lat, _w_lon)
        _np_anchor = {"lat0": _np_lat, "lon0": _np_lon, "theta_deg": 0.0}
        _np_yaml = _wm_dir / "newpath.yaml"
        _np_yaml.write_text("format: blueboat_trajectory/1\n")
        _note = _worlds.duplicate_world_for_path(
            _w_in, _w_root, "newpath", _np_yaml, _np_anchor)
        _dup = _w_root / "newpath" / "w_in"
        assert _dup.is_dir(), _note
        assert (_dup / "world.sdf").read_bytes() == \
            (_w_in / "world.sdf").read_bytes()
        assert (_dup / "source_path_world.yaml").read_bytes() == \
            (_w_in / "source_path_world.yaml").read_bytes()
        _dup_meta = _yaml.safe_load((_dup / "metadata.yaml").read_text())
        assert _dup_meta["source_path"]["name"] == "newpath"
        assert _dup_meta["source_path"]["file"] == str(_np_yaml)
        _off = _dup_meta["source_path"]["design_offset"]
        assert abs(_off[0] + 100.0) < 2e-2 and abs(_off[1] - 40.0) < 2e-2, \
            _off
        assert _dup_meta["geo_anchor"]["lat0"] == _w_lat  # world not moved
        _dup_bs = _yaml.safe_load((_dup / "builder_state.yaml").read_text())
        assert _dup_bs["path"]["name"] == "newpath"
        assert _dup_bs["path"]["file"] == str(_np_yaml)
        assert _dup_bs["path"]["points"] == _bs_points    # geometry frozen
        assert _dup_bs["path"]["geo_anchor"]["lat0"] == _w_lat
        # Write-once: a second save never touches the existing copy.
        (_dup / "sentinel").write_text("x")
        _note2 = _worlds.duplicate_world_for_path(
            _w_in, _w_root, "newpath", _np_yaml, _np_anchor)
        assert "not copied" in _note2, _note2
        assert (_dup / "sentinel").exists()
        assert _yaml.safe_load((_dup / "metadata.yaml").read_text()) == \
            _dup_meta
    finally:
        shutil.rmtree(_wm_dir, ignore_errors=True)
finally:
    shutil.rmtree(_w_root, ignore_errors=True)
print("worlds ok")

# --- Launch dialog: sim + GPS-anchored path arms the simulated-GPS flow ---
# In simulation an anchored mission takes the SAME deferred-deploy branch as
# on real water, plus gps_simulated and a FIXED spawn heading (two runs of one
# mission must start identically); non-anchored paths change nothing. The log
# note reaches every branch, sanitised, 'sim'-tagged in simulation.
from mcs.gui.dialogs.launch_dialog import LaunchDialog

_ld_dir = Path(tempfile.mkdtemp())
try:
    _cfg2 = AppConfig()
    _cfg2.designer.trajectories_dir = str(_ld_dir)
    _ld_m = MissionModel()
    _ld_m.from_dict({"name": "m", "speed": 0.5, "loop": False, "items": []})
    for _x, _y in ((0.0, 0.0), (8.0, 0.0)):
        _ld_m.add_waypoint(_x, _y)
    _ld_s = sample_mission(_ld_m, cfg.designer.sample_ds_m)
    _ld_anch = io_yaml.save_mission(
        _ld_dir, "anchored_sim", _ld_m, _ld_s,
        geo_anchor={"lat0": 43.2, "lon0": 5.8, "theta_deg": 0.0})
    _ld_plain = io_yaml.save_mission(_ld_dir, "plain_sim", _ld_m, _ld_s)

    _dlg = LaunchDialog(_cfg2, None)
    _dlg._mode.setCurrentText("Gazebo simulation")
    _i = _dlg._trajectory.findData(str(_ld_anch))
    assert _i >= 0, "anchored mission not listed"
    _dlg._trajectory.setCurrentIndex(_i)
    _p = _dlg.parameters()
    assert _p.simulation and _p.gps_simulated
    assert _p.gps_anchored_source == str(_ld_anch)
    assert _p.trajectory == \
        f"from_yaml:{io_yaml.deployed_path(_ld_dir, 'anchored_sim')}"
    assert _p.spawn_yaw_rad is not None
    assert -math.pi <= _p.spawn_yaw_rad <= math.pi
    # Deterministic spawn: the configured heading, and the same every call.
    assert abs(_p.spawn_yaw_rad
               - math.radians(_cfg2.launch.sim_spawn_yaw_deg)) < 1e-12
    assert _dlg.parameters().spawn_yaw_rad == _p.spawn_yaw_rad
    # The personalized-world choice happens AFTER the dialog (bottom
    # toolbar); parameters() itself never sets it.
    assert _p.world_dir == ""
    _cli = " ".join(_p.to_cli())
    assert "spawn_yaw:=" in _cli and "robot_file:=" in _cli \
        and "controller_type:=" in _cli, _cli
    # Log note: simulation carries the 'sim' marker with no operator note...
    assert "note:=sim" in _cli, _cli
    _dlg._note.setText("dam run/2")
    _pn = _dlg.parameters()
    assert _pn.note == "dam run/2"                      # raw text is kept
    assert _pn.wire_note() == "sim-dam-run-2"           # one safe CLI token
    assert f"note:={_pn.wire_note()}" in _pn.to_cli()
    _dlg._note.clear()

    _i = _dlg._trajectory.findData(str(_ld_plain))
    _dlg._trajectory.setCurrentIndex(_i)
    _p2 = _dlg.parameters()
    assert not _p2.gps_simulated and _p2.spawn_yaw_rad is None
    assert _p2.gps_anchored_source == ""
    assert _p2.trajectory == f"from_yaml:{_ld_plain}"
    assert "spawn_yaw:=" not in " ".join(_p2.to_cli())

    _dlg._mode.setCurrentText("Real robot")
    # The log note exists in both modes (simulation_interface names its poslog
    # from it exactly as robot_interface does), so the field stays visible.
    assert _dlg._note.isVisibleTo(_dlg) and _dlg._note_label.isVisibleTo(_dlg)
    _i = _dlg._trajectory.findData(str(_ld_anch))
    _dlg._trajectory.setCurrentIndex(_i)
    _p3 = _dlg.parameters()
    assert not _p3.simulation and not _p3.gps_simulated
    assert _p3.spawn_yaw_rad is None
    assert "note:=" not in " ".join(_p3.to_cli())   # empty note: no argument
    _dlg._note.setText(" field test ")
    _p3n = _dlg.parameters()
    assert _p3n.wire_note() == "field-test"        # no 'sim' tag on real water
    assert "note:=field-test" in _p3n.to_cli()
    _dlg._note.clear()
    assert _p3.gps_anchored_source == str(_ld_anch)   # real branch unchanged
    _dlg.deleteLater()
finally:
    shutil.rmtree(_ld_dir, ignore_errors=True)
print("launch dialog sim-gps ok")

# --- Personalized-world launch: CLI branch, target choice, choice dialog ---
# world_dir set -> blueboat_sss_sim full_mission_launch.py with EXACTLY the
# arguments it declares (an undeclared argument aborts ros2 launch, so
# robot_file/trajectory/spawn_yaw must not leak into this branch).
from mcs.core.geo import GeoFit as _GeoFit
from mcs.designer.designer_map import DesignerMapView as _DMV
from mcs.gui.dialogs.world_dialog import WorldChoiceDialog
from mcs.ros.launch_manager import LaunchParameters as _WLP
from mcs.ros.launch_manager import launch_target as _launch_target

_wp = _WLP(simulation=True, gps_simulated=True, controller_type="PID",
           world_dir="/w/pA/w1", gps_deployed_target="/tmp/d.yaml",
           extra_args={"with_mavros_shim": "false"})
_wcli = _wp.to_cli()
_wjoin = " ".join(_wcli)
assert "world_dir:=/w/pA/w1" in _wjoin and "with_control:=true" in _wjoin
assert "trajectory_file:=/tmp/d.yaml" in _wjoin
assert "controller_type:=PID" in _wjoin
assert "note:=sim" in _wjoin          # full_mission_launch.py declares it too
assert "robot_file:=" not in _wjoin and "spawn_yaw:=" not in _wjoin
assert "trajectory:=" not in _wjoin      # trajectory_file:= only
assert _wcli[-1] == "with_mavros_shim:=false"   # extra args stay last
assert _launch_target(cfg.launch, _wp) == \
    (cfg.launch.sim_world_package, cfg.launch.sim_world_launch_file)
assert _launch_target(cfg.launch, _WLP(simulation=True)) == \
    (cfg.launch.package, cfg.launch.sim_launch_file)
assert _launch_target(cfg.launch, _WLP()) == \
    (cfg.launch.package, cfg.launch.launch_file)

# The choice dialog, driven headless: it renders the list it is given and
# reports either a world dir or None ("Empty Gazebo").
_fake_worlds = [
    {"path_name": "pA", "world_name": "w1", "dir": Path("/w/pA/w1"),
     "meta": {"created": "2026-09-01T10:00:00", "objects": [],
              "geo_anchor": {"lat0": 43.1, "lon0": 5.9},
              "limits": {"local": {"x_min": -50.0, "y_min": -50.0,
                                   "x_max": 50.0, "y_max": 50.0}}}},
    {"path_name": "pB", "world_name": "w2", "dir": Path("/w/pB/w2"),
     "meta": {"created": "2026-08-30T10:00:00", "objects": [{}, {}],
              "geo_anchor": {"lat0": 43.2, "lon0": 5.8},
              "limits": {"local": {"x_min": 0.0, "y_min": 0.0,
                                   "x_max": 10.0, "y_max": 10.0}}}},
]
_wd = WorldChoiceDialog(_fake_worlds)
assert _wd._list.count() == 2
_wd._list.setCurrentRow(1)
_wd._accept_world()
assert _wd.selected_world_dir == str(Path("/w/pB/w2"))
assert _wd.result() == _wd.DialogCode.Accepted
_wd.deleteLater()
_wd2 = WorldChoiceDialog(_fake_worlds)
_wd2._accept_empty()
assert _wd2.selected_world_dir is None
assert _wd2.result() == _wd2.DialogCode.Accepted
_wd2.deleteLater()

# Designer overlay: the world-limits rectangle is drawn only once an anchor
# exists, repositions with the fit, and hides on request.
_dmv = _DMV(cfg, MissionModel())
_dmv_corners = [[43.0995, 5.8994], [43.0995, 5.9006],
                [43.1005, 5.9006], [43.1005, 5.8994]]
_dmv.set_world_limits(_dmv_corners, "pA/w1")
assert not _dmv._world_rect.isVisible()          # no geo fit yet
_dmv.set_geo_fit(_GeoFit(tx=0.0, ty=0.0, lat0=43.1, lon0=5.9,
                         rms_m=0.0, n_pairs=0))
assert _dmv._world_rect.isVisible() and _dmv._world_marker.isVisible()
_dmv.set_world_limits(None)
assert not _dmv._world_rect.isVisible()
assert not _dmv._world_marker.isVisible()
_dmv.deleteLater()
print("world launch ok")

# --- Sea state (simulator current + waves) ---
import json as _json_sea

from mcs.core.sea import (
    SeaCatalog,
    SeaChoice,
    compass_name,
    direction_text,
    find_presets_file,
    parse_direction,
    parse_readback,
    read_schedule,
    schedule_doc,
    schedule_rows,
    write_schedule,
)
from mcs.gui.dialogs.sea_state_dialog import SeaStateDialog
from mcs.ros.launch_manager import companion_command as _companion

_cat_fb = SeaCatalog.load(None)
assert next(iter(_cat_fb.current)) == "none" and next(iter(_cat_fb.waves)) == "calm"
assert _cat_fb.source == "" and _cat_fb.label("waves", "rough") == "rough"
_cat_bad = SeaCatalog.load(Path(tempfile.mkdtemp()) / "missing.yaml")
assert _cat_bad.source == ""                       # never raises
_pf = find_presets_file(cfg.sea.presets_file)
_cat = SeaCatalog.load(_pf)
if _pf is not None:                                 # simulator installed
    assert _cat.source and _cat.description("current", "weak")
    assert "Douglas" in _cat.label("waves", "rippled") or _cat.label("waves", "rippled")
assert compass_name(0) == "N" and compass_name(225) == "SW" and compass_name(359) == "N"
assert parse_direction("NE") == 45.0 and parse_direction("270") == 270.0
assert parse_direction("nonsense") is None
assert direction_text(0.0).startswith("from N (→ S")
_null = SeaChoice()
assert _null.is_null and _null.launch_args()[0] == "sea_current:=none"
_sea = SeaChoice(current="strong", current_from_deg=90.0, waves="slight",
                 waves_from_deg=45.0, seed=3)
assert not _sea.is_null
assert _sea.launch_args() == ["sea_current:=strong", "sea_current_from_deg:=90.0",
                              "sea_waves:=slight", "sea_waves_from_deg:=45.0",
                              "sea_seed:=3"]
_cmd = _json_sea.loads(_sea.command_json(12.0))
assert _cmd["current"]["preset"] == "strong" and _cmd["ramp_s"] == 12.0
# World branch: sea args ride full_mission_launch.py, before the extras.
_wps = _WLP(simulation=True, gps_simulated=True, controller_type="PID",
            world_dir="/w/pA/w1", gps_deployed_target="/tmp/d.yaml", sea=_sea,
            extra_args={"with_mavros_shim": "false"})
_wcs = _wps.to_cli()
assert "sea_current:=strong" in _wcs and "sea_waves_from_deg:=45.0" in _wcs
assert _wcs.index("sea_current:=strong") < _wcs.index("with_mavros_shim:=false")
assert _wcs[-1] == "with_mavros_shim:=false"
assert _companion(cfg, _wps) is None                # world mode: no companion
# Stock branch: Sim_launch.py gets no sea args; a companion carries them.
_sps = _WLP(simulation=True, controller_type="PID", sea=_sea)
assert not any(a.startswith("sea_") for a in _sps.to_cli())
_cc = _companion(cfg, _sps)
assert _cc[:4] == ["ros2", "launch", "blueboat_sss_sim", "sea_state_launch.py"]
assert "world_name:=ocean" in _cc and "sea_waves:=slight" in _cc
assert _companion(cfg, _WLP(simulation=True, sea=SeaChoice())) is None  # null
assert _companion(cfg, _WLP(simulation=False, sea=_sea)) is None       # real
# Dialog, headless: defaults are null; a choice round-trips; a timeline
# saves as the simulator's schedule format and loads back.
_sea_dir = Path(tempfile.mkdtemp())
_scfg = cfg.sea.__class__(schedules_dir=str(_sea_dir))
_sd = SeaStateDialog(_cat, _scfg, None)
assert _sd.choice().is_null
_sd.set_choice(_sea)
assert _sd.choice() == _sea
_sd._add_row()
_sd._add_row()
_rows = _sd.rows()
assert len(_rows) == 2 and _rows[1]["t_s"] == 300.0 and _rows[0]["current"] == "strong"
_sched_path = write_schedule(_sea_dir / "build.yaml",
                             schedule_doc(_rows, "build", seed=7))
_doc = read_schedule(_sched_path)
assert _doc["format"] == "blueboat_sea_schedule/1" and len(_doc["keyframes"]) == 2
assert schedule_rows(_doc) == _rows
_sd2 = SeaStateDialog(_cat, _scfg, SeaChoice(schedule_file=str(_sched_path)))
assert _sd2.choice().schedule_file == str(_sched_path) and len(_sd2.rows()) == 2
assert "sea_schedule:=" + str(_sched_path) in _sd2.choice().launch_args()
_sd.deleteLater()
_sd2.deleteLater()
# Custom waves: explicit Hs/Tp/gamma/events instead of a preset -> a
# one-keyframe schedule file at launch, explicit fields in a live command.
_sd3 = SeaStateDialog(_cat, _scfg, None)
_sd3._waves.setCurrentIndex(_sd3._waves.findData("custom"))
assert _sd3.is_custom() and _sd3._custom_box.isVisibleTo(_sd3)
_sd3._c_hs.setValue(0.3); _sd3._c_tp.setValue(4.0); _sd3._c_rate.setValue(9)
_sd3._accept()
_cc3 = _sd3.choice()
assert _cc3.custom_waves["hs_m"] == 0.3 and _cc3.custom_waves["events"]["rate_per_hour"] == 9.0
assert _cc3.schedule_file.endswith("custom_waves.yaml") and not _cc3.is_null
_doc3 = read_schedule(_cc3.schedule_file)
assert _doc3["keyframes"][0]["waves"]["hs_m"] == 0.3 and "preset" not in _doc3["keyframes"][0]["waves"]
assert "sea_schedule:=" + _cc3.schedule_file in _cc3.launch_args()
_live3 = _json_sea.loads(SeaChoice(custom_waves=_cc3.custom_waves, waves_from_deg=30.0).command_json(5.0))
assert _live3["waves"]["tp_s"] == 4.0 and _live3["waves"]["from_deg"] == 30.0
assert "custom waves Hs 0.30" in _cc3.summary(_cat)
_sd3.deleteLater()
# Readback parse + the store/panel gating.
_rb = parse_readback({"t_sim": 12.5, "current": {"preset": "strong", "label": "Strong",
                      "mean_speed_mps": 0.5, "from_deg": 90.0,
                      "instant": {"speed_mps": 0.47}},
                      "waves": {"preset": "slight", "label": "Slight", "hs_m": 0.35,
                                "tp_s": 2.5, "from_deg": 45.0, "status": "active",
                                "eta_m": 0.12},
                      "schedule": {"name": "x", "next_t_sim": 300.0, "next": "rough"},
                      "summary": "strong from E; slight from NE"}, received_mono=1.0)
assert _rb is not None and _rb.current_instant_mps == 0.47
assert _rb.waves_text.startswith("Slight Hs 0.35") and "in 288 s" in _rb.next_change_text
_rb2 = parse_readback({"waves": {"label": "Smooth", "hs_m": 0.15, "tp_s": 2.8,
                                 "status": "active", "events_per_hour": 12,
                                 "event": {"hs_m": 0.14, "tp_s": 5.5,
                                           "from_deg": 70, "t_remaining_s": 18}}})
assert "wake group 0.14 m 6 s from ENE, 18 s left" in _rb2.waves_text
assert parse_readback("not json") is None
print("sea state ok")

# --- Full window in offscreen mode, synthetic telemetry ---
from mcs.gui.main_window import MainWindow

w = MainWindow(cfg)  # RosManager connects, or degrades if rclpy is absent
# Construction must succeed in both regimes: with rclpy present
# RosManager connects, without it the app degrades GUI-only. Assert the
# flag is a real bool either way — `not available or True` was a
# tautology that could never fail.
assert w.ros.available in (True, False)

# Battery: nothing has arrived yet, so the row must say so rather than
# render a number. /mavros/battery exists only on the real boat.
w._on_tick(); app.processEvents()
assert w.store.robot.battery_pct is None
assert w.left_panel.robot_grid._values["Battery"].text() == "no data"

t0 = time.monotonic()
for i in range(200):
    tm = t0 + i * 0.05
    x, y, yaw = 0.1 * i, 0.05 * i, 0.01 * i
    w.bus.odom_received.emit(tm, [x, y, 0, 0, 0, yaw], [0.5, 0.1, 0, 0, 0, 0.02])
    w.bus.pinger_body_received.emit(tm, [3.0, 1.0, -2.0])
    w.bus.monitoring_received.emit(tm, [i*0.05, x, y, yaw, x+2, y+1, 0, 1.0, 1.2])
    w.bus.thruster_received.emit(tm, 1.0, 1.2)
    # GPS consistent with the local-ENU odom (world = EN about the origin
    # fix): on_gps pairs each fix with the concurrent odom pose.
    w.bus.gps_received.emit(tm, *local_en_to_latlon(x, y, 43.1, 5.9))
app.processEvents()

# --- Battery read-out (/mavros/battery, sensor_msgs/BatteryState) ---
# percentage is a 0..1 FRACTION on the wire; the panel shows percent.
_bt = time.monotonic()
w.bus.battery_received.emit(_bt, 15.62, 0.82)
w._on_tick(); app.processEvents()
assert w.store.robot.battery_pct == 0.82 and w.store.robot.battery_v == 15.62
_batt_txt = w.left_panel.robot_grid._values["Battery"].text()
assert "82.0 %" in _batt_txt and "15.62 V" in _batt_txt, _batt_txt
# A field the FCU does not report arrives as None and must not wipe the
# last good reading -- only the stamp advances.
w.bus.battery_received.emit(time.monotonic(), None, None)
w._on_tick(); app.processEvents()
assert w.store.robot.battery_pct == 0.82 and w.store.robot.battery_v == 15.62
# Colour follows the charge, and a stale row is greyed rather than trusted.
from mcs.gui import theme
from mcs.gui.left_panel import _battery_text as _bat_txt
from mcs.models.store import RobotState as _RS

assert _bat_txt(_RS(battery_pct=0.82, battery_t=time.monotonic()))[1] == theme.OK
assert _bat_txt(_RS(battery_pct=0.30, battery_t=time.monotonic()))[1] == theme.WARN
assert _bat_txt(_RS(battery_pct=0.05, battery_t=time.monotonic()))[1] == theme.ERR
assert _bat_txt(_RS(battery_pct=0.82,
                    battery_t=time.monotonic() - 60.0))[1] == theme.TEXT_DIM
assert _bat_txt(_RS())[0] == "no data"
# Voltage alone (no percentage reported) still renders.
assert "12.10 V" in _bat_txt(_RS(battery_v=12.1, battery_t=time.monotonic()))[0]
# The topic is registered for diagnostics purely by living in TopicsConfig.
assert cfg.topics.battery == "/mavros/battery"
assert cfg.topics.battery in cfg.diagnostics.warn_age_s
print("battery ok")

assert w.store.robot.has_odom
assert len(w.store.robot_track) == 200
assert w.store.pinger.seen and w.store.pinger.world is not None
assert w.store.pinger.distance_m is not None
d = w.store.active_target_distance()
print(f"store ok (pinger world={w.store.pinger.world}, robot travelled={w.store.robot.travelled_m:.2f} m)")

# manual target flow
w.store.mission.launch_running = True
w.store.mission.controller_type = "LoS"
w.store.mission.manual_target = (15.0, 5.0)
from mcs.models.store import TargetMode

assert w.store.mission.target_mode is TargetMode.MANUAL
assert w.store.active_target_world() == (15.0, 5.0)

# tick everything a few times (repaints offscreen)
for _ in range(5):
    w._on_tick()
    app.processEvents()

stats = w.store.statistics(0.0, 100.0)
assert stats.travelled_m > 0 and stats.max_speed > 0
print(f"stats ok ({stats})")

# --- N4: the pinger world position is anchored, not re-derived ---
# It is computed ONLY inside the pinger callback, with the pose concurrent
# with that message. Composing a stale body vector with each new odom pose
# made the marker trail the robot around the map.
_pa_t = t0 + 100.0
_pa_pose_a = (40.0, 20.0, 0.3)
w.bus.odom_received.emit(_pa_t, [_pa_pose_a[0], _pa_pose_a[1], 0, 0, 0,
                                 _pa_pose_a[2]], [0.0, 0.0, 0, 0, 0, 0.0])
w.bus.pinger_body_received.emit(_pa_t, [6.0, -2.0, -3.0])
_pa_anchored = w.store.pinger.world
assert _pa_anchored is not None

# Robot drives on; no new pinger message arrives.
for _i in range(1, 21):
    _pa_t += 0.05
    w.bus.odom_received.emit(
        _pa_t, [_pa_pose_a[0] + 0.4 * _i, _pa_pose_a[1] + 0.3 * _i, 0, 0, 0,
                _pa_pose_a[2] + 0.02 * _i], [0.5, 0.0, 0, 0, 0, 0.02])
    assert w.store.pinger.world == _pa_anchored, (
        _pa_anchored, w.store.pinger.world)
app.processEvents()

# Non-vacuous: the robot moved far enough (and turned enough) that a
# re-anchored marker would have visibly dragged along with it.
_pa_r = w.store.robot
_pa_c, _pa_s = math.cos(_pa_r.yaw), math.sin(_pa_r.yaw)
_pa_dragged = (_pa_r.x + _pa_c * 6.0 - _pa_s * -2.0,
               _pa_r.y + _pa_s * 6.0 + _pa_c * -2.0)
assert math.hypot(_pa_dragged[0] - _pa_anchored[0],
                  _pa_dragged[1] - _pa_anchored[1]) > 5.0

# A fresh pinger message is the only thing that moves it.
w.bus.pinger_body_received.emit(_pa_t, [6.0, -2.0, -3.0])
assert w.store.pinger.world != _pa_anchored
assert math.hypot(w.store.pinger.world[0] - _pa_dragged[0],
                  w.store.pinger.world[1] - _pa_dragged[1]) < 1e-9
print("pinger anchor ok")

# timeline slider behaviour
w.right_panel.slider.set_maximum(120.0, keep_high_at_end=True)
assert w.right_panel.slider.high_at_end()
w.right_panel.slider.set_values(10.0, 60.0)
assert not w.right_panel.slider.high_at_end()

# map modes
from mcs.gui.map.map_view import MapMode

w.map_view.set_mode(MapMode.MANUAL_TARGET)
got = []
w.map_view.target_clicked.connect(lambda x, y: got.append((x, y)))
w._on_target_clicked(12.0, -4.0)
assert w.store.mission.manual_target == (12.0, -4.0)

# --- Map frame: one GPS/ENU scene, gated on the anchor, exact inverses ---
# The scene is local east/north metres about the latched GPS origin (pure
# translation EN = world + t); every mouse position comes back through the
# exact inverse. Nothing is drawn — and manual-target clicks are refused —
# until the anchor exists; in simulation the identity anchor draws at once.
from PySide6.QtCore import QEvent, QPoint, QPointF, Qt
from PySide6.QtGui import QMouseEvent

from mcs.core.geo import GeoReferencer

_mv = w.map_view
_saved_geo = w.store.geo
_saved_publish = w.commands.publish_manual_target
_pub: list[tuple[float, float]] = []
_seen: list[str] = []
w.commands.publish_manual_target = lambda x, y: _pub.append((x, y))
_mv.point_inspected.connect(_seen.append)

def _click(px: int, py: int):
    """Dispatch a real left press at a viewport pixel; return its scene point."""
    _mv.mousePressEvent(QMouseEvent(
        QEvent.Type.MouseButtonPress, QPointF(px, py), QPointF(px, py),
        Qt.MouseButton.LeftButton, Qt.MouseButton.LeftButton,
        Qt.KeyboardModifier.NoModifier))
    return _mv.mapToScene(QPoint(px, py))

try:
    _c = _mv.viewport().rect().center()
    _px, _py = _c.x() + 37, _c.y() + 23

    # 0. pre-anchor gating: real water, no fit -> nothing drawn, clicks refused
    w.store.geo = GeoReferencer(cfg.geo)   # no pairs yet
    assert not w.store.mission.simulation
    assert not w.store.map_frame_ready()
    _mv.refresh()
    assert not _mv._waiting_label.isHidden()
    for _item in (_mv.robot_item, _mv.robot_track, _mv.mission_path,
                  _mv.pinger_marker, _mv.manual_marker, _mv.target_line):
        assert not _item.isVisible(), _item
    _mv.set_mode(MapMode.MANUAL_TARGET)
    _pub.clear(); _seen.clear()
    _click(_px, _py)
    assert not _pub, "click must be refused without a GPS anchor"
    assert _seen and "waiting for GPS" in _seen[-1], _seen

    # 0b. simulation: identity anchor, draws immediately (the F8 regression)
    w.store.mission.simulation = True
    assert w.store.map_frame_ready()
    _mv.refresh()
    assert _mv._waiting_label.isHidden()
    assert _mv.robot_item.isVisible()
    _mv.set_mode(MapMode.MANUAL_TARGET)
    _pub.clear()
    _scene = _click(_px, _py)
    assert _pub and _pub[-1] == (_scene.x(), _scene.y()), (_pub, _scene)

    # 0c. GPS-simulated run: the full anchor gate applies even in simulation
    w.store.mission.gps_simulated = True
    w.store.geo = GeoReferencer(cfg.geo)
    assert not w.store.map_frame_ready(), "gps-sim must wait for the anchor"
    _mv.refresh()
    assert not _mv._waiting_label.isHidden()
    _mv.set_mode(MapMode.MANUAL_TARGET)
    _pub.clear()
    _click(_px, _py)
    assert not _pub, "click must be refused before the simulated anchor"
    # Simulated fixes anchor the frame — no motion required.
    _t0c = time.monotonic() + 500.0
    for _i in range(cfg.geo.min_pairs + 1):
        _t0c += 0.2
        w.bus.odom_received.emit(_t0c, [0.2 * _i, 0.1 * _i, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0])
        w.bus.gps_received.emit(
            _t0c, *local_en_to_latlon(0.2 * _i, 0.1 * _i, 43.25, 5.85))
    assert w.store.geo.is_valid and w.store.map_frame_ready()
    _mv.refresh()
    assert _mv._waiting_label.isHidden() and _mv.robot_item.isVisible()
    _mv.set_mode(MapMode.MANUAL_TARGET)
    _pub.clear()
    _scene = _click(_px, _py)
    assert _pub, "anchored gps-sim click must publish"
    _bk = _mv._to_scene(*_pub[-1])
    assert math.hypot(_bk[0] - _scene.x(), _bk[1] - _scene.y()) < 1e-6
    w.store.mission.simulation = False
    w.store.mission.gps_simulated = False

    # -- anchored fit (t = (37, -23)): the scene is EN about (lat0, lon0)
    w.store.geo = geo
    assert w.store.map_frame_ready()
    _mv.refresh()
    assert _mv.north_up and _mv._waiting_label.isHidden()

    # scene <-> world: exact round trip, and scene = world + t (non-vacuous)
    for _p in ((0.0, 0.0), (13.5, -7.25), (-40.0, 60.0)):
        _sp = _mv._to_scene(*_p)
        assert abs(_sp[0] - (_p[0] + geo.fit.tx)) < 1e-9
        assert abs(_sp[1] - (_p[1] + geo.fit.ty)) < 1e-9
        _rt = _mv._to_world(*_sp)
        assert math.hypot(_rt[0] - _p[0], _rt[1] - _p[1]) < 1e-9, (_p, _rt)

    # 1. publish path: what reaches CommandCenter must be WORLD
    _mv.set_mode(MapMode.MANUAL_TARGET)
    _pub.clear()
    _scene = _click(_px, _py)
    assert _pub, "manual target click published nothing"
    _sent = _pub[-1]
    _back = _mv._to_scene(*_sent)
    assert math.hypot(_back[0] - _scene.x(), _back[1] - _scene.y()) < 1e-6, (_sent, _scene)
    # non-vacuous: publishing the scene point verbatim is the bug being guarded
    assert math.hypot(_sent[0] - _scene.x(), _sent[1] - _scene.y()) > 1.0, _sent
    assert w.store.mission.manual_target == _sent
    _mk = _mv.manual_marker.scene_pos()   # crosshair stays where it was clicked
    assert math.hypot(_mk[0] - _scene.x(), _mk[1] - _scene.y()) < 1e-6, (_mk, _scene)
    _mv._refresh_manual_target()          # and survives a repaint tick
    _mk = _mv.manual_marker.scene_pos()
    assert math.hypot(_mk[0] - _scene.x(), _mk[1] - _scene.y()) < 1e-6, _mk

    # 2. inspector: world read-out, GPS from the world point, distance to robot
    _r = w.store.robot
    _rs = _mv._to_scene(_r.x, _r.y)
    assert math.hypot(_rs[0] - _r.x, _rs[1] - _r.y) > 1.0, "fit too weak to be a test"
    _seen.clear()
    _mv._inspect_point(QPointF(*_rs))
    _lat, _lon = geo.fit.world_to_latlon(_r.x, _r.y)
    assert f"world ({_r.x:+.2f}, {_r.y:+.2f}) m" in _seen[-1], _seen[-1]
    assert f"GPS {_lat:.6f}°, {_lon:.6f}°" in _seen[-1], _seen[-1]
    assert "robot ↔ point 0.00 m" in _seen[-1], _seen[-1]

    # 3. measure: distance is rigid-invariant, endpoints are reported in world
    _mv.set_mode(MapMode.MEASURE)
    _a, _b = QPointF(4.0, -2.0), QPointF(19.0, 11.5)
    _seen.clear()
    _mv._handle_measure_click(_a)
    _mv._handle_measure_click(_b)
    _d = math.hypot(_b.x() - _a.x(), _b.y() - _a.y())
    _aw = _mv._to_world(_a.x(), _a.y())
    _bw = _mv._to_world(_b.x(), _b.y())
    assert abs(math.hypot(_bw[0] - _aw[0], _bw[1] - _aw[1]) - _d) < 1e-9
    _want = (f"measure: {_d:.2f} m   |   A ({_aw[0]:+.2f}, {_aw[1]:+.2f})"
             f"   B ({_bw[0]:+.2f}, {_bw[1]:+.2f})")
    _scene_text = (f"measure: {_d:.2f} m   |   A ({_a.x():+.2f}, {_a.y():+.2f})"
                   f"   B ({_b.x():+.2f}, {_b.y():+.2f})")
    assert _seen[-1] == _want, (_seen[-1], _want)
    assert _want != _scene_text        # non-vacuous
    _mv.set_mode(MapMode.NORMAL)

    # 4. center_on_robot: world in, scene out (the mirror-image defect)
    _mv.center_on_robot()
    assert _mv._did_initial_center
    _ctr = _mv.mapToScene(_mv.viewport().rect().center())
    _tol = 2.0 / max(_mv._px_per_m(), 1e-9)   # centerOn scrolls to whole pixels
    assert math.hypot(_ctr.x() - _rs[0], _ctr.y() - _rs[1]) <= _tol, (_ctr, _rs)

    # 5. glyph heading: compass first, else the ABSOLUTE ENU odom yaw, with
    # no frame correction in between (the old theta machinery is gone).
    _saved_ch = w.store.robot.compass_heading
    w.store.robot.compass_heading = None
    assert abs(_mv._scene_heading(0.0) - w.store.robot.yaw) < 1e-12
    w.store.robot.compass_heading = 1.234
    assert abs(_mv._scene_heading(0.0) - 1.234) < 1e-12
    w.store.robot.compass_heading = _saved_ch
finally:
    w.commands.publish_manual_target = _saved_publish
    _mv.point_inspected.disconnect(_seen.append)
    _mv.set_mode(MapMode.NORMAL)
    w.store.mission.simulation = False
    w.store.mission.gps_simulated = False
    w.store.geo = _saved_geo
    _mv.refresh()
print("map frame ok")

# --- N2: [0.0, 0.0] on /blueboat/manual_target is a handover sentinel ---
# master_control reads it as "resume the original mission", never as a
# coordinate, so a genuine origin click is nudged and only Continue Original
# Mission may emit the literal pair. The recorder replaces the bridge NODE
# rather than CommandCenter, because publish_manual_target and
# resume_original_mission reach it by independent paths - and because that
# makes this block behave identically with and without a sourced ROS.
class _RecordingNode:
    def __init__(self):
        self.targets: list[tuple[float, float]] = []

    def publish_manual_target(self, x: float, y: float) -> None:
        self.targets.append((float(x), float(y)))


_sent_node = _RecordingNode()
_saved_node = w.ros.node
w.ros.node = _sent_node          # restored below: w.close() destroys the node
try:
    w._on_target_clicked(0.0, 0.0)
    assert _sent_node.targets[-1] == (1e-3, 0.0), _sent_node.targets[-1]
    assert w.store.mission.manual_target == (1e-3, 0.0)

    w._on_target_clicked(8.0, -3.0)
    assert _sent_node.targets[-1] == (8.0, -3.0)

    # One-shot arming: disarming restores NORMAL, publishes nothing, and
    # leaves the target set. Clearing belongs to Continue Original Mission.
    _n_before = len(_sent_node.targets)
    w._on_manual_mode(True)
    w._on_manual_mode(False)
    assert len(_sent_node.targets) == _n_before, "arming must publish nothing"
    assert w.store.mission.manual_target == (8.0, -3.0)

    # No click path has produced the literal sentinel...
    assert (0.0, 0.0) not in _sent_node.targets
    # ...and Continue Original Mission is the one thing that does.
    w._on_continue_mission()
    assert _sent_node.targets[-1] == (0.0, 0.0)
    assert _sent_node.targets.count((0.0, 0.0)) == 1
    assert w.store.mission.manual_target is None
finally:
    w.ros.node = _saved_node
print("sentinel ok")

# --- Per-launch georeference reset (+ sim-GPS mission flag lifecycle) ---
# Every launch restarts the robot side with a NEW world origin, so the
# anchor must start fresh — inheriting the previous run's pair window would
# blend two frames (the latent stale-anchor bug).
from mcs.core.geo import GeoReferencer as _GR
from mcs.ros.launch_manager import LaunchParameters as _LP

for _round in range(2):
    _t4 = time.monotonic() + 1000.0 + 100.0 * _round
    for _i in range(cfg.geo.min_pairs + 1):
        _t4 += 0.2
        w.bus.odom_received.emit(_t4, [1.0 * _i, 0.5 * _i, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0])
        w.bus.gps_received.emit(
            _t4, *local_en_to_latlon(1.0 * _i, 0.5 * _i, 43.3, 5.7))
    assert w.store.geo.is_valid, "setup: anchor should be valid pre-launch"
    w._on_mission_launched(_LP(simulation=True, controller_type="MPC",
                               trajectory="circle"))
    assert w.store.geo.fit is None, "launch must reset the georeference"
    assert isinstance(w.store.geo, _GR)
    assert not w.store.mission.gps_simulated
w.store.mission.gps_simulated = True
w._on_launch_state("idle")
assert not w.store.mission.gps_simulated
assert not w.store.mission.launch_running
print("georef reset ok")

# --- The Pattern Designer never adopts the station's live georeference ---
# Regression: _active_fit() used to prefer store.geo.fit whenever the app
# held a GPS lock, silently replacing the mission's own anchor with the
# boat's launch point. Everything geographic then translated by the distance
# between the two (the "every path is shifted" symptom), and saving re-wrote
# geo_anchor from it, corrupting the file on disk. The window is built here
# against a store with a VALID live fit — the exact condition that used to
# break it.
from mcs.designer.designer_window import DesignerWindow

_dsn_win_dir = Path(tempfile.mkdtemp())
# A live fit anchored far from the design site: 43.1/5.9 vs 33.66/130.65 is
# most of the planet, so a leak could not pass as rounding.
w.store.reset_georeference()
_t_live = time.monotonic()
for i in range(20):
    _tm = _t_live + i * 0.05
    w.bus.odom_received.emit(_tm, [0.0, 0.0, 0, 0, 0, 0.0], [0, 0, 0, 0, 0, 0])
    w.bus.gps_received.emit(_tm, *local_en_to_latlon(0.0, 0.0, 43.1, 5.9))
app.processEvents()
assert w.store.geo.is_valid, "the regression condition itself must hold"

_dw = DesignerWindow(cfg, w.store)
_dw._dir = _dsn_win_dir   # keep the real library out of the way
try:
    # 1. A fresh window is un-anchored despite the valid live fit.
    assert _dw._manual_fit is None
    assert _dw._active_fit() is None
    assert "none" in _dw._anchor_label.text()
    # ... so a non-GPS mission still reaches the Align-to-Start offer, whose
    # whole guard is `_active_fit() is None`.
    assert not _dw._sat_box.isEnabled()

    # 2. An explicit anchor is the one in force, and stays in force.
    _dw._set_anchor(_GeoFit(tx=0.0, ty=0.0, lat0=33.660196,
                                lon0=130.657780, rms_m=0.0, n_pairs=0),
                    "typed")
    _fit = _dw._active_fit()
    assert (_fit.lat0, _fit.lon0) == (33.660196, 130.657780), _fit
    assert "33.660196" in _dw._anchor_label.text()
    # More live fixes must not move it (the old code re-latched every 250 ms
    # AND crept with each refit of the rolling window).
    for i in range(20):
        _tm = time.monotonic() + i * 0.05
        w.bus.odom_received.emit(_tm, [5.0, 5.0, 0, 0, 0, 0.0], [0]*6)
        w.bus.gps_received.emit(_tm, *local_en_to_latlon(5.0, 5.0, 43.1, 5.9))
    app.processEvents()
    assert _dw._active_fit().lat0 == 33.660196

    # 3. Saving writes THAT anchor, not a live-derived one.
    _dw.model.add_waypoint(0.0, 0.0)
    _dw.model.add_waypoint(20.0, 0.0)
    _dw._write("anchor_test")
    _saved = _yaml.safe_load(
        io_yaml.runtime_path(_dsn_win_dir, "anchor_test").read_text())
    assert _saved["geo_anchor"]["lat0"] == 33.660196, _saved["geo_anchor"]
    assert _saved["geo_anchor"]["lon0"] == 130.657780

    # 4. The anchor is a property of the mission: New clears it, so the next
    # mission cannot inherit an origin it was never drawn at.
    _dw._dirty = False
    _dw._file_new()
    assert _dw._manual_fit is None and _dw._active_fit() is None

    # 5. The robot snapshot is one-off and explicit. It reads the CURRENT fix.
    _dw._set_gps_origin_from_robot()
    _rf = _dw._active_fit()
    assert _rf is not None and "robot" in _dw._anchor_label.text()
    assert abs(_rf.lat0 - w.store.robot.lat) < 1e-9
    assert abs(_rf.lon0 - w.store.robot.lon) < 1e-9
    # ... and it does not track the boat afterwards.
    _lat_before = _rf.lat0
    for i in range(20):
        _tm = time.monotonic() + i * 0.05
        w.bus.gps_received.emit(_tm, *local_en_to_latlon(500.0, 500.0, 43.1, 5.9))
    app.processEvents()
    assert _dw._active_fit().lat0 == _lat_before
    # With no fix at all it refuses rather than anchoring on nothing — and
    # says so, instead of failing silently. The modal is stubbed because an
    # offscreen QMessageBox still blocks on input.
    from PySide6.QtWidgets import QMessageBox as _QMB

    _warned = []
    _real_warning = _QMB.warning
    _QMB.warning = staticmethod(lambda *a, **k: _warned.append(a))
    try:
        _no_gps = DesignerWindow(cfg, None)
        try:
            _no_gps._set_gps_origin_from_robot()
            assert _no_gps._active_fit() is None
            assert len(_warned) == 1, "the refusal must be reported"
        finally:
            _no_gps._dirty = False
            _no_gps.close()
    finally:
        _QMB.warning = _real_warning

    # 6. No live overlay is drawn in the designer at all.
    assert not hasattr(_dw.map, "robot_item")
    assert not hasattr(_dw.map, "pinger_marker")
    assert not hasattr(_dw.map, "refresh_overlays")
    assert not hasattr(_dw, "_refresh_overlays")
finally:
    _dw._dirty = False
    _dw.close()
    shutil.rmtree(_dsn_win_dir, ignore_errors=True)
print("designer anchor ok")

# --- N1 / root CM-15: 'default' is published and confirmed BEFORE any kill ---
# Terminating the launch first can leave the motors in override. The sequence
# is asserted by ORDER, not by completion: one shared event log is written by
# both the publish stub and the terminate stub. Own bus + own stubs, so this
# block is independent of the live window and of whether rclpy is present.
from mcs.core.signals import SignalBus
from mcs.ros.command_center import CommandCenter

_es_events: list[tuple] = []


class _StubNode:
    matched = 1

    def input_str_subscriber_count(self) -> int:
        return self.matched

    def publish_input_str(self, command: str) -> None:
        _es_events.append(("publish", command))


class _StubRos:
    def __init__(self, node):
        self.node = node


class _StubLauncher:
    def stop(self) -> None:
        _es_events.append(("terminate",))


def _pump(predicate, timeout_s: float) -> bool:
    """Advance the Qt event loop until *predicate* holds (QTimer-driven)."""
    _end = time.monotonic() + timeout_s
    while time.monotonic() < _end and not predicate():
        app.processEvents()
        time.sleep(0.005)
    app.processEvents()
    return predicate()


_es_cfg = AppConfig()
_es_cfg.estop_confirm_timeout_s = 0.30
_es_cfg.estop_flush_delay_s = 0.05
_es_node = _StubNode()
_es_ros = _StubRos(_es_node)
_es_bus = SignalBus()
_es_done: list[int] = []
_es_states: list[str] = []
_es_bus.shutdown_sequence_finished.connect(lambda: _es_done.append(1))
_es_bus.estop_state_changed.connect(_es_states.append)
_es_cc = CommandCenter(_es_cfg, _es_bus, _es_ros, _StubLauncher())

# --- E-STOP: 'stop', an ack, and NOTHING else -----------------------------
# The panic button cuts the motors. It must not change the parameter mode and
# must not end the mission: those are two further, separate operator actions.
_es_events.clear(); _es_states.clear()
_es_cc.emergency_stop()
assert _es_events == [("publish", "stop")], _es_events
assert _es_states == ["estop"], _es_states
# controller_ready going False is robot_interface latching the stop.
_es_bus.controller_ready_received.emit(0.0, True)          # not an ack
assert _es_states == ["estop"], _es_states
_es_bus.controller_ready_received.emit(0.0, False)
assert _es_states == ["estop", "estop-confirmed"], _es_states
assert _es_events == [("publish", "stop")], _es_events     # nothing terminated
assert not _es_done, "E-STOP must not run the shutdown sequence"

# No ack ever: the operator is told, and still nothing is killed.
_es_events.clear(); _es_states.clear()
_es_cc.emergency_stop()
assert _pump(lambda: "estop-timeout" in _es_states, 3.0), _es_states
assert _es_events == [("publish", "stop")], _es_events

# The Default/Override label must not move on an E-STOP: 'stop' is not a mode.
_before_toggle = _es_cc.next_mode_command
_es_cc.emergency_stop()
_es_bus.controller_ready_received.emit(0.0, False)
assert _es_cc.next_mode_command == _before_toggle

# --- E-STOP + Stop Override: 'stop' THEN 'default', still no terminate ----
_es_events.clear(); _es_states.clear(); _es_done.clear()
_es_cc.stop_override()
assert _es_events == [("publish", "stop"), ("publish", "default")], _es_events
_es_bus.param_mode_received.emit(0.0, "override")          # wrong mode
assert ("terminate",) not in _es_events, _es_events
_es_bus.param_mode_received.emit(0.0, "default")           # the real transition
assert _es_events == [("publish", "stop"), ("publish", "default")], _es_events
assert len(_es_done) == 1
assert _es_states[-1] == "idle", _es_states   # the status label clears itself

# --- Only Stop Mission and App Exit terminate -----------------------------
for _door in (_es_cc.safe_stop_mission, _es_cc.safe_app_exit):
    _es_events.clear(); _es_done.clear()
    _es_bus.param_mode_received.emit(0.0, "override")      # leave a mode to move from
    _door()
    assert _es_events == [("publish", "default")], (_door, _es_events)
    _es_bus.param_mode_received.emit(0.0, "default")
    assert _es_events == [("publish", "default"), ("terminate",)], _door
    assert len(_es_done) == 1

# --- param_set's 1 Hz heartbeat must not impersonate an acknowledgement ---
# A repeat of a mode the boat was ALREADY in proves nothing about this command.
# The sequence therefore requires a transition, and reports the already-safe
# case as its own confirmation level instead of pretending an echo arrived.
_es_events.clear(); _es_done.clear(); _es_states.clear()
_es_bus.param_mode_received.emit(0.0, "default")           # heartbeat, pre-publish
_es_cc.safe_stop_mission()
assert _es_states[:2] == ["publishing", "already-default"], _es_states
assert _es_events == [("publish", "default")], _es_events  # not killed yet
assert _pump(lambda: bool(_es_done), 3.0), "already-default path never finished"
assert _es_events == [("publish", "default"), ("terminate",)], _es_events

# Boat in override, heartbeat repeating override: no false confirmation, and
# the timeout path still republishes ONCE at T/2 and still terminates, so a
# silent param_set cannot wedge the application.
_es_events.clear(); _es_done.clear()
_es_bus.param_mode_received.emit(0.0, "override")
_es_cc.safe_stop_mission()
_es_bus.param_mode_received.emit(0.0, "override")          # heartbeat, mid-flight
assert _es_events == [("publish", "default")], _es_events
assert _pump(lambda: bool(_es_done), 3.0), "timeout path never finished"
assert _es_events == [("publish", "default"), ("publish", "default"),
                      ("terminate",)], _es_events

# --- A stronger request supersedes a weaker one in flight -----------------
# Stop Override is confirming when the operator decides to end the mission:
# the launch must still be terminated, not silently swallowed.
_es_events.clear(); _es_done.clear()
_es_bus.param_mode_received.emit(0.0, "override")
_es_cc.safe_shutdown(terminate_nodes=False, reason="test-weak")
_es_cc.safe_shutdown(terminate_nodes=True, reason="test-strong")
_es_bus.param_mode_received.emit(0.0, "default")
assert ("terminate",) in _es_events, _es_events
# ...and never the other way round.
_es_events.clear(); _es_done.clear()
_es_bus.param_mode_received.emit(0.0, "override")
_es_cc.safe_shutdown(terminate_nodes=True, reason="test-strong")
_es_cc.safe_shutdown(terminate_nodes=False, reason="test-weak")
_es_bus.param_mode_received.emit(0.0, "default")
assert ("terminate",) in _es_events, _es_events

# --- A warned-about empty graph does not skip the publish or the ordering --
_es_node.matched = 0
_es_events.clear(); _es_done.clear()
_es_bus.param_mode_received.emit(0.0, "override")
_es_cc.safe_shutdown(terminate_nodes=True, reason="test-unmatched")
assert _es_events == [("publish", "default")], _es_events
assert _pump(lambda: bool(_es_done), 3.0)
assert _es_events[-1] == ("terminate",)
_es_node.matched = 1

# --- Simulation: no robot_interface, no param_set, so no ack can arrive ----
_es_cc.set_simulation_mode(True)
_es_events.clear(); _es_states.clear(); _es_done.clear()
_es_cc.emergency_stop()
assert _es_events == [("publish", "stop")], _es_events
assert _es_states == ["estop-sim"], _es_states
_es_events.clear(); _es_states.clear(); _es_done.clear()
_es_cc.safe_stop_mission()
assert _pump(lambda: bool(_es_done), 3.0)
assert _es_events == [("publish", "default"), ("terminate",)], _es_events
_es_cc.set_simulation_mode(False)

# --- Degraded mode (no ROS): nothing published, launch still torn down -----
# Application exit must not be able to hang on a missing ROS layer.
_es_ros.node = None
_es_events.clear(); _es_done.clear()
_es_cc.safe_shutdown(terminate_nodes=True, reason="test-degraded")
assert _es_events == [("terminate",)], _es_events
assert len(_es_done) == 1
_es_events.clear()
_es_cc.emergency_stop()
assert _es_events == [], _es_events
_es_ros.node = _es_node
print("safe shutdown ok")

# --- Path preview: one failed /path_request must never be terminal ---
# The preview used to have exactly one shot: a failure (or a request left
# hanging by a dying path_generation) meant no mission path for the rest of
# the app session. Asserted here: the tick issues the pending request, a
# failure re-arms it (bounded, spaced), retries stop when exhausted, a
# deferred-GPS launch leaves the preview to the deployment poll (the
# trajectory argument points at a file that was just unlinked), and mission
# end cancels whatever the bridge still holds.


class _PathNode:
    def __init__(self):
        self.requests: list[tuple[float, float]] = []
        self.cancels = 0
        self.sim_gps_calls: list = []      # SimGpsModel (arm) / None (disarm)

    def request_mission_path(self, total_time: float, dt: float) -> None:
        self.requests.append((total_time, dt))

    def cancel_mission_path_request(self) -> None:
        self.cancels += 1

    def set_sim_gps(self, model) -> None:  # arm/disarm on every launch
        self.sim_gps_calls.append(model)


_pv_node = _PathNode()
_pv_saved_node = w.ros.node
_pv_saved_delay = cfg.launch.path_request_retry_delay_s
w.ros.node = _pv_node
cfg.launch.path_request_retry_delay_s = 0.02
try:
    w._on_mission_launched(_LP(simulation=True, controller_type="MPC",
                               trajectory="circle"))
    assert w._pending_preview_trajectory == "circle"
    assert w.store.map_frame_ready()      # non-GPS sim: identity anchor
    w._on_tick()
    assert w._pending_preview_trajectory is None
    assert len(_pv_node.requests) == 1, _pv_node.requests
    assert w._last_preview_trajectory == "circle"

    # A failure re-arms the pending preview; a tick then re-requests. (The
    # window's own 10 Hz tick also runs under _pump, so assert on the
    # request count, which is tick-source-agnostic.)
    w.bus.mission_path_failed.emit("synthetic failure")
    assert w._preview_retries_left == cfg.launch.path_request_max_retries - 1
    assert _pump(lambda: len(_pv_node.requests) >= 2, 2.0), \
        "failure did not re-arm the preview"
    w._on_tick()
    assert len(_pv_node.requests) == 2, _pv_node.requests

    # Bounded: with no retries left a failure must NOT re-arm/re-request.
    w._preview_retries_left = 0
    w.bus.mission_path_failed.emit("synthetic failure")
    assert not _pump(lambda: len(_pv_node.requests) > 2, 0.2), _pv_node.requests
    assert w._pending_preview_trajectory is None

    # Mission end drops the bridge's pending/in-flight request and the
    # retry target — a dead run's request cannot poison the next mission.
    w._on_launch_state("idle")
    assert _pv_node.cancels == 1
    assert w._last_preview_trajectory is None
    assert w._preview_retries_left == 0

    # Deferred-GPS launch: params.trajectory names the DEPLOYED file, which
    # the launch just unlinked — the tick path must not preview it; the
    # deployment poll owns the request once the file exists.
    _pv_dir = Path(tempfile.mkdtemp(prefix="mcs_smoke_pv_"))
    try:
        _pv_src = _pv_dir / "design.yaml"
        _pv_src.write_text("format: blueboat_trajectory/1\n")
        _pv_dst = _pv_dir / ".deployed" / "design.yaml"
        _pv_dst.parent.mkdir()
        w._on_mission_launched(_LP(
            simulation=False, controller_type="LoS",
            trajectory=f"from_yaml:{_pv_dst}",
            gps_anchored_source=str(_pv_src),
            gps_deployed_target=str(_pv_dst)))
        assert w._pending_preview_trajectory is None
        _n = len(_pv_node.requests)
        w._on_tick()
        assert len(_pv_node.requests) == _n, "deferred launch must not request"
        w._on_launch_state("idle")
    finally:
        shutil.rmtree(_pv_dir, ignore_errors=True)

    # Personalized-world launch: the simulator's mavros shim owns the GPS
    # topic, so the station must NOT arm its own SimGps (a second publisher
    # with an arbitrary origin would deploy the path into the wrong frame
    # relative to the world geometry) — while the deferred-deploy poll must
    # still arm, because the shim's fixes are what anchor the frame.
    _pw_dir = Path(tempfile.mkdtemp(prefix="mcs_smoke_pw_"))
    try:
        _pw_src = _pw_dir / "anchored.yaml"
        _pw_src.write_text(_yaml.safe_dump(
            {"format": io_yaml.FORMAT, "speed": 0.5, "loop": False,
             "geo_anchor": {"lat0": 43.1, "lon0": 5.9, "theta_deg": 0.0},
             "points": [[0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 0.0, 0.0]]}))
        _pw_dst = _pw_dir / ".deployed" / "anchored.yaml"
        _pw_dst.parent.mkdir()
        # Empty-Gazebo anchored sim run: the station itself arms a model...
        w._on_mission_launched(_LP(
            simulation=True, gps_simulated=True, controller_type="PID",
            trajectory=f"from_yaml:{_pw_dst}",
            gps_anchored_source=str(_pw_src),
            gps_deployed_target=str(_pw_dst), spawn_yaw_rad=0.5))
        assert _pv_node.sim_gps_calls[-1] is not None, \
            "empty-Gazebo anchored sim run must arm SimGps"
        w._on_launch_state("idle")
        # ...a personalized-world run must NOT.
        w._on_mission_launched(_LP(
            simulation=True, gps_simulated=True, controller_type="PID",
            world_dir="/some/world",
            trajectory=f"from_yaml:{_pw_dst}",
            gps_anchored_source=str(_pw_src),
            gps_deployed_target=str(_pw_dst)))
        assert _pv_node.sim_gps_calls[-1] is None, \
            "world mode must leave SimGps disarmed"
        assert w._gps_timer is not None, \
            "world mode must still arm the deferred deploy"
        assert w._pending_preview_trajectory is None
        w._on_launch_state("idle")
    finally:
        shutil.rmtree(_pw_dir, ignore_errors=True)
finally:
    w.ros.node = _pv_saved_node
    cfg.launch.path_request_retry_delay_s = _pv_saved_delay
    w._stop_gps_deployment()

# Bridge-side request lifecycle, exercised on the REAL _poll_path_future code
# with the ROS machinery stubbed out (env-independent: no rclpy needed).
import threading as _threading
from types import SimpleNamespace as _NS

from mcs.ros.bridge_node import BridgeNode as _BridgeNode


class _FakeFuture:
    def __init__(self, result=None):
        self._result = result
        self.cancelled = False

    def done(self) -> bool:
        return self._result is not None

    def cancel(self) -> None:
        self.cancelled = True

    def result(self):
        return self._result


_bb = _BridgeNode.__new__(_BridgeNode)   # path-service state only, no super()
_bb._cfg = cfg
_bb._bus = SignalBus()
_bb._path_client = _NS(service_is_ready=lambda: False)
_bb._path_pending_lock = _threading.Lock()
_bb._path_request_args = None
_bb._path_cancel = False
_bb._path_future = None
_bb._path_issued_t = 0.0
_bb_fails: list[str] = []
_bb_paths: list = []
_bb._bus.mission_path_failed.connect(_bb_fails.append)
_bb._bus.mission_path_received.connect(_bb_paths.append)

# (1) A hung in-flight call is dropped at the deadline — it no longer blocks
# every later request for the rest of the session.
_bb._path_future = _f = _FakeFuture()
_bb._path_issued_t = time.monotonic() - cfg.launch.path_request_timeout_s - 1.0
_bb._poll_path_future()
app.processEvents()
assert _bb._path_future is None and _f.cancelled
assert _bb_fails and "no reply" in _bb_fails[-1], _bb_fails

# (2) An empty path is a FAILURE, not a silent blank map.
_bb._path_future = _FakeFuture(result=_NS(path=_NS(poses=[])))
_bb._poll_path_future()
app.processEvents()
assert _bb._path_future is None
assert "empty" in _bb_fails[-1], _bb_fails

# (3) A real reply still comes through as poses.
_pose = _NS(pose=_NS(position=_NS(x=2.0, y=-1.0),
                     orientation=_NS(w=1.0, x=0.0, y=0.0, z=0.0)))
_bb._path_future = _FakeFuture(result=_NS(path=_NS(poses=[_pose])))
_bb._poll_path_future()
app.processEvents()
assert _bb_paths and _bb_paths[-1][0][:2] == (2.0, -1.0), _bb_paths

# (4) Cancel drops both the queued args and the in-flight future, and a
# completed-but-cancelled reply is NOT delivered (it belongs to a dead run).
_bb.request_mission_path(120.0, 0.5)
_bb._path_future = _f2 = _FakeFuture(result=_NS(path=_NS(poses=[_pose])))
_n_paths = len(_bb_paths)
_bb.cancel_mission_path_request()
_bb._poll_path_future()
app.processEvents()
assert _bb._path_future is None and _f2.cancelled
assert _bb._path_request_args is None
assert len(_bb_paths) == _n_paths
print("path preview ok")

# --- Crashed launch returns the state machine to 'idle' ---
# A launch that died on its own used to leave the manager at
# 'starting'/'running' forever: the Launch button stayed disabled and the
# dead run's path was never cleared. The exit watch (started by start(),
# reused by stop()) finalises it from the GUI thread.
import os as _os
import signal as _signal
import subprocess as _subprocess

from mcs.ros.launch_manager import LaunchManager as _LM

_lm_bus = SignalBus()
_lm_states: list[str] = []
_lm_bus.launch_state_changed.connect(_lm_states.append)
_lm = _LM(cfg, _lm_bus)
_lm._proc = _subprocess.Popen(["sleep", "0.15"])
_lm._set_state("running")
_lm._watch_exit()
_lm._watch_exit()                       # second call: no duplicate chain
assert _lm.state == "running"           # process still alive
assert _pump(lambda: _lm.state == "idle", 5.0), "crash never finalised"
assert _lm._proc is None
assert not _lm._exit_poll_active
assert _lm_states[-1] == "idle"

# --- The SIGTERM/SIGKILL escalation must not outlive its own launch ---
# stop() arms two delayed escalations (8 s and 12 s by default). They used to
# read self._proc / self._sea_proc at FIRE time and were never cancelled, so a
# relaunch inside that window killpg'd a recycled group and SIGTERM'd the NEW
# sea companion. Now each escalation is bound to the process it was armed for
# and is disarmed as soon as the tree exits.
#
# The children below IGNORE SIGINT so they outlive stop()'s first signal: that
# is what makes the escalation observably armed instead of racing the child's
# death (a child that dies on SIGINT is finalised synchronously, which correctly
# disarms the escalation and leaves nothing to assert on).
# start_new_session, exactly as LaunchManager.start() does — stop() signals the
# whole process group, and a child sharing this script's group would SIGINT the
# smoke test itself.
_DEAF = [sys.executable, "-u", "-c",
         ("import signal, sys, time; "
          "signal.signal(signal.SIGINT, signal.SIG_IGN); "
          "print('ready'); sys.stdout.flush(); time.sleep(30)")]


def _deaf_child():
    """A child in its own process group that is already ignoring SIGINT.

    The handshake is load-bearing: without it stop()'s SIGINT can arrive during
    the interpreter's startup, before the handler is installed, and the child
    dies on the default action instead of surviving to be escalated against.
    """
    proc = _subprocess.Popen(_DEAF, start_new_session=True,
                             stdout=_subprocess.PIPE)
    assert proc.stdout is not None
    assert proc.stdout.readline().strip() == b"ready"
    return proc


_lm._proc = _deaf_child()
_lm._set_state("running")
_lm.stop()
assert len(_lm._escalation_timers) == 2, _lm._escalation_timers
assert all(t.isActive() for t in _lm._escalation_timers)

# The tree exits (here by force): the pending escalation must be disarmed, not
# left to fire into whatever runs next.
_os.killpg(_os.getpgid(_lm._proc.pid), _signal.SIGKILL)
assert _pump(lambda: _lm.state == "idle", 6.0), "stop never finalised"
assert _lm._escalation_timers == [], "escalation still armed after the tree exited"

# And an escalation that does fire is bound to the launch it was armed for: a
# relaunch inside the window must be untouched.
_doomed = _deaf_child()
_lm._proc = _doomed
_lm._set_state("running")
_lm.stop()
assert len(_lm._escalation_timers) == 2
_pending = list(_lm._escalation_timers)
_os.killpg(_os.getpgid(_doomed.pid), _signal.SIGKILL); _doomed.wait()

_survivor = _deaf_child()                        # the NEW launch
_lm._proc = _survivor
_lm._sea_proc = None
for _t in _pending:
    _t.stop()
    _t.timeout.emit()                       # fire as if the delay had elapsed
assert _survivor.poll() is None, "a dead launch's escalation hit the new one"
_os.killpg(_os.getpgid(_survivor.pid), _signal.SIGKILL)
_survivor.wait()
_lm._proc = None
_lm._cancel_escalation()
print("launch crash ok")

# --- Floating SEA STATE box (mirror of the mission-stats box) ---
# The sea read-out is an overlay on the map, not a left-panel section: it
# hides itself outside a simulation run, sits at the map's top-LEFT corner
# while the stats box holds the top-right, and grows downwards as the
# wrapped strings get longer (the map cannot widen to fit them).
assert w.sea_box.parent() is w.map_view
assert not hasattr(w.left_panel, "sea_grid"), "sea read-out duplicated in the left panel"
w.store.mission.simulation = False
w.store.mission.launch_running = False
w.store.sea = None
w._on_tick(); app.processEvents()
assert w.sea_box.isHidden()

w.store.mission.simulation = True
w.store.mission.launch_running = True
w._on_tick(); app.processEvents()
assert not w.sea_box.isHidden()
assert (w.sea_box.pos().x(), w.sea_box.pos().y()) == (8, 8)
# The exact mirror of the stats box: same top margin, opposite edge.
assert (w.stats_box.pos().x(), w.stats_box.pos().y()) == (
    w.map_view.width() - w.stats_box.width() - 8, 8)
assert "waiting for the simulator" in w.sea_box._values["Status"].text()

w.store.sea = parse_readback({"t_sim": 0.0,
                              "current": {"label": "Strong", "mean_speed_mps": 0.5,
                                          "from_deg": 90.0, "instant": {"speed_mps": 0.47}},
                              "waves": {"label": "Slight", "hs_m": 0.35, "tp_s": 2.5,
                                        "from_deg": 45.0, "status": "active",
                                        "eta_m": 0.12},
                              "summary": "strong from E; slight from NE"},
                             received_mono=time.monotonic())
w._on_tick(); app.processEvents()
assert w.sea_box._values["Surface"].text() == "\u03b7 +0.12 m"
assert w.sea_box._values["Status"].text() == "strong from E; slight from NE"
# A longer wave string wraps to more lines; the box gets taller, the rows
# never overlap and the width stays put.
_w_fixed = w.sea_box.width()
_h_waves = w.sea_box._values["Waves"].height()
w.store.sea = parse_readback({"waves": {"label": "Slight", "hs_m": 0.35, "tp_s": 2.5,
                                        "from_deg": 200.0, "status": "active",
                                        "event": {"hs_m": 0.5, "tp_s": 4.0,
                                                  "from_deg": 10.0,
                                                  "t_remaining_s": 12.0}}},
                             received_mono=time.monotonic())
w._on_tick(); app.processEvents()
assert w.sea_box.width() == _w_fixed
assert w.sea_box._values["Waves"].height() > _h_waves   # wrapped to more lines
# Every row measures its own wrapped height, which is what keeps a grown
# row from painting over the one below it (offscreen and never shown, the
# window lays nothing out, so only the heights are checkable here).
assert all(_l.height() >= _l.fontMetrics().height()
           for _l in w.sea_box._values.values())

# The button is the live-command door and must still reach MainWindow's
# handler — which opens a modal dialog, so the click is not driven here.
w.sea_box.modify_sea_clicked.disconnect(w._on_modify_sea)   # raises if unwired
w.sea_box.modify_sea_clicked.connect(w._on_modify_sea)
assert w.sea_box.modify_sea_btn.isEnabled()
w.store.mission.simulation = False
w.store.mission.launch_running = False
w.store.sea = None
w._on_tick(); app.processEvents()
assert w.sea_box.isHidden()
print("sea box ok")

w.close()
print("window ok")
print("SMOKE TEST PASSED")
