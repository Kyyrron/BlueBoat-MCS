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

# --- Launch dialog: sim + GPS-anchored path arms the simulated-GPS flow ---
# In simulation an anchored mission takes the SAME deferred-deploy branch as
# on real water, plus gps_simulated and a random spawn heading; non-anchored
# paths change nothing.
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
    _cli = " ".join(_p.to_cli())
    assert "spawn_yaw:=" in _cli and "robot_file:=" in _cli \
        and "controller_type:=" in _cli, _cli

    _i = _dlg._trajectory.findData(str(_ld_plain))
    _dlg._trajectory.setCurrentIndex(_i)
    _p2 = _dlg.parameters()
    assert not _p2.gps_simulated and _p2.spawn_yaw_rad is None
    assert _p2.gps_anchored_source == ""
    assert _p2.trajectory == f"from_yaml:{_ld_plain}"
    assert "spawn_yaw:=" not in " ".join(_p2.to_cli())

    _dlg._mode.setCurrentText("Real robot")
    _i = _dlg._trajectory.findData(str(_ld_anch))
    _dlg._trajectory.setCurrentIndex(_i)
    _p3 = _dlg.parameters()
    assert not _p3.simulation and not _p3.gps_simulated
    assert _p3.spawn_yaw_rad is None
    assert _p3.gps_anchored_source == str(_ld_anch)   # real branch unchanged
    _dlg.deleteLater()
finally:
    shutil.rmtree(_ld_dir, ignore_errors=True)
print("launch dialog sim-gps ok")

# --- Full window in offscreen mode, synthetic telemetry ---
from mcs.gui.main_window import MainWindow

w = MainWindow(cfg)  # RosManager connects, or degrades if rclpy is absent
# Construction must succeed in both regimes: with rclpy present
# RosManager connects, without it the app degrades GUI-only. Assert the
# flag is a real bool either way — `not available or True` was a
# tautology that could never fail.
assert w.ros.available in (True, False)

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
_es_bus.shutdown_sequence_finished.connect(lambda: _es_done.append(1))
_es_cc = CommandCenter(_es_cfg, _es_bus, _es_ros, _StubLauncher())

# (1) echo path: publish first, terminate only after the acknowledgement.
_es_events.clear(); _es_done.clear()
_es_cc.safe_shutdown(terminate_nodes=True, reason="test-echo")
assert _es_events == [("publish", "default")], _es_events   # nothing killed yet
assert not _es_done
_es_bus.param_mode_received.emit(0.0, "override")           # wrong mode
assert ("terminate",) not in _es_events, _es_events
_es_bus.param_mode_received.emit(0.0, "default")            # the real echo
assert _es_events == [("publish", "default"), ("terminate",)], _es_events
assert len(_es_done) == 1

# (2) timeout path: no echo ever arrives. The retry republishes ONCE at T/2
# and the flush still terminates - a silent param_set must not wedge exit.
_es_events.clear(); _es_done.clear()
_es_cc.safe_shutdown(terminate_nodes=True, reason="test-timeout")
assert _es_events == [("publish", "default")], _es_events
assert _pump(lambda: bool(_es_done), 3.0), "timeout path never finished"
assert _es_events == [("publish", "default"), ("publish", "default"),
                      ("terminate",)], _es_events

# (3) all three operator doors funnel through the same sequence.
for _door in (lambda: _es_cc.emergency_stop(True),
              _es_cc.safe_stop_mission,
              _es_cc.safe_app_exit):
    _es_events.clear(); _es_done.clear()
    _door()
    assert _es_events == [("publish", "default")], (_door, _es_events)
    _es_bus.param_mode_received.emit(0.0, "default")
    assert _es_events == [("publish", "default"), ("terminate",)], _door
    assert len(_es_done) == 1

# (4) E-STOP without node termination still publishes, and kills nothing.
_es_events.clear(); _es_done.clear()
_es_cc.emergency_stop(False)
_es_bus.param_mode_received.emit(0.0, "default")
assert _es_events == [("publish", "default")], _es_events
assert len(_es_done) == 1

# (5) a warned-about empty graph does not skip the publish or the ordering.
_es_node.matched = 0
_es_events.clear(); _es_done.clear()
_es_cc.safe_shutdown(terminate_nodes=True, reason="test-unmatched")
assert _es_events == [("publish", "default")], _es_events
assert _pump(lambda: bool(_es_done), 3.0)
assert _es_events[-1] == ("terminate",)
_es_node.matched = 1

# (6) degraded mode (no ROS): nothing is published, the launch is still torn
# down and the sequence still completes, so application exit cannot hang.
_es_ros.node = None
_es_events.clear(); _es_done.clear()
_es_cc.safe_shutdown(terminate_nodes=True, reason="test-degraded")
assert _es_events == [("terminate",)], _es_events
assert len(_es_done) == 1
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

    def request_mission_path(self, total_time: float, dt: float) -> None:
        self.requests.append((total_time, dt))

    def cancel_mission_path_request(self) -> None:
        self.cancels += 1

    def set_sim_gps(self, model) -> None:  # disarm call on every launch
        pass


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
print("launch crash ok")

w.close()
print("window ok")
print("SMOKE TEST PASSED")
