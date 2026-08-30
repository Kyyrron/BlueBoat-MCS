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

# --- GeoReferencer: synthetic boat moving, world = R(30deg)@EN + (5, -3) ---
geo = GeoReferencer(cfg.geo)
theta = math.radians(30); lat0, lon0 = 43.10, 5.90
from mcs.core.geo import local_en_to_latlon

t = 0.0
for i in range(300):
    t += 1.0
    east, north = 0.05 * i, 0.03 * i  # boat path in EN metres
    x = math.cos(theta) * east - math.sin(theta) * north + 5.0
    y = math.sin(theta) * east + math.cos(theta) * north - 3.0
    lat, lon = local_en_to_latlon(east, north, lat0, lon0)
    geo.add_pair(t, x, y, lat, lon)
assert geo.fit is not None, "geo fit missing"
assert geo.is_valid, f"geo rms {geo.fit.rms_m}"
lat, lon = geo.fit.world_to_latlon(5.0, -3.0)
assert abs(lat - lat0) < 1e-6 and abs(lon - lon0) < 1e-6
xx, yy = geo.fit.latlon_to_world(lat0, lon0)
assert abs(xx - 5.0) < 1e-3 and abs(yy + 3.0) < 1e-3
print(f"GeoReferencer ok (theta_hat={math.degrees(geo.fit.theta):.2f} deg, rms={geo.fit.rms_m:.3f} m)")

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

# --- N8: deploy_mission refuses a non-heading-aligned georeference ---
# A translation-only fit carries theta = 0 as a PLACEHOLDER, not a measurement.
# Deploying against it silently rotates the whole mission by the true heading
# offset, so the guard must raise rather than write a wrong file.
from mcs.core.geo import GeoFit

_dep_dir = Path(tempfile.mkdtemp())
try:
    _dep_m = MissionModel()
    _dep_m.from_dict({"name": "anchored", "speed": 0.5, "loop": False,
                      "items": []})
    for _x, _y in ((0.0, 0.0), (10.0, 0.0), (10.0, 8.0)):
        _dep_m.add_waypoint(_x, _y)
    _dep_s = sample_mission(_dep_m, cfg.designer.sample_ds_m)
    _dep_src = io_yaml.save_mission(
        _dep_dir, "anchored", _dep_m, _dep_s,
        geo_anchor={"lat0": lat0, "lon0": lon0, "theta_deg": 0.0})
    _dep_n = len(_yaml.safe_load(_dep_src.read_text())["points"])

    _dep_dst = io_yaml.deployed_path(_dep_dir, "anchored")
    assert _dep_dst == _dep_dir / ".deployed" / "anchored.yaml", _dep_dst

    # Translation-only fit -> refuse, and write nothing at all.
    _flat_fit = GeoFit(theta=0.0, tx=0.0, ty=0.0, lat0=lat0, lon0=lon0,
                       rms_m=0.0, n_pairs=1, heading_aligned=False)
    try:
        io_yaml.deploy_mission(_dep_src, _flat_fit, _dep_dst)
        raise AssertionError("deploy_mission accepted a non-aligned fit")
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

    # Heading-aligned fit -> deploy into THIS run's world frame.
    io_yaml.deploy_mission(_dep_src, geo.fit, _dep_dst)
    _dep_out = _yaml.safe_load(_dep_dst.read_text())
    assert "geo_anchor" not in _dep_out, "deployed file must not stay anchored"
    assert _dep_out["deployed_from"] == str(_dep_src)
    assert abs(_dep_out["deployed_fit_rms_m"] - round(geo.fit.rms_m, 3)) < 1e-9
    assert len(_dep_out["points"]) == _dep_n
    assert _dep_out["speed"] == _dep_m.speed and _dep_out["loop"] is False
    # Non-vacuous: the fit rotates by 30 deg and offsets by (5, -3), so the
    # deployed coordinates cannot be the design-frame ones.
    _dep_in_pts = _yaml.safe_load(_dep_src.read_text())["points"]
    assert max(math.hypot(_o[1] - _i[1], _o[2] - _i[2])
               for _i, _o in zip(_dep_in_pts, _dep_out["points"])) > 1.0
    # ...and they are exactly what the design->GPS->world chain produces.
    _anchor_fit = GeoFit(theta=0.0, tx=0.0, ty=0.0, lat0=lat0, lon0=lon0,
                         rms_m=0.0, n_pairs=0)
    for _i, _o in zip(_dep_in_pts, _dep_out["points"]):
        _wx, _wy = geo.fit.latlon_to_world(
            *_anchor_fit.world_to_latlon(_i[1], _i[2]))
        assert math.hypot(_o[1] - _wx, _o[2] - _wy) < 1e-3, (_i, _o)
finally:
    shutil.rmtree(_dep_dir, ignore_errors=True)
print("deploy guard ok")

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
    w.bus.gps_received.emit(tm, 43.1 + 1e-6 * i, 5.9 + 2e-6 * i)
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

# --- Map frame round trip: every mouse position read back out of the view ---
# The scene is the raw world frame before heading alignment and local ENU
# after, so an un-inverted input is wrong by the georeference rotation. The
# published manual target is the one that moves the boat (/blueboat/manual_target
# is a WORLD-frame topic); the inspector, the measure endpoints and the
# recentring are the same defect where it only shows in a read-out.
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
    _flat = GeoReferencer(cfg.geo)   # translation-only fit: world-up scene
    _flat.add_pair(0.0, 0.0, 0.0, 43.1, 5.9)
    assert _flat.is_valid and not _flat.heading_aligned
    _c = _mv.viewport().rect().center()
    _px, _py = _c.x() + 37, _c.y() + 23

    # -- rotated fit (theta = 30 deg, heading-aligned): the scene becomes ENU
    w.store.geo = geo
    _mv._update_scene_mode()
    assert _mv.north_up, "ENU scene expected once the fit is heading-aligned"

    # scene <-> world is an exact round trip
    for _p in ((0.0, 0.0), (13.5, -7.25), (-40.0, 60.0)):
        _rt = _mv._to_world(*_mv._to_scene(*_p))
        assert math.hypot(_rt[0] - _p[0], _rt[1] - _p[1]) < 1e-9, (_p, _rt)

    # 1. publish path: what reaches CommandCenter must be WORLD
    _mv.set_mode(MapMode.MANUAL_TARGET)
    _scene = _click(_px, _py)
    assert _pub, "manual target click published nothing"
    _sent = _pub[-1]
    _back = _mv._to_scene(*_sent)
    assert math.hypot(_back[0] - _scene.x(), _back[1] - _scene.y()) < 1e-6, (_sent, _scene)
    # non-vacuous: publishing the scene point verbatim is the bug being guarded
    assert math.hypot(_sent[0] - _scene.x(), _sent[1] - _scene.y()) > 0.1, _sent
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

    # 3b. a measurement in progress, and the inspector dot, hold bare scene
    # points: the regime switch must drop them rather than strand them.
    _mv.set_mode(MapMode.MEASURE)
    _mv._handle_measure_click(_a)
    _mv._inspect_point(QPointF(*_rs))
    assert _mv._measure_start is not None and _mv.click_marker.isVisible()
    w.store.geo = _flat
    _mv._update_scene_mode()
    assert not _mv.north_up
    assert _mv._measure_start is None and not _mv.click_marker.isVisible()
    w.store.geo = geo
    _mv._update_scene_mode()
    _mv.set_mode(MapMode.NORMAL)

    # 4. center_on_robot: world in, scene out (the mirror-image defect)
    _mv.center_on_robot()
    assert _mv._did_initial_center
    _ctr = _mv.mapToScene(_mv.viewport().rect().center())
    _tol = 2.0 / max(_mv._px_per_m(), 1e-9)   # centerOn scrolls to whole pixels
    assert math.hypot(_ctr.x() - _rs[0], _ctr.y() - _rs[1]) <= _tol, (_ctr, _rs)

    # 5. translation-only fit: the pre-alignment regime must be untouched
    w.store.geo = _flat
    _mv._update_scene_mode()
    assert not _mv.north_up
    _mv.set_mode(MapMode.MANUAL_TARGET)
    _pub.clear()
    _scene = _click(_px, _py)
    assert _pub[-1] == (_scene.x(), _scene.y()), (_pub[-1], _scene)

    # 6. the crosshair follows the scene across the switch to ENU: it is
    # placed once at click time, so only the refresh tick can move it.
    w.store.geo = geo
    _mv._update_scene_mode()
    _mv._refresh_manual_target()
    _want_mk = _mv._to_scene(*w.store.mission.manual_target)
    _mk = _mv.manual_marker.scene_pos()
    assert math.hypot(_mk[0] - _want_mk[0], _mk[1] - _want_mk[1]) < 1e-6, (_mk, _want_mk)
    assert math.hypot(_want_mk[0] - _scene.x(), _want_mk[1] - _scene.y()) > 1.0
finally:
    w.commands.publish_manual_target = _saved_publish
    _mv.point_inspected.disconnect(_seen.append)
    _mv.set_mode(MapMode.NORMAL)
    w.store.geo = _saved_geo
    _mv._update_scene_mode()
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

w.close()
print("window ok")
print("SMOKE TEST PASSED")
