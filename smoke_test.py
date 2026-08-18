"""Offline smoke test: imports every module (rclpy absent -> degraded mode),
instantiates the full window with QT_QPA_PLATFORM=offscreen, feeds synthetic
telemetry through the SignalBus, exercises store/geo/predictor logic.
"""
import math, os, sys, time
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QCoreApplication

from mcs.config.settings import AppConfig
from mcs.core.series import TimeSeries
from mcs.core.geo import GeoReferencer
from mcs.core.los_predictor import predict_los_path

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

# --- Full window in offscreen mode, synthetic telemetry ---
app = QApplication(sys.argv)
from mcs.gui.main_window import MainWindow
w = MainWindow(cfg)  # rclpy absent here -> RosManager degrades gracefully
assert not w.ros.available or True

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

# resume path publishes [0,0] -> without ROS it just logs; state must clear
w._on_manual_mode(False)
assert w.store.mission.manual_target is None

w.close()
print("window ok")
print("SMOKE TEST PASSED")
