# BlueBoat-MCS — map and georeferencing, Pattern Designer and trajectory files, control-stack files

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## 4. Map & georeferencing

One scene regime, GPS-frame-only (`gui/map/map_view.py`); the authoritative description is
`docs/GPS_MAP_ARCHITECTURE.md`. A change must preserve:

- The scene is **local east/north metres about the latched first GPS fix**, north-up (N5). The
  only estimated quantity is a **translation** `t` (`core/geo.py::GeoFit`, `EN = world + t`), a
  per-axis median over GPS↔odom pairs fed at GPS rate (odom fresher than 0.5 s); valid within
  ~1 s of GPS, no motion needed, no rotation estimated.
- **Nothing is drawn and manual-target clicks are refused before `store.map_frame_ready()`**
  (`geo.is_valid`, or a non-GPS, non-replay simulation, which uses the identity anchor).
- **The georeferencer is reset on every mission launch** (new control-stack world origin).
- Satellite tiles are placed **axis-aligned** from `(lat0, lon0)`.
- `MapView._to_world()` is the exact inverse of `_to_scene()`, and **every** mouse position
  read out of the view goes through it; the crosshair is re-placed from the stored world
  target every tick.
- The preview waits for `map_frame_ready()`; launch state `idle` clears `store.mission_path`
  and cancels any in-flight request, so a finished run's path never re-anchors onto the next.

---

## 5. Survey Pattern Designer & trajectory files

Full guidance in **`mcs/designer/CLAUDE.md`**. Files in `~/.config/blueboat_mcs/trajectories/`:
`<name>.yaml` (runtime `blueboat_trajectory/1`, read by `path_generation`, the World Builder
and the GCS), `<name>.meta.yaml` (editor only), `.deployed/<name>.yaml` (regenerated each
launch of a GPS-anchored mission), `.session/run.yaml` (the run session's watched file) —
never hand-edit the last two. Format contract: `docs/05_trajectory_format.md`. **The designer
reads no live telemetry.** Other state: `~/.config/blueboat_mcs/config.json`,
`tile_cache/` (disposable), `launch_cwd/` (the launch tree's working directory, so tools that
write relative paths never write into this repository).

---

## 6. Control-stack files — in the `BlueBoat-Control` submodule

Committed in and installed by `blueboat_control`; change them there and `colcon build`.
Trajectory selection needs no launch-file change: `trajectory:=from_yaml:/abs/path.yaml`.

| File (under `BlueBoat-Control/blueboat_control/`) | What the station depends on |
|---|---|
| `src/_custom_libraries/path_generation.py` | The `from_yaml` branch + a file **watcher** that reloads on change, holding a station-keeping pose until the file appears (deferred GPS deployment, run sessions). |
| `src/_custom_libraries/yaml_trajectory.py` | Loads `blueboat_trajectory/1` and evaluates at `t` (PyYAML + numpy). |
| `src/master_control.py` | World-frame `/monitoring_data` target in every branch (N3); 20 Hz loop (`control_dt`); run session (`session_mode`: idle until `start`, `configure` between runs, one `.npy` per run, `/blueboat/run_state`); arrow keys (highest-priority branch, `arrow_timeout`). |
| `src/robot_interaction/robot_interface.py` | Producer of odom / pinger / `controller_ready`; CSV logging (one CSV per run in a session, reported on `/blueboat/run_log`). |
| `src/_custom_libraries/robot_log_schema.py` | The position-CSV layouts written by `robot_interface` (real) and `simulation_interface` (Gazebo), `columns_for(use_UWgps)`; `target_*` only in the no-pinger layout. Read this, not the nodes, to learn the CSV format. |

**Behaviours the station does not compensate for.** Point-LoS arrival checking is the pinger
branch's alone: `safety_distance` is `-1.0` (off) on the real boat, `+1.0` in simulation; when
positive, `stopping_sequence` latches on arrival and the boat brakes with nothing on screen
saying why. A manual target has its own arrival state (`manual_keep_location`). Pinger-marker
lag is inherent to the source (§2); N4 only keeps it from also dragging.

**⚠ What runs is `~/ros2_ws/install/`, refreshed only by `colcon build`.** A stale install can
publish the pre-2026-08-31 hybrid odom frame (launch-relative yaw) and silently break the map
and trajectory following from non-East headings (`docs/03_ros_integration.md` §Observations).
Line numbers into `BlueBoat-Control` drift: anchor on symbol names, and rebuild before a field
session.

---
