# BlueBoat-MCS — Mission Control Station

Operator ground-station GUI for a BlueRobotics BlueBoat USV used in a master's
thesis on aspect-aware side-scan sonar survey. Runs on the operator laptop
(`~/ros2_ws`), like every node of the project; the boat carries only its hardware
and its own firmware, reached through the BlueBoat Base Station WiFi.

**The station supervises and commands. It performs no control computation and
duplicates no logic that already exists on a ROS topic.** Every number it shows
comes from a topic or service published by the existing `blueboat_control`
stack. When something looks wrong on screen, the first question is whether the
producing node is right, not whether the GUI should compensate.

Build system: standalone Python (no colcon, no `setup.py`). The package is run
from the repo root; ROS2 comes from the sourced workspace, never from pip.

---

## 1. Layout & internals

`mcs/` is the application package (`main.py` bootstraps; `run.py` is a thin launcher):
`config/settings.py` holds **all** topics, thresholds, gains and paths (dataclasses, JSON
overrides); `core/` and `models/` import no widgets and no rclpy (only `QObject` for signals)
— that is what makes the offline smoke test possible; `ros/` holds the rclpy thread
(`ros_manager`), `bridge_node` (**the only file that knows message types**), `launch_manager`,
`command_center` and `run_session` (the station's half of a real-robot run session, §2);
`gui/` the window, panels, `map/`, `plot/`, `dialogs/`, plus `run_panel.py` (the RUN box),
`arrow_box.py` (ARROW CONTROL) and `replay_box.py` (REPLAY, v1.1); `designer/` the Survey
Pattern Designer (its own `CLAUDE.md`). `core/arrows.py` is the Qt-free arrow-key thrust law.
`core/worlds.py` reimplements the SSS-Sim world-folder contract locally and never imports it
(CM-3). `core/replay.py` (v1.1) reads a recorded run's bag folder (`metadata.yaml` with PyYAML;
its GPS track through `rosbag2_py` only when that imports) and decodes `/replay/state`.

`ruff.toml` and `requirements-dev.txt` are gitignored: a fresh clone has no lint config and no
pinned ruff. Control-stack code lives in the **`BlueBoat-Control` submodule**, not here (§6).

### Threading model
`rclpy` spins on its own thread. `core/signals.py::SignalBus` is the **only**
crossing point: ROS callbacks emit Qt queued signals, `DataStore` slots mutate
state on the GUI thread, and widgets repaint from a single ~10 Hz tick in
`MainWindow` (`MapConfig.ui_refresh_hz`). Never touch Qt objects from a ROS
callback, and never block the ROS thread on GUI work.

`DataStore` is the single source of truth for the GUI. Widgets read it; they do
not subscribe to ROS directly.

### Degraded mode
If `rclpy` is not importable the app still starts, GUI-only (this is what makes
`smoke_test.py` possible without ROS). Keep new ROS imports lazy or guarded so
this keeps working.

### Data flow, end to end

Odometry, the canonical example:

1. `BridgeNode._on_odom` (ROS thread) converts the message to plain Python
   lists, stamps `time.monotonic()`, updates its `TopicStats` via `_mark`, and
   emits `SignalBus.odom_received`.
2. Qt queues the signal into the GUI thread; `DataStore.on_odom` updates
   `RobotState`, appends to `robot_track` / `speed_hist`, integrates travelled
   distance, feeds the `GeoReferencer`, refreshes the pinger world position.
3. Nothing repaints yet. At the next 10 Hz tick `MainWindow._on_tick` calls
   `left_panel.refresh()`, `right_panel.refresh()`, `map_view.refresh()`, which
   *pull* from the store and repaint once.

User intents flow the other way: widget → signal → `MainWindow` slot →
`CommandCenter` → `BridgeNode.publish_*` (thread-safe) → ROS graph.

### Clocks

All history timestamps are `time.monotonic()` at **reception**, one clock for
everything. Experiment-relative time (what the timeline shows) is `t − store.t0`,
`t0` being the first odometry sample. The controller's own `t` inside
`/monitoring_data` is **never** used as a clock — reception time is, which keeps
every series mutually consistent even if a node restarts.

### Repaint bounds

`TimeSeries.decimated_window` bounds every repaint: at most
`map.trajectory_max_points_drawn` (20 000) vertices per polyline and 2000 per
plot. `PolylineItem.set_points` rebuilds a `QPainterPath` per tick from the
decimated window — measured adequate at 10 Hz well past 10⁵ stored points. The
tile layer refuses to populate more than 64 tiles per viewport instead of
hammering the tile server.

### Map rendering

Constant-pixel glyphs (`ItemIgnoresTransformations`) for the boat and markers,
cosmetic pens for lines, an adaptive 1/2/5-decade metric grid painted in
`drawBackground`, and the satellite `TileLayer` at z = −100. Interaction modes
(`NORMAL` / `MANUAL_TARGET` / `MEASURE`) are a small state machine inside
`MapView`; the view emits intents (`target_clicked`, `point_inspected`) and
never publishes anything itself.

### How to add…

**A subscribed topic.** Name it in `TopicsConfig`; add thresholds to
`DiagnosticsConfig` (it then appears in the diagnostics panel automatically);
add a `Signal` to `SignalBus`; create the subscription + `_on_x` callback in
`BridgeNode` (call `self._mark(topic)` first); add a `DataStore.on_x` slot and
connect it in `MainWindow._connect_signals`. Display it from any `refresh()`.

**A published command.** Add a publisher in `BridgeNode.__init__` and a
`publish_x` wrapper (keep the `self._pub_lock` pattern and the `command_sent`
emission), expose a semantic method on `CommandCenter`, call it from the GUI.

**A map layer.** Create an item in `gui/map/map_items.py` (`_cosmetic_pen` for
constant-pixel lines, `ItemIgnoresTransformations` for constant-pixel glyphs),
add it to the scene in `MapView.__init__`, update it in `MapView.refresh()`,
register a key in `MapView.set_layer_visible`, add one entry to `_LAYERS` in
`left_panel.py`. That is the whole checklist.

**A plot.** Follow `DistancePlot`: a `QWidget` reading one `TimeSeries`, honouring
`set_time_window`, painting in `paintEvent`. Add it to a `CollapsibleSection` in
the right panel and call its `refresh()` from `RightPanel.refresh()`.

**A launch argument.** Four sites, all in `mcs/ros/launch_manager.py` plus the
dialog: `LaunchParameters.to_cli()` (the main argument list, which splices in
`params.sea_args()`), `companion_command()` (the empty-world sea companion),
`launch_target()` (which of the four launch files runs), and
`gui/dialogs/launch_dialog.py` for the widget. The replay branch (`replay_bag` set) comes first
in `to_cli()` and `launch_target()`, and `companion_command()` returns None for it.

---

## 2. ROS2 interface — the exact contract

Everything here is configured in `mcs/config/settings.py::TopicsConfig`; change
names there, never inline. Peer module for all of these is `blueboat_control`
(nodes `robot_interface.py`, `master_control.py`, `path_generation.py`,
`param_set`), plus MAVROS.

### Subscriptions

| Topic | Type | Produced by | Used for | Notes |
|---|---|---|---|---|
| `/blueboat/odom` | `nav_msgs/Odometry` | `robot_interface.py` | pose, yaw, speed, trail | **Local ENU**: position translated at that node's first callback (world origin = boat position at launch), axes East/North, yaw **absolute ENU** (0 = East, CCW+; NOT re-zeroed). Same frame kind as the simulator's Gazebo-bridged odom — only the origin differs. The pre-2026-08-31 hybrid (translated position, launch-relative yaw) is gone; see `docs/03_ros_integration.md` §Observations. |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | MAVROS (real) · **the station's own bridge node** (Gazebo run of a GPS-anchored mission: a 5 Hz timer synthesises fixes from the sim odom — world metres → lat/lon about a receiver origin placed `SimGpsConfig.offset_north_m` north of the mission's first point — received back through its own subscription, so the whole pipeline incl. diagnostics runs as on real water) | georeference, GPS read-out | BEST_EFFORT QoS. `lat==0 and lon==0` means no fix and is discarded. |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | MAVROS | **robot glyph heading (preferred)** | Degrees **clockwise from north** (0=N, 90=E). Converted to the app-wide convention (radians CCW from east) as `radians(90 - hdg)`. Fallback: the odom yaw directly (absolute ENU). Subscribed **BEST_EFFORT** — this, the GPS fix and the battery are the three sensor streams that are; every other subscription uses the default reliable depth-10 profile. |
| `/mavros/state` | `mavros_msgs/State` | MAVROS | FCU **connection** flag, one of the three inputs to the readiness count | Only subscribed when `mavros_msgs` imports; absent in simulation. `armed` and `mode` ride the same message but nothing displays them, so the store does not keep them. |
| `/mavros/battery` | `sensor_msgs/BatteryState` | MAVROS (`sys_status`) | battery charge read-out (left panel, ROBOT) | BEST_EFFORT QoS. `percentage` is a **0..1 fraction**, not a percent — MAVROS divides the MAVLink value by 100. A field the FCU does not report arrives NaN, is passed on as `None` and never overwrites the last good reading; the row greys out after 15 s of silence. No simulation graph publishes this topic. |
| `/blueboat/pinger_coordinates` | `std_msgs/Float32MultiArray` | `robot_interface.py` | USBL pinger marker | **ROBOT/BODY frame.** With `fixed_pinger=False` (the default) it is seeded from the Waterlinked *filtered* (`filaco`) position and dead-reckoned at odom rate between USBL updates. |
| `/uw_gps_data` | `std_msgs/Float32MultiArray` | `uwgps_log.py` | raw USBL freshness | 19 values: date(7), aco xyz, ant xyz, lat/lon/dep, filaco xyz. |
| `/monitoring_data` | `std_msgs/Float32MultiArray` | `master_control.py` | target display, distance plot | `[t, x, y, psi, x_d, y_d, psi_d, u1, u2]`, published at the controller's 20 Hz loop rate. `x_d/y_d/psi_d` are **WORLD frame for every controller branch** (§6, non-negotiable N3). |
| `/thruster_input` | `std_msgs/Float32MultiArray` | `master_control.py` | motor read-out | Order is **`[right, left]`** in Newtons. |
| `/blueboat/controller_ready` | `std_msgs/Bool` | `robot_interface.py` | readiness gating; **E-STOP acknowledgement** | Default QoS (depth 10, volatile). **Re-published every 1 s** rather than once, because a one-shot handshake races DDS discovery — that periodic republish, not a latched QoS, is what makes a late subscriber receive it. Since 2026-09-04 it also carries `False`, published the instant `full_stop()` latches, which is what confirms an E-STOP landed (N1b). |
| `/blueboat/param_mode` | `std_msgs/String` | `param_set` | safe-shutdown acknowledgement | `'default'` / `'override'`, or `''` for "alive but no mode locked". **Heartbeated at 1 Hz** since 2026-09-04, so a late station sees the mode without waiting for a transition — which is exactly why the shutdown sequence requires a transition rather than any matching value (N1). |
| `/blueboat/run_state` | `std_msgs/String` (JSON) | `master_control` (run session only) | RUN box, START RUN gating, run acknowledgement | `{seq, ok, detail, state: idle\|loading\|running\|error, controller_type, use_pinger, note, mission, run, npy}`. Answers every run command (`seq`) and is heartbeated at 1 Hz. The truth about whether a run is active. |
| `/blueboat/run_log` | `std_msgs/String` (JSON) | `robot_interface` (run session only) | RUN box "Log" and "Bag" rows | `{run, logging, csv, layout, bag_enabled, bag_recording, bag}` (the `bag_*` fields since v1.1, absent from an older control stack → "—"), heartbeated at 1 Hz: confirms the run's position CSV is being written (a run it does not confirm within 3 s is flagged) and its ros2bag recorded (flagged the same way: the CSV is there but the run will not be replayable). |
| `/replay/state` | `std_msgs/String` (JSON) | `bag_replay` (bag replay only, v1.1) | REPLAY box | **TRANSIENT_LOCAL depth 1** (like the sea state), 2 Hz: `{bag, t, duration, rate, paused, finished, frame, world, gazebo, …}` → `DataStore.replay` (`mcs.core.replay.ReplayReadout`). |

**Sea state (sim only, 2026-09-03; plain-language redesign 2026-10-08, by
the user's order).** `/sim/sea_state` (`std_msgs/String` JSON, subscribed
**TRANSIENT_LOCAL depth 1** — the one latched reader in the station) feeds
`DataStore.sea` (`mcs.core.sea.SeaReadout`) and the floating SEA STATE box
(`gui/sea_state_box.py`, parented to the map view at its top-left — the
mirror of the mission-stats box; rows Current / Waves / **Boat motion**
(the simulator's own 30 s roll/pitch/heave window) / Next change /
Status), visible only while a Gazebo mission runs.
`/sim/sea_state/command` (String JSON) is published by "Change sea state…"
through `CommandCenter.send_sea_state`.

The operator's model is `mcs.core.sea`: `CurrentSetting` (on/off, mean
speed, from, `variation` level), `WaveSetting` (on/off, a wave level or a
custom height, from, period automatic or set, `crest`, `wakes`) and the
advanced `SeaTimeline` of `Segment`s (start, transition, both settings),
gathered in `SeaChoice` (+ seed). Each emits **the simulator's own
schedule blocks with plain keys** — the simulator resolves them
(`blueboat_sss_sim/sea/presets.py`); the station does no sea physics. The
one number it derives itself, the "automatic" period shown next to the
radio button, mirrors the simulator's rule from the same table and is a
display hint only. Labels and plain descriptions come from the
simulator's installed `share/blueboat_sss_sim/config/sea_states.yaml`
(`SeaCatalog`, never-raises, CM-3; built-in names when absent);
literature references are deliberately **not** shown to the operator.
At launch every non-null choice is written as
`~/.config/blueboat_mcs/sea_states/last_launch.yaml` and passed as
`sea_schedule:= sea_seed:=` — on `full_mission_launch.py` in world mode,
on a companion `ros2 launch blueboat_sss_sim sea_state_launch.py
world_name:=ocean …` (`launch_manager.companion_command`, its own session,
best effort) in the empty world. Since 2026-10-09 (user's order) that
companion starts for **every** empty-world launch, a null choice included
(then with no `sea_*` argument): the simulator's catamaran hydrostatics carry
the hull in every run, so without it this path alone floated on Gazebo's URDF
box and a live "Change sea state…" had no listener. The SEA STATE Status row
turns amber "· hull: Gazebo box" when the readback's additive
`hull.model` says the box is in charge (`SeaReadout.hull_model`). Saved
plans live in the same folder (`list_schedules` hides `last_launch` and
the pre-2026-10 `custom_waves` / `timeline_unsaved`); older `linear`
timelines open as segments whose change spans the whole gap (the same
water). The dialog (`gui/dialogs/sea_state_dialog.py`, cards
`CurrentCard` / `WavesCard` reused by `gui/dialogs/sea_timeline_dialog.py`)
is shown by the bottom toolbar after the world dialog for **every**
simulation launch and again, live, from the floating box's button.

**Run sessions (real-robot launches, 2026-10-09, by the user's order).** Every
real-robot launch is a session (`BlueBoat_launch.py session_mode:=True`,
`LaunchParameters.session`): the boat connects and goes into override, but `master_control` idles — Manual Target and
the arrow keys drive it; no path or pinger is followed and nothing is logged — until **START
RUN** (the RUN box pinned under the left panel). **STOP RUN** ends the run: thrust zero,
still in override, the run's CSV/report and `.npy` written by the control stack. Between
runs **Update Mission** (the Launch button's session label) chooses the next run's designer
path or the pinger, the controller (MPC loads on a worker thread in `master_control`; START
RUN waits) and the log note. **Turn Off Connection** (the Stop button's session label) is
Stop Mission's sequence, an active run being stopped first. `mcs/ros/run_session.py` owns
the protocol (`seq`-acknowledged `configure`/`start`/`stop`, the gating in
`start_blockers()`) and the **one file `path_generation` watches in a session**,
`LaunchConfig.session_trajectory_file`
(`~/.config/blueboat_mcs/trajectories/.session/run.yaml`), rewritten atomically just before each `start`: a GPS-anchored mission is re-deployed with the
current fit (`io_yaml.deploy_mission`), any other mission is **placed at the boat**
(`io_yaml.place_mission`: first sample on the boat's odom position, first heading along its
odom yaw). Writing before sending is what lets the boat move the moment START RUN is
pressed. Built-in shapes are simulation/CLI only. Histories record only during a run
(`DataStore.recording()`), so trail, statistics, live distance and timeline freeze on the
last run between runs. A Gazebo launch keeps the v1.0 flow; only the arrow box and the map's
circled **S** (first point of the mission path) are shared with it. Since v1.1 each run also
records a **ros2bag** (every non-sonar topic), filed by the control stack into the run's
`Robot_data/<stem>/` folder; the RUN box's **Bag** row shows recording / saved / off.

**Bag replay — Gazebo Source "From a ros2bag" (v1.1, 2026-10-13, by the user's order).** The
launch dialog's Gazebo mode has a **Source** row: *Controller* (the v1.0 simulation) or *From a
ros2bag* — a recorded run folder (or its `*-bag` folder, validated: a bag with a pose topic)
and an initial playback rate, launched as `blueboat_control Replay_launch.py bag:= rate:=
[world_dir:=]` (`LaunchParameters.replay_bag` / `replay_rate`). The world dialog then offers
the personalized worlds the bag's **GPS track** touches (`worlds.eligible_worlds_for_track`);
**no sea-state dialog and no companion**: the recording already holds the real sea, and the
replayed boat (Control's force-free `replay:=true` description) cannot be moved by one. The
replay publishes the **recorded** GPS fixes, compass and odom (full attitude), so
`mission.replay` makes `map_frame_ready()` wait for a real GPS anchor exactly as on real
water, and SimGps stays disarmed (one GPS publisher). The floating **REPLAY** box (top-left,
the SEA STATE box's corner — never shown together) shows the run, time / duration and status,
with Pause/Resume and the rate. Manual Target and the Default/Override toggle are greyed:
nothing commands the replayed boat. Stop Mission runs the simulation shutdown sequence.

### Publications

| Topic | Type | Consumed by | Contract |
|---|---|---|---|
| `/blueboat/input_str` | `std_msgs/String` | `robot_interface.py` (dispatch), `param_set` | Values: `enable`, `disable`, `default`, `override`, `stop`, `arm`, `disarm`, `move <l> <r> <s>`. Any unrecognised token falls through to `move_callback`; an empty message is ignored. The station publishes `default`, `override`, — from either E-STOP button — `stop`, and — from **Release E-STOP** (run session, motors enabled, latched) — `enable`, the only thing that clears `robot_interface`'s E-STOP latch (it also re-opens the motor gate). |
| `/blueboat/manual_target` | `std_msgs/Float32MultiArray` | `master_control.py` | `[x, y]` in the **WORLD frame**. `[0.0, 0.0]` is the *resume-original-mission sentinel*, not a coordinate — a genuine click at the origin is nudged by `1e-3`. Only the explicit "Continue Original Mission" action (labelled "Release Target" between the runs of a session) may publish `[0,0]`. Turning arrow control off publishes the boat's own position here (hold where released). |
| `/blueboat/run_command` | `std_msgs/String` (JSON) | `master_control.py` (run session only) | `{"cmd": "configure", controller_type, use_pinger, note, mission, seq}` / `{"cmd": "start", seq}` / `{"cmd": "stop", seq}`. `configure` is re-sent until acknowledged (idempotent); `start`/`stop` are sent once. |
| `/replay/command` | `std_msgs/String` (JSON) | `bag_replay` (bag replay only, v1.1) | `{"cmd": "pause"\|"resume"}` / `{"cmd": "rate", "rate": x}` from the REPLAY box (`CommandCenter.send_replay_command`, `mcs.core.replay.command_json`). |
| `/blueboat/arrow_thrust` | `std_msgs/Float32MultiArray` | `master_control.py` (every mode) | `[right, left]` N at 20 Hz while ARROW CONTROL is on (`[0,0]` with no key held); `[]` releases. Overrides every branch, pauses a run like a manual target; `master_control`'s dead-man (`arrow_timeout` 0.5 s) zeroes a gap and stays engaged. No `command_sent` per tick (it would flood the log). |

### Services

| Service | Type | Role |
|---|---|---|
| `/path_request` | `blueboat_interfaces/srv/RequestPath` | **Client.** Request is `std_msgs/Float32MultiArray path_request` (an array of path-parameter values); response is `nav_msgs/Path path`. This is how the mission path is fetched for display. |

### Frame conventions (the recurring source of bugs)
- Manual target: **world** on the wire. Pinger target: **body** on the wire.
  `master_control` converts the manual target via `inRobotFrame()` before
  `solve_LoS()` and passes the pinger vector straight through, so **both are
  robot-frame at the solver** — that part is consistent by design.
- Yaw everywhere in the app: radians, CCW-positive about +z, y-up world.
- Odometry yaw is **absolute ENU** (0 = East). `DataStore.robot_true_heading()`
  prefers the compass and falls back to the odom yaw directly — there is no
  rotation correction anywhere (see `docs/GPS_MAP_ARCHITECTURE.md`).

---

## 3. NON-NEGOTIABLE constraints

Violating any of these breaks another module, hardware safety, or field data.

**N1 — Safe shutdown before terminating a launch.** **Stop Mission** (labelled
**Turn Off Connection** in a real-robot run session) and **app exit** — the only
two paths that terminate anything — must run the `command_center` sequence (in a
session, a run still active is stopped first through `/blueboat/run_command`, so
the control stack closes and files its log itself): publish `default` on `/blueboat/input_str` → verify a
matched subscriber via `get_subscription_count()` → wait for the
`/blueboat/param_mode` echo (one republish at T/2) → flush delay → *only then*
terminate the process. Killing the launch first can leave the motors in
override. In a simulation graph (`CommandCenter.set_simulation_mode(True)`) the
*acknowledgement wait* is skipped — `Sim_launch.py` starts neither
`robot_interface` nor `param_set`, so no echo can arrive — but the publish still
happens before the flush and the terminate, so the ordering guarantee is
unchanged.

Since `param_set` heartbeats `/blueboat/param_mode` at 1 Hz, the echo that
confirms *this* command must be a **transition** into `default`: the sequence
snapshots the last known mode before publishing and ignores repeats. A boat
already in `default` is reported as its own confirmation level
(`already-default`) rather than being credited with an echo it never sent.

**N1b — E-STOP cuts the motors and does nothing else.** The three concerns are
separate operator actions and must never be re-nested (they were until
2026-09-04, which is why an operator could not stop the boat without also
surrendering the servo mapping, nor surrender the mapping without ending the
mission):

| Button | Publishes | Acknowledged by | Terminates? |
|---|---|---|---|
| **E-STOP** | `stop` | `/blueboat/controller_ready` → `False` | no |
| **E-STOP + Stop Override** | `stop`, then `default` | the above, then the `param_mode` transition | no |
| **Release E-STOP** (run session, motors enabled, while latched) | `enable` | `controller_ready` returning `True` | no |
| **Stop Mission** / **Turn Off Connection** / app exit | (`stop` run, then) `default` | the `param_mode` transition | yes, after confirmation |

`stop` is the real emergency primitive: `robot_interface.full_stop()` zeroes the
thrust, closes the motor gate, disarms and **latches** until an explicit
`enable`, all without leaving override. Leaving override is a *transfer* of
authority — the RC channels go back to whatever else is transmitting — so it is
never part of the panic button. The E-STOP acknowledgement runs in its own state
machine so it can neither block nor be blocked by an in-flight `default`
sequence.

**N2 — `[0,0]` on `/blueboat/manual_target` is a control-handover sentinel.**
Never publish it as a position.

**N2b — A real-robot launch never moves the boat or logs before START RUN, and a
run's path is written before its start.** In a run session `master_control`
follows nothing and records nothing until `start`; the station writes the run's
path into the session file (deployed or placed, atomically) and only then sends
`start`, so the first window `master_control` asks for after the start is the new
path's beginning. A non-anchored mission is placed at the boat at that moment,
never at the launch point — the boat has been driven since. Never send `start`
first and write after: one window of the previous run's path would be followed.

**N2c — Arrow control never hands the boat back to autonomy by itself.** The
stream's dead-man zeroes thrust and stays engaged; turning the box off holds the
boat where it was released (a manual target at its position); only Continue
Original Mission resumes a paused run. Keys are captured only while the main
window is active and no modal dialog is open, and all held keys are forgotten on
focus loss.

**N3 — Never re-apply a frame correction to `/monitoring_data` in the app.**
`x_d/y_d` are world-frame at the source: `master_control` captures the world
target before its `inRobotFrame()` conversion, at every branch (the
`# --- world-frame monitoring target ---` markers). An app-side robot→world
fixup would double-convert.

**N4 — The pinger world position is computed only inside the pinger message
callback**, using the pose concurrent with that message, and stays fixed
between messages. Re-anchoring a stale body vector to each new odom pose makes
the marker drag along behind the robot.

**N5 — The map view never rotates.** North-up is achieved by placing items in
the ENU frame, not by rotating the view; rotating the view rotates the satellite
tiles with it. Only the robot glyph rotates.

**N6 — The robot glyph's heading must come from an absolute source.** The
compass is preferred; the odom yaw is an acceptable fallback because it is
absolute ENU (0 = East, not launch-zeroed). Never re-apply a rotation
correction to either — the old `theta` machinery is gone and re-adding one
would rotate a correct heading wrong.

**N7 — Mission-path preview uses the `/path_request` service directly, never
`path_publisher`.** `path_publisher.py` is only started by `Sim_launch.py`, so
depending on it breaks every real-robot run.

**N8 — A GPS-anchored mission is never deployed without a valid translation
anchor.** `io_yaml.deploy_mission()` raises when `current_fit` is None; the
watcher polls `geo.is_valid`, which converges from the first few GPS fixes
with **no vehicle motion** (the world frame is local ENU, so only a
translation is estimated — the old heading-alignment requirement and its
station-keeping deadlock are gone). Deferred deployment runs on real water
**and** in Gazebo runs of GPS-anchored missions (`gps_simulated`: the
station synthesises the fixes itself and spawns the boat at the **fixed**
`LaunchConfig.sim_spawn_yaw_deg` via `spawn_yaw:=`) — only a sim launch of a
*non-anchored* mission
skips it and executes design-frame points directly. A GPS-anchored sim
launch may instead target a **personalized world**
(`LaunchParameters.world_dir` set → `blueboat_sss_sim
full_mission_launch.py`): the deferred deploy still runs, but the fixes
come from the simulator's mavros shim (the world's own anchor) and the
station's SimGps stays **disarmed** — arming it too would put a second
publisher on `/mavros/global_position/global` with an arbitrary origin and
deploy the path into the wrong frame relative to the world geometry. That
launch file declares no `spawn_yaw`; the boat spawns at world (0, 0)
heading east.

**N8b — A simulated run is reproducible: same spawn pose, same fix noise.**
The spawn heading is a configured constant (`LaunchConfig.sim_spawn_yaw_deg`,
0° = east; the position is always Gazebo (0, 0)) and `SimGpsModel` is
constructed with `SimGpsConfig.noise_seed`, never an unseeded generator.
Both used to be random — the heading drawn per launch, the noise per
process — and since the fix noise feeds the anchor estimate, the *deployed*
path landed a little differently every run, so two sim runs of one mission
could not be compared and a difference could not be attributed to the change
under test. Rehearsing another orientation is a *choice*: `spawn_yaw:=` in
Extra args (appended last, so it wins) or the config value.

**N9 — Keep `mcs/` free of control-stack code.** Control-stack changes belong in
the `BlueBoat-Control` submodule (§6); nothing under `mcs/` imports
`blueboat_control` modules at runtime.

**N10 — Never weaken a comparison baseline or overstate evidence** in anything
this module produces for the thesis. See the project-level
`project_synthesis.md` §4/§8.6: the two-pass orthogonal baseline must be tuned
seriously, and simulation-derived results are stated as model-conditional.

---

## 4. Map & georeferencing (current model)

**One scene regime, GPS-frame-only** (`mcs/gui/map/map_view.py`); the authoritative,
self-contained description is `docs/GPS_MAP_ARCHITECTURE.md`. What a change must preserve:

- The scene is **local east/north metres about the latched first GPS fix**, north-up, never
  rotating (N5). The only estimated quantity is a **translation** `t` (`core/geo.py::GeoFit`,
  `EN = world + t`), a per-axis median over GPS↔odom pairs (< 0.5 s odom freshness);
  `is_valid` within ~1 s of GPS, **no motion needed**. There is no rotation estimation — with
  an ENU odom frame the rotation is zero by construction.
- **Nothing is drawn, and manual-target clicks are refused, before
  `store.map_frame_ready()`** (`geo.is_valid or (simulation and not gps_simulated)`).
  Non-GPS sim uses the identity anchor; GPS-sim runs the full real-water pipeline on
  synthesised fixes.
- **The georeferencer is reset on every mission launch** (new control-stack world origin);
  a relaunch briefly showing "waiting for GPS fix" is correct.
- Satellite tiles are placed **axis-aligned** from `(lat0, lon0)`, never rotated.
- `MapView._to_world()` is the exact inverse of `_to_scene()`, and **every** mouse position
  read out of the view goes through it (manual target, inspector, measure); the crosshair is
  re-placed from the stored world target every tick.
- The `/path_request` preview waits for `map_frame_ready()`; `_on_launch_state("idle")`
  clears `store.mission_path` so a finished run's path never re-anchors onto the next frame.

---

## 5. Survey Pattern Designer & trajectory files

Full guidance in **`mcs/designer/CLAUDE.md`** (loads when working there). Cross-cutting facts:

- Output in `~/.config/blueboat_mcs/trajectories/`: `<name>.yaml` (runtime,
  `blueboat_trajectory/1`, dense `[t, x, y, yaw]` — read by `path_generation`, the World
  Builder and the GCS), `<name>.meta.yaml` (editor only), `.deployed/<name>.yaml`
  (regenerated every launch for GPS-anchored missions — **never hand-edit**; carries
  `deployed_from` / `deployed_fit_rms_m`), `.session/run.yaml` (the real-robot run session's
  one watched file, rewritten atomically before every run: GPS-deployed, or placed at the boat
  with `deployed_from` / `placed_at` — **never hand-edit**). Format contract:
  `docs/05_trajectory_format.md`.
- **The designer reads no live telemetry**: its anchor comes only from typed coordinates, a
  Gazebo world, a one-off robot-fix snapshot, or the opened file's `geo_anchor` — a live fit
  once silently shifted every path and rewrote anchors on save.
- Other persistent state: `~/.config/blueboat_mcs/config.json` (user overrides) and
  `~/.config/blueboat_mcs/tile_cache/` (disposable).

---

## 6. Control-stack files — in the `BlueBoat-Control` submodule

The four nodes the station depends on behaving a particular way are **committed
to `blueboat_control`** and installed by its `CMakeLists.txt`. There is no copy
step from this repo: change them in the `BlueBoat-Control` submodule, commit
there, and `colcon build`.

| File (path under `BlueBoat-Control/blueboat_control/`) | What the station depends on |
|---|---|
| `src/_custom_libraries/path_generation.py` | The `from_yaml` branch + a file **watcher** that reloads when the file appears/changes, holding a station-keeping pose until it does (this is what makes deferred GPS deployment possible). `generate_path()` and every hardcoded trajectory are unchanged. |
| `src/_custom_libraries/yaml_trajectory.py` | Loads `blueboat_trajectory/1` and evaluates at time `t`. Depends only on PyYAML + numpy. |
| `src/master_control.py` | Captures the **world-frame** target before the `inRobotFrame()` conversion so `/monitoring_data` is uniform across branches (`# --- world-frame monitoring target ---`, five capture sites: manual, the MPC / PID / LoS path branches, and the pinger branch). 20 Hz loop — `self.dt = dbl('control_dt', 0.05)`, a committed **declared ROS parameter**, so the rate the station's `DiagnosticsConfig.expected_hz` assumes is settable per launch (see the ⚠ below). `/controller_target` unchanged. **Run session** (`session_mode`, section 6 of the file): idle until `start`, `configure` between runs (MPC on a worker thread), one `.npy` per run, `/blueboat/run_state`. **Arrow keys** (`/blueboat/arrow_thrust`, every mode): highest-priority branch, `arrow_timeout` dead-man that stays engaged. |
| `src/robot_interaction/robot_interface.py` | CSV logging: important columns first (names unchanged), rows filled **by column name**, `[right, left]` thruster order, and the no-pinger target logged from `/monitoring_data`. Also the producer of `/blueboat/odom`, `/blueboat/pinger_coordinates` and `/blueboat/controller_ready`. **Run session**: one CSV per run, opened/closed from `/blueboat/run_state`, filed by `poslog_report` in a subprocess, reported on `/blueboat/run_log`; the UGPS is tracked whatever the first run's mode. |
| `src/_custom_libraries/robot_log_schema.py` | The position-CSV column layouts themselves (`COLUMNS_PINGER` / `COLUMNS_NO_PINGER`, selected by `columns_for(use_UWgps)`). ROS-free data module — read this, not the node, to learn the CSV format offline analysis consumes. |

Trajectory selection needs **no launch-file change** — the path rides inside the
existing argument: `trajectory:=from_yaml:/abs/path.yaml`.

### Control-stack behaviours the station does not compensate for
**Point-LoS arrival checking is the pinger branch's alone.** `safety_distance`
is a declared ROS parameter on `master_control`, defaulting to `-1.0` on the
real boat (arrival check off) and `+1.0` in simulation. When it is positive,
`stopping_sequence` latches on arrival and the boat brakes to zero thrust with
nothing on screen saying why. A **manual** target no longer goes through either:
it has its own arrival state (`manual_keep_location`, hold / reacquire radii),
`stopping_sequence` is not consulted for it, and `manual_target_callback` clears
the latch on every new target — so the old "one reached target mutes every later
one" failure mode is gone from that path.

Pinger-marker lag is inherent to the source — the filtered seed plus dead
reckoning described in §2 (`robot_interface.odom_callback`); N4 is what keeps it
from *also* dragging behind the robot. Neither is corrected in `mcs/`: the
station supervises, and a station-side workaround would hide a control-stack defect
behind a display that looks right.

**⚠ Line numbers into `BlueBoat-Control` are volatile, and the installed build
can lag the source.** `control_dt` (`self.dt = dbl('control_dt', 0.05)`) and
`safety_distance` are **committed** declared parameters now — the old
working-tree-only caveat no longer applies — but what runs is the copy in
`~/ros2_ws/install/`, which only `colcon build` refreshes. That matters doubly
since the 2026-08-31 local-ENU odom fix (`docs/03_ros_integration.md` §Observations): a
stale install publishes the old hybrid frame (launch-relative yaw over ENU axes)
and silently breaks the GPS map and trajectory following from non-East headings.
Anchor on the symbol name, not the line, and rebuild in `~/ros2_ws` before a field
session so the install matches the SHA the superproject records.

### CSV logs
Written by `robot_interface.py` (real) and `simulation_interface.py` (Gazebo); layouts in
`BlueBoat-Control/blueboat_control/src/_custom_libraries/robot_log_schema.py`. `target_*`
exists **only** in the no-pinger layout. In a real-robot run session there is one CSV (and
one report folder, one `.npy`) **per run**, named after the run's start time and the note
in force for it; nothing is written between runs.

---

## 7. Commands known to work

```bash
# run (the one workspace is ~/ros2_ws; env.sh activates .venv + sources
# ROS2 and the overlay)
cd ~/ros2_ws && source env.sh
cd src/BlueBoat-SideScanSonar/BlueBoat-MCS
pip install -r requirements.txt   # first time only; NOT --user inside the venv
python3 run.py
python3 run.py --config /path/to/config.json --verbose

# build.sh does the whole sequence (colcon build in ~/ros2_ws, then run.py)

# regression script — headless, no ROS needed
QT_QPA_PLATFORM=offscreen python3 smoke_test.py

# lint — no ROS and no venv needed; ruff resolves no imports
pip install -r requirements-dev.txt   # first time only
ruff check .

# launching a mission from the CLI. A real-robot launch from the station is a
# run session (the RUN box then starts/stops runs); without session_mode the
# CLI keeps the one-launch-one-mission behaviour
ros2 launch blueboat_control BlueBoat_launch.py session_mode:=True \
    enable_motors:=True controller_type:=PID use_pinger:=False \
    trajectory:=from_yaml:$HOME/.config/blueboat_mcs/trajectories/.session/run.yaml
ros2 launch blueboat_control BlueBoat_launch.py \
    enable_motors:=True controller_type:=LoS use_pinger:=True
ros2 launch blueboat_control BlueBoat_launch.py \
    controller_type:=LoS trajectory:=from_yaml:/abs/path/mission.yaml
ros2 launch blueboat_control Sim_launch.py \
    robot_file:=thrusters_ur controller_type:=MPC trajectory:=circle \
    note:=sim-tuning   # note:= names the poslog CSV, as on the real boat
ros2 launch blueboat_sss_sim full_mission_launch.py \
    world_dir:=$HOME/worlds/<path>/<world> with_control:=true \
    trajectory_file:=$HOME/.config/blueboat_mcs/trajectories/.deployed/<name>.yaml \
    controller_type:=PID note:=sim-dam   # personalized-world launch (needs blueboat_sss_sim built)

# field debugging
ros2 topic echo /mavros/global_position/compass_hdg
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: default"
```

`~/ros2_ws` is the project's only workspace: `BlueBoat-Control`, this station
and every other node are built and run there, on this laptop (`/blueboat_ws`
never existed in this project — do not reintroduce it).

`ruff check .` is the lint gate, configured in `ruff.toml`. It exits clean on

`ruff check .` (pinned `ruff==0.16.5`, because the gate is ruff's *default* rule selection)
exits clean in any environment; suppressions and their reasons are in `ruff.toml`. The
fourteen broad `except Exception` clauses carry `# noqa: BLE001` with a reason each and stay
broad.

**No type-checker, deliberately.** A mypy survey of `mcs/` found 45 errors and *no* real
defect (ROS message stubs, PySide6 stub-narrowing around the `schema` extension points).
Re-open only with new evidence.

`smoke_test.py` is a linear script, not a test framework: it imports every module, builds the
window offscreen, drives synthetic telemetry through the `SignalBus`, prints one `… ok`
checkpoint per block and aborts on the first failed assertion; it ends with
`SMOKE TEST PASSED`. It runs identically with no `rclpy`, with `rclpy` but no
`blueboat_interfaces`, and fully sourced. Read the script for what each block asserts; the
§3 non-negotiables it pins are N1/N1b (`safe shutdown ok`: separation, ordering, the
transition-not-repeat rule), N2 (`sentinel ok`), N4 (`pinger anchor ok`), N8/N8b
(`deploy guard ok`, `launch dialog sim-gps ok`), and the map frame (`map frame ok`) and
designer anchor (`designer anchor ok`) rules. The run session (2026-10-09) adds
`session launch ok` (real launch = session: CLI, designer paths only, no empty controller,
MPC greyed with the pinger), `placement ok` (N2b: first sample on the boat, ahead of it,
rigid, atomic), `arrow ok` (N2c: the thrust law with the operator's examples, auto-repeat,
focus loss, hold point before the release), `run session ok` (configure on the first
heartbeat, `seq` acknowledgement, START RUN gating, path written before `start`, recording
gate, refused start, Update Mission, E-STOP latch / Release E-STOP, and Turn Off Connection's
order: run `stop` → `default` → terminate), `start marker ok` and `session window ok`. v1.1 adds
`replay ok` (the replay launch arguments, target and absent companion, bag discovery from a
run folder, the dialog's Source row and bag validation, the REPLAY box and its commands, the
map waiting for GPS, greyed controls, the RUN box's Bag row incl. a pre-v1.1 control stack).

Still uncovered: the rest of the designer UI (`designer_map.py`, `panels.py`,
`designer_window.py`, including the `_push_undo` contract). A green run is not field
readiness — it cannot catch QoS mismatches, real message-type drift or launch-file argument
errors.

Simulation runs (`Sim_launch.py`) have no MAVROS, no `robot_interface`, no
`param_set` and no pinger; code paths gated on those must degrade quietly. They are
not run sessions: no `/blueboat/run_*` topic exists there, and the RUN box, Update
Mission and Turn Off Connection never appear.
Personalized-world runs (`full_mission_launch.py`) still have no
`robot_interface`/`param_set`/pinger, but the simulator's mavros shim DOES
publish `/mavros/global_position/global`, `compass_hdg` and `imu/data`.
A bag replay (`Replay_launch.py`) has no controller, no `simulation_interface` and no sea
state; `bag_replay` publishes every recorded `/mavros/*` topic (but `rc/override`),
`/blueboat/odom`, `/monitoring_data`, `/thruster_input`, the pinger topics and `/set_path`,
and puts commands and session plumbing under `/replay/` — the station's run-session and
E-STOP paths see nothing on their live topics.

---

## 8. Documentation set

Four documents, all under `docs/`. Architecture, data flow, the module
inventory and the "how to add X" checklists live in §1 of this file, not in
`docs/` — there is no separate architecture or developer guide.

| Doc | Content |
|---|---|
| `03_ros_integration.md` | every topic / command / service, the launch targets, the stop actions and the safe-shutdown sequence, open control-stack observations |
| `04_user_guide.md` | every screen and control, including the run session (RUN box, Update Mission, arrow control) and the Survey Pattern Designer |
| `05_trajectory_format.md` | the `blueboat_trajectory/1` YAML contract, deferred GPS deployment and the run session's placed / deployed file |
| `GPS_MAP_ARCHITECTURE.md` | the authoritative, self-contained description of the GPS-only map model (§4), written for reuse in any application that draws a GPS vehicle on a real-world map |

`03_ros_integration.md` §Observations carries what is still open or carried
deliberately in the control stack, plus the table of `BlueBoat-Control` fixes that
a stale `~/ros2_ws/install` build silently reintroduces. §6 above owns the station-side half.

Line citations into `BlueBoat-Control` drift whenever that module is edited;
`BlueBoat-Control` owns them, and its own `CLAUDE.md` §2.1.1 records the
section layout they point into. Prefer the symbol name over the line number.

This file stays authoritative where it and `docs/` disagree — `docs/` is the
long-form explanation, not a second source of truth. There is no `TODO.md`:
the module is feature-complete, and what remains is field verification, which
this repository cannot settle.
