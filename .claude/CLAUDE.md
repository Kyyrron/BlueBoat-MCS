# BlueBoat-MCS — Mission Control Station

Operator ground-station GUI for a BlueRobotics BlueBoat USV (master's thesis on aspect-aware
side-scan sonar survey). Runs on the operator laptop (`~/ros2_ws`) like every node of the
project; the boat carries only its hardware and firmware, reached over the BlueBoat Base
Station WiFi.

**The station supervises and commands. It performs no control computation and duplicates no
logic that already exists on a ROS topic.** Every number it shows comes from a topic or
service of the `blueboat_control` stack. When something looks wrong on screen, first ask
whether the producing node is right, not whether the GUI should compensate.

Standalone Python (no colcon, no `setup.py`), run from the repo root; ROS 2 comes from the
sourced workspace, never from pip. Frozen at `v1.0` and **add-only** (superproject
`CLAUDE.md` §3): change only by additions, or by refactors whose behaviour is shown
identical by `smoke_test.py`.

---

## 1. Layout & internals

`mcs/main.py` bootstraps (`run.py` is a thin launcher).

| Package | Role |
|---|---|
| `config/settings.py` | **All** topics, thresholds, gains and paths (dataclasses; JSON overrides from `~/.config/blueboat_mcs/config.json`). |
| `core/`, `models/` | No widgets, no rclpy (only `QObject` signals) — what makes the offline smoke test possible. `core/signals.py` (SignalBus), `geo.py` (GeoReferencer), `series.py` (TimeSeries), `clock.py` (the one elapsed-time format), `arrows.py` (arrow-key thrust law), `guardian.py` (GUARDIAN box rules), `replay.py` (bag folders, `/replay/state`), `sea.py` (sea-state model), `worlds.py` (SSS-Sim world-folder contract, reimplemented locally — never imported, CM-3), `sim_gps.py`, `los_predictor.py`. `models/store.py` is the DataStore. |
| `ros/` | `ros_manager` (the rclpy thread), `bridge_node` (**the only file that knows message types**), `launch_manager`, `command_center` (commands, E-STOP, safe shutdown), `run_session` (the station's half of a real-robot run session). |
| `gui/` | `main_window` (composition root), panels, `map/`, `plot/`, `dialogs/`, and the boxes: `run_panel` (RUN), `guardian_box` (GUARDIAN), `arrow_box`, `replay_box`, `sea_state_box`, `mission_stats`. Shared widgets/helpers in `widgets.py` (incl. `elide_middle`) and `theme.py` (incl. `floating_box_style`). |
| `designer/` | Survey Pattern Designer — its own `CLAUDE.md`. |

`ruff.toml` and `requirements-dev.txt` are gitignored. Control-stack code lives in the
**`BlueBoat-Control` submodule**, never here (§6, N9).

**Threading.** `rclpy` spins on its own thread. `SignalBus` is the **only** crossing point:
ROS callbacks emit Qt queued signals, `DataStore` slots mutate state on the GUI thread, and
widgets repaint from a single ~10 Hz tick in `MainWindow` (`MapConfig.ui_refresh_hz`). Never
touch Qt objects from a ROS callback; never block the ROS thread on GUI work. `DataStore` is
the single source of truth for the GUI: widgets read it, never subscribe to ROS.

**Degraded mode.** Without `rclpy` the app still starts GUI-only. Keep new ROS imports lazy or
guarded.

**Data flow (odometry, the canonical example).** `BridgeNode._on_odom` (ROS thread) converts
to plain lists, stamps the station clock, marks its `TopicStats` and emits
`SignalBus.odom_received` → `DataStore.on_odom` (GUI thread) updates `RobotState`, histories,
travelled distance → at the next tick `MainWindow._on_tick` calls each panel's / the map's
`refresh()`, which *pull* from the store. User intents flow back: widget → signal →
`MainWindow` slot → `CommandCenter` → `BridgeNode.publish_*` (thread-safe) → ROS.

**Clocks.** Every history timestamp is the **station clock** at **reception**
(`core/clock.station_clock()`, read through `store.now_mono()`): `time.monotonic()` in a
real-robot or replay launch and before any launch; in a **simulation launch** it follows
`/clock` (the bridge feeds it; `MainWindow._on_mission_launched` → `set_follow`), advancing in
sim seconds, continuous, interpolated, never backwards, wall-paced after
`sim_speed.clock_stall_s` (3 s) of `/clock` silence. So a run slowed below real time (the
ray-cast sonar) shows rates, ages, elapsed time, plots and statistics in sim time. Every
comparison against a reception stamp uses it; self-contained operator deadlines (N1
shutdown, guardian waits, the path-preview timeout, the arrow `QTimer`) stay monotonic.
Experiment time is `t − store.t0` (first odometry sample). The controller's own `t` in
`/monitoring_data` is never used as a clock.

**Repaint bounds.** `TimeSeries.decimated_window` caps every repaint at
`map.trajectory_max_points_drawn` (20 000) vertices per polyline and 2000 per plot; the tile
layer refuses more than 64 tiles per viewport.

**Map rendering.** Constant-pixel glyphs (`ItemIgnoresTransformations`), cosmetic pens, an
adaptive 1/2/5 metric grid in `drawBackground`, satellite `TileLayer` at z = −100. Modes
`NORMAL` / `MANUAL_TARGET` / `MEASURE` are a state machine inside `MapView`, which emits
intents (`target_clicked`, `point_inspected`) and never publishes.

### How to add…

- **A subscribed topic:** name it in `TopicsConfig`; thresholds in `DiagnosticsConfig` (it
  then appears in the diagnostics panel); a `Signal` on `SignalBus`; subscription + `_on_x`
  in `BridgeNode` (call `self._mark(topic)` first); a `DataStore.on_x` slot connected in
  `MainWindow._connect_signals`.
- **A published command:** a publisher in `BridgeNode.__init__` and a `publish_x` wrapper
  (`_publish_text` for String topics; keep the `_pub_lock` pattern and the `command_sent`
  emission), a semantic method on `CommandCenter`, called from the GUI.
- **A map layer:** an item in `gui/map/map_items.py`, added in `MapView.__init__`, updated in
  `MapView.refresh()`, a key in `MapView.set_layer_visible`, one `_LAYERS` entry in
  `right_panel.py`.
- **A plot:** follow `DistancePlot` (a `QWidget` over one `TimeSeries`, honouring
  `set_time_window`), in a `CollapsibleSection` of the right panel, refreshed from
  `RightPanel.refresh()`.
- **A launch argument:** all in `mcs/ros/launch_manager.py` plus the dialog —
  `LaunchParameters.to_cli()` (the replay branch first; splices `sea_args()`),
  `launch_target()`, the companions (`companion_command` = empty-world sea state,
  `guardian_command` = real-robot link guardian, `perception_command`; held in
  `LaunchManager._companions`, signalled together by `stop()` / `_finalise()`), and
  `gui/dialogs/launch_dialog.py`. A per-run dialog setting that is not a launch argument
  (like `perception_mode`) is its own row and its own field, ignored by `to_cli()`.

---

## 2. ROS 2 interface — the exact contract

Names live in `TopicsConfig`; change them there, never inline. Peer: `blueboat_control`
(`robot_interface`, `master_control`, `path_generation`, `param_set`) plus MAVROS.
Subscriptions use the default reliable depth-10 profile unless stated.

### Subscriptions

| Topic | Type | Producer | Notes |
|---|---|---|---|
| `/blueboat/odom` | `nav_msgs/Odometry` | `robot_interface` (real) · sim bridge · `bag_replay` | **Local ENU**: origin = boat at launch, axes East/North, yaw **absolute ENU** (0 = East, CCW+, not re-zeroed); the sim differs only in origin. Pose, speed, trail. |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | MAVROS · **the station's own bridge** (Gazebo run of a GPS-anchored mission: a 5 Hz timer synthesises fixes from the sim odom about an origin `SimGpsConfig.offset_north_m` north of the first point, received back through its own subscription, so the whole real pipeline runs) | **BEST_EFFORT.** `lat == lon == 0` = no fix, discarded. Georeference, GPS read-out. |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | MAVROS | **BEST_EFFORT.** Degrees clockwise from north → `radians(90 − hdg)`. Preferred glyph heading; fallback the odom yaw. |
| `/mavros/battery` | `sensor_msgs/BatteryState` | MAVROS | **BEST_EFFORT.** `percentage` is a **0..1 fraction**. NaN fields pass as `None` and never overwrite the last good reading; the row greys after 15 s. Real boat only. |
| `/mavros/state` | `mavros_msgs/State` | MAVROS | Only when `mavros_msgs` imports. FCU connection (readiness count); `armed` (the **autopilot's** arming, not the motor gate) and `mode` for the GUARDIAN box and N1's MANUAL step. |
| `/blueboat/pinger_coordinates` | `Float32MultiArray` | `robot_interface` | **Body frame**, filtered (`filaco`) USBL fix dead-reckoned at odom rate. |
| `/uw_gps_data` | `Float32MultiArray` | `uwgps_log` | Raw USBL freshness only (19 values). |
| `/monitoring_data` | `Float32MultiArray` | `master_control` | `[t, x, y, psi, x_d, y_d, psi_d, u1, u2]` at 20 Hz; `x_d/y_d/psi_d` **world frame in every branch** (N3). |
| `/thruster_input` | `Float32MultiArray` | `master_control` | **`[right, left]`** N. |
| `/blueboat/controller_ready` | `std_msgs/Bool` | `robot_interface` | Readiness gating; re-published every 1 s (no latched QoS — the republish covers late subscribers). `False` the instant `full_stop()` latches = **E-STOP acknowledgement** (N1b). |
| `/blueboat/param_mode` | `std_msgs/String` | `param_set` | `'default'` / `'override'` / `''` (alive, no mode). **Heartbeated 1 Hz**, so shutdown requires a *transition* (N1). |
| `/blueboat/run_state` | String JSON | `master_control` (session) | `{seq, ok, detail, state: idle\|loading\|running\|error, controller_type, use_pinger, note, mission, run, npy}`, answers each command's `seq`, heartbeated 1 Hz. **The truth about whether a run is active.** |
| `/blueboat/run_log` | String JSON | `robot_interface` (session) | `{run, logging, csv, layout, bag_enabled, bag_recording, bag}`, 1 Hz. A run whose CSV is not confirmed within 3 s is flagged; same for its ros2bag. No `bag_*` keys from an older control stack → "—". |
| `/link_guardian/status` | String JSON | `guardian.py watch` (real sessions) | 1 Hz: `{connected, armed, mode, want_on, guardian_on, check, home, run_running, messages, busy, ack}` → `GuardianReadout`; stale after 4 s. |
| `/sim/sea_state` | String JSON | `sea_state_node` (sim) | **TRANSIENT_LOCAL depth 1** → `SeaReadout` (SEA STATE box). |
| `/replay/state` | String JSON | `bag_replay` (replay) | **TRANSIENT_LOCAL depth 1**, 2 Hz → `ReplayReadout` (REPLAY box). |

### Publications

| Topic | Type | Consumer | Contract |
|---|---|---|---|
| `/blueboat/input_str` | `std_msgs/String` | `robot_interface` | Tokens: `enable`, `disable`, `default`, `override`, `stop`, `arm`, `disarm`, `rearm`, `manual`, `move <l> <r> <s>`; anything else falls through to the move handler, empty is ignored. The station sends `default` / `override` (mode toggle, N1), `stop` (both E-STOPs), `enable` (**Release E-STOP**, the only token that clears the latch — it also re-opens the motor gate, so only offered in a session launched with motors enabled), `rearm` (GUARDIAN **Arm again**), `disarm` (turn-off choice), `manual` (N1, after *Guardian OFF*). |
| `/blueboat/manual_target` | `Float32MultiArray` | `master_control` | `[x, y]` **world frame**. `[0, 0]` is the resume sentinel (N2). Turning arrow control off publishes the boat's position (hold where released). |
| `/blueboat/run_command` | String JSON | `master_control` (session) | `{"cmd": "configure", controller_type, use_pinger, note, mission, seq}` (re-sent until acknowledged) / `{"cmd": "start"\|"stop", seq}` (sent once). |
| `/blueboat/arrow_thrust` | `Float32MultiArray` | `master_control` (every mode) | `[right, left]` N at 20 Hz while ARROW CONTROL is on (`[0, 0]` with no key), `[]` releases. Overrides every branch and pauses a run; `master_control`'s dead-man (`arrow_timeout` 0.5 s) zeroes a gap and stays engaged. No `command_sent` per tick. |
| `/link_guardian/command` | String JSON | `guardian.py watch` | `{"cmd": "on"\|"off"\|"check"\|"setup"\|"reclaim", "seq": n}`, answered in the status `ack`. |
| `/sim/sea_state/command` | String JSON | `sea_state_node` | `SeaChoice.command_json()` from "Change sea state…". |
| `/replay/command` | String JSON | `bag_replay` | `{"cmd": "pause"\|"resume"}` / `{"cmd": "rate", "rate": x}`. |

**Service client:** `/path_request` (`blueboat_interfaces/srv/RequestPath`: a
`Float32MultiArray` of path-parameter values → `nav_msgs/Path`) — the mission-path preview
(N7). A call unanswered after `path_request_timeout_s` is dropped; a failed preview is
re-armed up to `path_request_max_retries` times.

### Launch modes

`launch_manager.launch_target()` picks one of four files; `to_cli()` emits exactly the
arguments that file declares (another target's arguments would abort `ros2 launch`).

| Mode | Launch | Companions | Station behaviour |
|---|---|---|---|
| **Real robot = run session** | `BlueBoat_launch.py session_mode:=True trajectory:=from_yaml:<session file>` | guardian, perception (normal) | Below. |
| **Empty Gazebo** | `Sim_launch.py` (`robot_file`, `trajectory`, `controller_type`, `note`, optional `spawn_yaw`) | sea state (`sea_state_launch.py world_name:=ocean`, **every** launch, null choice included — the simulator's catamaran hull floats the boat) | No MAVROS / `robot_interface` / `param_set` / pinger; N1's acknowledgement wait skipped. |
| **Personalized world** | `blueboat_sss_sim full_mission_launch.py` (`world_dir`, `with_control:=true`, `trajectory_file`, `controller_type`, `note`, `sea_*`; **no** `spawn_yaw`; with the ray-cast sonar also `sonar_renderer:=raycast real_time_factor:=<f>`) | perception (dialog **Session** row: normal / dataset → `dataset_name:=<YYYY_MM_DD-HH_MM_SS>-<note>`; `use_sim_time:=true`) | The simulator's mavros shim publishes GPS from the world's anchor: SimGps stays **disarmed** (N8). |
| **Bag replay** ("From a ros2bag") | `Replay_launch.py bag:= rate:= [world_dir:=] [placement:=origin]` | none | Below. |

A simulated run's log note always carries the `sim` marker (`LaunchParameters.wire_note`).
Every Gazebo controller launch asks for the sea state after the world dialog.

**Run session (every real-robot launch).** The boat connects and goes into override, but
`master_control` idles — Manual Target and the arrow keys drive it, nothing is followed or
logged — until **START RUN** (RUN box, pinned under the left panel). **STOP RUN**: thrust zero,
still in override, the run's CSV / report / `.npy` / ros2bag filed by the control stack.
Between runs **Update Mission** (the Launch button's session label) chooses the next run's
designer path or the pinger, the controller (MPC loads on a worker thread; START RUN waits;
MPC cannot follow the pinger) and the note. **Turn Off Connection** (the Stop button's label)
is N1, an active run stopped first. `run_session.py` owns the `seq`-acknowledged protocol and
`start_blockers()`, and the **one file `path_generation` watches**,
`LaunchConfig.session_trajectory_file` (`~/.config/blueboat_mcs/trajectories/.session/run.yaml`),
rewritten atomically just before each `start` (N2b). Designer paths only — built-in shapes are
simulation / CLI. Histories record only during a run (`DataStore.recording()`), so trail,
statistics, plot and timeline freeze between runs.

**Bag replay.** The dialog validates the folder (a bag with a pose topic); the world dialog
offers the worlds the bag's GPS track touches (`worlds.eligible_worlds_for_track`), or every
world with *Place the run on the world's origin*. No sea-state dialog, no companion: the
recording holds the real sea and the replayed boat takes no force. The replay publishes the
**recorded** GPS, compass and odom, so `map_frame_ready()` waits for a real anchor and SimGps
stays disarmed. The REPLAY box (top-left, never together with SEA STATE) shows run, time,
motion (attitude source / rate, amber under 10 Hz) and status, with Pause/Resume and rate.
Manual Target and the Default/Override toggle are greyed. **The station never starts a sonar
for a replay** (the operator starts the simulator's sonar; the GCS can).

**Sea state (simulation).** `mcs.core.sea`: `CurrentSetting`, `WaveSetting` and the advanced
`SeaTimeline` of `Segment`s, gathered in `SeaChoice` (+ seed). They emit **the simulator's own
schedule blocks with plain keys**; the simulator resolves them (`sea/presets.py`) — the
station does no sea physics (the "automatic" wave period it shows is a display hint mirroring
the simulator's rule). Labels and descriptions come from the simulator's installed
`share/blueboat_sss_sim/config/sea_states.yaml` (`SeaCatalog`, never raises; built-in names
when absent); literature references are not shown. A non-null choice is written to
`~/.config/blueboat_mcs/sea_states/last_launch.yaml` and passed as `sea_schedule:= sea_seed:=`.
Saved plans live in the same folder. The SEA STATE box's Status row turns amber
"· hull: Gazebo box" when the readback says the URDF box floats the boat.

**Link guardian (real-robot sessions).** An ArduPilot Lua script (superproject
`link_guardian/`): after `LGRD_TIMEOUT` (15 s) of laptop silence it retraces the boat's trail,
and on the first laptop heartbeat hands control back (HOLD → servo mapping → MANUAL), so the
run goes on. **ON by default**: `robot_interface` arms the autopilot at connection before
override, and the station starts `guardian.py watch` (sets `LGRD_ON`, moves HOME to each run
start, serves the GUARDIAN box, top of the left column; rules in Qt-free
`core/guardian.py::summarize`). Buttons Check / Apply settings / Turn OFF-ON / Arm again /
Take back control (a fallback, not gated on the run), each enabled only when it can act. The
E-STOP does not turn it off. **Turn Off Connection and app exit always ask in a real
session** (`needs_turn_off_choice`): *Guardian OFF + disarm* (`guardian_off_and_disarm`:
`off` → ack → `disarm` → `/mavros/state` disarmed; nothing terminated if OFF is unconfirmed),
*Keep ON* (the boat retraces its path), or *Cancel*; then N1.

**BlueBoat-Perception.** Started with every real-robot launch and every personalized-world
launch, best effort; the station subscribes to none of its topics and imports nothing from it
(`PerceptionConfig`).

**Sonar model and slow-down (owner order, 2026-11-07).** Launch dialog, Gazebo + Controller:
**Sonar model** Geometric (default) / Ray-cast, and for the ray-cast **Slow-down**
x0.010-x4.000 (x0.500 until the owner order of 2026-11-10; `SimSpeedConfig`; `LaunchParameters.sonar_renderer` / `real_time_factor`). The
ray-cast renders far slower than its 50 ms ping period, so the whole simulation runs at that
real-time factor on one clock (Gazebo `-z`; control `time_source: ros`; processor and
Perception on `use_sim_time`; GCS and station clocks sim-paced): 20 Hz pings in sim time, and
every record as at real speed. Only the world branch of `to_cli()` emits the two arguments,
and only for the ray-cast (a geometric command line is unchanged). No world chosen → the
empty world has no sonar: geometric at x1, said in a message box
(`drop_sonar_choice_without_world`). The toolbar's **SIM** badge shows the requested and the
measured factor (`core/clock.sim_badge`; amber beyond `rtf_warn_rel` or when `/clock`
stops). Smoke block `sim speed ok`. **Ray-cast variants (owner order, 2026-11-09):** after the
unchanged Geometric / Ray-cast items, the menu lists every simulator profile with a top-level
`variant:` block (`core/sonar_variants.py`: YAML only, never an import of SSS-Sim, CM-3; searched
in `SimSpeedConfig.variant_dirs`, the installed `blueboat_sss_sim` share, then the superproject
checkout), its `description` as the item tooltip; chosen, `LaunchParameters.sonar_profile` is
emitted as `sonar_config:=<path>` after `real_time_factor:=` (plain Ray-cast and geometric command
lines unchanged; Extra args still win). Today: *Ray-cast — live* (first-order multipath,
bit-identical) and *— fast* (also no diffraction, identical by eye). The simulator also pauses
Gazebo while a slow ping renders (its `sonar_hold_sim`, on by default), so the hint now says x0.500
is the fastest setting; the SIM badge's measured factor drops during those pauses.

### Frame conventions (the recurring source of bugs)
- Manual target **world** on the wire; pinger **body**. `master_control` converts the manual
  target via `inRobotFrame()` before `solve_LoS()` and passes the pinger through, so both are
  robot-frame at the solver — consistent by design.
- Yaw everywhere in the app: radians, CCW-positive about +z, y-up world. Odometry yaw is
  absolute ENU; `DataStore.robot_true_heading()` prefers the compass, falls back to the odom
  yaw, with no rotation correction anywhere (`docs/GPS_MAP_ARCHITECTURE.md`).

---

## 3. NON-NEGOTIABLE constraints

Violating any of these breaks another module, hardware safety, or field data.

**N1 — Safe shutdown before terminating a launch.** Stop Mission (**Turn Off Connection** in a
session) and app exit — the only two paths that terminate anything — run the
`command_center` sequence: (in a session, stop an active run via `/blueboat/run_command`) →
publish `default` on `/blueboat/input_str` → check a matched subscriber
(`get_subscription_count()`) → wait for the `/blueboat/param_mode` echo (one republish at T/2)
→ flush delay → *only then* terminate. Killing the launch first can leave the motors in
override. The wait is `shutdown_confirm_timeout_s` (25 s, above `param_set`'s own 20 s bound;
a shorter wait killed `param_set` mid-sequence, leaving SERVO1/3 on passthrough and the GCS
sysid on mavros). A **terminating** sequence still unconfirmed **asks the operator**
(`shutdown_unconfirmed`: *Wait again* republishes, *Terminate anyway* flushes and terminates;
a late echo still confirms) — nothing is killed before they answer. After *Guardian OFF +
disarm*, a confirmed `default` is followed by `manual` (unless `/mavros/state` already reads
MANUAL) and a bounded wait (`guardian.manual_confirm_timeout_s`); never after *Keep ON*. In a
simulation graph (`set_simulation_mode(True)`) the acknowledgement wait is skipped (no
`robot_interface` / `param_set` exists), but the publish still precedes flush and terminate.
Because `param_set` heartbeats, the confirming echo must be a **transition** into `default`:
the sequence snapshots the last mode before publishing and ignores repeats; a boat already in
`default` is reported as its own level (`already-default`).

**N1b — E-STOP cuts the motors and does nothing else.** Three separate operator actions,
never re-nested:

| Button | Publishes | Acknowledged by | Terminates? |
|---|---|---|---|
| **E-STOP** | `stop` | `controller_ready` → `False` | no |
| **E-STOP + Stop Override** | `stop`, then `default` | the above, then the `param_mode` transition | no |
| **Release E-STOP** (session, motors enabled, latched) | `enable` | `controller_ready` → `True` | no |
| **Stop Mission / Turn Off Connection / app exit** | (run `stop`), `default`, `manual` after *Guardian OFF* | the `param_mode` transition (unconfirmed: the operator decides) | yes, after confirmation |

`stop` zeroes thrust, closes the motor gate, disarms and **latches** until `enable`, all
without leaving override. Leaving override *transfers* authority (the RC channels go back to
whatever transmits), so it is never part of the panic button. The E-STOP acknowledgement has
its own state machine: it neither blocks nor is blocked by an in-flight `default` sequence.

**N2 — `[0, 0]` on `/blueboat/manual_target` is a control-handover sentinel**, never a
position: a genuine click at the origin is nudged by `1e-3`; only Continue Original Mission
(**Release Target** between runs) publishes it. Arming Manual Target publishes nothing.

**N2b — A real-robot launch never moves the boat or logs before START RUN, and a run's path is
written before its start.** The station writes the session file (GPS-deployed with the
*current* fit, or a non-anchored mission placed at the boat *now* — never at the launch
point) atomically, and only then sends `start`, so `master_control`'s first window after the
start is the new path. Never `start` first.

**N2c — Arrow control never hands the boat back to autonomy by itself.** The dead-man zeroes
thrust and stays engaged; turning the box off holds the boat where released; only Continue
Original Mission resumes a paused run. Keys are captured only while the main window is active
and no modal dialog is open; auto-repeat is ignored; held keys are forgotten on focus loss.

**N3 — Never re-apply a frame correction to `/monitoring_data`.** `x_d/y_d` are world-frame at
the source (`master_control`'s `# --- world-frame monitoring target ---` markers); an app-side
robot→world fixup would double-convert.

**N4 — The pinger world position is computed only in the pinger callback**, with the pose
concurrent with that message, and stays fixed between messages. Re-anchoring a stale body
vector to each new odom pose drags the marker behind the robot.

**N5 — The map view never rotates.** North-up comes from placing items in ENU; rotating the
view rotates the satellite tiles. Only the robot glyph rotates.

**N6 — The robot glyph's heading comes from an absolute source**: compass preferred, odom yaw
(absolute ENU) as fallback. Never re-apply a rotation correction to either.

**N7 — The mission-path preview uses the `/path_request` service directly, never
`path_publisher`**, which only `Sim_launch.py` starts (no `/set_path` in a real-robot run).

**N8 — A GPS-anchored mission is never deployed without a valid translation anchor.**
`io_yaml.deploy_mission()` raises when `current_fit` is None; the watcher polls
`geo.is_valid`, which converges from the first few fixes with **no vehicle motion**. Deferred
deployment runs on real water and in Gazebo runs of GPS-anchored missions (`gps_simulated`:
fixes synthesised by the station, spawn at the fixed `LaunchConfig.sim_spawn_yaw_deg`); only a
sim launch of a non-anchored mission executes design-frame points directly. In a
personalized world the fixes come from the simulator's mavros shim and the station's SimGps
stays **disarmed** — a second publisher on `/mavros/global_position/global` with an arbitrary
origin would deploy the path into the wrong frame. That launch file has no `spawn_yaw` (spawn
at world (0, 0) heading east).

**N8b — A simulated run is reproducible: same spawn pose, same fix noise.** The spawn heading
is a configured constant and `SimGpsModel` is seeded with `SimGpsConfig.noise_seed`; the fix
noise feeds the anchor estimate, so an unseeded generator would deploy the same mission
differently every run. Another orientation is a *choice*: `spawn_yaw:=` in Extra args
(appended last, so it wins) or the config value.

**N9 — Keep `mcs/` free of control-stack code.** Control-stack changes belong in
`BlueBoat-Control` (§6); nothing under `mcs/` imports `blueboat_control` at runtime.

**N10 — Never weaken a comparison baseline or overstate evidence** in anything produced for
the thesis (`project_synthesis.md` §4/§8.6: the two-pass orthogonal baseline is tuned
seriously; simulation-derived results are model-conditional).

---

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

## 7. Commands and gates

```bash
cd ~/ros2_ws && source env.sh                        # venv + ROS 2 + overlay
cd src/BlueBoat-SideScanSonar/BlueBoat-MCS
pip install -r requirements.txt                      # first time; not --user in the venv
python3 run.py [--config /path/config.json] [--verbose]   # build.sh: colcon build, then this
```

```bash
QT_QPA_PLATFORM=offscreen python3 smoke_test.py      # regression gate
ruff check .                                         # lint gate (pinned ruff==0.16.5, ruff.toml)
```

```bash
# CLI launches (the station's real-robot launch is a session; the CLI may omit session_mode)
ros2 launch blueboat_control BlueBoat_launch.py session_mode:=True enable_motors:=True \
    controller_type:=PID use_pinger:=False \
    trajectory:=from_yaml:$HOME/.config/blueboat_mcs/trajectories/.session/run.yaml
ros2 launch blueboat_control Sim_launch.py robot_file:=thrusters_ur controller_type:=MPC \
    trajectory:=circle note:=sim-tuning
ros2 launch blueboat_sss_sim full_mission_launch.py world_dir:=$HOME/worlds/<path>/<world> \
    with_control:=true controller_type:=PID note:=sim-dam \
    trajectory_file:=$HOME/.config/blueboat_mcs/trajectories/.deployed/<name>.yaml
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: default"
```

**`smoke_test.py`** is a linear script, not a framework: it imports every module, builds the
window offscreen, drives synthetic telemetry through the `SignalBus`, prints one `… ok`
checkpoint per block (block names say what each pins — N1/N1b `safe shutdown ok`, N2
`sentinel ok`, N2b `placement ok`, N2c `arrow ok`, N4 `pinger anchor ok`, N8/N8b
`deploy guard ok` / `launch dialog sim-gps ok`, the run session, replay, guardian, perception,
`helpers ok` for the shared helpers) and aborts on the first failed assertion; success ends
with `SMOKE TEST PASSED`. **Environment it needs** (else it aborts at the first such block,
not a regression): the `sea state ok` timeline asserts need the simulator's installed
`sea_states.yaml`; the bridge-node path-service block imports `bridge_node`, hence
`nav_msgs` / `sensor_msgs` / `std_msgs` (a sourced ROS 2); `guardian ok` needs the
superproject's `link_guardian/laptop/guardian.py` beside this repository. Uncovered: most of
the designer UI (`designer_map.py`, `panels.py`, `designer_window.py`, the `_push_undo`
contract). A green run is not field readiness — it cannot catch QoS mismatches, message-type
drift or launch-argument errors.

**Lint.** Default rule selection, suppressions with reasons in `ruff.toml`; the broad
`except Exception` clauses carry `# noqa: BLE001` with a reason and stay broad. **No
type-checker, deliberately** (a mypy survey found 45 errors and no real defect; re-open only
with new evidence).

---

## 8. Documentation set

`docs/` holds the long-form explanation; this file stays authoritative where they disagree.
There is no `TODO.md`: what remains is field verification, which this repository cannot settle.

| Doc | Content |
|---|---|
| `03_ros_integration.md` | Every topic / command / service, launch targets, stop actions, the safe-shutdown sequence; §Observations = open control-stack items and the fixes a stale install reintroduces. |
| `04_user_guide.md` | Every screen and control, incl. run session, arrow control and the designer. |
| `05_trajectory_format.md` | The `blueboat_trajectory/1` contract, deferred GPS deployment, the session file. |
| `GPS_MAP_ARCHITECTURE.md` | The self-contained GPS-only map model (§4). |
