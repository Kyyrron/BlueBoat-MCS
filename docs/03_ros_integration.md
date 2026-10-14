# 3 — ROS2 Integration Guide

Complete contract between the station and the existing BlueBoat stack.
Topic names are configurable (`config/settings.py` → `TopicsConfig`); the
defaults below match the stack as provided.

## Subscribed topics

| Topic | Type | Producer | Used for |
|---|---|---|---|
| `/blueboat/odom` | `nav_msgs/Odometry` | `robot_interface.py` | World pose in **local ENU** (origin = launch point, axes East/North, yaw **absolute** ENU — 0 = East, CCW+), heading fallback, speed, trajectory, travelled distance, georeference input |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | MAVROS | Robot GPS read-out, georeference input (BEST_EFFORT QoS) |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | MAVROS | **Robot glyph heading (preferred source)** — degrees clockwise from north (0=N, 90=E), converted to the app convention (radians CCW from east) as `radians(90 - hdg)`. The fallback is the odom yaw directly, which is absolute ENU too. BEST_EFFORT QoS |
| `/mavros/state` | `mavros_msgs/State` | MAVROS | FCU **connected** flag (one of the three readiness inputs); since v1.1 also `armed` (the **autopilot's** arming, not the motor gate) and `mode` for the GUARDIAN box. Only subscribed when `mavros_msgs` imports; absent in simulation |
| `/mavros/battery` | `sensor_msgs/BatteryState` | MAVROS (`sys_status`) | Battery charge in the left panel's ROBOT section. `percentage` is a **0..1 fraction** (MAVROS divides the MAVLink percent by 100); `percentage`/`voltage` arrive NaN when the FCU reports neither and are then shown as "no data" rather than as a number. BEST_EFFORT QoS. Real boat only — no simulation graph publishes it |
| `/blueboat/pinger_coordinates` | `Float32MultiArray[3]` | `robot_interface.py` | Pinger in **robot/body frame** (sensor-fused); distance; world position computed once per message, with the pose concurrent with it, and held fixed in between (§3 N4) |
| `/uw_gps_data` | `Float32MultiArray[19]` | `uwgps_log.py` | Timestamp of the last raw USBL packet ("Last update" field) |
| `/monitoring_data` | `Float32MultiArray[9]` `[t,x,y,ψ,x_d,y_d,ψ_d,u1,u2]` | `master_control.py` | **Current path target** `(x_d, y_d)` → target line and robot↔path distance; no recomputation of the controller's target |
| `/thruster_input` | `Float32MultiArray[2]` `[right, left]` (N) | `master_control.py` | Motor command display and history |
| `/blueboat/controller_ready` | `std_msgs/Bool` | `robot_interface.py` | Mission-readiness aggregation |
| `/blueboat/param_mode` | `std_msgs/String` | `param_set.py` | Confirmation echo of `default`/`override`; E-STOP acknowledgement |
| `/blueboat/run_state` | `std_msgs/String` (JSON) | `master_control.py` | **Real-robot run session only.** `{seq, ok, detail, state: idle/loading/running/error, controller_type, use_pinger, note, mission, run, npy}`: the answer to every run command (`seq`) and a 1 Hz heartbeat. Drives the RUN box and START RUN's gating; it is the truth about whether a run is active |
| `/blueboat/run_log` | `std_msgs/String` (JSON) | `robot_interface.py` | **Real-robot run session only.** `{run, logging, csv, layout, bag_enabled, bag_recording, bag}`, 1 Hz: confirms the run's position CSV is being written (RUN box "Log" row; flagged if a started run is not confirmed within 3 s) and, since v1.1, its ros2bag (RUN box "Bag" row, flagged the same way; the `bag_*` fields are absent from an older control stack) |
| `/link_guardian/status` | `std_msgs/String` (JSON) | `guardian.py watch` (superproject `link_guardian/`, started by the station) | **Real-robot session only (v1.1).** 1 Hz: `{connected, armed, mode, want_on, guardian_on, check{ready, script, items[{name, level, text}]}, home{lat, lon, source}, run_running, messages[], busy, ack{seq, cmd, ok, detail}}` → the GUARDIAN box (`mcs.core.guardian`) |
| `/replay/state` | `std_msgs/String` (JSON) | `bag_replay` (BlueBoat-Control) | **Bag replay only (v1.1).** `{bag, t, duration, rate, paused, finished, frame, world, world_offset, pose_source, gazebo}` at 2 Hz → the floating REPLAY box. Subscribed **TRANSIENT_LOCAL depth 1**, like the sea state |
| `/sim/sea_state` | `std_msgs/String` (JSON) | BlueBoat-SSS-Sim `sea_state_node` | **Simulation only.** The live sea situation (current mean / instantaneous speed and direction at the boat / variation level, waves strength / Hs / Tp / from-bearing / status / passing wake, the boat's roll / pitch / heave std over 30 s, next planned change or blend progress, summary) shown in the floating SEA STATE box (top-left of the map view). Subscribed **TRANSIENT_LOCAL depth 1** to match the node's latched publisher, so a station started after the simulator still gets the current situation at once |

## Published commands

| Topic | Type | Payload | Effect (control stack) |
|---|---|---|---|
| `/blueboat/input_str` | `std_msgs/String` | `"default"` | `robot_interface.str_input_callback` → forwarded to `param_set` → safe parameters; echoed on `/blueboat/param_mode`. Published by **Stop Mission** / app exit (the safe-shutdown sequence), by **E-STOP + Stop Override**, and by the Default/Override toggle. **Not** by E-STOP alone. |
| `/blueboat/input_str` | `std_msgs/String` | `"override"` | Direct-control parameters; the other state of the toggle button. |
| `/blueboat/input_str` | `std_msgs/String` | `"stop"` | **The emergency primitive.** `robot_interface.full_stop()` zeroes the thrust, closes the motor gate, disarms and **latches** until an explicit `enable`. Stays in override and terminates nothing. Published by both E-STOP buttons. |
| `/blueboat/input_str` | `std_msgs/String` | `"enable"` | `robot_interface.release_estop()`: clears the E-STOP latch **and turns the motor gate on**. Published by **Release E-STOP**, offered only in a run session launched with motors enabled, while the latch is on. |
| `/blueboat/input_str` | `std_msgs/String` | `"rearm"` | **Real-robot session (v1.1).** `robot_interface` re-runs its arm-before-override sequence (`default` mapping → arm → `override`; refused during a run or while the E-STOP latch is on). GUARDIAN box **Arm again**. |
| `/blueboat/input_str` | `std_msgs/String` | `"disarm"` | Disarms the autopilot. Published only by the Turn Off Connection / exit choice "Guardian OFF + disarm", after the guardian is confirmed OFF. |
| `/link_guardian/command` | `std_msgs/String` (JSON) | `{"cmd": "on"\|"off"\|"check"\|"setup"\|"reclaim", "seq": n}` | **Real-robot session only (v1.1).** To `guardian.py watch`: the guardian flag `LGRD_ON` on the autopilot, the readiness check, the recommended failsafe parameters, and Take back control (HOLD → servo mapping back to `param_set`'s → MANUAL). Answered in `/link_guardian/status.ack`. |
| `/blueboat/manual_target` | `Float32MultiArray[2]` | `[x, y]` world metres | `master_control` steers to it with LoS, overriding the mission. |
| `/blueboat/manual_target` | `Float32MultiArray[2]` | `[0.0, 0.0]` | Sentinel: `master_control` resumes the original mission (between the runs of a session: just releases the target). Published **only** by the explicit **Continue Original Mission** / **Release Target** action (`main_window._on_continue_mission`); arming or disarming Manual Target publishes nothing, and a genuine origin click is nudged by `1e-3` (N2). Turning arrow control off publishes the boat's own position here (hold where released). |
| `/blueboat/run_command` | `std_msgs/String` (JSON) | `{"cmd": "configure", "controller_type": "PID", "use_pinger": false, "note": "dock", "mission": "lawn_a", "seq": 7}` · `{"cmd": "start", "seq": 8}` · `{"cmd": "stop", "seq": 9}` | **Real-robot run session only** (`master_control`). `configure` sets the next run (refused during a run; MPC with the pinger refused; MPC built on a worker thread → `state: loading`), `start` starts following + logging (refused unless `state: idle`), `stop` ends the run. `configure` is re-sent until acknowledged; `start` / `stop` are sent once (RUN box, Turn Off Connection). |
| `/blueboat/arrow_thrust` | `Float32MultiArray` | `[right, left]` N, or `[]` | ARROW CONTROL box, 20 Hz while on (`[0, 0]` with no key held); `[]` releases. `master_control` gives it priority over every branch, freezes `tau` (a run is paused) and keeps logging the last target; a gap longer than `arrow_timeout` (0.5 s) commands zero and stays in arrow control. Any manual target also releases it. Both graphs (simulation too). |
| `/replay/command` | `std_msgs/String` (JSON) | `{"cmd": "pause"}` · `{"cmd": "resume"}` · `{"cmd": "rate", "rate": 2.0}` | **Bag replay only (v1.1).** The REPLAY box's Pause/Resume and rate combo; `bag_replay` freezes or re-paces its bag clock (the boat holds its pose while paused). |
| `/sim/sea_state/command` | `std_msgs/String` (JSON) | `{"current": {"speed_mps": 0.3, "from_deg": 0, "variation": "natural"}, "waves": {"preset": "smooth", "from_deg": 45, "crest": "short", "wakes": "occasional"}, "ramp_s": 20}` (a disabled card sends `{"preset": "none"}` / `{"preset": "calm"}`; a custom height sends `hs_m` + `tp_s: "auto"`) or `{"schedule": <blueboat_sea_schedule/1>, "t0": "now"}` | **Simulation only.** "Change sea state…" in the floating SEA STATE box; the simulator resolves the plain-language keys and ramps to the new situation, never steps. |

Equivalent shell commands (what the buttons do):

```bash
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: default"
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: override"
ros2 topic pub --once /blueboat/manual_target std_msgs/msg/Float32MultiArray "data: [12.5, -3.0]"
```

Because `(0,0)` is the reserved resume sentinel, a click exactly on the world
origin is nudged to `(0.001, 0)` before publishing.

## Services

`/path_request` (`blueboat_interfaces/RequestPath`): the station requests
`linspace(0, total_time, n)` and renders the returned `nav_msgs/Path` as the
mission-path layer, with a circled **S** on its first point (where the run
begins). In simulation it is requested at launch (path_generation always runs).
In a real-robot run session it previews the session file: a GPS-anchored mission
as soon as it is deployed (at connection, or after Update Mission), a non-anchored
one only once START RUN has placed it at the boat; between runs of a placed
mission the preview is cleared, and nothing is previewed in pinger mode.

The request is **held until the map frame is anchored**
(`MainWindow._on_tick` keeps it in `_pending_preview_trajectory` until
`store.map_frame_ready()`): a few GPS fixes on real water (no motion needed),
immediately in simulation. A briefly empty map right after a real-water
launch is expected, not a fault.
The horizon is `launch.path_preview_total_time_s` (120 s by default); a
designer trajectory's own `duration_s` replaces it automatically, so long custom
missions are previewed completely.

The station calls the service itself rather than subscribing to `/set_path`,
because `path_publisher.py` is started only by `Sim_launch.py` and by the
simulator's `full_mission_launch.py` — there is no `/set_path` in a real-robot run,
so depending on it would break every real-robot run (`CLAUDE.md` §3 N7).

## Mission launch

```
ros2 launch blueboat_control BlueBoat_launch.py \
    enable_motors:=<bool> trajectory:=from_yaml:<session file> \
    use_pinger:=<bool> controller_type:=<PID|LoS|MPC> [note:=<str>] \
    session_mode:=True [extra:=args]
```

**Every real-robot launch is a run session (2026-10-09).** The boat connects and
goes into override (`robot_interface`'s handshake; since v1.1 it first **arms the
autopilot** in the normal mapping — ArduPilot refuses to arm on passthrough — for the
link guardian, and proceeds anyway after `arm_timeout`), then `master_control`
**idles**: Manual Target and the arrow keys drive the boat, nothing follows a path or
the pinger, `robot_interface` writes no CSV and `master_control` no `.npy`. The
lifecycle, all on `/blueboat/run_command` / `/blueboat/run_state`:

1. **Connect** — the first `run_state` heartbeat makes the station send `configure`
   with the dialog's choices; START RUN waits for its acknowledgement.
2. **START RUN** — the station writes the run's path into the session file
   (`LaunchConfig.session_trajectory_file`, `~/.config/blueboat_mcs/trajectories/.session/run.yaml`,
   which `path_generation` watches): a GPS-anchored mission re-deployed with the
   current fit, any other mission **placed at the boat** (`io_yaml.place_mission`). Only
   then does it publish `start`; `master_control` resets τ and its controller state,
   reserves the run's `.npy` and starts following at once; `robot_interface` opens the
   run's CSV on the `running` state and reports it on `/blueboat/run_log`.
3. **STOP RUN** — `stop`: thrust zero, still in override; the `.npy` is written, the CSV
   closed and filed (report PNG, `Robot_data/<stem>/`) in a background process.
4. **Update Mission** (between runs) — `configure` with the next path / pinger,
   controller and log note. A controller change is loaded at once (MPC on a worker
   thread, the loop keeps running); an anchored path is deployed and previewed at once,
   a non-anchored one is placed only when its run starts.
5. **Turn Off Connection** — an active run is stopped first, then the safe-shutdown
   sequence below, unchanged.

**Link guardian companion (v1.1).** With every real-robot launch the station also
starts `python3 <superproject>/link_guardian/laptop/guardian.py watch`
(`launch_manager.guardian_command`, `GuardianConfig.script`; console prefix
`[guardian]`, stopped with the launch like the sea companion). It switches the
on-boat guardian ON (`LGRD_ON = 1`), moves HOME to each run's start and back
(`/mavros/cmd/set_home`), and serves the GUARDIAN box over `/link_guardian/*`.

The station passes `session_mode:=True` on every real launch; without it (CLI)
`BlueBoat_launch.py` keeps the one-launch-one-mission behaviour. Built-in shapes are
not offered for a session: only designer missions can be placed or deployed into the
session file.

**Run bags (v1.1, 2026-10-13).** Every run of a session also records a ros2bag —
every topic except `/side_scan_sonar/*`, `/sss_processor/*`, `/sss_ai/*` (the `.svlog`
is the sonar's record) — by a separate `ros2 bag record` process that `robot_interface`
pre-arms paused between runs and resumes the instant a run starts. At STOP RUN it is
closed and filed with the CSV: `Robot_data/<stem>/<date>-<note>-bag/`, plus
`<date>-<note>-path.yaml` (the mission the run followed). `record_bag:=false` on the
launch disables it; the station never does. The RUN box's **Bag** row follows it.

**Bag replay — Gazebo Source "From a ros2bag" (v1.1)** runs:

```
ros2 launch blueboat_control Replay_launch.py bag:=<run folder or its bag> \
    rate:=<x> [world_dir:=<personalized world>] [extra:=args]
```

`bag_replay` moves the boat along the recording with Gazebo's `set_pose` (the boat is
spawned with `blueboat_description replay:=true`: no force can act on it, so no sea
state is offered and no companion starts) and republishes the recording: the recorded
`/mavros/*` (GPS fixes, compass, IMU, state — the station's SimGps stays disarmed),
`/blueboat/odom` rebuilt with the full attitude, `/monitoring_data`, `/thruster_input`,
the pinger topics and `/set_path`; commands and session plumbing go under `/replay/`.
The map therefore anchors on the recorded GPS exactly as on real water
(`map_frame_ready()` waits for `geo.is_valid`). Stop Mission runs the simulation
shutdown sequence (`set_simulation_mode(True)`).

**Gazebo simulation alternative** — the dialog's "Gazebo simulation" mode
runs instead:

```
ros2 launch blueboat_control Sim_launch.py \
    robot_file:=<name> trajectory:=<name> controller_type:=<PID|LoS|MPC> \
    note:=<str> [spawn_yaw:=<radians>]
```

`note:=` is the same argument as on the real boat and the dialog's **Log
note** field feeds it in both modes: `simulation_interface` names its
position log `{date}-{note}-poslog.csv` exactly as `robot_interface` does.
A simulated run's note always carries the `sim` marker — `note:=sim` with
an empty field (`Sim_launch.py`'s own default, so that case is unchanged),
`note:=sim-<note>` with one — and the text is sanitised into a single
filename-safe token (`LaunchParameters.wire_note` /
`launch_manager.sanitize_note`).

That graph consists of the Gazebo world, `simulation_interface.py`
(publishes `/blueboat/controller_ready` and `/monitoring_data`, consumes
`/thruster_input` and `/blueboat/odom` from Gazebo), `path_generation`,
`path_publisher` and `master_control` — and notably **no** MAVROS,
robot_interface, param_set or pinger nodes. The station adapts coherently:
readiness gating drops the FCU check, the mission path is always requested
(path_generation always runs), manual targets work unchanged
(`master_control` subscribes regardless), and the safe-shutdown sequence
skips the `param_mode` acknowledgement wait, which structurally cannot
arrive (the command is still published; the publish-before-terminate
ordering is preserved).

**GPS-anchored missions in simulation** take the same deferred-deploy path
as on real water: the dialog passes a **fixed** `spawn_yaw`
(`LaunchConfig.sim_spawn_yaw_deg`, 0° = east by default; the boat always
spawns at Gazebo (0, 0)) and the station's own bridge node
synthesises the GPS feed — a 5 Hz timer converts the sim odom (world
metres) to lat/lon about a receiver origin placed
`SimGpsConfig.offset_north_m` (10 m) north of the mission's first point,
publishing `sensor_msgs/NavSatFix` on `/mavros/global_position/global`
(nothing else publishes it in the sim graph) with configurable Gaussian
noise drawn from a **seeded** generator (`SimGpsConfig.noise_seed`).
Spawn pose and fix noise are both deterministic deliberately: the boat
starts at the same pose and the anchor converges to the same translation
on every run of a mission, so two simulated runs are comparable and a
difference between them is the change under test. One run can still be
started from another heading with `spawn_yaw:=<radians>` in Extra args,
which is appended last and therefore wins. Its own subscription receives the fixes back, so anchoring,
satellite tiles, diagnostics and the deferred deployment run identically
to a field trial — which is the point: an offline-planned GPS path can be
rehearsed in sim before the real-world session. Non-anchored sim missions
simulate no GPS and use the plain identity-anchor map.

**Personalized Gazebo worlds (third launch target)** — a GPS-anchored sim
launch additionally offers the generated world folders under
`~/worlds/<path_name>/<world_name>/` (BlueBoat-SSS-Sim's World Builder
output) whose `metadata.yaml` limits contain at least one GPS point of the
selected path. Choosing one runs, instead of `Sim_launch.py`:

```
ros2 launch blueboat_sss_sim full_mission_launch.py \
    world_dir:=<abs world folder> with_control:=true \
    trajectory_file:=<trajectories>/.deployed/<name>.yaml \
    controller_type:=<PID|LoS|MPC> note:=<str>
```

With `with_control:=true` that launch starts the same control graph as
`Sim_launch.py` (`simulation_interface`, `path_generation` watching the
deployed file, `path_publisher`, `master_control`) plus the simulated
side-scan sonar and a **mavros shim** that publishes
`/mavros/global_position/global` (and `compass_hdg`, `imu/data`) from the
world's own GPS anchor. The station therefore does **not** arm its own
simulated GPS in this mode — two publishers on the fix topic, with the
station's arbitrary receiver origin, would deploy the trajectory into the
wrong frame relative to the world geometry. The GeoReferencer converges on
the shim's fixes and the deferred deploy lands the path in the Gazebo
world frame at its true GPS location. Limitations: `full_mission_launch.py`
declares no spawn arguments, so the boat always spawns at world (0, 0)
(the first waypoint of the path the world was built from) heading east —
launching a *different* path in a world means a controller transit to its
start — and the `spawn_yaw` of the empty-Gazebo mode does not apply (that
spawn is deterministic too). It does declare `note` and `data_dir`
(added 2026-09-13 in `BlueBoat-SSS-Sim`, forwarded to
`simulation_interface`), so the **Log note** reaches this target as well. Requires `blueboat_sss_sim` built in the sourced workspace;
`ros2 launch` fails fast otherwise and the launch manager returns to idle.

**Sea state (2026-09-03; plain-language model 2026-10-08).** Every Gazebo
launch — empty world or personalized world — ends with a third dialog
choosing the current and the waves (`mcs/core/sea.py`: a Current card, a
Waves card, optionally a timeline of segments). Every non-null choice is
written as the simulator's schedule document
(`~/.config/blueboat_mcs/sea_states/last_launch.yaml`,
`blueboat_sea_schedule/1`: `hold` keyframes with the plain keys
`variation`, `tp_s: auto`, `crest`, `wakes` and per-keyframe `ramp_s`,
which the simulator resolves) and rides the launch as
`sea_schedule:=<file> sea_seed:=<n>` — in a personalized world on
`full_mission_launch.py`, placed before the operator's extra arguments.
`Sim_launch.py` declares no sea argument, so in the empty world the
station spawns a **companion** process,

```
ros2 launch blueboat_sss_sim sea_state_launch.py world_name:=ocean \
    sea_schedule:=…/last_launch.yaml sea_seed:=…
```

in its own session (stdout prefixed `[sea]` in the console, signalled
with the mission on Stop/E-STOP, its failure never aborts the mission).
Since 2026-10-09 it starts for every empty-world launch, a null choice (no
current, calm) included — then with only `world_name:=ocean` — because the
simulator's catamaran hydrostatics carry the hull in every run (and a live
"Change sea state…" needs a listener). The readback's additive
`hull.model` (`catamaran` | `gazebo_box`) is shown as an amber
"hull: Gazebo box" on the SEA STATE Status row when the box is in charge.
The preset table is read from the simulator's
installed `share/blueboat_sss_sim/config/sea_states.yaml` (CM-3: a file,
never an import; built-in names when it is missing); timelines are saved
under `~/.config/blueboat_mcs/sea_states/`.

Started in its own process session; stopped with SIGINT to the group
(graceful, propagated by `ros2 launch`), escalating to SIGTERM after
`launch.sigint_timeout_s` and SIGKILL after `sigterm_timeout_s` more.
Coupling rules encoded in the dialog: `trajectory` is disabled when
`use_pinger` is checked (no `path_generation` node in that branch), and
`enable_motors` always requires explicit re-confirmation.

## The three operator stop actions (2026-09-04)

They are separate on purpose. Before this they were nested — E-STOP published
`default`, which dropped out of override, and "E-STOP + Stop Override" also
killed the launch — so "cut the motors", "give the servo mapping back" and "end
the mission" could not be asked for one at a time.

| Button | Publishes | Acknowledged by | Terminates the launch? |
|---|---|---|---|
| **E-STOP** | `stop` | `/blueboat/controller_ready` → `False` | no |
| **E-STOP + Stop Override** | `stop`, then `default` | the above, then the `param_mode` transition | no |
| **Release E-STOP** (run session, motors enabled, latched) | `enable` | — (`controller_ready` returns to `True`) | no |
| **Stop Mission** / **Turn Off Connection** (run session), **Application Exit** | (`stop` on `/blueboat/run_command` if a run is active, then) `default` | the `param_mode` transition | yes, after confirmation |

E-STOP + Stop Override is **not** Turn Off Connection: it disarms and latches
(`stop`) before handing the servo mapping back (`default`) and leaves every node
running, so the session survives; Turn Off Connection does not disarm and ends the
session. After either E-STOP the latch holds until **Release E-STOP**; after E-STOP +
Stop Override also publish `override` again (mode button) to take the thrusters back.

`stop` is the emergency primitive. In the control stack, `robot_interface.full_stop()`
zeroes the thrust, closes the `enable_motors` gate, disarms, publishes
`controller_ready=False` immediately (that is the acknowledgement the station
waits on) and **latches** until an explicit `enable`. The latch is what makes it
stick: without it the next 20 Hz tick would re-apply whatever `master_control`
is publishing, 50 ms later.

It deliberately does **not** leave override. Leaving override is a *transfer* of
authority — `SERVO1/3_FUNCTION` go back to the stock mapping and the RC channels
to whatever else is transmitting — which is the operator's separate second
action, not part of a panic button.

## Safe-shutdown sequence (normative — guards **every** termination path)

**Link guardian pre-step (v1.1).** In a real-robot session, while the guardian may be
ON (`mcs.core.guardian.needs_turn_off_choice` — a silent companion counts as ON), Turn
Off Connection and Application Exit first ask: *Guardian OFF + disarm* (`off` on
`/link_guardian/command`, wait for its acknowledgement, then `disarm`, then the
sequence below — and if the OFF cannot be confirmed, nothing terminates), *Keep ON*
(the sequence below; the boat then returns HOME on its own), or *Cancel*.

**Stop Mission** (**Turn Off Connection** in a run session) and **Application Exit** —
the only two paths that terminate anything — run the same sequence
(`CommandCenter.safe_shutdown`); nothing in the application terminates nodes outside of
it. In a run session an active run is stopped first (`stop` on
`/blueboat/run_command`, through `CommandCenter.set_before_terminate`), so its CSV is
closed and filed by `robot_interface` itself rather than by the teardown. "E-STOP + Stop Override" runs the
same sequence with termination switched off:

1. Verify the ROS graph: `get_subscription_count()` on the reliable
   `/blueboat/input_str` publisher must show a matched subscription
   (`robot_interface`) — the DDS delivery precondition. A count of 0 is
   reported and the sequence continues (publish + late-discovery hold).
2. Publish `String("default")` — synchronous into the reliable DDS writer.
3. Wait for the end-to-end `param_mode == "default"` echo (proves the
   command was received *and acted on*), up to `estop_confirm_timeout_s`
   (2 s default), republishing once at half-timeout (idempotent).
   Since `param_set` now heartbeats that topic at 1 Hz, the echo must be a
   **transition**: the sequence snapshots the last known mode before publishing
   and ignores repeats, so a heartbeat cannot impersonate an acknowledgement.
   A boat already in `default` is reported as `already-default` — the demanded
   state holds, which is a weaker and honestly-labelled confirmation, not an
   echo.
4. On timeout, hold the process/writer alive for `estop_flush_delay_s` so
   the reliable protocol can complete delivery, and tell the operator which
   confirmation level was obtained.
5. Only then terminate the launch process tree — and only when the caller
   asked for it. `shutdown_sequence_finished` is emitted for asynchronous
   callers (window close waits on it), and `estop_state_changed("idle")`
   clears the toolbar's status label, which used to keep showing the last
   phase text for the rest of the session.

A request that terminates **supersedes** one already in flight that does not, so
pressing Stop Mission while a Stop Override is confirming still ends the
mission. The reverse never happens.

Node termination is never initiated before steps 1–4 complete.

## Observations on the existing stack (flagged, not silently patched)

Control-stack code lives in the `BlueBoat-Control` submodule and is built with
`colcon build` in `~/ros2_ws`;
nothing is copied from this repo (`CLAUDE.md` §6, §3 N9). Anchor on symbol
names, not line numbers — they drift with that module.

**Open — world-frame pinger is not published.** `robot_interface` computes
`corrected_pinger` but only writes it to the CSV and the pinger-GPS
conversion; `/blueboat/pinger_coordinates` carries the **body-frame** vector.
The station derives the world position itself, once per pinger message, from
`/blueboat/pinger_coordinates` + `/blueboat/odom` (§3 N4). To make it a single
source of truth, publish it from `robot_interface` on a new topic and point `TopicsConfig`
at it. Residual marker sluggishness is inherent to the source, not to that
choice: with `fixed_pinger=False` the vector is seeded from the Waterlinked
*filtered* acoustic position (seconds of smoothing) and dead-reckoned with odom
twist between USBL updates.

**Carried deliberately — keep-position semantics.** YAML trajectories clamp at
their final pose forever, so every controller station-keeps at the end of a
custom path. A **manual** target is held the same way: `master_control` gives it
its own arrival state (`manual_keep_location`, hold/reacquire radii) and consults
neither `safety_distance` nor `stopping_sequence` for it. Those two govern the
**pinger branch alone** — `safety_distance` defaults to `-1.0` on the real boat
(arrival check off) and `+1.0` in simulation, and the `stopping_sequence` latch
it arms is cleared on every new manual target. **Do not raise `safety_distance`
without reading `CLAUDE.md` §6 first.**

**Carried deliberately — MPC self-orbits on `fsin`.** Under MPC on the `fsin`
trajectory the boat locks into a circle near the start while the reference runs
ahead. The `fsin` reference is a chain of near-closed ~2 m loops (364.75°
heading swing per half-cycle) and the MPC cost/governor settles into a
self-orbit once displaced. Mechanism and remedies:
§0.3 ("MPC on `fsin` — orbit limit cycle") of `BlueBoat-Control`'s pre-v1.0
`TODO.md`, deleted at the freeze — `git show d64ac71^:.claude/TODO.md` in that
submodule. The map is displaying the truth — nothing to compensate here.

**Cosmetic.** `robot_interface` publishes monitoring on the *relative*
`blueboat/monitoring_data` while `master_control` uses the global
`/monitoring_data`; the station follows `master_control`.

### Fixed in `BlueBoat-Control` — but only if the install is rebuilt

These four are fixed at the source. There is no version handshake on a ROS
topic, so a **stale build** in `~/ros2_ws/install` reintroduces each one
silently. Rebuild (`colcon build` in `~/ros2_ws`) at the SHA the superproject
records before any field session, and never compensate for them in the station.

| Was | Symptom on a stale build |
|---|---|
| `/blueboat/odom` published a **hybrid frame** (ENU axes, launch-relative yaw) — the yaw re-zeroing is gone, the frame is now local ENU | GPS map, manual targets and trajectory following all break for any launch heading but East. This was the root cause of "trajectory following only works starting East" |
| `/monitoring_data`'s `x_d/y_d` mixed frames per controller branch — `master_control` now captures the **world** target before `inRobotFrame()` in all five branches | The robot→target line is drawn at a meaningless point (and the no-pinger CSV `target_*` columns are corrupted) while the boat still tracks its path correctly. Looks cosmetic; is not |
| `param_set` could latch `busy` forever when a MAVROS call never returned — now watchdogged, generation-fenced, and heartbeating `param_mode` at 1 Hz | Override never locks: `Waiting for param mode 'override' (current: '')` against `Parameter sequence in progress, ignoring request`, forever. The heartbeat is also why the safe-shutdown echo now requires a *transition* |
| A closed motor gate published **nothing** while `robot_interface` had already requested `override`, so `SERVO1/3_FUNCTION` sat on RCIN passthrough — the gate now streams neutral 1500/1500 | With `enable_motors:=False`, the ESCs follow RC channels 1 and 3 from any transmitter or QGC joystick |
| `robot_interface` / `param_set` ended in a bare `rclpy.spin()` — both now have `try/finally` teardown | Ctrl-C leaves the boat in RC passthrough with nobody streaming, and the position CSV unclosed |

**Path-following speed** is a field measurement, not a defect: path following
uses `los_guidance()` (Fossen lookahead with a path-parameter governor) at
20 Hz, and `solve_LoS` — thrust law `v = 5·ln(0.15·d + 1)`, doubled for manual
targets — is reached only for pinger and manual point targets. Never
compensate for it in the station.

## QoS

MAVROS sensor topics are subscribed BEST_EFFORT (matching the stack); all
custom topics are subscribed default RELIABLE depth 10, matching their
publishers.

`/sim/sea_state` is the one **TRANSIENT_LOCAL** subscription (depth 1):
the simulator latches its status so a late station gets the current
situation immediately; a volatile reader would also match, but would wait
up to half a second for the next 2 Hz message.

One asymmetry worth knowing before "fixing" it: `/blueboat/controller_ready`
is published volatile depth 10 by `robot_interface.py` on the real robot, but
with a **latched (TRANSIENT_LOCAL)** profile by `simulation_interface.py` in
the Gazebo graph. The station's volatile depth-10 subscription is compatible
with both — a volatile subscriber receives from a TRANSIENT_LOCAL publisher —
so the subscription must stay as it is. On the real robot, late subscribers are
covered by `robot_interface`'s periodic re-publish (every 1 s) rather than by
durability.


## Controller rate

The `master_control` loop runs at **20 Hz** — `self.dt = dbl('control_dt', 0.05)`,
a declared ROS parameter, so it is settable per launch. `/monitoring_data` follows
that rate, which is why `diagnostics.expected_hz` is `20.0`.

The position CSV `robot_interface` writes is **not** a station artefact: its column layout
is defined by `BlueBoat-Control/blueboat_control/src/_custom_libraries/robot_log_schema.py`
(`COLUMNS_PINGER` / `COLUMNS_NO_PINGER`), a ROS-free module — read that to consume
the CSV offline.

## Designer trajectories — how `path_generation.py` loads them

The Survey Pattern Designer exports `blueboat_trajectory/1` YAML files
(specification: `05_trajectory_format.md`). The YAML path affects **only the
loading mechanism** of `path_generation.py`; `generate_path()` and every
hard-coded trajectory are untouched.

Nothing is copied from this repo. Both files are committed in the
`BlueBoat-Control` submodule and installed by its `CMakeLists.txt`
(`CLAUDE.md` §6):

* `BlueBoat-Control/blueboat_control/src/_custom_libraries/yaml_trajectory.py`
  — loads `blueboat_trajectory/1` and evaluates it at time `t`; depends only on
  PyYAML and numpy.
* `BlueBoat-Control/blueboat_control/src/_custom_libraries/path_generation.py`
  — carries the loader in `__init__` (parses `trajectory:=from_yaml:<path>` or
  the optional `yaml_path` parameter, falls back to `station_keeping` with an
  error log on failure), the `from_yaml` branch at the top of `single_pose()`,
  and an **mtime watcher** that reloads when the file appears or changes. Until
  the file exists the branch returns a zero pose, which is the station-keeping
  hold that makes deferred GPS deployment possible.

To change either one: edit it in the `BlueBoat-Control` submodule, commit
there, and `colcon build` in `~/ros2_ws`. No launch-file modification is
needed — the YAML path rides inside the existing `trajectory` argument
(`trajectory:=from_yaml:/abs/path.yaml`), which both `BlueBoat_launch.py` and
`Sim_launch.py` already forward to the node.

Station side, saved missions appear automatically in **Launch Mission →
Trajectory → custom paths**; selecting one builds the `from_yaml:` string.
GPS-anchored missions are labelled **“(GPS)”** and follow the deferred
deployment flow of `05_trajectory_format.md`: the node holds a station-keeping
pose at the origin until the station writes the deployed file, which the mtime
watcher then picks up on its next path request. That hold is what makes
deferred, GPS-anchored deployment possible — it is not a hang, and the
operator does nothing: the deploy lands within seconds of the first fixes,
with no driving needed.
