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
| `/mavros/state` | `mavros_msgs/State` | MAVROS | FCU connected / armed / mode. Only subscribed when `mavros_msgs` imports; absent in simulation |
| `/blueboat/pinger_coordinates` | `Float32MultiArray[3]` | `robot_interface.py` | Pinger in **robot/body frame** (sensor-fused); distance; world position computed once per message, with the pose concurrent with it, and held fixed in between (§3 N4) |
| `/uw_gps_data` | `Float32MultiArray[19]` | `uwgps_log.py` | Timestamp of the last raw USBL packet ("Last update" field) |
| `/monitoring_data` | `Float32MultiArray[9]` `[t,x,y,ψ,x_d,y_d,ψ_d,u1,u2]` | `master_control.py` | **Current path target** `(x_d, y_d)` → target line and robot↔path distance; no recomputation of the controller's target |
| `/thruster_input` | `Float32MultiArray[2]` `[right, left]` (N) | `master_control.py` | Motor command display and history |
| `/blueboat/controller_ready` | `std_msgs/Bool` | `robot_interface.py` | Mission-readiness aggregation |
| `/blueboat/param_mode` | `std_msgs/String` | `param_set.py` | Confirmation echo of `default`/`override`; E-STOP acknowledgement |
| `/sim/sea_state` | `std_msgs/String` (JSON) | BlueBoat-SSS-Sim `sea_state_node` | **Simulation only.** The live sea situation (current preset / mean and instantaneous speed / from-bearing, wave preset / Hs / Tp / from-bearing / surface elevation / status, next scheduled change, summary) shown in the floating SEA STATE box (top-left of the map view). Subscribed **TRANSIENT_LOCAL depth 1** to match the node's latched publisher, so a station started after the simulator still gets the current situation at once |

## Published commands

| Topic | Type | Payload | Effect (robot side) |
|---|---|---|---|
| `/blueboat/input_str` | `std_msgs/String` | `"default"` | `robot_interface.str_input_callback` → forwarded to `param_set` → safe parameters; echoed on `/blueboat/param_mode`. Published by **Emergency Stop** and by the mode-toggle button. |
| `/blueboat/input_str` | `std_msgs/String` | `"override"` | Direct-control parameters; the other state of the toggle button. |
| `/blueboat/manual_target` | `Float32MultiArray[2]` | `[x, y]` world metres | `master_control` steers to it with LoS, overriding the mission. |
| `/blueboat/manual_target` | `Float32MultiArray[2]` | `[0.0, 0.0]` | Sentinel: `master_control` resumes the original mission. Published automatically when Manual Target mode is deactivated. |
| `/sim/sea_state/command` | `std_msgs/String` (JSON) | `{"current": {"preset": "strong", "from_deg": 90}, "waves": {"preset": "slight", "from_deg": 45}, "ramp_s": 20}` or `{"schedule": <blueboat_sea_schedule/1>, "t0": "now"}` | **Simulation only.** "Modify situation…" in the floating SEA STATE box; the simulator ramps to the new situation, never steps. |

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
mission-path layer. It is requested for a launch with a non-empty controller
and `use_pinger:=False` (the launch file only starts `path_generation` in that
case), and for the simulation.

The request is **held until the map frame is anchored**
(`MainWindow._on_tick` keeps it in `_pending_preview_trajectory` until
`store.map_frame_ready()`): a few GPS fixes on real water (no motion needed),
immediately in simulation. A briefly empty map right after a real-water
launch is expected, not a fault.
The horizon is `launch.path_preview_total_time_s` (120 s by default); a
designer trajectory's own `duration_s` replaces it automatically.

## Mission launch

```
ros2 launch blueboat_control BlueBoat_launch.py \
    enable_motors:=<bool> note:=<str> controller_type:=<''|PID|LoS|MPC> \
    trajectory:=<name> use_pinger:=<bool> [extra:=args]
```

**Gazebo simulation alternative** — the dialog's "Gazebo simulation" mode
runs instead:

```
ros2 launch blueboat_control Sim_launch.py \
    robot_file:=<name> trajectory:=<name> controller_type:=<PID|LoS|MPC> \
    [spawn_yaw:=<radians>]
```

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
as on real water: the dialog passes a random `spawn_yaw` (boat spawns at
Gazebo (0, 0) with an arbitrary heading) and the station's own bridge node
synthesises the GPS feed — a 5 Hz timer converts the sim odom (world
metres) to lat/lon about a receiver origin placed
`SimGpsConfig.offset_north_m` (10 m) north of the mission's first point,
publishing `sensor_msgs/NavSatFix` on `/mavros/global_position/global`
(nothing else publishes it in the sim graph) with configurable Gaussian
noise. Its own subscription receives the fixes back, so anchoring,
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
    controller_type:=<PID|LoS|MPC>
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
start — and the random `spawn_yaw` of the empty-Gazebo mode does not
apply. Requires `blueboat_sss_sim` built in the sourced workspace;
`ros2 launch` fails fast otherwise and the launch manager returns to idle.

**Sea state (2026-09-03).** Every Gazebo launch — empty world or
personalized world — ends with a third dialog choosing the current and
the waves (presets + compass FROM bearings, optional timeline). In a
personalized world the choice rides `full_mission_launch.py` as
`sea_current:= sea_current_from_deg:= sea_waves:= sea_waves_from_deg:=
[sea_schedule:= sea_seed:=]`, placed before the operator's extra
arguments. `Sim_launch.py` declares no sea argument, so in the empty world
the station spawns a **companion** process,

```
ros2 launch blueboat_sss_sim sea_state_launch.py world_name:=ocean \
    sea_current:=… sea_current_from_deg:=… sea_waves:=… sea_waves_from_deg:=…
```

in its own session (stdout prefixed `[sea]` in the console, signalled
with the mission on Stop/E-STOP, its failure never aborts the mission); a
null choice (no current, calm) starts nothing, keeping that path
byte-identical to before. The preset table is read from the simulator's
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
| **Stop Mission**, **Application Exit** | `default` | the `param_mode` transition | yes, after confirmation |

`stop` is the emergency primitive. Robot-side, `robot_interface.full_stop()`
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

**Stop Mission** and **Application Exit** — the only two paths that terminate
anything — run the same sequence (`CommandCenter.safe_shutdown`); nothing in the
application terminates nodes outside of it. "E-STOP + Stop Override" runs the
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

Verified against `BlueBoat-Control` on 2026-08-28; frame item 00 added
2026-08-31. Robot-side code lives in the `BlueBoat-Control` submodule and is
built there — nothing is copied from this repo (`CLAUDE.md` §6, §3 N9).

000. **`param_set` could latch `busy` forever — fixed (2026-09-04).**
   Observed in the field as an override that never locks: `robot_interface`
   logging `Waiting for param mode 'override' (current: ''), re-requesting…`
   once a second forever, against `blueboat_parameter_control` answering
   `Parameter sequence in progress, ignoring request` just as often.
   `param_set` set `self.busy = True` before its first MAVROS call and cleared
   it **only inside a `call_async` done-callback**, with no timeout on any of
   the four calls. A `/mavros/param/pull` that never returned — link drop,
   mavros restart, a lost ACK — left `busy` True permanently, so every later
   request was discarded while the requester retried forever. Nothing gave up
   and nothing said so. Compounding it, `publish_state()` published
   `param_mode` only when a mode had already been applied, so before the first
   success the topic was silent and `current` read `''` — indistinguishable
   from a dead node.
   Fixed in `BlueBoat-Control`: a wall-clock watchdog abandons a sequence held
   longer than `param_sequence_timeout_s` (new declared parameter, default
   20 s — a cold `ParamPull` is genuinely slow); a generation counter fences
   off futures belonging to an abandoned sequence so a late completion cannot
   clobber the one that replaced it; `publish_state()` always publishes (`''`
   meaning "alive, no mode locked"); a 1 Hz heartbeat lets a late station see
   the mode; and failures schedule a bounded internal retry instead of
   stopping. **Station-side consequence:** the heartbeat is why the
   safe-shutdown echo now requires a transition (see above).

00b. **Motors could spin with `enable_motors:=False` — fixed (2026-09-04).**
   The gate itself was sound: `robot_interface.manualMove` returns before every
   thrust-bearing write to `/mavros/rc/override`. But returning meant
   publishing **nothing**, and by that point `robot_interface` had already
   requested `override` *unconditionally*, so `param_set` had mapped
   `SERVO1_FUNCTION`/`SERVO3_FUNCTION` to RCIN1/RCIN3 passthrough. In that
   state the ESCs follow RC channels 1 and 3 from *any* source — a hand
   transmitter, a QGC joystick, ArduPilot's RC failsafe — and nothing was
   feeding `RC_OVERRIDE_TIME` to keep those channels ours.
   Fixed by making the closed gate **hold neutral** rather than go silent:
   while in override with motors disabled, `manualMove` streams 1500/1500 at
   the loop rate (0 N is an exact knot of the calibration table, and the
   reversed side is `3000 − 1500 = 1500`). No commanded thrust reaches the
   water — which is all the gate ever promised — and the CSV's
   `actuation_state` still reports `0` for the whole run. Superproject **CM-16
   is reworded accordingly**: the gate now means "only neutral PWM may be
   written while disabled", not "nothing may be written".

00c. **No shutdown hook on `robot_interface` or `param_set` — fixed
   (2026-09-04).** Both ended in a bare `rclpy.spin(node)`, so `KeyboardInterrupt`
   skipped straight past `destroy_node()`: the boat was left in RC passthrough
   with nobody streaming, and the position CSV was never closed. Both now run a
   `try/except KeyboardInterrupt/finally` (the pattern `master_control.py`
   already used). `robot_interface` stops the motors, releases the RC channels,
   requests `default`, closes the CSV and then writes the post-mission report;
   `param_set` restores the default servo mapping, bounded to ~3 s so teardown
   cannot hang. Both are best-effort and independent — mavros is usually dying
   in the same process group — so an operator `default` before shutdown remains
   the reliable route (CM-15).

00. **`/blueboat/odom` frame — fixed to local ENU (2026-08-31).**
   `robot_interface.odom_callback` used to translate the MAVROS position to
   the launch point but ALSO re-zero yaw (`yaw − yaw0`) without rotating the
   position axes — a hybrid frame (ENU axes, launch-relative heading) that
   was self-consistent only when the boat launched facing East. This was the
   root cause of the field symptoms "trajectory following only works starting
   East", wrong manual-target behaviour and broken GPS anchoring. The fix
   (committed in `BlueBoat-Control`) drops the yaw re-zeroing: the frame is
   now **local ENU** — origin = launch point, +x = East, yaw absolute — the
   same frame kind the simulator publishes. The station's rotation-estimation
   machinery (Kabsch fit, `heading_aligned`, two scene regimes) was removed
   with it; see `GPS_MAP_ARCHITECTURE.md`. **A boat running a stale build
   reintroduces the hybrid frame silently** — rebuild `/blueboat_ws` before
   trusting the map.

0. **Monitoring target frame — uniform WORLD in every branch.**
   `master_control.py` captures the world target *before* its
   `inRobotFrame(...)` conversion and monitors that, in all four controller
   branches (the `# --- world-frame monitoring target ---` markers), so
   `/monitoring_data`'s `x_d/y_d` no longer mix frames per branch. Control
   behaviour and `/controller_target` are untouched. **The station's display
   and `robot_interface`'s no-pinger CSV both rely on this**, and the app
   applies no frame fixup of its own — re-adding one would double-convert
   (§3 N3).
   *Residual risk is deployment, not source:* a boat running a **stale build**
   that predates the capture sends robot-frame LoS/manual/pinger targets, so
   `store.active_target_world()` — which in path mode is `/monitoring_data[4:5]`
   — draws the robot→target line at a meaningless point while the boat still
   tracks its path correctly. The symptom looks cosmetic; the fix is to rebuild
   the boat workspace at the SHA the superproject records (`TODO.md` A3), never
   to compensate in the station.
   Frame answer for the record: the manual target is WORLD-frame on the wire
   (converted by `inRobotFrame` before `solve_LoS`); the pinger target is
   ROBOT-frame on the wire (passed directly) — both reach `solve_LoS` in the
   robot frame, so LoS is consistent.

0b. **Keep-position semantics** — YAML trajectories clamp at their final pose
   forever, so every controller station-keeps at the end of a custom path; the
   hard-coded shapes already "default to last known point". For the manual
   target, the default `safety_distance = -1.0` disables the LoS arrival check,
   so the boat naturally station-keeps on the fixed target. **Do not raise
   `safety_distance` without reading `CLAUDE.md` §6 first** — it owns the
   current-state fact about the `stopping_sequence` latch that follows from it.

1. ~~Manual-target resume comparison~~ — **fixed** in `BlueBoat-Control`. The
   guard is now `list(self.manual_target) != [0.0, 0.0]`
   (`master_control.py:406`), so the `array('f')`-vs-`list` mismatch is
   gone and the `[0,0]` resume sentinel is recognised. "Continue Original
   Mission" needs no robot-side change.
2. ~~Pinger branch publishes the target on `/thruster_input`~~ — **fixed**. The
   "Publish controller target (for data recording)" block uses
   `self.target_publisher.publish(msg)` (`master_control.py:517`); the thrust
   stream carries thrust only.
3. **World-frame pinger not published** — still open. `robot_interface`
   computes `corrected_pinger` (`:599`) but only writes it to the CSV and the
   pinger-GPS conversion; `/blueboat/pinger_coordinates` carries the body-frame
   vector. The station derives the world position itself, once per pinger
   message, from `/blueboat/pinger_coordinates` + `/blueboat/odom` (§3 N4). If
   you prefer a single source of truth, publish it robot-side on a new topic
   and point `TopicsConfig` at it.
4. Cosmetic: `robot_interface` publishes monitoring on
   `blueboat/monitoring_data` (relative) while `master_control` uses the
   global `/monitoring_data` — the station follows `master_control`.
5. **MPC on `fsin` orbits — expected from the current construction, not a
   station bug (2026-09-01).** Observed on the station map: under MPC on the
   `fsin` trajectory the boat locks into a perfect circle near the start
   while the reference runs ahead (live-distance plot oscillating at the
   loop period, growing). The `fsin` reference itself is a chain of
   near-closed ~2 m loops (364.75° heading swing per half-cycle), and the
   MPC's cost/governor combination settles into a self-orbit once displaced.
   Full mechanism and the sim-only remedies:
   `BlueBoat-Control/blueboat_control/src/CONTROLLERS.md` finding **C10**
   and that module's `TODO.md` §0.3. Nothing to compensate in the station —
   the map is displaying the truth.

**Path-following speed.** The old "LoS crawls in path-following mode" analysis
that used to sit here is superseded: path following now uses `los_guidance()`
(canonical Fossen lookahead with a path-parameter governor,
`master_control.py:648-697`) at a 20 Hz loop, and `solve_LoS` — whose thrust
law is `v = 5·ln(0.15·d + 1)`, doubled to `10·ln(v + 1)` for manual targets
(`:426`, `:429`) — is reached only for pinger and manual point targets. What
remains is a field measurement, tracked as `TODO.md` C5. Never compensate for
it in the station.

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


## CSV logging layout (`robot_interface.py`)

Columns are reorganised **important-first, names unchanged**: date fields,
`relative_x/y/psi`, `target_x/y[/psi]`, [`corrected_pinger_x/y`, GPS,
pinger GPS,] `right_thr_in`,`left_thr_in`, then raw sensors (and raw USBL
fields in pinger mode). Rows are filled **by column name**, the swapped
thruster columns are fixed (`thruster_input` is `[right, left]`), and the
no-pinger target now logs the world-frame `/monitoring_data` target for
every controller (empty buffer → zeros, no debug spam). In **pinger mode**
the duplicated `target_x/y/psi` columns were removed: they held
`/controller_target`, i.e. the same pinger vector as `corrected_pinger_x/y`
but in the robot frame — redundant. The world-frame `corrected_pinger_x/y`
columns are kept; the no-pinger CSV still logs `target_x/y` (its only
target source).

The `master_control` loop runs at **20 Hz** — `self.dt = dbl('control_dt',
0.05)` (`master_control.py:263`), now a declared ROS parameter; the file header
(`:1-15`) records the move off the old 1 Hz loop. `/monitoring_data` and the
target-column refresh follow that rate, so the station's
`diagnostics.expected_hz` of `20.0` is correct and needs no change.

## Pinger latency (field observation explained)

The marker used to trail the robot because the station re-anchored the
latest body-frame pinger vector to *every new robot pose*; it now computes
the world position once per `/blueboat/pinger_coordinates` message, with
the pose concurrent with that message, and holds it fixed in between.
Residual sluggishness is robot-side and inherent: with
`fixed_pinger=False` the published vector is seeded from the Waterlinked
*filtered* acoustic position (seconds of smoothing) and dead-reckoned with
odom twist between USBL updates — drift there shows up as slow marker
convergence, not as robot-following.

## Designer trajectories — how `path_generation.py` loads them

The Survey Pattern Designer exports `blueboat_trajectory/1` YAML files
(specification: `08_trajectory_format.md`). The YAML path affects **only the
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
  and an **mtime watcher** that reloads when the file appears or changes:

  ```python
  if path_shape.startswith('from_yaml') and self.yaml_traj is not None:
      x, y, z, roll, pitch, yaw = yt.read_yaml(self.yaml_traj, t)
      ...
  ```

To change either one: edit it in the `BlueBoat-Control` submodule, commit
there, and `colcon build` the boat's workspace. No launch-file modification is
needed — the YAML path rides inside the existing `trajectory` argument
(`trajectory:=from_yaml:/abs/path.yaml`), which both `BlueBoat_launch.py` and
`Sim_launch.py` already forward to the node.

Station side, saved missions appear automatically in **Launch Mission →
Trajectory → custom paths**; selecting one builds the `from_yaml:` string.
GPS-anchored missions are labelled **“(GPS)”** and follow the deferred
deployment flow of `08_trajectory_format.md`: the node holds a station-keeping
pose at the origin until the station writes the deployed file, which the mtime
watcher then picks up on its next path request. That hold is what makes
deferred, GPS-anchored deployment possible — it is not a hang; see
`07_getting_started.md` for what the operator does about it.

## Mission-path preview and `path_publisher.py`

The station previews the mission path by calling the **`/path_request`
service directly** (the same request `path_publisher.py` makes at startup),
so the preview works identically on the real robot and in simulation and
never depends on `path_publisher`. The request horizon is
`launch.path_preview_total_time_s` (default 120 s, configurable); for
designer trajectories the YAML's own `duration_s` replaces it
automatically, so long custom missions are previewed completely.

`path_publisher.py` itself is only started by `Sim_launch.py`. It is not
simulation-specific code — it merely was never added to the real-robot
launch. To make it available in the real world (e.g. to keep RViz support),
add to `BlueBoat_launch.py` inside the `controller_type != ''` /
`use_pinger == False` branch, next to `path_generation.py`:

```python
sl.node('blueboat_control',
        'path_publisher.py',
        parameters={'total_time': 300.0,   # horizon in seconds
                    'dt': 0.5})
```

Its 120 s "time limit" is just the default of its declared `total_time`
parameter — override it as above (match the mission duration; for YAML
trajectories, the `duration_s` field of the file). Note it also busy-waits
for the service at startup, which is harmless in this launch ordering.
