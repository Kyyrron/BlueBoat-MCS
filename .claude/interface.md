# BlueBoat-MCS — ROS 2 interface: the exact contract (subscriptions, publications, launch modes, frames)

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## 2. ROS 2 interface — the exact contract

Names live in `TopicsConfig`; change them there, never inline. Peer: `blueboat_control`
(`robot_interface`, `master_control`, `path_generation`, `param_set`, and since 2026-11-24
`pinger_estimator`) plus MAVROS.
Subscriptions use the default reliable depth-10 profile unless stated.

### Subscriptions

| Topic | Type | Producer | Notes |
|---|---|---|---|
| `/blueboat/odom` | `nav_msgs/Odometry` | `robot_interface` (real) · sim bridge · `bag_replay` | **Local ENU**: origin = boat at launch, axes East/North, yaw **absolute ENU** (0 = East, CCW+, not re-zeroed); the sim differs only in origin. Pose, speed, trail. |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | MAVROS · **the station's own bridge** (Gazebo run of a GPS-anchored mission: a 5 Hz timer synthesises fixes from the sim odom about an origin `SimGpsConfig.offset_north_m` north of the first point, received back through its own subscription, so the whole real pipeline runs) | **BEST_EFFORT.** `lat == lon == 0` = no fix, discarded. Georeference, GPS read-out. |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | MAVROS | **BEST_EFFORT.** Degrees clockwise from north → `radians(90 − hdg)`. Preferred glyph heading; fallback the odom yaw. |
| `/mavros/battery` | `sensor_msgs/BatteryState` | MAVROS | **BEST_EFFORT.** `percentage` is a **0..1 fraction**. NaN fields pass as `None` and never overwrite the last good reading; the row greys after 15 s. Real boat only. |
| `/mavros/state` | `mavros_msgs/State` | MAVROS | Only when `mavros_msgs` imports. FCU connection (readiness count); `armed` (the **autopilot's** arming, not the motor gate) and `mode` for the GUARDIAN box and N1's MANUAL step. |
| `/blueboat/pinger_coordinates` | `Float32MultiArray` | `pinger_estimator` (2026-11-24; `robot_interface` before) | **Body frame**, odom rate: what `master_control` steers to — the filtered vector (zeros until the filter initialises), or the legacy one with `pinger_source` legacy → `PingerEstimate.body` (distance fallback). Feeds the legacy layer too, but only while no `/blueboat/pinger/legacy_body` publisher exists (older stack / old bag; `BridgeNode._legacy_publisher_absent`). |
| `/blueboat/pinger/legacy_body` | `Float32MultiArray` | `pinger_estimator` | (2026-11-24) The legacy algorithm's body vector, always → `store.on_pinger_body` (the **legacy** layer, world position per message, N4). |
| `/blueboat/pinger/estimate` | String JSON `blueboat_pinger_estimate/1` | `pinger_estimator` | (2026-11-24) Per fix + ~1 Hz heartbeat. `x, y`, `ellipse {a, b, theta}` **world frame at the source**: the filtered marker / track / ellipse are drawn from them verbatim (`PingerEstimate`); status, fix age / rate, r95, vx / vy, stats, `range_m`, `source` read by the PINGER panel. `TargetMode.PINGER` steers to it (legacy position when `source` is legacy). |
| `/blueboat/pinger/fix` | String JSON `blueboat_pinger_fix/1` | `pinger_estimator` | (2026-11-24) **RELIABLE depth 50.** Per processed fix: `world` + `decision` → `store.pinger_fixes` (raw-fix layer). |
| `/blueboat/pinger/filter_config` | String JSON | `pinger_estimator` | (2026-11-24) **TRANSIENT_LOCAL depth 1 RELIABLE.** Applied set: `preset, params, hash, pinger_source, seq, ok, detail, mapping, allow_live_retune` → `PingerFilterState` (applied = echo of the sent `seq`, `ok`, sent params and source). |
| `/uw_gps_data` | `Float32MultiArray` | `uwgps_log` | Raw USBL freshness only (19 values). |
| `/monitoring_data` | `Float32MultiArray` | `master_control` | `[t, x, y, psi, x_d, y_d, psi_d, u1, u2]` at 20 Hz; `x_d/y_d/psi_d` **world frame in every branch** (N3). |
| `/thruster_input` | `Float32MultiArray` | `master_control` | **`[right, left]`** N. |
| `/blueboat/controller_ready` | `std_msgs/Bool` | `robot_interface` | Readiness gating; re-published every 1 s (no latched QoS — the republish covers late subscribers). `False` the instant `full_stop()` latches = **E-STOP acknowledgement** (N1b). |
| `/blueboat/param_mode` | `std_msgs/String` | `param_set` | `'default'` / `'override'` / `''` (alive, no mode). **Heartbeated 1 Hz**, so shutdown requires a *transition* (N1). |
| `/blueboat/run_state` | String JSON | `master_control` (session) | `{seq, ok, detail, state: idle\|loading\|running\|error, controller_type, use_pinger, note, mission, run, npy}` + (2026-11-24) `pinger_mode, pinger_source, has_path, tau, path_complete`, answers each command's `seq`, heartbeated 1 Hz. **The truth about whether a run is active.** `_config_applied` also requires `pinger_mode` (absent = an older stack: off / follow only) and, when present, `has_path` and `path_source == "mission"`; `path_complete` during a run → RUN box "Path complete — holding final pose". |
| `/blueboat/run_log` | String JSON | `robot_interface` (session) | `{run, logging, csv, layout, bag_enabled, bag_recording, bag}` (`layout` `no_pinger` / `pinger` / since 2026-11-24 `pinger_log`), 1 Hz. A run whose CSV is not confirmed within 3 s is flagged; same for its ros2bag. No `bag_*` keys from an older control stack → "—". |
| `/link_guardian/status` | String JSON | `guardian.py watch` (real sessions) | 1 Hz: `{connected, armed, mode, want_on, guardian_on, check, home, run_running, messages, busy, ack}` (`home` = the Guardian Home since 2026-11-26) + (2026-11-25) `safety: {state: none\|loading\|ready\|failed, n, run, detail}` → `GuardianReadout`; stale after 4 s. |
| `/sim/sea_state` | String JSON | `sea_state_node` (sim) | **TRANSIENT_LOCAL depth 1** → `SeaReadout` (SEA STATE box). |
| `/replay/state` | String JSON | `bag_replay` (replay) | **TRANSIENT_LOCAL depth 1**, 2 Hz → `ReplayReadout` (REPLAY box). |

### Publications

| Topic | Type | Consumer | Contract |
|---|---|---|---|
| `/blueboat/input_str` | `std_msgs/String` | `robot_interface` | Tokens: `enable`, `disable`, `default`, `override`, `stop`, `arm`, `disarm`, `rearm`, `manual`, `move <l> <r> <s>`; anything else falls through to the move handler, empty is ignored. The station sends `default` / `override` (mode toggle, N1), `stop` (both E-STOPs), `enable` (**Release E-STOP**, the only token that clears the latch — it also re-opens the motor gate, so only offered in a session launched with motors enabled), `rearm` (GUARDIAN **Arm again**), `disarm` (turn-off choice), `manual` (N1, after *Guardian OFF*). |
| `/blueboat/manual_target` | `Float32MultiArray` | `master_control` | `[x, y]` **world frame**. `[0, 0]` is the resume sentinel (N2). Turning arrow control off publishes the boat's position (hold where released). |
| `/blueboat/run_command` | String JSON | `master_control` (session), `pinger_estimator` (its pinger block) | `{"cmd": "configure", controller_type, use_pinger, pinger_mode, has_path, path_source: "mission", pinger_source, pinger_filter: {preset, params}, note, mission, seq}` (2026-11-24: `use_pinger` = follow, kept for an older stack; `has_path` false for a run without a path (off or log); the full parameter set, `preset` `custom` when edited). Re-sent until acknowledged, and — pinger mode not off — until the estimator's `filter_config` echoes its `seq` (`RunSession._filter_timer`, the same 5 tries, between runs, after an MPC build) / `{"cmd": "start"\|"stop", seq}` (sent once). |
| `/blueboat/arrow_thrust` | `Float32MultiArray` | `master_control` (every mode) | `[right, left]` N at 20 Hz while ARROW CONTROL is on (`[0, 0]` with no key), `[]` releases. Overrides every branch and pauses a run; `master_control`'s dead-man (`arrow_timeout` 0.5 s) zeroes a gap and stays engaged. No `command_sent` per tick. |
| `/link_guardian/command` | String JSON | `guardian.py watch` | `{"cmd": "on"\|"off"\|"check"\|"setup"\|"reclaim"\|"set_home", "seq": n}` (`set_home` 2026-11-26), answered in the status `ack`. (2026-11-25) `{"cmd": "safety_load", "points": [[lat, lon], ...], "run": name, "seq"}` at START RUN of a GPS-anchored path with a safety way back, `{"cmd": "safety_clear", "seq"}` at STOP RUN (`RunSession`). |
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
| **Real robot = run session** | `BlueBoat_launch.py session_mode:=True trajectory:=from_yaml:<session file>` + (2026-11-24) `pinger_mode:= has_path:= pinger_source:= pinger_preset:= [pinger_filter_yaml:=]` | guardian, perception (normal) | Below. |
| **Empty Gazebo** | `Sim_launch.py` (`robot_file`, `trajectory`, `controller_type`, `note`, optional `spawn_yaw`) | sea state (`sea_state_launch.py world_name:=ocean`, **every** launch, null choice included — the simulator's catamaran hull floats the boat) | No MAVROS / `robot_interface` / `param_set` / pinger; N1's acknowledgement wait skipped. |
| **Personalized world** | `blueboat_sss_sim full_mission_launch.py` (`world_dir`, `with_control:=true`, `trajectory_file`, `controller_type`, `note`, `sea_*`; **no** `spawn_yaw`; with the ray-cast sonar also `sonar_renderer:=raycast real_time_factor:=<f>`) | perception (dialog **Session** row: normal / dataset → `dataset_name:=<YYYY_MM_DD-HH_MM_SS>-<note>`; `use_sim_time:=true`) | The simulator's mavros shim publishes GPS from the world's anchor: SimGps stays **disarmed** (N8). |
| **Bag replay** ("From a ros2bag") | `Replay_launch.py bag:= rate:= [world_dir:=] [placement:=origin]` + with *Re-estimate the pinger* (2026-11-24) `pinger_estimator:=true pinger_source:= pinger_preset:= [pinger_filter_yaml:=]` | none | Below. |

A simulated run's log note always carries the `sim` marker (`LaunchParameters.wire_note`).
Every Gazebo controller launch asks for the sea state after the world dialog.

**Run session (every real-robot launch).** The boat connects and goes into override, but
`master_control` idles — Manual Target and the arrow keys drive it, nothing is followed or
logged — until **START RUN** (RUN box, pinned under the left panel). **STOP RUN**: thrust zero,
still in override, the run's CSV / report / `.npy` / ros2bag filed by the control stack.
Between runs **Update Mission** (the Launch button's session label) chooses the next run's
pinger mode, designer path, pinger source / filter, the controller (MPC loads on a worker
thread; START RUN waits; MPC cannot follow the pinger) and the note. **Turn Off Connection** (the Stop button's label)
is N1, an active run stopped first. `run_session.py` owns the `seq`-acknowledged protocol and
`start_blockers()`, and the **one file `path_generation` watches**,
`LaunchConfig.session_trajectory_file` (`~/.config/blueboat_mcs/trajectories/.session/run.yaml`),
rewritten atomically just before each `start` (N2b). Designer paths only — built-in shapes are
simulation / CLI. Histories record only during a run (`DataStore.recording()`), so trail,
statistics, plot and timeline freeze between runs.

**The pinger (2026-11-24, owner order).** Modes (`pinger_filter.MODES`): **off** (a path),
**follow** (steer to the pinger, no path; PID / LoS — MPC greyed in the dialogs, refused by
`master_control`), **log** (the boat never steers to it: with a path a path run, any
controller). In off and log the path list offers "(no path)": `has_path: false`
(`NextMission.source` "" outside follow), no session file, no preview, no path blockers —
only Manual Target / arrows. The source and filter rows show only when the mode is not off. `NextMission.pinger_mode` / `has_path` /
`uses_path`; `use_pinger` stays the follow alias. The dialogs share
`gui/dialogs/pinger_filter_dialog.PingerForm` (mode, source, preset, **Advanced…** =
`PingerFilterDialog`, generated from BlueBoat-Control's `config/pinger_filter.yaml` read as
DATA by `core/pinger_filter.py` — `PingerConfig.schema_file`, else the installed share, else
`schema_checkout`; never an import, N9). Every launch has its pinger rows hidden in
Gazebo. A custom set is written by `LaunchManager.start` to
`PingerConfig.filter_dir/last_launch.yaml` for `pinger_filter_yaml:=`
(`write_pinger_filter_file`). START RUN blocker *pinger filter configuration not applied
yet* (`— refused: <detail>`) while mode ≠ off and `PingerFilterState.applied()` is false.
`store.reset_pinger()` (legacy `PingerState`, `PingerEstimate`, every pinger trail) runs at
each launch and each START RUN (`reset_experiment`); the filter echo resets per launch.
Smoke blocks `pinger schema ok`, `pinger advanced ok`, `pinger dialogs ok`, `pinger
configure ok`, `pinger store ok`, `pinger bridge ok`, `pinger layers ok`.

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
Saved plans live in the same folder. **Sea situation** (CR-03-3, 2026-11-16): the dialog's
top row also offers the simulator's `config/sea_situations/*.yaml` (`SeaConfig.situations_dir`,
"" = installed share, then the checkout) or any schedule file; chosen, it is passed **as is**
(`SeaChoice.situation_file` → `sea_schedule:=<that file> sea_seed:=`; live, its document as
`{"schedule": …}`), nothing written, cards and plan greyed — the only way to the wind and
yaw-moment blocks. "None" (default) = unchanged. The SEA STATE box's Status row turns amber
"· hull: Gazebo box" when the readback says the URDF box floats the boat.

**Link guardian (real-robot sessions).** An ArduPilot Lua script (superproject
`link_guardian/`): after `LGRD_TIMEOUT` (15 s) of laptop silence it retraces the boat's trail,
and on the first laptop heartbeat hands control back (HOLD → servo mapping → MANUAL), so the
run goes on. **ON by default**: `robot_interface` arms the autopilot at connection before
override, and the station starts `guardian.py watch` (sets `LGRD_ON`, reports the Guardian
Home, serves the GUARDIAN box, top of the left column; rules in Qt-free
`core/guardian.py::summarize`). Buttons Check / Apply settings / Turn OFF-ON / Arm again /
Set Home / Take back control (a fallback, not gated on the run), each enabled only when it can act.
**Guardian Home** (2026-11-26, owner order): where a trail retrace ends (LOITER) — where the
guardian came ON, or the boat's position at the last **Set Home** (`set_home`: the script
restarts its trail there; gated on guardian ON, `check.set_home`, not under guardian control; no
run gate). Only Set Home moves it (HOME is no longer moved at run start / end). Status `home`
`{lat, lon, source: "guardian ON"|"Set Home"}` → the HOME row and the map's green *Guardian Home*
point (`MapView.guardian_home`, layer `guardian_home`, ON by default,
`core/guardian.guardian_home_point`: guardian ON and the script's own home only). The
E-STOP does not turn it off. **Turn Off Connection and app exit always ask in a real
session** (`needs_turn_off_choice`): *Guardian OFF + disarm* (`guardian_off_and_disarm`:
`off` → ack → `disarm` → `/mavros/state` disarmed; nothing terminated if OFF is unconfirmed),
*Keep ON* (the boat retraces its path), or *Cancel*; then N1. **Safety way back**
(2026-11-25, owner order): a GPS-anchored designer path may carry one (`mcs/designer/CLAUDE.md`);
`RunSession` sends it with `safety_load` right after `start` (never waits for it: owner, "warn,
start anyway") and `safety_clear` at STOP RUN; `guardian.py` uploads it as the autopilot's
mission and the script follows it on a takeover during the run. The box's **Way back** row and
state line say which route a link loss would use; the map draws it like the designer
(`MapView.safety_path` + `safety_points` + LOITER end; (2026-11-26) its own layer
`safety_way_back`, ON by default) from `SessionState.safety_way_back` during the run, else
`SessionState.planned_safety_way_back` (`RunSession.plan_safety_way_back`: the next run's
GPS-anchored path at `begin` / Update Mission, a Gazebo launch's anchored path; cleared at idle).

**BlueBoat-Perception.** Started with every real-robot launch and every personalized-world
launch, best effort; the station subscribes to none of its topics and imports nothing from it
(`PerceptionConfig`).

**Sonar model and slow-down (owner order, 2026-11-07).** Launch dialog, Gazebo + Controller:
**Sonar model** Geometric (default) / Ray-cast ((2026-11-18) the UI default is unchanged, but simulated **results** use
Ray-cast with the production profile `raycast_harbour_fast_sonar.yaml`; the geometric renderer is legacy,
`SIM_VALIDITY_CONDITIONS.md` §2 in the superproject), and for the ray-cast **Slow-down**
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
(2026-11-24, owner) A fresh dialog preselects *Ray-cast — harbour, fast* (`SimSpeedConfig.default_variant`;
"" = Geometric) and the menu leaves out the five harbour-fast bracket variants (flash narrow/wide, shells
opaque, corner low/high; `SimSpeedConfig.hidden_variants`); a bracket run passes `sonar_config:=<file>` in Extra args.

**Thrust gain (CR-03-3, 2026-11-16).** Launch dialog, Gazebo + Controller: **Thrust gain**
`[right,left]` (empty = the description's 0.493 / 0.493; FIT nominal `[0.3805,0.6055]`),
canonicalised by `parse_thrust_gain`, emitted as `thrust_gain:=` by the world branch only, after
the sonar arguments and before Extra args; an empty-world launch drops it with a message
(`drop_thrust_gain_without_world`: `Sim_launch.py` declares none). Smoke blocks
`sea situation ok`, `thrust gain ok`.

### Frame conventions (the recurring source of bugs)
- Manual target **world** on the wire; pinger **body** on `/blueboat/pinger_coordinates`
  and `/blueboat/pinger/legacy_body`. `master_control` converts the manual target via
  `inRobotFrame()` before `solve_LoS()` and passes the pinger through, so both are
  robot-frame at the solver — consistent by design. (2026-11-24) The filtered estimate
  (`/blueboat/pinger/estimate`, `/fix`) is **world** frame at the source and drawn as
  published; only the legacy body vector is transformed, once per message (N4).
- Yaw everywhere in the app: radians, CCW-positive about +z, y-up world. Odometry yaw is
  absolute ENU; `DataStore.robot_true_heading()` prefers the compass, falls back to the odom
  yaw, with no rotation correction anywhere (`docs/GPS_MAP_ARCHITECTURE.md`).

---
