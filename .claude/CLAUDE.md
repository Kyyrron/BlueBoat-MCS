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

Simulated results follow `../SIM_VALIDITY_CONDITIONS.md` (superproject root).

**Simulation launches (simulator frozen 2026-11-18).** For a run whose results are used: **Sonar model**
*Ray-cast* with `sonar_config:=raycast_harbour_fast_sonar.yaml` in **Extra args** (the production profile; Geometric is
legacy, even though it stays the UI default) and a Slow-down; seas `harbour_chop` (easy) or `field_matched`
(hard) as the sea situation file, with `thrust_gain:=[0.3805,0.6055]` for the FIT seas; `field_matched_x2` is
**not** a rougher sea (only the yaw moment doubled: a yaw arm for PID and LoS only); Control's
`sim_config:=field_equivalent`; MPC field gains for the headline, LoS as the disturbance arm. The simulator
code is frozen; MCS stays add-only.

On 2026-11-18 §1, §2 and §4–§8 of this file moved **verbatim** to `.claude/layout.md`, `.claude/interface.md`,
`.claude/map-designer-control.md` and `.claude/commands-docs.md` (section numbers kept). §3 stays here.

**Detail loads by itself:** each `.claude/rules/*.md` is a path-scoped rule that imports one `.claude/` topic
file when you read or edit a matching file; the tables below name the same files for reading by hand.

| Before … | Read |
|---|---|
| adding a panel, a topic, a setting, a launch mode, a box | `.claude/layout.md` (§1, "How to add…") |
| a subscription, a publication, a launch mode, a frame | `.claude/interface.md` (§2) |
| the map, georeferencing, the Pattern Designer, trajectory files, the control-stack files | `.claude/map-designer-control.md` (§4–§6), `mcs/designer/CLAUDE.md` |
| running the station, its smoke test, the gates, the docs | `.claude/commands-docs.md` (§7–§8), superproject skill `module-gates` |

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

Superproject rules hold here: never `git commit`, `git push` or `git tag`; this module changes by additions only,
on an explicit user order.
