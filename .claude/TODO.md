# BlueBoat-MCS — open items

Everything actionable lives here. `CLAUDE.md` describes current state only.
Ordered roughly by how much damage the item can do if left alone.

Items marked **NOT VERIFIABLE HERE** need the Linux development machine, a
sourced ROS2 environment, or the boat; they were not attempted on the Windows
checkout where this list was last audited.

---

## A. Field verification (blocking real confidence)

### A1 — Confirm the robot glyph heading is finally correct on the water
**NOT VERIFIABLE HERE** (needs a running MAVROS link and a field session).

Reported wrong three times across sessions. Two successive fixes were made and
both are present in the code, but **neither has been confirmed in the field**:
1. north-up via an ENU scene instead of a rotating view;
2. glyph heading taken from `/mavros/global_position/compass_hdg` rather than
   the launch-zeroed odom yaw.

Check: point the boat at a known bearing (e.g. along a quay wall visible in the
satellite layer), compare the on-screen arrow against
`ros2 topic echo /mavros/global_position/compass_hdg`, and confirm the map does
not rotate as the boat turns. Do this **before** trusting any heading-dependent
display in a real campaign. Note D5 below — the manual-target frame bug lives on
the same code path and should be fixed before this test is worth running.

### A2 — Confirm the compass topic is actually published in your setup
**NOT VERIFIABLE HERE** (needs MAVROS).

`/mavros/global_position/compass_hdg` was confirmed live once via `ros2 topic
echo`. Confirm it is present with the same QoS on the boat's MAVROS config, and
decide what the arrow should do if it disappears mid-mission (currently the last
value sticks, since `compass_heading` is only overwritten, never invalidated —
`store.py:162-169`).

### A3 — Confirm the boat's `blueboat_control` build is current
**NOT VERIFIABLE HERE** (needs the boat's `/blueboat_ws` and `colcon build`).

The source side is verified: `BlueBoat-Control/blueboat_control/src/master_control.py`
carries the `--- world-frame monitoring target ---` capture in all four branches,
`_custom_libraries/path_generation.py` has the `from_yaml` branch plus the mtime
watcher, `_custom_libraries/yaml_trajectory.py` exists, and `CMakeLists.txt`
installs all of them.

What remains is deployment: confirm the boat's checkout is at the commit the
superproject records for `BlueBoat-Control` and that the workspace was rebuilt
after the last update. Running against a stale build silently sends robot-frame
targets on `/monitoring_data` (target line points at nothing, no-pinger CSV
`target_*` columns corrupted) while the boat still tracks its path correctly —
so the symptom looks cosmetic.

---

## B. Testing

### B1 — `smoke_test.py` does not pass
It fails at `smoke_test.py:112`:

```
assert w.store.mission.manual_target is None
AssertionError
```

The script asserts the *old* manual-target model, where toggling the Manual
Target button off cleared the stored target. Under the current one-shot arming
model `_on_manual_mode(False)` only restores `MapMode.NORMAL`
(`main_window.py:346-359`); clearing moved to `_on_continue_mission()`
(`main_window.py:374-379`). The test line was never updated — it should drive
`w._on_continue_mission()` instead.

Five of seven checkpoints pass before the abort (`TimeSeries ok`,
`GeoReferencer ok`, `LoS predictor ok`, `store ok`, `stats ok`); `window ok` and
`SMOKE TEST PASSED` are never reached. Not platform-specific — this is pure
application logic. Verified on Python 3.12.9 / PySide6 6.11.1 / numpy 2.4.2 /
scipy 1.17.0 with `QT_QPA_PLATFORM=offscreen`.

### B2 — Coverage gaps in `smoke_test.py`
It runs entirely offscreen with synthetic telemetry and no ROS, so it cannot
catch QoS mismatches, real message-type drift, or launch-file argument errors —
those only fail in the field. Beyond that it exercises **none** of
`mcs/designer/`, `io_yaml` (including the N8 `deploy_mission` guard), or the
`command_center` safe-shutdown sequence, despite those carrying non-negotiables.
Add blocks for N1, N2, N4 and N8 rather than assuming they are protected.

### B3 — No linter or type-checker is configured
If one is added, expect noise from the deliberate broad `except` clauses in the
ROS-facing and logging paths.

---

## C. Known limitations carried deliberately

### C1 — `stopping_sequence` latches if `safety_distance ≥ 0`
`master_control.solve_LoS()` sets `stopping_sequence = True` and never resets it
(`master_control.py:334-341`), so every later point-LoS command yields zero
thrust. Harmless at the default `safety_distance = -1.` (`:208`, feature
disabled). If that feature is ever enabled, reset the flag in
`manual_target_callback`. Still present and unchanged.

### C2 — Residual pinger lag is inherent
After the app-side fix (N4), remaining sluggishness comes from the Waterlinked
*filtered* seed plus dead-reckoning drift between USBL updates
(`robot_interface.py:574-579`). Only addressable robot-side, if at all.

### C3 — GPS-anchored missions need an initial motion phase
Deployment waits for `heading_aligned`, which is unobservable while stationary
(`geo.py:198-217`, `main_window.py:276-288`). The boat holds position until the
operator drives a few metres. This is physics, not a defect — but it needs to be
in the field checklist so nobody reports it as a hang.

### C4 — Pre-existing missions are not start-aligned
"Align to Start" is offered when saving a non-GPS mission
(`designer_window.py:252-272`), so older files only get aligned the next time
they are saved. Consider surfacing a start-misalignment badge in the launch
dialog for legacy custom paths.

### C5 — Confirm path-following speed on the water
**NOT VERIFIABLE HERE** (needs a field session).

The old "LoS crawls in path-following mode" defect is gone in the source: path
following now uses `los_guidance()` — canonical Fossen lookahead with a path
parameter governor (`master_control.py:274-316`) — and `solve_LoS`, whose thrust
law is `v = 5·ln(0.15·d + 1)` (doubled to `10·ln(v+1)` for manual targets,
`:318-350`), is reached only for pinger and manual point targets. Confirm on the
water that path-following speed is now adequate. Never compensate for it in the
station.

---

## D. Code defects found by audit

### D1 — Manual-target clicks are published in scene coordinates, not world
`MapView.mousePressEvent` emits `mapToScene(...)` straight out as
`target_clicked` (`map_view.py:414-418`), and `_on_target_clicked` publishes it
verbatim on `/blueboat/manual_target` (`main_window.py:361-367`). Once
`_enu_scene` is True the scene **is** ENU, so the boat receives an ENU point on
a world-frame topic — it will drive to the wrong place, and the error grows with
the georeference rotation `theta`.

The placement path has the conversion (`_to_scene` / `GeoFit.world_to_enu`) but
the input path has no inverse, even though `GeoFit.enu_to_world` already exists
(`geo.py:112-114`). Same bug in `_inspect_point` (`map_view.py:433-447`, wrong
GPS read-out and robot-distance after alignment) and the measure tool
(`:460-469`). Mirror image in `center_on_robot()` (`:400-405`), which passes
world coordinates to `centerOn` without `_to_scene`, so recentring jumps to the
wrong spot in the ENU scene.

Contradicts the `/blueboat/manual_target` contract in `CLAUDE.md` §2. Fix on the
input path, then extend `smoke_test.py` to cover the round trip.

### D2 — `LaunchConfig.readiness_topics` is dead config
Declared at `settings.py:111-113` and read nowhere. Readiness is gated inline on
`fcu_connected` / `has_odom` (`main_window.py:219-223`). Either wire it up or
delete it — as it stands, editing it in `config.json` silently does nothing.

### D3 — Stale `integration/` references in shipped code comments
`store.py:212` and `io_yaml.py:8` point at `integration/master_control.py` and
`integration/yaml_trajectory.py`. That directory does not exist in this repo and
never has; the files live in `BlueBoat-Control` (`CLAUDE.md` §6).

---

## E. Docs

### E1 — `README.md` quotes the wrong workspace
`README.md:11` says `~/blueboat_ws`. The basestation workspace is `~/ros2_ws`,
sourced with `source env.sh`, with the app run from
`~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-MCS` (`build.sh`, and the
superproject's `terminals.txt`). `/blueboat_ws` is the **boat's** workspace, not
the basestation's.

### E2 — Refresh or retire the stale docs
Confirmed errors, not suspicions:
- `docs/03_ros_integration.md:191-192` states the `master_control` loop runs at
  `dt = 1.0 s` (1 Hz `/monitoring_data`) and tells the reader to change
  `diagnostics.expected_hz` to match. The loop is 20 Hz (`master_control.py:106`,
  with a header comment at `:6-11` explaining the move off 1 Hz), so
  `expected_hz: 20.0` is already correct and the doc is what needs fixing.
- `docs/03_ros_integration.md:66` credits `master_control` with publishing
  `/blueboat/controller_ready`. It is published by `robot_interface.py:83` and
  *subscribed* by `master_control.py:86`.
- `integration/` paths that resolve to nothing: `docs/02:110`,
  `docs/03:110,130,177,216,219,242`.

`docs/01`, `02`, `05`, `06`, `07` were not revised after the map rework and may
still describe the rotating-view model.
`docs/HEADING_AND_MAP_ALIGNMENT.md` was updated for the ENU model but predates
the compass heading source — it should mention that an absolute heading source
outranks the georeference offset, since that file is explicitly written to be
reused in the other application.

---

## F. Automation (Skill / subagent / hook)

**Not justified yet.** The one candidate previously listed here — a checker that
diffs `integration/*.py` against the copies deployed on the robot — is moot: the
files live in `BlueBoat-Control` and are version-controlled there, so ordinary
submodule hygiene covers it. The residual risk (A3: the boat's checkout drifting
from the SHA the superproject records) is a one-line
`git submodule status` check, not an agent.

Release/packaging automation is also moot — the zip-per-update workflow was an
artifact of chat delivery and disappeared once the repo was edited directly. A
doc-generation agent is not warranted either; the docs have drifted, but the fix
is the one-off cleanup in E2.
