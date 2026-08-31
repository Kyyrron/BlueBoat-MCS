# BlueBoat-MCS — open items

Everything actionable lives here. `CLAUDE.md` describes current state only.
Ordered roughly by how much damage the item can do if left alone.

Items marked **NOT VERIFIABLE HERE** need hardware: a live MAVROS link, the
boat's own workspace, or a field session. Everything verifiable from a Linux
checkout with ROS2 sourced has been checked there; what remains in section A
cannot be settled without going on the water.

---

## A. Field verification (blocking real confidence)

### A1 — Confirm the GPS-only map architecture on the water (2026-08-31 rework)
**NOT VERIFIABLE HERE** (needs a running MAVROS link and a field session).

The glyph heading was reported wrong three times across sessions; the root
cause is now identified and fixed at the source (`/blueboat/odom` published a
hybrid frame — ENU axes, launch-relative yaw; `BlueBoat-Control` now publishes
local ENU with absolute yaw), and the station's map was rebuilt GPS-only on
top of it (one ENU scene, translation-only anchor, no Kabsch/`theta` — see
`GPS_MAP_ARCHITECTURE.md`). **None of it is field-confirmed yet.**

Check, in order, after rebuilding BOTH the boat's `/blueboat_ws` and the
basestation workspace (A3 first — a stale boat build reintroduces the hybrid
frame silently):
1. glyph heading: point the boat at a known bearing (e.g. along a quay wall
   visible in the satellite layer), compare the arrow against
   `ros2 topic echo /mavros/global_position/compass_hdg`, and confirm the map
   never rotates as the boat turns;
2. map anchoring: the map stays empty with the "waiting for GPS fix" notice,
   then appears within seconds with tiles/glyph/trail aligned;
3. manual target: click a feature visible in the imagery, confirm the boat
   drives to that feature — from a NON-East initial heading;
4. trajectory following from a non-East launch heading (the old failure);
5. GPS-anchored mission: deploys within seconds while station-keeping, path
   lands on its real-world coordinates.

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

**Raised stakes since 2026-08-31:** the local-ENU odom fix
(`robot_interface.odom_callback` no longer re-zeroes yaw) also lives in
`BlueBoat-Control`. A boat running the pre-fix build publishes the old hybrid
frame, which silently breaks the GPS-only map, manual targets and non-East
trajectory following — there is no wire-level version handshake. Rebuild
`/blueboat_ws` before any A1 field check.

---

## C. Known limitations carried deliberately

### C5 — Confirm path-following speed on the water
**NOT VERIFIABLE HERE** (needs a field session).

The old "LoS crawls in path-following mode" defect is gone in the source: path
following now uses `los_guidance()` — canonical Fossen lookahead with a path
parameter governor (`master_control.py:278-320`) — and `solve_LoS`, whose thrust
law is `v = 5·ln(0.15·d + 1)` (doubled to `10·ln(v+1)` for manual targets,
`:318-350`), is reached only for pinger and manual point targets. Confirm on the
water that path-following speed is now adequate. Never compensate for it in the
station.

---

## F. Automation (Skill / subagent / hook)

**Not justified yet.** The one candidate previously listed here — a checker
that diffed a local copy of the robot-side nodes against the ones deployed on
the robot — is moot: those files live in `BlueBoat-Control` and are
version-controlled there, so ordinary submodule hygiene covers it. The residual
risk (A3: the boat's checkout drifting from the SHA the superproject records)
is a one-line `git submodule status` check, not an agent.

Release/packaging automation is also moot — the zip-per-update workflow was an
artifact of chat delivery and disappeared once the repo was edited directly. A
doc-generation agent is not warranted either: the docs drifted once, over one
tree move and one map rework, and the fix was a one-off cleanup.
