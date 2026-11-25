# CLAUDE.md — `mcs/designer` (Survey Pattern Designer)

Loaded when working under `mcs/designer/`; module-wide rules (N1–N10, §-references) are in
`BlueBoat-MCS/.claude/CLAUDE.md`.

**Layering.** `model.py`, `interpolation.py`, `patterns.py`, `sampling.py`, `io_yaml.py` are
widget-free (`sampling`, `io_yaml`, `interpolation`, `patterns` import no Qt at all);
`designer_map.py`, `panels.py`, `designer_window.py` are UI. The window is titled "Survey
Pattern Designer". The widget-free modules never import `mcs.gui` (the smoke test asserts it
before anything pulls `mcs.gui` in); only the UI modules use `mcs.gui.widgets`.

**Extension points are registries:** add an `Interpolation` or `Pattern` subclass and register
it in `REGISTRY`; parameter forms are generated from each class's `schema`. The runtime never
learns about new types — interpolation is resolved at export time.

**Undo is snapshot-based: any new mutating entry point must call
`DesignerWindow._push_undo()` first** (not covered by the smoke test).

## Files produced (in `~/.config/blueboat_mcs/trajectories/`)

| File | Role |
|---|---|
| `<name>.yaml` | **Runtime**, `blueboat_trajectory/1`: dense `[t, x, y, yaw]` samples + `speed`, `loop`, `length_m`, `duration_s`. Read by `path_generation`, the World Builder and the GCS. |
| `<name>.meta.yaml` | **Editor only**: groups, locks, per-segment interpolation + speed, comments, `close_path`, `repeat`. |
| `.deployed/<name>.yaml` | **Regenerated every launch** of a GPS-anchored mission; carries `deployed_from` / `deployed_fit_rms_m`. Never hand-edit. |
| `.session/run.yaml` | **Real-robot run session**: the one file `path_generation` watches, rewritten atomically (`_atomic_write_yaml`) before every run — `deploy_mission` for an anchored mission, `place_mission` (moved rigidly onto the boat's pose at START RUN) otherwise. Never hand-edit. |

Why dense samples: `path_generation.single_pose(t)` evaluates at arbitrary times, so the
runtime only does a binary search plus linear interpolation (yaw wrap-aware). With
`loop: false` evaluation past the end **clamps to the final pose** (the mission ends in
station-keeping; `master_control` reports `path_complete` and the station's RUN box says
*Path complete — holding final pose*); with `loop: true` time wraps modulo `duration`
and the path repeats forever.

## Two mission options, not one "loop" (2026-11-24, owner order)

The single "Loop mission (close last → first)" box used to set `MissionModel.loop`, which
both added the closing leg in `sampling.sample_mission` **and** wrote the runtime
`loop: true` — so every closed path also repeated forever in the control stack (19 of 23
saved missions carried `loop: true`). Split into two independent options:

| Option (MISSION SETTINGS) | Model field | Drives ONLY |
|---|---|---|
| **Return to start (close last → first)** | `close_path` | the closing leg in `sampling.sample_mission` (it is in the samples) |
| **Repeat forever** | `repeat` | the runtime file's `loop:` (`io_yaml.save_mission`) |

Both default **off** for a new mission. **Migration** (`MissionModel.from_dict`): a model
dict with neither `close_path` nor `repeat` (a `.meta.yaml` written before the split) sets
**both** from its old `loop`, so an old mission keeps its behaviour until it is re-saved;
new metas write the two keys and no `loop` (nothing else reads the meta file — the
simulator's builder only excludes `*.meta.yaml`). A runtime file re-imported without its
meta gets `close_path: false` (its samples already hold any closing leg) and `repeat` =
its `loop`. `deploy_mission` / `place_mission` copy the runtime dict, so `loop` passes
through to the deployed / session file unchanged. The launch and Update Mission dialogs
show a path's end **read-only** (`io_yaml.read_path_ending`: `loop`, and whether the last
sample returns to the first) — this designer stays the one place to change it. Smoke
block `designer split ok`.

## Safety way back (2026-11-25, owner order)

`MissionModel.safety_way_back` (`[[x, y], ...]`, design frame) is a second, hand-drawn
polyline, **not** part of the survey path or its samples: the route the link guardian follows
if the link is lost during a run of a GPS-anchored mission (from its point closest to the
boat to its last point, LOITER there). Editor: `EditMode.ADD_SAFETY` / `SafetyPointItem`
(square handles, moved through `on_safety_moved`; the shared drag code keys `_drag_origin`
by item) / `safety_preview` in `designer_map.py`; toolbar *✚ Safety way back* (B) and
*Clear way back*, enabled only with an anchor (`_update_safety_actions`), in one exclusive
`QActionGroup` with *✚ Add Waypoints*. Mutators
`add_safety_point`, `move_safety_point`, `remove_safety_points`, `clear_safety_way_back`
(each preceded by `_push_undo()`; it is in `to_dict`, so undo covers it; `Align to Start`
moves it rigidly). `save_mission` writes the runtime block `safety_way_back:
{format: blueboat_safety_way_back/1, points: [[lat, lon], ...]}` only when anchored and
≥ 2 points (≤ `SAFETY_MAX_POINTS` 200); `read_safety_way_back` reads it.
`RunSession._send_safety_way_back` sends it to `guardian.py` at START RUN (`safety_load`),
`stop_run` sends `safety_clear`. Format: `docs/05_trajectory_format.md`. Smoke block
`safety way back ok`.

**Anchoring and start alignment.** Optional `geo_anchor: {lat0, lon0, theta_deg}`
georeferences the design frame; anchored missions are relocated into the run's world frame
at deploy time. Non-GPS missions are normally *start-aligned* (first sample `(0, 0)`, first
tangent `+x`, i.e. at the launch point heading east) — offered on save by
`_maybe_offer_alignment`, and from Edit ▸ Align to Start. The launch dialog badges a non-GPS
path that does not start that way `(not start-aligned)`; GPS-anchored missions are exempt
(geographically fixed), and so is every path in a run session (placed at the boat). The badge
is informational — the launch path never rewrites a mission file.
`sampling.start_misalignment` is the single predicate behind both, with its tolerance in
`DesignerConfig` (`start_align_tol_m` / `start_align_tol_deg`).

## The design anchor is explicit, and never live

**The designer reads no live telemetry.** Its georeference — `_manual_fit`, returned by
`_active_fit()` and set only through `_set_anchor()` — comes from exactly four deliberate
sources: typed coordinates, a Gazebo world, a one-off snapshot of the robot's current fix
(**Set GPS Origin ▸ From the Robot's Current Position**, `_set_gps_origin_from_robot` — the
only use of the store), or the opened mission's `geo_anchor`. `New` and `Open` reset it, so
the anchor belongs to the mission, never to the session. A status-bar read-out names the
anchor in force and its source.

*Why it is non-negotiable:* adopting the station's live fit replaces the mission's own anchor
with the boat's launch point. Tiles, per-waypoint GPS read-outs and the world-limits rectangle
then translate rigidly by the distance between the two, `_write()` re-derives `geo_anchor`
from it (so `Ctrl+S` silently repoints the file at the boat and `deploy_mission` runs the
survey that far off), `worlds.duplicate_world_for_path`'s "`design_offset` is [0, 0] when the
anchor came from the world" invariant breaks, and `_maybe_offer_alignment` treats every
mission as GPS-anchored. The smoke test's `designer anchor ok` block pins all of it against a
store with a *valid* live fit. Consequently the designer draws **no robot or pinger overlay**,
and `DesignerMapView.resizeEvent` refreshes the tiles itself.

**Set GPS Origin ▸ From a Gazebo World** anchors on a world's `metadata.yaml`
(`core/worlds.py::read_world_meta`): its `geo_anchor` becomes the design origin (design frame
== world local frame) and its limit rectangle is a read-only dashed overlay
(`DesignerMapView.set_world_limits`, corners kept in GPS and re-projected through the active
fit). The reference is transient window state (`_world_ref`) — never on the model, in undo, or
in the runtime YAML. Saving with it set copies the world folder to
`~/worlds/<mission name>/<world name>/` with `metadata.yaml`'s `source_path` and
`builder_state.yaml`'s `path.name` / `path.file` re-pointed at the new mission
(`worlds.duplicate_world_for_path`); an existing target is never overwritten (write-once,
CM-7).
