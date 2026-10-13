# CLAUDE.md — `mcs/designer` (Survey Pattern Designer)

Loaded when working under `mcs/designer/`. Moved verbatim from `BlueBoat-MCS/.claude/CLAUDE.md`
§5 on 2026-10-08 (section and N references point there).

`mcs/designer/` is layered so the logic is testable without Qt widgets:
`model.py`, `interpolation.py`, `patterns.py`, `sampling.py`, `io_yaml.py` are
widget-free; `designer_map.py`, `panels.py`, `designer_window.py` are UI. The
window is titled "Survey Pattern Designer".

Extension points are registries — add an `Interpolation` subclass or a `Pattern`
subclass and register it in `REGISTRY`; the parameter forms are generated from
each class's `schema`. The runtime never learns about new types, because
interpolation is resolved at export time.

Undo is snapshot-based: **any new mutating entry point must call
`DesignerWindow._push_undo()` first.**

## Files produced (in `~/.config/blueboat_mcs/trajectories/`)

| File | Role |
|---|---|
| `<name>.yaml` | **Runtime.** Format tag `blueboat_trajectory/1`. Dense `[t, x, y, yaw]` samples + `speed`, `loop`, `length_m`, `duration_s`. Consumed by the robot. |
| `<name>.meta.yaml` | **Editor only.** Groups, locks, per-segment interpolation + speed, comments. The robot never reads it. |
| `.deployed/<name>.yaml` | **Generated per run**, for GPS-anchored missions. Regenerated every launch. |
| `.session/run.yaml` | **Real-robot run session** (2026-10-09): the one file `path_generation` watches, rewritten atomically before every run — `deploy_mission` for an anchored mission, `place_mission` (rigidly moved onto the boat's pose at START RUN) otherwise. Never hand-edited. |

Why dense samples: `path_generation.single_pose(t)` evaluates at arbitrary
times, so the runtime only ever does a binary search plus linear interpolation
(yaw wrap-aware). With `loop: false` evaluation past the end **clamps to the
final pose**, so a mission ends in station-keeping; with `loop: true` time wraps
modulo `duration`.

Optional `geo_anchor: {lat0, lon0, theta_deg}` georeferences the design frame.
Anchored missions are relocated into the current run's world frame at deploy
time; non-GPS missions are normally *start-aligned* (first sample `(0,0)`,
first tangent `+x`) so the boat starts at the mission and moves forward —
offered on save by `_maybe_offer_alignment`, and available any time from
Edit ▸ Align to Start. The launch dialog badges a **non-GPS** custom path whose
runtime file does not start that way as `(not start-aligned)`; GPS-anchored
missions are exempt (they are geographically fixed and the boat turns toward
them), and so is every path in a real-robot run session, where
`io_yaml.place_mission` puts the first sample on the boat and the first
heading along the boat's at START RUN (the badge is not shown there). It is
informational — the launch path never rewrites a mission file.
`sampling.start_misalignment` is the single predicate behind both the designer
action and the badge, with its tolerance in `DesignerConfig`
(`start_align_tol_m` / `start_align_tol_deg`), so the two cannot disagree about
the same file. `mcs/designer/{sampling,io_yaml,interpolation,patterns}.py`
import no Qt at runtime; only `model.py` and the UI modules do.

## The design anchor is explicit, and never live (2026-09-09)

**The designer reads no live telemetry.** Its georeference — `_manual_fit`,
returned by `_active_fit()` and set only through `_set_anchor()` — comes from
exactly four places, all of them deliberate: typed coordinates, a Gazebo world,
a one-off snapshot of the robot's current fix (**Set GPS Origin ▸ From the
Robot's Current Position**), or the `geo_anchor` of the mission being opened.
`New` and `Open` reset it, so the anchor is a property of the mission rather
than of the session, and an un-anchored mission can never inherit an origin it
was not drawn at. A permanent status-bar read-out names the anchor in force and
where it came from.

*Why this is a non-negotiable and not a preference:* `_active_fit()` used to
prefer `store.geo.fit` whenever the station held a GPS lock, deriving the design
origin from the boat's launch point and discarding whatever anchor the operator
or the file had set. Waypoint `x/y` were untouched, so satellite tiles, the
per-waypoint GPS read-outs and the world-limits rectangle all translated
rigidly by the distance between the mission's real anchor and the boat — the
"every path is shifted, restart the app and it's fine" symptom (a fresh app has
no lock, so the real anchor won). Worse, `_write()` re-derived `geo_anchor` from
it, so `Ctrl+S` on an opened mission silently repointed it at the boat on disk
and `deploy_mission` then ran the survey that far off. It also broke
`worlds.duplicate_world_for_path`'s documented "`design_offset` is [0,0] when
the anchor came from the world" invariant, and made `_maybe_offer_alignment`
treat every mission as GPS-anchored. `smoke_test.py`'s `designer anchor ok`
block pins all of it against a store with a *valid* live fit.

Consequently the designer draws **no robot and no pinger overlay** — a live
glyph placed at raw world `(x, y)` is only correct while the design frame is
the robot's world frame, which is precisely the assumption that was wrong. The
250 ms overlay timer that re-pushed the live fit is gone with them;
`DesignerMapView.resizeEvent` now carries the tile refresh that tick provided.

**Set GPS Origin ▸ From a Gazebo World** anchors the design frame on a
generated world's `metadata.yaml` (`core/worlds.py::read_world_meta`): the
world's `geo_anchor` becomes the design origin (design frame == world local
frame) and its limit rectangle is drawn as a read-only dashed overlay
(`DesignerMapView.set_world_limits`, corners kept in GPS and re-projected
through the active fit). The reference is transient window state
(`_world_ref`) — never on the model, never in undo, never in the runtime
YAML. Saving with it set copies the world folder to
`~/worlds/<mission name>/<world name>/` with `metadata.yaml`'s
`source_path` and `builder_state.yaml`'s `path.name`/`path.file` re-pointed
at the new mission (`core/worlds.py::duplicate_world_for_path`); an
existing target is never overwritten (write-once, CM-7).

**Do not hand-edit files under `.deployed/`** — they are regenerated and carry
`deployed_from` / `deployed_fit_rms_m` provenance.

Other persistent state: `~/.config/blueboat_mcs/config.json` (user overrides,
hand-editable) and `~/.config/blueboat_mcs/tile_cache/` (disposable).
