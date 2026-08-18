# Heading & Map Alignment — why the glyph pointed wrong, and the fix

Short, self-contained note. Applies to any app that draws a robot on a map
from odometry while also having GPS. Reusable outside this project.

## The core misunderstanding

There are **two different frames**, and they are rotated relative to each
other by an angle nobody told you about:

1. **World/odom frame** — the frame your `nav_msgs/Odometry` is in. Its
   origin and axes are decided by wherever the localization node started.
   Its `+x` is *not* east. `yaw` from odometry is measured in this frame.
2. **Geographic frame (ENU)** — east/north, i.e. the real world and the
   satellite tiles.

The rotation between them, call it **`theta`** (world `+x` axis, as a
bearing CCW from east), is unknown at startup. If you draw the robot using
odometry `yaw` directly on a north-up / tile-backed map, everything is off
by `theta`: the glyph heading is wrong and world coordinates don't line up
with the map. It looks like "a heading bug" but it is a **frame-alignment**
problem — the heading is just its most visible symptom.

## Why you cannot know `theta` at startup

`theta` is only observable once the vehicle **moves**: you compare how the
GPS track (in ENU) turns versus how the odom track (in world) turns. With
one stationary fix you know the *origin* (a translation) but not the
*rotation*. So there are two stages:

- **Translation-only (immediately):** from the first GPS fix you can place
  the map origin and show tiles. `theta = 0` is a placeholder — **do not
  trust heading yet.**
- **Rotation-aligned (after a few metres):** fit a rigid transform
  (Kabsch/Umeyama, scale = 1) between paired `(odom_xy, gps_en)` samples.
  Now `theta` is real and heading can be trusted.

## The fix (backend only)

In the georeferencer (`mcs/core/geo.py`):

- Emit a **translation-only `GeoFit` from the very first GPS fix** so
  `is_valid` is true immediately. This is what lets the satellite layer turn
  on "at the first sign of GPS" instead of waiting for motion.
- Add a **`heading_aligned`** flag: `False` for the translation-only fit,
  `True` once the Kabsch fit succeeds (enough motion, `min_spread_m`).
- Expose the alignment on `GeoFit`:
  - `world_heading_offset` → `theta` (add to a world-frame yaw).
  - `world_yaw_to_true(world_yaw)` → `world_yaw + theta`, normalised.

In the state store (`mcs/models/store.py`):

- `robot_true_heading()` returns the north-referenced heading
  (`world_yaw_to_true(yaw)`) once `heading_aligned`, else `None` so callers
  fall back to raw `yaw`.

In the map (`mcs/gui/map/map_view.py`) — QGroundControl model,
implemented:

- **The view is always north-up and never rotates.** This is the key point:
  rotating the whole view (an earlier approach) spins the tiles and the
  world with it — the wrong behaviour. Like QGC, the map stays fixed and
  only the **vehicle glyph rotates**.
- Two scene regimes, switched once by `heading_aligned`:
  - **Before alignment** — the scene is the raw robot world frame
    (world-up). The glyph is at raw `/blueboat/odom`, pointing at raw yaw.
    A "world-up (north unknown)" notice is shown.
  - **After alignment** — the scene is **local east/north (ENU)**. Every
    world-frame quantity (glyph position, trails, pinger, targets, mission
    paths, and the satellite tiles) is converted with
    `GeoFit.world_to_enu()` at placement, so north points up with no view
    rotation. The glyph is rotated to its **true heading**
    (`GeoFit.world_yaw_to_true(yaw) = yaw + theta`); an `N↑` badge appears.
- The glyph ignores view transforms (constant pixel size); its device-space
  rotation is `-degrees(heading)` where `heading` is already the scene-frame
  heading (true heading in ENU, raw yaw before). There is **no separate
  view-rotation term** — a single conversion at the source keeps the glyph
  and everything under it consistent, which is what fixes the "arrow always
  faces right / points nowhere" symptom.
- "Hardcoded paths always draw horizontal" is the same story: those paths
  are world-frame, whose `+x` **is** the robot's launch heading, so before
  alignment horizontal is *correct*; after alignment the ENU conversion
  renders them at their true geographic bearing.

The rule of thumb:

> Compute heading **after** you have both robot world info (odom `yaw`) **and**
> the odom↔GPS georeference — never from `yaw` alone.

## Guard rails

- Coordinate conversion that needs rotation (e.g. deploying a GPS-anchored
  mission into the current world frame) must **refuse a fit that is not
  `heading_aligned`** and retry — otherwise it silently rotates everything
  by the placeholder `theta = 0`.
- A manually entered GPS origin (no live GPS) is a *declared* frame: treat
  its rotation as given, not as heading-aligned evidence.

## One-paragraph recipe for the other app

Keep two fit stages. Turn tiles on as soon as you have any GPS fix
(translation-only). Keep a boolean that says whether rotation has been
solved yet. Keep the map north-up and fixed at all times. Before rotation is
solved, draw the glyph from raw yaw in the raw world frame and label it
"north not yet aligned". After, convert every point to east/north with the
one offset `theta` and rotate ONLY the vehicle icon to `yaw + theta` — never
rotate the view, or the satellite tiles rotate with it (the exact bug this
note exists to prevent). Solve `theta` by fitting odom-vs-GPS tracks after a
few metres of motion.
