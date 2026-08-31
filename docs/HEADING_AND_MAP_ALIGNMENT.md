# Heading & Map Alignment

**This document is superseded.** The alignment architecture it described —
an online odom↔GPS *rotation* estimate (`theta`, Kabsch fit), a
`heading_aligned` flag, and a two-regime map scene — has been removed. The
current architecture is documented in the repository root:

→ **[`GPS_MAP_ARCHITECTURE.md`](../GPS_MAP_ARCHITECTURE.md)**

## The current model, in four lines

- `/blueboat/odom` is **local ENU**: origin = launch point, axes East/North,
  yaw **absolute** (0 = East, CCW+) — on the real boat and in simulation.
- The map scene is local east/north metres about the first GPS fix; the only
  estimated quantity is a **translation** `t = EN(world origin)`, converged
  from the first few fixes with no vehicle motion.
- Glyph heading = compass (`radians(90 − compass_deg)`) if present, else the
  odom yaw **directly** — no `theta`, no correction, no fallback chain.
- Nothing is drawn before the anchor exists; there is exactly one scene
  regime and it never switches.

## Why the old model is gone (history)

The old design was internally coherent but rested on a false premise: that
the odom frame was rotated from ENU by an unknown launch-heading angle. The
robot's `robot_interface` in fact only *translated* position while re-zeroing
yaw — a hybrid frame. Against ENU positions the Kabsch fit converged to
`theta ≈ 0` regardless of launch heading, so the estimated angle could never
recover the real heading; `world_yaw_to_true` additionally carried a sign
error; the rolling refit could silently regress the fit and flip the scene
regime mid-mission; and the motion requirement deadlocked GPS-anchored
deployments (the boat held position waiting for a fit that needed motion).
The fix was to make the odom frame genuinely ENU at the source (stop
re-zeroing yaw) and delete the rotation machinery here. Commits prior to this
change reference the old model; read them against this paragraph.
