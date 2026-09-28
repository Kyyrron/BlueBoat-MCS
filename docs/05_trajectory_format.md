# 5 — YAML Trajectory Format Specification

Format tag: **`blueboat_trajectory/1`**. Produced by the Survey Pattern
Designer, consumed by the control stack's
`BlueBoat-Control/blueboat_control/src/_custom_libraries/yaml_trajectory.py`
next to `path_generation.py`. Two files per mission; the runtime never needs
the second one.

## 1. Runtime file — `<name>.yaml`

Contains only what mission execution requires. The design principle: the
navigation pipeline evaluates trajectories at arbitrary times
(`single_pose(t)`), so the file stores **dense, time-stamped samples** and
the runtime does nothing but linear interpolation. All interpolation
mathematics (arcs, splines, Béziers, …) is resolved at export time and
never runs in the control stack.

```yaml
format: blueboat_trajectory/1        # REQUIRED, exact string
name: harbor_survey                  # informative
generator: mission-pattern-designer/1.0
created: "2026-07-15T10:12:03"       # informative
frame: world                         # the ROS local/world frame of the stack
speed: 0.5                           # m/s used for time-parameterization
loop: false                          # true => evaluation wraps t modulo duration
length_m: 118.402                    # informative
duration_s: 236.804                  # informative (= points[-1][0])
points:                              # REQUIRED, ordered by strictly increasing t
  - [0.0,   0.0,    0.0,   0.0]      # [t (s), x (m), y (m), yaw (rad)]
  - [0.5,   0.25,   0.0,   0.0]
  - [1.0,   0.5,    0.0,   0.0]
  # ...
```

Semantics:

* `points` rows are `[t, x, y, yaw]` in the world frame, yaw CCW about +z
  (identical convention to `master_control` / `as_euler('xyz')`).
* Evaluation at time `t`: binary search, linear interpolation of `x`/`y`,
  angular interpolation of `yaw` (shortest way, wrap-safe).
* `t` beyond the last sample: **clamped** to the final pose — the same
  "default to last known point" convention as the hard-coded trajectories —
  unless `loop: true`, in which case `t` wraps modulo `duration_s`.
* `speed` is the mission cruise speed; per-segment speeds set in the
  designer are already baked into the sample timing (`t` column), so the
  runtime needs no per-segment knowledge;
* `z`, `roll`, `pitch` are always 0 for this surface vehicle; a future
  `blueboat_trajectory/2` may extend rows to 7 columns — readers must reject
  unknown `format` values rather than guess (the provided loader does).
* Sample spacing is an exporter choice (default 0.25 m); readers must not
  assume uniform `t` steps.

### Optional `geo_anchor` block — GPS-anchored missions

```yaml
geo_anchor:
  lat0: 33.660196      # GPS of the design frame's (0, 0)
  lon0: 130.657780
  theta_deg: 0.0       # always 0 — the design frame IS local east/north
```

`theta_deg` is retained for backward compatibility only. The designer has
always drawn in a local ENU frame, so every anchor it writes carries `0.0`; a
legacy file with a non-zero value is still honoured (its points are rotated
about its own anchor at deploy time, and the designer warns when it opens one).

`points` stay in the design frame. Because the robot's world origin is
created wherever `robot_interface` starts, an anchored mission is **never
executed directly**: at launch the station points `path_generation` at a
*deployed* file (`<dir>/.deployed/<name>.yaml`) that does not exist yet;
`path_generation` holds position (station-keeping fallback) and re-checks the
file on every path request. Once the run's odom↔GPS fit is established — a
second or so of fixes, **no vehicle motion needed**, since only a translation
is estimated — the station converts every sample design-frame → GPS → today's
world frame, writes the deployed file (with `deployed_from` /
`deployed_fit_rms_m` provenance fields and no `geo_anchor`), and the robot
transitions onto the true-GPS path. Every waypoint lands on its real-world
coordinates regardless of where the robot was switched on. The anchor also serves the editor: it is
the remembered GPS origin restored when the mission is reopened, and — since
2026-09-09 — the *only* origin the editor uses for that file. The designer
does not consult the station's live georeference, so opening and re-saving an
anchored mission while the boat has a GPS lock leaves `geo_anchor` byte-for-byte
unchanged.

Start-alignment note: a non-GPS runtime file is normally *start-aligned* —
its first sample is `(0,0)` and the first tangent is `+x` — so that on
launch (world frame zeroed at the boat) the robot starts at the mission and
moves forward. GPS-anchored files are NOT start-aligned: their `points`
stay in the design frame and are relocated to the boat's real GPS position
at deploy time instead.

## 2. Editor metadata file — `<name>.meta.yaml`

Everything needed to *re-edit* the mission, and nothing the runtime reads:

```yaml
format: blueboat_trajectory_meta/1
model:
  name: harbor_survey
  comment: "east breakwater, spring campaign"
  speed: 0.5
  loop: false
  items:                       # ordered; waypoint or group
    - type: group
      uid: 4
      name: Lawnmower
      pattern: lawnmower       # generator key ('group' = manual grouping)
      params: {width: 20, height: 12, spacing: 3, x0: 0, y0: 0, ...}
      locked: false
      children:
        - {type: waypoint, uid: 5, name: Lawnmower.1, x: 0.0, y: 0.0,
           locked: false,
           seg_out: {kind: straight, params: {}, speed: 0.0}}
        # ...
    - {type: waypoint, uid: 12, name: WP9, x: 14.0, y: -6.0, locked: false,
       seg_out: {kind: bezier, params: {c1_frac: 0.33, c1_angle_deg: 30, ...},
                 speed: 0.35}}
```

`seg_out` is the interpolation of the segment **leaving** that waypoint
(kinds: `straight`, `sine`, `arc`, `spline`, `bezier`; registry in
`mcs/designer/interpolation.py`), plus its `speed` in m/s — `0.0` means "use the
mission cruise speed". Per-segment speeds are already baked into the runtime
file's `t` column, which is why the control stack never reads this.

If the metadata file is missing, the editor re-imports the runtime samples as
plain waypoints (decimated), so a mission is never unopenable.

## 3. Selecting a YAML trajectory at launch

No launch-file change is required — the file path rides inside the existing
`trajectory` argument:

```bash
ros2 launch blueboat_control BlueBoat_launch.py \
    controller_type:=LoS trajectory:=from_yaml:/home/op/.config/blueboat_mcs/trajectories/harbor_survey.yaml
```

The station's Launch Mission dialog builds this string automatically when a
"custom: …" entry is selected. An optional dedicated `yaml_path` node
parameter is also supported (`trajectory:=from_yaml yaml_path:=<path>`) for
hand-written launch files.
