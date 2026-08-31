# GPS-Only Map Architecture

How this ground station draws a live robot on a real-world (satellite) map
without the map ever lying — and how to apply the same architecture to any GUI
that places a GPS-equipped vehicle on a map. Written so another agent can port
it to a different project without reading this repository.

This replaced an architecture that estimated an odom↔GPS **rotation** online
(Kabsch fit over a rolling window, two scene regimes switched by a
"heading-aligned" flag). That design failed in the field in every way that
matters: clicks landed tens of metres off, GPS-anchored missions deployed
rotated by the launch heading or deadlocked, the imagery slid under the
overlays, and the vehicle glyph disagreed with its own trail. Section 7 lists
each trap so you don't rebuild it.

---

## 1. The principle

**One scene frame, fixed forever: local east/north metres about a latched GPS
origin. North-up, east-right. The view never rotates — only the vehicle glyph
does. Nothing is drawn until the frame is anchored to GPS.**

Everything on the map — vehicle glyph, trails, targets, planned paths,
satellite tiles — is placed in that one frame. Every mouse position is read
back out through the exact inverse of the placement transform. There is no
second regime, no mode switch mid-mission, and no rotation estimated from
data.

Mission-start sequence (the user-visible contract):

1. Wait for the first GPS fixes → latch the projection origin `(lat0, lon0)`
   and estimate the anchor translation (seconds, no vehicle motion needed).
   Until then the map shows a "waiting for GPS fix" notice and **nothing
   else** — an empty map is honest; a wrongly-rotated one is not.
2. Draw the satellite tiles (axis-aligned, north-up).
3. Draw the vehicle at its position, rotated to its **true** heading.
4. Convert every other product (planned path, targets, trails) into the same
   frame and draw it.

## 2. The one precondition: the vehicle's local frame must be ENU

The whole architecture rests on the vehicle publishing its odometry in a
**local-ENU** frame: origin anywhere (typically the power-on/launch point),
but axes East/North and yaw **absolute** (0 = East, counter-clockwise
positive). MAVROS `/mavros/local_position/odom` is already exactly this;
Gazebo's world frame is too.

If your vehicle republishes odometry through any node, audit that node first.
Ours re-zeroed yaw at boot (`yaw − yaw0`) while only *translating* the
position — producing a hybrid frame (ENU axes, launch-relative heading) that
was self-consistent **only when the vehicle booted facing East**. Every
downstream consumer inherited a constant heading bias; trajectory following
diverged for launch headings ≥ 90°. The fix was one deletion: stop subtracting
`yaw0`; keep the position translation. A frame is either rotated as a whole
(position **and** yaw) or not at all — never one without the other.

With that precondition met, the map problem collapses: the only unknown
between vehicle-world and GPS is a **2-D translation**.

## 3. Data sources

| Source | Rate | Content | Notes |
|---|---|---|---|
| Odometry (`/blueboat/odom`) | ~20 Hz | position (local ENU metres), absolute ENU yaw, body-frame velocity | the smooth pose used for the glyph and trails |
| GPS (`/mavros/global_position/global`, NavSatFix) | ~5 Hz | lat/lon | BEST_EFFORT QoS; `lat==0 && lon==0` means *no fix* — discard it |
| Compass (`/mavros/global_position/compass_hdg`) | ~5 Hz | heading, **degrees clockwise from North** | convert once: `yaw_enu = radians(90 − hdg)` |

Heading priority for the glyph: compass first (magnetometer, absolute,
available immediately), else the odom yaw **directly** (it is absolute ENU —
apply no correction). Show the same value in any text panel that says
"Heading", so the panel and the glyph can never disagree.

## 4. The one transform

Define it once, in one class, and route every conversion through it:

```
EN = world + t          world = EN − t          t = (tx, ty)
```

- `EN` = metres east/north of the latched origin `(lat0, lon0)`
  (equirectangular projection, below).
- `world` = the vehicle's odom frame.
- `t` = the EN position of the vehicle's odom origin. That's it. No angle.

**Latching the origin**: the first accepted GPS fix becomes `(lat0, lon0)`
and never changes for the session. All EN math is relative to it.

**Estimating `t` online**:

- Pair each **GPS fix** with the **concurrent** odom position — pair at GPS
  rate, and only when the newest odom sample is fresh (< 0.5 s). Never pair
  at odom rate against the last-known fix: reusing one 5 Hz fix against four
  20 Hz poses biases the estimate.
- Over a rolling window (we use 180 s): `t = median(EN_i − world_i)`,
  per axis. The median makes one GPS glitch harmless.
- Health figure: a robust spread of the residuals
  (`1.4826 · median(‖(EN_i − world_i) − t‖)`). Above a threshold (we use
  6 m) the anchor is declared invalid and the map hides again — a sustained
  odom/GPS disagreement means something is genuinely wrong, and a misleading
  map is worse than none.
- Validity: `n_pairs ≥ 5` and health under the threshold. That's ~1 second
  of GPS. **No vehicle motion is required** — a translation is observable
  from a stationary vehicle, which is precisely what the old rotation fit
  could not do (and why GPS-anchored missions deadlocked: the boat held
  position waiting for a fit that needed motion).

**Why no rotation estimation**: with an ENU odom frame the true rotation is
zero by construction, so a fitted angle is pure noise; it needs motion to
converge; it wanders with every refit (the whole scene slides); and its
placeholder value before convergence (θ = 0 — indistinguishable from a real
answer) is exactly the wrong number everywhere except one lucky heading. If
you find yourself fitting an angle between your vehicle frame and ENU, fix
the vehicle frame instead.

**Lat/lon ↔ EN** (equirectangular, fine below ~10 km spans):

```python
R = 6378137.0  # WGS84 equatorial radius, metres
east  = radians(lon − lon0) · R · cos(radians(lat0))
north = radians(lat − lat0) · R
# inverse: solve the two lines for lat, lon
```

## 5. Placement pipeline (drawing things)

- **Scene = EN metres.** The Qt view applies `scale(s, −s)`: the y-flip lives
  in the **view transform**, never in the data, so scene +y is north-up on
  screen. The view transform never gains a rotation.
- **Every world-frame item** (glyph position, trails, targets, mission path):
  `scene = world + t`. One helper (`_to_scene` / vectorised `_scene_points`),
  used by every placement site.
- **Glyph rotation**: the true heading (Section 3) *is* the scene heading —
  draw it directly. (Mind your framework's screen-space convention: Qt's
  `setRotation` is clockwise in device space, so negate a CCW-from-east
  angle.)
- **Satellite tiles** (web-mercator XYZ / "slippy" tiles): for each tile,
  convert its NW corner lat/lon → EN about `(lat0, lon0)`, translate there,
  scale by metres-per-pixel at the tile's centre latitude
  (`2πR·cos(lat)/(256·2^z)`), with a negative y-scale (tile pixels go south).
  **Axis-aligned, no rotation, ever.** Pick the zoom so one tile pixel ≈ one
  screen pixel.
- **Simulation modes** (two, both the same drawing code):
  - *No GPS* (mission not anchored to GPS): the sim world is already ENU, so
    use the identity (`t = 0`), draw immediately, keep tiles off.
  - *Simulated GPS* (GPS-anchored mission): synthesise the receiver instead
    of special-casing the map. A pure model converts world metres →
    lat/lon by translation from the vehicle's **first** position
    (`fix = latlon(world − world_first + noise, origin)`), with the origin
    placed a fixed offset (we use 10 m north) from the mission's first
    point, small Gaussian noise (σ ≈ 0.4 m) and the real fix rate (5 Hz).
    Publish it on the *real* GPS topic and let your own subscription receive
    it back — anchor gating, tiles (real imagery of the planned location),
    diagnostics and deferred deployment then run the identical real-water
    path, which is what makes offline-planned GPS missions rehearsable
    before a field trial. Spawn the vehicle with a random heading to also
    rehearse anchoring at arbitrary orientations.

## 6. Interaction inverse (reading things back)

Every mouse position arrives in scene coordinates and must come back through
the **exact inverse** of the placement transform before it is used for
anything real:

- **Command clicks** (our manual target): `world = scene − t`, then publish.
  This is the one conversion that steers the vehicle — get it wrong and the
  vehicle drives to a point that isn't the one clicked, while the on-screen
  crosshair (placed through the forward transform) looks perfectly correct.
  Refuse the click entirely while the anchor is not valid.
- **Read-outs** (inspector, measure endpoints): scene → world for frame
  numbers, scene → lat/lon (`local_en_to_latlon(sx, sy, lat0, lon0)`) for
  GPS numbers.
- **Distances** may be computed in scene space — the transform is rigid, so
  they are identical either way.
- **Re-centre on vehicle**: `centerOn(world + t)` — the forward transform,
  the mirror image of the click inverse.

Regression-test the round trip with a **non-trivial `t`** (ours asserts
`|t| > 1 m`): with `t = 0` every one of these bugs is invisible.

## 7. Pitfalls (each one was a real field failure here)

1. **Hybrid vehicle frames.** Re-zeroing yaw without rotating position (or
   vice versa) makes a frame that agrees with ENU at exactly one boot
   heading. Symptom: "missions only work when the robot starts facing East".
   Audit the odometry republisher before touching the GUI.
2. **Estimating rotation you could define away.** See Section 4. Symptoms:
   map "feels weird", scene slides at each refit, behavior depends on how
   far the vehicle has moved, deploy logic deadlocks waiting for motion.
3. **Two scene regimes.** A "rough frame now, correct frame later" switch
   strands everything placed before the switch, can oscillate back, and
   means clicks made on early imagery are silently wrong. One frame, gated
   on readiness: draw nothing until the frame is right.
4. **Compass vs ENU angle conventions.** Compass is degrees **CW from
   North**; math is radians **CCW from East**. Convert once, at ingestion
   (`radians(90 − hdg)`), and never again downstream. A leftover `−π/2` in a
   conversion helper cost us a 90° error in every logged GPS target.
5. **Pairing mixed-rate streams.** Pair at the *slow* stream's rate with a
   freshness guard on the fast one. Reject the (0,0) "no fix" NavSatFix.
6. **Trusting `is_valid` flags that hard-code their quality figure.** The old
   translation-only fit reported `rms = 0.0` by fiat, so "valid" meant
   nothing. Compute the health figure from actual residuals, robustly.
7. **Sentinel values on command topics.** Our `[0,0]` on the target topic
   means "resume mission", so a genuine origin click is nudged by 1 mm.
   Know your wire contracts before publishing raw clicks.
8. **Equirectangular limits.** The flat-earth projection above is
   centimetre-accurate at harbour scale but degrades over ~10 km spans or
   near the poles; switch to a proper local projection (UTM, ENU via
   pyproj) if your operating area is larger.
9. **Stale peers.** If the vehicle-side frame fix and the GUI fix ship
   together, a vehicle running the old build silently reintroduces the
   hybrid frame — there is no version handshake on a ROS topic. Rebuild and
   redeploy the vehicle workspace, and verify heading behaviour on the
   water before trusting the map.

## 8. What this bought us

- Manual target clicks land where clicked (pure translation, exact inverse).
- GPS-anchored missions deploy within seconds of the first fix, stationary —
  the deploy transform is design-EN → lat/lon → world-EN, translations only.
- Trajectory following works from any launch heading (vehicle-side fix).
- The map is north-up with correctly-aligned imagery from the moment it
  appears, and it never rotates, slides, or switches modes afterwards.
- Real water and simulation run the identical drawing code; a non-GPS sim is
  just the identity anchor with tiles off, and a GPS-anchored mission can be
  rehearsed end-to-end in sim through a synthesised receiver (Section 5) —
  anchor, tiles, deferred deployment and all.
