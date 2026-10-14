# 4 — User Guide

The station is the single operator interface during an experiment. It stays
open the whole time; missions are launched, stopped and relaunched from it.

**On the real boat a launch is a run session** (since 2026-10-09): you connect
once, the boat goes into override and stays controllable from the station —
Manual Target or the arrow keys — and you start and stop as many **runs** as
you like (each logged on its own) without ever handing the boat back to
QGroundControl. A run session in four steps:

1. **Launch Mission** → *Real robot* → choose the first run's path (or the
   pinger), controller, motors, log note → **Launch**. The boat connects and
   goes into override but **does not move and nothing is logged**.
2. Drive it wherever you want with **Manual Target** or **Control using
   arrows**. A GPS-anchored path is already drawn on the map, its start
   circled **S**.
3. **START RUN** (left panel, under ROS diagnostics): the path is written —
   placed at the boat if it is not GPS-anchored — logging starts and the boat
   follows at once. **STOP RUN** stops it (zero thrust, still in override) and
   saves the run's log and report. Change the next run with **Update
   Mission**; repeat.
4. **Turn Off Connection** ends the session: a run still going is stopped
   first, then `default` is published and every node is shut down.

A Gazebo simulation keeps the classic flow: launch = mission, Stop Mission ends
it.

## Screen layout

Left: the link guardian, live information and diagnostics. Center: the
interactive mission map. Right: map layers and tools, then monitoring (distance
plot, timeline, statistics). Bottom: the mission
control toolbar and its one-line launch console. Status bar: last click
inspection, georeference quality, ROS connection state.

## Left panel

**Guardian** (v1.1, real-robot sessions) — the **link guardian** brings the boat
back to HOME by itself when this laptop goes silent (WiFi lost, laptop switched
off, station crashed). It is an ArduPilot script on the boat, installed once
(superproject `link_guardian/README.md` §3), and it is **ON by default in every
real-robot session**: at connection the control stack arms the autopilot before
going to override, and the station starts its companion, which switches the
guardian ON. The box shows a one-line state (ON / OFF / NOT READY / TAKEN OVER /
LINK LOST), the link, the autopilot (armed · ArduPilot mode — the autopilot's
own arming, not the motor gate), the guardian flag, the setup check (tooltip:
every item), HOME (session start, or the current run's start) and the boat's
last guardian / pre-arm messages. Buttons, each enabled only when it can act
(the tooltip says why not):

* **Check** — re-run the readiness check (script, failsafe parameters, laptop
  identity).
* **Apply settings** — write the recommended ArduPilot failsafe parameters
  (one-time setup; restart the autopilot afterwards if scripting was off).
* **Turn OFF / Turn ON** — the guardian for this session (asks before turning
  OFF: the boat would then drift on a link loss).
* **Arm again** — re-arm the autopilot (the boat briefly leaves override);
  needed after an E-STOP, which disarms, or when arming failed at connection.
  The guardian also arms the boat itself if it has to take over.
* **Take back control** — after the guardian (or an ArduPilot failsafe) has
  taken the boat: HOLD, thrusters back to the station, MANUAL. Press STOP RUN
  first if a run was active.

The E-STOP does **not** switch the guardian off. **Turn Off Connection** and
closing the station ask, while the guardian is ON, whether to turn it **OFF and
disarm** first (the boat stays put), to **keep it ON** (the boat sees the laptop
go silent and returns HOME), or to cancel. Outside a real-robot session (Gazebo,
bag replay) the box is greyed.

**Robot** — world coordinates, GPS (when fixed), heading, speed, active
controller, mission state (an aggregated readiness count over FCU connection,
odometry flow and controller-ready), left/right motor commands in Newtons,
total travelled distance and mission elapsed time.

**Pinger** — world coordinates, robot-frame coordinates, live robot↔pinger
distance, and the age of the last raw USBL packet (green < 3 s, orange < 10 s,
red beyond).

**Target** — the active steering mode (path following / pinger homing /
MANUAL TARGET) and the live distance to that target. The value comes from the
controller's own published target; nothing is recomputed.

**ROS diagnostics** — one row per monitored topic with a colour LED
(green OK / orange late / red stale / grey never seen), measured rate against
the expected rate, and the age of the last message. A communication problem is
visible within one second.

**RUN** (real-robot run session only; pinned under the panel, so it never
scrolls away) — what the next run executes (**Next run**: the path, marked
*GPS* or *placed at start*, or the USBL pinger), the **Controller** as
`master_control` reports it (*loaded*, *loading…* while MPC builds its solver,
*error* with the reason), the **State** (*Connected — no run yet*, *Run 3 —
2:31*, *Connected — last run 3*, *E-STOP latched*), the **Log** (the run's
CSV once `robot_interface` confirms it — red *not confirmed* means field data
may be missing — or *saved: …* after the run) and the **Bag** (the run's
ros2bag: *recording*, *saved: …-bag* after the run, *armed* between runs, *off*
when the launch disabled it; red *not recording* means the run will have its
CSV but cannot be replayed). One big button:

* **▶ START RUN** (green) writes the run's path — a non-anchored path is
  *placed at the boat*: it begins exactly where the boat is and runs out ahead
  of it, along its heading; a GPS-anchored path is re-deployed with the current
  GPS fit and the boat drives to its **S** — then starts following and logging
  immediately. Greyed out, its tooltip says why (controller loading, no path
  chosen, GPS not anchored yet, E-STOP latched, no odometry, …). It also ends
  arrow control and clears a manual target: you asked for the run now.
* **■ STOP RUN** (amber) ends the run: following stops, the boat holds zero
  thrust **and stays in override**, the run's CSV is closed and filed with its
  report, the controller log is written. The display freezes on that run
  (trail, statistics, distance plot, timeline) until the next START RUN.

## Mission map

Drag to pan, mouse-wheel to zoom (anchored under the cursor). The grid's
scale bar in the bottom-left corner shows the current cell size in metres.
The map is always **north-up and fixed** — it never rotates, and it is
**GPS-frame-only**: on real water nothing is drawn until the first GPS fixes
anchor the frame (a "Waiting for GPS fix" notice shows — typically a few
seconds, no driving needed), then tiles, glyph and overlays appear together,
geographically placed, with the glyph at its true heading. In a simulation
of a **GPS-anchored mission** the station simulates the GPS itself, so the
map behaves exactly as above — including satellite imagery of the location
the path was planned at (the boat's first fix appears 10 m north of the
path start, and the boat spawns at a fixed heading — east by default — at
Gazebo (0, 0), the same every run). In a simulation
without a GPS-anchored path the map draws immediately (no tiles — there is
no GPS). Only the
vehicle icon rotates, exactly like QGroundControl. A briefly empty map right
after a real-water launch is expected, not a fault; manual-target clicks
during that window are refused rather than sent somewhere wrong.
**Replay** — during a *From a ros2bag* launch the top-left corner holds the
**REPLAY** box instead of the sea state: the recorded **Run** (and whether it
plays in the empty world or a personalized one), the **Time** played against
the run's duration with a progress bar, and the **Status** (*playing at 2×*,
*paused*, *finished — holding the last pose*, *waiting for Gazebo…*). **Pause /
Resume** freezes the boat where it is (its odometry and GPS keep arriving, so
nothing goes stale) and the rate combo changes the playback speed live. Manual
Target and the Default/Override toggle are greyed during a replay: nothing
commands the replayed boat.

**Sea state (sim)** — a floating box in the **top-left corner of the
map**, the mirror of the mission-statistics box in the top-right. It
appears only while a Gazebo mission runs (not a replay), all read back from the
simulator:

| Row | What it says |
|---|---|
| Current | the current at the boat right now, then its configured mean ("0.31 m/s from N · mean 0.30 from N (natural)") |
| Waves | strength, period and direction ("Light chop (0.15 m), every 2.8 s, from NE"), plus a passing wake while one rocks the boat |
| Boat motion | what the sea does to the hull over the last 30 s: "roll ±1.4°, pitch ±1.1°, heave ±4 cm", or "level and steady" |
| Next change | the next planned change of a timeline, or "changing now (40 %)" while one is being blended in |
| Status | live / stale; amber "· hull: Gazebo box" if the simulator could not put its catamaran hull under the boat (the runs are validated on that hull) |

**Change sea state…** opens the same dialog as at launch, in live mode:
set the new current / waves (or a plan) and how long to blend them in;
**Apply now** sends it and the simulator changes gradually.

A simple click anywhere shows, in the status bar: world coordinates, GPS
coordinates (once georeferenced) and the live distance from the robot to that
point; the point is marked on the map.

**Manual Target** (toolbar button): a one-shot arming control. Press it,
then click the map once — that point is published as the target on
`/blueboat/manual_target` and the button disarms itself, so the map
immediately returns to normal interaction: you can pan, inspect points and
measure distances while the boat drives to the target. The target stays
highlighted with a crosshair and a dashed purple line shows the
*approximate* LoS path, re-simulated live. To replace the target, press
**Manual Target** again and click a new point; pressing it while armed
cancels arming and publishes nothing. While a target is active, a
**Continue Original Mission** button is shown: it does exactly one thing —
publish `[0.0, 0.0]`, which hands control back to the mission — and the
target highlight is cleared. Between the runs of a session there is no
mission to go back to, so the same button reads **Release Target**: the boat
lets go of the target and holds zero thrust. When the boat arrives (≤ 1 m), a
"Manual Target Reached" banner appears at the top of the map. During a run a
manual target **pauses** the run; Continue Original Mission resumes it where
it stopped.

**Start marker** — the mission path's first point is circled **S**: where a
GPS-anchored run begins (the boat drives there first), or, for a placed path,
the boat's position at START RUN.

**Control using arrows** — the floating **ARROW CONTROL** box in the
**bottom-right corner of the map** (shown whenever a launch with a controller
is running, real or Gazebo). Press **Control using arrows**: the box turns
amber and the keyboard drives the boat —

* **↑** ahead / **↓** astern: both thrusters at the **Power** set on the
  slider (0–20 N, default 10 N, applied as you move it);
* **←** / **→**: turn counter-clockwise / clockwise by adding ± the
  **Rotation** slider (3–15 N, default 5 N, applied as you move it) per side to
  whatever ↑/↓ commands (→ alone at 5 N: left +5 N, right −5 N; ↑ at 6 N with →
  at 5 N: left 11 N, right 1 N). Opposite keys cancel. A side above 20 N is
  scaled down by `master_control` together with the other, so the turn keeps
  its direction. The live command is shown as `L … N  R … N`. Below about 2 N
  per side the propellers do not turn (ESC deadband).

It **pauses** a running run exactly like a manual target (the logged target
stays the last one), and works between runs too. Press the button again to
stop: the boat **holds position where you released it** (a manual target at
that point — crosshair and *Continue Original Mission* / *Release Target*
appear). Clicking a manual target, START RUN or the end of the launch also end
arrow control. Safety: the keys are only read while the station window is the
active window and no dialog is open; switching window forgets every held key;
if the station stops streaming, `master_control` zeroes the thrust within
0.5 s and stays under arrow control rather than resuming the run on its own.

**Measure** (toolbar button): first click sets point A, the line and distance
follow the cursor, second click freezes the measurement; the status bar shows
distance and both coordinates. Deactivating the tool clears any frozen
measurement from the map.

## Right panel

**Layers** — checkboxes toggling every map layer: satellite imagery, robot
trajectory, published mission path, pinger position and trajectory, the thin
robot→target line and the metric grid (the robot heading arrow was removed in
v1.1 — the boat glyph itself points along the heading). The satellite box
unlocks as soon as the map frame is anchored — a second or so of GPS, no
driving needed; watch "georef" in the status bar.

**Map tools** — **Zoom +**, **Zoom −** (view-center anchored), **Center
Robot**, which recenters the view on the boat exactly once (the camera then
stays completely free — it is never a follow mode), and **Clear Paths**, which
wipes the robot and pinger trails from the map without touching recorded data.

**Live distance** — robot↔current-target distance versus experiment time; the
title states whether the target is the pinger, the path target or a manual
target. The current value is highlighted at the live edge. In a run session
the live distance, the timeline, the statistics and the map trail cover the
**current or last run** only: they reset at START RUN and freeze at STOP RUN
(nothing is recorded between runs; the boat, target, pinger and path stay
live).

**Mission timeline** — a dual-handle slider selecting the displayed time
window for both the plot and the map trajectories. With the right handle at
the end, the display is *live* and follows the experiment. Drag it back to
freeze on any window (e.g. 5 min → 10 min) while recording continues
underneath; **Go Live** snaps back to the present.

**Mission statistics** — duration, travelled distance, average and maximum
speed computed over the selected window, plus the active controller.

**Launch console** (lower part of the panel, resizable via the splitter) —
the complete stdout/stderr of the launched ROS2 processes, exclusively.
Lines are order-preserving, auto-scroll while you are at the bottom (scroll
up to read history without pausing capture), stay memory-bounded across long
experiments, and are tinted by severity (`[INFO]` normal, `[WARN]` orange,
`[ERROR]`/tracebacks red). The toolbox above it filters the display by
category matching `master_control`'s own messages — Everything / Targets
only / Thrust only / Warnings & errors — plus a free keyword field; filters
never affect what is captured.

## Bottom toolbar

**Launch Mission** opens the configuration dialog. The **Mission type**
selector chooses between the *Real robot* launch and the *Gazebo simulation*
(`Sim_launch.py`). Real robot — a **run session**: the controller of the
first run (PID / LoS / MPC; MPC cannot follow the pinger and is greyed out
with it), the first run's trajectory (designer paths only — built-in shapes
are for simulation and the CLI), use-pinger, motor enable (always
re-confirmed, with a second warning dialog), a log note and free-form extra
launch arguments. On **Launch** the boat connects and goes into override but
does not move and logs nothing until START RUN. Gazebo
simulation: robot file, trajectory, controller and the same **log note** —
the simulation always runs a controller, and only the motor and pinger
fields are hidden because they do not exist in that graph; mission-state
readiness shows "(sim)" and does not wait for a flight controller. The log
note lands in the run's position-log file name just as in a real-robot run
(`{date}-{note}-poslog.csv`), with a simulated run always tagged `sim`
(`sim-<note>`), so a Gazebo log can never be mistaken for a field record.
Selecting a GPS-anchored custom path in simulation additionally arms the
simulated GPS feed and passes a fixed `spawn_yaw` (shown in the status bar
and the console command line) — the boat spawns at the same pose every run,
which is what makes two simulated runs comparable; `spawn_yaw:=<radians>`
in Extra args overrides it for one run. On OK the station runs the ROS2 launch file; the LED turns
orange while nodes come up and green once the required ones report
(FCU connected + odometry flowing). Launch output streams to the console line.

**From a ros2bag — replay a recorded run (v1.1).** In *Gazebo simulation*
the **Source** row chooses between *Controller* (the simulation above) and
*From a ros2bag*. The latter replaces the controller, trajectory, robot file
and note fields with **Recorded run** — a run folder of
`~/ros2_ws/data/Robot_data/` (or the `…-bag` folder inside it; **Browse…** opens
there, and the line under it summarises the bag: duration, date, topics) — and
a **Playback rate**. Every real-robot run since v1.1 has such a bag. On
**Launch**, the world dialog below lists the personalized worlds the run's GPS
track passes through; there is **no sea-state step**. The boat then moves
*exactly* as it did in the water — position, roll, pitch, yaw, heave, speed —
and the map shows the recorded GPS, compass, targets and thrust as if live; in
the empty world the replay uses the run's own launch frame, in a personalized
world the boat sits at its recorded GPS position in the world's geometry.
Wind, current and waves cannot push it: the replayed boat takes no force at
all. The command line is `ros2 launch blueboat_control Replay_launch.py
bag:=… rate:=… [world_dir:=…]`.

**Personalized Gazebo worlds.** After OK on a GPS-anchored custom path in
simulation, a second small dialog lists every generated world
(`~/worlds/<path>/<world>/`, made with the SSS-Sim World Builder) whose
limit rectangle contains at least one point of the selected path — any
matching world qualifies, not only those built from this exact path.
**Launch in World** runs the mission inside it (simulated seabed,
side-scan sonar and GPS from the world's own anchor); **Empty Gazebo**
launches exactly as before; **Cancel** aborts the launch. In a
personalized world the boat spawns at the world's origin heading east and
the controller drives it onto the path. When no world matches (or
`~/worlds` doesn't exist) the dialog is skipped entirely.

**Sea state.** After the world step, every Gazebo launch shows a third
dialog, *What the simulated boat will feel*, with two cards. Leave both
off for still, flat water (a controlled bench run — the default).

* **Current — moving water** (tick to turn it on): the **mean speed**
  (or one of the Weak / Moderate / Strong shortcuts), the compass
  direction it **comes from** (16-point combo synced with a degrees box;
  the hint says where it pushes the boat), and the **Variation**: *Steady*
  (±5 %, for controlled comparisons), *Natural* (±10 % speed, ±10°
  direction over a minute or two, weak eddies a few tens of metres
  across) or *Gusty / eddying* (±20 %, a stress level). You set averages;
  the simulator adds the smooth variation itself — never a jump.
* **Waves — boat motion**: the **strength** (Ripples 0.05 m / Light chop
  0.15 m / Choppy 0.35 m / Rough 0.6 m — stress test, or a custom height),
  the direction they **come from**, the **period** (automatic from the
  height, shown, or set by hand), the **pattern** (short-crested harbour
  chop or long-crested swell) and **passing wakes** (none / occasional /
  frequent). Each choice shows one sentence on what the boat and the
  sonar will experience.
* **Seed**: the same seed and the same settings give the same sea, wave
  for wave; **New** draws another.
* **Plan changes over time…** (advanced) opens a planner: a list of
  **segments**, each a full current + waves setting (the same two cards)
  from its start time, with how long the change into it takes; a strip
  draws the planned current speed and wave height against time. Plans
  are saved by name under `~/.config/blueboat_mcs/sea_states/` and
  reloaded from the combo; older timelines open as segments.

**Launch** writes the choice as the simulator's scenario file
(`last_launch.yaml` in the same folder) and starts the mission; in the
empty Gazebo world a companion `sea_state_launch.py` process carries it
(its output is prefixed `[sea]` in the console) — calm water included,
because that process also floats the boat on the simulator's hull; in a
personalized world
it is part of the world launch. No literature references are shown in
the dialog — they live in the simulator's `docs/sea_state_model.md`.

**Update Mission** (the Launch button, while a real-robot session is
connected, between runs) opens a small window choosing what the **next run**
executes: **Follow** a path or the USBL pinger, the **Path** (every saved
designer path, re-read each time the window opens — a path you just drew in
the Survey Pattern Designer is there), the **Controller** and the **Log
note**. **Apply to next run** sends it at once: a controller change is loaded
right away (MPC can take up to a minute after an MPC tuning change; the RUN
box shows *loading…* and START RUN waits), a GPS-anchored path is drawn at its
real position with its **S** as soon as the GPS frame is anchored, a
non-anchored path is placed at the boat only when its run starts. Disabled
during a run.

**Stop Mission** first publishes `default` on `/blueboat/input_str` and waits
for the boat to confirm it left override, then shuts every launched node down
gracefully (SIGINT first) and releases the process; the station remains open and
the mission can be relaunched. It is the only button that terminates anything.
In a real-robot session it reads **Turn Off Connection**: a run still going is
stopped first (its log is saved by the control stack), then the same sequence.

**E-STOP** and **E-STOP + Stop Override** are two direct, one-click emergency
buttons (no confirmation dialog). **Neither ends the mission** — that is Stop
Mission's (Turn Off Connection's) job alone. In particular E-STOP + Stop
Override is *not* Turn Off Connection: it disarms and latches first and keeps
the session connected; Turn Off Connection does not disarm and ends it. A run
in progress keeps logging through an E-STOP (the CSV's `actuation_state` shows
the stop); press STOP RUN when you want it to end.

**E-STOP** publishes `stop`. `robot_interface` then zeroes the thrust, closes the
motor gate, disarms, and **latches**: nothing moves again until an explicit
`enable`, even though the controller keeps commanding. It stays in override and
leaves every node running, so the mission can be resumed rather than restarted.
Confirmation comes back as the robot withdrawing its readiness, and the label
beside the buttons reports each phase.

**E-STOP + Stop Override** does the same `stop` first — motors dead before
anything else changes — and only then publishes `default` to hand the servo
mapping back to QGC / the RC receiver, waiting for the `param_mode` echo. Use it
when you want the boat drivable from the transmitter again. It does not stop the
mission either.

**Publish Default Control Mode** publishes the same `default` command
immediately; the button then alternates to **Publish Override Control Mode**
(publishing `override`) and back on each click.

**After an E-STOP**, the boat ignores all thrust until it is released. In a
real-robot session launched with motors enabled, an amber **Release E-STOP**
button appears next to E-STOP while the latch is on: it publishes `enable`,
which clears the latch **and turns the motor gate back on**. After E-STOP +
Stop Override the boat is also out of override: press **Publish Override
Control Mode** to take the thrusters back. (In a session launched with motors
disabled the button is not offered — `enable` would enable them.) From a
terminal:

```bash
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: enable"
```

## Post-mission report

Every mission produces one picture, automatically — written by the control
stack, not by the station. When the launch is torn down, `robot_interface` closes its
position CSV and files the whole run into its own folder. In a real-robot run
session this happens **at every STOP RUN**, one folder per run (named after the
run's start time and its log note), in the background while the boat stays
connected:

```
<workspace>/data/Robot_data/2026_09_04-14_18_19-poslog/
    …-poslog.csv    the log, moved here     …-origin.yaml   its world-frame origin
    …-poslog.png    GPS track, distance-to-target, speed, thrust, summary table
    …-bag/          the run's ros2bag (run sessions, v1.1): every topic but the sonar's
    …-path.yaml     the mission the run followed (path runs, v1.1)
```

The bag is what *From a ros2bag* replays (Launch Mission ▸ Gazebo simulation),
and any ROS tool reads it (`ros2 bag info …-bag`). It is recorded by its own
process alongside the session, so it cannot slow the station or the control
loop, and it adds no traffic on the boat's radio link.

A filed run is never re-filed or overwritten, and the CSV is only ever moved —
it is primary field data. To re-render an older log:

```bash
ros2 run blueboat_control poslog_report.py <file.csv>
ros2 run blueboat_control poslog_report.py <dir> --all
```

The renderer and the plot contents belong to `BlueBoat-Control`; see that
module's docs for the detail.

## Closing

Closing the window with a mission running asks for confirmation, then runs
the same safe-shutdown guarantee: `default` is published and its transmission
confirmed *before* the nodes are stopped; the window closes once the launch
tree has exited. Application logs are mirrored to the terminal that started
the station throughout (the terminal is the complete debug output; add
`--verbose` to also see the raw launch stream there).


## Survey Pattern Designer

**Create Survey Pattern** (bottom toolbar) opens the designer — a small
path-design application inside the station. Its map pans (middle-button
drag), zooms (wheel), shows the metric grid — with a scale bar in the bottom-left corner whose
length always equals one grid cell — and, when a georeference exists, the
same satellite layer as the main map. **Center Pattern** (`F`) frames the
current selection, or the whole mission, on screen; **Zoom + / Zoom −**
toolbar buttons (`+` / `−`) zoom about the view centre. **Align to Start**
rigid-transforms the whole mission so it begins at world (0,0) with its
first tangent along +x: the world frame is local ENU — origin = the boat's
launch position, +x = EAST — so an aligned mission starts at the launch
point heading east (mainly useful in simulation; real missions should be
GPS-anchored). Saving a non-GPS mission that doesn't already start that way
offers this alignment automatically; GPS-anchored missions are
geographically fixed and are never realigned (the boat turns toward them
instead).

The designer is **independent of the live station**: it shows no robot or
pinger, and it never picks up the GPS lock on its own. Its origin is always
something you chose, and the status bar names the one in force at all times
(`Anchor: 33.660196, 130.657780 (mission)`, or `Anchor: none — design frame =
the boat's launch point`). This is deliberate — the designer used to adopt the
station's live georeference whenever the boat had a fix, which silently moved
an opened path off the imagery it was drawn on and rewrote its anchor on the
next save.

**Set GPS Origin…** offers three ways to define world (0,0): **Enter
Coordinates…** accepts Google-Maps-format coordinates (`33.660196,
130.657780`); **From a Gazebo World…** browses `~/worlds/` for a world's
`metadata.yaml` — the world's GPS anchor becomes the design origin (the
design frame then coincides with the world's local frame) and its limit
rectangle is drawn on the map as a dashed read-only reference, so the
path can be laid out inside it; and **From the Robot's Current Position…**
takes a one-off snapshot of the boat's latest GPS fix, so design (0,0) is
where the boat is right now. That snapshot is taken once, when you click it —
the design frame does not follow the boat afterwards. Saving such a mission also copies the
world folder to `~/worlds/<mission name>/<world name>/` with its
provenance updated to the new path, so the launch dialog finds it grouped
with this mission; an already-existing copy is never overwritten (the
status bar says which happened). The origin remains optional, and without
one the editor simply works in the local world frame. A mission saved with a GPS
reference remembers it: reopening the mission restores the origin (and the
satellite layer), every waypoint is linked to real-world GPS coordinates,
and the launch dialog labels it “(GPS)”. The origin belongs to the mission,
not to the session — **New** and **Open** reset it to whatever the file
declares, so an un-anchored path can never pick up the previous one's origin. Launching such a mission uses
deferred deployment: the boat holds position for the first seconds while
the run's georeference anchors from the incoming GPS fixes — no driving
needed — then the path deploys automatically onto its true GPS
coordinates, independent of where the robot was powered on. The same flow
runs in **simulation** (the station simulates the GPS feed and spawns the
boat at a fixed heading), so a planned path can be rehearsed before the
field trial, repeatably. At the end of any custom path the target clamps at the final
pose forever, so the boat station-keeps there; the same holds on a reached
manual target.

**Editing.** *✚ Add Waypoints* (or `A`) arms click-to-add: Shift constrains
the new point horizontally/vertically from the previous waypoint, Ctrl
creates at a fixed distance (multiples of the grid step); right-click
leaves the mode. In select mode, drag handles to move (multi-selection via
rubber band or Ctrl-click): Shift = axis constraint, Ctrl = snap to grid,
snapping to nearby waypoints is on by default (Alt disables). Copy/Paste
(`Ctrl+C/V`), Duplicate+Offset (`Ctrl+D`), Delete, Align ─ / Align │,
Distribute (equal spacing between the first and last selected), Group /
Explode, Lock/Unlock, rename (edit in the tree), reorder (▲/▼) and
snapshot Undo/Redo (`Ctrl+Z/Y`) are all in the toolbar and tree panel.

**Segments.** Select a waypoint to edit the segment *leaving* it: straight,
sinusoidal, circular arc, Catmull-Rom spline or cubic Bézier, each with its
own parameters, plus a **per-segment speed** (leave at “mission speed” to
use the mission-wide setting) — the status-bar duration updates
accordingly; the waypoint's live GPS coordinates are shown whenever a
georeference exists; the preview updates live with interpolation curves,
travel-direction chevrons, waypoint numbering and START/END markers.

**Patterns.** The library inserts parameterized survey patterns as grouped
objects in the mission tree — Lawnmower (area, spacing, orientation,
starting corner), Circle (radius, points, direction), Rectangle, Square,
Figure Eight, Spiral, Expanding Square, Station Keeping, Regular Polygon.
Groups move, duplicate, reorder and lock as one object; "Edit pattern
parameters…" regenerates in place, "Explode" turns a group into individual
waypoints.

**Mission settings & files.** Cruise speed (time-parameterization), loop
and a comment live in the right column; the status bar shows waypoint
count, length and duration. Save / Save As (with overwrite confirmation),
and the library dialog's Duplicate / Rename / Delete manage the mission
files (`~/.config/blueboat_mcs/trajectories/`). Saved missions immediately
appear in **Launch Mission → Trajectory → custom paths** for both the real
robot and the simulation.
