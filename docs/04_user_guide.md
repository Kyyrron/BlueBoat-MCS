# 4 — User Guide

The station is the single operator interface during an experiment. It stays
open the whole time; missions are launched, stopped and relaunched from it.

## Screen layout

Left: live information and diagnostics. Center: the interactive mission map.
Right: monitoring (distance plot, timeline, statistics). Bottom: the mission
control toolbar and its one-line launch console. Status bar: last click
inspection, georeference quality, ROS connection state.

## Left panel

**Layers** — checkboxes toggling every map layer: satellite imagery, robot
trajectory, published mission path, pinger position and trajectory, the thin
robot→target line, the heading arrow and the metric grid. The satellite box
unlocks as soon as there is a **GPS fix** — the georeference places the tiles
from its first, translation-only fit; watch "georef" in the status bar. (North
alignment is a separate, later stage — see the map section below.)

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
path start, and the boat spawns with a random heading). In a simulation
without a GPS-anchored path the map draws immediately (no tiles — there is
no GPS). Only the
vehicle icon rotates, exactly like QGroundControl. A briefly empty map right
after a real-water launch is expected, not a fault; manual-target clicks
during that window are refused rather than sent somewhere wrong.
**Clear Paths** in MAP TOOLS wipes the robot
and pinger trails without touching live data. A simple click
anywhere shows, in the status bar: world coordinates, GPS coordinates (once
georeferenced) and the live distance from the robot to that point; the point
is marked on the map.

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
target highlight is cleared. When the boat arrives (≤ 1 m), a "Manual
Target Reached" banner appears at the top of the map.

**Measure** (toolbar button): first click sets point A, the line and distance
follow the cursor, second click freezes the measurement; the status bar shows
distance and both coordinates. Deactivating the tool clears any frozen
measurement from the map.

## Right panel

**Map tools** — **Zoom +**, **Zoom −** (view-center anchored) and **Center
Robot**, which recenters the view on the boat exactly once; the camera then
remains completely free — it is never a follow mode, and after a manual
center no automatic recentering can move the camera again.

**Live distance** — robot↔current-target distance versus experiment time; the
title states whether the target is the pinger, the path target or a manual
target. The current value is highlighted at the live edge.

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
(`Sim_launch.py`). Real robot: controller type (empty / PID / LoS / MPC),
trajectory, use-pinger, motor enable (always re-confirmed, with a second
warning dialog), a log note and free-form extra launch arguments. Gazebo
simulation: robot file, trajectory and controller — the simulation
always runs a controller, and motor/pinger/note fields are hidden because
they do not exist in that graph; mission-state readiness shows "(sim)" and
does not wait for a flight controller. Selecting a GPS-anchored custom
path in simulation additionally arms the simulated GPS feed and passes a
random `spawn_yaw` (shown in the status bar and the console command line). On OK the station runs the ROS2 launch file; the LED turns
orange while nodes come up and green once the required ones report
(FCU connected + odometry flowing). Launch output streams to the console line.

**Stop Mission** first publishes `default` on `/blueboat/input_str` and
waits for confirmed transmission (the same guarantee as the Emergency Stop),
then shuts every launched node down gracefully (SIGINT first) and releases
the process; the station remains open and the mission can be relaunched.

**E-STOP + Stop Override** and **E-STOP** are two direct, one-click
emergency buttons (no confirmation dialog). Both first publish `default` on
`/blueboat/input_str` and wait until the command is confirmed transmitted
(the `param_mode` echo, with a graph-verified reliable-delivery fallback).
**E-STOP** stops there — nodes keep running with safe parameters restored.
**E-STOP + Stop Override** additionally terminates every launched node once
transmission is confirmed, stopping whatever is driving the motors. The
label next to the buttons reports each phase, and the Default/Override
toggle is resynchronized automatically (after either E-STOP the mode is
`default`, so its next command is `override`).

**Publish Default Control Mode** publishes the same `default` command
immediately; the button then alternates to **Publish Override Control Mode**
(publishing `override`) and back on each click.

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
instead). If the robot is
connected, its position sets the initial view and the live robot arrow and
pinger overlays can be toggled from the toolbar; otherwise **Set GPS
Origin…** accepts Google-Maps-format coordinates (`33.660196, 130.657780`)
to define world (0,0) — the origin remains optional, and without one the
editor simply works in the local world frame. A mission saved with a GPS
reference remembers it: reopening the mission restores the origin (and the
satellite layer), every waypoint is linked to real-world GPS coordinates,
and the launch dialog labels it “(GPS)”. Launching such a mission uses
deferred deployment: the boat holds position for the first seconds while
the run's georeference anchors from the incoming GPS fixes — no driving
needed — then the path deploys automatically onto its true GPS
coordinates, independent of where the robot was powered on. The same flow
runs in **simulation** (the station simulates the GPS feed and spawns the
boat with a random heading), so a planned path can be rehearsed before the
field trial. `07_getting_started.md` has this as a field checklist. At the end of any custom
path the target clamps at the final pose forever, so the boat
station-keeps there; the same holds on a reached manual target.

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
