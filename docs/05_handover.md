# 5 — Handover Document

For the next researcher inheriting the station. `01_architecture.md` explains
the structure; this document tells you where the seams are and what to build
next.

## Where the seams are

The design deliberately concentrates change points:

* **`config/settings.py`** — a renamed topic, a new threshold, a different
  tile provider or LoS gain never requires touching logic. Field overrides
  live in `~/.config/blueboat_mcs/config.json` and survive updates.
* **`SignalBus`** — any new data stream is one new signal; producers and
  consumers stay decoupled.
* **`DataStore`** — the single place where derived state may be computed. If
  you are tempted to compute something in a widget, put it here instead.
* **`BridgeNode`** — the only file that knows message types.
* **`MapView.set_layer_visible` + `_LAYERS` in `left_panel.py`** — the whole
  cost of a new map layer.

The per-feature checklists ("How to…") are in `02_developer_guide.md`.

## Natural next extensions

**Side-scan sonar overlay.** The SSS pipeline's mosaics could be drawn as a
georeferenced pixmap layer: follow `TileLayer` (a `QGraphicsPixmapItem` with a
`QTransform` from world metres) but source the image from a topic or file
instead of the network. z-order between −100 (tiles) and 10 (mission path).

**Detection markers for the adaptive-survey thesis work.** When the
detect→prioritize→revisit loop publishes contacts, subscribe to them in
`BridgeNode`, store them in a `TimeSeries` (x, y, class, score), and render
`MarkerItem`s with class-coloured labels. The belief grid can reuse the pixmap
approach above. The timeline already gives you replay of the replanner's
behaviour for free.

**Session persistence / export.** `TimeSeries` exposes `t()`/`v()` numpy
views; dumping the store to an `.npz` (plus the launch parameters as JSON) is
~30 lines and would let the station reload and scrub past experiments through
the existing timeline. Recommended location: `models/session_io.py`, wired to
File-menu actions.

**Multiple plots.** `DistancePlot` is generic apart from its data source —
parametrise it with a `TimeSeries` getter and add speed / thrust plots as new
`CollapsibleSection`s. `thrust_hist` and `speed_hist` are already recorded.

**Robot-side improvements worth adopting** (details and one-line patches in
`03_ros_integration.md` §Observations): fix the manual-target resume
comparison (required for "Continue Original Mission" to behave), publish the
world-frame pinger, fix the target-on-thruster-topic slip in pinger mode. If
the world-frame pinger is published, point `TopicsConfig` at it and delete
`DataStore._update_pinger_world`.

**Waypoint mission editing.** The map's click plumbing (modes, markers,
publishers) generalises directly to click-to-build waypoint lists; the missing
piece is a robot-side consumer, since `path_generation` currently serves
analytic trajectories only.

**Mission Pattern Designer — future work.** Cubic Bézier segments are
implemented; the following were assessed and deliberately deferred as they
change the product's scope rather than extend a registry: **Dubins paths**
(turn-radius-constrained segments — natural fit as one more
`Interpolation`, but honest support requires the vehicle's real minimum
turn radius, which should first be identified from field logs);
**polygon-area coverage planners** (the lawnmower already covers
rectangular areas; general polygons need cell decomposition — recommended
as a new `Pattern` backed by a `coverage.py` module); **automatic survey
generation** (detector-driven replanning belongs to the thesis's adaptive
pipeline, not the manual designer — the YAML format is deliberately
sufficient as its output target: an adaptive planner can write
`blueboat_trajectory/1` files and they run unchanged).

## Things to be careful with

* Keep the ROS/GUI boundary intact: never call a widget from the ROS thread,
  never spin/wait in the GUI thread. Everything crossing goes through
  `SignalBus` or the thread-safe publisher wrappers.
* Keep repaint work bounded: any new drawn history must go through
  `decimated_window`.
* `(0,0)` on `/blueboat/manual_target` is a protocol sentinel — never publish
  it as a real target (the map already nudges origin clicks).
* The E-STOP ordering (publish → confirm → terminate) is normative; do not
  "simplify" it.
* The LoS prediction is a sketch. If you tune `LosApproximation`, tune it
  against recorded real runs, and keep the line dashed — operators must not
  mistake it for a plan.

## Maintenance

Dependencies are PySide6, numpy, scipy (pip) and the ROS2 workspace
(rclpy, mavros_msgs, blueboat_interfaces). `smoke_test.py` runs without ROS or
a display and should stay green on every change; extend it with any new store
or core logic. The tile cache (`~/.config/blueboat_mcs/tile_cache`) can be
deleted at any time.
