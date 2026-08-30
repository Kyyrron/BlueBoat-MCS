# 2 — Developer Documentation

Audience: a researcher/developer extending or maintaining the station.
Read `01_architecture.md` first for the big picture.

## Code layout and conventions

The package is `mcs/`. Conventions used throughout:

* Python 3.10+ syntax, `from __future__ import annotations`, full type hints.
* Module docstrings state *why* the module exists, not just what it contains.
* `core/` and `models/` never import widgets and never import rclpy — they can
  be unit-tested headless (see `smoke_test.py` for the pattern:
  `QT_QPA_PLATFORM=offscreen`).
* GUI code never blocks: no service waits, no subprocess waits, no sleeps.
  Anything that could take time runs in the ROS thread, the launch-output
  reader thread, or is polled by `QTimer`.
* All colors come from `gui/theme.py`; all tunables from `config/settings.py`.

## Data flow, end to end

Taking odometry as the canonical example:

1. `BridgeNode._on_odom` (ROS thread) converts the message to plain Python
   lists, stamps `time.monotonic()`, updates its `TopicStats`, and emits
   `SignalBus.odom_received`.
2. Qt queues the signal into the GUI thread; `DataStore.on_odom` updates the
   live `RobotState`, appends to `robot_track` / `speed_hist`, integrates
   travelled distance, feeds the `GeoReferencer`, refreshes the pinger world
   position, and records the active-target distance.
3. Nothing repaints yet. At the next 10 Hz tick, `MainWindow._on_tick` calls
   `left_panel.refresh()`, `right_panel.refresh()`, `map_view.refresh()`,
   which *pull* from the store and repaint once.

User intents flow the opposite way: widget → signal → `MainWindow` slot →
`CommandCenter` → `BridgeNode.publish_*` (thread-safe) → ROS graph.

## How to…

**Add a subscribed topic.** Add its name to `TopicsConfig`; add thresholds to
`DiagnosticsConfig` (it then appears automatically in the diagnostics panel);
add a `Signal` to `SignalBus`; create the subscription + `_on_x` callback in
`BridgeNode` (call `self._mark(topic)` first for statistics); add a
`DataStore.on_x` slot and connect it in `MainWindow._connect_signals`. Display
it from any panel's `refresh()`.

**Add a published command.** Add a publisher in `BridgeNode.__init__` and a
`publish_x` wrapper (keep the `self._pub_lock` pattern and the
`command_sent` emission), expose a semantic method on `CommandCenter`, and
call it from the GUI.

**Add a map layer.** Create an item in `gui/map/map_items.py` (use
`_cosmetic_pen` for constant-pixel lines, `ItemIgnoresTransformations` for
constant-pixel glyphs), add it to the scene in `MapView.__init__`, update it
in `MapView.refresh()`, register a key in `MapView.set_layer_visible`, and add
one entry to `_LAYERS` in `left_panel.py`. That is the whole checklist.

**Add a plot.** Follow `DistancePlot`: a `QWidget` that reads one
`TimeSeries`, honors `set_time_window`, and paints in `paintEvent`. Add it to
a `CollapsibleSection` in the right panel and call its `refresh()` from
`RightPanel.refresh()`.

**Change launch arguments.** `LaunchParameters.to_cli()` and the dialog in
`gui/dialogs/launch_dialog.py` are the only two places.

## Timing and clocks

All history timestamps are `time.monotonic()` at *reception*, one clock for
everything. Experiment-relative time (what the timeline shows) is
`t − store.t0` where `t0` is the first odometry sample. The controller's own
`t` field inside `/monitoring_data` is not used as a clock — reception time
is, which keeps every series mutually consistent even if a node restarts.

## Performance notes

* `TimeSeries.decimated_window` bounds every repaint: at most
  `map.trajectory_max_points_drawn` vertices per polyline and 2000 per plot.
* `PolylineItem.set_points` rebuilds a `QPainterPath` per tick from the
  decimated window — measured adequate at 10 Hz well past 10⁵ stored points.
  If a future sensor pushes this, switch to incremental `lineTo` appends when
  the window is live and unchanged.
* The tile layer refuses to populate more than 64 tiles per viewport (zoomed
  far out) instead of hammering the tile server.

## Testing

`smoke_test.py` at the repo root is a **linear script, not a test
framework**: it aborts on the first failed assertion, so anything after a
failure never runs. It prints fourteen checkpoints and ends with
`SMOKE TEST PASSED`. Run it with:

```bash
QT_QPA_PLATFORM=offscreen python3 smoke_test.py
```

It runs identically with or without `rclpy` importable. Four of the §3
non-negotiables in `CLAUDE.md` are asserted there (pinger anchoring, the
`[0,0]` sentinel, the deploy guard, and safe-shutdown *ordering*); `CLAUDE.md`
§7 has the checkpoint list and the current coverage gaps. Extend it whenever
you add store, core or designer logic.

A green run is not field readiness: it is offscreen and synthetic, so it
cannot catch QoS mismatches, real message-type drift or launch-file argument
errors.

### Lint

`ruff` is the second gate, configured in `ruff.toml` at the repo root. It needs
neither ROS nor the venv — it resolves no imports — so it runs from any shell:

```bash
pip install -r requirements-dev.txt   # first time only
ruff check .
```

`requirements-dev.txt` is separate from `requirements.txt` on purpose: the
basestation installs runtime dependencies only. The version is pinned exactly,
because the gate is ruff's default rule selection rather than an explicit
`select` list, so an unpinned upgrade would change what it enforces.

The broad `except Exception` clauses in the ROS-facing and logging paths are
deliberate and carry inline `# noqa: BLE001` with a reason — a ROS callback must
never take down the GUI thread. Keep that pattern; do not narrow them to satisfy
the linter. `CLAUDE.md` §7 has the two configured rule suppressions and the
reason no type-checker is configured.

## Robot-side behaviours the station does not compensate for

The station supervises; it does not paper over the stack. When something looks
wrong on screen, the first question is whether the producing node is right.
`03_ros_integration.md` §"Observations" is the running list, verified against
`BlueBoat-Control` — some entries there are now fixed and recorded as such, and
the ones that remain are flagged, not worked around. `CLAUDE.md` §6 owns the
two limitations that are carried deliberately.


## Survey Pattern Designer (mcs/designer/)

Layering mirrors the station: `model.py` (mission data + mutations +
snapshot undo), `interpolation.py` and `patterns.py` (pure registries),
`sampling.py` (mission → time-stamped samples), `io_yaml.py` (runtime +
metadata files, library ops) are Qt-widget-free and covered by the smoke
test; `designer_map.py`, `panels.py`, `designer_window.py` are
presentation. Robot-side, the only runtime dependency is
`BlueBoat-Control/blueboat_control/src/_custom_libraries/yaml_trajectory.py`.

How to extend:

* **New interpolation** — subclass `Interpolation` in `interpolation.py`
  (`key`, `label`, `schema`, `sample()` returning points from A inclusive
  to B exclusive) and add it to `REGISTRY`. The properties panel form, the
  sampler, the YAML metadata and the runtime need no changes (the runtime
  only ever sees samples).
* **New pattern** — subclass `Pattern` in `patterns.py` (`generate(params)`
  with `x0`/`y0` anchor injected by the dialog) and register it; the
  library button and parameter dialog are generated from `schema`.
* **New file format revision** — bump `FORMAT` in `io_yaml.py` and
  `SUPPORTED_FORMAT` in the runtime loader together; readers reject unknown
  tags by contract (see `08_trajectory_format.md`).

Undo is snapshot-based: any new mutating entry point must call
`DesignerWindow._push_undo()` first — nothing else is required.
