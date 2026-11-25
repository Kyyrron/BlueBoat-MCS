# BlueBoat-MCS — layout, internals, how to add

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## 1. Layout & internals

`mcs/main.py` bootstraps (`run.py` is a thin launcher).

| Package | Role |
|---|---|
| `config/settings.py` | **All** topics, thresholds, gains and paths (dataclasses; JSON overrides from `~/.config/blueboat_mcs/config.json`). |
| `core/`, `models/` | No widgets, no rclpy (only `QObject` signals) — what makes the offline smoke test possible. `core/signals.py` (SignalBus), `geo.py` (GeoReferencer), `series.py` (TimeSeries), `clock.py` (the one elapsed-time format), `arrows.py` (arrow-key thrust law), `guardian.py` (GUARDIAN box rules), `replay.py` (bag folders, `/replay/state`), `sea.py` (sea-state model), `worlds.py` (SSS-Sim world-folder contract, reimplemented locally — never imported, CM-3), `sim_gps.py`, `los_predictor.py`, `pinger_filter.py` ((2026-11-24) BlueBoat-Control's pinger filter schema `config/pinger_filter.yaml` read as DATA — never an import, N9 —, its validation rules, the parameter-file format, the pinger modes / sources / fix decisions). `models/store.py` is the DataStore (since 2026-11-24 also `PingerEstimate` = the filtered pinger verbatim, `PingerFilterState` = sent vs applied filter, legacy `PingerState`). |
| `ros/` | `ros_manager` (the rclpy thread), `bridge_node` (**the only file that knows message types**), `launch_manager`, `command_center` (commands, E-STOP, safe shutdown), `run_session` (the station's half of a real-robot run session). |
| `gui/` | `main_window` (composition root), panels, `map/` (incl. `legend.py`, the pinger legend, 2026-11-24), `plot/`, `dialogs/` (incl. `pinger_filter_dialog.py`: `PingerForm`, the pinger rows shared by the Launch and Update Mission dialogs, and `PingerFilterDialog`, the schema-generated Advanced panel: a "Field tuning" tab of `TuningCard`s from the schema's `field_tuning` block, pictures beside the schema, and the "All parameters (expert)" tab), and the boxes: `run_panel` (RUN), `guardian_box` (GUARDIAN), `arrow_box`, `replay_box`, `sea_state_box`, `mission_stats`. Shared widgets/helpers in `widgets.py` (incl. `elide_middle`) and `theme.py` (incl. `floating_box_style`). |
| `designer/` | Survey Pattern Designer — its own `CLAUDE.md`. |

`ruff.toml` and `requirements-dev.txt` are gitignored. Control-stack code lives in the
**`BlueBoat-Control` submodule**, never here (§6, N9).

**Threading.** `rclpy` spins on its own thread. `SignalBus` is the **only** crossing point:
ROS callbacks emit Qt queued signals, `DataStore` slots mutate state on the GUI thread, and
widgets repaint from a single ~10 Hz tick in `MainWindow` (`MapConfig.ui_refresh_hz`). Never
touch Qt objects from a ROS callback; never block the ROS thread on GUI work. `DataStore` is
the single source of truth for the GUI: widgets read it, never subscribe to ROS.

**Degraded mode.** Without `rclpy` the app still starts GUI-only. Keep new ROS imports lazy or
guarded.

**Data flow (odometry, the canonical example).** `BridgeNode._on_odom` (ROS thread) converts
to plain lists, stamps the station clock, marks its `TopicStats` and emits
`SignalBus.odom_received` → `DataStore.on_odom` (GUI thread) updates `RobotState`, histories,
travelled distance → at the next tick `MainWindow._on_tick` calls each panel's / the map's
`refresh()`, which *pull* from the store. User intents flow back: widget → signal →
`MainWindow` slot → `CommandCenter` → `BridgeNode.publish_*` (thread-safe) → ROS.

**Clocks.** Every history timestamp is the **station clock** at **reception**
(`core/clock.station_clock()`, read through `store.now_mono()`): `time.monotonic()` in a
real-robot or replay launch and before any launch; in a **simulation launch** it follows
`/clock` (the bridge feeds it; `MainWindow._on_mission_launched` → `set_follow`), advancing in
sim seconds, continuous, interpolated, never backwards, wall-paced after
`sim_speed.clock_stall_s` (3 s) of `/clock` silence. So a run slowed below real time (the
ray-cast sonar) shows rates, ages, elapsed time, plots and statistics in sim time. Every
comparison against a reception stamp uses it; self-contained operator deadlines (N1
shutdown, guardian waits, the path-preview timeout, the arrow `QTimer`) stay monotonic.
Experiment time is `t − store.t0` (first odometry sample). The controller's own `t` in
`/monitoring_data` is never used as a clock.

**Repaint bounds.** `TimeSeries.decimated_window` caps every repaint at
`map.trajectory_max_points_drawn` (20 000) vertices per polyline and 2000 per plot; the tile
layer refuses more than 64 tiles per viewport.

**Map rendering.** Constant-pixel glyphs (`ItemIgnoresTransformations`), cosmetic pens, an
adaptive 1/2/5 metric grid in `drawBackground`, satellite `TileLayer` at z = −100. Modes
`NORMAL` / `MANUAL_TARGET` / `MEASURE` are a state machine inside `MapView`, which emits
intents (`target_clicked`, `point_inspected`) and never publishes.

### How to add…

- **A subscribed topic:** name it in `TopicsConfig`; thresholds in `DiagnosticsConfig` (it
  then appears in the diagnostics panel); a `Signal` on `SignalBus`; subscription + `_on_x`
  in `BridgeNode` (call `self._mark(topic)` first); a `DataStore.on_x` slot connected in
  `MainWindow._connect_signals`.
- **A published command:** a publisher in `BridgeNode.__init__` and a `publish_x` wrapper
  (`_publish_text` for String topics; keep the `_pub_lock` pattern and the `command_sent`
  emission), a semantic method on `CommandCenter`, called from the GUI.
- **A map layer:** an item in `gui/map/map_items.py`, added in `MapView.__init__`, updated in
  `MapView.refresh()` (and hidden in its pre-anchor branch), a key in
  `MapView.set_layer_visible`, one `_LAYERS` entry in `right_panel.py`. A pinger layer also
  gets a line in `gui/map/legend.py` (`_LINES`) and a key in `MapView.pinger_layers()`.
- **A plot:** follow `DistancePlot` (a `QWidget` over one `TimeSeries`, honouring
  `set_time_window`), in a `CollapsibleSection` of the right panel, refreshed from
  `RightPanel.refresh()`.
- **A launch argument:** all in `mcs/ros/launch_manager.py` plus the dialog —
  `LaunchParameters.to_cli()` (the replay branch first; splices `sea_args()`),
  `launch_target()`, the companions (`companion_command` = empty-world sea state,
  `guardian_command` = real-robot link guardian, `perception_command`; held in
  `LaunchManager._companions`, signalled together by `stop()` / `_finalise()`), and
  `gui/dialogs/launch_dialog.py`. A per-run dialog setting that is not a launch argument
  (like `perception_mode`, or the `pinger_filter` block that rides the session's
  `configure`) is its own row and its own field, ignored by `to_cli()`. A launch-time file
  (the custom pinger filter's `last_launch.yaml`) is written by `LaunchManager.start`
  (`write_pinger_filter_file`), never by the dialog's `parameters()`, which stays free of
  side effects for the smoke test.

---
