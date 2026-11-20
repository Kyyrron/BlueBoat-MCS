# BlueBoat-MCS — commands, gates and documentation set

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## 7. Commands and gates

```bash
cd ~/ros2_ws && source env.sh                        # venv + ROS 2 + overlay
cd src/BlueBoat-SideScanSonar/BlueBoat-MCS
pip install -r requirements.txt                      # first time; not --user in the venv
python3 run.py [--config /path/config.json] [--verbose]   # build.sh: colcon build, then this
```

```bash
QT_QPA_PLATFORM=offscreen python3 smoke_test.py      # regression gate
ruff check .                                         # lint gate (pinned ruff==0.16.5, ruff.toml)
```

```bash
# CLI launches (the station's real-robot launch is a session; the CLI may omit session_mode)
ros2 launch blueboat_control BlueBoat_launch.py session_mode:=True enable_motors:=True \
    controller_type:=PID use_pinger:=False \
    trajectory:=from_yaml:$HOME/.config/blueboat_mcs/trajectories/.session/run.yaml
ros2 launch blueboat_control Sim_launch.py robot_file:=thrusters_ur controller_type:=MPC \
    trajectory:=circle note:=sim-tuning
ros2 launch blueboat_sss_sim full_mission_launch.py world_dir:=$HOME/worlds/<path>/<world> \
    with_control:=true controller_type:=PID note:=sim-dam \
    trajectory_file:=$HOME/.config/blueboat_mcs/trajectories/.deployed/<name>.yaml
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: default"
```

**`smoke_test.py`** is a linear script, not a framework: it imports every module, builds the
window offscreen, drives synthetic telemetry through the `SignalBus`, prints one `… ok`
checkpoint per block (block names say what each pins — N1/N1b `safe shutdown ok`, N2
`sentinel ok`, N2b `placement ok`, N2c `arrow ok`, N4 `pinger anchor ok`, N8/N8b
`deploy guard ok` / `launch dialog sim-gps ok`, the run session, replay, guardian, perception,
`helpers ok` for the shared helpers) and aborts on the first failed assertion; success ends
with `SMOKE TEST PASSED`. **Environment it needs** (else it aborts at the first such block,
not a regression): the `sea state ok` timeline asserts need the simulator's installed
`sea_states.yaml`; the bridge-node path-service block imports `bridge_node`, hence
`nav_msgs` / `sensor_msgs` / `std_msgs` (a sourced ROS 2); `guardian ok` needs the
superproject's `link_guardian/laptop/guardian.py` beside this repository. Uncovered: most of
the designer UI (`designer_map.py`, `panels.py`, `designer_window.py`, the `_push_undo`
contract). A green run is not field readiness — it cannot catch QoS mismatches, message-type
drift or launch-argument errors.

**Lint.** Default rule selection, suppressions with reasons in `ruff.toml`; the broad
`except Exception` clauses carry `# noqa: BLE001` with a reason and stay broad. **No
type-checker, deliberately** (a mypy survey found 45 errors and no real defect; re-open only
with new evidence).

---

## 8. Documentation set

`docs/` holds the long-form explanation; this file stays authoritative where they disagree.
There is no `TODO.md`: what remains is field verification, which this repository cannot settle.

| Doc | Content |
|---|---|
| `03_ros_integration.md` | Every topic / command / service, launch targets, stop actions, the safe-shutdown sequence; §Observations = open control-stack items and the fixes a stale install reintroduces. |
| `04_user_guide.md` | Every screen and control, incl. run session, arrow control and the designer. |
| `05_trajectory_format.md` | The `blueboat_trajectory/1` contract, deferred GPS deployment, the session file. |
| `GPS_MAP_ARCHITECTURE.md` | The self-contained GPS-only map model (§4). |
