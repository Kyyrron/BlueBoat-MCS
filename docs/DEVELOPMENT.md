# BlueBoat-MCS — development guide

How to work on the Mission Control Station: the AI coding assistant setup, the tests, the rules
and what to improve next. For using the app, see the [README](../README.md) and the
[user guide](04_user_guide.md).

## 1. Install Claude Code

The project is developed with [Claude Code](https://docs.claude.com/en/docs/claude-code/overview),
an AI coding assistant that runs in the terminal.

```bash
curl -fsSL https://claude.ai/install.sh | bash        # native installer
# or, with Node.js 18 or newer:
npm install -g @anthropic-ai/claude-code
```

Then start it and log in with your Anthropic account:

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar               # the whole project: cross-module rules included
claude
# or only this app:
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-MCS && claude
```

Starting from the superproject is better: Claude then also reads the project-wide rules, the
interface contract between modules and the shared skills (test commands, file formats).

## 2. What Claude reads in this module

| File | Role |
|---|---|
| `.claude/CLAUDE.md` | Loaded at start: what the station is and its non-negotiable rules (safe shutdown, E-STOP, frames, GPS deployment) |
| `.claude/layout.md` | The code layout, threading, and "how to add" a topic, command, map layer, plot or launch argument |
| `.claude/interface.md` | Every subscription, publication and launch mode |
| `.claude/map-designer-control.md` | The map, the Survey Pattern Designer and the control-stack files the station depends on |
| `.claude/commands-docs.md` | Commands, tests and the documentation set |
| `.claude/rules/*.md` | Load the right topic file automatically when Claude reads or edits a matching file |
| `mcs/designer/CLAUDE.md` | Notes for the Survey Pattern Designer |

These files are written for Claude, but they are plain text and worth reading yourself before a
change. The "how to add" list in `.claude/layout.md` is the fastest way into the code.

## 3. What to set up after a fresh clone

- **The workspace**: BlueBoat-Control built in `~/ros2_ws` and `~/ros2_ws/env.sh` (README).
- **The lint tool**, which is not tracked by git (`ruff.toml` and `requirements-dev.txt` are in
  `.gitignore`):
  ```bash
  pip install ruff==0.16.5
  ```
  and a `ruff.toml` with `target-version = "py310"`, `line-length = 88` and
  `extend-exclude = ["build", "install", "log"]`. Ruff's default rule set is the gate.
- **`.claude/settings.json`** is ignored by git and not needed: the station has no Claude hook.
  A personal `.claude/settings.local.json` may allow commands such as `ruff check` without asking.
- **The simulator's sea-state file and the link guardian**: two blocks of the smoke test need the
  installed BlueBoat-SSS-Sim (`sea_states.yaml`) and the superproject's
  `link_guardian/laptop/guardian.py` beside this repository. Without them the smoke test stops at
  that block; this is a missing environment, not a bug.

## 4. Tests

From `BlueBoat-MCS/`, with `source ~/ros2_ws/env.sh` done:

```bash
QT_QPA_PLATFORM=offscreen python3 smoke_test.py      # must end with SMOKE TEST PASSED
ruff check .                                         # lint
```

The smoke test is one linear script. It builds the window offscreen, feeds it synthetic
telemetry, prints one `... ok` line per block, and stops at the first failed check. A green run
does not prove field readiness: it cannot catch delivery-setting (QoS) mismatches, message type
changes or launch argument errors. Most of the designer's user interface is not covered. There is
no type checker, on purpose.

## 5. Rules for changes

- **This app is frozen at version 1.0 and changes by additions only**, on the owner's explicit
  request. Refactors must keep `smoke_test.py` green and the behaviour identical.
- **Safe shutdown before terminating a launch**: publish `default`, wait for the boat's
  confirmation, then stop the nodes. Never reverse this order.
- **E-STOP, servo release and ending the mission are three separate operator actions.**
- **`[0, 0]` on `/blueboat/manual_target` means "resume the mission"**, never a position.
- **The map never rotates**; only the boat glyph does. Never re-apply a frame correction to
  `/monitoring_data`: it is already in the world frame.
- **No control computation in `mcs/`.** Control-stack changes belong in BlueBoat-Control.
- **Never touch Qt widgets from a ROS callback.** Data crosses from the ROS thread to the GUI
  through the signal bus only.
- Topic names live in `mcs/config/settings.py`, never inline. Never rename a topic.
- AI sessions never commit, push or tag: the owner does.

## 6. Next steps

There is no open bug list for this app; what remains is mostly verification on water. Useful
improvements:

1. **A "Replanner mode"** in the launch dialog: start a run whose path comes from
   BlueBoat-Replanner (`path_source:=replanner`) with the station's safety buttons. Today this needs
   the **Extra args** field or the command line. The request is in BlueBoat-Control's
   `docs/REQUESTS.md`.
2. **Show an updated path on the map.** When the Replanner changes the path, the map still shows
   the path of the run's start.
3. **Test more of the designer automatically**: the designer map, its panels and undo are not
   covered by the smoke test.
4. **Make "Ray-cast — harbour, fast" the default sonar model** for simulated results, once the owner
   agrees, so it no longer has to be picked by hand.
5. **Field-check the run session and the link guardian** on the boat, with the station's own
   buttons, and record the result in `docs/04_user_guide.md`.
