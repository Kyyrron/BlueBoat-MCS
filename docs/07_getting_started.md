# 7 — Getting Started (new researcher, ~5 minutes)

## 1. Install and start (2 min)

```bash
cd ~/ros2_ws && source env.sh               # basestation workspace
cd src/BlueBoat-SideScanSonar/BlueBoat-MCS
pip install -r requirements.txt             # first time only
python3 run.py
```

(`~/ros2_ws` is the basestation's workspace. `/blueboat_ws` is the **boat's** —
different machine, different venv. See `06_installation.md`.)

The window opens; the status bar should read **ROS: connected**. Every row in
"ROS DIAGNOSTICS" is grey until the boat's nodes exist — normal.

## 2. Launch a safe first mission (1 min)

Click **Launch Mission**. Leave **motors disabled** (the default — the boat
will not move; every node still runs). Pick controller `LoS`, trajectory
`station_keeping`, press **Launch**.

Watch: the toolbar LED turns orange, launch output scrolls on the console
line, diagnostics rows turn green one by one, "Mission state" in the left
panel counts up to *operational*, and the LED turns green. The boat glyph
appears on the map. The green mission path is drawn only once the map's
georeference is heading-aligned (the `N↑` badge) — which needs a few metres of
motion, so on a stationary boat the map stays empty. That is expected.

## 3. Read the map (1 min)

Drag to pan, wheel to zoom. Click anywhere: the status bar shows the world
coordinates, the GPS coordinates (once "georef: ok" appears) and the distance
from the boat. Toggle layers on and off in the left panel. Click **Measure**,
then two points, to measure a distance.

## 4. Try a manual target (1 min)

Click **Manual Target**, then click a point a few metres from the boat. The
point is marked with a crosshair, a dashed purple line sketches the expected
LoS approach, and the "TARGET" panel switches to *MANUAL TARGET* with the live
distance. (With motors enabled on the water, the boat drives there and a
"Manual Target Reached" banner pops on arrival.) Click **Continue Original
Mission** to hand control back.

## 5. Monitor and control time (30 s)

The right panel plots the robot↔target distance live. Drag the right timeline
handle back to freeze the display on an earlier window — recording continues —
and press **Go Live** to return to the present. Statistics follow the selected
window.

## 6. Stop — two very different buttons

**Stop Mission** = normal end of experiment: graceful node shutdown, station
stays open, relaunch anytime.

**EMERGENCY STOP** = safety action: it *first* publishes the `default`
command to the boat, confirms transmission, and only then optionally kills
nodes. On the water, this is the button you reach for.

## Going on the water — pre-departure checklist

Boat powered and on the network; station started with the workspace sourced;
launch once with motors **disabled** and verify every diagnostics row is
green; verify the pinger panel updates when the USBL is wet; only then
relaunch with **ENABLE MOTORS** checked (the station asks you to confirm
twice).

### After launching a GPS-anchored mission — drive a few metres

A mission saved with a GPS reference (labelled **“(GPS)”** in the launch
dialog) is placed on its real-world coordinates, which the station can only do
once it knows how the boat's world frame is rotated against north. **That
rotation cannot be measured while the boat is stationary** — it comes from
comparing the GPS track against the odometry track, and a boat sitting still
produces neither.

So, right after launching one:

1. The boat **holds position** and the map shows no mission path. This is
   expected. It is not a hang, and nothing is wrong with the mission file.
2. **Drive the boat forward a few metres**, in whatever mode you normally use.
3. Watch the map for the `N↑` badge (and "georef" in the status bar). When it
   appears, the georeference is heading-aligned.
4. The mission deploys itself at that moment, and the green mission path
   appears on its true coordinates. No further operator action.

The same waiting period explains an empty map on *any* mission: the mission
path is deliberately not drawn before the `N↑` badge, because it cannot be
placed geographically until then. Satellite tiles are different — they need
only a GPS fix, so they usually appear first.

Where everything is explained: `04_user_guide.md`. What every topic means:
`03_ros_integration.md`.
