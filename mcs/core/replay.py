"""Gazebo replay of a recorded real-robot run ("From a ros2bag", v1.1).

Qt-free and rclpy-free at import, like the rest of ``core/``: the smoke test
exercises it offline.

Since v1.1 every real-robot RUN files a ros2bag into its run folder,
``Robot_data/<stem>/<date>-<note>-bag/`` (BlueBoat-Control ``robot_interface`` +
``run_bag.py``). ``blueboat_control Replay_launch.py`` replays one in Gazebo:
the boat is moved KINEMATICALLY along the recorded pose (no controller, no sea
state, no force on the hull), and the recording is republished so the station
displays it as it would a live run -- the recorded GPS fixes, compass, odom
rebuilt with the full attitude, targets, thrust. The station supervises the
replay through two topics of ``bag_replay``:

* ``/replay/state`` (String JSON, latched, 2 Hz) -> :class:`ReplayReadout`;
* ``/replay/command`` (String JSON): :func:`command_json` -- pause, resume,
  playback rate.

This module reads a bag folder's ``metadata.yaml`` with PyYAML (no ROS), and
its GPS track through ``rosbag2_py`` only when that imports (sourced
workspace); without it, world eligibility degrades to "Empty Gazebo" only.
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass
from pathlib import Path

_LOG = logging.getLogger(__name__)

GPS_TOPIC = "/mavros/global_position/global"


# ------------------------------------------------------------------ bag files
def find_bag(path: str | Path) -> Path | None:
    """The bag directory for a picked folder: the bag itself (it holds a
    ``metadata.yaml``), or the run folder holding one (a ``*-bag`` preferred).
    None when there is none."""
    p = Path(path).expanduser()
    if (p / "metadata.yaml").is_file():
        return p
    if not p.is_dir():
        return None
    candidates = sorted(d for d in p.iterdir()
                        if d.is_dir() and (d / "metadata.yaml").is_file())
    preferred = [d for d in candidates
                 if d.name.rstrip("-0123456789").endswith("-bag")]
    found = preferred or candidates
    return found[0] if found else None


@dataclass
class BagSummary:
    path: Path
    duration_s: float
    message_count: int
    topics: dict[str, int]          # topic -> message count
    start_epoch_s: float | None

    @property
    def has_pose(self) -> bool:
        return any(self.topics.get(t, 0) > 0 for t in
                   ("/mavros/local_position/odom", "/blueboat/odom"))

    @property
    def has_gps(self) -> bool:
        return self.topics.get(GPS_TOPIC, 0) > 0

    def describe(self) -> str:
        start = (time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.start_epoch_s))
                 if self.start_epoch_s else "?")
        return (f"{self.path.name}: {format_clock(self.duration_s)} recorded "
                f"{start}, {self.message_count} messages on "
                f"{len(self.topics)} topics"
                f"{'' if self.has_gps else ' — no GPS fix (empty world only)'}"
                f"{'' if self.has_pose else ' — NO POSE TOPIC, cannot replay'}")


def bag_summary(bag: str | Path) -> BagSummary | None:
    """Parse a bag's ``metadata.yaml``; None when unreadable."""
    import yaml  # lazy, like core/worlds.py
    bag = Path(bag)
    try:
        doc = yaml.safe_load((bag / "metadata.yaml").read_text()) or {}
        info = doc["rosbag2_bagfile_information"]
        topics = {}
        for entry in info.get("topics_with_message_count") or []:
            meta = entry.get("topic_metadata") or {}
            topics[str(meta.get("name"))] = int(entry.get("message_count") or 0)
        start = (info.get("starting_time") or {}).get("nanoseconds_since_epoch")
        return BagSummary(
            path=bag,
            duration_s=float((info.get("duration") or {}).get("nanoseconds", 0)) * 1e-9,
            message_count=int(info.get("message_count") or 0),
            topics=topics,
            start_epoch_s=float(start) * 1e-9 if start else None)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        _LOG.warning("Unreadable bag metadata %s: %s", bag, exc)
        return None


def bag_gps_track(bag: str | Path, max_points: int = 400) -> list[tuple[float, float]]:
    """The bag's GPS fixes as (lat, lon), decimated to ``max_points``.

    Empty when ``rosbag2_py`` is unavailable or the bag holds no fix: the
    caller then offers the empty world only."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import NavSatFix
    except ImportError:
        return []
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=""),
                    rosbag2_py.ConverterOptions("", ""))
        reader.set_filter(rosbag2_py.StorageFilter(topics=[GPS_TOPIC]))
        pts: list[tuple[float, float]] = []
        while reader.has_next():
            _, data, _ = reader.read_next()
            fix = deserialize_message(data, NavSatFix)
            if fix.status.status < 0 or (fix.latitude == 0.0 and fix.longitude == 0.0):
                continue
            pts.append((fix.latitude, fix.longitude))
    except Exception as exc:  # noqa: BLE001 - any reader failure = no track
        _LOG.warning("Could not read the GPS track of %s: %s", bag, exc)
        return []
    step = max(1, len(pts) // max(1, max_points))
    return pts[::step]


# ----------------------------------------------------------------- live state
@dataclass
class ReplayReadout:
    """One ``/replay/state`` message, decoded."""

    bag: str
    t: float
    duration: float
    rate: float
    paused: bool
    finished: bool
    frame: str              # "empty" | "anchored"
    world: str
    gazebo: bool            # set_pose service reachable
    received_mono: float

    @property
    def progress(self) -> float:
        return min(1.0, self.t / self.duration) if self.duration > 0 else 0.0

    @property
    def status_text(self) -> str:
        if self.finished:
            return "finished — holding the last pose"
        if not self.gazebo:
            return "waiting for Gazebo…"
        return "paused" if self.paused else f"playing at {self.rate:g}×"


def parse_state(payload: dict, received_mono: float = 0.0) -> ReplayReadout | None:
    try:
        return ReplayReadout(
            bag=str(payload.get("bag", "")),
            t=float(payload.get("t", 0.0)),
            duration=float(payload.get("duration", 0.0)),
            rate=float(payload.get("rate", 1.0)),
            paused=bool(payload.get("paused", False)),
            finished=bool(payload.get("finished", False)),
            frame=str(payload.get("frame", "")),
            world=str(payload.get("world", "")),
            gazebo=bool(payload.get("gazebo", False)),
            received_mono=received_mono)
    except (TypeError, ValueError, AttributeError):
        return None


def command_json(cmd: str, rate: float | None = None) -> str:
    """A ``/replay/command`` payload: ``pause`` / ``resume`` / ``rate``."""
    if cmd not in ("pause", "resume", "rate"):
        raise ValueError(f"unknown replay command {cmd!r}")
    body: dict = {"cmd": cmd}
    if cmd == "rate":
        if rate is None or rate <= 0:
            raise ValueError("a rate command needs a positive rate")
        body["rate"] = float(rate)
    return json.dumps(body)


def format_clock(seconds: float | None) -> str:
    if seconds is None:
        return "0:00"
    seconds = int(max(0.0, seconds))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    return f"{h:d}:{m:02d}:{s:02d}" if h else f"{m:d}:{s:02d}"
