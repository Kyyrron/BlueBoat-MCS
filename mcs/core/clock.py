"""Elapsed-time text shared by every read-out, and the station clock every
reception stamp, age, rate and elapsed time reads (Qt-free)."""

from __future__ import annotations

import threading as _threading
import time as _time


def format_clock(seconds: float | None) -> str:
    """``m:ss``, or ``h:mm:ss`` from one hour, truncated to whole seconds;
    None (and anything negative) reads ``0:00``."""
    if seconds is None:
        return "0:00"
    seconds = int(max(0.0, seconds))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    return f"{h:d}:{m:02d}:{s:02d}" if h else f"{m:d}:{s:02d}"


# ---------------------------------------------------------------------------
# The station clock (2026-11-07)
# ---------------------------------------------------------------------------
# Every reception stamp, age, rate and elapsed time of the station reads
# :func:`station_clock` ``.now()`` rather than ``time.monotonic()``.
#
# * **Real-robot and replay launches** (and before any launch): the clock is
#   ``time.monotonic()`` -- nothing changes.
# * **Simulation launches** (empty Gazebo and personalized worlds): the bridge
#   feeds every ``/clock`` sample and the clock advances by **sim seconds**,
#   continuous with the monotonic value it had when the feed began,
#   interpolated between samples by the measured sim/wall ratio. A run slowed
#   below real time (the ray-cast sonar, Gazebo real-time factor 0.01-0.5)
#   then shows its rates, ages, elapsed time, plots and statistics in sim
#   time -- 20 Hz odometry reads 20 Hz, not 2 Hz.
# * ``/clock`` silent for ``stall_s`` wall seconds (Gazebo closed, paused or
#   hung): the clock advances at wall pace again, so staleness still shows.
# * It never goes backwards (a restarted Gazebo re-bases).
#
# Self-contained operator deadlines (the safe-shutdown and guardian waits, the
# path-preview timeout, the arrow-key stream) stay on ``time.monotonic()``:
# they time the operator and the link, not the simulated world.
# The algorithm is the GCS's ``core/stream_clock.py``, reimplemented here
# (CM-3: no module imports a neighbour).

#: Time constant [wall s] of the measured sim/wall ratio's smoothing.
_RATIO_TAU_S = 2.0
#: Largest sim/wall ratio measured and interpolated (owner order 2026-11-10:
#: a run may go up to x4; it was 2.0 / 1.5 when runs stopped at x1).
_RATIO_CAP = 5.0


class StationClock:
    """See the section docstring. Thread-safe (fed from the ROS thread,
    read from both)."""

    def __init__(self, stall_s: float = 3.0) -> None:
        self.stall_s = float(stall_s)
        self._lock = _threading.Lock()
        self._floor = -float("inf")
        self._follow = False
        self._clear()

    def _clear(self) -> None:
        self._fed = False
        self._v_ref = self._s_ref = self._s_last = self._m_last = 0.0
        self._ratio = 1.0

    def set_follow(self, follow: bool) -> None:
        """Follow ``/clock`` (a simulation launch) or not (real, replay, idle).
        Every call drops the current feed: a new launch re-anchors."""
        with self._lock:
            self._follow = bool(follow)
            self._clear()

    def feed(self, sim_s: float, mono_s: float | None = None) -> None:
        """One ``/clock`` sample; ignored unless following."""
        m = _time.monotonic() if mono_s is None else float(mono_s)
        s = float(sim_s)
        with self._lock:
            if not self._follow:
                return
            if not self._fed:
                self._v_ref, self._s_ref = max(m, self._floor), s
                self._s_last, self._m_last, self._fed = s, m, True
                return
            if s < self._s_last - 1e-9:                  # Gazebo restarted
                self._v_ref = max(self._value(m), self._floor)
                self._s_ref, self._s_last, self._m_last = s, s, m
                return
            dm = m - self._m_last
            if dm > 1e-6:
                a = min(1.0, dm / _RATIO_TAU_S)
                inst = (s - self._s_last) / dm
                self._ratio = max(0.0, min(_RATIO_CAP, (1.0 - a) * self._ratio + a * inst))
            self._s_last, self._m_last = s, m

    def now(self, mono_s: float | None = None) -> float:
        m = _time.monotonic() if mono_s is None else float(mono_s)
        with self._lock:
            v = self._value(m) if self._fed else m
            v = max(v, self._floor)
            self._floor = v
            return v

    def _value(self, m: float) -> float:
        gap = max(0.0, m - self._m_last)
        interp = min(gap, self.stall_s)
        return (self._v_ref + (self._s_last - self._s_ref)
                + interp * min(self._ratio, _RATIO_CAP) + max(0.0, gap - self.stall_s))

    @property
    def sim_paced(self) -> bool:
        with self._lock:
            return self._fed

    @property
    def following(self) -> bool:
        with self._lock:
            return self._follow

    def stalled(self, mono_s: float | None = None) -> bool:
        m = _time.monotonic() if mono_s is None else float(mono_s)
        with self._lock:
            return self._fed and m - self._m_last > self.stall_s

    @property
    def ratio(self) -> float | None:
        """The measured real-time factor, None while not fed."""
        with self._lock:
            return self._ratio if self._fed else None


def sim_badge(clock: StationClock, requested_rtf: float | None,
              warn_rel: float = 0.20, mono_s: float | None = None) -> tuple[str, str]:
    """The toolbar's simulation-speed badge: ``(text, level)``, ``("", "")``
    when the clock is not following a simulation. ``requested_rtf`` is the
    launch's real-time factor (1.0 at real speed)."""
    if not clock.following:
        return "", ""
    req = 1.0 if requested_rtf is None else float(requested_rtf)
    head = f"SIM x{req:.3f}" if abs(req - 1.0) > 1e-9 else "SIM x1"   # x2-x4 since 2026-11-10
    if not clock.sim_paced:
        return f"{head} · waiting for /clock", "ok"
    if clock.stalled(mono_s):
        return f"{head} · sim clock stopped", "warn"
    meas = clock.ratio or 0.0
    level = "warn" if req > 0 and abs(meas - req) > warn_rel * req else "ok"
    return f"{head} · measured x{meas:.3f}", level


_STATION_CLOCK = StationClock()


def station_clock() -> StationClock:
    """The process-wide station clock."""
    return _STATION_CLOCK
