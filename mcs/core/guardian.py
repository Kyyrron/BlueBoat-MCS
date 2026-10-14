"""Link guardian — the station's model of it (v1.1, 2026-10-14, by the user's order).

The guardian brings the boat home on its own when this laptop goes silent: an
ArduPilot Lua script on the autopilot (superproject ``link_guardian/``), switched
ON for every real-robot session by its laptop companion ``guardian.py watch``,
which the station starts with the launch (``launch_manager.guardian_command``).
The companion reports on ``/link_guardian/status`` (String JSON, 1 Hz) and obeys
``/link_guardian/command`` (``{cmd: on|off|check|setup|reclaim, seq}``). The
station adds no guardian logic of its own: this module only turns the companion's
report into what the GUARDIAN box shows and which of its buttons may act.

Qt-free and rclpy-free, so the smoke test can pin every rule.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

#: Modes the guardian (or an ArduPilot failsafe) puts the boat in.
AUTONOMOUS_MODES = frozenset({"RTL", "SMART_RTL", "GUIDED", "LOITER", "HOLD"})
#: A companion silent for longer than this is "not reporting".
STATUS_FRESH_S = 4.0


@dataclass
class GuardianReadout:
    """One ``/link_guardian/status`` message."""

    received_t: float = 0.0
    connected: bool = False
    armed: bool = False
    mode: str = ""
    want_on: bool = True
    guardian_on: bool | None = None          # what the autopilot confirmed
    check: dict | None = None                # last readiness report
    home: dict | None = None                 # {lat, lon, source}
    run_running: bool = False
    messages: list = field(default_factory=list)   # [{t, severity, text}]
    busy: str | None = None
    ack: dict | None = None                  # {seq, cmd, ok, detail, t}

    @classmethod
    def from_json(cls, payload: dict, received_mono: float | None = None
                  ) -> GuardianReadout | None:
        if not isinstance(payload, dict):
            return None
        on = payload.get("guardian_on")
        check = payload.get("check")
        home = payload.get("home")
        ack = payload.get("ack")
        msgs = payload.get("messages")
        return cls(
            received_t=time.monotonic() if received_mono is None else received_mono,
            connected=bool(payload.get("connected", False)),
            armed=bool(payload.get("armed", False)),
            mode=str(payload.get("mode") or ""),
            want_on=bool(payload.get("want_on", True)),
            guardian_on=None if on is None else bool(on),
            check=check if isinstance(check, dict) else None,
            home=home if isinstance(home, dict) else None,
            run_running=bool(payload.get("run_running", False)),
            messages=[m for m in msgs if isinstance(m, dict)] if isinstance(msgs, list) else [],
            busy=payload.get("busy") or None,
            ack=ack if isinstance(ack, dict) else None,
        )

    def fresh(self, now: float | None = None, max_age_s: float = STATUS_FRESH_S) -> bool:
        now = time.monotonic() if now is None else now
        return (now - self.received_t) <= max_age_s

    @property
    def script_installed(self) -> bool | None:
        return None if self.check is None else bool(self.check.get("script"))

    @property
    def ready(self) -> bool | None:
        return None if self.check is None else bool(self.check.get("ready"))

    def check_items(self) -> list[dict]:
        items = (self.check or {}).get("items") or []
        return [i for i in items if isinstance(i, dict)]


@dataclass
class Action:
    """One button: may it act now, and if not, why."""

    enabled: bool
    reason: str = ""


@dataclass
class GuardianView:
    """Everything the GUARDIAN box displays, derived from the readout."""

    state: str
    level: str                     # 'off' | 'ok' | 'info' | 'warn' | 'err'
    link: tuple[str, str]          # (text, level)
    autopilot: tuple[str, str]
    flag: tuple[str, str]
    setup: tuple[str, str]
    home: tuple[str, str]
    messages: list[str]
    check: Action
    setup_action: Action
    toggle: Action
    toggle_on: bool                # the toggle would turn the guardian ON
    rearm: Action
    reclaim: Action


def summarize(r: GuardianReadout | None, *, real_session: bool, companion_found: bool,
              fcu_connected: bool, armed: bool, mode: str, run_active: bool,
              estop_latched: bool, now: float | None = None) -> GuardianView:
    """Derive the box's state, rows and button availability.

    ``fcu_connected`` / ``armed`` / ``mode`` come from the station's own
    ``/mavros/state`` subscription (current even when the companion lags);
    the rest from the companion's report.
    """
    off = Action(False, "real-robot sessions only")
    if not real_session:
        return GuardianView(
            state="OFF — real-robot sessions only", level="off",
            link=("—", "off"), autopilot=("—", "off"), flag=("—", "off"),
            setup=("—", "off"), home=("—", "off"), messages=[],
            check=off, setup_action=off, toggle=off, toggle_on=False,
            rearm=off, reclaim=off)

    reporting = r is not None and r.fresh(now)
    busy = r.busy if reporting else None
    autonomous = mode in AUTONOMOUS_MODES

    # ---- rows -------------------------------------------------------------
    link = (("connected", "ok") if fcu_connected
            else ("LOST", "err"))
    autopilot = ((f"ARMED · {mode or '?'}", "err" if autonomous else "ok") if armed
                 else (f"disarmed · {mode or '?'}", "warn"))
    if not reporting:
        flag = ("companion not reporting", "warn")
    elif r.guardian_on is True:
        flag = ("ON", "ok")
    elif r.guardian_on is False:
        flag = ("OFF (operator)" if not r.want_on else "OFF", "warn")
    else:
        flag = ("switching on…" if r.want_on else "—", "info")
    if not reporting or r.check is None:
        setup = ("checking…" if reporting else "—", "info")
    elif not r.script_installed:
        setup = ("script NOT installed", "err")
    elif r.ready:
        warns = [i["name"] for i in r.check_items() if i.get("level") == "warn"]
        setup = (("READY" + (f" ({len(warns)} advice)" if warns else "")),
                 "warn" if warns else "ok")
    else:
        fails = [i["name"] for i in r.check_items() if i.get("level") == "fail"]
        setup = ("NOT READY: " + ", ".join(fails), "err")
    if reporting and r.home and r.home.get("lat") is not None:
        home = (f"{r.home['lat']:.6f}, {r.home['lon']:.6f}"
                + (f" ({r.home['source']})" if r.home.get("source") else ""), "ok")
    else:
        home = ("not set yet (set when the autopilot arms)", "warn")
    messages = [str(m.get("text", "")) for m in (r.messages if reporting else [])][-3:]

    # ---- state ------------------------------------------------------------
    if not companion_found:
        state, level = "UNAVAILABLE — guardian.py not found (see the config)", "err"
    elif not fcu_connected:
        if r is not None and r.guardian_on:
            state, level = "LINK LOST — the boat returns HOME on its own", "err"
        else:
            state, level = "Waiting for the boat to connect…", "info"
    elif not reporting:
        state, level = "Starting the guardian companion…", "info"
    elif r.check is None:
        state, level = "Checking the guardian on the autopilot…", "info"
    elif not r.script_installed:
        state, level = "NOT INSTALLED — upload the script (README)", "err"
    elif not r.ready:
        state, level = "NOT READY — see Setup", "err"
    elif r.guardian_on is False:
        state, level = "OFF — the boat will NOT return on link loss", "warn"
    elif r.guardian_on is None:
        state, level = "Switching the guardian ON…", "info"
    elif armed and autonomous:
        state, level = f"TAKEN OVER — boat in {mode}; Take back control", "err"
    elif armed:
        state, level = "ON — returns HOME after 5 s without this laptop", "ok"
    else:
        state, level = "ON — autopilot disarmed: arms itself on link loss", "warn"

    # ---- buttons ----------------------------------------------------------
    def gate(*conds: tuple[bool, str]) -> Action:
        for ok, why in conds:
            if not ok:
                return Action(False, why)
        return Action(True)

    base = ((companion_found, "guardian.py not found"),
            (fcu_connected, "the boat is not connected"),
            (reporting, "the guardian companion is not reporting"),
            (busy is None, f"busy: {busy}"))
    check = gate(*base)
    setup_action = gate(*base, (not run_active, "a run is active"))
    toggle_on = bool(reporting and not r.want_on)
    toggle = gate(*base, (bool(reporting and r.script_installed), "the script is not installed"))
    rearm = gate((fcu_connected, "the boat is not connected"),
                 (not armed, "the autopilot is already armed"),
                 (not estop_latched, "E-STOP latched — release it first"),
                 (not run_active, "a run is active — stop it first"))
    reclaim = gate(*base, (autonomous, ("nothing to take back: the boat is not "
                                        "under guardian / failsafe control")),
                   (not run_active, "a run is still active — press STOP RUN first"))

    return GuardianView(state=state, level=level, link=link, autopilot=autopilot,
                        flag=flag, setup=setup, home=home, messages=messages,
                        check=check, setup_action=setup_action, toggle=toggle,
                        toggle_on=toggle_on, rearm=rearm, reclaim=reclaim)


def needs_turn_off_choice(r: GuardianReadout | None, *, real_session: bool,
                          now: float | None = None) -> bool:
    """Should Turn Off Connection / app exit ask what the guardian does?

    True whenever a real-robot session might leave the guardian ON on the
    autopilot: the boat would then see this laptop go silent and return HOME.
    Only a fresh report that says the script is absent, or the flag is OFF,
    makes the question unnecessary — a silent companion is no proof of OFF.
    """
    if not real_session:
        return False
    if r is None or not r.fresh(now):
        return True
    if r.check is not None and not r.script_installed:
        return False
    return r.guardian_on is not False


def command_json(cmd: str, seq: int, **extra) -> dict:
    """One ``/link_guardian/command`` payload."""
    payload = {"cmd": cmd, "seq": int(seq)}
    payload.update(extra)
    return payload
