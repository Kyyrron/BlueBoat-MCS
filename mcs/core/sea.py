"""Sea state for simulated missions — presets, the launch choice, schedules
and the live readback of the simulator's ``sea_state_node``.

The simulator (BlueBoat-SSS-Sim) owns the physics and the preset table
(``share/blueboat_sss_sim/config/sea_states.yaml``,
``format: blueboat_sea_state_presets/1``). This module **reads that file
through the installed share directory and never imports the package** —
project rule CM-3, the same way :mod:`mcs.core.worlds` reads
``metadata.yaml``. Everything here is Qt-free so the smoke test can drive
it headless, and every reader follows the never-raises contract: a missing
or malformed file yields the built-in preset names with no descriptions.

Directions are compass degrees the current / waves come **from**
(meteorological convention, "from North to South" = 0°); the launch
arguments and the JSON command carry them unchanged.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path

import yaml

PRESETS_FORMAT = "blueboat_sea_state_presets/1"
SCHEDULE_FORMAT = "blueboat_sea_schedule/1"
#: Fallback preset names (the simulator's shipped table) when the file
#: cannot be read: the launch still works, only labels/descriptions are lost.
FALLBACK_CURRENT = ("none", "weak", "moderate", "strong", "very_strong")
FALLBACK_WAVES = ("calm", "rippled", "smooth", "slight", "rough")
NULL_CURRENT, NULL_WAVES = "none", "calm"

_COMPASS = ("N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW")


# ------------------------------------------------------------- compass
def compass_name(deg: float) -> str:
    """16-point compass name of a bearing in degrees clockwise from north."""
    return _COMPASS[int(round((float(deg) % 360.0) / 22.5)) % 16]


def compass_deg(name: str) -> float | None:
    """Bearing of a 16-point compass name (``None`` when unknown)."""
    key = str(name).strip().upper()
    return 22.5 * _COMPASS.index(key) if key in _COMPASS else None


def parse_direction(text: str) -> float | None:
    """Degrees from a compass name or a number; ``None`` when unparseable."""
    t = str(text).strip()
    d = compass_deg(t)
    if d is not None:
        return d
    try:
        return float(t) % 360.0
    except ValueError:
        return None


def direction_text(from_deg: float) -> str:
    """Operator-facing ``from N (→ S)`` for a from-bearing."""
    return (f"from {compass_name(from_deg)} "
            f"(→ {compass_name(from_deg + 180.0)}, {float(from_deg) % 360:.0f}°)")


# ------------------------------------------------------------- catalogue
def find_presets_file(explicit: str = "") -> Path | None:
    """The installed ``sea_states.yaml``: an explicit path, else every
    ``$AMENT_PREFIX_PATH`` entry, else ``~/ros2_ws/install``."""
    if explicit:
        p = Path(explicit).expanduser()
        return p if p.is_file() else None
    rel = Path("share") / "blueboat_sss_sim" / "config" / "sea_states.yaml"
    roots = [Path(p) for p in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if p]
    roots.append(Path.home() / "ros2_ws" / "install" / "blueboat_sss_sim")
    for root in roots:
        cand = root / rel
        if cand.is_file():
            return cand
    return None


@dataclass
class SeaCatalog:
    """Preset names + their labels/descriptions/references, in file order."""

    current: dict[str, dict] = field(default_factory=dict)
    waves: dict[str, dict] = field(default_factory=dict)
    source: str = ""            # the file the table came from ("" = fallback)

    @classmethod
    def fallback(cls) -> SeaCatalog:
        return cls(current={n: {"label": n.replace("_", " ")} for n in FALLBACK_CURRENT},
                   waves={n: {"label": n} for n in FALLBACK_WAVES})

    @classmethod
    def load(cls, path: str | Path | None) -> SeaCatalog:
        """Never raises: any failure answers the fallback catalogue."""
        if path is None:
            return cls.fallback()
        try:
            with open(path, "r", encoding="utf-8") as f:
                doc = yaml.safe_load(f)
            if not isinstance(doc, dict) or doc.get("format") != PRESETS_FORMAT:
                return cls.fallback()
            cur = {str(k): dict(v or {}) for k, v in (doc.get("current") or {}).items()}
            wav = {str(k): dict(v or {}) for k, v in (doc.get("waves") or {}).items()}
            if not cur or not wav:
                return cls.fallback()
            return cls(current=cur, waves=wav, source=str(path))
        except (OSError, yaml.YAMLError, AttributeError, TypeError):
            return cls.fallback()

    def label(self, kind: str, name: str) -> str:
        table = self.current if kind == "current" else self.waves
        return str((table.get(name) or {}).get("label") or name)

    def description(self, kind: str, name: str) -> str:
        table = self.current if kind == "current" else self.waves
        e = table.get(name) or {}
        ref = e.get("reference")
        desc = str(e.get("description") or "")
        return f"{desc}\n[{ref}]" if ref and ref != "--" else desc


# ------------------------------------------------------------- the choice
@dataclass
class SeaChoice:
    """What the operator picked for a simulated launch (or a live change)."""

    current: str = NULL_CURRENT
    current_from_deg: float = 0.0
    waves: str = NULL_WAVES
    waves_from_deg: float = 0.0
    schedule_file: str = ""       # a saved timeline overrides the four above
    seed: int = 0
    #: Custom waves (Hs, Tp, gamma, events...) instead of a preset: carried
    #: to the simulator as explicit numeric fields (a one-keyframe schedule
    #: at launch, the fields themselves in a live command).
    custom_waves: dict | None = None

    @property
    def is_null(self) -> bool:
        return (not self.schedule_file and self.current == NULL_CURRENT
                and self.waves == NULL_WAVES and not self.custom_waves)

    def waves_block(self) -> dict:
        """The ``waves:`` block of a keyframe / command for this choice."""
        if self.custom_waves:
            block = dict(self.custom_waves)
            block["from_deg"] = float(self.waves_from_deg)
            return block
        return {"preset": self.waves, "from_deg": float(self.waves_from_deg)}

    def launch_args(self) -> list[str]:
        """``sea_*:=`` arguments of ``full_mission_launch.py`` /
        ``sea_state_launch.py`` (declared there since 2026-09-03)."""
        args = [f"sea_current:={self.current}",
                f"sea_current_from_deg:={float(self.current_from_deg):.1f}",
                f"sea_waves:={self.waves}",
                f"sea_waves_from_deg:={float(self.waves_from_deg):.1f}"]
        if self.schedule_file:
            args.append(f"sea_schedule:={self.schedule_file}")
        if self.seed:
            args.append(f"sea_seed:={int(self.seed)}")
        return args

    def command_json(self, ramp_s: float = 20.0) -> str:
        """The ``/sim/sea_state/command`` payload for a live change."""
        if self.schedule_file:
            try:
                with open(self.schedule_file, "r", encoding="utf-8") as f:
                    doc = yaml.safe_load(f) or {}
            except (OSError, yaml.YAMLError):
                doc = {}
            return json.dumps({"schedule": doc, "t0": "now"})
        return json.dumps({
            "current": {"preset": self.current,
                        "from_deg": float(self.current_from_deg)},
            "waves": self.waves_block(),
            "ramp_s": float(ramp_s)})

    def summary(self, catalog: SeaCatalog | None = None) -> str:
        if self.schedule_file and not self.custom_waves:
            return f"timeline {Path(self.schedule_file).stem}"
        cat = catalog or SeaCatalog.fallback()
        parts = []
        if self.current != NULL_CURRENT:
            parts.append(f"current {cat.label('current', self.current)} "
                         f"{direction_text(self.current_from_deg)}")
        if self.custom_waves:
            cw = self.custom_waves
            parts.append(f"custom waves Hs {cw.get('hs_m', 0):.2f} m Tp "
                         f"{cw.get('tp_s', 0):.1f} s "
                         f"{direction_text(self.waves_from_deg)}")
        elif self.waves != NULL_WAVES:
            parts.append(f"waves {cat.label('waves', self.waves)} "
                         f"{direction_text(self.waves_from_deg)}")
        return "; ".join(parts) if parts else "calm water, no current"


# ------------------------------------------------------------- schedules
def list_schedules(schedules_dir: str | Path) -> list[Path]:
    """Saved timelines (``*.yaml`` of the schedule format), sorted by name."""
    d = Path(schedules_dir).expanduser()
    if not d.is_dir():
        return []
    out = []
    for p in sorted(d.glob("*.yaml")):
        try:
            with open(p, "r", encoding="utf-8") as f:
                doc = yaml.safe_load(f)
            if isinstance(doc, dict) and doc.get("format") == SCHEDULE_FORMAT:
                out.append(p)
        except (OSError, yaml.YAMLError):
            continue
    return out


def schedule_rows(doc: dict) -> list[dict]:
    """Keyframes of a schedule document as editor rows
    ``{t_s, current, current_from_deg, waves, waves_from_deg}``."""
    rows = []
    for kf in doc.get("keyframes") or []:
        cur = kf.get("current") or {}
        wav = kf.get("waves") or {}
        rows.append({
            "t_s": float(kf.get("t_s", 0.0)),
            "current": str(cur.get("preset", NULL_CURRENT)),
            "current_from_deg": _from_deg(cur),
            "waves": str(wav.get("preset", NULL_WAVES)),
            "waves_from_deg": _from_deg(wav),
        })
    return sorted(rows, key=lambda r: r["t_s"])


def _from_deg(block: dict) -> float:
    if "from_deg" in block:
        return float(block["from_deg"]) % 360.0
    d = parse_direction(str(block.get("from", 0.0)))
    return 0.0 if d is None else d


CUSTOM_WAVES = "custom"


def schedule_doc(rows: list[dict], name: str, seed: int = 0,
                 interpolation: str = "linear",
                 custom_waves: dict | None = None) -> dict:
    """Editor rows → a schedule document (the simulator's YAML/JSON shape).
    A row whose ``waves`` is ``custom`` expands to the explicit numeric
    fields of ``custom_waves`` (Hs, Tp, gamma, events...)."""
    def waves_of(r: dict) -> dict:
        if str(r["waves"]) == CUSTOM_WAVES and custom_waves:
            block = dict(custom_waves)
            block["from_deg"] = float(r["waves_from_deg"])
            return block
        return {"preset": str(r["waves"]), "from_deg": float(r["waves_from_deg"])}

    return {
        "format": SCHEDULE_FORMAT,
        "name": name,
        "seed": int(seed),
        "interpolation": interpolation,
        "keyframes": [{
            "t_s": float(r["t_s"]),
            "current": {"preset": str(r["current"]),
                        "from_deg": float(r["current_from_deg"])},
            "waves": waves_of(r),
        } for r in sorted(rows, key=lambda r: float(r["t_s"]))],
    }


def read_schedule(path: str | Path) -> dict | None:
    try:
        with open(path, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f)
    except (OSError, yaml.YAMLError):
        return None
    if not isinstance(doc, dict) or doc.get("format") != SCHEDULE_FORMAT:
        return None
    return doc


def write_schedule(path: str | Path, doc: dict) -> Path:
    p = Path(path).expanduser()
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    return p


# ------------------------------------------------------------- readback
@dataclass
class SeaReadout:
    """What ``/sim/sea_state`` (String JSON, 2 Hz, latched) says."""

    t_sim: float = 0.0
    current_label: str = ""
    current_speed_mps: float = 0.0
    current_from_deg: float = 0.0
    current_instant_mps: float = 0.0
    waves_label: str = ""
    waves_hs_m: float = 0.0
    waves_tp_s: float = 0.0
    waves_from_deg: float = 0.0
    waves_status: str = ""
    waves_reason: str = ""
    eta_m: float = 0.0
    events_per_hour: float = 0.0
    event: dict | None = None        # the live wake / swell group, if any
    next_t_sim: float | None = None
    next_text: str = ""
    summary: str = ""
    received_mono: float = 0.0

    @property
    def current_text(self) -> str:
        if self.current_speed_mps <= 0.0:
            return "none"
        return (f"{self.current_label} {direction_text(self.current_from_deg)}, "
                f"now {self.current_instant_mps:.2f} m/s")

    @property
    def waves_text(self) -> str:
        if self.waves_status == "unavailable":
            return f"unavailable ({self.waves_reason or 'no wrench path'})"
        if self.waves_hs_m <= 0.0 and not self.event:
            return "calm"
        base = (f"{self.waves_label} Hs {self.waves_hs_m:.2f} m Tp {self.waves_tp_s:.1f} s "
                f"{direction_text(self.waves_from_deg)}")
        if self.event:
            e = self.event
            base += (f" + wake group {float(e.get('hs_m', 0)):.2f} m "
                     f"{float(e.get('tp_s', 0)):.0f} s from "
                     f"{compass_name(float(e.get('from_deg', 0)))}, "
                     f"{float(e.get('t_remaining_s', 0)):.0f} s left")
        elif self.events_per_hour > 0:
            base += f" (wake groups ~{self.events_per_hour:.0f}/h)"
        return base

    @property
    def next_change_text(self) -> str:
        if self.next_t_sim is None:
            return "none scheduled"
        dt = self.next_t_sim - self.t_sim
        when = f"in {dt:.0f} s" if dt >= 0 else "now"
        return f"{when}: {self.next_text}" if self.next_text else when


def parse_readback(payload: str | dict, received_mono: float = 0.0
                   ) -> SeaReadout | None:
    """Decode one status message; ``None`` on malformed input."""
    try:
        d = json.loads(payload) if isinstance(payload, str) else dict(payload)
        cur, wav = d.get("current") or {}, d.get("waves") or {}
        inst = cur.get("instant") or {}
        sch = d.get("schedule") or {}
        return SeaReadout(
            t_sim=float(d.get("t_sim", 0.0)),
            current_label=str(cur.get("label", cur.get("preset", ""))),
            current_speed_mps=float(cur.get("mean_speed_mps", 0.0)),
            current_from_deg=float(cur.get("from_deg", 0.0)),
            current_instant_mps=float(inst.get("speed_mps", 0.0)),
            waves_label=str(wav.get("label", wav.get("preset", ""))),
            waves_hs_m=float(wav.get("hs_m", 0.0)),
            waves_tp_s=float(wav.get("tp_s", 0.0)),
            waves_from_deg=float(wav.get("from_deg", 0.0)),
            waves_status=str(wav.get("status", "")),
            waves_reason=str(wav.get("reason", "")),
            eta_m=float(wav.get("eta_m", 0.0)),
            events_per_hour=float(wav.get("events_per_hour", 0.0) or 0.0),
            event=(dict(wav["event"]) if isinstance(wav.get("event"), dict) else None),
            next_t_sim=(None if sch.get("next_t_sim") is None
                        else float(sch["next_t_sim"])),
            next_text=str(sch.get("next") or ""),
            summary=str(d.get("summary", "")),
            received_mono=float(received_mono),
        )
    except (ValueError, TypeError, AttributeError):
        return None
