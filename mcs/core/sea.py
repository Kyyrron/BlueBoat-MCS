"""Sea state for simulated missions — what the operator chooses, how it
travels to the simulator, and the live readback of its ``sea_state_node``.

The simulator (BlueBoat-SSS-Sim) owns the physics and the preset table
(``share/blueboat_sss_sim/config/sea_states.yaml``,
``format: blueboat_sea_state_presets/1``). This module **reads that file
through the installed share directory and never imports the package** —
project rule CM-3, the same way :mod:`mcs.core.worlds` reads
``metadata.yaml``. Everything here is Qt-free so the smoke test can drive
it headless, and every reader follows the never-raises contract: a missing
or malformed file yields built-in names with no descriptions.

The operator's model (2026-10-08) is deliberately plain:

* **Current** — on/off, a *mean* speed, the direction it comes from, and a
  variation level (``steady`` / ``natural`` / ``strong``). The simulator
  turns the level into smooth, bounded variation around the mean.
* **Waves** — on/off, a strength (one of the simulator's wave presets, or
  a custom height), the direction they come from, the period (automatic
  from the height, or set), the crest length (short chop / long swell) and
  how often passing boats' wakes arrive.
* **Timeline** (advanced) — segments, each a full current + waves setting
  from its start time, the change into it blended over its transition.

Every choice travels as the simulator's own schedule document
(``blueboat_sea_schedule/1``: plain-language keys ``variation``,
``tp_s: auto``, ``crest``, ``wakes`` and per-keyframe ``ramp_s``) — a file
passed as ``sea_schedule:=`` at launch, or the JSON command of a live
change. Directions are compass degrees the current / waves come **from**
(meteorological convention, "from North to South" = 0°).
"""

from __future__ import annotations

import itertools
import json
import os
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import yaml

PRESETS_FORMAT = "blueboat_sea_state_presets/1"
SCHEDULE_FORMAT = "blueboat_sea_schedule/1"
#: Fallback names (the simulator's shipped table) when the file cannot be
#: read: the launch still works, only labels and descriptions are lost.
FALLBACK_CURRENT = ("none", "weak", "moderate", "strong", "very_strong")
FALLBACK_WAVES = ("calm", "rippled", "smooth", "slight", "rough")
FALLBACK_VARIATION = ("steady", "natural", "strong")
FALLBACK_CREST = ("short", "long")
FALLBACK_WAKES = ("none", "occasional", "frequent")
NULL_CURRENT, NULL_WAVES = "none", "calm"
CUSTOM_WAVES = "custom"

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


def clock_text(seconds: float) -> str:
    """``m:ss`` (or ``h:mm:ss``) of a time offset."""
    s = int(round(max(float(seconds), 0.0)))
    h, rem = divmod(s, 3600)
    m, sec = divmod(rem, 60)
    return f"{h}:{m:02d}:{sec:02d}" if h else f"{m}:{sec:02d}"


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
    """The simulator's tables, in file order: current and wave presets,
    variation levels, crest lengths, wake rates (labels, plain-language
    descriptions, and the numbers the dialog shows as hints)."""

    current: dict[str, dict] = field(default_factory=dict)
    waves: dict[str, dict] = field(default_factory=dict)
    variation: dict[str, dict] = field(default_factory=dict)
    crest: dict[str, dict] = field(default_factory=dict)
    wakes: dict[str, dict] = field(default_factory=dict)
    source: str = ""            # the file the table came from ("" = fallback)

    @classmethod
    def fallback(cls) -> SeaCatalog:
        def names(seq):
            return {n: {"label": n.replace("_", " ").capitalize()} for n in seq}
        return cls(current=names(FALLBACK_CURRENT), waves=names(FALLBACK_WAVES),
                   variation=names(FALLBACK_VARIATION), crest=names(FALLBACK_CREST),
                   wakes=names(FALLBACK_WAKES))

    @classmethod
    def load(cls, path: str | Path | None) -> SeaCatalog:
        """Never raises: any failure answers the fallback catalogue. A file
        from before the plain-language blocks (2026-10-08) keeps the
        fallback variation / crest / wakes names."""
        if path is None:
            return cls.fallback()
        try:
            with open(path, "r", encoding="utf-8") as f:
                doc = yaml.safe_load(f)
            if not isinstance(doc, dict) or doc.get("format") != PRESETS_FORMAT:
                return cls.fallback()

            def table(d) -> dict[str, dict]:
                return {str(k): dict(v or {}) for k, v in (d or {}).items()}
            cur, wav = table(doc.get("current")), table(doc.get("waves"))
            if not cur or not wav:
                return cls.fallback()
            fb = cls.fallback()
            opts = doc.get("wave_options") or {}
            return cls(current=cur, waves=wav,
                       variation=table(doc.get("current_variation")) or fb.variation,
                       crest=table(opts.get("crest")) or fb.crest,
                       wakes=table(opts.get("wakes")) or fb.wakes,
                       source=str(path))
        except (OSError, yaml.YAMLError, AttributeError, TypeError):
            return cls.fallback()

    # ---------------------------------------------------------- lookups
    def _table(self, kind: str) -> dict[str, dict]:
        return getattr(self, kind) if kind in ("current", "waves", "variation",
                                                "crest", "wakes") else {}

    def label(self, kind: str, name: str) -> str:
        return str((self._table(kind).get(name) or {}).get("label") or name)

    def description(self, kind: str, name: str) -> str:
        """The plain-language sentence for an entry (no references: those
        belong to the documentation, not to the operator's screen)."""
        return str((self._table(kind).get(name) or {}).get("description") or "")

    def number(self, kind: str, name: str, key: str, default: float = 0.0) -> float:
        try:
            return float((self._table(kind).get(name) or {}).get(key, default))
        except (TypeError, ValueError):
            return default

    def wave_levels(self) -> list[str]:
        """The non-calm wave presets, mildest first."""
        return [n for n in self.waves if n != NULL_WAVES]

    def current_shortcuts(self) -> list[tuple[str, float]]:
        """(label, speed) of the non-null current presets."""
        out = []
        for n, e in self.current.items():
            v = self.number("current", n, "speed_mps", 0.0)
            if n != NULL_CURRENT and v > 0.0:
                out.append((str(e.get("label") or n), v))
        return out

    def nearest_current(self, speed: float) -> str:
        """The current preset nearest a speed (for its description)."""
        best, bd = NULL_CURRENT, float("inf")
        for n in self.current:
            v = self.number("current", n, "speed_mps", -1.0)
            if v > 0.0 and abs(v - speed) < bd:
                best, bd = n, abs(v - speed)
        return best

    def wave_hs(self, level: str) -> float:
        return self.number("waves", level, "hs_m", 0.0)

    def auto_tp(self, hs: float) -> float:
        """The period the simulator picks for ``tp_s: auto`` -- the same
        rule as ``blueboat_sss_sim.sea.presets.SeaPresets.auto_tp``
        (piecewise linear through the presets' own (Hs, Tp), ``∝ sqrt(Hs)``
        beyond them), shown as a hint only: the simulator resolves it."""
        pts = sorted((self.number("waves", n, "hs_m"), self.number("waves", n, "tp_s"))
                     for n in self.waves)
        pts = [p for p in pts if p[0] > 0.0 and p[1] > 0.0]
        if hs <= 0.0 or not pts:
            return 0.0
        if hs <= pts[0][0]:
            return pts[0][1] * (hs / pts[0][0]) ** 0.5
        if hs >= pts[-1][0]:
            return pts[-1][1] * (hs / pts[-1][0]) ** 0.5
        for (h0, t0), (h1, t1) in itertools.pairwise(pts):
            if h0 <= hs <= h1:
                return t0 + (t1 - t0) * (hs - h0) / max(h1 - h0, 1e-9)
        return pts[-1][1]

    def variation_text(self, level: str) -> str:
        """``±10 % speed, ±10° direction`` for a variation level."""
        f = self.number("variation", level, "sigma_speed_frac", -1.0)
        d = self.number("variation", level, "sigma_dir_deg", -1.0)
        if f < 0.0 or d < 0.0:
            return ""
        return f"±{100 * f:.0f} % speed, ±{d:.0f}° direction"


# -------------------------------------------------------------- settings
@dataclass
class CurrentSetting:
    """The current card: a mean speed and direction, and how much it wanders."""

    enabled: bool = False
    speed_mps: float = 0.25
    from_deg: float = 0.0
    variation: str = "natural"

    @property
    def active(self) -> bool:
        return self.enabled and self.speed_mps > 0.0

    def block(self) -> dict:
        """The ``current:`` block of a keyframe / command."""
        if not self.active:
            return {"preset": NULL_CURRENT}
        return {"speed_mps": round(float(self.speed_mps), 4),
                "from_deg": round(float(self.from_deg) % 360.0, 2),
                "variation": str(self.variation)}

    @classmethod
    def from_block(cls, block: dict | None, catalog: SeaCatalog | None = None
                   ) -> CurrentSetting:
        """Tolerant reader: plain keys, a preset name (its speed from the
        catalogue), or the explicit numbers of older timelines."""
        b = dict(block or {})
        d = _from_deg(b)
        if "speed_mps" in b:
            v = float(b["speed_mps"])
        else:
            name = str(b.get("preset", NULL_CURRENT))
            v = (catalog.number("current", name, "speed_mps", 0.0) if catalog else 0.0)
            if name != NULL_CURRENT and v <= 0.0 and catalog is None:
                v = {"weak": 0.1, "moderate": 0.25, "strong": 0.5,
                     "very_strong": 0.8}.get(name, 0.0)
        var = str(b.get("variation") or "natural")
        return cls(enabled=v > 0.0, speed_mps=v if v > 0.0 else 0.25,
                   from_deg=d, variation=var)

    def summary(self, catalog: SeaCatalog | None = None) -> str:
        if not self.active:
            return "no current"
        var = (catalog.label("variation", self.variation) if catalog
               else self.variation).lower()
        return (f"current {self.speed_mps:.2f} m/s {direction_text(self.from_deg)}, "
                f"{var} variation")


@dataclass
class WaveSetting:
    """The waves card: a strength, a direction and how the sea looks."""

    enabled: bool = False
    level: str = "smooth"          # a wave preset, or CUSTOM_WAVES
    hs_m: float = 0.20             # the custom height
    from_deg: float = 0.0
    period_auto: bool = True
    tp_s: float = 3.0              # the set period when not automatic
    crest: str = "short"
    wakes: str = "occasional"

    @property
    def active(self) -> bool:
        return self.enabled and (self.level != CUSTOM_WAVES or self.hs_m > 0.0)

    def height(self, catalog: SeaCatalog | None = None) -> float:
        if self.level == CUSTOM_WAVES:
            return float(self.hs_m)
        return catalog.wave_hs(self.level) if catalog else 0.0

    def block(self) -> dict:
        """The ``waves:`` block of a keyframe / command."""
        if not self.active:
            return {"preset": NULL_WAVES}
        b: dict = ({"hs_m": round(float(self.hs_m), 3)} if self.level == CUSTOM_WAVES
                   else {"preset": str(self.level)})
        b["from_deg"] = round(float(self.from_deg) % 360.0, 2)
        if not self.period_auto:
            b["tp_s"] = round(float(self.tp_s), 2)
        elif self.level == CUSTOM_WAVES:
            b["tp_s"] = "auto"
        b["crest"] = str(self.crest)
        b["wakes"] = str(self.wakes)
        return b

    @classmethod
    def from_block(cls, block: dict | None, catalog: SeaCatalog | None = None
                   ) -> WaveSetting:
        b = dict(block or {})
        d = _from_deg(b)
        tp = b.get("tp_s")
        out = cls(from_deg=d, crest=str(b.get("crest") or "short"),
                  wakes=str(b.get("wakes") or "occasional"))
        if "hs_m" in b and "preset" not in b:
            out.level, out.hs_m = CUSTOM_WAVES, float(b["hs_m"])
            out.enabled = out.hs_m > 0.0
        else:
            name = str(b.get("preset", NULL_WAVES))
            out.enabled = name != NULL_WAVES
            out.level = name if out.enabled else "smooth"
        if isinstance(tp, (int, float)) and float(tp) > 0.0:
            out.period_auto, out.tp_s = False, float(tp)
        if "events" in b and "wakes" not in b:
            ev = b.get("events") or {}
            rate = float(ev.get("rate_per_hour", 0.0) or 0.0)
            out.wakes = "none" if rate <= 0.0 else "occasional"
        return out

    def summary(self, catalog: SeaCatalog | None = None) -> str:
        if not self.active:
            return "flat water"
        name = (f"waves {self.hs_m:.2f} m" if self.level == CUSTOM_WAVES else
                (catalog.label("waves", self.level) if catalog else self.level).lower())
        return f"{name} {direction_text(self.from_deg)}"


def _from_deg(block: dict) -> float:
    if "from_deg" in block:
        return float(block["from_deg"]) % 360.0
    d = parse_direction(str(block.get("from", 0.0)))
    return 0.0 if d is None else d


@dataclass
class Segment:
    """One timeline segment: the sea from ``start_s``, reached over
    ``transition_s`` (the simulator always blends, never steps)."""

    start_s: float = 0.0
    transition_s: float = 60.0
    current: CurrentSetting = field(default_factory=CurrentSetting)
    waves: WaveSetting = field(default_factory=WaveSetting)

    def summary(self, catalog: SeaCatalog | None = None) -> str:
        return f"{self.current.summary(catalog)}; {self.waves.summary(catalog)}"


MIN_TRANSITION_S = 5.0


@dataclass
class SeaTimeline:
    """The advanced plan: segments in start order (the first starts at 0)."""

    segments: list[Segment] = field(default_factory=list)
    name: str = ""

    def sorted(self) -> list[Segment]:
        return sorted(self.segments, key=lambda s: s.start_s)

    def to_doc(self, seed: int = 0) -> dict:
        """The simulator's schedule document: ``hold`` keyframes, each with
        its own ``ramp_s`` (the "change gradually over" of the editor)."""
        kfs = []
        for i, s in enumerate(self.sorted()):
            kf = {"t_s": 0.0 if i == 0 else round(float(s.start_s), 1)}
            if i > 0:
                kf["ramp_s"] = round(max(float(s.transition_s), MIN_TRANSITION_S), 1)
            kf["current"] = s.current.block()
            kf["waves"] = s.waves.block()
            kfs.append(kf)
        return {"format": SCHEDULE_FORMAT, "name": self.name or "timeline",
                "seed": int(seed), "interpolation": "hold", "keyframes": kfs}

    @classmethod
    def from_doc(cls, doc: dict, catalog: SeaCatalog | None = None,
                 name: str = "") -> SeaTimeline:
        """Any schedule document, including older ``linear`` ones: a linear
        blend from keyframe k-1 to k is the same water as a segment that
        starts at t(k-1) and changes over the whole gap."""
        kfs = sorted((dict(k) for k in doc.get("keyframes") or []),
                     key=lambda k: float(k.get("t_s", k.get("t", 0.0))))
        linear = str(doc.get("interpolation", "linear")) == "linear"
        segs: list[Segment] = []
        prev_t = 0.0
        for i, k in enumerate(kfs):
            t = float(k.get("t_s", k.get("t", 0.0)))
            cur = CurrentSetting.from_block(k.get("current"), catalog)
            wav = WaveSetting.from_block(k.get("waves"), catalog)
            if i == 0:
                segs.append(Segment(0.0, 60.0, cur, wav))
            elif linear:
                segs.append(Segment(prev_t, max(t - prev_t, MIN_TRANSITION_S), cur, wav))
            else:
                segs.append(Segment(t, float(k.get("ramp_s", 20.0)), cur, wav))
            prev_t = t
        return cls(segs, name or str(doc.get("name") or ""))

    def end_s(self) -> float:
        s = self.sorted()
        return (s[-1].start_s + max(s[-1].transition_s, 0.0)) if s else 0.0

    def summary(self) -> str:
        n = len(self.segments)
        return (f"timeline '{self.name or 'unsaved'}': {n} segment"
                f"{'s' if n != 1 else ''}, last change at "
                f"{clock_text(self.sorted()[-1].start_s) if n else '0:00'}")


# ------------------------------------------------------------- the choice
@dataclass
class SeaChoice:
    """What the operator picked for a simulated launch (or a live change).

    ``scenario_file`` is the schedule document the launch passes as
    ``sea_schedule:=`` (written by :meth:`write_scenario`; the dialog does it
    on OK). ``timeline`` replaces the two cards when set.
    """

    current: CurrentSetting = field(default_factory=CurrentSetting)
    waves: WaveSetting = field(default_factory=WaveSetting)
    seed: int = 7
    timeline: SeaTimeline | None = None
    scenario_file: str = ""

    @property
    def is_null(self) -> bool:
        if self.timeline is not None and self.timeline.segments:
            return False
        return not self.current.active and not self.waves.active

    def schedule_document(self, name: str = "") -> dict:
        if self.timeline is not None and self.timeline.segments:
            doc = self.timeline.to_doc(self.seed)
            if name:
                doc["name"] = name
            return doc
        return {"format": SCHEDULE_FORMAT, "name": name or "sea_state",
                "seed": int(self.seed), "interpolation": "hold",
                "keyframes": [{"t_s": 0.0, "current": self.current.block(),
                               "waves": self.waves.block()}]}

    def write_scenario(self, schedules_dir: str | Path,
                       name: str = "last_launch") -> str:
        """Write the scenario the launch will pass and remember its path."""
        p = write_schedule(Path(schedules_dir).expanduser() / f"{name}.yaml",
                           self.schedule_document(name))
        self.scenario_file = str(p)
        return self.scenario_file

    def launch_args(self, schedules_dir: str | Path | None = None) -> list[str]:
        """``sea_schedule:=`` / ``sea_seed:=`` arguments of
        ``full_mission_launch.py`` / ``sea_state_launch.py``. Every choice
        travels as a scenario file (the plain-language keys have no preset
        name); one is written on demand if the dialog has not."""
        if not self.scenario_file:
            self.write_scenario(schedules_dir or SCENARIO_DIR_DEFAULT)
        return [f"sea_schedule:={self.scenario_file}", f"sea_seed:={int(self.seed)}"]

    def command_json(self, ramp_s: float = 20.0) -> str:
        """The ``/sim/sea_state/command`` payload for a live change."""
        if self.timeline is not None and self.timeline.segments:
            return json.dumps({"schedule": self.schedule_document(
                self.timeline.name or "timeline"), "t0": "now"})
        return json.dumps({"current": self.current.block(),
                           "waves": self.waves.block(),
                           "ramp_s": float(ramp_s)})

    def summary(self, catalog: SeaCatalog | None = None) -> str:
        if self.timeline is not None and self.timeline.segments:
            return self.timeline.summary()
        if self.is_null:
            return "calm water, no current"
        parts = []
        if self.current.active:
            parts.append(self.current.summary(catalog))
        if self.waves.active:
            parts.append(self.waves.summary(catalog))
        return "; ".join(parts)

    def copy(self) -> SeaChoice:
        return replace(self, current=replace(self.current), waves=replace(self.waves),
                       timeline=(None if self.timeline is None else SeaTimeline(
                           [replace(s, current=replace(s.current), waves=replace(s.waves))
                            for s in self.timeline.segments], self.timeline.name)))

    def to_dict(self) -> dict:
        return asdict(self)


#: Where a choice built outside the dialog writes its scenario file.
SCENARIO_DIR_DEFAULT = Path.home() / ".config" / "blueboat_mcs" / "sea_states"


# ------------------------------------------------------------- schedules
def list_schedules(schedules_dir: str | Path) -> list[Path]:
    """Saved timelines (``*.yaml`` of the schedule format), sorted by name;
    the automatic ``last_launch`` scenario is not offered."""
    d = Path(schedules_dir).expanduser()
    if not d.is_dir():
        return []
    out = []
    for p in sorted(d.glob("*.yaml")):
        if p.stem in ("last_launch", "custom_waves", "timeline_unsaved"):
            continue
        try:
            with open(p, "r", encoding="utf-8") as f:
                doc = yaml.safe_load(f)
            if isinstance(doc, dict) and doc.get("format") == SCHEDULE_FORMAT:
                out.append(p)
        except (OSError, yaml.YAMLError):
            continue
    return out


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
    current_instant_from_deg: float = 0.0
    current_variation: str = ""
    waves_label: str = ""
    waves_hs_m: float = 0.0
    waves_tp_s: float = 0.0
    waves_from_deg: float = 0.0
    waves_status: str = ""
    waves_reason: str = ""
    waves_crest: str = ""
    eta_m: float = 0.0
    events_per_hour: float = 0.0
    event: dict | None = None        # the live wake / swell group, if any
    boat: dict = field(default_factory=dict)   # roll/pitch/heave std over 30 s
    hull_model: str = ""             # "catamaran" | "gazebo_box" ("" = older sim)
    next_t_sim: float | None = None
    next_text: str = ""
    blend: float = 1.0
    summary: str = ""
    received_mono: float = 0.0

    @property
    def current_text(self) -> str:
        if self.current_speed_mps <= 0.0:
            return "none"
        return (f"{self.current_instant_mps:.2f} m/s from "
                f"{compass_name(self.current_instant_from_deg)} · mean "
                f"{self.current_speed_mps:.2f} from {compass_name(self.current_from_deg)}"
                + (f" ({self.current_variation})" if self.current_variation else ""))

    @property
    def waves_text(self) -> str:
        if self.waves_status == "unavailable":
            return f"unavailable ({self.waves_reason or 'no wrench path'})"
        if self.waves_hs_m <= 0.0 and not self.event:
            return "flat water"
        name = (self.waves_label if " m)" in self.waves_label
                else f"{self.waves_label or 'waves'} {self.waves_hs_m:.2f} m")
        base = (f"{name}, every {self.waves_tp_s:.1f} s, from "
                f"{compass_name(self.waves_from_deg)}"
                if self.waves_hs_m > 0.0 else "flat water")
        if self.event:
            e = self.event
            base += (f"; passing wake {float(e.get('hs_m', 0)):.2f} m "
                     f"from {compass_name(float(e.get('from_deg', 0)))}, "
                     f"{float(e.get('t_remaining_s', 0)):.0f} s left")
        elif self.events_per_hour > 0:
            base += f" (wakes ~{self.events_per_hour:.0f}/h)"
        return base

    @property
    def boat_text(self) -> str:
        """What the sea does to the hull, from the simulator's own odometry
        window (empty until it has one)."""
        b = self.boat or {}
        if "roll_std_deg" not in b:
            return "—"
        r, p, h = (float(b.get("roll_std_deg", 0.0)), float(b.get("pitch_std_deg", 0.0)),
                   float(b.get("heave_std_m", 0.0)))
        if r < 0.05 and p < 0.05 and h < 0.002:
            return "level and steady"
        return f"roll ±{r:.1f}°, pitch ±{p:.1f}°, heave ±{100 * h:.0f} cm"

    @property
    def next_change_text(self) -> str:
        if self.blend < 1.0:
            return f"changing now ({100 * self.blend:.0f} %)"
        if self.next_t_sim is None:
            return "none planned"
        dt = self.next_t_sim - self.t_sim
        when = f"in {clock_text(dt)}" if dt >= 0 else "now"
        return f"{when}: {self.next_text}" if self.next_text else when


def parse_readback(payload: str | dict, received_mono: float = 0.0
                   ) -> SeaReadout | None:
    """Decode one status message; ``None`` on malformed input. Every field
    added after 2026-09 is optional (older simulators omit it)."""
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
            current_instant_from_deg=float(inst.get("from_deg", cur.get("from_deg", 0.0))),
            current_variation=str(cur.get("variation") or ""),
            waves_label=str(wav.get("label", wav.get("preset", ""))),
            waves_hs_m=float(wav.get("hs_m", 0.0)),
            waves_tp_s=float(wav.get("tp_s", 0.0)),
            waves_from_deg=float(wav.get("from_deg", 0.0)),
            waves_status=str(wav.get("status", "")),
            waves_reason=str(wav.get("reason", "")),
            waves_crest=str(wav.get("crest") or ""),
            eta_m=float(wav.get("eta_m", 0.0)),
            events_per_hour=float(wav.get("events_per_hour", 0.0) or 0.0),
            event=(dict(wav["event"]) if isinstance(wav.get("event"), dict) else None),
            boat=dict(d.get("boat") or {}),
            hull_model=str((d.get("hull") or {}).get("model") or ""),
            next_t_sim=(None if sch.get("next_t_sim") is None
                        else float(sch["next_t_sim"])),
            next_text=str(sch.get("next") or ""),
            blend=float(d.get("blend", 1.0)),
            summary=str(d.get("summary", "")),
            received_mono=float(received_mono),
        )
    except (ValueError, TypeError, AttributeError):
        return None
