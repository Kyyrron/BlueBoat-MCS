"""The pinger filter's parameter schema, read as DATA (2026-11-24).

BlueBoat-Control publishes its pinger filter parameters as one data file,
``blueboat_control/config/pinger_filter.yaml`` (``schema:
blueboat_pinger_filter/1``): groups, every parameter's type / unit / default /
range or choices / description, and the presets. The station's "Advanced…"
panel is generated from it, and the station reads it with PyYAML only -- it
never imports ``blueboat_control`` (N9).

The validation below mirrors the rules Control's ``pinger_params`` applies to
a ``configure`` (type, min / max, enum choices, ``y_sign`` / ``z_sign`` = +/-1,
``gate_hard >= gate_soft``, ``stale_timeout_s >= coast_after_s``), so a set the
form accepts is a set the estimator accepts. That is input validation of the
form, not a computation: the applied set always comes back from Control on
``/blueboat/pinger/filter_config`` and only that echo counts as applied.

Parameter files (the Advanced panel's Save / Load, and the launch's
``pinger_filter_yaml:=``) use the format Control writes and reads::

    schema: blueboat_pinger_filter/1
    preset: <preset name or custom>
    params: {<every parameter>: <value>}

(a ``hash`` key, present when Control wrote the file, is ignored on load).

Qt-free and rclpy-free, like the rest of ``mcs.core``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

SCHEMA_ID = "blueboat_pinger_filter/1"
SCHEMA_NAME = "pinger_filter.yaml"
CUSTOM = "custom"
DEFAULT_PRESET = "slow_rov"

#: Pinger modes of a run (Control's pinger_mode).
MODE_OFF, MODE_FOLLOW, MODE_LOG = "off", "follow", "log"
MODES = (MODE_OFF, MODE_FOLLOW, MODE_LOG)
#: What /blueboat/pinger_coordinates carries (Control's pinger_source).
SOURCE_FILTER, SOURCE_LEGACY = "filter", "legacy"
SOURCES = (SOURCE_FILTER, SOURCE_LEGACY)
#: The estimate's status values (blueboat_pinger_estimate/1).
STATUSES = ("no_data", "initialising", "tracking", "coasting", "stale")
#: The fix decisions of blueboat_pinger_fix/1, in the order of their map codes
#: (``decision_code``); anything else gets ``len(DECISIONS)``.
DECISIONS = ("accepted", "downweighted", "set_aside", "invalid", "duplicate",
             "initialising", "reinit")


def decision_code(decision) -> int:
    """Map code of a fix decision (the raw-fix layer colours by it)."""
    try:
        return DECISIONS.index(str(decision))
    except ValueError:
        return len(DECISIONS)


def effective_mode(pinger_mode: str, use_pinger: bool) -> str:
    """``pinger_mode`` when it is one of :data:`MODES`, else the ``use_pinger``
    alias (True = follow, False = off) -- Control's own rule."""
    mode = str(pinger_mode or "").strip().lower()
    if mode in MODES:
        return mode
    return MODE_FOLLOW if use_pinger else MODE_OFF


# ===================================================================== schema
def schema_candidates(schema_file: str = "", checkout: str = "") -> list[Path]:
    """Where the schema is looked for, in order: an explicit file, the
    installed share of ``blueboat_control`` (ament index), the checkout."""
    out: list[Path] = []
    if schema_file:
        out.append(Path(schema_file).expanduser())
    try:
        from ament_index_python.packages import get_package_share_directory
        out.append(Path(get_package_share_directory("blueboat_control"))
                   / "config" / SCHEMA_NAME)
    except Exception:  # noqa: BLE001, S110 - no sourced ROS / package not built: optional
        pass
    if checkout:
        out.append(Path(checkout).expanduser())
    return out


def find_schema(schema_file: str = "", checkout: str = "") -> Path | None:
    """The first existing candidate of :func:`schema_candidates`, or None."""
    for p in schema_candidates(schema_file, checkout):
        if p.is_file():
            return p
    return None


@dataclass
class FilterSchema:
    """The parsed ``blueboat_pinger_filter/1`` file."""

    path: Path
    groups: list[str]
    parameters: dict[str, dict]          # in file order
    presets: dict[str, dict] = field(default_factory=dict)
    #: The "Field tuning" cards (``field_tuning``): title, params, picture,
    #: what / lower / higher / field texts, optional slider ranges. Cards
    #: naming an unknown parameter are dropped (never half-shown).
    tuning: list[dict] = field(default_factory=list)

    def picture_path(self, card: dict) -> Path | None:
        """A card's picture (relative to the schema file), or None when absent."""
        rel = str(card.get("picture") or "")
        if not rel:
            return None
        p = self.path.parent / rel
        return p if p.is_file() else None

    @staticmethod
    def slider_range(card: dict, name: str) -> tuple[float, float, bool] | None:
        """(low, high, log) of a card parameter's slider, or None."""
        spec = (card.get("slider") or {}).get(name)
        try:
            lo, hi, scale = float(spec[0]), float(spec[1]), str(spec[2])
        except (TypeError, ValueError, IndexError):
            return None
        log = scale == "log"
        if not lo < hi or (log and lo <= 0.0):
            return None
        return lo, hi, log

    def group_names(self) -> list[str]:
        """The schema's groups in order, then any group a parameter names
        that the list omits (never silently dropped)."""
        out = list(self.groups)
        for spec in self.parameters.values():
            g = str(spec.get("group", "Other"))
            if g not in out:
                out.append(g)
        return out

    def names_in_group(self, group: str) -> list[str]:
        return [n for n, s in self.parameters.items()
                if str(s.get("group", "Other")) == group]

    def preset_names(self) -> list[str]:
        return list(self.presets)

    def preset_description(self, name: str) -> str:
        return str((self.presets.get(name) or {}).get("description", ""))

    def defaults(self) -> dict:
        return {n: s.get("default") for n, s in self.parameters.items()}

    def preset_values(self, name: str) -> dict:
        """Defaults overlaid with the preset's values (``custom`` or an
        unknown name: the defaults) -- Control's ``load_params`` order."""
        values = self.defaults()
        values.update((self.presets.get(name) or {}).get("values") or {})
        return values


def load_schema(path: Path) -> FilterSchema:
    """Parse *path*; ``ValueError`` naming the problem otherwise."""
    try:
        data = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"{path}: unreadable ({exc})") from None
    if not isinstance(data, dict) or data.get("schema") != SCHEMA_ID:
        raise ValueError(f"{path}: not a {SCHEMA_ID} file")
    params = data.get("parameters")
    if not isinstance(params, dict) or not params:
        raise ValueError(f"{path}: no parameters")
    for name, spec in params.items():
        if not isinstance(spec, dict) or spec.get("type") not in (
                "float", "int", "bool", "enum"):
            raise ValueError(f"{path}: parameter {name!r} has no valid type")
    presets = data.get("presets") or {}
    tuning = []
    for card in data.get("field_tuning") or []:
        if (isinstance(card, dict) and isinstance(card.get("params"), list) and card["params"]
                and all(n in params for n in card["params"])):
            tuning.append(dict(card))
    return FilterSchema(path=Path(path), groups=[str(g) for g in data.get("groups") or []],
                        parameters=dict(params),
                        presets=dict(presets) if isinstance(presets, dict) else {},
                        tuning=tuning)


#: path -> (mtime_ns, schema): the dialogs open before every run, the file
#: changes only with a rebuild. Callers never mutate a FilterSchema.
_SCHEMA_CACHE: dict[Path, tuple[int, FilterSchema]] = {}


def load_schema_or_none(schema_file: str = "", checkout: str = "") -> FilterSchema | None:
    """The schema, or None when it cannot be found or read (never raises).
    Parsed once per file version (path and modification time)."""
    path = find_schema(schema_file, checkout)
    if path is None:
        return None
    try:
        mtime = path.stat().st_mtime_ns
        cached = _SCHEMA_CACHE.get(path)
        if cached is not None and cached[0] == mtime:
            return cached[1]
        schema = load_schema(path)
    except (OSError, ValueError):
        return None
    _SCHEMA_CACHE[path] = (mtime, schema)
    return schema


# ================================================================= validation
def coerce(name: str, value, spec: dict):
    """*value* as the parameter's type, or ``ValueError`` with the reason."""
    kind = spec.get("type")
    try:
        if kind == "bool":
            if isinstance(value, bool):
                return value
            if isinstance(value, str) and value.strip().lower() in (
                    "true", "false", "1", "0", "yes", "no", "on", "off"):
                return value.strip().lower() in ("true", "1", "yes", "on")
            if isinstance(value, int | float) and value in (0, 1):
                return bool(value)
            raise ValueError(value)
        if kind == "int":
            if isinstance(value, bool):
                raise ValueError(value)
            f = float(value)
            if not math.isfinite(f) or f != int(f):
                raise ValueError(value)
            return int(f)
        if kind == "float":
            if isinstance(value, bool):
                raise ValueError(value)
            return float(value)
        if kind == "enum":
            return str(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name}: {value!r} is not a {kind}") from None
    raise ValueError(f"{name}: unknown type {kind!r} in the schema")


def check_value(name: str, value, spec: dict) -> str | None:
    """None if the (coerced) *value* is allowed, else the reason."""
    kind = spec.get("type")
    unit = str(spec.get("unit") or "")
    if kind == "enum":
        choices = [str(c) for c in spec.get("choices") or []]
        if value not in choices:
            return f"{name}: {value!r} is not one of {', '.join(choices)}"
        return None
    if kind in ("int", "float"):
        if not math.isfinite(float(value)):
            return f"{name}: {value} is not a finite number"
        lo, hi = spec.get("min"), spec.get("max")
        if lo is not None and value < lo:
            return f"{name}: {value} is below the minimum {lo} {unit}".rstrip()
        if hi is not None and value > hi:
            return f"{name}: {value} is above the maximum {hi} {unit}".rstrip()
    if name in ("y_sign", "z_sign") and value == 0:
        return f"{name}: must be +1 or -1 (0 is invalid)"
    return None


def cross_checks(values: dict) -> list[str]:
    """Reasons that involve several parameters."""
    out = []
    if ("gate_hard" in values and "gate_soft" in values
            and values["gate_hard"] < values["gate_soft"]):
        out.append("gate_hard must be >= gate_soft")
    if ("stale_timeout_s" in values and "coast_after_s" in values
            and values["stale_timeout_s"] < values["coast_after_s"]):
        out.append("stale_timeout_s must be >= coast_after_s")
    return out


def validate(values: dict, schema: FilterSchema) -> tuple[dict, list[str]]:
    """``(coerced values, problems)``; an empty problem list means the set
    is complete and valid. Unknown names are problems (a typo must never be
    silently ignored), as in Control."""
    problems: list[str] = []
    out: dict = {}
    for name in values:
        if name not in schema.parameters:
            problems.append(f"{name}: unknown parameter")
    for name, spec in schema.parameters.items():
        if name not in values:
            problems.append(f"{name}: missing")
            continue
        try:
            v = coerce(name, values[name], spec)
        except ValueError as exc:
            problems.append(str(exc))
            continue
        why = check_value(name, v, spec)
        if why:
            problems.append(why)
        out[name] = v
    if not problems:
        problems.extend(cross_checks(out))
    return out, problems


def same_params(a: dict | None, b: dict | None) -> bool:
    """Do two parameter sets hold the same names and values? Numbers
    compare by value (an int 1 equals a float 1.0 after a JSON round trip)."""
    a, b = dict(a or {}), dict(b or {})
    if set(a) != set(b):
        return False
    for k, va in a.items():
        vb = b[k]
        if isinstance(va, bool) or isinstance(vb, bool):
            if not (isinstance(va, bool) and isinstance(vb, bool)) or va != vb:
                return False
        elif isinstance(va, int | float) and isinstance(vb, int | float):
            if float(va) != float(vb):
                return False
        elif va != vb:
            return False
    return True


def changed_names(values: dict, base: dict) -> list[str]:
    """The names in *values* whose value differs from *base* (:func:`same_params`
    rules), in *values*' order."""
    return [n for n, v in values.items() if not same_params({n: v}, {n: base.get(n)})]


# ============================================================ parameter files
def read_param_file(path: Path, schema: FilterSchema,
                    base_preset: str = DEFAULT_PRESET) -> tuple[str, dict]:
    """``(preset, values)`` of a saved parameter file, validated.

    The values are the file's ``params`` (or a flat mapping) over the preset
    the file names (else *base_preset*) -- the order Control's
    ``load_params`` uses, so a partial file means the same thing on both
    sides. ``ValueError`` naming every problem otherwise; nothing is
    clamped or guessed."""
    try:
        data = yaml.safe_load(Path(path).read_text()) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"{Path(path).name}: unreadable ({exc})") from None
    if not isinstance(data, dict):
        # One error type for every unusable file: the dialog reports them alike.
        raise ValueError(f"{Path(path).name}: not a mapping")  # noqa: TRY004
    schema_tag = data.get("schema")
    if schema_tag not in (None, SCHEMA_ID):
        raise ValueError(f"{Path(path).name}: schema {schema_tag!r}, expected {SCHEMA_ID}")
    if isinstance(data.get("params"), dict):
        params = dict(data["params"])
    else:
        params = {k: v for k, v in data.items()
                  if k not in ("schema", "preset", "hash", "description")}
    preset = str(data.get("preset") or base_preset)
    base = preset if preset in schema.presets else base_preset
    values = schema.preset_values(base)
    values.update(params)
    out, problems = validate(values, schema)
    if problems:
        raise ValueError("; ".join(problems))
    return preset, out


def write_param_file(path: Path, preset: str, values: dict) -> Path:
    """Write ``{schema, preset, params}`` (the format Control reads), creating
    the directory on demand."""
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {"schema": SCHEMA_ID, "preset": str(preset or CUSTOM),
           "params": {k: values[k] for k in values}}
    path.write_text(yaml.safe_dump(doc, sort_keys=False))
    return path


def filter_block(schema: FilterSchema | None, preset: str,
                 custom: dict | None = None) -> dict:
    """The ``pinger_filter`` block of a ``configure``: ``{"preset", "params"}``
    with the full parameter set -- the preset's values, or *custom* values
    (then named ``custom``). Without a schema the params are empty and
    Control fills them from the preset."""
    if custom is not None:
        return {"preset": CUSTOM, "params": dict(custom)}
    if schema is None:
        return {"preset": preset, "params": {}}
    return {"preset": preset, "params": schema.preset_values(preset)}
