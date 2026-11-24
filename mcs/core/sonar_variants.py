"""Ray-cast sonar variants for the launch dialog's **Sonar model** menu
(owner order, 2026-11-09).

BlueBoat-SSS-Sim ships, beside its full ray-cast profile
(``config/raycast_default_sonar.yaml`` -- the dialog's plain "Ray-cast"
entry), generated *live variants*: the same model with only parts switched
off that cost GPU time without changing how the image looks (first-order
multipath; no shadow-edge diffraction). Each is a profile YAML whose
top-level ``variant:`` block (``label``, ``order``, ``description``) is the
whole contract with this station:

    renderer: raycast
    variant: {label: "Ray-cast — live (...)", order: 10, description: "..."}

The dialog lists every such file and launches the chosen one with
``sonar_config:=<absolute path>``; a new variant in the simulator appears
here without any station edit.

Deliberately a local YAML reader, never an import of the simulator (project
rule CM-3, as :mod:`mcs.core.worlds`). Search order: the extra folders of
``SimSpeedConfig.variant_dirs``, then the installed ``blueboat_sss_sim``
share (what ``ros2 launch`` runs), then the superproject checkout -- the
first folder that has a file name wins. An unreadable or malformed file is
skipped, never an exception: a broken profile must not break the launch
dialog. Qt-free, rclpy-free.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

#: The simulator package whose ``config/`` holds the profiles.
SIM_PACKAGE = "blueboat_sss_sim"


@dataclass(frozen=True)
class SonarVariant:
    """One ray-cast variant profile."""

    path: str            # absolute path, passed as sonar_config:=
    label: str
    description: str
    order: int

    @property
    def file(self) -> str:
        return Path(self.path).name


def default_variant_dirs(extra: list[str] | tuple[str, ...] = ()) -> list[Path]:
    """Where variant profiles are looked for, in priority order."""
    dirs = [Path(d).expanduser() for d in extra if str(d).strip()]
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(Path(get_package_share_directory(SIM_PACKAGE)) / "config")
    except Exception:  # noqa: BLE001 -- no sourced workspace: the checkout only
        pass
    # <superproject>/BlueBoat-MCS/mcs/core/sonar_variants.py
    dirs.append(Path(__file__).resolve().parents[3] / "BlueBoat-SSS-Sim"
                / SIM_PACKAGE / "config")
    return dirs


def read_variant(path: Path) -> SonarVariant | None:
    """The variant a profile file declares, or None (not a ray-cast
    variant, unreadable or malformed)."""
    try:
        doc = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError, UnicodeDecodeError):
        return None
    if not isinstance(doc, dict) or str(doc.get("renderer", "")) != "raycast":
        return None
    v = doc.get("variant")
    if not isinstance(v, dict) or not str(v.get("label", "")).strip():
        return None
    try:
        order = int(v.get("order", 100))
    except (TypeError, ValueError):
        order = 100
    return SonarVariant(path=str(Path(path).resolve()), label=str(v["label"]).strip(),
                        description=str(v.get("description", "")).strip(), order=order)


def list_variants(dirs: list[Path] | None = None) -> list[SonarVariant]:
    """Every ray-cast variant profile (``*_sonar.yaml`` with a ``variant:``
    block) in ``dirs``, the first folder winning per file name, sorted by
    ``order`` then label."""
    found: dict[str, SonarVariant] = {}
    for d in (dirs if dirs is not None else default_variant_dirs()):
        try:
            files = sorted(Path(d).glob("*_sonar.yaml"))
        except OSError:
            continue
        for f in files:
            if f.name in found:
                continue
            v = read_variant(f)
            if v is not None:
                found[f.name] = v
    return sorted(found.values(), key=lambda v: (v.order, v.label))
