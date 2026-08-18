"""Mission Pattern Designer — data model.

Qt-free except for the two change signals. A :class:`MissionModel` is an
ordered list of top-level items, each either a :class:`Waypoint` or a
:class:`PatternGroup` (a named, lockable container of waypoints produced by
a pattern generator or by grouping a selection). The *flattened* waypoint
sequence defines the mission order; every waypoint carries the interpolation
specification of the segment **leaving** it (``seg_out``), so reordering
keeps segment settings attached to their origin.

Undo/redo is snapshot-based: the window captures :meth:`to_dict` before each
mutation; :meth:`from_dict` restores. Cheap, uniform, and immune to
forgotten inverse operations.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from typing import Iterator, Union

from PySide6.QtCore import QObject, Signal


@dataclass
class SegmentSpec:
    """Interpolation + speed of the segment from this waypoint to the next.

    ``speed`` in m/s; ``0.0`` means "use the mission cruise speed".
    """

    kind: str = "straight"
    params: dict = field(default_factory=dict)
    speed: float = 0.0

    def to_dict(self) -> dict:
        return {"kind": self.kind, "params": dict(self.params),
                "speed": self.speed}

    @classmethod
    def from_dict(cls, d: dict) -> "SegmentSpec":
        return cls(kind=d.get("kind", "straight"),
                   params=dict(d.get("params", {})),
                   speed=float(d.get("speed", 0.0)))


@dataclass
class Waypoint:
    uid: int
    name: str
    x: float
    y: float
    locked: bool = False
    seg_out: SegmentSpec = field(default_factory=SegmentSpec)

    def to_dict(self) -> dict:
        return {"type": "waypoint", "uid": self.uid, "name": self.name,
                "x": self.x, "y": self.y, "locked": self.locked,
                "seg_out": self.seg_out.to_dict()}

    @classmethod
    def from_dict(cls, d: dict) -> "Waypoint":
        return cls(uid=d["uid"], name=d.get("name", ""), x=float(d["x"]),
                   y=float(d["y"]), locked=bool(d.get("locked", False)),
                   seg_out=SegmentSpec.from_dict(d.get("seg_out", {})))


@dataclass
class PatternGroup:
    uid: int
    name: str
    pattern: str                 # generator key, or "group" for a manual group
    params: dict = field(default_factory=dict)
    children: list[Waypoint] = field(default_factory=list)
    locked: bool = False

    def to_dict(self) -> dict:
        return {"type": "group", "uid": self.uid, "name": self.name,
                "pattern": self.pattern, "params": dict(self.params),
                "locked": self.locked,
                "children": [w.to_dict() for w in self.children]}

    @classmethod
    def from_dict(cls, d: dict) -> "PatternGroup":
        return cls(uid=d["uid"], name=d.get("name", ""),
                   pattern=d.get("pattern", "group"),
                   params=dict(d.get("params", {})),
                   locked=bool(d.get("locked", False)),
                   children=[Waypoint.from_dict(c) for c in d.get("children", [])])


MissionItem = Union[Waypoint, PatternGroup]


class MissionModel(QObject):
    """The mission being edited."""

    #: geometry / parameters changed (preview must resample)
    changed = Signal()
    #: items added / removed / reordered / regrouped (tree + scene rebuild)
    structure_changed = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.name: str = ""
        self.comment: str = ""
        self.speed: float = 0.5
        self.loop: bool = False
        self.items: list[MissionItem] = []
        self._uid = itertools.count(1)

    # ------------------------------------------------------------- queries
    def flatten(self) -> list[Waypoint]:
        out: list[Waypoint] = []
        for item in self.items:
            if isinstance(item, PatternGroup):
                out.extend(item.children)
            else:
                out.append(item)
        return out

    def iter_all(self) -> Iterator[MissionItem]:
        for item in self.items:
            yield item
            if isinstance(item, PatternGroup):
                yield from item.children

    def waypoint(self, uid: int) -> Waypoint | None:
        for item in self.iter_all():
            if isinstance(item, Waypoint) and item.uid == uid:
                return item
        return None

    def item(self, uid: int) -> MissionItem | None:
        for it in self.iter_all():
            if it.uid == uid:
                return it
        return None

    def container_of(self, uid: int) -> PatternGroup | None:
        for item in self.items:
            if isinstance(item, PatternGroup):
                if any(w.uid == uid for w in item.children):
                    return item
        return None

    def effective_locked(self, wp: Waypoint) -> bool:
        parent = self.container_of(wp.uid)
        return wp.locked or (parent.locked if parent else False)

    def next_uid(self) -> int:
        return next(self._uid)

    def next_wp_name(self) -> str:
        return f"WP{len(self.flatten()) + 1}"

    # ------------------------------------------------------------- mutation
    def add_waypoint(self, x: float, y: float, name: str = "") -> Waypoint:
        wp = Waypoint(uid=self.next_uid(), name=name or self.next_wp_name(),
                      x=x, y=y)
        self.items.append(wp)
        self.structure_changed.emit()
        self.changed.emit()
        return wp

    def add_group(self, name: str, pattern: str, params: dict,
                  points: list[tuple[float, float]]) -> PatternGroup:
        group = PatternGroup(uid=self.next_uid(), name=name, pattern=pattern,
                             params=dict(params))
        for i, (x, y) in enumerate(points, start=1):
            group.children.append(Waypoint(
                uid=self.next_uid(), name=f"{name}.{i}", x=x, y=y))
        self.items.append(group)
        self.structure_changed.emit()
        self.changed.emit()
        return group

    def regenerate_group(self, uid: int, params: dict,
                         points: list[tuple[float, float]]) -> None:
        group = self.item(uid)
        if not isinstance(group, PatternGroup):
            return
        group.params = dict(params)
        group.children = [Waypoint(uid=self.next_uid(),
                                   name=f"{group.name}.{i}", x=x, y=y)
                          for i, (x, y) in enumerate(points, start=1)]
        self.structure_changed.emit()
        self.changed.emit()

    def remove(self, uids: set[int]) -> None:
        """Remove top-level items and group children whose uid is selected."""
        self.items = [it for it in self.items if it.uid not in uids]
        for item in self.items:
            if isinstance(item, PatternGroup):
                item.children = [w for w in item.children if w.uid not in uids]
        self.items = [it for it in self.items
                      if not (isinstance(it, PatternGroup) and not it.children)]
        self.structure_changed.emit()
        self.changed.emit()

    def move_item(self, uid: int, delta: int) -> None:
        """Reorder: move a top-level item, or a waypoint inside its group."""
        parent = self.container_of(uid)
        seq: list = parent.children if parent else self.items
        idx = next((i for i, it in enumerate(seq) if it.uid == uid), None)
        if idx is None:
            if parent is None:  # uid may be a grouped wp reached w/o parent
                return
            return
        j = idx + delta
        if 0 <= j < len(seq):
            seq[idx], seq[j] = seq[j], seq[idx]
            self.structure_changed.emit()
            self.changed.emit()

    def translate(self, uids: set[int], dx: float, dy: float) -> None:
        """Move waypoints / whole groups, honouring locks."""
        for item in self.items:
            if item.uid in uids and isinstance(item, PatternGroup):
                if item.locked:
                    continue
                for w in item.children:
                    w.x += dx
                    w.y += dy
        for wp in self.flatten():
            if wp.uid in uids and not self.effective_locked(wp):
                wp.x += dx
                wp.y += dy
        self.changed.emit()

    def duplicate(self, uids: set[int], offset: float = 2.0) -> list[int]:
        """Duplicate selected items with an offset; returns the new uids.

        Pattern-aware semantics (the map selects *child* uids while the tree
        may select the *group* uid, so both must work and must not combine
        into double duplication):

        * a group is duplicated **as a group** when its own uid is selected
          OR all of its children are (the map's way of selecting a pattern);
        * grouped children selected partially are duplicated as free
          top-level waypoints;
        * top-level waypoints duplicate as before.
        """
        uids = set(uids)
        new_uids: list[int] = []
        handled_children: set[int] = set()
        originals = list(self.items)
        original_flat = self.flatten()

        def copy_wp(w: Waypoint, name_suffix: str = " copy") -> Waypoint:
            return Waypoint(uid=self.next_uid(), name=w.name + name_suffix,
                            x=w.x + offset, y=w.y + offset,
                            seg_out=SegmentSpec.from_dict(w.seg_out.to_dict()))

        for item in originals:
            if isinstance(item, PatternGroup):
                child_uids = {w.uid for w in item.children}
                if item.uid in uids or (child_uids and child_uids <= uids):
                    copy = PatternGroup(uid=self.next_uid(),
                                        name=item.name + " copy",
                                        pattern=item.pattern,
                                        params=dict(item.params))
                    copy.children = [copy_wp(w, "") for w in item.children]
                    self.items.append(copy)
                    new_uids.append(copy.uid)
                    handled_children |= child_uids
            elif item.uid in uids:
                wp = copy_wp(item)
                self.items.append(wp)
                new_uids.append(wp.uid)

        # Partially-selected grouped children → duplicated as free waypoints.
        for wp in original_flat:
            if wp.uid in uids and wp.uid not in handled_children \
                    and self.container_of(wp.uid) is not None:
                copy = copy_wp(wp)
                self.items.append(copy)
                new_uids.append(copy.uid)

        if new_uids:
            self.structure_changed.emit()
            self.changed.emit()
        return new_uids

    def group_selection(self, uids: set[int], name: str = "Group") -> int | None:
        """Group selected *top-level* waypoints into a manual group."""
        selected = [it for it in self.items
                    if it.uid in uids and isinstance(it, Waypoint)]
        if len(selected) < 2:
            return None
        first_idx = next(i for i, it in enumerate(self.items)
                         if it.uid == selected[0].uid)
        group = PatternGroup(uid=self.next_uid(), name=name, pattern="group",
                             children=selected)
        self.items = [it for it in self.items if it not in selected]
        self.items.insert(min(first_idx, len(self.items)), group)
        self.structure_changed.emit()
        self.changed.emit()
        return group.uid

    def ungroup(self, uid: int) -> None:
        """Explode a group into individual waypoints at its position."""
        for i, item in enumerate(self.items):
            if item.uid == uid and isinstance(item, PatternGroup):
                self.items[i:i + 1] = item.children
                self.structure_changed.emit()
                self.changed.emit()
                return

    def set_locked(self, uids: set[int], locked: bool) -> None:
        for item in self.iter_all():
            if item.uid in uids:
                item.locked = locked
        self.changed.emit()

    def rename(self, uid: int, name: str) -> None:
        item = self.item(uid)
        if item is not None and name:
            item.name = name
            self.structure_changed.emit()

    # ------------------------------------------------ alignment / spacing
    def align(self, uids: set[int], axis: str) -> None:
        """Align selected waypoints on 'x' (same x) or 'y' (same y)."""
        wps = [w for w in self.flatten()
               if w.uid in uids and not self.effective_locked(w)]
        if len(wps) < 2:
            return
        ref = wps[0]
        for w in wps[1:]:
            if axis == "x":
                w.x = ref.x
            else:
                w.y = ref.y
        self.changed.emit()

    def distribute(self, uids: set[int]) -> None:
        """Space selected waypoints equally along the first→last line
        (mission order preserved)."""
        wps = [w for w in self.flatten()
               if w.uid in uids and not self.effective_locked(w)]
        if len(wps) < 3:
            return
        a, b = wps[0], wps[-1]
        n = len(wps) - 1
        for i, w in enumerate(wps):
            u = i / n
            w.x = a.x + u * (b.x - a.x)
            w.y = a.y + u * (b.y - a.y)
        self.changed.emit()

    def align_to_start(self, origin: tuple[float, float],
                       angle: float) -> None:
        """Rigid-transform the WHOLE mission so that *origin* (the first
        sampled point) lands on world (0, 0) and the initial tangent
        *angle* lands on +x.

        Rationale: the robot's world frame is zeroed at launch (origin =
        boat position, +x = boat heading), so an aligned mission always
        starts at the boat and begins by moving forward — identically in
        simulation and on the real robot. Locked waypoints are transformed
        too: locks protect against editing mistakes, and a whole-mission
        rigid transform is not one. Segment interpolation parameters are
        chord-relative and therefore invariant under this transform."""
        import math as _math
        c, s = _math.cos(-angle), _math.sin(-angle)
        ox, oy = origin
        for wp in self.flatten():
            dx, dy = wp.x - ox, wp.y - oy
            wp.x = c * dx - s * dy
            wp.y = s * dx + c * dy
        self.changed.emit()

    # --------------------------------------------------------- serialization
    def to_dict(self) -> dict:
        return {"name": self.name, "comment": self.comment,
                "speed": self.speed, "loop": self.loop,
                "items": [it.to_dict() for it in self.items]}

    def from_dict(self, d: dict) -> None:
        self.name = d.get("name", "")
        self.comment = d.get("comment", "")
        self.speed = float(d.get("speed", 0.5))
        self.loop = bool(d.get("loop", False))
        self.items = []
        for it in d.get("items", []):
            if it.get("type") == "group":
                self.items.append(PatternGroup.from_dict(it))
            else:
                self.items.append(Waypoint.from_dict(it))
        max_uid = max((it.uid for it in self.iter_all()), default=0)
        self._uid = itertools.count(max_uid + 1)
        self.structure_changed.emit()
        self.changed.emit()
