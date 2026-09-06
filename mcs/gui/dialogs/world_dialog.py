"""World-choice dialog for GPS-anchored simulation launches.

Shown by the bottom toolbar AFTER the launch dialog, only for a simulated
GPS-anchored mission with at least one eligible personalized world (a
generated Gazebo world folder whose limits contain a point of the path —
see :mod:`mcs.core.worlds`). The operator picks between the standard empty
Gazebo world (today's ``Sim_launch.py`` behavior) and launching inside one
of the listed worlds via ``blueboat_sss_sim full_mission_launch.py``.

The dialog does no I/O of its own: it renders the pre-filtered list it is
given, so the smoke test can drive it headless.
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QVBoxLayout,
)


class WorldChoiceDialog(QDialog):
    """Pick a personalized Gazebo world for this launch, or the empty one.

    Result: ``selected_world_dir`` — the chosen world folder as a string,
    or ``None`` for the empty Gazebo world. Rejecting (Cancel/Esc) aborts
    the launch entirely.
    """

    def __init__(self, worlds: list[dict], parent=None) -> None:
        super().__init__(parent)
        self.selected_world_dir: str | None = None
        self.setWindowTitle("Launch Mission — Gazebo world")
        self.setMinimumSize(460, 320)
        layout = QVBoxLayout(self)

        intro = QLabel(
            f"This GPS-anchored path lies inside {len(worlds)} personalized "
            "world(s). Launch inside one of them (simulated seabed, sonar "
            "and GPS — blueboat_sss_sim), or in the standard empty Gazebo "
            "world. In a personalized world the boat spawns at the world's "
            "own origin heading east; the controller then drives it onto "
            "the path.")
        intro.setWordWrap(True)
        layout.addWidget(intro)

        self._list = QListWidget()
        for w in worlds:
            meta = w["meta"]
            n_obj = len(meta.get("objects") or [])
            created = str(meta.get("created", ""))[:10]
            item = QListWidgetItem(
                f"{w['path_name']} / {w['world_name']} — "
                f"{n_obj} objects · {created}")
            item.setData(Qt.ItemDataRole.UserRole, str(w["dir"]))
            anchor = meta.get("geo_anchor", {})
            lim = (meta.get("limits") or {}).get("local", {})
            item.setToolTip(
                f"{w['dir']}\n"
                f"anchor: {anchor.get('lat0', 0.0):.6f}, "
                f"{anchor.get('lon0', 0.0):.6f}\n"
                f"limits: x [{lim.get('x_min', 0.0):.0f}, "
                f"{lim.get('x_max', 0.0):.0f}] m, "
                f"y [{lim.get('y_min', 0.0):.0f}, "
                f"{lim.get('y_max', 0.0):.0f}] m")
            self._list.addItem(item)
        if self._list.count():
            self._list.setCurrentRow(0)
        self._list.itemDoubleClicked.connect(lambda _: self._accept_world())
        layout.addWidget(self._list)

        buttons = QDialogButtonBox()
        empty_btn = buttons.addButton(
            "Empty Gazebo", QDialogButtonBox.ButtonRole.AcceptRole)
        world_btn = buttons.addButton(
            "Launch in World", QDialogButtonBox.ButtonRole.AcceptRole)
        buttons.addButton(QDialogButtonBox.StandardButton.Cancel)
        empty_btn.clicked.connect(self._accept_empty)
        world_btn.clicked.connect(self._accept_world)
        world_btn.setDefault(True)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _accept_empty(self) -> None:
        self.selected_world_dir = None
        self.accept()

    def _accept_world(self) -> None:
        item = self._list.currentItem()
        if item is None:
            return
        self.selected_world_dir = item.data(Qt.ItemDataRole.UserRole)
        self.accept()
