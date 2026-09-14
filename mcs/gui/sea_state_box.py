"""Floating box for the simulated sea state.

Mirror of :class:`~mcs.gui.mission_stats.FloatingStatsBox`: same look, same
map-view parenting, glued to the **top-left** of the map (i.e. right against
the left panel) instead of the top-right.

The box exists only for a running simulation — `/sim/sea_state` is published
by the simulator's ``sea_state_node`` and nothing else — and hides itself
whenever no simulated mission is running.

It does not reuse :class:`~mcs.gui.widgets.InfoGrid`: the sea strings are
long sentences, and a floating overlay cannot widen itself across the map to
fit them. The values wrap inside a fixed width instead, with each row
measuring its own wrapped height so the box grows downwards exactly as far
as the text needs.
"""

from __future__ import annotations

import time

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QGridLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from mcs.gui import theme
from mcs.gui.widgets import CollapsibleSection
from mcs.models.store import DataStore

_BOX_W = 300
_KEY_W = 84
_VALUE_W = _BOX_W - _KEY_W - 34  # box margins + section padding + spacing


class _WrappedValue(QLabel):
    """Right-aligned monospace value that wraps at a fixed width.

    Qt only asks a widget for a width-dependent height when every layout
    above it advertises one, which is not the case inside the collapsible
    section. The label therefore measures the wrapped text itself and pins
    its own height, so the plain ``sizeHint`` chain above it is exact.
    """

    _FLAGS = Qt.TextFlag.TextWordWrap | Qt.AlignmentFlag.AlignRight

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("—", parent)
        self.setObjectName("valueLabel")
        self.setFont(QFont("DejaVu Sans Mono"))
        self.setWordWrap(True)
        self.setFixedWidth(_VALUE_W)
        self.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTop)
        self.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self._remeasure("—")

    def setText(self, text: str) -> None:
        super().setText(text)
        self._remeasure(text)

    def _remeasure(self, text: str) -> None:
        fm = self.fontMetrics()
        rect = fm.boundingRect(0, 0, _VALUE_W, 10_000, self._FLAGS, text)
        self.setFixedHeight(max(fm.height(), rect.height()))


class FloatingSeaBox(QWidget):
    """Floating widget that displays and commands the simulated sea state."""

    modify_sea_clicked = Signal()

    _ROWS = ("Current", "Waves", "Surface", "Next change", "Status")

    def __init__(self, store: DataStore, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._store = store

        self.setObjectName("floatingSeaBox")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet("""
            QWidget#floatingSeaBox {
                background-color: rgba(45, 45, 45, 230);
                border: 1px solid #555;
                border-radius: 6px;
            }
        """)
        self.setFixedWidth(_BOX_W)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        sec_sea = CollapsibleSection("SEA STATE (sim)")
        grid_host = QWidget()
        grid = self._grid = QGridLayout(grid_host)
        grid.setContentsMargins(2, 2, 2, 2)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(4)
        self._values: dict[str, _WrappedValue] = {}
        for row, key in enumerate(self._ROWS):
            klabel = QLabel(key)
            klabel.setStyleSheet(f"color: {theme.TEXT_DIM};")
            klabel.setFixedWidth(_KEY_W)
            klabel.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
            value = _WrappedValue()
            grid.addWidget(klabel, row, 0)
            grid.addWidget(value, row, 1)
            self._values[key] = value
        sec_sea.add_widget(grid_host)

        self.modify_sea_btn = QPushButton("Modify situation…")
        self.modify_sea_btn.setToolTip(
            "Change the current / waves of the running simulation; the "
            "simulator ramps to the new situation.")
        self.modify_sea_btn.clicked.connect(self.modify_sea_clicked.emit)
        sec_sea.add_widget(self.modify_sea_btn)
        layout.addWidget(sec_sea)

        self.setVisible(False)

    # -------------------------------------------------------------------- set
    def _set(self, key: str, value: str, color: str | None = None) -> None:
        label = self._values[key]
        if label.text() != value:
            label.setText(value)
        style = f"color: {color};" if color else ""
        if label.styleSheet() != style:
            label.setStyleSheet(style)

    def _fit(self) -> None:
        """Re-fit the box to the text just set.

        Nothing else can: the box floats over the map view, which has no
        layout of its own, so its height has to follow the wrapped rows by
        hand whenever a string changes length.
        """
        self._grid.invalidate()
        self.layout().invalidate()
        self.adjustSize()

    # ---------------------------------------------------------------- refresh
    def refresh(self) -> None:
        """Pulled on the shared UI tick; hides the box outside a sim run."""
        s = self._store
        show = bool(s.mission.simulation and s.mission.launch_running)
        # setVisible, not `if isVisible() != show`: isVisible() is also false
        # whenever an ancestor is hidden, which would leave the box's own flag
        # stuck on from an earlier run.
        self.setVisible(show)
        if not show:
            return
        r = s.sea
        if r is None:
            choice = s.mission.sea_choice
            wanted = choice.summary() if choice is not None else "calm water, no current"
            self._set("Current", "—")
            self._set("Waves", "—")
            self._set("Surface", "—")
            self._set("Next change", "—")
            self._set("Status", f"waiting for the simulator ({wanted})",
                      theme.TEXT_DIM)
            self._fit()
            return
        age = time.monotonic() - r.received_mono if r.received_mono else 0.0
        self._set("Current", r.current_text)
        self._set("Waves", r.waves_text,
                  theme.WARN if r.waves_status == "unavailable" else None)
        self._set("Surface", f"η {r.eta_m:+.2f} m")
        self._set("Next change", r.next_change_text)
        if age > 5.0:
            self._set("Status", f"stale ({age:.0f} s)", theme.WARN)
        else:
            self._set("Status", r.summary or "live", theme.OK)
        self._fit()
