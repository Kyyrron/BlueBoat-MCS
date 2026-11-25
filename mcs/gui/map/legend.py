"""PINGER legend — a small box floating at the map's bottom-left (2026-11-24).

The map's first legend: the pinger layers are told apart by colour and
style only (filtered purple, legacy orange, raw fixes by the filter's
decision), so each pinger layer that is switched on gets one line here --
a layer switched off in LAYERS drops its line. The box itself is a layer
too (``pinger_legend``), and shows only while the map is anchored and some
pinger data exists (MapView.refresh decides).
"""

from __future__ import annotations

from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget

from mcs.gui import theme

#: (layer key, swatch, text) in display order. A layer may own several lines.
_LINES: list[tuple[str, str, str]] = [
    ("pinger_filtered", f"<span style='color:{theme.C_PINGER_FILTERED.name()}'>●</span>",
     "pinger, filtered"),
    ("pinger_filtered_track",
     f"<span style='color:{theme.C_PINGER_FILTERED_TRACK.name()}'>━━</span>",
     "filtered track"),
    ("pinger_ellipse", f"<span style='color:{theme.C_PINGER_ELLIPSE.name()}'>◯</span>",
     "95 % uncertainty"),
    ("pinger_fixes", f"<span style='color:{theme.C_FIX_ACCEPTED.name()}'>•</span>",
     "fix accepted"),
    ("pinger_fixes", f"<span style='color:{theme.C_FIX_DOWNWEIGHTED.name()}'>•</span>",
     "fix down-weighted"),
    ("pinger_fixes", f"<span style='color:{theme.C_FIX_SET_ASIDE.name()}'>•</span>",
     "fix set aside"),
    ("pinger_fixes", f"<span style='color:{theme.C_FIX_OTHER.name()}'>•</span>",
     "fix invalid / other"),
    ("pinger", f"<span style='color:{theme.C_PINGER.name()}'>●</span>",
     "pinger, legacy"),
    ("pinger_track", f"<span style='color:{theme.C_PINGER_TRACK.name()}'>┅┅</span>",
     "legacy track"),
]


class PingerLegend(QWidget):
    """Legend of the pinger layers; one line per enabled layer."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("pingerLegend")
        self.setStyleSheet(theme.floating_box_style("pingerLegend")
                           + " QLabel { background: transparent; font-size: 10px; }")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(0)
        title = QLabel("PINGER")
        title.setStyleSheet(f"color: {theme.TEXT_DIM}; font-weight: bold;"
                            " font-size: 10px; background: transparent;")
        layout.addWidget(title)
        self.lines: list[tuple[str, QLabel]] = []
        self._last: tuple | None = None
        for key, swatch, text in _LINES:
            label = QLabel(f"{swatch}&nbsp;{text}")
            layout.addWidget(label)
            self.lines.append((key, label))
        self.hide()

    def set_layers(self, enabled: dict[str, bool]) -> bool:
        """Show the line of every enabled layer; True if any line shows."""
        state = tuple(sorted(enabled.items()))
        if state == self._last:
            return any(enabled.get(k, False) for k, _label in self.lines)
        self._last = state
        any_on = False
        for key, label in self.lines:
            on = bool(enabled.get(key, False))
            label.setVisible(on)
            any_on = any_on or on
        self.adjustSize()
        return any_on

    def visible_texts(self) -> list[str]:
        """Texts of the lines currently shown (for tests)."""
        return [label.text().split("&nbsp;", 1)[-1] for _key, label in self.lines
                if not label.isHidden()]
