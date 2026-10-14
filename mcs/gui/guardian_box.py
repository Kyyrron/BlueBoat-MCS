"""GUARDIAN box — top of the left column (v1.1, 2026-10-14, by the user's order).

The link guardian brings the boat home on its own when this laptop goes silent
(WiFi lost, laptop switched off): an ArduPilot Lua script on the autopilot,
switched ON for every real-robot session by its companion ``guardian.py watch``,
which the station starts with the launch. Everything about it is operated from
here once the on-boat part is installed (superproject ``link_guardian/README.md``).

Read-out (from the companion's ``/link_guardian/status`` and the station's own
``/mavros/state``, through :func:`mcs.core.guardian.summarize`):

* a one-line **state** (ON / OFF / NOT READY / TAKEN OVER / LINK LOST …);
* **Link**, **Autopilot** (armed · mode), **Guardian** (the flag the autopilot
  confirmed), **Setup** (the readiness check), **HOME** (and whether it is the
  session start or the current run's start), the last **boat messages**.

Buttons — each enabled only when it can act, its tooltip saying why not:

* **Check** — re-run the readiness check;
* **Apply settings** — write the recommended ArduPilot failsafe parameters;
* **Turn OFF / Turn ON** — the session flag (ON by default);
* **Arm again** — ``rearm``: robot_interface re-arms the autopilot (needed after
  an E-STOP or a failed arm at connection; the script arms by itself anyway);
* **Take back control** — after a guardian return: HOLD, thrusters back to the
  station, MANUAL.

Greyed outside a real-robot session (Gazebo runs and bag replays have no
guardian).
"""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QGridLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from mcs.core.guardian import Action, GuardianReadout, GuardianView
from mcs.gui import theme
from mcs.gui.widgets import CollapsibleSection, InfoGrid

_COLORS = {"ok": theme.OK, "warn": theme.WARN, "err": theme.ERR,
           "info": theme.ACCENT, "off": theme.TEXT_DIM}
_MAX_CHARS = 34


class GuardianBox(CollapsibleSection):
    """The link guardian's read-out and controls."""

    #: 'check' | 'setup' | 'on' | 'off' | 'rearm' | 'reclaim'
    action_requested = Signal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("GUARDIAN", parent)
        body = QWidget()
        layout = QVBoxLayout(body)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self.state_label = QLabel("OFF — real-robot sessions only")
        self.state_label.setWordWrap(True)
        self.state_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.state_label)

        self.grid = InfoGrid()
        for key in ("Link", "Autopilot", "Guardian", "Setup", "HOME"):
            self.grid.add_row(key)
        layout.addWidget(self.grid)

        self.messages_label = QLabel("")
        self.messages_label.setWordWrap(True)
        self.messages_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; font-size: 10px;")
        layout.addWidget(self.messages_label)

        buttons = QGridLayout()
        buttons.setHorizontalSpacing(4)
        buttons.setVerticalSpacing(4)
        self.check_button = QPushButton("Check")
        self.setup_button = QPushButton("Apply settings")
        self.toggle_button = QPushButton("Turn OFF")
        self.rearm_button = QPushButton("Arm again")
        self.reclaim_button = QPushButton("Take back control")
        self.check_button.clicked.connect(lambda: self.action_requested.emit("check"))
        self.setup_button.clicked.connect(self._on_setup)
        self.toggle_button.clicked.connect(self._on_toggle)
        self.rearm_button.clicked.connect(lambda: self.action_requested.emit("rearm"))
        self.reclaim_button.clicked.connect(self._on_reclaim)
        buttons.addWidget(self.check_button, 0, 0)
        buttons.addWidget(self.setup_button, 0, 1)
        buttons.addWidget(self.toggle_button, 1, 0)
        buttons.addWidget(self.rearm_button, 1, 1)
        buttons.addWidget(self.reclaim_button, 2, 0, 1, 2)
        wrap = QWidget()
        wrap.setLayout(buttons)
        layout.addWidget(wrap)
        self.add_widget(body)

        self._toggle_on = False
        self._readout: GuardianReadout | None = None
        self.view: GuardianView | None = None

    # ---------------------------------------------------------------- actions
    def _on_toggle(self) -> None:
        if self._toggle_on:
            self.action_requested.emit("on")
            return
        answer = QMessageBox.warning(
            self, "Turn the guardian OFF?",
            "With the guardian OFF the boat will NOT come back by itself if the "
            "link to this laptop is lost — it stops and drifts.\n\nTurn it OFF?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No)
        if answer == QMessageBox.StandardButton.Yes:
            self.action_requested.emit("off")

    def _on_setup(self) -> None:
        answer = QMessageBox.question(
            self, "Apply ArduPilot settings",
            "Write the recommended failsafe parameters on the autopilot "
            "(SCR_ENABLE=1, FS_GCS_ENABLE=1, FS_ACTION=1 RTL, FS_GCS_TIMEOUT=10, "
            "FS_THR_ENABLE=0)?\n\nIf scripting was off, restart the autopilot "
            "afterwards (BlueOS > Autopilot Firmware > Restart).",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No)
        if answer == QMessageBox.StandardButton.Yes:
            self.action_requested.emit("setup")

    def _on_reclaim(self) -> None:
        answer = QMessageBox.question(
            self, "Take back control",
            "The boat stops (HOLD), the thrusters return to the station's "
            "mapping, then MANUAL: the station drives again (it stays armed).\n\n"
            "Take back control now?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.Yes)
        if answer == QMessageBox.StandardButton.Yes:
            self.action_requested.emit("reclaim")

    # ---------------------------------------------------------------- refresh
    def _set(self, key: str, value: tuple[str, str]) -> None:
        text, level = value
        shown = text if len(text) <= _MAX_CHARS else text[:_MAX_CHARS - 1] + "…"
        self.grid.set(key, shown, _COLORS.get(level))
        label = self.grid._values.get(key)
        if label is not None and label.toolTip() != text:
            label.setToolTip(text)

    @staticmethod
    def _apply(button: QPushButton, action: Action, tip: str) -> None:
        button.setEnabled(action.enabled)
        button.setToolTip(tip if action.enabled else f"Not available: {action.reason}")

    def update_view(self, view: GuardianView, readout: GuardianReadout | None) -> None:
        self.view = view
        self._readout = readout
        self.state_label.setText(view.state)
        self.state_label.setStyleSheet(
            f"font-weight: bold; color: {_COLORS.get(view.level, theme.TEXT)};")
        self._set("Link", view.link)
        self._set("Autopilot", view.autopilot)
        self._set("Guardian", view.flag)
        self._set("Setup", view.setup)
        self._set("HOME", view.home)
        if readout is not None and readout.check is not None:
            details = "\n".join(f"{i.get('name')}: {i.get('text')}"
                                for i in readout.check_items())
            label = self.grid._values.get("Setup")
            if label is not None:
                label.setToolTip(details)
        self.messages_label.setText("\n".join(view.messages))
        self.messages_label.setVisible(bool(view.messages))

        self._toggle_on = view.toggle_on
        self.toggle_button.setText("Turn ON" if view.toggle_on else "Turn OFF")
        self._apply(self.check_button, view.check,
                    "Re-check the guardian on the autopilot (script, failsafe "
                    "parameters, laptop identity).")
        self._apply(self.setup_button, view.setup_action,
                    "Write the recommended ArduPilot failsafe parameters.")
        self._apply(self.toggle_button, view.toggle,
                    "Switch the guardian ON: the boat returns HOME on its own if "
                    "this laptop goes silent." if view.toggle_on else
                    "Switch the guardian OFF for this session (it is ON by default).")
        self._apply(self.rearm_button, view.rearm,
                    "Arm the autopilot again (the boat briefly leaves override). "
                    "Needed after an E-STOP; the guardian also arms by itself.")
        self._apply(self.reclaim_button, view.reclaim,
                    "After a guardian return: HOLD, thrusters back to the station, "
                    "MANUAL.")


def ask_turn_off_choice(parent: QWidget, exiting: bool) -> str:
    """Turn Off Connection / exit while the guardian may be ON.

    Returns ``'off'`` (turn the guardian off and disarm, then turn off),
    ``'keep'`` (turn off anyway: the boat returns HOME by itself) or
    ``'cancel'``.
    """
    what = "quit the station" if exiting else "turn off the connection"
    box = QMessageBox(parent)
    box.setIcon(QMessageBox.Icon.Warning)
    box.setWindowTitle("Link guardian is ON")
    box.setText(f"The link guardian is ON. If you {what} now, the boat will see "
                "this laptop go silent and RETURN HOME by itself.")
    box.setInformativeText(
        "Turn the guardian OFF and disarm first (the boat then stays where it is), "
        "or keep it ON and let the boat come home?")
    off = box.addButton("Guardian OFF + disarm, then " + ("quit" if exiting else "turn off"),
                        QMessageBox.ButtonRole.AcceptRole)
    keep = box.addButton("Keep ON — the boat returns home",
                         QMessageBox.ButtonRole.DestructiveRole)
    cancel = box.addButton(QMessageBox.StandardButton.Cancel)
    box.setDefaultButton(off)
    box.exec()
    clicked = box.clickedButton()
    if clicked is off:
        return "off"
    if clicked is keep:
        return "keep"
    _ = cancel
    return "cancel"
