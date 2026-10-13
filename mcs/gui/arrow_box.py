"""ARROW CONTROL — drive the boat with the keyboard's arrow keys.

A floating box pinned to the map's bottom-right corner (the mission-stats box
holds the top-right, the sea box the top-left). Shown whenever a launch with a
controller is running: a real-robot run session, between runs or during one,
and -- because it costs nothing there -- a Gazebo run.

* **Control using arrows** (checkable) engages it; while engaged the box is
  outlined amber and the arrow keys belong to it.
* **Power** (0-20 N, default 10, live): Up / Down drive both thrusters at
  +power / -power.
* **Rotation** (3-15 N, default 5, live): Left / Right add a +/-rotation
  differential per side on top of Up / Down, counter-clockwise / clockwise
  (:func:`mcs.core.arrows.arrow_thrust`). A sum above 20 N per side is scaled
  down uniformly by master_control, so the turn keeps its direction.

The command is streamed to ``master_control`` on ``/blueboat/arrow_thrust``
at ``publish_hz`` (20 Hz, ``[0, 0]`` while no key is held). There it overrides
every other branch -- a running path or pinger run is PAUSED exactly as by a
manual target, its logged target stays the last one registered -- and goes
out through ``publish_thrust`` (uniform 20 N saturation). If the stream stops,
master_control's dead-man (``arrow_timeout``, 0.5 s) zeroes the thrust and
stays in arrow control: a lost station never hands the boat back to
autonomous control.

Turning it OFF holds the boat where it was released: the station publishes the
boat's position as a manual target (crosshair + Continue Original Mission /
Release Target), then releases arrow control. A START RUN ends it without a
hold point.

Keyboard safety: keys are captured through an application event filter only
while the main window is the active window and no modal dialog is open;
auto-repeat is ignored; every held key is forgotten the moment the window
loses focus, so a key release the station never saw cannot leave thrust on.
"""

from __future__ import annotations

from PySide6.QtCore import QEvent, QObject, Qt, QTimer, Signal
from PySide6.QtGui import QGuiApplication
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from mcs.config.settings import AppConfig
from mcs.core.arrows import arrow_thrust
from mcs.gui import theme
from mcs.gui.widgets import CollapsibleSection
from mcs.models.store import DataStore
from mcs.ros.command_center import CommandCenter

_ARROWS = (Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Right)

_STYLE = """
    QWidget#floatingArrowBox {{
        background-color: rgba(45, 45, 45, 230);
        border: {width}px solid {color};
        border-radius: 6px;
    }}
"""


class FloatingArrowBox(QWidget):
    """Arrow-key thrust control floating over the map."""

    #: arrow control turned ON by the operator
    engaged = Signal()
    #: turned OFF; True = hold the boat where it was released
    released = Signal(bool)

    def __init__(self, cfg: AppConfig, store: DataStore, commands: CommandCenter,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self._store = store
        self._commands = commands
        self._keys: set = set()
        self._filter_installed = False
        self._hold_on_release = True     # see set_active

        self.setObjectName("floatingArrowBox")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self._set_frame(active=False)
        self.setFixedWidth(250)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSizeConstraint(QVBoxLayout.SizeConstraint.SetFixedSize)
        sec = CollapsibleSection("ARROW CONTROL")

        self.toggle = QPushButton("Control using arrows")
        self.toggle.setCheckable(True)
        self.toggle.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.toggle.setToolTip(
            "Drive with the arrow keys: ↑ ahead, ↓ astern at the power below; "
            "← / → turn counter-clockwise / clockwise (± the rotation set "
            "below, per side). Pauses a running path or pinger run. Turning "
            "it off holds the boat where it is.")
        self.toggle.toggled.connect(self._on_toggled)
        sec.add_widget(self.toggle)

        a = cfg.arrows
        row, self.slider, self._power_label = self._slider_row(
            "Power", 0, a.max_power_n, a.default_power_n,
            "Forward / astern thrust per thruster, applied immediately. Below "
            "about 2 N the propellers do not turn (ESC deadband).",
            self._on_power)
        sec.add_widget(row)
        row, self.turn_slider, self._turn_label = self._slider_row(
            "Rotation", a.min_turn_n, a.max_turn_n, a.turn_n,
            "Turn strength: ← / → add ± this much per thruster on top of "
            "↑ / ↓, applied immediately. A side above 20 N is scaled down by "
            "master_control together with the other, so the turn keeps its "
            "direction.",
            self._on_turn)
        sec.add_widget(row)

        hint = QLabel("↑ ahead   ↓ astern   ← ↺   → ↻")
        hint.setStyleSheet(f"color: {theme.TEXT_DIM}; font-size: 10px;")
        sec.add_widget(hint)

        self._readout = QLabel()
        self._readout.setStyleSheet(
            "font-family: 'DejaVu Sans Mono', monospace; font-size: 11px;")
        sec.add_widget(self._readout)
        layout.addWidget(sec)

        self._timer = QTimer(self)
        self._timer.setInterval(int(1000 / max(cfg.arrows.publish_hz, 1.0)))
        self._timer.timeout.connect(self._publish)

        app = QGuiApplication.instance()
        if app is not None:
            app.applicationStateChanged.connect(self._on_app_state)
            app.focusWindowChanged.connect(self._on_focus_window)

        self._on_power(self.slider.value())
        self._on_turn(self.turn_slider.value())
        self._update_readout()
        self.setVisible(False)

    @staticmethod
    def _slider_row(name: str, lo: float, hi: float, value: float,
                    tooltip: str, slot) -> tuple[QWidget, QSlider, QLabel]:
        """One labelled N slider: ``name  [----o----]  12 N``."""
        row = QWidget()
        hl = QHBoxLayout(row)
        hl.setContentsMargins(0, 0, 0, 0)
        name_label = QLabel(name)
        name_label.setMinimumWidth(56)
        hl.addWidget(name_label)
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(int(round(lo)), int(round(hi)))
        slider.setValue(int(round(value)))
        slider.setFocusPolicy(Qt.FocusPolicy.NoFocus)   # arrows are ours
        slider.setToolTip(tooltip)
        slider.valueChanged.connect(slot)
        hl.addWidget(slider, 1)
        value_label = QLabel()
        value_label.setMinimumWidth(36)
        value_label.setAlignment(Qt.AlignmentFlag.AlignRight
                                 | Qt.AlignmentFlag.AlignVCenter)
        hl.addWidget(value_label)
        return row, slider, value_label

    # ================================================================ state
    @property
    def active(self) -> bool:
        return self.toggle.isChecked()

    def set_active(self, on: bool, hold: bool = True) -> None:
        """Programmatic on/off (``hold`` only matters when turning off)."""
        self._hold_on_release = hold
        self.toggle.setChecked(on)
        self._hold_on_release = True

    def _on_toggled(self, on: bool) -> None:
        self._keys.clear()
        app = QApplication.instance()
        if on:
            if app is not None and not self._filter_installed:
                app.installEventFilter(self)
                self._filter_installed = True
            self._store.session.arrow_active = True
            self._timer.start()
            self._publish()
            self.engaged.emit()
        else:
            self._timer.stop()
            if app is not None and self._filter_installed:
                app.removeEventFilter(self)
                self._filter_installed = False
            self._store.session.arrow_active = False
            self._store.session.arrow_cmd = (0.0, 0.0)
            self.released.emit(self._hold_on_release)
        self._set_frame(active=on)
        self._update_readout()

    def set_available(self, available: bool) -> None:
        """Shown only while a launch with a controller runs; a vanished
        launch ends arrow control (no hold point -- nothing to hold with)."""
        if not available and self.active:
            self.set_active(False, hold=False)
        self.setVisible(available)

    # ================================================================ keys
    def _capture_allowed(self) -> bool:
        return (QApplication.activeModalWidget() is None
                and QApplication.activeWindow() is self.window())

    def eventFilter(self, obj: QObject, event: QEvent) -> bool:
        et = event.type()
        if et not in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            return False
        key = event.key()
        if key not in _ARROWS:
            return False
        if not self._capture_allowed():
            if self._keys:
                self._keys.clear()
                self._publish()
            return False
        if event.isAutoRepeat():
            return True                      # held key: state already known
        if et == QEvent.Type.KeyPress:
            self._keys.add(key)
        else:
            self._keys.discard(key)
        self._publish()                      # react at once, not at the next tick
        return True

    def _on_app_state(self, state) -> None:
        if state != Qt.ApplicationState.ApplicationActive:
            self._forget_keys()

    def _on_focus_window(self, window) -> None:
        if window is None or window is not self.window().windowHandle():
            self._forget_keys()

    def _forget_keys(self) -> None:
        if self._keys:
            self._keys.clear()
            if self.active:
                self._publish()

    # ================================================================ output
    def command(self) -> tuple[float, float]:
        """(right, left) N for the keys held now."""
        k = self._keys
        return arrow_thrust(Qt.Key.Key_Up in k, Qt.Key.Key_Down in k,
                            Qt.Key.Key_Left in k, Qt.Key.Key_Right in k,
                            float(self.slider.value()),
                            float(self.turn_slider.value()))

    def _publish(self) -> None:
        if not self.active:
            return
        right, left = self.command()
        self._store.session.arrow_cmd = (right, left)
        self._commands.publish_arrow_thrust(right, left)
        self._update_readout()

    def _on_power(self, value: int) -> None:
        self._power_label.setText(f"{value} N")
        self._publish()

    def _on_turn(self, value: int) -> None:
        self._turn_label.setText(f"±{value} N")
        self._publish()

    def _update_readout(self) -> None:
        if not self.active:
            self._readout.setText("off")
            self._readout.setStyleSheet(
                f"font-family: 'DejaVu Sans Mono', monospace; font-size: 11px;"
                f" color: {theme.TEXT_DIM};")
            return
        right, left = self._store.session.arrow_cmd
        self._readout.setText(f"ACTIVE   L {left:+5.1f} N   R {right:+5.1f} N")
        self._readout.setStyleSheet(
            f"font-family: 'DejaVu Sans Mono', monospace; font-size: 11px;"
            f" color: {theme.WARN}; font-weight: bold;")

    def _set_frame(self, active: bool) -> None:
        self.setStyleSheet(_STYLE.format(width=2 if active else 1,
                                         color=theme.WARN if active else "#555"))
