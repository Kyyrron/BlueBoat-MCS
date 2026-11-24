"""Update Mission — what the NEXT run of a real-robot session executes.

Opened from the bottom toolbar while a run session is connected and no run is
active. It chooses:

* **Follow** — a designer path or the USBL pinger;
* **Path** — any saved Survey Pattern Designer mission, re-read from disk each
  time the window opens, so a path drawn while the boat is connected is
  offered straight away. ``(GPS)`` missions are deployed onto their real-world
  position (and previewed with their start "S" at once); the others are
  placed at the boat when the run starts;
* **Controller** — PID / LoS / MPC. A change is loaded by ``master_control``
  immediately (MPC on a worker thread); START RUN stays disabled until the
  station shows it loaded. MPC cannot follow the pinger;
* **Log note** — the ``{note}`` of the next run's ``{date}-{note}-poslog.csv``.

The dialog only returns a :class:`~mcs.models.store.NextMission`; applying it
is :meth:`mcs.ros.run_session.RunSession.update_mission`'s job.
"""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QRadioButton,
    QVBoxLayout,
    QWidget,
)

from mcs.config.settings import AppConfig
from mcs.designer import io_yaml
from mcs.gui import theme
from mcs.gui.dialogs.launch_dialog import custom_badge, list_custom_paths
from mcs.models.store import NextMission

_CONTROLLERS = ("PID", "LoS", "MPC")


class MissionUpdateDialog(QDialog):
    """Small modal window: path / pinger, controller and log note."""

    def __init__(self, cfg: AppConfig, current: NextMission,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self.setWindowTitle("Update Mission — next run")
        self.setMinimumWidth(420)
        layout = QVBoxLayout(self)

        intro = QLabel("Choose what the next START RUN executes. The boat stays "
                       "connected and controllable meanwhile.")
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color: {theme.TEXT_DIM};")
        layout.addWidget(intro)

        form = QFormLayout()
        layout.addLayout(form)

        # ---- Follow: path or pinger -------------------------------------------
        follow = QWidget()
        row = QHBoxLayout(follow)
        row.setContentsMargins(0, 0, 0, 0)
        self._path_radio = QRadioButton("Path")
        self._pinger_radio = QRadioButton("USBL pinger")
        group = QButtonGroup(self)
        group.addButton(self._path_radio)
        group.addButton(self._pinger_radio)
        row.addWidget(self._path_radio)
        row.addWidget(self._pinger_radio)
        row.addStretch(1)
        form.addRow("Follow", follow)

        # ---- Path list (fresh from disk) -----------------------------------------
        self._paths = QListWidget()
        self._paths.setMinimumHeight(150)
        for name, path in list_custom_paths(cfg):
            badge, tip = custom_badge(cfg, path, session=True)
            item = QListWidgetItem(f"{name}{badge}")
            item.setData(Qt.ItemDataRole.UserRole, str(path))
            if tip:
                item.setToolTip(tip)
            self._paths.addItem(item)
        if self._paths.count() == 0:
            empty = QListWidgetItem("(no designer path yet — use Create Survey "
                                    "Pattern)")
            empty.setFlags(Qt.ItemFlag.NoItemFlags)
            self._paths.addItem(empty)
        form.addRow("Path", self._paths)

        # ---- Controller --------------------------------------------------------
        self._controller = QComboBox()
        self._controller.addItems(_CONTROLLERS)
        self._controller.setToolTip(
            "Loaded by master_control as soon as you apply (MPC can take up "
            "to a minute after an MPC tuning change); START RUN waits for it.")
        form.addRow("Controller", self._controller)

        # ---- Log note ----------------------------------------------------------
        self._note = QLineEdit()
        self._note.setPlaceholderText("added to the next run's log file names")
        self._note.setToolTip("{date}-{note}-poslog.csv of the next run.")
        form.addRow("Log note", self._note)

        self._hint = QLabel()
        self._hint.setWordWrap(True)
        self._hint.setStyleSheet(f"color: {theme.TEXT_DIM}; font-size: 10px;")
        layout.addWidget(self._hint)

        self._buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        self._buttons.button(QDialogButtonBox.StandardButton.Ok).setText(
            "Apply to next run")
        self._buttons.accepted.connect(self.accept)
        self._buttons.rejected.connect(self.reject)
        layout.addWidget(self._buttons)

        # ---- Current selection --------------------------------------------------
        (self._pinger_radio if current.use_pinger else self._path_radio).setChecked(True)
        for i in range(self._paths.count()):
            if self._paths.item(i).data(Qt.ItemDataRole.UserRole) == current.source:
                self._paths.setCurrentRow(i)
                break
        if current.controller_type in _CONTROLLERS:
            self._controller.setCurrentText(current.controller_type)
        self._note.setText(current.note)

        self._pinger_radio.toggled.connect(self._refresh)
        self._paths.currentItemChanged.connect(self._refresh)
        self._controller.currentTextChanged.connect(self._refresh)
        self._refresh()

    # ------------------------------------------------------------------ gating
    def _source(self) -> str:
        item = self._paths.currentItem()
        data = item.data(Qt.ItemDataRole.UserRole) if item is not None else None
        return str(data) if data else ""

    def _refresh(self, *_args) -> None:
        pinger = self._pinger_radio.isChecked()
        self._paths.setEnabled(not pinger)
        # MPC cannot follow the pinger (master_control's pinger branch is
        # PID / LoS only).
        mpc = self._controller.findText("MPC")
        self._controller.model().item(mpc).setEnabled(not pinger)
        if pinger and self._controller.currentText() == "MPC":
            self._controller.setCurrentText("PID")

        source = self._source()
        ok = pinger or bool(source)
        self._buttons.button(QDialogButtonBox.StandardButton.Ok).setEnabled(ok)
        if pinger:
            hint = "The next run steers toward the USBL pinger."
        elif not source:
            hint = "Select a path."
        elif io_yaml.read_geo_anchor(Path(source)) is not None:
            hint = ("GPS-anchored: shown on the map at its real-world position "
                    "(start circled 'S') as soon as the GPS frame is anchored; "
                    "the boat drives to its start when the run begins.")
        else:
            hint = ("Not GPS-anchored: placed at the boat when you press START "
                    "RUN — the path begins where the boat is and runs out "
                    "ahead of it, so it is not drawn before then.")
        self._hint.setText(hint)

    # ------------------------------------------------------------------ result
    def selection(self) -> NextMission:
        pinger = self._pinger_radio.isChecked()
        source = "" if pinger else self._source()
        return NextMission(
            source=source,
            name=Path(source).stem if source else "",
            anchored=bool(source) and io_yaml.read_geo_anchor(Path(source)) is not None,
            use_pinger=pinger,
            controller_type=self._controller.currentText(),
            note=self._note.text().strip(),
        )
