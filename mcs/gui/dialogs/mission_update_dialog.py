"""Update Mission — what the NEXT run of a real-robot session executes.

Opened from the bottom toolbar while a run session is connected and no run is
active. It chooses:

* **Pinger** (2026-11-24) — *Off* (follow a path), *Follow pinger (no path)*
  or *Log pinger only* (follow a path, or "(no path)"), with the pinger
  source, the filter preset and **Advanced…** -- the same rows as the launch
  dialog (:class:`~mcs.gui.dialogs.pinger_filter_dialog.PingerForm`);
* **Path** — any saved Survey Pattern Designer mission, re-read from disk each
  time the window opens, so a path drawn while the boat is connected is
  offered straight away. ``(GPS)`` missions are deployed onto their real-world
  position (and previewed with their start "S" at once); the others are
  placed at the boat when the run starts. Disabled for Follow; Log adds
  "(no path)". **Path end** shows read-only how the selected file ends
  (repeats forever / holds the final pose, returns to start);
* **Controller** — PID / LoS / MPC. A change is loaded by ``master_control``
  immediately (MPC on a worker thread); START RUN stays disabled until the
  station shows it loaded. MPC cannot follow the pinger (greyed in Follow
  only);
* **Log note** — the ``{note}`` of the next run's ``{date}-{note}-poslog.csv``.

The dialog only returns a :class:`~mcs.models.store.NextMission`; applying it
is :meth:`mcs.ros.run_session.RunSession.update_mission`'s job.
"""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QVBoxLayout,
    QWidget,
)

from mcs.config.settings import AppConfig
from mcs.core import pinger_filter as pf
from mcs.designer import io_yaml
from mcs.gui import theme
from mcs.gui.dialogs.launch_dialog import (
    NO_PATH,
    NO_PATH_LABEL,
    NO_PATH_TIP,
    custom_badge,
    list_custom_paths,
    path_end_text,
)
from mcs.gui.dialogs.pinger_filter_dialog import PingerForm
from mcs.models.store import NextMission

_CONTROLLERS = ("PID", "LoS", "MPC")


class MissionUpdateDialog(QDialog):
    """Small modal window: pinger mode, path, filter, controller and log note."""

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
        self._form = form
        layout.addLayout(form)

        # ---- Pinger mode (the same rows as the launch dialog) -------------------
        self._pinger = PingerForm(cfg, self)
        self._pinger.add_mode_row(form)

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
        self._path_end = QLabel("—")
        self._path_end.setToolTip(
            "From the path's runtime file: loop: true repeats it forever, false "
            "ends it and the boat holds the final pose. Change it in the Survey "
            "Pattern Designer ('Repeat forever', 'Return to start').")
        form.addRow("Path end", self._path_end)
        self._pinger.add_filter_rows(form)

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
        self._pinger.set_mode(current.pinger_mode)
        self._pinger.set_source(current.pinger_source)
        self._pinger.restore(current.pinger_preset or cfg.pinger.default_preset,
                             current.pinger_filter)
        self._sync_no_path_item()
        want = (NO_PATH if (current.pinger_mode != pf.MODE_FOLLOW and not current.source)
                else current.source)
        for i in range(self._paths.count()):
            if self._paths.item(i).data(Qt.ItemDataRole.UserRole) == want:
                self._paths.setCurrentRow(i)
                break
        if current.controller_type in _CONTROLLERS:
            self._controller.setCurrentText(current.controller_type)
        self._note.setText(current.note)

        self._pinger.mode_changed.connect(self._on_pinger_mode_changed)
        self._paths.currentItemChanged.connect(self._refresh)
        self._controller.currentTextChanged.connect(self._refresh)
        self._refresh()

    # ------------------------------------------------------------------ gating
    def _source(self) -> str:
        """The selected designer file ("" for none and for "(no path)")."""
        item = self._paths.currentItem()
        data = item.data(Qt.ItemDataRole.UserRole) if item is not None else None
        return str(data) if data and data != NO_PATH else ""

    def _no_path_selected(self) -> bool:
        item = self._paths.currentItem()
        return item is not None and item.data(Qt.ItemDataRole.UserRole) == NO_PATH

    def _sync_no_path_item(self) -> None:
        """"(no path)" heads the list unless following the pinger."""
        first = self._paths.item(0)
        has = first is not None and first.data(Qt.ItemDataRole.UserRole) == NO_PATH
        want = self._pinger.mode_value() != pf.MODE_FOLLOW
        if want and not has:
            item = QListWidgetItem(NO_PATH_LABEL)
            item.setData(Qt.ItemDataRole.UserRole, NO_PATH)
            item.setToolTip(NO_PATH_TIP)
            self._paths.insertItem(0, item)
        elif has and not want:
            self._paths.takeItem(0)

    def _on_pinger_mode_changed(self, mode: str) -> None:
        self._sync_no_path_item()
        if mode == pf.MODE_LOG and self._paths.currentItem() is None:
            self._paths.setCurrentRow(0)
        self._refresh()

    def _refresh(self, *_args) -> None:
        mode = self._pinger.mode_value()
        follow = mode == pf.MODE_FOLLOW
        self._paths.setEnabled(not follow)
        # Source and filter only matter with the pinger on.
        self._pinger.set_rows_visible(self._form, mode=True, filt=mode != pf.MODE_OFF)
        # MPC cannot follow the pinger (master_control's pinger branch is
        # PID / LoS only): greyed in Follow mode only.
        mpc = self._controller.findText("MPC")
        self._controller.model().item(mpc).setEnabled(not follow)
        if follow and self._controller.currentText() == "MPC":
            self._controller.setCurrentText("PID")

        source = self._source()
        no_path = not follow and self._no_path_selected()
        ok = follow or bool(source) or no_path
        self._buttons.button(QDialogButtonBox.StandardButton.Ok).setEnabled(ok)
        self._path_end.setText("—" if follow or not source
                               else path_end_text(self._cfg, Path(source)))
        if no_path:
            self._path_end.setText("no path")
        if follow:
            hint = "The next run steers toward the USBL pinger."
        elif no_path:
            hint = ("The next run follows no path: the boat holds Manual Target / "
                    "the arrow keys" + (" while the pinger is logged."
                                         if mode == pf.MODE_LOG else "."))
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
        if mode == pf.MODE_LOG and source:
            hint += " The pinger is logged too."
        self._hint.setText(hint)

    # ------------------------------------------------------------------ result
    def selection(self) -> NextMission:
        mode = self._pinger.mode_value()
        source = "" if mode == pf.MODE_FOLLOW else self._source()
        return NextMission(
            source=source,
            name=Path(source).stem if source else "",
            anchored=bool(source) and io_yaml.read_geo_anchor(Path(source)) is not None,
            use_pinger=mode == pf.MODE_FOLLOW,
            controller_type=self._controller.currentText(),
            note=self._note.text().strip(),
            pinger_mode=mode,
            pinger_source=self._pinger.source_value(),
            pinger_preset=self._pinger.preset_value(),
            pinger_filter=self._pinger.filter_block(),
        )
