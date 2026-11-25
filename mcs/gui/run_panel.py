"""RUN box — pinned under the left panel's scroll area during a real-robot session.

Visually directly below ROS DIAGNOSTICS, but outside the scroll area so the
START RUN / STOP RUN button can never scroll out of sight. Shown only while a
real-robot run session is launched (``store.session.active``); a Gazebo
launch never shows it.

Read-out (pulled from :meth:`mcs.ros.run_session.RunSession.status` on the
shared 10 Hz tick):

* **Next run** — the path (``GPS`` deployed / ``placed`` at the boat at the
  start), the USBL pinger (follow), or "pinger log only" (no path); a path
  run that also logs the pinger says "+ pinger log";
* **Controller** — loaded / loading… / error, as master_control reports it;
* **State** — connected, starting, Run n with its timer, last run;
* **Log** — the run's CSV as robot_interface confirms it, or where the last
  one was saved;
* **Bag** — the run's ros2bag: recording, saved (filed into the run's
  folder with the CSV), or off.

Under the rows, an amber "Path complete — holding final pose" line while
``master_control`` reports ``path_complete`` for the active run (a path
whose runtime file has ``loop: false`` has ended; the boat holds its final
pose until STOP RUN).

One big button: START RUN (green) while idle, STOP RUN (amber) during a run.
Its tooltip says why START RUN is unavailable when it is.
"""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QFrame, QLabel, QPushButton, QVBoxLayout, QWidget

from mcs.gui import theme
from mcs.gui.widgets import InfoGrid, elide_middle
from mcs.models.store import RUN_NO_PATH, RUN_PINGER, RUN_PINGER_LOG, DataStore
from mcs.ros.run_session import RunSession

_COLORS = {"ok": theme.OK, "warn": theme.WARN, "err": theme.ERR,
           "dim": theme.TEXT_DIM}
#: Longest value shown in full; longer ones are elided (full text in tooltip).
_MAX_CHARS = 30


class RunPanel(QFrame):
    """The session's run control: status read-out + START/STOP RUN."""

    start_clicked = Signal()
    stop_clicked = Signal()

    def __init__(self, store: DataStore, session: RunSession,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._store = store
        self._session = session
        self.setObjectName("panel")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 6)
        layout.setSpacing(4)

        title = QLabel("RUN")
        title.setStyleSheet(f"font-weight: bold; color: {theme.TEXT_DIM}; "
                            "letter-spacing: 1px;")
        layout.addWidget(title)

        self.grid = InfoGrid()
        for key in ("Next run", "Controller", "State", "Log", "Bag"):
            self.grid.add_row(key)
        layout.addWidget(self.grid)

        self.path_complete = QLabel("Path complete — holding final pose")
        self.path_complete.setWordWrap(True)
        self.path_complete.setStyleSheet(f"color: {theme.WARN}; font-weight: bold;")
        self.path_complete.setToolTip(
            "master_control reports path_complete: the run's path (loop: false) has "
            "ended and the boat holds its final pose. STOP RUN ends the run.")
        self.path_complete.setVisible(False)
        layout.addWidget(self.path_complete)

        self.button = QPushButton("▶  START RUN")
        self.button.setObjectName("runStartButton")
        self.button.setMinimumHeight(46)
        self.button.clicked.connect(self._on_clicked)
        layout.addWidget(self.button)

        self._running = False
        self.setVisible(False)

    # ---------------------------------------------------------------- actions
    def _on_clicked(self) -> None:
        if self._running:
            self.stop_clicked.emit()
        else:
            self.start_clicked.emit()

    # ---------------------------------------------------------------- refresh
    def _set(self, key: str, text: str, color: str | None) -> None:
        """Set a row, eliding long text (CSV names, error details) in the
        middle so the pinned box can never widen the left panel; the full text
        is the row's tooltip."""
        self.grid.set(key, elide_middle(text, _MAX_CHARS), color)
        label = self.grid._values.get(key)
        if label is not None and label.toolTip() != text:
            label.setToolTip(text)

    def refresh(self) -> None:
        ss = self._store.session
        self.setVisible(ss.active)
        if not ss.active:
            return
        st = self._session.status()
        nxt = ss.next
        kind = nxt.kind()
        if kind == RUN_PINGER:
            next_text = "USBL pinger"
        elif kind == RUN_PINGER_LOG:
            next_text = "pinger log only (no path)"
        elif kind == RUN_NO_PATH:
            next_text = "no path — Manual Target / arrows"
        else:
            next_text = f"{nxt.name} ({'GPS' if nxt.anchored else 'placed at start'})"
            if nxt.logs_pinger:
                next_text += " + pinger log"
        self._set("Next run", next_text, None)
        self._set("Controller", st.controller, _COLORS[st.controller_color])
        self._set("State", st.state, _COLORS[st.state_color])
        self._set("Log", st.log, _COLORS[st.log_color])
        self._set("Bag", st.bag, _COLORS[st.bag_color])
        if self.path_complete.isHidden() == st.path_complete:
            self.path_complete.setVisible(st.path_complete)

        running = st.running
        if running != self._running or self.button.objectName() == "":
            self._running = running
            self.button.setObjectName("runStopButton" if running else "runStartButton")
            self.button.setText("■  STOP RUN" if running else "▶  START RUN")
            # objectName-keyed styles need a re-polish to take effect.
            self.button.style().unpolish(self.button)
            self.button.style().polish(self.button)
        if running:
            self.button.setEnabled(st.pending != "stop")
            self.button.setToolTip(
                "Stop the run: following stops, the boat holds zero thrust (still "
                "in override), the run's CSV, .npy and report are written.")
        else:
            blockers = self._session.start_blockers()
            self.button.setEnabled(not blockers)
            self.button.setToolTip(
                "Start the run now: the path is written (placed at the boat, or "
                "GPS-deployed), logging starts and the boat follows at once."
                if not blockers else "Not available: " + "; ".join(blockers))
