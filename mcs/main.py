"""Application bootstrap.

Usage::

    python run.py [--config path/to/config.json]
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication

from mcs.config.settings import AppConfig
from mcs.gui import theme
from mcs.gui.main_window import MainWindow


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="BlueBoat Mission Control Station")
    parser.add_argument("--config", type=Path, default=None,
                        help="JSON configuration override file")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)

    # Terminal = complete debug output of the application. The root logger
    # writes every record to the launching terminal (stderr); GUI-facing
    # messages are fanned out to BOTH sinks from the same SignalBus signals
    # (see MainWindow._connect_signals), so terminal and GUI stay
    # synchronized by construction. --verbose additionally surfaces the raw
    # ros2-launch output ([launch] …) at DEBUG level.
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        stream=sys.stderr,
    )

    cfg = AppConfig.load(args.config)

    app = QApplication(sys.argv)
    app.setApplicationName("BlueBoat Mission Control Station")
    app.setStyleSheet(theme.STYLESHEET)

    window = MainWindow(cfg)
    window.show()

    # Let Ctrl-C in the launching terminal close the station cleanly.
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
