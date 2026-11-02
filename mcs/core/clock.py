"""Elapsed-time text shared by every read-out (Qt-free)."""

from __future__ import annotations


def format_clock(seconds: float | None) -> str:
    """``m:ss``, or ``h:mm:ss`` from one hour, truncated to whole seconds;
    None (and anything negative) reads ``0:00``."""
    if seconds is None:
        return "0:00"
    seconds = int(max(0.0, seconds))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    return f"{h:d}:{m:02d}:{s:02d}" if h else f"{m:d}:{s:02d}"
