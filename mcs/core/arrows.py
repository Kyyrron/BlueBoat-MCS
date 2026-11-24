"""Arrow-key thrust: which keys are held -> the ``[right, left]`` command.

Qt-free and ROS-free (like the rest of ``mcs/core``), so the arithmetic is
pinned by the smoke test without a window. The floating ARROW CONTROL box
(``mcs/gui/arrow_box.py``) owns the keys and the 20 Hz stream; master_control
applies the command through its own ``publish_thrust`` (uniform saturation to
``thrust_limit``, 20 N) -- nothing is clipped here.

The law, as the operator specified it:

* **Up** / **Down**: both thrusters at ``+power`` / ``-power`` (straight
  ahead / astern). Both held: they cancel.
* **Right** (turn clockwise): ``+turn`` on the LEFT thruster, ``-turn`` on the
  right one; **Left** (counter-clockwise) the opposite. Added to whatever
  Up/Down commands, so turning while driving keeps the drive. Both held: they
  cancel.

``power`` and ``turn`` are the box's Power (0-20 N, default 10) and Rotation
(3-15 N, default 5) sliders.

Clockwise alone at 5 N is right -5 / left +5; Up at 6 N plus clockwise is
right 1 / left 11. (A command under ~2 N per side sits in the ESC deadband
and does not turn the propeller.)
"""

from __future__ import annotations


def arrow_thrust(up: bool, down: bool, left: bool, right: bool,
                 power_n: float, turn_n: float) -> tuple[float, float]:
    """``(right_thruster_n, left_thruster_n)`` for the held arrow keys.

    Returned in the ``/thruster_input`` order, ``[right, left]``.
    """
    surge = float(power_n) * ((1 if up else 0) - (1 if down else 0))
    # +1 = clockwise (Right key), -1 = counter-clockwise (Left key)
    spin = (1 if right else 0) - (1 if left else 0)
    differential = float(turn_n) * spin
    return surge - differential, surge + differential
