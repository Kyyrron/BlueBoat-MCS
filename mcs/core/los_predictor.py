"""Display-only approximation of the robot's future path under LoS control.

When the operator sets a manual target, ``master_control.py`` steers the
boat with its LoS law (see ``solve_LoS``)::

    yaw_rate_cmd = k_psi * atan2(y_body, x_body)
    v_cmd        = 2 * ln(k_v * d + 1)
    u_right/left = v_cmd +/- 0.295 * yaw_rate_cmd     (Newtons)

The station must *not* re-implement control; it only sketches a plausible
trajectory for situational awareness.  We therefore integrate a simple
unicycle whose speed / turn-rate response to the commanded thrusts is scaled
and saturated by configurable constants (:class:`~mcs.config.settings.
LosApproximation`).  The result is drawn as an explicitly dashed
"approximation" line and re-simulated live from the latest pose.
"""

from __future__ import annotations

import math

import numpy as np

from mcs.config.settings import LosApproximation


def predict_los_path(
    pose: tuple[float, float, float],
    target: tuple[float, float],
    cfg: LosApproximation,
) -> np.ndarray:
    """Return an (n, 2) array of world-frame waypoints from *pose* to *target*.

    Parameters
    ----------
    pose:
        Current robot ``(x, y, yaw)`` in the world frame.
    target:
        Manual target ``(x, y)`` in the world frame.
    cfg:
        Approximation constants (mirrors of the controller gains plus the
        thrust-to-motion scaling used only for display).
    """
    x, y, psi = pose
    tx, ty = target
    points = [(x, y)]

    steps = int(cfg.horizon_s / cfg.sim_dt)
    for _ in range(steps):
        # Target in body frame — identical maths to master_control.inRobotFrame
        dx, dy = tx - x, ty - y
        xb = dx * math.cos(psi) + dy * math.sin(psi)
        yb = dy * math.cos(psi) - dx * math.sin(psi)
        d = math.hypot(xb, yb)
        if d <= cfg.reached_distance_m:
            break

        # Commanded quantities, exactly as in solve_LoS
        yaw_rate_cmd = cfg.k_psi * math.atan2(yb, xb)
        v_cmd = 2.0 * math.log(cfg.k_v * d + 1.0)

        # Approximate vehicle response (saturated first-order-ish mapping)
        v = min(v_cmd * cfg.thrust_to_speed / 0.35, cfg.v_max)  # normalised scaling
        r = max(-cfg.r_max, min(cfg.r_max, yaw_rate_cmd * cfg.thrust_to_yaw_rate))

        psi += r * cfg.sim_dt
        x += v * math.cos(psi) * cfg.sim_dt
        y += v * math.sin(psi) * cfg.sim_dt
        points.append((x, y))

    return np.asarray(points, dtype=np.float64)
