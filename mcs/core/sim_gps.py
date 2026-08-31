"""Simulated GPS receiver: world metres -> lat/lon about a fixed origin.

Used when a GPS-anchored mission is launched in Gazebo simulation. The sim
world is local ENU (same frame kind as the real boat's /blueboat/odom), so the
mapping is a pure translation: the FIRST queried world position is latched as
the receiver origin's world coordinates, and every fix is

    fix = local_en_to_latlon(world - world_first (+ noise), origin_lat/lon)

— exactly the inverse of what the station's GeoReferencer estimates back from
the fixes. The caller chooses the origin (the boat's first fix is placed a
configurable distance north of the mission's first point, see SimGpsConfig).

Pure Python: no ROS, no Qt — covered directly by smoke_test.py. The bridge
node drives it from the ROS executor thread; after construction and handover
nothing else touches it, so no locking is needed here.
"""

from __future__ import annotations

import random

from mcs.core.geo import local_en_to_latlon


class SimGpsModel:
    """World-position -> simulated NavSatFix lat/lon, with optional noise."""

    def __init__(self, origin_lat: float, origin_lon: float,
                 sigma_m: float = 0.0, seed: int | None = None) -> None:
        self._lat0 = float(origin_lat)
        self._lon0 = float(origin_lon)
        self._sigma = float(sigma_m)
        self._rng = random.Random(seed)
        self._world_first: tuple[float, float] | None = None

    def fix_for(self, world_x: float, world_y: float) -> tuple[float, float]:
        """Simulated (lat, lon) for the given world position.

        The first call latches the receiver origin onto the given position,
        so the first fix is exactly (origin_lat, origin_lon) when sigma is 0.
        """
        if self._world_first is None:
            self._world_first = (float(world_x), float(world_y))
        east = world_x - self._world_first[0]
        north = world_y - self._world_first[1]
        if self._sigma > 0.0:
            east += self._rng.gauss(0.0, self._sigma)
            north += self._rng.gauss(0.0, self._sigma)
        return local_en_to_latlon(east, north, self._lat0, self._lon0)
