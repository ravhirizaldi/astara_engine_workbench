from __future__ import annotations

import math
import unittest

import numpy as np

from aerospace_workbench.mathematics.frames import (
    EARTH_MU,
    EARTH_RADIUS_M,
    EARTH_ROTATION_RAD_S,
)
from aerospace_workbench.simulation.orbit import orbital_elements


class OrbitTests(unittest.TestCase):
    def test_circular_equatorial_ecef_state(self) -> None:
        radius_m = EARTH_RADIUS_M + 200_000.0
        inertial_speed_m_s = math.sqrt(EARTH_MU / radius_m)
        elements = orbital_elements(
            np.array([radius_m, 0.0, 0.0]),
            np.array(
                [
                    0.0,
                    inertial_speed_m_s - EARTH_ROTATION_RAD_S * radius_m,
                    0.0,
                ]
            ),
        )

        self.assertAlmostEqual(elements["eccentricity"], 0.0, places=12)
        self.assertAlmostEqual(elements["inclination_deg"], 0.0, places=12)
        self.assertAlmostEqual(
            elements["periapsis_altitude_m"], 200_000.0, places=6
        )
        self.assertAlmostEqual(
            elements["apoapsis_altitude_m"], 200_000.0, places=6
        )


if __name__ == "__main__":
    unittest.main()
