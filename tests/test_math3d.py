import unittest

import numpy as np

from astara.math3d import (
    cross3,
    ecef_to_ned,
    geodetic_to_ecef,
    initial_attitude,
    ned_to_ecef,
    quat_normalize,
    quat_rotate,
)


class Math3dTests(unittest.TestCase):
    def test_cross3_matches_numpy(self) -> None:
        left = np.array([1.5, -2.0, 4.25])
        right = np.array([-3.0, 0.75, 2.0])
        np.testing.assert_array_equal(cross3(left, right), np.cross(left, right))

    def test_quaternion_normalization_and_rotation(self) -> None:
        quaternion = quat_normalize(np.array([2.0, 0.0, 0.0, 0.0]))
        np.testing.assert_allclose(quat_rotate(quaternion, np.array([1.0, 2.0, 3.0])), [1, 2, 3])
        self.assertAlmostEqual(float(np.linalg.norm(quaternion)), 1.0)

    def test_ned_round_trip(self) -> None:
        position = geodetic_to_ecef(-6.2, 106.8, 50.0)
        vector = np.array([10.0, -4.0, 2.0])
        np.testing.assert_allclose(
            ecef_to_ned(ned_to_ecef(vector, position), position),
            vector,
            atol=1e-9,
        )
        axis = quat_rotate(initial_attitude(position, 90.0), np.array([1.0, 0.0, 0.0]))
        self.assertGreater(float(np.dot(axis, position / np.linalg.norm(position))), 0.999)


if __name__ == "__main__":
    unittest.main()
