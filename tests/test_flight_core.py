import ctypes
import unittest

from astara.flight_core import FlightCore, SensorFrame
from astara.scenario import default_scenario


class FlightCoreTests(unittest.TestCase):
    def test_core_advances_to_first_boost(self) -> None:
        scenario = default_scenario()
        with FlightCore(scenario) as core:
            output = None
            for step in range(50):
                time_s = step * 0.005
                output = core.step(
                    SensorFrame(
                        time_s,
                        0.005,
                        (ctypes.c_double * 3)(20.0, 0.0, 0.0),
                        (ctypes.c_double * 3)(0.0, 0.0, 0.0),
                        (ctypes.c_double * 3)(1.0, 0.0, 0.0),
                        time_s,
                        (ctypes.c_double * 3)(1.0, 2.0, 3.0),
                        (ctypes.c_double * 3)(0.0, 0.0, 20.0),
                        20.0,
                        1000.0,
                        100.0,
                        1,
                        1,
                        0,
                    )
                )
            self.assertEqual(output.mode, 3)
            self.assertLessEqual(abs(output.tvc_pitch_rad), 0.11)


if __name__ == "__main__":
    unittest.main()
