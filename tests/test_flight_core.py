import ctypes
import unittest

from astara.flight_core import (
    FSW_COMMAND_ARM,
    FSW_COMMAND_LAUNCH,
    FlightCore,
    SensorFrame,
    sensor_suite_from_frames,
)
from astara.scenario import default_scenario


class FlightCoreTests(unittest.TestCase):
    def test_three_channel_bridge_preserves_each_channel(self) -> None:
        frames = []
        for channel in range(3):
            frame = SensorFrame()
            frame.time_s = 0.0
            frame.dt_s = 0.005
            frame.acceleration_body_m_s2[0] = 10.0 + channel
            frame.magnetic_body[0] = 1.0
            frame.barometric_altitude_m = float(channel)
            frame.barometer_valid = 1
            frame.gnss_position_ecef_m[0] = 6_371_000.0 + channel
            frame.gnss_valid = 1
            frames.append(frame)
        suite = sensor_suite_from_frames(frames)
        self.assertEqual(suite.imu_count, 3)
        self.assertEqual(suite.barometer_count, 3)
        self.assertEqual(suite.gnss_count, 3)
        self.assertEqual(suite.imus[2].acceleration_body_m_s2[0], 12.0)
        self.assertEqual(suite.barometers[1].altitude_m, 1.0)

    def test_safe_requires_explicit_arm_and_launch(self) -> None:
        scenario = default_scenario()
        frame = SensorFrame(
            0.0,
            0.005,
            (ctypes.c_double * 3)(20.0, 0.0, 0.0),
            (ctypes.c_double * 3)(0.0, 0.0, 0.0),
            (ctypes.c_double * 3)(1.0, 0.0, 0.0),
            0.0,
            (ctypes.c_double * 3)(6_371_000.0, 0.0, 0.0),
            (ctypes.c_double * 3)(0.0, 0.0, 0.0),
            0.0,
            1000.0,
            100.0,
            1,
            1,
            0,
            0.0,
            0.0,
            1,
            0,
            0,
            0,
        )
        with FlightCore(scenario, auto_commands=False) as core:
            output = core.step(frame, command_type=0)
            self.assertEqual(output.mode, 0)
            frame.time_s = 0.005
            frame.barometer_sample_time_s = frame.time_s
            frame.gnss_sample_time_s = frame.time_s
            output = core.step(frame, command_type=FSW_COMMAND_ARM)
            self.assertEqual(output.mode, 1)
            frame.time_s = 0.01
            frame.barometer_sample_time_s = frame.time_s
            frame.gnss_sample_time_s = frame.time_s
            output = core.step(frame, command_type=FSW_COMMAND_LAUNCH)
            self.assertEqual(output.mode, 2)
            self.assertEqual(output.stage1_ignite, 1)

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
                        (ctypes.c_double * 3)(6_371_000.0, 0.0, 0.0),
                        (ctypes.c_double * 3)(0.0, 0.0, 20.0),
                        20.0,
                        1000.0,
                        100.0,
                        1,
                        1,
                        0,
                        time_s,
                        time_s,
                        1,
                        int(time_s > 0.0),
                        0,
                        0,
                    )
                )
            self.assertEqual(output.mode, 3)
            self.assertEqual(output.navigation_status, 0)
            self.assertEqual(output.attitude_valid, 1)
            self.assertEqual(output.imu_usable_mask, 1)
            self.assertEqual(output.barometer_usable_mask, 1)
            self.assertEqual(output.gnss_usable_mask, 1)
            self.assertEqual(output.sensor_status_flags, 0b111)
            self.assertEqual(output.imu_rejected_mask, 0)
            self.assertEqual(output.disagreement_flags, 0)
            self.assertLessEqual(abs(output.tvc_pitch_rad), 0.11)
            with self.assertRaisesRegex(RuntimeError, "status -2"):
                core.step(
                    SensorFrame(
                        time_s,
                        0.005,
                        (ctypes.c_double * 3)(20.0, 0.0, 0.0),
                        (ctypes.c_double * 3)(0.0, 0.0, 0.0),
                        (ctypes.c_double * 3)(1.0, 0.0, 0.0),
                        time_s,
                        (ctypes.c_double * 3)(6_371_000.0, 0.0, 0.0),
                        (ctypes.c_double * 3)(0.0, 0.0, 20.0),
                        20.0,
                        1000.0,
                        100.0,
                        1,
                        1,
                        0,
                        time_s,
                        time_s,
                        1,
                        1,
                        0,
                        0,
                    )
                )


if __name__ == "__main__":
    unittest.main()
