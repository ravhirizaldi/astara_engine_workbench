import csv
import gzip
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from aerospace_workbench.configuration.scenarios import (
    default_scenario,
    load_scenario,
    scenario_hash,
)
from aerospace_workbench.flight_software.abi import FswOutput
from aerospace_workbench.flight_software.bridge import (
    FSW_SENSOR_DIAGNOSTIC_FIELDS,
)
from aerospace_workbench.mathematics.frames import (
    EARTH_ROTATION_RAD_S,
    geodetic_to_ecef,
    initial_attitude,
)
from aerospace_workbench.mathematics.quaternions import (
    quat_conjugate,
    quat_rotate,
)
from aerospace_workbench.simulation.actuators import actuator_commands
from aerospace_workbench.simulation.propulsion import propulsion_step
from aerospace_workbench.simulation.runner import (
    AVIONICS_CSV_FIELDS,
    _derivative,
    _sensor_frame,
    run_simulation,
)
from aerospace_workbench.simulation.sensors import apply_sensor_faults
from aerospace_workbench.simulation.truth_model import Body


class TwinTests(unittest.TestCase):
    def test_avionics_timeline_is_delayed_and_seeded(self) -> None:
        scenario = default_scenario()
        scenario["simulation"].update(
            {"max_time_s": 0.03, "output_rate_hz": 200.0}
        )
        gnss_timing = scenario["avionics"]["devices"]["gnss"]
        gnss_timing.update(
            {
                "clock_offset_s": 0.001,
                "drift_ppm": 20.0,
                "jitter_s": 0.0001,
                "phase_offset_s": 0.002,
            }
        )

        first = run_simulation(
            scenario, seed=9, create_report=False, persist=False
        )
        second = run_simulation(
            scenario, seed=9, create_report=False, persist=False
        )

        self.assertEqual(first.avionics_timeline, second.avionics_timeline)
        self.assertTrue(
            {
                "air_data_computer",
                "engine_controller",
                "discrete_input_module",
                "recovery_controller",
                "flight_computer_platform",
            }.issubset(
                {row["subsystem"] for row in first.avionics_timeline}
            )
        )
        gnss = next(
            row
            for row in first.avionics_timeline
            if row["subsystem"] == "gnss"
        )
        self.assertEqual(set(gnss), set(AVIONICS_CSV_FIELDS))
        self.assertLess(
            gnss["sensor_sample_time_s"],
            gnss["sensor_completion_time_s"],
        )
        self.assertLess(
            gnss["sensor_completion_time_s"],
            gnss["bus_publish_time_s"],
        )
        self.assertLess(
            gnss["bus_publish_time_s"],
            gnss["fsw_receive_time_s"],
        )

        gnss_timing["deadline_s"] = 0.001
        dropped = run_simulation(
            scenario, seed=9, create_report=False, persist=False
        )
        self.assertGreater(
            dropped.manifest["avionics_timing"]["dropped_deadlines"][
                "devices"
            ]["gnss"],
            0,
        )

    def test_core_estimator_attitude_is_continuous_at_separation(self) -> None:
        scenario = default_scenario()
        scenario["simulation"].update(
            {
                "max_time_s": 9.0,
                "output_rate_hz": scenario["sensors"]["imu_rate_hz"],
            }
        )
        result = run_simulation(
            scenario,
            create_report=False,
            persist=False,
        )
        separation_time = next(
            event["time_s"]
            for event in result.events
            if event["event"] == "stage_separation"
        )
        core_row = next(
            row
            for row in result.telemetry
            if row["body"] == "core_stage"
        )
        upper_row = next(
            row
            for row in result.telemetry
            if row["body"] == "upper_stage"
            and row["time_s"] == core_row["time_s"]
        )
        integrated_row = max(
            (
                row
                for row in result.telemetry
                if row["body"] == "integrated_stack"
            ),
            key=lambda row: row["time_s"],
        )
        core_attitude = np.array(
            [
                core_row[f"fsw_estimated_attitude_{axis}"]
                for axis in "wxyz"
            ]
        )
        upper_attitude = np.array(
            [
                upper_row[f"fsw_estimated_attitude_{axis}"]
                for axis in "wxyz"
            ]
        )
        branch_difference_rad = 2.0 * math.acos(
            min(1.0, abs(float(np.dot(core_attitude, upper_attitude))))
        )
        integrated_attitude = np.array(
            [
                integrated_row[f"fsw_estimated_attitude_{axis}"]
                for axis in "wxyz"
            ]
        )
        core_jump_rad = 2.0 * math.acos(
            min(1.0, abs(float(np.dot(core_attitude, integrated_attitude))))
        )
        self.assertAlmostEqual(core_row["time_s"], separation_time)
        self.assertLess(branch_difference_rad, 1e-3)
        self.assertLess(core_jump_rad, 1e-3)

    def test_upper_stage_burn_persists_after_ignition_pulse(self) -> None:
        scenario = default_scenario()
        stage = scenario["vehicle"]["stages"][1]
        body = Body(
            name="upper_stage",
            stage_index=1,
            stage=stage,
            position_ecef_m=np.array([6_378_137.0, 0.0, 0.0]),
            velocity_ecef_m_s=np.zeros(3),
            attitude_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            body_rates_rad_s=np.zeros(3),
            fuel_kg=stage["fuel_mass_kg"],
            oxidizer_kg=stage["oxidizer_mass_kg"],
        )
        ignition = FswOutput()
        ignition.stage2_ignite = 1

        propulsion_step(
            body, ignition, 10.0, 0.005, scenario
        )
        later_thrust, _, _ = propulsion_step(
            body, FswOutput(), 10.1, 0.005, scenario
        )

        self.assertEqual(body.engine_started_s, 10.0)
        self.assertGreater(later_thrust, 0.0)

    def test_body_to_ecef_attitude_removes_earth_rate(self) -> None:
        scenario = default_scenario()
        stage = scenario["vehicle"]["stages"][0]
        position = geodetic_to_ecef(-6.2, 106.8, 50.0)
        attitude = initial_attitude(position, 90.0)
        earth_rate_body = quat_rotate(
            quat_conjugate(attitude),
            np.array([0.0, 0.0, EARTH_ROTATION_RAD_S]),
        )
        body = Body(
            name="integrated_stack",
            stage_index=0,
            stage=stage,
            position_ecef_m=position,
            velocity_ecef_m_s=np.zeros(3),
            attitude_wxyz=attitude,
            body_rates_rad_s=earth_rate_body,
            fuel_kg=stage["fuel_mass_kg"],
            oxidizer_kg=stage["oxidizer_mass_kg"],
        )
        state = np.concatenate(
            (
                body.position_ecef_m,
                body.velocity_ecef_m_s,
                body.attitude_wxyz,
                body.body_rates_rad_s,
            )
        )

        derivative = _derivative(
            body,
            scenario,
            state,
            0.0,
            np.zeros(2),
            np.zeros(4),
            0.0,
        )

        np.testing.assert_allclose(derivative[6:10], 0.0, atol=1e-15)

    def test_three_sensor_channels_are_independent_and_faults_hold(self) -> None:
        scenario = default_scenario()
        scenario["sensors"]["magnetometer_rate_hz"] = 100.0
        scenario["sensors"]["magnetometer_noise"] = 0.0
        scenario["sensors"]["magnetometer_bias_sigma"] = 0.0
        stage = scenario["vehicle"]["stages"][0]
        launch_position = np.array([6_378_137.0, 0.0, 0.0])
        body = Body(
            name="integrated_stack",
            stage_index=0,
            stage=stage,
            position_ecef_m=launch_position.copy(),
            velocity_ecef_m_s=np.zeros(3),
            attitude_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            body_rates_rad_s=np.zeros(3),
            fuel_kg=stage["fuel_mass_kg"],
            oxidizer_kg=stage["oxidizer_mass_kg"],
        )
        body.last_specific_force_body_m_s2[:] = (9.764, 0.0, 0.0)
        rng = np.random.default_rng(123)
        first = [
            _sensor_frame(
                body,
                scenario,
                rng,
                0.0,
                0.005,
                launch_position,
                False,
                channel,
            )
            for channel in range(3)
        ]
        self.assertEqual(len(body.sensor_channels), 3)
        self.assertFalse(
            np.array_equal(
                first[0].acceleration_body_m_s2,
                first[1].acceleration_body_m_s2,
            )
        )
        np.testing.assert_allclose(
            first[0].magnetic_body,
            np.array([0.28, 0.08, -0.52])
            / np.linalg.norm([0.28, 0.08, -0.52]),
            rtol=0.0,
            atol=1e-15,
        )
        body.sensor_channels[1].next_imu_sample_s = 0.02
        body.last_specific_force_body_m_s2[0] = 15.0
        second = [
            _sensor_frame(
                body,
                scenario,
                rng,
                0.005,
                0.005,
                launch_position,
                False,
                channel,
            )
            for channel in range(3)
        ]
        self.assertEqual(second[1].imu_sample_time_s, 0.0)
        self.assertEqual(second[0].imu_sample_time_s, 0.005)
        self.assertEqual(second[0].magnetometer_sample_time_s, 0.0)

        scenario["faults"] = [
            {
                "body": "integrated_stack",
                "sensor": "imu",
                "channel": 0,
                "type": "freeze",
                "start_s": 0.01,
                "duration_s": 0.01,
            },
            {
                "body": "integrated_stack",
                "sensor": "imu",
                "channel": 1,
                "type": "dropout",
                "start_s": 0.01,
                "duration_s": 0.01,
            },
            {
                "body": "integrated_stack",
                "sensor": "imu",
                "channel": 1,
                "type": "stuck-valid",
                "start_s": 0.01,
                "duration_s": 0.01,
            },
            {
                "body": "integrated_stack",
                "sensor": "imu",
                "channel": 2,
                "type": "stale",
                "start_s": 0.01,
                "duration_s": 0.01,
            },
        ]
        before = [
            state.acceleration_body_m_s2.copy()
            for state in body.sensor_channels
        ]
        faulted = [
            _sensor_frame(
                body,
                scenario,
                rng,
                0.01,
                0.005,
                launch_position,
                False,
                channel,
            )
            for channel in range(3)
        ]
        self.assertTrue(
            np.array_equal(faulted[0].acceleration_body_m_s2, before[0])
        )
        self.assertEqual(faulted[0].imu_sample_time_s, 0.01)
        self.assertTrue(
            np.array_equal(faulted[1].acceleration_body_m_s2, before[1])
        )
        self.assertEqual(faulted[1].imu_sample_time_s, 0.0)
        self.assertEqual(faulted[1].accel_valid, 1)
        self.assertTrue(
            np.array_equal(faulted[2].acceleration_body_m_s2, before[2])
        )
        self.assertEqual(faulted[2].imu_sample_time_s, 0.005)

        value, timestamp, valid = apply_sensor_faults(
            np.array([4.0]),
            np.array([1.0]),
            2.0,
            1.0,
            [
                {"type": "scale_error", "value": 2.0},
                {"type": "bias", "value": 3.0},
            ],
        )
        self.assertEqual(value[0], 11.0)
        self.assertEqual(timestamp, 2.0)
        self.assertEqual(valid, 1)

    def test_fixed_fins_do_not_accept_fsw_commands(self) -> None:
        scenario = default_scenario()
        stage = scenario["vehicle"]["stages"][0]
        body = Body(
            name="integrated_stack",
            stage_index=0,
            stage=stage,
            position_ecef_m=np.zeros(3),
            velocity_ecef_m_s=np.zeros(3),
            attitude_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            body_rates_rad_s=np.zeros(3),
            fuel_kg=stage["fuel_mass_kg"],
            oxidizer_kg=stage["oxidizer_mass_kg"],
        )
        output = FswOutput()
        output.fin_roll_rad = output.fin_pitch_rad = output.fin_yaw_rad = 0.1

        _, fins = actuator_commands(output, scenario, body, 0.0, 0.1)

        self.assertTrue(np.array_equal(fins, np.zeros(3)))

    def test_actuator_commands_are_rate_limited(self) -> None:
        scenario = default_scenario()
        stage = scenario["vehicle"]["stages"][0]
        body = Body(
            name="integrated_stack",
            stage_index=0,
            stage=stage,
            position_ecef_m=np.zeros(3),
            velocity_ecef_m_s=np.zeros(3),
            attitude_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            body_rates_rad_s=np.zeros(3),
            fuel_kg=stage["fuel_mass_kg"],
            oxidizer_kg=stage["oxidizer_mass_kg"],
        )
        output = FswOutput()
        output.tvc_pitch_rad = 1.0
        output.tvc_yaw_rad = -1.0
        dt_s = 0.001
        max_delta = math.radians(
            scenario["actuators"]["max_rate_deg_s"]
        ) * dt_s

        first, _ = actuator_commands(output, scenario, body, 0.0, dt_s)
        second, _ = actuator_commands(output, scenario, body, dt_s, dt_s)

        np.testing.assert_allclose(first, [max_delta, -max_delta])
        np.testing.assert_allclose(second, [2.0 * max_delta, -2.0 * max_delta])

    def test_multi_engine_telemetry_and_engine_cutoff(self) -> None:
        scenario = default_scenario()
        scenario["simulation"]["max_time_s"] = 0.5
        scenario["vehicle"]["stages"][0]["engines"] = [
            {
                "id": "core-left",
                "position_body_m": [0.0, -0.25, 0.0],
                "direction_body": [1.0, 0.0, 0.0],
                "performance_scale": 0.5,
                "enabled": True,
                "gimbal_enabled": True,
            },
            {
                "id": "core-right",
                "position_body_m": [0.0, 0.25, 0.0],
                "direction_body": [1.0, 0.0, 0.0],
                "performance_scale": 0.5,
                "enabled": True,
                "gimbal_enabled": True,
            },
        ]
        scenario["faults"] = [
            {
                "body": "integrated_stack",
                "component": "engine",
                "engine_id": "core-right",
                "type": "cutoff",
                "start_s": 0.0,
                "duration_s": 1.0,
            }
        ]

        result = run_simulation(
            scenario,
            seed=1,
            create_report=False,
            persist=False,
        )
        row = next(row for row in result.telemetry if float(row["thrust_n"]) > 0.0)
        engines = json.loads(row["engine_thrusts_n"])

        self.assertEqual(row["engine_count"], 2)
        self.assertEqual(row["active_engines"], 1)
        self.assertEqual(engines["core-right"], 0.0)
        self.assertGreater(engines["core-left"], 0.0)
        self.assertAlmostEqual(
            float(row["thrust_n"]),
            float(engines["core-left"]),
            places=6,
        )

    def test_stream_can_cancel_without_waiting_for_batch_result(self) -> None:
        scenario = default_scenario()
        streamed: list[dict] = []

        result = run_simulation(
            scenario,
            seed=3,
            create_report=False,
            persist=False,
            on_sample=lambda _time, rows, _events: streamed.extend(rows),
            should_cancel=lambda: bool(streamed),
        )

        self.assertEqual(result.manifest["status"], "CANCELLED")
        self.assertTrue(result.manifest["cancelled"])
        self.assertEqual(streamed[0]["body"], "integrated_stack")
        self.assertTrue(
            any(event["event"] == "simulation_cancelled" for event in result.events)
        )

    def test_fsw_timing_modes(self) -> None:
        scenario = default_scenario()
        scenario["simulation"].update(
            {"max_time_s": 0.02, "output_rate_hz": 200.0}
        )

        deterministic = run_simulation(
            scenario, create_report=False, persist=False
        )
        timer_ns = iter(range(0, 1_000_000_000, 20_000_000))
        with mock.patch(
            "aerospace_workbench.flight_software.timing.time.perf_counter_ns",
            side_effect=lambda: next(timer_ns),
        ):
            measured = run_simulation(
                scenario,
                create_report=False,
                persist=False,
                timing_mode="measured",
            )
        injected = run_simulation(
            scenario,
            create_report=False,
            persist=False,
            timing_mode="injected",
            injected_execution_time_s=0.02,
        )

        self.assertEqual(deterministic.manifest["fsw_timing"]["mode"], "deterministic")
        self.assertTrue(
            all(
                row["previous_execution_time_s"] == 0.0
                for row in deterministic.fsw_telemetry
            )
        )
        self.assertTrue(
            set(FSW_SENSOR_DIAGNOSTIC_FIELDS)
            <= deterministic.fsw_telemetry[0].keys()
        )
        self.assertTrue(
            any(
                row["accelerometer_usable_mask"] > 0
                for row in deterministic.fsw_telemetry
            )
        )
        self.assertTrue(
            any(
                row["previous_execution_time_s"] > 0.01
                for row in measured.fsw_telemetry
            )
        )
        injected_execution_times = {
            row["previous_execution_time_s"]
            for row in injected.fsw_telemetry
        }
        self.assertEqual(injected_execution_times, {0.0, 0.02})
        self.assertGreater(
            max(row["consecutive_overruns"] for row in injected.fsw_telemetry),
            0,
        )
        self.assertTrue(
            all(
                "DEADLINE_OVERRUN" in row["faults"]
                for row in injected.fsw_telemetry
                if row["previous_execution_time_s"] == 0.02
            )
        )
        with self.assertRaisesRegex(ValueError, "requires"):
            run_simulation(
                scenario,
                create_report=False,
                persist=False,
                timing_mode="injected",
            )

    def test_short_two_stage_run_is_finite_and_reproducible(self) -> None:
        scenario = default_scenario()
        scenario["simulation"].update(
            {"time_step_s": 0.005, "max_time_s": 14.0, "output_rate_hz": 10.0}
        )
        scenario["vehicle"]["stages"][0]["propulsion"]["burn_duration_s"] = 2.0
        scenario["vehicle"]["stages"][1]["propulsion"]["burn_duration_s"] = 2.0
        for stage in scenario["vehicle"]["stages"]:
            stage["propulsion"].pop("performance_curve", None)
        scenario["mission"]["events"][0]["delay"] = 0.2
        scenario["mission"]["events"][1]["delay"] = 0.3
        scenario["faults"] = [
            {
                "body": "upper_stage",
                "sensor": "gnss",
                "type": "dropout",
                "start_s": 3.0,
                "duration_s": 4.0,
            }
        ]
        with tempfile.TemporaryDirectory() as directory:
            first = run_simulation(
                scenario, seed=19, output_root=directory, create_report=False
            )
            second = run_simulation(
                scenario, seed=19, output_root=directory, create_report=False
            )
            self.assertTrue(any(event["event"] == "stage_separation" for event in first.events))
            self.assertTrue(any(event["event"] == "burnout_stage_1" for event in first.events))
            self.assertTrue(any(event["event"] == "stage2_ignition" for event in first.events))
            self.assertTrue(any(row["mode"] == "BOOST_2" for row in first.telemetry))
            self.assertTrue(
                any(
                    row["body"] == "upper_stage"
                    and int(row["fault_flags"]) & 1
                    for row in first.fsw_telemetry
                )
            )
            self.assertTrue(
                any(
                    row["body"] == "upper_stage"
                    and row["navigation_status"] in ("DEGRADED", "INERTIAL")
                    for row in first.fsw_telemetry
                )
            )
            before_separation = [
                row for row in first.fsw_telemetry
                if row["body"] == "integrated_stack"
            ][-1]
            after_separation = next(
                row for row in first.fsw_telemetry
                if row["body"] == "upper_stage"
            )
            attitude_jump = math.sqrt(
                sum(
                    (
                        float(before_separation[f"estimated_attitude_{axis}"])
                        - float(after_separation[f"estimated_attitude_{axis}"])
                    ) ** 2
                    for axis in "wxyz"
                )
            )
            self.assertLess(attitude_jump, 0.005)
            self.assertTrue(
                all(
                    math.isfinite(float(row["altitude_m"]))
                    and math.isfinite(float(row["mass_kg"]))
                    and float(row["mass_kg"]) > 0
                    for row in first.telemetry
                )
            )
            comparable = [
                (row["time_s"], row["body"], round(float(row["altitude_m"]), 8))
                for row in first.telemetry
            ]
            self.assertEqual(
                comparable,
                [
                    (row["time_s"], row["body"], round(float(row["altitude_m"]), 8))
                    for row in second.telemetry
                ],
            )
            self.assertTrue((Path(first.output_dir) / "manifest.json").exists())
            self.assertEqual(
                first.manifest["schema_version"],
                "aerospace-workbench.run-manifest.v1",
            )
            self.assertTrue(
                (Path(first.output_dir) / "vehicle_definition.json").exists()
            )
            self.assertTrue(
                (Path(first.output_dir) / "source_scenario.json").exists()
            )
            self.assertTrue(
                (
                    Path(first.output_dir)
                    / "source_vehicle_definition.json"
                ).exists()
            )
            sensor_path = Path(first.output_dir) / "sensors.csv.gz"
            self.assertTrue(sensor_path.exists())
            self.assertIn("sensors.csv.gz", first.manifest["artifacts"])
            self.assertIn("avionics.csv", first.manifest["artifacts"])
            command_path = Path(first.output_dir) / "commands.csv"
            self.assertTrue(command_path.exists())
            self.assertIn("commands.csv", first.manifest["artifacts"])
            self.assertIn(
                "source_scenario.json", first.manifest["artifacts"]
            )
            self.assertIn(
                "source_vehicle_definition.json",
                first.manifest["artifacts"],
            )
            with gzip.open(
                sensor_path, "rt", newline="", encoding="utf-8"
            ) as file:
                sensor_rows = list(csv.DictReader(file))
            self.assertTrue(sensor_rows)
            self.assertEqual(
                {row["schema_version"] for row in sensor_rows},
                {"aerospace-workbench.sensor-stream.v1"},
            )
            self.assertEqual({row["channel"] for row in sensor_rows}, {"0", "1", "2"})
            with command_path.open(newline="", encoding="utf-8") as file:
                command_rows = list(csv.DictReader(file))
            self.assertEqual(
                [row["command_type"] for row in command_rows],
                ["1", "3"],
            )
            self.assertTrue(
                any(
                    abs(float(row["engine_health_percent"]) - 100.0) > 1e-9
                    for row in sensor_rows
                )
            )
            reloaded = load_scenario(Path(first.output_dir) / "scenario.json")
            self.assertEqual(
                scenario_hash(reloaded),
                first.manifest["scenario_sha256"],
            )


if __name__ == "__main__":
    unittest.main()
