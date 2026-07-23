import json
import math
import tempfile
import unittest
from pathlib import Path

from astara.scenario import default_scenario, load_scenario, scenario_hash
from astara.twin import run_simulation


class TwinTests(unittest.TestCase):
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
            self.assertTrue(
                (Path(first.output_dir) / "vehicle_definition.json").exists()
            )
            reloaded = load_scenario(Path(first.output_dir) / "scenario.json")
            self.assertEqual(
                scenario_hash(reloaded),
                first.manifest["scenario_sha256"],
            )


if __name__ == "__main__":
    unittest.main()
