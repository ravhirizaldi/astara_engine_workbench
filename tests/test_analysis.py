import csv
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from aerospace_workbench import __version__
from aerospace_workbench.evidence.analysis import (
    _metrics,
    run_credibility_analysis,
    worker_count,
)
from aerospace_workbench.evidence.retention import selected_success_samples
from aerospace_workbench.configuration.scenarios import default_scenario
from aerospace_workbench.simulation.runner import RunResult, run_simulation


class CredibilityAnalysisTests(unittest.TestCase):
    def test_worker_budget_reserves_quarter_of_available_cpus(self) -> None:
        with patch(
            "aerospace_workbench.evidence.analysis.available_cpu_count",
            return_value=8,
        ):
            self.assertEqual(worker_count(20), (8, 6))
            self.assertEqual(worker_count(4), (8, 4))
            self.assertEqual(worker_count(20, requested=1), (8, 1))
            with self.assertRaisesRegex(ValueError, "cannot exceed 8"):
                worker_count(20, requested=9)

    def test_successful_telemetry_selection_is_seeded(self) -> None:
        rows = [
            {"sample": sample, "status": "PASS"} for sample in range(1, 101)
        ]
        statuses = frozenset({"PASS"})
        first = selected_success_samples(
            rows, seed=7, percent=2.0, success_statuses=statuses
        )
        second = selected_success_samples(
            rows, seed=7, percent=2.0, success_statuses=statuses
        )
        self.assertEqual(first, second)
        self.assertEqual(len(first), 2)

    def test_metrics_include_burnout_impact_and_failure_reason(self) -> None:
        result = RunResult(
            Path("."),
            {
                "maximum_altitude_m": 1234.0,
                "aero_out_of_envelope_samples": 0,
                "aero_out_of_envelope_pre_recovery_samples": 0,
                "duration_s": 30.0,
                "status": "FAIL",
                "checks": {"landed": False, "finite": True},
                "impact_points": {
                    "core_stage": {
                        "latitude_deg": -6.2,
                        "longitude_deg": 106.8,
                    },
                    "upper_stage": {
                        "latitude_deg": -6.1,
                        "longitude_deg": 107.0,
                    },
                },
            },
            [
                {
                    "mach": 3.0,
                    "dynamic_pressure_pa": 40_000.0,
                    "angle_of_attack_deg": 2.0,
                    "engine_health_percent": 95.0,
                }
            ],
            [],
            [
                {"event": "burnout_stage_1", "time_s": 10.0},
                {"event": "burnout_stage_2", "time_s": 20.0},
            ],
        )
        metrics = _metrics(result)
        self.assertEqual(metrics["stage1_burnout_time_s"], 10.0)
        self.assertEqual(metrics["stage2_burnout_time_s"], 20.0)
        self.assertEqual(metrics["core_impact_longitude_deg"], 106.8)
        self.assertEqual(metrics["upper_impact_latitude_deg"], -6.1)
        self.assertEqual(metrics["failure_reason"], "landed")

    def test_orbit_metrics_use_insertion_as_final_stage2_cutoff(self) -> None:
        result = RunResult(
            Path("."),
            {
                "maximum_altitude_m": 215_000.0,
                "aero_out_of_envelope_samples": 0,
                "aero_out_of_envelope_pre_recovery_samples": 0,
                "duration_s": 300.0,
                "status": "FAIL",
                "checks": {"orbit_insertion": False},
            },
            [],
            [],
            [{"event": "stage2_first_cutoff", "time_s": 190.0}],
        )

        self.assertIsNone(_metrics(result)["stage2_burnout_time_s"])
        result.events.append({"event": "orbit_insertion", "time_s": 250.0})
        self.assertEqual(_metrics(result)["stage2_burnout_time_s"], 250.0)

    def test_simulation_records_both_burnout_events(self) -> None:
        scenario = default_scenario()
        scenario["simulation"].update(
            {"max_time_s": 6.0, "output_rate_hz": 1.0}
        )
        for stage in scenario["vehicle"]["stages"]:
            stage["propulsion"]["burn_duration_s"] = 2.0
            stage["propulsion"].pop("performance_curve", None)
        scenario["mission"]["flight_core"].update(
            separation_delay_s=0.2,
            stage2_ignition_delay_s=0.3,
            stage2_first_burn_s=1.0,
        )
        scenario["mission"]["orbit"]["enabled"] = False
        result = run_simulation(
            scenario,
            create_report=False,
            persist=False,
            summary_only=True,
        )
        events = {event["event"] for event in result.events}
        self.assertIn("meco", events)
        self.assertIn("stage2_first_cutoff", events)
        self.assertIsNotNone(_metrics(result)["stage2_burnout_time_s"])

    def test_tabulated_models_and_short_analysis(self) -> None:
        scenario = default_scenario()
        scenario["simulation"].update(
            {
                "max_time_s": 0.4,
                "time_step_s": 0.005,
                "output_rate_hz": 20.0,
                "seed": 4,
            }
        )
        result = run_simulation(
            scenario, create_report=False, persist=False
        )
        compact_result = run_simulation(
            scenario,
            create_report=False,
            persist=False,
            summary_only=True,
        )
        self.assertEqual(compact_result.telemetry, [])
        self.assertEqual(compact_result.fsw_telemetry, [])
        self.assertEqual(_metrics(compact_result), _metrics(result))
        self.assertEqual(result.manifest["seed"], 4)
        self.assertEqual(result.manifest["model_version"], __version__)
        self.assertEqual(
            result.manifest["model_configuration"]["propulsion"],
            ["tabulated_curve", "tabulated_curve"],
        )
        self.assertGreater(max(row["thrust_n"] for row in result.telemetry), 0.0)
        self.assertTrue(
            all(
                row["center_of_mass_m"] > 0.0
                and row["inertia_y_kg_m2"] > 0.0
                for row in result.telemetry
            )
        )

        scenario["monte_carlo"].update({"samples": 2, "seed": 7})
        with tempfile.TemporaryDirectory() as directory:
            output_dir = run_credibility_analysis(
                scenario, output_root=directory
            )
            output_dir = Path(output_dir)
            summary = json.loads(
                (output_dir / "summary.json").read_text(encoding="utf-8")
            )
            with (output_dir / "monte_carlo.csv").open(
                encoding="utf-8"
            ) as stream:
                rows = list(csv.DictReader(stream))
            with (output_dir / "retained_runs.csv").open(
                encoding="utf-8"
            ) as stream:
                retained = list(csv.DictReader(stream))
            serial_dir = run_credibility_analysis(
                scenario,
                output_root=directory,
                workers=1,
                telemetry_sample_percent=0.0,
            )
            with (Path(serial_dir) / "monte_carlo.csv").open(
                encoding="utf-8"
            ) as stream:
                serial_rows = list(csv.DictReader(stream))
            compact = lambda row: {
                name: value
                for name, value in row.items()
                if not name.startswith("retention")
                and name not in {"telemetry_retained", "telemetry_path"}
            }
            self.assertEqual(
                [compact(row) for row in rows],
                [compact(row) for row in serial_rows],
            )
            self.assertEqual(
                summary["schema_version"],
                "aerospace-workbench.credibility.v2",
            )
            self.assertEqual(summary["samples"], 2)
            self.assertEqual(summary["seed"], 7)
            self.assertEqual(summary["model_version"], __version__)
            self.assertGreaterEqual(summary["execution"]["workers"], 1)
            self.assertLessEqual(summary["execution"]["workers"], 2)
            self.assertEqual(summary["telemetry_retention"]["failure_runs"], 2)
            self.assertEqual(summary["telemetry_retention"]["retained_runs"], 2)
            self.assertIn("maximum_altitude_m", summary["monte_carlo"])
            self.assertEqual(len(rows), 2)
            self.assertIn("stage2_burnout_time_s", rows[0])
            self.assertIn("upper_impact_longitude_deg", rows[0])
            self.assertEqual(len(retained), 2)
            for retained_row in retained:
                run_dir = output_dir / retained_row["path"]
                for filename in (
                    "manifest.json",
                    "sensors.csv.gz",
                    "truth.csv",
                    "fsw.csv",
                    "events.csv",
                ):
                    self.assertTrue((run_dir / filename).exists())
            self.assertTrue((output_dir / "convergence.csv").exists())
            self.assertTrue((output_dir / "vehicle_definition.json").exists())
            self.assertTrue((output_dir / "source_scenario.json").exists())
            self.assertTrue(
                (output_dir / "source_vehicle_definition.json").exists()
            )
            self.assertIn("source_scenario.json", summary["artifacts"])
            self.assertIn(
                "source_vehicle_definition.json", summary["artifacts"]
            )


if __name__ == "__main__":
    unittest.main()
