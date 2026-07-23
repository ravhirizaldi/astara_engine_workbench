import json
import tempfile
import unittest
from pathlib import Path

from astara.analysis import run_credibility_analysis
from astara.scenario import default_scenario
from astara.twin import run_simulation


class CredibilityAnalysisTests(unittest.TestCase):
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
        self.assertEqual(result.manifest["seed"], 4)
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
            summary = json.loads(
                (Path(output_dir) / "summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(summary["samples"], 2)
            self.assertEqual(summary["seed"], 7)
            self.assertIn("maximum_altitude_m", summary["monte_carlo"])
            self.assertTrue((Path(output_dir) / "convergence.csv").exists())
            self.assertTrue((Path(output_dir) / "monte_carlo.csv").exists())
            self.assertTrue((Path(output_dir) / "vehicle_definition.json").exists())


if __name__ == "__main__":
    unittest.main()
