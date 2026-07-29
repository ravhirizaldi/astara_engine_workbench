import tempfile
import unittest
from pathlib import Path

from aerospace_workbench.configuration.scenarios import default_scenario
from aerospace_workbench.adapters.rocketpy import run_rocketpy_reference
from aerospace_workbench.simulation.runner import run_simulation


class RocketPyAdapterTests(unittest.TestCase):
    def test_writes_reference_comparison(self) -> None:
        scenario = default_scenario()
        scenario["simulation"]["max_time_s"] = 8.5
        native = run_simulation(
            scenario,
            seed=1,
            create_report=False,
            persist=False,
        )

        with tempfile.TemporaryDirectory() as directory:
            comparison = run_rocketpy_reference(
                scenario,
                native.telemetry,
                directory,
            )

            self.assertEqual(comparison["status"], "REFERENCE_ONLY_UNVALIDATED")
            self.assertEqual(
                comparison["schema_version"],
                "aerospace-workbench.rocketpy-reference.v1",
            )
            self.assertGreater(comparison["rocketpy"]["altitude_m"], 0.0)
            self.assertTrue((Path(directory) / "rocketpy_reference.json").exists())


if __name__ == "__main__":
    unittest.main()
