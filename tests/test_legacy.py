import unittest

from aerospace_workbench.presentation.desktop import engine_bench


class LegacyRegressionTests(unittest.TestCase):
    def test_default_engine_regression(self) -> None:
        results = engine_bench.simulate(engine_bench.default_config())
        self.assertEqual(len(results["time"]), 1101)
        self.assertAlmostEqual(float(results["engine_health_percent"][-1]), 91.4378, places=3)
        self.assertIn("WARNING", set(results["safety_state"]))


if __name__ == "__main__":
    unittest.main()
