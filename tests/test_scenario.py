import copy
import unittest

from astara.scenario import (
    LEGACY_SCHEMA_VERSION,
    SCHEMA_VERSION,
    default_scenario,
    evidence_documents,
    load_scenario_documents,
    resolve_mission_events,
    scenario_hash,
    validate_scenario,
)


class ScenarioTests(unittest.TestCase):
    def test_default_scenario_is_valid_and_stable(self) -> None:
        scenario = default_scenario()
        validate_scenario(scenario)
        self.assertEqual(scenario["schema_version"], SCHEMA_VERSION)
        self.assertEqual(len(scenario["vehicle"]["stages"]), 2)
        self.assertEqual(scenario_hash(scenario), scenario_hash(copy.deepcopy(scenario)))
        self.assertEqual(
            resolve_mission_events(scenario),
            {
                "burnout_stage_1": 8.0,
                "stage_separation": 8.5,
                "stage2_ignition": 9.5,
            },
        )

    def test_scenario_and_vehicle_documents_are_separate(self) -> None:
        scenario_document, vehicle_document, vehicle_path = load_scenario_documents()

        self.assertNotIn("vehicle", scenario_document)
        self.assertNotIn("sensors", scenario_document)
        self.assertNotIn("actuators", scenario_document)
        self.assertIsNotNone(vehicle_document)
        self.assertIn("vehicle", vehicle_document)
        self.assertTrue(vehicle_path.is_file())

        resolved = default_scenario()
        evidence_scenario, evidence_vehicle = evidence_documents(resolved)
        self.assertNotIn("vehicle", evidence_scenario)
        self.assertEqual(
            evidence_scenario["vehicle_definition"],
            "vehicle_definition.json",
        )
        self.assertIn("vehicle", evidence_vehicle)

    def test_rejects_nonpositive_timestep(self) -> None:
        scenario = default_scenario()
        scenario["simulation"]["time_step_s"] = 0.0
        with self.assertRaisesRegex(ValueError, "time_step_s"):
            validate_scenario(scenario)

    def test_rejects_truth_step_slower_than_flight_software(self) -> None:
        scenario = default_scenario()
        scenario["simulation"]["time_step_s"] = 0.01
        with self.assertRaisesRegex(ValueError, "flight-software period"):
            validate_scenario(scenario)

    def test_rejects_arbitrary_stage_count(self) -> None:
        scenario = default_scenario()
        scenario["vehicle"]["stages"].append(copy.deepcopy(scenario["vehicle"]["stages"][1]))
        with self.assertRaisesRegex(ValueError, "exactly two"):
            validate_scenario(scenario)

    def test_rejects_curve_that_does_not_cover_burn(self) -> None:
        scenario = default_scenario()
        scenario["vehicle"]["stages"][0]["propulsion"]["performance_curve"][-1][
            "time_s"
        ] = 7.9
        with self.assertRaisesRegex(ValueError, "must end at burn_duration_s"):
            validate_scenario(scenario)

    def test_accepts_symmetric_engine_cluster(self) -> None:
        scenario = default_scenario()
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

        validate_scenario(scenario)

    def test_rejects_duplicate_engine_ids(self) -> None:
        scenario = default_scenario()
        duplicate = copy.deepcopy(scenario["vehicle"]["stages"][0]["engines"][0])
        scenario["vehicle"]["stages"][0]["engines"].append(duplicate)

        with self.assertRaisesRegex(ValueError, "duplicate id"):
            validate_scenario(scenario)

    def test_rejects_cyclic_mission_events(self) -> None:
        scenario = default_scenario()
        scenario["mission"]["events"] = [
            {
                "event": "stage_separation",
                "trigger": "stage2_ignition",
                "delay": 0.5,
            },
            {
                "event": "stage2_ignition",
                "trigger": "stage_separation",
                "delay": 1.0,
            },
        ]

        with self.assertRaisesRegex(ValueError, "cyclic or unknown"):
            validate_scenario(scenario)

    def test_accepts_legacy_mission_delays(self) -> None:
        scenario = default_scenario()
        scenario["schema_version"] = LEGACY_SCHEMA_VERSION
        scenario.pop("vehicle_definition")
        scenario["mission"].pop("events")
        scenario["mission"].update(
            {"separation_delay_s": 0.5, "stage2_ignition_delay_s": 1.0}
        )

        validate_scenario(scenario)
        self.assertEqual(resolve_mission_events(scenario)["stage2_ignition"], 9.5)


if __name__ == "__main__":
    unittest.main()
