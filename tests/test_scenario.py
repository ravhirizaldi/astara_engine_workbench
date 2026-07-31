import copy
import unittest

from aerospace_workbench.configuration.scenarios import (
    default_scenario_path,
    default_scenario,
    load_scenario,
    load_scenario_documents,
    resolve_mission_events,
    scenario_hash,
)
from aerospace_workbench.configuration.schemas import (
    SCENARIO_SCHEMA_VERSION as SCHEMA_VERSION,
)
from aerospace_workbench.configuration.validation import validate_scenario
from aerospace_workbench.configuration.vehicles import evidence_documents


class ScenarioTests(unittest.TestCase):
    def test_all_catalog_scenarios_are_valid(self) -> None:
        paths = sorted(default_scenario_path().parent.glob("*.json"))
        self.assertGreater(len(paths), 1)
        for path in paths:
            with self.subTest(path=path.name):
                validate_scenario(load_scenario(path))

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
        self.assertEqual(
            evidence_vehicle["name"], "Anthariksa reference vehicle"
        )
        self.assertEqual(evidence_vehicle["engine_family"], "Cendrawasih")
        metadata_changed = copy.deepcopy(resolved)
        metadata_changed.vehicle_document["engine_family"] = "Other"
        self.assertNotEqual(
            scenario_hash(resolved), scenario_hash(metadata_changed)
        )

    def test_default_repository_path_resolves_to_configs(self) -> None:
        self.assertEqual(default_scenario_path().parent.name, "scenarios")
        self.assertEqual(default_scenario_path().parent.parent.name, "configs")

    def test_unknown_schema_is_rejected(self) -> None:
        scenario = default_scenario()
        scenario["schema_version"] = "unknown.scenario.v1"
        with self.assertRaisesRegex(ValueError, "unknown.scenario.v1"):
            validate_scenario(scenario)

    def test_rejects_nonpositive_timestep(self) -> None:
        scenario = default_scenario()
        scenario["simulation"]["time_step_s"] = 0.0
        with self.assertRaisesRegex(ValueError, "time_step_s"):
            validate_scenario(scenario)

    def test_rejects_invalid_runtime_settings(self) -> None:
        cases = (
            ("simulation", "output_rate_hz", 0.0, "output_rate_hz"),
            ("simulation", "output_rate_hz", 201.0, "output_rate_hz"),
            ("actuators", "tvc_kp", 0.0, "tvc_kp"),
            ("actuators", "tvc_kd", 0.0, "tvc_kd"),
            ("environment", "wind_ned_m_s", [0.0, 0.0], "wind_ned_m_s"),
            (
                "environment",
                "wind_ned_m_s",
                [0.0, "east", 0.0],
                "wind_ned_m_s",
            ),
        )
        for section, name, value, message in cases:
            with self.subTest(section=section, name=name, value=value):
                scenario = default_scenario()
                scenario[section][name] = value
                with self.assertRaisesRegex(ValueError, message):
                    validate_scenario(scenario)

    def test_rejects_nan_and_infinity_anywhere_in_configuration(self) -> None:
        for value in (float("nan"), float("inf"), -float("inf")):
            scenario = default_scenario()
            scenario["simulation"]["time_step_s"] = value
            with self.assertRaisesRegex(ValueError, "must be finite"):
                validate_scenario(scenario)

        scenario = default_scenario()
        scenario["vehicle"]["stages"][0]["inertia_kg_m2"][0] = float("nan")
        with self.assertRaisesRegex(ValueError, "must be finite"):
            validate_scenario(scenario)

    def test_rejects_invalid_fsw_command_and_channel_count(self) -> None:
        scenario = default_scenario()
        scenario["mission"]["commands"][1]["time_s"] = 0.0
        with self.assertRaisesRegex(ValueError, "commands times"):
            validate_scenario(scenario)

        scenario = default_scenario()
        scenario["sensors"]["channel_count"] = 4
        with self.assertRaisesRegex(ValueError, "channel_count"):
            validate_scenario(scenario)

    def test_rejects_invalid_telemetry_retention_percent(self) -> None:
        scenario = default_scenario()
        scenario["monte_carlo"]["telemetry_sample_percent"] = 101.0
        with self.assertRaisesRegex(ValueError, "telemetry_sample_percent"):
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

    def test_requires_mission_events(self) -> None:
        scenario = default_scenario()
        scenario["mission"].pop("events")
        with self.assertRaisesRegex(ValueError, "mission.events"):
            validate_scenario(scenario)


if __name__ == "__main__":
    unittest.main()
