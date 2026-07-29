import copy
import json
import tempfile
import unittest
import warnings
from pathlib import Path

from aerospace_workbench.configuration.scenarios import (
    default_scenario_path,
    default_scenario,
    load_scenario_documents,
    load_scenario,
    resolve_scenario_path,
    resolve_mission_events,
    scenario_hash,
)
from aerospace_workbench.configuration.schemas import (
    LEGACY_SCENARIO_SCHEMA_VERSION as LEGACY_SCHEMA_VERSION,
    SCENARIO_SCHEMA_VERSION as SCHEMA_VERSION,
    VEHICLE_SCHEMA_VERSION,
    SchemaMigrationWarning,
)
from aerospace_workbench.configuration.validation import validate_scenario
from aerospace_workbench.configuration.vehicles import evidence_documents
from aerospace_workbench.evidence.artifacts import (
    write_configuration_artifacts,
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

    def test_default_and_legacy_repository_paths_resolve_to_configs(self) -> None:
        self.assertEqual(default_scenario_path().parent.name, "scenarios")
        self.assertEqual(default_scenario_path().parent.parent.name, "configs")
        with self.assertWarns(SchemaMigrationWarning):
            resolved = resolve_scenario_path(
                "scenarios/anthariksa_reference_mission.json"
            )
        self.assertEqual(resolved, default_scenario_path())

    def test_legacy_files_warn_normalize_and_preserve_source(self) -> None:
        canonical_scenario = json.loads(
            default_scenario_path().read_text(encoding="utf-8")
        )
        canonical_vehicle_path = (
            default_scenario_path().parent
            / canonical_scenario["vehicle_definition"]
        ).resolve()
        canonical_vehicle = json.loads(
            canonical_vehicle_path.read_text(encoding="utf-8")
        )
        canonical_scenario["schema_version"] = "c1.scenario.v1"
        canonical_vehicle["schema_version"] = "astara.vehicle.v1"

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario_dir = root / "scenarios"
            vehicle_dir = root / "vehicles"
            scenario_dir.mkdir()
            vehicle_dir.mkdir()
            scenario_path = scenario_dir / "legacy.json"
            vehicle_path = vehicle_dir / "legacy_vehicle.json"
            canonical_scenario["vehicle_definition"] = (
                "../vehicles/legacy_vehicle.json"
            )
            scenario_path.write_text(
                json.dumps(canonical_scenario, indent=4) + "\n",
                encoding="utf-8",
            )
            vehicle_path.write_text(
                json.dumps(canonical_vehicle, separators=(",", ":")),
                encoding="utf-8",
            )
            source_scenario = scenario_path.read_bytes()
            source_vehicle = vehicle_path.read_bytes()

            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                scenario = load_scenario(scenario_path)
            self.assertEqual(len(caught), 2)
            self.assertTrue(
                all(
                    issubclass(warning.category, SchemaMigrationWarning)
                    for warning in caught
                )
            )
            self.assertEqual(scenario["schema_version"], SCHEMA_VERSION)

            evidence_dir = root / "evidence"
            evidence_dir.mkdir()
            paths = write_configuration_artifacts(evidence_dir, scenario)
            self.assertEqual(
                (evidence_dir / "source_scenario.json").read_bytes(),
                source_scenario,
            )
            self.assertEqual(
                (
                    evidence_dir / "source_vehicle_definition.json"
                ).read_bytes(),
                source_vehicle,
            )
            self.assertEqual(
                json.loads(
                    (evidence_dir / "scenario.json").read_text(
                        encoding="utf-8"
                    )
                )["schema_version"],
                SCHEMA_VERSION,
            )
            self.assertEqual(
                json.loads(
                    (evidence_dir / "vehicle_definition.json").read_text(
                        encoding="utf-8"
                    )
                )["schema_version"],
                VEHICLE_SCHEMA_VERSION,
            )
            self.assertEqual({path.name for path in paths}, {
                "scenario.json",
                "vehicle_definition.json",
                "source_scenario.json",
                "source_vehicle_definition.json",
            })

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

    def test_accepts_legacy_mission_delays(self) -> None:
        scenario = default_scenario()
        scenario["schema_version"] = LEGACY_SCHEMA_VERSION
        scenario.pop("vehicle_definition")
        scenario["mission"].pop("events")
        scenario["mission"].update(
            {"separation_delay_s": 0.5, "stage2_ignition_delay_s": 1.0}
        )

        with self.assertWarns(SchemaMigrationWarning):
            validate_scenario(scenario)
        self.assertEqual(scenario["schema_version"], SCHEMA_VERSION)
        self.assertEqual(resolve_mission_events(scenario)["stage2_ignition"], 9.5)


if __name__ == "__main__":
    unittest.main()
