import csv
import gzip
import tempfile
import unittest
from pathlib import Path

from aerospace_workbench.flight_software.abi import (
    SensorFrame,
)
from aerospace_workbench.flight_software.bridge import (
    FSW_SENSOR_DIAGNOSTIC_FIELDS,
    SENSOR_CSV_FIELDS,
    sensor_frame_from_row,
    sensor_frame_to_row,
)
from aerospace_workbench.configuration.scenarios import default_scenario
from aerospace_workbench.replay.reader import validated_sensor_rows
from aerospace_workbench.replay.runner import replay_fsw


class ReplayTests(unittest.TestCase):
    def test_sensor_log_replays_deterministically(self) -> None:
        scenario = default_scenario()
        vector = SensorFrame._fields_[2][1]
        with tempfile.TemporaryDirectory() as directory:
            sensor_path = Path(directory) / "sensors.csv.gz"
            with gzip.open(
                sensor_path, "wt", newline="", encoding="utf-8"
            ) as file:
                writer = csv.DictWriter(file, fieldnames=SENSOR_CSV_FIELDS)
                writer.writeheader()
                for index in range(40):
                    frame = SensorFrame(
                        index * 0.005,
                        0.005,
                        vector(10.0, 0.0, 0.0),
                        vector(0.0, 0.0, 0.0),
                        vector(1.0, 0.0, 0.0),
                        index * 0.2,
                        vector(6_378_137.0, 0.0, 0.0),
                        vector(20.0, 0.0, 0.0),
                        20.0,
                        100.0,
                        100.0,
                        1,
                        1,
                        0,
                        index * 0.005,
                        index * 0.005,
                        1,
                        int(index > 0),
                        0,
                        0,
                        index * 0.005,
                        index * 0.005,
                        1,
                        1,
                        1,
                    )
                    row = sensor_frame_to_row("integrated_stack", frame)
                    writer.writerow(row)
            with (Path(directory) / "commands.csv").open(
                "w", newline="", encoding="utf-8"
            ) as file:
                writer = csv.DictWriter(
                    file,
                    fieldnames=("body", "time_s", "command_type"),
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "body": "integrated_stack",
                        "time_s": 0.0,
                        "command_type": 1,
                    }
                )
                writer.writerow(
                    {
                        "body": "integrated_stack",
                        "time_s": 0.005,
                        "command_type": 3,
                    }
                )

            first = replay_fsw(
                scenario, sensor_path, Path(directory) / "first.csv"
            )
            second = replay_fsw(
                scenario, sensor_path, Path(directory) / "second.csv"
            )

            self.assertEqual(first.read_bytes(), second.read_bytes())
            with first.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 40)
            self.assertEqual(rows[-1]["mode"], "BOOST_1")
            self.assertEqual(rows[-1]["command_source"], "recorded")
            self.assertTrue(
                set(FSW_SENSOR_DIAGNOSTIC_FIELDS) <= rows[-1].keys()
            )
            self.assertGreater(
                int(rows[-1]["accelerometer_usable_mask"]), 0
            )

    def test_sensor_rows_require_current_schema_and_fields(self) -> None:
        vector = SensorFrame._fields_[2][1]
        frame = SensorFrame(
            1.5,
            0.005,
            vector(10.0, 0.0, 0.0),
            vector(0.0, 0.0, 0.0),
            vector(1.0, 0.0, 0.0),
            10.0,
            vector(6_378_137.0, 0.0, 0.0),
            vector(20.0, 0.0, 0.0),
            20.0,
            100.0,
            100.0,
            1,
            1,
            0,
            1.4,
            1.4,
        )
        row = sensor_frame_to_row("integrated_stack", frame)
        missing_schema = dict(row)
        missing_schema.pop("schema_version")
        with self.assertRaisesRegex(ValueError, "schema_version"):
            list(validated_sensor_rows([missing_schema], Path("sensors.csv")))

        missing_field = dict(row)
        missing_field.pop("imu_sample_time_s")
        with self.assertRaises(KeyError):
            sensor_frame_from_row(missing_field)


if __name__ == "__main__":
    unittest.main()
