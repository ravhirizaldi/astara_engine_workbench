import csv
import gzip
import tempfile
import unittest
from pathlib import Path

from astara.flight_core import (
    SENSOR_CSV_FIELDS,
    SensorFrame,
    sensor_frame_from_row,
    sensor_frame_to_row,
)
from astara.replay import replay_fsw
from astara.scenario import default_scenario


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
                    )
                    writer.writerow(sensor_frame_to_row("integrated_stack", frame))
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

            first = replay_fsw(scenario, sensor_path, Path(directory) / "first.csv")
            second = replay_fsw(scenario, sensor_path, Path(directory) / "second.csv")

            self.assertEqual(first.read_bytes(), second.read_bytes())
            with first.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 40)
            self.assertEqual(rows[-1]["mode"], "BOOST_1")
            self.assertEqual(rows[-1]["command_source"], "recorded")

    def test_old_sensor_rows_default_sample_times_to_frame_time(self) -> None:
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
        row.pop("barometer_sample_time_s")
        row.pop("gnss_sample_time_s")
        restored = sensor_frame_from_row(row)
        self.assertEqual(restored.barometer_sample_time_s, 1.5)
        self.assertEqual(restored.gnss_sample_time_s, 1.5)


if __name__ == "__main__":
    unittest.main()
