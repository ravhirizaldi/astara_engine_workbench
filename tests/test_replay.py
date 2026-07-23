import csv
import gzip
import tempfile
import unittest
from pathlib import Path

from astara.flight_core import (
    SENSOR_CSV_FIELDS,
    SensorFrame,
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
                    )
                    writer.writerow(sensor_frame_to_row("integrated_stack", frame))

            first = replay_fsw(scenario, sensor_path, Path(directory) / "first.csv")
            second = replay_fsw(scenario, sensor_path, Path(directory) / "second.csv")

            self.assertEqual(first.read_bytes(), second.read_bytes())
            with first.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 40)
            self.assertEqual(rows[-1]["mode"], "BOOST_1")


if __name__ == "__main__":
    unittest.main()
