"""Replay recorded sensor frames through ASTARA Flight Core."""

from __future__ import annotations

import csv
import gzip
from pathlib import Path

from .flight_core import MODE_NAMES, FlightCore, sensor_frame_from_row
from .scenario import validate_scenario

REPLAY_FIELDS = (
    "body",
    "time_s",
    "mode",
    "stage_separate",
    "stage2_ignite",
    "deploy_drogue",
    "deploy_main",
    "abort",
    "estimated_altitude_m",
    "estimated_vertical_velocity_m_s",
    "tvc_pitch_rad",
    "tvc_yaw_rad",
    "fin_roll_rad",
    "fin_pitch_rad",
    "fin_yaw_rad",
    "fault_flags",
)


def replay_fsw(
    scenario: dict,
    sensor_log: str | Path,
    output: str | Path | None = None,
) -> Path:
    validate_scenario(scenario)
    sensor_path = Path(sensor_log)
    output_path = (
        Path(output) if output else sensor_path.with_name("fsw_replay.csv")
    )
    if output_path.resolve() == sensor_path.resolve():
        raise ValueError("replay output must differ from sensor log")

    roles = {"integrated_stack": 0, "core_stage": 1, "upper_stage": 2}
    cores: dict[str, FlightCore] = {}
    try:
        source_file = (
            gzip.open(sensor_path, "rt", newline="", encoding="utf-8")
            if sensor_path.suffix == ".gz"
            else sensor_path.open(newline="", encoding="utf-8")
        )
        with (
            source_file as source,
            output_path.open("w", newline="", encoding="utf-8") as destination,
        ):
            reader = csv.DictReader(source)
            writer = csv.DictWriter(destination, fieldnames=REPLAY_FIELDS)
            writer.writeheader()
            for row in reader:
                body = row.get("body", "")
                if body not in roles:
                    raise ValueError(f"unknown replay body {body!r}")
                core = cores.get(body)
                if core is None:
                    core = cores[body] = FlightCore(scenario, roles[body])
                result = core.step(sensor_frame_from_row(row))
                writer.writerow(
                    {
                        "body": body,
                        "time_s": row["time_s"],
                        "mode": MODE_NAMES[result.mode],
                        "stage_separate": result.stage_separate,
                        "stage2_ignite": result.stage2_ignite,
                        "deploy_drogue": result.deploy_drogue,
                        "deploy_main": result.deploy_main,
                        "abort": result.abort,
                        "estimated_altitude_m": result.estimated_altitude_m,
                        "estimated_vertical_velocity_m_s": (
                            result.estimated_vertical_velocity_m_s
                        ),
                        "tvc_pitch_rad": result.tvc_pitch_rad,
                        "tvc_yaw_rad": result.tvc_yaw_rad,
                        "fin_roll_rad": result.fin_roll_rad,
                        "fin_pitch_rad": result.fin_pitch_rad,
                        "fin_yaw_rad": result.fin_yaw_rad,
                        "fault_flags": result.fault_flags,
                    }
                )
    finally:
        for core in cores.values():
            core.close()
    return output_path
