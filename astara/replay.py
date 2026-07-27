"""Replay recorded sensor frames through the generic flight core."""

from __future__ import annotations

import csv
import gzip
import itertools
from pathlib import Path

from .flight_core import (
    FSW_BODY_CORE,
    FSW_BODY_INTEGRATED,
    FSW_BODY_UPPER,
    MODE_NAMES,
    NAVIGATION_STATUS_NAMES,
    FlightCore,
    decode_faults,
    sensor_frame_from_row,
)
from .scenario import validate_scenario

REPLAY_FIELDS = (
    "body",
    "time_s",
    "mode",
    "navigation_status",
    "stage_separate",
    "stage2_ignite",
    "deploy_drogue",
    "deploy_main",
    "abort",
    "estimated_altitude_m",
    "estimated_vertical_velocity_m_s",
    "estimated_position_ecef_x_m",
    "estimated_position_ecef_y_m",
    "estimated_position_ecef_z_m",
    "estimated_velocity_ecef_x_m_s",
    "estimated_velocity_ecef_y_m_s",
    "estimated_velocity_ecef_z_m_s",
    "discrete_actuation_sequence",
    "discrete_actuation_action",
    "tvc_pitch_rad",
    "tvc_yaw_rad",
    "fin_roll_rad",
    "fin_pitch_rad",
    "fin_yaw_rad",
    "fault_flags",
    "active_fault_flags",
    "latched_fault_flags",
    "highest_fault_severity",
    "altitude_sigma_m",
    "vertical_velocity_sigma_m_s",
    "command_sequence",
    "command_type",
    "command_result",
    "inhibit_flags",
    "command_source",
    "faults",
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

    roles = {
        "integrated_stack": FSW_BODY_INTEGRATED,
        "core_stage": FSW_BODY_CORE,
        "upper_stage": FSW_BODY_UPPER,
    }
    command_path = sensor_path.with_name("commands.csv")
    recorded_commands: dict[tuple[str, float], int] = {}
    if command_path.exists():
        with command_path.open(newline="", encoding="utf-8") as command_file:
            for command_row in csv.DictReader(command_file):
                recorded_commands[
                    (
                        command_row["body"],
                        round(float(command_row["time_s"]), 9),
                    )
                ] = int(command_row["command_type"])
    command_source = "recorded" if recorded_commands else "legacy_synthesized"
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
            grouped_rows = itertools.groupby(
                reader,
                key=lambda item: (
                    item.get("body", ""),
                    round(float(item["time_s"]), 9),
                ),
            )
            for _, row_group in grouped_rows:
                rows = list(row_group)
                row = rows[0]
                body = row.get("body", "")
                if body not in roles:
                    raise ValueError(f"unknown replay body {body!r}")
                core = cores.get(body)
                if core is None:
                    if body == "upper_stage" and "integrated_stack" in cores:
                        core = cores.pop("integrated_stack")
                    else:
                        core = FlightCore(
                            scenario,
                            roles[body],
                            auto_commands=not bool(recorded_commands),
                        )
                    cores[body] = core
                frames = [
                    sensor_frame_from_row(channel_row)
                    for channel_row in sorted(
                        rows, key=lambda item: int(item.get("channel", 0))
                    )
                ]
                frame = frames[0]
                result = core.step(
                    frame,
                    command_type=(
                        recorded_commands.get(
                            (body, round(float(row["time_s"]), 9)),
                            0,
                        )
                        if recorded_commands
                        else None
                    ),
                    sensor_channels=frames[1:],
                    previous_execution_time_s=0.0,
                    deadline_missed=False,
                )
                writer.writerow(
                    {
                        "body": body,
                        "time_s": row["time_s"],
                        "mode": MODE_NAMES[result.mode],
                        "navigation_status": NAVIGATION_STATUS_NAMES[
                            result.navigation_status
                        ],
                        "stage_separate": result.stage_separate,
                        "stage2_ignite": result.stage2_ignite,
                        "deploy_drogue": result.deploy_drogue,
                        "deploy_main": result.deploy_main,
                        "abort": result.abort,
                        "estimated_altitude_m": result.estimated_altitude_m,
                        "estimated_vertical_velocity_m_s": (
                            result.estimated_vertical_velocity_m_s
                        ),
                        "estimated_position_ecef_x_m": (
                            result.estimated_position_ecef_m[0]
                        ),
                        "estimated_position_ecef_y_m": (
                            result.estimated_position_ecef_m[1]
                        ),
                        "estimated_position_ecef_z_m": (
                            result.estimated_position_ecef_m[2]
                        ),
                        "estimated_velocity_ecef_x_m_s": (
                            result.estimated_velocity_ecef_m_s[0]
                        ),
                        "estimated_velocity_ecef_y_m_s": (
                            result.estimated_velocity_ecef_m_s[1]
                        ),
                        "estimated_velocity_ecef_z_m_s": (
                            result.estimated_velocity_ecef_m_s[2]
                        ),
                        "discrete_actuation_sequence": (
                            result.discrete_actuation.sequence
                        ),
                        "discrete_actuation_action": (
                            result.discrete_actuation.action
                        ),
                        "tvc_pitch_rad": result.tvc_pitch_rad,
                        "tvc_yaw_rad": result.tvc_yaw_rad,
                        "fin_roll_rad": result.fin_roll_rad,
                        "fin_pitch_rad": result.fin_pitch_rad,
                        "fin_yaw_rad": result.fin_yaw_rad,
                        "fault_flags": int(
                            result.active_fault_flags
                            | result.latched_fault_flags
                        ),
                        "active_fault_flags": result.active_fault_flags,
                        "latched_fault_flags": result.latched_fault_flags,
                        "highest_fault_severity": (
                            result.highest_fault_severity
                        ),
                        "altitude_sigma_m": result.altitude_sigma_m,
                        "vertical_velocity_sigma_m_s": (
                            result.vertical_velocity_sigma_m_s
                        ),
                        "command_sequence": result.command_sequence,
                        "command_type": result.command_type,
                        "command_result": result.command_result,
                        "inhibit_flags": result.inhibit_flags,
                        "command_source": command_source,
                        "faults": decode_faults(
                            result.active_fault_flags
                            | result.latched_fault_flags
                        ),
                    }
                )
    finally:
        for core in cores.values():
            core.close()
    return output_path
