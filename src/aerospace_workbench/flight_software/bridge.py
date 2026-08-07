"""Aerospace Workbench ctypes bridge to the generic flight-software core."""

from __future__ import annotations

import ctypes
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from ..configuration.schemas import SENSOR_STREAM_SCHEMA_VERSION
from .abi import (
    FSW_ABI_VERSION,
    FSW_BODY_CORE,
    FSW_BODY_INTEGRATED,
    FSW_BODY_UPPER,
    FSW_COMMAND_ABORT,
    FSW_COMMAND_ARM,
    FSW_COMMAND_CLEAR_FAULTS,
    FSW_COMMAND_DISARM,
    FSW_COMMAND_LAUNCH,
    FSW_COMMAND_NONE,
    FSW_MAX_GUIDANCE_POINTS,
    FSW_MAX_SENSOR_CHANNELS,
    FswAirDataSample,
    FswBarometerSample,
    FswCommand,
    FswConfig,
    FswDiscreteInputs,
    FswDiscreteSample,
    FswGnssSample,
    FswImuSample,
    FswInput,
    FswMagnetometerSample,
    FswOutput,
    FswPlatformStatus,
    FswPropulsionStatus,
    FswSensorSuite,
    GuidancePoint,
    SensorFrame,
)
from .build import build_library
from .timing import clock_ns

SENSOR_CSV_FIELDS = (
    "schema_version",
    "body",
    "channel",
    "time_s",
    "dt_s",
    "acceleration_x_m_s2",
    "acceleration_y_m_s2",
    "acceleration_z_m_s2",
    "gyro_x_rad_s",
    "gyro_y_rad_s",
    "gyro_z_rad_s",
    "magnetic_x",
    "magnetic_y",
    "magnetic_z",
    "barometric_altitude_m",
    "gnss_position_x_m",
    "gnss_position_y_m",
    "gnss_position_z_m",
    "gnss_velocity_x_m_s",
    "gnss_velocity_y_m_s",
    "gnss_velocity_z_m_s",
    "vertical_velocity_m_s",
    "dynamic_pressure_pa",
    "engine_health_percent",
    "gnss_valid",
    "barometer_valid",
    "stage_separated",
    "barometer_sample_time_s",
    "gnss_sample_time_s",
    "imu_sample_time_s",
    "magnetometer_sample_time_s",
    "accel_valid",
    "gyro_valid",
    "magnetometer_valid",
    "propulsion_ready",
    "propulsion_running",
    "drogue_deployed",
    "main_deployed",
)

FSW_SENSOR_NAMES = (
    "accelerometer",
    "gyroscope",
    "magnetometer",
    "barometer",
    "gnss",
)

FSW_SENSOR_DIAGNOSTIC_FIELDS = (
    *(f"{name}_usable_mask" for name in FSW_SENSOR_NAMES),
    *(f"{name}_rejected_mask" for name in FSW_SENSOR_NAMES),
    "disagreement_flags",
    "sensor_status_flags",
    *(
        f"{name}_health_{channel}"
        for name in FSW_SENSOR_NAMES
        for channel in range(FSW_MAX_SENSOR_CHANNELS)
    ),
    *(
        f"{name}_age_s_{channel}"
        for name in FSW_SENSOR_NAMES
        for channel in range(FSW_MAX_SENSOR_CHANNELS)
    ),
)


@dataclass(frozen=True)
class FswDeviceInputs:
    air_data: FswAirDataSample
    propulsion: FswPropulsionStatus
    discretes: FswDiscreteInputs
    platform: FswPlatformStatus
    command_type: int = FSW_COMMAND_NONE
    command_issue_time_s: float = 0.0


def fsw_sensor_diagnostics_to_row(
    output: FswOutput,
) -> dict[str, float | int]:
    row = {
        **{
            f"{name}_usable_mask": int(
                getattr(output, f"{name}_usable_mask")
            )
            for name in FSW_SENSOR_NAMES
        },
        **{
            f"{name}_rejected_mask": int(
                getattr(output, f"{name}_rejected_mask")
            )
            for name in FSW_SENSOR_NAMES
        },
        "disagreement_flags": int(output.disagreement_flags),
        "sensor_status_flags": int(output.sensor_status_flags),
    }
    for name in FSW_SENSOR_NAMES:
        health = getattr(output, f"{name}_health_flags")
        age = getattr(output, f"{name}_age_s")
        for channel in range(FSW_MAX_SENSOR_CHANNELS):
            row[f"{name}_health_{channel}"] = int(health[channel])
            row[f"{name}_age_s_{channel}"] = float(age[channel])
    return row


def sensor_frame_to_row(
    body: str, frame: SensorFrame, channel: int = 0
) -> dict[str, float | int | str]:
    return {
        "schema_version": SENSOR_STREAM_SCHEMA_VERSION,
        "body": body,
        "channel": channel,
        "time_s": frame.time_s,
        "dt_s": frame.dt_s,
        "acceleration_x_m_s2": frame.acceleration_body_m_s2[0],
        "acceleration_y_m_s2": frame.acceleration_body_m_s2[1],
        "acceleration_z_m_s2": frame.acceleration_body_m_s2[2],
        "gyro_x_rad_s": frame.gyro_body_rad_s[0],
        "gyro_y_rad_s": frame.gyro_body_rad_s[1],
        "gyro_z_rad_s": frame.gyro_body_rad_s[2],
        "magnetic_x": frame.magnetic_body[0],
        "magnetic_y": frame.magnetic_body[1],
        "magnetic_z": frame.magnetic_body[2],
        "barometric_altitude_m": frame.barometric_altitude_m,
        "gnss_position_x_m": frame.gnss_position_ecef_m[0],
        "gnss_position_y_m": frame.gnss_position_ecef_m[1],
        "gnss_position_z_m": frame.gnss_position_ecef_m[2],
        "gnss_velocity_x_m_s": frame.gnss_velocity_ecef_m_s[0],
        "gnss_velocity_y_m_s": frame.gnss_velocity_ecef_m_s[1],
        "gnss_velocity_z_m_s": frame.gnss_velocity_ecef_m_s[2],
        "vertical_velocity_m_s": frame.vertical_velocity_m_s,
        "dynamic_pressure_pa": frame.dynamic_pressure_pa,
        "engine_health_percent": frame.engine_health_percent,
        "gnss_valid": frame.gnss_valid,
        "barometer_valid": frame.barometer_valid,
        "stage_separated": frame.stage_separated,
        "barometer_sample_time_s": frame.barometer_sample_time_s,
        "gnss_sample_time_s": frame.gnss_sample_time_s,
        "imu_sample_time_s": frame.imu_sample_time_s,
        "magnetometer_sample_time_s": frame.magnetometer_sample_time_s,
        "accel_valid": frame.accel_valid,
        "gyro_valid": frame.gyro_valid,
        "magnetometer_valid": frame.magnetometer_valid,
        "propulsion_ready": frame.propulsion_ready,
        "propulsion_running": frame.propulsion_running,
        "drogue_deployed": frame.drogue_deployed,
        "main_deployed": frame.main_deployed,
    }


def sensor_frame_from_row(row: Mapping[str, str]) -> SensorFrame:
    vector = ctypes.c_double * 3
    return SensorFrame(
        float(row["time_s"]),
        float(row["dt_s"]),
        vector(
            float(row["acceleration_x_m_s2"]),
            float(row["acceleration_y_m_s2"]),
            float(row["acceleration_z_m_s2"]),
        ),
        vector(
            float(row["gyro_x_rad_s"]),
            float(row["gyro_y_rad_s"]),
            float(row["gyro_z_rad_s"]),
        ),
        vector(
            float(row["magnetic_x"]),
            float(row["magnetic_y"]),
            float(row["magnetic_z"]),
        ),
        float(row["barometric_altitude_m"]),
        vector(
            float(row["gnss_position_x_m"]),
            float(row["gnss_position_y_m"]),
            float(row["gnss_position_z_m"]),
        ),
        vector(
            float(row["gnss_velocity_x_m_s"]),
            float(row["gnss_velocity_y_m_s"]),
            float(row["gnss_velocity_z_m_s"]),
        ),
        float(row["vertical_velocity_m_s"]),
        float(row["dynamic_pressure_pa"]),
        float(row["engine_health_percent"]),
        int(row["gnss_valid"]),
        int(row["barometer_valid"]),
        int(row["stage_separated"]),
        float(row["barometer_sample_time_s"]),
        float(row["gnss_sample_time_s"]),
        int(row["propulsion_ready"]),
        int(row["propulsion_running"]),
        int(row["drogue_deployed"]),
        int(row["main_deployed"]),
        float(row["imu_sample_time_s"]),
        float(row["magnetometer_sample_time_s"]),
        int(row["accel_valid"]),
        int(row["gyro_valid"]),
        int(row["magnetometer_valid"]),
    )


def sensor_suite_from_frame(frame: SensorFrame) -> FswSensorSuite:
    return sensor_suite_from_frames((frame,))


def sensor_suite_from_frames(
    frames: Sequence[SensorFrame],
) -> FswSensorSuite:
    if not 1 <= len(frames) <= FSW_MAX_SENSOR_CHANNELS:
        raise ValueError(
            f"sensor suite requires 1-{FSW_MAX_SENSOR_CHANNELS} channels"
        )
    first = frames[0]
    imus = (FswImuSample * FSW_MAX_SENSOR_CHANNELS)()
    magnetometers = (
        FswMagnetometerSample * FSW_MAX_SENSOR_CHANNELS
    )()
    barometers = (FswBarometerSample * FSW_MAX_SENSOR_CHANNELS)()
    gnss = (FswGnssSample * FSW_MAX_SENSOR_CHANNELS)()
    for index, frame in enumerate(frames):
        if (
            abs(frame.time_s - first.time_s) > 1e-12
            or abs(frame.dt_s - first.dt_s) > 1e-12
        ):
            raise ValueError("sensor channels must share time_s and dt_s")
        imus[index] = FswImuSample(
            frame.acceleration_body_m_s2,
            frame.gyro_body_rad_s,
            frame.imu_sample_time_s,
            frame.accel_valid,
            frame.gyro_valid,
        )
        magnetometers[index] = FswMagnetometerSample(
            frame.magnetic_body,
            frame.magnetometer_sample_time_s,
            frame.magnetometer_valid,
        )
        barometers[index] = FswBarometerSample(
            frame.barometric_altitude_m,
            frame.barometer_sample_time_s,
            frame.barometer_valid,
        )
        gnss[index] = FswGnssSample(
            frame.gnss_position_ecef_m,
            frame.gnss_velocity_ecef_m_s,
            frame.gnss_sample_time_s,
            frame.gnss_valid,
        )
    return FswSensorSuite(
        first.time_s,
        first.dt_s,
        len(frames),
        len(frames),
        len(frames),
        len(frames),
        imus,
        magnetometers,
        barometers,
        gnss,
    )


def fsw_input_from_frame(
    frame: SensorFrame,
    *,
    device_inputs: FswDeviceInputs | None = None,
    command_type: int = FSW_COMMAND_NONE,
    command_sequence: int = 0,
    command_issue_time_s: float | None = None,
    propulsion_ready: bool | None = None,
    propulsion_running: bool | None = None,
    drogue_deployed: bool | None = None,
    main_deployed: bool | None = None,
    previous_execution_time_s: float = 0.0,
    deadline_missed: bool = False,
    watchdog_healthy: bool = True,
) -> FswInput:
    input_frame = FswInput()
    input_frame.abi_version = FSW_ABI_VERSION
    input_frame.struct_size = ctypes.sizeof(FswInput)
    input_frame.sensors = sensor_suite_from_frame(frame)
    if device_inputs is not None:
        input_frame.air_data = device_inputs.air_data
        input_frame.propulsion = device_inputs.propulsion
        input_frame.discretes = device_inputs.discretes
        input_frame.platform = device_inputs.platform
        input_frame.command = FswCommand(
            command_sequence,
            device_inputs.command_issue_time_s,
            command_type,
        )
        return input_frame
    input_frame.air_data = FswAirDataSample(
        frame.dynamic_pressure_pa,
        frame.time_s,
        1,
    )
    input_frame.propulsion = FswPropulsionStatus(
        frame.engine_health_percent,
        frame.time_s,
        1,
        int(frame.propulsion_ready if propulsion_ready is None else propulsion_ready),
        int(
            frame.propulsion_running
            if propulsion_running is None
            else propulsion_running
        ),
    )
    input_frame.discretes = FswDiscreteInputs(
        FswDiscreteSample(frame.time_s, 1, frame.stage_separated),
        FswDiscreteSample(
            frame.time_s,
            1,
            int(
                frame.drogue_deployed
                if drogue_deployed is None
                else drogue_deployed
            ),
        ),
        FswDiscreteSample(
            frame.time_s,
            1,
            int(
                frame.main_deployed
                if main_deployed is None
                else main_deployed
            ),
        ),
    )
    input_frame.platform = FswPlatformStatus(
        frame.time_s,
        previous_execution_time_s,
        1,
        int(deadline_missed),
        int(watchdog_healthy),
    )
    input_frame.command = FswCommand(
        command_sequence,
        frame.time_s if command_issue_time_s is None else command_issue_time_s,
        command_type,
    )
    return input_frame


def recovery_stage_index(body_role: int) -> int:
    try:
        return {
            FSW_BODY_INTEGRATED: 1,
            FSW_BODY_CORE: 0,
            FSW_BODY_UPPER: 1,
        }[body_role]
    except KeyError as error:
        raise ValueError(f"unsupported FSW body role {body_role}") from error


class FlightCore:
    def __init__(
        self,
        scenario: dict,
        body_role: int = FSW_BODY_INTEGRATED,
        library_path: Path | None = None,
        auto_commands: bool = True,
    ):
        library = ctypes.CDLL(str(library_path or build_library()))
        library.fsw_abi_version.argtypes = []
        library.fsw_abi_version.restype = ctypes.c_uint32
        if library.fsw_abi_version() != FSW_ABI_VERSION:
            raise RuntimeError("flight-core ABI version mismatch")
        library.fsw_create.argtypes = [ctypes.POINTER(FswConfig)]
        library.fsw_create.restype = ctypes.c_void_p
        library.fsw_reset.argtypes = [ctypes.c_void_p]
        library.fsw_step.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(FswInput),
            ctypes.POINTER(FswOutput),
        ]
        library.fsw_step.restype = ctypes.c_int32
        library.fsw_destroy.argtypes = [ctypes.c_void_p]
        stages = scenario["vehicle"]["stages"]
        mission = scenario["mission"]
        actuators = scenario["actuators"]
        schedule = mission["attitude_schedule"]
        if len(schedule) > FSW_MAX_GUIDANCE_POINTS:
            raise ValueError(
                f"flight core accepts at most {FSW_MAX_GUIDANCE_POINTS} guidance points"
            )
        sensors = scenario["sensors"]
        devices = scenario["avionics"]["devices"]
        guidance = (GuidancePoint * FSW_MAX_GUIDANCE_POINTS)()
        for index, point in enumerate(schedule):
            guidance[index] = GuidancePoint(
                float(point["time_s"]),
                math.radians(float(point["pitch_deg"])),
                math.radians(float(point["azimuth_deg"])),
            )
        recovery_index = recovery_stage_index(body_role)
        config = FswConfig()
        config.abi_version = FSW_ABI_VERSION
        config.struct_size = ctypes.sizeof(FswConfig)
        config.stage1_burn_s = stages[0]["propulsion"]["burn_duration_s"]
        config.separation_delay_s = mission["flight_core"][
            "separation_delay_s"
        ]
        config.stage2_ignition_delay_s = mission["flight_core"][
            "stage2_ignition_delay_s"
        ]
        config.stage2_burn_s = mission["flight_core"].get(
            "stage2_first_burn_s",
            stages[1]["propulsion"]["burn_duration_s"],
        )
        orbit = mission.get("orbit", {})
        config.orbit_enabled = int(bool(orbit.get("enabled", False)))
        config.orbit_target_altitude_m = orbit["target_altitude_m"]
        config.orbit_altitude_tolerance_m = orbit["altitude_tolerance_m"]
        config.orbit_cutoff_speed_margin_m_s = orbit[
            "cutoff_speed_margin_m_s"
        ]
        config.orbit_radial_velocity_tolerance_m_s = orbit[
            "radial_velocity_tolerance_m_s"
        ]
        config.circularization_max_burn_s = orbit[
            "circularization_max_burn_s"
        ]
        config.payload_deploy_delay_s = mission["payload"]["deploy_delay_s"]
        config.main_deploy_altitude_m = stages[recovery_index]["recovery"][
            "main_deploy_altitude_m"
        ]
        config.max_tvc_rad = math.radians(actuators["max_tvc_deg"])
        config.max_fin_rad = math.radians(actuators["max_fin_deg"])
        config.control_kp = actuators["tvc_kp"]
        config.control_kd = actuators["tvc_kd"]
        config.imu_timeout_s = float(sensors["imu_timeout_s"])
        config.magnetometer_timeout_s = float(
            sensors["magnetometer_timeout_s"]
        )
        config.barometer_timeout_s = float(sensors["barometer_timeout_s"])
        config.gnss_timeout_s = float(sensors["gnss_timeout_s"])
        config.air_data_timeout_s = float(
            devices["air_data_computer"]["timeout_s"]
        )
        config.propulsion_status_timeout_s = float(
            devices["engine_controller"]["timeout_s"]
        )
        config.discrete_feedback_timeout_s = float(
            max(
                devices["discrete_input_module"]["timeout_s"],
                devices["recovery_controller"]["timeout_s"],
            )
        )
        config.platform_status_timeout_s = float(
            devices["flight_computer_platform"]["timeout_s"]
        )
        scalar_fields = {
            name
            for name, _ctype in FswConfig._fields_
            if name
            not in {
                "abi_version",
                "struct_size",
                "guidance_count",
                "guidance",
                "body_role",
            }
        }
        flight_core_config = scenario["flight_core"]
        unknown = set(flight_core_config) - scalar_fields
        if unknown:
            raise ValueError(
                "unknown flight_core configuration fields: "
                + ", ".join(sorted(unknown))
            )
        for name, value in flight_core_config.items():
            setattr(config, name, value)
        config.launch_azimuth_rad = math.radians(
            float(schedule[0]["azimuth_deg"])
        )
        config.guidance_count = len(schedule)
        config.guidance = guidance
        config.body_role = body_role
        self._library = library
        self._handle = library.fsw_create(ctypes.byref(config))
        if not self._handle:
            raise RuntimeError("fsw_create rejected the flight-core configuration")
        command_names = {
            "ARM": FSW_COMMAND_ARM,
            "DISARM": FSW_COMMAND_DISARM,
            "LAUNCH": FSW_COMMAND_LAUNCH,
            "ABORT": FSW_COMMAND_ABORT,
            "CLEAR_FAULTS": FSW_COMMAND_CLEAR_FAULTS,
        }
        configured_commands = (
            mission.get("commands")
            if auto_commands and body_role == FSW_BODY_INTEGRATED
            else []
        )
        if configured_commands is None and body_role == FSW_BODY_INTEGRATED:
            configured_commands = [
                {"time_s": 0.0, "command": "ARM"},
                {
                    "time_s": 1.0 / float(sensors["imu_rate_hz"]),
                    "command": "LAUNCH",
                },
            ]
        self._scheduled_commands = [
            (float(item["time_s"]), command_names[str(item["command"]).upper()])
            for item in (configured_commands or [])
        ]
        self._scheduled_command_index = 0
        self._next_command_sequence = 1
        self._previous_execution_time_s = 0.0
        self._loop_deadline_s = config.loop_deadline_s

    @property
    def previous_execution_time_s(self) -> float:
        return self._previous_execution_time_s

    @property
    def loop_deadline_s(self) -> float:
        return self._loop_deadline_s

    def next_scheduled_command(self, time_s: float) -> tuple[int, float] | None:
        if self._scheduled_command_index >= len(self._scheduled_commands):
            return None
        command_time, command_type = self._scheduled_commands[
            self._scheduled_command_index
        ]
        if time_s + 1e-12 < command_time:
            return None
        self._scheduled_command_index += 1
        return command_type, command_time

    def step(
        self,
        sensor: SensorFrame,
        *,
        command_type: int | None = None,
        device_inputs: FswDeviceInputs | None = None,
        propulsion_ready: bool | None = None,
        propulsion_running: bool | None = None,
        drogue_deployed: bool | None = None,
        main_deployed: bool | None = None,
        previous_execution_time_s: float | None = None,
        deadline_missed: bool | None = None,
        watchdog_healthy: bool = True,
        sensor_channels: Sequence[SensorFrame] | None = None,
    ) -> FswOutput:
        output = FswOutput()
        command_sequence = 0
        if command_type is None:
            if device_inputs is not None:
                command_type = device_inputs.command_type
            else:
                scheduled = self.next_scheduled_command(sensor.time_s)
                command_type = (
                    FSW_COMMAND_NONE if scheduled is None else scheduled[0]
                )
        if command_type != FSW_COMMAND_NONE:
            command_sequence = self._next_command_sequence
            self._next_command_sequence += 1
        reported_execution_time_s = (
            self._previous_execution_time_s
            if previous_execution_time_s is None
            else previous_execution_time_s
        )
        input_frame = fsw_input_from_frame(
            sensor,
            device_inputs=device_inputs,
            command_type=command_type,
            command_sequence=command_sequence,
            propulsion_ready=propulsion_ready,
            propulsion_running=propulsion_running,
            drogue_deployed=drogue_deployed,
            main_deployed=main_deployed,
            previous_execution_time_s=reported_execution_time_s,
            deadline_missed=(
                reported_execution_time_s > self._loop_deadline_s
                if deadline_missed is None
                else deadline_missed
            ),
            watchdog_healthy=watchdog_healthy,
        )
        if sensor_channels is not None:
            input_frame.sensors = sensor_suite_from_frames(
                (sensor, *sensor_channels)
            )
        started_ns = clock_ns()
        status = self._library.fsw_step(
            self._handle, ctypes.byref(input_frame), ctypes.byref(output)
        )
        self._previous_execution_time_s = (
            clock_ns() - started_ns
        ) * 1e-9
        if status != 0 or not output.output_valid:
            raise RuntimeError(f"fsw_step failed with status {status}")
        return output

    def close(self) -> None:
        if self._handle:
            self._library.fsw_destroy(self._handle)
            self._handle = None

    def __enter__(self) -> "FlightCore":
        return self

    def __exit__(self, *_exc_info) -> None:
        self.close()
