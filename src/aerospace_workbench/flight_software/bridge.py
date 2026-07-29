"""Aerospace Workbench ctypes bridge to the generic flight-software core."""

from __future__ import annotations

import ctypes
import math
from pathlib import Path
from typing import Mapping, Sequence

from ..configuration.scenarios import resolve_mission_events
from ..configuration.schemas import SENSOR_STREAM_SCHEMA_VERSION
from .abi import (
    FSW_ABI_VERSION,
    FSW_BODY_INTEGRATED,
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
        float(row.get("barometer_sample_time_s") or row["time_s"]),
        float(row.get("gnss_sample_time_s") or row["time_s"]),
        int(row.get("propulsion_ready", "1")),
        int(row.get("propulsion_running", int(float(row["time_s"]) > 0.0))),
        int(row.get("drogue_deployed", "0")),
        int(row.get("main_deployed", "0")),
        float(row.get("imu_sample_time_s") or row["time_s"]),
        float(row.get("magnetometer_sample_time_s") or row["time_s"]),
        int(row.get("accel_valid", "1")),
        int(row.get("gyro_valid", "1")),
        int(row.get("magnetometer_valid", "1")),
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
    command_type: int = FSW_COMMAND_NONE,
    command_sequence: int = 0,
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
        frame.time_s,
        command_type,
    )
    return input_frame


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
        event_times = resolve_mission_events(scenario)
        actuators = scenario["actuators"]
        schedule = mission["attitude_schedule"]
        if len(schedule) > FSW_MAX_GUIDANCE_POINTS:
            raise ValueError(
                f"flight core accepts at most {FSW_MAX_GUIDANCE_POINTS} guidance points"
            )
        sensors = scenario["sensors"]
        accelerometer_noise = float(sensors["accelerometer_noise_m_s2"])
        gyro_noise = float(sensors["gyro_noise_rad_s"])
        barometer_noise = float(sensors["barometer_noise_m"])
        gnss_position_noise = float(sensors["gnss_position_noise_m"])
        gnss_velocity_noise = float(sensors["gnss_velocity_noise_m_s"])
        guidance = (GuidancePoint * FSW_MAX_GUIDANCE_POINTS)()
        for index, point in enumerate(schedule):
            guidance[index] = GuidancePoint(
                float(point["time_s"]),
                math.radians(float(point["pitch_deg"])),
                math.radians(float(point["azimuth_deg"])),
            )
        recovery_stage_index = min(body_role, 1)
        config = FswConfig()
        config.abi_version = FSW_ABI_VERSION
        config.struct_size = ctypes.sizeof(FswConfig)
        config.stage1_burn_s = stages[0]["propulsion"]["burn_duration_s"]
        config.separation_delay_s = (
            event_times["stage_separation"] - event_times["burnout_stage_1"]
        )
        config.stage2_ignition_delay_s = (
            event_times["stage2_ignition"] - event_times["stage_separation"]
        )
        config.stage2_burn_s = stages[1]["propulsion"]["burn_duration_s"]
        config.main_deploy_altitude_m = stages[recovery_stage_index]["recovery"][
            "main_deploy_altitude_m"
        ]
        config.max_tvc_rad = math.radians(actuators["max_tvc_deg"])
        config.max_fin_rad = math.radians(actuators["max_fin_deg"])
        config.control_kp = actuators["tvc_kp"]
        config.control_kd = actuators["tvc_kd"]
        config.imu_timeout_s = float(
            sensors.get(
                "imu_timeout_s",
                max(3.0 / float(sensors["imu_rate_hz"]), 0.02),
            )
        )
        config.barometer_timeout_s = float(
            sensors.get(
                "barometer_timeout_s",
                max(3.0 / float(sensors["barometer_rate_hz"]), 0.05),
            )
        )
        config.gnss_timeout_s = float(
            sensors.get(
                "gnss_timeout_s",
                max(3.0 / float(sensors["gnss_rate_hz"]), 0.25),
            )
        )
        config.air_data_timeout_s = float(
            sensors.get("air_data_timeout_s", config.imu_timeout_s)
        )
        config.propulsion_status_timeout_s = float(
            sensors.get(
                "propulsion_status_timeout_s",
                config.imu_timeout_s,
            )
        )
        config.discrete_feedback_timeout_s = float(
            sensors.get(
                "discrete_feedback_timeout_s",
                config.imu_timeout_s,
            )
        )
        config.platform_status_timeout_s = float(
            sensors.get(
                "platform_status_timeout_s",
                config.imu_timeout_s,
            )
        )
        config.acceleration_disagreement_m_s2 = float(
            sensors.get(
                "acceleration_disagreement_m_s2",
                max(6.0 * accelerometer_noise, 0.5),
            )
        )
        config.gyro_disagreement_rad_s = float(
            sensors.get(
                "gyro_disagreement_rad_s",
                max(6.0 * gyro_noise, 0.01),
            )
        )
        config.magnetic_disagreement = float(
            sensors.get("magnetic_disagreement", 0.15)
        )
        config.barometer_disagreement_m = float(
            sensors.get(
                "barometer_disagreement_m",
                max(6.0 * barometer_noise, 10.0),
            )
        )
        config.gnss_position_disagreement_m = float(
            sensors.get(
                "gnss_position_disagreement_m",
                max(6.0 * gnss_position_noise, 15.0),
            )
        )
        config.gnss_velocity_disagreement_m_s = float(
            sensors.get(
                "gnss_velocity_disagreement_m_s",
                max(6.0 * gnss_velocity_noise, 1.0),
            )
        )
        config.cross_altitude_disagreement_m = float(
            sensors.get(
                "cross_altitude_disagreement_m",
                max(
                    6.0 * math.hypot(barometer_noise, gnss_position_noise),
                    20.0,
                ),
            )
        )
        config.voter_reject_samples = int(sensors.get("voter_reject_samples", 3))
        config.voter_recover_samples = int(sensors.get("voter_recover_samples", 5))
        config.imu_loss_abort_delay_s = float(
            sensors.get("imu_loss_abort_delay_s", 0.05)
        )
        config.gyro_bias_time_constant_s = float(
            sensors.get("gyro_bias_time_constant_s", 2.0)
        )
        config.stationary_gyro_threshold_rad_s = float(
            sensors.get("stationary_gyro_threshold_rad_s", 0.02)
        )
        config.altitude_filter_tau_s = float(
            sensors.get("altitude_filter_tau_s", 0.20)
        )
        config.velocity_filter_tau_s = float(
            sensors.get("velocity_filter_tau_s", 0.60)
        )
        config.command_timeout_s = 1.0
        config.launch_confirm_timeout_s = 2.0
        config.separation_confirm_timeout_s = 5.0
        config.stage2_ignition_timeout_s = 5.0
        config.drogue_confirm_timeout_s = 2.0
        config.main_confirm_timeout_s = 2.0
        config.fault_recovery_persistence_s = 0.25
        config.min_step_s = 1e-6
        config.max_step_s = 0.1
        config.step_time_tolerance_s = float(
            sensors.get("step_time_tolerance_s", 1e-6)
        )
        config.loop_deadline_s = 2.0 / float(sensors["imu_rate_hz"])
        config.overrun_abort_count = 3
        config.propulsion_abort_health_percent = 20.0
        config.propulsion_abort_persistence_s = 0.05
        config.max_acceleration_m_s2 = 500.0
        config.max_gyro_rad_s = 20.0
        config.min_magnetic_norm = 0.25
        config.max_magnetic_norm = 2.0
        config.min_barometer_altitude_m = -1_000.0
        config.max_barometer_altitude_m = 2_000_000.0
        config.max_barometer_rate_m_s = 5_000.0
        config.min_gnss_radius_m = 5_000_000.0
        config.max_gnss_radius_m = 8_000_000.0
        config.max_gnss_speed_m_s = 15_000.0
        config.max_gnss_velocity_rate_m_s2 = 1_000.0
        config.accelerometer_process_sigma_m_s2 = max(accelerometer_noise, 0.01)
        config.gyro_process_sigma_rad_s = max(gyro_noise, 1e-5)
        config.barometer_sigma_m = max(barometer_noise, 0.1)
        config.gnss_altitude_sigma_m = max(gnss_position_noise, 0.1)
        config.gnss_velocity_sigma_m_s = max(gnss_velocity_noise, 0.01)
        config.max_altitude_sigma_m = 500.0
        config.max_velocity_sigma_m_s = 200.0
        config.max_attitude_sigma_rad = 1.0
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

    def step(
        self,
        sensor: SensorFrame,
        *,
        command_type: int | None = None,
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
            command_type = FSW_COMMAND_NONE
            if self._scheduled_command_index < len(self._scheduled_commands):
                command_time, scheduled_type = self._scheduled_commands[
                    self._scheduled_command_index
                ]
                if sensor.time_s + 1e-12 >= command_time:
                    command_type = scheduled_type
                    self._scheduled_command_index += 1
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
