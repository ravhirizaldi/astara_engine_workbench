"""ctypes bridge to the generic flight-software core."""

from __future__ import annotations

import ctypes
import math
import subprocess
from pathlib import Path
from typing import Mapping

from .scenario import resolve_mission_events

FSW_MAX_GUIDANCE_POINTS = 32
FSW_MAX_SENSOR_CHANNELS = 3
FSW_BODY_INTEGRATED = 0
FSW_BODY_CORE = 1
FSW_BODY_UPPER = 2


class GuidancePoint(ctypes.Structure):
    _fields_ = [
        ("time_s", ctypes.c_double),
        ("pitch_rad", ctypes.c_double),
        ("azimuth_rad", ctypes.c_double),
    ]


class FswConfig(ctypes.Structure):
    _fields_ = [
        ("stage1_burn_s", ctypes.c_double),
        ("separation_delay_s", ctypes.c_double),
        ("stage2_ignition_delay_s", ctypes.c_double),
        ("stage2_burn_s", ctypes.c_double),
        ("main_deploy_altitude_m", ctypes.c_double),
        ("max_tvc_rad", ctypes.c_double),
        ("max_fin_rad", ctypes.c_double),
        ("control_kp", ctypes.c_double),
        ("control_kd", ctypes.c_double),
        ("imu_timeout_s", ctypes.c_double),
        ("barometer_timeout_s", ctypes.c_double),
        ("gnss_timeout_s", ctypes.c_double),
        ("acceleration_disagreement_m_s2", ctypes.c_double),
        ("gyro_disagreement_rad_s", ctypes.c_double),
        ("barometer_disagreement_m", ctypes.c_double),
        ("gnss_position_disagreement_m", ctypes.c_double),
        ("gnss_velocity_disagreement_m_s", ctypes.c_double),
        ("cross_altitude_disagreement_m", ctypes.c_double),
        ("voter_reject_samples", ctypes.c_uint32),
        ("voter_recover_samples", ctypes.c_uint32),
        ("imu_loss_abort_delay_s", ctypes.c_double),
        ("gyro_bias_time_constant_s", ctypes.c_double),
        ("stationary_gyro_threshold_rad_s", ctypes.c_double),
        ("altitude_filter_tau_s", ctypes.c_double),
        ("velocity_filter_tau_s", ctypes.c_double),
        ("guidance_count", ctypes.c_uint32),
        ("guidance", GuidancePoint * FSW_MAX_GUIDANCE_POINTS),
        ("body_role", ctypes.c_int32),
    ]


class SensorFrame(ctypes.Structure):
    _fields_ = [
        ("time_s", ctypes.c_double),
        ("dt_s", ctypes.c_double),
        ("acceleration_body_m_s2", ctypes.c_double * 3),
        ("gyro_body_rad_s", ctypes.c_double * 3),
        ("magnetic_body", ctypes.c_double * 3),
        ("barometric_altitude_m", ctypes.c_double),
        ("gnss_position_ecef_m", ctypes.c_double * 3),
        ("gnss_velocity_ecef_m_s", ctypes.c_double * 3),
        ("vertical_velocity_m_s", ctypes.c_double),
        ("dynamic_pressure_pa", ctypes.c_double),
        ("engine_health_percent", ctypes.c_double),
        ("gnss_valid", ctypes.c_int32),
        ("barometer_valid", ctypes.c_int32),
        ("stage_separated", ctypes.c_int32),
        ("barometer_sample_time_s", ctypes.c_double),
        ("gnss_sample_time_s", ctypes.c_double),
    ]


class FswImuSample(ctypes.Structure):
    _fields_ = [
        ("acceleration_body_m_s2", ctypes.c_double * 3),
        ("gyro_body_rad_s", ctypes.c_double * 3),
        ("magnetic_body", ctypes.c_double * 3),
        ("sample_time_s", ctypes.c_double),
        ("valid", ctypes.c_int32),
    ]


class FswBarometerSample(ctypes.Structure):
    _fields_ = [
        ("altitude_m", ctypes.c_double),
        ("sample_time_s", ctypes.c_double),
        ("valid", ctypes.c_int32),
    ]


class FswGnssSample(ctypes.Structure):
    _fields_ = [
        ("gnss_position_ecef_m", ctypes.c_double * 3),
        ("gnss_velocity_ecef_m_s", ctypes.c_double * 3),
        ("vertical_velocity_m_s", ctypes.c_double),
        ("sample_time_s", ctypes.c_double),
        ("valid", ctypes.c_int32),
    ]


class FswSensorSuite(ctypes.Structure):
    _fields_ = [
        ("time_s", ctypes.c_double),
        ("dt_s", ctypes.c_double),
        ("imu_count", ctypes.c_uint32),
        ("barometer_count", ctypes.c_uint32),
        ("gnss_count", ctypes.c_uint32),
        ("imus", FswImuSample * FSW_MAX_SENSOR_CHANNELS),
        ("barometers", FswBarometerSample * FSW_MAX_SENSOR_CHANNELS),
        ("gnss", FswGnssSample * FSW_MAX_SENSOR_CHANNELS),
        ("dynamic_pressure_pa", ctypes.c_double),
        ("engine_health_percent", ctypes.c_double),
        ("stage_separated", ctypes.c_int32),
    ]


class FswOutput(ctypes.Structure):
    _fields_ = [
        ("mode", ctypes.c_int32),
        ("navigation_status", ctypes.c_int32),
        ("stage_separate", ctypes.c_int32),
        ("stage2_ignite", ctypes.c_int32),
        ("deploy_drogue", ctypes.c_int32),
        ("deploy_main", ctypes.c_int32),
        ("abort", ctypes.c_int32),
        ("attitude_valid", ctypes.c_int32),
        ("imu_usable_mask", ctypes.c_uint32),
        ("barometer_usable_mask", ctypes.c_uint32),
        ("gnss_usable_mask", ctypes.c_uint32),
        ("imu_rejected_mask", ctypes.c_uint32),
        ("barometer_rejected_mask", ctypes.c_uint32),
        ("gnss_rejected_mask", ctypes.c_uint32),
        ("disagreement_flags", ctypes.c_uint32),
        ("sensor_status_flags", ctypes.c_uint32),
        ("estimated_altitude_m", ctypes.c_double),
        ("estimated_vertical_velocity_m_s", ctypes.c_double),
        ("estimated_attitude_wxyz", ctypes.c_double * 4),
        ("gyro_bias_rad_s", ctypes.c_double * 3),
        ("tvc_pitch_rad", ctypes.c_double),
        ("tvc_yaw_rad", ctypes.c_double),
        ("fin_roll_rad", ctypes.c_double),
        ("fin_pitch_rad", ctypes.c_double),
        ("fin_yaw_rad", ctypes.c_double),
        ("fault_flags", ctypes.c_uint32),
    ]


MODE_NAMES = (
    "SAFE",
    "ARMED",
    "IGNITION",
    "BOOST_1",
    "SEPARATION",
    "INTERSTAGE",
    "BOOST_2",
    "COAST",
    "APOGEE",
    "DROGUE",
    "MAIN",
    "LANDED",
    "ABORT",
)

NAVIGATION_STATUS_NAMES = ("NOMINAL", "DEGRADED", "INERTIAL")

FAULT_NAMES = (
    (1 << 0, "GNSS_UNAVAILABLE"),
    (1 << 1, "BAROMETER_UNAVAILABLE"),
    (1 << 2, "ENGINE_HEALTH"),
    (1 << 3, "NAV_INERTIAL"),
    (1 << 4, "IMU_UNAVAILABLE"),
    (1 << 5, "IMU_DISAGREEMENT"),
    (1 << 6, "BAROMETER_DISAGREEMENT"),
    (1 << 7, "GNSS_DISAGREEMENT"),
    (1 << 8, "NAV_DISAGREEMENT"),
)


def decode_faults(flags: int) -> str:
    return "|".join(name for bit, name in FAULT_NAMES if flags & bit)


SENSOR_CSV_FIELDS = (
    "body",
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
)


def sensor_frame_to_row(body: str, frame: SensorFrame) -> dict[str, float | int | str]:
    return {
        "body": body,
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
    )


def sensor_suite_from_frame(frame: SensorFrame) -> FswSensorSuite:
    imus = (FswImuSample * FSW_MAX_SENSOR_CHANNELS)()
    barometers = (FswBarometerSample * FSW_MAX_SENSOR_CHANNELS)()
    gnss = (FswGnssSample * FSW_MAX_SENSOR_CHANNELS)()
    imus[0] = FswImuSample(
        frame.acceleration_body_m_s2,
        frame.gyro_body_rad_s,
        frame.magnetic_body,
        frame.time_s,
        1,
    )
    barometers[0] = FswBarometerSample(
        frame.barometric_altitude_m,
        frame.barometer_sample_time_s,
        frame.barometer_valid,
    )
    gnss[0] = FswGnssSample(
        frame.gnss_position_ecef_m,
        frame.gnss_velocity_ecef_m_s,
        frame.vertical_velocity_m_s,
        frame.gnss_sample_time_s,
        frame.gnss_valid,
    )
    return FswSensorSuite(
        frame.time_s,
        frame.dt_s,
        1,
        1,
        1,
        imus,
        barometers,
        gnss,
        frame.dynamic_pressure_pa,
        frame.engine_health_percent,
        frame.stage_separated,
    )


def build_library() -> Path:
    root = Path(__file__).resolve().parent.parent
    source = root / "flight_core"
    build = source / "build"
    library = build / "libfsw_core.so"
    inputs = (
        tuple((source / "src").glob("*.cpp"))
        + tuple((source / "include").glob("*.h"))
        + (source / "CMakeLists.txt",)
    )
    if library.exists() and all(path.stat().st_mtime <= library.stat().st_mtime for path in inputs):
        return library
    subprocess.run(
        ["cmake", "-S", str(source), "-B", str(build), "-DCMAKE_BUILD_TYPE=Release"],
        check=True,
    )
    subprocess.run(["cmake", "--build", str(build), "--parallel"], check=True)
    return library


class FlightCore:
    def __init__(
        self,
        scenario: dict,
        body_role: int = FSW_BODY_INTEGRATED,
        library_path: Path | None = None,
    ):
        library = ctypes.CDLL(str(library_path or build_library()))
        library.fsw_create.argtypes = [ctypes.POINTER(FswConfig)]
        library.fsw_create.restype = ctypes.c_void_p
        library.fsw_reset.argtypes = [ctypes.c_void_p]
        library.fsw_step.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(FswSensorSuite),
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
        config = FswConfig(
            stages[0]["propulsion"]["burn_duration_s"],
            event_times["stage_separation"] - event_times["burnout_stage_1"],
            event_times["stage2_ignition"] - event_times["stage_separation"],
            stages[1]["propulsion"]["burn_duration_s"],
            stages[recovery_stage_index]["recovery"]["main_deploy_altitude_m"],
            math.radians(actuators["max_tvc_deg"]),
            math.radians(actuators["max_fin_deg"]),
            actuators["tvc_kp"],
            actuators["tvc_kd"],
            float(
                sensors.get(
                    "imu_timeout_s",
                    max(3.0 / float(sensors["imu_rate_hz"]), 0.02),
                )
            ),
            float(
                sensors.get(
                    "barometer_timeout_s",
                    max(3.0 / float(sensors["barometer_rate_hz"]), 0.05),
                )
            ),
            float(
                sensors.get(
                    "gnss_timeout_s",
                    max(3.0 / float(sensors["gnss_rate_hz"]), 0.25),
                )
            ),
            float(
                sensors.get(
                    "acceleration_disagreement_m_s2",
                    max(6.0 * accelerometer_noise, 0.5),
                )
            ),
            float(
                sensors.get(
                    "gyro_disagreement_rad_s",
                    max(6.0 * gyro_noise, 0.01),
                )
            ),
            float(
                sensors.get(
                    "barometer_disagreement_m",
                    max(6.0 * barometer_noise, 10.0),
                )
            ),
            float(
                sensors.get(
                    "gnss_position_disagreement_m",
                    max(6.0 * gnss_position_noise, 15.0),
                )
            ),
            float(
                sensors.get(
                    "gnss_velocity_disagreement_m_s",
                    max(6.0 * gnss_velocity_noise, 1.0),
                )
            ),
            float(
                sensors.get(
                    "cross_altitude_disagreement_m",
                    max(
                        6.0
                        * math.hypot(barometer_noise, gnss_position_noise),
                        20.0,
                    ),
                )
            ),
            int(sensors.get("voter_reject_samples", 3)),
            int(sensors.get("voter_recover_samples", 5)),
            float(sensors.get("imu_loss_abort_delay_s", 0.05)),
            float(sensors.get("gyro_bias_time_constant_s", 2.0)),
            float(sensors.get("stationary_gyro_threshold_rad_s", 0.02)),
            float(sensors.get("altitude_filter_tau_s", 0.20)),
            float(sensors.get("velocity_filter_tau_s", 0.60)),
            len(schedule),
            guidance,
            body_role,
        )
        self._library = library
        self._handle = library.fsw_create(ctypes.byref(config))
        if not self._handle:
            raise RuntimeError("fsw_create rejected the flight-core configuration")

    def step(self, sensor: SensorFrame) -> FswOutput:
        output = FswOutput()
        suite = sensor_suite_from_frame(sensor)
        status = self._library.fsw_step(
            self._handle, ctypes.byref(suite), ctypes.byref(output)
        )
        if status != 0:
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
