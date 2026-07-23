"""ctypes bridge to ASTARA Flight Core."""

from __future__ import annotations

import ctypes
import math
import subprocess
from pathlib import Path
from typing import Mapping

from .scenario import resolve_mission_events


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
        ("pitch_start_s", ctypes.c_double),
        ("pitch_end_s", ctypes.c_double),
        ("max_pitch_rad", ctypes.c_double),
        ("target_azimuth_rad", ctypes.c_double),
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
    ]


class FswOutput(ctypes.Structure):
    _fields_ = [
        ("mode", ctypes.c_int32),
        ("stage_separate", ctypes.c_int32),
        ("stage2_ignite", ctypes.c_int32),
        ("deploy_drogue", ctypes.c_int32),
        ("deploy_main", ctypes.c_int32),
        ("abort", ctypes.c_int32),
        ("estimated_altitude_m", ctypes.c_double),
        ("estimated_vertical_velocity_m_s", ctypes.c_double),
        ("estimated_attitude_wxyz", ctypes.c_double * 4),
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
    )


def build_library() -> Path:
    root = Path(__file__).resolve().parent.parent
    source = root / "flight_core"
    build = source / "build"
    library = build / "libastara_fsw.so"
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
    def __init__(self, scenario: dict, body_role: int = 0, library_path: Path | None = None):
        library = ctypes.CDLL(str(library_path or build_library()))
        library.astara_fsw_create.argtypes = [ctypes.POINTER(FswConfig)]
        library.astara_fsw_create.restype = ctypes.c_void_p
        library.astara_fsw_step.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(SensorFrame),
            ctypes.POINTER(FswOutput),
        ]
        library.astara_fsw_step.restype = ctypes.c_int32
        library.astara_fsw_destroy.argtypes = [ctypes.c_void_p]
        stages = scenario["vehicle"]["stages"]
        mission = scenario["mission"]
        event_times = resolve_mission_events(scenario)
        actuators = scenario["actuators"]
        schedule = mission["attitude_schedule"]
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
            schedule[0]["time_s"],
            schedule[-1]["time_s"],
            math.radians(schedule[-1]["pitch_deg"]),
            0.0,
            body_role,
        )
        self._library = library
        self._handle = library.astara_fsw_create(ctypes.byref(config))
        if not self._handle:
            raise RuntimeError("astara_fsw_create failed")

    def step(self, sensor: SensorFrame) -> FswOutput:
        output = FswOutput()
        status = self._library.astara_fsw_step(
            self._handle, ctypes.byref(sensor), ctypes.byref(output)
        )
        if status != 0:
            raise RuntimeError(f"astara_fsw_step failed with status {status}")
        return output

    def close(self) -> None:
        if self._handle:
            self._library.astara_fsw_destroy(self._handle)
            self._handle = None

    def __enter__(self) -> "FlightCore":
        return self

    def __exit__(self, *_exc_info) -> None:
        self.close()
