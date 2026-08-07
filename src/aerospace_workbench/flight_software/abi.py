"""Product-neutral C ABI declarations for the flight-software core."""

from __future__ import annotations

import ctypes

FSW_MAX_GUIDANCE_POINTS = 32
FSW_MAX_SENSOR_CHANNELS = 3
FSW_FAULT_COUNT = 21
FSW_ABI_VERSION = 0x00090000
FSW_BODY_INTEGRATED = 0
FSW_BODY_CORE = 1
FSW_BODY_UPPER = 2
FSW_COMMAND_NONE = 0
FSW_COMMAND_ARM = 1
FSW_COMMAND_DISARM = 2
FSW_COMMAND_LAUNCH = 3
FSW_COMMAND_ABORT = 4
FSW_COMMAND_CLEAR_FAULTS = 5
FSW_DISCRETE_ACTION_STAGE_SEPARATE = 1
FSW_DISCRETE_ACTION_DEPLOY_DROGUE = 2
FSW_DISCRETE_ACTION_DEPLOY_MAIN = 3
FSW_DISCRETE_ACTION_DEPLOY_PAYLOAD = 4


class GuidancePoint(ctypes.Structure):
    _fields_ = [
        ("time_s", ctypes.c_double),
        ("pitch_rad", ctypes.c_double),
        ("azimuth_rad", ctypes.c_double),
    ]


class FswConfig(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
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
        ("magnetometer_timeout_s", ctypes.c_double),
        ("barometer_timeout_s", ctypes.c_double),
        ("gnss_timeout_s", ctypes.c_double),
        ("air_data_timeout_s", ctypes.c_double),
        ("propulsion_status_timeout_s", ctypes.c_double),
        ("discrete_feedback_timeout_s", ctypes.c_double),
        ("platform_status_timeout_s", ctypes.c_double),
        ("max_voter_sample_skew_s", ctypes.c_double),
        ("acceleration_disagreement_m_s2", ctypes.c_double),
        ("gyro_disagreement_rad_s", ctypes.c_double),
        ("magnetic_disagreement", ctypes.c_double),
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
        ("command_timeout_s", ctypes.c_double),
        ("launch_confirm_timeout_s", ctypes.c_double),
        ("separation_confirm_timeout_s", ctypes.c_double),
        ("stage2_ignition_timeout_s", ctypes.c_double),
        ("drogue_confirm_timeout_s", ctypes.c_double),
        ("main_confirm_timeout_s", ctypes.c_double),
        ("fault_recovery_persistence_s", ctypes.c_double),
        ("min_step_s", ctypes.c_double),
        ("max_step_s", ctypes.c_double),
        ("step_time_tolerance_s", ctypes.c_double),
        ("loop_deadline_s", ctypes.c_double),
        ("overrun_abort_count", ctypes.c_uint32),
        ("propulsion_abort_health_percent", ctypes.c_double),
        ("propulsion_abort_persistence_s", ctypes.c_double),
        ("max_acceleration_m_s2", ctypes.c_double),
        ("max_gyro_rad_s", ctypes.c_double),
        ("min_magnetic_norm", ctypes.c_double),
        ("max_magnetic_norm", ctypes.c_double),
        ("min_barometer_altitude_m", ctypes.c_double),
        ("max_barometer_altitude_m", ctypes.c_double),
        ("max_barometer_rate_m_s", ctypes.c_double),
        ("min_gnss_radius_m", ctypes.c_double),
        ("max_gnss_radius_m", ctypes.c_double),
        ("max_gnss_speed_m_s", ctypes.c_double),
        ("max_gnss_velocity_rate_m_s2", ctypes.c_double),
        ("accelerometer_process_sigma_m_s2", ctypes.c_double),
        ("gyro_process_sigma_rad_s", ctypes.c_double),
        ("barometer_sigma_m", ctypes.c_double),
        ("gnss_altitude_sigma_m", ctypes.c_double),
        ("gnss_velocity_sigma_m_s", ctypes.c_double),
        ("max_altitude_sigma_m", ctypes.c_double),
        ("max_velocity_sigma_m_s", ctypes.c_double),
        ("max_attitude_sigma_rad", ctypes.c_double),
        ("launch_azimuth_rad", ctypes.c_double),
        ("launch_acceleration_threshold_m_s2", ctypes.c_double),
        ("launch_persistence_s", ctypes.c_double),
        ("burnout_acceleration_threshold_m_s2", ctypes.c_double),
        ("burnout_persistence_s", ctypes.c_double),
        ("apogee_min_altitude_m", ctypes.c_double),
        ("apogee_descent_velocity_m_s", ctypes.c_double),
        ("apogee_persistence_s", ctypes.c_double),
        ("landing_altitude_m", ctypes.c_double),
        ("landing_speed_m_s", ctypes.c_double),
        ("landing_persistence_s", ctypes.c_double),
        ("aero_reference_dynamic_pressure_pa", ctypes.c_double),
        ("aero_high_q_authority_scale", ctypes.c_double),
        ("orbit_target_altitude_m", ctypes.c_double),
        ("orbit_altitude_tolerance_m", ctypes.c_double),
        ("orbit_cutoff_speed_margin_m_s", ctypes.c_double),
        ("orbit_radial_velocity_tolerance_m_s", ctypes.c_double),
        ("circularization_max_burn_s", ctypes.c_double),
        ("payload_deploy_delay_s", ctypes.c_double),
        ("orbit_enabled", ctypes.c_int32),
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
        ("propulsion_ready", ctypes.c_int32),
        ("propulsion_running", ctypes.c_int32),
        ("drogue_deployed", ctypes.c_int32),
        ("main_deployed", ctypes.c_int32),
        ("imu_sample_time_s", ctypes.c_double),
        ("magnetometer_sample_time_s", ctypes.c_double),
        ("accel_valid", ctypes.c_int32),
        ("gyro_valid", ctypes.c_int32),
        ("magnetometer_valid", ctypes.c_int32),
    ]


class FswImuSample(ctypes.Structure):
    _fields_ = [
        ("acceleration_body_m_s2", ctypes.c_double * 3),
        ("gyro_body_rad_s", ctypes.c_double * 3),
        ("sample_time_s", ctypes.c_double),
        ("accel_valid", ctypes.c_int32),
        ("gyro_valid", ctypes.c_int32),
    ]


class FswMagnetometerSample(ctypes.Structure):
    _fields_ = [
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
        ("sample_time_s", ctypes.c_double),
        ("valid", ctypes.c_int32),
    ]


class FswSensorSuite(ctypes.Structure):
    _fields_ = [
        ("time_s", ctypes.c_double),
        ("dt_s", ctypes.c_double),
        ("imu_count", ctypes.c_uint32),
        ("magnetometer_count", ctypes.c_uint32),
        ("barometer_count", ctypes.c_uint32),
        ("gnss_count", ctypes.c_uint32),
        ("imus", FswImuSample * FSW_MAX_SENSOR_CHANNELS),
        ("magnetometers", FswMagnetometerSample * FSW_MAX_SENSOR_CHANNELS),
        ("barometers", FswBarometerSample * FSW_MAX_SENSOR_CHANNELS),
        ("gnss", FswGnssSample * FSW_MAX_SENSOR_CHANNELS),
    ]


class FswAirDataSample(ctypes.Structure):
    _fields_ = [
        ("dynamic_pressure_pa", ctypes.c_double),
        ("sample_time_s", ctypes.c_double),
        ("valid", ctypes.c_int32),
    ]


class FswPropulsionStatus(ctypes.Structure):
    _fields_ = [
        ("health_percent", ctypes.c_double),
        ("sample_time_s", ctypes.c_double),
        ("valid", ctypes.c_int32),
        ("ready", ctypes.c_int32),
        ("running", ctypes.c_int32),
    ]


class FswDiscreteSample(ctypes.Structure):
    _fields_ = [
        ("sample_time_s", ctypes.c_double),
        ("valid", ctypes.c_int32),
        ("asserted", ctypes.c_int32),
    ]


class FswDiscreteInputs(ctypes.Structure):
    _fields_ = [
        ("stage_separated", FswDiscreteSample),
        ("drogue_deployed", FswDiscreteSample),
        ("main_deployed", FswDiscreteSample),
    ]


class FswPlatformStatus(ctypes.Structure):
    _fields_ = [
        ("sample_time_s", ctypes.c_double),
        ("previous_execution_time_s", ctypes.c_double),
        ("valid", ctypes.c_int32),
        ("deadline_missed", ctypes.c_int32),
        ("watchdog_healthy", ctypes.c_int32),
    ]


class FswCommand(ctypes.Structure):
    _fields_ = [
        ("sequence", ctypes.c_uint64),
        ("issue_time_s", ctypes.c_double),
        ("type", ctypes.c_int32),
    ]


class FswDiscreteActuationCommand(ctypes.Structure):
    _fields_ = [
        ("sequence", ctypes.c_uint64),
        ("action", ctypes.c_int32),
        ("pulse_duration_s", ctypes.c_double),
        ("valid", ctypes.c_int32),
    ]


class FswInput(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("sensors", FswSensorSuite),
        ("air_data", FswAirDataSample),
        ("propulsion", FswPropulsionStatus),
        ("discretes", FswDiscreteInputs),
        ("platform", FswPlatformStatus),
        ("command", FswCommand),
    ]


class FswOutput(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("output_valid", ctypes.c_int32),
        ("step_status", ctypes.c_int32),
        ("mode", ctypes.c_int32),
        ("navigation_status", ctypes.c_int32),
        ("stage1_ignite", ctypes.c_int32),
        ("stage_separate", ctypes.c_int32),
        ("stage2_ignite", ctypes.c_int32),
        ("stage2_shutdown", ctypes.c_int32),
        ("deploy_drogue", ctypes.c_int32),
        ("deploy_main", ctypes.c_int32),
        ("deploy_payload", ctypes.c_int32),
        ("abort", ctypes.c_int32),
        ("attitude_valid", ctypes.c_int32),
        ("command_sequence", ctypes.c_uint64),
        ("command_type", ctypes.c_int32),
        ("command_result", ctypes.c_int32),
        ("inhibit_flags", ctypes.c_uint32),
        ("event_flags", ctypes.c_uint32),
        ("discrete_actuation", FswDiscreteActuationCommand),
        ("accelerometer_usable_mask", ctypes.c_uint32),
        ("gyroscope_usable_mask", ctypes.c_uint32),
        ("magnetometer_usable_mask", ctypes.c_uint32),
        ("barometer_usable_mask", ctypes.c_uint32),
        ("gnss_usable_mask", ctypes.c_uint32),
        ("accelerometer_rejected_mask", ctypes.c_uint32),
        ("gyroscope_rejected_mask", ctypes.c_uint32),
        ("magnetometer_rejected_mask", ctypes.c_uint32),
        ("barometer_rejected_mask", ctypes.c_uint32),
        ("gnss_rejected_mask", ctypes.c_uint32),
        ("disagreement_flags", ctypes.c_uint32),
        ("sensor_status_flags", ctypes.c_uint32),
        (
            "accelerometer_health_flags",
            ctypes.c_uint32 * FSW_MAX_SENSOR_CHANNELS,
        ),
        ("gyroscope_health_flags", ctypes.c_uint32 * FSW_MAX_SENSOR_CHANNELS),
        (
            "magnetometer_health_flags",
            ctypes.c_uint32 * FSW_MAX_SENSOR_CHANNELS,
        ),
        ("barometer_health_flags", ctypes.c_uint32 * FSW_MAX_SENSOR_CHANNELS),
        ("gnss_health_flags", ctypes.c_uint32 * FSW_MAX_SENSOR_CHANNELS),
        ("accelerometer_age_s", ctypes.c_double * FSW_MAX_SENSOR_CHANNELS),
        ("gyroscope_age_s", ctypes.c_double * FSW_MAX_SENSOR_CHANNELS),
        ("magnetometer_age_s", ctypes.c_double * FSW_MAX_SENSOR_CHANNELS),
        ("barometer_age_s", ctypes.c_double * FSW_MAX_SENSOR_CHANNELS),
        ("gnss_age_s", ctypes.c_double * FSW_MAX_SENSOR_CHANNELS),
        ("estimated_altitude_m", ctypes.c_double),
        ("estimated_vertical_velocity_m_s", ctypes.c_double),
        ("estimated_position_ecef_m", ctypes.c_double * 3),
        ("estimated_velocity_ecef_m_s", ctypes.c_double * 3),
        ("estimated_attitude_wxyz", ctypes.c_double * 4),
        ("altitude_sigma_m", ctypes.c_double),
        ("vertical_velocity_sigma_m_s", ctypes.c_double),
        ("attitude_sigma_rad", ctypes.c_double * 3),
        ("barometer_innovation_m", ctypes.c_double),
        ("gnss_altitude_innovation_m", ctypes.c_double),
        ("gnss_velocity_innovation_m_s", ctypes.c_double),
        ("gyro_bias_rad_s", ctypes.c_double * 3),
        ("tvc_pitch_rad", ctypes.c_double),
        ("tvc_yaw_rad", ctypes.c_double),
        ("fin_roll_rad", ctypes.c_double),
        ("fin_pitch_rad", ctypes.c_double),
        ("fin_yaw_rad", ctypes.c_double),
        ("active_fault_flags", ctypes.c_uint32),
        ("latched_fault_flags", ctypes.c_uint32),
        ("changed_fault_flags", ctypes.c_uint32),
        ("fault_occurrence_count", ctypes.c_uint32 * FSW_FAULT_COUNT),
        ("highest_fault_severity", ctypes.c_int32),
        ("previous_execution_time_s", ctypes.c_double),
        ("consecutive_overruns", ctypes.c_uint32),
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
    "ORBIT_INSERTION",
    "ORBIT",
    "PAYLOAD_DEPLOYED",
)

NAVIGATION_STATUS_NAMES = ("NOMINAL", "DEGRADED", "INERTIAL")

FAULT_NAMES = (
    (1 << 0, "GNSS_UNAVAILABLE"),
    (1 << 1, "BAROMETER_UNAVAILABLE"),
    (1 << 2, "PROPULSION_HEALTH"),
    (1 << 3, "NAV_INERTIAL"),
    (1 << 4, "IMU_UNAVAILABLE"),
    (1 << 5, "IMU_DISAGREEMENT"),
    (1 << 6, "BAROMETER_DISAGREEMENT"),
    (1 << 7, "GNSS_DISAGREEMENT"),
    (1 << 8, "NAV_DISAGREEMENT"),
    (1 << 9, "MAGNETOMETER_DISAGREEMENT"),
    (1 << 10, "AIR_DATA_UNAVAILABLE"),
    (1 << 11, "PROPULSION_UNAVAILABLE"),
    (1 << 12, "DEADLINE_OVERRUN"),
    (1 << 13, "WATCHDOG"),
    (1 << 14, "LAUNCH_NOT_CONFIRMED"),
    (1 << 15, "SEPARATION_NOT_CONFIRMED"),
    (1 << 16, "STAGE2_IGNITION"),
    (1 << 17, "DROGUE_NOT_CONFIRMED"),
    (1 << 18, "MAIN_NOT_CONFIRMED"),
    (1 << 19, "NAV_UNCERTAINTY"),
    (1 << 20, "INPUT_TIMING"),
)


def decode_faults(flags: int) -> str:
    return "|".join(name for bit, name in FAULT_NAMES if flags & bit)
