#ifndef FSW_H
#define FSW_H

#include <stdint.h>

#define FSW_ABI_VERSION 0x00040000u
#define FSW_MAX_GUIDANCE_POINTS 32
#define FSW_MAX_SENSOR_CHANNELS 3
#define FSW_FAULT_COUNT 21

#if defined(_WIN32)
#define FSW_API __declspec(dllexport)
#else
#define FSW_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum FswStatus {
    FSW_STATUS_OK = 0,
    FSW_STATUS_INVALID_ARGUMENT = -1,
    FSW_STATUS_INVALID_INPUT = -2,
    FSW_STATUS_ABI_MISMATCH = -3
};

enum FswBodyRole {
    FSW_BODY_INTEGRATED = 0,
    FSW_BODY_CORE = 1,
    FSW_BODY_UPPER = 2
};

enum FswFlightMode {
    FSW_MODE_SAFE = 0,
    FSW_MODE_ARMED = 1,
    FSW_MODE_IGNITION = 2,
    FSW_MODE_BOOST_1 = 3,
    FSW_MODE_SEPARATION = 4,
    FSW_MODE_INTERSTAGE = 5,
    FSW_MODE_BOOST_2 = 6,
    FSW_MODE_COAST = 7,
    FSW_MODE_APOGEE = 8,
    FSW_MODE_DROGUE = 9,
    FSW_MODE_MAIN = 10,
    FSW_MODE_LANDED = 11,
    FSW_MODE_ABORT = 12
};

enum FswNavigationStatus {
    FSW_NAV_NOMINAL = 0,
    FSW_NAV_DEGRADED = 1,
    FSW_NAV_INERTIAL = 2
};

enum FswCommandType {
    FSW_COMMAND_NONE = 0,
    FSW_COMMAND_ARM = 1,
    FSW_COMMAND_DISARM = 2,
    FSW_COMMAND_LAUNCH = 3,
    FSW_COMMAND_ABORT = 4,
    FSW_COMMAND_CLEAR_FAULTS = 5
};

enum FswCommandResult {
    FSW_COMMAND_NOT_PROCESSED = 0,
    FSW_COMMAND_ACCEPTED = 1,
    FSW_COMMAND_REJECTED_STALE = 2,
    FSW_COMMAND_REJECTED_INVALID_STATE = 3,
    FSW_COMMAND_REJECTED_INHIBITED = 4,
    FSW_COMMAND_REJECTED_INVALID = 5
};

enum FswFaultSeverity {
    FSW_SEVERITY_NONE = 0,
    FSW_SEVERITY_WARNING = 1,
    FSW_SEVERITY_DEGRADED = 2,
    FSW_SEVERITY_CRITICAL = 3,
    FSW_SEVERITY_MISSION_ENDING = 4
};

enum FswFaultFlag {
    FSW_FAULT_GNSS_UNAVAILABLE = 1u << 0,
    FSW_FAULT_BAROMETER_UNAVAILABLE = 1u << 1,
    FSW_FAULT_PROPULSION_HEALTH = 1u << 2,
    FSW_FAULT_NAV_INERTIAL = 1u << 3,
    FSW_FAULT_IMU_UNAVAILABLE = 1u << 4,
    FSW_FAULT_IMU_DISAGREEMENT = 1u << 5,
    FSW_FAULT_BAROMETER_DISAGREEMENT = 1u << 6,
    FSW_FAULT_GNSS_DISAGREEMENT = 1u << 7,
    FSW_FAULT_NAV_DISAGREEMENT = 1u << 8,
    FSW_FAULT_MAGNETOMETER_DISAGREEMENT = 1u << 9,
    FSW_FAULT_AIR_DATA_UNAVAILABLE = 1u << 10,
    FSW_FAULT_PROPULSION_UNAVAILABLE = 1u << 11,
    FSW_FAULT_DEADLINE_OVERRUN = 1u << 12,
    FSW_FAULT_WATCHDOG = 1u << 13,
    FSW_FAULT_LAUNCH_NOT_CONFIRMED = 1u << 14,
    FSW_FAULT_SEPARATION_NOT_CONFIRMED = 1u << 15,
    FSW_FAULT_STAGE2_IGNITION = 1u << 16,
    FSW_FAULT_DROGUE_NOT_CONFIRMED = 1u << 17,
    FSW_FAULT_MAIN_NOT_CONFIRMED = 1u << 18,
    FSW_FAULT_NAV_UNCERTAINTY = 1u << 19,
    FSW_FAULT_INPUT_TIMING = 1u << 20
};

enum FswDisagreementFlag {
    FSW_DISAGREEMENT_ACCELERATION = 1u << 0,
    FSW_DISAGREEMENT_GYRO = 1u << 1,
    FSW_DISAGREEMENT_BAROMETER = 1u << 2,
    FSW_DISAGREEMENT_GNSS_POSITION = 1u << 3,
    FSW_DISAGREEMENT_GNSS_VELOCITY = 1u << 4,
    FSW_DISAGREEMENT_CROSS_ALTITUDE = 1u << 5,
    FSW_DISAGREEMENT_MAGNETOMETER = 1u << 6
};

enum FswSensorStatusFlag {
    FSW_SENSOR_STATUS_IMU_SINGLE_SOURCE = 1u << 0,
    FSW_SENSOR_STATUS_BAROMETER_SINGLE_SOURCE = 1u << 1,
    FSW_SENSOR_STATUS_GNSS_SINGLE_SOURCE = 1u << 2
};

enum FswSensorHealthFlag {
    FSW_SENSOR_HEALTH_INVALID = 1u << 0,
    FSW_SENSOR_HEALTH_STALE = 1u << 1,
    FSW_SENSOR_HEALTH_OUT_OF_RANGE = 1u << 2,
    FSW_SENSOR_HEALTH_RATE_LIMIT = 1u << 3,
    FSW_SENSOR_HEALTH_DISAGREEMENT = 1u << 4,
    FSW_SENSOR_HEALTH_REJECTED = 1u << 5
};

enum FswInhibitFlag {
    FSW_INHIBIT_IMU = 1u << 0,
    FSW_INHIBIT_ATTITUDE = 1u << 1,
    FSW_INHIBIT_PROPULSION = 1u << 2,
    FSW_INHIBIT_CRITICAL_FAULT = 1u << 3,
    FSW_INHIBIT_NAVIGATION = 1u << 4,
    FSW_INHIBIT_SEPARATION = 1u << 5,
    FSW_INHIBIT_TIMING = 1u << 6
};

enum FswEventFlag {
    FSW_EVENT_STATE_CHANGED = 1u << 0,
    FSW_EVENT_FAULT_CHANGED = 1u << 1,
    FSW_EVENT_COMMAND_PROCESSED = 1u << 2
};

typedef struct {
    double time_s;
    double pitch_rad;
    double azimuth_rad;
} FswGuidancePoint;

typedef struct {
    uint32_t abi_version;
    uint32_t struct_size;
    double stage1_burn_s;
    double separation_delay_s;
    double stage2_ignition_delay_s;
    double stage2_burn_s;
    double main_deploy_altitude_m;
    double max_tvc_rad;
    double max_fin_rad;
    double control_kp;
    double control_kd;
    double imu_timeout_s;
    double barometer_timeout_s;
    double gnss_timeout_s;
    double acceleration_disagreement_m_s2;
    double gyro_disagreement_rad_s;
    double magnetic_disagreement;
    double barometer_disagreement_m;
    double gnss_position_disagreement_m;
    double gnss_velocity_disagreement_m_s;
    double cross_altitude_disagreement_m;
    uint32_t voter_reject_samples;
    uint32_t voter_recover_samples;
    double imu_loss_abort_delay_s;
    double gyro_bias_time_constant_s;
    double stationary_gyro_threshold_rad_s;
    double altitude_filter_tau_s;
    double velocity_filter_tau_s;
    double command_timeout_s;
    double launch_confirm_timeout_s;
    double separation_confirm_timeout_s;
    double stage2_ignition_timeout_s;
    double drogue_confirm_timeout_s;
    double main_confirm_timeout_s;
    double fault_recovery_persistence_s;
    double min_step_s;
    double max_step_s;
    double loop_deadline_s;
    uint32_t overrun_abort_count;
    double propulsion_abort_health_percent;
    double propulsion_abort_persistence_s;
    double max_acceleration_m_s2;
    double max_gyro_rad_s;
    double min_magnetic_norm;
    double max_magnetic_norm;
    double min_barometer_altitude_m;
    double max_barometer_altitude_m;
    double max_barometer_rate_m_s;
    double min_gnss_radius_m;
    double max_gnss_radius_m;
    double max_gnss_speed_m_s;
    double max_gnss_velocity_rate_m_s2;
    double accelerometer_process_sigma_m_s2;
    double gyro_process_sigma_rad_s;
    double barometer_sigma_m;
    double gnss_altitude_sigma_m;
    double gnss_velocity_sigma_m_s;
    double max_altitude_sigma_m;
    double max_velocity_sigma_m_s;
    double max_attitude_sigma_rad;
    uint32_t guidance_count;
    FswGuidancePoint guidance[FSW_MAX_GUIDANCE_POINTS];
    int32_t body_role;
} FswConfig;

typedef struct {
    double acceleration_body_m_s2[3];
    double gyro_body_rad_s[3];
    double magnetic_body[3];
    double sample_time_s;
    int32_t valid;
} FswImuSample;

typedef struct {
    double altitude_m;
    double sample_time_s;
    int32_t valid;
} FswBarometerSample;

typedef struct {
    double gnss_position_ecef_m[3];
    double gnss_velocity_ecef_m_s[3];
    double vertical_velocity_m_s;
    double sample_time_s;
    int32_t valid;
} FswGnssSample;

typedef struct {
    double time_s;
    double dt_s;
    uint32_t imu_count;
    uint32_t barometer_count;
    uint32_t gnss_count;
    FswImuSample imus[FSW_MAX_SENSOR_CHANNELS];
    FswBarometerSample barometers[FSW_MAX_SENSOR_CHANNELS];
    FswGnssSample gnss[FSW_MAX_SENSOR_CHANNELS];
} FswSensorSuite;

typedef struct {
    double dynamic_pressure_pa;
    double sample_time_s;
    int32_t valid;
} FswAirDataSample;

typedef struct {
    double health_percent;
    double sample_time_s;
    int32_t valid;
    int32_t ready;
    int32_t running;
} FswPropulsionStatus;

typedef struct {
    double sample_time_s;
    int32_t valid;
    int32_t asserted;
} FswDiscreteSample;

typedef struct {
    FswDiscreteSample stage_separated;
    FswDiscreteSample drogue_deployed;
    FswDiscreteSample main_deployed;
} FswDiscreteInputs;

typedef struct {
    double sample_time_s;
    double previous_execution_time_s;
    int32_t valid;
    int32_t deadline_missed;
    int32_t watchdog_healthy;
} FswPlatformStatus;

typedef struct {
    uint64_t sequence;
    double issue_time_s;
    int32_t type;
} FswCommand;

typedef struct {
    uint32_t abi_version;
    uint32_t struct_size;
    FswSensorSuite sensors;
    FswAirDataSample air_data;
    FswPropulsionStatus propulsion;
    FswDiscreteInputs discretes;
    FswPlatformStatus platform;
    FswCommand command;
} FswInput;

typedef struct {
    uint32_t abi_version;
    uint32_t struct_size;
    int32_t output_valid;
    int32_t step_status;
    int32_t mode;
    int32_t navigation_status;
    int32_t stage1_ignite;
    int32_t stage_separate;
    int32_t stage2_ignite;
    int32_t deploy_drogue;
    int32_t deploy_main;
    int32_t abort;
    int32_t attitude_valid;
    uint64_t command_sequence;
    int32_t command_type;
    int32_t command_result;
    uint32_t inhibit_flags;
    uint32_t event_flags;
    uint32_t imu_usable_mask;
    uint32_t barometer_usable_mask;
    uint32_t gnss_usable_mask;
    uint32_t imu_rejected_mask;
    uint32_t barometer_rejected_mask;
    uint32_t gnss_rejected_mask;
    uint32_t disagreement_flags;
    uint32_t sensor_status_flags;
    uint32_t imu_health_flags[FSW_MAX_SENSOR_CHANNELS];
    uint32_t barometer_health_flags[FSW_MAX_SENSOR_CHANNELS];
    uint32_t gnss_health_flags[FSW_MAX_SENSOR_CHANNELS];
    double imu_age_s[FSW_MAX_SENSOR_CHANNELS];
    double barometer_age_s[FSW_MAX_SENSOR_CHANNELS];
    double gnss_age_s[FSW_MAX_SENSOR_CHANNELS];
    double estimated_altitude_m;
    double estimated_vertical_velocity_m_s;
    double estimated_attitude_wxyz[4];
    double altitude_sigma_m;
    double vertical_velocity_sigma_m_s;
    double attitude_sigma_rad[3];
    double barometer_innovation_m;
    double gnss_altitude_innovation_m;
    double gnss_velocity_innovation_m_s;
    double gyro_bias_rad_s[3];
    double tvc_pitch_rad;
    double tvc_yaw_rad;
    double fin_roll_rad;
    double fin_pitch_rad;
    double fin_yaw_rad;
    uint32_t active_fault_flags;
    uint32_t latched_fault_flags;
    uint32_t changed_fault_flags;
    uint32_t fault_occurrence_count[FSW_FAULT_COUNT];
    int32_t highest_fault_severity;
    double previous_execution_time_s;
    uint32_t consecutive_overruns;
} FswOutput;

typedef void* FswHandle;

FSW_API uint32_t fsw_abi_version(void);
FSW_API FswHandle fsw_create(const FswConfig* config);
FSW_API void fsw_reset(FswHandle handle);
FSW_API int32_t fsw_step(
    FswHandle handle,
    const FswInput* input,
    FswOutput* output
);
FSW_API void fsw_destroy(FswHandle handle);
FSW_API const char* fsw_version(void);

#ifdef __cplusplus
}
#endif

#endif
