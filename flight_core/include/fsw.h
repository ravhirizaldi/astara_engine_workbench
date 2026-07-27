#ifndef FSW_H
#define FSW_H

#include <stdint.h>

#define FSW_MAX_GUIDANCE_POINTS 32
#define FSW_MAX_SENSOR_CHANNELS 3

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
    FSW_STATUS_INVALID_SAMPLE = -2
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

enum FswFaultFlag {
    FSW_FAULT_GNSS_UNAVAILABLE = 1u << 0,
    FSW_FAULT_BAROMETER_UNAVAILABLE = 1u << 1,
    FSW_FAULT_ENGINE_HEALTH = 1u << 2,
    FSW_FAULT_NAV_INERTIAL = 1u << 3,
    FSW_FAULT_IMU_UNAVAILABLE = 1u << 4,
    FSW_FAULT_IMU_DISAGREEMENT = 1u << 5,
    FSW_FAULT_BAROMETER_DISAGREEMENT = 1u << 6,
    FSW_FAULT_GNSS_DISAGREEMENT = 1u << 7,
    FSW_FAULT_NAV_DISAGREEMENT = 1u << 8
};

enum FswDisagreementFlag {
    FSW_DISAGREEMENT_ACCELERATION = 1u << 0,
    FSW_DISAGREEMENT_GYRO = 1u << 1,
    FSW_DISAGREEMENT_BAROMETER = 1u << 2,
    FSW_DISAGREEMENT_GNSS_POSITION = 1u << 3,
    FSW_DISAGREEMENT_GNSS_VELOCITY = 1u << 4,
    FSW_DISAGREEMENT_CROSS_ALTITUDE = 1u << 5
};

enum FswSensorStatusFlag {
    FSW_SENSOR_STATUS_IMU_SINGLE_SOURCE = 1u << 0,
    FSW_SENSOR_STATUS_BAROMETER_SINGLE_SOURCE = 1u << 1,
    FSW_SENSOR_STATUS_GNSS_SINGLE_SOURCE = 1u << 2
};

typedef struct {
    double time_s;
    double pitch_rad;
    double azimuth_rad;
} FswGuidancePoint;

typedef struct {
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
    double dynamic_pressure_pa;
    double engine_health_percent;
    int32_t stage_separated;
} FswSensorSuite;

typedef struct {
    int32_t mode;
    int32_t navigation_status;
    int32_t stage_separate;
    int32_t stage2_ignite;
    int32_t deploy_drogue;
    int32_t deploy_main;
    int32_t abort;
    int32_t attitude_valid;
    uint32_t imu_usable_mask;
    uint32_t barometer_usable_mask;
    uint32_t gnss_usable_mask;
    uint32_t imu_rejected_mask;
    uint32_t barometer_rejected_mask;
    uint32_t gnss_rejected_mask;
    uint32_t disagreement_flags;
    uint32_t sensor_status_flags;
    double estimated_altitude_m;
    double estimated_vertical_velocity_m_s;
    double estimated_attitude_wxyz[4];
    double gyro_bias_rad_s[3];
    double tvc_pitch_rad;
    double tvc_yaw_rad;
    double fin_roll_rad;
    double fin_pitch_rad;
    double fin_yaw_rad;
    uint32_t fault_flags;
} FswOutput;

typedef void* FswHandle;

FSW_API FswHandle fsw_create(const FswConfig* config);
FSW_API void fsw_reset(FswHandle handle);
FSW_API int32_t fsw_step(
    FswHandle handle,
    const FswSensorSuite* sensor,
    FswOutput* output
);
FSW_API void fsw_destroy(FswHandle handle);
FSW_API const char* fsw_version(void);

#ifdef __cplusplus
}
#endif

#endif
