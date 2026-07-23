#ifndef ASTARA_FSW_H
#define ASTARA_FSW_H

#include <stdint.h>

#if defined(_WIN32)
#define ASTARA_API __declspec(dllexport)
#else
#define ASTARA_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum AstaraFlightMode {
    ASTARA_SAFE = 0,
    ASTARA_ARMED = 1,
    ASTARA_IGNITION = 2,
    ASTARA_BOOST_1 = 3,
    ASTARA_SEPARATION = 4,
    ASTARA_INTERSTAGE = 5,
    ASTARA_BOOST_2 = 6,
    ASTARA_COAST = 7,
    ASTARA_APOGEE = 8,
    ASTARA_DROGUE = 9,
    ASTARA_MAIN = 10,
    ASTARA_LANDED = 11,
    ASTARA_ABORT = 12
};

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
    double pitch_start_s;
    double pitch_end_s;
    double max_pitch_rad;
    double target_azimuth_rad;
    int32_t body_role;
} AstaraFswConfig;

typedef struct {
    double time_s;
    double dt_s;
    double acceleration_body_m_s2[3];
    double gyro_body_rad_s[3];
    double magnetic_body[3];
    double barometric_altitude_m;
    double gnss_position_ecef_m[3];
    double gnss_velocity_ecef_m_s[3];
    double vertical_velocity_m_s;
    double dynamic_pressure_pa;
    double engine_health_percent;
    int32_t gnss_valid;
    int32_t barometer_valid;
    int32_t stage_separated;
} AstaraSensorFrame;

typedef struct {
    int32_t mode;
    int32_t stage_separate;
    int32_t stage2_ignite;
    int32_t deploy_drogue;
    int32_t deploy_main;
    int32_t abort;
    double estimated_altitude_m;
    double estimated_vertical_velocity_m_s;
    double estimated_attitude_wxyz[4];
    double tvc_pitch_rad;
    double tvc_yaw_rad;
    double fin_roll_rad;
    double fin_pitch_rad;
    double fin_yaw_rad;
    uint32_t fault_flags;
} AstaraFswOutput;

typedef void* AstaraFswHandle;

ASTARA_API AstaraFswHandle astara_fsw_create(const AstaraFswConfig* config);
ASTARA_API void astara_fsw_reset(AstaraFswHandle handle);
ASTARA_API int32_t astara_fsw_step(
    AstaraFswHandle handle,
    const AstaraSensorFrame* sensor,
    AstaraFswOutput* output
);
ASTARA_API void astara_fsw_destroy(AstaraFswHandle handle);
ASTARA_API const char* astara_fsw_version(void);

#ifdef __cplusplus
}
#endif

#endif
