#pragma once

#include <array>

#include "fsw/fsw.h"
#include "sensors/channel_health.hpp"

namespace fsw::internal {

struct TimingState {
    bool initialized{};
    double last_time_s{};
    double step_delta_s{};
    bool input_timing_mismatch{};
    double last_integrated_accelerometer_sample_time_s{-1.0};
    double last_integrated_gyroscope_sample_time_s{-1.0};
    double last_barometer_sample_time_s{-1.0};
    double last_gnss_sample_time_s{-1.0};
    uint32_t consecutive_overruns{};
    double previous_execution_time_s{};
};

struct SensorState {
    std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS> accelerometer_health{};
    std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS> gyroscope_health{};
    std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS> magnetometer_health{};
    std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS> barometer_health{};
    std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS> gnss_health{};
    uint32_t disagreement_flags{};
    uint32_t sensor_status_flags{};
    uint32_t accelerometer_usable_mask{};
    uint32_t gyroscope_usable_mask{};
    uint32_t magnetometer_usable_mask{};
    uint32_t barometer_usable_mask{};
    uint32_t gnss_usable_mask{};
    uint32_t accelerometer_rejected_mask{};
    uint32_t gyroscope_rejected_mask{};
    uint32_t magnetometer_rejected_mask{};
    uint32_t barometer_rejected_mask{};
    uint32_t gnss_rejected_mask{};
};

struct NavigationState {
    double altitude{};
    double vertical_velocity{};
    std::array<double, 3> position_ecef{};
    std::array<double, 3> velocity_ecef{};
    std::array<double, 4> attitude{1.0, 0.0, 0.0, 0.0};
    std::array<double, 4> launch_reference_attitude{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> last_gyro{};
    std::array<double, 3> gyro_bias{};
    std::array<double, 3> accelerometer_bias{};
    bool attitude_valid{};
    bool attitude_initialized{};
    int32_t navigation_status{FSW_NAV_INERTIAL};
    bool initialized{};
    bool gnss_altitude_reference_initialized{};
    double gnss_radius_reference_m{};
    double altitude_variance{};
    double velocity_variance{};
    std::array<double, 3> attitude_variance{};
    double barometer_innovation{};
    double gnss_altitude_innovation{};
    double gnss_velocity_innovation{};
};

struct MissionState {
    int32_t mode{};
    int32_t previous_mode{};
    bool apogee_seen{};
    bool drogue_deployed{};
    bool main_deployed{};
    double launch_evidence_s{};
    double burnout_evidence_s{};
    double apogee_evidence_s{};
    double landing_evidence_s{};
    double imu_loss_evidence_s{};
    double propulsion_loss_evidence_s{};
    double launch_commanded_s{-1.0};
    double ignition_confirmed_s{-1.0};
    double burnout_detected_s{-1.0};
    double separation_commanded_s{-1.0};
    double separation_confirmed_s{-1.0};
    double stage2_ignition_commanded_s{-1.0};
    double stage2_ignition_confirmed_s{-1.0};
    double circularization_ignition_s{-1.0};
    double orbit_achieved_s{-1.0};
    bool payload_deploy_requested{};
    double drogue_commanded_s{-1.0};
    double main_commanded_s{-1.0};
    uint64_t next_discrete_actuation_sequence{1};
    FswDiscreteActuationCommand discrete_actuation{};
};

struct FaultState {
    uint32_t active_fault_flags{};
    uint32_t latched_fault_flags{};
    uint32_t previous_active_fault_flags{};
    uint32_t changed_fault_flags{};
    std::array<uint32_t, FSW_FAULT_COUNT> fault_occurrence_count{};
    int32_t highest_fault_severity{FSW_SEVERITY_NONE};
    std::array<double, FSW_FAULT_COUNT> fault_healthy_time_s{};
};

struct ControlState {
    uint64_t last_command_sequence{};
    uint64_t command_sequence{};
    int32_t command_type{FSW_COMMAND_NONE};
    int32_t command_result{FSW_COMMAND_NOT_PROCESSED};
    uint32_t inhibit_flags{};
    uint32_t event_flags{};
    bool stage1_ignite_request{};
    bool stage2_ignite_request{};
    bool stage2_shutdown_request{};
};

struct Context {
    explicit Context(const FswConfig& supplied) : config(supplied) { reset(); }

    void reset() {
        timing = {};
        sensors = {};
        navigation = {};
        mission = {};
        faults = {};
        control = {};
        navigation.altitude_variance = config.barometer_sigma_m
            * config.barometer_sigma_m;
        navigation.velocity_variance = config.gnss_velocity_sigma_m_s
            * config.gnss_velocity_sigma_m_s;
        navigation.attitude_variance = {
            config.gyro_process_sigma_rad_s * config.gyro_process_sigma_rad_s,
            config.gyro_process_sigma_rad_s * config.gyro_process_sigma_rad_s,
            config.gyro_process_sigma_rad_s * config.gyro_process_sigma_rad_s,
        };
        mission.mode = config.body_role == FSW_BODY_CORE
            ? FSW_MODE_COAST
            : (config.body_role == FSW_BODY_UPPER
                ? FSW_MODE_INTERSTAGE
                : FSW_MODE_SAFE);
        mission.previous_mode = mission.mode;
    }

    FswConfig config;
    TimingState timing;
    SensorState sensors;
    NavigationState navigation;
    MissionState mission;
    FaultState faults;
    ControlState control;
};

}  // namespace fsw::internal
