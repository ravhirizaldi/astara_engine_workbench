#pragma once

#include <array>

#include "fsw/fsw.h"
#include "sensors/channel_health.hpp"

namespace fsw::internal {

struct Context {
    explicit Context(const FswConfig& supplied) : config(supplied) { reset(); }

    void reset() {
        mode = config.body_role == FSW_BODY_CORE
            ? FSW_MODE_COAST
            : (
                config.body_role == FSW_BODY_UPPER
                    ? FSW_MODE_INTERSTAGE
                    : FSW_MODE_SAFE
            );
        altitude = 0.0;
        vertical_velocity = 0.0;
        position_ecef = {};
        velocity_ecef = {};
        attitude = {1.0, 0.0, 0.0, 0.0};
        launch_reference_attitude = {1.0, 0.0, 0.0, 0.0};
        last_gyro = {0.0, 0.0, 0.0};
        gyro_bias = {0.0, 0.0, 0.0};
        accelerometer_bias = {0.0, 0.0, 0.0};
        imu_health = {};
        magnetometer_health = {};
        barometer_health = {};
        gnss_health = {};
        apogee_seen = false;
        drogue_deployed = false;
        main_deployed = false;
        active_fault_flags = 0;
        latched_fault_flags = 0;
        previous_active_fault_flags = 0;
        changed_fault_flags = 0;
        fault_occurrence_count = {};
        highest_fault_severity = FSW_SEVERITY_NONE;
        disagreement_flags = 0;
        sensor_status_flags = 0;
        imu_usable_mask = 0;
        magnetometer_usable_mask = 0;
        barometer_usable_mask = 0;
        gnss_usable_mask = 0;
        imu_rejected_mask = 0;
        magnetometer_rejected_mask = 0;
        barometer_rejected_mask = 0;
        gnss_rejected_mask = 0;
        attitude_valid = false;
        attitude_initialized = false;
        navigation_status = FSW_NAV_INERTIAL;
        navigation_initialized = false;
        gnss_altitude_reference_initialized = false;
        gnss_radius_reference_m = 0.0;
        time_initialized = false;
        last_time_s = 0.0;
        step_delta_s = 0.0;
        input_timing_mismatch = false;
        last_integrated_imu_sample_time_s = -1.0;
        last_barometer_sample_time_s = -1.0;
        last_gnss_sample_time_s = -1.0;
        launch_evidence_s = 0.0;
        burnout_evidence_s = 0.0;
        apogee_evidence_s = 0.0;
        landing_evidence_s = 0.0;
        imu_loss_evidence_s = 0.0;
        propulsion_loss_evidence_s = 0.0;
        fault_healthy_time_s = {};
        launch_commanded_s = -1.0;
        ignition_confirmed_s = -1.0;
        burnout_detected_s = -1.0;
        separation_commanded_s = -1.0;
        separation_confirmed_s = -1.0;
        stage2_ignition_commanded_s = -1.0;
        stage2_ignition_confirmed_s = -1.0;
        drogue_commanded_s = -1.0;
        main_commanded_s = -1.0;
        next_discrete_actuation_sequence = 1;
        discrete_actuation = {};
        last_command_sequence = 0;
        command_sequence = 0;
        command_type = FSW_COMMAND_NONE;
        command_result = FSW_COMMAND_NOT_PROCESSED;
        inhibit_flags = 0;
        event_flags = 0;
        stage1_ignite_request = false;
        stage2_ignite_request = false;
        previous_mode = mode;
        altitude_variance = config.barometer_sigma_m
            * config.barometer_sigma_m;
        velocity_variance = config.gnss_velocity_sigma_m_s
            * config.gnss_velocity_sigma_m_s;
        attitude_variance = {
            config.gyro_process_sigma_rad_s
                * config.gyro_process_sigma_rad_s,
            config.gyro_process_sigma_rad_s
                * config.gyro_process_sigma_rad_s,
            config.gyro_process_sigma_rad_s
                * config.gyro_process_sigma_rad_s,
        };
        barometer_innovation = 0.0;
        gnss_altitude_innovation = 0.0;
        gnss_velocity_innovation = 0.0;
        consecutive_overruns = 0;
        previous_execution_time_s = 0.0;
    }

    FswConfig config;
    int32_t mode{};
    double altitude{};
    double vertical_velocity{};
    std::array<double, 3> position_ecef{};
    std::array<double, 3> velocity_ecef{};
    std::array<double, 4> attitude{};
    std::array<double, 4> launch_reference_attitude{};
    std::array<double, 3> last_gyro{};
    std::array<double, 3> gyro_bias{};
    std::array<double, 3> accelerometer_bias{};
    std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS> imu_health{};
    std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS>
        magnetometer_health{};
    std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS> barometer_health{};
    std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS> gnss_health{};
    bool apogee_seen{};
    bool drogue_deployed{};
    bool main_deployed{};
    uint32_t active_fault_flags{};
    uint32_t latched_fault_flags{};
    uint32_t previous_active_fault_flags{};
    uint32_t changed_fault_flags{};
    std::array<uint32_t, FSW_FAULT_COUNT> fault_occurrence_count{};
    int32_t highest_fault_severity{};
    uint32_t disagreement_flags{};
    uint32_t sensor_status_flags{};
    uint32_t imu_usable_mask{};
    uint32_t magnetometer_usable_mask{};
    uint32_t barometer_usable_mask{};
    uint32_t gnss_usable_mask{};
    uint32_t imu_rejected_mask{};
    uint32_t magnetometer_rejected_mask{};
    uint32_t barometer_rejected_mask{};
    uint32_t gnss_rejected_mask{};
    bool attitude_valid{};
    bool attitude_initialized{};
    int32_t navigation_status{};
    bool navigation_initialized{};
    bool gnss_altitude_reference_initialized{};
    double gnss_radius_reference_m{};
    bool time_initialized{};
    double last_time_s{};
    double step_delta_s{};
    bool input_timing_mismatch{};
    double last_integrated_imu_sample_time_s{};
    double last_barometer_sample_time_s{};
    double last_gnss_sample_time_s{};
    double launch_evidence_s{};
    double burnout_evidence_s{};
    double apogee_evidence_s{};
    double landing_evidence_s{};
    double imu_loss_evidence_s{};
    double propulsion_loss_evidence_s{};
    std::array<double, FSW_FAULT_COUNT> fault_healthy_time_s{};
    double launch_commanded_s{};
    double ignition_confirmed_s{};
    double burnout_detected_s{};
    double separation_commanded_s{};
    double separation_confirmed_s{};
    double stage2_ignition_commanded_s{};
    double stage2_ignition_confirmed_s{};
    double drogue_commanded_s{};
    double main_commanded_s{};
    uint64_t next_discrete_actuation_sequence{};
    FswDiscreteActuationCommand discrete_actuation{};
    uint64_t last_command_sequence{};
    uint64_t command_sequence{};
    int32_t command_type{};
    int32_t command_result{};
    uint32_t inhibit_flags{};
    uint32_t event_flags{};
    bool stage1_ignite_request{};
    bool stage2_ignite_request{};
    int32_t previous_mode{};
    double altitude_variance{};
    double velocity_variance{};
    std::array<double, 3> attitude_variance{};
    double barometer_innovation{};
    double gnss_altitude_innovation{};
    double gnss_velocity_innovation{};
    uint32_t consecutive_overruns{};
    double previous_execution_time_s{};
};

}  // namespace fsw::internal
