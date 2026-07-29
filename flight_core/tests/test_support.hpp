#pragma once

#include "fsw/fsw.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace fsw_test {

void require(bool condition, const char* expression, int line) {
    if (!condition) {
        std::cerr << "REQUIRE failed at line " << line
                  << ": " << expression << '\n';
        std::exit(EXIT_FAILURE);
    }
}

#define REQUIRE(expression) require((expression), #expression, __LINE__)

constexpr double kEarthRadiusM = 6'378'137.0;
constexpr double kEarthRotationRadS = 7.292115e-5;
constexpr double kRestSpecificForceM_S2 = 9.764;

FswConfig default_config() {
    FswConfig config{};
    config.abi_version = FSW_ABI_VERSION;
    config.struct_size = sizeof(FswConfig);
    config.stage1_burn_s = 0.20;
    config.separation_delay_s = 0.10;
    config.stage2_ignition_delay_s = 0.20;
    config.stage2_burn_s = 0.20;
    config.main_deploy_altitude_m = 300.0;
    config.max_tvc_rad = 0.1;
    config.max_fin_rad = 0.1;
    config.control_kp = 0.1;
    config.control_kd = 0.01;
    config.imu_timeout_s = 0.10;
    config.magnetometer_timeout_s = 0.10;
    config.barometer_timeout_s = 0.20;
    config.gnss_timeout_s = 0.50;
    config.air_data_timeout_s = 0.05;
    config.propulsion_status_timeout_s = 0.07;
    config.discrete_feedback_timeout_s = 0.08;
    config.platform_status_timeout_s = 0.09;
    config.acceleration_disagreement_m_s2 = 0.5;
    config.gyro_disagreement_rad_s = 0.01;
    config.magnetic_disagreement = 0.1;
    config.barometer_disagreement_m = 10.0;
    config.gnss_position_disagreement_m = 15.0;
    config.gnss_velocity_disagreement_m_s = 1.0;
    config.cross_altitude_disagreement_m = 20.0;
    config.voter_reject_samples = 3;
    config.voter_recover_samples = 5;
    config.imu_loss_abort_delay_s = 0.05;
    config.gyro_bias_time_constant_s = 2.0;
    config.stationary_gyro_threshold_rad_s = 0.02;
    config.altitude_filter_tau_s = 0.20;
    config.velocity_filter_tau_s = 0.60;
    config.command_timeout_s = 1.0;
    config.launch_confirm_timeout_s = 0.20;
    config.separation_confirm_timeout_s = 0.25;
    config.stage2_ignition_timeout_s = 0.25;
    config.drogue_confirm_timeout_s = 0.25;
    config.main_confirm_timeout_s = 0.25;
    config.fault_recovery_persistence_s = 0.10;
    config.min_step_s = 0.001;
    config.max_step_s = 0.10;
    config.step_time_tolerance_s = 1e-6;
    config.loop_deadline_s = 0.02;
    config.overrun_abort_count = 3;
    config.propulsion_abort_health_percent = 20.0;
    config.propulsion_abort_persistence_s = 0.05;
    config.max_acceleration_m_s2 = 500.0;
    config.max_gyro_rad_s = 20.0;
    config.min_magnetic_norm = 0.25;
    config.max_magnetic_norm = 2.0;
    config.min_barometer_altitude_m = -1000.0;
    config.max_barometer_altitude_m = 2.0e6;
    config.max_barometer_rate_m_s = 5000.0;
    config.min_gnss_radius_m = 5.0e6;
    config.max_gnss_radius_m = 8.0e6;
    config.max_gnss_speed_m_s = 15000.0;
    config.max_gnss_velocity_rate_m_s2 = 1000.0;
    config.accelerometer_process_sigma_m_s2 = 0.05;
    config.gyro_process_sigma_rad_s = 0.001;
    config.barometer_sigma_m = 2.0;
    config.gnss_altitude_sigma_m = 5.0;
    config.gnss_velocity_sigma_m_s = 0.5;
    config.max_altitude_sigma_m = 500.0;
    config.max_velocity_sigma_m_s = 200.0;
    config.max_attitude_sigma_rad = 1.0;
    config.launch_azimuth_rad = 0.0;
    config.guidance_count = 2;
    config.guidance[0] = {0.0, 0.0, 0.0};
    config.guidance[1] = {10.0, 0.0, 0.0};
    config.body_role = FSW_BODY_INTEGRATED;
    return config;
}

FswInput input_at(
    double time_s,
    double acceleration = kRestSpecificForceM_S2
) {
    FswInput input{};
    input.abi_version = FSW_ABI_VERSION;
    input.struct_size = sizeof(FswInput);
    input.sensors.time_s = time_s;
    input.sensors.dt_s = 0.01;
    input.sensors.imu_count = 1;
    input.sensors.magnetometer_count = 1;
    input.sensors.barometer_count = 1;
    input.sensors.gnss_count = 1;
    input.sensors.imus[0].acceleration_body_m_s2[0] =
        acceleration;
    input.sensors.imus[0].sample_time_s = time_s;
    input.sensors.imus[0].accel_valid = 1;
    input.sensors.imus[0].gyro_valid = 1;
    input.sensors.magnetometers[0].magnetic_body[0] = 1.0;
    input.sensors.magnetometers[0].sample_time_s = time_s;
    input.sensors.magnetometers[0].valid = 1;
    input.sensors.barometers[0] = {0.0, time_s, 1};
    input.sensors.gnss[0].gnss_position_ecef_m[0] =
        kEarthRadiusM;
    input.sensors.gnss[0].sample_time_s = time_s;
    input.sensors.gnss[0].valid = 1;
    input.air_data = {1000.0, time_s, 1};
    input.propulsion = {100.0, time_s, 1, 1, 0};
    input.discretes.stage_separated = {time_s, 1, 0};
    input.discretes.drogue_deployed = {time_s, 1, 0};
    input.discretes.main_deployed = {time_s, 1, 0};
    input.platform = {time_s, 0.001, 1, 0, 1};
    return input;
}

void set_command(FswInput& input, uint64_t sequence, int32_t type) {
    input.command = {sequence, input.sensors.time_s, type};
}

void step(FswHandle handle, FswInput& input, FswOutput& output) {
    REQUIRE(fsw_step(handle, &input, &output) == FSW_STATUS_OK);
    REQUIRE(output.output_valid);
}

void fill_three_channels(FswInput& input) {
    input.sensors.imu_count = 3;
    input.sensors.magnetometer_count = 3;
    input.sensors.barometer_count = 3;
    input.sensors.gnss_count = 3;
    for (int index = 1; index < 3; ++index) {
        input.sensors.imus[index] = input.sensors.imus[0];
        input.sensors.magnetometers[index] =
            input.sensors.magnetometers[0];
        input.sensors.barometers[index] =
            input.sensors.barometers[0];
        input.sensors.gnss[index] = input.sensors.gnss[0];
    }
}

void arm_and_launch(
    FswHandle handle,
    FswOutput& output,
    int start_tick
) {
    auto input = input_at(start_tick * 0.01);
    set_command(input, 1, FSW_COMMAND_ARM);
    step(handle, input, output);
    REQUIRE(output.mode == FSW_MODE_ARMED);
    input = input_at((start_tick + 1) * 0.01);
    set_command(input, 2, FSW_COMMAND_LAUNCH);
    step(handle, input, output);
    REQUIRE(output.mode == FSW_MODE_IGNITION);
    REQUIRE(output.stage1_ignite);
}

}  // namespace fsw_test
