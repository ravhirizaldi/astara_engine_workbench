#include "fsw.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {

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

void test_delayed_launch_event_timing_and_one_shot_separation() {
    const FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    REQUIRE(handle != nullptr);
    FswOutput output{};
    for (int tick = 0; tick < 500; ++tick) {
        auto input = input_at(tick * 0.01);
        step(handle, input, output);
    }
    arm_and_launch(handle, output, 500);

    int separation_pulses = 0;
    uint64_t separation_sequence = 0;
    int tick = 502;
    for (; tick < 580 && output.mode != FSW_MODE_INTERSTAGE; ++tick) {
        auto input = input_at(
            tick * 0.01,
            (
                output.mode == FSW_MODE_IGNITION
                || (
                    output.mode == FSW_MODE_BOOST_1
                    && tick < 528
                )
            ) ? 20.0 : 0.0
        );
        input.propulsion.running =
            output.mode == FSW_MODE_IGNITION
            || (
                output.mode == FSW_MODE_BOOST_1
                && tick < 528
            );
        if (separation_pulses > 0) {
            input.discretes.stage_separated.asserted = 1;
        }
        step(handle, input, output);
        if (output.stage_separate) {
            ++separation_pulses;
            separation_sequence =
                output.discrete_actuation.sequence;
            REQUIRE(
                output.discrete_actuation.action
                == FSW_DISCRETE_ACTION_STAGE_SEPARATE
            );
        }
    }
    REQUIRE(output.mode == FSW_MODE_INTERSTAGE);
    REQUIRE(separation_pulses == 1);
    REQUIRE(separation_sequence > 0);

    int stage2_pulses = 0;
    for (; tick < 650 && output.mode != FSW_MODE_BOOST_2; ++tick) {
        auto input = input_at(tick * 0.01, 20.0);
        input.discretes.stage_separated.asserted = 1;
        input.propulsion.running = stage2_pulses > 0;
        step(handle, input, output);
        stage2_pulses += output.stage2_ignite ? 1 : 0;
    }
    REQUIRE(output.mode == FSW_MODE_BOOST_2);
    REQUIRE(stage2_pulses == 1);
    fsw_destroy(handle);
}

void test_stale_imu_is_not_reintegrated() {
    const FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    auto input = input_at(0.0);
    step(handle, input, output);
    input = input_at(0.01, 20.0);
    input.sensors.imus[0].gyro_body_rad_s[2] = 1.0;
    input.sensors.barometers[0].valid = 0;
    input.sensors.gnss[0].valid = 0;
    step(handle, input, output);
    const double velocity = output.estimated_velocity_ecef_m_s[0];
    const double attitude_z = output.estimated_attitude_wxyz[3];
    input = input_at(0.02, 20.0);
    input.sensors.imus[0].sample_time_s = 0.01;
    input.sensors.barometers[0].valid = 0;
    input.sensors.gnss[0].valid = 0;
    step(handle, input, output);
    REQUIRE(output.estimated_velocity_ecef_m_s[0] == velocity);
    REQUIRE(output.estimated_attitude_wxyz[3] == attitude_z);
    fsw_destroy(handle);
}

void test_ecef_gravity_rotation_and_vertical_derivation() {
    const FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    auto input = input_at(0.0);
    step(handle, input, output);
    input = input_at(0.10, 0.0);
    input.sensors.dt_s = 0.10;
    input.sensors.barometers[0].valid = 0;
    input.sensors.gnss[0].valid = 0;
    step(handle, input, output);
    REQUIRE(output.estimated_vertical_velocity_m_s < -0.8);
    const double radius = std::sqrt(
        output.estimated_position_ecef_m[0]
            * output.estimated_position_ecef_m[0]
        + output.estimated_position_ecef_m[1]
            * output.estimated_position_ecef_m[1]
        + output.estimated_position_ecef_m[2]
            * output.estimated_position_ecef_m[2]
    );
    const double radial_velocity = (
        output.estimated_position_ecef_m[0]
            * output.estimated_velocity_ecef_m_s[0]
        + output.estimated_position_ecef_m[1]
            * output.estimated_velocity_ecef_m_s[1]
        + output.estimated_position_ecef_m[2]
            * output.estimated_velocity_ecef_m_s[2]
    ) / radius;
    REQUIRE(
        std::abs(
            radial_velocity
            - output.estimated_vertical_velocity_m_s
        ) < 1e-9
    );
    fsw_destroy(handle);

    handle = fsw_create(&config);
    input = input_at(0.0);
    step(handle, input, output);
    for (int tick = 1; tick <= 100; ++tick) {
        input = input_at(tick * 0.01);
        input.sensors.imus[0].gyro_body_rad_s[2] =
            kEarthRotationRadS;
        step(handle, input, output);
    }
    REQUIRE(std::abs(output.estimated_attitude_wxyz[3]) < 1e-12);
    fsw_destroy(handle);

    handle = fsw_create(&config);
    input = input_at(0.0);
    step(handle, input, output);
    for (int tick = 1; tick <= 10; ++tick) {
        input = input_at(tick * 0.01, 20.0);
        input.sensors.imus[0].gyro_body_rad_s[2] = 10.0;
        input.sensors.barometers[0].valid = 0;
        input.sensors.gnss[0].valid = 0;
        step(handle, input, output);
    }
    REQUIRE(std::abs(output.estimated_attitude_wxyz[3]) > 0.3);
    REQUIRE(std::abs(output.estimated_velocity_ecef_m_s[1]) > 0.2);
    fsw_destroy(handle);
}

void test_independent_magnetometer_health() {
    const FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    auto input = input_at(0.0);
    input.sensors.magnetometers[0].valid = 0;
    step(handle, input, output);
    REQUIRE(output.imu_usable_mask == 1);
    REQUIRE(output.magnetometer_usable_mask == 0);
    REQUIRE(output.attitude_valid);
    input = input_at(0.01);
    input.sensors.magnetometers[0].valid = 0;
    set_command(input, 1, FSW_COMMAND_ARM);
    step(handle, input, output);
    REQUIRE(output.mode == FSW_MODE_ARMED);
    input = input_at(0.02);
    input.sensors.magnetometers[0].valid = 0;
    set_command(input, 2, FSW_COMMAND_LAUNCH);
    step(handle, input, output);
    REQUIRE(output.mode == FSW_MODE_IGNITION);
    for (int tick = 3; tick <= 8; ++tick) {
        input = input_at(tick * 0.01, 20.0);
        input.sensors.magnetometers[0].valid = 0;
        input.propulsion.running = 1;
        step(handle, input, output);
    }
    REQUIRE(output.mode == FSW_MODE_BOOST_1);
    REQUIRE(output.attitude_valid);
    fsw_destroy(handle);
}

void test_voting_and_outlier_baseline_recovery() {
    FswConfig config = default_config();
    config.max_barometer_rate_m_s = 100.0;
    config.max_gnss_velocity_rate_m_s2 = 100.0;
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    for (int tick = 0; tick < 3; ++tick) {
        auto input = input_at(tick * 0.01);
        fill_three_channels(input);
        input.sensors.imus[2].acceleration_body_m_s2[0] = 20.0;
        input.sensors.magnetometers[2].magnetic_body[0] = 0.8;
        input.sensors.magnetometers[2].magnetic_body[1] = 0.6;
        step(handle, input, output);
    }
    REQUIRE(output.imu_rejected_mask & (1u << 2));
    REQUIRE(output.magnetometer_rejected_mask & (1u << 2));
    for (int tick = 3; tick < 8; ++tick) {
        auto input = input_at(tick * 0.01);
        fill_three_channels(input);
        step(handle, input, output);
    }
    REQUIRE((output.imu_rejected_mask & (1u << 2)) == 0);
    REQUIRE(
        (output.magnetometer_rejected_mask & (1u << 2)) == 0
    );
    fsw_destroy(handle);

    handle = fsw_create(&config);
    auto input = input_at(0.0);
    step(handle, input, output);
    input = input_at(0.01);
    input.sensors.barometers[0].altitude_m = 1000.0;
    input.sensors.gnss[0].gnss_velocity_ecef_m_s[0] = 100.0;
    step(handle, input, output);
    REQUIRE(output.barometer_usable_mask == 0);
    REQUIRE(output.gnss_usable_mask == 0);
    input = input_at(0.02);
    input.sensors.barometers[0].altitude_m = 0.5;
    input.sensors.gnss[0].gnss_velocity_ecef_m_s[0] = 0.5;
    step(handle, input, output);
    REQUIRE(output.barometer_usable_mask == 1);
    REQUIRE(output.gnss_usable_mask == 1);
    fsw_destroy(handle);
}

void test_timing_timeouts_and_zero_safe_rejection() {
    const FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    auto input = input_at(0.0);
    step(handle, input, output);
    input = input_at(0.06);
    input.sensors.dt_s = 0.06;
    input.air_data.sample_time_s = 0.0;
    step(handle, input, output);
    REQUIRE(output.active_fault_flags & FSW_FAULT_AIR_DATA_UNAVAILABLE);
    REQUIRE((output.active_fault_flags & FSW_FAULT_IMU_UNAVAILABLE) == 0);
    input = input_at(0.08);
    input.sensors.dt_s = 0.02;
    input.propulsion.sample_time_s = 0.0;
    step(handle, input, output);
    REQUIRE(output.active_fault_flags & FSW_FAULT_PROPULSION_UNAVAILABLE);

    input = input_at(0.10);
    input.sensors.dt_s = 0.01;
    input.platform.sample_time_s = 0.0;
    step(handle, input, output);
    REQUIRE(output.active_fault_flags & FSW_FAULT_INPUT_TIMING);
    REQUIRE(output.active_fault_flags & FSW_FAULT_WATCHDOG);
    const double altitude = output.estimated_altitude_m;
    input = input_at(0.10);
    REQUIRE(
        fsw_step(handle, &input, &output) == FSW_STATUS_INVALID_INPUT
    );
    REQUIRE(!output.output_valid);
    REQUIRE(output.estimated_altitude_m == 0.0);
    input = input_at(0.11);
    step(handle, input, output);
    REQUIRE(std::isfinite(altitude));
    fsw_destroy(handle);
}

void set_altitude_velocity(
    FswInput& input,
    double altitude_m,
    double vertical_velocity_m_s
) {
    input.sensors.barometers[0].altitude_m = altitude_m;
    input.sensors.gnss[0].gnss_position_ecef_m[0] =
        kEarthRadiusM + altitude_m;
    input.sensors.gnss[0].gnss_velocity_ecef_m_s[0] =
        vertical_velocity_m_s;
}

void test_one_shot_recovery_commands_and_confirmation_timeout() {
    FswConfig config = default_config();
    config.body_role = FSW_BODY_CORE;
    config.main_deploy_altitude_m = 600.0;
    config.max_barometer_rate_m_s = 1.0e6;
    config.max_gnss_velocity_rate_m_s2 = 1.0e6;
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    int drogue_pulses = 0;
    for (int tick = 0; tick <= 25; ++tick) {
        auto input = input_at(tick * 0.01, 0.0);
        set_altitude_velocity(input, 500.0, -10.0);
        step(handle, input, output);
        drogue_pulses += output.deploy_drogue ? 1 : 0;
    }
    REQUIRE(output.mode == FSW_MODE_DROGUE);
    REQUIRE(drogue_pulses == 1);

    auto input = input_at(0.26, 0.0);
    set_altitude_velocity(input, 500.0, -10.0);
    input.discretes.drogue_deployed.asserted = 1;
    step(handle, input, output);
    REQUIRE(!output.deploy_drogue);
    REQUIRE(output.mode == FSW_MODE_MAIN);
    REQUIRE(output.deploy_main);
    const uint64_t main_sequence =
        output.discrete_actuation.sequence;
    input = input_at(0.27, 0.0);
    set_altitude_velocity(input, 500.0, -10.0);
    input.discretes.drogue_deployed.asserted = 1;
    step(handle, input, output);
    REQUIRE(!output.deploy_main);
    REQUIRE(output.discrete_actuation.sequence != main_sequence);
    fsw_destroy(handle);

    config.drogue_confirm_timeout_s = 0.05;
    handle = fsw_create(&config);
    drogue_pulses = 0;
    for (int tick = 0; tick <= 35; ++tick) {
        input = input_at(tick * 0.01, 0.0);
        set_altitude_velocity(input, 500.0, -10.0);
        step(handle, input, output);
        drogue_pulses += output.deploy_drogue ? 1 : 0;
    }
    REQUIRE(drogue_pulses == 1);
    REQUIRE(
        output.latched_fault_flags
        & FSW_FAULT_DROGUE_NOT_CONFIRMED
    );
    fsw_destroy(handle);
}

void test_abi_contract_and_release_active_requirements() {
    FswConfig bad_config = default_config();
    bad_config.abi_version = 0;
    REQUIRE(fsw_create(&bad_config) == nullptr);
    const FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    auto input = input_at(0.0);
    input.abi_version = 0;
    REQUIRE(
        fsw_step(handle, &input, &output)
        == FSW_STATUS_ABI_MISMATCH
    );
    REQUIRE(!output.output_valid);
    REQUIRE(!output.stage1_ignite);
    REQUIRE(!output.discrete_actuation.valid);
    fsw_destroy(handle);
}

}  // namespace

int main() {
    REQUIRE(fsw_abi_version() == FSW_ABI_VERSION);
    REQUIRE(std::strcmp(fsw_version(), "fsw-core-0.5.0") == 0);
    test_delayed_launch_event_timing_and_one_shot_separation();
    test_stale_imu_is_not_reintegrated();
    test_ecef_gravity_rotation_and_vertical_derivation();
    test_independent_magnetometer_health();
    test_voting_and_outlier_baseline_recovery();
    test_timing_timeouts_and_zero_safe_rejection();
    test_one_shot_recovery_commands_and_confirmation_timeout();
    test_abi_contract_and_release_active_requirements();
    return EXIT_SUCCESS;
}
