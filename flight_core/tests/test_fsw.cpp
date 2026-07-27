#include "fsw.h"

#include <cassert>
#include <cmath>
#include <cstring>

namespace {

FswConfig default_config() {
    FswConfig config{};
    config.abi_version = FSW_ABI_VERSION;
    config.struct_size = sizeof(FswConfig);
    config.stage1_burn_s = 1.0;
    config.separation_delay_s = 0.1;
    config.stage2_ignition_delay_s = 0.2;
    config.stage2_burn_s = 1.0;
    config.main_deploy_altitude_m = 300.0;
    config.max_tvc_rad = 0.1;
    config.max_fin_rad = 0.1;
    config.control_kp = 0.1;
    config.control_kd = 0.01;
    config.imu_timeout_s = 0.1;
    config.barometer_timeout_s = 0.2;
    config.gnss_timeout_s = 0.5;
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
    config.altitude_filter_tau_s = 0.2;
    config.velocity_filter_tau_s = 0.6;
    config.command_timeout_s = 1.0;
    config.launch_confirm_timeout_s = 0.2;
    config.separation_confirm_timeout_s = 0.5;
    config.stage2_ignition_timeout_s = 0.5;
    config.drogue_confirm_timeout_s = 0.5;
    config.main_confirm_timeout_s = 0.5;
    config.fault_recovery_persistence_s = 0.1;
    config.min_step_s = 0.001;
    config.max_step_s = 0.1;
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
    config.guidance_count = 2;
    config.guidance[0] = {0.0, 0.0, 0.0};
    config.guidance[1] = {10.0, 0.0, 0.0};
    config.body_role = FSW_BODY_INTEGRATED;
    return config;
}

FswInput input_at(double time_s, double acceleration = 0.0) {
    FswInput input{};
    input.abi_version = FSW_ABI_VERSION;
    input.struct_size = sizeof(FswInput);
    input.sensors.time_s = time_s;
    input.sensors.dt_s = 0.01;
    input.sensors.imu_count = 1;
    input.sensors.barometer_count = 1;
    input.sensors.gnss_count = 1;
    input.sensors.imus[0].acceleration_body_m_s2[0] = acceleration;
    input.sensors.imus[0].magnetic_body[0] = 1.0;
    input.sensors.imus[0].sample_time_s = time_s;
    input.sensors.imus[0].valid = 1;
    input.sensors.barometers[0].altitude_m = 0.0;
    input.sensors.barometers[0].sample_time_s = time_s;
    input.sensors.barometers[0].valid = 1;
    input.sensors.gnss[0].gnss_position_ecef_m[0] = 6371000.0;
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

void fill_three_channels(FswInput& input) {
    input.sensors.imu_count = 3;
    input.sensors.barometer_count = 3;
    input.sensors.gnss_count = 3;
    for (int index = 1; index < 3; ++index) {
        input.sensors.imus[index] = input.sensors.imus[0];
        input.sensors.barometers[index] = input.sensors.barometers[0];
        input.sensors.gnss[index] = input.sensors.gnss[0];
    }
}

void test_explicit_arm_launch_and_stage2() {
    FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    assert(handle != nullptr);
    FswOutput output{};

    auto input = input_at(0.0);
    assert(fsw_step(handle, &input, &output) == FSW_STATUS_OK);
    assert(output.output_valid);
    assert(output.mode == FSW_MODE_SAFE);

    input = input_at(0.01);
    set_command(input, 1, FSW_COMMAND_ARM);
    assert(fsw_step(handle, &input, &output) == FSW_STATUS_OK);
    assert(output.mode == FSW_MODE_ARMED);
    assert(output.command_result == FSW_COMMAND_ACCEPTED);

    input = input_at(0.02);
    set_command(input, 2, FSW_COMMAND_LAUNCH);
    assert(fsw_step(handle, &input, &output) == FSW_STATUS_OK);
    assert(output.mode == FSW_MODE_IGNITION);
    assert(output.stage1_ignite);

    for (int step = 3; step <= 8; ++step) {
        input = input_at(step * 0.01, 20.0);
        input.propulsion.running = 1;
        assert(fsw_step(handle, &input, &output) == FSW_STATUS_OK);
    }
    assert(output.mode == FSW_MODE_BOOST_1);

    for (int step = 9; step <= 106; ++step) {
        input = input_at(step * 0.01, step >= 101 ? 0.0 : 20.0);
        input.propulsion.running = step < 101;
        assert(fsw_step(handle, &input, &output) == FSW_STATUS_OK);
    }
    assert(output.mode == FSW_MODE_SEPARATION);

    input = input_at(1.11);
    input.discretes.stage_separated.asserted = 1;
    assert(fsw_step(handle, &input, &output) == FSW_STATUS_OK);
    assert(output.mode == FSW_MODE_INTERSTAGE);

    for (int step = 112; step <= 130; ++step) {
        input = input_at(step * 0.01);
        input.discretes.stage_separated.asserted = 1;
        assert(fsw_step(handle, &input, &output) == FSW_STATUS_OK);
    }
    assert(output.stage2_ignite);
    input = input_at(1.31, 20.0);
    input.discretes.stage_separated.asserted = 1;
    input.propulsion.running = 1;
    assert(fsw_step(handle, &input, &output) == FSW_STATUS_OK);
    assert(output.mode == FSW_MODE_BOOST_2);
    fsw_destroy(handle);
}

void test_command_inhibits_and_launch_timeout() {
    FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};

    auto input = input_at(0.0);
    input.sensors.imus[0].valid = 0;
    set_command(input, 1, FSW_COMMAND_ARM);
    assert(fsw_step(handle, &input, &output) == FSW_STATUS_OK);
    assert(output.mode == FSW_MODE_SAFE);
    assert(output.command_result == FSW_COMMAND_REJECTED_INHIBITED);
    assert(output.inhibit_flags & FSW_INHIBIT_IMU);

    fsw_reset(handle);
    input = input_at(0.0);
    set_command(input, 1, FSW_COMMAND_ARM);
    fsw_step(handle, &input, &output);
    input = input_at(0.01);
    set_command(input, 2, FSW_COMMAND_LAUNCH);
    fsw_step(handle, &input, &output);
    for (int step = 2; step <= 22; ++step) {
        input = input_at(step * 0.01);
        fsw_step(handle, &input, &output);
    }
    assert(output.mode == FSW_MODE_ABORT);
    assert(output.latched_fault_flags & FSW_FAULT_LAUNCH_NOT_CONFIRMED);
    fsw_destroy(handle);
}

void test_voting_rejection_recovery_and_magnetometer() {
    FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    for (int step = 0; step < 3; ++step) {
        auto input = input_at(step * 0.01);
        fill_three_channels(input);
        input.sensors.imus[2].acceleration_body_m_s2[0] = 10.0;
        input.sensors.imus[2].magnetic_body[0] = 0.8;
        input.sensors.imus[2].magnetic_body[1] = 0.6;
        fsw_step(handle, &input, &output);
    }
    assert(output.imu_rejected_mask & (1u << 2));
    assert(output.disagreement_flags & FSW_DISAGREEMENT_ACCELERATION);
    assert(output.disagreement_flags & FSW_DISAGREEMENT_MAGNETOMETER);
    assert(
        output.imu_health_flags[2]
        & FSW_SENSOR_HEALTH_REJECTED
    );

    for (int step = 3; step < 8; ++step) {
        auto input = input_at(step * 0.01);
        fill_three_channels(input);
        fsw_step(handle, &input, &output);
    }
    assert((output.imu_rejected_mask & (1u << 2)) == 0);
    fsw_destroy(handle);
}

void test_two_channel_disagreement_does_not_guess() {
    FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    auto input = input_at(0.0);
    input.sensors.imu_count = 2;
    input.sensors.imus[1] = input.sensors.imus[0];
    input.sensors.imus[1].acceleration_body_m_s2[0] = 10.0;
    fsw_step(handle, &input, &output);
    assert(output.imu_usable_mask == 0);
    assert(output.active_fault_flags & FSW_FAULT_IMU_UNAVAILABLE);
    assert(output.active_fault_flags & FSW_FAULT_IMU_DISAGREEMENT);
    fsw_destroy(handle);
}

void test_fault_latching_watchdog_and_reset() {
    FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    for (int step = 0; step < 3; ++step) {
        auto input = input_at(step * 0.01);
        input.platform.deadline_missed = 1;
        input.platform.previous_execution_time_s = 0.03;
        fsw_step(handle, &input, &output);
    }
    assert(output.active_fault_flags & FSW_FAULT_DEADLINE_OVERRUN);
    assert(output.latched_fault_flags & FSW_FAULT_DEADLINE_OVERRUN);
    assert(output.fault_occurrence_count[12] == 1);

    auto input = input_at(0.03);
    set_command(input, 1, FSW_COMMAND_CLEAR_FAULTS);
    fsw_step(handle, &input, &output);
    assert(output.latched_fault_flags & FSW_FAULT_DEADLINE_OVERRUN);
    fsw_reset(handle);
    input = input_at(0.0);
    fsw_step(handle, &input, &output);
    assert(output.latched_fault_flags == 0);
    fsw_destroy(handle);
}

void test_transient_fault_requires_healthy_persistence() {
    FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    auto input = input_at(0.0);
    input.sensors.gnss[0].valid = 0;
    fsw_step(handle, &input, &output);
    assert(output.active_fault_flags & FSW_FAULT_GNSS_UNAVAILABLE);

    for (int step = 1; step <= 5; ++step) {
        input = input_at(step * 0.01);
        fsw_step(handle, &input, &output);
    }
    assert(output.active_fault_flags & FSW_FAULT_GNSS_UNAVAILABLE);

    for (int step = 6; step <= 12; ++step) {
        input = input_at(step * 0.01);
        fsw_step(handle, &input, &output);
    }
    assert((output.active_fault_flags & FSW_FAULT_GNSS_UNAVAILABLE) == 0);
    assert(output.latched_fault_flags & FSW_FAULT_GNSS_UNAVAILABLE);
    fsw_destroy(handle);
}

void test_invalid_abi_and_input_output_contract() {
    FswConfig bad_config = default_config();
    bad_config.abi_version = 0;
    assert(fsw_create(&bad_config) == nullptr);

    FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    auto input = input_at(0.0);
    input.abi_version = 0;
    assert(
        fsw_step(handle, &input, &output) == FSW_STATUS_ABI_MISMATCH
    );
    assert(!output.output_valid);
    assert(!output.stage1_ignite);
    assert(!output.stage2_ignite);

    input = input_at(0.0);
    input.sensors.dt_s = 1.0;
    assert(fsw_step(handle, &input, &output) == FSW_STATUS_INVALID_INPUT);
    assert(!output.output_valid);
    fsw_destroy(handle);
}

}  // namespace

int main() {
    assert(fsw_abi_version() == FSW_ABI_VERSION);
    assert(std::strcmp(fsw_version(), "fsw-core-0.4.0") == 0);
    test_explicit_arm_launch_and_stage2();
    test_command_inhibits_and_launch_timeout();
    test_voting_rejection_recovery_and_magnetometer();
    test_two_channel_disagreement_does_not_guess();
    test_fault_latching_watchdog_and_reset();
    test_transient_fault_requires_healthy_persistence();
    test_invalid_abi_and_input_output_contract();
    return 0;
}
