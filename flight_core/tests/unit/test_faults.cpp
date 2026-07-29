#include "test_support.hpp"

using namespace fsw_test;

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

int main() {
    test_timing_timeouts_and_zero_safe_rejection();
    test_one_shot_recovery_commands_and_confirmation_timeout();
    return EXIT_SUCCESS;
}
