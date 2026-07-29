#include "test_support.hpp"

using namespace fsw_test;

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

int main() {
    test_independent_magnetometer_health();
    test_voting_and_outlier_baseline_recovery();
    return EXIT_SUCCESS;
}
