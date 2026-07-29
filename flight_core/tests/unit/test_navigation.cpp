#include "test_support.hpp"

using namespace fsw_test;

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

int main() {
    test_stale_imu_is_not_reintegrated();
    test_ecef_gravity_rotation_and_vertical_derivation();
    return EXIT_SUCCESS;
}
