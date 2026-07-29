#include "test_support.hpp"

#include <cmath>

#include "core/context.hpp"
#include "navigation/attitude.hpp"

using namespace fsw_test;

double quaternion_norm(const double quaternion[4]) {
    return std::sqrt(
        quaternion[0] * quaternion[0]
        + quaternion[1] * quaternion[1]
        + quaternion[2] * quaternion[2]
        + quaternion[3] * quaternion[3]
    );
}

void test_quaternion_normalization() {
    fsw::internal::Context context(default_config());
    context.navigation.attitude_initialized = true;
    fsw::internal::integrate_attitude(
        context, {0.0, 0.0, 0.1}, 1.0
    );
    const auto relative = fsw::internal::relative_attitude(context);
    const double norm = std::sqrt(
        relative[0] * relative[0]
        + relative[1] * relative[1]
        + relative[2] * relative[2]
        + relative[3] * relative[3]
    );
    REQUIRE(std::abs(norm - 1.0) < 1e-12);
    REQUIRE(relative[3] > 0.0);
}

void test_stationary_body_earth_rate_compensation() {
    auto config = default_config();
    config.gyro_bias_time_constant_s = 1.0e9;
    FswHandle handle = fsw_create(&config);
    REQUIRE(handle != nullptr);
    FswOutput output{};
    auto input = input_at(0.0);
    input.sensors.imus[0].gyro_body_rad_s[2] = kEarthRotationRadS;
    step(handle, input, output);
    for (int tick = 1; tick <= 100; ++tick) {
        input = input_at(tick * 0.01);
        input.sensors.imus[0].gyro_body_rad_s[2] =
            kEarthRotationRadS;
        step(handle, input, output);
    }
    REQUIRE(std::abs(output.estimated_attitude_wxyz[3]) < 1e-10);
    REQUIRE(std::abs(quaternion_norm(output.estimated_attitude_wxyz) - 1.0)
        < 1e-12);
    fsw_destroy(handle);
}

void test_inertially_fixed_body_rotates_relative_to_ecef() {
    auto config = default_config();
    config.gyro_bias_time_constant_s = 1.0e9;
    FswHandle handle = fsw_create(&config);
    REQUIRE(handle != nullptr);
    FswOutput output{};
    auto input = input_at(0.0);
    step(handle, input, output);
    for (int tick = 1; tick <= 100; ++tick) {
        input = input_at(tick * 0.01);
        step(handle, input, output);
    }
    REQUIRE(output.estimated_attitude_wxyz[3] < 0.0);
    REQUIRE(
        std::abs(
            output.estimated_attitude_wxyz[3]
            + 0.5 * kEarthRotationRadS
        ) < 1e-8
    );
    fsw_destroy(handle);
}

void test_invalid_gyro_does_not_propagate_attitude() {
    const auto config = default_config();
    FswHandle handle = fsw_create(&config);
    REQUIRE(handle != nullptr);
    FswOutput output{};
    auto input = input_at(0.0);
    step(handle, input, output);
    double initial_attitude[4]{};
    for (int index = 0; index < 4; ++index) {
        initial_attitude[index] = output.estimated_attitude_wxyz[index];
    }

    input = input_at(0.01, 20.0);
    input.sensors.imus[0].gyro_valid = 0;
    step(handle, input, output);
    REQUIRE(!output.attitude_valid);
    REQUIRE(output.accelerometer_usable_mask == 1);
    REQUIRE(output.gyroscope_usable_mask == 0);
    for (int index = 0; index < 4; ++index) {
        REQUIRE(
            output.estimated_attitude_wxyz[index]
            == initial_attitude[index]
        );
    }
    fsw_destroy(handle);
}

int main() {
    test_quaternion_normalization();
    test_stationary_body_earth_rate_compensation();
    test_inertially_fixed_body_rotates_relative_to_ecef();
    test_invalid_gyro_does_not_propagate_attitude();
    return EXIT_SUCCESS;
}
