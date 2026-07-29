#include "test_support.hpp"

#include "core/context.hpp"
#include "navigation/navigation.hpp"
#include "sensors/sensor_voting.hpp"

using namespace fsw_test;

fsw::internal::VotedSensors launch_site_measurement(double time_s) {
    fsw::internal::VotedSensors voted{};
    voted.gnss_valid = true;
    voted.barometer_valid = true;
    voted.gnss_position = {kEarthRadiusM, 0.0, 0.0};
    voted.gnss_velocity = {0.0, 0.0, 0.0};
    voted.barometric_altitude = 0.0;
    voted.gnss_sample_time_s = time_s;
    voted.barometer_sample_time_s = time_s;
    return voted;
}

void test_launch_site_initialization_and_aiding() {
    fsw::internal::Context context(default_config());
    context.timing.step_delta_s = 0.01;
    auto voted = launch_site_measurement(0.0);
    REQUIRE(!fsw::internal::update_navigation(context, voted));
    REQUIRE(context.navigation.initialized);
    REQUIRE(context.navigation.navigation_status == FSW_NAV_NOMINAL);
    REQUIRE(context.navigation.position_ecef[0] == kEarthRadiusM);
    REQUIRE(context.navigation.altitude == 0.0);
    REQUIRE(context.navigation.attitude_initialized);

    voted = {};
    voted.barometer_valid = true;
    voted.barometric_altitude = 100.0;
    voted.barometer_sample_time_s = 0.10;
    context.timing.step_delta_s = 0.10;
    REQUIRE(!fsw::internal::update_navigation(context, voted));
    REQUIRE(context.navigation.altitude > 0.0);
    REQUIRE(context.navigation.altitude < 100.0);
    REQUIRE(context.navigation.navigation_status == FSW_NAV_DEGRADED);

    fsw::internal::Context gnss_context(default_config());
    gnss_context.timing.step_delta_s = 0.01;
    voted = launch_site_measurement(0.0);
    fsw::internal::update_navigation(gnss_context, voted);
    voted = {};
    voted.gnss_valid = true;
    voted.gnss_position = {kEarthRadiusM + 100.0, 0.0, 0.0};
    voted.gnss_velocity = {10.0, 0.0, 0.0};
    voted.vertical_velocity = 10.0;
    voted.gnss_sample_time_s = 0.10;
    gnss_context.timing.step_delta_s = 0.10;
    fsw::internal::update_navigation(gnss_context, voted);
    REQUIRE(gnss_context.navigation.position_ecef[0] > kEarthRadiusM);
    REQUIRE(gnss_context.navigation.position_ecef[0]
        < kEarthRadiusM + 100.0);
    REQUIRE(gnss_context.navigation.velocity_ecef[0] > 0.0);
    REQUIRE(gnss_context.navigation.navigation_status == FSW_NAV_DEGRADED);
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
    REQUIRE(output.navigation_status == FSW_NAV_INERTIAL);
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
    test_launch_site_initialization_and_aiding();
    test_stale_imu_is_not_reintegrated();
    test_ecef_gravity_rotation_and_vertical_derivation();
    return EXIT_SUCCESS;
}
