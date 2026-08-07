#include "test_support.hpp"

#include <cmath>

#include "control/control.hpp"
#include "core/context.hpp"
#include "guidance/guidance.hpp"

using namespace fsw_test;

fsw::internal::Context control_context(FswConfig config) {
    fsw::internal::Context context(config);
    context.mission.mode = FSW_MODE_BOOST_1;
    context.navigation.attitude_valid = true;
    context.navigation.attitude_initialized = true;
    context.mission.ignition_confirmed_s = 0.0;
    return context;
}

void test_guidance_and_attitude_error_response() {
    auto config = default_config();
    config.guidance[0] = {0.0, 0.2, 0.0};
    config.guidance[1] = {10.0, 0.2, 0.0};
    const auto midpoint = fsw::internal::guidance_at(config, 5.0);
    REQUIRE(std::abs(midpoint.pitch_rad - 0.2) < 1e-12);

    auto context = control_context(config);
    auto input = input_at(5.0);
    input.air_data.valid = 0;
    FswOutput output{};
    fsw::internal::calculate_controls(context, input, output);
    REQUIRE(output.tvc_pitch_rad > 0.0);
    REQUIRE(std::abs(output.tvc_yaw_rad) < 1e-12);
}

void test_rate_damping_and_saturation() {
    auto config = default_config();
    config.guidance[0] = {0.0, 0.0, 0.0};
    config.guidance[1] = {10.0, 0.0, 0.0};
    auto context = control_context(config);
    context.navigation.last_gyro[1] = 1.0;
    auto input = input_at(5.0);
    input.air_data.valid = 0;
    FswOutput output{};
    fsw::internal::calculate_controls(context, input, output);
    REQUIRE(output.tvc_pitch_rad < 0.0);

    config.control_kp = 100.0;
    config.guidance[0].pitch_rad = 1.0;
    config.guidance[1].pitch_rad = 1.0;
    context = control_context(config);
    output = {};
    fsw::internal::calculate_controls(context, input, output);
    REQUIRE(output.tvc_pitch_rad == config.max_tvc_rad);
}

void test_invalid_navigation_is_zero_safe() {
    const auto config = default_config();
    auto context = control_context(config);
    context.navigation.attitude_valid = false;
    auto input = input_at(5.0);
    FswOutput output{};
    fsw::internal::calculate_controls(context, input, output);
    REQUIRE(output.tvc_pitch_rad == 0.0);
    REQUIRE(output.tvc_yaw_rad == 0.0);
    REQUIRE(output.fin_roll_rad == 0.0);
    REQUIRE(output.fin_pitch_rad == 0.0);
    REQUIRE(output.fin_yaw_rad == 0.0);
}

void test_orbit_control_tracks_inertial_prograde() {
    auto config = default_config();
    config.orbit_enabled = 1;
    auto context = control_context(config);
    context.mission.mode = FSW_MODE_COAST;
    context.navigation.position_ecef = {kEarthRadiusM, 0.0, 0.0};
    context.navigation.velocity_ecef = {0.0, 1000.0, 0.0};
    auto input = input_at(5.0);
    input.air_data.valid = 0;
    FswOutput output{};
    fsw::internal::calculate_controls(context, input, output);
    REQUIRE(output.tvc_yaw_rad > 0.0);

    context.navigation.velocity_ecef[1] = -1000.0;
    output = {};
    fsw::internal::calculate_controls(context, input, output);
    REQUIRE(output.tvc_yaw_rad < 0.0);
}

int main() {
    test_guidance_and_attitude_error_response();
    test_rate_damping_and_saturation();
    test_invalid_navigation_is_zero_safe();
    test_orbit_control_tracks_inertial_prograde();
    return EXIT_SUCCESS;
}
