#include "test_support.hpp"

#include <cmath>

#include "control/control.hpp"
#include "core/context.hpp"
#include "guidance/guidance.hpp"

using namespace fsw_test;

int main() {
    auto config = default_config();
    config.guidance[1] = {10.0, 0.2, 0.4};
    const auto midpoint = fsw::internal::guidance_at(config, 5.0);
    REQUIRE(std::abs(midpoint.pitch_rad - 0.1) < 1e-12);
    REQUIRE(std::abs(midpoint.azimuth_rad - 0.2) < 1e-12);

    fsw::internal::Context context(config);
    context.mission.mode = FSW_MODE_BOOST_1;
    context.navigation.attitude_valid = true;
    context.mission.ignition_confirmed_s = 0.0;
    auto input = input_at(5.0);
    FswOutput output{};
    fsw::internal::calculate_controls(context, input, output);
    REQUIRE(std::abs(output.tvc_pitch_rad) <= config.max_tvc_rad);
    REQUIRE(std::abs(output.tvc_yaw_rad) <= config.max_tvc_rad);
    REQUIRE(std::abs(output.fin_pitch_rad) <= config.max_fin_rad);
    return EXIT_SUCCESS;
}
