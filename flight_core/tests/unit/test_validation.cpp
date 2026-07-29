#include "test_support.hpp"

#include "validation/config_validation.hpp"
#include "validation/input_validation.hpp"

using namespace fsw_test;

int main() {
    auto config = default_config();
    REQUIRE(fsw::internal::valid_config(config));
    config.guidance_count = 1;
    REQUIRE(!fsw::internal::valid_config(config));

    config = default_config();
    auto input = input_at(0.0);
    REQUIRE(fsw::internal::valid_input(input, config));
    input.sensors.dt_s = 0.0;
    REQUIRE(!fsw::internal::valid_input(input, config));
    return EXIT_SUCCESS;
}
