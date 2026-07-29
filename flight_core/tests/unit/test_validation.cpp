#include "test_support.hpp"

#include <limits>

#include "validation/config_validation.hpp"
#include "validation/input_validation.hpp"

using namespace fsw_test;

void test_abi_and_struct_sizes() {
    auto config = default_config();
    REQUIRE(fsw::internal::valid_config(config));
    config.abi_version = FSW_ABI_VERSION - 1;
    REQUIRE(!fsw::internal::valid_config(config));
    config = default_config();
    config.struct_size = sizeof(FswConfig) - 1;
    REQUIRE(!fsw::internal::valid_config(config));

    config = default_config();
    auto input = input_at(0.0);
    REQUIRE(fsw::internal::valid_input(input, config));
    input.abi_version = FSW_ABI_VERSION - 1;
    REQUIRE(!fsw::internal::valid_input(input, config));
    input = input_at(0.0);
    input.struct_size = sizeof(FswInput) - 1;
    REQUIRE(!fsw::internal::valid_input(input, config));
}

void test_non_finite_values_are_rejected() {
    const auto config = default_config();
    auto input = input_at(0.0);
    input.sensors.time_s = std::numeric_limits<double>::quiet_NaN();
    REQUIRE(!fsw::internal::valid_input(input, config));

    input = input_at(0.0);
    input.sensors.imus[0].acceleration_body_m_s2[1] =
        std::numeric_limits<double>::infinity();
    REQUIRE(!fsw::internal::valid_input(input, config));

    input = input_at(0.0);
    input.sensors.imus[0].gyro_body_rad_s[2] =
        -std::numeric_limits<double>::infinity();
    REQUIRE(!fsw::internal::valid_input(input, config));

    input = input_at(0.0);
    input.propulsion.health_percent =
        std::numeric_limits<double>::quiet_NaN();
    REQUIRE(!fsw::internal::valid_input(input, config));
}

void test_timestamp_and_actual_timestep_validation() {
    const auto config = default_config();
    FswHandle handle = fsw_create(&config);
    REQUIRE(handle != nullptr);
    FswOutput output{};

    auto input = input_at(0.0);
    step(handle, input, output);

    input = input_at(0.0);
    REQUIRE(
        fsw_step(handle, &input, &output) == FSW_STATUS_INVALID_INPUT
    );
    REQUIRE(!output.output_valid);

    input = input_at(0.0005);
    input.sensors.dt_s = config.min_step_s;
    REQUIRE(
        fsw_step(handle, &input, &output) == FSW_STATUS_INVALID_INPUT
    );

    input = input_at(0.20);
    input.sensors.dt_s = config.max_step_s;
    REQUIRE(
        fsw_step(handle, &input, &output) == FSW_STATUS_INVALID_INPUT
    );

    input = input_at(0.01);
    input.sensors.dt_s = 0.02;
    step(handle, input, output);
    REQUIRE(output.active_fault_flags & FSW_FAULT_INPUT_TIMING);

    input = input_at(0.02);
    step(handle, input, output);
    REQUIRE(output.output_valid);
    fsw_destroy(handle);
}

int main() {
    test_abi_and_struct_sizes();
    test_non_finite_values_are_rejected();
    test_timestamp_and_actual_timestep_validation();
    return EXIT_SUCCESS;
}
