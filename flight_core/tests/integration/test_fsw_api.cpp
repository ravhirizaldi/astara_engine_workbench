#include "test_support.hpp"

#include <cstring>

using namespace fsw_test;

void test_abi_contract_and_release_active_requirements() {
    REQUIRE(
        fsw_step(nullptr, nullptr, nullptr)
        == FSW_STATUS_INVALID_ARGUMENT
    );
    FswOutput invalid_output{};
    invalid_output.stage1_ignite = 1;
    REQUIRE(
        fsw_step(nullptr, nullptr, &invalid_output)
        == FSW_STATUS_INVALID_ARGUMENT
    );
    REQUIRE(invalid_output.abi_version == FSW_ABI_VERSION);
    REQUIRE(invalid_output.struct_size == sizeof(FswOutput));
    REQUIRE(!invalid_output.output_valid);
    REQUIRE(!invalid_output.stage1_ignite);

    FswConfig bad_config = default_config();
    bad_config.abi_version = 0;
    REQUIRE(fsw_create(&bad_config) == nullptr);
    const FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    REQUIRE(handle != nullptr);
    FswOutput output{};
    output.stage1_ignite = 1;
    REQUIRE(
        fsw_step(handle, nullptr, &output)
        == FSW_STATUS_INVALID_ARGUMENT
    );
    REQUIRE(!output.output_valid);
    REQUIRE(!output.stage1_ignite);
    auto input = input_at(0.0);
    REQUIRE(
        fsw_step(handle, &input, nullptr)
        == FSW_STATUS_INVALID_ARGUMENT
    );
    input.abi_version = 0;
    REQUIRE(
        fsw_step(handle, &input, &output)
        == FSW_STATUS_ABI_MISMATCH
    );
    REQUIRE(!output.output_valid);
    REQUIRE(!output.stage1_ignite);
    REQUIRE(!output.discrete_actuation.valid);
    fsw_destroy(handle);
}

void test_reset_matches_fresh_context() {
    const FswConfig config = default_config();
    FswHandle reset_handle = fsw_create(&config);
    FswHandle fresh_handle = fsw_create(&config);
    REQUIRE(reset_handle != nullptr);
    REQUIRE(fresh_handle != nullptr);

    FswOutput output{};
    arm_and_launch(reset_handle, output, 0);
    auto input = input_at(0.02, 30.0);
    step(reset_handle, input, output);
    fsw_reset(reset_handle);

    for (int tick = 0; tick < 3; ++tick) {
        auto reset_input = input_at(tick * 0.01);
        auto fresh_input = reset_input;
        FswOutput reset_output{};
        FswOutput fresh_output{};
        step(reset_handle, reset_input, reset_output);
        step(fresh_handle, fresh_input, fresh_output);
        REQUIRE(
            std::memcmp(&reset_output, &fresh_output, sizeof(FswOutput)) == 0
        );
    }

    fsw_destroy(reset_handle);
    fsw_destroy(fresh_handle);
}

int main() {
    REQUIRE(fsw_abi_version() == FSW_ABI_VERSION);
    REQUIRE(std::strcmp(fsw_version(), FSW_VERSION_STRING) == 0);
    test_abi_contract_and_release_active_requirements();
    test_reset_matches_fresh_context();
    return EXIT_SUCCESS;
}
