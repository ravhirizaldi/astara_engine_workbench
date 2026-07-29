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
    FswOutput output{};
    auto input = input_at(0.0);
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

int main() {
    REQUIRE(fsw_abi_version() == FSW_ABI_VERSION);
    REQUIRE(std::strcmp(fsw_version(), FSW_VERSION_STRING) == 0);
    test_abi_contract_and_release_active_requirements();
    return EXIT_SUCCESS;
}
