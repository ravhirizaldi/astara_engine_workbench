#include "test_support.hpp"

using namespace fsw_test;

void test_delayed_launch_event_timing_and_one_shot_separation() {
    const FswConfig config = default_config();
    FswHandle handle = fsw_create(&config);
    REQUIRE(handle != nullptr);
    FswOutput output{};
    for (int tick = 0; tick < 500; ++tick) {
        auto input = input_at(tick * 0.01);
        step(handle, input, output);
    }
    arm_and_launch(handle, output, 500);

    int separation_pulses = 0;
    uint64_t separation_sequence = 0;
    int tick = 502;
    for (; tick < 580 && output.mode != FSW_MODE_INTERSTAGE; ++tick) {
        auto input = input_at(
            tick * 0.01,
            (
                output.mode == FSW_MODE_IGNITION
                || (
                    output.mode == FSW_MODE_BOOST_1
                    && tick < 528
                )
            ) ? 20.0 : 0.0
        );
        input.propulsion.running =
            output.mode == FSW_MODE_IGNITION
            || (
                output.mode == FSW_MODE_BOOST_1
                && tick < 528
            );
        if (separation_pulses > 0) {
            input.discretes.stage_separated.asserted = 1;
        }
        step(handle, input, output);
        if (output.stage_separate) {
            ++separation_pulses;
            separation_sequence =
                output.discrete_actuation.sequence;
            REQUIRE(
                output.discrete_actuation.action
                == FSW_DISCRETE_ACTION_STAGE_SEPARATE
            );
        }
    }
    REQUIRE(output.mode == FSW_MODE_INTERSTAGE);
    REQUIRE(separation_pulses == 1);
    REQUIRE(separation_sequence > 0);

    int stage2_pulses = 0;
    for (; tick < 650 && output.mode != FSW_MODE_BOOST_2; ++tick) {
        auto input = input_at(tick * 0.01, 20.0);
        input.discretes.stage_separated.asserted = 1;
        input.propulsion.running = stage2_pulses > 0;
        step(handle, input, output);
        stage2_pulses += output.stage2_ignite ? 1 : 0;
    }
    REQUIRE(output.mode == FSW_MODE_BOOST_2);
    REQUIRE(stage2_pulses == 1);
    fsw_destroy(handle);
}

int main() {
    test_delayed_launch_event_timing_and_one_shot_separation();
    return EXIT_SUCCESS;
}
