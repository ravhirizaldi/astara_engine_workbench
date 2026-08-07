#include "test_support.hpp"

#include "core/context.hpp"
#include "mission/commands.hpp"
#include "mission/mission.hpp"
#include "sensors/sensor_voting.hpp"

#include <cmath>

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
    int ignition_confirmed_tick = -1;
    int separation_pulse_tick = -1;
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
        if (
            ignition_confirmed_tick < 0
            && output.mode == FSW_MODE_BOOST_1
        ) {
            ignition_confirmed_tick = tick;
        }
        if (output.stage_separate) {
            ++separation_pulses;
            separation_pulse_tick = tick;
            separation_sequence =
                output.discrete_actuation.sequence;
            REQUIRE(
                output.discrete_actuation.action
                == FSW_DISCRETE_ACTION_STAGE_SEPARATE
            );
        }
    }
    REQUIRE(output.mode == FSW_MODE_INTERSTAGE);
    REQUIRE(ignition_confirmed_tick >= 506);
    REQUIRE(separation_pulses == 1);
    REQUIRE(separation_sequence > 0);
    REQUIRE(
        separation_pulse_tick - ignition_confirmed_tick
        >= 35
    );

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

void test_standalone_upper_accepts_separation_feedback() {
    auto config = default_config();
    config.body_role = FSW_BODY_UPPER;
    FswHandle handle = fsw_create(&config);
    REQUIRE(handle != nullptr);

    FswOutput output{};
    int stage2_pulses = 0;
    for (int tick = 0; tick < 40 && output.mode != FSW_MODE_BOOST_2; ++tick) {
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

void test_drogue_main_and_landing() {
    auto config = default_config();
    fsw::internal::Context context(config);
    fsw::internal::VotedSensors voted{};
    context.mission.mode = FSW_MODE_COAST;
    context.mission.apogee_seen = true;
    context.navigation.navigation_status = FSW_NAV_DEGRADED;
    context.navigation.altitude = 500.0;
    context.navigation.vertical_velocity = -10.0;
    context.timing.step_delta_s = 0.01;

    auto input = input_at(0.0);
    fsw::internal::update_mode(context, input, voted);
    REQUIRE(context.mission.mode == FSW_MODE_DROGUE);
    REQUIRE(
        context.mission.discrete_actuation.action
        == FSW_DISCRETE_ACTION_DEPLOY_DROGUE
    );

    context.mission.discrete_actuation = {};
    context.navigation.altitude = 250.0;
    input = input_at(0.01);
    input.discretes.drogue_deployed.asserted = 1;
    fsw::internal::update_mode(context, input, voted);
    REQUIRE(context.mission.drogue_deployed);
    REQUIRE(context.mission.mode == FSW_MODE_MAIN);
    REQUIRE(
        context.mission.discrete_actuation.action
        == FSW_DISCRETE_ACTION_DEPLOY_MAIN
    );

    context.mission.discrete_actuation = {};
    input = input_at(0.02);
    input.discretes.main_deployed.asserted = 1;
    fsw::internal::update_mode(context, input, voted);
    REQUIRE(context.mission.drogue_deployed);
    REQUIRE(context.mission.main_deployed);

    context.navigation.altitude = 1.0;
    context.navigation.vertical_velocity = -1.0;
    context.timing.step_delta_s = 0.25;
    for (int tick = 1; tick <= 4; ++tick) {
        input = input_at(0.02 + tick * 0.25);
        input.sensors.dt_s = 0.25;
        fsw::internal::update_mode(context, input, voted);
    }
    REQUIRE(context.mission.drogue_deployed);
    REQUIRE(context.mission.main_deployed);
    REQUIRE(context.mission.mode == FSW_MODE_LANDED);
}

void test_manual_and_automatic_abort() {
    const auto config = default_config();
    FswHandle handle = fsw_create(&config);
    REQUIRE(handle != nullptr);
    FswOutput output{};
    auto input = input_at(0.0);
    step(handle, input, output);
    input = input_at(0.01);
    set_command(input, 1, FSW_COMMAND_ABORT);
    step(handle, input, output);
    REQUIRE(output.mode == FSW_MODE_ABORT);
    REQUIRE(output.command_result == FSW_COMMAND_ACCEPTED);
    fsw_destroy(handle);

    fsw::internal::Context context(config);
    fsw::internal::VotedSensors voted{};
    voted.accelerometer_valid = true;
    voted.gyroscope_valid = true;
    voted.imu_valid = true;
    context.mission.mode = FSW_MODE_BOOST_1;
    context.navigation.attitude_valid = false;
    context.timing.step_delta_s = 0.01;
    for (int tick = 0; tick < 5; ++tick) {
        input = input_at(tick * 0.01);
        fsw::internal::update_mode(context, input, voted);
    }
    REQUIRE(context.mission.mode == FSW_MODE_ABORT);
}

void test_accelerometer_loss_cannot_trigger_burnout() {
    const auto config = default_config();
    fsw::internal::Context context(config);
    fsw::internal::VotedSensors voted{};
    voted.accelerometer_valid = false;
    voted.gyroscope_valid = true;
    voted.imu_valid = false;
    context.mission.mode = FSW_MODE_BOOST_1;
    context.mission.ignition_confirmed_s = 0.0;
    context.navigation.attitude_valid = true;
    context.timing.step_delta_s = 0.01;

    FswInput input{};
    for (int tick = 0; tick < 4; ++tick) {
        input = input_at(1.0 + tick * 0.01);
        input.propulsion.running = 1;
        fsw::internal::update_mode(context, input, voted);
        REQUIRE(context.mission.mode == FSW_MODE_BOOST_1);
        REQUIRE(context.mission.burnout_detected_s < 0.0);
    }

    input = input_at(1.04);
    input.propulsion.running = 1;
    fsw::internal::update_mode(context, input, voted);
    REQUIRE(context.mission.mode == FSW_MODE_ABORT);
    REQUIRE(context.mission.burnout_detected_s < 0.0);
}

void test_orbit_cutoff_uses_configured_speed_margin() {
    auto config = default_config();
    config.orbit_enabled = 1;
    config.orbit_cutoff_speed_margin_m_s = 10.0;
    fsw::internal::Context context(config);
    fsw::internal::VotedSensors voted{};
    voted.accelerometer_valid = true;
    voted.gyroscope_valid = true;
    voted.imu_valid = true;
    context.mission.mode = FSW_MODE_ORBIT_INSERTION;
    context.mission.circularization_ignition_s = 0.0;
    context.navigation.attitude_valid = true;
    context.navigation.position_ecef = {6571000.0, 0.0, 0.0};
    context.timing.step_delta_s = 0.01;

    constexpr double earth_mu_m3_s2 = 3.986004418e14;
    constexpr double earth_rotation_rad_s = 7.2921150e-5;
    const double radius_m = context.navigation.position_ecef[0];
    const double circular_speed_m_s = std::sqrt(
        earth_mu_m3_s2 / radius_m
    );
    context.navigation.velocity_ecef = {
        0.0,
        circular_speed_m_s
            - earth_rotation_rad_s * radius_m
            + config.orbit_cutoff_speed_margin_m_s
            - 0.1,
        0.0,
    };
    auto input = input_at(10.0);
    input.propulsion.running = 1;
    fsw::internal::update_mode(context, input, voted);
    REQUIRE(context.mission.mode == FSW_MODE_ORBIT_INSERTION);

    context.navigation.velocity_ecef[1] += 0.2;
    input = input_at(10.01);
    input.propulsion.running = 1;
    fsw::internal::update_mode(context, input, voted);
    REQUIRE(context.mission.mode == FSW_MODE_ORBIT);
    REQUIRE(context.control.stage2_shutdown_request);
}

void test_orbit_coast_requires_two_sided_altitude_and_radial_bands() {
    auto config = default_config();
    config.orbit_enabled = 1;
    config.body_role = FSW_BODY_UPPER;
    fsw::internal::Context context(config);
    fsw::internal::VotedSensors voted{};
    voted.accelerometer_valid = true;
    voted.gyroscope_valid = true;
    voted.imu_valid = true;
    context.mission.mode = FSW_MODE_COAST;
    context.navigation.attitude_valid = true;
    context.navigation.altitude = config.orbit_target_altitude_m
        + config.orbit_altitude_tolerance_m + 1.0;
    context.navigation.position_ecef = {
        kEarthRadiusM + context.navigation.altitude, 0.0, 0.0
    };
    context.navigation.velocity_ecef = {0.0, 0.0, 0.0};
    auto input = input_at(10.0);
    fsw::internal::update_mode(context, input, voted);
    REQUIRE(context.mission.mode == FSW_MODE_COAST);

    context.navigation.altitude = config.orbit_target_altitude_m;
    context.navigation.position_ecef[0] =
        kEarthRadiusM + context.navigation.altitude;
    context.navigation.velocity_ecef[0] =
        config.orbit_radial_velocity_tolerance_m_s + 1.0;
    input = input_at(10.01);
    fsw::internal::update_mode(context, input, voted);
    REQUIRE(context.mission.mode == FSW_MODE_COAST);

    context.navigation.velocity_ecef[0] = 0.0;
    input = input_at(10.02);
    fsw::internal::update_mode(context, input, voted);
    REQUIRE(context.mission.mode == FSW_MODE_ORBIT_INSERTION);
}

void test_interstage_clear_removes_only_recovered_faults() {
    const auto config = default_config();
    fsw::internal::Context context(config);
    fsw::internal::VotedSensors voted{};
    context.mission.mode = FSW_MODE_INTERSTAGE;
    context.faults.active_fault_flags = FSW_FAULT_GNSS_UNAVAILABLE;
    context.faults.latched_fault_flags =
        FSW_FAULT_GNSS_UNAVAILABLE | FSW_FAULT_IMU_UNAVAILABLE;
    auto input = input_at(1.0);
    set_command(input, 1, FSW_COMMAND_CLEAR_FAULTS);

    fsw::internal::process_command(context, input, voted);

    REQUIRE(context.control.command_result == FSW_COMMAND_ACCEPTED);
    REQUIRE(
        context.faults.latched_fault_flags == FSW_FAULT_GNSS_UNAVAILABLE
    );
}

int main() {
    test_delayed_launch_event_timing_and_one_shot_separation();
    test_standalone_upper_accepts_separation_feedback();
    test_drogue_main_and_landing();
    test_manual_and_automatic_abort();
    test_accelerometer_loss_cannot_trigger_burnout();
    test_orbit_cutoff_uses_configured_speed_margin();
    test_orbit_coast_requires_two_sided_altitude_and_radial_bands();
    test_interstage_clear_removes_only_recovered_faults();
    return EXIT_SUCCESS;
}
