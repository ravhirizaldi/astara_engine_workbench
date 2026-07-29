#include "test_support.hpp"

#include "core/context.hpp"
#include "sensors/sensor_voting.hpp"

using namespace fsw_test;

void test_one_two_and_three_channel_voting() {
    const FswConfig config = default_config();

    fsw::internal::Context one_context(config);
    auto suite = input_at(0.0).sensors;
    auto voted = fsw::internal::vote_sensors(one_context, suite);
    REQUIRE(voted.accelerometer_valid);
    REQUIRE(voted.gyroscope_valid);
    REQUIRE(voted.accelerometer_usable_mask == 0b001u);
    REQUIRE(voted.gyroscope_usable_mask == 0b001u);
    REQUIRE(
        std::abs(
            voted.acceleration[0] - kRestSpecificForceM_S2
        ) < 1e-12
    );

    fsw::internal::Context two_context(config);
    suite.imu_count = 2;
    suite.imus[1] = suite.imus[0];
    suite.imus[0].acceleration_body_m_s2[0] = 9.6;
    suite.imus[1].acceleration_body_m_s2[0] = 9.8;
    suite.imus[0].gyro_body_rad_s[2] = 0.002;
    suite.imus[1].gyro_body_rad_s[2] = 0.004;
    voted = fsw::internal::vote_sensors(two_context, suite);
    REQUIRE(voted.accelerometer_usable_mask == 0b011u);
    REQUIRE(voted.gyroscope_usable_mask == 0b011u);
    REQUIRE(std::abs(voted.acceleration[0] - 9.7) < 1e-12);
    REQUIRE(std::abs(voted.gyro[2] - 0.003) < 1e-12);

    fsw::internal::Context three_context(config);
    suite.imu_count = 3;
    suite.imus[2] = suite.imus[1];
    suite.imus[2].acceleration_body_m_s2[0] = 20.0;
    suite.imus[2].gyro_body_rad_s[2] = 0.006;
    voted = fsw::internal::vote_sensors(three_context, suite);
    REQUIRE(voted.accelerometer_usable_mask == 0b011u);
    REQUIRE(voted.gyroscope_usable_mask == 0b111u);
    REQUIRE(std::abs(voted.acceleration[0] - 9.7) < 1e-12);
    REQUIRE(std::abs(voted.gyro[2] - 0.004) < 1e-12);
}

void test_disagreement_rejection_and_health_independence() {
    const FswConfig config = default_config();
    fsw::internal::Context context(config);
    auto suite = input_at(0.0).sensors;
    suite.imu_count = 2;
    suite.imus[1] = suite.imus[0];
    suite.imus[1].acceleration_body_m_s2[0] = 20.0;

    const auto voted = fsw::internal::vote_sensors(context, suite);
    REQUIRE(!voted.accelerometer_valid);
    REQUIRE(voted.gyroscope_valid);
    REQUIRE(voted.accelerometer_usable_mask == 0);
    REQUIRE(voted.gyroscope_usable_mask == 0b011u);
    REQUIRE(
        voted.disagreement_flags & FSW_DISAGREEMENT_ACCELERATION
    );
    REQUIRE((voted.disagreement_flags & FSW_DISAGREEMENT_GYRO) == 0);

    fsw::internal::Context invalid_accel_context(config);
    suite = input_at(0.0).sensors;
    suite.imus[0].accel_valid = 0;
    const auto gyro_only = fsw::internal::vote_sensors(
        invalid_accel_context, suite
    );
    REQUIRE(!gyro_only.accelerometer_valid);
    REQUIRE(gyro_only.gyroscope_valid);
    REQUIRE(
        invalid_accel_context.sensors.accelerometer_health[0].flags
        & FSW_SENSOR_HEALTH_INVALID
    );
    REQUIRE(
        invalid_accel_context.sensors.gyroscope_health[0].flags == 0
    );

    fsw::internal::Context invalid_gyro_context(config);
    suite = input_at(0.0).sensors;
    suite.imus[0].gyro_valid = 0;
    const auto accel_only = fsw::internal::vote_sensors(
        invalid_gyro_context, suite
    );
    REQUIRE(accel_only.accelerometer_valid);
    REQUIRE(!accel_only.gyroscope_valid);
    REQUIRE(
        invalid_gyro_context.sensors.accelerometer_health[0].flags == 0
    );
    REQUIRE(
        invalid_gyro_context.sensors.gyroscope_health[0].flags
        & FSW_SENSOR_HEALTH_INVALID
    );
}

void test_channel_recovery_staleness_and_asynchronous_samples() {
    FswConfig config = default_config();
    config.voter_reject_samples = 2;
    config.voter_recover_samples = 3;
    fsw::internal::Context context(config);
    fsw::internal::VotedSensors voted{};

    for (int tick = 0; tick < 2; ++tick) {
        auto input = input_at(tick * 0.01);
        fill_three_channels(input);
        input.sensors.imus[2].acceleration_body_m_s2[0] = 20.0;
        voted = fsw::internal::vote_sensors(context, input.sensors);
    }
    REQUIRE(context.sensors.accelerometer_rejected_mask & 0b100u);
    REQUIRE(context.sensors.gyroscope_rejected_mask == 0);

    for (int tick = 2; tick < 5; ++tick) {
        auto input = input_at(tick * 0.01);
        fill_three_channels(input);
        voted = fsw::internal::vote_sensors(context, input.sensors);
    }
    REQUIRE((context.sensors.accelerometer_rejected_mask & 0b100u) == 0);

    fsw::internal::Context asynchronous_context(config);
    auto suite = input_at(0.20).sensors;
    suite.imu_count = 2;
    suite.imus[1] = suite.imus[0];
    suite.imus[0].sample_time_s = 0.11;
    suite.imus[1].sample_time_s = 0.19;
    voted = fsw::internal::vote_sensors(asynchronous_context, suite);
    REQUIRE(voted.accelerometer_usable_mask == 0b011u);
    REQUIRE(voted.gyroscope_usable_mask == 0b011u);
    REQUIRE(std::abs(voted.accelerometer_sample_time_s - 0.19) < 1e-12);
    REQUIRE(std::abs(voted.gyroscope_sample_time_s - 0.19) < 1e-12);
    REQUIRE(
        std::abs(
            asynchronous_context.sensors.accelerometer_health[0].age_s
            - 0.09
        ) < 1e-12
    );

    fsw::internal::Context stale_context(config);
    suite = input_at(0.20).sensors;
    suite.imus[0].sample_time_s = 0.0;
    voted = fsw::internal::vote_sensors(stale_context, suite);
    REQUIRE(!voted.accelerometer_valid);
    REQUIRE(!voted.gyroscope_valid);
    REQUIRE(
        stale_context.sensors.accelerometer_health[0].flags
        & FSW_SENSOR_HEALTH_STALE
    );
    REQUIRE(
        stale_context.sensors.gyroscope_health[0].flags
        & FSW_SENSOR_HEALTH_STALE
    );
}

void test_public_split_and_other_sensor_recovery() {
    FswConfig config = default_config();
    config.max_barometer_rate_m_s = 100.0;
    config.max_gnss_velocity_rate_m_s2 = 100.0;
    FswHandle handle = fsw_create(&config);
    REQUIRE(handle != nullptr);
    FswOutput output{};

    auto input = input_at(0.0);
    input.sensors.imus[0].accel_valid = 0;
    input.sensors.magnetometers[0].valid = 0;
    step(handle, input, output);
    REQUIRE(output.accelerometer_usable_mask == 0);
    REQUIRE(output.gyroscope_usable_mask == 1);
    REQUIRE(output.magnetometer_usable_mask == 0);
    REQUIRE(
        output.sensor_status_flags
        & FSW_SENSOR_STATUS_GYROSCOPE_SINGLE_SOURCE
    );

    input = input_at(0.01);
    input.sensors.barometers[0].altitude_m = 1000.0;
    input.sensors.gnss[0].gnss_velocity_ecef_m_s[0] = 100.0;
    step(handle, input, output);
    REQUIRE(output.barometer_usable_mask == 0);
    REQUIRE(output.gnss_usable_mask == 0);

    input = input_at(0.02);
    input.sensors.barometers[0].altitude_m = 0.5;
    input.sensors.gnss[0].gnss_velocity_ecef_m_s[0] = 0.5;
    step(handle, input, output);
    REQUIRE(output.barometer_usable_mask == 1);
    REQUIRE(output.gnss_usable_mask == 1);
    fsw_destroy(handle);
}

int main() {
    test_one_two_and_three_channel_voting();
    test_disagreement_rejection_and_health_independence();
    test_channel_recovery_staleness_and_asynchronous_samples();
    test_public_split_and_other_sensor_recovery();
    return EXIT_SUCCESS;
}
