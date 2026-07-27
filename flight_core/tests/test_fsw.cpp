#include "fsw.h"

#include <cassert>
#include <cmath>
#include <cstring>

namespace {

constexpr double kEarthRadiusM = 6'371'000.0;

FswConfig default_config() {
    FswConfig config{};
    config.stage1_burn_s = 1.0;
    config.separation_delay_s = 0.2;
    config.stage2_ignition_delay_s = 0.2;
    config.stage2_burn_s = 1.0;
    config.main_deploy_altitude_m = 100.0;
    config.max_tvc_rad = 0.1;
    config.max_fin_rad = 0.2;
    config.control_kp = 0.7;
    config.control_kd = 0.2;
    config.imu_timeout_s = 0.02;
    config.barometer_timeout_s = 0.06;
    config.gnss_timeout_s = 0.30;
    config.acceleration_disagreement_m_s2 = 0.5;
    config.gyro_disagreement_rad_s = 0.01;
    config.barometer_disagreement_m = 10.0;
    config.gnss_position_disagreement_m = 15.0;
    config.gnss_velocity_disagreement_m_s = 1.0;
    config.cross_altitude_disagreement_m = 20.0;
    config.voter_reject_samples = 3;
    config.voter_recover_samples = 5;
    config.imu_loss_abort_delay_s = 0.05;
    config.gyro_bias_time_constant_s = 2.0;
    config.stationary_gyro_threshold_rad_s = 0.02;
    config.altitude_filter_tau_s = 0.20;
    config.velocity_filter_tau_s = 0.60;
    config.guidance_count = 3;
    config.guidance[0] = {0.0, 0.0, 1.5};
    config.guidance[1] = {0.5, 0.1, 1.5};
    config.guidance[2] = {1.5, 0.3, 1.7};
    config.body_role = FSW_BODY_INTEGRATED;
    return config;
}

FswSensorSuite sensor_at(double time_s) {
    FswSensorSuite sensor{};
    sensor.time_s = time_s;
    sensor.dt_s = 0.01;
    sensor.imu_count = 1;
    sensor.barometer_count = 1;
    sensor.gnss_count = 1;
    sensor.imus[0].acceleration_body_m_s2[0] = 10.0;
    sensor.imus[0].magnetic_body[0] = 1.0;
    sensor.imus[0].sample_time_s = time_s;
    sensor.imus[0].valid = 1;
    sensor.barometers[0].altitude_m = time_s * 10.0;
    sensor.barometers[0].sample_time_s = time_s;
    sensor.barometers[0].valid = 1;
    sensor.gnss[0].gnss_position_ecef_m[0] =
        kEarthRadiusM + time_s * 10.0;
    sensor.gnss[0].gnss_velocity_ecef_m_s[0] = 10.0;
    sensor.gnss[0].vertical_velocity_m_s = 10.0;
    sensor.gnss[0].sample_time_s = time_s;
    sensor.gnss[0].valid = 1;
    sensor.dynamic_pressure_pa = 1000.0;
    sensor.engine_health_percent = 100.0;
    return sensor;
}

void fill_three_channels(FswSensorSuite& sensor) {
    sensor.imu_count = 3;
    sensor.barometer_count = 3;
    sensor.gnss_count = 3;
    for (uint32_t index = 1; index < 3; ++index) {
        sensor.imus[index] = sensor.imus[0];
        sensor.barometers[index] = sensor.barometers[0];
        sensor.gnss[index] = sensor.gnss[0];
    }
}

void test_single_channel_compatibility() {
    auto invalid_config = default_config();
    invalid_config.guidance_count = 1;
    assert(fsw_create(&invalid_config) == nullptr);

    auto config = default_config();
    FswHandle handle = fsw_create(&config);
    assert(handle != nullptr);
    FswOutput output{};
    for (int step = 0; step <= 75; ++step) {
        auto sensor = sensor_at(step * 0.01);
        assert(fsw_step(handle, &sensor, &output) == FSW_STATUS_OK);
    }
    assert(output.mode == FSW_MODE_BOOST_1);
    assert(output.navigation_status == FSW_NAV_NOMINAL);
    assert(output.attitude_valid == 1);
    assert(output.imu_usable_mask == 1);
    assert(output.barometer_usable_mask == 1);
    assert(output.gnss_usable_mask == 1);
    assert(
        output.sensor_status_flags
        & FSW_SENSOR_STATUS_IMU_SINGLE_SOURCE
    );
    assert(output.tvc_pitch_rad > 0.0);
    assert(std::abs(output.tvc_pitch_rad) <= config.max_tvc_rad);
    assert(std::abs(output.tvc_yaw_rad) <= config.max_tvc_rad);

    auto duplicate = sensor_at(0.75);
    assert(
        fsw_step(handle, &duplicate, &output)
        == FSW_STATUS_INVALID_SAMPLE
    );

    fsw_reset(handle);
    auto first = sensor_at(0.0);
    assert(fsw_step(handle, &first, &output) == FSW_STATUS_OK);
    auto stale = sensor_at(0.4);
    stale.dt_s = 0.4;
    stale.barometers[0].sample_time_s = 0.0;
    stale.gnss[0].sample_time_s = 0.0;
    assert(fsw_step(handle, &stale, &output) == FSW_STATUS_OK);
    assert(output.navigation_status == FSW_NAV_INERTIAL);
    assert(output.fault_flags & FSW_FAULT_GNSS_UNAVAILABLE);
    assert(output.fault_flags & FSW_FAULT_BAROMETER_UNAVAILABLE);
    assert(output.fault_flags & FSW_FAULT_NAV_INERTIAL);

    auto recovered = sensor_at(0.41);
    assert(fsw_step(handle, &recovered, &output) == FSW_STATUS_OK);
    assert(output.navigation_status == FSW_NAV_NOMINAL);

    auto abort_sensor = sensor_at(0.42);
    abort_sensor.engine_health_percent = 0.0;
    assert(fsw_step(handle, &abort_sensor, &output) == FSW_STATUS_OK);
    assert(output.mode == FSW_MODE_ABORT);
    assert(output.abort == 1);
    assert(output.tvc_pitch_rad == 0.0);
    fsw_destroy(handle);
}

void test_imu_rejection_recovery_and_new_sample_debounce() {
    auto config = default_config();
    config.imu_timeout_s = 1.0;
    FswHandle handle = fsw_create(&config);
    FswOutput output{};

    auto sensor = sensor_at(0.0);
    fill_three_channels(sensor);
    sensor.imus[2].acceleration_body_m_s2[0] = 30.0;
    sensor.imus[2].gyro_body_rad_s[1] = 1.0;
    assert(fsw_step(handle, &sensor, &output) == FSW_STATUS_OK);
    assert(output.imu_usable_mask == 0b011);
    assert(output.imu_rejected_mask == 0);
    assert(output.fault_flags & FSW_FAULT_IMU_DISAGREEMENT);

    for (int step = 1; step <= 5; ++step) {
        sensor.time_s = step * 0.001;
        sensor.dt_s = 0.001;
        assert(fsw_step(handle, &sensor, &output) == FSW_STATUS_OK);
    }
    assert(output.imu_rejected_mask == 0);

    for (int step = 1; step <= 2; ++step) {
        sensor.time_s = 0.01 + step * 0.01;
        sensor.dt_s = 0.01;
        for (uint32_t index = 0; index < 3; ++index) {
            sensor.imus[index].sample_time_s = sensor.time_s;
        }
        assert(fsw_step(handle, &sensor, &output) == FSW_STATUS_OK);
    }
    assert(output.imu_rejected_mask == 0b100);

    for (int step = 1; step <= 5; ++step) {
        sensor.time_s = 0.03 + step * 0.01;
        for (uint32_t index = 0; index < 3; ++index) {
            sensor.imus[index].sample_time_s = sensor.time_s;
        }
        sensor.imus[2] = sensor.imus[0];
        assert(fsw_step(handle, &sensor, &output) == FSW_STATUS_OK);
    }
    assert(output.imu_rejected_mask == 0);
    sensor.time_s = 0.09;
    for (uint32_t index = 0; index < 3; ++index) {
        sensor.imus[index].sample_time_s = sensor.time_s;
    }
    assert(fsw_step(handle, &sensor, &output) == FSW_STATUS_OK);
    assert(output.imu_usable_mask == 0b111);
    fsw_destroy(handle);
}

void test_two_channel_disagreement_does_not_guess() {
    auto config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    for (int step = 0; step < 5; ++step) {
        auto sensor = sensor_at(step * 0.01);
        sensor.imu_count = 2;
        sensor.imus[1] = sensor.imus[0];
        sensor.imus[1].acceleration_body_m_s2[0] = 30.0;
        assert(fsw_step(handle, &sensor, &output) == FSW_STATUS_OK);
        assert(output.attitude_valid == 0);
        assert(output.imu_usable_mask == 0);
        assert(output.imu_rejected_mask == 0);
    }
    assert(output.mode == FSW_MODE_ARMED);
    assert(output.fault_flags & FSW_FAULT_IMU_UNAVAILABLE);
    assert(output.fault_flags & FSW_FAULT_IMU_DISAGREEMENT);
    fsw_destroy(handle);
}

void test_barometer_and_gnss_voting() {
    auto config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    for (int step = 0; step < 3; ++step) {
        auto sensor = sensor_at(step * 0.01);
        fill_three_channels(sensor);
        sensor.barometers[2].altitude_m += 100.0;
        sensor.gnss[2].gnss_position_ecef_m[0] += 100.0;
        sensor.gnss[2].gnss_velocity_ecef_m_s[0] += 10.0;
        sensor.gnss[2].vertical_velocity_m_s += 10.0;
        assert(fsw_step(handle, &sensor, &output) == FSW_STATUS_OK);
        assert(output.barometer_usable_mask == 0b011);
        assert(output.gnss_usable_mask == 0b011);
    }
    assert(output.barometer_rejected_mask == 0b100);
    assert(output.gnss_rejected_mask == 0b100);
    assert(output.fault_flags & FSW_FAULT_BAROMETER_DISAGREEMENT);
    assert(output.fault_flags & FSW_FAULT_GNSS_DISAGREEMENT);
    assert(output.navigation_status == FSW_NAV_NOMINAL);
    fsw_destroy(handle);
}

void test_cross_altitude_disagreement() {
    auto config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};

    auto initial = sensor_at(0.0);
    initial.barometers[0].altitude_m = 0.0;
    initial.gnss[0].gnss_position_ecef_m[0] = kEarthRadiusM;
    assert(fsw_step(handle, &initial, &output) == FSW_STATUS_OK);

    auto isolated = sensor_at(0.01);
    isolated.barometers[0].altitude_m = 1.0;
    isolated.gnss[0].gnss_position_ecef_m[0] = kEarthRadiusM + 100.0;
    assert(fsw_step(handle, &isolated, &output) == FSW_STATUS_OK);
    assert(output.navigation_status == FSW_NAV_DEGRADED);
    assert(
        output.disagreement_flags
        & FSW_DISAGREEMENT_CROSS_ALTITUDE
    );
    assert(output.fault_flags & FSW_FAULT_NAV_DISAGREEMENT);

    auto unresolved = sensor_at(0.02);
    unresolved.barometers[0].altitude_m = 100.0;
    unresolved.gnss[0].gnss_position_ecef_m[0] =
        kEarthRadiusM - 100.0;
    assert(fsw_step(handle, &unresolved, &output) == FSW_STATUS_OK);
    assert(output.navigation_status == FSW_NAV_INERTIAL);
    fsw_destroy(handle);
}

void test_gyro_bias_and_powered_imu_loss() {
    auto bias_config = default_config();
    bias_config.gyro_bias_time_constant_s = 0.01;
    FswHandle bias_handle = fsw_create(&bias_config);
    FswOutput output{};
    for (int step = 0; step < 40; ++step) {
        auto sensor = sensor_at(step * 0.001);
        sensor.dt_s = 0.001;
        sensor.imus[0].gyro_body_rad_s[1] = 0.005;
        assert(fsw_step(bias_handle, &sensor, &output) == FSW_STATUS_OK);
    }
    assert(output.gyro_bias_rad_s[1] > 0.0048);
    auto ignition = sensor_at(0.05);
    ignition.imus[0].gyro_body_rad_s[1] = 0.005;
    assert(fsw_step(bias_handle, &ignition, &output) == FSW_STATUS_OK);
    const double frozen_bias = output.gyro_bias_rad_s[1];
    auto after_ignition = sensor_at(0.06);
    after_ignition.imus[0].gyro_body_rad_s[1] = 0.01;
    assert(
        fsw_step(bias_handle, &after_ignition, &output)
        == FSW_STATUS_OK
    );
    assert(std::abs(output.gyro_bias_rad_s[1] - frozen_bias) < 1e-12);
    fsw_destroy(bias_handle);

    auto config = default_config();
    FswHandle handle = fsw_create(&config);
    for (int step = 0; step <= 10; ++step) {
        auto sensor = sensor_at(step * 0.01);
        assert(fsw_step(handle, &sensor, &output) == FSW_STATUS_OK);
    }
    assert(output.mode == FSW_MODE_BOOST_1);
    for (int step = 11; step <= 14; ++step) {
        auto sensor = sensor_at(step * 0.01);
        sensor.imu_count = 0;
        assert(fsw_step(handle, &sensor, &output) == FSW_STATUS_OK);
        assert(output.mode == FSW_MODE_BOOST_1);
        assert(output.tvc_pitch_rad == 0.0);
    }
    auto lost = sensor_at(0.15);
    lost.imu_count = 0;
    assert(fsw_step(handle, &lost, &output) == FSW_STATUS_OK);
    assert(output.mode == FSW_MODE_ABORT);
    assert(output.abort == 1);
    assert(output.tvc_pitch_rad == 0.0);
    fsw_destroy(handle);
}

void test_missing_imu_prevents_ignition_and_nav_loss_gates_recovery() {
    auto config = default_config();
    FswHandle handle = fsw_create(&config);
    FswOutput output{};
    for (int step = 0; step <= 10; ++step) {
        auto sensor = sensor_at(step * 0.01);
        sensor.imu_count = 0;
        assert(fsw_step(handle, &sensor, &output) == FSW_STATUS_OK);
    }
    assert(output.mode == FSW_MODE_ARMED);
    fsw_destroy(handle);

    auto recovery_config = default_config();
    recovery_config.body_role = FSW_BODY_CORE;
    FswHandle recovery_handle = fsw_create(&recovery_config);
    for (int step = 0; step < 30; ++step) {
        auto sensor = sensor_at(step * 0.01);
        sensor.barometer_count = 0;
        sensor.gnss_count = 0;
        assert(
            fsw_step(recovery_handle, &sensor, &output)
            == FSW_STATUS_OK
        );
    }
    assert(output.mode == FSW_MODE_COAST);

    for (int step = 30; step < 55; ++step) {
        auto sensor = sensor_at(step * 0.01);
        sensor.imus[0].acceleration_body_m_s2[0] = 0.0;
        sensor.barometers[0].altitude_m = 200.0;
        sensor.gnss[0].gnss_position_ecef_m[0] =
            kEarthRadiusM + 200.0;
        sensor.gnss[0].vertical_velocity_m_s = -10.0;
        assert(
            fsw_step(recovery_handle, &sensor, &output)
            == FSW_STATUS_OK
        );
    }
    assert(output.mode == FSW_MODE_DROGUE);
    fsw_destroy(recovery_handle);
}

}  // namespace

int main() {
    assert(std::strcmp(fsw_version(), "fsw-core-0.3.0") == 0);
    test_single_channel_compatibility();
    test_imu_rejection_recovery_and_new_sample_debounce();
    test_two_channel_disagreement_does_not_guess();
    test_barometer_and_gnss_voting();
    test_cross_altitude_disagreement();
    test_gyro_bias_and_powered_imu_loss();
    test_missing_imu_prevents_ignition_and_nav_loss_gates_recovery();
    return 0;
}
