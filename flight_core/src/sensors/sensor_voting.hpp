#pragma once

#include <array>
#include <cstdint>

#include "fsw/fsw.h"

namespace fsw::internal {

struct Context;

struct VotedSensors {
    bool accelerometer_valid{};
    bool gyroscope_valid{};
    bool imu_valid{};
    bool magnetometer_valid{};
    bool barometer_valid{};
    bool gnss_valid{};
    std::array<double, 3> acceleration{};
    std::array<double, 3> gyro{};
    std::array<double, 3> magnetic{};
    std::array<double, 3> gnss_position{};
    std::array<double, 3> gnss_velocity{};
    double barometric_altitude{};
    double vertical_velocity{};
    double accelerometer_sample_time_s{-1.0};
    double gyroscope_sample_time_s{-1.0};
    double magnetometer_sample_time_s{-1.0};
    double barometer_sample_time_s{-1.0};
    double gnss_sample_time_s{-1.0};
    uint32_t accelerometer_usable_mask{};
    uint32_t gyroscope_usable_mask{};
    uint32_t magnetometer_usable_mask{};
    uint32_t barometer_usable_mask{};
    uint32_t gnss_usable_mask{};
    uint32_t disagreement_flags{};
    double barometer_innovation{};
    double gnss_altitude_innovation{};
    double gnss_velocity_innovation{};
};

VotedSensors vote_sensors(Context& context, const FswSensorSuite& suite);

}  // namespace fsw::internal
