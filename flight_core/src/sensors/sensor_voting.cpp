
#include "sensors/sensor_voting.hpp"

#include <algorithm>
#include <cmath>

#include "core/context.hpp"
#include "math/frames.hpp"
#include "math/vector3.hpp"
#include "validation/input_validation.hpp"

namespace fsw::internal {

namespace {

double median3(double first, double second, double third) {
    return first + second + third
        - std::min(first, std::min(second, third))
        - std::max(first, std::max(second, third));
}

uint32_t bit_count(uint32_t mask) {
    uint32_t count = 0;
    while (mask != 0) {
        count += mask & 1u;
        mask >>= 1u;
    }
    return count;
}

}  // namespace

bool new_sample(ChannelHealth& health, double sample_time_s) {
    if (sample_time_s <= health.last_evaluated_sample_time_s + kEpsilon) {
        return false;
    }
    health.last_evaluated_sample_time_s = sample_time_s;
    return true;
}

void observe_channel(
    ChannelHealth& health,
    bool agrees,
    bool allow_rejection,
    const FswConfig& config
) {
    if (agrees) {
        health.bad_samples = 0;
        ++health.good_samples;
        if (
            health.rejected
            && health.good_samples >= config.voter_recover_samples
        ) {
            health.rejected = false;
            health.good_samples = 0;
        }
        return;
    }
    health.good_samples = 0;
    ++health.bad_samples;
    if (
        allow_rejection
        && !health.rejected
        && health.bad_samples >= config.voter_reject_samples
    ) {
        health.rejected = true;
        health.bad_samples = 0;
    }
}

uint32_t rejected_mask(
    const std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS>& health,
    uint32_t count
) {
    uint32_t mask = 0;
    for (uint32_t index = 0; index < count; ++index) {
        if (health[index].rejected) {
            mask |= 1u << index;
        }
    }
    return mask;
}

bool evaluate_imu_channel(
    ChannelHealth& health,
    const FswImuSample& sample,
    double time_s,
    const FswConfig& config
) {
    health.flags = 0;
    health.age_s = sample.accel_valid || sample.gyro_valid
        ? std::max(time_s - sample.sample_time_s, 0.0)
        : 0.0;
    if (!sample.accel_valid || !sample.gyro_valid) {
        health.flags |= FSW_SENSOR_HEALTH_INVALID;
        return false;
    }
    if (!fresh(
        true, sample.sample_time_s, time_s, config.imu_timeout_s
    )) {
        health.flags |= FSW_SENSOR_HEALTH_STALE;
        return false;
    }
    if (
        vector_norm(sample.acceleration_body_m_s2, 3)
            > config.max_acceleration_m_s2
        || vector_norm(sample.gyro_body_rad_s, 3) > config.max_gyro_rad_s
    ) {
        health.flags |= FSW_SENSOR_HEALTH_OUT_OF_RANGE;
        return false;
    }
    if (health.rejected) {
        health.flags |= FSW_SENSOR_HEALTH_REJECTED;
    }
    return true;
}

bool evaluate_magnetometer_channel(
    ChannelHealth& health,
    const FswMagnetometerSample& sample,
    double time_s,
    const FswConfig& config
) {
    health.flags = 0;
    health.age_s = sample.valid
        ? std::max(time_s - sample.sample_time_s, 0.0)
        : 0.0;
    if (!sample.valid) {
        health.flags |= FSW_SENSOR_HEALTH_INVALID;
        return false;
    }
    if (!fresh(
        true, sample.sample_time_s, time_s, config.imu_timeout_s
    )) {
        health.flags |= FSW_SENSOR_HEALTH_STALE;
        return false;
    }
    const double norm = vector_norm(sample.magnetic_body, 3);
    if (
        norm < config.min_magnetic_norm
        || norm > config.max_magnetic_norm
    ) {
        health.flags |= FSW_SENSOR_HEALTH_OUT_OF_RANGE;
        return false;
    }
    if (health.rejected) {
        health.flags |= FSW_SENSOR_HEALTH_REJECTED;
    }
    return true;
}

bool evaluate_barometer_channel(
    ChannelHealth& health,
    const FswBarometerSample& sample,
    double time_s,
    const FswConfig& config
) {
    health.flags = 0;
    health.age_s = sample.valid
        ? std::max(time_s - sample.sample_time_s, 0.0)
        : 0.0;
    if (!sample.valid) {
        health.flags |= FSW_SENSOR_HEALTH_INVALID;
        return false;
    }
    if (!fresh(
        true, sample.sample_time_s, time_s, config.barometer_timeout_s
    )) {
        health.flags |= FSW_SENSOR_HEALTH_STALE;
        return false;
    }
    if (
        sample.altitude_m < config.min_barometer_altitude_m
        || sample.altitude_m > config.max_barometer_altitude_m
    ) {
        health.flags |= FSW_SENSOR_HEALTH_OUT_OF_RANGE;
        return false;
    }
    if (
        health.last_accepted_time_s >= 0.0
        && sample.sample_time_s
            > health.last_accepted_time_s + kEpsilon
        && std::abs(
            sample.altitude_m - health.last_accepted_values[0]
        ) / (sample.sample_time_s - health.last_accepted_time_s)
            > config.max_barometer_rate_m_s
    ) {
        health.flags |= FSW_SENSOR_HEALTH_RATE_LIMIT;
        health.last_seen_values[0] = sample.altitude_m;
        health.last_seen_time_s = sample.sample_time_s;
        return false;
    }
    if (sample.sample_time_s > health.last_seen_time_s + kEpsilon) {
        health.last_seen_values[0] = sample.altitude_m;
        health.last_seen_time_s = sample.sample_time_s;
    }
    if (
        sample.sample_time_s > health.last_accepted_time_s + kEpsilon
    ) {
        health.last_accepted_values[0] = sample.altitude_m;
        health.last_accepted_time_s = sample.sample_time_s;
    }
    if (health.rejected) {
        health.flags |= FSW_SENSOR_HEALTH_REJECTED;
    }
    return true;
}

bool evaluate_gnss_channel(
    ChannelHealth& health,
    const FswGnssSample& sample,
    double time_s,
    const FswConfig& config
) {
    health.flags = 0;
    health.age_s = sample.valid
        ? std::max(time_s - sample.sample_time_s, 0.0)
        : 0.0;
    if (!sample.valid) {
        health.flags |= FSW_SENSOR_HEALTH_INVALID;
        return false;
    }
    if (!fresh(
        true, sample.sample_time_s, time_s, config.gnss_timeout_s
    )) {
        health.flags |= FSW_SENSOR_HEALTH_STALE;
        return false;
    }
    const double radius = vector_norm(sample.gnss_position_ecef_m, 3);
    const double speed = vector_norm(sample.gnss_velocity_ecef_m_s, 3);
    if (
        radius < config.min_gnss_radius_m
        || radius > config.max_gnss_radius_m
        || speed > config.max_gnss_speed_m_s
    ) {
        health.flags |= FSW_SENSOR_HEALTH_OUT_OF_RANGE;
        return false;
    }
    if (
        health.last_accepted_time_s >= 0.0
        && sample.sample_time_s
            > health.last_accepted_time_s + kEpsilon
        && vector_distance(
            sample.gnss_velocity_ecef_m_s,
            health.last_accepted_values.data(),
            3
        ) / (sample.sample_time_s - health.last_accepted_time_s)
            > config.max_gnss_velocity_rate_m_s2
    ) {
        health.flags |= FSW_SENSOR_HEALTH_RATE_LIMIT;
        for (int axis = 0; axis < 3; ++axis) {
            health.last_seen_values[axis] =
                sample.gnss_velocity_ecef_m_s[axis];
        }
        health.last_seen_time_s = sample.sample_time_s;
        return false;
    }
    if (sample.sample_time_s > health.last_seen_time_s + kEpsilon) {
        for (int axis = 0; axis < 3; ++axis) {
            health.last_seen_values[axis] =
                sample.gnss_velocity_ecef_m_s[axis];
        }
        health.last_seen_time_s = sample.sample_time_s;
    }
    if (
        sample.sample_time_s > health.last_accepted_time_s + kEpsilon
    ) {
        for (int axis = 0; axis < 3; ++axis) {
            health.last_accepted_values[axis] =
                sample.gnss_velocity_ecef_m_s[axis];
        }
        health.last_accepted_time_s = sample.sample_time_s;
    }
    if (health.rejected) {
        health.flags |= FSW_SENSOR_HEALTH_REJECTED;
    }
    return true;
}

bool imu_samples_agree(
    const FswImuSample& left,
    const FswImuSample& right,
    const FswConfig& config,
    uint32_t& disagreement_flags
) {
    const bool acceleration_agrees = vector_distance(
        left.acceleration_body_m_s2, right.acceleration_body_m_s2, 3
    ) <= config.acceleration_disagreement_m_s2;
    const bool gyro_agrees = vector_distance(
        left.gyro_body_rad_s, right.gyro_body_rad_s, 3
    ) <= config.gyro_disagreement_rad_s;
    if (!acceleration_agrees) {
        disagreement_flags |= FSW_DISAGREEMENT_ACCELERATION;
    }
    if (!gyro_agrees) {
        disagreement_flags |= FSW_DISAGREEMENT_GYRO;
    }
    return acceleration_agrees && gyro_agrees;
}

bool magnetometer_samples_agree(
    const FswMagnetometerSample& left,
    const FswMagnetometerSample& right,
    const FswConfig& config
) {
    return vector_distance(
        left.magnetic_body, right.magnetic_body, 3
    ) <= config.magnetic_disagreement;
}

void average_imus(
    const FswSensorSuite& suite,
    uint32_t mask,
    VotedSensors& voted
) {
    const double divisor = static_cast<double>(bit_count(mask));
    for (uint32_t index = 0; index < suite.imu_count; ++index) {
        if ((mask & (1u << index)) == 0) {
            continue;
        }
        const auto& sample = suite.imus[index];
        for (int axis = 0; axis < 3; ++axis) {
            voted.acceleration[axis] +=
                sample.acceleration_body_m_s2[axis] / divisor;
            voted.gyro[axis] += sample.gyro_body_rad_s[axis] / divisor;
        }
        voted.imu_sample_time_s = std::max(
            voted.imu_sample_time_s, sample.sample_time_s
        );
    }
    voted.imu_usable_mask = mask;
    voted.imu_valid = mask != 0;
}

void vote_imus(
    Context& context,
    const FswSensorSuite& suite,
    VotedSensors& voted
) {
    uint32_t fresh_mask = 0;
    uint32_t healthy_mask = 0;
    for (uint32_t index = 0; index < suite.imu_count; ++index) {
        const auto& sample = suite.imus[index];
        if (evaluate_imu_channel(
            context.sensors.imu_health[index],
            sample,
            suite.time_s,
            context.config
        )) {
            fresh_mask |= 1u << index;
            if (!context.sensors.imu_health[index].rejected) {
                healthy_mask |= 1u << index;
            }
        }
    }

    uint32_t consensus_mask = 0;
    const uint32_t healthy_count = bit_count(healthy_mask);
    if (healthy_count == 1) {
        consensus_mask = healthy_mask;
    } else if (healthy_count == 2) {
        int indices[2]{};
        int found = 0;
        for (uint32_t index = 0; index < suite.imu_count; ++index) {
            if (healthy_mask & (1u << index)) {
                indices[found++] = static_cast<int>(index);
            }
        }
        if (imu_samples_agree(
            suite.imus[indices[0]],
            suite.imus[indices[1]],
            context.config,
            voted.disagreement_flags
        )) {
            consensus_mask = healthy_mask;
        }
    } else if (healthy_count == 3) {
        double acceleration_median[3]{};
        double gyro_median[3]{};
        for (int axis = 0; axis < 3; ++axis) {
            acceleration_median[axis] = median3(
                suite.imus[0].acceleration_body_m_s2[axis],
                suite.imus[1].acceleration_body_m_s2[axis],
                suite.imus[2].acceleration_body_m_s2[axis]
            );
            gyro_median[axis] = median3(
                suite.imus[0].gyro_body_rad_s[axis],
                suite.imus[1].gyro_body_rad_s[axis],
                suite.imus[2].gyro_body_rad_s[axis]
            );
        }
        for (uint32_t index = 0; index < suite.imu_count; ++index) {
            const bool acceleration_agrees = vector_distance(
                suite.imus[index].acceleration_body_m_s2,
                acceleration_median,
                3
            ) <= context.config.acceleration_disagreement_m_s2;
            const bool gyro_agrees = vector_distance(
                suite.imus[index].gyro_body_rad_s,
                gyro_median,
                3
            ) <= context.config.gyro_disagreement_rad_s;
            if (acceleration_agrees && gyro_agrees) {
                consensus_mask |= 1u << index;
            } else {
                if (!acceleration_agrees) {
                    voted.disagreement_flags |=
                        FSW_DISAGREEMENT_ACCELERATION;
                }
                if (!gyro_agrees) {
                    voted.disagreement_flags |= FSW_DISAGREEMENT_GYRO;
                }
            }
        }
        if (bit_count(consensus_mask) < 2) {
            consensus_mask = 0;
        }
    }

    if (consensus_mask != 0) {
        average_imus(suite, consensus_mask, voted);
    }
    for (uint32_t index = 0; index < suite.imu_count; ++index) {
        if ((fresh_mask & (1u << index)) == 0) {
            continue;
        }
        auto& health = context.sensors.imu_health[index];
        if (!new_sample(health, suite.imus[index].sample_time_s)) {
            continue;
        }
        bool agrees = (consensus_mask & (1u << index)) != 0;
        if (!agrees) {
            health.flags |= FSW_SENSOR_HEALTH_DISAGREEMENT;
        }
        if (health.rejected && voted.imu_valid) {
            FswImuSample fused{};
            for (int axis = 0; axis < 3; ++axis) {
                fused.acceleration_body_m_s2[axis] =
                    voted.acceleration[axis];
                fused.gyro_body_rad_s[axis] = voted.gyro[axis];
            }
            uint32_t ignored_flags = 0;
            agrees = imu_samples_agree(
                suite.imus[index],
                fused,
                context.config,
                ignored_flags
            );
        }
        observe_channel(
            health,
            agrees,
            healthy_count >= 3,
            context.config
        );
        if (health.rejected) {
            health.flags |= FSW_SENSOR_HEALTH_REJECTED;
        }
    }
}

void average_magnetometers(
    const FswSensorSuite& suite,
    uint32_t mask,
    VotedSensors& voted
) {
    const double divisor = static_cast<double>(bit_count(mask));
    for (
        uint32_t index = 0;
        index < suite.magnetometer_count;
        ++index
    ) {
        if ((mask & (1u << index)) == 0) {
            continue;
        }
        const auto& sample = suite.magnetometers[index];
        for (int axis = 0; axis < 3; ++axis) {
            voted.magnetic[axis] += sample.magnetic_body[axis] / divisor;
        }
        voted.magnetometer_sample_time_s = std::max(
            voted.magnetometer_sample_time_s,
            sample.sample_time_s
        );
    }
    voted.magnetometer_usable_mask = mask;
    voted.magnetometer_valid = mask != 0;
}

void vote_magnetometers(
    Context& context,
    const FswSensorSuite& suite,
    VotedSensors& voted
) {
    uint32_t fresh_mask = 0;
    uint32_t healthy_mask = 0;
    for (
        uint32_t index = 0;
        index < suite.magnetometer_count;
        ++index
    ) {
        const auto& sample = suite.magnetometers[index];
        if (evaluate_magnetometer_channel(
            context.sensors.magnetometer_health[index],
            sample,
            suite.time_s,
            context.config
        )) {
            fresh_mask |= 1u << index;
            if (!context.sensors.magnetometer_health[index].rejected) {
                healthy_mask |= 1u << index;
            }
        }
    }

    uint32_t consensus_mask = 0;
    const uint32_t healthy_count = bit_count(healthy_mask);
    if (healthy_count == 1) {
        consensus_mask = healthy_mask;
    } else if (healthy_count == 2) {
        int indices[2]{};
        int found = 0;
        for (
            uint32_t index = 0;
            index < suite.magnetometer_count;
            ++index
        ) {
            if (healthy_mask & (1u << index)) {
                indices[found++] = static_cast<int>(index);
            }
        }
        if (magnetometer_samples_agree(
            suite.magnetometers[indices[0]],
            suite.magnetometers[indices[1]],
            context.config
        )) {
            consensus_mask = healthy_mask;
        } else {
            voted.disagreement_flags |=
                FSW_DISAGREEMENT_MAGNETOMETER;
        }
    } else if (healthy_count == 3) {
        double center[3]{};
        for (int axis = 0; axis < 3; ++axis) {
            center[axis] = median3(
                suite.magnetometers[0].magnetic_body[axis],
                suite.magnetometers[1].magnetic_body[axis],
                suite.magnetometers[2].magnetic_body[axis]
            );
        }
        for (
            uint32_t index = 0;
            index < suite.magnetometer_count;
            ++index
        ) {
            if (
                vector_distance(
                    suite.magnetometers[index].magnetic_body,
                    center,
                    3
                ) <= context.config.magnetic_disagreement
            ) {
                consensus_mask |= 1u << index;
            } else {
                voted.disagreement_flags |=
                    FSW_DISAGREEMENT_MAGNETOMETER;
            }
        }
        if (bit_count(consensus_mask) < 2) {
            consensus_mask = 0;
        }
    }

    if (consensus_mask != 0) {
        average_magnetometers(suite, consensus_mask, voted);
    }
    for (
        uint32_t index = 0;
        index < suite.magnetometer_count;
        ++index
    ) {
        if ((fresh_mask & (1u << index)) == 0) {
            continue;
        }
        auto& health = context.sensors.magnetometer_health[index];
        if (!new_sample(
            health, suite.magnetometers[index].sample_time_s
        )) {
            continue;
        }
        bool agrees = (consensus_mask & (1u << index)) != 0;
        if (!agrees) {
            health.flags |= FSW_SENSOR_HEALTH_DISAGREEMENT;
        }
        if (health.rejected && voted.magnetometer_valid) {
            FswMagnetometerSample fused{};
            for (int axis = 0; axis < 3; ++axis) {
                fused.magnetic_body[axis] = voted.magnetic[axis];
            }
            agrees = magnetometer_samples_agree(
                suite.magnetometers[index],
                fused,
                context.config
            );
        }
        observe_channel(
            health,
            agrees,
            healthy_count >= 3,
            context.config
        );
        if (health.rejected) {
            health.flags |= FSW_SENSOR_HEALTH_REJECTED;
        }
    }
}

void vote_barometers(
    Context& context,
    const FswSensorSuite& suite,
    VotedSensors& voted
) {
    uint32_t fresh_mask = 0;
    uint32_t healthy_mask = 0;
    for (uint32_t index = 0; index < suite.barometer_count; ++index) {
        const auto& sample = suite.barometers[index];
        if (evaluate_barometer_channel(
            context.sensors.barometer_health[index],
            sample,
            suite.time_s,
            context.config
        )) {
            fresh_mask |= 1u << index;
            if (!context.sensors.barometer_health[index].rejected) {
                healthy_mask |= 1u << index;
            }
        }
    }

    uint32_t consensus_mask = 0;
    const uint32_t healthy_count = bit_count(healthy_mask);
    if (healthy_count == 1) {
        consensus_mask = healthy_mask;
    } else if (healthy_count == 2) {
        int indices[2]{};
        int found = 0;
        for (uint32_t index = 0; index < suite.barometer_count; ++index) {
            if (healthy_mask & (1u << index)) {
                indices[found++] = static_cast<int>(index);
            }
        }
        if (
            std::abs(
                suite.barometers[indices[0]].altitude_m
                - suite.barometers[indices[1]].altitude_m
            ) <= context.config.barometer_disagreement_m
        ) {
            consensus_mask = healthy_mask;
        } else {
            voted.disagreement_flags |= FSW_DISAGREEMENT_BAROMETER;
        }
    } else if (healthy_count == 3) {
        const double center = median3(
            suite.barometers[0].altitude_m,
            suite.barometers[1].altitude_m,
            suite.barometers[2].altitude_m
        );
        for (uint32_t index = 0; index < suite.barometer_count; ++index) {
            if (
                std::abs(suite.barometers[index].altitude_m - center)
                <= context.config.barometer_disagreement_m
            ) {
                consensus_mask |= 1u << index;
            } else {
                voted.disagreement_flags |= FSW_DISAGREEMENT_BAROMETER;
            }
        }
        if (bit_count(consensus_mask) < 2) {
            consensus_mask = 0;
        }
    }

    if (consensus_mask != 0) {
        const double divisor = static_cast<double>(bit_count(consensus_mask));
        for (uint32_t index = 0; index < suite.barometer_count; ++index) {
            if ((consensus_mask & (1u << index)) == 0) {
                continue;
            }
            voted.barometric_altitude +=
                suite.barometers[index].altitude_m / divisor;
            voted.barometer_sample_time_s = std::max(
                voted.barometer_sample_time_s,
                suite.barometers[index].sample_time_s
            );
        }
        voted.barometer_usable_mask = consensus_mask;
        voted.barometer_valid = true;
    }
    for (uint32_t index = 0; index < suite.barometer_count; ++index) {
        if ((fresh_mask & (1u << index)) == 0) {
            continue;
        }
        auto& health = context.sensors.barometer_health[index];
        if (!new_sample(health, suite.barometers[index].sample_time_s)) {
            continue;
        }
        bool agrees = (consensus_mask & (1u << index)) != 0;
        if (!agrees) {
            health.flags |= FSW_SENSOR_HEALTH_DISAGREEMENT;
        }
        if (health.rejected && voted.barometer_valid) {
            agrees = std::abs(
                suite.barometers[index].altitude_m
                - voted.barometric_altitude
            ) <= context.config.barometer_disagreement_m;
        }
        observe_channel(
            health,
            agrees,
            healthy_count >= 3,
            context.config
        );
        if (health.rejected) {
            health.flags |= FSW_SENSOR_HEALTH_REJECTED;
        }
    }
}

bool gnss_samples_agree(
    const FswGnssSample& left,
    const FswGnssSample& right,
    const FswConfig& config,
    uint32_t& disagreement_flags
) {
    const bool position_agrees = vector_distance(
        left.gnss_position_ecef_m, right.gnss_position_ecef_m, 3
    ) <= config.gnss_position_disagreement_m;
    const bool velocity_agrees = vector_distance(
        left.gnss_velocity_ecef_m_s, right.gnss_velocity_ecef_m_s, 3
    ) <= config.gnss_velocity_disagreement_m_s;
    if (!position_agrees) {
        disagreement_flags |= FSW_DISAGREEMENT_GNSS_POSITION;
    }
    if (!velocity_agrees) {
        disagreement_flags |= FSW_DISAGREEMENT_GNSS_VELOCITY;
    }
    return position_agrees && velocity_agrees;
}

void average_gnss(
    const FswSensorSuite& suite,
    uint32_t mask,
    VotedSensors& voted
) {
    const double divisor = static_cast<double>(bit_count(mask));
    for (uint32_t index = 0; index < suite.gnss_count; ++index) {
        if ((mask & (1u << index)) == 0) {
            continue;
        }
        const auto& sample = suite.gnss[index];
        for (int axis = 0; axis < 3; ++axis) {
            voted.gnss_position[axis] +=
                sample.gnss_position_ecef_m[axis] / divisor;
            voted.gnss_velocity[axis] +=
                sample.gnss_velocity_ecef_m_s[axis] / divisor;
        }
        voted.gnss_sample_time_s = std::max(
            voted.gnss_sample_time_s, sample.sample_time_s
        );
    }
    voted.gnss_usable_mask = mask;
    voted.gnss_valid = mask != 0;
    voted.vertical_velocity = radial_velocity(
        voted.gnss_position.data(),
        voted.gnss_velocity.data()
    );
}

void vote_gnss(
    Context& context,
    const FswSensorSuite& suite,
    VotedSensors& voted
) {
    uint32_t fresh_mask = 0;
    uint32_t healthy_mask = 0;
    for (uint32_t index = 0; index < suite.gnss_count; ++index) {
        const auto& sample = suite.gnss[index];
        if (evaluate_gnss_channel(
            context.sensors.gnss_health[index],
            sample,
            suite.time_s,
            context.config
        )) {
            fresh_mask |= 1u << index;
            if (!context.sensors.gnss_health[index].rejected) {
                healthy_mask |= 1u << index;
            }
        }
    }

    uint32_t consensus_mask = 0;
    const uint32_t healthy_count = bit_count(healthy_mask);
    if (healthy_count == 1) {
        consensus_mask = healthy_mask;
    } else if (healthy_count == 2) {
        int indices[2]{};
        int found = 0;
        for (uint32_t index = 0; index < suite.gnss_count; ++index) {
            if (healthy_mask & (1u << index)) {
                indices[found++] = static_cast<int>(index);
            }
        }
        if (gnss_samples_agree(
            suite.gnss[indices[0]],
            suite.gnss[indices[1]],
            context.config,
            voted.disagreement_flags
        )) {
            consensus_mask = healthy_mask;
        }
    } else if (healthy_count == 3) {
        double position_median[3]{};
        double velocity_median[3]{};
        for (int axis = 0; axis < 3; ++axis) {
            position_median[axis] = median3(
                suite.gnss[0].gnss_position_ecef_m[axis],
                suite.gnss[1].gnss_position_ecef_m[axis],
                suite.gnss[2].gnss_position_ecef_m[axis]
            );
            velocity_median[axis] = median3(
                suite.gnss[0].gnss_velocity_ecef_m_s[axis],
                suite.gnss[1].gnss_velocity_ecef_m_s[axis],
                suite.gnss[2].gnss_velocity_ecef_m_s[axis]
            );
        }
        for (uint32_t index = 0; index < suite.gnss_count; ++index) {
            const bool position_agrees = vector_distance(
                suite.gnss[index].gnss_position_ecef_m,
                position_median,
                3
            ) <= context.config.gnss_position_disagreement_m;
            const bool velocity_agrees = vector_distance(
                suite.gnss[index].gnss_velocity_ecef_m_s,
                velocity_median,
                3
            ) <= context.config.gnss_velocity_disagreement_m_s;
            if (position_agrees && velocity_agrees) {
                consensus_mask |= 1u << index;
            } else {
                if (!position_agrees) {
                    voted.disagreement_flags |=
                        FSW_DISAGREEMENT_GNSS_POSITION;
                }
                if (!velocity_agrees) {
                    voted.disagreement_flags |=
                        FSW_DISAGREEMENT_GNSS_VELOCITY;
                }
            }
        }
        if (bit_count(consensus_mask) < 2) {
            consensus_mask = 0;
        }
    }

    if (consensus_mask != 0) {
        average_gnss(suite, consensus_mask, voted);
    }
    for (uint32_t index = 0; index < suite.gnss_count; ++index) {
        if ((fresh_mask & (1u << index)) == 0) {
            continue;
        }
        auto& health = context.sensors.gnss_health[index];
        if (!new_sample(health, suite.gnss[index].sample_time_s)) {
            continue;
        }
        bool agrees = (consensus_mask & (1u << index)) != 0;
        if (!agrees) {
            health.flags |= FSW_SENSOR_HEALTH_DISAGREEMENT;
        }
        if (health.rejected && voted.gnss_valid) {
            FswGnssSample fused{};
            for (int axis = 0; axis < 3; ++axis) {
                fused.gnss_position_ecef_m[axis] =
                    voted.gnss_position[axis];
                fused.gnss_velocity_ecef_m_s[axis] =
                    voted.gnss_velocity[axis];
            }
            uint32_t ignored_flags = 0;
            agrees = gnss_samples_agree(
                suite.gnss[index],
                fused,
                context.config,
                ignored_flags
            );
        }
        observe_channel(
            health,
            agrees,
            healthy_count >= 3,
            context.config
        );
        if (health.rejected) {
            health.flags |= FSW_SENSOR_HEALTH_REJECTED;
        }
    }
}

VotedSensors vote_sensors(Context& context, const FswSensorSuite& suite) {
    VotedSensors voted{};
    vote_imus(context, suite, voted);
    vote_magnetometers(context, suite, voted);
    vote_barometers(context, suite, voted);
    vote_gnss(context, suite, voted);

    context.sensors.imu_usable_mask = voted.imu_usable_mask;
    context.sensors.magnetometer_usable_mask =
        voted.magnetometer_usable_mask;
    context.sensors.barometer_usable_mask = voted.barometer_usable_mask;
    context.sensors.gnss_usable_mask = voted.gnss_usable_mask;
    context.sensors.imu_rejected_mask = rejected_mask(
        context.sensors.imu_health, suite.imu_count
    );
    context.sensors.magnetometer_rejected_mask = rejected_mask(
        context.sensors.magnetometer_health, suite.magnetometer_count
    );
    context.sensors.barometer_rejected_mask = rejected_mask(
        context.sensors.barometer_health, suite.barometer_count
    );
    context.sensors.gnss_rejected_mask = rejected_mask(
        context.sensors.gnss_health, suite.gnss_count
    );
    context.sensors.disagreement_flags = voted.disagreement_flags;
    context.sensors.sensor_status_flags = 0;
    if (bit_count(voted.imu_usable_mask) == 1) {
        context.sensors.sensor_status_flags |= FSW_SENSOR_STATUS_IMU_SINGLE_SOURCE;
    }
    if (bit_count(voted.magnetometer_usable_mask) == 1) {
        context.sensors.sensor_status_flags |=
            FSW_SENSOR_STATUS_MAGNETOMETER_SINGLE_SOURCE;
    }
    if (bit_count(voted.barometer_usable_mask) == 1) {
        context.sensors.sensor_status_flags |=
            FSW_SENSOR_STATUS_BAROMETER_SINGLE_SOURCE;
    }
    if (bit_count(voted.gnss_usable_mask) == 1) {
        context.sensors.sensor_status_flags |= FSW_SENSOR_STATUS_GNSS_SINGLE_SOURCE;
    }
    return voted;
}

}  // namespace fsw::internal
