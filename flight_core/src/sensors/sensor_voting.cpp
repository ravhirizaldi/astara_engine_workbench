
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

const double* imu_values(const FswImuSample& sample, bool accelerometer) {
    return accelerometer
        ? sample.acceleration_body_m_s2
        : sample.gyro_body_rad_s;
}

bool evaluate_imu_component(
    ChannelHealth& health,
    const FswImuSample& sample,
    double time_s,
    const FswConfig& config,
    bool accelerometer
) {
    health.flags = 0;
    const bool valid = accelerometer ? sample.accel_valid : sample.gyro_valid;
    health.age_s = valid
        ? std::max(time_s - sample.sample_time_s, 0.0)
        : 0.0;
    if (!valid) {
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
        vector_norm(imu_values(sample, accelerometer), 3)
            > (
                accelerometer
                    ? config.max_acceleration_m_s2
                    : config.max_gyro_rad_s
            )
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

bool imu_components_agree(
    const FswImuSample& left,
    const FswImuSample& right,
    double threshold,
    bool accelerometer
) {
    return vector_distance(
        imu_values(left, accelerometer),
        imu_values(right, accelerometer),
        3
    ) <= threshold;
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

struct ImuComponentVote {
    bool valid{};
    std::array<double, 3> value{};
    double sample_time_s{-1.0};
    uint32_t usable_mask{};
};

void average_imu_component(
    const FswSensorSuite& suite,
    uint32_t mask,
    bool accelerometer,
    ImuComponentVote& vote
) {
    const double divisor = static_cast<double>(bit_count(mask));
    for (uint32_t index = 0; index < suite.imu_count; ++index) {
        if ((mask & (1u << index)) == 0) {
            continue;
        }
        const auto& sample = suite.imus[index];
        for (int axis = 0; axis < 3; ++axis) {
            vote.value[axis] +=
                imu_values(sample, accelerometer)[axis] / divisor;
        }
        vote.sample_time_s = std::max(
            vote.sample_time_s, sample.sample_time_s
        );
    }
    vote.usable_mask = mask;
    vote.valid = mask != 0;
}

ImuComponentVote vote_imu_component(
    Context& context,
    const FswSensorSuite& suite,
    std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS>& health,
    bool accelerometer,
    double disagreement_threshold,
    uint32_t disagreement_flag,
    uint32_t& disagreement_flags
) {
    uint32_t fresh_mask = 0;
    uint32_t healthy_mask = 0;
    for (uint32_t index = 0; index < suite.imu_count; ++index) {
        const auto& sample = suite.imus[index];
        if (evaluate_imu_component(
            health[index],
            sample,
            suite.time_s,
            context.config,
            accelerometer
        )) {
            fresh_mask |= 1u << index;
            if (!health[index].rejected) {
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
        if (imu_components_agree(
            suite.imus[indices[0]],
            suite.imus[indices[1]],
            disagreement_threshold,
            accelerometer
        )) {
            consensus_mask = healthy_mask;
        } else {
            disagreement_flags |= disagreement_flag;
        }
    } else if (healthy_count == 3) {
        double median[3]{};
        for (int axis = 0; axis < 3; ++axis) {
            median[axis] = median3(
                imu_values(suite.imus[0], accelerometer)[axis],
                imu_values(suite.imus[1], accelerometer)[axis],
                imu_values(suite.imus[2], accelerometer)[axis]
            );
        }
        for (uint32_t index = 0; index < suite.imu_count; ++index) {
            if (vector_distance(
                imu_values(suite.imus[index], accelerometer),
                median,
                3
            ) <= disagreement_threshold) {
                consensus_mask |= 1u << index;
            } else {
                disagreement_flags |= disagreement_flag;
            }
        }
        if (bit_count(consensus_mask) < 2) {
            consensus_mask = 0;
        }
    }

    ImuComponentVote vote{};
    if (consensus_mask != 0) {
        average_imu_component(
            suite, consensus_mask, accelerometer, vote
        );
    }
    for (uint32_t index = 0; index < suite.imu_count; ++index) {
        if ((fresh_mask & (1u << index)) == 0) {
            continue;
        }
        auto& channel = health[index];
        if (!new_sample(channel, suite.imus[index].sample_time_s)) {
            continue;
        }
        bool agrees = (consensus_mask & (1u << index)) != 0;
        if (!agrees) {
            channel.flags |= FSW_SENSOR_HEALTH_DISAGREEMENT;
        }
        if (channel.rejected && vote.valid) {
            FswImuSample fused{};
            for (int axis = 0; axis < 3; ++axis) {
                if (accelerometer) {
                    fused.acceleration_body_m_s2[axis] = vote.value[axis];
                } else {
                    fused.gyro_body_rad_s[axis] = vote.value[axis];
                }
            }
            agrees = imu_components_agree(
                suite.imus[index],
                fused,
                disagreement_threshold,
                accelerometer
            );
        }
        observe_channel(
            channel,
            agrees,
            healthy_count >= 3,
            context.config
        );
        if (channel.rejected) {
            channel.flags |= FSW_SENSOR_HEALTH_REJECTED;
        }
    }
    return vote;
}

void vote_imus(
    Context& context,
    const FswSensorSuite& suite,
    VotedSensors& voted
) {
    const ImuComponentVote accelerometer = vote_imu_component(
        context,
        suite,
        context.sensors.accelerometer_health,
        true,
        context.config.acceleration_disagreement_m_s2,
        FSW_DISAGREEMENT_ACCELERATION,
        voted.disagreement_flags
    );
    const ImuComponentVote gyroscope = vote_imu_component(
        context,
        suite,
        context.sensors.gyroscope_health,
        false,
        context.config.gyro_disagreement_rad_s,
        FSW_DISAGREEMENT_GYRO,
        voted.disagreement_flags
    );
    voted.accelerometer_valid = accelerometer.valid;
    voted.gyroscope_valid = gyroscope.valid;
    voted.imu_valid = accelerometer.valid && gyroscope.valid;
    voted.acceleration = accelerometer.value;
    voted.gyro = gyroscope.value;
    voted.accelerometer_sample_time_s = accelerometer.sample_time_s;
    voted.gyroscope_sample_time_s = gyroscope.sample_time_s;
    voted.accelerometer_usable_mask = accelerometer.usable_mask;
    voted.gyroscope_usable_mask = gyroscope.usable_mask;
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

    context.sensors.accelerometer_usable_mask =
        voted.accelerometer_usable_mask;
    context.sensors.gyroscope_usable_mask = voted.gyroscope_usable_mask;
    context.sensors.magnetometer_usable_mask =
        voted.magnetometer_usable_mask;
    context.sensors.barometer_usable_mask = voted.barometer_usable_mask;
    context.sensors.gnss_usable_mask = voted.gnss_usable_mask;
    context.sensors.accelerometer_rejected_mask = rejected_mask(
        context.sensors.accelerometer_health, suite.imu_count
    );
    context.sensors.gyroscope_rejected_mask = rejected_mask(
        context.sensors.gyroscope_health, suite.imu_count
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
    if (bit_count(voted.accelerometer_usable_mask) == 1) {
        context.sensors.sensor_status_flags |=
            FSW_SENSOR_STATUS_ACCELEROMETER_SINGLE_SOURCE;
    }
    if (bit_count(voted.gyroscope_usable_mask) == 1) {
        context.sensors.sensor_status_flags |=
            FSW_SENSOR_STATUS_GYROSCOPE_SINGLE_SOURCE;
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
