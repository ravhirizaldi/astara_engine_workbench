#include "sensors/sensor_voting_internal.hpp"

#include <algorithm>
#include <array>

#include "core/context.hpp"
#include "math/vector3.hpp"
#include "sensors/sensor_voting.hpp"
#include "validation/input_validation.hpp"

namespace fsw::internal {

namespace {

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
    if (!fresh(true, sample.sample_time_s, time_s, config.imu_timeout_s)) {
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

}  // namespace

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

}  // namespace fsw::internal
