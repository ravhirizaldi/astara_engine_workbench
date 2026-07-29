#include "sensors/sensor_voting_internal.hpp"

#include <algorithm>

#include "core/context.hpp"
#include "math/vector3.hpp"
#include "sensors/sensor_voting.hpp"
#include "validation/input_validation.hpp"

namespace fsw::internal {

namespace {

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
        true,
        sample.sample_time_s,
        time_s,
        config.magnetometer_timeout_s
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

bool magnetometer_samples_agree(
    const FswMagnetometerSample& left,
    const FswMagnetometerSample& right,
    const FswConfig& config
) {
    return vector_distance(
        left.magnetic_body, right.magnetic_body, 3
    ) <= config.magnetic_disagreement;
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

}  // namespace

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

}  // namespace fsw::internal
