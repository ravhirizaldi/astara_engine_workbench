#include "sensors/sensor_voting_internal.hpp"

#include <algorithm>
#include <cmath>

#include "core/context.hpp"
#include "math/frames.hpp"
#include "sensors/sensor_voting.hpp"
#include "validation/input_validation.hpp"

namespace fsw::internal {

namespace {

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

}  // namespace

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

}  // namespace fsw::internal
