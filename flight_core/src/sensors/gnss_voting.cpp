#include "sensors/sensor_voting_internal.hpp"

#include <algorithm>

#include "core/context.hpp"
#include "math/frames.hpp"
#include "math/vector3.hpp"
#include "sensors/sensor_voting.hpp"
#include "validation/input_validation.hpp"

namespace fsw::internal {

namespace {

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

}  // namespace

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

    const uint32_t aligned_healthy_mask = freshest_aligned_mask(
        suite.gnss,
        suite.gnss_count,
        healthy_mask,
        context.config.max_voter_sample_skew_s
    );
    const uint32_t aligned_fresh_mask = aligned_to_freshest_mask(
        suite.gnss,
        suite.gnss_count,
        fresh_mask,
        aligned_healthy_mask,
        context.config.max_voter_sample_skew_s
    );
    uint32_t consensus_mask = 0;
    const uint32_t aligned_count = bit_count(aligned_healthy_mask);
    if (aligned_count == 1) {
        consensus_mask = aligned_healthy_mask;
    } else if (aligned_count == 2) {
        int indices[2]{};
        int found = 0;
        for (uint32_t index = 0; index < suite.gnss_count; ++index) {
            if (aligned_healthy_mask & (1u << index)) {
                indices[found++] = static_cast<int>(index);
            }
        }
        if (gnss_samples_agree(
            suite.gnss[indices[0]],
            suite.gnss[indices[1]],
            context.config,
            voted.disagreement_flags
        )) {
            consensus_mask = aligned_healthy_mask;
        }
    } else if (aligned_count == 3) {
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
        if ((aligned_fresh_mask & (1u << index)) == 0) {
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
            aligned_count >= 3,
            context.config
        );
        if (health.rejected) {
            health.flags |= FSW_SENSOR_HEALTH_REJECTED;
        }
    }
}

}  // namespace fsw::internal
