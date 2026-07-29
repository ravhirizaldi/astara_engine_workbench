#pragma once

#include <array>
#include <cstdint>

#include "fsw/fsw.h"
#include "sensors/channel_health.hpp"

namespace fsw::internal {

struct Context;
struct VotedSensors;

double median3(double first, double second, double third);
uint32_t bit_count(uint32_t mask);

template <typename Samples>
uint32_t aligned_to_freshest_mask(
    const Samples& samples,
    uint32_t count,
    uint32_t candidate_mask,
    uint32_t reference_mask,
    double max_skew_s
) {
    double freshest_time_s = -1.0;
    for (uint32_t index = 0; index < count; ++index) {
        if (
            reference_mask & (1u << index)
            && samples[index].sample_time_s > freshest_time_s
        ) {
            freshest_time_s = samples[index].sample_time_s;
        }
    }
    uint32_t aligned_mask = 0;
    for (uint32_t index = 0; index < count; ++index) {
        double skew_s = samples[index].sample_time_s - freshest_time_s;
        if (skew_s < 0.0) {
            skew_s = -skew_s;
        }
        if (
            candidate_mask & (1u << index)
            && freshest_time_s >= 0.0
            && skew_s <= max_skew_s + 1e-12
        ) {
            aligned_mask |= 1u << index;
        }
    }
    return aligned_mask;
}

template <typename Samples>
uint32_t freshest_aligned_mask(
    const Samples& samples,
    uint32_t count,
    uint32_t candidate_mask,
    double max_skew_s
) {
    return aligned_to_freshest_mask(
        samples,
        count,
        candidate_mask,
        candidate_mask,
        max_skew_s
    );
}

bool new_sample(ChannelHealth& health, double sample_time_s);
void observe_channel(
    ChannelHealth& health,
    bool agrees,
    bool allow_rejection,
    const FswConfig& config
);
uint32_t rejected_mask(
    const std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS>& health,
    uint32_t count
);

void vote_imus(
    Context& context,
    const FswSensorSuite& suite,
    VotedSensors& voted
);
void vote_magnetometers(
    Context& context,
    const FswSensorSuite& suite,
    VotedSensors& voted
);
void vote_barometers(
    Context& context,
    const FswSensorSuite& suite,
    VotedSensors& voted
);
void vote_gnss(
    Context& context,
    const FswSensorSuite& suite,
    VotedSensors& voted
);

}  // namespace fsw::internal
