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
