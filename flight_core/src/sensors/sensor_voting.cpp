#include "sensors/sensor_voting.hpp"

#include <algorithm>

#include "core/context.hpp"
#include "math/frames.hpp"
#include "sensors/sensor_voting_internal.hpp"

namespace fsw::internal {

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
        context.sensors.sensor_status_flags |=
            FSW_SENSOR_STATUS_GNSS_SINGLE_SOURCE;
    }
    return voted;
}

}  // namespace fsw::internal
