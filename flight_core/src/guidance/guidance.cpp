
#include "guidance/guidance.hpp"

#include "math/frames.hpp"

namespace fsw::internal {

FswGuidancePoint guidance_at(const FswConfig& config, double time_s) {
    if (time_s <= config.guidance[0].time_s) {
        return config.guidance[0];
    }
    for (uint32_t index = 1; index < config.guidance_count; ++index) {
        const auto& right = config.guidance[index];
        if (time_s <= right.time_s) {
            const auto& left = config.guidance[index - 1];
            const double fraction = (
                time_s - left.time_s
            ) / (right.time_s - left.time_s);
            return {
                time_s,
                left.pitch_rad
                    + fraction * (right.pitch_rad - left.pitch_rad),
                left.azimuth_rad
                    + fraction * wrap_angle(
                        right.azimuth_rad - left.azimuth_rad
                    ),
            };
        }
    }
    return config.guidance[config.guidance_count - 1];
}

}  // namespace fsw::internal
