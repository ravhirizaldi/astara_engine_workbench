#pragma once

#include <array>
#include <cstdint>

namespace fsw::internal {

struct ChannelHealth {
    uint32_t bad_samples{};
    uint32_t good_samples{};
    bool rejected{};
    double last_evaluated_sample_time_s{-1.0};
    double last_seen_time_s{-1.0};
    double last_accepted_time_s{-1.0};
    std::array<double, 3> last_seen_values{};
    std::array<double, 3> last_accepted_values{};
    uint32_t flags{};
    double age_s{};
};

}  // namespace fsw::internal
