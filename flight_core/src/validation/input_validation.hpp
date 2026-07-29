#pragma once

#include "fsw/fsw.h"

namespace fsw::internal {

bool valid_input(const FswInput& input, const FswConfig& config);
bool fresh(
    bool valid,
    double sample_time_s,
    double time_s,
    double timeout_s
);

}  // namespace fsw::internal
