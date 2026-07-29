#pragma once

#include "fsw/fsw.h"

namespace fsw::internal {

FswGuidancePoint guidance_at(const FswConfig& config, double time_s);

}  // namespace fsw::internal
