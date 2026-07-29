#pragma once

#include "fsw/fsw.h"

namespace fsw::internal {

struct Context;
struct VotedSensors;

bool update_navigation(
    Context& context,
    const VotedSensors& voted
);

}  // namespace fsw::internal
