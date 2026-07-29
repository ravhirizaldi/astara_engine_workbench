#pragma once

#include "fsw/fsw.h"

namespace fsw::internal {

struct Context;
struct VotedSensors;

void update_navigation(
    Context& context,
    const FswInput& input,
    const VotedSensors& voted
);

}  // namespace fsw::internal
