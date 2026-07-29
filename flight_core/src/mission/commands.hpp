#pragma once

#include <cstdint>

#include "fsw/fsw.h"

namespace fsw::internal {

struct Context;
struct VotedSensors;

uint32_t launch_inhibits(
    const Context& context,
    const FswInput& input,
    const VotedSensors& voted
);
void process_command(
    Context& context,
    const FswInput& input,
    const VotedSensors& voted
);

}  // namespace fsw::internal
