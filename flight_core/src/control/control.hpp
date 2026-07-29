#pragma once

#include "fsw/fsw.h"

namespace fsw::internal {

struct Context;

void calculate_controls(
    const Context& context,
    const FswInput& input,
    FswOutput& output
);

}  // namespace fsw::internal
