#pragma once

#include "fsw/fsw.h"

namespace fsw::internal {

struct Context;

void clear_output(
    FswOutput& output,
    int32_t status = FSW_STATUS_INVALID_ARGUMENT
);
void populate_output(const Context& context, FswOutput& output);

}  // namespace fsw::internal
