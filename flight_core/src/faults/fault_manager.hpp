#pragma once

#include <cstdint>

#include "fsw/fsw.h"

namespace fsw::internal {

struct Context;
struct VotedSensors;

uint32_t critical_fault_mask();
void commit_faults(Context& context, uint32_t flags);
void set_faults(
    Context& context,
    const FswInput& input,
    const VotedSensors& voted,
    bool navigation_disagreement
);
void raise_fault(Context& context, uint32_t fault);

}  // namespace fsw::internal
