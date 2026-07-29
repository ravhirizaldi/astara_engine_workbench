#pragma once

#include <array>

namespace fsw::internal {

struct Context;
struct VotedSensors;

std::array<double, 4> relative_attitude(const Context& context);
void update_gyro_bias(
    Context& context,
    const VotedSensors& voted,
    double dt_s
);
void integrate_attitude(
    Context& context,
    const std::array<double, 3>& gyro,
    double dt_s
);

}  // namespace fsw::internal
