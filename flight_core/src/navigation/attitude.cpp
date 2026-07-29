
#include "navigation/attitude.hpp"

#include <cmath>

#include "core/context.hpp"
#include "math/frames.hpp"
#include "math/quaternion.hpp"
#include "math/vector3.hpp"
#include "sensors/sensor_voting.hpp"

namespace fsw::internal {

std::array<double, 4> relative_attitude(const Context& context) {
    auto relative = multiply(
        conjugate(context.launch_reference_attitude),
        context.attitude
    );
    normalize(relative);
    return relative;
}

void update_gyro_bias(
    Context& context,
    const VotedSensors& voted,
    double dt_s
) {
    if (
        context.mode > FSW_MODE_ARMED
        || vector_norm(voted.gyro.data(), 3)
            > context.config.stationary_gyro_threshold_rad_s
    ) {
        return;
    }
    const double alpha = 1.0 - std::exp(
        -dt_s / context.config.gyro_bias_time_constant_s
    );
    const std::array<double, 3> earth_rate_ecef{
        0.0, 0.0, kEarthRotationRadS
    };
    const auto earth_rate_body = rotate(
        conjugate(context.attitude), earth_rate_ecef
    );
    for (int axis = 0; axis < 3; ++axis) {
        context.gyro_bias[axis] += alpha * (
            voted.gyro[axis]
            - earth_rate_body[axis]
            - context.gyro_bias[axis]
        );
    }
}

void integrate_attitude(
    Context& context,
    const std::array<double, 3>& gyro,
    double dt_s
) {
    const auto q = context.attitude;
    const double gx = gyro[0];
    const double gy = gyro[1];
    const double gz = gyro[2];
    const double half_dt = 0.5 * dt_s;
    context.attitude = {
        q[0] + (-q[1] * gx - q[2] * gy - q[3] * gz) * half_dt,
        q[1] + (q[0] * gx + q[2] * gz - q[3] * gy) * half_dt,
        q[2] + (q[0] * gy - q[1] * gz + q[3] * gx) * half_dt,
        q[3] + (q[0] * gz + q[1] * gy - q[2] * gx) * half_dt,
    };
    normalize(context.attitude);
    context.last_gyro = gyro;
}

}  // namespace fsw::internal
