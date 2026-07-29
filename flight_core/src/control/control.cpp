
#include "control/control.hpp"

#include <algorithm>

#include "core/context.hpp"
#include "guidance/guidance.hpp"
#include "math/frames.hpp"
#include "math/quaternion.hpp"
#include "navigation/attitude.hpp"
#include "validation/input_validation.hpp"

namespace fsw::internal {

void calculate_controls(
    const Context& context,
    const FswInput& input,
    FswOutput& output
) {
    const auto& suite = input.sensors;
    if (
        !context.attitude_valid
        || (
            context.mode != FSW_MODE_BOOST_1
            && context.mode != FSW_MODE_BOOST_2
        )
    ) {
        return;
    }
    const double guidance_time_s = context.ignition_confirmed_s >= 0.0
        ? std::max(suite.time_s - context.ignition_confirmed_s, 0.0)
        : 0.0;
    const auto target = guidance_at(context.config, guidance_time_s);
    const auto angles = euler(relative_attitude(context));
    const double roll_error = -angles[0];
    const double pitch_error = target.pitch_rad - angles[1];
    const double yaw_error = wrap_angle(
        target.azimuth_rad
        - context.config.guidance[0].azimuth_rad
        - angles[2]
    );
    const double pitch_effort =
        context.config.control_kp * pitch_error
        - context.config.control_kd * context.last_gyro[1];
    const double yaw_effort =
        context.config.control_kp * yaw_error
        - context.config.control_kd * context.last_gyro[2];
    const double roll_effort =
        context.config.control_kp * roll_error
        - context.config.control_kd * context.last_gyro[0];
    const bool air_data_valid = fresh(
        input.air_data.valid,
        input.air_data.sample_time_s,
        suite.time_s,
        context.config.air_data_timeout_s
    );
    const double aero_blend = air_data_valid
        ? std::clamp(
            input.air_data.dynamic_pressure_pa / 35'000.0, 0.0, 1.0
        )
        : 0.0;
    output.tvc_pitch_rad = std::clamp(
        pitch_effort * (1.0 - 0.65 * aero_blend),
        -context.config.max_tvc_rad,
        context.config.max_tvc_rad
    );
    output.tvc_yaw_rad = std::clamp(
        yaw_effort * (1.0 - 0.65 * aero_blend),
        -context.config.max_tvc_rad,
        context.config.max_tvc_rad
    );
    output.fin_roll_rad = std::clamp(
        roll_effort * aero_blend,
        -context.config.max_fin_rad,
        context.config.max_fin_rad
    );
    output.fin_pitch_rad = std::clamp(
        pitch_effort * aero_blend,
        -context.config.max_fin_rad,
        context.config.max_fin_rad
    );
    output.fin_yaw_rad = std::clamp(
        yaw_effort * aero_blend,
        -context.config.max_fin_rad,
        context.config.max_fin_rad
    );
}

}  // namespace fsw::internal
