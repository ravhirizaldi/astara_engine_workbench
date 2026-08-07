
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
        !context.navigation.attitude_valid
        || (
            context.mission.mode != FSW_MODE_BOOST_1
            && context.mission.mode != FSW_MODE_BOOST_2
            && context.mission.mode != FSW_MODE_ORBIT_INSERTION
            && !(
                context.config.orbit_enabled
                && context.mission.mode == FSW_MODE_COAST
            )
        )
    ) {
        return;
    }
    const double guidance_time_s = context.mission.ignition_confirmed_s >= 0.0
        ? std::max(suite.time_s - context.mission.ignition_confirmed_s, 0.0)
        : 0.0;
    const auto target = guidance_at(context.config, guidance_time_s);
    const auto angles = euler(relative_attitude(context));
    double roll_error = -angles[0];
    double pitch_error = target.pitch_rad - angles[1];
    double yaw_error = wrap_angle(
        target.azimuth_rad
        - context.config.guidance[0].azimuth_rad
        - angles[2]
    );
    if (
        context.config.orbit_enabled
        && (
            context.mission.mode == FSW_MODE_COAST
            || context.mission.mode == FSW_MODE_ORBIT_INSERTION
        )
    ) {
        const auto& position = context.navigation.position_ecef;
        const auto& velocity = context.navigation.velocity_ecef;
        const std::array<double, 3> inertial_velocity{
            velocity[0] - kEarthRotationRadS * position[1],
            velocity[1] + kEarthRotationRadS * position[0],
            velocity[2],
        };
        const auto prograde_body = rotate(
            conjugate(context.navigation.attitude),
            inertial_velocity
        );
        pitch_error = -std::atan2(prograde_body[2], prograde_body[0]);
        yaw_error = std::atan2(prograde_body[1], prograde_body[0]);
    }
    const double pitch_effort =
        context.config.control_kp * pitch_error
        - context.config.control_kd * context.navigation.last_gyro[1];
    const double yaw_effort =
        context.config.control_kp * yaw_error
        - context.config.control_kd * context.navigation.last_gyro[2];
    const double roll_effort =
        context.config.control_kp * roll_error
        - context.config.control_kd * context.navigation.last_gyro[0];
    const bool air_data_valid = fresh(
        input.air_data.valid,
        input.air_data.sample_time_s,
        suite.time_s,
        context.config.air_data_timeout_s
    );
    const double aero_blend = air_data_valid
        ? std::clamp(
            input.air_data.dynamic_pressure_pa
                / context.config.aero_reference_dynamic_pressure_pa,
            0.0,
            1.0
        )
        : 0.0;
    output.tvc_pitch_rad = std::clamp(
        pitch_effort
            * (
                1.0
                - context.config.aero_high_q_authority_scale * aero_blend
            ),
        -context.config.max_tvc_rad,
        context.config.max_tvc_rad
    );
    output.tvc_yaw_rad = std::clamp(
        yaw_effort
            * (
                1.0
                - context.config.aero_high_q_authority_scale * aero_blend
            ),
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
