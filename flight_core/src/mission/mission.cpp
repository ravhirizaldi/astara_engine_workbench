
#include "mission/mission.hpp"

#include <algorithm>
#include <cmath>

#include "core/context.hpp"
#include "faults/fault_manager.hpp"
#include "math/frames.hpp"
#include "mission/commands.hpp"
#include "sensors/sensor_voting.hpp"
#include "validation/input_validation.hpp"

namespace fsw::internal {

bool persisted(
    bool condition,
    double dt_s,
    double required_s,
    double& evidence_s
) {
    evidence_s = condition ? evidence_s + std::max(dt_s, 0.0) : 0.0;
    return evidence_s + kEpsilon >= required_s;
}

bool discrete_asserted(
    const FswDiscreteSample& sample,
    double time_s,
    double timeout_s
) {
    return fresh(
        sample.valid, sample.sample_time_s, time_s, timeout_s
    ) && sample.asserted;
}

bool propulsion_fresh(const Context& context, const FswInput& input) {
    return fresh(
        input.propulsion.valid,
        input.propulsion.sample_time_s,
        input.sensors.time_s,
        context.config.propulsion_status_timeout_s
    );
}

double orbit_radial_velocity(const Context& context) {
    const auto& position = context.navigation.position_ecef;
    const auto& velocity = context.navigation.velocity_ecef;
    const double radius = std::sqrt(
        position[0] * position[0]
        + position[1] * position[1]
        + position[2] * position[2]
    );
    return radius > 0.0
        ? (
            position[0] * velocity[0]
            + position[1] * velocity[1]
            + position[2] * velocity[2]
        ) / radius
        : 0.0;
}

double orbit_inertial_speed(const Context& context) {
    const auto& position = context.navigation.position_ecef;
    const auto& velocity = context.navigation.velocity_ecef;
    const double inertial[] = {
        velocity[0] - kEarthRotationRadS * position[1],
        velocity[1] + kEarthRotationRadS * position[0],
        velocity[2],
    };
    return std::sqrt(
        inertial[0] * inertial[0]
        + inertial[1] * inertial[1]
        + inertial[2] * inertial[2]
    );
}

void request_discrete_actuation(
    Context& context,
    int32_t action
) {
    context.mission.discrete_actuation.sequence =
        context.mission.next_discrete_actuation_sequence++;
    context.mission.discrete_actuation.action = action;
    context.mission.discrete_actuation.pulse_duration_s =
        context.timing.step_delta_s;
    context.mission.discrete_actuation.valid = 1;
    context.control.event_flags |= FSW_EVENT_DISCRETE_ACTUATION;
}

void update_integrated_mode(
    Context& context,
    const FswInput& input,
    const VotedSensors& voted
) {
    const auto& suite = input.sensors;
    switch (context.mission.mode) {
        case FSW_MODE_SAFE:
        case FSW_MODE_ARMED:
            break;
        case FSW_MODE_IGNITION:
            if (
                voted.accelerometer_valid
                && context.navigation.attitude_valid
                && propulsion_fresh(context, input)
                && input.propulsion.running
                && persisted(
                    voted.acceleration[0]
                        > context.config.launch_acceleration_threshold_m_s2,
                    context.timing.step_delta_s,
                    context.config.launch_persistence_s,
                    context.mission.launch_evidence_s
                )
            ) {
                context.mission.ignition_confirmed_s = suite.time_s;
                context.mission.mode = FSW_MODE_BOOST_1;
            } else if (
                context.mission.launch_commanded_s >= 0.0
                && suite.time_s - context.mission.launch_commanded_s
                    > context.config.launch_confirm_timeout_s
            ) {
                raise_fault(context, FSW_FAULT_LAUNCH_NOT_CONFIRMED);
                context.mission.mode = FSW_MODE_ABORT;
            }
            break;
        case FSW_MODE_BOOST_1:
            if (
                context.mission.ignition_confirmed_s >= 0.0
                && suite.time_s - context.mission.ignition_confirmed_s
                    >= context.config.stage1_burn_s
                && persisted(
                    voted.accelerometer_valid
                        && context.navigation.attitude_valid
                        && voted.acceleration[0]
                            < context.config
                                .burnout_acceleration_threshold_m_s2,
                    context.timing.step_delta_s,
                    context.config.burnout_persistence_s,
                    context.mission.burnout_evidence_s
                )
            ) {
                context.mission.burnout_detected_s = suite.time_s;
                context.mission.mode = FSW_MODE_SEPARATION;
            }
            break;
        case FSW_MODE_SEPARATION:
            if (
                context.mission.separation_commanded_s < 0.0
                && context.mission.burnout_detected_s >= 0.0
                && suite.time_s - context.mission.burnout_detected_s
                    >= context.config.separation_delay_s
            ) {
                context.mission.separation_commanded_s = suite.time_s;
                request_discrete_actuation(
                    context, FSW_DISCRETE_ACTION_STAGE_SEPARATE
                );
            }
            if (
                discrete_asserted(
                    input.discretes.stage_separated,
                    suite.time_s,
                    context.config.discrete_feedback_timeout_s
                )
            ) {
                context.mission.separation_confirmed_s = suite.time_s;
                context.mission.mode = FSW_MODE_INTERSTAGE;
            } else if (
                context.mission.separation_commanded_s >= 0.0
                && suite.time_s - context.mission.separation_commanded_s
                    > context.config.separation_confirm_timeout_s
            ) {
                raise_fault(
                    context, FSW_FAULT_SEPARATION_NOT_CONFIRMED
                );
                context.mission.mode = FSW_MODE_ABORT;
            }
            break;
        case FSW_MODE_INTERSTAGE:
            if (
                context.mission.separation_confirmed_s < 0.0
                && discrete_asserted(
                    input.discretes.stage_separated,
                    suite.time_s,
                    context.config.discrete_feedback_timeout_s
                )
            ) {
                context.mission.separation_confirmed_s = suite.time_s;
            }
            if (
                context.mission.stage2_ignition_commanded_s < 0.0
                && context.mission.separation_confirmed_s >= 0.0
                && suite.time_s - context.mission.separation_confirmed_s
                    >= context.config.stage2_ignition_delay_s
            ) {
                context.control.inhibit_flags = launch_inhibits(
                    context, input, voted
                );
                if (
                    !discrete_asserted(
                        input.discretes.stage_separated,
                        suite.time_s,
                        context.config.discrete_feedback_timeout_s
                    )
                ) {
                    context.control.inhibit_flags |= FSW_INHIBIT_SEPARATION;
                }
                if (context.control.inhibit_flags == 0) {
                    context.control.stage2_ignite_request = true;
                    context.mission.stage2_ignition_commanded_s =
                        suite.time_s;
                }
            }
            if (
                context.mission.stage2_ignition_commanded_s >= 0.0
                && propulsion_fresh(context, input)
                && input.propulsion.running
            ) {
                context.mission.stage2_ignition_confirmed_s = suite.time_s;
                context.mission.mode = FSW_MODE_BOOST_2;
            } else if (
                context.mission.stage2_ignition_commanded_s >= 0.0
                && suite.time_s
                    - context.mission.stage2_ignition_commanded_s
                    > context.config.stage2_ignition_timeout_s
            ) {
                raise_fault(context, FSW_FAULT_STAGE2_IGNITION);
                context.mission.mode = FSW_MODE_ABORT;
            }
            break;
        case FSW_MODE_BOOST_2:
            if (
                context.mission.stage2_ignition_confirmed_s >= 0.0
                && suite.time_s
                    >= context.mission.stage2_ignition_confirmed_s
                        + context.config.stage2_burn_s
            ) {
                context.control.stage2_shutdown_request = true;
                context.mission.mode = FSW_MODE_COAST;
            }
            break;
        case FSW_MODE_COAST:
            if (
                context.config.orbit_enabled
                && context.config.body_role != FSW_BODY_CORE
                && std::abs(
                    context.navigation.altitude
                    - context.config.orbit_target_altitude_m
                ) <= context.config.orbit_altitude_tolerance_m
                && std::abs(orbit_radial_velocity(context))
                    <= context.config.orbit_radial_velocity_tolerance_m_s
            ) {
                context.control.stage2_ignite_request = true;
                context.mission.circularization_ignition_s = suite.time_s;
                context.mission.mode = FSW_MODE_ORBIT_INSERTION;
            }
            break;
        case FSW_MODE_ORBIT_INSERTION: {
            const auto& position = context.navigation.position_ecef;
            const double radius = std::sqrt(
                position[0] * position[0]
                + position[1] * position[1]
                + position[2] * position[2]
            );
            const double circular_speed = std::sqrt(kEarthMuM3S2 / radius);
            if (
                propulsion_fresh(context, input)
                && input.propulsion.running
                && orbit_inertial_speed(context)
                    >= circular_speed
                        + context.config.orbit_cutoff_speed_margin_m_s
            ) {
                context.control.stage2_shutdown_request = true;
                context.mission.orbit_achieved_s = suite.time_s;
                context.mission.mode = FSW_MODE_ORBIT;
            } else if (
                context.mission.circularization_ignition_s >= 0.0
                && suite.time_s
                    - context.mission.circularization_ignition_s
                    > context.config.circularization_max_burn_s
            ) {
                context.control.stage2_shutdown_request = true;
                context.mission.mode = FSW_MODE_ABORT;
            }
            break;
        }
        case FSW_MODE_ORBIT:
            if (
                !context.mission.payload_deploy_requested
                && context.mission.orbit_achieved_s >= 0.0
                && suite.time_s - context.mission.orbit_achieved_s
                    >= context.config.payload_deploy_delay_s
            ) {
                context.mission.payload_deploy_requested = true;
                request_discrete_actuation(
                    context, FSW_DISCRETE_ACTION_DEPLOY_PAYLOAD
                );
                context.mission.mode = FSW_MODE_PAYLOAD_DEPLOYED;
            }
            break;
        default:
            break;
    }
}

void update_mode(
    Context& context,
    const FswInput& input,
    const VotedSensors& voted
) {
    const auto& suite = input.sensors;
    const bool powered = context.mission.mode == FSW_MODE_IGNITION
        || context.mission.mode == FSW_MODE_BOOST_1
        || context.mission.mode == FSW_MODE_BOOST_2
        || context.mission.mode == FSW_MODE_ORBIT_INSERTION;
    if (persisted(
        powered
            && (
                !voted.imu_valid
                || !context.navigation.attitude_valid
            ),
        context.timing.step_delta_s,
        context.config.imu_loss_abort_delay_s,
        context.mission.imu_loss_evidence_s
    )) {
        context.mission.mode = FSW_MODE_ABORT;
        return;
    }
    if (persisted(
        powered
            && (
                !propulsion_fresh(context, input)
                || input.propulsion.health_percent
                    < context.config.propulsion_abort_health_percent
            ),
        context.timing.step_delta_s,
        context.config.propulsion_abort_persistence_s,
        context.mission.propulsion_loss_evidence_s
    )) {
        context.mission.mode = FSW_MODE_ABORT;
        return;
    }
    if (
        powered
        && (
            context.faults.active_fault_flags
                & (
                    FSW_FAULT_WATCHDOG
                    | FSW_FAULT_LAUNCH_NOT_CONFIRMED
                    | FSW_FAULT_STAGE2_IGNITION
                )
            || context.timing.consecutive_overruns
                >= context.config.overrun_abort_count
        )
    ) {
        context.mission.mode = FSW_MODE_ABORT;
        return;
    }

    switch (context.config.body_role) {
        case FSW_BODY_CORE:
            if (
                discrete_asserted(
                    input.discretes.stage_separated,
                    suite.time_s,
                    context.config.discrete_feedback_timeout_s
                )
                && context.mission.mode < FSW_MODE_COAST
            ) {
                context.mission.mode = FSW_MODE_COAST;
            }
            break;
        case FSW_BODY_UPPER:
            update_integrated_mode(context, input, voted);
            break;
        case FSW_BODY_INTEGRATED:
            update_integrated_mode(context, input, voted);
            break;
        default:
            break;
    }

    const bool altitude_aided =
        context.navigation.navigation_status != FSW_NAV_INERTIAL
        && (
            context.sensors.disagreement_flags
            & FSW_DISAGREEMENT_CROSS_ALTITUDE
        ) == 0;
    const bool recovery_enabled = !(
        context.config.orbit_enabled
        && context.config.body_role != FSW_BODY_CORE
    );
    if (
        recovery_enabled
        && altitude_aided
        && context.mission.mode >= FSW_MODE_COAST
        && context.mission.mode < FSW_MODE_APOGEE
        && context.navigation.altitude
            > context.config.apogee_min_altitude_m
        && persisted(
            context.navigation.vertical_velocity
                < context.config.apogee_descent_velocity_m_s,
            context.timing.step_delta_s,
            context.config.apogee_persistence_s,
            context.mission.apogee_evidence_s
        )
    ) {
        context.mission.mode = FSW_MODE_APOGEE;
        context.mission.apogee_seen = true;
    }
    if (context.mission.apogee_seen && !context.mission.drogue_deployed) {
        context.mission.mode = FSW_MODE_DROGUE;
        if (context.mission.drogue_commanded_s < 0.0) {
            context.mission.drogue_commanded_s = suite.time_s;
            request_discrete_actuation(
                context, FSW_DISCRETE_ACTION_DEPLOY_DROGUE
            );
        }
        context.mission.drogue_deployed =
            context.mission.drogue_deployed || discrete_asserted(
            input.discretes.drogue_deployed,
            suite.time_s,
            context.config.discrete_feedback_timeout_s
        );
        if (
            !context.mission.drogue_deployed
            && suite.time_s - context.mission.drogue_commanded_s
                > context.config.drogue_confirm_timeout_s
        ) {
            raise_fault(context, FSW_FAULT_DROGUE_NOT_CONFIRMED);
        }
    }
    if (
        altitude_aided
        && context.mission.drogue_deployed
        && !context.mission.main_deployed
        && context.navigation.altitude <= context.config.main_deploy_altitude_m
    ) {
        context.mission.mode = FSW_MODE_MAIN;
        if (context.mission.main_commanded_s < 0.0) {
            context.mission.main_commanded_s = suite.time_s;
            request_discrete_actuation(
                context, FSW_DISCRETE_ACTION_DEPLOY_MAIN
            );
        }
        context.mission.main_deployed =
            context.mission.main_deployed || discrete_asserted(
            input.discretes.main_deployed,
            suite.time_s,
            context.config.discrete_feedback_timeout_s
        );
        if (
            !context.mission.main_deployed
            && suite.time_s - context.mission.main_commanded_s
                > context.config.main_confirm_timeout_s
        ) {
            raise_fault(context, FSW_FAULT_MAIN_NOT_CONFIRMED);
        }
    }
    if (
        altitude_aided
        && context.mission.main_deployed
        && persisted(
            context.navigation.altitude <= context.config.landing_altitude_m
                && std::abs(context.navigation.vertical_velocity)
                    < context.config.landing_speed_m_s,
            context.timing.step_delta_s,
            context.config.landing_persistence_s,
            context.mission.landing_evidence_s
        )
    ) {
        context.mission.mode = FSW_MODE_LANDED;
    }
}

}  // namespace fsw::internal
