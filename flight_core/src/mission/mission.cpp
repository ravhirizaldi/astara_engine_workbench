
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

void request_discrete_actuation(
    Context& context,
    int32_t action
) {
    context.discrete_actuation.sequence =
        context.next_discrete_actuation_sequence++;
    context.discrete_actuation.action = action;
    context.discrete_actuation.pulse_duration_s =
        context.step_delta_s;
    context.discrete_actuation.valid = 1;
    context.event_flags |= FSW_EVENT_DISCRETE_ACTUATION;
}

void update_integrated_mode(
    Context& context,
    const FswInput& input,
    const VotedSensors& voted
) {
    const auto& suite = input.sensors;
    switch (context.mode) {
        case FSW_MODE_SAFE:
        case FSW_MODE_ARMED:
            break;
        case FSW_MODE_IGNITION:
            if (
                context.attitude_valid
                && propulsion_fresh(context, input)
                && input.propulsion.running
                && persisted(
                    voted.acceleration[0] > 2.0,
                    context.step_delta_s,
                    0.05,
                    context.launch_evidence_s
                )
            ) {
                context.ignition_confirmed_s = suite.time_s;
                context.mode = FSW_MODE_BOOST_1;
            } else if (
                context.launch_commanded_s >= 0.0
                && suite.time_s - context.launch_commanded_s
                    > context.config.launch_confirm_timeout_s
            ) {
                raise_fault(context, FSW_FAULT_LAUNCH_NOT_CONFIRMED);
                context.mode = FSW_MODE_ABORT;
            }
            break;
        case FSW_MODE_BOOST_1:
            if (
                context.ignition_confirmed_s >= 0.0
                && suite.time_s - context.ignition_confirmed_s
                    >= context.config.stage1_burn_s
                && persisted(
                    context.attitude_valid
                        && voted.acceleration[0] < 2.0,
                    context.step_delta_s,
                    0.05,
                    context.burnout_evidence_s
                )
            ) {
                context.burnout_detected_s = suite.time_s;
                context.mode = FSW_MODE_SEPARATION;
            }
            break;
        case FSW_MODE_SEPARATION:
            if (
                context.separation_commanded_s < 0.0
                && context.burnout_detected_s >= 0.0
                && suite.time_s - context.burnout_detected_s
                    >= context.config.separation_delay_s
            ) {
                context.separation_commanded_s = suite.time_s;
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
                context.separation_confirmed_s = suite.time_s;
                context.mode = FSW_MODE_INTERSTAGE;
            } else if (
                context.separation_commanded_s >= 0.0
                && suite.time_s - context.separation_commanded_s
                    > context.config.separation_confirm_timeout_s
            ) {
                raise_fault(
                    context, FSW_FAULT_SEPARATION_NOT_CONFIRMED
                );
                context.mode = FSW_MODE_ABORT;
            }
            break;
        case FSW_MODE_INTERSTAGE:
            if (
                context.stage2_ignition_commanded_s < 0.0
                && context.separation_confirmed_s >= 0.0
                && suite.time_s - context.separation_confirmed_s
                    >= context.config.stage2_ignition_delay_s
            ) {
                context.inhibit_flags = launch_inhibits(
                    context, input, voted
                );
                if (
                    !discrete_asserted(
                        input.discretes.stage_separated,
                        suite.time_s,
                        context.config.discrete_feedback_timeout_s
                    )
                ) {
                    context.inhibit_flags |= FSW_INHIBIT_SEPARATION;
                }
                if (context.inhibit_flags == 0) {
                    context.stage2_ignite_request = true;
                    context.stage2_ignition_commanded_s =
                        suite.time_s;
                }
            }
            if (
                context.stage2_ignition_commanded_s >= 0.0
                && propulsion_fresh(context, input)
                && input.propulsion.running
            ) {
                context.stage2_ignition_confirmed_s = suite.time_s;
                context.mode = FSW_MODE_BOOST_2;
            } else if (
                context.stage2_ignition_commanded_s >= 0.0
                && suite.time_s
                    - context.stage2_ignition_commanded_s
                    > context.config.stage2_ignition_timeout_s
            ) {
                raise_fault(context, FSW_FAULT_STAGE2_IGNITION);
                context.mode = FSW_MODE_ABORT;
            }
            break;
        case FSW_MODE_BOOST_2:
            if (
                context.stage2_ignition_confirmed_s >= 0.0
                && suite.time_s
                    >= context.stage2_ignition_confirmed_s
                        + context.config.stage2_burn_s
            ) {
                context.mode = FSW_MODE_COAST;
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
    const bool powered = context.mode == FSW_MODE_IGNITION
        || context.mode == FSW_MODE_BOOST_1
        || context.mode == FSW_MODE_BOOST_2;
    if (persisted(
        powered && !context.attitude_valid,
        context.step_delta_s,
        context.config.imu_loss_abort_delay_s,
        context.imu_loss_evidence_s
    )) {
        context.mode = FSW_MODE_ABORT;
        return;
    }
    if (persisted(
        powered
            && (
                !propulsion_fresh(context, input)
                || input.propulsion.health_percent
                    < context.config.propulsion_abort_health_percent
            ),
        context.step_delta_s,
        context.config.propulsion_abort_persistence_s,
        context.propulsion_loss_evidence_s
    )) {
        context.mode = FSW_MODE_ABORT;
        return;
    }
    if (
        powered
        && (
            context.active_fault_flags
                & (
                    FSW_FAULT_WATCHDOG
                    | FSW_FAULT_LAUNCH_NOT_CONFIRMED
                    | FSW_FAULT_STAGE2_IGNITION
                )
            || context.consecutive_overruns
                >= context.config.overrun_abort_count
        )
    ) {
        context.mode = FSW_MODE_ABORT;
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
                && context.mode < FSW_MODE_COAST
            ) {
                context.mode = FSW_MODE_COAST;
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
        context.navigation_status != FSW_NAV_INERTIAL
        && (
            context.disagreement_flags
            & FSW_DISAGREEMENT_CROSS_ALTITUDE
        ) == 0;
    if (
        altitude_aided
        && context.mode >= FSW_MODE_COAST
        && context.mode < FSW_MODE_APOGEE
        && context.altitude > 100.0
        && persisted(
            context.vertical_velocity < -0.5,
            context.step_delta_s,
            0.20,
            context.apogee_evidence_s
        )
    ) {
        context.mode = FSW_MODE_APOGEE;
        context.apogee_seen = true;
    }
    if (context.apogee_seen && !context.drogue_deployed) {
        context.mode = FSW_MODE_DROGUE;
        if (context.drogue_commanded_s < 0.0) {
            context.drogue_commanded_s = suite.time_s;
            request_discrete_actuation(
                context, FSW_DISCRETE_ACTION_DEPLOY_DROGUE
            );
        }
        context.drogue_deployed = discrete_asserted(
            input.discretes.drogue_deployed,
            suite.time_s,
            context.config.discrete_feedback_timeout_s
        );
        if (
            !context.drogue_deployed
            && suite.time_s - context.drogue_commanded_s
                > context.config.drogue_confirm_timeout_s
        ) {
            raise_fault(context, FSW_FAULT_DROGUE_NOT_CONFIRMED);
        }
    }
    if (
        altitude_aided
        && context.drogue_deployed
        && !context.main_deployed
        && context.altitude <= context.config.main_deploy_altitude_m
    ) {
        context.mode = FSW_MODE_MAIN;
        if (context.main_commanded_s < 0.0) {
            context.main_commanded_s = suite.time_s;
            request_discrete_actuation(
                context, FSW_DISCRETE_ACTION_DEPLOY_MAIN
            );
        }
        context.main_deployed = discrete_asserted(
            input.discretes.main_deployed,
            suite.time_s,
            context.config.discrete_feedback_timeout_s
        );
        if (
            !context.main_deployed
            && suite.time_s - context.main_commanded_s
                > context.config.main_confirm_timeout_s
        ) {
            raise_fault(context, FSW_FAULT_MAIN_NOT_CONFIRMED);
        }
    }
    if (
        altitude_aided
        && context.main_deployed
        && persisted(
            context.altitude <= 2.0
                && std::abs(context.vertical_velocity) < 15.0,
            context.step_delta_s,
            1.0,
            context.landing_evidence_s
        )
    ) {
        context.mode = FSW_MODE_LANDED;
    }
}

}  // namespace fsw::internal
