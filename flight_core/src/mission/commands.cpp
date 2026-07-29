
#include "mission/commands.hpp"

#include "core/context.hpp"
#include "faults/fault_manager.hpp"
#include "sensors/sensor_voting.hpp"
#include "validation/input_validation.hpp"

namespace fsw::internal {

uint32_t launch_inhibits(
    const Context& context,
    const FswInput& input,
    const VotedSensors& voted
) {
    uint32_t inhibits = 0;
    if (!voted.imu_valid) {
        inhibits |= FSW_INHIBIT_IMU;
    }
    if (!context.attitude_valid) {
        inhibits |= FSW_INHIBIT_ATTITUDE;
    }
    const bool propulsion_ready = fresh(
        input.propulsion.valid,
        input.propulsion.sample_time_s,
        input.sensors.time_s,
        context.config.propulsion_status_timeout_s
    ) && input.propulsion.ready
        && input.propulsion.health_percent
            >= context.config.propulsion_abort_health_percent;
    if (!propulsion_ready) {
        inhibits |= FSW_INHIBIT_PROPULSION;
    }
    if (context.latched_fault_flags & critical_fault_mask()) {
        inhibits |= FSW_INHIBIT_CRITICAL_FAULT;
    }
    if (
        context.navigation_status == FSW_NAV_INERTIAL
        || context.active_fault_flags & FSW_FAULT_NAV_UNCERTAINTY
    ) {
        inhibits |= FSW_INHIBIT_NAVIGATION;
    }
    if (
        context.consecutive_overruns
            >= context.config.overrun_abort_count
        || context.active_fault_flags & FSW_FAULT_WATCHDOG
    ) {
        inhibits |= FSW_INHIBIT_TIMING;
    }
    return inhibits;
}

void process_command(
    Context& context,
    const FswInput& input,
    const VotedSensors& voted
) {
    const auto& command = input.command;
    if (command.type == FSW_COMMAND_NONE) {
        return;
    }
    context.command_sequence = command.sequence;
    context.command_type = command.type;
    context.command_result = FSW_COMMAND_REJECTED_INVALID;
    context.inhibit_flags = 0;
    context.event_flags |= FSW_EVENT_COMMAND_PROCESSED;
    if (
        command.sequence <= context.last_command_sequence
        || input.sensors.time_s - command.issue_time_s
            > context.config.command_timeout_s
    ) {
        context.command_result = FSW_COMMAND_REJECTED_STALE;
        return;
    }
    context.last_command_sequence = command.sequence;
    switch (command.type) {
        case FSW_COMMAND_ARM:
            if (context.mode != FSW_MODE_SAFE) {
                context.command_result =
                    FSW_COMMAND_REJECTED_INVALID_STATE;
                return;
            }
            context.inhibit_flags = launch_inhibits(
                context, input, voted
            ) & ~FSW_INHIBIT_NAVIGATION;
            if (context.inhibit_flags != 0) {
                context.command_result = FSW_COMMAND_REJECTED_INHIBITED;
                return;
            }
            context.mode = FSW_MODE_ARMED;
            context.command_result = FSW_COMMAND_ACCEPTED;
            return;
        case FSW_COMMAND_DISARM:
            if (context.mode != FSW_MODE_ARMED) {
                context.command_result =
                    FSW_COMMAND_REJECTED_INVALID_STATE;
                return;
            }
            context.mode = FSW_MODE_SAFE;
            context.command_result = FSW_COMMAND_ACCEPTED;
            return;
        case FSW_COMMAND_LAUNCH:
            if (context.mode != FSW_MODE_ARMED) {
                context.command_result =
                    FSW_COMMAND_REJECTED_INVALID_STATE;
                return;
            }
            context.inhibit_flags = launch_inhibits(
                context, input, voted
            );
            if (context.inhibit_flags != 0) {
                context.command_result = FSW_COMMAND_REJECTED_INHIBITED;
                return;
            }
            context.mode = FSW_MODE_IGNITION;
            context.launch_commanded_s = input.sensors.time_s;
            context.stage1_ignite_request = true;
            context.command_result = FSW_COMMAND_ACCEPTED;
            return;
        case FSW_COMMAND_ABORT:
            if (context.mode == FSW_MODE_LANDED) {
                context.command_result =
                    FSW_COMMAND_REJECTED_INVALID_STATE;
                return;
            }
            context.mode = FSW_MODE_ABORT;
            context.command_result = FSW_COMMAND_ACCEPTED;
            return;
        case FSW_COMMAND_CLEAR_FAULTS:
            if (context.mode != FSW_MODE_SAFE) {
                context.command_result =
                    FSW_COMMAND_REJECTED_INVALID_STATE;
                return;
            }
            context.latched_fault_flags &= critical_fault_mask();
            context.command_result = FSW_COMMAND_ACCEPTED;
            return;
        default:
            return;
    }
}

}  // namespace fsw::internal
