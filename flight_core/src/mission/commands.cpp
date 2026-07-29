
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
    if (!context.navigation.attitude_valid) {
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
    if (context.faults.latched_fault_flags & critical_fault_mask()) {
        inhibits |= FSW_INHIBIT_CRITICAL_FAULT;
    }
    if (
        context.navigation.navigation_status == FSW_NAV_INERTIAL
        || context.faults.active_fault_flags & FSW_FAULT_NAV_UNCERTAINTY
    ) {
        inhibits |= FSW_INHIBIT_NAVIGATION;
    }
    if (
        context.timing.consecutive_overruns
            >= context.config.overrun_abort_count
        || context.faults.active_fault_flags & FSW_FAULT_WATCHDOG
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
    context.control.command_sequence = command.sequence;
    context.control.command_type = command.type;
    context.control.command_result = FSW_COMMAND_REJECTED_INVALID;
    context.control.inhibit_flags = 0;
    context.control.event_flags |= FSW_EVENT_COMMAND_PROCESSED;
    if (
        command.sequence <= context.control.last_command_sequence
        || input.sensors.time_s - command.issue_time_s
            > context.config.command_timeout_s
    ) {
        context.control.command_result = FSW_COMMAND_REJECTED_STALE;
        return;
    }
    context.control.last_command_sequence = command.sequence;
    switch (command.type) {
        case FSW_COMMAND_ARM:
            if (context.mission.mode != FSW_MODE_SAFE) {
                context.control.command_result =
                    FSW_COMMAND_REJECTED_INVALID_STATE;
                return;
            }
            context.control.inhibit_flags = launch_inhibits(
                context, input, voted
            ) & ~FSW_INHIBIT_NAVIGATION;
            if (context.control.inhibit_flags != 0) {
                context.control.command_result = FSW_COMMAND_REJECTED_INHIBITED;
                return;
            }
            context.mission.mode = FSW_MODE_ARMED;
            context.control.command_result = FSW_COMMAND_ACCEPTED;
            return;
        case FSW_COMMAND_DISARM:
            if (context.mission.mode != FSW_MODE_ARMED) {
                context.control.command_result =
                    FSW_COMMAND_REJECTED_INVALID_STATE;
                return;
            }
            context.mission.mode = FSW_MODE_SAFE;
            context.control.command_result = FSW_COMMAND_ACCEPTED;
            return;
        case FSW_COMMAND_LAUNCH:
            if (context.mission.mode != FSW_MODE_ARMED) {
                context.control.command_result =
                    FSW_COMMAND_REJECTED_INVALID_STATE;
                return;
            }
            context.control.inhibit_flags = launch_inhibits(
                context, input, voted
            );
            if (context.control.inhibit_flags != 0) {
                context.control.command_result = FSW_COMMAND_REJECTED_INHIBITED;
                return;
            }
            context.mission.mode = FSW_MODE_IGNITION;
            context.mission.launch_commanded_s = input.sensors.time_s;
            context.control.stage1_ignite_request = true;
            context.control.command_result = FSW_COMMAND_ACCEPTED;
            return;
        case FSW_COMMAND_ABORT:
            if (context.mission.mode == FSW_MODE_LANDED) {
                context.control.command_result =
                    FSW_COMMAND_REJECTED_INVALID_STATE;
                return;
            }
            context.mission.mode = FSW_MODE_ABORT;
            context.control.command_result = FSW_COMMAND_ACCEPTED;
            return;
        case FSW_COMMAND_CLEAR_FAULTS:
            if (context.mission.mode != FSW_MODE_SAFE) {
                context.control.command_result =
                    FSW_COMMAND_REJECTED_INVALID_STATE;
                return;
            }
            context.faults.latched_fault_flags &= critical_fault_mask();
            context.control.command_result = FSW_COMMAND_ACCEPTED;
            return;
        default:
            return;
    }
}

}  // namespace fsw::internal
