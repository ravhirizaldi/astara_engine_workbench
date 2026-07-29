#include "core/controller.hpp"

#include <cmath>

#include "control/control.hpp"
#include "faults/fault_manager.hpp"
#include "math/frames.hpp"
#include "mission/commands.hpp"
#include "mission/mission.hpp"
#include "navigation/navigation.hpp"
#include "output/output_builder.hpp"
#include "sensors/sensor_voting.hpp"
#include "validation/input_validation.hpp"

namespace fsw::internal {

namespace {

int32_t validate_step_input(const Context& context, const FswInput& input) {
    if (
        input.abi_version != FSW_ABI_VERSION
        || input.struct_size != sizeof(FswInput)
    ) {
        return FSW_STATUS_ABI_MISMATCH;
    }
    if (
        !valid_input(input, context.config)
        || (
            context.timing.initialized
            && input.sensors.time_s
                <= context.timing.last_time_s + kEpsilon
        )
    ) {
        return FSW_STATUS_INVALID_INPUT;
    }
    return FSW_STATUS_OK;
}

void begin_step(Context& context, const FswSensorSuite& sensors) {
    context.timing.step_delta_s = context.timing.initialized
        ? sensors.time_s - context.timing.last_time_s
        : sensors.dt_s;
    context.timing.input_timing_mismatch = context.timing.initialized
        && std::abs(sensors.dt_s - context.timing.step_delta_s)
            > context.config.step_time_tolerance_s;
    context.control.event_flags = 0;
    context.faults.changed_fault_flags = 0;
    context.control.stage1_ignite_request = false;
    context.control.stage2_ignite_request = false;
    context.mission.discrete_actuation = {};
    context.mission.previous_mode = context.mission.mode;
}

void finish_step(Context& context, const FswSensorSuite& sensors) {
    if (context.mission.mode != context.mission.previous_mode) {
        context.control.event_flags |= FSW_EVENT_STATE_CHANGED;
    }
    context.timing.initialized = true;
    context.timing.last_time_s = sensors.time_s;
}

}  // namespace

Controller::Controller(const FswConfig& config) : context_(config) {}

void Controller::reset() {
    context_.reset();
}

int32_t Controller::step(const FswInput& input, FswOutput& output) {
    clear_output(output);
    const int32_t status = validate_step_input(context_, input);
    if (status != FSW_STATUS_OK) {
        output.step_status = status;
        return status;
    }

    begin_step(context_, input.sensors);
    const VotedSensors voted = vote_sensors(context_, input.sensors);
    const bool navigation_disagreement =
        update_navigation(context_, voted);
    set_faults(context_, input, voted, navigation_disagreement);
    process_command(context_, input, voted);
    update_mode(context_, input, voted);
    finish_step(context_, input.sensors);
    calculate_controls(context_, input, output);
    populate_output(context_, output);
    return FSW_STATUS_OK;
}

}  // namespace fsw::internal
