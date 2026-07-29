#include "core/controller.hpp"

#include <cmath>

#include "control/control.hpp"
#include "math/frames.hpp"
#include "mission/commands.hpp"
#include "mission/mission.hpp"
#include "navigation/navigation.hpp"
#include "output/output_builder.hpp"
#include "sensors/sensor_voting.hpp"
#include "validation/input_validation.hpp"

namespace fsw::internal {

Controller::Controller(const FswConfig& config) : context_(config) {}

void Controller::reset() {
    context_.reset();
}

int32_t Controller::step(const FswInput* input, FswOutput* output) {
    if (output == nullptr) {
        return FSW_STATUS_INVALID_ARGUMENT;
    }
    *output = {};
    output->abi_version = FSW_ABI_VERSION;
    output->struct_size = sizeof(FswOutput);
    output->output_valid = 0;
    output->step_status = FSW_STATUS_INVALID_ARGUMENT;
    if (input == nullptr) {
        return FSW_STATUS_INVALID_ARGUMENT;
    }
    if (
        input->abi_version != FSW_ABI_VERSION
        || input->struct_size != sizeof(FswInput)
    ) {
        output->step_status = FSW_STATUS_ABI_MISMATCH;
        return FSW_STATUS_ABI_MISMATCH;
    }
    const auto& sensor = input->sensors;
    if (
        !valid_input(*input, context_.config)
        || (
            context_.time_initialized
            && sensor.time_s <= context_.last_time_s + kEpsilon
        )
    ) {
        output->step_status = FSW_STATUS_INVALID_INPUT;
        return FSW_STATUS_INVALID_INPUT;
    }

    context_.step_delta_s = context_.time_initialized
        ? sensor.time_s - context_.last_time_s
        : sensor.dt_s;
    context_.input_timing_mismatch = context_.time_initialized
        && std::abs(sensor.dt_s - context_.step_delta_s)
            > context_.config.step_time_tolerance_s;
    context_.event_flags = 0;
    context_.changed_fault_flags = 0;
    context_.stage1_ignite_request = false;
    context_.stage2_ignite_request = false;
    context_.discrete_actuation = {};
    context_.previous_mode = context_.mode;
    const VotedSensors voted = vote_sensors(context_, sensor);
    update_navigation(context_, *input, voted);
    process_command(context_, *input, voted);
    update_mode(context_, *input, voted);
    if (context_.mode != context_.previous_mode) {
        context_.event_flags |= FSW_EVENT_STATE_CHANGED;
    }
    context_.time_initialized = true;
    context_.last_time_s = sensor.time_s;
    populate_output(context_, *output);
    calculate_controls(context_, *input, *output);
    return FSW_STATUS_OK;
}

}  // namespace fsw::internal
