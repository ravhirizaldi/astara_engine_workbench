
#include "output/output_builder.hpp"

#include <algorithm>
#include <cmath>

#include "core/context.hpp"
#include "navigation/attitude.hpp"

namespace fsw::internal {

void clear_output(FswOutput& output, int32_t status) {
    output = {};
    output.abi_version = FSW_ABI_VERSION;
    output.struct_size = sizeof(FswOutput);
    output.step_status = status;
}

void populate_output(const Context& context, FswOutput& output) {
    output.abi_version = FSW_ABI_VERSION;
    output.struct_size = sizeof(FswOutput);
    output.output_valid = 1;
    output.step_status = FSW_STATUS_OK;
    output.mode = context.mission.mode;
    output.navigation_status = context.navigation.navigation_status;
    output.stage1_ignite = context.control.stage1_ignite_request;
    output.stage2_ignite = context.control.stage2_ignite_request;
    output.discrete_actuation = context.mission.discrete_actuation;
    output.stage_separate =
        context.mission.discrete_actuation.valid
        && context.mission.discrete_actuation.action
            == FSW_DISCRETE_ACTION_STAGE_SEPARATE;
    output.deploy_drogue =
        context.mission.discrete_actuation.valid
        && context.mission.discrete_actuation.action
            == FSW_DISCRETE_ACTION_DEPLOY_DROGUE;
    output.deploy_main =
        context.mission.discrete_actuation.valid
        && context.mission.discrete_actuation.action
            == FSW_DISCRETE_ACTION_DEPLOY_MAIN;
    output.abort = context.mission.mode == FSW_MODE_ABORT;
    output.attitude_valid = context.navigation.attitude_valid;
    output.command_sequence = context.control.command_sequence;
    output.command_type = context.control.command_type;
    output.command_result = context.control.command_result;
    output.inhibit_flags = context.control.inhibit_flags;
    output.event_flags = context.control.event_flags;
    output.accelerometer_usable_mask =
        context.sensors.accelerometer_usable_mask;
    output.gyroscope_usable_mask = context.sensors.gyroscope_usable_mask;
    output.magnetometer_usable_mask =
        context.sensors.magnetometer_usable_mask;
    output.barometer_usable_mask = context.sensors.barometer_usable_mask;
    output.gnss_usable_mask = context.sensors.gnss_usable_mask;
    output.accelerometer_rejected_mask =
        context.sensors.accelerometer_rejected_mask;
    output.gyroscope_rejected_mask =
        context.sensors.gyroscope_rejected_mask;
    output.magnetometer_rejected_mask =
        context.sensors.magnetometer_rejected_mask;
    output.barometer_rejected_mask = context.sensors.barometer_rejected_mask;
    output.gnss_rejected_mask = context.sensors.gnss_rejected_mask;
    output.disagreement_flags = context.sensors.disagreement_flags;
    output.sensor_status_flags = context.sensors.sensor_status_flags;
    for (int index = 0; index < FSW_MAX_SENSOR_CHANNELS; ++index) {
        output.accelerometer_health_flags[index] =
            context.sensors.accelerometer_health[index].flags;
        output.gyroscope_health_flags[index] =
            context.sensors.gyroscope_health[index].flags;
        output.magnetometer_health_flags[index] =
            context.sensors.magnetometer_health[index].flags;
        output.barometer_health_flags[index] =
            context.sensors.barometer_health[index].flags;
        output.gnss_health_flags[index] =
            context.sensors.gnss_health[index].flags;
        output.accelerometer_age_s[index] =
            context.sensors.accelerometer_health[index].age_s;
        output.gyroscope_age_s[index] =
            context.sensors.gyroscope_health[index].age_s;
        output.magnetometer_age_s[index] =
            context.sensors.magnetometer_health[index].age_s;
        output.barometer_age_s[index] =
            context.sensors.barometer_health[index].age_s;
        output.gnss_age_s[index] = context.sensors.gnss_health[index].age_s;
    }
    output.estimated_altitude_m = context.navigation.altitude;
    output.estimated_vertical_velocity_m_s = context.navigation.vertical_velocity;
    output.altitude_sigma_m = std::sqrt(
        std::max(context.navigation.altitude_variance, 0.0)
    );
    output.vertical_velocity_sigma_m_s = std::sqrt(
        std::max(context.navigation.velocity_variance, 0.0)
    );
    output.barometer_innovation_m = context.navigation.barometer_innovation;
    output.gnss_altitude_innovation_m =
        context.navigation.gnss_altitude_innovation;
    output.gnss_velocity_innovation_m_s =
        context.navigation.gnss_velocity_innovation;
    const auto attitude = relative_attitude(context);
    for (int index = 0; index < 4; ++index) {
        output.estimated_attitude_wxyz[index] = attitude[index];
    }
    for (int index = 0; index < 3; ++index) {
        output.estimated_position_ecef_m[index] =
            context.navigation.position_ecef[index];
        output.estimated_velocity_ecef_m_s[index] =
            context.navigation.velocity_ecef[index];
        output.gyro_bias_rad_s[index] = context.navigation.gyro_bias[index];
        output.attitude_sigma_rad[index] = std::sqrt(
            std::max(context.navigation.attitude_variance[index], 0.0)
        );
    }
    output.active_fault_flags = context.faults.active_fault_flags;
    output.latched_fault_flags = context.faults.latched_fault_flags;
    output.changed_fault_flags = context.faults.changed_fault_flags;
    for (uint32_t index = 0; index < FSW_FAULT_COUNT; ++index) {
        output.fault_occurrence_count[index] =
            context.faults.fault_occurrence_count[index];
    }
    output.highest_fault_severity = context.faults.highest_fault_severity;
    output.previous_execution_time_s =
        context.timing.previous_execution_time_s;
    output.consecutive_overruns = context.timing.consecutive_overruns;
}

}  // namespace fsw::internal
