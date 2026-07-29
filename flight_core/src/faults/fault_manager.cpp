
#include "faults/fault_manager.hpp"

#include <algorithm>
#include <cmath>

#include "core/context.hpp"
#include "sensors/sensor_voting.hpp"
#include "validation/input_validation.hpp"

namespace fsw::internal {

uint32_t critical_fault_mask() {
    return FSW_FAULT_PROPULSION_HEALTH
        | FSW_FAULT_IMU_UNAVAILABLE
        | FSW_FAULT_DEADLINE_OVERRUN
        | FSW_FAULT_WATCHDOG
        | FSW_FAULT_LAUNCH_NOT_CONFIRMED
        | FSW_FAULT_SEPARATION_NOT_CONFIRMED
        | FSW_FAULT_STAGE2_IGNITION
        | FSW_FAULT_DROGUE_NOT_CONFIRMED
        | FSW_FAULT_MAIN_NOT_CONFIRMED
        | FSW_FAULT_INPUT_TIMING;
}

int32_t fault_severity(uint32_t flags) {
    if (
        flags & (
            FSW_FAULT_DROGUE_NOT_CONFIRMED
            | FSW_FAULT_MAIN_NOT_CONFIRMED
        )
    ) {
        return FSW_SEVERITY_MISSION_ENDING;
    }
    if (flags & critical_fault_mask()) {
        return FSW_SEVERITY_CRITICAL;
    }
    if (
        flags & (
            FSW_FAULT_NAV_INERTIAL
            | FSW_FAULT_NAV_DISAGREEMENT
            | FSW_FAULT_NAV_UNCERTAINTY
            | FSW_FAULT_PROPULSION_UNAVAILABLE
        )
    ) {
        return FSW_SEVERITY_DEGRADED;
    }
    return flags == 0 ? FSW_SEVERITY_NONE : FSW_SEVERITY_WARNING;
}

void commit_faults(Context& context, uint32_t flags) {
    const uint32_t rising = flags & ~context.faults.previous_active_fault_flags;
    context.faults.changed_fault_flags =
        flags ^ context.faults.previous_active_fault_flags;
    for (uint32_t index = 0; index < FSW_FAULT_COUNT; ++index) {
        if (rising & (1u << index)) {
            ++context.faults.fault_occurrence_count[index];
        }
    }
    context.faults.active_fault_flags = flags;
    context.faults.latched_fault_flags |= flags;
    context.faults.previous_active_fault_flags = flags;
    context.faults.highest_fault_severity = fault_severity(flags);
    if (context.faults.changed_fault_flags != 0) {
        context.control.event_flags |= FSW_EVENT_FAULT_CHANGED;
    }
}

void set_faults(
    Context& context,
    const FswInput& input,
    const VotedSensors& voted,
    bool navigation_disagreement
) {
    uint32_t flags = 0;
    if (!voted.gnss_valid) {
        flags |= FSW_FAULT_GNSS_UNAVAILABLE;
    }
    if (!voted.barometer_valid) {
        flags |= FSW_FAULT_BAROMETER_UNAVAILABLE;
    }
    if (!voted.imu_valid) {
        flags |= FSW_FAULT_IMU_UNAVAILABLE;
    }
    if (
        context.sensors.disagreement_flags
        & (FSW_DISAGREEMENT_ACCELERATION | FSW_DISAGREEMENT_GYRO)
    ) {
        flags |= FSW_FAULT_IMU_DISAGREEMENT;
    }
    if (
        context.sensors.disagreement_flags
        & FSW_DISAGREEMENT_MAGNETOMETER
    ) {
        flags |= FSW_FAULT_MAGNETOMETER_DISAGREEMENT;
    }
    if (context.sensors.disagreement_flags & FSW_DISAGREEMENT_BAROMETER) {
        flags |= FSW_FAULT_BAROMETER_DISAGREEMENT;
    }
    if (
        context.sensors.disagreement_flags
        & (
            FSW_DISAGREEMENT_GNSS_POSITION
            | FSW_DISAGREEMENT_GNSS_VELOCITY
        )
    ) {
        flags |= FSW_FAULT_GNSS_DISAGREEMENT;
    }
    if (navigation_disagreement) {
        flags |= FSW_FAULT_NAV_DISAGREEMENT;
    }
    if (context.navigation.navigation_status == FSW_NAV_INERTIAL) {
        flags |= FSW_FAULT_NAV_INERTIAL;
    }
    const auto& suite = input.sensors;
    const bool air_data_valid = fresh(
        input.air_data.valid,
        input.air_data.sample_time_s,
        suite.time_s,
        context.config.air_data_timeout_s
    );
    if (!air_data_valid) {
        flags |= FSW_FAULT_AIR_DATA_UNAVAILABLE;
    }
    const bool propulsion_valid = fresh(
        input.propulsion.valid,
        input.propulsion.sample_time_s,
        suite.time_s,
        context.config.propulsion_status_timeout_s
    );
    if (!propulsion_valid) {
        flags |= FSW_FAULT_PROPULSION_UNAVAILABLE;
    } else if (
        input.propulsion.health_percent
        < context.config.propulsion_abort_health_percent
    ) {
        flags |= FSW_FAULT_PROPULSION_HEALTH;
    }
    if (
        std::sqrt(context.navigation.altitude_variance)
            > context.config.max_altitude_sigma_m
        || std::sqrt(context.navigation.velocity_variance)
            > context.config.max_velocity_sigma_m_s
        || std::sqrt(*std::max_element(
            context.navigation.attitude_variance.begin(),
            context.navigation.attitude_variance.end()
        )) > context.config.max_attitude_sigma_rad
    ) {
        flags |= FSW_FAULT_NAV_UNCERTAINTY;
    }
    const bool platform_valid = fresh(
        input.platform.valid,
        input.platform.sample_time_s,
        suite.time_s,
        context.config.platform_status_timeout_s
    );
    if (platform_valid) {
        context.timing.previous_execution_time_s =
            input.platform.previous_execution_time_s;
        const bool overrun = input.platform.deadline_missed
            || input.platform.previous_execution_time_s
                > context.config.loop_deadline_s;
        context.timing.consecutive_overruns = overrun
            ? context.timing.consecutive_overruns + 1
            : 0;
        if (context.timing.consecutive_overruns > 0) {
            flags |= FSW_FAULT_DEADLINE_OVERRUN;
        }
        if (!input.platform.watchdog_healthy) {
            flags |= FSW_FAULT_WATCHDOG;
        }
    } else {
        context.timing.consecutive_overruns = 0;
        flags |= FSW_FAULT_WATCHDOG;
    }
    if (context.timing.input_timing_mismatch) {
        flags |= FSW_FAULT_INPUT_TIMING;
    }
    flags |= context.faults.active_fault_flags & (
        FSW_FAULT_LAUNCH_NOT_CONFIRMED
        | FSW_FAULT_SEPARATION_NOT_CONFIRMED
        | FSW_FAULT_STAGE2_IGNITION
        | FSW_FAULT_DROGUE_NOT_CONFIRMED
        | FSW_FAULT_MAIN_NOT_CONFIRMED
    );
    for (uint32_t index = 0; index < FSW_FAULT_COUNT; ++index) {
        const uint32_t bit = 1u << index;
        if (flags & bit) {
            context.faults.fault_healthy_time_s[index] = 0.0;
        } else if (context.faults.active_fault_flags & bit) {
            context.faults.fault_healthy_time_s[index] += context.timing.step_delta_s;
            if (
                context.faults.fault_healthy_time_s[index]
                < context.config.fault_recovery_persistence_s
            ) {
                flags |= bit;
            }
        } else {
            context.faults.fault_healthy_time_s[index] = 0.0;
        }
    }
    commit_faults(context, flags);
}

void raise_fault(Context& context, uint32_t fault) {
    commit_faults(context, context.faults.active_fault_flags | fault);
}

}  // namespace fsw::internal
