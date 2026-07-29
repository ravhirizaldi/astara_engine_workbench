
#include "validation/input_validation.hpp"

#include <cmath>

#include "math/frames.hpp"
#include "math/vector3.hpp"

namespace fsw::internal {

bool valid_timestamp(double sample_time_s, double suite_time_s) {
    return std::isfinite(sample_time_s)
        && sample_time_s >= 0.0
        && sample_time_s <= suite_time_s + kEpsilon;
}

bool valid_discrete(
    const FswDiscreteSample& sample,
    double time_s
) {
    return !sample.valid || valid_timestamp(sample.sample_time_s, time_s);
}

bool valid_input(const FswInput& input, const FswConfig& config) {
    const auto& sensor = input.sensors;
    if (
        input.abi_version != FSW_ABI_VERSION
        || input.struct_size != sizeof(FswInput)
        || !std::isfinite(sensor.time_s)
        || sensor.time_s < 0.0
        || !std::isfinite(sensor.dt_s)
        || sensor.dt_s < config.min_step_s
        || sensor.dt_s > config.max_step_s
        || sensor.imu_count > FSW_MAX_SENSOR_CHANNELS
        || sensor.magnetometer_count > FSW_MAX_SENSOR_CHANNELS
        || sensor.barometer_count > FSW_MAX_SENSOR_CHANNELS
        || sensor.gnss_count > FSW_MAX_SENSOR_CHANNELS
    ) {
        return false;
    }
    for (uint32_t index = 0; index < sensor.imu_count; ++index) {
        const auto& sample = sensor.imus[index];
        if (
            (sample.accel_valid || sample.gyro_valid)
            && (
                (sample.accel_valid
                    && !finite_vector(sample.acceleration_body_m_s2, 3))
                || (sample.gyro_valid
                    && !finite_vector(sample.gyro_body_rad_s, 3))
                || !valid_timestamp(sample.sample_time_s, sensor.time_s)
            )
        ) {
            return false;
        }
    }
    for (
        uint32_t index = 0;
        index < sensor.magnetometer_count;
        ++index
    ) {
        const auto& sample = sensor.magnetometers[index];
        if (
            sample.valid
            && (
                !finite_vector(sample.magnetic_body, 3)
                || !valid_timestamp(sample.sample_time_s, sensor.time_s)
            )
        ) {
            return false;
        }
    }
    for (uint32_t index = 0; index < sensor.barometer_count; ++index) {
        const auto& sample = sensor.barometers[index];
        if (
            sample.valid
            && (
                !std::isfinite(sample.altitude_m)
                || !valid_timestamp(sample.sample_time_s, sensor.time_s)
            )
        ) {
            return false;
        }
    }
    for (uint32_t index = 0; index < sensor.gnss_count; ++index) {
        const auto& sample = sensor.gnss[index];
        if (
            sample.valid
            && (
                !finite_vector(sample.gnss_position_ecef_m, 3)
                || !finite_vector(sample.gnss_velocity_ecef_m_s, 3)
                || !valid_timestamp(sample.sample_time_s, sensor.time_s)
            )
        ) {
            return false;
        }
    }
    if (
        input.air_data.valid
        && (
            !std::isfinite(input.air_data.dynamic_pressure_pa)
            || input.air_data.dynamic_pressure_pa < 0.0
            || !valid_timestamp(
                input.air_data.sample_time_s, sensor.time_s
            )
        )
    ) {
        return false;
    }
    if (
        input.propulsion.valid
        && (
            !std::isfinite(input.propulsion.health_percent)
            || input.propulsion.health_percent < 0.0
            || input.propulsion.health_percent > 100.0
            || !valid_timestamp(
                input.propulsion.sample_time_s, sensor.time_s
            )
        )
    ) {
        return false;
    }
    if (
        !valid_discrete(input.discretes.stage_separated, sensor.time_s)
        || !valid_discrete(input.discretes.drogue_deployed, sensor.time_s)
        || !valid_discrete(input.discretes.main_deployed, sensor.time_s)
        || (
            input.platform.valid
            && (
                !std::isfinite(input.platform.sample_time_s)
                || !std::isfinite(input.platform.previous_execution_time_s)
                || input.platform.previous_execution_time_s < 0.0
                || !valid_timestamp(
                    input.platform.sample_time_s, sensor.time_s
                )
            )
        )
        || input.command.type < FSW_COMMAND_NONE
        || input.command.type > FSW_COMMAND_CLEAR_FAULTS
        || (
            input.command.type != FSW_COMMAND_NONE
            && (
                input.command.sequence == 0
                || !valid_timestamp(
                    input.command.issue_time_s, sensor.time_s
                )
            )
        )
    ) {
        return false;
    }
    return true;
}

bool fresh(
    bool valid,
    double sample_time_s,
    double time_s,
    double timeout_s
) {
    return valid
        && sample_time_s <= time_s + kEpsilon
        && time_s - sample_time_s <= timeout_s + kEpsilon;
}

}  // namespace fsw::internal
