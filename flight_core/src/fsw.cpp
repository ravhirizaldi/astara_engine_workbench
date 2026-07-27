#include "fsw.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <new>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-12;

double clamp(double value, double low, double high) {
    return std::max(low, std::min(value, high));
}

bool finite_vector(const double* values, int size) {
    for (int index = 0; index < size; ++index) {
        if (!std::isfinite(values[index])) {
            return false;
        }
    }
    return true;
}

double vector_distance(const double* left, const double* right, int size) {
    double squared = 0.0;
    for (int index = 0; index < size; ++index) {
        const double delta = left[index] - right[index];
        squared += delta * delta;
    }
    return std::sqrt(squared);
}

double vector_norm(const double* values, int size) {
    double squared = 0.0;
    for (int index = 0; index < size; ++index) {
        squared += values[index] * values[index];
    }
    return std::sqrt(squared);
}

double median3(double first, double second, double third) {
    return first + second + third
        - std::min(first, std::min(second, third))
        - std::max(first, std::max(second, third));
}

uint32_t bit_count(uint32_t mask) {
    uint32_t count = 0;
    while (mask != 0) {
        count += mask & 1u;
        mask >>= 1u;
    }
    return count;
}

double wrap_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

bool valid_config(const FswConfig& config) {
    const double finite_values[] = {
        config.stage1_burn_s,
        config.separation_delay_s,
        config.stage2_ignition_delay_s,
        config.stage2_burn_s,
        config.main_deploy_altitude_m,
        config.max_tvc_rad,
        config.max_fin_rad,
        config.control_kp,
        config.control_kd,
        config.imu_timeout_s,
        config.barometer_timeout_s,
        config.gnss_timeout_s,
        config.acceleration_disagreement_m_s2,
        config.gyro_disagreement_rad_s,
        config.barometer_disagreement_m,
        config.gnss_position_disagreement_m,
        config.gnss_velocity_disagreement_m_s,
        config.cross_altitude_disagreement_m,
        config.imu_loss_abort_delay_s,
        config.gyro_bias_time_constant_s,
        config.stationary_gyro_threshold_rad_s,
        config.altitude_filter_tau_s,
        config.velocity_filter_tau_s,
        config.magnetic_disagreement,
        config.command_timeout_s,
        config.launch_confirm_timeout_s,
        config.separation_confirm_timeout_s,
        config.stage2_ignition_timeout_s,
        config.drogue_confirm_timeout_s,
        config.main_confirm_timeout_s,
        config.fault_recovery_persistence_s,
        config.min_step_s,
        config.max_step_s,
        config.loop_deadline_s,
        config.propulsion_abort_health_percent,
        config.propulsion_abort_persistence_s,
        config.max_acceleration_m_s2,
        config.max_gyro_rad_s,
        config.min_magnetic_norm,
        config.max_magnetic_norm,
        config.min_barometer_altitude_m,
        config.max_barometer_altitude_m,
        config.max_barometer_rate_m_s,
        config.min_gnss_radius_m,
        config.max_gnss_radius_m,
        config.max_gnss_speed_m_s,
        config.max_gnss_velocity_rate_m_s2,
        config.accelerometer_process_sigma_m_s2,
        config.gyro_process_sigma_rad_s,
        config.barometer_sigma_m,
        config.gnss_altitude_sigma_m,
        config.gnss_velocity_sigma_m_s,
        config.max_altitude_sigma_m,
        config.max_velocity_sigma_m_s,
        config.max_attitude_sigma_rad,
    };
    if (
        config.abi_version != FSW_ABI_VERSION
        || config.struct_size != sizeof(FswConfig)
        || !finite_vector(
            finite_values,
            static_cast<int>(sizeof(finite_values) / sizeof(finite_values[0]))
        )
        || config.stage1_burn_s <= 0.0
        || config.separation_delay_s < 0.0
        || config.stage2_ignition_delay_s < 0.0
        || config.stage2_burn_s <= 0.0
        || config.main_deploy_altitude_m <= 0.0
        || config.max_tvc_rad <= 0.0
        || config.max_fin_rad <= 0.0
        || config.control_kp < 0.0
        || config.control_kd < 0.0
        || config.imu_timeout_s <= 0.0
        || config.barometer_timeout_s <= 0.0
        || config.gnss_timeout_s <= 0.0
        || config.acceleration_disagreement_m_s2 <= 0.0
        || config.gyro_disagreement_rad_s <= 0.0
        || config.barometer_disagreement_m <= 0.0
        || config.gnss_position_disagreement_m <= 0.0
        || config.gnss_velocity_disagreement_m_s <= 0.0
        || config.cross_altitude_disagreement_m <= 0.0
        || config.voter_reject_samples == 0
        || config.voter_recover_samples == 0
        || config.imu_loss_abort_delay_s <= 0.0
        || config.gyro_bias_time_constant_s <= 0.0
        || config.stationary_gyro_threshold_rad_s <= 0.0
        || config.altitude_filter_tau_s <= 0.0
        || config.velocity_filter_tau_s <= 0.0
        || config.magnetic_disagreement <= 0.0
        || config.command_timeout_s <= 0.0
        || config.launch_confirm_timeout_s <= 0.0
        || config.separation_confirm_timeout_s <= 0.0
        || config.stage2_ignition_timeout_s <= 0.0
        || config.drogue_confirm_timeout_s <= 0.0
        || config.main_confirm_timeout_s <= 0.0
        || config.fault_recovery_persistence_s <= 0.0
        || config.min_step_s <= 0.0
        || config.max_step_s < config.min_step_s
        || config.loop_deadline_s <= 0.0
        || config.overrun_abort_count == 0
        || config.propulsion_abort_health_percent < 0.0
        || config.propulsion_abort_health_percent > 100.0
        || config.propulsion_abort_persistence_s <= 0.0
        || config.max_acceleration_m_s2 <= 0.0
        || config.max_gyro_rad_s <= 0.0
        || config.min_magnetic_norm < 0.0
        || config.max_magnetic_norm <= config.min_magnetic_norm
        || config.max_barometer_altitude_m
            <= config.min_barometer_altitude_m
        || config.max_barometer_rate_m_s <= 0.0
        || config.max_gnss_radius_m <= config.min_gnss_radius_m
        || config.max_gnss_speed_m_s <= 0.0
        || config.max_gnss_velocity_rate_m_s2 <= 0.0
        || config.accelerometer_process_sigma_m_s2 <= 0.0
        || config.gyro_process_sigma_rad_s <= 0.0
        || config.barometer_sigma_m <= 0.0
        || config.gnss_altitude_sigma_m <= 0.0
        || config.gnss_velocity_sigma_m_s <= 0.0
        || config.max_altitude_sigma_m <= 0.0
        || config.max_velocity_sigma_m_s <= 0.0
        || config.max_attitude_sigma_rad <= 0.0
        || config.guidance_count < 2
        || config.guidance_count > FSW_MAX_GUIDANCE_POINTS
        || config.body_role < FSW_BODY_INTEGRATED
        || config.body_role > FSW_BODY_UPPER
    ) {
        return false;
    }
    for (uint32_t index = 0; index < config.guidance_count; ++index) {
        const auto& point = config.guidance[index];
        if (
            !std::isfinite(point.time_s)
            || !std::isfinite(point.pitch_rad)
            || !std::isfinite(point.azimuth_rad)
            || (index > 0 && point.time_s <= config.guidance[index - 1].time_s)
        ) {
            return false;
        }
    }
    return true;
}

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
        || sensor.barometer_count > FSW_MAX_SENSOR_CHANNELS
        || sensor.gnss_count > FSW_MAX_SENSOR_CHANNELS
    ) {
        return false;
    }
    for (uint32_t index = 0; index < sensor.imu_count; ++index) {
        const auto& sample = sensor.imus[index];
        if (
            sample.valid
            && (
                !finite_vector(sample.acceleration_body_m_s2, 3)
                || !finite_vector(sample.gyro_body_rad_s, 3)
                || !finite_vector(sample.magnetic_body, 3)
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
                || !std::isfinite(sample.vertical_velocity_m_s)
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

struct ChannelHealth {
    uint32_t bad_samples{};
    uint32_t good_samples{};
    bool rejected{};
    double last_evaluated_sample_time_s{-1.0};
    double last_value_time_s{-1.0};
    std::array<double, 3> last_values{};
    uint32_t flags{};
    double age_s{};
};

struct VotedSensors {
    bool imu_valid{};
    bool barometer_valid{};
    bool gnss_valid{};
    std::array<double, 3> acceleration{};
    std::array<double, 3> gyro{};
    std::array<double, 3> magnetic{};
    std::array<double, 3> gnss_position{};
    std::array<double, 3> gnss_velocity{};
    double barometric_altitude{};
    double vertical_velocity{};
    double imu_sample_time_s{-1.0};
    double barometer_sample_time_s{-1.0};
    double gnss_sample_time_s{-1.0};
    uint32_t imu_usable_mask{};
    uint32_t barometer_usable_mask{};
    uint32_t gnss_usable_mask{};
    uint32_t disagreement_flags{};
    double barometer_innovation{};
    double gnss_altitude_innovation{};
    double gnss_velocity_innovation{};
};

struct Context {
    explicit Context(const FswConfig& supplied) : config(supplied) { reset(); }

    void reset() {
        mode = config.body_role == FSW_BODY_CORE
            ? FSW_MODE_COAST
            : (
                config.body_role == FSW_BODY_UPPER
                    ? FSW_MODE_INTERSTAGE
                    : FSW_MODE_SAFE
            );
        altitude = 0.0;
        vertical_velocity = 0.0;
        attitude = {1.0, 0.0, 0.0, 0.0};
        last_gyro = {0.0, 0.0, 0.0};
        gyro_bias = {0.0, 0.0, 0.0};
        imu_health = {};
        barometer_health = {};
        gnss_health = {};
        apogee_seen = false;
        drogue_deployed = false;
        main_deployed = false;
        active_fault_flags = 0;
        latched_fault_flags = 0;
        previous_active_fault_flags = 0;
        changed_fault_flags = 0;
        fault_occurrence_count = {};
        highest_fault_severity = FSW_SEVERITY_NONE;
        disagreement_flags = 0;
        sensor_status_flags = 0;
        imu_usable_mask = 0;
        barometer_usable_mask = 0;
        gnss_usable_mask = 0;
        imu_rejected_mask = 0;
        barometer_rejected_mask = 0;
        gnss_rejected_mask = 0;
        attitude_valid = false;
        navigation_status = FSW_NAV_INERTIAL;
        navigation_initialized = false;
        gnss_altitude_reference_initialized = false;
        gnss_radius_reference_m = 0.0;
        time_initialized = false;
        last_time_s = 0.0;
        last_barometer_sample_time_s = -1.0;
        last_gnss_sample_time_s = -1.0;
        launch_evidence_s = 0.0;
        burnout_evidence_s = 0.0;
        apogee_evidence_s = 0.0;
        landing_evidence_s = 0.0;
        imu_loss_evidence_s = 0.0;
        propulsion_loss_evidence_s = 0.0;
        fault_healthy_time_s = {};
        ignition_started_s = -1.0;
        separation_started_s = -1.0;
        stage2_ignition_started_s = -1.0;
        drogue_commanded_s = -1.0;
        main_commanded_s = -1.0;
        last_command_sequence = 0;
        command_sequence = 0;
        command_type = FSW_COMMAND_NONE;
        command_result = FSW_COMMAND_NOT_PROCESSED;
        inhibit_flags = 0;
        event_flags = 0;
        stage1_ignite_request = false;
        stage2_ignite_request = false;
        previous_mode = mode;
        altitude_variance = config.barometer_sigma_m
            * config.barometer_sigma_m;
        velocity_variance = config.gnss_velocity_sigma_m_s
            * config.gnss_velocity_sigma_m_s;
        attitude_variance = {
            config.gyro_process_sigma_rad_s
                * config.gyro_process_sigma_rad_s,
            config.gyro_process_sigma_rad_s
                * config.gyro_process_sigma_rad_s,
            config.gyro_process_sigma_rad_s
                * config.gyro_process_sigma_rad_s,
        };
        barometer_innovation = 0.0;
        gnss_altitude_innovation = 0.0;
        gnss_velocity_innovation = 0.0;
        consecutive_overruns = 0;
        previous_execution_time_s = 0.0;
    }

    FswConfig config;
    int32_t mode{};
    double altitude{};
    double vertical_velocity{};
    std::array<double, 4> attitude{};
    std::array<double, 3> last_gyro{};
    std::array<double, 3> gyro_bias{};
    std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS> imu_health{};
    std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS> barometer_health{};
    std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS> gnss_health{};
    bool apogee_seen{};
    bool drogue_deployed{};
    bool main_deployed{};
    uint32_t active_fault_flags{};
    uint32_t latched_fault_flags{};
    uint32_t previous_active_fault_flags{};
    uint32_t changed_fault_flags{};
    std::array<uint32_t, FSW_FAULT_COUNT> fault_occurrence_count{};
    int32_t highest_fault_severity{};
    uint32_t disagreement_flags{};
    uint32_t sensor_status_flags{};
    uint32_t imu_usable_mask{};
    uint32_t barometer_usable_mask{};
    uint32_t gnss_usable_mask{};
    uint32_t imu_rejected_mask{};
    uint32_t barometer_rejected_mask{};
    uint32_t gnss_rejected_mask{};
    bool attitude_valid{};
    int32_t navigation_status{};
    bool navigation_initialized{};
    bool gnss_altitude_reference_initialized{};
    double gnss_radius_reference_m{};
    bool time_initialized{};
    double last_time_s{};
    double last_barometer_sample_time_s{};
    double last_gnss_sample_time_s{};
    double launch_evidence_s{};
    double burnout_evidence_s{};
    double apogee_evidence_s{};
    double landing_evidence_s{};
    double imu_loss_evidence_s{};
    double propulsion_loss_evidence_s{};
    std::array<double, FSW_FAULT_COUNT> fault_healthy_time_s{};
    double ignition_started_s{};
    double separation_started_s{};
    double stage2_ignition_started_s{};
    double drogue_commanded_s{};
    double main_commanded_s{};
    uint64_t last_command_sequence{};
    uint64_t command_sequence{};
    int32_t command_type{};
    int32_t command_result{};
    uint32_t inhibit_flags{};
    uint32_t event_flags{};
    bool stage1_ignite_request{};
    bool stage2_ignite_request{};
    int32_t previous_mode{};
    double altitude_variance{};
    double velocity_variance{};
    std::array<double, 3> attitude_variance{};
    double barometer_innovation{};
    double gnss_altitude_innovation{};
    double gnss_velocity_innovation{};
    uint32_t consecutive_overruns{};
    double previous_execution_time_s{};
};

bool persisted(
    bool condition,
    double dt_s,
    double required_s,
    double& evidence_s
) {
    evidence_s = condition ? evidence_s + std::max(dt_s, 0.0) : 0.0;
    return evidence_s + kEpsilon >= required_s;
}

bool new_sample(ChannelHealth& health, double sample_time_s) {
    if (sample_time_s <= health.last_evaluated_sample_time_s + kEpsilon) {
        return false;
    }
    health.last_evaluated_sample_time_s = sample_time_s;
    return true;
}

void observe_channel(
    ChannelHealth& health,
    bool agrees,
    bool allow_rejection,
    const FswConfig& config
) {
    if (agrees) {
        health.bad_samples = 0;
        ++health.good_samples;
        if (
            health.rejected
            && health.good_samples >= config.voter_recover_samples
        ) {
            health.rejected = false;
            health.good_samples = 0;
        }
        return;
    }
    health.good_samples = 0;
    ++health.bad_samples;
    if (
        allow_rejection
        && !health.rejected
        && health.bad_samples >= config.voter_reject_samples
    ) {
        health.rejected = true;
        health.bad_samples = 0;
    }
}

uint32_t rejected_mask(
    const std::array<ChannelHealth, FSW_MAX_SENSOR_CHANNELS>& health,
    uint32_t count
) {
    uint32_t mask = 0;
    for (uint32_t index = 0; index < count; ++index) {
        if (health[index].rejected) {
            mask |= 1u << index;
        }
    }
    return mask;
}

bool evaluate_imu_channel(
    ChannelHealth& health,
    const FswImuSample& sample,
    double time_s,
    const FswConfig& config
) {
    health.flags = 0;
    health.age_s = sample.valid
        ? std::max(time_s - sample.sample_time_s, 0.0)
        : 0.0;
    if (!sample.valid) {
        health.flags |= FSW_SENSOR_HEALTH_INVALID;
        return false;
    }
    if (!fresh(
        true, sample.sample_time_s, time_s, config.imu_timeout_s
    )) {
        health.flags |= FSW_SENSOR_HEALTH_STALE;
        return false;
    }
    const double magnetic_norm = vector_norm(sample.magnetic_body, 3);
    if (
        vector_norm(sample.acceleration_body_m_s2, 3)
            > config.max_acceleration_m_s2
        || vector_norm(sample.gyro_body_rad_s, 3) > config.max_gyro_rad_s
        || magnetic_norm < config.min_magnetic_norm
        || magnetic_norm > config.max_magnetic_norm
    ) {
        health.flags |= FSW_SENSOR_HEALTH_OUT_OF_RANGE;
        return false;
    }
    if (health.rejected) {
        health.flags |= FSW_SENSOR_HEALTH_REJECTED;
    }
    return true;
}

bool evaluate_barometer_channel(
    ChannelHealth& health,
    const FswBarometerSample& sample,
    double time_s,
    const FswConfig& config
) {
    health.flags = 0;
    health.age_s = sample.valid
        ? std::max(time_s - sample.sample_time_s, 0.0)
        : 0.0;
    if (!sample.valid) {
        health.flags |= FSW_SENSOR_HEALTH_INVALID;
        return false;
    }
    if (!fresh(
        true, sample.sample_time_s, time_s, config.barometer_timeout_s
    )) {
        health.flags |= FSW_SENSOR_HEALTH_STALE;
        return false;
    }
    if (
        sample.altitude_m < config.min_barometer_altitude_m
        || sample.altitude_m > config.max_barometer_altitude_m
    ) {
        health.flags |= FSW_SENSOR_HEALTH_OUT_OF_RANGE;
        return false;
    }
    if (
        health.last_value_time_s >= 0.0
        && sample.sample_time_s > health.last_value_time_s + kEpsilon
        && std::abs(sample.altitude_m - health.last_values[0])
            / (sample.sample_time_s - health.last_value_time_s)
            > config.max_barometer_rate_m_s
    ) {
        health.flags |= FSW_SENSOR_HEALTH_RATE_LIMIT;
        health.last_values[0] = sample.altitude_m;
        health.last_value_time_s = sample.sample_time_s;
        return false;
    }
    if (sample.sample_time_s > health.last_value_time_s + kEpsilon) {
        health.last_values[0] = sample.altitude_m;
        health.last_value_time_s = sample.sample_time_s;
    }
    if (health.rejected) {
        health.flags |= FSW_SENSOR_HEALTH_REJECTED;
    }
    return true;
}

bool evaluate_gnss_channel(
    ChannelHealth& health,
    const FswGnssSample& sample,
    double time_s,
    const FswConfig& config
) {
    health.flags = 0;
    health.age_s = sample.valid
        ? std::max(time_s - sample.sample_time_s, 0.0)
        : 0.0;
    if (!sample.valid) {
        health.flags |= FSW_SENSOR_HEALTH_INVALID;
        return false;
    }
    if (!fresh(
        true, sample.sample_time_s, time_s, config.gnss_timeout_s
    )) {
        health.flags |= FSW_SENSOR_HEALTH_STALE;
        return false;
    }
    const double radius = vector_norm(sample.gnss_position_ecef_m, 3);
    const double speed = vector_norm(sample.gnss_velocity_ecef_m_s, 3);
    if (
        radius < config.min_gnss_radius_m
        || radius > config.max_gnss_radius_m
        || speed > config.max_gnss_speed_m_s
    ) {
        health.flags |= FSW_SENSOR_HEALTH_OUT_OF_RANGE;
        return false;
    }
    if (
        health.last_value_time_s >= 0.0
        && sample.sample_time_s > health.last_value_time_s + kEpsilon
        && vector_distance(
            sample.gnss_velocity_ecef_m_s,
            health.last_values.data(),
            3
        ) / (sample.sample_time_s - health.last_value_time_s)
            > config.max_gnss_velocity_rate_m_s2
    ) {
        health.flags |= FSW_SENSOR_HEALTH_RATE_LIMIT;
        for (int axis = 0; axis < 3; ++axis) {
            health.last_values[axis] =
                sample.gnss_velocity_ecef_m_s[axis];
        }
        health.last_value_time_s = sample.sample_time_s;
        return false;
    }
    if (sample.sample_time_s > health.last_value_time_s + kEpsilon) {
        for (int axis = 0; axis < 3; ++axis) {
            health.last_values[axis] =
                sample.gnss_velocity_ecef_m_s[axis];
        }
        health.last_value_time_s = sample.sample_time_s;
    }
    if (health.rejected) {
        health.flags |= FSW_SENSOR_HEALTH_REJECTED;
    }
    return true;
}

bool imu_samples_agree(
    const FswImuSample& left,
    const FswImuSample& right,
    const FswConfig& config,
    uint32_t& disagreement_flags
) {
    const bool acceleration_agrees = vector_distance(
        left.acceleration_body_m_s2, right.acceleration_body_m_s2, 3
    ) <= config.acceleration_disagreement_m_s2;
    const bool gyro_agrees = vector_distance(
        left.gyro_body_rad_s, right.gyro_body_rad_s, 3
    ) <= config.gyro_disagreement_rad_s;
    const bool magnetic_agrees = vector_distance(
        left.magnetic_body, right.magnetic_body, 3
    ) <= config.magnetic_disagreement;
    if (!acceleration_agrees) {
        disagreement_flags |= FSW_DISAGREEMENT_ACCELERATION;
    }
    if (!gyro_agrees) {
        disagreement_flags |= FSW_DISAGREEMENT_GYRO;
    }
    if (!magnetic_agrees) {
        disagreement_flags |= FSW_DISAGREEMENT_MAGNETOMETER;
    }
    return acceleration_agrees && gyro_agrees && magnetic_agrees;
}

void average_imus(
    const FswSensorSuite& suite,
    uint32_t mask,
    VotedSensors& voted
) {
    const double divisor = static_cast<double>(bit_count(mask));
    for (uint32_t index = 0; index < suite.imu_count; ++index) {
        if ((mask & (1u << index)) == 0) {
            continue;
        }
        const auto& sample = suite.imus[index];
        for (int axis = 0; axis < 3; ++axis) {
            voted.acceleration[axis] +=
                sample.acceleration_body_m_s2[axis] / divisor;
            voted.gyro[axis] += sample.gyro_body_rad_s[axis] / divisor;
            voted.magnetic[axis] += sample.magnetic_body[axis] / divisor;
        }
        voted.imu_sample_time_s = std::max(
            voted.imu_sample_time_s, sample.sample_time_s
        );
    }
    voted.imu_usable_mask = mask;
    voted.imu_valid = mask != 0;
}

void vote_imus(
    Context& context,
    const FswSensorSuite& suite,
    VotedSensors& voted
) {
    uint32_t fresh_mask = 0;
    uint32_t healthy_mask = 0;
    for (uint32_t index = 0; index < suite.imu_count; ++index) {
        const auto& sample = suite.imus[index];
        if (evaluate_imu_channel(
            context.imu_health[index],
            sample,
            suite.time_s,
            context.config
        )) {
            fresh_mask |= 1u << index;
            if (!context.imu_health[index].rejected) {
                healthy_mask |= 1u << index;
            }
        }
    }

    uint32_t consensus_mask = 0;
    const uint32_t healthy_count = bit_count(healthy_mask);
    if (healthy_count == 1) {
        consensus_mask = healthy_mask;
    } else if (healthy_count == 2) {
        int indices[2]{};
        int found = 0;
        for (uint32_t index = 0; index < suite.imu_count; ++index) {
            if (healthy_mask & (1u << index)) {
                indices[found++] = static_cast<int>(index);
            }
        }
        if (imu_samples_agree(
            suite.imus[indices[0]],
            suite.imus[indices[1]],
            context.config,
            voted.disagreement_flags
        )) {
            consensus_mask = healthy_mask;
        }
    } else if (healthy_count == 3) {
        double acceleration_median[3]{};
        double gyro_median[3]{};
        double magnetic_median[3]{};
        for (int axis = 0; axis < 3; ++axis) {
            acceleration_median[axis] = median3(
                suite.imus[0].acceleration_body_m_s2[axis],
                suite.imus[1].acceleration_body_m_s2[axis],
                suite.imus[2].acceleration_body_m_s2[axis]
            );
            gyro_median[axis] = median3(
                suite.imus[0].gyro_body_rad_s[axis],
                suite.imus[1].gyro_body_rad_s[axis],
                suite.imus[2].gyro_body_rad_s[axis]
            );
            magnetic_median[axis] = median3(
                suite.imus[0].magnetic_body[axis],
                suite.imus[1].magnetic_body[axis],
                suite.imus[2].magnetic_body[axis]
            );
        }
        for (uint32_t index = 0; index < suite.imu_count; ++index) {
            const bool acceleration_agrees = vector_distance(
                suite.imus[index].acceleration_body_m_s2,
                acceleration_median,
                3
            ) <= context.config.acceleration_disagreement_m_s2;
            const bool gyro_agrees = vector_distance(
                suite.imus[index].gyro_body_rad_s,
                gyro_median,
                3
            ) <= context.config.gyro_disagreement_rad_s;
            const bool magnetic_agrees = vector_distance(
                suite.imus[index].magnetic_body,
                magnetic_median,
                3
            ) <= context.config.magnetic_disagreement;
            if (acceleration_agrees && gyro_agrees && magnetic_agrees) {
                consensus_mask |= 1u << index;
            } else {
                if (!acceleration_agrees) {
                    voted.disagreement_flags |=
                        FSW_DISAGREEMENT_ACCELERATION;
                }
                if (!gyro_agrees) {
                    voted.disagreement_flags |= FSW_DISAGREEMENT_GYRO;
                }
                if (!magnetic_agrees) {
                    voted.disagreement_flags |=
                        FSW_DISAGREEMENT_MAGNETOMETER;
                }
            }
        }
        if (bit_count(consensus_mask) < 2) {
            consensus_mask = 0;
        }
    }

    if (consensus_mask != 0) {
        average_imus(suite, consensus_mask, voted);
    }
    for (uint32_t index = 0; index < suite.imu_count; ++index) {
        if ((fresh_mask & (1u << index)) == 0) {
            continue;
        }
        auto& health = context.imu_health[index];
        if (!new_sample(health, suite.imus[index].sample_time_s)) {
            continue;
        }
        bool agrees = (consensus_mask & (1u << index)) != 0;
        if (!agrees) {
            health.flags |= FSW_SENSOR_HEALTH_DISAGREEMENT;
        }
        if (health.rejected && voted.imu_valid) {
            FswImuSample fused{};
            for (int axis = 0; axis < 3; ++axis) {
                fused.acceleration_body_m_s2[axis] =
                    voted.acceleration[axis];
                fused.gyro_body_rad_s[axis] = voted.gyro[axis];
            }
            uint32_t ignored_flags = 0;
            agrees = imu_samples_agree(
                suite.imus[index],
                fused,
                context.config,
                ignored_flags
            );
        }
        observe_channel(
            health,
            agrees,
            healthy_count >= 3,
            context.config
        );
        if (health.rejected) {
            health.flags |= FSW_SENSOR_HEALTH_REJECTED;
        }
    }
}

void vote_barometers(
    Context& context,
    const FswSensorSuite& suite,
    VotedSensors& voted
) {
    uint32_t fresh_mask = 0;
    uint32_t healthy_mask = 0;
    for (uint32_t index = 0; index < suite.barometer_count; ++index) {
        const auto& sample = suite.barometers[index];
        if (evaluate_barometer_channel(
            context.barometer_health[index],
            sample,
            suite.time_s,
            context.config
        )) {
            fresh_mask |= 1u << index;
            if (!context.barometer_health[index].rejected) {
                healthy_mask |= 1u << index;
            }
        }
    }

    uint32_t consensus_mask = 0;
    const uint32_t healthy_count = bit_count(healthy_mask);
    if (healthy_count == 1) {
        consensus_mask = healthy_mask;
    } else if (healthy_count == 2) {
        int indices[2]{};
        int found = 0;
        for (uint32_t index = 0; index < suite.barometer_count; ++index) {
            if (healthy_mask & (1u << index)) {
                indices[found++] = static_cast<int>(index);
            }
        }
        if (
            std::abs(
                suite.barometers[indices[0]].altitude_m
                - suite.barometers[indices[1]].altitude_m
            ) <= context.config.barometer_disagreement_m
        ) {
            consensus_mask = healthy_mask;
        } else {
            voted.disagreement_flags |= FSW_DISAGREEMENT_BAROMETER;
        }
    } else if (healthy_count == 3) {
        const double center = median3(
            suite.barometers[0].altitude_m,
            suite.barometers[1].altitude_m,
            suite.barometers[2].altitude_m
        );
        for (uint32_t index = 0; index < suite.barometer_count; ++index) {
            if (
                std::abs(suite.barometers[index].altitude_m - center)
                <= context.config.barometer_disagreement_m
            ) {
                consensus_mask |= 1u << index;
            } else {
                voted.disagreement_flags |= FSW_DISAGREEMENT_BAROMETER;
            }
        }
        if (bit_count(consensus_mask) < 2) {
            consensus_mask = 0;
        }
    }

    if (consensus_mask != 0) {
        const double divisor = static_cast<double>(bit_count(consensus_mask));
        for (uint32_t index = 0; index < suite.barometer_count; ++index) {
            if ((consensus_mask & (1u << index)) == 0) {
                continue;
            }
            voted.barometric_altitude +=
                suite.barometers[index].altitude_m / divisor;
            voted.barometer_sample_time_s = std::max(
                voted.barometer_sample_time_s,
                suite.barometers[index].sample_time_s
            );
        }
        voted.barometer_usable_mask = consensus_mask;
        voted.barometer_valid = true;
    }
    for (uint32_t index = 0; index < suite.barometer_count; ++index) {
        if ((fresh_mask & (1u << index)) == 0) {
            continue;
        }
        auto& health = context.barometer_health[index];
        if (!new_sample(health, suite.barometers[index].sample_time_s)) {
            continue;
        }
        bool agrees = (consensus_mask & (1u << index)) != 0;
        if (!agrees) {
            health.flags |= FSW_SENSOR_HEALTH_DISAGREEMENT;
        }
        if (health.rejected && voted.barometer_valid) {
            agrees = std::abs(
                suite.barometers[index].altitude_m
                - voted.barometric_altitude
            ) <= context.config.barometer_disagreement_m;
        }
        observe_channel(
            health,
            agrees,
            healthy_count >= 3,
            context.config
        );
        if (health.rejected) {
            health.flags |= FSW_SENSOR_HEALTH_REJECTED;
        }
    }
}

bool gnss_samples_agree(
    const FswGnssSample& left,
    const FswGnssSample& right,
    const FswConfig& config,
    uint32_t& disagreement_flags
) {
    const bool position_agrees = vector_distance(
        left.gnss_position_ecef_m, right.gnss_position_ecef_m, 3
    ) <= config.gnss_position_disagreement_m;
    const bool velocity_agrees = vector_distance(
        left.gnss_velocity_ecef_m_s, right.gnss_velocity_ecef_m_s, 3
    ) <= config.gnss_velocity_disagreement_m_s
        && std::abs(
            left.vertical_velocity_m_s - right.vertical_velocity_m_s
        ) <= config.gnss_velocity_disagreement_m_s;
    if (!position_agrees) {
        disagreement_flags |= FSW_DISAGREEMENT_GNSS_POSITION;
    }
    if (!velocity_agrees) {
        disagreement_flags |= FSW_DISAGREEMENT_GNSS_VELOCITY;
    }
    return position_agrees && velocity_agrees;
}

void average_gnss(
    const FswSensorSuite& suite,
    uint32_t mask,
    VotedSensors& voted
) {
    const double divisor = static_cast<double>(bit_count(mask));
    for (uint32_t index = 0; index < suite.gnss_count; ++index) {
        if ((mask & (1u << index)) == 0) {
            continue;
        }
        const auto& sample = suite.gnss[index];
        for (int axis = 0; axis < 3; ++axis) {
            voted.gnss_position[axis] +=
                sample.gnss_position_ecef_m[axis] / divisor;
            voted.gnss_velocity[axis] +=
                sample.gnss_velocity_ecef_m_s[axis] / divisor;
        }
        voted.vertical_velocity += sample.vertical_velocity_m_s / divisor;
        voted.gnss_sample_time_s = std::max(
            voted.gnss_sample_time_s, sample.sample_time_s
        );
    }
    voted.gnss_usable_mask = mask;
    voted.gnss_valid = mask != 0;
}

void vote_gnss(
    Context& context,
    const FswSensorSuite& suite,
    VotedSensors& voted
) {
    uint32_t fresh_mask = 0;
    uint32_t healthy_mask = 0;
    for (uint32_t index = 0; index < suite.gnss_count; ++index) {
        const auto& sample = suite.gnss[index];
        if (evaluate_gnss_channel(
            context.gnss_health[index],
            sample,
            suite.time_s,
            context.config
        )) {
            fresh_mask |= 1u << index;
            if (!context.gnss_health[index].rejected) {
                healthy_mask |= 1u << index;
            }
        }
    }

    uint32_t consensus_mask = 0;
    const uint32_t healthy_count = bit_count(healthy_mask);
    if (healthy_count == 1) {
        consensus_mask = healthy_mask;
    } else if (healthy_count == 2) {
        int indices[2]{};
        int found = 0;
        for (uint32_t index = 0; index < suite.gnss_count; ++index) {
            if (healthy_mask & (1u << index)) {
                indices[found++] = static_cast<int>(index);
            }
        }
        if (gnss_samples_agree(
            suite.gnss[indices[0]],
            suite.gnss[indices[1]],
            context.config,
            voted.disagreement_flags
        )) {
            consensus_mask = healthy_mask;
        }
    } else if (healthy_count == 3) {
        double position_median[3]{};
        double velocity_median[3]{};
        for (int axis = 0; axis < 3; ++axis) {
            position_median[axis] = median3(
                suite.gnss[0].gnss_position_ecef_m[axis],
                suite.gnss[1].gnss_position_ecef_m[axis],
                suite.gnss[2].gnss_position_ecef_m[axis]
            );
            velocity_median[axis] = median3(
                suite.gnss[0].gnss_velocity_ecef_m_s[axis],
                suite.gnss[1].gnss_velocity_ecef_m_s[axis],
                suite.gnss[2].gnss_velocity_ecef_m_s[axis]
            );
        }
        const double vertical_median = median3(
            suite.gnss[0].vertical_velocity_m_s,
            suite.gnss[1].vertical_velocity_m_s,
            suite.gnss[2].vertical_velocity_m_s
        );
        for (uint32_t index = 0; index < suite.gnss_count; ++index) {
            const bool position_agrees = vector_distance(
                suite.gnss[index].gnss_position_ecef_m,
                position_median,
                3
            ) <= context.config.gnss_position_disagreement_m;
            const bool velocity_agrees = vector_distance(
                suite.gnss[index].gnss_velocity_ecef_m_s,
                velocity_median,
                3
            ) <= context.config.gnss_velocity_disagreement_m_s
                && std::abs(
                    suite.gnss[index].vertical_velocity_m_s
                    - vertical_median
                ) <= context.config.gnss_velocity_disagreement_m_s;
            if (position_agrees && velocity_agrees) {
                consensus_mask |= 1u << index;
            } else {
                if (!position_agrees) {
                    voted.disagreement_flags |=
                        FSW_DISAGREEMENT_GNSS_POSITION;
                }
                if (!velocity_agrees) {
                    voted.disagreement_flags |=
                        FSW_DISAGREEMENT_GNSS_VELOCITY;
                }
            }
        }
        if (bit_count(consensus_mask) < 2) {
            consensus_mask = 0;
        }
    }

    if (consensus_mask != 0) {
        average_gnss(suite, consensus_mask, voted);
    }
    for (uint32_t index = 0; index < suite.gnss_count; ++index) {
        if ((fresh_mask & (1u << index)) == 0) {
            continue;
        }
        auto& health = context.gnss_health[index];
        if (!new_sample(health, suite.gnss[index].sample_time_s)) {
            continue;
        }
        bool agrees = (consensus_mask & (1u << index)) != 0;
        if (!agrees) {
            health.flags |= FSW_SENSOR_HEALTH_DISAGREEMENT;
        }
        if (health.rejected && voted.gnss_valid) {
            FswGnssSample fused{};
            for (int axis = 0; axis < 3; ++axis) {
                fused.gnss_position_ecef_m[axis] =
                    voted.gnss_position[axis];
                fused.gnss_velocity_ecef_m_s[axis] =
                    voted.gnss_velocity[axis];
            }
            fused.vertical_velocity_m_s = voted.vertical_velocity;
            uint32_t ignored_flags = 0;
            agrees = gnss_samples_agree(
                suite.gnss[index],
                fused,
                context.config,
                ignored_flags
            );
        }
        observe_channel(
            health,
            agrees,
            healthy_count >= 3,
            context.config
        );
        if (health.rejected) {
            health.flags |= FSW_SENSOR_HEALTH_REJECTED;
        }
    }
}

VotedSensors vote_sensors(Context& context, const FswSensorSuite& suite) {
    VotedSensors voted{};
    vote_imus(context, suite, voted);
    vote_barometers(context, suite, voted);
    vote_gnss(context, suite, voted);

    context.imu_usable_mask = voted.imu_usable_mask;
    context.barometer_usable_mask = voted.barometer_usable_mask;
    context.gnss_usable_mask = voted.gnss_usable_mask;
    context.imu_rejected_mask = rejected_mask(
        context.imu_health, suite.imu_count
    );
    context.barometer_rejected_mask = rejected_mask(
        context.barometer_health, suite.barometer_count
    );
    context.gnss_rejected_mask = rejected_mask(
        context.gnss_health, suite.gnss_count
    );
    context.disagreement_flags = voted.disagreement_flags;
    context.sensor_status_flags = 0;
    if (bit_count(voted.imu_usable_mask) == 1) {
        context.sensor_status_flags |= FSW_SENSOR_STATUS_IMU_SINGLE_SOURCE;
    }
    if (bit_count(voted.barometer_usable_mask) == 1) {
        context.sensor_status_flags |=
            FSW_SENSOR_STATUS_BAROMETER_SINGLE_SOURCE;
    }
    if (bit_count(voted.gnss_usable_mask) == 1) {
        context.sensor_status_flags |= FSW_SENSOR_STATUS_GNSS_SINGLE_SOURCE;
    }
    return voted;
}

void normalize(std::array<double, 4>& quaternion) {
    double norm = 0.0;
    for (double value : quaternion) {
        norm += value * value;
    }
    norm = std::sqrt(norm);
    if (norm < kEpsilon) {
        quaternion = {1.0, 0.0, 0.0, 0.0};
        return;
    }
    for (double& value : quaternion) {
        value /= norm;
    }
}

void update_gyro_bias(
    Context& context,
    const VotedSensors& voted,
    double dt_s
) {
    if (
        context.mode > FSW_MODE_ARMED
        || vector_norm(voted.gyro.data(), 3)
            > context.config.stationary_gyro_threshold_rad_s
    ) {
        return;
    }
    const double alpha = 1.0 - std::exp(
        -dt_s / context.config.gyro_bias_time_constant_s
    );
    for (int axis = 0; axis < 3; ++axis) {
        context.gyro_bias[axis] += alpha * (
            voted.gyro[axis] - context.gyro_bias[axis]
        );
    }
}

void integrate_attitude(
    Context& context,
    const std::array<double, 3>& gyro,
    double dt_s
) {
    const auto q = context.attitude;
    const double gx = gyro[0];
    const double gy = gyro[1];
    const double gz = gyro[2];
    const double half_dt = 0.5 * dt_s;
    context.attitude = {
        q[0] + (-q[1] * gx - q[2] * gy - q[3] * gz) * half_dt,
        q[1] + (q[0] * gx + q[2] * gz - q[3] * gy) * half_dt,
        q[2] + (q[0] * gy - q[1] * gz + q[3] * gx) * half_dt,
        q[3] + (q[0] * gz + q[1] * gy - q[2] * gx) * half_dt,
    };
    normalize(context.attitude);
    context.last_gyro = gyro;
}

std::array<double, 3> euler(const std::array<double, 4>& q) {
    const double roll = std::atan2(
        2.0 * (q[0] * q[1] + q[2] * q[3]),
        1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
    );
    const double pitch = std::asin(clamp(
        2.0 * (q[0] * q[2] - q[3] * q[1]), -1.0, 1.0
    ));
    const double yaw = std::atan2(
        2.0 * (q[0] * q[3] + q[1] * q[2]),
        1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
    );
    return {roll, pitch, yaw};
}

double filter_alpha(double sample_delta_s, double tau_s) {
    return 1.0 - std::exp(-std::max(sample_delta_s, kEpsilon) / tau_s);
}

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
    const uint32_t rising = flags & ~context.previous_active_fault_flags;
    context.changed_fault_flags =
        flags ^ context.previous_active_fault_flags;
    for (uint32_t index = 0; index < FSW_FAULT_COUNT; ++index) {
        if (rising & (1u << index)) {
            ++context.fault_occurrence_count[index];
        }
    }
    context.active_fault_flags = flags;
    context.latched_fault_flags |= flags;
    context.previous_active_fault_flags = flags;
    context.highest_fault_severity = fault_severity(flags);
    if (context.changed_fault_flags != 0) {
        context.event_flags |= FSW_EVENT_FAULT_CHANGED;
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
        context.disagreement_flags
        & (FSW_DISAGREEMENT_ACCELERATION | FSW_DISAGREEMENT_GYRO)
    ) {
        flags |= FSW_FAULT_IMU_DISAGREEMENT;
    }
    if (
        context.disagreement_flags
        & FSW_DISAGREEMENT_MAGNETOMETER
    ) {
        flags |= FSW_FAULT_MAGNETOMETER_DISAGREEMENT;
    }
    if (context.disagreement_flags & FSW_DISAGREEMENT_BAROMETER) {
        flags |= FSW_FAULT_BAROMETER_DISAGREEMENT;
    }
    if (
        context.disagreement_flags
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
    if (context.navigation_status == FSW_NAV_INERTIAL) {
        flags |= FSW_FAULT_NAV_INERTIAL;
    }
    const auto& suite = input.sensors;
    const bool air_data_valid = fresh(
        input.air_data.valid,
        input.air_data.sample_time_s,
        suite.time_s,
        context.config.imu_timeout_s
    );
    if (!air_data_valid) {
        flags |= FSW_FAULT_AIR_DATA_UNAVAILABLE;
    }
    const bool propulsion_valid = fresh(
        input.propulsion.valid,
        input.propulsion.sample_time_s,
        suite.time_s,
        context.config.imu_timeout_s
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
        std::sqrt(context.altitude_variance)
            > context.config.max_altitude_sigma_m
        || std::sqrt(context.velocity_variance)
            > context.config.max_velocity_sigma_m_s
        || std::sqrt(*std::max_element(
            context.attitude_variance.begin(),
            context.attitude_variance.end()
        )) > context.config.max_attitude_sigma_rad
    ) {
        flags |= FSW_FAULT_NAV_UNCERTAINTY;
    }
    if (input.platform.valid) {
        context.previous_execution_time_s =
            input.platform.previous_execution_time_s;
        const bool overrun = input.platform.deadline_missed
            || input.platform.previous_execution_time_s
                > context.config.loop_deadline_s;
        context.consecutive_overruns = overrun
            ? context.consecutive_overruns + 1
            : 0;
        if (context.consecutive_overruns > 0) {
            flags |= FSW_FAULT_DEADLINE_OVERRUN;
        }
        if (!input.platform.watchdog_healthy) {
            flags |= FSW_FAULT_WATCHDOG;
        }
    }
    flags |= context.active_fault_flags & (
        FSW_FAULT_LAUNCH_NOT_CONFIRMED
        | FSW_FAULT_SEPARATION_NOT_CONFIRMED
        | FSW_FAULT_STAGE2_IGNITION
        | FSW_FAULT_DROGUE_NOT_CONFIRMED
        | FSW_FAULT_MAIN_NOT_CONFIRMED
        | FSW_FAULT_INPUT_TIMING
    );
    for (uint32_t index = 0; index < FSW_FAULT_COUNT; ++index) {
        const uint32_t bit = 1u << index;
        if (flags & bit) {
            context.fault_healthy_time_s[index] = 0.0;
        } else if (context.active_fault_flags & bit) {
            context.fault_healthy_time_s[index] += suite.dt_s;
            if (
                context.fault_healthy_time_s[index]
                < context.config.fault_recovery_persistence_s
            ) {
                flags |= bit;
            }
        } else {
            context.fault_healthy_time_s[index] = 0.0;
        }
    }
    commit_faults(context, flags);
}

void update_navigation(
    Context& context,
    const FswInput& input,
    const VotedSensors& voted
) {
    const auto& suite = input.sensors;
    context.attitude_valid = voted.imu_valid;
    if (voted.imu_valid) {
        update_gyro_bias(context, voted, suite.dt_s);
        std::array<double, 3> corrected_gyro{};
        for (int axis = 0; axis < 3; ++axis) {
            corrected_gyro[axis] =
                voted.gyro[axis] - context.gyro_bias[axis];
        }
        integrate_attitude(context, corrected_gyro, suite.dt_s);
        const double attitude_process =
            context.config.gyro_process_sigma_rad_s * suite.dt_s;
        for (double& variance : context.attitude_variance) {
            variance += attitude_process * attitude_process;
        }
    } else {
        for (double& variance : context.attitude_variance) {
            variance += context.config.max_attitude_sigma_rad
                * context.config.max_attitude_sigma_rad
                * suite.dt_s;
        }
    }

    if (!context.navigation_initialized) {
        context.altitude = voted.barometer_valid
            ? voted.barometric_altitude
            : 0.0;
        context.vertical_velocity = voted.gnss_valid
            ? voted.vertical_velocity
            : 0.0;
        if (voted.gnss_valid) {
            context.gnss_radius_reference_m = vector_norm(
                voted.gnss_position.data(), 3
            ) - context.altitude;
            context.gnss_altitude_reference_initialized = true;
        }
        context.last_barometer_sample_time_s = voted.barometer_valid
            ? voted.barometer_sample_time_s
            : -1.0;
        context.last_gnss_sample_time_s = voted.gnss_valid
            ? voted.gnss_sample_time_s
            : -1.0;
        context.navigation_initialized = true;
    } else {
        context.altitude += context.vertical_velocity * suite.dt_s;
        context.altitude_variance +=
            context.velocity_variance * suite.dt_s * suite.dt_s;
        if (voted.imu_valid) {
            context.vertical_velocity +=
                voted.acceleration[0] * suite.dt_s;
            const double velocity_process =
                context.config.accelerometer_process_sigma_m_s2
                * suite.dt_s;
            context.velocity_variance +=
                velocity_process * velocity_process;
        }
    }

    if (
        voted.gnss_valid
        && !context.gnss_altitude_reference_initialized
    ) {
        context.gnss_radius_reference_m = vector_norm(
            voted.gnss_position.data(), 3
        ) - context.altitude;
        context.gnss_altitude_reference_initialized = true;
    }

    bool use_barometer = voted.barometer_valid;
    bool use_gnss = voted.gnss_valid;
    bool navigation_disagreement = false;
    if (
        use_barometer
        && use_gnss
        && context.gnss_altitude_reference_initialized
    ) {
        const double gnss_altitude = vector_norm(
            voted.gnss_position.data(), 3
        ) - context.gnss_radius_reference_m;
        context.gnss_altitude_innovation =
            gnss_altitude - context.altitude;
        context.barometer_innovation =
            voted.barometric_altitude - context.altitude;
        if (
            std::abs(voted.barometric_altitude - gnss_altitude)
            > context.config.cross_altitude_disagreement_m
        ) {
            context.disagreement_flags |=
                FSW_DISAGREEMENT_CROSS_ALTITUDE;
            navigation_disagreement = true;
            const double barometer_innovation = std::abs(
                voted.barometric_altitude - context.altitude
            );
            const double gnss_innovation = std::abs(
                gnss_altitude - context.altitude
            );
            if (
                barometer_innovation
                    <= context.config.cross_altitude_disagreement_m
                && gnss_innovation
                    > context.config.cross_altitude_disagreement_m
            ) {
                use_gnss = false;
            } else if (
                gnss_innovation
                    <= context.config.cross_altitude_disagreement_m
                && barometer_innovation
                    > context.config.cross_altitude_disagreement_m
            ) {
                use_barometer = false;
            } else {
                use_barometer = false;
                use_gnss = false;
            }
        }
    }

    if (
        use_barometer
        && voted.barometer_sample_time_s
            > context.last_barometer_sample_time_s + kEpsilon
    ) {
        const double delta_s = context.last_barometer_sample_time_s < 0.0
            ? suite.dt_s
            : voted.barometer_sample_time_s
                - context.last_barometer_sample_time_s;
        const double alpha = filter_alpha(
            delta_s, context.config.altitude_filter_tau_s
        );
        context.barometer_innovation =
            voted.barometric_altitude - context.altitude;
        context.altitude += alpha * context.barometer_innovation;
        const double measurement_variance =
            context.config.barometer_sigma_m
            * context.config.barometer_sigma_m;
        context.altitude_variance =
            (1.0 - alpha) * (1.0 - alpha)
                * context.altitude_variance
            + alpha * alpha * measurement_variance;
        context.last_barometer_sample_time_s =
            voted.barometer_sample_time_s;
    }
    if (
        use_gnss
        && voted.gnss_sample_time_s
            > context.last_gnss_sample_time_s + kEpsilon
    ) {
        const double delta_s = context.last_gnss_sample_time_s < 0.0
            ? suite.dt_s
            : voted.gnss_sample_time_s - context.last_gnss_sample_time_s;
        const double alpha = filter_alpha(
            delta_s, context.config.velocity_filter_tau_s
        );
        context.gnss_velocity_innovation =
            voted.vertical_velocity - context.vertical_velocity;
        context.vertical_velocity +=
            alpha * context.gnss_velocity_innovation;
        const double measurement_variance =
            context.config.gnss_velocity_sigma_m_s
            * context.config.gnss_velocity_sigma_m_s;
        context.velocity_variance =
            (1.0 - alpha) * (1.0 - alpha)
                * context.velocity_variance
            + alpha * alpha * measurement_variance;
        context.last_gnss_sample_time_s = voted.gnss_sample_time_s;
    }

    context.navigation_status = use_barometer && use_gnss
        ? FSW_NAV_NOMINAL
        : (use_barometer || use_gnss ? FSW_NAV_DEGRADED : FSW_NAV_INERTIAL);
    set_faults(context, input, voted, navigation_disagreement);
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

void raise_fault(Context& context, uint32_t fault) {
    commit_faults(context, context.active_fault_flags | fault);
}

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
        context.config.imu_timeout_s
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
            context.ignition_started_s = input.sensors.time_s;
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
                && input.propulsion.running
                && persisted(
                    voted.acceleration[0] > 2.0,
                    suite.dt_s,
                    0.05,
                    context.launch_evidence_s
                )
            ) {
                context.mode = FSW_MODE_BOOST_1;
            } else if (
                context.ignition_started_s >= 0.0
                && suite.time_s - context.ignition_started_s
                    > context.config.launch_confirm_timeout_s
            ) {
                raise_fault(context, FSW_FAULT_LAUNCH_NOT_CONFIRMED);
                context.mode = FSW_MODE_ABORT;
            }
            break;
        case FSW_MODE_BOOST_1:
            if (
                suite.time_s >= context.config.stage1_burn_s
                && persisted(
                    context.attitude_valid
                        && voted.acceleration[0] < 2.0,
                    suite.dt_s,
                    0.05,
                    context.burnout_evidence_s
                )
            ) {
                context.mode = FSW_MODE_SEPARATION;
                context.separation_started_s = suite.time_s;
            }
            break;
        case FSW_MODE_SEPARATION:
            if (
                discrete_asserted(
                    input.discretes.stage_separated,
                    suite.time_s,
                    context.config.imu_timeout_s
                )
            ) {
                context.mode = FSW_MODE_INTERSTAGE;
                context.stage2_ignition_started_s = -1.0;
            } else if (
                context.separation_started_s >= 0.0
                && suite.time_s - context.separation_started_s
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
                context.stage2_ignition_started_s < 0.0
                && suite.time_s
                    >= context.config.stage1_burn_s
                        + context.config.separation_delay_s
                        + context.config.stage2_ignition_delay_s
            ) {
                context.inhibit_flags = launch_inhibits(
                    context, input, voted
                );
                if (
                    !discrete_asserted(
                        input.discretes.stage_separated,
                        suite.time_s,
                        context.config.imu_timeout_s
                    )
                ) {
                    context.inhibit_flags |= FSW_INHIBIT_SEPARATION;
                }
                if (context.inhibit_flags == 0) {
                    context.stage2_ignite_request = true;
                    context.stage2_ignition_started_s = suite.time_s;
                }
            }
            if (
                context.stage2_ignition_started_s >= 0.0
                && input.propulsion.running
            ) {
                context.mode = FSW_MODE_BOOST_2;
            } else if (
                suite.time_s
                    > context.config.stage1_burn_s
                        + context.config.separation_delay_s
                        + context.config.stage2_ignition_delay_s
                        + context.config.stage2_ignition_timeout_s
            ) {
                raise_fault(context, FSW_FAULT_STAGE2_IGNITION);
                context.mode = FSW_MODE_ABORT;
            }
            break;
        case FSW_MODE_BOOST_2:
            if (
                context.stage2_ignition_started_s >= 0.0
                && suite.time_s
                    >= context.stage2_ignition_started_s
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
        suite.dt_s,
        context.config.imu_loss_abort_delay_s,
        context.imu_loss_evidence_s
    )) {
        context.mode = FSW_MODE_ABORT;
        return;
    }
    if (persisted(
        powered
            && (
                !input.propulsion.valid
                || input.propulsion.health_percent
                    < context.config.propulsion_abort_health_percent
            ),
        suite.dt_s,
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
                    context.config.imu_timeout_s
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
            suite.dt_s,
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
        }
        context.drogue_deployed = discrete_asserted(
            input.discretes.drogue_deployed,
            suite.time_s,
            context.config.imu_timeout_s
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
        }
        context.main_deployed = discrete_asserted(
            input.discretes.main_deployed,
            suite.time_s,
            context.config.imu_timeout_s
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
            suite.dt_s,
            1.0,
            context.landing_evidence_s
        )
    ) {
        context.mode = FSW_MODE_LANDED;
    }
}

FswGuidancePoint guidance_at(const FswConfig& config, double time_s) {
    if (time_s <= config.guidance[0].time_s) {
        return config.guidance[0];
    }
    for (uint32_t index = 1; index < config.guidance_count; ++index) {
        const auto& right = config.guidance[index];
        if (time_s <= right.time_s) {
            const auto& left = config.guidance[index - 1];
            const double fraction = (
                time_s - left.time_s
            ) / (right.time_s - left.time_s);
            return {
                time_s,
                left.pitch_rad
                    + fraction * (right.pitch_rad - left.pitch_rad),
                left.azimuth_rad
                    + fraction * wrap_angle(
                        right.azimuth_rad - left.azimuth_rad
                    ),
            };
        }
    }
    return config.guidance[config.guidance_count - 1];
}

void calculate_controls(
    const Context& context,
    const FswInput& input,
    FswOutput& output
) {
    const auto& suite = input.sensors;
    if (
        !context.attitude_valid
        || (
            context.mode != FSW_MODE_BOOST_1
            && context.mode != FSW_MODE_BOOST_2
        )
    ) {
        return;
    }
    const auto target = guidance_at(context.config, suite.time_s);
    const auto angles = euler(context.attitude);
    const double roll_error = -angles[0];
    const double pitch_error = target.pitch_rad - angles[1];
    const double yaw_error = wrap_angle(
        target.azimuth_rad
        - context.config.guidance[0].azimuth_rad
        - angles[2]
    );
    const double pitch_effort =
        context.config.control_kp * pitch_error
        - context.config.control_kd * context.last_gyro[1];
    const double yaw_effort =
        context.config.control_kp * yaw_error
        - context.config.control_kd * context.last_gyro[2];
    const double roll_effort =
        context.config.control_kp * roll_error
        - context.config.control_kd * context.last_gyro[0];
    const bool air_data_valid = fresh(
        input.air_data.valid,
        input.air_data.sample_time_s,
        suite.time_s,
        context.config.imu_timeout_s
    );
    const double aero_blend = air_data_valid
        ? clamp(
            input.air_data.dynamic_pressure_pa / 35'000.0, 0.0, 1.0
        )
        : 0.0;
    output.tvc_pitch_rad = clamp(
        pitch_effort * (1.0 - 0.65 * aero_blend),
        -context.config.max_tvc_rad,
        context.config.max_tvc_rad
    );
    output.tvc_yaw_rad = clamp(
        yaw_effort * (1.0 - 0.65 * aero_blend),
        -context.config.max_tvc_rad,
        context.config.max_tvc_rad
    );
    output.fin_roll_rad = clamp(
        roll_effort * aero_blend,
        -context.config.max_fin_rad,
        context.config.max_fin_rad
    );
    output.fin_pitch_rad = clamp(
        pitch_effort * aero_blend,
        -context.config.max_fin_rad,
        context.config.max_fin_rad
    );
    output.fin_yaw_rad = clamp(
        yaw_effort * aero_blend,
        -context.config.max_fin_rad,
        context.config.max_fin_rad
    );
}

void populate_output(const Context& context, FswOutput& output) {
    output.abi_version = FSW_ABI_VERSION;
    output.struct_size = sizeof(FswOutput);
    output.output_valid = 1;
    output.step_status = FSW_STATUS_OK;
    output.mode = context.mode;
    output.navigation_status = context.navigation_status;
    output.stage1_ignite = context.stage1_ignite_request;
    output.stage2_ignite = context.stage2_ignite_request;
    output.deploy_drogue = context.drogue_commanded_s >= 0.0;
    output.deploy_main = context.main_commanded_s >= 0.0;
    output.abort = context.mode == FSW_MODE_ABORT;
    output.attitude_valid = context.attitude_valid;
    output.command_sequence = context.command_sequence;
    output.command_type = context.command_type;
    output.command_result = context.command_result;
    output.inhibit_flags = context.inhibit_flags;
    output.event_flags = context.event_flags;
    output.imu_usable_mask = context.imu_usable_mask;
    output.barometer_usable_mask = context.barometer_usable_mask;
    output.gnss_usable_mask = context.gnss_usable_mask;
    output.imu_rejected_mask = context.imu_rejected_mask;
    output.barometer_rejected_mask = context.barometer_rejected_mask;
    output.gnss_rejected_mask = context.gnss_rejected_mask;
    output.disagreement_flags = context.disagreement_flags;
    output.sensor_status_flags = context.sensor_status_flags;
    for (int index = 0; index < FSW_MAX_SENSOR_CHANNELS; ++index) {
        output.imu_health_flags[index] =
            context.imu_health[index].flags;
        output.barometer_health_flags[index] =
            context.barometer_health[index].flags;
        output.gnss_health_flags[index] =
            context.gnss_health[index].flags;
        output.imu_age_s[index] = context.imu_health[index].age_s;
        output.barometer_age_s[index] =
            context.barometer_health[index].age_s;
        output.gnss_age_s[index] = context.gnss_health[index].age_s;
    }
    output.estimated_altitude_m = context.altitude;
    output.estimated_vertical_velocity_m_s = context.vertical_velocity;
    output.altitude_sigma_m = std::sqrt(
        std::max(context.altitude_variance, 0.0)
    );
    output.vertical_velocity_sigma_m_s = std::sqrt(
        std::max(context.velocity_variance, 0.0)
    );
    output.barometer_innovation_m = context.barometer_innovation;
    output.gnss_altitude_innovation_m =
        context.gnss_altitude_innovation;
    output.gnss_velocity_innovation_m_s =
        context.gnss_velocity_innovation;
    for (int index = 0; index < 4; ++index) {
        output.estimated_attitude_wxyz[index] = context.attitude[index];
    }
    for (int index = 0; index < 3; ++index) {
        output.gyro_bias_rad_s[index] = context.gyro_bias[index];
        output.attitude_sigma_rad[index] = std::sqrt(
            std::max(context.attitude_variance[index], 0.0)
        );
    }
    output.active_fault_flags = context.active_fault_flags;
    output.latched_fault_flags = context.latched_fault_flags;
    output.changed_fault_flags = context.changed_fault_flags;
    for (uint32_t index = 0; index < FSW_FAULT_COUNT; ++index) {
        output.fault_occurrence_count[index] =
            context.fault_occurrence_count[index];
    }
    output.highest_fault_severity = context.highest_fault_severity;
    output.previous_execution_time_s =
        context.previous_execution_time_s;
    output.consecutive_overruns = context.consecutive_overruns;
}

}  // namespace

extern "C" {

uint32_t fsw_abi_version(void) {
    return FSW_ABI_VERSION;
}

FswHandle fsw_create(const FswConfig* config) {
    if (config == nullptr || !valid_config(*config)) {
        return nullptr;
    }
    return new (std::nothrow) Context(*config);
}

void fsw_reset(FswHandle handle) {
    if (handle != nullptr) {
        static_cast<Context*>(handle)->reset();
    }
}

int32_t fsw_step(
    FswHandle handle,
    const FswInput* input,
    FswOutput* output
) {
    if (output == nullptr) {
        return FSW_STATUS_INVALID_ARGUMENT;
    }
    *output = {};
    output->abi_version = FSW_ABI_VERSION;
    output->struct_size = sizeof(FswOutput);
    output->output_valid = 0;
    output->step_status = FSW_STATUS_INVALID_ARGUMENT;
    if (handle == nullptr || input == nullptr) {
        return FSW_STATUS_INVALID_ARGUMENT;
    }
    auto& context = *static_cast<Context*>(handle);
    if (
        input->abi_version != FSW_ABI_VERSION
        || input->struct_size != sizeof(FswInput)
    ) {
        output->step_status = FSW_STATUS_ABI_MISMATCH;
        return FSW_STATUS_ABI_MISMATCH;
    }
    const auto& sensor = input->sensors;
    if (
        !valid_input(*input, context.config)
        || (
            context.time_initialized
            && sensor.time_s <= context.last_time_s + kEpsilon
        )
    ) {
        output->step_status = FSW_STATUS_INVALID_INPUT;
        return FSW_STATUS_INVALID_INPUT;
    }

    context.event_flags = 0;
    context.changed_fault_flags = 0;
    context.stage1_ignite_request = false;
    context.stage2_ignite_request = false;
    context.previous_mode = context.mode;
    const VotedSensors voted = vote_sensors(context, sensor);
    update_navigation(context, *input, voted);
    process_command(context, *input, voted);
    update_mode(context, *input, voted);
    if (context.mode != context.previous_mode) {
        context.event_flags |= FSW_EVENT_STATE_CHANGED;
    }
    context.time_initialized = true;
    context.last_time_s = sensor.time_s;
    populate_output(context, *output);
    output->stage_separate =
        context.mode == FSW_MODE_SEPARATION
        && sensor.time_s >= context.config.stage1_burn_s
            + context.config.separation_delay_s;
    calculate_controls(context, *input, *output);
    return FSW_STATUS_OK;
}

void fsw_destroy(FswHandle handle) {
    delete static_cast<Context*>(handle);
}

const char* fsw_version(void) {
    return "fsw-core-0.4.0";
}

}  // extern "C"
