
#include "validation/config_validation.hpp"

#include <cmath>

#include "math/vector3.hpp"

namespace fsw::internal {

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
        config.magnetometer_timeout_s,
        config.barometer_timeout_s,
        config.gnss_timeout_s,
        config.air_data_timeout_s,
        config.propulsion_status_timeout_s,
        config.discrete_feedback_timeout_s,
        config.platform_status_timeout_s,
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
        config.step_time_tolerance_s,
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
        config.launch_azimuth_rad,
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
        || config.magnetometer_timeout_s <= 0.0
        || config.barometer_timeout_s <= 0.0
        || config.gnss_timeout_s <= 0.0
        || config.air_data_timeout_s <= 0.0
        || config.propulsion_status_timeout_s <= 0.0
        || config.discrete_feedback_timeout_s <= 0.0
        || config.platform_status_timeout_s <= 0.0
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
        || config.step_time_tolerance_s < 0.0
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

}  // namespace fsw::internal
