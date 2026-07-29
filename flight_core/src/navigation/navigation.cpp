
#include "navigation/navigation.hpp"

#include <algorithm>
#include <cmath>

#include "core/context.hpp"
#include "faults/fault_manager.hpp"
#include "math/frames.hpp"
#include "math/quaternion.hpp"
#include "math/vector3.hpp"
#include "navigation/attitude.hpp"
#include "sensors/sensor_voting.hpp"

namespace fsw::internal {

double filter_alpha(double sample_delta_s, double tau_s) {
    return 1.0 - std::exp(-std::max(sample_delta_s, kEpsilon) / tau_s);
}

void update_navigation(
    Context& context,
    const FswInput& input,
    const VotedSensors& voted
) {
    if (!context.navigation_initialized && voted.gnss_valid) {
        context.position_ecef = voted.gnss_position;
        context.velocity_ecef = voted.gnss_velocity;
        context.altitude = voted.barometer_valid
            ? voted.barometric_altitude
            : 0.0;
        context.gnss_radius_reference_m = vector_norm(
            context.position_ecef.data(), 3
        ) - context.altitude;
        context.gnss_altitude_reference_initialized = true;
        context.launch_reference_attitude = launch_attitude(
            context.position_ecef, context.config.launch_azimuth_rad
        );
        context.attitude = context.launch_reference_attitude;
        context.attitude_initialized = true;
        context.navigation_initialized = true;
        context.last_gnss_sample_time_s = voted.gnss_sample_time_s;
        context.last_barometer_sample_time_s = voted.barometer_valid
            ? voted.barometer_sample_time_s
            : -1.0;
        context.last_integrated_imu_sample_time_s = voted.imu_valid
            ? voted.imu_sample_time_s
            : -1.0;
    }

    context.attitude_valid =
        context.attitude_initialized && voted.imu_valid;
    const bool newer_imu = voted.imu_valid
        && context.navigation_initialized
        && voted.imu_sample_time_s
            > context.last_integrated_imu_sample_time_s + kEpsilon;
    if (newer_imu) {
        const double imu_delta_s =
            context.last_integrated_imu_sample_time_s < 0.0
                ? context.step_delta_s
                : voted.imu_sample_time_s
                    - context.last_integrated_imu_sample_time_s;
        update_gyro_bias(context, voted, imu_delta_s);
        std::array<double, 3> corrected_gyro{};
        std::array<double, 3> corrected_acceleration{};
        for (int axis = 0; axis < 3; ++axis) {
            corrected_gyro[axis] =
                voted.gyro[axis] - context.gyro_bias[axis];
            corrected_acceleration[axis] =
                voted.acceleration[axis]
                - context.accelerometer_bias[axis];
        }
        const std::array<double, 3> earth_rate{
            0.0, 0.0, kEarthRotationRadS
        };
        const auto earth_rate_body = rotate(
            conjugate(context.attitude), earth_rate
        );
        std::array<double, 3> ecef_relative_gyro{};
        for (int axis = 0; axis < 3; ++axis) {
            ecef_relative_gyro[axis] =
                corrected_gyro[axis] - earth_rate_body[axis];
        }
        integrate_attitude(context, ecef_relative_gyro, imu_delta_s);
        const auto specific_force_ecef = rotate(
            context.attitude, corrected_acceleration
        );
        const double radius = std::max(
            vector_norm(context.position_ecef.data(), 3),
            kEarthRadiusM
        );
        std::array<double, 3> gravity{};
        for (int axis = 0; axis < 3; ++axis) {
            gravity[axis] = -kEarthMuM3S2
                * context.position_ecef[axis]
                / (radius * radius * radius);
        }
        const auto coriolis = cross(earth_rate, context.velocity_ecef);
        const auto centrifugal = cross(
            earth_rate, cross(earth_rate, context.position_ecef)
        );
        for (int axis = 0; axis < 3; ++axis) {
            const double acceleration_ecef =
                specific_force_ecef[axis]
                + gravity[axis]
                - 2.0 * coriolis[axis]
                - centrifugal[axis];
            context.position_ecef[axis] +=
                context.velocity_ecef[axis] * imu_delta_s
                + 0.5 * acceleration_ecef
                    * imu_delta_s * imu_delta_s;
            context.velocity_ecef[axis] +=
                acceleration_ecef * imu_delta_s;
        }
        context.last_integrated_imu_sample_time_s =
            voted.imu_sample_time_s;
        const double attitude_process =
            context.config.gyro_process_sigma_rad_s * imu_delta_s;
        for (double& variance : context.attitude_variance) {
            variance += attitude_process * attitude_process;
        }
        const double velocity_process =
            context.config.accelerometer_process_sigma_m_s2
            * imu_delta_s;
        context.velocity_variance +=
            velocity_process * velocity_process;
        context.altitude_variance +=
            context.velocity_variance * imu_delta_s * imu_delta_s;
    } else if (!voted.imu_valid) {
        for (double& variance : context.attitude_variance) {
            variance += context.config.max_attitude_sigma_rad
                * context.config.max_attitude_sigma_rad
                * context.step_delta_s;
        }
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
            ? context.step_delta_s
            : voted.barometer_sample_time_s
                - context.last_barometer_sample_time_s;
        const double alpha = filter_alpha(
            delta_s, context.config.altitude_filter_tau_s
        );
        context.barometer_innovation =
            voted.barometric_altitude - context.altitude;
        if (context.navigation_initialized) {
            const auto up = unit(context.position_ecef);
            for (int axis = 0; axis < 3; ++axis) {
                context.position_ecef[axis] +=
                    alpha * context.barometer_innovation * up[axis];
            }
        }
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
            ? context.step_delta_s
            : voted.gnss_sample_time_s - context.last_gnss_sample_time_s;
        const double alpha = filter_alpha(
            delta_s, context.config.velocity_filter_tau_s
        );
        context.gnss_velocity_innovation = voted.vertical_velocity
            - radial_velocity(
                context.position_ecef.data(),
                context.velocity_ecef.data()
            );
        for (int axis = 0; axis < 3; ++axis) {
            context.position_ecef[axis] += alpha * (
                voted.gnss_position[axis]
                - context.position_ecef[axis]
            );
            context.velocity_ecef[axis] += alpha * (
                voted.gnss_velocity[axis]
                - context.velocity_ecef[axis]
            );
        }
        const double measurement_variance =
            context.config.gnss_velocity_sigma_m_s
            * context.config.gnss_velocity_sigma_m_s;
        context.velocity_variance =
            (1.0 - alpha) * (1.0 - alpha)
                * context.velocity_variance
            + alpha * alpha * measurement_variance;
        context.last_gnss_sample_time_s = voted.gnss_sample_time_s;
    }

    if (context.navigation_initialized) {
        context.altitude = vector_norm(
            context.position_ecef.data(), 3
        ) - context.gnss_radius_reference_m;
        context.vertical_velocity = radial_velocity(
            context.position_ecef.data(),
            context.velocity_ecef.data()
        );
    }

    context.navigation_status = context.navigation_initialized
        && use_barometer && use_gnss
        ? FSW_NAV_NOMINAL
        : (
            context.navigation_initialized && (use_barometer || use_gnss)
                ? FSW_NAV_DEGRADED
                : FSW_NAV_INERTIAL
        );
    set_faults(context, input, voted, navigation_disagreement);
}

}  // namespace fsw::internal
