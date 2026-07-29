
#include "navigation/navigation.hpp"

#include <algorithm>
#include <cmath>

#include "core/context.hpp"
#include "math/frames.hpp"
#include "math/quaternion.hpp"
#include "math/vector3.hpp"
#include "navigation/attitude.hpp"
#include "sensors/sensor_voting.hpp"

namespace fsw::internal {

double filter_alpha(double sample_delta_s, double tau_s) {
    return 1.0 - std::exp(-std::max(sample_delta_s, kEpsilon) / tau_s);
}

bool update_navigation(
    Context& context,
    const VotedSensors& voted
) {
    if (!context.navigation.initialized && voted.gnss_valid) {
        context.navigation.position_ecef = voted.gnss_position;
        context.navigation.velocity_ecef = voted.gnss_velocity;
        context.navigation.altitude = voted.barometer_valid
            ? voted.barometric_altitude
            : 0.0;
        context.navigation.gnss_radius_reference_m = vector_norm(
            context.navigation.position_ecef.data(), 3
        ) - context.navigation.altitude;
        context.navigation.gnss_altitude_reference_initialized = true;
        context.navigation.launch_reference_attitude = launch_attitude(
            context.navigation.position_ecef, context.config.launch_azimuth_rad
        );
        context.navigation.attitude = context.navigation.launch_reference_attitude;
        context.navigation.attitude_initialized = true;
        context.navigation.initialized = true;
        context.timing.last_gnss_sample_time_s = voted.gnss_sample_time_s;
        context.timing.last_barometer_sample_time_s = voted.barometer_valid
            ? voted.barometer_sample_time_s
            : -1.0;
        context.timing.last_integrated_imu_sample_time_s = voted.imu_valid
            ? voted.imu_sample_time_s
            : -1.0;
    }

    context.navigation.attitude_valid =
        context.navigation.attitude_initialized && voted.imu_valid;
    const bool newer_imu = voted.imu_valid
        && context.navigation.initialized
        && voted.imu_sample_time_s
            > context.timing.last_integrated_imu_sample_time_s + kEpsilon;
    if (newer_imu) {
        const double imu_delta_s =
            context.timing.last_integrated_imu_sample_time_s < 0.0
                ? context.timing.step_delta_s
                : voted.imu_sample_time_s
                    - context.timing.last_integrated_imu_sample_time_s;
        update_gyro_bias(context, voted, imu_delta_s);
        std::array<double, 3> corrected_gyro{};
        std::array<double, 3> corrected_acceleration{};
        for (int axis = 0; axis < 3; ++axis) {
            corrected_gyro[axis] =
                voted.gyro[axis] - context.navigation.gyro_bias[axis];
            corrected_acceleration[axis] =
                voted.acceleration[axis]
                - context.navigation.accelerometer_bias[axis];
        }
        const std::array<double, 3> earth_rate{
            0.0, 0.0, kEarthRotationRadS
        };
        const auto earth_rate_body = rotate(
            conjugate(context.navigation.attitude), earth_rate
        );
        std::array<double, 3> ecef_relative_gyro{};
        for (int axis = 0; axis < 3; ++axis) {
            ecef_relative_gyro[axis] =
                corrected_gyro[axis] - earth_rate_body[axis];
        }
        integrate_attitude(context, ecef_relative_gyro, imu_delta_s);
        const auto specific_force_ecef = rotate(
            context.navigation.attitude, corrected_acceleration
        );
        const double radius = std::max(
            vector_norm(context.navigation.position_ecef.data(), 3),
            kEarthRadiusM
        );
        std::array<double, 3> gravity{};
        for (int axis = 0; axis < 3; ++axis) {
            gravity[axis] = -kEarthMuM3S2
                * context.navigation.position_ecef[axis]
                / (radius * radius * radius);
        }
        const auto coriolis = cross(earth_rate, context.navigation.velocity_ecef);
        const auto centrifugal = cross(
            earth_rate, cross(earth_rate, context.navigation.position_ecef)
        );
        for (int axis = 0; axis < 3; ++axis) {
            const double acceleration_ecef =
                specific_force_ecef[axis]
                + gravity[axis]
                - 2.0 * coriolis[axis]
                - centrifugal[axis];
            context.navigation.position_ecef[axis] +=
                context.navigation.velocity_ecef[axis] * imu_delta_s
                + 0.5 * acceleration_ecef
                    * imu_delta_s * imu_delta_s;
            context.navigation.velocity_ecef[axis] +=
                acceleration_ecef * imu_delta_s;
        }
        context.timing.last_integrated_imu_sample_time_s =
            voted.imu_sample_time_s;
        const double attitude_process =
            context.config.gyro_process_sigma_rad_s * imu_delta_s;
        for (double& variance : context.navigation.attitude_variance) {
            variance += attitude_process * attitude_process;
        }
        const double velocity_process =
            context.config.accelerometer_process_sigma_m_s2
            * imu_delta_s;
        context.navigation.velocity_variance +=
            velocity_process * velocity_process;
        context.navigation.altitude_variance +=
            context.navigation.velocity_variance * imu_delta_s * imu_delta_s;
    } else if (!voted.imu_valid) {
        for (double& variance : context.navigation.attitude_variance) {
            variance += context.config.max_attitude_sigma_rad
                * context.config.max_attitude_sigma_rad
                * context.timing.step_delta_s;
        }
    }

    bool use_barometer = voted.barometer_valid;
    bool use_gnss = voted.gnss_valid;
    bool navigation_disagreement = false;
    if (
        use_barometer
        && use_gnss
        && context.navigation.gnss_altitude_reference_initialized
    ) {
        const double gnss_altitude = vector_norm(
            voted.gnss_position.data(), 3
        ) - context.navigation.gnss_radius_reference_m;
        context.navigation.gnss_altitude_innovation =
            gnss_altitude - context.navigation.altitude;
        context.navigation.barometer_innovation =
            voted.barometric_altitude - context.navigation.altitude;
        if (
            std::abs(voted.barometric_altitude - gnss_altitude)
            > context.config.cross_altitude_disagreement_m
        ) {
            context.sensors.disagreement_flags |=
                FSW_DISAGREEMENT_CROSS_ALTITUDE;
            navigation_disagreement = true;
            const double barometer_innovation = std::abs(
                voted.barometric_altitude - context.navigation.altitude
            );
            const double gnss_innovation = std::abs(
                gnss_altitude - context.navigation.altitude
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
            > context.timing.last_barometer_sample_time_s + kEpsilon
    ) {
        const double delta_s = context.timing.last_barometer_sample_time_s < 0.0
            ? context.timing.step_delta_s
            : voted.barometer_sample_time_s
                - context.timing.last_barometer_sample_time_s;
        const double alpha = filter_alpha(
            delta_s, context.config.altitude_filter_tau_s
        );
        context.navigation.barometer_innovation =
            voted.barometric_altitude - context.navigation.altitude;
        if (context.navigation.initialized) {
            const auto up = unit(context.navigation.position_ecef);
            for (int axis = 0; axis < 3; ++axis) {
                context.navigation.position_ecef[axis] +=
                    alpha * context.navigation.barometer_innovation * up[axis];
            }
        }
        const double measurement_variance =
            context.config.barometer_sigma_m
            * context.config.barometer_sigma_m;
        context.navigation.altitude_variance =
            (1.0 - alpha) * (1.0 - alpha)
                * context.navigation.altitude_variance
            + alpha * alpha * measurement_variance;
        context.timing.last_barometer_sample_time_s =
            voted.barometer_sample_time_s;
    }
    if (
        use_gnss
        && voted.gnss_sample_time_s
            > context.timing.last_gnss_sample_time_s + kEpsilon
    ) {
        const double delta_s = context.timing.last_gnss_sample_time_s < 0.0
            ? context.timing.step_delta_s
            : voted.gnss_sample_time_s - context.timing.last_gnss_sample_time_s;
        const double alpha = filter_alpha(
            delta_s, context.config.velocity_filter_tau_s
        );
        context.navigation.gnss_velocity_innovation = voted.vertical_velocity
            - radial_velocity(
                context.navigation.position_ecef.data(),
                context.navigation.velocity_ecef.data()
            );
        for (int axis = 0; axis < 3; ++axis) {
            context.navigation.position_ecef[axis] += alpha * (
                voted.gnss_position[axis]
                - context.navigation.position_ecef[axis]
            );
            context.navigation.velocity_ecef[axis] += alpha * (
                voted.gnss_velocity[axis]
                - context.navigation.velocity_ecef[axis]
            );
        }
        const double measurement_variance =
            context.config.gnss_velocity_sigma_m_s
            * context.config.gnss_velocity_sigma_m_s;
        context.navigation.velocity_variance =
            (1.0 - alpha) * (1.0 - alpha)
                * context.navigation.velocity_variance
            + alpha * alpha * measurement_variance;
        context.timing.last_gnss_sample_time_s = voted.gnss_sample_time_s;
    }

    if (context.navigation.initialized) {
        context.navigation.altitude = vector_norm(
            context.navigation.position_ecef.data(), 3
        ) - context.navigation.gnss_radius_reference_m;
        context.navigation.vertical_velocity = radial_velocity(
            context.navigation.position_ecef.data(),
            context.navigation.velocity_ecef.data()
        );
    }

    context.navigation.navigation_status = context.navigation.initialized
        && use_barometer && use_gnss
        ? FSW_NAV_NOMINAL
        : (
            context.navigation.initialized && (use_barometer || use_gnss)
                ? FSW_NAV_DEGRADED
                : FSW_NAV_INERTIAL
        );
    return navigation_disagreement;
}

}  // namespace fsw::internal
