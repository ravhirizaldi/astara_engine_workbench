#pragma once

#include <array>
#include <cmath>

#include "math/quaternion.hpp"
#include "math/vector3.hpp"

namespace fsw::internal {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kEpsilon = 1e-12;
inline constexpr double kEarthMuM3S2 = 3.986004418e14;
inline constexpr double kEarthRadiusM = 6'378'137.0;
inline constexpr double kEarthRotationRadS = 7.292115e-5;

inline double wrap_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

inline std::array<double, 4> launch_attitude(
    const std::array<double, 3>& position_ecef,
    double azimuth_rad
) {
    const auto up = unit(position_ecef);
    const auto east = unit(
        std::array<double, 3>{-up[1], up[0], 0.0},
        {0.0, 1.0, 0.0}
    );
    const auto north = unit(cross(up, east), {0.0, 0.0, 1.0});
    std::array<double, 3> body_y{};
    for (int axis = 0; axis < 3; ++axis) {
        body_y[axis] = std::cos(azimuth_rad) * east[axis]
            - std::sin(azimuth_rad) * north[axis];
    }
    const auto body_z = unit(cross(up, body_y));
    return quaternion_from_matrix({{
        {up[0], body_y[0], body_z[0]},
        {up[1], body_y[1], body_z[1]},
        {up[2], body_y[2], body_z[2]},
    }});
}

}  // namespace fsw::internal
