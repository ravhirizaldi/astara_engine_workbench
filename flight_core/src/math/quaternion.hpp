#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace fsw::internal {

inline void normalize(std::array<double, 4>& quaternion) {
    double norm = 0.0;
    for (double value : quaternion) {
        norm += value * value;
    }
    norm = std::sqrt(norm);
    if (norm < 1e-12) {
        quaternion = {1.0, 0.0, 0.0, 0.0};
        return;
    }
    for (double& value : quaternion) {
        value /= norm;
    }
}

inline std::array<double, 4> multiply(
    const std::array<double, 4>& left,
    const std::array<double, 4>& right
) {
    return {
        left[0] * right[0] - left[1] * right[1]
            - left[2] * right[2] - left[3] * right[3],
        left[0] * right[1] + left[1] * right[0]
            + left[2] * right[3] - left[3] * right[2],
        left[0] * right[2] - left[1] * right[3]
            + left[2] * right[0] + left[3] * right[1],
        left[0] * right[3] + left[1] * right[2]
            - left[2] * right[1] + left[3] * right[0],
    };
}

inline std::array<double, 4> conjugate(
    const std::array<double, 4>& quaternion
) {
    return {
        quaternion[0],
        -quaternion[1],
        -quaternion[2],
        -quaternion[3],
    };
}

inline std::array<double, 3> rotate(
    const std::array<double, 4>& quaternion,
    const std::array<double, 3>& vector
) {
    const std::array<double, 4> pure{
        0.0, vector[0], vector[1], vector[2]
    };
    const auto rotated = multiply(
        multiply(quaternion, pure), conjugate(quaternion)
    );
    return {rotated[1], rotated[2], rotated[3]};
}

inline std::array<double, 4> quaternion_from_matrix(
    const std::array<std::array<double, 3>, 3>& matrix
) {
    std::array<double, 4> result{};
    const double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
    if (trace > 0.0) {
        const double scale = 2.0 * std::sqrt(trace + 1.0);
        result = {
            0.25 * scale,
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
        };
    } else if (
        matrix[0][0] > matrix[1][1]
        && matrix[0][0] > matrix[2][2]
    ) {
        const double scale = 2.0 * std::sqrt(
            1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]
        );
        result = {
            (matrix[2][1] - matrix[1][2]) / scale,
            0.25 * scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
        };
    } else if (matrix[1][1] > matrix[2][2]) {
        const double scale = 2.0 * std::sqrt(
            1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]
        );
        result = {
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            0.25 * scale,
            (matrix[1][2] + matrix[2][1]) / scale,
        };
    } else {
        const double scale = 2.0 * std::sqrt(
            1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]
        );
        result = {
            (matrix[1][0] - matrix[0][1]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            0.25 * scale,
        };
    }
    normalize(result);
    return result;
}

inline std::array<double, 3> euler(const std::array<double, 4>& q) {
    const double roll = std::atan2(
        2.0 * (q[0] * q[1] + q[2] * q[3]),
        1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
    );
    const double pitch = std::asin(std::clamp(
        2.0 * (q[0] * q[2] - q[3] * q[1]), -1.0, 1.0
    ));
    const double yaw = std::atan2(
        2.0 * (q[0] * q[3] + q[1] * q[2]),
        1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
    );
    return {roll, pitch, yaw};
}

}  // namespace fsw::internal
