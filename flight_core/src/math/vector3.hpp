#pragma once

#include <array>
#include <cmath>

namespace fsw::internal {

inline bool finite_vector(const double* values, int size) {
    for (int index = 0; index < size; ++index) {
        if (!std::isfinite(values[index])) {
            return false;
        }
    }
    return true;
}

inline double vector_distance(const double* left, const double* right, int size) {
    double squared = 0.0;
    for (int index = 0; index < size; ++index) {
        const double delta = left[index] - right[index];
        squared += delta * delta;
    }
    return std::sqrt(squared);
}

inline double vector_norm(const double* values, int size) {
    double squared = 0.0;
    for (int index = 0; index < size; ++index) {
        squared += values[index] * values[index];
    }
    return std::sqrt(squared);
}

inline double radial_velocity(const double* position, const double* velocity) {
    const double radius = vector_norm(position, 3);
    if (radius <= 1e-12) {
        return 0.0;
    }
    return (
        position[0] * velocity[0]
        + position[1] * velocity[1]
        + position[2] * velocity[2]
    ) / radius;
}

inline std::array<double, 3> cross(
    const std::array<double, 3>& left,
    const std::array<double, 3>& right
) {
    return {
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    };
}

inline std::array<double, 3> unit(
    const std::array<double, 3>& vector,
    const std::array<double, 3>& fallback = {1.0, 0.0, 0.0}
) {
    const double length = vector_norm(vector.data(), 3);
    if (length < 1e-12) {
        return fallback;
    }
    return {
        vector[0] / length,
        vector[1] / length,
        vector[2] / length,
    };
}

}  // namespace fsw::internal
