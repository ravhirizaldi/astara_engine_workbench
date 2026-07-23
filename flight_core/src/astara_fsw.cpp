#include "astara_fsw.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <new>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr uint32_t kFaultGnssLost = 1u << 0;
constexpr uint32_t kFaultBarometerLost = 1u << 1;
constexpr uint32_t kFaultEngineHealth = 1u << 2;

double clamp(double value, double low, double high) {
    return std::max(low, std::min(value, high));
}

struct Context {
    explicit Context(const AstaraFswConfig& supplied) : config(supplied) { reset(); }

    void reset() {
        mode = config.body_role == 1
            ? ASTARA_COAST
            : (config.body_role == 2 ? ASTARA_INTERSTAGE : ASTARA_SAFE);
        altitude = 0.0;
        vertical_velocity = 0.0;
        attitude = {1.0, 0.0, 0.0, 0.0};
        last_gyro = {0.0, 0.0, 0.0};
        apogee_seen = false;
        drogue_deployed = false;
        main_deployed = false;
        fault_flags = 0;
        navigation_initialized = false;
    }

    AstaraFswConfig config;
    int32_t mode{};
    double altitude{};
    double vertical_velocity{};
    std::array<double, 4> attitude{};
    std::array<double, 3> last_gyro{};
    bool apogee_seen{};
    bool drogue_deployed{};
    bool main_deployed{};
    uint32_t fault_flags{};
    bool navigation_initialized{};
};

void normalize(std::array<double, 4>& quaternion) {
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

void integrate_attitude(Context& context, const AstaraSensorFrame& sensor) {
    const auto& q = context.attitude;
    const double gx = sensor.gyro_body_rad_s[0];
    const double gy = sensor.gyro_body_rad_s[1];
    const double gz = sensor.gyro_body_rad_s[2];
    const double half_dt = 0.5 * sensor.dt_s;
    context.attitude = {
        q[0] + (-q[1] * gx - q[2] * gy - q[3] * gz) * half_dt,
        q[1] + (q[0] * gx + q[2] * gz - q[3] * gy) * half_dt,
        q[2] + (q[0] * gy - q[1] * gz + q[3] * gx) * half_dt,
        q[3] + (q[0] * gz + q[1] * gy - q[2] * gx) * half_dt,
    };
    normalize(context.attitude);
    context.last_gyro = {gx, gy, gz};
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

double wrap_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

void update_navigation(Context& context, const AstaraSensorFrame& sensor) {
    integrate_attitude(context, sensor);
    if (!context.navigation_initialized) {
        context.altitude = sensor.barometer_valid
            ? sensor.barometric_altitude_m
            : 0.0;
        context.vertical_velocity = sensor.gnss_valid
            ? sensor.vertical_velocity_m_s
            : 0.0;
        context.navigation_initialized = true;
        return;
    }
    const double predicted_altitude =
        context.altitude + context.vertical_velocity * sensor.dt_s;
    context.altitude = sensor.barometer_valid
        ? 0.90 * predicted_altitude + 0.10 * sensor.barometric_altitude_m
        : predicted_altitude;
    context.vertical_velocity = sensor.gnss_valid
        ? 0.85 * context.vertical_velocity + 0.15 * sensor.vertical_velocity_m_s
        : context.vertical_velocity
            + sensor.acceleration_body_m_s2[0] * sensor.dt_s;
}

void update_faults(Context& context, const AstaraSensorFrame& sensor) {
    context.fault_flags = 0;
    if (!sensor.gnss_valid) {
        context.fault_flags |= kFaultGnssLost;
    }
    if (!sensor.barometer_valid) {
        context.fault_flags |= kFaultBarometerLost;
    }
    if (sensor.engine_health_percent < 20.0) {
        context.fault_flags |= kFaultEngineHealth;
    }
}

void update_mode(Context& context, const AstaraSensorFrame& sensor) {
    if (sensor.engine_health_percent <= 0.0) {
        context.mode = ASTARA_ABORT;
        return;
    }
    const double burn1 = context.config.stage1_burn_s;
    const double separation = burn1 + context.config.separation_delay_s;
    const double ignition2 = separation + context.config.stage2_ignition_delay_s;
    const double burnout2 = ignition2 + context.config.stage2_burn_s;

    if (context.config.body_role == 1) {
        if (sensor.stage_separated && context.mode < ASTARA_COAST) {
            context.mode = ASTARA_COAST;
        }
    } else if (
        context.config.body_role == 2
        && context.mode == ASTARA_INTERSTAGE
        && sensor.time_s >= ignition2
    ) {
        context.mode = ASTARA_BOOST_2;
    } else if (
        context.config.body_role == 2
        && context.mode == ASTARA_BOOST_2
        && sensor.time_s >= burnout2
    ) {
        context.mode = ASTARA_COAST;
    } else if (context.config.body_role == 0 && context.mode == ASTARA_SAFE && sensor.time_s >= 0.0) {
        context.mode = ASTARA_ARMED;
    } else if (context.config.body_role == 0 && context.mode == ASTARA_ARMED && sensor.time_s >= 0.05) {
        context.mode = ASTARA_IGNITION;
    } else if (context.config.body_role == 0 && context.mode == ASTARA_IGNITION && sensor.time_s >= 0.15) {
        context.mode = ASTARA_BOOST_1;
    } else if (context.config.body_role == 0 && context.mode == ASTARA_BOOST_1 && sensor.time_s >= burn1) {
        context.mode = ASTARA_SEPARATION;
    } else if (context.config.body_role == 0 && context.mode == ASTARA_SEPARATION && sensor.stage_separated) {
        context.mode = ASTARA_INTERSTAGE;
    } else if (context.config.body_role == 0 && context.mode == ASTARA_INTERSTAGE && sensor.time_s >= ignition2) {
        context.mode = ASTARA_BOOST_2;
    } else if (context.config.body_role == 0 && context.mode == ASTARA_BOOST_2 && sensor.time_s >= burnout2) {
        context.mode = ASTARA_COAST;
    }

    if (
        context.mode >= ASTARA_COAST
        && context.mode < ASTARA_APOGEE
        && context.altitude > 100.0
        && context.vertical_velocity < -0.5
    ) {
        context.mode = ASTARA_APOGEE;
        context.apogee_seen = true;
    }
    if (context.apogee_seen && !context.drogue_deployed) {
        context.mode = ASTARA_DROGUE;
        context.drogue_deployed = true;
    }
    if (
        context.drogue_deployed
        && !context.main_deployed
        && context.altitude <= context.config.main_deploy_altitude_m
    ) {
        context.mode = ASTARA_MAIN;
        context.main_deployed = true;
    }
    if (
        context.main_deployed
        && context.altitude <= 2.0
        && std::abs(context.vertical_velocity) < 15.0
    ) {
        context.mode = ASTARA_LANDED;
    }
}

void calculate_controls(
    const Context& context,
    const AstaraSensorFrame& sensor,
    AstaraFswOutput& output
) {
    if (context.mode < ASTARA_BOOST_1 || context.mode > ASTARA_BOOST_2) {
        return;
    }
    const double span = std::max(
        context.config.pitch_end_s - context.config.pitch_start_s, 0.1
    );
    const double fraction = clamp(
        (sensor.time_s - context.config.pitch_start_s) / span, 0.0, 1.0
    );
    const double target_pitch = context.config.max_pitch_rad * fraction;
    const auto angles = euler(context.attitude);
    const double roll_error = -angles[0];
    const double pitch_error = target_pitch - angles[1];
    const double yaw_error = wrap_angle(context.config.target_azimuth_rad - angles[2]);
    const double pitch_effort =
        context.config.control_kp * pitch_error
        - context.config.control_kd * context.last_gyro[1];
    const double yaw_effort =
        context.config.control_kp * yaw_error
        - context.config.control_kd * context.last_gyro[2];
    const double roll_effort =
        context.config.control_kp * roll_error
        - context.config.control_kd * context.last_gyro[0];
    const double aero_blend = clamp(sensor.dynamic_pressure_pa / 35'000.0, 0.0, 1.0);
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

}  // namespace

extern "C" {

AstaraFswHandle astara_fsw_create(const AstaraFswConfig* config) {
    if (config == nullptr) {
        return nullptr;
    }
    return new (std::nothrow) Context(*config);
}

void astara_fsw_reset(AstaraFswHandle handle) {
    if (handle != nullptr) {
        static_cast<Context*>(handle)->reset();
    }
}

int32_t astara_fsw_step(
    AstaraFswHandle handle,
    const AstaraSensorFrame* sensor,
    AstaraFswOutput* output
) {
    if (handle == nullptr || sensor == nullptr || output == nullptr || sensor->dt_s <= 0.0) {
        return -1;
    }
    auto& context = *static_cast<Context*>(handle);
    *output = {};
    update_navigation(context, *sensor);
    update_faults(context, *sensor);
    update_mode(context, *sensor);
    output->mode = context.mode;
    output->stage_separate =
        context.mode == ASTARA_SEPARATION
        && sensor->time_s >= context.config.stage1_burn_s
            + context.config.separation_delay_s;
    output->stage2_ignite = context.mode == ASTARA_BOOST_2;
    output->deploy_drogue = context.drogue_deployed;
    output->deploy_main = context.main_deployed;
    output->abort = context.mode == ASTARA_ABORT;
    output->estimated_altitude_m = context.altitude;
    output->estimated_vertical_velocity_m_s = context.vertical_velocity;
    for (int index = 0; index < 4; ++index) {
        output->estimated_attitude_wxyz[index] = context.attitude[index];
    }
    output->fault_flags = context.fault_flags;
    calculate_controls(context, *sensor, *output);
    return 0;
}

void astara_fsw_destroy(AstaraFswHandle handle) {
    delete static_cast<Context*>(handle);
}

const char* astara_fsw_version(void) {
    return "astara-fsw-0.1.0";
}

}  // extern "C"
