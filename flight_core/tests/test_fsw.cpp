#include "astara_fsw.h"

#include <cassert>
#include <cmath>

int main() {
    AstaraFswConfig config{};
    config.stage1_burn_s = 1.0;
    config.separation_delay_s = 0.2;
    config.stage2_ignition_delay_s = 0.2;
    config.stage2_burn_s = 1.0;
    config.main_deploy_altitude_m = 100.0;
    config.max_tvc_rad = 0.1;
    config.max_fin_rad = 0.2;
    config.control_kp = 0.7;
    config.control_kd = 0.2;
    config.pitch_start_s = 0.2;
    config.pitch_end_s = 1.5;
    config.max_pitch_rad = 0.2;
    config.target_azimuth_rad = 0.0;

    AstaraFswHandle handle = astara_fsw_create(&config);
    assert(handle != nullptr);
    AstaraFswOutput output{};
    AstaraSensorFrame sensor{};
    sensor.dt_s = 0.01;
    sensor.barometer_valid = 1;
    sensor.gnss_valid = 1;
    sensor.engine_health_percent = 100.0;
    sensor.magnetic_body[0] = 1.0;

    for (int step = 0; step < 180; ++step) {
        sensor.time_s = step * sensor.dt_s;
        sensor.barometric_altitude_m = step * 0.5;
        sensor.vertical_velocity_m_s = 50.0;
        if (sensor.time_s > 1.25) {
            sensor.stage_separated = 1;
        }
        assert(astara_fsw_step(handle, &sensor, &output) == 0);
    }
    assert(output.mode == ASTARA_BOOST_2);
    assert(std::abs(output.tvc_pitch_rad) <= config.max_tvc_rad);
    assert(std::abs(output.fin_pitch_rad) <= config.max_fin_rad);
    assert(output.estimated_attitude_wxyz[0] > 0.99);
    astara_fsw_destroy(handle);
    return 0;
}
