# ASTARA Engineering Workbench — Foundation Requirements

These requirements define the current software gate. They do not qualify
hardware or authorize flight.

| ID | Requirement | Verification |
|---|---|---|
| AST-SIM-001 | Identical scenario, source, and seed shall produce identical numerical telemetry. | `tests.test_twin.TwinTests.test_short_two_stage_run_is_finite_and_reproducible` |
| AST-SIM-002 | Stage separation shall apply equal and opposite configured impulse, preserve mass and linear/angular momentum, and account for the resulting kinetic-energy increase. | `tests.test_twin.TwinTests.test_separation_conserves_mass_and_momentum` |
| AST-SIM-003 | The launch rail shall use scenario-defined hold-down release, friction, rail-button constraints, and a one-shot rail-exit event. | `tests.test_twin.TwinTests.test_launch_rail_releases_and_emits_exit_once` |
| AST-SIM-004 | Continuous actuators shall model configured command delay, first- or second-order response, rate and position limits, deadband, backlash, position feedback, current/power limits, and fault modes. | `tests.test_twin.TwinTests.test_actuator_delay_response_feedback_and_power_limit` |
| AST-ARC-001 | Truth, sensor readings, and flight-software estimates shall remain distinct data products. | `truth.csv`, `sensors.csv.gz`, `fsw.csv` plus `tests.test_twin` |
| AST-UI-001 | The desktop solver shall run outside the GUI process and exchange compact samples through a bounded queue without changing persisted simulation evidence. | `tests.test_ui.LiveUiTests.test_solver_process_streams_and_finishes` |
| AST-UI-002 | Live plots shall use bounded display history, preserve the newest sample, and leave full-resolution evidence ownership with the solver worker. | `tests.test_ui.LiveUiTests.test_live_view_has_three_lightweight_telemetry_panels` and `test_stream_bounds_history_but_draws_latest_point` |
| AST-UI-003 | Operator fault commands shall be validated across the process boundary, reuse the simulation fault models, emit injection and clearing evidence, and leave saved configuration unchanged. | `tests.test_ui.LiveUiTests.test_fault_dock_queues_valid_engine_fault` and `tests.test_twin.TwinTests.test_live_fault_control_injects_clears_and_leaves_scenario_clean` |
| AST-FSW-001 | Flight core shall consume only the versioned `FswInput` C ABI; it shall not access simulator state or hardware. | C ABI review and `tests.test_flight_core` |
| AST-FSW-002 | Flight core shall use explicit time input and perform no dynamic allocation after initialization. | C++ review and `flight_core/tests/unit/` |
| AST-FSW-003 | Flight core shall consume every validated guidance point and interpolate pitch plus launch-relative azimuth between points. | `flight_core/tests/unit/test_control.cpp` |
| AST-FSW-004 | Held, invalid, or stale barometer/GNSS samples shall not be fused as new measurements; navigation degradation shall be reported. | `flight_core/tests/unit/test_navigation.cpp` plus `tests.test_twin` |
| AST-FSW-005 | Upper-stage flight-software state shall remain continuous across simulated stage separation. | `tests.test_twin` |
| AST-FSW-006 | Flight core shall vote up to three channels per sensor family, debounce rejection/recovery on new samples, and expose usable/rejected masks plus disagreement flags. | `flight_core/tests/unit/test_sensor_voting.cpp` |
| AST-FSW-007 | Flight core shall inhibit ignition without a usable IMU, inhibit attitude control while attitude is invalid, and enter abort after confirmed total IMU loss during powered flight. | `flight_core/tests/unit/test_faults.cpp` |
| AST-FSW-008 | SAFE shall require explicit ARM and LAUNCH commands; ignition requests shall be one-shot, inhibited when prerequisites fail, and confirmed through propulsion feedback. | `flight_core/tests/unit/test_mission.cpp` plus `tests.test_flight_core` |
| AST-FSW-009 | Air data, propulsion status, recovery feedback, command input, and platform timing shall cross typed, timestamped ABI fields rather than sensor placeholders. | C ABI review and `tests.test_flight_core` |
| AST-FSW-010 | Flight core shall expose active, latched, changed, counted, and severity-ranked faults; transient faults shall require persistent healthy evidence to clear and critical history shall require reset. | `flight_core/tests/unit/test_faults.cpp` |
| AST-FSW-011 | Flight core shall report bounded altitude, velocity, and attitude uncertainty plus measurement innovations and shall inhibit guarded events when configured uncertainty limits are exceeded. | `flight_core/tests/unit/test_navigation.cpp` |
| AST-FSW-012 | Flight core shall reject invalid ABI, non-monotonic time, and out-of-range step input without changing controller state or emitting non-zero actuator commands. | `flight_core/tests/unit/test_validation.cpp` and `flight_core/tests/integration/test_fsw_api.cpp` |
| AST-FSW-013 | Platform deadline and watchdog status shall be observable as diagnostic faults and repeated overruns shall drive the configured safe response. | `flight_core/tests/unit/test_faults.cpp` |
| AST-FSW-014 | Navigation shall propagate body-to-ECEF attitude from inertial gyro rates with Earth-rate compensation, and ECEF position and velocity from attitude-rotated body specific force plus the same gravity and rotating-Earth model as the twin; altitude and vertical velocity shall be derived from ECEF state. | `flight_core/tests/unit/test_navigation.cpp` and `flight_core/tests/unit/test_attitude.cpp` |
| AST-FSW-015 | An IMU sample shall be propagated only once, using its positive sample-time delta; repeated or older timestamps shall not change attitude or velocity. | `flight_core/tests/unit/test_navigation.cpp` |
| AST-FSW-016 | Magnetometer validity, voting, masks, and disagreement shall remain independent of accelerometer and gyro usability. | `flight_core/tests/unit/test_sensor_voting.cpp` |
| AST-FSW-017 | Stage and recovery delays shall be relative to confirmed mission events, and separation/drogue/main requests shall carry one-shot sequence identities while confirmation remains pending. | `flight_core/tests/unit/test_mission.cpp` |
| AST-FSW-018 | Air-data, propulsion, discrete-feedback, and platform freshness shall use subsystem-specific timeouts; a configured input step-time mismatch shall raise `FSW_FAULT_INPUT_TIMING`. | `flight_core/tests/unit/test_faults.cpp` |
| AST-EVT-001 | Launch, burnout, apogee, and landing evidence shall persist across multiple samples before transition. | `flight_core/tests/unit/test_mission.cpp` |
| AST-RPL-001 | Recorded multi-channel `sensors.csv.gz` and `commands.csv` using the current schema shall replay deterministically through the same C++ flight core. | `tests.test_replay` |
| AST-ANL-001 | Monte Carlo samples shall preserve seeded results when executed serially or in independent worker processes, with automatic execution reserving 25 percent of available logical CPUs. | `tests.test_analysis` |
| AST-ANL-002 | Credibility analysis shall persist full telemetry for every failed sample and a deterministic configured percentage of successful samples while retaining compact results for every sample. | `tests.test_analysis` |
| AST-DAT-001 | Each persisted run shall record scenario, vehicle, seed, source hash, and artifact hashes. | `tests.test_twin` |
| AST-VER-001 | Python and C++ regression tests shall run in CI on every proposed change. | `.github/workflows/ci.yml` |
| AST-SAF-001 | Workbench shall expose no ignition, valve, pyrotechnic, GPIO, serial, or flight-termination hardware interface. | Architecture and interface review |

## Known open requirements

- Replace host `ctypes` calls with a versioned inter-process transport.
- Add communication loss, delay, corruption, duplicate, and out-of-order tests.
- Add cross-build, timing budget, memory budget, and HIL interface specification.
- Replace the diagonal uncertainty baseline with a validated navigation filter
  and calibrated attitude-reference correction.
