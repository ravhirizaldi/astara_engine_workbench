# ASTARA Engineering Workbench — Foundation Requirements

These requirements define the current software gate. They do not qualify
hardware or authorize flight.

| ID | Requirement | Verification |
|---|---|---|
| AST-SIM-001 | Identical scenario, source, and seed shall produce identical numerical telemetry. | `tests.test_twin.TwinTests.test_short_two_stage_run_is_finite_and_reproducible` |
| AST-ARC-001 | Truth, sensor readings, and flight-software estimates shall remain distinct data products. | `truth.csv`, `sensors.csv.gz`, `fsw.csv` plus `tests.test_twin` |
| AST-FSW-001 | Flight core shall consume only the versioned `FswInput` C ABI; it shall not access simulator state or hardware. | C ABI review and `tests.test_flight_core` |
| AST-FSW-002 | Flight core shall use explicit time input and perform no dynamic allocation after initialization. | C++ review and `flight_core/tests/test_fsw.cpp` |
| AST-FSW-003 | Flight core shall consume every validated guidance point and interpolate pitch plus launch-relative azimuth between points. | `flight_core/tests/test_fsw.cpp` |
| AST-FSW-004 | Held, invalid, or stale barometer/GNSS samples shall not be fused as new measurements; navigation degradation shall be reported. | `flight_core/tests/test_fsw.cpp` plus `tests.test_twin` |
| AST-FSW-005 | Upper-stage flight-software state shall remain continuous across simulated stage separation. | `tests.test_twin` |
| AST-FSW-006 | Flight core shall vote up to three channels per sensor family, debounce rejection/recovery on new samples, and expose usable/rejected masks plus disagreement flags. | `flight_core/tests/test_fsw.cpp` |
| AST-FSW-007 | Flight core shall inhibit ignition without a usable IMU, inhibit attitude control while attitude is invalid, and enter abort after confirmed total IMU loss during powered flight. | `flight_core/tests/test_fsw.cpp` |
| AST-FSW-008 | SAFE shall require explicit ARM and LAUNCH commands; ignition requests shall be one-shot, inhibited when prerequisites fail, and confirmed through propulsion feedback. | `flight_core/tests/test_fsw.cpp` plus `tests.test_flight_core` |
| AST-FSW-009 | Air data, propulsion status, recovery feedback, command input, and platform timing shall cross typed, timestamped ABI fields rather than sensor placeholders. | C ABI review and `tests.test_flight_core` |
| AST-FSW-010 | Flight core shall expose active, latched, changed, counted, and severity-ranked faults; transient faults shall require persistent healthy evidence to clear and critical history shall require reset. | `flight_core/tests/test_fsw.cpp` |
| AST-FSW-011 | Flight core shall report bounded altitude, velocity, and attitude uncertainty plus measurement innovations and shall inhibit guarded events when configured uncertainty limits are exceeded. | `flight_core/tests/test_fsw.cpp` |
| AST-FSW-012 | Flight core shall reject invalid ABI, non-monotonic time, and out-of-range step input without changing controller state or emitting non-zero actuator commands. | `flight_core/tests/test_fsw.cpp` |
| AST-FSW-013 | Platform deadline and watchdog status shall be observable as diagnostic faults and repeated overruns shall drive the configured safe response. | `flight_core/tests/test_fsw.cpp` |
| AST-EVT-001 | Launch, burnout, apogee, and landing evidence shall persist across multiple samples before transition. | `flight_core/tests/test_fsw.cpp` |
| AST-RPL-001 | Recorded multi-channel `sensors.csv.gz` and `commands.csv` shall replay deterministically through the same C++ flight core; legacy single-channel runs shall remain readable. | `tests.test_replay` |
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
