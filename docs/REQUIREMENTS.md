# ASTARA Engineering Workbench — Foundation Requirements

These requirements define the current software gate. They do not qualify
hardware or authorize flight.

| ID | Requirement | Verification |
|---|---|---|
| AST-SIM-001 | Identical scenario, source, and seed shall produce identical numerical telemetry. | `tests.test_twin.TwinTests.test_short_two_stage_run_is_finite_and_reproducible` |
| AST-ARC-001 | Truth, sensor readings, and flight-software estimates shall remain distinct data products. | `truth.csv`, `sensors.csv.gz`, `fsw.csv` plus `tests.test_twin` |
| AST-FSW-001 | Flight core shall consume only `FswSensorSuite`; it shall not access simulator state or hardware. | C ABI review and `tests.test_flight_core` |
| AST-FSW-002 | Flight core shall use explicit time input and perform no dynamic allocation after initialization. | C++ review and `flight_core/tests/test_fsw.cpp` |
| AST-FSW-003 | Flight core shall consume every validated guidance point and interpolate pitch plus launch-relative azimuth between points. | `flight_core/tests/test_fsw.cpp` |
| AST-FSW-004 | Held, invalid, or stale barometer/GNSS samples shall not be fused as new measurements; navigation degradation shall be reported. | `flight_core/tests/test_fsw.cpp` plus `tests.test_twin` |
| AST-FSW-005 | Upper-stage flight-software state shall remain continuous across simulated stage separation. | `tests.test_twin` |
| AST-FSW-006 | Flight core shall vote up to three channels per sensor family, debounce rejection/recovery on new samples, and expose usable/rejected masks plus disagreement flags. | `flight_core/tests/test_fsw.cpp` |
| AST-FSW-007 | Flight core shall inhibit ignition without a usable IMU, inhibit attitude control while attitude is invalid, and enter abort after confirmed total IMU loss during powered flight. | `flight_core/tests/test_fsw.cpp` |
| AST-EVT-001 | Launch, burnout, apogee, and landing evidence shall persist across multiple samples before transition. | `flight_core/tests/test_fsw.cpp` |
| AST-RPL-001 | A recorded `sensors.csv.gz` shall replay deterministically through the same C++ flight core. | `tests.test_replay` |
| AST-ANL-001 | Monte Carlo samples shall preserve seeded results when executed serially or in independent worker processes, with automatic execution reserving 25 percent of available logical CPUs. | `tests.test_analysis` |
| AST-ANL-002 | Credibility analysis shall persist full telemetry for every failed sample and a deterministic configured percentage of successful samples while retaining compact results for every sample. | `tests.test_analysis` |
| AST-DAT-001 | Each persisted run shall record scenario, vehicle, seed, source hash, and artifact hashes. | `tests.test_twin` |
| AST-VER-001 | Python and C++ regression tests shall run in CI on every proposed change. | `.github/workflows/ci.yml` |
| AST-SAF-001 | Workbench shall expose no ignition, valve, pyrotechnic, GPIO, serial, or flight-termination hardware interface. | Architecture and interface review |

## Known open requirements

- Replace host `ctypes` calls with a versioned inter-process transport.
- Add communication loss, delay, corruption, duplicate, and out-of-order tests.
- Add uncertainty/covariance estimation and redundant sensor emulation.
- Add cross-build, timing budget, memory budget, and HIL interface specification.
