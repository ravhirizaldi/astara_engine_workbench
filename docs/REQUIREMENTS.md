# ASTARA Engineering Workbench — Foundation Requirements

These requirements define the current software gate. They do not qualify
hardware or authorize flight.

| ID | Requirement | Verification |
|---|---|---|
| AST-SIM-001 | Identical scenario, source, and seed shall produce identical numerical telemetry. | `tests.test_twin.TwinTests.test_short_two_stage_run_is_finite_and_reproducible` |
| AST-ARC-001 | Truth, sensor readings, and flight-software estimates shall remain distinct data products. | `truth.csv`, `sensors.csv.gz`, `fsw.csv` plus `tests.test_twin` |
| AST-FSW-001 | Flight core shall consume only `AstaraSensorFrame`; it shall not access simulator state or hardware. | C ABI review and `tests.test_flight_core` |
| AST-FSW-002 | Flight core shall use explicit time input and perform no dynamic allocation after initialization. | C++ review and `flight_core/tests/test_fsw.cpp` |
| AST-EVT-001 | Launch, burnout, apogee, and landing evidence shall persist across multiple samples before transition. | `flight_core/tests/test_fsw.cpp` |
| AST-RPL-001 | A recorded `sensors.csv.gz` shall replay deterministically through the same C++ flight core. | `tests.test_replay` |
| AST-DAT-001 | Each persisted run shall record scenario, vehicle, seed, source hash, and artifact hashes. | `tests.test_twin` |
| AST-VER-001 | Python and C++ regression tests shall run in CI on every proposed change. | `.github/workflows/ci.yml` |
| AST-SAF-001 | Workbench shall expose no ignition, valve, pyrotechnic, GPIO, serial, or flight-termination hardware interface. | Architecture and interface review |

## Known open requirements

- Replace host `ctypes` calls with a versioned inter-process transport.
- Add communication loss, delay, corruption, duplicate, and out-of-order tests.
- Add sensor freshness, disagreement, uncertainty, and degraded-mode outputs.
- Add cross-build, timing budget, memory budget, and HIL interface specification.
