# ASTARA Scenario Catalog

| ID | Scenario | Evidence |
|---|---|---|
| AST-SCN-001 | Nominal Anthariksa reference mission | `scenarios/anthariksa_reference_mission.json` and `tests.test_twin` |
| AST-SCN-002 | Upper-stage GNSS dropout | `tests.test_twin.TwinTests.test_short_two_stage_run_is_finite_and_reproducible` |
| AST-SCN-003 | Symmetric multi-engine cutoff | `tests.test_twin.TwinTests.test_multi_engine_telemetry_and_engine_cutoff` |
| AST-SCN-004 | Operator cancellation | `tests.test_twin.TwinTests.test_stream_can_cancel_without_waiting_for_batch_result` |
| AST-SCN-005 | Recorded sensor replay | `tests.test_replay.ReplayTests.test_sensor_log_replays_deterministically` |

Fault variants remain code-generated until a scenario requires independent
configuration ownership. This avoids duplicated JSON while the schema evolves.
