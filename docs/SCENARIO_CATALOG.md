# ASTARA Scenario Catalog

| ID | Scenario file | Injected condition | Expected evidence |
|---|---|---|---|
| AST-SCN-001 | `anthariksa_reference_mission.json` | None | 215 km insertion target, 20 kg payload deployment, and 60 s payload propagation |
| AST-SCN-002 | `anthariksa_gnss_dropout.json` | Timeline drops all upper-stage GNSS channels for 30 s after stage-2 ignition | GNSS acquisition state, unusable masks, degraded navigation, and reacquisition |
| AST-SCN-003 | `anthariksa_imu_disagreement.json` | Timeline biases IMU channel 2 for 6 s after launch | Channel disagreement and rejection while healthy channels remain |
| AST-SCN-004 | `anthariksa_upper_engine_cutoff.json` | Timeline forces the upper-stage engine off for 10 s after ignition | Missing insertion/deployment and an off-nominal trajectory |

The deterministic seed-1 reference run currently remains an honest `FAIL`:
it reaches 100.6 km but does not enter orbit or deploy the payload. The 215 km
target is retained while the provisional propulsion, mass, and control data
await calibration.

All files are independent variants of the nominal mission and reference the
same vehicle definition. Scenario schema v2 uses a typed `mission.timeline`;
fault start and stop are timeline actions, not embedded timestamps. Run one
with:

```bash
awb validate configs/scenarios/anthariksa_gnss_dropout.json
awb simulate configs/scenarios/anthariksa_gnss_dropout.json --no-report
```
