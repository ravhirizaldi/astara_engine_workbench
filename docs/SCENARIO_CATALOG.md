# ASTARA Scenario Catalog

| ID | Scenario file | Injected condition | Expected evidence |
|---|---|---|---|
| AST-SCN-001 | `anthariksa_reference_mission.json` | None | Nominal software reference |
| AST-SCN-002 | `anthariksa_gnss_dropout.json` | All upper-stage GNSS channels drop out from T+10 s through T+40 s | GNSS unusable masks and degraded navigation |
| AST-SCN-003 | `anthariksa_imu_disagreement.json` | IMU channel 2 receives a large bias from T+1 s through T+7 s | Channel disagreement and rejection while healthy channels remain |
| AST-SCN-004 | `anthariksa_upper_engine_cutoff.json` | Upper-stage engine is cut off from scheduled ignition through burnout | Zero upper-stage thrust and off-nominal trajectory |

All files are independent variants of the nominal mission and reference the
same vehicle definition. Run one with:

```bash
awb validate configs/scenarios/anthariksa_gnss_dropout.json
awb simulate configs/scenarios/anthariksa_gnss_dropout.json --no-report
```
