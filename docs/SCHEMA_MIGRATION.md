# Configuration and Schema Migration

Bundled inputs now live under `configs/scenarios/` and `configs/vehicles/`.
The preferred reference scenario is:

```text
configs/scenarios/anthariksa_reference_mission.json
```

The old repository-relative form
`scenarios/anthariksa_reference_mission.json` remains accepted by the CLI and
desktop loader. It resolves to the new location and emits
`SchemaMigrationWarning`. No duplicate compatibility directories or symlinks
are maintained.

## Canonical identifiers

| Document | Canonical schema |
|---|---|
| Scenario | `aerospace-workbench.scenario.v1` |
| Vehicle | `aerospace-workbench.vehicle.v1` |
| Run manifest | `aerospace-workbench.run-manifest.v1` |
| Sensor stream | `aerospace-workbench.sensor-stream.v1` |
| Credibility summary | `aerospace-workbench.credibility.v2` |
| RocketPy reference | `aerospace-workbench.rocketpy-reference.v1` |

## Recognized legacy identifiers

| Legacy schema | Normalized schema |
|---|---|
| `astara.scenario.v0` | `aerospace-workbench.scenario.v1` |
| `astara.scenario.v1` | `aerospace-workbench.scenario.v1` |
| `c1.scenario.v1` | `aerospace-workbench.scenario.v1` |
| `astara.vehicle.v1` | `aerospace-workbench.vehicle.v1` |
| `astara.run.v1` | `aerospace-workbench.run-manifest.v1` |
| `astara.sensor-stream.v1` | `aerospace-workbench.sensor-stream.v1` |
| `astara.credibility.v2` | `aerospace-workbench.credibility.v2` |
| `astara.rocketpy-reference.v1` | `aerospace-workbench.rocketpy-reference.v1` |

Recognized identifiers are normalized in memory and emit a visible
`SchemaMigrationWarning` naming the old identifier, replacement, and source.
Unknown identifiers fail validation. No removal release is currently
scheduled for the recognized legacy forms.

Inline `astara.scenario.v0` vehicle data remains supported. Its runtime form is
normalized and generated evidence separates it into canonical scenario and
vehicle snapshots.

Historical sensor CSV files without a `schema_version` column also remain
replayable. Replay treats them as pre-schema streams, emits one warning per
file, and otherwise preserves the existing deterministic behavior. New sensor
logs include `aerospace-workbench.sensor-stream.v1` in every row.

## Evidence behavior

Generated evidence always uses canonical schema identifiers:

- `scenario.json` and `vehicle_definition.json` are normalized, self-contained
  snapshots.
- `source_scenario.json` and `source_vehicle_definition.json` are byte-for-byte
  copies of file-backed inputs.
- All four files are included in the generated artifact hashes.

Programmatically constructed scenarios have normalized snapshots but no source
files are invented. Updating a legacy configuration in place requires changing
only its recognized schema identifier and, for bundled paths, moving it under
`configs/`; engineering values and example model names do not change.
