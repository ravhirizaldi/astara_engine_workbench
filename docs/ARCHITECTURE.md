# ASTARA Foundation Architecture

## Data boundary

```text
Scenario + Vehicle
        |
        v
6-DOF Truth Model --> truth.csv
        |
        v
Sensor Emulator --> sensors.csv.gz --+
Command Schedule --> commands.csv ----+--> typed FswInput --> C++ Flight Core
Truth-derived adapters ----------------+                         |
                                                                 +--> fsw.csv
                                                                 |
                                                                 v
                                                        Actuator Emulator
                                                                 |
                                                                 +--> Truth Model
```

`truth.csv`, `sensors.csv.gz`, `commands.csv`, and `fsw.csv` are deliberately
separate. Flight software receives only the versioned `FswInput` ABI. The
Python bridge maps recorded sensor channels plus typed air-data, propulsion,
recovery-feedback, command, and platform-timing fields, so recorded inputs can
replace the simulator through `python -m astara replay`.

The sensor suite accepts up to three timestamped IMU, barometer, and GNSS
channels. The reference emulator writes all three channels. Flight Core votes
fresh channels, debounces rejection and recovery, reports per-channel reason,
age, usable/rejected masks, and disagreement flags, and falls back to the
remaining source or inertial propagation when aiding is unavailable.

The generic C ABI uses `fsw_*`, `Fsw*`, and `FSW_*` names. ASTARA naming remains
at the company workbench, schema, and requirement layers.

## Current process model

The desktop solver owns the truth model and loads Flight Core through its stable
C ABI. GUI rendering runs separately from the solver. This is host SIL, but not
yet the roadmap's independent-process SIL target.

Guidance points are copied into a fixed 32-point configuration table during
initialization. The upper stage inherits the integrated-stack Flight Core
instance at separation, preserving attitude, navigation, and mission state.
SAFE requires explicit ARM and LAUNCH commands. Stage-one ignition is a
one-step request confirmed by propulsion feedback. Stage-two ignition remains
automatic but is guarded by separation feedback, navigation, attitude,
propulsion, timing, and uncertainty inhibits.

The replayable sensor contract is the migration boundary. A future transport
may move Flight Core into another process without changing mission logic or
recorded sensor semantics.

Credibility analysis precomputes sampled scenarios and seeds in the parent,
then distributes independent Monte Carlo simulations to a bounded process
pool. Workers return compact summaries; full telemetry is rerun and persisted
only for failures and the configured deterministic sample.

## Time and memory

- Scenario time is explicit and independent of wall-clock display pacing.
- Flight Core receives `time_s` and `dt_s` on every step.
- Flight Core allocates its context once during initialization.
- ABI version/size mismatch and invalid or non-monotonic input are rejected
  before controller state changes; output validity is explicit and actuation
  remains zero.
- Platform execution time, deadline misses, watchdog health, navigation
  uncertainty, innovations, and fault lifecycle are observable outputs.
- UI telemetry and queues are bounded.

## Safety boundary

No module exposes physical ignition, valve, pyrotechnic, GPIO, serial, or
flight-termination control. Hardware integration requires a separate reviewed
adapter and a later roadmap gate.
