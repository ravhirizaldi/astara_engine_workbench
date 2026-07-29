# ASTARA Foundation Architecture

## Python package boundaries

`aerospace_workbench` uses a standard `src` layout. Configuration and
mathematics are leaf packages; simulation composes the truth, sensor,
actuator, propulsion, aerodynamic, event, and FSW boundaries; evidence and
replay persist or re-execute results; application and presentation are the
outer entry points. The package root exposes version metadata only, so
importing it does not initialize the CLI, solver, Matplotlib, or Tk.

The primary workbench lives under `presentation.desktop.app`. The legacy
Engine Bench remains an independent module under
`presentation.desktop.engine_bench` and is launched separately through
`engine_bench_ui.py`.

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
replace the simulator through `awb replay`.

The sensor suite accepts up to three timestamped accelerometer/gyro,
magnetometer, barometer, and GNSS channels. Magnetometer health is independent
of accelerometer/gyro health; disturbed magnetic data therefore does not remove
a usable inertial channel. The reference emulator gives every virtual channel
its own bias, noise stream, sample schedule, retained sample, timestamp, and
fault state. Flight Core votes fresh channels, debounces rejection and
recovery, reports per-channel reason, age, usable/rejected masks, and
disagreement flags, and falls back to inertial propagation when aiding is
unavailable. Rate checks use the last accepted sample, so a rejected outlier
cannot poison the next channel baseline.

Navigation is a minimal ECEF strapdown solution. The first accepted GNSS
position establishes the radial altitude reference and the configured launch
azimuth establishes the body-to-ECEF attitude. Voted body specific force is
rotated into ECEF. Gyro measurements remain inertial body rates; Earth rate is
rotated into body coordinates and removed only for body-to-ECEF attitude
propagation, and an Earth-fixed launch stack starts with that inertial body
rate. Point-mass Earth gravity and rotating-Earth terms are then applied before
position and velocity integration. Gravity uses
`mu = 3.986004418e14 m^3/s^2`, spherical radius `6378137 m`, and Earth rate
`7.292115e-5 rad/s`, matching the Python twin. Altitude and vertical velocity
are derived from ECEF radius and the local radial unit vector. GNSS vertical
velocity is likewise derived inside Flight Core; simulator truth is not an FSW
input.

The generic C ABI uses `fsw_*`, `Fsw*`, and `FSW_*` names. ASTARA naming remains
at the company workbench, schema, and requirement layers.

Only `flight_core/include/fsw/fsw.h` and `version.h` are public. The exported
shared-library surface is limited to the six C functions declared there.
Controller state and the validation, voting, navigation, mission, fault,
guidance, control, and output modules remain private C++ implementation details
under `flight_core/src/`.

## Current process model

The desktop solver owns the truth model and loads Flight Core through its stable
C ABI. GUI rendering runs separately from the solver. This is host SIL, but not
yet the roadmap's independent-process SIL target.

Guidance points are copied into a fixed 32-point configuration table during
initialization. The upper stage inherits the integrated-stack Flight Core
instance at separation when available, preserving attitude, navigation, and
mission state. A standalone upper controller is also valid: it starts in
INTERSTAGE, needs no state handoff, and uses fresh asserted separation feedback
as its local separation-confirmation epoch.
SAFE requires explicit ARM and LAUNCH commands. Stage-one ignition is a
one-step request confirmed by propulsion feedback. Stage-two ignition remains
automatic but is guarded by separation feedback, navigation, attitude,
propulsion, timing, and uncertainty inhibits.

Mission delays are relative to confirmed events, not absolute simulation time:
stage-one burn begins at ignition confirmation, separation delay begins at
burnout detection, stage-two ignition delay begins at separation confirmation,
and stage-two burn begins at its ignition confirmation. Separation, drogue,
and main outputs are fixed-size one-shot commands with monotonically increasing
sequence identities. The actuator emulator consumes each identity once while
Flight Core continues waiting for timestamped confirmation after the pulse.

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
- Navigation integrates an IMU sample only when its sample timestamp advances,
  using that sample-time delta. Repeated or older timestamps do not propagate
  attitude, position, or velocity.
- A `dt_s`/timestamp-delta mismatch is processed using the timestamp delta and
  raises `FSW_FAULT_INPUT_TIMING`; non-monotonic time remains rejected without
  state mutation.
- Air data, propulsion status, discrete feedback, and platform status use
  separate configured freshness timeouts.
- Flight Core allocates its context once during initialization.
- ABI version/size mismatch and invalid or non-monotonic input are rejected
  before controller state changes; output validity is explicit and actuation
  remains zero.
- The Python host measures native `fsw_step()` duration with a monotonic
  high-resolution clock and supplies it on the following step. This is host
  monitoring only, not hardware real-time qualification. Twin runs default to
  a deterministic zero-duration override, with explicit measured profiling and
  repeatable injected-duration fault-test modes. Replay remains deterministic.
- Platform execution time, deadline misses, watchdog health, navigation
  uncertainty, innovations, ECEF estimates, and fault lifecycle are observable
  outputs.
- UI telemetry and queues are bounded.

## Safety boundary

No module exposes physical ignition, valve, pyrotechnic, GPIO, serial, or
flight-termination control. Hardware integration requires a separate reviewed
adapter and a later roadmap gate.

## Current limitations

- The ECEF estimator is a minimal complementary solution without covariance
  cross-coupling, coning/sculling compensation, Earth ellipsoid, GNSS clock
  modeling, or magnetometer attitude correction.
- Guidance and control remain simple scheduled attitude targets with
  proportional/derivative effort and basic aerodynamic blending.
- There is no hardware abstraction layer or physical I/O implementation.
- Host SIL results are simulation-only and unvalidated; they are not HIL,
  real-time, flight-qualified, or certification evidence.

## Flight Core modules

The public C API owns no flight logic. It validates the opaque handle and
delegates to a private controller. The controller preserves the step order
across private validation, sensor voting, navigation, command, mission, output,
and control modules. Internal headers are available only to the native target
and unit tests; Python continues loading `libfsw_core.so` through `ctypes`.

## Verification

```bash
cmake -S flight_core -B flight_core/build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build flight_core/build-debug --parallel
ctest --test-dir flight_core/build-debug --output-on-failure

cmake -S flight_core -B flight_core/build-release -DCMAKE_BUILD_TYPE=Release
cmake --build flight_core/build-release --parallel
ctest --test-dir flight_core/build-release --output-on-failure

python -m unittest discover -s tests -v
```
