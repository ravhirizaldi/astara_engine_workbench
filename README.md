# ASTARA Engineering Workbench

**ASTARA Engineering Workbench** is offline engineering software for the
Anthariksa two-stage liquid-bipropellant
digital twin and C++ software-in-the-loop flight stack. It includes a full
six-degree-of-freedom truth model, independent core/upper-stage recovery,
simulated sensors and faults, reference-trajectory guidance, TVC plus
aerodynamic control, reproducible evidence files, and the original realtime
combustion-chamber dashboard.

The bundled vehicle uses the Anthariksa family name. Its engine models use the
Cendrawasih series name.

## Engineering Status

The workbench supports vehicle development, architecture trades, sensitivity
analysis, test planning, and flight-software integration. The bundled reference vehicle currently has
`PRELIMINARY_UNVALIDATED` model status because its propulsion, aerodynamics, and
mass-property inputs are provisional.

Startup use is an intended use. Before results support a design release, flight
readiness decision, or safety-critical decision, replace provisional inputs with
configuration-controlled analysis and test data, then complete independent
verification and validation against static-fire, aerodynamic, mass-property,
environmental, and flight-test evidence.

## Files

- `src/aerospace_workbench/` - responsibility-based Python package for configuration, simulation, FSW, evidence, replay, CLI, and presentation
- `flight_core/` - dependency-free C++17 flight-software core and CTest check
- `configs/scenarios/` - versioned SI-unit mission inputs
- `configs/vehicles/` - stable mass, geometry, propulsion, aerodynamics, sensors, and actuators
- `tests/` - Python regression and SIL integration checks
- `main.py` - ASTARA Engineering Workbench launcher
- `engine_bench_ui.py` - standalone launcher for the legacy engine dashboard
- `requirements.txt` - Python dependencies
- `runs/` - generated digital-twin evidence
- `output/` - generated legacy engine PNG/CSV output

## Setup on Ubuntu

Install Python 3 and virtual environment support if needed:

```bash
sudo apt update
sudo apt install python3 python3-venv python3-pip python3-tk
```

Create and activate a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.lock
pip install -e . --no-deps
```

ASTARA is intentionally supported as a repository-run application rather than
a distributable wheel. See [the packaging decision](docs/PACKAGING.md).

## Run

```bash
python3 main.py
```

This opens ASTARA Engineering Workbench. Select a scenario, run the deterministic mission,
and inspect flight-software events and plots.
The Live Mission tab streams bounded telemetry into altitude, speed, thrust, and
ground-track plots while the solver runs. The solver runs in a separate process,
so calculation and report generation cannot block Tk. The All Values view shows
every truth, propulsion, per-engine, actuator, and C++ flight-software field.
Use Maximum, 20×, 5×, or 1× display speed plus Pause and Cancel controls. UI
plots retain at most 6,000 row references and draw at most 800 points per body.
At Maximum speed, stale display snapshots may be dropped to keep the UI current;
the saved CSV data remains complete.

Use `awb` as the preferred CLI:

```bash
awb validate
awb build-fsw
awb simulate --seed 1 --no-report
awb replay runs/<run>/sensors.csv.gz
awb analyze configs/scenarios/anthariksa_reference_mission.json \
  --samples 20 --seed 1
```

The equivalent long command is `aerospace-workbench`. The module entry point
also remains available:

```bash
python3 -m aerospace_workbench simulate --seed 1 --no-report
```

Python integrations should import concrete responsibility modules:

```python
from aerospace_workbench.configuration.scenarios import load_scenario
from aerospace_workbench.simulation.runner import run_simulation

result = run_simulation(load_scenario(), seed=1)
```

Simulation progress appears automatically in an interactive terminal. Use
`--progress` to force newline-delimited progress when redirecting output, or
`--quiet` to suppress it. Progress and mission events use stderr; stdout remains
the run directory followed by the JSON manifest.

FSW host timing is deterministic by default. Use measured timing for profiling,
or inject a repeatable duration to test deadline faults:

```bash
awb simulate --no-report --timing-mode measured
awb simulate --no-report --timing-mode injected \
  --injected-execution-time 0.02
```

Measured timing depends on host load and can change timing faults or mission
state. Injected timing remains repeatable. Replay and analysis runs remain
deterministic. The selected mode is recorded in `manifest.json`; reported
durations are written to `truth.csv` and `fsw.csv`.

Each mission creates a unique `runs/` directory containing:

- `scenario.json` and a SHA-256 scenario identity
- `vehicle_definition.json` as the self-contained stable vehicle snapshot
- `source_scenario.json` and `source_vehicle_definition.json` as byte-for-byte
  copies of file-backed inputs
- `manifest.json` with model version, seed, frames, checks, and artifact hashes
- `truth.csv`, `sensors.csv.gz`, `commands.csv`, `fsw.csv`, and `events.csv`
- `rocketpy_reference.json` with an optional independent powered-ascent comparison
- `report.pdf` and analysis PNGs

The `analyze` command creates a separate credibility bundle containing:

- `convergence.csv` for `dt`, `dt/2`, and `dt/4`
- `monte_carlo.csv` with every sampled factor, seed, status, failure reason,
  max Q, max Mach, apogee, burnout times, and both impact points
- `retained_runs/` with full telemetry for every failure plus a deterministic
  2% sample of successful runs
- `summary.json` with P5/median/P95 ranges, provenance, worker usage, retention
  counts, and convergence status
- normalized scenario/vehicle snapshots, original input copies, and SHA-256
  artifact identities

Monte Carlo samples run in independent processes. Automatic worker selection
uses process CPU affinity and reserves 25% of logical CPUs for the system: an
8-core allocation uses 6 workers. Override this with `--workers`, or change
successful telemetry retention with `--telemetry-percent`.

The C++ flight core receives simulated sensor data only. It has no serial,
network, GPIO, ignition, valve, pyrotechnic, or flight-termination interfaces.
Its public C ABI is intentionally product-neutral: `FswConfig`, `FswInput`,
`FswOutput`, `FSW_MODE_*`, and `fsw_*`. ASTARA remains the
company/workbench name, not the controller namespace.

Replay a recorded sensor stream through the same C++ flight core:

```bash
awb replay runs/<run>/sensors.csv.gz
```

The controller consumes the complete mission attitude schedule with piecewise
pitch and launch-relative azimuth interpolation. Its v0.5 ABI accepts typed
sensor, air-data, propulsion, discrete-feedback, command, and platform-timing
inputs. It votes up to three independently timestamped accelerometer/gyro,
magnetometer, barometer, and GNSS channels, propagates a minimal ECEF
navigation state, emits sequence-identified one-shot recovery commands,
performs deterministic rejection/recovery, and reports channel health,
uncertainty, ECEF estimates, command inhibits, timing, and active/latched
faults. Existing
single-sensor logs remain replayable through the Python bridge. Recorded
commands replay from `commands.csv`; older runs use the legacy deterministic
launch schedule. The upper stage keeps the integrated-stack controller state
through separation.

Current controller limitations remain explicit:

- attitude estimation remains Earth-rate-compensated gyro integration without
  calibrated absolute magnetometer correction;
- uncertainty is a bounded diagonal estimate, not a validated navigation filter;
- the C ABI is loaded in-process with `ctypes`, without transport fault testing;
- timing/memory budgets, cross-platform baselines, HIL evidence, and controller
  gain validation against physical data remain future verification work.

See `docs/REQUIREMENTS.md` for requirement-to-test links,
`docs/SCENARIO_CATALOG.md` for current qualification cases, and
`docs/ARCHITECTURE.md` for the current data and process boundaries.

## Scenario and Vehicle Definition

Scenario files for ASTARA Engineering Workbench contain run-dependent inputs only: vehicle reference,
simulation duration/rate/seed, Monte Carlo settings, launch environment, mission
events, faults, uncertainty, and validation settings. Stable hardware lives in a
separate `aerospace-workbench.vehicle.v1` file:

```json
{
  "schema_version": "aerospace-workbench.scenario.v1",
  "vehicle_definition": "../vehicles/anthariksa_reference_vehicle.json"
}
```

The loader resolves both files before simulation. Run evidence writes separate,
self-contained `scenario.json` and `vehicle_definition.json` snapshots with
canonical schema identifiers while preserving the original files separately.
Recognized legacy schemas remain readable with a deprecation warning. See
[`docs/SCHEMA_MIGRATION.md`](docs/SCHEMA_MIGRATION.md) for the migration table
and compatibility behavior.

## Digital Twin Scope

- ECEF translation, body quaternions, rigid-body angular dynamics, rotating
  Earth, gravity, layered atmosphere through 100 km, and NED report views
- two stages with single engines or explicit multi-engine clusters, replaceable
  propulsion performance curves, per-engine position/direction/scale, engine-out
  faults, and
  propellant-fraction-dependent mass, CG, and inertia
- replaceable Mach-dependent aerodynamic coefficient tables, static stability,
  control derivatives, Mach/AoA validity warnings, wind, launch rail, and
  separation impulse
- independent core-stage and upper-stage state after separation
- drogue/main recovery and landing detection for both stages
- one to three virtual channels with independent bias, noise, schedule, retained
  sample, timestamp, and composable dropout/stale/freeze/stuck-valid/bias/scale
  faults
- 200 Hz C++ mission logic, estimation, reference guidance, TVC, movable-fin
  control allocation, recovery commands, and fault flags

The bundled scenario is an internal software reference, not a real vehicle
design. Replace estimated inputs with reviewed analysis and test evidence before
using results for engineering decisions.

RocketPy runs only as a lazy reference backend when
`reference_backends.rocketpy.enabled` is true. It compares core-stage powered
ascent using the same provisional thrust and drag inputs. Agreement between both
solvers is a software cross-check, not physical model validation.

Each stage accepts an engine cluster:

```json
"engines": [
  {
    "id": "cendrawasih-core-left",
    "model": "Cendrawasih Core-Cluster",
    "position_body_m": [0.0, -0.25, 0.0],
    "direction_body": [1.0, 0.0, 0.0],
    "performance_scale": 0.5,
    "enabled": true,
    "gimbal_enabled": true
  }
]
```

Multiple entries share the stage propulsion curve. Their forces and moments are
calculated independently, so an asymmetric cutoff produces asymmetric torque.

Mission sequencing uses triggerable events:

```json
"events": [
  {
    "event": "stage_separation",
    "trigger": "burnout_stage_1",
    "delay": 0.5
  },
  {
    "event": "stage2_ignition",
    "trigger": "stage_separation",
    "delay": 1.0
  }
]
```

Events may be reordered in the file; trigger dependencies determine their
timing. Duplicate events, negative delays, missing triggers, and cycles are
rejected. Existing scenarios using `separation_delay_s` and
`stage2_ignition_delay_s` remain supported.

See `docs/MODEL_CREDIBILITY.md` for the model-evidence contract, verification
matrix, limitations, and the required path from provisional inputs to reviewed
engineering data.

## Engine Bench

Run the standalone realtime chamber dashboard directly:

```bash
python3 engine_bench_ui.py
```

Use its sliders to adjust simulation values, then use:

- `Start` - begin realtime simulation
- `Pause` - pause at the current timestep
- `Reset` - restart with current slider values
- `EMERGENCY SHUTDOWN` - stop generated propellant gas and continue chamber blowdown
- `Save PNGs` - save the current graph state into `output/`
- `Export CSV` - save all current timestep data to `output/simulation_data.csv`

The dashboard also includes live and peak measurements, a color-coded engine
state banner, pressure/temperature limit lines, a timestamped event log, and an
Engine Health panel. Health, cooling efficiency, wall stress, nozzle erosion,
and combustion instability update during the run. Slider changes during an
active or paused run are applied after Reset.

Default provisional visualization thresholds are configured in
`src/aerospace_workbench/presentation/desktop/engine_bench.py`:

- temperature warning / critical / failure: `3200 / 3700 / 4200 K`
- pressure warning / critical / failure: `1.0 / 1.6 / 2.2 MPa`

These thresholds are visualization settings only, not real propulsion safety
limits. Warning-band degradation is mild; critical-band degradation accelerates
nonlinearly. Sustained exposure causes more damage than a brief spike.

Emergency shutdown stops generated mass flow and allows chamber pressure,
temperature, thrust, and instability to decay. Pressure/temperature hard-limit
failure is suppressed during shutdown, while health depletion and 100%
instability still cause simulated failure.

If the GUI window cannot open, the script runs the default simulation, saves PNG
charts, and falls back to opening them with an available desktop file viewer such
as `wslview`, `xdg-open`, or `explorer.exe`.

To skip opening image windows and only save PNG files:

```bash
ASTARA_NO_GUI=1 ASTARA_NO_OPEN=1 python3 engine_bench_ui.py
```

If you use WSL with GUI support and the window does not open, confirm Tk support:

```bash
sudo apt install python3-tk
```

If the virtual environment was created before installing `python3-tk`, recreate
the environment and reinstall dependencies.

To confirm WSL can create Tk windows:

```bash
python3 -c "import tkinter as tk; root = tk.Tk(); root.destroy(); print('Tk works')"
```

If this prints `couldn't connect to display`, restart WSL from PowerShell:

```powershell
wsl --shutdown
```

Then reopen Ubuntu and run the project again.

## What the Simulation Tracks

- Chamber pressure: estimated pressure from gas mass, temperature, chamber
  volume, and simplified ideal gas behavior.
- Chamber temperature: simplified chamber thermal response during ignition,
  steady burn, shutdown, and post-burn cooling.
- Mass flow in: generated combustion gas flow entering the chamber. It ramps up
  during ignition, holds steady, then ramps down during shutdown.
- Mass flow out: simplified nozzle exhaust flow based on pressure above ambient.
- Estimated thrust: rough estimate from exhaust momentum plus a pressure-area
  term at the nozzle exit.
- Engine health: accumulated simplified damage from sustained pressure and
  temperature stress.
- Cooling efficiency: remaining simplified thermal-management effectiveness.
- Wall stress: accumulated pressure-related chamber loading.
- Nozzle erosion: slow wear after pressure stays above warning for two seconds;
  erosion subtly reduces effective exhaust velocity.
- Instability: simplified pressure/mass-flow interaction that adds small,
  deterministic pressure and thrust oscillations above 50%.

`Save PNGs` writes six charts, including `output/engine_health.png`. `Export
CSV` writes telemetry, degradation values, effective exhaust velocity, status,
limiting factor, and recommendation to `output/simulation_data.csv`.

## Tuning

Edit constants near the top of
`src/aerospace_workbench/presentation/desktop/engine_bench.py`:

- `CHAMBER_VOLUME`
- `COMBUSTION_TEMPERATURE`
- `GAS_CONSTANT`
- `BURN_DURATION`
- `PROPELLANT_MASS_FLOW_RATE`
- `NOZZLE_COEFFICIENT`
- `EXHAUST_VELOCITY`
- `NOZZLE_EXIT_AREA`
- `AMBIENT_PRESSURE`
- `TIME_STEP`

## Intended Use and Validation Status

This repository is intended for aerospace startup research and development. It
supports simulation, software integration, requirements development, trade
studies, Monte Carlo analysis, and planning engineering tests.

Model maturity is tracked per scenario and vehicle input. The included reference
data is preliminary; therefore its numerical results are engineering estimates,
not released design allowables or qualification evidence. Promotion to
flight-decision use requires reviewed source data, uncertainty bounds,
requirements-based verification, correlation against physical tests, documented
acceptance criteria, independent review, and the applicable Indonesian
regulatory and range-safety approvals.
