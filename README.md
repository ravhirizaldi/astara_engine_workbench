# ASTARA Engineering Workbench

ASTARA Engineering Workbench is an offline two-stage sounding-rocket digital
twin and software-in-the-loop (SIL) environment. It combines a Python 6-DOF
simulation, configurable vehicle and mission models, simulated sensors and
faults, a C++17 flight-software core, deterministic replay, and evidence
generation.

> **Engineering status:** preliminary and unvalidated. Every run is marked
> `simulation_only: true` and `unvalidated: true`. Results are suitable for
> software integration, sensitivity studies, and test planning—not flight
> safety, hardware qualification, range approval, or certification.

## What the Workbench Provides

| Area | Current capability | Primary evidence |
|---|---|---|
| Vehicle dynamics | ECEF translation, body quaternions, rotating Earth, configurable constrained launch rail, composited stack mass properties, momentum-audited staging, and recovery | `truth.csv`, `events.csv` |
| Vehicle definition | Two stages, engine clusters, tabulated propulsion, mass properties, aerodynamic coefficients, fixed or movable fins, sensors, and actuators | `vehicle_definition.json` |
| Mission definition | Launch environment, command schedule, event-triggered separation and ignition, guidance schedule, faults, and uncertainty | `scenario.json`, `events.csv` |
| Avionics simulation | Independently clocked navigation sensors, air-data computer, engine controller, discrete/recovery modules, flight-computer platform, bus, and FSW task with latency, quantization, startup/reset behavior, faults, and deadline drops | `avionics.csv`, `sensors.csv.gz` |
| Flight software | C++17 mission logic, ECEF navigation, guidance, sensor voting, fault handling, TVC/fin commands, recovery sequencing, and health reporting | `fsw.csv` |
| Replay | Recorded sensor channels and commands replay through the same C++ flight core | `fsw_replay.csv` |
| Analysis | Timestep convergence, seeded Monte Carlo, retained failure runs, percentile summaries, and optional RocketPy comparison | `convergence.csv`, `monte_carlo.csv`, `summary.json` |

## Quick Start

Ubuntu packages:

```bash
sudo apt update
sudo apt install build-essential cmake python3 python3-pip python3-tk python3-venv
```

Install the Workbench from the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.lock
pip install -e . --no-deps
```

Validate the reference configuration, build the flight core, and run a
deterministic mission:

```bash
awb validate
awb build-fsw
awb simulate --seed 1 --no-report
```

Launch the desktop Workbench:

```bash
python3 main.py
```

The Qt mission console keeps controls, event history, live metrics, vehicle
tracks, and altitude/speed/thrust strips in one view. After staging, each body
keeps its own colored trail and vector marker. The Configuration panel edits
and validates complete scenario and vehicle JSON working copies; **Save As**
writes a new paired configuration so the reference files remain unchanged.

The desktop solver runs in a worker process. The GUI receives only compact
display rows through a bounded queue; complete telemetry and reports are
written by the worker. Live metrics refresh at 30 Hz, while the trajectory and
the three lightweight strips refresh at 12 Hz from a mission-aware display
budget of at most 2,001 points per body. Display sampling does not change the
simulation timestep, Flight Core inputs, or persisted evidence.

CPU rendering is the reliable default, including under WSL. Qt OpenGL can be
enabled explicitly on a host with working EGL/OpenGL drivers:

```bash
ASTARA_UI_OPENGL=1 python3 main.py
```

If Mesa, EGL, or Zink reports a driver error, return to the optimized CPU path:

```bash
ASTARA_UI_OPENGL=0 python3 main.py
```

The header reports `CPU VIEW` or `OPENGL VIEW`; this affects presentation only.

### Live fault injection

The **Fault Injection** dock is available during an active desktop run. Select
the body, component, fault type, value when required, and duration, then choose
**Inject**. A zero duration remains active until **Clear**. Sensor faults apply
to all channels of the selected sensor. Engine faults apply to the selected
body's engine set. **Clear** removes operator-injected faults only; configured
timeline faults remain under scenario control.

Supported live sensor faults are dropout, stale, freeze, stuck-valid, bias,
and scale error. Supported engine faults are cutoff, thrust scale, and
overtemperature. Commands are validated at both the GUI and worker boundary.
Injection, rejection, and clearing are recorded as mission events. Live faults
do not modify the saved scenario or vehicle files.

The application is supported as a repository-run tool. See
[docs/PACKAGING.md](docs/PACKAGING.md) for the packaging boundary.

## Command Reference

| Command | Purpose | Useful options |
|---|---|---|
| `awb validate [scenario]` | Validate scenario, vehicle, mission, sensor, and actuator contracts | Omit `scenario` to use the reference mission |
| `awb build-fsw` | Configure and build the C++ flight core | Produces `flight_core/build/` |
| `awb simulate [scenario]` | Run one deterministic SIL mission | `--seed`, `--output`, `--no-report`, `--quiet` |
| `awb replay <sensors.csv.gz>` | Replay current-schema sensor evidence | `--scenario`, `--output` |
| `awb analyze [scenario]` | Run convergence and Monte Carlo analysis | `--samples`, `--seed`, `--workers`, `--telemetry-percent` |

Use `awb <command> --help` for complete options. The equivalent executable is
`aerospace-workbench`; `python3 -m aerospace_workbench` also works.

Simulation progress and mission events go to stderr. Stdout remains
machine-readable: the run directory followed by the JSON manifest. Use
`--progress` for newline-delimited redirected progress or `--quiet` to suppress
progress.

### Flight-software timing modes

Deterministic timing is the default. Measured timing profiles host execution;
injected timing tests repeatable deadline faults:

```bash
awb simulate --no-report --timing-mode measured
awb simulate --no-report --timing-mode injected \
  --injected-execution-time 0.02
```

Measured timing depends on host load. The selected mode and observed durations
are saved with the run.

## Configuration

The reference mission is
[`configs/scenarios/anthariksa_reference_mission.json`](configs/scenarios/anthariksa_reference_mission.json).
It references
[`configs/vehicles/anthariksa_reference_vehicle.json`](configs/vehicles/anthariksa_reference_vehicle.json).

| Document | Owns | Required schema |
|---|---|---|
| Scenario | Simulation rate and seed, launch environment, typed mission timeline, guidance, faults, orbit target, payload, uncertainty, and analysis settings | `aerospace-workbench.scenario.v2` |
| Vehicle | Stage geometry and mass, propulsion, aerodynamics, recovery, Flight Core parameters, sensor models, and actuator dynamics | `aerospace-workbench.vehicle.v2` |

All quantities use SI units unless the field name explicitly says otherwise.
Missing or mismatched schema identifiers are rejected. Scenario files must use
`vehicle_definition`; inline vehicle definitions are not accepted.

Mission sequencing is event-driven:

```json
{
  "timeline": [
    {
      "id": "launch_command",
      "trigger": {"type": "time", "at_s": 0.03},
      "action": {
        "type": "fsw_command",
        "command": "LAUNCH",
        "target": "integrated_stack"
      }
    },
    {
      "id": "stage_separation",
      "trigger": {"type": "fsw_fact", "fact": "stage_separation"},
      "action": {"type": "split_stage"}
    }
  ]
}
```

Timeline triggers may use scenario time, another event, a Flight Core fact, or
a truth detector. The single stable event queue also schedules physics,
devices, FSW tasks, bus delivery, faults, and evidence capture. Same-time
events use an explicit priority followed by insertion order. Duplicate IDs,
missing dependencies, invalid actions, negative delays, and dependency cycles
are rejected.

## Run Evidence

Each simulation creates a unique directory under `runs/` unless `--output`
selects another root.

| Artifact | Contents |
|---|---|
| `manifest.json` | Model version, seed, status, `simulation_only`/`unvalidated` flags, checks, timing mode, and artifact hashes |
| `scenario.json` | Canonical run snapshot referencing the bundled vehicle snapshot |
| `vehicle_definition.json` | Canonical self-contained vehicle snapshot |
| `source_scenario.json` | Byte-for-byte source scenario |
| `source_vehicle_definition.json` | Byte-for-byte source vehicle definition |
| `truth.csv` | Truth state, environment, propulsion, and actuation history |
| `sensors.csv.gz` | Timestamped per-channel sensor inputs sent to flight software |
| `avionics.csv` | Truth, sample, completion, bus-publication, and FSW-receive times for each device transaction |
| `commands.csv` | Recorded flight-software commands used by replay |
| `fsw.csv` | Navigation, modes, faults, sensor health, commands, timing, and actuation |
| `events.csv` | Mission and simulation event timeline |
| `report.pdf` and PNG files | Optional plots and run summary |

Replay requires the current sensor-stream schema and the original
`commands.csv` beside the sensor log:

```bash
awb replay runs/<run>/sensors.csv.gz
```

## Python Use

Import concrete responsibility modules:

```python
from aerospace_workbench.configuration.scenarios import load_scenario
from aerospace_workbench.simulation.runner import run_simulation

scenario = load_scenario()
result = run_simulation(scenario, seed=1)
print(result.output_dir)
```

## Verification

Run the Python and native regression gates:

```bash
.venv/bin/python -W error::RuntimeWarning -m unittest discover -s tests -v
awb build-fsw
ctest --test-dir flight_core/build --output-on-failure
git diff --check
```

## Repository Map

| Path | Responsibility |
|---|---|
| `src/aerospace_workbench/configuration/` | Scenario, vehicle, schema, and validation contracts |
| `src/aerospace_workbench/simulation/` | Truth, sensor, actuator, propulsion, aerodynamic, and mission simulation |
| `src/aerospace_workbench/flight_software/` | Python/C ABI boundary and flight-core build integration |
| `src/aerospace_workbench/replay/` | Sensor and command replay |
| `src/aerospace_workbench/evidence/` | Artifacts, reports, convergence, Monte Carlo, and provenance |
| `src/aerospace_workbench/presentation/` | Desktop Workbench |
| `flight_core/` | Dependency-free C++17 flight-software core |
| `configs/` | Reference scenario and vehicle definitions |
| `tests/` | Python regression and SIL integration checks |

## Limits and Safety Boundary

- Propulsion, aerodynamic, mass-property, and environmental inputs are
  provisional until replaced by reviewed analysis and test evidence.
- Attitude estimation uses Earth-rate-compensated gyro integration without a
  calibrated absolute magnetometer correction.
- Navigation uncertainty is a bounded diagonal estimate, not a validated
  navigation filter.
- The C++ core is loaded in-process through `ctypes`; transport faults, HIL,
  flight-computer timing, and memory budgets are not qualified.
- The Workbench exposes no serial, network, GPIO, ignition, valve,
  pyrotechnic, or flight-termination hardware interface.

More detail:

| Document | Use |
|---|---|
| [Architecture](docs/ARCHITECTURE.md) | Software and data boundaries |
| [Requirements](docs/REQUIREMENTS.md) | Requirement-to-test traceability |
| [Model credibility](docs/MODEL_CREDIBILITY.md) | Evidence contract and validation limits |
| [Scenario catalog](docs/SCENARIO_CATALOG.md) | Reference and qualification cases |
| [Glossary](docs/GLOSSARY.md) | Domain terms and abbreviations |
