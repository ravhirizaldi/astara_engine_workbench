# ASTARA Foundation Architecture

## Data boundary

```text
Scenario + Vehicle
        |
        v
6-DOF Truth Model --> truth.csv
        |
        v
Sensor Emulator --> sensors.csv.gz --> C++ Flight Core ----> fsw.csv
                                                |
                                                v
                                      Actuator Emulator
                                                |
                                                +----> Truth Model
```

`truth.csv`, `sensors.csv.gz`, and `fsw.csv` are deliberately separate. Flight
software receives only `AstaraSensorFrame`. Recorded sensor frames can replace
the simulator through `python -m astara replay`.

## Current process model

The desktop solver owns the truth model and loads Flight Core through its stable
C ABI. GUI rendering runs separately from the solver. This is host SIL, but not
yet the roadmap's independent-process SIL target.

The replayable sensor contract is the migration boundary. A future transport
may move Flight Core into another process without changing mission logic or
recorded sensor semantics.

## Time and memory

- Scenario time is explicit and independent of wall-clock display pacing.
- Flight Core receives `time_s` and `dt_s` on every step.
- Flight Core allocates its context once during initialization.
- UI telemetry and queues are bounded.

## Safety boundary

No module exposes physical ignition, valve, pyrotechnic, GPIO, serial, or
flight-termination control. Hardware integration requires a separate reviewed
adapter and a later roadmap gate.
