# ASTARA Engineering Glossary

This glossary defines the principal abbreviations, coordinate frames, software
boundaries, and engineering terms used by ASTARA Engineering Workbench. It is
intended as a quick reference for contributors reading the code, scenarios,
run artifacts, and architecture documents.

## Software and test levels

| Term | Meaning | Use in ASTARA |
|---|---|---|
| **FSW** | Flight Software | The C++ controller that consumes typed sensor, command, feedback, and timing inputs and produces navigation, mode, fault, and actuator outputs. |
| **Flight Core** | Host-executed FSW implementation | The dependency-free C++17 controller under `flight_core/`. It contains mission logic, navigation, sensor voting, fault handling, guidance, and control. |
| **SIL** | Software-in-the-Loop | Testing real flight-software logic against simulated sensors, vehicle dynamics, actuators, commands, and faults. ASTARA currently provides host-based SIL. |
| **HIL** | Hardware-in-the-Loop | Testing software and representative flight hardware against a real-time simulator. ASTARA does not currently provide HIL. |
| **PIL** | Processor-in-the-Loop | Running FSW on its target or representative processor while the surrounding vehicle and environment remain simulated. |
| **V&V** | Verification and Validation | Verification checks that software implements its requirements; validation checks that the model or system represents the intended real-world behavior adequately. |
| **CI** | Continuous Integration | Automated builds and tests run for repository changes, ideally covering Debug, Release, sanitizers, replay, ABI, and regression checks. |
| **I/O** | Input/Output | Data crossing a software or hardware boundary. ASTARA Flight Core currently exposes no physical ignition, valve, serial, GPIO, or flight-termination I/O. |
| **GPIO** | General-Purpose Input/Output | Digital hardware pins. Mentioned only as an excluded physical interface in the current safety boundary. |
| **GUI** | Graphical User Interface | The desktop workbench interface used to select scenarios, run simulations, replay results, and inspect plots and telemetry. |
| **CLI** | Command-Line Interface | Commands such as `awb simulate`, `awb validate`, `awb analyze`, and `awb replay`; `python3 -m aerospace_workbench` is the module form. |

## Interfaces and data contracts

| Term | Meaning | Use in ASTARA |
|---|---|---|
| **API** | Application Programming Interface | A general software interface between components. |
| **ABI** | Application Binary Interface | The exact binary layout and calling contract between Python and the compiled C++ Flight Core. ASTARA exposes versioned `FswConfig`, `FswInput`, `FswOutput`, and `fsw_*` symbols. |
| **C ABI** | C-compatible binary interface | The stable interface loaded by Python through `ctypes`, even though Flight Core is implemented in C++. |
| **FFI** | Foreign Function Interface | A mechanism for one programming language to call another. Python-to-C++ calls through the C ABI are an FFI boundary. |
| **Schema** | Machine-readable data structure contract | Scenario, vehicle, and run artifacts carry explicit schema versions such as `aerospace-workbench.scenario.v1` and `aerospace-workbench.vehicle.v1`. |
| **Typed input/output** | Fields with explicit meaning, units, and structure | Flight Core receives structured sensor, command, propulsion, recovery, air-data, and timing fields instead of an unstructured dictionary or byte stream. |
| **Replay** | Re-execution from recorded inputs | `awb replay` sends a recorded sensor and command stream through the same Flight Core without rerunning vehicle truth dynamics. |
| **Artifact** | Persisted output from a run | Examples include `truth.csv`, `sensors.csv.gz`, `commands.csv`, `fsw.csv`, `events.csv`, and `manifest.json`. |
| **Manifest** | Run metadata and integrity record | Stores scenario identity, model version, seed, coordinate frames, checks, timing mode, hashes, and result status. |
| **SHA-256** | Secure Hash Algorithm, 256-bit | Used to identify scenario, vehicle, model source, and generated artifacts reproducibly. |

## Simulation and modeling

| Term | Meaning | Use in ASTARA |
|---|---|---|
| **Digital twin** | Executable model of a physical system | ASTARA models vehicle dynamics, propulsion, aerodynamics, sensors, actuators, environment, recovery, and FSW interaction. The bundled model remains preliminary and unvalidated. |
| **Truth model** | Simulator state treated as the reference physical state | Produces ideal vehicle position, velocity, attitude, rates, loads, propulsion, and environment values. Flight Core does not receive truth state directly. |
| **6-DOF** | Six Degrees of Freedom | Three translational axes plus three rotational axes. ASTARA propagates position, velocity, attitude, and angular motion. |
| **DOF** | Degree of Freedom | One independent direction of translation or rotation. |
| **RK4** | Fourth-order Runge-Kutta | A numerical integration method used to advance continuous vehicle dynamics. |
| **Monte Carlo** | Repeated simulation with sampled uncertainty | Used to study sensitivity, dispersion, failure cases, and output distributions across many seeds and parameter variations. |
| **Deterministic** | Repeatable for identical controlled inputs | The same scenario, seed, model version, and deterministic timing policy should produce the same results. |
| **Wall clock** | Time measured by the host computer | Used for profiling native `fsw_step()` execution. It is separate from scenario time and may vary with machine load. |
| **Scenario time** | Simulated mission time | The authoritative time used by physics, sensors, events, FSW, and run artifacts. It is independent of display pacing. |
| **Timestep (`dt`)** | Simulated time advanced per integration or FSW step | A timestep must be positive, bounded, and consistent with sample timestamps. |
| **Convergence study** | Comparison across smaller timesteps | ASTARA analysis compares `dt`, `dt/2`, and `dt/4` to detect excessive timestep sensitivity. |
| **Preliminary / unvalidated** | Model not yet correlated to controlled engineering evidence | Results may support software integration and planning, but not flight safety, qualification, certification, or design release. |

## Coordinate frames and navigation

| Term | Meaning | Use in ASTARA |
|---|---|---|
| **ECEF** | Earth-Centered, Earth-Fixed | A Cartesian frame rotating with Earth. ASTARA truth translation and the minimal Flight Core navigation solution use ECEF. |
| **ECI** | Earth-Centered Inertial | A non-Earth-fixed frame used for inertial propagation and orbital mechanics. ASTARA currently propagates body attitude relative to ECEF with Earth-rate compensation rather than moving the full attitude solution to ECI. |
| **NED** | North-East-Down | A local navigation frame used for human-readable reports and local directional quantities. |
| **Body frame** | Coordinate frame fixed to the vehicle | Sensor axes, angular rates, thrust directions, actuator forces, and moments are commonly expressed in body coordinates. |
| **Inertial frame** | Frame that does not rotate with Earth | An ideal gyroscope measures body angular velocity relative to an inertial frame. |
| **Quaternion** | Four-component attitude representation | Used to represent body orientation without Euler-angle singularities. ASTARA uses a body-to-ECEF attitude quaternion. |
| **DCM** | Direction Cosine Matrix | A 3x3 rotation matrix derived from attitude. Symbols such as `C_e^b` rotate vectors from ECEF into body coordinates. |
| **Earth rate** | Earth's angular rotation rate | Approximately `7.292115e-5 rad/s`. It is transformed into body coordinates and removed from inertial gyro rates when propagating body-to-ECEF attitude. |
| **Strapdown navigation** | Navigation by integrating body-mounted inertial sensors | Flight Core rotates specific force into ECEF, applies gravity and rotating-Earth terms, and integrates velocity and position. |
| **Specific force** | Non-gravitational acceleration measured by an accelerometer | It is not identical to total kinematic acceleration because an accelerometer does not directly measure gravity in the usual inertial-navigation formulation. |
| **Coriolis term** | Apparent acceleration in a rotating frame | Required when integrating motion in ECEF. |
| **Centrifugal term** | Apparent outward acceleration in a rotating frame | Also required for consistent ECEF propagation. |
| **CG** | Center of Gravity | The effective point through which vehicle weight acts; it changes as propellant is consumed. |
| **Inertia tensor** | Resistance to angular acceleration about vehicle axes | Used by the rigid-body rotational dynamics model. |

## Sensors and estimation

| Term | Meaning | Use in ASTARA |
|---|---|---|
| **IMU** | Inertial Measurement Unit | Typically combines accelerometers and gyroscopes. ASTARA supports independently timestamped virtual channels. |
| **GNSS** | Global Navigation Satellite System | Provides simulated position and velocity aiding. GPS is one GNSS constellation, but GNSS is the broader term. |
| **GPS** | Global Positioning System | The United States GNSS constellation. Use `GNSS` when referring to the generic sensor class. |
| **Gyro** | Gyroscope | Measures angular velocity of the body relative to an inertial frame, expressed in body coordinates. |
| **Accelerometer** | Specific-force sensor | Measures non-gravitational specific force along its axes. |
| **Magnetometer** | Magnetic-field sensor | Available as an independent sensor class; calibrated absolute attitude correction is not yet implemented. |
| **Barometer** | Pressure-based altitude sensor | Provides atmospheric pressure-derived altitude aiding in the simulated sensor suite. |
| **Aiding** | External measurement used to correct propagated navigation | GNSS and barometer measurements can constrain inertial drift. |
| **Fusion** | Combining multiple measurements or channels | Flight Core votes fresh channels and combines usable measurements while tracking disagreement and rejection. |
| **Voting** | Selecting or combining redundant sensor channels | Used to tolerate outliers, stale data, and failed channels. |
| **Innovation** | Difference between a measurement and its predicted value | Large innovations can indicate disagreement, outliers, model error, or sensor faults. |
| **Bias** | Persistent sensor offset | May be configured per channel or injected as a fault. |
| **Scale factor** | Multiplicative sensor error | Causes reported measurements to be proportionally high or low. |
| **Noise** | Random measurement variation | Generated independently per virtual channel according to scenario settings. |
| **Stale data** | A sample that has not been refreshed within its allowed age | Stale channels are rejected or marked unusable according to configured freshness rules. |
| **Sample skew** | Timestamp difference among measurements being fused | Excessive skew means channels do not represent the same physical epoch and should not be naively averaged. |
| **RMSE** | Root Mean Square Error | A summary of estimation error magnitude, such as navigation altitude RMSE. |
| **EKF** | Extended Kalman Filter | A nonlinear state estimator using covariance propagation and measurement updates. It is a future refinement, not the current navigation implementation. |
| **ESKF** | Error-State Kalman Filter | A Kalman-filter formulation that estimates small navigation errors around a separately propagated nominal state. |

## Guidance, control, propulsion, and mission logic

| Term | Meaning | Use in ASTARA |
|---|---|---|
| **GNC** | Guidance, Navigation, and Control | Guidance determines desired motion, navigation estimates current state, and control commands actuators to reduce the difference. |
| **Guidance** | Desired trajectory or attitude generation | Flight Core interpolates scheduled attitude targets from the configured mission profile. |
| **Navigation** | Estimation of position, velocity, attitude, and related state | Flight Core maintains a minimal ECEF navigation state from simulated sensors. |
| **Control** | Calculation and allocation of actuator effort | Flight Core produces TVC and aerodynamic-control commands from attitude and rate errors. |
| **TVC** | Thrust Vector Control | Steering by changing the engine thrust direction. |
| **AoA** | Angle of Attack | Angle between the vehicle reference axis and relative airflow. Large values may exceed the aerodynamic model's validity envelope. |
| **Mach** | Speed divided by local speed of sound | Used by aerodynamic coefficient tables and mission metrics. |
| **Max Q** | Maximum dynamic pressure | The point of highest aerodynamic dynamic pressure, where `q = 0.5 rho V^2`. |
| **Dynamic pressure (`q`)** | Aerodynamic pressure associated with motion through air | Used to estimate aerodynamic forces and identify high-load flight regions. |
| **Burnout** | End of useful engine burn | Confirmed burnout can trigger event-relative delays such as stage separation. |
| **Interstage** | Phase or structure between stages | Flight Core uses an interstage mission mode between separation and upper-stage ignition. |
| **Pyro** | Pyrotechnic device or command | ASTARA represents recovery and separation actuation as sequence-identified one-shot commands; no physical pyrotechnic interface exists. |
| **One-shot command** | Command that must be consumed once | Sequence identities prevent repeated actuator activation when an output persists across multiple FSW steps. |
| **Latch / latched fault** | State retained after its triggering condition clears | Useful for post-run evidence and faults requiring explicit reset or safing. |
| **Active fault** | Fault whose triggering condition is currently present | Reported separately from historical or latched fault state. |
| **Watchdog** | Independent monitor for software execution health | A watchdog fault may indicate that expected execution or servicing did not occur. |
| **Deadline** | Maximum allowed execution interval or duration | Platform timing inputs can report or inject deadline misses for deterministic fault testing. |
| **Abort** | Transition to a safe or terminated mission path | Triggered only when configured fault severity and mission logic require it. |

## Build and diagnostic terms

| Term | Meaning | Use in ASTARA |
|---|---|---|
| **Debug build** | Build retaining assertions and debugging support | Used to catch invalid state and aid development. |
| **Release build** | Optimized build representative of normal deployment settings | Must still preserve safety checks that cannot depend on debug-only assertions. |
| **CTest** | CMake's test runner | Executes Flight Core native tests. |
| **ASan** | AddressSanitizer | Detects invalid memory access, use-after-free, and related C/C++ memory defects. |
| **UBSan** | UndefinedBehaviorSanitizer | Detects many forms of undefined C/C++ behavior. |
| **Fuzz testing** | Automated generation of malformed or unusual inputs | Useful for timestamps, ABI inputs, commands, NaN/Inf handling, and sensor-voter edge cases. |
| **Regression test** | Test ensuring previously correct behavior remains correct | Golden replay and deterministic artifact checks are examples. |
| **Golden replay** | A recorded input stream with expected stable outputs | Detects unintended changes in Flight Core behavior across refactors. |
| **NaN** | Not a Number | Invalid floating-point value that must be rejected or safely contained. |
| **Inf** | Infinity | Non-finite floating-point value that must not silently enter navigation or control state. |

## Naming guidance

Use the following terms consistently in code and documentation:

- Use **Flight Core** for the C++ FSW implementation.
- Use **truth** only for simulator reference state, never for an FSW estimate.
- Use **GNSS** for the generic satellite-navigation sensor class.
- Use **scenario time** for simulated mission time and **wall clock** for host timing.
- Use **SIL** for the current host-executed software loop. Do not describe current runs as HIL, real-time qualification, flight validation, or certification evidence.
- State coordinate frames explicitly for vectors, rates, quaternions, forces, and moments.
- Include units in schema fields, column names, or adjacent documentation whenever practical.
