# ASTARA Engineering Workbench Model Credibility Baseline

## Decision status

The workbench truth model is a deterministic, preliminary engineering simulator.
Its present intended use is software integration, sensitivity analysis, and
planning physical test campaigns. It is not validated for flight safety,
hardware qualification, certification, or range-safety decisions.

Python and NumPy are the truth-model implementation because model equations,
tables, and analysis are still evolving. The dependency-free C++ core remains
the software-in-the-loop flight-software boundary. A language rewrite does not
increase physical credibility; reviewed inputs and validation evidence do.

## Evidence contract

Every scenario used for an engineering result shall identify:

- SI units and coordinate frames
- source, revision, owner, and review status for each model input
- the operating envelope in which each model is valid
- one-sigma or bounded uncertainty for significant inputs
- numerical acceptance criteria
- prohibited uses and known limitations

The reference scenario embeds this information under `credibility` and
`uncertainty`. Its propulsion, aerodynamic, and mass-property values are
explicitly marked provisional.

## Model inputs

### Propulsion

`performance_curve` is linearly interpolated in burn time and contains thrust,
fuel flow, oxidizer flow, chamber pressure, and temperature. Curves must start
at zero seconds, end at `burn_duration_s`, remain nonnegative, and consume no
more propellant than the stage contains.

Replace the provisional curves with calibrated static-fire data. Preserve the
raw test identity, calibration information, test conditions, uncertainty, and
the processing script used to create the scenario table.

### Aerodynamics

`coefficient_table` is linearly interpolated in Mach and contains drag,
normal-force slope, damping, and control-force slope per radian of fin
deflection. Angle-of-attack effects, static margin, and table validity limits
are evaluated by the 6DOF model.

Replace the provisional values with reviewed CFD, wind-tunnel, or independently
validated coefficient data. A future two-dimensional Mach/AoA table is warranted
only when source data supports it.

### Mass properties

`mass_properties` is linearly interpolated by remaining propellant fraction and
contains center of mass and principal inertia. Replace it with a reviewed mass
breakdown plus measured component masses and CG/inertia analysis.

### Environment

The baseline uses a layered ISA approximation and a constant NED wind vector.
Campaign analysis requires a time- and altitude-dependent launch-site profile
with its source and timestamp.

## Automated evidence

Run:

```bash
python3 -m astara analyze scenarios/anthariksa_reference_mission.json --samples 20 --seed 1
```

The command writes:

- `convergence.csv`: declared-horizon results at `dt`, `dt/2`, and `dt/4`
- `monte_carlo.csv`: sampled factors and result metrics for every run
- `summary.json`: provenance, declared uncertainty, convergence result, and
  P5/median/P95 result ranges
- `scenario.json`: the exact analyzed input

The Monte Carlo result covers only the distributions declared in the scenario.
It is uncertainty propagation, not physical validation.

The reference convergence horizon is 60 seconds, covering powered ascent,
separation, second-stage burn, apogee, and initial recovery. Set
`credibility.acceptance_criteria.convergence_horizon_s` to the full mission
duration when landing-state convergence is required.

## Verification and validation matrix

| Area | Current evidence | Required next evidence |
|---|---|---|
| Scenario boundary | Schema and table validation tests | Independent input review |
| Determinism | Repeated-seed regression | Cross-platform baseline |
| Coordinates | Quaternion and NED/ECEF round-trip tests | Independent trajectory comparison |
| Numerical integration | Full-mission timestep convergence | Analytical force/torque cases |
| Propulsion | Curve bounds and propellant-integral checks | Static-fire correlation and residuals |
| Aerodynamics | Envelope flags and coefficient interpolation | CFD/wind-tunnel correlation |
| Mass properties | Fraction-dependent CG/inertia | Measured mass and reviewed inertia model |
| Flight software | C++ SIL mode-transition checks | Processor/HIL timing and interface tests |
| Recovery | Functional deployment simulation | Drop-test and inflation-load correlation |

## Review gate

A scenario may move beyond `PRELIMINARY_UNVALIDATED` only when:

1. every significant input has traceable evidence and an owner;
2. numerical convergence passes project-approved criteria;
3. model outputs are compared with independent analysis or physical tests;
4. residuals and measurement uncertainty are reported;
5. intended and prohibited uses are approved by responsible engineering and
   safety personnel.
