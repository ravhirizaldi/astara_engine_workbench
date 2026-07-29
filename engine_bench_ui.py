"""Standalone Engine Bench for the simplified rocket combustion chamber simulation.

This is an educational visualization, not a propulsion design tool.
The model intentionally uses simplified ideal gas and nozzle behavior.
"""

import csv
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

os.environ.setdefault(
    "MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "astara_matplotlib")
)

from matplotlib.figure import Figure
import numpy as np


# -----------------------------
# User-tweakable defaults
# -----------------------------

CHAMBER_VOLUME = 0.08  # m^3
COMBUSTION_TEMPERATURE = 3400.0  # K
GAS_CONSTANT = 355.0  # J/(kg*K), approximate combustion gas value
BURN_DURATION = 8.0  # s
PROPELLANT_MASS_FLOW_RATE = 1.2  # kg/s generated gas during steady burn
NOZZLE_COEFFICIENT = 2.5e-6  # kg/(s*Pa), simplified pressure-driven outflow
EXHAUST_VELOCITY = 2200.0  # m/s
NOZZLE_EXIT_AREA = 0.012  # m^2
AMBIENT_PRESSURE = 101_325.0  # Pa
TIME_STEP = 0.01  # s

OUTPUT_DIR = Path("output")
AMBIENT_TEMPERATURE = 293.15  # K
IGNITION_RAMP_FRACTION = 0.10
SHUTDOWN_RAMP_FRACTION = 0.10
POST_BURN_TIME = 3.0  # s to show chamber blowdown after shutdown
GUI_DISPLAY_NOTE = ""
GUI_UPDATE_MS = 40
SIM_STEPS_PER_UPDATE = 4

# Educational degradation thresholds. These are visualization limits, not
# validated propulsion design limits.
PRESSURE_WARNING = 1_000_000.0  # Pa
PRESSURE_CRITICAL = 1_600_000.0  # Pa
PRESSURE_FAILURE = 2_200_000.0  # Pa
TEMPERATURE_WARNING = 3200.0  # K
TEMPERATURE_CRITICAL = 3700.0  # K
TEMPERATURE_FAILURE = 4200.0  # K

# Rates are percentage points per simulated second at each band's upper edge.
# Squared severity and exposure ramping make short spikes mild while sustained
# critical operation accelerates degradation.
PRESSURE_WARNING_HEALTH_RATE = 5.0
PRESSURE_CRITICAL_HEALTH_RATE = 20.0
PRESSURE_WARNING_STRESS_RATE = 8.0
PRESSURE_CRITICAL_STRESS_RATE = 24.0
TEMPERATURE_WARNING_HEALTH_RATE = 9.0
TEMPERATURE_CRITICAL_HEALTH_RATE = 25.0
TEMPERATURE_WARNING_COOLING_RATE = 5.0
TEMPERATURE_CRITICAL_COOLING_RATE = 16.0
NOZZLE_EROSION_DELAY = 2.0  # s continuously above pressure warning
MAX_EXHAUST_VELOCITY_LOSS = 0.08
INSTABILITY_DETECTION_LEVEL = 30.0
INSTABILITY_OSCILLATION_LEVEL = 50.0


CONTROL_SPECS = [
    ("chamber_volume", "Chamber volume (m^3)", CHAMBER_VOLUME, 0.01, 0.30, 0.005),
    (
        "combustion_temperature",
        "Combustion temp (K)",
        COMBUSTION_TEMPERATURE,
        1000.0,
        4500.0,
        25.0,
    ),
    ("gas_constant", "Gas constant", GAS_CONSTANT, 100.0, 700.0, 5.0),
    ("burn_duration", "Burn duration (s)", BURN_DURATION, 1.0, 30.0, 0.25),
    (
        "propellant_mass_flow_rate",
        "Mass flow in (kg/s)",
        PROPELLANT_MASS_FLOW_RATE,
        0.05,
        5.0,
        0.05,
    ),
    (
        "nozzle_coefficient",
        "Nozzle coefficient",
        NOZZLE_COEFFICIENT,
        1.0e-7,
        1.0e-5,
        1.0e-7,
    ),
    ("exhaust_velocity", "Exhaust velocity (m/s)", EXHAUST_VELOCITY, 500.0, 4000.0, 50.0),
    ("nozzle_exit_area", "Exit area (m^2)", NOZZLE_EXIT_AREA, 0.001, 0.05, 0.001),
    ("ambient_pressure", "Ambient pressure (Pa)", AMBIENT_PRESSURE, 10_000.0, 150_000.0, 1000.0),
    ("time_step", "Time step (s)", TIME_STEP, 0.002, 0.05, 0.002),
]


GRAPH_DEFINITIONS = [
    ("pressure", "Chamber Pressure vs Time", "Pressure (Pa)", "chamber_pressure.png"),
    ("temperature", "Chamber Temperature vs Time", "Temperature (K)", "chamber_temperature.png"),
    ("mass_flow_in", "Generated Mass Flow vs Time", "Mass Flow In (kg/s)", "mass_flow_in.png"),
    ("mass_flow_out", "Nozzle Outflow vs Time", "Mass Flow Out (kg/s)", "mass_flow_out.png"),
    ("thrust", "Estimated Thrust vs Time", "Thrust (N)", "estimated_thrust.png"),
    (
        "engine_health_percent",
        "Engine Health vs Time",
        "Engine Health (%)",
        "engine_health.png",
    ),
]


def default_config() -> dict[str, float]:
    """Return editable default values for GUI controls and batch simulation."""
    return {key: default for key, _label, default, _min, _max, _step in CONTROL_SPECS}


def validate_config(config: dict[str, float]) -> None:
    """Validate basic simulation constants before running."""
    required_positive = {
        "chamber_volume": config["chamber_volume"],
        "combustion_temperature": config["combustion_temperature"],
        "gas_constant": config["gas_constant"],
        "burn_duration": config["burn_duration"],
        "propellant_mass_flow_rate": config["propellant_mass_flow_rate"],
        "nozzle_coefficient": config["nozzle_coefficient"],
        "exhaust_velocity": config["exhaust_velocity"],
        "nozzle_exit_area": config["nozzle_exit_area"],
        "ambient_pressure": config["ambient_pressure"],
        "time_step": config["time_step"],
    }

    for name, value in required_positive.items():
        if value <= 0:
            raise ValueError(f"{name} must be positive; got {value!r}")

    if config["time_step"] >= config["burn_duration"]:
        raise ValueError(
            "time_step must be smaller than burn_duration so ramp phases can be resolved"
        )


def burn_profile(time_s: float, config: dict[str, float]) -> float:
    """Return generated gas mass flow for ignition, steady burn, and shutdown.

    Propellant flow is treated as generated chamber gas. Ignition and shutdown
    use linear ramps so pressure and thrust do not jump instantly.
    """
    burn_duration = config["burn_duration"]
    ignition_ramp = burn_duration * IGNITION_RAMP_FRACTION
    shutdown_ramp = burn_duration * SHUTDOWN_RAMP_FRACTION
    shutdown_start = burn_duration - shutdown_ramp
    steady_flow = config["propellant_mass_flow_rate"]

    if time_s < 0:
        return 0.0
    if time_s < ignition_ramp:
        return steady_flow * (time_s / ignition_ramp)
    if time_s < shutdown_start:
        return steady_flow
    if time_s <= burn_duration:
        remaining = burn_duration - time_s
        return steady_flow * (remaining / shutdown_ramp)
    return 0.0


def target_temperature(mass_flow_in: float, config: dict[str, float]) -> float:
    """Map burn activity to a simplified chamber temperature target."""
    burn_fraction = mass_flow_in / config["propellant_mass_flow_rate"]
    return AMBIENT_TEMPERATURE + burn_fraction * (
        config["combustion_temperature"] - AMBIENT_TEMPERATURE
    )


def clamp_percent(value: float) -> float:
    """Clamp a degradation percentage to its valid range."""
    return min(max(value, 0.0), 100.0)


def band_severity(value: float, lower: float, upper: float) -> float:
    """Return normalized 0..1 severity within one threshold band."""
    if upper <= lower:
        return 0.0
    return min(max((value - lower) / (upper - lower), 0.0), 1.0)


class ChamberSimulation:
    """Step-by-step state for realtime plotting."""

    def __init__(self, config: dict[str, float]) -> None:
        validate_config(config)
        self.config = config
        self.total_time = config["burn_duration"] + POST_BURN_TIME
        self.reset()

    def reset(self) -> None:
        self.time_s = 0.0
        self.current_mass = 0.0
        self.current_temperature = AMBIENT_TEMPERATURE
        self.finished = False
        self.emergency_shutdown = False
        self.safety_state = "NOMINAL"
        self.safety_message = "Engine conditions nominal"
        self.engine_health_percent = 100.0
        self.cooling_efficiency_percent = 100.0
        self.wall_stress_percent = 0.0
        self.nozzle_erosion_percent = 0.0
        self.instability_percent = 0.0
        self.limiting_factor = "None"
        self.recommendation = "Engine conditions nominal."
        self.pressure_exposure_s = 0.0
        self.temperature_exposure_s = 0.0
        self.high_pressure_duration_s = 0.0
        self.nozzle_erosion_started = False
        self.instability_detected = False
        self.shutdown_stabilized = False
        self.pending_events: list[str] = []
        self.history = {
            "time": [],
            "gas_mass": [],
            "pressure": [],
            "temperature": [],
            "mass_flow_in": [],
            "mass_flow_out": [],
            "effective_exhaust_velocity": [],
            "thrust": [],
            "engine_health_percent": [],
            "cooling_efficiency_percent": [],
            "wall_stress_percent": [],
            "nozzle_erosion_percent": [],
            "instability_percent": [],
            "safety_state": [],
            "limiting_factor": [],
            "recommendation": [],
        }

    def step(self) -> bool:
        """Advance one timestep and store plotted values."""
        if self.finished:
            return False

        config = self.config
        incoming = 0.0 if self.emergency_shutdown else burn_profile(self.time_s, config)

        # First-order thermal response: chamber heats toward combustion temp
        # while burning and cools toward ambient after shutdown.
        temp_goal = target_temperature(incoming, config)
        thermal_response = 0.18 if incoming > 0 else 0.04
        self.current_temperature += (temp_goal - self.current_temperature) * thermal_response

        chamber_pressure = (
            self.current_mass
            * config["gas_constant"]
            * self.current_temperature
            / config["chamber_volume"]
        )
        pressure_delta = max(chamber_pressure - config["ambient_pressure"], 0.0)

        # Simplified nozzle: pressure above ambient drives exhaust mass flow.
        outgoing = config["nozzle_coefficient"] * pressure_delta
        self.current_mass = max(
            self.current_mass + (incoming - outgoing) * config["time_step"], 0.0
        )

        chamber_pressure = (
            self.current_mass
            * config["gas_constant"]
            * self.current_temperature
            / config["chamber_volume"]
        )

        self._apply_degradation(chamber_pressure, self.current_temperature, incoming)

        # High instability adds a small deterministic oscillation. A sine wave
        # keeps repeated educational runs reproducible while visualizing noise.
        instability_excess = max(
            (self.instability_percent - INSTABILITY_OSCILLATION_LEVEL)
            / (100.0 - INSTABILITY_OSCILLATION_LEVEL),
            0.0,
        )
        oscillation_amplitude = 0.02 * instability_excess**2
        pressure_wave = np.sin(2.0 * np.pi * 7.0 * self.time_s)
        measured_pressure = max(
            chamber_pressure * (1.0 + oscillation_amplitude * pressure_wave), 0.0
        )

        effective_exhaust_velocity = config["exhaust_velocity"] * (
            1.0
            - MAX_EXHAUST_VELOCITY_LOSS * self.nozzle_erosion_percent / 100.0
        )
        pressure_delta = max(measured_pressure - config["ambient_pressure"], 0.0)
        base_thrust = (
            outgoing * effective_exhaust_velocity
            + pressure_delta * config["nozzle_exit_area"]
        )
        thrust_wave = np.sin(2.0 * np.pi * 9.0 * self.time_s + 0.7)
        thrust = max(
            base_thrust * (1.0 + 1.25 * oscillation_amplitude * thrust_wave), 0.0
        )

        self._update_limiting_factor(measured_pressure, self.current_temperature)
        self._update_safety_state(measured_pressure, self.current_temperature)

        self.history["time"].append(self.time_s)
        self.history["gas_mass"].append(self.current_mass)
        self.history["pressure"].append(measured_pressure)
        self.history["temperature"].append(self.current_temperature)
        self.history["mass_flow_in"].append(incoming)
        self.history["mass_flow_out"].append(outgoing)
        self.history["effective_exhaust_velocity"].append(effective_exhaust_velocity)
        self.history["thrust"].append(thrust)
        self.history["engine_health_percent"].append(self.engine_health_percent)
        self.history["cooling_efficiency_percent"].append(
            self.cooling_efficiency_percent
        )
        self.history["wall_stress_percent"].append(self.wall_stress_percent)
        self.history["nozzle_erosion_percent"].append(self.nozzle_erosion_percent)
        self.history["instability_percent"].append(self.instability_percent)
        self.history["safety_state"].append(self.safety_state)
        self.history["limiting_factor"].append(self.limiting_factor)
        self.history["recommendation"].append(self.recommendation)

        self.time_s += config["time_step"]
        if self.time_s > self.total_time:
            self.finished = True
        return True

    def _apply_degradation(
        self, pressure: float, temperature: float, mass_flow_in: float
    ) -> None:
        """Apply simplified nonlinear wear from sustained chamber stress."""
        dt = self.config["time_step"]
        shutdown_factor = 0.1 if self.emergency_shutdown else 1.0

        if pressure > PRESSURE_WARNING:
            self.pressure_exposure_s += dt
            self.high_pressure_duration_s += dt
        else:
            self.pressure_exposure_s = max(self.pressure_exposure_s - 2.0 * dt, 0.0)
            self.high_pressure_duration_s = 0.0

        if temperature > TEMPERATURE_WARNING:
            self.temperature_exposure_s += dt
        else:
            self.temperature_exposure_s = max(
                self.temperature_exposure_s - 2.0 * dt, 0.0
            )

        pressure_ramp = min(self.pressure_exposure_s, 1.0)
        temperature_ramp = min(self.temperature_exposure_s, 1.0)
        pressure_health_rate = 0.0
        wall_stress_rate = 0.0
        temperature_health_rate = 0.0
        cooling_loss_rate = 0.0

        if PRESSURE_WARNING < pressure < PRESSURE_CRITICAL:
            severity = band_severity(pressure, PRESSURE_WARNING, PRESSURE_CRITICAL)
            pressure_health_rate = PRESSURE_WARNING_HEALTH_RATE * severity**2
            wall_stress_rate = PRESSURE_WARNING_STRESS_RATE * severity**2
        elif pressure >= PRESSURE_CRITICAL:
            severity = band_severity(pressure, PRESSURE_CRITICAL, PRESSURE_FAILURE)
            pressure_health_rate = (
                PRESSURE_WARNING_HEALTH_RATE
                + PRESSURE_CRITICAL_HEALTH_RATE * severity**2
            )
            wall_stress_rate = (
                PRESSURE_WARNING_STRESS_RATE
                + PRESSURE_CRITICAL_STRESS_RATE * severity**2
            )

        if TEMPERATURE_WARNING < temperature < TEMPERATURE_CRITICAL:
            severity = band_severity(
                temperature, TEMPERATURE_WARNING, TEMPERATURE_CRITICAL
            )
            temperature_health_rate = TEMPERATURE_WARNING_HEALTH_RATE * severity**2
            cooling_loss_rate = TEMPERATURE_WARNING_COOLING_RATE * severity**2
        elif temperature >= TEMPERATURE_CRITICAL:
            severity = band_severity(
                temperature, TEMPERATURE_CRITICAL, TEMPERATURE_FAILURE
            )
            temperature_health_rate = (
                TEMPERATURE_WARNING_HEALTH_RATE
                + TEMPERATURE_CRITICAL_HEALTH_RATE * severity**2
            )
            cooling_loss_rate = (
                TEMPERATURE_WARNING_COOLING_RATE
                + TEMPERATURE_CRITICAL_COOLING_RATE * severity**2
            )

        health_loss = (
            pressure_health_rate * pressure_ramp
            + temperature_health_rate * temperature_ramp
        ) * dt * shutdown_factor
        self.engine_health_percent = clamp_percent(
            self.engine_health_percent - health_loss
        )
        self.wall_stress_percent = clamp_percent(
            self.wall_stress_percent
            + wall_stress_rate * pressure_ramp * dt * shutdown_factor
        )
        self.cooling_efficiency_percent = clamp_percent(
            self.cooling_efficiency_percent
            - cooling_loss_rate * temperature_ramp * dt * shutdown_factor
        )

        if self.high_pressure_duration_s > NOZZLE_EROSION_DELAY:
            pressure_severity = band_severity(
                pressure, PRESSURE_WARNING, PRESSURE_FAILURE
            )
            erosion_rate = 0.1 + 0.4 * pressure_severity**2
            previous_erosion = self.nozzle_erosion_percent
            self.nozzle_erosion_percent = clamp_percent(
                self.nozzle_erosion_percent
                + erosion_rate * dt * shutdown_factor
            )
            if previous_erosion == 0.0 and self.nozzle_erosion_percent > 0.0:
                self.nozzle_erosion_started = True
                self.pending_events.append("Nozzle erosion started")

        instability_rate = 0.0
        high_mass_flow = mass_flow_in >= 0.85 * self.config["propellant_mass_flow_rate"]
        if pressure > PRESSURE_WARNING and high_mass_flow:
            pressure_severity = band_severity(
                pressure, PRESSURE_WARNING, PRESSURE_FAILURE
            )
            instability_rate += 0.5 + 5.0 * pressure_severity**2
        if temperature > TEMPERATURE_CRITICAL:
            temperature_severity = band_severity(
                temperature, TEMPERATURE_CRITICAL, TEMPERATURE_FAILURE
            )
            instability_rate += 0.5 + 3.0 * temperature_severity**2

        previous_instability = self.instability_percent
        if instability_rate > 0.0:
            self.instability_percent = clamp_percent(
                self.instability_percent + instability_rate * dt * shutdown_factor
            )
        else:
            decay_rate = 2.0 if self.emergency_shutdown else 0.5
            self.instability_percent = clamp_percent(
                self.instability_percent - decay_rate * dt
            )

        if (
            not self.instability_detected
            and previous_instability < INSTABILITY_DETECTION_LEVEL
            <= self.instability_percent
        ):
            self.instability_detected = True
            self.pending_events.append("Combustion instability detected")

        if (
            self.emergency_shutdown
            and not self.shutdown_stabilized
            and pressure < PRESSURE_WARNING
            and temperature < TEMPERATURE_WARNING
            and self.instability_percent < INSTABILITY_DETECTION_LEVEL
        ):
            self.shutdown_stabilized = True
            self.pending_events.append("Emergency shutdown stabilized the engine")

    def _update_safety_state(self, pressure: float, temperature: float) -> None:
        """Set NOMINAL/WARNING/CRITICAL/FAILURE from current health and load."""
        failure_reason = ""
        if not self.emergency_shutdown and pressure >= PRESSURE_FAILURE:
            failure_reason = "SIMULATED FAILURE: chamber overpressure"
        elif not self.emergency_shutdown and temperature >= TEMPERATURE_FAILURE:
            failure_reason = "SIMULATED FAILURE: chamber overheated"
        elif self.engine_health_percent <= 0.0:
            failure_reason = "SIMULATED FAILURE: engine health depleted"
        elif self.instability_percent >= 100.0:
            failure_reason = "SIMULATED FAILURE: combustion instability"

        if failure_reason:
            self.safety_state = "FAILURE"
            self.safety_message = failure_reason
            self.finished = True
        elif (
            pressure >= PRESSURE_CRITICAL
            or temperature >= TEMPERATURE_CRITICAL
            or self.engine_health_percent <= 50.0
            or self.instability_percent > 80.0
        ):
            self.safety_state = "CRITICAL"
            self.safety_message = f"Critical limit: {self.limiting_factor}"
        elif (
            pressure >= PRESSURE_WARNING
            or temperature >= TEMPERATURE_WARNING
            or self.engine_health_percent <= 80.0
            or self.instability_percent >= INSTABILITY_DETECTION_LEVEL
        ):
            self.safety_state = "WARNING"
            self.safety_message = f"Warning limit: {self.limiting_factor}"
        else:
            self.safety_state = "NOMINAL"
            self.safety_message = "Engine conditions nominal"

    def _update_limiting_factor(self, pressure: float, temperature: float) -> None:
        """Choose strongest current or accumulated degradation mechanism."""
        pressure_score = max(
            self.wall_stress_percent,
            100.0 * band_severity(pressure, PRESSURE_WARNING, PRESSURE_FAILURE),
        )
        temperature_score = max(
            100.0 - self.cooling_efficiency_percent,
            100.0
            * band_severity(
                temperature, TEMPERATURE_WARNING, TEMPERATURE_FAILURE
            ),
        )
        scores = {
            "Chamber Pressure": pressure_score,
            "Temperature": temperature_score,
            "Nozzle Erosion": self.nozzle_erosion_percent,
            "Combustion Instability": self.instability_percent,
        }
        factor, score = max(scores.items(), key=lambda item: item[1])
        if score < 1.0:
            self.limiting_factor = "None"
            self.recommendation = "Engine conditions nominal."
        else:
            self.limiting_factor = factor
            recommendations = {
                "Chamber Pressure": "Reduce mass flow or increase nozzle coefficient.",
                "Temperature": (
                    "Reduce combustion temperature or improve cooling efficiency."
                ),
                "Nozzle Erosion": "Reduce chamber pressure or shorten burn duration.",
                "Combustion Instability": (
                    "Reduce mass flow and smooth startup ramp."
                ),
            }
            self.recommendation = recommendations[factor]

    def trigger_emergency_shutdown(self) -> None:
        """Stop generated mass flow while allowing chamber blowdown to continue."""
        if not self.finished:
            self.emergency_shutdown = True
            self.safety_message = "Emergency shutdown active; chamber blowing down"

    def drain_events(self) -> list[str]:
        """Return one-time simulation events for the GUI event log."""
        events = self.pending_events.copy()
        self.pending_events.clear()
        return events

    def results(self) -> dict[str, np.ndarray]:
        """Return current history as NumPy arrays."""
        return {key: np.asarray(values) for key, values in self.history.items()}


def simulate(config: dict[str, float] | None = None) -> dict[str, np.ndarray]:
    """Run full fixed-timestep simulation and return time series arrays."""
    simulation = ChamberSimulation(config or default_config())
    while simulation.step():
        pass
    return simulation.results()


def plot_line(ax, time_s: np.ndarray, values: np.ndarray, title: str, ylabel: str) -> None:
    """Draw one time-series graph on an existing Matplotlib axis."""
    ax.plot(time_s, values, linewidth=2)
    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.35)


def add_safety_limits(ax, key: str) -> None:
    """Draw educational warning and failure limits on relevant graphs."""
    if key == "pressure":
        ax.axhline(PRESSURE_WARNING, color="#d97706", linestyle="--", label="Warning")
        ax.axhline(PRESSURE_CRITICAL, color="#ea580c", linestyle="-.", label="Critical")
        ax.axhline(PRESSURE_FAILURE, color="#7f1d1d", linestyle=":", label="Failure")
        ax.legend(loc="upper right", fontsize=8)
    elif key == "temperature":
        ax.axhline(TEMPERATURE_WARNING, color="#d97706", linestyle="--", label="Warning")
        ax.axhline(TEMPERATURE_CRITICAL, color="#ea580c", linestyle="-.", label="Critical")
        ax.axhline(TEMPERATURE_FAILURE, color="#7f1d1d", linestyle=":", label="Failure")
        ax.legend(loc="upper right", fontsize=8)
    elif key == "engine_health_percent":
        ax.axhline(80.0, color="#d97706", linestyle="--", label="Warning")
        ax.axhline(50.0, color="#ea580c", linestyle="-.", label="Critical")
        ax.axhline(20.0, color="#7f1d1d", linestyle=":", label="Severe")
        ax.legend(loc="upper right", fontsize=8)


def save_plot(
    time_s: np.ndarray,
    values: np.ndarray,
    title: str,
    ylabel: str,
    filename: str,
    key: str,
) -> Path:
    """Create one labeled line plot and save it to output/."""
    fig = Figure(figsize=(10, 5), dpi=100)
    ax = fig.add_subplot(111)
    plot_line(ax, time_s, values, title, ylabel)
    add_safety_limits(ax, key)
    fig.tight_layout()

    path = OUTPUT_DIR / filename
    fig.savefig(path, dpi=150)
    return path


def save_results_graphs(results: dict[str, np.ndarray]) -> list[Path]:
    """Save all graph PNG files into output/."""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    time_s = results["time"]
    return [
        save_plot(time_s, results[key], title, ylabel, filename, key)
        for key, title, ylabel, filename in GRAPH_DEFINITIONS
    ]


def save_results_csv(results: dict[str, np.ndarray]) -> Path:
    """Export current simulation history to a CSV file."""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUTPUT_DIR / "simulation_data.csv"
    columns = [
        "time",
        "pressure",
        "temperature",
        "mass_flow_in",
        "mass_flow_out",
        "gas_mass",
        "effective_exhaust_velocity",
        "thrust",
        "engine_health_percent",
        "cooling_efficiency_percent",
        "wall_stress_percent",
        "nozzle_erosion_percent",
        "instability_percent",
        "safety_state",
        "limiting_factor",
        "recommendation",
    ]
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(columns)
        writer.writerows(zip(*(results[column] for column in columns)))
    return path


def open_saved_graphs(paths: list[Path]) -> bool:
    """Open saved PNG charts with an available desktop file opener."""
    if os.environ.get("ASTARA_NO_OPEN") == "1":
        return False

    opener = shutil.which("wslview") or shutil.which("xdg-open")
    if opener:
        for path in paths:
            subprocess.Popen(
                [opener, str(path.resolve())],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        return True

    explorer = shutil.which("explorer.exe")
    wslpath = shutil.which("wslpath")
    if explorer and wslpath:
        for path in paths:
            result = subprocess.run(
                [wslpath, "-w", str(path.resolve())],
                check=True,
                capture_output=True,
                text=True,
            )
            subprocess.Popen(
                [explorer, result.stdout.strip()],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        return True

    return False


class RealtimeSimulationApp:
    """Tkinter dashboard with sliders, live plots, and simulation controls."""

    def __init__(self, root) -> None:
        import tkinter as tk
        from tkinter import ttk
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

        self.tk = tk
        self.ttk = ttk
        self.root = root
        self.root.title("Realtime Rocket Combustion Chamber Simulation")
        self.root.geometry("1440x920")

        self.variables: dict[str, tk.DoubleVar] = {}
        self.value_labels: dict[str, tk.StringVar] = {}
        self.simulation: ChamberSimulation | None = None
        self.running = False
        self.pending_config_change = False
        self.last_safety_state = "NOMINAL"

        self.status_var = tk.StringVar(value="Ready")
        self.metric_vars = {
            "time": tk.StringVar(value="Time: 0.00 s"),
            "pressure": tk.StringVar(value="Pressure: 0 Pa"),
            "temperature": tk.StringVar(value="Temperature: 293 K"),
            "thrust": tk.StringVar(value="Thrust: 0 N"),
            "peak_pressure": tk.StringVar(value="Peak pressure: 0 Pa"),
            "peak_temperature": tk.StringVar(value="Peak temperature: 293 K"),
            "peak_thrust": tk.StringVar(value="Peak thrust: 0 N"),
        }
        self.health_vars = {
            "engine_health_percent": tk.StringVar(value="Engine Health: 100%"),
            "cooling_efficiency_percent": tk.StringVar(
                value="Cooling Efficiency: 100%"
            ),
            "wall_stress_percent": tk.StringVar(value="Wall Stress: 0%"),
            "nozzle_erosion_percent": tk.StringVar(value="Nozzle Erosion: 0%"),
            "instability_percent": tk.StringVar(value="Instability: 0%"),
            "limiting_factor": tk.StringVar(value="Limiting Factor: None"),
            "recommendation": tk.StringVar(
                value="Recommendation: Engine conditions nominal."
            ),
        }
        self.health_progress_var = tk.DoubleVar(value=100.0)

        self._build_layout()
        self._build_plots(FigureCanvasTkAgg, NavigationToolbar2Tk)
        self.reset_simulation()

    def _build_layout(self) -> None:
        root = self.root
        ttk = self.ttk

        root.columnconfigure(0, weight=0)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(0, weight=1)

        self.sidebar_frame = ttk.Frame(root)
        self.sidebar_frame.grid(row=0, column=0, sticky="ns")
        self.sidebar_frame.rowconfigure(0, weight=1)
        self.sidebar_frame.columnconfigure(0, weight=1)

        self.sidebar_canvas = self.tk.Canvas(
            self.sidebar_frame, width=430, highlightthickness=0
        )
        sidebar_scrollbar = ttk.Scrollbar(
            self.sidebar_frame,
            orient="vertical",
            command=self.sidebar_canvas.yview,
        )
        self.sidebar_canvas.configure(yscrollcommand=sidebar_scrollbar.set)
        self.sidebar_canvas.grid(row=0, column=0, sticky="nsew")
        sidebar_scrollbar.grid(row=0, column=1, sticky="ns")

        self.controls_frame = ttk.Frame(self.sidebar_canvas, padding=12)
        self.sidebar_window = self.sidebar_canvas.create_window(
            (0, 0), window=self.controls_frame, anchor="nw"
        )
        self.controls_frame.bind(
            "<Configure>",
            lambda _event: self.sidebar_canvas.configure(
                scrollregion=self.sidebar_canvas.bbox("all")
            ),
        )
        self.sidebar_canvas.bind(
            "<Configure>",
            lambda event: self.sidebar_canvas.itemconfigure(
                self.sidebar_window, width=event.width
            ),
        )

        self.plot_frame = ttk.Frame(root, padding=8)
        self.plot_frame.grid(row=0, column=1, sticky="nsew")
        self.plot_frame.columnconfigure(0, weight=1)
        self.plot_frame.rowconfigure(2, weight=1)

        self.safety_banner = self.tk.Label(
            self.plot_frame,
            text="NOMINAL - Engine conditions nominal",
            bg="#166534",
            fg="white",
            font=("TkDefaultFont", 12, "bold"),
            padx=10,
            pady=8,
        )
        self.safety_banner.grid(row=0, column=0, sticky="ew", pady=(0, 6))

        ttk.Label(
            self.controls_frame,
            text="Simulation Controls",
            font=("TkDefaultFont", 14, "bold"),
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 12))

        for row, (key, label, default, min_value, max_value, _step) in enumerate(
            CONTROL_SPECS, start=1
        ):
            var = self.tk.DoubleVar(value=default)
            value_var = self.tk.StringVar(value=self._format_control_value(key, default))
            self.variables[key] = var
            self.value_labels[key] = value_var

            ttk.Label(self.controls_frame, text=label).grid(row=row, column=0, sticky="w")
            slider = ttk.Scale(
                self.controls_frame,
                from_=min_value,
                to=max_value,
                variable=var,
                command=lambda _value, name=key: self._on_control_change(name),
            )
            slider.grid(row=row, column=1, sticky="ew", padx=8, pady=4)
            ttk.Label(self.controls_frame, textvariable=value_var, width=12).grid(
                row=row, column=2, sticky="e"
            )

        self.controls_frame.columnconfigure(1, weight=1)

        button_row = len(CONTROL_SPECS) + 1
        ttk.Button(self.controls_frame, text="Start", command=self.start).grid(
            row=button_row, column=0, sticky="ew", pady=(14, 4)
        )
        ttk.Button(self.controls_frame, text="Pause", command=self.pause).grid(
            row=button_row, column=1, sticky="ew", padx=8, pady=(14, 4)
        )
        ttk.Button(self.controls_frame, text="Reset", command=self.reset_simulation).grid(
            row=button_row, column=2, sticky="ew", pady=(14, 4)
        )

        self.tk.Button(
            self.controls_frame,
            text="EMERGENCY SHUTDOWN",
            command=self.emergency_shutdown,
            bg="#b91c1c",
            fg="white",
            activebackground="#991b1b",
            activeforeground="white",
            font=("TkDefaultFont", 10, "bold"),
        ).grid(row=button_row + 1, column=0, columnspan=3, sticky="ew", pady=4)

        ttk.Button(self.controls_frame, text="Save PNGs", command=self.save_current_graphs).grid(
            row=button_row + 2, column=0, columnspan=2, sticky="ew", pady=4
        )
        ttk.Button(self.controls_frame, text="Export CSV", command=self.export_csv).grid(
            row=button_row + 2, column=2, sticky="ew", padx=(8, 0), pady=4
        )

        ttk.Label(self.controls_frame, textvariable=self.status_var).grid(
            row=button_row + 3, column=0, columnspan=3, sticky="w", pady=(12, 4)
        )

        metrics_start = button_row + 4
        for offset, metric_var in enumerate(self.metric_vars.values(), start=metrics_start):
            ttk.Label(self.controls_frame, textvariable=metric_var).grid(
                row=offset, column=0, columnspan=3, sticky="w"
            )

        health_row = metrics_start + len(self.metric_vars)
        ttk.Separator(self.controls_frame, orient="horizontal").grid(
            row=health_row,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=(12, 8),
        )
        ttk.Label(
            self.controls_frame,
            text="Engine Health",
            font=("TkDefaultFont", 12, "bold"),
        ).grid(row=health_row + 1, column=0, columnspan=3, sticky="w")
        ttk.Progressbar(
            self.controls_frame,
            variable=self.health_progress_var,
            maximum=100.0,
        ).grid(
            row=health_row + 2,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=(5, 7),
        )

        health_labels_start = health_row + 3
        health_label_keys = [
            "engine_health_percent",
            "cooling_efficiency_percent",
            "wall_stress_percent",
            "nozzle_erosion_percent",
            "instability_percent",
            "limiting_factor",
        ]
        for offset, key in enumerate(health_label_keys, start=health_labels_start):
            ttk.Label(self.controls_frame, textvariable=self.health_vars[key]).grid(
                row=offset, column=0, columnspan=3, sticky="w"
            )

        recommendation_row = health_labels_start + len(health_label_keys)
        ttk.Label(
            self.controls_frame,
            textvariable=self.health_vars["recommendation"],
            wraplength=390,
            justify="left",
        ).grid(row=recommendation_row, column=0, columnspan=3, sticky="w")

        event_row = recommendation_row + 1
        ttk.Label(
            self.controls_frame,
            text="Event Log",
            font=("TkDefaultFont", 10, "bold"),
        ).grid(row=event_row, column=0, columnspan=3, sticky="w", pady=(10, 4))
        self.event_list = self.tk.Listbox(self.controls_frame, height=6, width=48)
        self.event_list.grid(row=event_row + 1, column=0, columnspan=3, sticky="nsew")

    def _build_plots(self, canvas_cls, toolbar_cls) -> None:
        self.figure = Figure(figsize=(12, 8), dpi=100)
        axes = self.figure.subplots(3, 2)
        self.axes = axes.flatten()
        self.lines = {}

        for ax, (key, title, ylabel, _filename) in zip(self.axes, GRAPH_DEFINITIONS):
            (line,) = ax.plot([], [], linewidth=2)
            self.lines[key] = line
            ax.set_title(title)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.35)
            add_safety_limits(ax, key)

        self.figure.tight_layout(pad=2.0)

        self.canvas = canvas_cls(self.figure, master=self.plot_frame)
        toolbar = toolbar_cls(self.canvas, self.plot_frame, pack_toolbar=False)
        toolbar.grid(row=1, column=0, sticky="ew")
        self.canvas.get_tk_widget().grid(row=2, column=0, sticky="nsew")

    def _format_control_value(self, key: str, value: float) -> str:
        if key == "nozzle_coefficient":
            return f"{value:.2e}"
        if key in {"chamber_volume", "nozzle_exit_area", "time_step"}:
            return f"{value:.3f}"
        if key in {"burn_duration", "propellant_mass_flow_rate"}:
            return f"{value:.2f}"
        return f"{value:.0f}"

    def _on_control_change(self, key: str) -> None:
        value = self.variables[key].get()
        self.value_labels[key].set(self._format_control_value(key, value))
        has_history = bool(
            self.simulation is not None and self.simulation.history["time"]
        )
        if has_history and not self.simulation.finished:
            self.pending_config_change = True
            run_state = "Running" if self.running else "Paused"
            self.status_var.set(f"{run_state}; changed settings apply after Reset")
        elif not self.running:
            self.reset_simulation(redraw_only=True)

    def read_config(self) -> dict[str, float]:
        config = {key: var.get() for key, var in self.variables.items()}
        validate_config(config)
        return config

    def reset_simulation(self, redraw_only: bool = False) -> None:
        self.running = False
        try:
            self.simulation = ChamberSimulation(self.read_config())
        except ValueError as error:
            self.status_var.set(f"Invalid config: {error}")
            return
        self.pending_config_change = False
        if not redraw_only:
            self.event_list.delete(0, self.tk.END)
            self._log_event("Simulation reset")
        self.last_safety_state = "NOMINAL"
        self.status_var.set("Ready" if not redraw_only else "Ready with updated knobs")
        self._refresh_plots()

    def start(self) -> None:
        if self.simulation is None or self.simulation.finished:
            self.reset_simulation()
        if self.running:
            return
        self.running = True
        self.status_var.set("Running")
        self._log_event("Simulation started")
        self._tick()

    def pause(self) -> None:
        if not self.running:
            return
        self.running = False
        self.status_var.set("Paused")
        self._log_event("Simulation paused")

    def emergency_shutdown(self) -> None:
        if self.simulation is None or self.simulation.finished:
            return
        was_running = self.running
        self.simulation.trigger_emergency_shutdown()
        self.running = True
        self.status_var.set("Emergency shutdown: propellant flow stopped")
        self._log_event("EMERGENCY SHUTDOWN triggered")
        self._refresh_plots()
        if not was_running:
            self._tick()

    def _tick(self) -> None:
        if not self.running or self.simulation is None:
            return

        for _ in range(SIM_STEPS_PER_UPDATE):
            if not self.simulation.step():
                self.running = False
                self.save_current_graphs()
                if self.simulation.safety_state == "FAILURE":
                    self.status_var.set("SIMULATED ENGINE FAILURE. Simulation stopped.")
                else:
                    self.status_var.set("Complete. Saved PNG files to output/")
                break

            if self.simulation.finished:
                self.running = False
                self.save_current_graphs()
                if self.simulation.safety_state == "FAILURE":
                    self.status_var.set("SIMULATED ENGINE FAILURE. Simulation stopped.")
                else:
                    self.status_var.set("Complete. Saved PNG files to output/")
                break

        self._refresh_plots()
        if self.running:
            self.root.after(GUI_UPDATE_MS, self._tick)

    def _refresh_plots(self) -> None:
        if self.simulation is None:
            return

        results = self.simulation.results()
        time_s = results["time"]
        for key, _title, _ylabel, _filename in GRAPH_DEFINITIONS:
            self.lines[key].set_data(time_s, results[key])

        x_max = max(float(time_s[-1]) if len(time_s) else 0.1, 0.1)
        for ax, (key, _title, _ylabel, _filename) in zip(self.axes, GRAPH_DEFINITIONS):
            values = results[key]
            ax.set_xlim(0, max(self.simulation.total_time, x_max))
            if key == "engine_health_percent":
                ax.set_ylim(0, 105)
            elif len(values):
                y_max = max(float(np.max(values)), 1.0)
                y_min = min(float(np.min(values)), 0.0)
                padding = max((y_max - y_min) * 0.10, y_max * 0.05, 1.0)
                ax.set_ylim(y_min - padding, y_max + padding)
            else:
                ax.set_ylim(0, 1)

        self._refresh_metrics(results)
        self._refresh_engine_health()
        self._refresh_safety_display()
        for event in self.simulation.drain_events():
            self._log_event(event)
        self.canvas.draw_idle()

    def _refresh_safety_display(self) -> None:
        if self.simulation is None:
            return

        state = self.simulation.safety_state
        message = self.simulation.safety_message
        colors = {
            "NOMINAL": "#166534",
            "WARNING": "#b45309",
            "CRITICAL": "#c2410c",
            "FAILURE": "#7f1d1d",
        }
        self.safety_banner.configure(
            text=f"{state} - {message}",
            bg=colors.get(state, "#374151"),
        )
        if state != self.last_safety_state:
            if state in {"WARNING", "CRITICAL", "FAILURE"}:
                self._log_event(f"Engine entered {state}: {message}")
            elif state == "NOMINAL":
                self._log_event("Engine returned to NOMINAL")
            self.last_safety_state = state

    def _refresh_engine_health(self) -> None:
        if self.simulation is None:
            return

        simulation = self.simulation
        self.health_progress_var.set(simulation.engine_health_percent)
        self.health_vars["engine_health_percent"].set(
            f"Engine Health: {simulation.engine_health_percent:.1f}%"
        )
        self.health_vars["cooling_efficiency_percent"].set(
            f"Cooling Efficiency: {simulation.cooling_efficiency_percent:.1f}%"
        )
        self.health_vars["wall_stress_percent"].set(
            f"Wall Stress: {simulation.wall_stress_percent:.1f}%"
        )
        self.health_vars["nozzle_erosion_percent"].set(
            f"Nozzle Erosion: {simulation.nozzle_erosion_percent:.1f}%"
        )
        self.health_vars["instability_percent"].set(
            f"Instability: {simulation.instability_percent:.1f}%"
        )
        self.health_vars["limiting_factor"].set(
            f"Limiting Factor: {simulation.limiting_factor}"
        )
        self.health_vars["recommendation"].set(
            f"Recommendation: {simulation.recommendation}"
        )

    def _refresh_metrics(self, results: dict[str, np.ndarray]) -> None:
        if len(results["time"]) == 0:
            self.metric_vars["time"].set("Time: 0.00 s")
            self.metric_vars["pressure"].set("Pressure: 0 Pa")
            self.metric_vars["temperature"].set(f"Temperature: {AMBIENT_TEMPERATURE:.0f} K")
            self.metric_vars["thrust"].set("Thrust: 0 N")
            self.metric_vars["peak_pressure"].set("Peak pressure: 0 Pa")
            self.metric_vars["peak_temperature"].set(
                f"Peak temperature: {AMBIENT_TEMPERATURE:.0f} K"
            )
            self.metric_vars["peak_thrust"].set("Peak thrust: 0 N")
            return

        self.metric_vars["time"].set(f"Time: {results['time'][-1]:.2f} s")
        self.metric_vars["pressure"].set(f"Pressure: {results['pressure'][-1]:,.0f} Pa")
        self.metric_vars["temperature"].set(
            f"Temperature: {results['temperature'][-1]:,.0f} K"
        )
        self.metric_vars["thrust"].set(f"Thrust: {results['thrust'][-1]:,.0f} N")
        self.metric_vars["peak_pressure"].set(
            f"Peak pressure: {np.max(results['pressure']):,.0f} Pa"
        )
        self.metric_vars["peak_temperature"].set(
            f"Peak temperature: {np.max(results['temperature']):,.0f} K"
        )
        self.metric_vars["peak_thrust"].set(
            f"Peak thrust: {np.max(results['thrust']):,.0f} N"
        )

    def _log_event(self, message: str) -> None:
        time_s = self.simulation.time_s if self.simulation is not None else 0.0
        self.event_list.insert(self.tk.END, f"{time_s:6.2f}s  {message}")
        self.event_list.see(self.tk.END)

    def save_current_graphs(self) -> None:
        if self.simulation is None:
            return
        saved_paths = save_results_graphs(self.simulation.results())
        self.status_var.set(f"Saved {len(saved_paths)} PNG files to output/")
        self._log_event(f"Saved {len(saved_paths)} PNG files")

    def export_csv(self) -> None:
        if self.simulation is None:
            return
        path = save_results_csv(self.simulation.results())
        self.status_var.set(f"Exported data to {path}")
        self._log_event("Exported simulation_data.csv")


def show_realtime_window() -> bool:
    """Open realtime Tkinter simulation dashboard."""
    global GUI_DISPLAY_NOTE

    if os.environ.get("ASTARA_NO_GUI") == "1":
        GUI_DISPLAY_NOTE = "ASTARA_NO_GUI=1 was set."
        return False

    try:
        import tkinter as tk
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: F401
    except ModuleNotFoundError as error:
        GUI_DISPLAY_NOTE = f"GUI dependency unavailable: {error}"
        return False

    try:
        root = tk.Tk()
    except tk.TclError as error:
        GUI_DISPLAY_NOTE = f"tkinter could not connect to display: {error}"
        return False

    RealtimeSimulationApp(root)
    root.mainloop()
    return True


def main() -> None:
    if show_realtime_window():
        return

    results = simulate(default_config())
    saved_paths = save_results_graphs(results)
    if open_saved_graphs(saved_paths):
        print("Realtime GUI unavailable; opened saved PNG charts with desktop viewer.")
    elif GUI_DISPLAY_NOTE:
        print(f"Graphs were saved but not displayed. Reason: {GUI_DISPLAY_NOTE}")
    else:
        print("Graphs were saved but no display method was available.")

    print("Simulation complete. Saved graphs:")
    for path in saved_paths:
        print(f"- {path}")


if __name__ == "__main__":
    main()
