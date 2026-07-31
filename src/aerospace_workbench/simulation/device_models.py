"""Reduced-order models for non-navigation avionics devices."""

from __future__ import annotations

import random
from typing import Any


class _DeviceModel:
    def __init__(
        self,
        config: dict[str, Any],
        seed: int,
        boot_epoch_s: float = 0.0,
    ) -> None:
        self.config = config
        self._rng = random.Random(seed)
        self._boot_epoch_s = boot_epoch_s
        self._last: dict[str, Any] | None = None
        self._feedback: dict[str, bool] = {}
        self._pending_feedback: dict[str, tuple[float, bool]] = {}
        self._pending_commands: list[tuple[float, int, float]] = []

    @property
    def timeout_s(self) -> float:
        return float(self.config["timeout_s"])

    def reset(self, reset_epoch_s: float) -> None:
        self._boot_epoch_s = reset_epoch_s
        self._pending_feedback.clear()
        self._pending_commands.clear()
        if self.config["reset_behavior"] == "invalidate":
            self._last = None
            self._feedback.clear()

    def acknowledge(
        self,
        command: int,
        time_s: float,
        issue_time_s: float | None = None,
    ) -> bool:
        behavior = self.config["command_acknowledgment"]
        if behavior == "drop":
            return False
        delay_s = (
            0.0
            if behavior == "immediate"
            else float(self.config["command_ack_delay_s"])
        )
        self._pending_commands.append(
            (
                time_s + delay_s,
                command,
                time_s if issue_time_s is None else issue_time_s,
            )
        )
        return True

    def _acknowledged_command(self, time_s: float) -> tuple[int, float]:
        if not self._pending_commands or self._pending_commands[0][0] > time_s:
            return 0, time_s
        _due_s, command, issue_time_s = self._pending_commands.pop(0)
        return command, issue_time_s

    def _quantize(self, value: float) -> float:
        quantum = float(self.config["quantization"])
        return round(value / quantum) * quantum if quantum > 0.0 else value

    def _accurate_bool(self, value: bool) -> bool:
        accuracy = float(self.config.get("accuracy", 1.0))
        return value if self._rng.random() <= accuracy else not value

    def _acknowledged_feedback(
        self, name: str, truth: bool, time_s: float
    ) -> bool:
        current = self._feedback.setdefault(name, False)
        pending = self._pending_feedback.get(name)
        if pending and pending[0] <= time_s:
            current = pending[1]
            self._feedback[name] = current
            del self._pending_feedback[name]
        if truth != current and name not in self._pending_feedback:
            behavior = self.config["command_acknowledgment"]
            if behavior != "drop":
                delay_s = (
                    0.0
                    if behavior == "immediate"
                    else float(self.config["command_ack_delay_s"])
                )
                if delay_s == 0.0:
                    current = truth
                    self._feedback[name] = truth
                else:
                    self._pending_feedback[name] = (time_s + delay_s, truth)
        return self._accurate_bool(current)

    def _publish(
        self,
        values: dict[str, Any],
        sample_time_s: float,
        fault: dict[str, Any] | None,
    ) -> dict[str, Any]:
        started = (
            sample_time_s + 1e-12
            >= self._boot_epoch_s + float(self.config["startup_delay_s"])
        )
        if (
            not started
            and self.config["reset_behavior"] == "hold_last"
            and self._last is not None
        ):
            return dict(self._last)
        valid = bool(self.config["valid"]) and started
        sample = values | {"sample_time_s": sample_time_s, "valid": int(valid)}
        fault_type = str((fault or {}).get("type", ""))
        if fault_type == "dropout":
            sample["valid"] = 0
        elif fault_type in {"freeze", "stuck"} and self._last is not None:
            sample = dict(self._last)
            if fault_type == "stuck":
                sample["sample_time_s"] = sample_time_s
        self._last = dict(sample)
        return sample


class AirDataComputerModel(_DeviceModel):
    def sample(
        self,
        body: Any,
        sample_time_s: float,
        fault: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        pressure = max(
            0.0,
            float(body.last_dynamic_pressure_pa)
            + self._rng.gauss(0.0, float(self.config["noise_stddev"])),
        )
        sample = self._publish(
            {"dynamic_pressure_pa": self._quantize(pressure)},
            sample_time_s,
            fault,
        )
        if str((fault or {}).get("type", "")) not in {"freeze", "stuck"}:
            sample["valid"] &= int(bool(body.aero_valid))
        self._last = dict(sample)
        return sample


class EngineControllerModel(_DeviceModel):
    def __init__(
        self,
        config: dict[str, Any],
        seed: int,
        boot_epoch_s: float = 0.0,
    ) -> None:
        super().__init__(config, seed, boot_epoch_s)
        self._health_percent = 100.0

    def reset(self, reset_epoch_s: float) -> None:
        super().reset(reset_epoch_s)
        if self.config["reset_behavior"] == "invalidate":
            self._health_percent = 100.0

    def sample(
        self,
        body: Any,
        sample_time_s: float,
        fault: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        propulsion = body.stage["propulsion"]
        nominal_pressure = max(float(propulsion["chamber_pressure_pa"]), 1.0)
        nominal_temperature = max(
            float(propulsion["combustion_temperature_k"]), 294.15
        )
        nominal_rpm = max(float(self.config.get("nominal_rpm", 30_000.0)), 1.0)
        pressure = max(float(body.last_chamber_pressure_pa), 0.0)
        temperature = max(float(body.last_engine_temperature_k), 0.0)
        rpm = max(float(body.last_engine_rpm), 0.0)
        running = rpm > 1.0
        if running:
            pressure_ratio = pressure / nominal_pressure
            temperature_ratio = (temperature - 293.15) / (
                nominal_temperature - 293.15
            )
            rpm_ratio = rpm / nominal_rpm
            estimated = 100.0 - (
                45.0 * abs(pressure_ratio - rpm_ratio**2)
                + 30.0 * abs(temperature_ratio - pressure_ratio)
                + 25.0 * max(temperature_ratio - 1.05, 0.0)
            )
            self._health_percent = min(
                self._health_percent,
                max(0.0, min(100.0, estimated)),
            )
        noisy_health = self._health_percent + self._rng.gauss(
            0.0, float(self.config["noise_stddev"])
        )
        ready_truth = (
            body.fuel_kg > 0.0
            and body.oxidizer_kg > 0.0
            and self._health_percent > 0.0
        )
        return self._publish(
            {
                "health_percent": self._quantize(
                    max(0.0, min(100.0, noisy_health))
                ),
                "ready": int(
                    self._acknowledged_feedback(
                        "ready", ready_truth, sample_time_s
                    )
                ),
                "running": int(
                    self._acknowledged_feedback(
                        "running", running, sample_time_s
                    )
                ),
                "chamber_pressure_pa": pressure,
                "temperature_k": temperature,
                "rpm": rpm,
            },
            sample_time_s,
            fault,
        )


class DiscreteInputModule(_DeviceModel):
    def sample(
        self,
        body: Any,
        sample_time_s: float,
        fault: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        return self._publish(
            {
                "stage_separated": int(
                    self._acknowledged_feedback(
                        "stage_separated",
                        body.name != "integrated_stack",
                        sample_time_s,
                    )
                )
            },
            sample_time_s,
            fault,
        )


class RecoveryControllerModel(_DeviceModel):
    def sample(
        self,
        body: Any,
        sample_time_s: float,
        fault: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        return self._publish(
            {
                "drogue_deployed": int(
                    self._acknowledged_feedback(
                        "drogue_deployed",
                        bool(body.drogue_deployed),
                        sample_time_s,
                    )
                ),
                "main_deployed": int(
                    self._acknowledged_feedback(
                        "main_deployed",
                        bool(body.main_deployed),
                        sample_time_s,
                    )
                ),
            },
            sample_time_s,
            fault,
        )


class FlightComputerPlatformModel(_DeviceModel):
    def sample(
        self,
        sample_time_s: float,
        previous_execution_time_s: float,
        deadline_missed: bool,
        watchdog_healthy: bool,
        command: tuple[int, float] | None = None,
        fault: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        if command is not None:
            self.acknowledge(command[0], sample_time_s, command[1])
        command_type, command_issue_time_s = self._acknowledged_command(
            sample_time_s
        )
        return self._publish(
            {
                "previous_execution_time_s": self._quantize(
                    max(previous_execution_time_s, 0.0)
                ),
                "deadline_missed": int(
                    self._accurate_bool(deadline_missed)
                ),
                "watchdog_healthy": int(
                    self._accurate_bool(watchdog_healthy)
                ),
                "command_type": command_type,
                "command_issue_time_s": command_issue_time_s,
            },
            sample_time_s,
            fault,
        )
