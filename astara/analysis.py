"""ASTARA timestep-convergence and Monte Carlo credibility analysis."""

from __future__ import annotations

import copy
import csv
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from . import __version__
from .scenario import evidence_documents, model_source_hash, scenario_hash, validate_scenario
from .twin import RunResult, run_simulation


def _metrics(result: RunResult) -> dict[str, float]:
    telemetry_count = max(len(result.telemetry), 1)
    return {
        "maximum_altitude_m": float(result.manifest["maximum_altitude_m"]),
        "maximum_mach": max(
            (float(row["mach"]) for row in result.telemetry), default=0.0
        ),
        "maximum_dynamic_pressure_pa": max(
            (float(row["dynamic_pressure_pa"]) for row in result.telemetry),
            default=0.0,
        ),
        "maximum_angle_of_attack_deg": max(
            (float(row["angle_of_attack_deg"]) for row in result.telemetry),
            default=0.0,
        ),
        "aero_out_of_envelope_fraction": float(
            result.manifest["aero_out_of_envelope_samples"]
        )
        / telemetry_count,
        "aero_out_of_envelope_pre_recovery_fraction": float(
            result.manifest["aero_out_of_envelope_pre_recovery_samples"]
        )
        / telemetry_count,
        "minimum_engine_health_percent": min(
            (float(row["engine_health_percent"]) for row in result.telemetry),
            default=100.0,
        ),
        "duration_s": float(result.manifest["duration_s"]),
    }


def _sample_scenario(
    scenario: dict[str, Any], rng: np.random.Generator
) -> tuple[dict[str, Any], dict[str, float]]:
    sampled = copy.deepcopy(scenario)
    uncertainty = scenario.get("uncertainty", {})
    factors = {
        "thrust_scale": max(
            0.1,
            float(rng.normal(1.0, uncertainty.get("thrust_scale_1sigma", 0.0))),
        ),
        "drag_scale": max(
            0.1,
            float(rng.normal(1.0, uncertainty.get("drag_scale_1sigma", 0.0))),
        ),
        "dry_mass_scale": max(
            0.1,
            float(rng.normal(1.0, uncertainty.get("dry_mass_scale_1sigma", 0.0))),
        ),
        "timing_offset_s": float(
            rng.normal(0.0, uncertainty.get("timing_1sigma_s", 0.0))
        ),
    }
    wind_sigma = float(uncertainty.get("wind_1sigma_m_s", 0.0))
    wind_offsets = rng.normal(0.0, wind_sigma, 3)
    factors.update(
        {
            "wind_north_offset_m_s": float(wind_offsets[0]),
            "wind_east_offset_m_s": float(wind_offsets[1]),
            "wind_down_offset_m_s": float(wind_offsets[2]),
        }
    )

    for stage in sampled["vehicle"]["stages"]:
        stage["dry_mass_kg"] *= factors["dry_mass_scale"]
        stage["inertia_kg_m2"] = [
            value * factors["dry_mass_scale"]
            for value in stage["inertia_kg_m2"]
        ]
        for row in stage.get("mass_properties", []):
            row["inertia_kg_m2"] = [
                value * factors["dry_mass_scale"]
                for value in row["inertia_kg_m2"]
            ]
        curve = stage["propulsion"].get("performance_curve")
        if curve:
            for row in curve:
                row["thrust_n"] *= factors["thrust_scale"]
        else:
            stage["propulsion"]["nozzle_efficiency"] *= factors["thrust_scale"]
        table = stage.get("aerodynamics", {}).get("coefficient_table")
        if table:
            for row in table:
                row["drag_coefficient"] *= factors["drag_scale"]
        else:
            stage["aerodynamics"]["base_drag_coefficient"] *= factors["drag_scale"]

    sampled["environment"]["wind_ned_m_s"] = [
        float(value) + float(offset)
        for value, offset in zip(
            sampled["environment"]["wind_ned_m_s"], wind_offsets, strict=True
        )
    ]
    if "events" in sampled["mission"]:
        for event in sampled["mission"]["events"]:
            event["delay"] = max(
                0.0, float(event["delay"]) + factors["timing_offset_s"]
            )
    else:
        for name in ("separation_delay_s", "stage2_ignition_delay_s"):
            sampled["mission"][name] = max(
                1e-6,
                float(sampled["mission"][name]) + factors["timing_offset_s"],
            )
    validate_scenario(sampled)
    return sampled, factors


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _percentiles(rows: list[dict[str, Any]]) -> dict[str, dict[str, float]]:
    summary: dict[str, dict[str, float]] = {}
    for name in (
        "maximum_altitude_m",
        "maximum_mach",
        "maximum_dynamic_pressure_pa",
        "maximum_angle_of_attack_deg",
        "aero_out_of_envelope_fraction",
        "aero_out_of_envelope_pre_recovery_fraction",
        "minimum_engine_health_percent",
        "duration_s",
    ):
        values = np.asarray([row[name] for row in rows], dtype=float)
        summary[name] = {
            "minimum": float(np.min(values)),
            "p05": float(np.percentile(values, 5)),
            "median": float(np.median(values)),
            "mean": float(np.mean(values)),
            "p95": float(np.percentile(values, 95)),
            "maximum": float(np.max(values)),
        }
    return summary


def run_credibility_analysis(
    scenario: dict[str, Any],
    samples: int | None = None,
    seed: int | None = None,
    output_root: str | Path = "runs",
) -> Path:
    """Write convergence, Monte Carlo, and provenance evidence for one scenario."""
    validate_scenario(scenario)
    monte_carlo = scenario.get("monte_carlo", {})
    if samples is None:
        samples = int(monte_carlo.get("samples", 20))
    if seed is None:
        seed = int(monte_carlo.get("seed", 1))
    if samples < 1:
        raise ValueError("samples must be at least 1")
    credibility = scenario.get("credibility")
    if not isinstance(credibility, dict):
        raise ValueError("credibility metadata is required for analysis")
    missing_sources = {
        "propulsion",
        "aerodynamics",
        "mass_properties",
        "environment",
    } - set(credibility.get("model_sources", {}))
    if missing_sources:
        raise ValueError(
            f"credibility.model_sources is missing: {', '.join(sorted(missing_sources))}"
        )
    uncertainty = scenario.get("uncertainty", {})
    required_uncertainty = {
        "thrust_scale_1sigma",
        "drag_scale_1sigma",
        "dry_mass_scale_1sigma",
        "wind_1sigma_m_s",
        "timing_1sigma_s",
    }
    missing_uncertainty = required_uncertainty - set(uncertainty)
    if missing_uncertainty:
        raise ValueError(
            f"uncertainty is missing: {', '.join(sorted(missing_uncertainty))}"
        )

    analysis_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = (
        Path(output_root)
        / f"{analysis_id}-{scenario_hash(scenario)[:8]}-credibility-s{seed}"
    )
    suffix = 1
    while output_dir.exists():
        output_dir = output_dir.with_name(f"{output_dir.name}-{suffix}")
        suffix += 1
    output_dir.mkdir(parents=True)

    base_dt = float(scenario["simulation"]["time_step_s"])
    convergence_horizon_s = min(
        float(scenario["simulation"]["max_time_s"]),
        float(
            credibility.get("acceptance_criteria", {}).get(
                "convergence_horizon_s", scenario["simulation"]["max_time_s"]
            )
        ),
    )
    convergence_rows: list[dict[str, Any]] = []
    for time_step_s in (base_dt, base_dt / 2.0, base_dt / 4.0):
        candidate = copy.deepcopy(scenario)
        candidate["simulation"].update(
            {"time_step_s": time_step_s, "max_time_s": convergence_horizon_s}
        )
        result = run_simulation(
            candidate, seed=seed, create_report=False, persist=False
        )
        convergence_rows.append({"time_step_s": time_step_s, **_metrics(result)})

    finest = convergence_rows[-1]
    compared_metrics = (
        "maximum_altitude_m",
        "maximum_mach",
        "maximum_dynamic_pressure_pa",
    )
    for row in convergence_rows:
        errors = [
            abs(float(row[name]) - float(finest[name]))
            / max(abs(float(finest[name])), 1e-12)
            for name in compared_metrics
        ]
        row["max_relative_change_vs_finest"] = max(errors)

    criterion = float(
        scenario.get("credibility", {})
        .get("acceptance_criteria", {})
        .get("timestep_max_relative_change", 0.01)
    )
    convergence_passed = (
        float(convergence_rows[0]["max_relative_change_vs_finest"]) <= criterion
    )

    rng = np.random.default_rng(seed)
    monte_carlo_rows: list[dict[str, Any]] = []
    for index in range(samples):
        sampled_scenario, factors = _sample_scenario(scenario, rng)
        simulation_seed = seed + index + 1
        result = run_simulation(
            sampled_scenario,
            seed=simulation_seed,
            create_report=False,
            persist=False,
        )
        monte_carlo_rows.append(
            {
                "sample": index + 1,
                "simulation_seed": simulation_seed,
                **factors,
                **_metrics(result),
            }
        )

    _write_csv(output_dir / "convergence.csv", convergence_rows)
    _write_csv(output_dir / "monte_carlo.csv", monte_carlo_rows)
    scenario_document, vehicle_document = evidence_documents(scenario)
    (output_dir / "scenario.json").write_text(
        json.dumps(scenario_document, indent=2), encoding="utf-8"
    )
    if vehicle_document is not None:
        (output_dir / "vehicle_definition.json").write_text(
            json.dumps(vehicle_document, indent=2), encoding="utf-8"
        )
    summary = {
        "schema_version": "astara.credibility.v1",
        "model_version": __version__,
        "model_source_sha256": model_source_hash(),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scenario_name": scenario["name"],
        "scenario_sha256": scenario_hash(scenario),
        "seed": seed,
        "samples": samples,
        "credibility": credibility,
        "uncertainty": uncertainty,
        "physical_validation_evidence_count": len(
            credibility.get("validation_evidence", [])
        ),
        "convergence": {
            "horizon_s": convergence_horizon_s,
            "criterion_max_relative_change": criterion,
            "observed_max_relative_change": convergence_rows[0][
                "max_relative_change_vs_finest"
            ],
            "passed": convergence_passed,
        },
        "monte_carlo": _percentiles(monte_carlo_rows),
        "interpretation": (
            "Convergence checks numerical sensitivity. Monte Carlo reflects only the "
            "declared provisional input distributions. Neither constitutes physical "
            "validation."
        ),
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )

    artifacts = {}
    for filename in (
        "scenario.json",
        "convergence.csv",
        "monte_carlo.csv",
        *(("vehicle_definition.json",) if vehicle_document is not None else ()),
    ):
        artifacts[filename] = hashlib.sha256(
            (output_dir / filename).read_bytes()
        ).hexdigest()
    summary["artifacts"] = artifacts
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    return output_dir
