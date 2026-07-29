"""Timestep-convergence and Monte Carlo credibility analysis."""

from __future__ import annotations

import copy
import json
import math
import os
import traceback
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import numpy as np

from .. import __version__
from ..configuration.scenarios import (
    model_source_hash,
    scenario_hash,
)
from ..configuration.schemas import CREDIBILITY_SCHEMA_VERSION
from ..configuration.validation import validate_scenario
from ..flight_software.build import build_library
from ..simulation.runner import RunResult, run_simulation
from .artifacts import (
    artifact_hashes,
    write_configuration_artifacts,
    write_csv,
    write_json,
)
from .retention import same_metric, selected_success_samples

PERCENTILE_METRICS = (
    "maximum_altitude_m",
    "maximum_mach",
    "maximum_dynamic_pressure_pa",
    "maximum_angle_of_attack_deg",
    "aero_out_of_envelope_fraction",
    "aero_out_of_envelope_pre_recovery_fraction",
    "minimum_engine_health_percent",
    "duration_s",
    "stage1_burnout_time_s",
    "stage2_burnout_time_s",
)
COMPARISON_FIELDS = (
    *PERCENTILE_METRICS,
    "core_impact_latitude_deg",
    "core_impact_longitude_deg",
    "upper_impact_latitude_deg",
    "upper_impact_longitude_deg",
    "status",
    "failure_reason",
)
SUCCESS_STATUSES = {"PASS", "PASS_WITH_MODEL_WARNINGS"}
FAILURE_STATUSES = {"FAIL", "CANCELLED", "ERROR"}


def _event_time(result: RunResult, event_name: str) -> float | None:
    return next(
        (
            float(event["time_s"])
            for event in result.events
            if event["event"] == event_name
        ),
        None,
    )


def _failure_reason(manifest: dict[str, Any]) -> str:
    if manifest.get("cancelled"):
        return "simulation_cancelled"
    failed_checks = sorted(
        name for name, passed in manifest.get("checks", {}).items() if not passed
    )
    return "|".join(failed_checks)


def _metrics(result: RunResult) -> dict[str, Any]:
    summary_metrics = result.manifest.get("summary_metrics", {})
    telemetry_count = max(
        int(summary_metrics.get("telemetry_samples", len(result.telemetry))),
        1,
    )
    impacts = result.manifest.get("impact_points", {})
    core_impact = impacts.get("core_stage", {})
    upper_impact = impacts.get("upper_stage", {})
    return {
        "maximum_altitude_m": float(result.manifest["maximum_altitude_m"]),
        "maximum_mach": float(
            summary_metrics.get(
                "maximum_mach",
                max(
                    (float(row["mach"]) for row in result.telemetry),
                    default=0.0,
                ),
            )
        ),
        "maximum_dynamic_pressure_pa": float(
            summary_metrics.get(
                "maximum_dynamic_pressure_pa",
                max(
                    (
                        float(row["dynamic_pressure_pa"])
                        for row in result.telemetry
                    ),
                    default=0.0,
                ),
            )
        ),
        "maximum_angle_of_attack_deg": float(
            summary_metrics.get(
                "maximum_angle_of_attack_deg",
                max(
                    (
                        float(row["angle_of_attack_deg"])
                        for row in result.telemetry
                    ),
                    default=0.0,
                ),
            )
        ),
        "aero_out_of_envelope_fraction": float(
            result.manifest["aero_out_of_envelope_samples"]
        )
        / telemetry_count,
        "aero_out_of_envelope_pre_recovery_fraction": float(
            result.manifest["aero_out_of_envelope_pre_recovery_samples"]
        )
        / telemetry_count,
        "minimum_engine_health_percent": float(
            summary_metrics.get(
                "minimum_engine_health_percent",
                min(
                    (
                        float(row["engine_health_percent"])
                        for row in result.telemetry
                    ),
                    default=100.0,
                ),
            )
        ),
        "duration_s": float(result.manifest["duration_s"]),
        "stage1_burnout_time_s": _event_time(result, "burnout_stage_1"),
        "stage2_burnout_time_s": _event_time(result, "burnout_stage_2"),
        "core_impact_latitude_deg": core_impact.get("latitude_deg"),
        "core_impact_longitude_deg": core_impact.get("longitude_deg"),
        "upper_impact_latitude_deg": upper_impact.get("latitude_deg"),
        "upper_impact_longitude_deg": upper_impact.get("longitude_deg"),
        "status": str(result.manifest["status"]),
        "failure_reason": _failure_reason(result.manifest),
    }


def _empty_metrics(reason: str) -> dict[str, Any]:
    return {
        **{name: None for name in PERCENTILE_METRICS},
        "core_impact_latitude_deg": None,
        "core_impact_longitude_deg": None,
        "upper_impact_latitude_deg": None,
        "upper_impact_longitude_deg": None,
        "status": "ERROR",
        "failure_reason": reason,
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


def _percentiles(rows: list[dict[str, Any]]) -> dict[str, dict[str, float | int | None]]:
    summary: dict[str, dict[str, float | int | None]] = {}
    for name in PERCENTILE_METRICS:
        values = np.asarray(
            [
                row[name]
                for row in rows
                if isinstance(row.get(name), (int, float))
                and math.isfinite(float(row[name]))
            ],
            dtype=float,
        )
        if not len(values):
            summary[name] = {
                "count": 0,
                "minimum": None,
                "p05": None,
                "median": None,
                "mean": None,
                "p95": None,
                "maximum": None,
            }
            continue
        summary[name] = {
            "count": int(len(values)),
            "minimum": float(np.min(values)),
            "p05": float(np.percentile(values, 5)),
            "median": float(np.median(values)),
            "mean": float(np.mean(values)),
            "p95": float(np.percentile(values, 95)),
            "maximum": float(np.max(values)),
        }
    return summary


def available_cpu_count() -> int:
    if hasattr(os, "sched_getaffinity"):
        return max(len(os.sched_getaffinity(0)), 1)
    return max(os.cpu_count() or 1, 1)


def worker_count(samples: int, requested: int | None = None) -> tuple[int, int]:
    available = available_cpu_count()
    if requested is not None:
        if requested < 1:
            raise ValueError("workers must be at least 1")
        if requested > available:
            raise ValueError(
                f"workers cannot exceed {available} available logical CPUs"
            )
        return available, min(samples, requested)
    reserved = max(1, math.ceil(available * 0.25))
    return available, max(1, min(samples, available - reserved))


def _monte_carlo_worker(
    job: tuple[int, int, dict[str, Any], dict[str, float]],
) -> dict[str, Any]:
    sample, simulation_seed, sampled_scenario, factors = job
    try:
        result = run_simulation(
            sampled_scenario,
            seed=simulation_seed,
            create_report=False,
            persist=False,
            summary_only=True,
        )
        metrics = _metrics(result)
        error_traceback = ""
    except Exception as error:
        metrics = _empty_metrics(f"{type(error).__name__}: {error}")
        error_traceback = traceback.format_exc()
    return {
        "sample": sample,
        "simulation_seed": simulation_seed,
        **factors,
        **metrics,
        "telemetry_retained": 0,
        "retention_reason": "",
        "telemetry_path": "",
        "retention_metrics_match": "",
        "retention_error": "",
        "_error_traceback": error_traceback,
    }


def _retention_worker(
    job: tuple[
        int,
        int,
        dict[str, Any],
        dict[str, Any],
        str,
        str,
    ],
) -> dict[str, Any]:
    (
        sample,
        simulation_seed,
        sampled_scenario,
        reference,
        output_root,
        reason,
    ) = job
    try:
        result = run_simulation(
            sampled_scenario,
            seed=simulation_seed,
            output_root=output_root,
            create_report=False,
            persist=True,
        )
        retained_metrics = _metrics(result)
        matches = all(
            same_metric(reference.get(name), retained_metrics.get(name))
            for name in COMPARISON_FIELDS
        )
        return {
            "sample": sample,
            "simulation_seed": simulation_seed,
            "reason": reason,
            "path": str(result.output_dir),
            "metrics_match": matches,
            "error": "",
            "traceback": "",
        }
    except Exception as error:
        return {
            "sample": sample,
            "simulation_seed": simulation_seed,
            "reason": reason,
            "path": "",
            "metrics_match": False,
            "error": f"{type(error).__name__}: {error}",
            "traceback": traceback.format_exc(),
        }


def _run_jobs(
    function: Callable[[Any], dict[str, Any]],
    jobs: list[Any],
    workers: int,
) -> list[dict[str, Any]]:
    if not jobs:
        return []
    active_workers = min(workers, len(jobs))
    if active_workers == 1:
        return [function(job) for job in jobs]
    with ProcessPoolExecutor(max_workers=active_workers) as executor:
        return list(executor.map(function, jobs, chunksize=1))


def run_credibility_analysis(
    scenario: dict[str, Any],
    samples: int | None = None,
    seed: int | None = None,
    output_root: str | Path = "runs",
    workers: int | None = None,
    telemetry_sample_percent: float | None = None,
) -> Path:
    """Write convergence, parallel Monte Carlo, and provenance evidence."""
    validate_scenario(scenario)
    monte_carlo = scenario.get("monte_carlo", {})
    if samples is None:
        samples = int(monte_carlo.get("samples", 20))
    if seed is None:
        seed = int(monte_carlo.get("seed", 1))
    if telemetry_sample_percent is None:
        telemetry_sample_percent = float(
            monte_carlo.get("telemetry_sample_percent", 2.0)
        )
    if samples < 1:
        raise ValueError("samples must be at least 1")
    if not 0.0 <= telemetry_sample_percent <= 100.0:
        raise ValueError("telemetry_sample_percent must be between 0 and 100")
    available_cpus, active_workers = worker_count(samples, workers)
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

    build_library()
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
            candidate,
            seed=seed,
            create_report=False,
            persist=False,
            summary_only=True,
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
    jobs = []
    for index in range(samples):
        sampled_scenario, factors = _sample_scenario(scenario, rng)
        jobs.append((index + 1, seed + index + 1, sampled_scenario, factors))
    monte_carlo_rows = _run_jobs(_monte_carlo_worker, jobs, active_workers)
    monte_carlo_rows.sort(key=lambda row: int(row["sample"]))

    worker_errors: dict[int, str] = {}
    for row in monte_carlo_rows:
        error_traceback = row.pop("_error_traceback")
        if error_traceback:
            worker_errors[int(row["sample"])] = error_traceback

    retained_successes = selected_success_samples(
        monte_carlo_rows,
        seed,
        telemetry_sample_percent,
        SUCCESS_STATUSES,
    )
    retention_reasons = {
        int(row["sample"]): "failure"
        for row in monte_carlo_rows
        if row["status"] in FAILURE_STATUSES
    }
    retention_reasons.update(
        {
            sample: "random_sample"
            for sample in retained_successes
            if sample not in retention_reasons
        }
    )

    rows_by_sample = {
        int(row["sample"]): row for row in monte_carlo_rows
    }
    jobs_by_sample = {job[0]: job for job in jobs}
    retained_root = output_dir / "retained_runs"
    retention_jobs = []
    if retention_reasons:
        retained_root.mkdir()
        for sample, reason in sorted(retention_reasons.items()):
            _, simulation_seed, sampled_scenario, _ = jobs_by_sample[sample]
            retention_jobs.append(
                (
                    sample,
                    simulation_seed,
                    sampled_scenario,
                    rows_by_sample[sample],
                    str(retained_root),
                    reason,
                )
            )
    retained_results = _run_jobs(
        _retention_worker, retention_jobs, active_workers
    )
    retained_rows = []
    retention_errors: dict[int, str] = {}
    for retained in retained_results:
        sample = int(retained["sample"])
        row = rows_by_sample[sample]
        relative_path = ""
        if retained["path"]:
            relative_path = str(
                Path(retained["path"]).relative_to(output_dir)
            )
            row["telemetry_retained"] = 1
        row["retention_reason"] = retained["reason"]
        row["telemetry_path"] = relative_path
        row["retention_metrics_match"] = retained["metrics_match"]
        row["retention_error"] = retained["error"]
        if retained["traceback"]:
            retention_errors[sample] = retained["traceback"]
        retained_rows.append(
            {
                "sample": sample,
                "simulation_seed": retained["simulation_seed"],
                "reason": retained["reason"],
                "path": relative_path,
                "metrics_match": retained["metrics_match"],
                "error": retained["error"],
            }
        )

    error_artifacts = []
    all_error_samples = sorted(set(worker_errors) | set(retention_errors))
    if all_error_samples:
        error_root = output_dir / "errors"
        error_root.mkdir()
        for sample in all_error_samples:
            path = error_root / f"sample-{sample}.json"
            path.write_text(
                json.dumps(
                    {
                        "sample": sample,
                        "simulation_seed": rows_by_sample[sample][
                            "simulation_seed"
                        ],
                        "summary_traceback": worker_errors.get(sample, ""),
                        "retention_traceback": retention_errors.get(sample, ""),
                    },
                    indent=2,
                ),
                encoding="utf-8",
            )
            error_artifacts.append(path)

    write_csv(output_dir / "convergence.csv", convergence_rows)
    write_csv(output_dir / "monte_carlo.csv", monte_carlo_rows)
    if retained_rows:
        write_csv(output_dir / "retained_runs.csv", retained_rows)
    configuration_artifacts = write_configuration_artifacts(
        output_dir, scenario
    )

    status_counts = Counter(str(row["status"]) for row in monte_carlo_rows)
    reason_counts = Counter(
        str(row["failure_reason"])
        for row in monte_carlo_rows
        if row["failure_reason"]
    )
    summary = {
        "schema_version": CREDIBILITY_SCHEMA_VERSION,
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
        "execution": {
            "available_logical_cpus": available_cpus,
            "workers": active_workers,
            "reserved_logical_cpus": max(available_cpus - active_workers, 0),
            "policy": (
                "explicit"
                if workers is not None
                else "affinity-aware reserve 25 percent"
            ),
        },
        "convergence": {
            "horizon_s": convergence_horizon_s,
            "criterion_max_relative_change": criterion,
            "observed_max_relative_change": convergence_rows[0][
                "max_relative_change_vs_finest"
            ],
            "passed": convergence_passed,
        },
        "monte_carlo": _percentiles(monte_carlo_rows),
        "status_counts": dict(sorted(status_counts.items())),
        "failure_reason_counts": dict(sorted(reason_counts.items())),
        "telemetry_retention": {
            "successful_sample_percent": telemetry_sample_percent,
            "selected_successful_runs": len(retained_successes),
            "failure_runs": sum(
                count
                for status, count in status_counts.items()
                if status in FAILURE_STATUSES
            ),
            "requested_runs": len(retention_reasons),
            "retained_runs": sum(
                int(row["telemetry_retained"]) for row in monte_carlo_rows
            ),
            "directory": "retained_runs",
        },
        "interpretation": (
            "Convergence checks numerical sensitivity. Monte Carlo reflects only the "
            "declared provisional input distributions. Neither constitutes physical "
            "validation."
        ),
    }

    artifact_paths = [
        *configuration_artifacts,
        output_dir / "convergence.csv",
        output_dir / "monte_carlo.csv",
        *([output_dir / "retained_runs.csv"] if retained_rows else []),
        *error_artifacts,
    ]
    summary["artifacts"] = artifact_hashes(output_dir, artifact_paths)
    write_json(output_dir / "summary.json", summary)
    return output_dir
