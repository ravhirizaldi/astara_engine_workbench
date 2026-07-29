"""Aerospace Workbench PNG and PDF evidence generation."""

from __future__ import annotations

from collections import defaultdict
import os
import tempfile
from pathlib import Path

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(tempfile.gettempdir()) / "aerospace_workbench_matplotlib"),
)

import matplotlib

matplotlib.use("Agg")
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.figure import Figure

from ..simulation.runner import RunResult
from .manifest import register_artifacts, write_manifest


def _group(result: RunResult) -> dict[str, list[dict]]:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in result.telemetry:
        grouped[row["body"]].append(row)
    return grouped


def _line_figure(
    grouped: dict[str, list[dict]], key: str, title: str, ylabel: str
) -> Figure:
    figure = Figure(figsize=(10, 5.5), layout="constrained")
    axis = figure.add_subplot(111)
    for body, rows in grouped.items():
        axis.plot([row["time_s"] for row in rows], [row[key] for row in rows], label=body)
    axis.set_title(title)
    axis.set_xlabel("Time (s)")
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.3)
    axis.legend()
    return figure


def create_report_artifacts(result: RunResult) -> None:
    grouped = _group(result)
    definitions = (
        ("altitude_m", "Altitude vs Time", "Altitude above launch (m)", "altitude.png"),
        ("speed_m_s", "Speed vs Time", "Speed (m/s)", "speed.png"),
        ("mach", "Mach vs Time", "Mach", "mach.png"),
        ("dynamic_pressure_pa", "Dynamic Pressure vs Time", "Dynamic pressure (Pa)", "dynamic_pressure.png"),
        ("engine_health_percent", "Engine Health vs Time", "Health (%)", "engine_health.png"),
        ("angle_of_attack_deg", "Angle of Attack vs Time", "Angle (deg)", "angle_of_attack.png"),
    )
    figures: list[Figure] = []
    for key, title, ylabel, filename in definitions:
        figure = _line_figure(grouped, key, title, ylabel)
        figure.savefig(result.output_dir / filename, dpi=140)
        figures.append(figure)

    trajectory = Figure(figsize=(9, 7), layout="constrained")
    trajectory_axis = trajectory.add_subplot(111, projection="3d")
    for body, rows in grouped.items():
        origin = rows[0]
        trajectory_axis.plot(
            [
                (row["position_ecef_x_m"] - origin["position_ecef_x_m"]) / 1000.0
                for row in rows
            ],
            [
                (row["position_ecef_y_m"] - origin["position_ecef_y_m"]) / 1000.0
                for row in rows
            ],
            [
                (row["position_ecef_z_m"] - origin["position_ecef_z_m"]) / 1000.0
                for row in rows
            ],
            label=body,
        )
    trajectory_axis.set_title("Relative ECEF Trajectory")
    trajectory_axis.set_xlabel("ΔX (km)")
    trajectory_axis.set_ylabel("ΔY (km)")
    trajectory_axis.set_zlabel("ΔZ (km)")
    trajectory_axis.legend()
    trajectory.savefig(result.output_dir / "trajectory_3d.png", dpi=140)
    figures.append(trajectory)

    summary = Figure(figsize=(10, 7), layout="constrained")
    axis = summary.add_subplot(111)
    axis.axis("off")
    maximum_altitude = max(
        (row["altitude_m"] for row in result.telemetry), default=0.0
    )
    maximum_mach = max((row["mach"] for row in result.telemetry), default=0.0)
    axis.text(
        0.02,
        0.98,
        "\n".join(
            (
                "ASTARA DIGITAL TWIN — SIMULATION ONLY / UNVALIDATED",
                "",
                f"Scenario: {result.manifest['scenario_name']}",
                f"Seed: {result.manifest['seed']}",
                f"Duration: {result.manifest['duration_s']:.2f} s",
                f"Maximum altitude: {maximum_altitude:,.1f} m",
                f"Maximum Mach: {maximum_mach:.2f}",
                f"Aero out-of-envelope samples: {result.manifest['aero_out_of_envelope_samples']}",
                "  before recovery: "
                f"{result.manifest['aero_out_of_envelope_pre_recovery_samples']}",
                "  during recovery: "
                f"{result.manifest['aero_out_of_envelope_recovery_samples']}",
                f"Landing status: {result.manifest['landed']}",
                "",
                "These outputs are engineering estimates, not flight certification evidence.",
            )
        ),
        va="top",
        family="monospace",
        fontsize=12,
    )
    with PdfPages(result.output_dir / "report.pdf") as pdf:
        pdf.savefig(summary)
        for figure in figures:
            pdf.savefig(figure)
    filenames = (
        *[item[3] for item in definitions],
        "trajectory_3d.png",
        "report.pdf",
    )
    register_artifacts(
        result.manifest,
        result.output_dir,
        (result.output_dir / filename for filename in filenames),
    )
    write_manifest(result.output_dir, result.manifest)
