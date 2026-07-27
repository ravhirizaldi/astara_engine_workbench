"""ASTARA command-line entrypoints for reproducible digital-twin runs."""

from __future__ import annotations

import argparse
from collections import deque
import json
import sys
import time
from pathlib import Path
from typing import Any, Callable, TextIO

from .analysis import run_credibility_analysis
from .flight_core import build_library
from .replay import replay_fsw
from .scenario import default_scenario_path, load_scenario, validate_scenario
from .twin import FSW_TIMING_MODES, run_simulation


class SimulationProgressReporter:
    """Render throttled CLI progress from simulation callback samples."""

    IMPORTANT_EVENTS = {
        "flight_mode": "mode transition",
        "stage_separation": "stage separation",
        "stage2_ignition": "stage-two ignition",
        "drogue_deployed": "drogue deployment",
        "main_deployed": "main deployment",
        "landed": "landing",
        "abort": "abort",
    }

    def __init__(
        self,
        max_time_s: float,
        interval_s: float = 1.0,
        stream: TextIO | None = None,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.max_time_s = max_time_s
        self.interval_s = interval_s
        self.stream = stream if stream is not None else sys.stderr
        self.clock = clock or time.perf_counter
        self.interactive = self.stream.isatty()
        self.started_wall_s = 0.0
        self.last_render_wall_s = 0.0
        self.last_simulation_time_s = 0.0
        self.last_row: dict[str, Any] = {}
        self.event_count = 0
        self.seen_faults: dict[str, int] = {}
        self.aborted_bodies: set[str] = set()
        self.window: deque[tuple[float, float]] = deque()
        self.line_active = False

    def start(self) -> None:
        now = self.clock()
        self.started_wall_s = now
        self.last_render_wall_s = now
        self.window.append((now, 0.0))
        self._render(now, complete=False)

    def update(
        self,
        simulation_time_s: float,
        rows: list[dict[str, Any]],
        events: list[dict[str, Any]],
    ) -> None:
        now = self.clock()
        self.last_simulation_time_s = simulation_time_s
        if rows:
            self.last_row = self._select_row(rows)
        self.window.append((now, simulation_time_s))
        while len(self.window) > 1 and now - self.window[0][0] > 5.0:
            self.window.popleft()

        event_printed = False
        for event in events[self.event_count :]:
            label = self.IMPORTANT_EVENTS.get(str(event.get("event")))
            if label:
                detail = str(event.get("detail") or "")
                if label == "mode transition" and detail:
                    label = f"{label} -> {detail}"
                self._print_event(
                    float(event.get("time_s", simulation_time_s)),
                    str(event.get("body", "unknown")),
                    label,
                )
                event_printed = True
        self.event_count = len(events)

        for row in rows:
            body = str(row.get("body", "unknown"))
            if row.get("fsw_abort") and body not in self.aborted_bodies:
                self.aborted_bodies.add(body)
                self._print_event(simulation_time_s, body, "abort")
                event_printed = True
            active = int(row.get("fsw_active_fault_flags", 0))
            latched = int(row.get("fsw_latched_fault_flags", 0))
            faults = active | latched
            if faults & ~self.seen_faults.get(body, 0):
                names = str(row.get("fsw_faults") or "unknown")
                self._print_event(
                    simulation_time_s,
                    body,
                    f"new active or latched faults: {names}",
                )
                event_printed = True
            self.seen_faults[body] = self.seen_faults.get(body, 0) | faults

        if event_printed and self.interactive:
            self._render(now, complete=False)
        elif now - self.last_render_wall_s >= self.interval_s:
            self._render(now, complete=False)

    def finish(self) -> None:
        self._render(self.clock(), complete=True)

    def cancel(self) -> None:
        self._clear_line()
        self.stream.write("Simulation cancelled.\n")
        self.stream.flush()

    @staticmethod
    def _select_row(rows: list[dict[str, Any]]) -> dict[str, Any]:
        by_body = {str(row.get("body")): row for row in rows}
        return (
            by_body.get("upper_stage")
            or by_body.get("integrated_stack")
            or rows[0]
        )

    def _throughput(self) -> float:
        if len(self.window) < 2:
            return 0.0
        first_wall, first_sim = self.window[0]
        last_wall, last_sim = self.window[-1]
        wall_delta = last_wall - first_wall
        return max(0.0, (last_sim - first_sim) / wall_delta) if wall_delta > 0 else 0.0

    @staticmethod
    def _duration(seconds: float | None) -> str:
        if seconds is None:
            return "--:--"
        total = max(0, round(seconds))
        hours, remainder = divmod(total, 3600)
        minutes, secs = divmod(remainder, 60)
        return (
            f"{hours:02d}:{minutes:02d}:{secs:02d}"
            if hours
            else f"{minutes:02d}:{secs:02d}"
        )

    def _line(self, now: float, complete: bool) -> str:
        throughput = self._throughput()
        remaining_sim_s = max(0.0, self.max_time_s - self.last_simulation_time_s)
        eta_s = remaining_sim_s / throughput if throughput > 0 else None
        percent = (
            100.0
            if complete
            else min(
                100.0,
                100.0 * self.last_simulation_time_s / self.max_time_s
                if self.max_time_s > 0
                else 0.0,
            )
        )
        altitude_m = float(self.last_row.get("fsw_estimated_altitude_m", 0.0))
        altitude = (
            f"{altitude_m / 1000.0:,.1f} km"
            if abs(altitude_m) >= 1000.0
            else f"{altitude_m:,.0f} m"
        )
        return (
            f"[{percent:5.1f}%] "
            f"T+{self.last_simulation_time_s:.1f}/{self.max_time_s:.1f}s | "
            f"{self.last_row.get('body', 'integrated_stack')} | "
            f"{self.last_row.get('mode', 'INIT')} | "
            f"alt {altitude} | "
            f"{float(self.last_row.get('speed_m_s', 0.0)):,.0f} m/s | "
            f"{throughput:,.1f} sim-s/s | "
            f"elapsed {self._duration(now - self.started_wall_s)} | "
            f"ETA {self._duration(0.0 if complete else eta_s)}"
        )

    def _render(self, now: float, complete: bool) -> None:
        line = self._line(now, complete)
        if self.interactive:
            self.stream.write(f"\r\033[2K{line}")
            self.line_active = True
            if complete:
                self.stream.write("\n")
                self.line_active = False
        else:
            self.stream.write(f"{line}\n")
        self.stream.flush()
        self.last_render_wall_s = now

    def _clear_line(self) -> None:
        if self.interactive and self.line_active:
            self.stream.write("\r\033[2K")
            self.line_active = False

    def _print_event(self, simulation_time_s: float, body: str, message: str) -> None:
        self._clear_line()
        self.stream.write(f"[event T+{simulation_time_s:.1f}s] {body}: {message}\n")
        self.stream.flush()


def _positive_float(value: str) -> float:
    number = float(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def _nonnegative_float(value: str) -> float:
    number = float(value)
    if number < 0:
        raise argparse.ArgumentTypeError("must be nonnegative")
    return number


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m astara")
    commands = parser.add_subparsers(dest="command", required=True)

    simulate = commands.add_parser("simulate", help="run a deterministic SIL mission")
    simulate.add_argument(
        "scenario", nargs="?", default=str(default_scenario_path())
    )
    simulate.add_argument("--seed", type=int)
    simulate.add_argument("--output", default="runs")
    simulate.add_argument("--no-report", action="store_true")
    simulate.add_argument("--progress", action="store_true")
    simulate.add_argument("--progress-interval", type=_positive_float, default=1.0)
    simulate.add_argument("--quiet", action="store_true")
    simulate.add_argument(
        "--timing-mode",
        choices=FSW_TIMING_MODES,
        default="deterministic",
        help="FSW timing source (default: deterministic)",
    )
    simulate.add_argument(
        "--injected-execution-time",
        type=_nonnegative_float,
        metavar="SECONDS",
    )

    validate = commands.add_parser("validate", help="validate a scenario")
    validate.add_argument(
        "scenario", nargs="?", default=str(default_scenario_path())
    )

    analyze = commands.add_parser(
        "analyze", help="run timestep-convergence and Monte Carlo analysis"
    )
    analyze.add_argument(
        "scenario", nargs="?", default=str(default_scenario_path())
    )
    analyze.add_argument("--samples", type=int)
    analyze.add_argument("--seed", type=int)
    analyze.add_argument("--workers", type=int)
    analyze.add_argument("--telemetry-percent", type=float)
    analyze.add_argument("--output", default="runs")

    replay = commands.add_parser(
        "replay", help="replay a recorded sensors.csv through the C++ flight core"
    )
    replay.add_argument("sensor_log")
    replay.add_argument(
        "--scenario", default=str(default_scenario_path())
    )
    replay.add_argument("--output")

    commands.add_parser("build-fsw", help="build the C++ flight core")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    arguments = parser.parse_args(argv)
    if arguments.command == "build-fsw":
        print(build_library())
        return 0
    scenario = load_scenario(arguments.scenario)
    if arguments.command == "validate":
        validate_scenario(scenario)
        print(json.dumps({"valid": True, "scenario": scenario["name"]}))
        return 0
    if arguments.command == "replay":
        print(replay_fsw(scenario, arguments.sensor_log, arguments.output))
        return 0
    if arguments.command == "analyze":
        output_dir = run_credibility_analysis(
            scenario,
            samples=arguments.samples,
            seed=arguments.seed,
            output_root=Path(arguments.output),
            workers=arguments.workers,
            telemetry_sample_percent=arguments.telemetry_percent,
        )
        print(output_dir)
        print((output_dir / "summary.json").read_text(encoding="utf-8"))
        return 0
    if (
        arguments.timing_mode == "injected"
    ) != (arguments.injected_execution_time is not None):
        parser.error(
            "--injected-execution-time is required only with "
            "--timing-mode injected"
        )
    show_progress = not arguments.quiet and (
        arguments.progress or sys.stdout.isatty() or sys.stderr.isatty()
    )
    reporter = (
        SimulationProgressReporter(
            float(scenario["simulation"]["max_time_s"]),
            arguments.progress_interval,
        )
        if show_progress
        else None
    )
    if reporter:
        reporter.start()
    try:
        result = run_simulation(
            scenario,
            seed=arguments.seed,
            output_root=Path(arguments.output),
            create_report=not arguments.no_report,
            on_sample=reporter.update if reporter else None,
            timing_mode=arguments.timing_mode,
            injected_execution_time_s=arguments.injected_execution_time,
        )
    except KeyboardInterrupt:
        if reporter:
            reporter.cancel()
        else:
            print("Simulation cancelled.", file=sys.stderr)
        return 130
    if reporter:
        reporter.update(
            float(result.manifest.get("duration_s", reporter.last_simulation_time_s)),
            [],
            getattr(result, "events", []),
        )
        reporter.finish()
    print(result.output_dir)
    print(json.dumps(result.manifest, indent=2))
    return 0
