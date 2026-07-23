"""ASTARA command-line entrypoints for reproducible digital-twin runs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .analysis import run_credibility_analysis
from .flight_core import build_library
from .replay import replay_fsw
from .scenario import default_scenario_path, load_scenario, validate_scenario
from .twin import run_simulation


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
    arguments = _parser().parse_args(argv)
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
        )
        print(output_dir)
        print((output_dir / "summary.json").read_text(encoding="utf-8"))
        return 0
    result = run_simulation(
        scenario,
        seed=arguments.seed,
        output_root=Path(arguments.output),
        create_report=not arguments.no_report,
    )
    print(result.output_dir)
    print(json.dumps(result.manifest, indent=2))
    return 0
