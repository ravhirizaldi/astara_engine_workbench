import io
import json
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from aerospace_workbench.cli import SimulationProgressReporter, main
from aerospace_workbench.configuration.scenarios import default_scenario


class _Stream(io.StringIO):
    def __init__(self, tty: bool = False) -> None:
        super().__init__()
        self.tty = tty

    def isatty(self) -> bool:
        return self.tty


class _Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def _row(time_s: float = 0.0, body: str = "integrated_stack") -> dict:
    return {
        "time_s": time_s,
        "body": body,
        "mode": "COAST",
        "fsw_estimated_altitude_m": 71_400.0,
        "speed_m_s": 2_130.0,
        "fsw_abort": 0,
        "fsw_active_fault_flags": 0,
        "fsw_latched_fault_flags": 0,
        "fsw_faults": "",
    }


def _result() -> SimpleNamespace:
    return SimpleNamespace(
        output_dir=Path("runs/test-run"),
        manifest={"status": "PASS", "seed": 1, "duration_s": 50.0},
        events=[],
    )


class SimulationProgressReporterTests(unittest.TestCase):
    def test_percentage_eta_startup_and_completion_formatting(self) -> None:
        stream = _Stream()
        clock = _Clock()
        reporter = SimulationProgressReporter(100.0, stream=stream, clock=clock)

        reporter.start()
        clock.now = 2.0
        reporter.update(50.0, [_row(50.0)], [])
        reporter.finish()

        lines = stream.getvalue().splitlines()
        self.assertIn("[  0.0%]", lines[0])
        self.assertIn("ETA --:--", lines[0])
        self.assertIn("[ 50.0%]", lines[1])
        self.assertIn("elapsed 00:02 | ETA 00:02", lines[1])
        self.assertIn("[100.0%]", lines[-1])

    def test_prefers_upper_stage_and_events_preserve_interactive_line(self) -> None:
        stream = _Stream(tty=True)
        clock = _Clock()
        reporter = SimulationProgressReporter(100.0, stream=stream, clock=clock)
        reporter.start()
        clock.now = 1.0
        fault_row = _row(10.0, "upper_stage")
        fault_row.update(
            {
                "fsw_active_fault_flags": 1,
                "fsw_faults": "GNSS_INVALID",
            }
        )

        reporter.update(
            10.0,
            [_row(10.0, "core_stage"), fault_row],
            [
                {
                    "time_s": 10.0,
                    "body": "integrated_stack",
                    "event": "stage_separation",
                    "detail": "",
                }
            ],
        )
        reporter.update(11.0, [fault_row], [])

        output = stream.getvalue()
        self.assertIn("\r\033[2K[event T+10.0s]", output)
        self.assertIn("stage separation", output)
        self.assertIn("new active or latched faults: GNSS_INVALID", output)
        self.assertEqual(output.count("new active or latched faults"), 1)
        self.assertTrue(reporter.line_active)
        self.assertEqual(reporter.last_row["body"], "upper_stage")
        self.assertTrue(output.rstrip().endswith("ETA 00:09"))


class CliProgressTests(unittest.TestCase):
    def _run_cli(
        self,
        arguments: list[str],
        *,
        stderr_tty: bool = False,
    ) -> tuple[_Stream, _Stream, object]:
        stdout = _Stream()
        stderr = _Stream(stderr_tty)

        def run_simulation(_scenario: dict, **kwargs: object) -> SimpleNamespace:
            callback = kwargs["on_sample"]
            events = [
                {
                    "time_s": 50.0,
                    "body": "integrated_stack",
                    "event": "flight_mode",
                    "detail": "COAST",
                }
            ]
            if callback:
                callback(50.0, [_row(50.0)], events)
            result = _result()
            result.events = events + [
                {
                    "time_s": 50.1,
                    "body": "upper_stage",
                    "event": "landed",
                    "detail": "",
                }
            ]
            return result

        with (
            patch(
                "aerospace_workbench.cli.load_scenario",
                return_value=default_scenario(),
            ),
            patch(
                "aerospace_workbench.cli.run_simulation",
                side_effect=run_simulation,
            ) as run,
            patch("sys.stdout", stdout),
            patch("sys.stderr", stderr),
        ):
            self.assertEqual(main(["simulate", *arguments]), 0)
        return stdout, stderr, run

    def test_quiet_disables_progress(self) -> None:
        stdout, stderr, run = self._run_cli(["--quiet"], stderr_tty=True)

        self.assertEqual(stderr.getvalue(), "")
        self.assertIsNone(run.call_args.kwargs["on_sample"])
        self.assertTrue(stdout.getvalue().startswith("runs/test-run\n"))

    def test_progress_is_forced_to_stderr_without_carriage_returns(self) -> None:
        stdout, stderr, run = self._run_cli(["--progress"])

        self.assertIsNotNone(run.call_args.kwargs["on_sample"])
        self.assertNotIn("\r", stderr.getvalue())
        self.assertIn("[100.0%]", stderr.getvalue())
        self.assertIn("upper_stage: landing", stderr.getvalue())
        stdout_lines = stdout.getvalue().splitlines()
        self.assertEqual(stdout_lines[0], "runs/test-run")
        self.assertEqual(json.loads("\n".join(stdout_lines[1:])), _result().manifest)

    def test_progress_is_automatic_when_stderr_is_a_tty(self) -> None:
        _stdout, stderr, run = self._run_cli([], stderr_tty=True)

        self.assertIsNotNone(run.call_args.kwargs["on_sample"])
        self.assertIn("\r\033[2K", stderr.getvalue())

    def test_timing_mode_options_reach_simulation(self) -> None:
        _stdout, _stderr, run = self._run_cli(
            [
                "--timing-mode",
                "injected",
                "--injected-execution-time",
                "0.02",
            ]
        )

        self.assertEqual(run.call_args.kwargs["timing_mode"], "injected")
        self.assertEqual(
            run.call_args.kwargs["injected_execution_time_s"],
            0.02,
        )

        stderr = _Stream()
        with patch("sys.stderr", stderr), self.assertRaises(SystemExit):
            main(["simulate", "--timing-mode", "injected"])
        self.assertIn("--injected-execution-time is required", stderr.getvalue())

    def test_nonpositive_progress_interval_is_rejected(self) -> None:
        stderr = _Stream()
        with patch("sys.stderr", stderr), self.assertRaises(SystemExit):
            main(["simulate", "--progress-interval", "0"])
        self.assertIn("must be greater than zero", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
