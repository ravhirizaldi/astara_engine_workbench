import multiprocessing as mp
import queue
import subprocess
import threading
import time
import unittest
from collections import deque
from unittest.mock import MagicMock, patch

from astara.scenario import default_scenario
from astara.ui import AstaraWorkbench, _run_simulation_process


class LiveUiTests(unittest.TestCase):
    def test_close_releases_process_and_queue(self) -> None:
        app = AstaraWorkbench.__new__(AstaraWorkbench)
        app._closing = False
        app._refresh_after_id = "refresh-id"
        app.root = MagicMock()
        app.cancel_event = MagicMock()
        app.pause_event = MagicMock()
        app.process_messages = MagicMock()
        process = MagicMock()
        process.is_alive.return_value = False
        app.simulation_process = process

        app._close()

        app.cancel_event.set.assert_called_once_with()
        app.pause_event.clear.assert_called_once_with()
        app.root.after_cancel.assert_called_once_with("refresh-id")
        process.join.assert_called_once_with(timeout=5.0)
        process.close.assert_called_once_with()
        app.process_messages.close.assert_called_once_with()
        app.process_messages.join_thread.assert_called_once_with()
        app.root.destroy.assert_called_once_with()

    def test_copy_json_copies_full_viewer_text(self) -> None:
        app = AstaraWorkbench.__new__(AstaraWorkbench)
        app.root = MagicMock()
        app.status_var = MagicMock()
        app.is_wsl = False
        text = MagicMock()
        text.get.return_value = '{"name": "Anthariksa"}'

        app._copy_json(text, "Scenario JSON")

        text.get.assert_called_once_with("1.0", "end-1c")
        app.root.clipboard_clear.assert_called_once_with()
        app.root.clipboard_append.assert_called_once_with('{"name": "Anthariksa"}')
        app.status_var.set.assert_called_once_with("Scenario JSON copied to clipboard")

    @patch("astara.ui.subprocess.run")
    @patch("astara.ui.shutil.which", return_value="/mnt/c/Windows/System32/clip.exe")
    def test_copy_json_uses_windows_clipboard_under_wsl(
        self, _which: MagicMock, run: MagicMock
    ) -> None:
        app = AstaraWorkbench.__new__(AstaraWorkbench)
        app.root = MagicMock()
        app.status_var = MagicMock()
        app.is_wsl = True
        text = MagicMock()
        text.get.return_value = '{"name": "Anthariksa"}'

        app._copy_json(text, "Vehicle JSON")

        run.assert_called_once_with(
            ["/mnt/c/Windows/System32/clip.exe"],
            input='{"name": "Anthariksa"}',
            text=True,
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        app.root.clipboard_clear.assert_not_called()
        app.status_var.set.assert_called_once_with("Vehicle JSON copied to clipboard")

    def test_metrics_refresh_without_forcing_plot_redraw(self) -> None:
        app = AstaraWorkbench.__new__(AstaraWorkbench)
        app.live_lock = threading.Lock()
        app.live_dirty = True
        app.live_rows = deque(
            [
                {
                    "time_s": 1.0,
                    "altitude_m": 2.0,
                    "speed_m_s": 3.0,
                    "mach": 0.1,
                    "thrust_n": 4000.0,
                    "dynamic_pressure_pa": 2500.0,
                    "active_engines": 1,
                    "engine_count": 1,
                    "mode": "BOOST_1",
                }
            ]
        )
        app.live_events = []
        app._rendered_event_count = 0
        app._last_plot_wall = time.perf_counter()
        app.plot_interval_s = 60.0
        app._current_thrust_n = 4000.0
        app._peak_thrust_n = 5000.0
        app.metric_vars = {
            name: MagicMock()
            for name in (
                "time",
                "altitude",
                "speed",
                "mach",
                "thrust",
                "dynamic_pressure",
                "engines",
                "phase",
            )
        }
        app.progress_var = MagicMock()
        app._run_max_time_s = 10.0
        app._render_events = MagicMock()
        app._render_live_values = MagicMock()
        app._plot_rows = MagicMock()
        app.root = MagicMock()
        app.process_messages = queue.Queue()
        app.simulation_process = None
        app._closing = False
        app._refresh_after_id = None

        app._refresh_live()

        app.metric_vars["time"].set.assert_called_once_with("1.0 s")
        app.metric_vars["thrust"].set.assert_called_once_with("4.0 / 5.0 kN")
        app._plot_rows.assert_not_called()
        app.root.after.assert_called_once()

    def test_stream_sums_body_thrust_and_keeps_peak(self) -> None:
        app = AstaraWorkbench.__new__(AstaraWorkbench)
        app.live_lock = threading.Lock()
        app.live_rows = deque()
        app.live_events = []
        app.latest_by_body = {}
        app.live_dirty = False
        app._current_thrust_n = 0.0
        app._peak_thrust_n = 0.0

        app._stream_sample(
            1.0,
            [
                {"body": "core_stage", "thrust_n": 4000.0},
                {"body": "upper_stage", "thrust_n": 2000.0},
            ],
            [],
        )
        app._stream_sample(
            2.0,
            [{"body": "core_stage", "thrust_n": 0.0}],
            [],
        )

        self.assertEqual(app._current_thrust_n, 0.0)
        self.assertEqual(app._peak_thrust_n, 6000.0)

    def test_solver_process_streams_and_finishes(self) -> None:
        scenario = default_scenario()
        scenario["simulation"]["max_time_s"] = scenario["simulation"]["time_step_s"]
        context = mp.get_context("spawn")
        messages = context.Queue()
        cancel_event = context.Event()
        pause_event = context.Event()
        speed = context.Value("d", 0.0)
        process = context.Process(
            target=_run_simulation_process,
            args=(
                scenario,
                1,
                messages,
                cancel_event,
                pause_event,
                speed,
                False,
            ),
        )
        process.start()
        process.join(timeout=10.0)

        self.assertEqual(process.exitcode, 0)
        self.assertEqual(messages.get_nowait()[0], "sample")
        kind, result = messages.get_nowait()
        self.assertEqual(kind, "finished")
        self.assertFalse(result.manifest["cancelled"])


if __name__ == "__main__":
    unittest.main()
