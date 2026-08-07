import copy
import json
import multiprocessing as mp
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from aerospace_workbench.configuration.scenarios import (
    configuration_source_files,
    default_scenario,
    default_scenario_path,
    load_scenario_documents,
    scenario_from_documents,
)
from aerospace_workbench.mathematics.frames import geodetic_to_ecef, ned_basis
from aerospace_workbench.presentation.desktop.app import (
    LIVE_DISPLAY_POINT_LIMIT,
    MAJOR_EVENT_BANNERS,
    EngineeringWorkbench,
)
from aerospace_workbench.presentation.desktop.mission_view import (
    DISPLAY_ROW_FIELDS,
    run_simulation_process,
)
from aerospace_workbench.presentation.plotting import live_telemetry_series


class LiveUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.qt_app = QApplication.instance() or QApplication([])
        cls.window = EngineeringWorkbench()
        cls.window.refresh_timer.stop()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.window._dirty = False
        cls.window._shutdown()
        cls.window.deleteLater()
        cls.qt_app.processEvents()

    @staticmethod
    def _display_row(
        body: str,
        time_s: float,
        position: np.ndarray,
        altitude_m: float,
        *,
        speed_m_s: float = 0.0,
        thrust_n: float = 0.0,
        mach: float = 0.0,
        mode: str = "BOOST_1",
        landed: int = 0,
    ) -> dict[str, float | int | str]:
        return {
            "body": body,
            "time_s": time_s,
            "position_ecef_x_m": float(position[0]),
            "position_ecef_y_m": float(position[1]),
            "position_ecef_z_m": float(position[2]),
            "altitude_m": altitude_m,
            "speed_m_s": speed_m_s,
            "thrust_n": thrust_n,
            "mach": mach,
            "mode": mode,
            "landed": landed,
        }

    @patch("main.show_workbench", return_value=True)
    def test_main_launcher_opens_only_astara(self, show_workbench: MagicMock) -> None:
        import main

        main.main()

        show_workbench.assert_called_once_with()

    def test_manual_trajectory_zoom_disables_auto_follow(self) -> None:
        self.window.auto_follow_check.setChecked(True)

        self.window.trajectory_plot.getViewBox().sigRangeChangedManually.emit(
            [True, True]
        )

        self.assertFalse(self.window.auto_follow_check.isChecked())
        self.window.auto_follow_check.setChecked(True)

    def test_new_run_restores_trajectory_auto_follow(self) -> None:
        self.window.auto_follow_check.setChecked(False)
        process = MagicMock()
        with (
            patch.object(
                self.window,
                "_apply_editors",
                return_value=copy.deepcopy(self.window.working_scenario),
            ),
            patch.object(self.window.process_context, "Process", return_value=process),
        ):
            self.window._start_run()

        self.assertTrue(self.window.auto_follow_check.isChecked())
        process.start.assert_called_once_with()
        self.window.simulation_process = None

    def test_major_event_banners_cover_mission_milestones(self) -> None:
        expected = {
            "hold_down_released",
            "rail_exit",
            "max_q",
            "meco",
            "stage_separation",
            "stage2_ignition",
            "stage2_first_cutoff",
            "stage2_second_ignition",
            "orbit_insertion",
            "payload_deploy",
            "drogue_deployed",
            "main_deployed",
            "landed",
            "mission_complete",
        }
        self.assertLessEqual(expected, set(MAJOR_EVENT_BANNERS))

        self.window._banner_event_count = 0
        self.window._render_events(
            [
                {
                    "time_s": 94.1,
                    "body": "integrated_stack",
                    "event": "max_q",
                    "detail": {"dynamic_pressure_pa": 32_061.6},
                }
            ]
        )

        self.assertEqual(self.window.mission_banner.text(), "MAX-Q · 32.1 kPa")

    def test_body_selector_allows_pre_sample_empty_state(self) -> None:
        self.window.latest_by_body = {}
        self.window.body_combo.clear()

        self.window._update_body_selector()

        self.assertEqual(self.window.body_combo.count(), 0)

    def test_console_loads_full_working_copy_and_orbit_overlay(self) -> None:
        self.assertIn("mission", self.window.scenario_document)
        self.assertIn("vehicle", self.window.vehicle_document)
        self.assertIsNotNone(self.window.orbit_line)
        self.assertFalse(self.window.save_button.isEnabled())
        runtime = self.window._apply_editors(False)
        self.assertEqual(
            set(configuration_source_files(runtime)), {"scenario", "vehicle"}
        )

    def test_live_view_has_three_lightweight_telemetry_panels(self) -> None:
        self.assertFalse(hasattr(self.window, "telemetry_dock"))
        self.assertIsNotNone(self.window.altitude_plot)
        self.assertIsNotNone(self.window.speed_plot)
        self.assertIsNotNone(self.window.thrust_plot)
        self.assertEqual(
            set(self.window.metric_labels),
            {"time", "altitude", "speed", "phase"},
        )
        self.assertLessEqual(
            self.window.trajectory_capacity,
            LIVE_DISPLAY_POINT_LIMIT + 1,
        )

    def test_fault_dock_queues_valid_engine_fault(self) -> None:
        process = MagicMock()
        process.is_alive.return_value = True
        self.window.simulation_process = process
        self.window.fault_component_combo.setCurrentText("engine")
        self.window.fault_type_combo.setCurrentText("thrust_scale")
        self.window.fault_value_edit.setText("0.5")
        self.window.fault_duration_spin.setValue(2.0)

        with patch.object(self.window.control_messages, "put_nowait") as enqueue:
            self.window._inject_fault()

        enqueue.assert_called_once_with(
            {
                "action": "inject",
                "body": "all",
                "component": "engine",
                "fault_type": "thrust_scale",
                "duration_s": 2.0,
                "value": 0.5,
            }
        )
        self.window.simulation_process = None

    def test_header_buttons_toggle_docks(self) -> None:
        button = self.window.configuration_button
        dock = self.window.configuration_dock
        button.setChecked(False)
        dock.hide()

        button.click()
        self.qt_app.processEvents()
        self.assertFalse(dock.isHidden())

        button.click()
        self.qt_app.processEvents()
        self.assertTrue(dock.isHidden())

    def test_save_pair_keeps_reference_files_untouched(self) -> None:
        original = default_scenario_path().read_text(encoding="utf-8")
        with tempfile.TemporaryDirectory() as directory:
            scenario_path = Path(directory) / "working.json"
            vehicle_path = Path(directory) / "working.vehicle.json"
            try:
                self.assertTrue(self.window._write_pair(scenario_path, vehicle_path))
                saved = json.loads(scenario_path.read_text(encoding="utf-8"))
                self.assertEqual(saved["vehicle_definition"], vehicle_path.name)
                self.assertTrue(vehicle_path.is_file())
            finally:
                self.window._load_path(default_scenario_path(), saved_pair=False)
        self.assertEqual(default_scenario_path().read_text(encoding="utf-8"), original)

    def test_in_memory_documents_use_normal_validation(self) -> None:
        scenario, vehicle, _vehicle_path = load_scenario_documents()
        edited = copy.deepcopy(scenario)
        edited["mission"]["payload"]["mass_kg"] += 0.25

        runtime = scenario_from_documents(edited, vehicle)

        self.assertEqual(runtime["mission"]["payload"]["mass_kg"], edited["mission"]["payload"]["mass_kg"])
        self.assertIn("stages", runtime["vehicle"])

    def test_separated_bodies_keep_independent_downrange_trails(self) -> None:
        scenario = default_scenario()
        environment = scenario["environment"]
        launch = geodetic_to_ecef(
            environment["latitude_deg"],
            environment["longitude_deg"],
            environment["launch_altitude_m"],
        )
        north, east, _down = ned_basis(launch)
        azimuth = np.radians(environment["launch_azimuth_deg"])
        axis = np.cos(azimuth) * north + np.sin(azimuth) * east

        def row(body: str, time_s: float, distance_m: float) -> dict[str, float | str]:
            position = launch + distance_m * axis
            return {
                "body": body,
                "time_s": time_s,
                "position_ecef_x_m": position[0],
                "position_ecef_y_m": position[1],
                "position_ecef_z_m": position[2],
                "altitude_m": distance_m / 10.0,
                "speed_m_s": distance_m,
                "thrust_n": 1000.0,
            }

        telemetry = [
            row("integrated_stack", 0.0, 0.0),
            row("integrated_stack", 1.0, 1000.0),
            row("core_stage", 2.0, 1200.0),
            row("upper_stage", 2.0, 1300.0),
            row("upper_stage", 3.0, 2000.0),
        ]

        series = live_telemetry_series(telemetry, scenario, point_limit=2)

        self.assertEqual(set(series), {"integrated_stack", "core_stage", "upper_stage"})
        self.assertAlmostEqual(series["integrated_stack"]["downrange_km"][-1], 1.0)
        self.assertEqual(series["upper_stage"]["time_s"], [2.0, 3.0])

    def test_stale_integrated_marker_hides_after_separation(self) -> None:
        scenario = self.window.working_scenario
        environment = scenario["environment"]
        launch = geodetic_to_ecef(
            environment["latitude_deg"],
            environment["longitude_deg"],
            environment["launch_altitude_m"],
        )

        self.window._clear_plot_items()
        self.window.trajectory_points.clear()
        self.window._stream_sample(
            1.0,
            [self._display_row("integrated_stack", 1.0, launch, 100.0)],
            [],
        )
        self.window._stream_sample(
            2.0,
            [
                self._display_row("core_stage", 2.0, launch, 200.0),
                self._display_row("upper_stage", 2.0, launch, 200.0),
            ],
            [],
        )
        self.window._plot_trajectory()

        self.assertFalse(self.window.plot_items["integrated_stack"]["marker"].isVisible())
        self.assertTrue(self.window.plot_items["core_stage"]["marker"].isVisible())
        self.assertTrue(self.window.plot_items["upper_stage"]["marker"].isVisible())

    def test_stream_bounds_history_but_draws_latest_point(self) -> None:
        window = self.window
        launch = window._trajectory_origin
        window._clear_plot_items()
        window.trajectory_points = {}
        window.telemetry_points = {}
        window.latest_plot_points = {}
        window.display_sample_counts = {}
        original_stride = window.display_sample_stride
        window.display_sample_stride = 2
        window.live_events = []
        window.latest_by_body = {}
        window.live_dirty = False

        window._stream_sample(
            1.0,
            [
                self._display_row(
                    "core_stage", 1.0, launch, 100.0,
                    speed_m_s=10.0, thrust_n=20_000.0,
                ),
                self._display_row("upper_stage", 1.0, launch, 120.0),
            ],
            [],
        )
        window._stream_sample(
            1.1,
            [self._display_row(
                "core_stage", 1.1, launch, 200.0,
                speed_m_s=15.0, thrust_n=10_000.0,
            )],
            [{"time_s": 1.1, "body": "core_stage", "event": "max_q"}],
        )

        self.assertEqual(window.trajectory_points["core_stage"][0], (1.0, 0.0, 0.1))
        self.assertEqual(len(window.trajectory_points["core_stage"]), 1)
        self.assertEqual(window.latest_plot_points["core_stage"][:3], (1.1, 0.0, 0.2))
        self.assertEqual(window.latest_by_body["core_stage"]["altitude_m"], 200.0)
        self.assertEqual(len(window.latest_by_body), 2)
        window._plot_trajectory()
        curve = window.plot_items["core_stage"]["speed"]
        self.assertEqual(curve.xData.tolist(), [1.0, 1.1])
        self.assertEqual(curve.yData.tolist(), [10.0, 15.0])
        window.display_sample_stride = original_stride

    def test_supersonic_crossing_updates_banner_once(self) -> None:
        self.window._supersonic_announced = False

        self.window._stream_sample(
            50.0,
            [
                self._display_row(
                    "integrated_stack",
                    50.0,
                    self.window._trajectory_origin,
                    10_000.0,
                    mach=1.03,
                )
            ],
            [],
        )

        self.assertTrue(self.window._supersonic_announced)
        self.assertEqual(self.window.mission_banner.text(), "SUPERSONIC · MACH 1.03")

    def test_trajectory_autorange_follows_vehicle_not_orbit_overlay(self) -> None:
        self.window.auto_follow_check.setChecked(True)
        environment = self.window.working_scenario["environment"]
        launch = geodetic_to_ecef(
            environment["latitude_deg"],
            environment["longitude_deg"],
            environment["launch_altitude_m"],
        )
        north, east, _down = ned_basis(launch)
        azimuth = np.radians(environment["launch_azimuth_deg"])
        downrange_axis = np.cos(azimuth) * north + np.sin(azimuth) * east

        def row(time_s: float, altitude_m: float, downrange_m: float) -> dict:
            position = launch + downrange_m * downrange_axis
            return self._display_row(
                "integrated_stack",
                time_s,
                position,
                altitude_m,
                speed_m_s=10.0 * time_s,
            )

        self.window._clear_plot_items()
        self.window.live_events = [
            {
                "time_s": 1.0,
                "body": "integrated_stack",
                "event": "stage_separation",
                "detail": "",
            }
        ]
        self.window.show()
        self.window.trajectory_points.clear()
        self.window._stream_sample(
            1.0, [row(0.0, 0.0, 0.0), row(1.0, 120.0, 0.3)], self.window.live_events
        )
        self.window._plot_trajectory()
        self.qt_app.processEvents()

        x_range, y_range = self.window.trajectory_plot.viewRange()
        self.assertGreaterEqual(x_range[1] - x_range[0], 1.0)
        self.assertLess(x_range[1] - x_range[0], 2.0)
        self.assertLess(y_range[1], 1.0)

        self.window.auto_follow_check.setChecked(False)
        with patch.object(self.window.trajectory_plot, "setRange") as set_range:
            self.window._plot_trajectory()
        set_range.assert_not_called()
        self.window.auto_follow_check.setChecked(True)

    def test_solver_process_streams_and_finishes(self) -> None:
        scenario = default_scenario()
        scenario["simulation"]["max_time_s"] = 0.2
        context = mp.get_context("spawn")
        messages = context.Queue()
        cancel_event = context.Event()
        pause_event = context.Event()
        speed = context.Value("d", 0.0)
        process = context.Process(
            target=run_simulation_process,
            args=(scenario, 1, messages, cancel_event, pause_event, speed, False),
        )
        process.start()
        process.join(timeout=10.0)

        self.assertEqual(process.exitcode, 0)
        samples = [messages.get(timeout=1.0) for _ in range(3)]
        self.assertTrue(all(kind == "sample" for kind, _payload in samples))
        _time_s, first_rows, _events = samples[0][1]
        self.assertEqual(set(first_rows[0]), set(DISPLAY_ROW_FIELDS))
        kind, completion = messages.get(timeout=1.0)
        self.assertEqual(kind, "finished")
        self.assertEqual(
            set(completion),
            {"output_dir", "manifest", "events"},
        )
        self.assertFalse(completion["manifest"]["cancelled"])
        self.assertNotIn("telemetry", completion)
        self.assertNotIn("avionics_timeline", completion)
        messages.close()
        messages.join_thread()


if __name__ == "__main__":
    unittest.main()
