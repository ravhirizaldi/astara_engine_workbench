"""ASTARA offline mission engineering workbench."""

from __future__ import annotations

import json
import multiprocessing as mp
import os
import queue
import shutil
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

from matplotlib.figure import Figure

from .flight_core import build_library
from .scenario import default_scenario_path, load_scenario, load_scenario_documents
from .twin import RunResult, run_simulation


def _run_simulation_process(
    scenario: dict,
    seed: int,
    messages,
    cancel_event,
    pause_event,
    speed_factor,
    persist: bool = True,
) -> None:
    last_stream_time = 0.0
    last_stream_wall = time.perf_counter()

    def stream(time_s: float, rows: list[dict], events: list[dict]) -> None:
        nonlocal last_stream_time, last_stream_wall
        while pause_event.is_set() and not cancel_event.is_set():
            time.sleep(0.05)
            last_stream_wall = time.perf_counter()

        factor = float(speed_factor.value)
        if factor > 0.0 and not cancel_event.is_set():
            target = max(time_s - last_stream_time, 0.0) / factor
            delay = target - (time.perf_counter() - last_stream_wall)
            while delay > 0.0 and not cancel_event.is_set():
                chunk = min(delay, 0.05)
                time.sleep(chunk)
                delay -= chunk

        last_stream_time = time_s
        last_stream_wall = time.perf_counter()
        try:
            messages.put_nowait(("sample", (time_s, rows, events)))
        except queue.Full:
            pass

    try:
        result = run_simulation(
            scenario,
            seed=seed,
            create_report=persist,
            persist=persist,
            on_sample=stream,
            should_cancel=cancel_event.is_set,
        )
    except Exception as error:
        messages.put(("failed", f"{type(error).__name__}: {error}"))
    else:
        messages.put(("finished", result))


class AstaraWorkbench:
    def __init__(self, root, legacy_app_class=None) -> None:
        import tkinter as tk
        from tkinter import filedialog, ttk
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

        self.tk = tk
        self.ttk = ttk
        self.filedialog = filedialog
        self.root = root
        self.legacy_app_class = legacy_app_class
        self.result: RunResult | None = None
        self.process_context = mp.get_context("spawn")
        self.cancel_event = self.process_context.Event()
        self.pause_event = self.process_context.Event()
        self.speed_value = self.process_context.Value("d", 0.0, lock=False)
        self.process_messages = self.process_context.Queue(maxsize=32)
        self.simulation_process = None
        self.live_lock = threading.Lock()
        self._closing = False
        self._refresh_after_id = None
        self.live_rows: deque[dict] = deque(maxlen=6000)
        self.live_events: list[dict] = []
        self.latest_by_body: dict[str, dict] = {}
        self.live_dirty = False
        self.is_wsl = bool(os.environ.get("WSL_DISTRO_NAME"))
        self.plot_interval_s = 1.0 if self.is_wsl else 0.5
        self.plot_point_limit = 300 if self.is_wsl else 800
        self._last_plot_wall = 0.0
        self._rendered_event_count = -1
        self._current_thrust_n = 0.0
        self._peak_thrust_n = 0.0
        self._run_max_time_s = 1.0
        root.title("ASTARA Engineering Workbench — SIMULATION ONLY")
        root.geometry("1360x880")
        root.minsize(1000, 680)

        self.path_var = tk.StringVar(value=str(default_scenario_path()))
        self.vehicle_path_var = tk.StringVar(value="No external vehicle definition")
        self.seed_var = tk.IntVar(value=1)
        self.speed_var = tk.StringVar(value="Maximum")
        self.progress_var = tk.DoubleVar(value=0.0)
        self.status_var = tk.StringVar(value="Ready — outputs are unvalidated engineering estimates")
        self.output_var = tk.StringVar(value="No run completed")
        self.metric_vars = {
            name: tk.StringVar(value=value)
            for name, value in (
                ("time", "0.0 s"),
                ("altitude", "0 m"),
                ("speed", "0 m/s"),
                ("mach", "0.00"),
                ("thrust", "0.0 kN"),
                ("dynamic_pressure", "0.0 kPa"),
                ("engines", "0 / 0"),
                ("phase", "READY"),
            )
        }

        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True, padx=10, pady=10)
        self.scenario_tab = ttk.Frame(self.notebook, padding=12)
        self.vehicle_tab = ttk.Frame(self.notebook, padding=12)
        self.run_tab = ttk.Frame(self.notebook, padding=12)
        self.fsw_tab = ttk.Frame(self.notebook, padding=12)
        self.reports_tab = ttk.Frame(self.notebook, padding=12)
        self.notebook.add(self.scenario_tab, text="Scenario")
        self.notebook.add(self.vehicle_tab, text="Vehicle Definition")
        self.notebook.add(self.run_tab, text="Live Mission")
        self.notebook.add(self.fsw_tab, text="Flight Software")
        self.notebook.add(self.reports_tab, text="Artifacts")

        self._build_scenario_tab()
        self._build_vehicle_tab()
        self._build_run_tab()
        self._build_fsw_tab()
        self._build_reports_tab()

        self.figure = Figure(figsize=(9, 6), layout="constrained")
        self.altitude_axis = self.figure.add_subplot(221)
        self.speed_axis = self.figure.add_subplot(222)
        self.thrust_axis = self.figure.add_subplot(223)
        self.trajectory_axis = self.figure.add_subplot(224)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.plot_host)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._load_preview()
        self._refresh_after_id = self.root.after(250, self._refresh_live)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

    def _json_viewer(self, parent):
        frame = self.ttk.Frame(parent)
        text = self.tk.Text(
            frame, wrap="none", height=32, font=("TkFixedFont", 10)
        )
        vertical = self.ttk.Scrollbar(frame, orient="vertical", command=text.yview)
        horizontal = self.ttk.Scrollbar(
            frame, orient="horizontal", command=text.xview
        )
        text.configure(
            yscrollcommand=vertical.set,
            xscrollcommand=horizontal.set,
        )
        text.grid(row=0, column=0, sticky="nsew")
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        return frame, text

    def _copy_json(self, text, label: str) -> None:
        value = text.get("1.0", "end-1c")
        try:
            clip = shutil.which("clip.exe") if self.is_wsl else None
            if clip:
                subprocess.run(
                    [clip],
                    input=value,
                    text=True,
                    check=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                )
            else:
                self.root.clipboard_clear()
                self.root.clipboard_append(value)
        except Exception as error:
            self.status_var.set(f"Could not copy {label}: {error}")
            return
        self.status_var.set(f"{label} copied to clipboard")

    def _build_scenario_tab(self) -> None:
        ttk = self.ttk
        ttk.Label(
            self.scenario_tab,
            text="Anthariksa Mission Scenario",
            font=("TkDefaultFont", 15, "bold"),
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 12))
        ttk.Entry(self.scenario_tab, textvariable=self.path_var).grid(
            row=1, column=0, sticky="ew"
        )
        ttk.Button(self.scenario_tab, text="Browse", command=self._browse).grid(
            row=1, column=1, padx=8
        )
        ttk.Button(self.scenario_tab, text="Validate", command=self._load_preview).grid(
            row=1, column=2
        )
        scenario_viewer, self.scenario_text = self._json_viewer(self.scenario_tab)
        scenario_viewer.grid(
            row=2, column=0, columnspan=3, sticky="nsew", pady=(12, 0)
        )
        ttk.Button(
            self.scenario_tab,
            text="Copy JSON",
            command=lambda: self._copy_json(self.scenario_text, "Scenario JSON"),
        ).grid(row=0, column=2, sticky="e", pady=(0, 12))
        self.scenario_tab.columnconfigure(0, weight=1)
        self.scenario_tab.rowconfigure(2, weight=1)

    def _build_vehicle_tab(self) -> None:
        ttk = self.ttk
        ttk.Label(
            self.vehicle_tab,
            text="Stable Vehicle Definition",
            font=("TkDefaultFont", 15, "bold"),
        ).grid(row=0, column=0, sticky="w", pady=(0, 6))
        ttk.Button(
            self.vehicle_tab,
            text="Copy JSON",
            command=lambda: self._copy_json(self.vehicle_text, "Vehicle JSON"),
        ).grid(row=0, column=1, sticky="e", pady=(0, 6))
        ttk.Label(
            self.vehicle_tab,
            textvariable=self.vehicle_path_var,
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 12))
        vehicle_viewer, self.vehicle_text = self._json_viewer(self.vehicle_tab)
        vehicle_viewer.grid(row=2, column=0, columnspan=2, sticky="nsew")
        self.vehicle_tab.columnconfigure(0, weight=1)
        self.vehicle_tab.rowconfigure(2, weight=1)

    def _build_run_tab(self) -> None:
        ttk = self.ttk
        self.run_tab.columnconfigure(0, weight=1)
        self.run_tab.rowconfigure(5, weight=1)
        ttk.Label(
            self.run_tab,
            text="Live Software-in-the-loop Mission",
            font=("TkDefaultFont", 15, "bold"),
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            self.run_tab,
            text=(
                "Deterministic 6-DOF truth model • 200 Hz C++ flight software • "
                "simulation only"
            ),
            wraplength=900,
        ).grid(row=1, column=0, sticky="w", pady=(3, 12))

        controls = ttk.Frame(self.run_tab)
        controls.grid(row=2, column=0, sticky="ew")
        ttk.Label(controls, text="Seed").pack(side="left")
        ttk.Spinbox(controls, from_=0, to=2**31 - 1, textvariable=self.seed_var, width=12).pack(
            side="left", padx=(6, 12)
        )
        ttk.Label(controls, text="Speed").pack(side="left")
        speed = ttk.Combobox(
            controls,
            textvariable=self.speed_var,
            values=("Maximum", "20×", "5×", "1×"),
            width=10,
            state="readonly",
        )
        speed.pack(side="left", padx=(6, 12))
        speed.bind("<<ComboboxSelected>>", self._on_speed_change)
        self.run_button = ttk.Button(
            controls, text="Start Mission", command=self._start_run
        )
        self.run_button.pack(side="left")
        self.pause_button = ttk.Button(
            controls, text="Pause", command=self._toggle_pause, state="disabled"
        )
        self.pause_button.pack(side="left", padx=(8, 0))
        self.cancel_button = ttk.Button(
            controls, text="Cancel", command=self._cancel_run, state="disabled"
        )
        self.cancel_button.pack(side="left", padx=(8, 0))
        ttk.Button(
            controls, text="Engine Bench", command=self._open_legacy
        ).pack(side="right")

        status = ttk.Frame(self.run_tab)
        status.grid(row=3, column=0, sticky="ew", pady=(12, 8))
        status.columnconfigure(0, weight=1)
        ttk.Label(status, textvariable=self.status_var).grid(row=0, column=0, sticky="w")
        ttk.Progressbar(
            status, variable=self.progress_var, maximum=100.0, length=260
        ).grid(row=0, column=1, sticky="e")

        metrics = ttk.Frame(self.run_tab)
        metrics.grid(row=4, column=0, sticky="ew", pady=(0, 8))
        for index, (name, label) in enumerate(
            (
                ("time", "SIM TIME"),
                ("altitude", "ALTITUDE"),
                ("speed", "SPEED"),
                ("mach", "MACH"),
                ("thrust", "THRUST / PEAK"),
                ("dynamic_pressure", "DYNAMIC PRESSURE"),
                ("engines", "ENGINES ACTIVE"),
                ("phase", "PHASE"),
            )
        ):
            row, column = divmod(index, 4)
            metrics.columnconfigure(column, weight=1)
            card = ttk.LabelFrame(metrics, text=label, padding=(10, 5))
            card.grid(
                row=row,
                column=column,
                sticky="ew",
                padx=(0 if column == 0 else 4, 0),
                pady=(0 if row == 0 else 4, 0),
            )
            ttk.Label(
                card,
                textvariable=self.metric_vars[name],
                font=("TkDefaultFont", 12, "bold"),
            ).pack(anchor="w")

        panes = ttk.Panedwindow(self.run_tab, orient="horizontal")
        panes.grid(row=5, column=0, sticky="nsew")
        detail_frame = ttk.LabelFrame(panes, text="Mission Data", padding=5)
        self.plot_host = ttk.LabelFrame(panes, text="Live Telemetry", padding=5)
        panes.add(detail_frame, weight=1)
        panes.add(self.plot_host, weight=3)
        detail_tabs = ttk.Notebook(detail_frame)
        detail_tabs.pack(fill="both", expand=True)
        event_frame = ttk.Frame(detail_tabs)
        values_frame = ttk.Frame(detail_tabs)
        detail_tabs.add(event_frame, text="Events")
        detail_tabs.add(values_frame, text="All Values")
        self.event_text = self.tk.Text(
            event_frame,
            width=42,
            height=28,
            state="disabled",
            font=("TkFixedFont", 9),
            wrap="none",
        )
        self.event_text.pack(fill="both", expand=True)
        self.telemetry_tree = ttk.Treeview(
            values_frame,
            columns=("value",),
            show="tree headings",
            selectmode="browse",
        )
        self.telemetry_tree.heading("#0", text="Telemetry")
        self.telemetry_tree.heading("value", text="Value")
        self.telemetry_tree.column("#0", width=185, stretch=True)
        self.telemetry_tree.column("value", width=145, anchor="e", stretch=True)
        telemetry_scroll = ttk.Scrollbar(
            values_frame, orient="vertical", command=self.telemetry_tree.yview
        )
        self.telemetry_tree.configure(yscrollcommand=telemetry_scroll.set)
        self.telemetry_tree.pack(side="left", fill="both", expand=True)
        telemetry_scroll.pack(side="right", fill="y")

    def _build_fsw_tab(self) -> None:
        text = (
            "C++17 SIL core\n\n"
            "• 200 Hz sensor/flight-software stepping\n"
            "• Mission states from SAFE through recovery and landing\n"
            "• Strapdown attitude propagation and altitude/velocity estimation\n"
            "• Reference-attitude guidance\n"
            "• Pitch/yaw thrust-vector control plus roll/pitch/yaw aerodynamic controls\n"
            "• GNSS, barometer, IMU fault flags\n"
            "• Plain C ABI loaded by Python ctypes\n\n"
            "SIMULATION ONLY: no serial, network, GPIO, valve, ignition, pyrotechnic, "
            "or flight-termination interfaces are present."
        )
        self.ttk.Label(
            self.fsw_tab,
            text=text,
            justify="left",
            wraplength=900,
            font=("TkDefaultFont", 11),
        ).pack(anchor="nw")
        self.ttk.Button(
            self.fsw_tab, text="Build / Refresh Flight Core", command=self._build_fsw
        ).pack(anchor="w", pady=18)

    def _build_reports_tab(self) -> None:
        self.ttk.Label(
            self.reports_tab,
            text="Run Evidence",
            font=("TkDefaultFont", 15, "bold"),
        ).pack(anchor="w")
        self.ttk.Label(
            self.reports_tab, textvariable=self.output_var, wraplength=1000
        ).pack(anchor="w", pady=12)
        self.ttk.Button(
            self.reports_tab, text="Open Output Folder", command=self._open_output
        ).pack(anchor="w")

    def _browse(self) -> None:
        selected = self.filedialog.askopenfilename(
            title="Select ASTARA scenario", filetypes=[("JSON", "*.json")]
        )
        if selected:
            self.path_var.set(selected)
            self._load_preview()

    def _load_preview(self) -> None:
        try:
            scenario_document, vehicle_document, vehicle_path = (
                load_scenario_documents(self.path_var.get())
            )
            scenario = load_scenario(self.path_var.get())
        except Exception as error:
            self.status_var.set(f"Scenario invalid: {error}")
            return
        self.scenario_text.delete("1.0", "end")
        self.scenario_text.insert("1.0", json.dumps(scenario_document, indent=2))
        self.vehicle_text.delete("1.0", "end")
        if vehicle_document is None:
            vehicle_document = {
                "schema_version": "inline-legacy",
                **{
                    key: scenario[key]
                    for key in ("vehicle", "sensors", "actuators")
                },
            }
            self.vehicle_path_var.set("Inline legacy vehicle definition")
        else:
            self.vehicle_path_var.set(str(vehicle_path))
        self.vehicle_text.insert("1.0", json.dumps(vehicle_document, indent=2))
        self.seed_var.set(int(scenario["simulation"].get("seed", 1)))
        self.status_var.set("Scenario and vehicle definition valid")

    def _start_run(self) -> None:
        try:
            scenario = load_scenario(self.path_var.get())
        except Exception as error:
            self.status_var.set(f"Scenario invalid: {error}")
            return
        self.result = None
        self.cancel_event.clear()
        self.pause_event.clear()
        self._run_max_time_s = float(scenario["simulation"]["max_time_s"])
        self._last_plot_wall = 0.0
        self._rendered_event_count = -1
        self._current_thrust_n = 0.0
        self._peak_thrust_n = 0.0
        with self.live_lock:
            self.live_rows.clear()
            self.live_events.clear()
            self.latest_by_body.clear()
            self.live_dirty = True
        self.telemetry_tree.delete(*self.telemetry_tree.get_children())
        self.progress_var.set(0.0)
        for name, value in (
            ("time", "0.0 s"),
            ("altitude", "0 m"),
            ("speed", "0 m/s"),
            ("mach", "0.00"),
            ("thrust", "0.0 kN"),
            ("dynamic_pressure", "0.0 kPa"),
            ("engines", "0 / 0"),
            ("phase", "STARTING"),
        ):
            self.metric_vars[name].set(value)
        self.run_button.configure(state="disabled")
        self.pause_button.configure(state="normal", text="Pause")
        self.cancel_button.configure(state="normal")
        self.status_var.set("Mission running — solver process active")
        self.notebook.select(self.run_tab)
        self.simulation_process = self.process_context.Process(
            target=_run_simulation_process,
            args=(
                scenario,
                self.seed_var.get(),
                self.process_messages,
                self.cancel_event,
                self.pause_event,
                self.speed_value,
            ),
            daemon=True,
        )
        self.simulation_process.start()

    def _stream_sample(
        self, time_s: float, rows: list[dict], events: list[dict]
    ) -> None:
        with self.live_lock:
            self.live_rows.extend(rows)
            for row in rows:
                self.latest_by_body[row["body"]] = row
            self._current_thrust_n = sum(float(row["thrust_n"]) for row in rows)
            self._peak_thrust_n = max(
                self._peak_thrust_n, self._current_thrust_n
            )
            if len(events) != len(self.live_events):
                self.live_events = list(events)
            self.live_dirty = True

    def _on_speed_change(self, _event=None) -> None:
        self.speed_value.value = {
            "Maximum": 0.0,
            "20×": 20.0,
            "5×": 5.0,
            "1×": 1.0,
        }[self.speed_var.get()]

    def _toggle_pause(self) -> None:
        if self.pause_event.is_set():
            self.pause_event.clear()
            self.pause_button.configure(text="Pause")
            self.status_var.set("Mission running — solver process active")
        else:
            self.pause_event.set()
            self.pause_button.configure(text="Resume")
            self.status_var.set("Mission paused")

    def _cancel_run(self) -> None:
        self.cancel_event.set()
        self.pause_event.clear()
        self.status_var.set("Cancellation requested…")

    def _run_failed(self, error: Exception) -> None:
        self.run_button.configure(state="normal")
        self.pause_button.configure(state="disabled", text="Pause")
        self.cancel_button.configure(state="disabled")
        self.status_var.set(f"Run failed: {error}")

    def _run_finished(self, result: RunResult) -> None:
        self.result = result
        self.run_button.configure(state="normal")
        self.pause_button.configure(state="disabled", text="Pause")
        self.cancel_button.configure(state="disabled")
        self.pause_event.clear()
        maximum_altitude = max(
            (row["altitude_m"] for row in result.telemetry), default=0.0
        )
        if result.manifest["cancelled"]:
            self.status_var.set(
                f"Cancelled at {result.manifest['duration_s']:.1f} simulated seconds"
            )
        else:
            self.progress_var.set(100.0)
            self.status_var.set(
                f"Complete: {maximum_altitude:,.0f} m maximum altitude; "
                f"landed={result.manifest['landed']}"
            )
        self.output_var.set(
            f"{result.output_dir}\n\n"
            "manifest.json • scenario.json • vehicle_definition.json • "
            "truth.csv • fsw.csv • events.csv • "
            + (
                "partial run"
                if result.manifest["cancelled"]
                else "report.pdf • PNG plots • rocketpy_reference.json when enabled"
            )
        )
        self._render_events(result.events)
        self._plot_rows(result.telemetry)
        if result.telemetry:
            self._update_metrics(result.telemetry[-1])
            self._render_live_values()
        with self.live_lock:
            self.live_rows.clear()
            self.live_events.clear()
            self.live_dirty = False
        result.telemetry.clear()
        result.fsw_telemetry.clear()
        result.events.clear()

    def _refresh_live(self) -> None:
        if self._closing:
            return
        for _ in range(64):
            try:
                kind, payload = self.process_messages.get_nowait()
            except queue.Empty:
                break
            if kind == "sample":
                self._stream_sample(*payload)
            elif kind == "finished":
                self._run_finished(payload)
            elif kind == "failed":
                self._run_failed(RuntimeError(payload))

        if self.simulation_process is not None and not self.simulation_process.is_alive():
            self.simulation_process.join()
            self.simulation_process.close()
            self.simulation_process = None

        now = time.perf_counter()
        plot_due = now - self._last_plot_wall >= self.plot_interval_s
        with self.live_lock:
            if self.live_dirty:
                latest = self.live_rows[-1] if self.live_rows else None
                self.live_dirty = False
            else:
                latest = None
            if len(self.live_events) != self._rendered_event_count:
                events = list(self.live_events)
            else:
                events = None
            rows = list(self.live_rows) if plot_due and self.live_rows else []
        if latest:
            self._update_metrics(latest)
            self._render_live_values()
        if events is not None:
            self._render_events(events)
            self._rendered_event_count = len(events)
        if rows:
            self._plot_rows(rows)
            self._last_plot_wall = now
        self._refresh_after_id = self.root.after(250, self._refresh_live)

    def _update_metrics(self, latest: dict) -> None:
        self.metric_vars["time"].set(f"{latest['time_s']:.1f} s")
        self.metric_vars["altitude"].set(f"{latest['altitude_m']:,.0f} m")
        self.metric_vars["speed"].set(f"{latest['speed_m_s']:,.1f} m/s")
        self.metric_vars["mach"].set(f"{latest['mach']:.2f}")
        self.metric_vars["thrust"].set(
            f"{self._current_thrust_n / 1000.0:.1f} / "
            f"{self._peak_thrust_n / 1000.0:.1f} kN"
        )
        self.metric_vars["dynamic_pressure"].set(
            f"{latest['dynamic_pressure_pa'] / 1000.0:.1f} kPa"
        )
        self.metric_vars["engines"].set(
            f"{latest['active_engines']} / {latest['engine_count']}"
        )
        self.metric_vars["phase"].set(latest["mode"])
        self.progress_var.set(
            min(100.0, 100.0 * latest["time_s"] / self._run_max_time_s)
        )

    def _render_live_values(self) -> None:
        for body, row in sorted(self.latest_by_body.items()):
            body_id = f"body:{body}"
            if not self.telemetry_tree.exists(body_id):
                self.telemetry_tree.insert(
                    "", "end", iid=body_id, text=body, values=("",), open=True
                )
            for name, value in row.items():
                item_id = f"{body_id}:{name}"
                if isinstance(value, float):
                    display = f"{value:.6g}"
                else:
                    display = str(value)
                if self.telemetry_tree.exists(item_id):
                    self.telemetry_tree.item(item_id, values=(display,))
                else:
                    self.telemetry_tree.insert(
                        body_id,
                        "end",
                        iid=item_id,
                        text=name,
                        values=(display,),
                    )

    def _render_events(self, events: list[dict]) -> None:
        self.event_text.configure(state="normal")
        self.event_text.delete("1.0", "end")
        for event in events[-200:]:
            self.event_text.insert(
                "end",
                f"{event['time_s']:8.2f}  {event['body']:16}  "
                f"{event['event']:20} {event['detail']}\n",
            )
        self.event_text.see("end")
        self.event_text.configure(state="disabled")

    def _plot_rows(self, telemetry: list[dict]) -> None:
        self.altitude_axis.clear()
        self.speed_axis.clear()
        self.thrust_axis.clear()
        self.trajectory_axis.clear()
        grouped: dict[str, list[dict]] = {}
        for row in telemetry:
            grouped.setdefault(row["body"], []).append(row)
        for body, body_rows in sorted(grouped.items()):
            stride = max(1, len(body_rows) // self.plot_point_limit)
            rows = body_rows[::stride]
            self.altitude_axis.plot(
                [row["time_s"] for row in rows],
                [row["altitude_m"] for row in rows],
                label=body,
            )
            self.speed_axis.plot(
                [row["time_s"] for row in rows],
                [row["speed_m_s"] for row in rows],
                label=body,
            )
            self.thrust_axis.plot(
                [row["time_s"] for row in rows],
                [row["thrust_n"] / 1000.0 for row in rows],
                label=body,
            )
            origin = rows[0]
            self.trajectory_axis.plot(
                [
                    (row["position_ecef_x_m"] - origin["position_ecef_x_m"]) / 1000.0
                    for row in rows
                ],
                [
                    (row["position_ecef_y_m"] - origin["position_ecef_y_m"]) / 1000.0
                    for row in rows
                ],
                label=body,
            )
        self.altitude_axis.set_title("Altitude")
        self.altitude_axis.set_ylabel("Altitude (m)")
        self.altitude_axis.set_xlabel("Time (s)")
        self.speed_axis.set_title("Speed")
        self.speed_axis.set_ylabel("Speed (m/s)")
        self.speed_axis.set_xlabel("Time (s)")
        self.thrust_axis.set_title("Thrust")
        self.thrust_axis.set_ylabel("Thrust (kN)")
        self.thrust_axis.set_xlabel("Time (s)")
        self.trajectory_axis.set_title("Relative ECEF Ground Track")
        self.trajectory_axis.set_xlabel("ΔX km")
        self.trajectory_axis.set_ylabel("ΔY km")
        for axis in (
            self.altitude_axis,
            self.speed_axis,
            self.thrust_axis,
            self.trajectory_axis,
        ):
            axis.grid(True, alpha=0.25)
            axis.legend(fontsize=7)
        self.canvas.draw_idle()

    def _build_fsw(self) -> None:
        try:
            path = build_library()
        except Exception as error:
            self.status_var.set(f"Flight-core build failed: {error}")
            return
        self.status_var.set(f"Flight core ready: {path}")

    def _open_legacy(self) -> None:
        if not self.legacy_app_class:
            return
        window = self.tk.Toplevel(self.root)
        self.legacy_app_class(window)

    def _open_output(self) -> None:
        if not self.result:
            return
        command = ["explorer.exe", str(self.result.output_dir)]
        if os.name != "nt":
            command = ["xdg-open", str(self.result.output_dir)]
        try:
            subprocess.Popen(command)
        except OSError as error:
            self.status_var.set(f"Could not open output folder: {error}")

    def _close(self) -> None:
        if self._closing:
            return
        self._closing = True
        self.cancel_event.set()
        self.pause_event.clear()
        if self._refresh_after_id is not None:
            self.root.after_cancel(self._refresh_after_id)
            self._refresh_after_id = None
        if self.simulation_process is not None:
            self.simulation_process.join(timeout=5.0)
            if self.simulation_process.is_alive():
                self.simulation_process.terminate()
                self.simulation_process.join()
            self.simulation_process.close()
            self.simulation_process = None
        self.process_messages.close()
        self.process_messages.join_thread()
        self.root.destroy()


def show_workbench(legacy_app_class=None) -> bool:
    if os.environ.get("ASTARA_NO_GUI") == "1":
        return False
    try:
        import tkinter as tk
    except ModuleNotFoundError:
        return False
    try:
        root = tk.Tk()
    except tk.TclError:
        return False
    app = AstaraWorkbench(root, legacy_app_class)
    try:
        root.mainloop()
    finally:
        app._close()
    return True
