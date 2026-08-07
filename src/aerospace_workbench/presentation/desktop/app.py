"""ASTARA Engineering Workbench desktop UI."""

from __future__ import annotations

import copy
import json
import math
import multiprocessing as mp
import os
import queue
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np

from ...configuration.scenarios import (
    default_scenario_path,
    load_scenario_documents,
    scenario_from_documents,
)
from ...flight_software.build import build_library
from ..plotting import trajectory_projection
from .mission_view import (
    LIVE_FAULT_BODIES,
    LIVE_FAULT_TYPES_BY_COMPONENT,
    LIVE_FAULT_VALUE_TYPES,
    normalize_live_fault_command,
    run_simulation_process,
)

try:
    import pyqtgraph as pg
    from PySide6.QtCore import QIODevice, QPointF, QSaveFile, Qt, QTimer, QUrl
    from PySide6.QtGui import (
        QAction,
        QBrush,
        QCloseEvent,
        QColor,
        QDesktopServices,
        QFontDatabase,
        QKeySequence,
        QPainterPath,
        QPen,
    )
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDockWidget,
        QDoubleSpinBox,
        QFileDialog,
        QFormLayout,
        QFrame,
        QGraphicsItem,
        QGraphicsPathItem,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPlainTextEdit,
        QProgressBar,
        QPushButton,
        QSpinBox,
        QSplitter,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )

    QT_AVAILABLE = True
except ModuleNotFoundError:
    pg = None
    QT_AVAILABLE = False
    QGraphicsPathItem = object
    QMainWindow = object


BACKGROUND = "#0b0f13"
PANEL = "#11171d"
PANEL_ALT = "#151c23"
BORDER = "#27323c"
TEXT = "#d8e0e7"
MUTED = "#7f8d99"
ACCENT = "#5fbf78"
WARNING = "#d5a84f"
DANGER = "#d26464"
PROCESS_MESSAGE_QUEUE_SIZE = 32
MAX_MESSAGES_PER_REFRESH = PROCESS_MESSAGE_QUEUE_SIZE
LIVE_METRIC_REFRESH_HZ = 30.0
LIVE_PLOT_REFRESH_HZ = 12.0
LIVE_PLOT_INTERVAL_S = 1.0 / LIVE_PLOT_REFRESH_HZ
UI_REFRESH_INTERVAL_MS = round(1000.0 / LIVE_METRIC_REFRESH_HZ)
MIN_TRAJECTORY_CAPACITY = 2
LIVE_DISPLAY_POINT_LIMIT = 2_000
TRAJECTORY_MIN_DOWNRANGE_SPAN_KM = 1.0
TRAJECTORY_MIN_ALTITUDE_SPAN_KM = 0.1
TRAJECTORY_LEFT_PADDING_FRACTION = 0.08
TRAJECTORY_RIGHT_PADDING_FRACTION = 0.18
TRAJECTORY_BOTTOM_PADDING_FRACTION = 0.02
TRAJECTORY_TOP_PADDING_FRACTION = 0.08
BODY_COLORS = {
    "integrated_stack": "#55b8b6",
    "core_stage": "#5d8fd8",
    "upper_stage": "#e18a4a",
    "payload": "#d8dde3",
}
BODY_LABEL_ANCHORS = {
    "integrated_stack": (1.0, 1.15),
    "core_stage": (1.0, 0.0),
    "upper_stage": (0.0, 1.15),
    "payload": (0.0, 0.0),
}


def _use_opengl_renderer() -> bool:
    return os.environ.get("ASTARA_UI_OPENGL", "0").casefold() in {
        "1",
        "true",
        "yes",
        "on",
    }
MAJOR_EVENT_BANNERS = {
    "hold_down_released": ("LIFTOFF · HOLD-DOWN RELEASED", "running"),
    "rail_exit": ("RAIL CLEAR · FREE FLIGHT", "running"),
    "max_q": ("MAX-Q · PEAK AERODYNAMIC LOAD", "warning"),
    "meco": ("MECO · STAGE ONE CUTOFF", "warning"),
    "stage_separation": ("STAGE SEPARATION · TWO BODIES TRACKED", "running"),
    "stage2_ignition": ("UPPER STAGE IGNITION · BOOST TWO", "running"),
    "stage2_first_cutoff": ("UPPER STAGE CUTOFF · COAST", "warning"),
    "stage2_second_ignition": ("CIRCULARIZATION BURN · UPPER STAGE", "running"),
    "orbit_insertion": ("ORBIT INSERTION · TARGET ACHIEVED", "success"),
    "orbit_insertion_failed": ("ORBIT INSERTION FAILED", "error"),
    "payload_deploy": ("PAYLOAD DEPLOYED · FREE FLIGHT", "success"),
    "drogue_deployed": ("DROGUE DEPLOYED · RECOVERY ACTIVE", "warning"),
    "main_deployed": ("MAIN PARACHUTE DEPLOYED · DESCENT", "success"),
    "landed": ("TOUCHDOWN · BODY LANDED", "success"),
    "payload_propagation_complete": ("PAYLOAD PROPAGATION COMPLETE", "success"),
    "fault_injected": ("FAULT INJECTED · SIMULATION INPUT DEGRADED", "warning"),
    "fault_cleared": ("FAULT CLEARED · NOMINAL INPUT RESTORED", "running"),
    "fault_rejected": ("FAULT REJECTED · INVALID OPERATOR REQUEST", "error"),
    "mission_complete": ("MISSION COMPLETE · REPORTS READY", "success"),
}

STYLE = f"""
QMainWindow, QWidget {{ background: {BACKGROUND}; color: {TEXT}; }}
QFrame#header, QFrame#panel, QGroupBox {{ background: {PANEL}; border: 1px solid {BORDER}; }}
QGroupBox {{ margin-top: 9px; padding: 12px 8px 8px; font-weight: 600; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 8px; padding: 0 4px; color: {MUTED}; }}
QLabel#eyebrow {{ color: {MUTED}; font-size: 10px; font-weight: 600; }}
QLabel#title {{ font-size: 18px; font-weight: 700; }}
QLabel#metric {{ font-size: 16px; font-weight: 600; }}
QLabel#statusBadge {{ background: #163621; color: #86d69a; border: 1px solid #2d6940; padding: 4px 7px; }}
QLabel#missionBanner {{ background: #13231a; color: #86d69a; border: 1px solid #2d6940; padding: 7px 10px; font-weight: 700; }}
QPushButton {{ background: {PANEL_ALT}; border: 1px solid {BORDER}; padding: 6px 10px; font-weight: 600; }}
QPushButton:hover {{ border-color: #4e6273; background: #1a242d; }}
QPushButton:pressed {{ background: #0f151a; }}
QPushButton:focus {{ border: 1px solid {ACCENT}; }}
QPushButton:disabled {{ color: #4f5a63; border-color: #202930; }}
QPushButton#primary {{ background: #286e3d; border-color: #4ea867; color: white; }}
QPushButton#primary:hover {{ background: #32834b; }}
QPushButton#danger {{ color: #e99595; }}
QLineEdit, QPlainTextEdit, QComboBox, QSpinBox, QDoubleSpinBox, QTreeWidget {{
  background: #0d1217; border: 1px solid {BORDER}; padding: 4px; selection-background-color: #27563a;
}}
QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QPlainTextEdit:focus {{ border-color: {ACCENT}; }}
QProgressBar {{ background: #0d1217; border: 1px solid {BORDER}; height: 8px; text-align: center; }}
QProgressBar::chunk {{ background: {ACCENT}; }}
QTabWidget::pane {{ border: 1px solid {BORDER}; }}
QTabBar::tab {{ background: {PANEL}; border: 1px solid {BORDER}; padding: 7px 12px; }}
QTabBar::tab:selected {{ color: #86d69a; border-bottom-color: {ACCENT}; }}
QDockWidget {{ color: {TEXT}; font-weight: 600; }}
QDockWidget::title {{ background: {PANEL}; padding: 7px; border: 1px solid {BORDER}; }}
QSplitter::handle {{ background: {BORDER}; width: 1px; height: 1px; }}
QToolTip {{ background: {PANEL_ALT}; color: {TEXT}; border: 1px solid {BORDER}; }}
"""


class VehicleMarker(QGraphicsPathItem):
    """Constant-size vector marker for a stage or payload."""

    def __init__(self, color: str, payload: bool = False) -> None:
        super().__init__()
        path = QPainterPath()
        if payload:
            path.moveTo(0, -5)
            path.lineTo(5, 0)
            path.lineTo(0, 5)
            path.lineTo(-5, 0)
        else:
            path.moveTo(0, -11)
            path.lineTo(4, -5)
            path.lineTo(4, 7)
            path.lineTo(7, 10)
            path.lineTo(2, 9)
            path.lineTo(0, 12)
            path.lineTo(-2, 9)
            path.lineTo(-7, 10)
            path.lineTo(-4, 7)
            path.lineTo(-4, -5)
        path.closeSubpath()
        self.setPath(path)
        self.setPen(QPen(QColor("#f3f6f8"), 1.0))
        self.setBrush(QBrush(QColor(color)))
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations, True)
        self.setZValue(30)


class EngineeringWorkbench(QMainWindow):
    """Unified mission console backed by the existing solver process."""

    def __init__(self) -> None:
        if not QT_AVAILABLE:
            raise ModuleNotFoundError("PySide6 and pyqtgraph are required for the Workbench")
        super().__init__()
        self.setWindowTitle("ASTARA Engineering Workbench")
        self.resize(1500, 900)
        self.setMinimumSize(1180, 720)
        self.setStyleSheet(STYLE)
        fixed_font = QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont)
        fixed_font.setPointSize(9)
        QApplication.instance().setFont(fixed_font)
        self.opengl_renderer = _use_opengl_renderer()
        pg.setConfigOptions(
            antialias=False,
            background=BACKGROUND,
            foreground=MUTED,
            useOpenGL=self.opengl_renderer,
        )

        self.output_dir: Path | None = None
        self.result_manifest: dict[str, Any] | None = None
        self.process_context = mp.get_context("spawn")
        self.cancel_event = self.process_context.Event()
        self.pause_event = self.process_context.Event()
        self.speed_value = self.process_context.Value("d", 0.0, lock=False)
        self.process_messages = self.process_context.Queue(
            maxsize=PROCESS_MESSAGE_QUEUE_SIZE
        )
        self.control_messages = self.process_context.Queue(
            maxsize=PROCESS_MESSAGE_QUEUE_SIZE
        )
        self.simulation_process = None
        self.trajectory_points: dict[str, deque[tuple[float, float, float]]] = {}
        self.telemetry_points: dict[
            str, deque[tuple[float, float, float, float]]
        ] = {}
        self.latest_plot_points: dict[
            str, tuple[float, float, float, float, float]
        ] = {}
        self.display_sample_counts: dict[str, int] = {}
        self.display_sample_stride = 1
        self.trajectory_capacity = 2
        self._trajectory_origin: Any = None
        self._downrange_axis: Any = None
        self.live_events: list[dict[str, Any]] = []
        self.latest_by_body: dict[str, dict[str, Any]] = {}
        self.active_bodies: set[str] = set()
        self.live_dirty = False
        self.plot_interval_s = LIVE_PLOT_INTERVAL_S
        self._last_plot_wall = 0.0
        self._rendered_event_count = -1
        self._annotated_event_count = -1
        self._banner_event_count = 0
        self._supersonic_announced = False
        self._run_max_time_s = 1.0
        self._closing = False
        self._syncing_controls = False
        self._syncing_editors = False
        self._editor_pending = False
        self._dirty = False
        self._saved_pair = False
        self.source_path: Path | None = None
        self.vehicle_path: Path | None = None
        self.scenario_document: dict[str, Any] = {}
        self.vehicle_document: dict[str, Any] = {}
        self.working_scenario: dict[str, Any] = {}
        self.plot_items: dict[str, dict[str, Any]] = {}
        self._latest_plot_series: dict[str, dict[str, Any]] = {}
        self.event_annotations: list[Any] = []
        self.orbit_region = None
        self.orbit_line = None

        self._build_ui()
        self._load_path(default_scenario_path(), saved_pair=False)
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh_live)
        self.refresh_timer.start(UI_REFRESH_INTERVAL_MS)

    def _build_ui(self) -> None:
        central = QWidget()
        outer = QVBoxLayout(central)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(8)
        outer.addWidget(self._build_header())

        split = QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(self._build_mission_rail())
        split.addWidget(self._build_plot_console())
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setSizes([280, 1180])
        outer.addWidget(split, 1)
        self.setCentralWidget(central)

        self._build_configuration_dock()
        self._build_fault_dock()

        self.save_action = QAction("Save", self)
        self.save_action.setShortcut(QKeySequence.StandardKey.Save)
        self.save_action.triggered.connect(self._save)
        self.addAction(self.save_action)
        open_action = QAction("Open scenario", self)
        open_action.setShortcut(QKeySequence.StandardKey.Open)
        open_action.triggered.connect(self._browse_scenario)
        self.addAction(open_action)

    def _build_header(self) -> QWidget:
        header = QFrame()
        header.setObjectName("header")
        layout = QHBoxLayout(header)
        layout.setContentsMargins(12, 8, 10, 8)
        titles = QVBoxLayout()
        title = QLabel("ASTARA / ENGINEERING WORKBENCH")
        title.setObjectName("title")
        self.scenario_name_label = QLabel("Loading scenario")
        self.scenario_name_label.setObjectName("eyebrow")
        titles.addWidget(title)
        titles.addWidget(self.scenario_name_label)
        layout.addLayout(titles)
        layout.addStretch(1)

        renderer = "OPENGL VIEW" if self.opengl_renderer else "CPU VIEW"
        simulation_badge = QLabel(f"SIMULATION ONLY · UNVALIDATED · {renderer}")
        simulation_badge.setObjectName("statusBadge")
        layout.addWidget(simulation_badge)
        self.configuration_button = QPushButton("Configuration")
        self.configuration_button.setCheckable(True)
        self.fault_button = QPushButton("Fault Injection")
        self.fault_button.setCheckable(True)
        build_button = QPushButton("Build Flight Core")
        build_button.clicked.connect(self._build_fsw)
        self.artifacts_button = QPushButton("Open Report Folder")
        self.artifacts_button.setEnabled(False)
        self.artifacts_button.clicked.connect(self._open_output)
        for button in (
            self.configuration_button,
            self.fault_button,
            build_button,
            self.artifacts_button,
        ):
            layout.addWidget(button)
        return header

    def _build_mission_rail(self) -> QWidget:
        rail = QFrame()
        rail.setObjectName("panel")
        rail.setMinimumWidth(250)
        rail.setMaximumWidth(330)
        layout = QVBoxLayout(rail)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(7)

        mission_label = QLabel("MISSION")
        mission_label.setObjectName("eyebrow")
        layout.addWidget(mission_label)
        self.path_label = QLabel()
        self.path_label.setWordWrap(True)
        self.path_label.setObjectName("eyebrow")
        layout.addWidget(self.path_label)
        open_button = QPushButton("Open scenario…")
        open_button.clicked.connect(self._browse_scenario)
        layout.addWidget(open_button)

        self.controls_group = QGroupBox("Run configuration")
        form = QFormLayout(self.controls_group)
        self.payload_spin = self._double_spin(0.001, 1_000_000_000.0, 3, " kg")
        self.orbit_check = QCheckBox("Enabled")
        self.target_altitude_spin = self._double_spin(0.001, 1_000_000.0, 3, " km")
        self.azimuth_spin = self._double_spin(-360.0, 360.0, 2, "°")
        self.wind_north_spin = self._double_spin(-10_000.0, 10_000.0, 2, " m/s")
        self.wind_east_spin = self._double_spin(-10_000.0, 10_000.0, 2, " m/s")
        self.seed_spin = QSpinBox()
        self.seed_spin.setRange(0, 2**31 - 1)
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(("Maximum", "20×", "5×", "1×"))
        form.addRow("Payload", self.payload_spin)
        form.addRow("Target orbit", self.orbit_check)
        form.addRow("Altitude", self.target_altitude_spin)
        form.addRow("Launch azimuth", self.azimuth_spin)
        form.addRow("Wind north", self.wind_north_spin)
        form.addRow("Wind east", self.wind_east_spin)
        form.addRow("Seed", self.seed_spin)
        form.addRow("Playback", self.speed_combo)
        layout.addWidget(self.controls_group)

        for control in (
            self.payload_spin,
            self.orbit_check,
            self.target_altitude_spin,
            self.azimuth_spin,
            self.wind_north_spin,
            self.wind_east_spin,
            self.seed_spin,
        ):
            signal = control.toggled if isinstance(control, QCheckBox) else control.valueChanged
            signal.connect(self._control_changed)
        self.speed_combo.currentTextChanged.connect(self._on_speed_change)

        run_row = QHBoxLayout()
        self.run_button = QPushButton("LAUNCH")
        self.run_button.setObjectName("primary")
        self.run_button.clicked.connect(self._start_run)
        self.pause_button = QPushButton("Pause")
        self.pause_button.setEnabled(False)
        self.pause_button.clicked.connect(self._toggle_pause)
        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.setObjectName("danger")
        self.cancel_button.setEnabled(False)
        self.cancel_button.clicked.connect(self._cancel_run)
        run_row.addWidget(self.run_button, 2)
        run_row.addWidget(self.pause_button)
        run_row.addWidget(self.cancel_button)
        layout.addLayout(run_row)

        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setTextVisible(False)
        layout.addWidget(self.progress)
        self.status_label = QLabel("Ready")
        self.status_label.setWordWrap(True)
        self.status_label.setObjectName("eyebrow")
        layout.addWidget(self.status_label)

        events_label = QLabel("EVENTS")
        events_label.setObjectName("eyebrow")
        layout.addWidget(events_label)
        self.event_text = QPlainTextEdit()
        self.event_text.setReadOnly(True)
        self.event_text.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.event_text.document().setMaximumBlockCount(200)
        layout.addWidget(self.event_text, 1)
        return rail

    @staticmethod
    def _double_spin(low: float, high: float, decimals: int, suffix: str) -> Any:
        spin = QDoubleSpinBox()
        spin.setRange(low, high)
        spin.setDecimals(decimals)
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        return spin

    def _build_plot_console(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(7)
        self.mission_banner = QLabel("READY · LOAD A VALID SCENARIO")
        self.mission_banner.setObjectName("missionBanner")
        layout.addWidget(self.mission_banner)

        top = QHBoxLayout()
        metrics = QFrame()
        metrics.setObjectName("panel")
        metrics.setFixedWidth(215)
        metric_layout = QVBoxLayout(metrics)
        tracked_label = QLabel("TRACKED BODY")
        tracked_label.setObjectName("eyebrow")
        self.body_combo = QComboBox()
        self.body_combo.currentTextChanged.connect(self._tracked_body_changed)
        metric_layout.addWidget(tracked_label)
        metric_layout.addWidget(self.body_combo)
        self.auto_follow_check = QCheckBox("Auto-follow trajectory")
        self.auto_follow_check.setChecked(True)
        metric_layout.addWidget(self.auto_follow_check)
        self.metric_labels: dict[str, QLabel] = {}
        for key, caption in (
            ("time", "SIM TIME"),
            ("altitude", "ALTITUDE"),
            ("speed", "SPEED / MACH"),
            ("phase", "FLIGHT MODE"),
        ):
            label = QLabel(caption)
            label.setObjectName("eyebrow")
            value = QLabel("—")
            value.setObjectName("metric")
            value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            self.metric_labels[key] = value
            metric_layout.addWidget(label)
            metric_layout.addWidget(value)
        self.orbit_summary_label = QLabel("Orbit target not loaded")
        self.orbit_summary_label.setWordWrap(True)
        self.orbit_summary_label.setObjectName("eyebrow")
        metric_layout.addWidget(self.orbit_summary_label)
        metric_layout.addStretch(1)
        top.addWidget(metrics)

        self.trajectory_plot = self._plot_widget(
            "MISSION TRAJECTORY", "Altitude (km)", "Downrange (km)"
        )
        trajectory_view = self.trajectory_plot.getViewBox()
        trajectory_view.setLimits(minXRange=TRAJECTORY_MIN_DOWNRANGE_SPAN_KM)
        trajectory_view.sigRangeChangedManually.connect(
            self._trajectory_range_changed_manually
        )
        self.auto_follow_check.toggled.connect(self._auto_follow_changed)
        self.trajectory_plot.addLegend(offset=(10, 10), labelTextColor=TEXT)
        self.trajectory_plot.addItem(
            pg.InfiniteLine(pos=0.0, angle=0, pen=pg.mkPen("#56616a", width=1))
        )
        top.addWidget(self.trajectory_plot, 1)
        layout.addLayout(top, 3)

        auxiliary = QHBoxLayout()
        self.altitude_plot = self._plot_widget(
            "ALTITUDE", "km", "Time (s)", time_series=True
        )
        self.speed_plot = self._plot_widget(
            "SPEED", "m/s", "Time (s)", time_series=True
        )
        self.thrust_plot = self._plot_widget(
            "THRUST", "kN", "Time (s)", time_series=True
        )
        self.speed_plot.setXLink(self.altitude_plot)
        self.thrust_plot.setXLink(self.altitude_plot)
        for plot in (self.altitude_plot, self.speed_plot, self.thrust_plot):
            auxiliary.addWidget(plot, 1)
        layout.addLayout(auxiliary, 1)
        return panel

    @staticmethod
    def _plot_widget(
        title: str,
        left: str,
        bottom: str,
        *,
        time_series: bool = False,
    ) -> Any:
        plot = pg.PlotWidget()
        plot.setTitle(title, color=MUTED, size="10pt")
        plot.setLabel("left", left)
        plot.setLabel("bottom", bottom)
        plot.getAxis("left").enableAutoSIPrefix(False)
        plot.getAxis("bottom").enableAutoSIPrefix(False)
        plot.showGrid(x=True, y=True, alpha=0.16)
        plot_item = plot.getPlotItem()
        if time_series:
            plot_item.setClipToView(True)
            plot_item.setDownsampling(auto=True, mode="peak")
        plot_item.getViewBox().setMouseEnabled(x=True, y=True)
        return plot

    def _build_configuration_dock(self) -> None:
        self.configuration_dock = QDockWidget("Configuration working copy", self)
        self.configuration_dock.setAllowedAreas(
            Qt.DockWidgetArea.LeftDockWidgetArea | Qt.DockWidgetArea.RightDockWidgetArea
        )
        content = QWidget()
        layout = QVBoxLayout(content)
        tabs = QTabWidget()
        self.scenario_editor = self._editor_tab(tabs, "Scenario JSON")
        self.vehicle_editor = self._editor_tab(tabs, "Vehicle JSON")
        layout.addWidget(tabs, 1)
        buttons = QHBoxLayout()
        apply_button = QPushButton("Apply + Validate")
        apply_button.clicked.connect(self._apply_editors)
        copy_button = QPushButton("Copy current JSON")
        copy_button.clicked.connect(
            lambda: self._copy_json(
                self.scenario_editor if tabs.currentIndex() == 0 else self.vehicle_editor,
                "Configuration JSON",
            )
        )
        revert_button = QPushButton("Revert")
        revert_button.clicked.connect(self._revert)
        self.save_button = QPushButton("Save")
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self._save)
        save_as_button = QPushButton("Save As…")
        save_as_button.clicked.connect(self._save_as)
        for button in (apply_button, copy_button, revert_button, self.save_button, save_as_button):
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.configuration_dock.setWidget(content)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.configuration_dock)
        self.configuration_dock.hide()
        self.configuration_button.toggled.connect(self.configuration_dock.setVisible)
        self.configuration_dock.visibilityChanged.connect(self.configuration_button.setChecked)

    def _build_fault_dock(self) -> None:
        self.fault_dock = QDockWidget("Live fault injection", self)
        self.fault_dock.setAllowedAreas(
            Qt.DockWidgetArea.LeftDockWidgetArea | Qt.DockWidgetArea.RightDockWidgetArea
        )
        content = QWidget()
        layout = QVBoxLayout(content)
        form = QFormLayout()
        self.fault_body_combo = QComboBox()
        self.fault_body_combo.addItems(LIVE_FAULT_BODIES)
        self.fault_component_combo = QComboBox()
        self.fault_component_combo.addItems(tuple(LIVE_FAULT_TYPES_BY_COMPONENT))
        self.fault_type_combo = QComboBox()
        self.fault_value_edit = QLineEdit("0.0")
        self.fault_value_edit.setPlaceholderText("Finite numeric value")
        self.fault_duration_spin = QDoubleSpinBox()
        self.fault_duration_spin.setRange(0.0, 1.0)
        self.fault_duration_spin.setDecimals(2)
        self.fault_duration_spin.setSuffix(" s")
        self.fault_duration_spin.setSpecialValueText("Until cleared")
        form.addRow("Body", self.fault_body_combo)
        form.addRow("Component", self.fault_component_combo)
        form.addRow("Fault type", self.fault_type_combo)
        form.addRow("Value", self.fault_value_edit)
        form.addRow("Duration", self.fault_duration_spin)
        layout.addLayout(form)

        help_label = QLabel(
            "Sensor faults affect all channels. Duration zero remains active until Clear."
        )
        help_label.setWordWrap(True)
        help_label.setObjectName("eyebrow")
        layout.addWidget(help_label)
        buttons = QHBoxLayout()
        self.inject_fault_button = QPushButton("Inject")
        self.inject_fault_button.setObjectName("danger")
        self.inject_fault_button.setEnabled(False)
        self.clear_fault_button = QPushButton("Clear")
        self.clear_fault_button.setEnabled(False)
        buttons.addWidget(self.inject_fault_button)
        buttons.addWidget(self.clear_fault_button)
        layout.addLayout(buttons)
        self.fault_status_label = QLabel("Launch a mission to enable injection")
        self.fault_status_label.setWordWrap(True)
        self.fault_status_label.setObjectName("eyebrow")
        layout.addWidget(self.fault_status_label)
        layout.addStretch(1)

        self.fault_component_combo.currentTextChanged.connect(
            self._fault_component_changed
        )
        self.fault_type_combo.currentTextChanged.connect(self._fault_type_changed)
        self.inject_fault_button.clicked.connect(self._inject_fault)
        self.clear_fault_button.clicked.connect(self._clear_faults)
        self._fault_component_changed(self.fault_component_combo.currentText())

        self.fault_dock.setWidget(content)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.fault_dock)
        self.fault_dock.hide()
        self.fault_button.toggled.connect(self.fault_dock.setVisible)
        self.fault_dock.visibilityChanged.connect(self.fault_button.setChecked)

    def _fault_component_changed(self, component: str) -> None:
        current = self.fault_type_combo.currentText()
        self.fault_type_combo.clear()
        self.fault_type_combo.addItems(LIVE_FAULT_TYPES_BY_COMPONENT.get(component, ()))
        if self.fault_type_combo.findText(current) >= 0:
            self.fault_type_combo.setCurrentText(current)
        self._fault_type_changed(self.fault_type_combo.currentText())

    def _fault_type_changed(self, fault_type: str) -> None:
        needs_value = fault_type in LIVE_FAULT_VALUE_TYPES
        self.fault_value_edit.setEnabled(needs_value)
        self.fault_value_edit.setToolTip(
            "Additive bias, multiplicative scale, or temperature increase"
            if needs_value
            else "This fault type does not use a numeric value"
        )

    def _queue_fault_control(self, command: dict[str, Any], status: str) -> None:
        if self.simulation_process is None or not self.simulation_process.is_alive():
            self.fault_status_label.setText("No mission is currently running")
            return
        try:
            self.control_messages.put_nowait(command)
        except queue.Full:
            self.fault_status_label.setText("Control queue full · request not sent")
            return
        self.fault_status_label.setText(status)

    def _inject_fault(self) -> None:
        command: dict[str, Any] = {
            "action": "inject",
            "body": self.fault_body_combo.currentText(),
            "component": self.fault_component_combo.currentText(),
            "fault_type": self.fault_type_combo.currentText(),
            "duration_s": self.fault_duration_spin.value(),
        }
        if command["fault_type"] in LIVE_FAULT_VALUE_TYPES:
            try:
                command["value"] = float(self.fault_value_edit.text())
            except ValueError:
                self.fault_status_label.setText("Value must be numeric")
                return
        try:
            command = normalize_live_fault_command(command)
        except (TypeError, ValueError) as error:
            self.fault_status_label.setText(str(error))
            return
        self._queue_fault_control(
            command,
            f"Queued {command['fault_type']} for {command['body']}",
        )

    def _clear_faults(self) -> None:
        self._queue_fault_control(
            {"action": "clear"},
            "Queued clear for all operator-injected faults",
        )

    def _editor_tab(self, tabs: Any, title: str) -> Any:
        editor = QPlainTextEdit()
        editor.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        editor.textChanged.connect(self._editor_changed)
        tabs.addTab(editor, title)
        return editor

    def _load_path(self, path: str | Path, saved_pair: bool) -> bool:
        try:
            scenario_document, vehicle_document, vehicle_path = load_scenario_documents(path)
            runtime = scenario_from_documents(
                scenario_document,
                vehicle_document,
                {"scenario": Path(path).resolve(), "vehicle": vehicle_path},
            )
        except Exception as error:
            self._set_status(f"Scenario invalid: {error}", error=True)
            return False
        self.source_path = Path(path).resolve()
        self.vehicle_path = vehicle_path
        self.scenario_document = scenario_document
        self.vehicle_document = vehicle_document
        self.working_scenario = runtime
        self._saved_pair = saved_pair
        self._dirty = False
        self._editor_pending = False
        self.save_button.setEnabled(saved_pair)
        self.path_label.setText(str(self.source_path))
        self.scenario_name_label.setText(str(scenario_document.get("name", self.source_path.stem)))
        self._set_editor_documents()
        self._sync_controls_from_documents()
        self._configure_trajectory_projection()
        self._configure_orbit_overlay()
        self._set_status("Scenario and vehicle definition valid")
        self.mission_banner.setText("READY · SOFTWARE-IN-THE-LOOP MISSION")
        return True

    def _set_editor_documents(self) -> None:
        self._syncing_editors = True
        try:
            self.scenario_editor.setPlainText(json.dumps(self.scenario_document, indent=2))
            self.vehicle_editor.setPlainText(json.dumps(self.vehicle_document, indent=2))
        finally:
            self._syncing_editors = False
        self._editor_pending = False
        self.controls_group.setEnabled(True)

    def _sync_controls_from_documents(self) -> None:
        self._syncing_controls = True
        try:
            mission = self.scenario_document["mission"]
            environment = self.scenario_document["environment"]
            orbit = mission["orbit"]
            self.payload_spin.setValue(float(mission["payload"]["mass_kg"]))
            self.orbit_check.setChecked(bool(orbit["enabled"]))
            self.target_altitude_spin.setValue(float(orbit["target_altitude_m"]) / 1000.0)
            self.target_altitude_spin.setEnabled(bool(orbit["enabled"]))
            self.azimuth_spin.setValue(float(environment["launch_azimuth_deg"]))
            self.wind_north_spin.setValue(float(environment["wind_ned_m_s"][0]))
            self.wind_east_spin.setValue(float(environment["wind_ned_m_s"][1]))
            self.seed_spin.setValue(int(self.scenario_document["simulation"].get("seed", 1)))
        finally:
            self._syncing_controls = False

    def _editor_changed(self) -> None:
        if self._syncing_editors:
            return
        self._editor_pending = True
        self._dirty = True
        self.controls_group.setEnabled(False)
        self._set_status("Configuration draft changed · apply before using common controls")

    def _apply_editors(self, show_status: bool = True) -> dict[str, Any] | None:
        try:
            scenario_document = json.loads(self.scenario_editor.toPlainText())
            vehicle_document = json.loads(self.vehicle_editor.toPlainText())
            if not isinstance(scenario_document, dict) or not isinstance(vehicle_document, dict):
                raise ValueError("scenario and vehicle documents must be JSON objects")
            sources = None
            if not self._dirty and self.source_path and self.vehicle_path:
                sources = {"scenario": self.source_path, "vehicle": self.vehicle_path}
            runtime = scenario_from_documents(scenario_document, vehicle_document, sources)
        except json.JSONDecodeError as error:
            self._set_status(
                f"JSON invalid at line {error.lineno}, column {error.colno}: {error.msg}",
                error=True,
            )
            return None
        except Exception as error:
            self._set_status(f"Configuration invalid: {error}", error=True)
            return None
        self.scenario_document = scenario_document
        self.vehicle_document = vehicle_document
        self.working_scenario = runtime
        self._editor_pending = False
        self.controls_group.setEnabled(True)
        self._sync_controls_from_documents()
        self._configure_trajectory_projection()
        self._configure_orbit_overlay()
        self.scenario_name_label.setText(str(scenario_document.get("name", "Unnamed scenario")))
        if show_status:
            self._set_status("Working copy valid")
        return runtime

    def _control_changed(self, _value: Any = None) -> None:
        if self._syncing_controls or self._editor_pending:
            return
        document = copy.deepcopy(self.scenario_document)
        mission = document["mission"]
        environment = document["environment"]
        mission["payload"]["mass_kg"] = self.payload_spin.value()
        mission["orbit"]["enabled"] = self.orbit_check.isChecked()
        mission["orbit"]["target_altitude_m"] = self.target_altitude_spin.value() * 1000.0
        environment["launch_azimuth_deg"] = self.azimuth_spin.value()
        environment["wind_ned_m_s"][0] = self.wind_north_spin.value()
        environment["wind_ned_m_s"][1] = self.wind_east_spin.value()
        document["simulation"]["seed"] = self.seed_spin.value()
        try:
            runtime = scenario_from_documents(document, self.vehicle_document)
        except Exception as error:
            self._set_status(f"Run control invalid: {error}", error=True)
            self._sync_controls_from_documents()
            return
        self.scenario_document = document
        self.working_scenario = runtime
        self.target_altitude_spin.setEnabled(self.orbit_check.isChecked())
        self._dirty = True
        self._set_editor_documents()
        self._configure_trajectory_projection()
        self._configure_orbit_overlay()
        self._set_status("Working copy changed")

    def _configure_trajectory_projection(self) -> None:
        if not self.working_scenario:
            return
        self._trajectory_origin, self._downrange_axis = trajectory_projection(
            self.working_scenario
        )
        maximum_time_s = float(self.working_scenario["simulation"]["max_time_s"])
        output_rate_hz = float(
            self.working_scenario["simulation"]["output_rate_hz"]
        )
        expected_samples = math.ceil(maximum_time_s * output_rate_hz) + 1
        self.display_sample_stride = max(
            1,
            math.ceil(expected_samples / LIVE_DISPLAY_POINT_LIMIT),
        )
        self.trajectory_capacity = max(
            MIN_TRAJECTORY_CAPACITY,
            math.ceil(expected_samples / self.display_sample_stride) + 1,
        )
        self.fault_duration_spin.setMaximum(maximum_time_s)

    def _browse_scenario(self) -> None:
        if not self._confirm_dirty():
            return
        selected, _filter = QFileDialog.getOpenFileName(
            self, "Open scenario", str(self.source_path or default_scenario_path()), "JSON (*.json)"
        )
        if selected:
            self._load_path(selected, saved_pair=False)

    def _copy_json(self, editor: Any, label: str) -> None:
        QApplication.clipboard().setText(editor.toPlainText())
        self._set_status(f"{label} copied to clipboard")

    def _save_as(self) -> bool:
        if self._apply_editors(False) is None:
            return False
        selected, _filter = QFileDialog.getSaveFileName(
            self, "Save scenario working copy", str(self.source_path or "mission.json"), "JSON (*.json)"
        )
        if not selected:
            return False
        scenario_path = Path(selected)
        if scenario_path.suffix.lower() != ".json":
            scenario_path = scenario_path.with_suffix(".json")
        vehicle_path = scenario_path.with_name(f"{scenario_path.stem}.vehicle.json")
        if (scenario_path.exists() or vehicle_path.exists()) and QMessageBox.question(
            self,
            "Replace working copy?",
            f"Replace these files?\n{scenario_path}\n{vehicle_path}",
        ) != QMessageBox.StandardButton.Yes:
            return False
        return self._write_pair(scenario_path, vehicle_path)

    def _save(self) -> bool:
        if not self._saved_pair or not self.source_path or not self.vehicle_path:
            return self._save_as()
        if self._apply_editors(False) is None:
            return False
        return self._write_pair(self.source_path, self.vehicle_path)

    def _write_pair(self, scenario_path: Path, vehicle_path: Path) -> bool:
        scenario_document = copy.deepcopy(self.scenario_document)
        scenario_document["vehicle_definition"] = vehicle_path.name
        try:
            scenario_from_documents(scenario_document, self.vehicle_document)
            self._write_json(vehicle_path, self.vehicle_document)
            self._write_json(scenario_path, scenario_document)
        except Exception as error:
            self._set_status(f"Could not save configuration: {error}", error=True)
            return False
        self.scenario_document = scenario_document
        if not self._load_path(scenario_path, saved_pair=True):
            return False
        self._set_status(f"Working copy saved: {scenario_path}")
        return True

    @staticmethod
    def _write_json(path: Path, document: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        output = QSaveFile(str(path))
        if not output.open(QIODevice.OpenModeFlag.WriteOnly | QIODevice.OpenModeFlag.Text):
            raise OSError(output.errorString())
        payload = (json.dumps(document, indent=2) + "\n").encode("utf-8")
        if output.write(payload) != len(payload) or not output.commit():
            raise OSError(output.errorString())

    def _revert(self) -> None:
        if not self.source_path:
            return
        if self._dirty and QMessageBox.question(
            self, "Discard edits?", "Reload the last opened or saved configuration?"
        ) != QMessageBox.StandardButton.Yes:
            return
        self._load_path(self.source_path, saved_pair=self._saved_pair)

    def _confirm_dirty(self) -> bool:
        if not self._dirty:
            return True
        box = QMessageBox(self)
        box.setWindowTitle("Unsaved configuration")
        box.setText("Save the scenario and vehicle working copy before continuing?")
        box.setStandardButtons(
            QMessageBox.StandardButton.Save
            | QMessageBox.StandardButton.Discard
            | QMessageBox.StandardButton.Cancel
        )
        choice = box.exec()
        if choice == QMessageBox.StandardButton.Save:
            return self._save()
        return choice == QMessageBox.StandardButton.Discard

    def _start_run(self) -> None:
        scenario = self._apply_editors(False)
        if scenario is None:
            return
        self.output_dir = None
        self.result_manifest = None
        self.cancel_event.clear()
        self.pause_event.clear()
        while True:
            try:
                self.control_messages.get_nowait()
            except queue.Empty:
                break
        self._run_max_time_s = float(scenario["simulation"]["max_time_s"])
        self._last_plot_wall = 0.0
        self._rendered_event_count = -1
        self._annotated_event_count = -1
        self._banner_event_count = 0
        self._supersonic_announced = False
        self.trajectory_points.clear()
        self.telemetry_points.clear()
        self.latest_plot_points.clear()
        self.display_sample_counts.clear()
        self.live_events.clear()
        self.latest_by_body.clear()
        self.active_bodies.clear()
        self.live_dirty = True
        self.body_combo.clear()
        self.progress.setValue(0)
        self._clear_plot_items()
        self.auto_follow_check.setChecked(True)
        self.run_button.setEnabled(False)
        self.pause_button.setEnabled(True)
        self.pause_button.setText("Pause")
        self.cancel_button.setEnabled(True)
        self.inject_fault_button.setEnabled(True)
        self.clear_fault_button.setEnabled(True)
        self.fault_status_label.setText("Mission active · choose a fault to inject")
        self.artifacts_button.setEnabled(False)
        self.mission_banner.setText("MISSION RUNNING · SOLVER PROCESS ACTIVE")
        self._set_banner_state("running")
        self._set_status("Mission running · solver process active")
        self.simulation_process = self.process_context.Process(
            target=run_simulation_process,
            args=(
                scenario,
                self.seed_spin.value(),
                self.process_messages,
                self.cancel_event,
                self.pause_event,
                self.speed_value,
                True,
                self.control_messages,
            ),
            daemon=True,
        )
        self.simulation_process.start()

    def _on_speed_change(self, label: str) -> None:
        factors = {"Maximum": 0.0, "20×": 20.0, "5×": 5.0, "1×": 1.0}
        self.speed_value.value = factors.get(label, 0.0)

    def _toggle_pause(self) -> None:
        if self.pause_event.is_set():
            self.pause_event.clear()
            self.pause_button.setText("Pause")
            self.mission_banner.setText("MISSION RUNNING · SOLVER PROCESS ACTIVE")
        else:
            self.pause_event.set()
            self.pause_button.setText("Resume")
            self.mission_banner.setText("MISSION PAUSED · VIEW HELD")

    def _cancel_run(self) -> None:
        self.cancel_event.set()
        self.cancel_button.setEnabled(False)
        self._set_status("Cancellation requested")

    def _stream_sample(
        self, _time_s: float, rows: list[dict[str, Any]], events: list[dict[str, Any]]
    ) -> None:
        self.active_bodies = {str(row["body"]) for row in rows}
        for row in rows:
            body = str(row["body"])
            self.latest_by_body[body] = row
            position = (
                float(row["position_ecef_x_m"]),
                float(row["position_ecef_y_m"]),
                float(row["position_ecef_z_m"]),
            )
            relative = tuple(
                position[index] - float(self._trajectory_origin[index])
                for index in range(3)
            )
            downrange_km = sum(
                relative[index] * float(self._downrange_axis[index])
                for index in range(3)
            ) / 1000.0
            plot_point = (
                float(row["time_s"]),
                downrange_km,
                float(row["altitude_m"]) / 1000.0,
                float(row["speed_m_s"]),
                float(row["thrust_n"]) / 1000.0,
            )
            self.latest_plot_points[body] = plot_point
            sample_count = self.display_sample_counts.get(body, 0)
            self.display_sample_counts[body] = sample_count + 1
            if sample_count % self.display_sample_stride:
                continue
            points = self.trajectory_points.setdefault(
                body, deque(maxlen=self.trajectory_capacity)
            )
            points.append(plot_point[:3])
            history = self.telemetry_points.setdefault(
                body, deque(maxlen=self.trajectory_capacity)
            )
            history.append(
                (
                    plot_point[0],
                    plot_point[2],
                    plot_point[3],
                    plot_point[4],
                )
            )
        if not self._supersonic_announced:
            mach = max((float(row.get("mach", 0.0)) for row in rows), default=0.0)
            if mach >= 1.0:
                self._supersonic_announced = True
                self.mission_banner.setText(f"SUPERSONIC · MACH {mach:.2f}")
                self._set_banner_state("running")
        if len(events) != len(self.live_events):
            self.live_events = list(events)
        self.live_dirty = True

    def _refresh_live(self) -> None:
        if self._closing:
            return
        for _ in range(MAX_MESSAGES_PER_REFRESH):
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
        events = (
            list(self.live_events)
            if len(self.live_events) != self._rendered_event_count
            else None
        )
        dirty = self.live_dirty
        if dirty:
            self._update_body_selector()
            tracked = self.latest_by_body.get(self.body_combo.currentText())
            if tracked:
                self._update_metrics(tracked)
        if events is not None:
            self._render_events(events)
            self._rendered_event_count = len(events)
        if self.live_dirty and now - self._last_plot_wall >= self.plot_interval_s:
            self._plot_trajectory()
            self._last_plot_wall = now
            self.live_dirty = False

    def _update_body_selector(self) -> None:
        bodies = list(self.latest_by_body)
        if not bodies:
            return
        current = self.body_combo.currentText()
        for body in bodies:
            if self.body_combo.findText(body) < 0:
                self.body_combo.addItem(body)
        if not current:
            preferred = "integrated_stack" if "integrated_stack" in bodies else bodies[0]
            self.body_combo.setCurrentText(preferred)
        elif current == "integrated_stack" and "upper_stage" in bodies:
            self.body_combo.setCurrentText("upper_stage")

    def _tracked_body_changed(self, body: str) -> None:
        row = self.latest_by_body.get(body)
        if row:
            self._update_metrics(row)

    def _trajectory_range_changed_manually(self, _axes: Any = None) -> None:
        if not self.auto_follow_check.isChecked():
            return
        self.auto_follow_check.blockSignals(True)
        self.auto_follow_check.setChecked(False)
        self.auto_follow_check.blockSignals(False)
        self._set_status("Manual trajectory view · enable Auto-follow to fit live data")

    def _auto_follow_changed(self, enabled: bool) -> None:
        if enabled and self._latest_plot_series:
            self._frame_trajectory(self._latest_plot_series)
            self._set_status("Trajectory auto-follow enabled")
        elif not enabled:
            self._set_status("Manual trajectory view")

    def _update_metrics(self, latest: dict[str, Any]) -> None:
        self.metric_labels["time"].setText(f"{latest['time_s']:.1f} s")
        self.metric_labels["altitude"].setText(f"{latest['altitude_m'] / 1000.0:,.2f} km")
        self.metric_labels["speed"].setText(
            f"{latest['speed_m_s']:,.1f} m/s · M {latest['mach']:.2f}"
        )
        self.metric_labels["phase"].setText(str(latest["mode"]))
        self.progress.setValue(
            round(min(1000.0, 1000.0 * float(latest["time_s"]) / self._run_max_time_s))
        )

    def _render_events(self, events: list[dict[str, Any]]) -> None:
        lines = [
            f"{float(event['time_s']):8.2f}  {str(event['body']):16}  "
            f"{str(event['event']):24} {event.get('detail', '')}"
            for event in events[-200:]
        ]
        self.event_text.setPlainText("\n".join(lines))
        self.event_text.verticalScrollBar().setValue(self.event_text.verticalScrollBar().maximum())
        if self._banner_event_count > len(events):
            self._banner_event_count = 0
        for event in events[self._banner_event_count :]:
            banner = MAJOR_EVENT_BANNERS.get(str(event.get("event", "")))
            if banner is None:
                continue
            text, state = banner
            detail = event.get("detail")
            if event.get("event") == "max_q" and isinstance(detail, dict):
                pressure_kpa = float(detail.get("dynamic_pressure_pa", 0.0)) / 1000.0
                text = f"MAX-Q · {pressure_kpa:.1f} kPa"
            elif event.get("event") == "landed":
                body = str(event.get("body", "body")).replace("_", " ").upper()
                text = f"TOUCHDOWN · {body}"
            self.mission_banner.setText(text)
            self._set_banner_state(state)
        self._banner_event_count = len(events)
        self._update_orbit_status(events)

    def _trajectory_series(self) -> dict[str, dict[str, np.ndarray]]:
        series: dict[str, dict[str, np.ndarray]] = {}
        for body, body_points in self.trajectory_points.items():
            points = np.asarray(body_points, dtype=float)
            if points.size == 0:
                continue
            latest = self.latest_plot_points.get(body)
            if latest is not None and latest[0] != points[-1, 0]:
                points = np.vstack((points, latest[:3]))
            series[body] = {
                "time_s": points[:, 0],
                "downrange_km": points[:, 1],
                "altitude_km": points[:, 2],
            }
        return series

    def _plot_trajectory(self) -> None:
        series = self._trajectory_series()
        if not series:
            return
        self._latest_plot_series = series
        for body, body_series in series.items():
            items = self._ensure_plot_items(body)
            downrange = body_series["downrange_km"]
            altitude = body_series["altitude_km"]
            items["trajectory"].setData(downrange, altitude)
            telemetry = np.asarray(self.telemetry_points.get(body, ()), dtype=float)
            latest = self.latest_plot_points.get(body)
            if latest is not None:
                latest_telemetry = np.asarray(
                    (latest[0], latest[2], latest[3], latest[4]),
                    dtype=float,
                )
                if telemetry.size == 0:
                    telemetry = latest_telemetry.reshape(1, 4)
                elif latest[0] != telemetry[-1, 0]:
                    telemetry = np.vstack((telemetry, latest_telemetry))
            if telemetry.size:
                items["altitude"].setData(telemetry[:, 0], telemetry[:, 1])
                items["speed"].setData(telemetry[:, 0], telemetry[:, 2])
                items["thrust"].setData(telemetry[:, 0], telemetry[:, 3])
            x, y = downrange[-1], altitude[-1]
            items["marker"].setPos(x, y)
            items["label"].setPos(x, y)
            active = body in self.active_bodies
            items["marker"].setVisible(active)
            items["label"].setVisible(active)
            latest = self.latest_by_body.get(body, {})
            items["marker"].setRotation(
                0.0 if int(latest.get("landed", 0)) else self._marker_angle(downrange, altitude)
            )
        present = set(series)
        for body, items in self.plot_items.items():
            if body not in present:
                items["marker"].setVisible(False)
                items["label"].setVisible(False)
        if self.auto_follow_check.isChecked():
            self._frame_trajectory(series)
        if len(self.live_events) != self._annotated_event_count:
            self._render_event_annotations(series)
            self._annotated_event_count = len(self.live_events)

    def _frame_trajectory(self, series: dict[str, dict[str, Any]]) -> None:
        all_downrange = [
            value
            for body_series in series.values()
            for value in body_series["downrange_km"]
        ]
        all_altitude = [
            value
            for body_series in series.values()
            for value in body_series["altitude_km"]
        ]
        x_min, x_max = min(all_downrange), max(all_downrange)
        y_min, y_max = min(0.0, min(all_altitude)), max(all_altitude)
        x_span = max(x_max - x_min, TRAJECTORY_MIN_DOWNRANGE_SPAN_KM)
        y_span = max(y_max - y_min, TRAJECTORY_MIN_ALTITUDE_SPAN_KM)
        self.trajectory_plot.setRange(
            xRange=(
                x_min - TRAJECTORY_LEFT_PADDING_FRACTION * x_span,
                x_max + TRAJECTORY_RIGHT_PADDING_FRACTION * x_span,
            ),
            yRange=(
                y_min - TRAJECTORY_BOTTOM_PADDING_FRACTION * y_span,
                y_max + TRAJECTORY_TOP_PADDING_FRACTION * y_span,
            ),
            padding=0.0,
            disableAutoRange=True,
        )

    def _ensure_plot_items(self, body: str) -> dict[str, Any]:
        if body in self.plot_items:
            return self.plot_items[body]
        color = BODY_COLORS.get(body, "#b4bec7")
        pen = pg.mkPen(color, width=1)
        items = {
            "trajectory": self.trajectory_plot.plot(
                pen=pen,
                name=body.replace("_", " "),
                antialias=False,
            ),
            "altitude": self.altitude_plot.plot(pen=pen, antialias=False),
            "speed": self.speed_plot.plot(pen=pen, antialias=False),
            "thrust": self.thrust_plot.plot(pen=pen, antialias=False),
            "marker": VehicleMarker(color, payload=body == "payload"),
            "label": pg.TextItem(
                body.replace("_", " ").upper(),
                color=color,
                anchor=BODY_LABEL_ANCHORS.get(body, (0.0, 1.0)),
            ),
        }
        for key in ("trajectory", "altitude", "speed", "thrust"):
            items[key].setSkipFiniteCheck(True)
        self.trajectory_plot.addItem(items["marker"], ignoreBounds=True)
        self.trajectory_plot.addItem(items["label"], ignoreBounds=True)
        self.plot_items[body] = items
        return items

    def _marker_angle(self, x_values: list[float], y_values: list[float]) -> float:
        if len(x_values) < 2:
            return 0.0
        start = self.trajectory_plot.getViewBox().mapViewToScene(
            QPointF(float(x_values[-2]), float(y_values[-2]))
        )
        end = self.trajectory_plot.getViewBox().mapViewToScene(
            QPointF(float(x_values[-1]), float(y_values[-1]))
        )
        if start == end:
            return 0.0
        return math.degrees(math.atan2(end.y() - start.y(), end.x() - start.x())) + 90.0

    def _clear_plot_items(self) -> None:
        for items in self.plot_items.values():
            for key, plot in (
                ("trajectory", self.trajectory_plot),
                ("altitude", self.altitude_plot),
                ("speed", self.speed_plot),
                ("thrust", self.thrust_plot),
                ("marker", self.trajectory_plot),
                ("label", self.trajectory_plot),
            ):
                plot.removeItem(items[key])
        self.plot_items.clear()
        self._latest_plot_series = {}
        for annotation in self.event_annotations:
            self.trajectory_plot.removeItem(annotation)
        self.event_annotations.clear()
        self._annotated_event_count = -1

    def _render_event_annotations(self, series: dict[str, dict[str, Any]]) -> None:
        for annotation in self.event_annotations:
            self.trajectory_plot.removeItem(annotation)
        self.event_annotations.clear()
        labels = {
            "stage_separation": "STAGE SEPARATION",
            "payload_deploy": "PAYLOAD DEPLOY",
            "orbit_insertion": "ORBIT INSERTION",
            "landed": "LANDED",
        }
        seen: set[tuple[str, str]] = set()
        for event in self.live_events:
            event_name = str(event.get("event", ""))
            body = str(event.get("body", ""))
            key = (event_name, body)
            body_series = series.get(body)
            if event_name not in labels or key in seen or not body_series:
                continue
            times = body_series["time_s"]
            index = min(
                range(len(times)),
                key=lambda position: abs(times[position] - float(event["time_s"])),
            )
            text = pg.TextItem(labels[event_name], color=WARNING, anchor=(1.05, 1.2))
            text.setPos(body_series["downrange_km"][index], body_series["altitude_km"][index])
            self.trajectory_plot.addItem(text, ignoreBounds=True)
            self.event_annotations.append(text)
            seen.add(key)

    def _configure_orbit_overlay(self) -> None:
        if self.orbit_region is not None:
            self.trajectory_plot.removeItem(self.orbit_region)
            self.orbit_region = None
        if self.orbit_line is not None:
            self.trajectory_plot.removeItem(self.orbit_line)
            self.orbit_line = None
        if not self.scenario_document:
            return
        orbit = self.scenario_document.get("mission", {}).get("orbit", {})
        if not orbit.get("enabled", False):
            self.orbit_summary_label.setText("Orbit targeting disabled")
            return
        target = float(orbit["target_altitude_m"]) / 1000.0
        tolerance = float(orbit["altitude_tolerance_m"]) / 1000.0
        self.orbit_region = pg.LinearRegionItem(
            values=(target - tolerance, target + tolerance),
            orientation="horizontal",
            movable=False,
            brush=pg.mkBrush(95, 191, 120, 24),
            pen=pg.mkPen(None),
        )
        self.orbit_region.setZValue(-20)
        self.orbit_line = pg.InfiniteLine(
            pos=target,
            angle=0,
            movable=False,
            pen=pg.mkPen(ACCENT, width=1, style=Qt.PenStyle.DashLine),
            label=f"TARGET ORBIT {target:.0f} km",
            labelOpts={"color": ACCENT, "position": 0.75},
        )
        self.trajectory_plot.addItem(self.orbit_region, ignoreBounds=True)
        self.trajectory_plot.addItem(self.orbit_line, ignoreBounds=True)
        self.orbit_summary_label.setText(
            f"Target {target:,.1f} ± {tolerance:,.1f} km · "
            f"inclination {float(orbit['target_inclination_deg']):.2f}°"
        )

    def _update_orbit_status(self, events: list[dict[str, Any]]) -> None:
        failed = next(
            (event for event in reversed(events) if event.get("event") == "orbit_insertion_failed"),
            None,
        )
        inserted = next(
            (event for event in reversed(events) if event.get("event") == "orbit_insertion"),
            None,
        )
        if failed and (not inserted or float(failed["time_s"]) > float(inserted["time_s"])):
            self.orbit_summary_label.setText("Orbit insertion failed · inspect mission events")
            self.mission_banner.setText("ORBIT INSERTION FAILED")
            self._set_banner_state("error")
            return
        if inserted and isinstance(inserted.get("detail"), dict):
            detail = inserted["detail"]
            self.orbit_summary_label.setText(
                f"Apo {float(detail['apoapsis_altitude_m']) / 1000.0:,.1f} km · "
                f"Peri {float(detail['periapsis_altitude_m']) / 1000.0:,.1f} km · "
                f"Inc {float(detail['inclination_deg']):.2f}°"
            )
            self.mission_banner.setText("ORBIT ACHIEVED · PAYLOAD SEQUENCE ACTIVE")
            self._set_banner_state("success")

    def _run_failed(self, error: Exception) -> None:
        self.run_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        self.cancel_button.setEnabled(False)
        self.inject_fault_button.setEnabled(False)
        self.clear_fault_button.setEnabled(False)
        self.fault_status_label.setText("Mission stopped · injection disabled")
        self.mission_banner.setText("MISSION FAILED · SOLVER STOPPED")
        self._set_banner_state("error")
        self._set_status(f"Mission failed: {error}", error=True)

    def _run_finished(self, completion: dict[str, Any]) -> None:
        manifest = completion["manifest"]
        events = completion["events"]
        self.output_dir = Path(completion["output_dir"])
        self.result_manifest = manifest
        self.run_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        self.pause_button.setText("Pause")
        self.cancel_button.setEnabled(False)
        self.inject_fault_button.setEnabled(False)
        self.clear_fault_button.setEnabled(False)
        self.fault_status_label.setText("Mission complete · injection disabled")
        self.pause_event.clear()
        self.artifacts_button.setEnabled(True)
        self.live_events = list(events)
        if self.live_dirty:
            self._plot_trajectory()
            self.live_dirty = False
        self._render_events(events)
        if manifest["cancelled"]:
            self.mission_banner.setText("MISSION CANCELLED · PARTIAL ARTIFACTS SAVED")
            self._set_banner_state("warning")
            self._set_status(
                f"Cancelled at {manifest['duration_s']:.1f} simulated seconds"
            )
        else:
            maximum_altitude = float(manifest["maximum_altitude_m"])
            self.progress.setValue(1000)
            if not any(event.get("event") == "orbit_insertion" for event in events):
                self.mission_banner.setText("MISSION COMPLETE · REPORTS READY")
                self._set_banner_state("success")
            self._set_status(f"Complete · maximum altitude {maximum_altitude / 1000.0:,.2f} km")

    def _build_fsw(self) -> None:
        try:
            path = build_library()
        except Exception as error:
            self._set_status(f"Flight-core build failed: {error}", error=True)
            return
        self._set_status(f"Flight core ready: {path}")

    def _open_output(self) -> None:
        if self.output_dir:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.output_dir)))

    def _set_status(self, text: str, error: bool = False) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {DANGER if error else MUTED};")

    def _set_banner_state(self, state: str) -> None:
        colors = {
            "success": ("#13231a", "#86d69a", "#2d6940"),
            "running": ("#13212b", "#83bde3", "#315e7a"),
            "warning": ("#2b2415", "#e4bd6d", "#6f592a"),
            "error": ("#2b1717", "#e89595", "#743838"),
        }
        background, color, border = colors[state]
        self.mission_banner.setStyleSheet(
            f"background:{background};color:{color};border:1px solid {border};"
            "padding:7px 10px;font-weight:700;"
        )

    def _shutdown(self) -> None:
        if self._closing:
            return
        self._closing = True
        if hasattr(self, "refresh_timer"):
            self.refresh_timer.stop()
        self.cancel_event.set()
        self.pause_event.clear()
        if self.simulation_process is not None:
            self.simulation_process.join(timeout=5.0)
            if self.simulation_process.is_alive():
                self.simulation_process.terminate()
                self.simulation_process.join()
            self.simulation_process.close()
            self.simulation_process = None
        self.process_messages.close()
        self.process_messages.join_thread()
        self.control_messages.close()
        self.control_messages.join_thread()

    def closeEvent(self, event: QCloseEvent) -> None:
        if not self._confirm_dirty():
            event.ignore()
            return
        self._shutdown()
        event.accept()


def show_workbench() -> bool:
    if os.environ.get("ASTARA_NO_GUI") == "1" or not QT_AVAILABLE:
        return False
    application = QApplication.instance()
    owns_application = application is None
    if application is None:
        application = QApplication(sys.argv)
    window = EngineeringWorkbench()
    window.show()
    application._astara_workbench = window
    if owns_application:
        application.exec()
    return True
