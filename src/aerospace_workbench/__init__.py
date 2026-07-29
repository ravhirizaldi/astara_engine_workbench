"""Aerospace Workbench mission engineering package."""

import os
import tempfile
from pathlib import Path

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(tempfile.gettempdir()) / "aerospace_workbench_matplotlib"),
)

__version__ = "0.1.0"

from .configuration.scenarios import default_scenario, load_scenario, validate_scenario
from .simulation.runner import RunResult, run_simulation

__all__ = [
    "RunResult",
    "default_scenario",
    "load_scenario",
    "run_simulation",
    "validate_scenario",
]
