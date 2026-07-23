"""ASTARA aerospace mission engineering workbench."""

import os
import tempfile
from pathlib import Path

os.environ.setdefault(
    "MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "astara_matplotlib")
)

__version__ = "0.1.0"

from .scenario import default_scenario, load_scenario, validate_scenario
from .twin import RunResult, run_simulation

__all__ = [
    "RunResult",
    "default_scenario",
    "load_scenario",
    "run_simulation",
    "validate_scenario",
]
