"""Compatibility entry point for mission simulation."""

from .avionics import AVIONICS_CSV_FIELDS
from .sensors import _sensor_frame
from .dynamics import _derivative
from .orchestration.mission_run import RunResult, run_simulation
from .separation import _split_stack

__all__ = ("RunResult", "run_simulation")
