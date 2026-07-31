"""Required version identifiers for persisted documents."""

from __future__ import annotations

from typing import Any

SCENARIO_SCHEMA_VERSION = "aerospace-workbench.scenario.v1"
VEHICLE_SCHEMA_VERSION = "aerospace-workbench.vehicle.v1"
RUN_SCHEMA_VERSION = "aerospace-workbench.run-manifest.v1"
SENSOR_STREAM_SCHEMA_VERSION = "aerospace-workbench.sensor-stream.v1"
CREDIBILITY_SCHEMA_VERSION = "aerospace-workbench.credibility.v2"
ROCKETPY_REFERENCE_SCHEMA_VERSION = "aerospace-workbench.rocketpy-reference.v1"

VEHICLE_KEYS = ("vehicle", "sensors", "avionics", "actuators")


def require_schema_version(
    document: dict[str, Any],
    expected: str,
    source: object | None = None,
) -> None:
    """Reject documents that do not use the required schema."""
    identifier = document.get("schema_version")
    if identifier == expected:
        return
    location = f" in {source}" if source is not None else ""
    raise ValueError(
        f"schema_version{location} must be {expected!r}, got {identifier!r}"
    )
