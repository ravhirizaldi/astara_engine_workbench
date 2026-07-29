"""Read recorded sensor channels and commands for deterministic replay."""

from __future__ import annotations

import csv
import gzip
import itertools
import warnings
from contextlib import contextmanager
from pathlib import Path
from typing import Iterable, Iterator, TextIO

from ..configuration.schemas import (
    SENSOR_STREAM_SCHEMA_VERSION,
    SchemaMigrationWarning,
    normalize_schema_identifier,
)


def load_recorded_commands(
    sensor_path: Path,
) -> dict[tuple[str, float], int]:
    command_path = sensor_path.with_name("commands.csv")
    if not command_path.exists():
        return {}
    commands: dict[tuple[str, float], int] = {}
    with command_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            commands[
                (row["body"], round(float(row["time_s"]), 9))
            ] = int(row["command_type"])
    return commands


@contextmanager
def open_sensor_log(path: Path) -> Iterator[csv.DictReader]:
    stream: TextIO
    if path.suffix == ".gz":
        stream = gzip.open(path, "rt", newline="", encoding="utf-8")
    else:
        stream = path.open(newline="", encoding="utf-8")
    with stream:
        yield csv.DictReader(stream)


def grouped_sensor_rows(
    rows: Iterable[dict[str, str]],
) -> Iterator[list[dict[str, str]]]:
    groups = itertools.groupby(
        rows,
        key=lambda item: (
            item.get("body", ""),
            round(float(item["time_s"]), 9),
        ),
    )
    for _, row_group in groups:
        yield list(row_group)


def normalized_sensor_rows(
    rows: Iterable[dict[str, str]], source: Path
) -> Iterator[dict[str, str]]:
    """Normalize legacy stream identifiers while preserving row contents."""
    warned: set[str | None] = set()
    for row in rows:
        identifier = row.get("schema_version") or None
        if identifier is None:
            if identifier not in warned:
                warnings.warn(
                    f"schema-less sensor stream {source} is deprecated; "
                    f"use {SENSOR_STREAM_SCHEMA_VERSION!r}",
                    SchemaMigrationWarning,
                    stacklevel=2,
                )
                warned.add(identifier)
            row["schema_version"] = SENSOR_STREAM_SCHEMA_VERSION
        elif identifier != SENSOR_STREAM_SCHEMA_VERSION:
            if identifier not in warned:
                row["schema_version"] = normalize_schema_identifier(
                    identifier, SENSOR_STREAM_SCHEMA_VERSION, source
                )
                warned.add(identifier)
            else:
                row["schema_version"] = SENSOR_STREAM_SCHEMA_VERSION
        yield row
