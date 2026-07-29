"""Read recorded sensor channels and commands for deterministic replay."""

from __future__ import annotations

import csv
import gzip
import itertools
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator, TextIO


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
    rows: csv.DictReader,
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
