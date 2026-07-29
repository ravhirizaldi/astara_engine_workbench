"""Write and identify files in simulation evidence bundles."""

from __future__ import annotations

import csv
import hashlib
import json
import shutil
from pathlib import Path
from typing import Any, Iterable

from ..configuration.scenarios import configuration_source_files
from ..configuration.vehicles import evidence_documents


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, document: Any) -> None:
    path.write_text(json.dumps(document, indent=2), encoding="utf-8")


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def artifact_hashes(
    root: Path, paths: Iterable[Path]
) -> dict[str, str]:
    return {
        str(path.relative_to(root)): sha256_file(path)
        for path in paths
    }


def write_configuration_artifacts(
    root: Path, scenario: dict[str, Any]
) -> list[Path]:
    """Write canonical snapshots and preserve any file-backed inputs."""
    scenario_document, vehicle_document = evidence_documents(scenario)
    paths = [root / "scenario.json"]
    write_json(paths[0], scenario_document)
    if vehicle_document is not None:
        paths.append(root / "vehicle_definition.json")
        write_json(paths[-1], vehicle_document)

    source_names = {
        "scenario": "source_scenario.json",
        "vehicle": "source_vehicle_definition.json",
    }
    for role, source in configuration_source_files(scenario).items():
        target = root / source_names[role]
        shutil.copyfile(source, target)
        paths.append(target)
    return paths
