"""Write and identify files in simulation evidence bundles."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable


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
