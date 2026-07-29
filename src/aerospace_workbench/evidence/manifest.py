"""Run-manifest persistence and artifact registration."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable

from .artifacts import artifact_hashes, write_json


def register_artifacts(
    manifest: dict[str, Any], root: Path, paths: Iterable[Path]
) -> None:
    manifest.setdefault("artifacts", {}).update(artifact_hashes(root, paths))


def write_manifest(root: Path, manifest: dict[str, Any]) -> None:
    write_json(root / "manifest.json", manifest)
