"""Run-manifest persistence and artifact registration."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

from ..configuration.schemas import (
    RUN_SCHEMA_VERSION,
    normalize_schema_document,
)
from .artifacts import artifact_hashes, write_json


def register_artifacts(
    manifest: dict[str, Any], root: Path, paths: Iterable[Path]
) -> None:
    manifest.setdefault("artifacts", {}).update(artifact_hashes(root, paths))


def write_manifest(root: Path, manifest: dict[str, Any]) -> None:
    write_json(root / "manifest.json", manifest)


def read_manifest(path: str | Path) -> dict[str, Any]:
    """Read a run manifest and normalize a recognized legacy identifier."""
    source = Path(path)
    manifest = json.loads(source.read_text(encoding="utf-8"))
    normalize_schema_document(manifest, RUN_SCHEMA_VERSION, source)
    return manifest
