"""Build and locate the native flight-software library."""

from __future__ import annotations

import subprocess
from pathlib import Path


def build_library() -> Path:
    root = Path(__file__).resolve().parents[3]
    source = root / "flight_core"
    build = source / "build"
    library = build / "libfsw_core.so"
    stamp = build / ".awb-fsw-build-stamp"
    inputs = (
        tuple((source / "src").rglob("*.cpp"))
        + tuple((source / "src").rglob("*.hpp"))
        + tuple((source / "include").rglob("*.h"))
        + (source / "CMakeLists.txt",)
    )
    if library.exists() and stamp.exists() and all(
        path.stat().st_mtime <= stamp.stat().st_mtime for path in inputs
    ):
        return library
    subprocess.run(
        [
            "cmake",
            "-S",
            str(source),
            "-B",
            str(build),
            "-DCMAKE_BUILD_TYPE=Release",
        ],
        check=True,
    )
    subprocess.run(["cmake", "--build", str(build), "--parallel"], check=True)
    stamp.touch()
    return library
