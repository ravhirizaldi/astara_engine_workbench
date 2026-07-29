"""Build and locate the native flight-software library."""

from __future__ import annotations

import subprocess
from pathlib import Path


def build_library() -> Path:
    root = Path(__file__).resolve().parents[3]
    source = root / "flight_core"
    build = source / "build"
    library = build / "libfsw_core.so"
    inputs = (
        tuple((source / "src").glob("*.cpp"))
        + tuple((source / "include").glob("*.h"))
        + (source / "CMakeLists.txt",)
    )
    if library.exists() and all(
        path.stat().st_mtime <= library.stat().st_mtime for path in inputs
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
    return library
