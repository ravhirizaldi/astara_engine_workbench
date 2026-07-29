import ast
import subprocess
import sys
import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1] / "src" / "aerospace_workbench"
CORE_PACKAGES = (
    "configuration",
    "mathematics",
    "flight_software",
    "simulation",
    "evidence",
    "replay",
    "adapters",
)
REMOVED_FLAT_MODULES = (
    "aero.py",
    "analysis.py",
    "cli.py",
    "flight_core.py",
    "math3d.py",
    "replay.py",
    "reporting.py",
    "rocketpy_adapter.py",
    "ui.py",
)


def imported_modules(path: Path) -> set[str]:
    modules: set[str] = set()
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            dots = "." * node.level
            modules.add(f"{dots}{node.module or ''}")
    return modules


class ImportBoundaryTests(unittest.TestCase):
    def test_root_import_is_lightweight(self) -> None:
        command = (
            "import sys; import aerospace_workbench; "
            "blocked={'tkinter','matplotlib','numpy'}; "
            "loaded={name.split('.')[0] for name in sys.modules}; "
            "assert not blocked & loaded, blocked & loaded"
        )
        subprocess.run([sys.executable, "-c", command], check=True)

    def test_core_packages_do_not_import_application_or_presentation(self) -> None:
        for package in CORE_PACKAGES:
            for path in (PACKAGE / package).rglob("*.py"):
                imports = imported_modules(path)
                with self.subTest(path=path):
                    self.assertFalse(
                        any(
                            "application" in name or "presentation" in name
                            for name in imports
                        ),
                        imports,
                    )

    def test_flight_software_does_not_import_simulation(self) -> None:
        for path in (PACKAGE / "flight_software").glob("*.py"):
            with self.subTest(path=path):
                self.assertFalse(
                    any(
                        "simulation" in name
                        for name in imported_modules(path)
                    )
                )

    def test_flat_modules_were_removed(self) -> None:
        self.assertEqual(
            [name for name in REMOVED_FLAT_MODULES if (PACKAGE / name).exists()],
            [],
        )
