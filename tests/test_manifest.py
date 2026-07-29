import json
import tempfile
import unittest
from pathlib import Path

from aerospace_workbench.configuration.schemas import (
    RUN_SCHEMA_VERSION,
)
from aerospace_workbench.evidence.manifest import read_manifest


class ManifestTests(unittest.TestCase):
    def test_manifest_requires_current_schema(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "manifest.json"
            path.write_text(
                json.dumps({"schema_version": RUN_SCHEMA_VERSION}),
                encoding="utf-8",
            )
            manifest = read_manifest(path)
            self.assertEqual(manifest["schema_version"], RUN_SCHEMA_VERSION)

            path.write_text(
                json.dumps({"schema_version": "unsupported.run.v0"}),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "schema_version"):
                read_manifest(path)


if __name__ == "__main__":
    unittest.main()
