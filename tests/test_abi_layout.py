import ctypes
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from aerospace_workbench.flight_software import abi


ABI_STRUCTS = (
    ("FswGuidancePoint", abi.GuidancePoint),
    ("FswConfig", abi.FswConfig),
    ("FswImuSample", abi.FswImuSample),
    ("FswMagnetometerSample", abi.FswMagnetometerSample),
    ("FswBarometerSample", abi.FswBarometerSample),
    ("FswGnssSample", abi.FswGnssSample),
    ("FswSensorSuite", abi.FswSensorSuite),
    ("FswAirDataSample", abi.FswAirDataSample),
    ("FswPropulsionStatus", abi.FswPropulsionStatus),
    ("FswDiscreteSample", abi.FswDiscreteSample),
    ("FswDiscreteInputs", abi.FswDiscreteInputs),
    ("FswPlatformStatus", abi.FswPlatformStatus),
    ("FswCommand", abi.FswCommand),
    ("FswDiscreteActuationCommand", abi.FswDiscreteActuationCommand),
    ("FswInput", abi.FswInput),
    ("FswOutput", abi.FswOutput),
)


class AbiLayoutTests(unittest.TestCase):
    def test_c_and_ctypes_sizes_and_offsets_match(self) -> None:
        root = Path(__file__).resolve().parents[1]
        compiler = shutil.which(os.environ.get("CC", "cc"))
        self.assertIsNotNone(compiler, "a C compiler is required for ABI tests")

        statements: list[str] = []
        for c_name, ctypes_type in ABI_STRUCTS:
            statements.append(
                f'printf("{c_name} %zu\\n", sizeof({c_name}));'
            )
            statements.extend(
                f'printf("{c_name}.{field} %zu\\n", '
                f"offsetof({c_name}, {field}));"
                for field, _field_type in ctypes_type._fields_
            )
        source = (
            "#include <stddef.h>\n"
            "#include <stdio.h>\n"
            '#include "fsw/fsw.h"\n'
            "int main(void) {\n"
            + "\n".join(statements)
            + "\nreturn 0;\n}\n"
        )

        with tempfile.TemporaryDirectory() as directory:
            temporary = Path(directory)
            source_path = temporary / "abi_layout.c"
            executable = temporary / "abi_layout"
            source_path.write_text(source, encoding="utf-8")
            subprocess.run(
                [
                    compiler,
                    "-std=c11",
                    "-I",
                    str(root / "flight_core" / "include"),
                    str(source_path),
                    "-o",
                    str(executable),
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            result = subprocess.run(
                [str(executable)],
                check=True,
                capture_output=True,
                text=True,
            )

        c_layout = {
            name: int(value)
            for name, value in (
                line.split() for line in result.stdout.splitlines()
            )
        }
        for c_name, ctypes_type in ABI_STRUCTS:
            self.assertEqual(
                c_layout[c_name],
                ctypes.sizeof(ctypes_type),
                c_name,
            )
            for field, _field_type in ctypes_type._fields_:
                self.assertEqual(
                    c_layout[f"{c_name}.{field}"],
                    getattr(ctypes_type, field).offset,
                    f"{c_name}.{field}",
                )


if __name__ == "__main__":
    unittest.main()
