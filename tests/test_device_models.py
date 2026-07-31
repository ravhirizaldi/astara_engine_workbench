import copy
import ctypes
import unittest
from types import SimpleNamespace

from aerospace_workbench.configuration.scenarios import default_scenario
from aerospace_workbench.flight_software.abi import (
    FswAirDataSample,
    FswDiscreteInputs,
    FswDiscreteSample,
    FswPlatformStatus,
    FswPropulsionStatus,
    SensorFrame,
)
from aerospace_workbench.flight_software.bridge import (
    FswDeviceInputs,
    fsw_input_from_frame,
)
from aerospace_workbench.simulation.device_models import (
    AirDataComputerModel,
    DiscreteInputModule,
    EngineControllerModel,
    FlightComputerPlatformModel,
    RecoveryControllerModel,
)


class DeviceModelTests(unittest.TestCase):
    def setUp(self) -> None:
        self.scenario = default_scenario()

    def _config(self, name: str) -> dict:
        config = copy.deepcopy(
            self.scenario["avionics"]["devices"][name]
        )
        config.update(
            {
                "noise_stddev": 0.0,
                "accuracy": 1.0,
                "startup_delay_s": 0.0,
            }
        )
        return config

    def test_engine_health_is_estimated_without_truth_health(self) -> None:
        propulsion = self.scenario["vehicle"]["stages"][0]["propulsion"]
        base = {
            "stage": {"propulsion": propulsion},
            "fuel_kg": 10.0,
            "oxidizer_kg": 10.0,
            "last_chamber_pressure_pa": propulsion["chamber_pressure_pa"],
            "last_engine_temperature_k": propulsion[
                "combustion_temperature_k"
            ],
            "last_engine_rpm": 30_000.0,
        }
        first = SimpleNamespace(**base, engine_health_percent=0.0)
        second = SimpleNamespace(**base, engine_health_percent=100.0)
        first_sample = EngineControllerModel(
            self._config("engine_controller"), 4
        ).sample(first, 1.0)
        second_sample = EngineControllerModel(
            self._config("engine_controller"), 4
        ).sample(second, 1.0)
        self.assertEqual(
            first_sample["health_percent"],
            second_sample["health_percent"],
        )

        degraded = SimpleNamespace(
            **(
                base
                | {
                    "last_engine_temperature_k": 3_000.0,
                    "last_engine_rpm": 6_000.0,
                }
            )
        )
        degraded_sample = EngineControllerModel(
            self._config("engine_controller"), 4
        ).sample(degraded, 1.0)
        self.assertLess(
            degraded_sample["health_percent"],
            first_sample["health_percent"],
        )

    def test_fault_reset_and_command_acknowledgment(self) -> None:
        config = self._config("air_data_computer")
        config["startup_delay_s"] = 0.01
        model = AirDataComputerModel(config, 2)
        body = SimpleNamespace(
            last_dynamic_pressure_pa=123.0,
            aero_valid=True,
        )
        self.assertEqual(model.sample(body, 0.0)["valid"], 0)
        nominal = model.sample(body, 0.02)
        body.last_dynamic_pressure_pa = 400.0
        frozen = model.sample(body, 0.03, {"type": "freeze"})
        stuck = model.sample(body, 0.04, {"type": "stuck"})
        dropped = model.sample(body, 0.05, {"type": "dropout"})
        self.assertEqual(frozen["dynamic_pressure_pa"], nominal["dynamic_pressure_pa"])
        self.assertEqual(frozen["sample_time_s"], nominal["sample_time_s"])
        self.assertEqual(stuck["sample_time_s"], 0.04)
        self.assertEqual(dropped["valid"], 0)
        model.reset(1.0)
        self.assertEqual(model.sample(body, 1.0)["valid"], 0)

        platform_config = self._config("flight_computer_platform")
        platform_config["command_ack_delay_s"] = 0.01
        platform = FlightComputerPlatformModel(platform_config, 3)
        pending = platform.sample(0.0, 0.0, False, True, (2, 0.0))
        acknowledged = platform.sample(0.01, 0.0, False, True)
        self.assertEqual(pending["command_type"], 0)
        self.assertEqual(acknowledged["command_type"], 2)

    def test_all_requested_device_models_construct(self) -> None:
        model_types = {
            "air_data_computer": AirDataComputerModel,
            "engine_controller": EngineControllerModel,
            "discrete_input_module": DiscreteInputModule,
            "recovery_controller": RecoveryControllerModel,
            "flight_computer_platform": FlightComputerPlatformModel,
        }
        for name, model_type in model_types.items():
            with self.subTest(name=name):
                self.assertIsInstance(
                    model_type(self._config(name), 1), model_type
                )

    def test_bridge_uses_device_samples_instead_of_frame_truth_fields(
        self,
    ) -> None:
        frame = SensorFrame()
        frame.time_s = 2.0
        frame.dt_s = 0.005
        frame.dynamic_pressure_pa = 999.0
        frame.engine_health_percent = 1.0
        inputs = FswDeviceInputs(
            FswAirDataSample(125.0, 1.9, 1),
            FswPropulsionStatus(82.0, 1.8, 1, 1, 1),
            FswDiscreteInputs(
                FswDiscreteSample(1.7, 1, 1),
                FswDiscreteSample(1.6, 1, 0),
                FswDiscreteSample(1.6, 1, 0),
            ),
            FswPlatformStatus(1.95, 0.001, 1, 0, 1),
            3,
            1.75,
        )
        packed = fsw_input_from_frame(
            frame,
            device_inputs=inputs,
            command_type=inputs.command_type,
            command_sequence=4,
        )
        self.assertEqual(packed.air_data.dynamic_pressure_pa, 125.0)
        self.assertEqual(packed.propulsion.health_percent, 82.0)
        self.assertEqual(packed.discretes.stage_separated.asserted, 1)
        self.assertEqual(packed.platform.sample_time_s, 1.95)
        self.assertEqual(packed.command.issue_time_s, 1.75)
        self.assertEqual(packed.struct_size, ctypes.sizeof(type(packed)))


if __name__ == "__main__":
    unittest.main()
