import unittest

from aerospace_workbench.simulation.kernel import SimulationKernel
from aerospace_workbench.simulation.scheduler import MissionScheduler


class SchedulerTests(unittest.TestCase):
    def test_global_queue_is_stable_and_cancels_by_owner(self) -> None:
        kernel = SimulationKernel()
        seen: list[str] = []
        kernel.register("work", lambda event: seen.append(event.subsystem))
        kernel.schedule(1.0, 20, "work", "later-priority", owner="keep")
        kernel.schedule(1.0, 10, "work", "first", owner="keep")
        kernel.schedule(1.0, 10, "work", "second", owner="keep")
        kernel.schedule(0.5, 10, "work", "cancelled", owner="drop")
        kernel.cancel_owner("drop")

        kernel.run(1.0)

        self.assertEqual(seen, ["first", "second", "later-priority"])
        with self.assertRaisesRegex(ValueError, "past"):
            kernel.schedule(0.0, 10, "work")

    def test_run_horizon_preserves_future_event(self) -> None:
        kernel = SimulationKernel()
        seen: list[float] = []
        kernel.register("work", lambda event: seen.append(event.truth_time_s))
        kernel.schedule(2.0, 10, "work")

        kernel.run(1.0)
        kernel.run(2.0)

        self.assertEqual(seen, [2.0])

    def test_relative_timeline_event_uses_dispatch_time(self) -> None:
        kernel = SimulationKernel()
        seen: list[tuple[str, float]] = []
        timeline = [
            {
                "id": "source",
                "trigger": {"type": "fsw_fact", "fact": "ready"},
                "action": {"type": "record"},
            },
            {
                "id": "dependent",
                "trigger": {
                    "type": "after_event",
                    "event": "source",
                    "delay_s": 2.0,
                },
                "action": {"type": "record"},
            },
        ]
        scheduler = MissionScheduler(
            kernel,
            timeline,
            lambda entry, event: seen.append(
                (entry["id"], event.truth_time_s)
            ),
        )
        scheduler.publish("fsw_fact", "ready", 3.0, "upper_stage")

        kernel.run(5.0)

        self.assertEqual(seen, [("source", 3.0), ("dependent", 5.0)])


if __name__ == "__main__":
    unittest.main()
