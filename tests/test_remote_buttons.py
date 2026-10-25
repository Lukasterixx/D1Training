"""The Go2 remote's A and B as every live program's two buttons (`lowlevel.RemoteButtons`): B is `a` + Enter, A is
Enter (`live` + Enter at the foot press's confirmation) and, after a stop, `r` + Enter. Here: the presses decoded, the
door program's keys (`combiner_door/robot/node.py`), and the holster draw's way back to its start (`holster_dog.py`)
on its simulated arm. The foot press's (`press_live.py`) is in `tests/test_press_live.py`. Without ROS or a robot: the
remote has been read on this dog only on `/lowstate`, never on `/lf/lowstate`, which the door and holster programs read.
"""
import queue
import types
import unittest

import numpy as np

from demos.combiner_key.robot import lowlevel as ll


def remote(*keys) -> list[int]:
    """`LowState.wireless_remote` with these keys down."""
    raw = [0] * 40
    for key in keys:
        bit = ll.KEYS.index(key)
        raw[2 + bit // 8] |= 1 << (bit % 8)
    return raw


class Presses(unittest.TestCase):
    def press(self, buttons, *keys):
        return buttons.update(ll.remote_keys(remote(*keys)))

    def test_the_bits_read_on_the_dog(self):
        # logs/leg_press/20261023T133243_leg_press_live recorded 32, 288 and 512.
        self.assertEqual(sum(1 << ll.KEYS.index(k) for k in ("L2", "A")), 288)
        self.assertEqual(1 << ll.KEYS.index("B"), 512)

    def test_each_counts_once_as_it_goes_down(self):
        b = ll.RemoteButtons()
        self.assertEqual(self.press(b, "A"), ["A"])
        self.assertEqual(self.press(b, "A"), [])            # held
        self.assertEqual(self.press(b), [])
        self.assertEqual(self.press(b, "A"), ["A"])          # pressed again

    def test_a_with_sport_modes_l2_is_not_ours(self):
        b = ll.RemoteButtons()
        self.assertEqual(self.press(b, "L2"), [])
        self.assertEqual(self.press(b, "L2", "A"), [])       # sport mode's stand up / lie down
        self.assertEqual(self.press(b, "A"), [])             # still the same press of A
        self.assertEqual(self.press(b), [])
        self.assertEqual(self.press(b, "L1", "A"), [])

    def test_b_stops_whatever_else_is_held(self):
        b = ll.RemoteButtons()
        self.assertEqual(self.press(b, "L2"), [])
        self.assertEqual(self.press(b, "L2", "B"), ["B"])    # sport mode's damping: a stop too
        self.assertEqual(self.press(b), [])
        self.assertEqual(self.press(b, "A", "B"), ["B"])     # both at once: the stop only


class DoorProgram(unittest.TestCase):
    """The door program's remote: onto its key queue from `/lf/lowstate`, read as the typed lines."""

    def setUp(self):
        try:
            from demos.combiner_door.robot import node
        except ImportError as exc:                              # its OpenCV/AprilTag dependencies
            self.skipTest(f"node.py not importable: {exc}")
        n = self.node = node.CombinerDoorNode.__new__(node.CombinerDoorNode)
        n.keys, n.buttons = queue.Queue(), ll.RemoteButtons()
        n.state, n.attempt, n.paused_for, n.confirmed, n.recovering = "paused", None, "grip", set(), False
        self.aborted, self.recovered = [], []
        n._abort = lambda why: (self.aborted.append(why), setattr(n, "state", "aborted"))
        n.attempt_stub = types.SimpleNamespace(recover=lambda: self.recovered.append(True),
                                               script=types.SimpleNamespace(phases=[]))

    def state(self, *keys):
        return types.SimpleNamespace(motor_state=[types.SimpleNamespace(q=0.0)] * 12, wireless_remote=remote(*keys))

    def tap(self, key):
        self.node._on_lowstate(self.state(key))
        self.node._on_lowstate(self.state())
        self.node._handle_keys()

    def test_a_goes_b_stops_a_recovers(self):
        n = self.node
        self.tap("A")                                           # Enter at the pause
        self.assertEqual(n.state, "waiting")
        self.assertIn("grip", n.confirmed)
        self.tap("B")                                           # a + Enter
        self.assertEqual(n.state, "aborted")
        self.assertEqual(len(self.aborted), 1)
        n.attempt = n.attempt_stub
        self.tap("A")                                           # r + Enter, once stopped
        self.assertEqual(self.recovered, [True])
        self.assertEqual(n.state, "attempt")
        self.assertTrue(n.recovering)

    def test_the_start_is_confirmed_once_for_the_whole_run(self):
        """Lukas, 2026-10-25: "make it only wait for user signal once at the start": the first Enter at the start
        marks the run started (`started`), which no stage the chain goes on to clears (`confirmed` is reset there)."""
        n = self.node
        walked = []
        n.paused_for, n.started, n._classic_walk = "start", False, lambda: walked.append(True)
        self.tap("A")
        self.assertTrue(n.started)
        self.assertEqual(walked, [True])
        n.confirmed = set()                                     # as `_then_door` and `_then_close` reset it
        self.assertTrue(n.started)

    def test_typed_lines_are_not_the_remote(self):
        n = self.node
        for typed in ("remote a", "remote b"):                  # typing is lowercased: never "remote A" or "remote B"
            n.keys.put(typed)
        n._handle_keys()
        self.assertEqual(n.state, "paused")
        self.assertEqual(self.aborted, [])


class HolsterRecovery(unittest.TestCase):
    """`holster_dog.recovery`: back to the start only over paths the arm has played, or the plan's way back."""

    @classmethod
    def setUpClass(cls):
        from demos.combiner_key.robot import holster_dog

        cls.dog = holster_dog
        cls.moves = holster_dog.plan()
        cls.names = [m["name"] for m in cls.moves]

    def runner(self, stop_in: str, after_ticks: int):
        """A dry runner on the simulated arm, stopped by the remote's B `after_ticks` into `stop_in`."""
        dog, k_stop = self.dog, self.names.index(stop_in)

        class Arm(dog.SimArm):
            ticks = 0

            def tick(arm):
                super().tick()
                if runner.k == k_stop:
                    arm.ticks += 1
                    if arm.ticks == after_ticks:
                        runner.keys.put("remote B")

        import tempfile
        from pathlib import Path

        runner = dog.Runner(Arm(), False, False, Path(tempfile.gettempdir()), self.moves)
        start = runner.arm.latest()[0].copy()
        with self.assertRaises(dog.Stop):
            runner.run(self.moves)
        self.assertEqual(runner.k, k_stop)
        return runner, start

    def test_stopped_on_the_way_out_it_plays_it_backwards(self):
        runner, start = self.runner("draw", 8)
        segments = self.dog.recovery(self.moves, runner.played, runner.k, runner.arm.latest()[0])
        self.assertEqual([m["name"] for m in segments],
                         ["undo draw", "undo close", "undo on", "undo lower", "undo reach", "undo turn"])
        by = {m["name"]: m for m in segments}
        held, opened = self.dog.GRIPPER_HOLD_UNITS, self.dog.GRIPPER_OPEN_UNITS
        self.assertEqual(by["undo draw"]["gripper"], held)      # still on the bar
        self.assertEqual(by["undo close"]["gripper"], opened)   # off it
        # Only points the arm was sent, and the first one where it is.
        played = np.concatenate([runner.played[n] for n in self.names[:self.names.index("draw") + 1]])
        for m in segments:
            gap = np.abs(m["points"][:, None, :] - played[None]).max(axis=2).min(axis=1)
            self.assertLess(gap.max(), 1e-9, m["name"])
        self.assertLess(np.abs(segments[0]["points"][0] - runner.arm.latest()[0]).max(), self.dog.ARRIVED_DEG + 2.0)
        runner.recover()
        self.assertLess(np.abs(runner.arm.latest()[0] - start).max(), self.dog.ARRIVED_DEG)

    def test_stopped_on_the_way_back_it_finishes_it(self):
        runner, start = self.runner("off", 5)
        segments = self.dog.recovery(self.moves, runner.played, runner.k, runner.arm.latest()[0])
        self.assertEqual([m["name"] for m in segments], ["off", "raise", "unreach", "fold"])
        self.assertLess(len(segments[0]["points"]), len(runner.played["off"]))   # from where it stopped
        runner.recover()
        self.assertLess(np.abs(runner.arm.latest()[0] - start).max(), self.dog.ARRIVED_DEG)

    def test_stopped_on_the_way_back_to_the_start_it_carries_on(self):
        runner, start = self.runner("draw", 8)
        k_undo = self.names.index("on")

        def tick(arm=runner.arm, base=runner.arm.tick):
            base()
            if runner.k == k_undo and not getattr(arm, "stopped_again", False):
                arm.stopped_again = True
                runner.keys.put("remote B")

        runner.arm.tick = tick
        with self.assertRaises(self.dog.Stop):
            runner.recover()
        self.assertEqual(runner.k, k_undo)                         # stopped undoing `on`
        runner.recover()                                           # the second A: on from there
        self.assertLess(np.abs(runner.arm.latest()[0] - start).max(), self.dog.ARRIVED_DEG)

    def test_the_remote_after_a_stop(self):
        runner = self.dog.Runner(self.dog.SimArm(), False, False, None, self.moves)
        for key in ("remote B", "", "remote A"):                 # B and a bare Enter do nothing once stopped
            runner.keys.put(key)
        self.assertTrue(runner.wait_recover())
        runner.keys.put("q")
        self.assertFalse(runner.wait_recover())


if __name__ == "__main__":
    unittest.main()
