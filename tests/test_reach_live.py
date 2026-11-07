"""`press_live.py` closing the reach's loop on the arm, against a fake arm link: what reaches the socket, and above all what does not."""
import json
from pathlib import Path
import math
import tempfile
import threading
import time
import unittest

import numpy as np

import d1_hardware
import d1_ik
from demos.combiner_door.robot import arm_link
from demos.combiner_key.robot import leg_policy, press_live, reach_arm
from tests.test_press_live import REACH_POLICY, Clock, FakeOperator, FakeRobot, args

PERIOD_S = 1.0 / 9.0
#: What would run on the dog, with its real stops. `REACH_POLICY`, the old export, is here only to show they catch a bad one.
DEPLOYED = REACH_POLICY.parent / "reach_force_sticky_model_66600.npz"
#: Where the real arm rests, URDF radians.
FOLD = reach_arm.from_servo_deg(d1_hardware.REST_SERVO_DEG)


class FakeLink:
    """The arm link's port: reports every 111 ms of an arm that follows its `follow` goals; every line written is kept."""

    def __init__(self, clock, q=None, observe=False, reports=True):
        self.clock, self.hello = clock, {"type": "hello", "observe": observe}
        self.sock = self
        self.q = np.zeros(6) if q is None else np.asarray(q, dtype=float).copy()
        self.goal = self.q.copy()
        self.reported, self.stamp, self.next_report = self.q.copy(), None, clock.now
        self.reports = reports
        # The real class: what a `follow` goal becomes on the dog, one command a report, in servo degrees.
        self.follower = arm_link.Follower(d1_ik.to_servo_deg(self.q))
        self.lines: list[dict] = []
        self.broken = False
        self.stuck = False
        self.points: list = []
        self.playing = None

    def sendall(self, data: bytes) -> None:
        if self.broken:
            raise OSError("the link went away")
        for line in data.decode().splitlines():
            msg = json.loads(line)
            self.lines.append(msg)
            if msg["type"] == "follow":
                if self.playing is not None or self.follower is None:
                    self.follower = arm_link.Follower(d1_ik.to_servo_deg(self.reported))
                    self.playing = None
                self.follower.goal = np.array(msg["q"])
            elif msg["type"] == "path":
                why = arm_link.check_path(msg["q"], msg["dt"], d1_ik.to_servo_deg(self.reported))
                assert why is None, why                                    # what the real link would refuse
                self.playing = arm_link.PathPlayer(msg["q"], msg["dt"])
            elif msg["type"] == "hold":
                self.playing, self.follower = None, None
                self.goal = self.q.copy()

    def step(self, dt: float) -> None:
        self.q += (self.goal - self.q) * (1.0 - math.exp(-dt / 0.077))
        if self.reports and self.clock.now >= self.next_report:
            self.reported, self.stamp = self.q.copy(), self.clock.now
            self.next_report += PERIOD_S
            measured = d1_ik.to_servo_deg(self.reported)
            player = self.playing or self.follower
            if player is not None and not self.stuck:
                wire = player.step(measured, 0.0, PERIOD_S)
                self.goal = np.ravel(d1_ik.from_servo_deg(np.array(wire)))

    def latest(self):
        return self.reported, 0.0

    def latest_report(self):
        return self.stamp, self.reported

    def kinds(self) -> list[str]:
        return [m["type"] for m in self.lines]


@unittest.skipUnless(DEPLOYED.exists() and REACH_POLICY.exists(), "no exported reach policy")
class Rig(unittest.TestCase):
    """The real `PressRun`, the deployed export and its real stops, legs live on the fake robot, the arm on the fake link."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.saved_time = press_live.time
        press_live.time = self.clock
        self.meta = leg_policy.LegPolicy(DEPLOYED).meta

    def tearDown(self):
        press_live.time = self.saved_time
        self.tmp.cleanup()

    def make(self, link, *, send=True, obs="reports", live=True, policy=DEPLOYED, **kw):
        base = dict(policy=str(policy), live=live, presses=0, stand_s=3.0, run_dir=self.tmp.name, reach_goal="init_start",
                    reach_arm_obs=obs, reach_arm_send=send, reach_max_leg_off_rad=0.9, start_on_blend=True, start_tol_rad=1.0,
                    start_pitch_deg=(-90.0, 90.0))
        base.update(kw)
        self.robot = FakeRobot(self.meta)
        return press_live.PressRun(args(**base), self.robot if live else None, FakeOperator(), arm_link=link)

    def drive(self, run, link, *, script=None, steps=6000):
        for k in range(steps):
            self.robot.step()
            run.on_state(self.robot.msg())
            if script:
                script(k, run, link)
            link.step(0.02)
            if k % 30 == 0 and run.mode in ("preflight", "damping"):
                run.operator.lines.append("live" if run.confirming else "")
            run.tick()
            self.clock.now += 0.02
            if run.mode == "done":
                return k
        self.fail(f"never ended: mode {run.mode}, arm {run.arm_phase}")


class WholeBody(Rig):
    # --- the run

    def test_the_whole_run_in_order(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link)
        self.drive(run, link)
        self.assertIsNone(run.abort_reason)
        kinds = [k for k in link.kinds() if k != "keepalive"]
        first_follow, last_follow = kinds.index("follow"), len(kinds) - 1 - kinds[::-1].index("follow")
        self.assertEqual(kinds[0], "path")                                  # out, before any goal
        self.assertEqual(set(kinds[1:first_follow]), set())
        self.assertEqual(kinds[last_follow + 1:], ["hold", "path"])         # held, then back
        self.assertEqual(run.arm_phase, "folded_again")
        self.assertLess(np.degrees(np.abs(link.q - reach_arm.from_servo_deg(reach_arm.FOLD_SERVO_DEG)).max()), 3.5)

    def test_the_policy_waits_for_the_arm(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link)
        seen = []

        def watch(k, r, lk):
            if r.mode == "policy":
                seen.append(r.arm_phase)

        self.drive(run, link, script=watch)
        self.assertEqual(set(seen), {"ready"})

    def test_the_dog_does_not_lie_down_on_an_arm_still_out(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link)
        seen = []

        def watch(k, r, lk):
            if r.mode == "lie_end":
                seen.append(r.arm_phase)

        self.drive(run, link, script=watch)
        self.assertEqual(set(seen), {"folded_again"})

    def test_out_by_home_and_back_by_home_and_tuck(self):
        link = FakeLink(self.clock, q=FOLD)
        self.drive(self.make(link), link)
        out, back = [m["q"] for m in link.lines if m["type"] == "path"]
        near = lambda path, pose: min(float(np.abs(np.array(p) - pose).max()) for p in path)
        self.assertLess(near(out, reach_arm.HOME_SERVO_DEG), 0.01)
        self.assertLess(float(np.abs(np.array(out[-1])).max()), 0.01)
        self.assertLess(near(back, reach_arm.HOME_SERVO_DEG), 0.01)
        self.assertLess(near(back, reach_arm.TUCK_SERVO_DEG), 0.01)
        np.testing.assert_allclose(back[-1], reach_arm.FOLD_SERVO_DEG, atol=0.01)

    def test_a_goal_is_the_policys_target_with_joint1_and_joint4_flipped(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link)
        seen = []

        def spy(k, r, lk):
            if r.mode == "policy" and lk.lines and lk.lines[-1]["type"] == "follow":
                seen.append((np.array(lk.lines[-1]["q"]), r.arm_targets.target(r.obs.prev_actions[12:18]).copy()))

        self.drive(run, link, script=spy)
        wire, target = seen[-1]
        np.testing.assert_allclose(wire, np.degrees(target) * np.array([-1, 1, 1, -1, 1, 1]), atol=2e-3)

    # --- what stops it

    def test_an_arm_that_never_unfolds_stops_the_run_before_the_policy(self):
        link = FakeLink(self.clock, q=FOLD)
        link.stuck = True
        run = self.make(link)
        modes = []
        self.drive(run, link, script=lambda k, r, lk: modes.append(r.mode))
        self.assertNotIn("policy", modes)
        self.assertNotIn("follow", link.kinds())
        self.assertIn("not at the policy's start", run.abort_reason)

    def test_no_report_before_the_unfold_moves_nothing(self):
        link = FakeLink(self.clock, q=FOLD, reports=False)
        run = self.make(link)
        self.drive(run, link)
        self.assertNotIn("path", link.kinds())
        self.assertNotIn("follow", link.kinds())

    def test_a_dead_feed_still_goes_to_damping(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link)
        modes = []

        def go_quiet(k, r, lk):
            modes.append(r.mode)
            if r.arm_sender.sent > 10:
                lk.reports = False

        self.drive(run, link, script=go_quiet)
        self.assertNotIn("stand_end", modes)
        self.assertIn("damping", modes)

    def test_an_arm_gone_quiet_stops_it_and_holds(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link)

        def go_quiet(k, r, lk):
            if r.arm_sender.sent > 10:
                lk.reports = False

        self.drive(run, link, script=go_quiet)
        self.assertIn("no report for", run.abort_reason)
        kinds = [k for k in link.kinds() if k != "keepalive"]
        self.assertEqual(kinds[-1], "hold")                                 # a stop: not folded blind

    def test_the_deployed_policy_runs_30_s_with_no_stop(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, stand_s=30.0)
        self.drive(run, link, steps=8000)
        self.assertIsNone(run.abort_reason)
        self.assertEqual(run.arm_phase, "folded_again")
        self.assertGreater(run.arm_sender.sent, 1400)

    def test_the_real_stops_end_the_old_policy_once_the_grace_is_over(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, policy=REACH_POLICY, stand_s=20.0, reach_max_leg_off_rad=1e9)
        self.drive(run, link)
        self.assertIn("arm:", run.abort_reason)
        self.assertGreater(run.arm_sender.sent, 30)                         # about a second's worth went first

    def test_a_lead_stop_does_not_drop_the_dog(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, policy=REACH_POLICY, stand_s=20.0, reach_max_leg_off_rad=1e9)
        modes = []
        self.drive(run, link, script=lambda k, r, lk: modes.append(r.mode))
        order = [m for i, m in enumerate(modes) if i == 0 or modes[i - 1] != m]
        at = order.index("policy")
        self.assertEqual(order[at + 1:at + 3], ["stand_end", "lie_end"])   # not straight to damping
        kinds = [k for k in link.kinds() if k != "keepalive"]
        self.assertEqual(kinds[-2:], ["hold", "path"])                     # held, then folded
        self.assertEqual(run.arm_phase, "folded_again")

    def leg_bound_run(self, leg_action, tick):
        """The policy's leg outputs forced to `leg_action` from `tick` on: the first stop, as a reason, or None."""
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link)
        real = run.obs.record_action

        def forced(action):
            legs = real(action)
            if run.policy_ticks >= tick:
                legs = np.full_like(legs, leg_action)
            return legs

        run.obs.record_action = forced
        self.drive(run, link)
        return run.abort_reason

    def test_the_handovers_leg_transient_passes(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link)
        real = run.obs.record_action

        def transient(action):                                             # one joint at 0.95 rad for a moment, as simulation has it
            legs = real(action).copy()
            if run.policy_ticks < 3:
                legs[0] = 3.8
            return legs

        run.obs.record_action = transient
        self.drive(run, link)
        self.assertNotIn("leg target", run.abort_reason or "")

    def test_a_runaway_is_stopped_inside_the_first_second(self):
        self.assertIn("leg target", self.leg_bound_run(100.0, 0))

    def test_past_the_first_second_the_bound_is_the_flags(self):
        self.assertIn("past 0.9", self.leg_bound_run(3.8, 60))

    def test_a_stop_in_the_policy_holds_and_does_not_fold(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link)

        def stop(k, r, lk):
            if r.mode == "policy" and r.arm_sender.sent > 20 and r.abort_reason is None:
                r.abort("a test's stop")

        self.drive(run, link, script=stop)
        kinds = [k for k in link.kinds() if k != "keepalive"]
        self.assertEqual(kinds[-1], "hold")
        self.assertEqual(kinds.count("path"), 1)

    def test_a_dead_socket_is_not_an_exception_in_the_loop(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link)

        def cut(k, r, lk):
            lk.broken = r.arm_sender.sent > 5 or lk.broken

        self.drive(run, link, script=cut)

    # --- what is refused

    def test_never_with_the_legs_dry(self):
        for live, kw in ((False, {}), (False, {"loose": False})):
            with self.assertRaises(SystemExit) as caught:
                self.make(FakeLink(self.clock), live=live, **kw)
            self.assertIn("F-252", str(caught.exception))

    def test_the_flag_and_the_state_both_count(self):
        with self.assertRaises(SystemExit):
            press_live.PressRun(args(policy=str(REACH_POLICY), live=False, presses=0, run_dir=self.tmp.name, reach_goal="init_start",
                                     reach_arm_obs="reports", reach_arm_send=True), FakeRobot(self.meta), FakeOperator(),
                                arm_link=FakeLink(self.clock))

    def test_never_blind(self):
        with self.assertRaises(SystemExit):
            self.make(FakeLink(self.clock), obs="virtual")

    def test_the_measured_arm_is_still_refused_live(self):
        with self.assertRaises(SystemExit):
            self.make(FakeLink(self.clock), send=False, obs="measured")

    def test_reports_without_the_send_are_refused_live(self):
        with self.assertRaises(SystemExit):
            self.make(FakeLink(self.clock), send=False, obs="reports")

    def test_never_to_a_link_that_only_observes(self):
        with self.assertRaises(SystemExit):
            self.make(FakeLink(self.clock, observe=True))

    def test_never_without_a_link(self):
        with self.assertRaises(SystemExit):
            press_live.PressRun(args(policy=str(REACH_POLICY), live=True, presses=0, run_dir=self.tmp.name, reach_goal="init_start",
                                     reach_arm_obs="reports", reach_arm_send=True, arm_link_port=0), FakeRobot(self.meta), FakeOperator())

    def test_a_press_export_is_not_sent(self):
        with self.assertRaises(SystemExit):
            press_live.PressRun(args(presses=0, live=True, run_dir=self.tmp.name, reach_goal="init_start", reach_arm_obs="reports",
                                     reach_arm_send=True), FakeRobot(self.meta), FakeOperator(), arm_link=FakeLink(self.clock))

    def test_reading_the_arm_dry_writes_nothing(self):
        link = FakeLink(self.clock)
        run = self.make(link, send=False, live=False, stand_s=2.0)
        self.drive(run, link, steps=800)
        self.assertEqual(link.lines, [])
        self.assertIsNone(run.arm_sender)
        self.assertGreater(run.arm_reports.count, 5)


class FarAndWalking(Rig):
    """The two plans past a held goal, through the real runner, the policy and the arm link's chain."""

    FAR = [[0.45, 0.0, 0.0, 3.0, 2.0]]

    def rows(self, run, key):
        return np.array(run.rows[key])

    def on_policy(self, run):
        mode = self.rows(run, "mode")
        return mode == press_live.DRIVEN.index("policy")

    # --- the goal's path

    def test_the_goal_goes_out_and_comes_back_and_the_arm_folds(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, stand_s=14.0, reach_via=self.FAR, reach_settle_s=3.0)
        self.drive(run, link, steps=8000)
        self.assertIsNone(run.abort_reason)
        self.assertEqual(run.arm_phase, "folded_again")
        goal = self.rows(run, "commands")[self.on_policy(run)][:, 3:6]
        start = np.asarray(run.meta["reach_goals"]["init_start"])
        np.testing.assert_allclose(goal[100], start)
        np.testing.assert_allclose(goal[400], [0.45, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(goal[-1], start, atol=1e-6)
        self.assertLess(np.abs(np.diff(goal[1:], axis=0)).max(), 0.01)

    def test_a_run_too_short_for_its_path_is_refused(self):
        with self.assertRaises(SystemExit) as stop:
            self.make(FakeLink(self.clock, q=FOLD), stand_s=9.0, reach_via=self.FAR)
        self.assertIn("shorter than this path", str(stop.exception))

    def test_a_goal_it_never_trained_on_is_refused(self):
        with self.assertRaises(SystemExit):
            self.make(FakeLink(self.clock, q=FOLD), stand_s=30.0, reach_via=[[0.7, 0.0, 0.0, 3.0, 2.0]])

    def test_a_path_and_the_sticks_together_are_refused(self):
        with self.assertRaises(SystemExit):
            self.make(FakeLink(self.clock, q=FOLD), stand_s=30.0, reach_via=self.FAR, reach_walk=True, walk_max=(0.3, 0.2, 0.5))

    # --- the sticks

    def walking(self, push, **kw):
        """A walking run, with `push(seconds on the policy)` giving the sticks (lx, rx, ry, L2, ly)."""
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, reach_walk=True, walk_max=(0.3, 0.2, 0.5), reach_settle_s=3.0, **kw)

        def script(k, r, lk):
            self.robot.sticks = push(r.policy_ticks * 0.02) if r.mode == "policy" else (0.0,) * 5

        self.drive(run, link, steps=8000, script=script)
        return run

    def test_the_stick_walks_and_the_clock_runs_only_while_it_does(self):
        run = self.walking(lambda t: (0, 0, 0, 0, 1.0) if 4.0 <= t < 6.0 else (0.0,) * 5, stand_s=9.0)
        self.assertIsNone(run.abort_reason)
        on = self.on_policy(run)
        commands, frame = self.rows(run, "commands")[on], self.rows(run, "frame")[on]
        t = np.arange(len(commands)) * 0.02
        walking = (t > 4.2) & (t < 5.9)
        self.assertTrue((np.abs(commands[walking, 0] - 0.3) < 1e-9).all())
        cos = frame[:, 60]
        self.assertLess(cos[walking].min(), -0.9)                 # the clock turns
        self.assertGreater(cos[walking].max(), 0.9)
        self.assertTrue((cos[t > 6.2] == 1.0).all())              # and is pinned once released
        self.assertTrue((cos[t < 3.9] == 1.0).all())

    def test_the_sticks_do_nothing_until_the_arm_has_settled(self):
        run = self.walking(lambda t: (0, 0, 0, 0, 1.0), stand_s=6.0)
        on = self.on_policy(run)
        commands = self.rows(run, "commands")[on]
        t = np.arange(len(commands)) * 0.02
        self.assertTrue((commands[t < 2.95, :3] == 0.0).all())
        self.assertGreater(commands[(t > 3.1) & (t < 4.5), 0].min(), 0.25)

    def test_it_stands_a_second_before_the_legs_change_gains(self):
        run = self.walking(lambda t: (0, 0, 0, 0, 1.0), stand_s=8.0)
        on = self.on_policy(run)
        commands = self.rows(run, "commands")[on]
        moving = np.nonzero(np.abs(commands[:, :3]).sum(axis=1) > 0)[0]
        self.assertGreaterEqual((len(commands) - 1 - moving[-1]) * 0.02, press_live.WALK_STOP_S - 0.05)
        self.assertEqual(run.arm_phase, "folded_again")

    def test_q_with_a_stick_held_stands_first_too(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, reach_walk=True, walk_max=(0.3, 0.2, 0.5), reach_settle_s=3.0, stand_s=30.0)

        def script(k, r, lk):
            self.robot.sticks = (0, 0, 0, 0, 1.0)
            if r.mode == "policy" and r.policy_ticks == 250:
                r.operator.lines.append("q")

        self.drive(run, link, steps=8000, script=script)
        self.assertEqual(run.end_reason, "operator: q")
        commands = self.rows(run, "commands")[self.on_policy(run)]
        self.assertTrue((commands[-45:, :3] == 0.0).all())

    def test_q_before_the_handover_still_comes_out_and_folds(self):
        """The console's link lost during the stand-up (F-262): no policy to speak of, and the arm ends folded on its way."""
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, stand_s=16.0)

        def script(k, r, lk):
            if r.mode == "stand_up" and r.arm_phase == "ready" and "q" not in seen:
                seen.append("q")
                r.operator.lines.append("q")

        seen = []
        self.drive(run, link, steps=8000, script=script)
        self.assertEqual(seen, ["q"])
        self.assertEqual(run.end_reason, "operator: q")
        self.assertIsNone(run.abort_reason)
        self.assertEqual(run.arm_phase, "folded_again")
        self.assertLess(run.policy_ticks, 5)

    def test_a_dry_run_reads_the_sticks_and_shows_the_policy_nothing(self):
        link = FakeLink(self.clock)
        run = self.make(link, send=False, live=False, stand_s=6.0, reach_walk=True, walk_max=(0.3, 0.2, 0.5), reach_settle_s=1.0)

        def script(k, r, lk):
            self.robot.sticks = (0, 0, 0, 0, 1.0)

        self.drive(run, link, steps=1500, script=script)
        self.assertEqual(link.lines, [])
        self.assertTrue(np.allclose(self.rows(run, "commands")[self.on_policy(run)][:, :3], 0.0))
        self.assertTrue(any(e["event"] == "sticks" and e["velocity"][0] == 0.3 for e in run.events))
        self.assertEqual(self.rows(run, "sticks")[-1].tolist()[1], 1.0)

    # --- the stops, with the arm out

    def tilted(self, degrees):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, reach_soft_tilt_deg=12.0, stand_s=20.0, tilt_limit_deg=15.0)
        modes = []

        def script(k, r, lk):
            modes.append(r.mode)
            if r.mode == "policy" and r.policy_ticks > 150:
                half = math.radians(degrees) / 2
                self.robot.quat = np.array([math.cos(half), 0.0, math.sin(half), 0.0])

        self.drive(run, link, script=script)
        return run, link, [m for i, m in enumerate(modes) if i == 0 or modes[i - 1] != m]

    def test_a_tilt_past_the_soft_limit_folds(self):
        run, link, order = self.tilted(13.0)
        self.assertIn("tilt", run.abort_reason)
        at = order.index("policy")
        self.assertEqual(order[at + 1:at + 3], ["stand_end", "lie_end"])
        self.assertEqual(run.arm_phase, "folded_again")
        self.assertEqual([k for k in link.kinds() if k != "keepalive"][-2:], ["hold", "path"])

    def test_a_tilt_past_the_hard_limit_is_still_damping(self):
        run, link, order = self.tilted(16.0)
        self.assertIn("tilt", run.abort_reason)
        self.assertEqual(order[order.index("policy") + 1], "damping")
        self.assertEqual([k for k in link.kinds() if k != "keepalive"][-1], "hold")

    def test_a_leg_bound_with_the_arm_out_folds(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link)
        real = run.obs.record_action

        def forced(action):
            legs = real(action)
            return np.full_like(legs, 3.8) if run.policy_ticks >= 60 else legs

        run.obs.record_action = forced
        sent = []
        real_send = run._send

        def send(target, kp, kd):
            if run.mode == "policy":
                sent.append(float(np.abs(np.asarray(target) - run.default_q).max()))
            real_send(target, kp, kd)

        run._send = send
        modes = []
        self.drive(run, link, script=lambda k, r, lk: modes.append(r.mode))
        self.assertIn("past 0.9", run.abort_reason)
        order = [m for i, m in enumerate(modes) if i == 0 or modes[i - 1] != m]
        at = order.index("policy")
        self.assertEqual(order[at + 1:at + 3], ["stand_end", "lie_end"])
        self.assertLess(max(sent), 0.9)                                    # the target past the bound was not sent
        self.assertEqual(run.arm_phase, "folded_again")

    def test_a_hard_stop_on_the_runs_last_tick_is_not_undone(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, stand_s=3.0)
        modes = []
        real_send_arm = run._send_arm

        def send_arm(action):                                              # on the tick `stand_s` runs out, and only then
            if press_live.time.monotonic() - run.mode_t0 > run.args.stand_s:
                run.arm_watch.check = lambda *a: "no report for 0.40 s"
            real_send_arm(action)

        run._send_arm = send_arm
        self.drive(run, link, script=lambda k, r, lk: modes.append(r.mode))
        order = [m for i, m in enumerate(modes) if i == 0 or modes[i - 1] != m]
        self.assertEqual(order[order.index("policy") + 1], "damping")
        self.assertIn("no report", run.abort_reason)

    def test_the_stick_check_reads_them_and_walks_nothing(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, reach_walk=True, walk_read_only=True, walk_max=(0.3, 0.2, 0.5), reach_settle_s=3.0, stand_s=8.0)

        def script(k, r, lk):
            self.robot.sticks = (0, 0, 0, 0, 1.0)

        self.drive(run, link, steps=8000, script=script)
        self.assertIsNone(run.abort_reason)
        on = self.on_policy(run)
        self.assertTrue((self.rows(run, "commands")[on][:, :3] == 0.0).all())
        self.assertTrue((self.rows(run, "frame")[on][:, 60] == 1.0).all())   # the clock never ran
        self.assertTrue(any(e["event"] == "sticks" and e["velocity"][0] == 0.3 for e in run.events))


class TheCloth(Rig):
    """`--reach_cloth`: the jaws wait open with the arm out, A shuts them, the run goes on, and they let go before any fold."""

    def run_of(self, **kw):
        link = FakeLink(self.clock, q=FOLD)
        base = dict(reach_cloth=True, stand_s=8.0, reach_via=[[0.54, -9.6, -25.0, 2.0, 1.0]], reach_settle_s=1.0)
        base.update(kw)
        return link, self.make(link, **base)

    @staticmethod
    def press_a(after_s=0.0):
        """A script: A, once, `after_s` into the wait."""
        state = {"at": None, "done": False}

        def script(k, r, lk):
            if r.arm_phase == "cloth":
                state["at"] = state["at"] if state["at"] is not None else k
                if not state["done"] and (k - state["at"]) * 0.02 >= after_s:
                    state["done"] = True
                    r.operator.lines.append("")
        return script

    def test_nothing_starts_until_a(self):
        link, run = self.run_of()
        seen = []

        def script(k, r, lk):
            if r.arm_phase == "cloth":
                seen.append((r.mode, r.policy_ticks))
            if len(seen) > 400:
                r.operator.lines.append("estop")

        self.drive(run, link, script=script, steps=4000)
        self.assertGreater(len(seen), 300)
        self.assertEqual(set(seen[:400]), {("stand_up", 0)})
        self.assertIn(reach_arm.JAWS_OPEN_UNITS, [m.get("gripper") for m in link.lines])
        self.assertNotIn(reach_arm.JAWS_SHUT_UNITS, [m.get("gripper") for m in link.lines])

    def test_a_shuts_the_jaws_slowly_and_gives_time_to_step_back(self):
        link, run = self.run_of()
        marks = {}

        def script(k, r, lk):
            self.press_a_once(k, r, lk)
            if r.cloth_shut_t0 is not None and "shut" not in marks:
                marks["shut"] = self.clock.now
            if r.mode == "policy" and "policy" not in marks:
                marks["policy"] = self.clock.now

        self.press_a_once = self.press_a()
        self.drive(run, link, script=script, steps=9000)
        self.assertGreaterEqual(marks["policy"] - marks["shut"], reach_arm.CLOTH_CLOSE_S + reach_arm.CLOTH_LET_GO_S)
        closing = [m["gripper"] for m in link.lines if m["type"] == "keepalive" and m.get("gripper") not in (None, reach_arm.JAWS_OPEN_UNITS)]
        steps = np.abs(np.diff(closing))
        self.assertLess(steps.max(), 1.0)                       # a ramp, never a snap: 70 units over 2 s is 0.7 a tick
        self.assertEqual(closing[-1], reach_arm.JAWS_SHUT_UNITS)
        self.assertEqual(run.arm_phase, "folded_again")

    def test_the_squeeze_goes_with_every_goal(self):
        """The link takes a goal with no gripper as "where the jaws measure": on a cloth that is letting go of it, with nothing to say so."""
        link, run = self.run_of()
        self.drive(run, link, script=self.press_a(), steps=9000)
        goals = [m for m in link.lines if m["type"] == "follow"]
        self.assertGreater(len(goals), 100)
        self.assertEqual({m.get("gripper") for m in goals}, {reach_arm.JAWS_SHUT_UNITS})

    def test_the_fold_lets_go_first(self):
        link, run = self.run_of()
        self.drive(run, link, script=self.press_a(), steps=9000)
        paths = [m for m in link.lines if m["type"] == "path"]
        self.assertEqual([p.get("gripper") for p in paths], [None, reach_arm.JAWS_OPEN_UNITS])
        self.assertEqual(run.arm_phase, "folded_again")
        self.assertIsNone(run.abort_reason)

    def test_a_soft_stop_lets_go_too(self):
        link, run = self.run_of(stand_s=30.0, reach_via=[[0.54, -9.6, -25.0, 2.0, 20.0]])
        press = self.press_a()

        def script(k, r, lk):
            press(k, r, lk)
            if r.mode == "policy" and r.policy_ticks == 100:
                r._arm_stop("the test's stop")

        self.drive(run, link, script=script, steps=9000)
        self.assertEqual(run.abort_reason, "the test's stop")
        self.assertEqual([m.get("gripper") for m in link.lines if m["type"] == "path"][-1], reach_arm.JAWS_OPEN_UNITS)
        self.assertEqual(run.arm_phase, "folded_again")

    def test_an_a_before_the_wait_does_not_count(self):
        link, run = self.run_of()
        seen = []

        def script(k, r, lk):
            if r.mode == "stand_up" and r.arm_phase == "unfolding" and not seen:
                seen.append(k)
                r.operator.lines.append("")
            if r.arm_phase == "cloth" and len(seen) < 200:
                seen.append(k)
            elif len(seen) >= 200:
                r.operator.lines.append("estop")

        self.drive(run, link, script=script, steps=5000)
        self.assertIsNone(run.cloth_shut_t0)
        self.assertNotIn(reach_arm.JAWS_SHUT_UNITS, [m.get("gripper") for m in link.lines])

    def test_q_while_waiting_goes_down_with_the_jaws_open(self):
        """End gently, or the console's link lost: the arm folds, and no jaw shuts on whoever is holding the cloth."""
        link, run = self.run_of()

        def script(k, r, lk):
            if r.arm_phase == "cloth" and r.end_reason is None:
                r.operator.lines.append("q")

        self.drive(run, link, script=script, steps=9000)
        self.assertEqual(run.end_reason, "operator: q")
        self.assertLess(run.policy_ticks, 5)
        self.assertNotIn(reach_arm.JAWS_SHUT_UNITS, [m.get("gripper") for m in link.lines])
        self.assertEqual(run.arm_phase, "folded_again")

    def test_with_nobody_there_the_run_ends(self):
        link, run = self.run_of()
        self.drive(run, link, steps=int((reach_arm.CLOTH_WAIT_S + 40.0) / 0.02))
        self.assertIn("no A in", run.end_reason)
        self.assertEqual(run.arm_phase, "folded_again")
        self.assertLess(run.policy_ticks, 5)

    def test_an_e_stop_during_the_wait_stops_it(self):
        link, run = self.run_of()

        def script(k, r, lk):
            if r.arm_phase == "cloth":
                r.operator.lines.append("estop")

        self.drive(run, link, script=script, steps=4000)
        self.assertIn("E-stop", run.abort_reason)
        self.assertEqual(run.policy_ticks, 0)

    def test_it_needs_the_arm(self):
        with self.assertRaises(SystemExit):
            self.make(FakeLink(self.clock, q=FOLD), send=False, reach_cloth=True)

    def test_a_run_without_it_sends_no_gripper(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, stand_s=3.0)
        self.drive(run, link, steps=6000)
        self.assertEqual({m.get("gripper") for m in link.lines}, {None})


class TheWalkFirst(Rig):
    """`--reach_walk_first`: the sticks until A, then the goals, never the two together."""

    VIA = [[0.54, -9.6, -25.0, 2.0, 1.0]]
    rows, on_policy = FarAndWalking.rows, FarAndWalking.on_policy

    def run_of(self, **kw):
        link = FakeLink(self.clock, q=FOLD)
        base = dict(stand_s=8.0, reach_via=self.VIA, reach_settle_s=1.0, reach_walk=True, walk_max=(0.2, 0.15, 0.4), reach_walk_first=30.0)
        base.update(kw)
        return link, self.make(link, **base)

    def goals(self, run):
        return self.rows(run, "commands")[self.on_policy(run)]

    def a_at(self, seconds, sticks=(0, 0, 0, 0, 1.0), released=None):
        """Sticks forward from the settle, A `seconds` into the policy."""
        def script(k, r, lk):
            now = r.policy_ticks * 0.02
            self.robot.sticks = sticks if r.mode == "policy" and (released is None or now < released) else (0, 0, 0, 0, 0)
            if r.mode == "policy" and r.policy_ticks == int(seconds / 0.02):
                r.operator.lines.append("")
        return script

    def test_the_sticks_walk_until_a_and_never_after(self):
        link, run = self.run_of()
        self.drive(run, link, script=self.a_at(5.0), steps=9000)
        velocity = self.goals(run)[:, :3]
        self.assertGreater(np.abs(velocity[:250]).max(), 0.1)
        self.assertEqual(np.abs(velocity[252:]).max(), 0.0)

    def test_the_goal_waits_for_a(self):
        link, run = self.run_of()
        self.drive(run, link, script=self.a_at(5.0), steps=9000)
        goal = self.goals(run)[1:, int(run.meta["cmd_ee_radius"]):int(run.meta["cmd_ee_radius"]) + 3]
        moved = np.nonzero(np.abs(goal - goal[0]).max(axis=1) > 1e-6)[0]
        self.assertGreaterEqual(moved[0] * 0.02, 5.0 + 1.0 - 0.1)                 # A, then the settle
        self.assertEqual(run.end_reason, "--stand_s 8.0 s")

    def test_time_walking_does_not_come_off_the_goals(self):
        link, run = self.run_of()
        self.drive(run, link, script=self.a_at(20.0), steps=12000)
        self.assertEqual(run.end_reason, "--stand_s 8.0 s")
        self.assertGreaterEqual(run.policy_ticks * 0.02, 20.0 + 8.0)
        self.assertEqual(run.arm_phase, "folded_again")

    def test_an_a_in_the_settle_does_not_count(self):
        link, run = self.run_of(reach_settle_s=3.0, stand_s=12.0)

        def script(k, r, lk):
            if r.mode == "policy" and r.policy_ticks == 50:
                r.operator.lines.append("")
            if r.mode == "policy" and r.policy_ticks == 400:
                r.operator.lines.append("q")

        self.drive(run, link, script=script, steps=9000)
        self.assertIsNone(run.path_t0)
        self.assertEqual(run.end_reason, "operator: q")

    def test_nobody_presses_a(self):
        link, run = self.run_of(reach_walk_first=4.0)
        self.drive(run, link, steps=9000)
        self.assertIn("no A in", run.end_reason)
        self.assertIsNone(run.path_t0)
        self.assertEqual(run.arm_phase, "folded_again")

    def test_q_while_walking_stands_first(self):
        link, run = self.run_of()

        def script(k, r, lk):
            self.robot.sticks = (0, 0, 0, 0, 1.0)
            if r.mode == "policy" and r.policy_ticks == 250:
                r.operator.lines.append("q")

        self.drive(run, link, script=script, steps=9000)
        self.assertEqual(run.end_reason, "operator: q")
        self.assertTrue((self.goals(run)[-45:, :3] == 0.0).all())

    def test_with_the_cloth_the_first_a_shuts_and_the_second_starts(self):
        link, run = self.run_of(reach_cloth=True)
        said = {"cloth": False, "go": False}

        def script(k, r, lk):
            if r.arm_phase == "cloth" and not said["cloth"]:
                said["cloth"] = True
                r.operator.lines.append("")
            if r.mode == "policy" and r.policy_ticks == 150 and not said["go"]:
                said["go"] = True
                r.operator.lines.append("")

        self.drive(run, link, script=script, steps=12000)
        self.assertIsNotNone(run.path_t0)
        self.assertEqual(run.end_reason, "--stand_s 8.0 s")
        self.assertEqual({m.get("gripper") for m in link.lines if m["type"] == "follow"}, {reach_arm.JAWS_SHUT_UNITS})
        self.assertEqual(run.arm_phase, "folded_again")

    def test_it_refuses_the_two_at_once(self):
        with self.assertRaises(SystemExit):
            self.run_of(reach_walk_first=0.0)
        with self.assertRaises(SystemExit):
            self.run_of(reach_walk=False)
        with self.assertRaises(SystemExit):
            self.run_of(walk_read_only=True)

    def test_a_run_without_it_keeps_its_clock(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, stand_s=6.0, reach_walk=True, walk_max=(0.3, 0.2, 0.5), reach_settle_s=1.0)
        self.drive(run, link, steps=9000)
        self.assertEqual(run.end_reason, "--stand_s 6.0 s")
        self.assertLess(run.policy_ticks * 0.02, 7.5)


class TheForceInput(Rig):
    """`--reach_force_n`: the policy told a downward force while the hand is on its goals, and at no other time."""

    VIA = [[0.54, -9.6, -25.0, 2.0, 2.0], [0.54, -9.6, 25.0, 3.0, 4.0]]

    rows, on_policy = FarAndWalking.rows, FarAndWalking.on_policy

    def told(self, run):
        return -self.rows(run, "commands")[self.on_policy(run)][:, leg_policy.CMD_EE_FORCE_Z]

    def run_of(self, levels, **kw):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, stand_s=20.0, reach_via=self.VIA, reach_settle_s=1.0, reach_force_n=levels, **kw)
        self.drive(run, link, steps=9000)
        return run

    def test_it_comes_on_after_the_hand_arrives_and_goes_before_it_leaves(self):
        run = self.run_of([4.0])
        told = self.told(run)
        arrived, leaves = run.goal_path.points[0][1], run.goal_path.points[-1][0]       # from the path itself, not the force's own window
        self.assertEqual(told[:int(arrived / 0.02)].max(), 0.0)
        self.assertEqual(told[int(leaves / 0.02) + 1:].max(), 0.0)
        self.assertAlmostEqual(told.max(), 4.0, places=5)
        self.assertEqual(run.arm_phase, "folded_again")

    def test_it_never_jumps(self):
        told = self.told(self.run_of([0.0, 3.0, 6.0]))
        self.assertLessEqual(np.abs(np.diff(told)).max(), 6.0 * 0.02 / reach_arm.FORCE_RAMP_S + 1e-6)     # the steepest: off from the top
        self.assertAlmostEqual(told.max(), 6.0, places=5)

    def test_a_run_without_it_tells_the_policy_nothing(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, stand_s=20.0, reach_via=self.VIA, reach_settle_s=1.0)
        self.drive(run, link, steps=9000)
        self.assertEqual(np.abs(self.told(run)).max(), 0.0)

    def test_it_follows_the_path_clock_after_a_walk(self):
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, stand_s=20.0, reach_via=self.VIA, reach_settle_s=1.0, reach_force_n=[4.0], reach_walk=True, walk_max=(0.2, 0.15, 0.4), reach_walk_first=30.0)

        def script(k, r, lk):
            if r.mode == "policy" and r.policy_ticks == 500:
                r.operator.lines.append("")

        self.drive(run, link, script=script, steps=12000)
        told = self.told(run)
        self.assertEqual(told[:500].max(), 0.0)
        self.assertGreater(told.max(), 3.9)

    def test_it_refuses(self):
        for kw in (dict(reach_force_n=[9.0], reach_via=self.VIA), dict(reach_force_n=[-1.0], reach_via=self.VIA), dict(reach_force_n=[4.0])):
            with self.assertRaises(SystemExit):
                self.make(FakeLink(self.clock, q=FOLD), stand_s=20.0, reach_settle_s=1.0, **kw)

    def test_the_consoles_force_check_stays_on_one_spot(self):
        words = next(p for p in json.loads((REACH_POLICY.parents[1] / "demos/robot_console/policies.json").read_text()) if p["id"] == "reach_66600")["modes"]
        for mode, height in (("live_force_check", 0.03), ("live_force_check_tall", 0.05)):
            via = press_live.make_parser().parse_args(["--policy", "x", *words[mode]]).reach_via
            self.assertEqual([v[2] for v in via], [0.0])
            self.assertAlmostEqual(0.49 + via[0][0] * math.sin(math.radians(via[0][1])), 0.40 + height, delta=0.001)    # told nothing, the hand on the scale's top
        for n in (0, 2, 4, 6):
            told = press_live.make_parser().parse_args(["--policy", "x", *words[f"live_force_{n}"]]).reach_force_n
            self.assertEqual(told, [float(n)])

    def test_the_consoles_force_wipes_press_one_way_and_lift_to_come_back(self):
        """Every stroke right to left, where the hand runs low (F-270); the way back well over the bench even told 6 N. Heights from the ground, m."""
        words = next(p for p in json.loads((REACH_POLICY.parents[1] / "demos/robot_console/policies.json").read_text()) if p["id"] == "reach_66600")["modes"]
        height = lambda v: 0.49 + v[0] * math.sin(math.radians(v[1]))
        for n in (0, 2, 4, 6):
            via = press_live.make_parser().parse_args(["--policy", "x", *words[f"live_force_{n}"]]).reach_via
            self.assertEqual([v[2] for v in via], [-25.0, 25.0, 25.0, -25.0, -25.0, 25.0])
            down, up = height(via[0]), height(via[3])
            self.assertAlmostEqual(down, 0.41, delta=0.001)              # 0.7 cm under a 41.7 cm bench, as Force check was under its scale
            self.assertGreater(up - 6.0 / 200.0, 0.417 + 0.04)           # told 6 N moves the target 3 cm: 4 cm clear
            self.assertEqual([height(v) == down for v in via], [True, True, False, False, True, True])


class TheConsolesModes(Rig):
    """Every mode the console offers for the deployed policy, through the real parser and into a real run: a typo must not reach the dog."""

    def modes(self):
        policy = next(p for p in json.loads((REACH_POLICY.parents[1] / "demos/robot_console/policies.json").read_text()) if p["id"] == "reach_66600")
        return policy

    def parsed(self, words):
        return press_live.make_parser().parse_args(["--policy", self.modes()["file"], *words])

    def test_each_parses_and_builds(self):
        policy = self.modes()
        for name, words in policy["modes"].items():
            a = self.parsed(words)
            a.policy = str(REACH_POLICY.parent / Path(a.policy).name)
            a.run_dir, a.status_port = self.tmp.name, 0
            self.robot = FakeRobot(self.meta)
            live = name != "dry"
            link = FakeLink(self.clock, q=FOLD)
            run = press_live.PressRun(a, self.robot if live else None, FakeOperator(), arm_link=link)
            self.assertEqual(run.arm_send, live, name)
            if name.startswith("live"):
                self.assertEqual(policy["confirm"][name], "arm", name)        # the word the program asks for, whatever it moves
            if "far" in name or "wipe" in name or "force" in name:
                self.assertGreaterEqual(a.stand_s, run.goal_path.duration, name)
            self.assertEqual(bool(getattr(a, "reach_cloth", False)), "wipe" in name or "force" in name, name)
            bench = "wipe" in name or "force" in name
            self.assertEqual(run.walk is not None, "walk" in name or bench or name == "live_sticks", name)
            self.assertEqual(bool(run.walk_first), bench, name)
            self.assertEqual(run.press_force is not None, "force" in name, name)
            self.assertEqual(bool(a.walk_read_only), name == "live_sticks", name)

    def test_walking_and_the_longest_path_fit_under_the_policys_limit(self):
        """A run on the dog ended on `--policy_s` 90 s, a limit the walk's 120 s would otherwise always lose to."""
        for name, words in self.modes()["modes"].items():
            if "wipe" in name or "force" in name:
                a = self.parsed(words)
                self.assertGreater(a.policy_s, a.reach_settle_s + a.reach_walk_first + a.stand_s, name)

    def test_the_wipe_ladder_is_the_heights_it_says(self):
        """The goal's height over a 40 cm bench, on a stroke the hand can make: `run_plan_probe --plan wipe` played these."""
        modes = self.modes()["modes"]
        rungs = {"high": 0.06, "near": 0.02, "touch": 0.0, "firm": -0.02, "hard": -0.03}
        self.assertEqual({n for n in modes if "wipe" in n}, {f"live_wipe_{r}" for r in rungs})
        for rung, height in rungs.items():
            via = self.parsed(modes[f"live_wipe_{rung}"]).reach_via
            self.assertEqual([v[2] for v in via], [-25.0, 25.0, -25.0], rung)
            for r, pitch, *_ in via:
                self.assertEqual(r, 0.54, rung)
                self.assertAlmostEqual(0.49 + r * math.sin(math.radians(pitch)), 0.40 + height, delta=0.001, msg=rung)

    def test_the_page_has_a_label_for_every_mode_of_every_policy(self):
        page = (REACH_POLICY.parents[1] / "demos/robot_console/static/index.html").read_text()
        labels = page[page.index("const MODES="):].split("};")[0]
        for policy in json.loads((REACH_POLICY.parents[1] / "demos/robot_console/policies.json").read_text()):
            for name in policy["modes"]:
                self.assertIn(f"{name}:[", labels, policy["id"])

    def test_a_far_tour_runs_to_the_end(self):
        words = self.modes()["modes"]["live_far_tour"]
        a = self.parsed(words)
        link = FakeLink(self.clock, q=FOLD)
        run = self.make(link, stand_s=a.stand_s, reach_via=a.reach_via, reach_settle_s=a.reach_settle_s)
        self.drive(run, link, steps=8000)
        self.assertIsNone(run.abort_reason)
        self.assertEqual(run.arm_phase, "folded_again")


class TheRoutes(unittest.TestCase):
    def test_the_constants_are_the_door_programs(self):
        from demos.combiner_door.robot import plan
        self.assertEqual(reach_arm.FOLD_SERVO_DEG, plan.FOLD_SERVO_DEG)
        np.testing.assert_allclose(reach_arm.TUCK_SERVO_DEG, plan.tuck_servo_deg())

    def test_a_route_is_a_path_the_link_takes(self):
        path = reach_arm.route(reach_arm.FOLD_SERVO_DEG, reach_arm.HOME_SERVO_DEG, [0.0] * 6)
        self.assertIsNone(arm_link.check_path(path, 0.1, reach_arm.FOLD_SERVO_DEG))

    def test_the_way_back_is_a_path_the_link_takes(self):
        start = [20.0, 15.0, 10.0, -30.0, 25.0, 40.0]
        path = reach_arm.route(start, reach_arm.HOME_SERVO_DEG, reach_arm.TUCK_SERVO_DEG, reach_arm.FOLD_SERVO_DEG)
        self.assertIsNone(arm_link.check_path(path, 0.1, start))

    def test_it_never_passes_the_pace(self):
        path = np.array(reach_arm.route(reach_arm.FOLD_SERVO_DEG, reach_arm.HOME_SERVO_DEG, [0.0] * 6))
        self.assertLessEqual(float(np.abs(np.diff(path, axis=0)).max()) / 0.1, reach_arm.JOINT_MOVE_DEG_S + 0.1)


class TheLinkSaysWhenItsReportsCame(unittest.TestCase):
    def feedback(self, client, wait_s=0.2):
        """What `Link.report` sends for this client in `wait_s`, and whether its thread lived."""
        link = arm_link.Link.__new__(arm_link.Link)
        sent = []
        link.client, link.stopping, link.conn, link.lock, link.streaming = client, False, object(), threading.Lock(), False
        link.send = lambda conn, msg: sent.append(msg)
        thread = threading.Thread(target=link.report, daemon=True)
        thread.start()
        time.sleep(wait_s)
        alive = thread.is_alive()
        link.stopping = True
        thread.join(1.0)
        return sent, alive

    class Client:
        """Angles that say which report they are from; `stamps` is what `servo_report_s` reads, in turn, the last one held."""
        feedback_age_s = 0.01

        def __init__(self, stamps, angles):
            self.stamps, self.angles = list(stamps), angles

        @property
        def servo_report_s(self):
            return self.stamps.pop(0) if len(self.stamps) > 1 else self.stamps[0]

        def get_joint_angles(self):
            return [self.angles] * 6

        @staticmethod
        def get_gripper_units():
            return 50.0

    def test_the_feedback_carries_the_reports_own_time(self):
        sent, _ = self.feedback(self.Client([100.0], 1.0))
        self.assertEqual(sent[0]["report_s"], 100.0)

    def test_angles_before_the_first_report_do_not_end_the_feedback(self):
        # `arm_Feedback/1` fills the angles without stamping `servo_report_s`; the thread must wait, not die on round(None).
        sent, alive = self.feedback(self.Client([None], 1.0))
        self.assertTrue(alive)
        self.assertEqual(sent, [])

    def test_a_report_landing_between_the_reads_is_not_sent_as_a_pair(self):
        # The first pass reads 100 then 200: its angles could be either report's. The next pass reads 200 twice and is sent.
        sent, alive = self.feedback(self.Client([100.0, 200.0, 200.0], 1.0))
        self.assertTrue(alive)
        self.assertEqual([m["report_s"] for m in sent][:1], [200.0])


class TheStartGate(unittest.TestCase):
    def test_the_fold_is_refused_and_the_default_pose_is_not(self):
        self.assertIsNone(reach_arm.at_start(np.zeros(6), np.zeros(6)))
        self.assertIsNotNone(reach_arm.at_start(FOLD, np.zeros(6)))

    def test_it_says_which_joint(self):
        self.assertIn("joint 3", reach_arm.at_start(FOLD, np.zeros(6)))


class TheSender(unittest.TestCase):
    def test_a_failed_write_is_not_counted_as_sent(self):
        link = FakeLink(Clock())
        link.broken = True
        sender = reach_arm.ArmSender(link)
        self.assertFalse(sender.follow(np.zeros(6)))
        self.assertEqual(sender.sent, 0)

    def test_hold_goes_once(self):
        link = FakeLink(Clock())
        sender = reach_arm.ArmSender(link)
        sender.hold()
        sender.hold()
        self.assertEqual(link.kinds(), ["hold"])


if __name__ == "__main__":
    unittest.main()
