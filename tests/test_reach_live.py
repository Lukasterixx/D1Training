"""`press_live.py` closing the reach's loop on the arm, against a fake arm link: what reaches the socket, and above all what does not."""
import json
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
DEPLOYED = REACH_POLICY.parent / "reach_force_locked_model_66200.npz"
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
class WholeBody(unittest.TestCase):
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
