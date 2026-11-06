"""The arm link (`unifp_train/arm_link.py`) on the CPU: what each part of the real arm does to commands and reports."""
import math
import unittest

import numpy as np
import torch

import motor_model
from position_only.core import TrapezoidTracker
from unifp_train import arm_link_check as check
from unifp_train.arm_link import ArmLink

DT = 0.02
N = 64


def link(**kw):
    made = ArmLink(N, "cpu", DT, **kw)
    made.reset(None, torch.zeros(N, 6))
    return made


class TestPassThrough(unittest.TestCase):
    def test_nothing_asked_for_changes_nothing(self):
        arm = link()
        target = torch.randn(N, 6)
        arm.command(target)
        pos, vel = arm.drive(0.005)
        self.assertTrue(torch.equal(pos, target))
        self.assertTrue(torch.equal(vel, torch.zeros_like(target)))
        q, dq = torch.randn(N, 6), torch.randn(N, 6)
        seen_q, seen_dq = arm.observe(q, dq)
        self.assertTrue(torch.equal(seen_q, q) and torch.equal(seen_dq, dq))


class TestCommands(unittest.TestCase):
    def test_the_arm_gets_one_command_a_period_and_holds_it(self):
        arm = link(command_hz=(10.0, 10.0))
        changes, last = torch.zeros(N), arm.target.clone()
        steps = 500                                            # 10 s
        for k in range(steps):
            arm.command(torch.full((N, 6), float(k + 1)))
            changes += (arm.target != last).any(dim=1).float()
            last = arm.target.clone()
        # 10 Hz over a 50 Hz grid: 100 commands in 10 s, within one for where the phase fell.
        self.assertTrue(((changes - 100).abs() <= 1).all(), changes)

    def test_each_environment_draws_a_rate_in_the_range(self):
        arm = link(command_hz=motor_model.D1_COMMAND_HZ_RANGE)
        hz = 1.0 / arm.period
        self.assertTrue(((hz >= 9.0 - 1e-4) & (hz <= 10.0 + 1e-4)).all())
        self.assertGreater(float(hz.std()), 0.1)

    def test_phases_differ_between_environments(self):
        arm = link(command_hz=(10.0, 10.0))
        self.assertGreater(len(set(arm.next_command.tolist())), N // 2)

    def test_a_target_past_the_limits_is_clamped_as_d1_hardware_clamps_it(self):
        arm = link(lower=[-1.0] * 6, upper=[1.0] * 6)
        arm.command(torch.full((N, 6), 7.0))
        self.assertTrue(torch.allclose(arm.target, torch.ones(N, 6)))


class TestPlanner(unittest.TestCase):
    def test_it_is_the_fitted_tracker(self):
        arm = link(planner=True)
        plan = motor_model.arm_trajectory("measured")
        limits = [motor_model.D1_VELOCITY_LIMIT_RAD_S[f"Joint{i}"] for i in range(1, 7)]
        ref = TrapezoidTracker(N, 6, plan["accel_rad_s2"], plan["decel_rad_s2"], limits, dead_time=plan["dead_time_s"],
                               retention=plan["replan_velocity_retention"])
        ref.reset(None, torch.zeros(N, 6))
        target = torch.full((N, 6), 0.5)
        arm.command(target)
        ref.command(torch.ones(N, dtype=torch.bool), target)
        for _ in range(40):
            pos, _ = arm.drive(0.005)
            self.assertTrue(torch.equal(pos, ref.step(0.005)))

    def test_speed_stays_under_the_measured_ceiling(self):
        arm = link(planner=True)
        arm.command(torch.full((N, 6), 3.0))
        top = 0.0
        for _ in range(400):
            _, vel = arm.drive(0.005)
            top = max(top, float(vel.abs().max()))
        self.assertLessEqual(top, max(motor_model.D1_VELOCITY_LIMIT_RAD_S.values()) + 1e-5)


class TestReports(unittest.TestCase):
    def test_nine_reports_a_second_to_a_tenth_of_a_degree(self):
        arm = link(feedback_hz=9.0, quantum_deg=0.1)
        changes = torch.zeros(N)
        last = arm.reported.clone()
        for k in range(500):
            arm.clock += DT
            q = torch.full((N, 6), 0.01 * (k + 1))             # never still, so every report differs
            seen, _ = arm.observe(q, torch.zeros_like(q))
            changes += (seen != last).any(dim=1).float()
            last = seen.clone()
        self.assertTrue(((changes - 90).abs() <= 2).all(), changes)
        steps = torch.rad2deg(last) / 0.1
        self.assertTrue(torch.allclose(steps, torch.round(steps), atol=1e-3))

    def test_speed_is_the_difference_of_reports_not_the_true_speed(self):
        arm = link(feedback_hz=9.0)
        speed = 0.5
        seen_rate = None
        for k in range(100):
            arm.clock += DT
            q = torch.full((N, 6), speed * DT * (k + 1))
            _, seen_rate = arm.observe(q, torch.full((N, 6), 99.0))
        self.assertLess(float((seen_rate - speed).abs().max()), 0.05)

    def test_observing_twice_in_a_step_changes_nothing(self):
        arm = link(feedback_hz=9.0)
        arm.next_report[:] = 0.0                               # a report due this step, as a 20 ms step finds it
        arm.clock += DT
        q = torch.randn(N, 6)
        first = [t.clone() for t in arm.observe(q, q)]
        second = arm.observe(q + 1.0, q)
        self.assertTrue(torch.equal(first[0], second[0]) and torch.equal(first[1], second[1]))


def first_report_of_motion(arm, offset_ticks=0, joint_step=0.5):
    """Seconds from a command to the first report that shows the joint moved, on the planner, per environment."""
    q = torch.zeros(N, 6)
    sent_at = torch.full((N,), float("nan"))
    seen_at = torch.full((N,), float("nan"))
    target = torch.zeros(N, 6)
    for k in range(120):
        before = arm.target.clone()
        if k == 20 + offset_ticks:
            target = torch.full((N, 6), joint_step)
        arm.command(target)
        sent = torch.isnan(sent_at) & (arm.target != before).any(dim=1)
        sent_at[sent] = arm.clock[sent]
        for _ in range(4):
            q, _ = arm.drive(0.005)
        seen, _ = arm.observe(q.clone(), torch.zeros(N, 6))
        moved = torch.isnan(seen_at) & ~torch.isnan(sent_at) & (seen.abs() > 1e-4).any(dim=1)
        seen_at[moved] = arm.clock[moved]
    return seen_at - sent_at


class TestLocked(unittest.TestCase):
    def test_one_command_a_report_at_the_reports_period(self):
        arm = link(feedback_hz=9.0, locked=True, planner=True)
        self.assertTrue(torch.allclose(arm.period, torch.full((N,), 1 / 9), atol=1e-6))
        sends, last = torch.zeros(N), arm.target.clone()
        for k in range(450):                                   # 9 s
            arm.command(torch.full((N, 6), float(k + 1)))
            sends += (arm.target != last).any(dim=1).float()
            last = arm.target.clone()
        self.assertTrue(((sends - 81).abs() <= 1).all(), sends)

    def test_a_locked_stream_keeps_its_place_against_the_reports(self):
        arm = link(feedback_hz=9.0, locked=True)
        # The first step past this, half a step later on average: 20 ms in the mean.
        offset = arm.next_command - arm.next_report
        self.assertTrue(torch.allclose(offset, torch.full((N,), 0.02 - DT / 2), atol=1e-6))

    def test_it_needs_reports(self):
        with self.assertRaises(ValueError):
            ArmLink(N, "cpu", DT, locked=True)

    def test_a_report_shows_the_arm_as_it_was(self):
        arm = link(feedback_hz=50.0, report_age_s=0.06)
        seen = []
        for k in range(10):
            arm.command(torch.zeros(N, 6))
            s, _ = arm.observe(torch.full((N, 6), float(k + 1)), torch.zeros(N, 6))
            seen.append(float(s[0, 0]))
        self.assertEqual(seen[-1], 7.0)                        # three 20 ms steps old

    def test_an_aged_report_is_still_idempotent(self):
        arm = link(feedback_hz=9.0, report_age_s=0.06)
        arm.next_report[:] = 0.0
        arm.command(torch.zeros(N, 6))
        q = torch.randn(N, 6)
        first = arm.observe(q, q)[0].clone()
        self.assertTrue(torch.equal(first, arm.observe(q + 1.0, q)[0]))

    def test_the_report_grid_alone_gives_most_of_the_measured_delay(self):
        # F-021's lesson (`motor_model`): a delay read off 9 Hz reports is mostly the wait for the next report.
        lag = first_report_of_motion(link(feedback_hz=9.0, locked=True, planner=True))
        self.assertGreater(float(lag.median()), 0.06)
        self.assertLess(float(lag.median()), 0.13)


class TestAgainstTheArm(unittest.TestCase):
    """The locked profile against the arm: the commands it was sent, replayed (`arm_link_check`, `tests/data/d1_locked_swing.json`)."""

    @classmethod
    def setUpClass(cls):
        cls.data = check.blocks()
        cls.arm = check.mean_read([check.read(*check.measured(b)) for b in cls.data.values()])
        cls.model = check.mean_read([check.read(check.measured(b)[0], np.c_[check.measured(b)[1][:, 0], check.replay(b)])
                                     for b in cls.data.values()])

    def test_the_arm_reads_as_it_was_reported(self):
        self.assertAlmostEqual(self.arm["shift_ms"], 98.0, delta=3.0)
        self.assertAlmostEqual(self.arm["half_range_deg"], 7.96, delta=0.1)

    def test_the_model_follows_the_reports(self):
        self.assertLess(check.replay_rms(), 0.3)

    def test_the_summaries_agree_on_one_measure(self):
        self.assertAlmostEqual(self.model["shift_ms"], self.arm["shift_ms"], delta=8.0)
        self.assertAlmostEqual(self.model["half_range_deg"], self.arm["half_range_deg"], delta=0.6)
        self.assertAlmostEqual(self.model["speed_p50_deg_s"], self.arm["speed_p50_deg_s"], delta=2.0)
        self.assertAlmostEqual(self.model["speed_peak_deg_s"], self.arm["speed_peak_deg_s"], delta=3.0)

    def test_the_profile_trained_on_by_mistake_fails_this(self):
        # Lag 80, dead 20, age 40: fitted to a 150 ms lag on a measure that reads 98 on the arm. This is what caught it.
        plan = dict(motor_model.arm_trajectory("locked"), dead_time_s=0.02)
        self.assertGreater(check.replay_rms(plan=plan, servo_lag_s=0.08, report_age_s=0.04), 1.0)

    def test_the_locked_link_agrees_on_the_dogs_steps_and_on_trainings(self):
        for dt in (0.005, 0.02):
            read = check.mean_read([check.read(*check.stream(b, dt)) for b in self.data.values()])
            self.assertAlmostEqual(read["shift_ms"], self.arm["shift_ms"], delta=12.0, msg=dt)
            self.assertAlmostEqual(read["half_range_deg"], self.arm["half_range_deg"], delta=0.8, msg=dt)

    def test_the_first_report_of_motion(self):
        self.assertAlmostEqual(check.first_motion_ms(0.005), 89.0, delta=6.0)

    def test_it_holds_on_the_six_joint_draw_it_was_not_fitted_to(self):
        rms = check.replay_draw()
        self.assertLess(max(rms["all"]), 0.7)
        self.assertLess(max(rms["moving"][:3]), 0.45)          # shoulder and elbow
        self.assertLess(max(rms["moving"][3:]), 1.0)           # the wrist is ~20 ms faster than this one lag

    def test_the_wrists_own_lag_brings_it_to_the_draw(self):
        rms = check.replay_draw(plan=motor_model.arm_trajectory("locked"), servo_lag_s=0.068, wrist_lag_s=0.052)
        self.assertLess(max(rms["moving"]), 0.5)

    def test_the_mistaken_profile_fails_the_draw_too(self):
        plan = dict(motor_model.arm_trajectory("locked"), dead_time_s=0.06)
        self.assertGreater(max(check.replay_draw(plan=plan, servo_lag_s=0.08)["moving"]), 2.0)

    def test_on_trainings_20_ms_steps_the_stream_alternates_about_111_ms(self):
        sends, _ = check.stream(next(iter(self.data.values())), 0.02)
        gaps = 1000 * np.diff(sends[:, 0])
        self.assertLess(abs(float(gaps.mean()) - 111.1), 1.0)
        self.assertTrue(set(gaps.round().astype(int).tolist()) <= {100, 120})


class TestSkips(unittest.TestCase):
    def test_a_skip_takes_its_command_and_leaves_a_two_cycle_gap(self):
        arm = link(feedback_hz=9.0, locked=True, skip_probability=1.0, planner=True)
        arm.skip_probability = 0.0
        for k in range(40):
            arm.command(torch.full((N, 6), float(k)))
            arm.observe(torch.zeros(N, 6), torch.zeros(N, 6))
        arm.skip_probability = 1.0                             # the next report skips, everywhere
        reported = arm.last_report.clone()
        for k in range(5):
            arm.command(torch.full((N, 6), 99.0))
            arm.observe(torch.zeros(N, 6), torch.zeros(N, 6))
        self.assertTrue(torch.equal(arm.last_report, reported))
        self.assertTrue(((arm.target == 99.0).any(dim=1).float().mean()) < 0.5)

    def test_the_rate_is_light_noise(self):
        arm = link(feedback_hz=9.0, locked=True, skip_probability=0.0015)
        missing = 0
        steps = 45000                                          # 900 s, about 8100 reports an environment
        for _ in range(steps):
            arm.clock += DT
            before = arm.last_report.clone()
            due = arm.clock >= arm.next_report
            arm.observe(torch.zeros(N, 6), torch.zeros(N, 6))
            missing += int((due & (arm.last_report == before)).sum())
        self.assertAlmostEqual(missing / (8100 * N), 0.0015, delta=0.0006)


class TestCommandMean(unittest.TestCase):
    def test_it_sends_the_mean_since_the_last_command(self):
        arm = link(command_hz=(10.0, 10.0), command_mean=True)
        arm.next_command[:] = 0.08                             # the fifth step's
        for k, value in enumerate((1.0, 2.0, 3.0, 4.0, 5.0)):
            arm.command(torch.full((N, 6), value))
        self.assertTrue(torch.allclose(arm.target, torch.full((N, 6), 3.0)))

    def test_the_clamp_keeps_the_mean(self):
        # The environment always passes limits: a clamp reading the raw target threw the mean away, and no test had limits.
        arm = link(command_hz=(10.0, 10.0), command_mean=True, lower=[-9.0] * 6, upper=[9.0] * 6)
        arm.next_command[:] = 0.08
        for value in (1.0, 2.0, 3.0, 4.0, 5.0):
            arm.command(torch.full((N, 6), value))
        self.assertTrue(torch.allclose(arm.target, torch.full((N, 6), 3.0)))

    def test_it_starts_again_after_each_command(self):
        arm = link(command_hz=(10.0, 10.0), command_mean=True)
        arm.next_command[:] = 0.0
        arm.command(torch.full((N, 6), 9.0))
        arm.next_command[:] = 0.04
        arm.command(torch.full((N, 6), 1.0))
        arm.command(torch.full((N, 6), 3.0))
        self.assertTrue(torch.allclose(arm.target, torch.full((N, 6), 2.0)))

    def test_a_jittering_output_sent_as_a_mean_stops_jittering(self):
        def spread(**kw):
            arm = link(command_hz=(10.0, 10.0), **kw)
            seen = []
            for k in range(400):
                arm.command(torch.full((N, 6), 1.0 if k % 2 else -1.0))
                seen.append(arm.target[0, 0].item())
            return max(seen[50:]) - min(seen[50:])
        self.assertGreater(spread(), 1.9)
        self.assertLess(spread(command_mean=True), 0.8)


class TestFollowerAgainstTheDogs(unittest.TestCase):
    """The link's `follower` against the class that would really run, step for step."""

    def real(self):
        from demos.combiner_door.robot import arm_link as dog
        return dog

    def test_the_constants_are_the_dogs(self):
        import unifp_train.arm_link as sim
        dog = self.real()
        self.assertEqual((sim.FOLLOW_SPEED_DEG_S, sim.FOLLOW_LEAD_DEG, sim.FOLLOW_LEAD_MAX_DEG),
                         (dog.MAX_SPEED_DEG_S, dog.LEAD_DEG, dog.PATH_LEAD_MAX_DEG))

    def test_step_for_step(self):
        dog = self.real()
        rng = np.random.default_rng(7)
        arm = ArmLink(1, "cpu", DT, command_hz=(9.0, 9.0), follower=True, seed=2)
        arm.reset(None, torch.zeros(1, 6))
        real = dog.Follower(np.zeros(6))
        goal = np.zeros(6)
        steps = waits = 0
        for k in range(3000):
            due = float(arm.clock[0]) >= float(arm.next_command[0])
            goal = goal + rng.normal(0.0, 1.5, 6)
            measured = np.degrees(arm.reported[0].numpy().astype(float))
            age = float(arm.clock[0] - arm.last_report[0])
            before = real.waited
            if due:
                real.goal = goal.copy()
                expected = real.step(measured, age, float(arm.period[0]))
            arm.command(torch.as_tensor(np.radians(goal), dtype=torch.float32).unsqueeze(0))
            if due:
                steps += 1
                waits += real.waited - before
                np.testing.assert_allclose(np.degrees(arm.target[0].numpy()), expected, atol=2e-3)
                arm.reported[0] = arm.target[0] * 0.8          # an arm that trails, so the lead rule is exercised
        self.assertGreater(steps, 300)
        self.assertGreater(waits, 5)


class TestSentStep(unittest.TestCase):
    def test_it_reads_the_change_the_arm_received_and_nothing_between(self):
        arm = link(command_hz=(10.0, 10.0))
        arm.next_command[:] = 0.04
        seen = []
        for value in (0.1, 0.2, 0.3, 0.4, 0.5):
            arm.command(torch.full((N, 6), value))
            seen.append(float(arm.sent_step[0]))
        self.assertEqual(seen[:2], [0.0, 0.0])                 # not yet due, whatever the policy does
        self.assertAlmostEqual(seen[2], 6 * 0.3 ** 2, places=5)
        self.assertEqual(seen[3:], [0.0, 0.0])

    def test_jitter_costs_far_more_than_a_real_step(self):
        ratio = (6 * math.radians(12.0) ** 2) / (2 * math.radians(4.0) ** 2)
        self.assertGreater(ratio, 20.0)


class TestChatterStatistics(unittest.TestCase):
    def test_back_and_forth_reads_as_reversals_and_a_ramp_does_not(self):
        def reversals(values):
            arm = link(command_hz=(50.0, 50.0), track=True)
            for value in values:
                arm.command(torch.full((N, 6), math.radians(value)), torch.zeros(N, 6))
            return arm.summary()["joints"][0]["reversals"]
        self.assertGreater(reversals([8.0 * (k % 2) for k in range(60)]), 0.9)
        self.assertLess(reversals([4.0 * k for k in range(60)]), 0.05)


class TestPerEnvironmentDraws(unittest.TestCase):
    def test_the_wrist_can_have_its_own_lag(self):
        arm = link(planner=True, servo_lag_s=0.068, wrist_lag_s=0.052)
        self.assertTrue(torch.allclose(arm.servo_lag[:, :3], torch.full((N, 3), 0.068)))
        self.assertTrue(torch.allclose(arm.servo_lag[:, 3:], torch.full((N, 3), 0.052)))

    def test_lag_and_age_are_drawn_in_their_ranges(self):
        arm = link(feedback_hz=9.0, locked=True, planner=True, servo_lag_s=(0.06, 0.10), report_age_s=(0.02, 0.06))
        self.assertTrue(((arm.servo_lag >= 0.06 - 1e-6) & (arm.servo_lag <= 0.10 + 1e-6)).all())
        self.assertGreater(float(arm.servo_lag.std()), 0.005)
        self.assertEqual(set(arm.age.tolist()), {1, 2, 3})


class TestStalls(unittest.TestCase):
    def test_the_share_of_time_matches_the_capture(self):
        arm = link(command_hz=(10.0, 10.0), stalls=True)
        stalled = 0
        steps = 15000                                          # 300 s
        for _ in range(steps):
            arm.command(torch.zeros(N, 6))
            stalled += int(arm.stalled().sum())
        self.assertAlmostEqual(stalled / (steps * N), 8.1 / 38.8, delta=0.02)

    def test_a_stall_drops_commands_and_reports_and_the_arm_finishes_its_move(self):
        arm = link(command_hz=(10.0, 10.0), feedback_hz=9.0, planner=True, stalls=True)
        arm.next_stall[:] = 1e9
        last_move = math.radians(4.0)                          # F-200: ~5 deg through a gap, about one command
        arm.target[:] = last_move
        arm.tracker.command(torch.ones(N, dtype=torch.bool), arm.target)
        arm.stall_until[:] = 10.0
        reports = arm.reported.clone()
        for _ in range(30):                                    # 0.6 s of commands asking for far more
            arm.command(torch.full((N, 6), 2.0))
            for _ in range(4):
                pos, vel = arm.drive(0.005)
            seen, _ = arm.observe(torch.ones(N, 6), torch.zeros(N, 6))
        self.assertTrue(torch.allclose(pos, torch.full_like(pos, last_move)))
        self.assertTrue(torch.equal(vel, torch.zeros_like(vel)))
        self.assertTrue(torch.equal(seen, reports))
        self.assertTrue(torch.allclose(arm.target, torch.full_like(pos, last_move)))

    def test_the_first_command_after_a_stall_is_taken(self):
        arm = link(command_hz=(10.0, 10.0), stalls=True)
        arm.next_stall[:] = 1e9
        arm.stall_until[:] = 0.2
        for _ in range(9):
            arm.command(torch.ones(N, 6))
        self.assertTrue(torch.equal(arm.target, torch.zeros(N, 6)))
        for _ in range(8):
            arm.command(torch.ones(N, 6))
        self.assertTrue(torch.equal(arm.target, torch.ones(N, 6)))

    def test_rarer_stalls_can_be_asked_for(self):
        arm = link(command_hz=(10.0, 10.0), stalls=True, stall_interval_s=(5.0, 7.0))
        stalled, steps = 0, 15000
        for _ in range(steps):
            arm.command(torch.zeros(N, 6))
            stalled += int(arm.stalled().sum())
        self.assertAlmostEqual(stalled / (steps * N), 0.5 / 6.5, delta=0.01)


class TestIsolation(unittest.TestCase):
    def test_the_link_draws_nothing_from_the_global_stream(self):
        torch.manual_seed(5)
        expected = torch.rand(4)
        torch.manual_seed(5)
        arm = link(command_hz=motor_model.D1_COMMAND_HZ_RANGE, feedback_hz=9.0, planner=True, stalls=True)
        for _ in range(100):
            arm.command(torch.zeros(N, 6))
            arm.drive(0.005)
            arm.observe(torch.zeros(N, 6), torch.zeros(N, 6))
        self.assertTrue(torch.equal(torch.rand(4), expected))

    def test_a_reset_touches_only_its_environments(self):
        arm = link(command_hz=(10.0, 10.0), planner=True)
        arm.command(torch.ones(N, 6))
        for _ in range(40):
            arm.drive(0.005)
        moved = arm.tracker.position.clone()
        ids = torch.tensor([0, 3])
        arm.reset(ids, torch.zeros(2, 6))
        self.assertTrue(torch.equal(arm.tracker.position[ids], torch.zeros(2, 6)))
        keep = torch.ones(N, dtype=torch.bool)
        keep[ids] = False
        self.assertTrue(torch.equal(arm.tracker.position[keep], moved[keep]))


class TestSummary(unittest.TestCase):
    def test_step_sizes_are_read_as_the_hardware_findings_read_them(self):
        arm = link(command_hz=(10.0, 10.0), track=True)
        one = math.radians(1.0)
        for k in range(500):
            arm.command(torch.full((N, 6), one * (k // 5)), torch.zeros(N, 6))
        joint = arm.summary()["joints"][0]
        self.assertAlmostEqual(joint["moving_step_deg_median"], 1.05, delta=0.11)
        self.assertEqual(joint["moving_steps_under_smooth"], 1.0)


if __name__ == "__main__":
    unittest.main()
