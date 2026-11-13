"""The D1 as a policy meets it on the dog, for UniFP's task (`--arm_command_hz` and the flags beside it).

UniFP's interface drives the arm like the legs: a 50 Hz joint target into an ideal PD (kp 60 / 40), with exact joint
angles and velocities coming back. None of that is the real arm's:

| what | the policy trained on | the real D1 | source |
| --- | --- | --- | --- |
| command rate | 50 Hz | 9-10 Hz, past 10 Hz it stops responding | Lukas, F-032 |
| what a target means | a spring: `kp x (target - angle)` N·m, so a far target is a torque | a place to go | F-045 |
| speed | whatever the torque gives | 1.20-1.29 rad/s | F-033 |
| between commands | the PD | a trapezoid replanned from rest at each command | F-045 |
| joint angles | exact, 50 Hz | 9 Hz, to 0.1 deg | F-020 |
| joint velocities | exact | not reported: differences of reports | F-020 |
| dropped commands | none | none taken, no reports, for 0.3-0.7 s, unless commands are locked to reports | Lukas, F-200, F-238 |

`ArmLink` puts these between the policy and the joints, each separately, so what each costs can be measured. It holds
what the arm does, in either of two streams: free-running at 9-10 Hz, which drops commands (`--arm_real`), or locked one
command per report, 20 ms after it, which does not (`--arm_locked`: what a runner should do, and `motor_model`'s locked
profile, fitted to that stream and checked by `arm_link_check.py`); `swap_arm` gives the arm the stiff drive that follows it. The legs are untouched and stay at 50 Hz.

Only the actor sees the arm through this. The critic and the rewards keep the true state, as the rest of the
observation's privileged half does.

Its draws (each environment's rates, phases and stalls) come from its own generator, never the global stream that the
goal schedule uses, so a frozen manifest's episodes are the same with the link on or off. Torch only: `tests/test_arm_link.py`
runs it on the CPU.
"""
from __future__ import annotations

import math

import torch

import motor_model
from position_only.core import TrapezoidTracker
from unifp_isaaclab import interface

NUM_ARM = 6
ARM_NAMES = tuple(interface.ISAACLAB_NAMES[12:18])
#: F-199: under this a command, the firmware stops between commands.
SMOOTH_STEP_DEG = 2.5
#: The arm link's lead on the dog (`demos/combiner_door/robot/arm_link.py`): how far ahead of the arm a target may be.
LEAD_DEG = 10.0
#: A joint counts as moving at a report's resolution or more.
MOVING_DEG = motor_model.D1_FEEDBACK_QUANTUM_DEG
_BIN_DEG, _BINS = 0.1, 1200
#: `demos/combiner_door/robot/arm_link.py`'s `MAX_SPEED_DEG_S`, `LEAD_DEG` and `PATH_LEAD_MAX_DEG`, repeated because that module
#: reaches for the arm's client (checked by `tests/test_arm_link.py`).
FOLLOW_SPEED_DEG_S, FOLLOW_LEAD_DEG, FOLLOW_LEAD_MAX_DEG = 45.0, 10.0, 15.0


class ArmLink:
    """The link between a 50 Hz policy and the D1, for `num_envs` environments.

    Call `reset` for each new episode; `command` once a policy step with the policy's six arm targets; `drive` each
    physics step for what the joints follow; `observe` with the true state for what the actor reads. Every feature is
    off unless asked for: with no arguments the link passes everything through.
    """

    def __init__(self, num_envs: int, device, policy_dt: float, *, command_hz=None, feedback_hz: float | None = None,
                 quantum_deg: float = 0.0, planner: bool = False, stalls: bool = False, stall_interval_s=None,
                 locked: bool = False, lock_offset_s: float = motor_model.D1_LOCK_OFFSET_S, report_age_s=0.0,
                 servo_lag_s=0.0, wrist_lag_s=None, skip_probability: float = 0.0, plan: dict | None = None, lower=None,
                 upper=None, seed: int = 0, track: bool = False, command_mean: bool = False, follower: bool = False):
        self.n, self.device, self.policy_dt = num_envs, device, float(policy_dt)
        self.command_hz = None if command_hz is None else (float(min(command_hz)), float(max(command_hz)))
        if self.command_hz is not None and self.command_hz[0] <= 0.0:
            raise ValueError(f"Command rate must be positive, got {command_hz}")
        self.feedback_hz = None if feedback_hz is None else float(feedback_hz)
        self.quantum = math.radians(quantum_deg)
        # Locked: one command a report, a fixed time after it, so the stream is as steady as the arm's own reports.
        self.locked, self.lock_offset = bool(locked), float(lock_offset_s)
        if self.locked and self.feedback_hz is None:
            raise ValueError("Commands locked to reports need reports (feedback_hz).")
        if self.locked:
            self.command_hz = (self.feedback_hz, self.feedback_hz)
        # A report shows the arm as it was this long ago, in policy steps; a pair draws each environment's at reset.
        self.report_age = tuple(report_age_s) if isinstance(report_age_s, (tuple, list)) else (float(report_age_s),) * 2
        self.age_ticks = int(round(max(self.report_age) / self.policy_dt))
        self.ticks = self.observed_tick = 0
        self.skip_probability = float(skip_probability)
        self.stalls = bool(stalls)
        self.stall_interval = tuple(stall_interval_s or motor_model.D1_STALL_INTERVAL_S)
        self.generator = torch.Generator(device=device)
        self.generator.manual_seed(int(seed))
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        self.lower = None if lower is None else torch.as_tensor(lower, dtype=torch.float32, device=device)
        self.upper = None if upper is None else torch.as_tensor(upper, dtype=torch.float32, device=device)

        self.clock = zeros(num_envs)
        self.period = zeros(num_envs)
        self.next_command = zeros(num_envs)
        self.target = zeros(num_envs, NUM_ARM)
        self.next_report = zeros(num_envs)
        self.last_report = zeros(num_envs)
        self.primed = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.reported = zeros(num_envs, NUM_ARM)
        self.rate = zeros(num_envs, NUM_ARM)
        self.next_stall = zeros(num_envs)
        self.stall_until = zeros(num_envs)

        self.history = zeros(self.age_ticks + 1, num_envs, NUM_ARM) if self.age_ticks else None
        self.age = torch.zeros(num_envs, dtype=torch.long, device=device)
        self.rows = torch.arange(num_envs, device=device)

        self.tracker = None
        self.servo_lag_range = tuple(servo_lag_s) if isinstance(servo_lag_s, (tuple, list)) else (float(servo_lag_s),) * 2
        self.servo_lag = zeros(num_envs, NUM_ARM)
        # Joint4-6's own lag, where it is given: the draw has them ~20 ms quicker than the shoulder and elbow (F-250).
        self.wrist_lag_range = None if wrist_lag_s is None else (
            tuple(wrist_lag_s) if isinstance(wrist_lag_s, (tuple, list)) else (float(wrist_lag_s),) * 2)
        self.joint = zeros(num_envs, NUM_ARM)
        self.joint_speed = zeros(num_envs, NUM_ARM)
        if planner:
            plan = plan or motor_model.arm_trajectory("measured")
            limits = [motor_model.D1_VELOCITY_LIMIT_RAD_S[name] for name in ARM_NAMES]
            self.tracker = TrapezoidTracker(num_envs, NUM_ARM, plan["accel_rad_s2"], plan["decel_rad_s2"], limits,
                                            dead_time=plan["dead_time_s"], retention=plan["replan_velocity_retention"],
                                            device=device)
        # Send the mean of the policy's targets since the last command, not the last of them: 9 Hz samples a 50 Hz output.
        self.command_mean = bool(command_mean)
        self.target_sum = zeros(num_envs, NUM_ARM)
        # The dog's `arm_link.Follower` in the stream: what a `follow` goal becomes before it goes on the wire.
        self.follower = bool(follower)
        self.follow_command = zeros(num_envs, NUM_ARM)
        self.follow_waited = 0
        #: rad², this step: the squared change the arm was just sent, zero on a step with no command. `--arm_step_weight`.
        self.sent_step = zeros(num_envs)
        #: The same per joint (n, 6), for a task that prices only some of them (the key hold: J6 follows the key by script).
        self.sent_step_joints = zeros(num_envs, NUM_ARM)
        self.target_count = zeros(num_envs, 1)
        self.track = bool(track)
        if self.track:
            self.last_step = zeros(num_envs, NUM_ARM)
            self.last_q = zeros(num_envs, NUM_ARM)
            self.primed_steps = torch.zeros(num_envs, dtype=torch.bool, device=device)
            self.chatter = {name: torch.zeros(NUM_ARM, dtype=torch.float64, device=device)
                            for name in ("n", "step2", "bend2", "reversals", "moves", "q2")}
            self.step_hist = torch.zeros(NUM_ARM, _BINS, dtype=torch.long, device=device)
            self.gap_hist = torch.zeros(NUM_ARM, _BINS, dtype=torch.long, device=device)
            self.commands = self.clamped = self.env_steps = self.stalled_steps = 0

    def _uniform(self, low, high, count: int) -> torch.Tensor:
        draw = torch.rand(count, device=self.device, generator=self.generator)
        return low + (high - low) * draw

    def _quantised(self, q: torch.Tensor) -> torch.Tensor:
        return torch.round(q / self.quantum) * self.quantum if self.quantum > 0.0 else q

    def stalled(self) -> torch.Tensor:
        return self.clock < self.stall_until

    def reset(self, env_ids, q: torch.Tensor) -> None:
        """A new episode at arm angles `q` (rows for `env_ids`): at rest, each environment's rates and phases drawn."""
        ids = torch.arange(self.n, device=self.device) if env_ids is None else env_ids
        count = len(ids)
        self.clock[ids] = 0.0
        self.target[ids] = q
        if self.command_hz is not None:
            self.period[ids] = self._uniform(1.0 / self.command_hz[1], 1.0 / self.command_hz[0], count)
            self.next_command[ids] = self._uniform(0.0, 1.0, count) * self.period[ids]
        if self.feedback_hz is not None:
            self.next_report[ids] = self._uniform(0.0, 1.0 / self.feedback_hz, count)
        if self.locked:
            # A command goes at the first step past this, half a step later on average: `lock_offset` is the mean.
            self.next_command[ids] = self.next_report[ids] + max(self.lock_offset - 0.5 * self.policy_dt, 0.0)
        if self.history is not None:
            self.history[:, ids] = q
            age = self._uniform(*self.report_age, count) / self.policy_dt
            self.age[ids] = torch.round(age).long().clamp(max=self.age_ticks)
        self.servo_lag[ids] = self._uniform(*self.servo_lag_range, count).unsqueeze(1).expand(-1, NUM_ARM)
        if self.wrist_lag_range is not None:
            self.servo_lag[ids, 3:] = self._uniform(*self.wrist_lag_range, count).unsqueeze(1).expand(-1, NUM_ARM - 3)
        self.last_report[ids] = 0.0
        self.primed[ids] = False
        self.reported[ids] = self._quantised(q)
        self.rate[ids] = 0.0
        self.stall_until[ids] = 0.0
        if self.stalls:
            self.next_stall[ids] = self._uniform(*self.stall_interval, count)
        if self.tracker is not None:
            self.tracker.reset(ids, q)
        self.joint[ids] = q
        self.joint_speed[ids] = 0.0
        self.target_sum[ids] = 0.0
        self.target_count[ids] = 0.0
        self.follow_command[ids] = q
        if self.track:
            self.last_step[ids] = 0.0
            self.last_q[ids] = q
            self.primed_steps[ids] = False

    def _start_stalls(self) -> None:
        begin = self.clock >= self.next_stall
        count = int(begin.sum())
        if count:
            length = self._uniform(*motor_model.D1_STALL_DURATION_S, count)
            self.stall_until[begin] = self.clock[begin] + length
            self.next_stall[begin] = self.stall_until[begin] + self._uniform(*self.stall_interval, count)

    def command(self, target: torch.Tensor, q: torch.Tensor | None = None) -> None:
        """One policy step: the environments with a command due send `target` (n, 6), and the clock moves on.

        A command that falls in a stall is dropped; the first one after it is taken. The arm finishes the move it has
        (F-200) and waits. `q`, the true angles, is only for the statistics.
        """
        self.ticks += 1
        if self.stalls:
            self._start_stalls()
        if self.command_hz is None:
            due = torch.ones(self.n, dtype=torch.bool, device=self.device)
        else:
            due = self.clock >= self.next_command
            self.next_command[due] += self.period[due]
        executed = due & ~self.stalled() if self.stalls or self.skip_probability else due
        asked = target
        if self.command_mean:
            self.target_sum += target
            self.target_count += 1.0
            asked = self.target_sum / self.target_count
            self.target_sum[due] = 0.0
            self.target_count[due] = 0.0
        wanted = asked
        if self.lower is not None:
            # `d1_hardware.set_all_joint_angles` clamps what goes on the wire to the hard limits.
            wanted = torch.minimum(torch.maximum(asked, self.lower), self.upper)
        if self.follower:
            wanted = self._follow(wanted, executed)
        if self.track:
            self._count(executed, asked, wanted, q)
        self.sent_step_joints = torch.where(executed.unsqueeze(-1), (wanted - self.target).square(),
                                            torch.zeros_like(self.sent_step_joints))
        self.sent_step = self.sent_step_joints.sum(dim=1)
        self.target[executed] = wanted[executed]
        if self.tracker is not None:
            self.tracker.command(executed, self.target)
        # After the command, not before: it acts from the start of this step, so it is stamped there. `observe`, after the
        # physics, then reads the end of the step.
        self.clock += self.policy_dt

    def _follow(self, goal: torch.Tensor, executed: torch.Tensor) -> torch.Tensor:
        """`demos/combiner_door/robot/arm_link.py`'s `Follower.step`, per environment, in degrees where it is. At most
        `FOLLOW_SPEED_DEG_S` along the line to the goal, held while that would lead the last report by more than its allowance."""
        delta = torch.rad2deg(goal - self.follow_command)
        dist = delta.abs().amax(dim=1, keepdim=True)
        period = self.period.unsqueeze(1).clamp(min=self.policy_dt)
        reach = FOLLOW_SPEED_DEG_S * period
        want = torch.rad2deg(self.follow_command) + torch.where(dist <= reach, delta, delta * (reach / dist.clamp(min=1e-9)))
        speed = torch.minimum(dist, reach) / period
        age = (self.clock - self.last_report).clamp(min=0.0).unsqueeze(1)
        allow = (FOLLOW_LEAD_DEG + speed * (age + period)).clamp(max=FOLLOW_LEAD_MAX_DEG)
        ok = (want - torch.rad2deg(self.reported)).abs().amax(dim=1, keepdim=True) <= allow
        step = executed.unsqueeze(1) & ok
        self.follow_waited += int((executed.unsqueeze(1) & ~ok).sum())
        self.follow_command = torch.where(step, torch.deg2rad(want), self.follow_command)
        return self.follow_command

    def drive(self, dt: float) -> tuple[torch.Tensor, torch.Tensor]:
        """One physics step: the angles and speeds for the drive to follow, (n, 6) each."""
        if self.tracker is None:
            return self.target, torch.zeros_like(self.target)
        self.tracker.step(dt)
        if self.servo_lag_range[1] <= 0.0:
            return self.tracker.position, self.tracker.velocity
        # The servo behind the plan: first order, exact for a held target over the step.
        keep = torch.exp(-dt / self.servo_lag.clamp(min=1e-6))
        follow = self.tracker.position + keep * (self.joint - self.tracker.position)
        self.joint_speed = (follow - self.joint) / dt
        self.joint = follow
        return self.joint, self.joint_speed

    def observe(self, q: torch.Tensor, dq: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """The arm's angles and speeds as a runner on the dog can know them, from the true `q`, `dq` (n, 6).

        A report is the angle to 0.1 deg; the speed is the difference of the last two reports over the time between
        them, which is all an angle-only feed gives. No report comes during a stall. Idempotent within a step.
        """
        if self.feedback_hz is None:
            return self._quantised(q), dq
        if self.history is not None:
            if self.ticks != self.observed_tick:
                self.observed_tick = self.ticks
                self.history = torch.roll(self.history, 1, dims=0)
                self.history[0] = q
            q = self.history[self.age, self.rows]
        period = 1.0 / self.feedback_hz
        due = self.clock >= self.next_report
        self.next_report[due] += period
        if self.stalls:
            due = due & ~self.stalled()
        if self.skip_probability:
            # A skipped report takes its locked command with it, leaving a gap of two cycles: the ~0.21 s measured.
            draw = torch.rand(self.n, device=self.device, generator=self.generator)
            skip = due & (draw < self.skip_probability)
            self.stall_until[skip] = self.clock[skip] + 0.5 * period
            due = due & ~skip
        sample = self._quantised(q)
        fresh = due & self.primed
        elapsed = (self.clock - self.last_report).clamp(min=self.policy_dt).unsqueeze(1)
        self.rate[fresh] = ((sample - self.reported) / elapsed)[fresh]
        self.reported[due] = sample[due]
        self.last_report[due] = self.clock[due]
        self.primed |= due
        return self.reported, self.rate

    def _count(self, executed, target, wanted, q) -> None:
        self.env_steps += self.n
        if self.stalls:
            self.stalled_steps += int(self.stalled().sum())
        rows = executed.nonzero().flatten()
        if len(rows) == 0:
            return
        self.commands += len(rows)
        self.clamped += int(((target[rows] - wanted[rows]).abs() > 1e-6).any(dim=1).sum())
        signed = torch.rad2deg(wanted[rows] - self.target[rows])
        step = signed.abs()
        self._bin(self.step_hist, step)
        # Is it going somewhere, or back and forth? A step against the one before it, and the arm's own motion per command.
        seen = self.primed_steps[rows].unsqueeze(1).float()
        before = self.last_step[rows]
        real = (step >= 0.5) & (before.abs() >= 0.5)
        chatter = self.chatter
        chatter["n"] += seen.sum(dim=0).double()
        chatter["step2"] += (seen * signed ** 2).sum(dim=0).double()
        chatter["bend2"] += (seen * (signed - before) ** 2).sum(dim=0).double()
        chatter["moves"] += (seen * real.float()).sum(dim=0).double()
        chatter["reversals"] += (seen * (real & (signed * before < 0)).float()).sum(dim=0).double()
        self.last_step[rows] = signed
        self.primed_steps[rows] = True
        if q is not None:
            chatter["q2"] += (seen * torch.rad2deg(q[rows] - self.last_q[rows]) ** 2).sum(dim=0).double()
            self.last_q[rows] = q[rows]
        if q is not None:
            self._bin(self.gap_hist, torch.rad2deg((wanted[rows] - q[rows]).abs()))

    @staticmethod
    def _bin(hist: torch.Tensor, degrees: torch.Tensor) -> None:
        index = (degrees / _BIN_DEG).long().clamp(max=_BINS - 1)
        for joint in range(NUM_ARM):
            hist[joint].index_add_(0, index[:, joint], torch.ones_like(index[:, joint]))

    def summary(self) -> dict:
        """The commands the arm would have received, in the terms the hardware findings use (`track=True`)."""
        if not self.track:
            return {}
        edges = torch.arange(_BINS, device=self.device, dtype=torch.float32) * _BIN_DEG + _BIN_DEG / 2

        def median(row):
            total = int(row.sum())
            return None if total == 0 else round(float(edges[int((row.cumsum(0) >= (total + 1) // 2).nonzero()[0])]), 2)

        def quantile(row, share):
            total = int(row.sum())
            return None if total == 0 else round(float(edges[int((row.cumsum(0) >= share * total).nonzero()[0])]), 1)

        moving_from = int(MOVING_DEG / _BIN_DEG)
        smooth_from = int(SMOOTH_STEP_DEG / _BIN_DEG)
        lead_from = int(LEAD_DEG / _BIN_DEG)
        per_joint = []
        for j, name in enumerate(ARM_NAMES):
            moving = self.step_hist[j, moving_from:]
            total = int(moving.sum())
            gaps = self.gap_hist[j]
            per_joint.append({
                "joint": name,
                "step_deg_median": median(self.step_hist[j]),
                "moving_steps": total,
                "moving_step_deg_median": median(moving) if total else None,
                "moving_steps_under_smooth": round(int(self.step_hist[j, moving_from:smooth_from].sum()) / max(total, 1), 4),
                "gap_deg_median": median(gaps),
                "gap_deg_p99": quantile(gaps, 0.99), "gap_deg_max": quantile(gaps, 1.0),
                "step_deg_p99": quantile(self.step_hist[j], 0.99), "step_deg_max": quantile(self.step_hist[j], 1.0),
                "gap_past_lead": round(int(gaps[lead_from:].sum()) / max(int(gaps.sum()), 1), 4),
            })
        c = {key: value.tolist() for key, value in self.chatter.items()}
        for j, row in enumerate(per_joint):
            n = max(c["n"][j], 1.0)
            row["command_step_rms_deg"] = round((c["step2"][j] / n) ** 0.5, 3)
            row["command_bend_rms_deg"] = round((c["bend2"][j] / n) ** 0.5, 3)
            row["reversals"] = round(c["reversals"][j] / max(c["moves"][j], 1.0), 4)
            row["arm_step_rms_deg"] = round((c["q2"][j] / n) ** 0.5, 3)
        return {
            "commands": self.commands,
            "commands_per_s": round(self.commands / max(self.env_steps * self.policy_dt, 1e-9), 3),
            "clamped_at_limits": round(self.clamped / max(self.commands, 1), 4),
            "follow_waited": round(self.follow_waited / max(self.commands, 1), 4) if self.follower else None,
            "stalled_time": round(self.stalled_steps / max(self.env_steps, 1), 4),
            "smooth_step_deg": SMOOTH_STEP_DEG, "lead_deg": LEAD_DEG,
            "joints": per_joint,
        }


def describe(cfg: dict | None, servo: bool) -> dict | None:
    """The link for the run record."""
    if cfg is None and not servo:
        return None
    cfg = cfg or {}
    plan = motor_model.arm_trajectory("measured") if cfg.get("planner") else None
    stalls = bool(cfg.get("stalls"))
    if cfg.get("plan"):
        plan = cfg["plan"]
    return {"command_hz": cfg.get("command_hz"), "feedback_hz": cfg.get("feedback_hz"),
            "quantum_deg": cfg.get("quantum_deg", 0.0), "planner": plan, "stalls": stalls,
            "servo_lag_s": cfg.get("servo_lag_s", 0.0), "wrist_lag_s": cfg.get("wrist_lag_s"),
            "command_mean": bool(cfg.get("command_mean")), "follower": bool(cfg.get("follower")),
            "skip_probability": cfg.get("skip_probability", 0.0),
            "stall_interval_s": (cfg.get("stall_interval_s") or motor_model.D1_STALL_INTERVAL_S) if stalls else None,
            "stall_duration_s": motor_model.D1_STALL_DURATION_S if stalls else None,
            "locked": bool(cfg.get("locked")), "lock_offset_s": cfg.get("lock_offset_s"),
            "report_age_s": cfg.get("report_age_s", 0.0), "servo": servo}


def swap_arm(robot_cfg):
    """Give the arm of `robot_cfg` the stiff position drive of `--arm_actuator d1_servo` (Week 1, F-010), in place.

    A real D1 servo closes its own loop and holds the angle, where UniFP's 60 / 40 N·m/rad is a spring a policy can lean
    on. Implicit, so 4000 N·m/rad integrates at 200 Hz; the published torques and the measured speed ceilings still
    limit it. The jaws and whatever legs `swap_legs` left keep their groups. Import only after Isaac Sim has started.
    """
    from isaaclab.actuators import ImplicitActuatorCfg

    from flat_env_cfg import ARM_BASE_DAMPING, ARM_BASE_STIFFNESS

    group = robot_cfg.actuators["unifp"]
    only = lambda value, names: {k: v for k, v in value.items() if k in names} if isinstance(value, dict) else value
    rest = [name for name in group.joint_names_expr if name not in ARM_NAMES]
    arm = ImplicitActuatorCfg(
        joint_names_expr=list(ARM_NAMES), stiffness=ARM_BASE_STIFFNESS, damping=ARM_BASE_DAMPING,
        armature=only(group.armature, ARM_NAMES), friction=only(group.friction, ARM_NAMES),
        effort_limit_sim={name: motor_model.D1_EFFORT_LIMIT_NM[name] for name in ARM_NAMES},
        velocity_limit_sim={name: motor_model.D1_VELOCITY_LIMIT_RAD_S[name] for name in ARM_NAMES})
    keep = group.replace(joint_names_expr=rest, stiffness=only(group.stiffness, rest), damping=only(group.damping, rest),
                         armature=only(group.armature, rest), friction=only(group.friction, rest),
                         effort_limit=only(group.effort_limit, rest), effort_limit_sim=only(group.effort_limit_sim, rest))
    robot_cfg.actuators = {**robot_cfg.actuators, "unifp": keep, "d1_servo": arm}
    return robot_cfg
