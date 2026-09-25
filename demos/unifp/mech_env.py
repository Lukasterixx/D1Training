"""The standing combiner demo, run the way the mechanism-trained policy was trained to be run.

`UniFPDemoEnv` drives UniFP's checkpoint through the demo script: a goal that follows the lever and
the door along their commanded arcs, and a friction grip on the bar. The mechanism policy
(F-108) was trained differently, and running it the old way wastes it. So this subclass adds:

  * **A claw** (`mech.ClawCoupling`). When the script leaves its grip phase, if the jaw centre is
    within `claw_capture_m` of the lever bar, the claw hooks it there: a spring-damper from the jaw
    centre to that point on the lever, applied to both bodies at the physics rate, torn out above
    `claw_grip_n`. Off by default; the old controller can have it too (`--claw`), which is what makes
    the comparison about the policy rather than the tool.
  * **The task layer's force law** (`mech.PlaneForceLaw`), while the claw holds the lever: the force
    command is the PI law on how far the handle lags the script's reference, projected onto the joint
    the phase drives (`LAW_JOINTS`: the lever to turn it, the door to open it); along that joint the goal
    is put on the handle, as training put it, and in every other direction it is the script's
    reference -- hybrid position/force control, so the joint not under force is held by position. The box's joint angles stand in for what tags on the door and the
    lever would report (the plan's B perception tracks moving parts); the script itself still never
    reads them.
  * **A roll command** that puts the jaws across the bar (`mech.jaw_roll`), in place of the wrist
    servo: the mechanism policy was trained with the roll objective and tracks it itself.
  * **A goal correction** (`goal_correction`): integral action on the jaw centre's position during the
    reach's holds. The mechanism policy arrives with a steady offset -- 5 cm across the approach on the
    first run, varying by 1-2 mm over a hold's last second -- because its training grasped wherever the
    tool had settled and never paid for precision. The task layer knows where the jaw centre is from
    the robot's own joint angles, and slides the commanded goal by what it is missing.

The robot, gains, control law and goal frame are the trained ones, as for `UniFPDemoEnv`. What the
launcher must also set for this policy (`run_demo.py --mech`) is recorded in `run.json`: the jaw-centre
tool point it was trained on, the arm's 0.01 kg·m² armature (F-102), and the roll objective.
"""
from __future__ import annotations

import math

import torch

from isaaclab.utils import configclass

from unifp_isaaclab import interface

from demos.combiner.geometry import GEOMETRY
from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6

from . import mech, wrist
from .env import UniFPDemoEnv, UniFPDemoEnvCfg

#: Phases in which the claw holds the lever and the force law acts; the claw lets go on "release".
CLAW_PHASES = ("turn", "crack", "ease", "pull")
RELEASE_PHASE = "release"
#: Which of the box's joints the force law drives in each claw phase: 0 the lever, 1 the door.
#:
#: Both joints in every phase (the script's reference as it stands) opened 15 of 16, but ease and pull --
#: which move the lever back up while the door opens -- wound the integral to the 80 N cap on a 0.4 N·m
#: lever and the claw spiked to 106-133 N. The door alone after the turn opened 2 of 16: on its own the
#: turn gets the lever only to ~41 degrees, short of the 45 degree release, and crack was what pushed it
#: past. So: the lever in turn, lever and door in crack (the lever held past the release while the door
#: starts), the door alone after that -- the latch only catches with the door shut *and* the lever under
#: 10 degrees, and a claw does not need the lever held once the door is moving.
LAW_JOINTS = {"turn": (0,), "crack": (0, 1), "ease": (1,), "pull": (1,)}
#: Where the goal goes in each claw phase. "handle": on the handle along the joints under force, the
#: reference elsewhere (hybrid, as training put it). "reference": on the script's reference, with the
#: law's force added -- position-led, as the old demo turns the lever. The turn is position-led: led by
#: force alone, the policy pushes the lever down with 6-10 N whatever it is commanded and the lever
#: stalls near 40 degrees, short of the latch's 45 (the 4 s turn to the stop, 2026-09-25).
LAW_GOAL = {"turn": "reference", "crack": "handle", "ease": "handle", "pull": "handle"}
#: Holds in which the goal correction integrates, and the moving phase between them that keeps its offset.
CORRECTION_PHASES = ("settle", "arrive", "grip")
CORRECTION_HELD_PHASES = ("enter",)


@configclass
class MechDemoEnvCfg(UniFPDemoEnvCfg):
    """The demo's configuration with the claw, the force law and the roll command available."""

    claw: bool = False
    #: The jaw centre must be this close to the lever bar's axis for the claw to hook it.
    claw_capture_m: float = 0.025
    #: The training's evaluation grasp (`unifp_train.mech_cfg.GRASP_STIFFNESS`, `GRASP_DAMPING`, `GRIP_N`).
    claw_stiffness: float = 2000.0
    claw_damping: float = 10.0
    claw_grip_n: float = 150.0
    force_law: bool = False
    #: `unifp_train.mech_cfg.FORCE_LAW_*`: 10 N at 2.5 cm of lag, the integral ~30 N/s per 3 cm, 80 N cap;
    #: the integral bleeds with a 1 s time constant once the handle has arrived (F-108 note).
    force_law_gains: tuple[float, float, float] = (400.0, 1000.0, 80.0)
    force_law_bleed_s: float | None = 1.0
    #: Command the jaws' roll across the bar (needs `roll_objective`, which the mechanism policy has).
    roll_command: bool = False
    #: While the latch still holds (the lever short of the release, the door shut), the law's force on the
    #: door is capped here: a task layer that can see the lever should not yank a latched door. Without it
    #: the failed turns wound the door command to 80 N and the claw spiked past 100 N.
    latched_door_cap_n: float = 15.0
    #: Hold the roll where it was when the claw hooked, as training held it. Off: the pads still squeeze the
    #: bar, so they turn with it, and a held roll clamps the lever -- 9 of 16 opened with it on, the lever
    #: at 15 degrees at the end of the turn against 42 without (2026-09-25 runs).
    hold_roll_when_hooked: bool = False
    #: Integral correction of the commanded goal by the jaw centre's measured miss, 1/s, during the
    #: phases in `CORRECTION_PHASES`; the offset is held (not integrated) through the phases between
    #: them and dropped once the claw holds the lever. Capped at `goal_correction_max_m`.
    goal_correction: bool = False
    goal_correction_gain: float = 1.5
    goal_correction_max_m: float = 0.10


class MechDemoEnv(UniFPDemoEnv):
    cfg: MechDemoEnvCfg

    def __init__(self, cfg: MechDemoEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, device = self.num_envs, self.device
        if cfg.task != "combiner" and (cfg.claw or cfg.force_law):
            raise ValueError("the claw and the force law are for the combiner task")
        self._handle_body = self._box.body_names.index("Handle") if cfg.task == "combiner" else None
        self._handle_body_ids = (torch.tensor([self._handle_body], dtype=torch.int32, device=device)
                                 if self._handle_body is not None else None)
        self._link6_ids = torch.tensor([self._link6], dtype=torch.int32, device=device)
        self._jaw_offset = torch.as_tensor(JAW_CENTRE_LINK6, device=device).expand(n, 1, 3).contiguous()
        self.claw = mech.ClawCoupling(n, device, stiffness=cfg.claw_stiffness, damping=cfg.claw_damping,
                                      grip_n=cfg.claw_grip_n)
        kp, ki, cap = cfg.force_law_gains
        self.law = mech.PlaneForceLaw(n, device, kp=kp, ki=ki, max_n=cap, bleed_s=cfg.force_law_bleed_s)
        #: Per attempt: whether the claw tried to hook the bar, how far the jaw centre was from it
        #: then, and whether it hooked. NaN distance until it tries.
        self.claw_tried = torch.zeros(n, dtype=torch.bool, device=device)
        self.claw_capture = torch.full((n,), float("nan"), device=device)
        self.claw_hooked = torch.zeros(n, dtype=torch.bool, device=device)
        self._wrench_on = False
        self._lag_w = torch.zeros(n, 3, device=device)
        self._held_roll = torch.zeros(n, device=device)
        self._roll_held = torch.zeros(n, dtype=torch.bool, device=device)
        #: The goal correction's offset, world frame, m.
        self.goal_offset = torch.zeros(n, 3, device=device)

    # --- the lever, as a body --------------------------------------------------------------------

    def _handle_pose(self):
        pos = self._box.data.body_pos_w[:, self._handle_body]
        quat = self._box.data.body_quat_w[:, self._handle_body]
        return pos, quat

    def _anchor_world(self) -> tuple[torch.Tensor, torch.Tensor]:
        """The claw's anchor on the lever and its velocity, world frame."""
        pos, quat = self._handle_pose()
        arm = interface.quat_apply(quat, self.claw.anchor_local)
        lin = self._box.data.body_lin_vel_w[:, self._handle_body]
        ang = self._box.data.body_ang_vel_w[:, self._handle_body]
        return pos + arm, lin + torch.cross(ang, arm, dim=-1)

    def _jaw_centre_state(self) -> tuple[torch.Tensor, torch.Tensor]:
        """The jaw centre and its velocity, world frame (Link6 plus the jaw-centre offset)."""
        pos = self._robot.data.body_pos_w[:, self._link6]
        quat = self._robot.data.body_quat_w[:, self._link6]
        arm = interface.quat_apply(quat, self._jaw_offset[:, 0])
        lin = self._robot.data.body_lin_vel_w[:, self._link6]
        ang = self._robot.data.body_ang_vel_w[:, self._link6]
        return pos + arm, lin + torch.cross(ang, arm, dim=-1)

    def _bar_distance(self, ids: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """(distance from the jaw centre to the lever bar, the jaw centre in the lever's frame).

        The bar is the Lever cylinder of `demos/combiner/asset.py`: in the lever body's frame it runs
        from the spindle along -y for `handle_length`, at x = `handle_projection`, z = 0.
        """
        centre, _ = self._jaw_centre_state()
        pos, quat = self._handle_pose()
        local = interface.quat_rotate_inverse(quat[ids], centre[ids] - pos[ids])
        along = torch.clamp(local[:, 1], min=-GEOMETRY.handle_length, max=0.0)
        dx = local[:, 0] - GEOMETRY.handle_projection
        dy = local[:, 1] - along
        return torch.sqrt(dx * dx + dy * dy + local[:, 2] ** 2), local

    # --- physics rate: the claw -------------------------------------------------------------------

    def _apply_action(self) -> None:
        super()._apply_action()
        if not self.cfg.claw:
            return
        if not bool(self.claw.engaged.any()) and not self._wrench_on:
            return
        centre, centre_vel = self._jaw_centre_state()
        anchor, anchor_vel = self._anchor_world()
        force = self.claw.force(centre, centre_vel, anchor, anchor_vel)
        self._ee_force_w = force
        hand_quat = self._robot.data.body_quat_w[:, self._link6]
        self._robot.permanent_wrench_composer.set_forces_and_torques(
            forces=interface.quat_rotate_inverse(hand_quat, force).unsqueeze(1).contiguous(),
            positions=self._jaw_offset, body_ids=self._link6_ids)
        _, handle_quat = self._handle_pose()
        self._box.permanent_wrench_composer.set_forces_and_torques(
            forces=interface.quat_rotate_inverse(handle_quat, -force).unsqueeze(1).contiguous(),
            positions=self.claw.anchor_local.unsqueeze(1).contiguous(), body_ids=self._handle_body_ids)
        # Once everything has let go, one more pass writes the zeros, then this stops running.
        self._wrench_on = bool(self.claw.engaged.any())

    # --- policy rate: hook, law, goal, roll -------------------------------------------------------

    def _scripted_goal(self) -> torch.Tensor:
        sphere = super()._scripted_goal()
        phases = [c.phase if c is not None else "" for c in self.commands]
        if self.cfg.goal_correction:
            sphere = self._corrected_goal(sphere, phases)
        if self.cfg.claw:
            self._hook_and_release(phases)
        if self.cfg.force_law:
            sphere = self._law_goal(sphere, phases)
        if self.cfg.roll_command and self.cfg.roll_objective:
            self._command_roll()
        return sphere

    def _script_target_world(self) -> torch.Tensor:
        """Where the script wants the jaw centre this step, world frame."""
        wanted = torch.as_tensor([c.point if c is not None else (0.0, 0.0, 0.0) for c in self.commands],
                                 dtype=torch.float32, device=self.device)
        return self.scene.env_origins + self._spawn_pos + interface.quat_apply(self._spawn_yaw_quat, wanted)

    def _corrected_goal(self, sphere: torch.Tensor, phases: list[str]) -> torch.Tensor:
        """Slide the goal by the jaw centre's steady miss during the reach's holds (`goal_correction`)."""
        integrate = torch.tensor([p in CORRECTION_PHASES for p in phases], device=self.device)
        keep = integrate | torch.tensor([p in CORRECTION_HELD_PHASES for p in phases], device=self.device)
        target = self._script_target_world()
        centre, _ = self._jaw_centre_state()
        step = self.cfg.goal_correction_gain * interface.POLICY_DT * (target - centre)
        offset = torch.where(integrate.unsqueeze(-1), self.goal_offset + step, self.goal_offset)
        norm = offset.norm(dim=-1, keepdim=True)
        offset = offset * (self.cfg.goal_correction_max_m / norm.clamp(min=1e-9)).clamp(max=1.0)
        self.goal_offset = torch.where(keep.unsqueeze(-1), offset, torch.zeros_like(offset))
        if not bool(keep.any()):
            return sphere
        goal = self._tip_goal_from_jaw_centre(target + self.goal_offset)
        local = interface.quat_rotate_inverse(self._base_yaw_quat(), goal - (self._goal_centre() + self.scene.env_origins))
        corrected = torch.where(keep.unsqueeze(-1), interface.cart2sphere(local), sphere)
        self._goals.current = corrected
        self._goals.start = corrected
        self._goals.goal = corrected
        return corrected

    def _hook_and_release(self, phases: list[str]) -> None:
        due = torch.tensor([p in CLAW_PHASES for p in phases], device=self.device) & ~self.claw_tried
        if bool(due.any()):
            ids = due.nonzero(as_tuple=False).flatten()
            distance, local = self._bar_distance(ids)
            self.claw_tried[ids] = True
            self.claw_capture[ids] = distance
            hooked = distance < self.cfg.claw_capture_m
            if bool(hooked.any()):
                self.claw.engage(ids[hooked], local[hooked])
                self.claw_hooked[ids[hooked]] = True
                self._wrench_on = True
        leaving = torch.tensor([p == RELEASE_PHASE or p == "back" or p == "done" for p in phases],
                               device=self.device) & self.claw.engaged
        if bool(leaving.any()):
            self.claw.release(leaving.nonzero(as_tuple=False).flatten())

    def _jacobian(self, index: int, handle_deg: float, door_deg: float) -> torch.Tensor:
        """(3, 2): how the grasp point moves per degree of lever and of door, world directions."""
        site = self.sites[index]
        step = 1.0
        d_lever = [(a - b) / (2 * step) for a, b in zip(site.grasp_point_m(handle_deg + step, door_deg),
                                                         site.grasp_point_m(handle_deg - step, door_deg))]
        d_door = [(a - b) / (2 * step) for a, b in zip(site.grasp_point_m(handle_deg, door_deg + step),
                                                        site.grasp_point_m(handle_deg, max(0.0, door_deg - step)))]
        local = torch.tensor([d_lever, d_door], dtype=torch.float32, device=self.device)   # (2, 3), spawn yaw frame
        world = interface.quat_apply(self._spawn_yaw_quat[index].expand(2, 4), local)
        return world.T

    def _law_goal(self, sphere: torch.Tensor, phases: list[str]) -> torch.Tensor:
        """For environments whose claw holds the lever: goal on the handle, force from the law."""
        active = self.claw.engaged & torch.tensor([p in CLAW_PHASES for p in phases], device=self.device)
        force_cmd = torch.zeros(self.num_envs, 3, device=self.device)
        if bool(active.any()):
            anchor, _ = self._anchor_world()
            reference = self._script_target_world()
            truth = self.box_state()
            jac = torch.zeros(self.num_envs, 3, 2, device=self.device)
            for index in active.nonzero(as_tuple=False).flatten().tolist():
                full = self._jacobian(index, float(truth["handle_deg"][index]), float(truth["door_deg"][index]))
                mask = torch.zeros(2, device=self.device)
                mask[list(LAW_JOINTS[phases[index]])] = 1.0
                # A dropped joint's column is zeroed; `project_onto` then projects onto the other alone.
                jac[index] = full * mask
            lag = mech.project_onto(reference - anchor, jac) * active.unsqueeze(-1)
            latched = ((truth["handle_deg"] < GEOMETRY.handle_release_deg)
                       & (truth["door_deg"] <= 1.0) & active)
            # The integral lives in this phase's force directions only: an integral built pushing the
            # lever down in crack must not keep pushing it once the door alone is under force.
            self.law.integral = mech.project_onto(self.law.integral, jac)
            force_cmd = self.law.step(lag, active, interface.POLICY_DT)
            if bool(latched.any()):
                # The door's share of the command, capped while the latch holds; the lever's share is not.
                door_dir = torch.zeros(self.num_envs, 3, 1, device=self.device)
                for index in latched.nonzero(as_tuple=False).flatten().tolist():
                    door_dir[index, :, 0] = mech.unit(self._jacobian(index, float(truth["handle_deg"][index]),
                                                                     float(truth["door_deg"][index]))[:, 1])
                door_part = mech.project_onto(force_cmd, door_dir)
                over = (door_part.norm(dim=-1, keepdim=True) - self.cfg.latched_door_cap_n).clamp(min=0.0)
                trimmed = force_cmd - door_part * (over / door_part.norm(dim=-1, keepdim=True).clamp(min=1e-9))
                force_cmd = torch.where(latched.unsqueeze(-1), trimmed, force_cmd)
                self.law.command = force_cmd
            self._lag_w = lag
            # Hybrid position/force: along the joints under force the goal is on the handle, as the
            # law-trained policy was trained; in every other direction it is the script's reference,
            # so a joint the law is not driving (the lever while the door opens) is held by position
            # rather than left to drift with the hand. The tool point is the jaw centre, so the goal is
            # a point for it directly.
            by_reference = torch.tensor([LAW_GOAL.get(p) == "reference" for p in phases], device=self.device)
            goal_w = torch.where(by_reference.unsqueeze(-1), reference, reference - lag)
            centre = self._goal_centre() + self.scene.env_origins
            local = interface.quat_rotate_inverse(self._base_yaw_quat(), goal_w - centre)
            held = interface.cart2sphere(local)
            sphere = torch.where(active.unsqueeze(-1), held, sphere)
            self._goals.current = sphere
            self._goals.start = sphere
            self._goals.goal = sphere
        else:
            self.law.command.zero_()
        self.law.integral = torch.where(active.unsqueeze(-1), self.law.integral, torch.zeros_like(self.law.integral))
        self._commands[:, interface.CMD_EE_FORCE] = interface.quat_rotate_inverse(self._base_yaw_quat(), force_cmd)
        return sphere

    def _command_roll(self) -> None:
        """Roll the jaws across the lever bar: the axis across both the hand's approach and the bar.

        Until the claw hooks the bar. After that the roll is held where it was when it hooked, as the
        mechanism training held it (`hold_roll`): a claw is a ball joint, so turning the jaws with the
        lever buys nothing, and a friction grip's 45 degrees of wrist roll through the turn is wrist
        motion competing with the push.
        """
        _, approach, _ = self._hand_frame()
        bar = torch.as_tensor([c.object_axis if c is not None else (0.0, 0.0, 1.0) for c in self.commands],
                              dtype=torch.float32, device=self.device)
        bar = wrist.unit(interface.quat_apply(self._spawn_yaw_quat, bar))
        jaw = wrist.desired_jaw_axis(approach, bar)
        forward = interface.quat_apply(self._base_yaw_quat(), torch.tensor(
            [1.0, 0.0, 0.0], device=self.device).expand(self.num_envs, 3))
        roll = mech.jaw_roll(approach, jaw, forward)
        if self.cfg.claw and self.cfg.hold_roll_when_hooked:
            fresh = self.claw.engaged & ~self._roll_held
            self._held_roll = torch.where(fresh, self._goals.current_roll, self._held_roll)
            self._roll_held |= fresh
            roll = torch.where(self.claw.engaged, self._held_roll, roll)
        self._goals.current_roll = roll

    # --- resetting ---------------------------------------------------------------------------------

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            ids = torch.arange(self.num_envs, device=self.device)
        else:
            ids = env_ids
        self.claw.reset(ids)
        self.law.reset(ids)
        self.claw_tried[ids] = False
        self.claw_hooked[ids] = False
        self.claw_capture[ids] = float("nan")
        self.goal_offset[ids] = 0.0
        self._roll_held[ids] = False
        super()._reset_idx(env_ids)
        if self.cfg.claw and self._handle_body_ids is not None:
            zeros = torch.zeros(self.num_envs, 1, 3, device=self.device)
            self._box.permanent_wrench_composer.set_forces_and_torques(
                forces=zeros, positions=zeros, body_ids=self._handle_body_ids)
            self._robot.permanent_wrench_composer.set_forces_and_torques(
                forces=zeros, positions=self._jaw_offset, body_ids=self._link6_ids)

    def mech_state(self) -> dict:
        """What the recorder reads: claw, spring force, and the law's command, per environment."""
        return {"claw_engaged": self.claw.engaged.float(), "claw_force_n": self.claw.spring_n,
                "force_cmd_n": self.law.command.norm(dim=-1)}
