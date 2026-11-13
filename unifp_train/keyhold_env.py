"""The arm holds the key in the lock while the front-right foot turns it: one policy for the legs and the arm.

Lukas's request of 2026-11-10: the key drawn from its holster and inserted by the classical arm (`demos/combiner_key`),
then, with the arm still holding it, the front-right foot turns it; the box raised so the lever is slightly above head
height; the policy takes over from the robot standing with the arm out. The arm has to keep the hand where the key is
while the body moves under it to lift the foot.

This is the key task (`leg_env.Go2D1LegEnv`, `leg_object="key"`) with these changes:

  1. **The arm is the policy's too** (18 actions). Its six are a residual on the joint pose that put the jaws on the key
     at the episode's start (`q_ref`, solved by `arm_kin` at reset): target = q_ref + 0.25 a. With `hold_arm="fixed"`
     the residual is not used (the position-only hold of the arm's joints: Lukas's baseline); with "ik" the reference is
     re-solved at 10 Hz from the base's true pose (a classical hold), the residual again unused.
  2. **The wrist rolls with the key.** The grip is the dog's (`holster.GRIP_ALONG_M` 0, roll +1): the key's shaft on the
     jaws' axis, so the turn is Joint6 alone. Joint6's target follows the lock's angle (`KEYHOLD_J6_SIGN`), not the
     policy; on the robot that is the foot's measured turn or a compliant wrist. Not validated either way.
  3. **The hand is tied to the key** by the demo's grip spring (`demos/combiner_key/env.py`: 800 N/m, here with 8 N·s/m
     and no rotational part), from the jaw centre to the point on the shaft's axis where they closed. Its force is put on
     Link6 at the jaw centre. The key lets go (episode over, `key_lost`) past `KEYHOLD_LOST_M` (20 N).
  4. **The policy sees the hand's miss**: the grip point minus the jaw centre, in the base's frame, in the observation's
     orientation channels (`interface.CMD_EE_ORN_*`, zero in every policy so far), x `KEYHOLD_OBS_SCALE`.
  5. **The box is higher and further**, so the arm reaches the key over the head (`leg_cfg.KEYHOLD_*`): the CPU model has
     the hold at the settled stance only with the lock's face 0.46-0.52 m ahead and 0.12-0.20 m right for a lock 0.47 m
     up, Joint5 near its soft limit (scan of 2026-11-10 in the week 9 log).

Plain additions to the leg task; nothing of it changes when `hold_arm` is unused.
"""
from __future__ import annotations

import math

import torch

from isaaclab.utils import configclass

from unifp_isaaclab import interface

from . import arm_kin, leg_cfg, observations, task_cfg
from .keyhold import ArmGravity, bar_at, follow_turn, grip_point, hand_rotation, quat_matrix, turn_ahead
from .leg_env import Go2D1LegEnv, Go2D1LegEnvCfg

HOLD_ARMS = ("policy", "fixed", "ik")


@configclass
class Go2D1KeyHoldEnvCfg(Go2D1LegEnvCfg):
    action_space = 18
    leg_object: str = "key"
    arm_pose: str = "rest"
    settled_spawn: bool = True
    handle_fraction: float = 1.0
    #: "policy" (the residual on `q_ref`), "fixed" (q_ref held), "ik" (re-solved at 10 Hz from the true base pose).
    hold_arm: str = "policy"
    #: The jaws' spring to the key, N/m and N·s/m.
    grip_k: float = leg_cfg.KEYHOLD_GRIP_K_N_PER_M
    grip_c: float = leg_cfg.KEYHOLD_GRIP_C_N_S_PER_M
    #: The arm's commands carry the sag its PD leaves under gravity (with the wrist RealSense, `--payload physical`), as
    #: the insertion's outer loop leaves the command above the key on the robot (F-222: 8.5-15 mm on the dog). Without
    #: it the fixed hold pulled 20 N on the key from the start (2026-11-10).
    #: Training: the key is lost only past `leg_cfg.KEYHOLD_LOST_EARLY_M` until `common_step_counter` passes this
    #: (counted from the resume's own start, as `force_start_step` is not).
    hold_loss_start_step: int = 0
    sag_compensation: bool = True
    wrist_payload: bool = True
    #: `--key_stance_sink_weight` (per second): how far the base sinks below its start, past `leg_cfg.KEYHOLD_SINK_FREE_M`,
    #: squared. Rising is not priced. 0: not used.
    stance_sink_weight: float = 0.0
    #: `--key_support_target_weight` (per second): the support legs' targets past `leg_cfg.KEYHOLD_SUPPORT_FREE_RAD`,
    #: squared and summed. 0: not used.
    support_target_weight: float = 0.0
    #: `--key_turn_follow_weight` (per second, negative): the turn follows the task layer's arc (F-283). The key's run
    #: ahead of the reference, past `leg_cfg.KEYHOLD_TURN_AHEAD_FREE_RAD`, is priced squared (rad) while the layer
    #: presses or holds; its progress is paid only up to the reference, and its open latch only once the reference
    #: reaches it (`keyhold.follow_turn`). 0: not used, the turn rewarded as before.
    turn_follow_weight: float = 0.0


class Go2D1KeyHoldEnv(Go2D1LegEnv):
    cfg: Go2D1KeyHoldEnvCfg

    def __init__(self, cfg: Go2D1KeyHoldEnvCfg, render_mode: str | None = None, **kwargs):
        if cfg.hold_arm not in HOLD_ARMS:
            raise ValueError(f"hold_arm must be one of {HOLD_ARMS}, not {cfg.hold_arm!r}")
        if cfg.leg_object != "key":
            raise ValueError("the key hold is the key task's")
        super().__init__(cfg, render_mode, **kwargs)
        n, device = self.num_envs, self.device
        # The IK and the gravity model run on the CPU: a few dozen tiny batches a step are launch-bound on the GPU (the
        # six-seed solve took 775 ms a call beside two trainings, 2026-11-10; on the CPU 55 ms, one seed 7 ms).
        self._kin = arm_kin.ArmKinematics("cpu", torch.float32)
        self._kin_seeds = torch.as_tensor(leg_cfg.KEYHOLD_IK_SEEDS, dtype=torch.float32)
        self._kin_nominal = torch.as_tensor(leg_cfg.KEYHOLD_IK_NOMINAL, dtype=torch.float32)
        self._link6 = self._robot.body_names.index("Link6")
        self._jaw_offset = torch.as_tensor(arm_kin.JAW_CENTRE_LINK6_M, device=device).expand(n, 3).contiguous()
        # The wrench goes on four bodies now: the foot, the knee, the base (the leg task's) and Link6 (the grip).
        self._wrench_bodies = torch.tensor([self._foot_body, self._knee_body, self._base_body, self._link6],
                                           dtype=torch.int32, device=device)
        positions = torch.zeros(n, 4, 3, device=device)
        positions[:, :3] = self._wrench_positions
        positions[:, 3] = self._jaw_offset
        self._wrench_positions = positions
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        self._q_ref, self._ik_ref = zeros(n, 6), zeros(n, 6)
        self._grip_w, self._approach_w, self._bar0_w = zeros(n, 3), zeros(n, 3), zeros(n, 3)
        self._grip_force_w = zeros(n, 3)
        self._holding = torch.zeros(n, dtype=torch.bool, device=device)
        self._hold_active = torch.zeros(n, dtype=torch.bool, device=device)
        self._lost, self._term_lost = (torch.zeros(n, dtype=torch.bool, device=device) for _ in range(2))
        self._hold_err, self._hold_tilt, self._grip_peak = zeros(n), zeros(n), zeros(n)
        self._hold_err_sum, self._hold_err_max, self._hold_steps = zeros(n), zeros(n), zeros(n)
        self._last_arm_actions = zeros(n, 6)
        self._z0 = zeros(n)
        self._first_step = None
        self._lower, self._upper = self._kin.soft_lower.to(device), self._kin.soft_upper.to(device)
        self._gravity = ArmGravity(self._kin, wrist_payload=cfg.wrist_payload)
        self._arm_kp = torch.as_tensor(interface.P_GAINS[12:18], dtype=torch.float32)
        self._ik_geo = zeros(n, 6)
        self._terms.update({
            "key_hold": lambda s: torch.exp(-torch.square(self._hold_err / leg_cfg.KEYHOLD_SIGMA_M)) * self._hold_active,
            "key_hold_coarse": lambda s: torch.exp(-torch.square(self._hold_err / leg_cfg.KEYHOLD_COARSE_SIGMA_M))
            * self._hold_active,
            "key_tilt": lambda s: self._hold_tilt * self._hold_active,
            "arm_action_rate": lambda s: torch.square(self._actions[:, 12:18] - self._last_arm_actions).sum(-1),
        })
        self._add_hold_terms()

    def _add_hold_terms(self) -> None:
        """The hold's reward terms, and the arm link's penalties (`--arm_step_weight`, `--arm_lead_weight`):
        `Go2D1LegEnv` rebuilds the reward table after `env.py` adds the link's, in `__init__` and `set_evaluation`.
        Until 2026-11-11 nothing added them back, so no key-hold training had either penalty (rounds 3-13 named them).
        `--base_tilt_weight` and `--leg_target_weight` (`env._shaping_terms`) come back the same way."""
        link_weights, link_terms = self._arm_link_terms()
        # J1-J5 only: Joint6 follows the key's turn by script (its target leads the reports by up to 46 deg in a turn,
        # 2026-11-11), so pricing it would price turning the key, which the policy does not command at the wrist.
        if "arm_lead" in link_terms:
            link_terms["arm_lead"] = lambda state: torch.sum(torch.square(torch.relu(
                (self._joint_targets[:, 12:17] - self._arm_link.reported[:, :5]).abs() - task_cfg.ARM_LEAD_FREE_RAD)), dim=1)
        if "arm_command_step" in link_terms:
            link_terms["arm_command_step"] = lambda state: self._arm_link.sent_step_joints[:, :5].sum(dim=1)
        shaping_weights, shaping_terms = self._shaping_terms()
        if self.cfg.stance_sink_weight:
            shaping_weights["stance_sink"] = self.cfg.stance_sink_weight * interface.POLICY_DT
            shaping_terms["stance_sink"] = lambda s: torch.square(torch.relu(
                self._z0 - (self._robot.data.root_pos_w[:, 2] - self.scene.env_origins[:, 2])
                - leg_cfg.KEYHOLD_SINK_FREE_M)) * self._hold_active
        if self.cfg.support_target_weight:
            support = [3 * f + k for f in leg_cfg.SUPPORT_FEET for k in range(3)]
            shaping_weights["support_target"] = self.cfg.support_target_weight * interface.POLICY_DT
            shaping_terms["support_target"] = lambda s: torch.sum(torch.square(torch.relu(
                (self._actions[:, support] * interface.ACTION_SCALE).abs() - leg_cfg.KEYHOLD_SUPPORT_FREE_RAD)), dim=1)
        if self.cfg.turn_follow_weight:
            shaping_weights["key_turn_ahead"] = self.cfg.turn_follow_weight * interface.POLICY_DT
            shaping_terms["key_turn_ahead"] = lambda s: torch.square(turn_ahead(
                self._handle.theta, self._layer.theta_ref, self._layer.theta_start)) * s.handle_pushing
        self._terms.update(link_terms | shaping_terms)
        extra = leg_cfg.scaled_weights(leg_cfg.KEYHOLD_WEIGHTS) | link_weights | shaping_weights
        self._weights.update(extra)
        self._episode_sums.update({name: torch.zeros(self.num_envs, device=self.device) for name in extra})

    def set_evaluation(self, plan: dict) -> None:
        super().set_evaluation(plan)
        self._add_hold_terms()

    # --- the key and the hand -----------------------------------------------------------------------------------------

    def _jaw_state(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """The jaw centre (env frame), its velocity (world) and Link6's rotation (world)."""
        quat = self._robot.data.body_quat_w[:, self._link6]
        arm = interface.quat_apply(quat, self._jaw_offset)
        pos = self._robot.data.body_pos_w[:, self._link6] + arm - self.scene.env_origins
        vel = (self._robot.data.body_lin_vel_w[:, self._link6]
               + torch.cross(self._robot.data.body_ang_vel_w[:, self._link6], arm, dim=-1))
        return pos, vel, quat

    def _grip_target(self, theta: torch.Tensor | None = None) -> tuple[torch.Tensor, torch.Tensor]:
        """The jaw centre's place and Link6's rotation holding the key at `theta` (the lock's angle), env frame."""
        theta = self._handle.theta if theta is None else theta
        return self._grip_w, hand_rotation(self._approach_w, bar_at(self._bar0_w, theta - self._handle.theta_min))

    def _in_base(self, point_env: torch.Tensor, rot_w: torch.Tensor | None = None, ids: torch.Tensor | None = None):
        """An env-frame point (and a world rotation) in the base's frame (of the environments `ids`, or all)."""
        base = self._robot.data.root_pos_w - self.scene.env_origins
        quat = self._robot.data.root_quat_w
        if ids is not None:
            base, quat = base[ids], quat[ids]
        p = interface.quat_rotate_inverse(quat, point_env - base)
        if rot_w is None:
            return p
        r_base = quat_matrix(quat)
        return p, r_base.transpose(1, 2) @ rot_w

    def _solve_ref(self, ids: torch.Tensor) -> torch.Tensor:
        """Joint angles holding the key at its start for `ids`, from the base's pose now: (q (k, 6), solved (k,))."""
        point, rot = self._grip_target()
        p_b, r_b = self._in_base(point[ids], rot[ids], ids)
        p_b, r_b = p_b.cpu(), r_b.cpu()
        lower, upper = self._kin.soft_lower, self._kin.soft_upper
        q, ok = self._kin.track(p_b, r_b, self._kin_nominal.expand(len(ids), 6), lower, upper, iterations=10)
        if not bool(ok.all()):
            miss = (~ok).nonzero().flatten()
            q2, ok2 = self._kin.inverse(p_b[miss], r_b[miss], self._kin_seeds, soft=True)
            q[miss], ok[miss] = q2, ok2
        return q.to(self.device), ok.to(self.device)

    def _sag(self, q: torch.Tensor, ids: torch.Tensor | None = None) -> torch.Tensor:
        """The command offset that leaves the arm at `q` under gravity against its PD (zero if not compensating, or
        with the servo's stiff drive, `arm_link.swap_arm`, which does not sag)."""
        if not self.cfg.sag_compensation or self.cfg.arm_servo:
            return torch.zeros_like(q)
        quat = self._robot.data.root_quat_w if ids is None else self._robot.data.root_quat_w[ids]
        g_w = torch.tensor([0.0, 0.0, -9.81], device=self.device).expand(q.shape[0], 3)
        g_b = interface.quat_rotate_inverse(quat, g_w).cpu()
        return (self._gravity.hold(q.cpu(), g_b) / self._arm_kp).to(q.device)

    # --- acting -------------------------------------------------------------------------------------------------------

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._last_actions = self._actions
        self._last_arm_actions = self._actions[:, 12:18].clone()
        if actions.shape[1] < 18:
            # A leg policy's 12 (a baseline): the arm's six are zero.
            actions = torch.cat((actions, torch.zeros(actions.shape[0], 18 - actions.shape[1], device=actions.device)), -1)
        clipped = torch.clip(actions, -interface.CLIP_ACTIONS, interface.CLIP_ACTIONS)
        legs = clipped[:, :12]
        arm = clipped[:, 12:18].clone() if self.cfg.hold_arm == "policy" else torch.zeros_like(clipped[:, 12:18])
        arm[:, 5] = 0.0                                   # the wrist's roll follows the key, not the policy
        arm = arm * self._hold_active.unsqueeze(-1)
        self._actions = torch.cat((legs, arm), dim=-1)
        self._joint_targets = interface.joint_targets(self._actions)
        self._answer_legs()
        if self.cfg.hold_arm == "ik":
            due = (self._since_reset % leg_cfg.KEYHOLD_IK_EVERY_STEPS) == 0
            if bool((due & self._hold_active).any()):
                point, rot = self._grip_target()
                p_b, r_b = self._in_base(point, rot)
                q, ok = self._kin.track(p_b.cpu(), r_b.cpu(), self._ik_geo.cpu(), self._kin.soft_lower, self._kin.soft_upper)
                q, ok = q.to(self.device), ok.to(self.device)
                take = (due & self._hold_active & ok).unsqueeze(-1)
                self._ik_geo = torch.where(take, q, self._ik_geo)
                self._ik_ref = torch.where(take, q + self._sag(q), self._ik_ref)
            ref = self._ik_ref
        else:
            ref = self._q_ref.clone()
            ref[:, 5] = ref[:, 5] + leg_cfg.KEYHOLD_J6_SIGN * (self._handle.theta - self._handle.theta_min)
        arm_t = ref + interface.ACTION_SCALE * arm
        arm_t = torch.max(torch.min(arm_t, self._upper), self._lower)
        held = self._hold_active.unsqueeze(-1)
        self._joint_targets[:, 12:18] = torch.where(held, arm_t, self._arm_pose)
        self._finish_targets()

    def _finish_targets(self) -> None:
        """After this step's targets are set: the arm link (if any), the chatter tracker, the forces."""
        if self._arm_link is not None:
            # The D1 as on the dog (`--arm_locked --arm_follower`): what the joints follow is the link's, at its reports.
            self._arm_link.command(self._joint_targets[:, 12:18], self._dof()[0][:, 12:18])
        self._track_chatter()
        self._step_forces()

    def _write_wrench(self) -> None:
        """The leg task's three forces, and the grip's spring on Link6 at the jaw centre."""
        jaw, jaw_v, quat6 = self._jaw_state()
        miss = jaw - self._grip_w
        force = -self.cfg.grip_k * miss - self.cfg.grip_c * jaw_v
        force = force * self._holding.unsqueeze(-1)
        size = force.norm(dim=-1, keepdim=True)
        force = force * (leg_cfg.KEYHOLD_GRIP_MAX_N / size.clamp(min=1e-9)).clamp(max=1.0)     # the jaws slip past it
        self._grip_force_w = force
        self._grip_peak = torch.maximum(self._grip_peak, force.norm(dim=-1))
        foot_quat = self._robot.data.body_quat_w[:, self._foot_body]
        knee_quat = self._robot.data.body_quat_w[:, self._knee_body]
        local = torch.stack((interface.quat_rotate_inverse(foot_quat, self._ee_force_w),
                             interface.quat_rotate_inverse(knee_quat, self._knee_force_w),
                             interface.quat_rotate_inverse(self._robot.data.root_quat_w, self._head_force_w),
                             interface.quat_rotate_inverse(quat6, force)), dim=1)
        self._robot.permanent_wrench_composer.set_forces_and_torques(
            forces=local.contiguous(), positions=self._wrench_positions, body_ids=self._wrench_bodies)
        self._wrench_active = True

    # --- the step -----------------------------------------------------------------------------------------------------

    def _update_hold(self) -> None:
        if self._first_step is None:
            self._first_step = self.common_step_counter        # a resume sets the counter after construction
        jaw, _, quat6 = self._jaw_state()
        miss = self._grip_w - jaw
        self._hold_err = miss.norm(dim=-1) * self._hold_active
        approach = interface.quat_apply(quat6, torch.tensor([0.0, 0.0, 1.0], device=self.device).expand_as(jaw))
        cos = (approach * self._approach_w).sum(-1).clamp(-1.0, 1.0)
        self._hold_tilt = torch.acos(cos) * self._hold_active
        # The reset's bounce (the spawned stance is not exactly at rest: up to 2 cm at the hand in the first 0.2 s with
        # zero actions) is not the task's: no loss, and no statistics, inside the grace.
        settled = self._since_reset >= leg_cfg.KEYHOLD_GRACE_STEPS
        early = self._evaluation is None and (self.common_step_counter - self._first_step) < self.cfg.hold_loss_start_step
        limit = leg_cfg.KEYHOLD_LOST_EARLY_M if early else leg_cfg.KEYHOLD_LOST_M
        lost = self._holding & (self._hold_err > limit) & settled
        self._lost |= lost
        self._holding &= ~self._lost
        counted = self._holding & settled
        self._hold_err_sum += self._hold_err * counted
        self._hold_steps += counted.float()
        self._hold_err_max = torch.maximum(self._hold_err_max, self._hold_err * counted)
        # What the policy sees: the grip point from the jaw centre, in the base's frame.
        miss_b = self._in_base(self._grip_w) - self._in_base(jaw)
        self._commands[:, interface.CMD_EE_ORN_R:interface.CMD_EE_ORN_Y + 1] = \
            miss_b * leg_cfg.KEYHOLD_OBS_SCALE * self._hold_active.unsqueeze(-1)

    def _get_rewards(self) -> torch.Tensor:
        self._update_hold()
        return super()._get_rewards()

    def _object_state(self, state) -> None:
        """The key's part of the reward state; with `turn_follow_weight`, its progress and latch as the arc has them."""
        super()._object_state(state)
        if self.cfg.turn_follow_weight:
            hd, layer = self._handle, self._layer
            theta, latch_open = follow_turn(hd.theta, layer.theta_ref, layer.theta_start, hd.theta_latch,
                                            hd.latch_open())
            state.handle_theta = theta - hd.theta_min
            state.handle_open = latch_open.float()

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        terminated, timed_out = super()._get_dones()
        self._term_lost = self._lost.clone()
        return terminated | self._lost, timed_out

    def _get_observations(self) -> dict:
        # After a reset the miss is the new episode's (0 by construction, the hand placed on the key).
        jaw, _, _ = self._jaw_state()
        miss_b = self._in_base(self._grip_w) - self._in_base(jaw)
        self._commands[:, interface.CMD_EE_ORN_R:interface.CMD_EE_ORN_Y + 1] = \
            miss_b * leg_cfg.KEYHOLD_OBS_SCALE * self._hold_active.unsqueeze(-1)
        if self._arm_link is None:
            return super()._get_observations()
        return self._observations_through_link()

    def _observations_through_link(self) -> dict:
        """`Go2D1LegEnv._get_observations` with the actor reading the arm through the link (the dog's reports); the
        critic and the estimator keep the true state, as `env.py`'s UniFP task does."""
        dof_pos, dof_vel = self._dof()
        arm_pos, arm_vel = self._arm_link.observe(dof_pos[:, 12:18], dof_vel[:, 12:18])
        seen_pos, seen_vel = dof_pos.clone(), dof_vel.clone()
        seen_pos[:, 12:18], seen_vel[:, 12:18] = arm_pos, arm_vel
        ang_vel = self._robot.data.root_ang_vel_b
        noisy = self.cfg.obs_noise and self._evaluation is None
        if noisy:
            noise, k = leg_cfg.OBS_NOISE, interface.NUM_ACTIONS - 6
            u = lambda shape, half: (2 * torch.rand(shape, device=self.device) - 1) * half
            seen_pos[:, :k] += u((self.num_envs, k), noise["dof_pos"])
            seen_vel[:, :k] += u((self.num_envs, k), noise["dof_vel"])
            ang_vel = ang_vel + u((self.num_envs, 3), noise["ang_vel"])
        actor = interface.single_obs(self._robot.data.root_quat_w, ang_vel, seen_pos, seen_vel, self._actions,
                                     self._gait_phase, self._commands)
        if noisy:
            actor[:, :2] += u((self.num_envs, 2), leg_cfg.OBS_NOISE["roll_pitch"])
        actor = torch.clip(actor, -interface.CLIP_OBSERVATIONS, interface.CLIP_OBSERVATIONS)
        yaw = self._base_yaw_quat()
        feet_forces = self._contact_forces(self._feet_contacts)
        measured = self._ee_force_w if self._force_aware else torch.zeros_like(self._ee_force_w)
        critic = observations.privileged_obs(
            base_lin_vel_b=self._robot.data.root_lin_vel_b,
            ee_pos_w=self._tip_pos(), ee_goal_w=self._goal_world(),
            goal_centre_w=self._goal_centre(), base_yaw_quat=yaw,
            ee_force_w=measured, base_force_w=self._base_force_w,
            gripper_force_kp=self._gripper_force_kp,
            ee_force_cmd=self._commands[:, interface.CMD_EE_FORCE],
            leg_ref_diff=dof_pos[:, :12] - torch.as_tensor(interface.DEFAULT_DOF_POS[:12], device=self.device),
            mass_params=self._privileged_extra(),
            friction=torch.ones(self.num_envs, 1, device=self.device),
            motor_strength=torch.ones(self.num_envs, interface.NUM_ACTIONS, device=self.device),
            stance_mask=self._stance_mask(),
            contact_mask=(feet_forces[:, :, 2] > 5.0).float(),
            projected_gravity=self._robot.data.projected_gravity_b,
            base_ang_vel_b=self._robot.data.root_ang_vel_b,
            dof_pos=dof_pos, dof_vel=dof_vel, actions=self._actions,
            gait_phase=self._gait_phase, commands=self._commands)
        critic = torch.clip(critic, -interface.CLIP_OBSERVATIONS, interface.CLIP_OBSERVATIONS)
        return {"policy": self._actor_history.append(actor), "critic": self._critic_history.append(critic),
                "estimates": critic[:, :observations.NUM_ESTIMATES]}

    # --- resetting ----------------------------------------------------------------------------------------------------

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        self._log_hold_episodes(env_ids)
        super()._reset_idx(env_ids)
        # The base's height as spawned (the root buffer is written with the spawn): `stance_sink`'s reference.
        self._z0[env_ids] = self._robot.data.root_pos_w[env_ids, 2] - self.scene.env_origins[env_ids, 2]
        ids = env_ids[self._handle_env[env_ids]]
        self._hold_active[env_ids] = False
        self._holding[env_ids] = False
        self._lost[env_ids] = False
        for buf in (self._hold_err, self._hold_tilt, self._grip_peak, self._hold_err_sum, self._hold_err_max,
                    self._hold_steps):
            buf[env_ids] = 0.0
        self._grip_force_w[env_ids] = 0.0
        self._last_arm_actions[env_ids] = 0.0
        if len(ids) == 0:
            return
        # The grip on the key as it stands at its start: the jaw centre on the shaft's axis, `KEYHOLD_JAW_OFF_DOOR_M` off
        # the door's face, the hand coming in along the shaft, the bar between the jaws.
        hd = self._handle
        self._grip_w[ids] = grip_point(hd.door[ids], hd.n[ids])
        self._approach_w[ids] = -hd.n[ids]
        self._bar0_w[ids] = hd.u0[ids]
        tries = 0
        q, ok = self._solve_ref(ids)
        # A placement the arm cannot hold from this stance is drawn again (training); a frozen one stays unheld.
        while self._evaluation is None and not bool(ok.all()) and tries < leg_cfg.KEYHOLD_REDRAWS:
            again = ids[~ok]
            self._place_objects(again, self._draw_objects(again))
            self._grip_w[again] = grip_point(hd.door[again], hd.n[again])
            self._approach_w[again] = -hd.n[again]
            self._bar0_w[again] = hd.u0[again]
            q2, ok2 = self._solve_ref(again)
            q[~ok], ok[~ok] = q2, ok2
            tries += 1
        command = q + self._sag(q, ids)
        self._q_ref[ids] = command
        self._ik_ref[ids] = command
        self._ik_geo[ids] = q
        held = ids[ok]
        self._hold_active[held] = True
        self._holding[held] = True
        if len(held) > 0:
            joint_pos = self._robot.data.joint_pos[held].clone()
            joint_vel = self._robot.data.joint_vel[held].clone()
            joint_pos[:, self._order[12:18]] = q[ok]
            joint_vel[:, self._order[12:18]] = 0.0
            self._robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=held)
            self._arm_pose[held] = q[ok]
        if self._arm_link is not None:
            self._arm_link.reset(env_ids, self._dof()[0][env_ids, 12:18])

    def _log_hold_episodes(self, env_ids: torch.Tensor) -> None:
        log = self.extras.setdefault("log", {})
        held = env_ids[self._hold_active[env_ids]]
        log["Hold/active_fraction"] = float(self._hold_active[env_ids].float().mean()) if len(env_ids) else 0.0
        if len(held) == 0:
            return
        steps = self._hold_steps[held].clamp(min=1.0)
        log["Hold/err_mm_mean"] = 1000.0 * float((self._hold_err_sum[held] / steps).median())
        log["Hold/err_mm_max"] = 1000.0 * float(self._hold_err_max[held].median())
        log["Hold/grip_peak_n"] = float(self._grip_peak[held].median())
        log["Hold/lost_fraction"] = float(self._lost[held].float().mean())
