"""The D1 puts Lukas's key into his cam lock, the whole body under it: the key hold's policy, inserting (2026-11-13).

Lukas: "train the key insertion to be force aware so it can help by feeling where the lock is. model the lock off the
lever's specific lock shape and assume the lever is plastic and the lock is metal. use the model for the legs that we
recently discovered doesnt cause rattle" (`66600`'s, `memory: dog-validated-leg-model`; the key hold's rounds 12-26).

The key hold (`keyhold_env.Go2D1KeyHoldEnv`, 18 actions: the legs and a residual on the arm's reference) with:

  1. **The key in the jaws, the lock ahead** (`keylock.KeyLock`): the box's cam lock as measured, metal, its keyway the
     lever's profile with 0.1-0.25 mm of play; the PLA key rigid in the dog's grip, sliding back past the grip's force.
     The contact's force and moment act on Link6. The box is the key hold's, raised (`leg_cfg.KEYHOLD_*`); the lock's
     face is the bare cylinder's, 4.5 mm proud of the door, not the printed adapter's 17 mm.
  2. **The lock is known only from an estimate.** The true lock is off it across the axis (to 3.5 mm), along it (to
     8 mm: the look's range was 3-11 mm off on the dog, F-164), in roll (4 deg) and in tilt (0.8 deg).
  3. **The task layer runs the insertion** (`keylock.InsertLayer`) on the arm's *reported* joints through the D1's link
     (their forward kinematics, as the dog program measures every depth): in until the face is felt by the lag, a
     spiral with the wrist rolling, in, seated, held. The arm's reference is re-solved by IK from the layer's goal at
     10 Hz (the base's pose against the estimated lock), its sag compensated; the policy's residual goes on top.
  4. **The two variants** (`leg_variant`), the plan's P0 and P1 as for the foot:
     - *position*: the goal is the reference; the press is the reference held `keylock.PRESS_M` into the face. The
       reward pays the jaw centre on the goal, so a contact that holds the key back is to be fought.
     - *force*: the press is a commanded force along the estimated axis (`CMD_EE_FORCE`, base yaw frame), and the goal
       is set back from the reference by it over the reward's compliance. The reward pays UniFP's compliant target,
       goal + C (F_ext + F_cmd), C 1/800 m/N along the axis and 1/3000 across: hold the press whatever the face's depth,
       give way sideways to the keyway's edges. The critic is given the contact force (the actor is not).
  5. **The front-right foot stays down**: its turn never starts (the stance, the foot's goal on its footfall).

What the actor sees is what the dog can: its joints and IMU, the arm's reported angles (the link's), the goal less the
jaw centre by the forward kinematics of those angles, in the base's frame (`CMD_EE_ORN_*`, x `INSERT_OBS_SCALE`), and
the commanded press. Not modelled: the jaws themselves, the key turning in them, the cylinder's own turn, the
wrist camera's view of the keyhole (the estimate's error is drawn, not seen).
"""
from __future__ import annotations

import math

import torch

from isaaclab.utils import configclass

from unifp_isaaclab import interface

from . import arm_kin, keylock, leg_cfg
from .keyhold import hand_rotation, quat_matrix
from .keyhold_env import Go2D1KeyHoldEnv, Go2D1KeyHoldEnvCfg
from .leg_env import Go2D1LegEnv
from .env import Go2D1PosForceEnv
from .keylock import APPROACH, DONE, HOLD, SEARCH, SEAT

#: The goal's miss in the observation, metres x this (then x `interface.COMMANDS_SCALE` 0.5): the key hold's 20, 1 mm at
#: 0.01. At 50 (2026-11-13 00:40) round 16's legs chased the spawn's 5-8 cm swing, 2.5 times anything they had seen, and
#: tipped the robot 35 degrees in 1.6 s.
INSERT_OBS_SCALE = leg_cfg.KEYHOLD_OBS_SCALE
#: The estimate's errors (training): across the axis to this radius (uniform in area), along it, the keyway's roll, the
#: axis's tilt. The frozen set's are `keyinsert_eval`'s.
ERR_LATERAL_M = 0.0035
ERR_DEPTH_M = 0.008
ERR_ROLL_RAD = math.radians(4.0)
ERR_TILT_RAD = math.radians(0.8)
#: The tip's reference starts this far short of the estimated face.
START_OFF_M = (0.015, 0.020)
#: The layer starts once the spawn has settled (`leg_cfg.KEYHOLD_GRACE_STEPS`).
START_STEPS = 40
#: The classical outer loop on the hand (the dog program's, `insert_plan.LOOP_GAIN`, `LOOP_MAX_M`): the jaw centre's miss
#: by the reported joints, integrated into the IK's goal (not the policy's): across the axis except while the keyway
#: holds the key (held sideways, seating, holding); along it only position-only, approaching, before any lag.
LOOP_GAIN = 1.5
LOOP_MAX_M = 0.015
#: The arm's limits here: this share of the D1's hard (URDF) range, for the IK and the joint targets alike. The key
#: hold's soft limits (0.9) leave Joint5 0.07 rad at the insertion's start (-1.34 against -1.41 at the dog's stance), and
#: the body 3 cm forward or pitched 8 deg takes it past (CPU scan, 2026-11-13 01:00: in round 1 the IK failed 53-80% of
#: its calls, the reference froze and the layer felt a face in free space).
LIMIT_SHARE = 0.97
#: Torch's CPU threads in this process (the IK's).
CPU_THREADS = 4
#: The IK's polish: damped least squares on the jaw centre's whole pose after `ArmKinematics.track`, whose wrist model
#: misses by ~1.5 mm (the same scan), more than the keyway's 0.2 mm of play.
POLISH_ITERATIONS = 2
POLISH_DAMPING = 1e-4
#: A polished solution counts within this of its goal (m).
POLISH_TOL_M = 5e-4
EPISODE_S = 20.0
EVAL_EPISODE_S = 25.0
#: The episode ends this long after the layer is done (training).
END_AFTER_DONE_S = 0.4
#: The reward (per second): the fine kernel on the variant's target (m), the depth the key is in (of the keyway's),
#: seated, the press past `keylock.PRESS_LIMIT_N` (as the overlap it takes, cm), the key lost from the jaws.
FINE_SIGMA_M = 0.002
INSERT_WEIGHTS = {
    "ins_fine": 3.0,
    "ins_depth": 2.0,
    "ins_seated": 4.0,
    "ins_press_excess": -50.0,
    "ins_lost": -25.0,
}


@configclass
class Go2D1KeyInsertEnvCfg(Go2D1KeyHoldEnvCfg):
    episode_length_s = EPISODE_S
    #: The force-aware press (N): drawn in this range per episode in training, `keylock.EVAL_PRESS_N` in a frozen one.
    press_n: tuple[float, float] = keylock.PRESS_N
    #: The estimate's error scale (1: `ERR_*`); a frozen evaluation sets its own.
    error_scale: float = 1.0
    #: End the episode, as the dog runner stops a whole-body run, once the base tilts past this (rad; None: never), after
    #: the spawn's settling (`START_STEPS`). Round 4's policies leaned to 15-20 deg (the 8 deg free band priced at -50).
    tilt_stop_rad: float | None = None
    #: `keylock.InsertLayer.light_probe`: ease the press through the feel after the touch (opt-in, 2026-11-13 06:00).
    light_probe: bool = False


class Go2D1KeyInsertEnv(Go2D1KeyHoldEnv):
    cfg: Go2D1KeyInsertEnvCfg
    is_key_insert = True

    def __init__(self, cfg: Go2D1KeyInsertEnvCfg, render_mode: str | None = None, **kwargs):
        # The IK runs on the CPU in small batches: at torch's default thread count, beside another training, one call took
        # 244 ms against 15 ms on 1-4 threads (2026-11-13), and round 2 ran at 10 s an iteration.
        torch.set_num_threads(CPU_THREADS)
        super().__init__(cfg, render_mode, **kwargs)
        n, device = self.num_envs, self.device
        self._lock = keylock.KeyLock(n, device)
        self._layer_ins = keylock.InsertLayer(n, device)
        self._layer_ins.light_probe = cfg.light_probe
        self._fk = arm_kin.ArmKinematics(device, torch.float32)
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        #: The estimated lock's frame (env frame): what the robot believes, and plans from.
        self._est_o, self._est_a, self._est_u, self._est_t = zeros(n, 3), zeros(n, 3), zeros(n, 3), zeros(n, 3)
        self._goal_jaw = zeros(n, 3)
        self._loop = zeros(n, 3)                         # the outer loop's correction: u, t, along the axis
        self._ik_fail, self._ik_calls = zeros(n), zeros(n)
        mid, half = 0.5 * (self._kin.hard_lower + self._kin.hard_upper), 0.5 * (self._kin.hard_upper - self._kin.hard_lower)
        self._lim_lo_cpu, self._lim_hi_cpu = mid - LIMIT_SHARE * half, mid + LIMIT_SHARE * half
        self._lower, self._upper = self._lim_lo_cpu.to(device), self._lim_hi_cpu.to(device)
        self._ins_ref, self._ins_geo = zeros(n, 6), zeros(n, 6)
        self._meas_depth, self._meas_lateral = zeros(n), zeros(n, 2)
        self._miss_b = zeros(n, 3)
        self._ins_active = torch.zeros(n, dtype=torch.bool, device=device)
        self._done_for = zeros(n)
        # Episode bookkeeping.
        self._seated_ever = torch.zeros(n, dtype=torch.bool, device=device)
        self._seat_step = torch.full((n,), -1, dtype=torch.long, device=device)
        self._touch_true = torch.full((n,), float("nan"), device=device)
        self._damaged = torch.zeros(n, dtype=torch.bool, device=device)
        self._press_steps, self._press_sum = zeros(n), zeros(n)
        self._term_ins_lost = torch.zeros(n, dtype=torch.bool, device=device)
        self._tilt_stopped = torch.zeros(n, dtype=torch.bool, device=device)
        self._insert_eval: dict | None = None
        self._start_depth, self._press_draw = zeros(n), zeros(n)
        self._layer_started = torch.zeros(n, dtype=torch.bool, device=device)
        self._add_insert_terms()

    # --- rewards ------------------------------------------------------------------------------------------------------

    def _add_insert_terms(self) -> None:
        lk = self._lock
        on = lambda: self._ins_active.float()
        self._terms.update({
            "ins_fine": lambda s: torch.exp(-torch.square(self._hold_err / FINE_SIGMA_M)) * on(),
            "ins_depth": lambda s: (lk.tip_depth / keylock.KEYWAY_DEPTH_M).clamp(0.0, 1.0) * lk.tip_inside * on(),
            "ins_seated": lambda s: lk.seated.float() * on(),
            "ins_press_excess": lambda s: 100.0 * (lk.force_w.norm(dim=-1) - keylock.PRESS_LIMIT_N).clamp(min=0.0)
            / keylock.CONTACT_K_N_PER_M * on(),
            "ins_lost": lambda s: lk.lost.float() * on(),
        })
        # The foot's turn does not happen here: its handle terms read a key that is never turned (all zero), and the key
        # hold's tie to a seated key is replaced by the insertion's target (`_hold_err`).
        extra = leg_cfg.scaled_weights(INSERT_WEIGHTS)
        self._weights.update(extra)
        self._episode_sums.update({name: torch.zeros(self.num_envs, device=self.device) for name in extra})

    def set_evaluation(self, plan: dict) -> None:
        super().set_evaluation(plan)
        self._insert_eval = {k: v.to(self.device) for k, v in plan["insert"].items()}
        self._add_insert_terms()

    def _task_state(self):
        state = super()._task_state()
        # The force channel is the arm's press here, not the foot's: the foot's tracking is position-only.
        state.ee_force_cmd = torch.zeros_like(state.ee_force_cmd)
        state.ee_force_measured = torch.zeros_like(state.ee_force_measured)
        return state

    # --- frames -------------------------------------------------------------------------------------------------------

    def _est_point(self, depth: torch.Tensor, lateral: torch.Tensor) -> torch.Tensor:
        """The env-frame point at `depth` along the estimated axis and `lateral` (u, t) across it."""
        return (self._est_o + lateral[:, :1] * self._est_u + lateral[:, 1:2] * self._est_t
                + depth.unsqueeze(-1) * self._est_a)

    def _fk_depth(self, true_joints: bool = False) -> torch.Tensor:
        """The tip's depth along the estimated axis by the forward kinematics (of the reported joints, or the true)."""
        q = self._dof()[0][:, 12:18] if true_joints or self._arm_link is None else self._arm_link.reported
        rot_b, jaw_b = self._fk.tool(q)
        base = self._robot.data.root_pos_w - self.scene.env_origins
        quat = self._robot.data.root_quat_w
        tip = base + interface.quat_apply(quat, jaw_b + keylock.TIP_FROM_JAW_M * rot_b[:, :, 2])
        return ((tip - self._est_o) * self._est_a).sum(-1)

    def _measure(self) -> tuple[torch.Tensor, torch.Tensor]:
        """The jaw centre and Link6's z as the dog measures them: the forward kinematics of the arm's reported angles
        (the link's), placed by the base's pose. Env frame."""
        q = self._arm_link.reported if self._arm_link is not None else self._dof()[0][:, 12:18]
        rot_b, jaw_b = self._fk.tool(q)
        base = self._robot.data.root_pos_w - self.scene.env_origins
        quat = self._robot.data.root_quat_w
        jaw = base + interface.quat_apply(quat, jaw_b)
        z6 = interface.quat_apply(quat, rot_b[:, :, 2])
        return jaw, z6, jaw_b

    # --- acting -------------------------------------------------------------------------------------------------------

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._last_actions = self._actions
        self._last_arm_actions = self._actions[:, 12:18].clone()
        if actions.shape[1] < 18:
            actions = torch.cat((actions, torch.zeros(actions.shape[0], 18 - actions.shape[1], device=actions.device)), -1)
        clipped = torch.clip(actions, -interface.CLIP_ACTIONS, interface.CLIP_ACTIONS)
        legs = clipped[:, :12]
        arm = clipped[:, 12:18].clone() if self.cfg.hold_arm == "policy" else torch.zeros_like(clipped[:, 12:18])
        arm[:, 5] = 0.0                                   # the wrist's roll is the layer's
        arm = arm * self._ins_active.unsqueeze(-1)
        self._actions = torch.cat((legs, arm), dim=-1)
        self._joint_targets = interface.joint_targets(self._actions)
        self._answer_legs()
        ref = self._ins_ref.clone()
        ref[:, 5] = ref[:, 5] + self._layer_ins.roll
        arm_t = ref + interface.ACTION_SCALE * arm
        arm_t = torch.max(torch.min(arm_t, self._upper), self._lower)
        self._joint_targets[:, 12:18] = torch.where(self._ins_active.unsqueeze(-1), arm_t, self._arm_pose)
        self._finish_targets()

    def _step_forces(self) -> None:
        self._push_w = torch.zeros_like(self._push_w)
        press = self._layer_ins.force_cmd * self._ins_active
        cmd_w = press.unsqueeze(-1) * self._est_a
        cmd_b = interface.quat_rotate_inverse(self._base_yaw_quat(), cmd_w)
        self._commands[:, interface.CMD_EE_FORCE] = cmd_b if self._force_aware else 0.0

    def _apply_action(self) -> None:
        Go2D1PosForceEnv._apply_action(self)
        quat6 = self._robot.data.body_quat_w[:, self._link6]
        rot6 = quat_matrix(quat6)
        pos6 = self._robot.data.body_pos_w[:, self._link6] - self.scene.env_origins
        self._lock.step(pos6, rot6, self._robot.data.body_lin_vel_w[:, self._link6],
                        self._robot.data.body_ang_vel_w[:, self._link6], interface.SIM_DT, self._ins_active)
        self._ee_force_w = torch.zeros_like(self._ee_force_w)
        self._knee_force_w = torch.zeros_like(self._knee_force_w)
        self._head_force_w = torch.zeros_like(self._head_force_w)
        self._write_wrench()

    def _write_wrench(self) -> None:
        """The key's contact on Link6 (force and moment about the jaw centre); nothing on the foot, knee or base."""
        quat6 = self._robot.data.body_quat_w[:, self._link6]
        forces = torch.zeros(self.num_envs, 4, 3, device=self.device)
        torques = torch.zeros(self.num_envs, 4, 3, device=self.device)
        forces[:, 3] = interface.quat_rotate_inverse(quat6, self._lock.force_w)
        torques[:, 3] = interface.quat_rotate_inverse(quat6, self._lock.torque_w)
        self._grip_force_w = self._lock.force_w
        self._grip_peak = torch.maximum(self._grip_peak, self._lock.force_w.norm(dim=-1))
        self._robot.permanent_wrench_composer.set_forces_and_torques(
            forces=forces, torques=torques, positions=self._wrench_positions, body_ids=self._wrench_bodies)
        self._wrench_active = True

    # --- the step -----------------------------------------------------------------------------------------------------

    def _update_hold(self) -> None:
        """Each policy step, before the rewards: the layer on the measured tip, the goal, the reference's IK at 10 Hz,
        the actor's miss, and the reward's target."""
        if self._first_step is None:
            self._first_step = self.common_step_counter
        lk, ly = self._lock, self._layer_ins
        jaw_m, z6_m, jaw_b = self._measure()
        tip_m = jaw_m + keylock.TIP_FROM_JAW_M * z6_m
        rel = tip_m - self._est_o
        self._meas_depth = (rel * self._est_a).sum(-1)
        self._meas_lateral = torch.stack(((rel * self._est_u).sum(-1), (rel * self._est_t).sum(-1)), -1)
        go = self._ins_active & (ly.phase == DONE) & (self._since_reset == START_STEPS) & ~self._layer_started
        if bool(go.any()):
            ids = go.nonzero().flatten()
            ly.start(ids, self._start_depth[ids], torch.full((len(ids),), self._force_aware, device=self.device),
                     self._press_draw[ids])
            self._layer_started[ids] = True
        ly.step(self._meas_depth, self._meas_lateral, interface.POLICY_DT)
        depth, lateral = ly.goal()
        tip_goal = self._est_point(depth, lateral)
        self._goal_jaw = tip_goal - keylock.TIP_FROM_JAW_M * self._est_a
        # The outer loop, on what the robot measures.
        dt = interface.POLICY_DT
        running = self._ins_active & self._layer_started & (ly.phase < DONE)
        free_lat = running & (ly.phase <= SEARCH) & (ly.held_for <= 0.0)
        lat_miss = ly.ref_lateral - self._meas_lateral
        self._loop[:, :2] = torch.where(free_lat.unsqueeze(-1), self._loop[:, :2] + LOOP_GAIN * dt * lat_miss,
                                        self._loop[:, :2])
        free_ax = running & (ly.phase == APPROACH) & (ly.phase_for < keylock.BASE_S)
        ax_miss = ly.ref_depth - self._meas_depth
        self._loop[:, 2] = torch.where(free_ax, self._loop[:, 2] + LOOP_GAIN * dt * ax_miss, self._loop[:, 2])
        self._loop = self._loop.clamp(-LOOP_MAX_M, LOOP_MAX_M)
        loop_w = self._loop[:, :1] * self._est_u + self._loop[:, 1:2] * self._est_t + self._loop[:, 2:3] * self._est_a
        # The reference: IK at 10 Hz from the base's pose against the estimated lock.
        due = self._ins_active & ((self._since_reset % leg_cfg.KEYHOLD_IK_EVERY_STEPS) == 0)
        if bool(due.any()):
            ids = due.nonzero().flatten()
            rot = hand_rotation(self._est_a[ids], self._est_u[ids])
            p_b, r_b = self._in_base(self._goal_jaw[ids] + loop_w[ids], rot, ids)
            q, ok = self._ik(p_b.cpu(), r_b.cpu(), self._ins_geo[ids].cpu())
            q, ok = q.to(self.device), ok.to(self.device)
            take = ids[ok]
            self._ik_fail[ids[~ok]] += 1.0
            self._ik_calls[ids] += 1.0
            self._ins_geo[take] = q[ok]
            self._ins_ref[take] = q[ok] + self._sag(q[ok], take)
        # The actor's miss: the goal less the measured jaw centre, in the base's frame.
        goal_b = self._in_base(self._goal_jaw)
        self._miss_b = (goal_b - jaw_b) * self._ins_active.unsqueeze(-1)
        # The reward's target: the goal, or (force-aware) UniFP's compliant target about it.
        target = self._goal_jaw.clone()
        if self._force_aware:
            f = lk.force_w + (ly.force_cmd * self._ins_active).unsqueeze(-1) * self._est_a
            f_ax = (f * self._est_a).sum(-1, keepdim=True) * self._est_a
            target = target + f_ax / keylock.K_AXIAL_N_PER_M + (f - f_ax) / keylock.K_LATERAL_N_PER_M
        self._grip_w = target
        jaw, _, quat6 = self._jaw_state()
        self._hold_err = (target - jaw).norm(dim=-1) * self._ins_active
        approach = interface.quat_apply(quat6, torch.tensor([0.0, 0.0, 1.0], device=self.device).expand_as(jaw))
        self._hold_tilt = torch.acos((approach * self._est_a).sum(-1).clamp(-1.0, 1.0)) * self._ins_active
        self._hold_active = self._ins_active.clone()
        # Bookkeeping.
        self._seated_ever |= lk.seated & self._ins_active
        first = lk.seated & (self._seat_step < 0) & self._ins_active
        self._seat_step = torch.where(first, self._since_reset, self._seat_step)
        felt = (ly.phase >= SEARCH) & torch.isnan(self._touch_true) & self._ins_active
        # Where the layer felt the face, against where the true face is along the estimated axis (m).
        self._touch_true = torch.where(felt, ly.touch_depth - self._true_off_depth(), self._touch_true)
        self._damaged |= (lk.peak_n > keylock.DAMAGE_N) & self._ins_active
        pressing = (ly.phase >= SEARCH) & (ly.phase <= SEAT) & self._ins_active
        self._press_sum += lk.normal_n * pressing
        self._press_steps += pressing.float()
        self._done_for = torch.where((ly.phase == DONE) & self._layer_started, self._done_for + interface.POLICY_DT,
                                     torch.zeros_like(self._done_for))
        self._lost = lk.lost & self._ins_active

    def _true_off_depth(self) -> torch.Tensor:
        """Where the true lock's face is along the estimated axis (from the estimated face), m: the depth error."""
        return ((self._lock.origin - self._est_o) * self._est_a).sum(-1)

    def _get_observations(self) -> dict:
        self._commands[:, interface.CMD_EE_ORN_R:interface.CMD_EE_ORN_Y + 1] = self._miss_b * INSERT_OBS_SCALE
        if self._arm_link is None:
            return Go2D1LegEnv._get_observations(self)
        return self._observations_through_link()

    def _privileged_extra(self) -> torch.Tensor:
        """The insertion as the critic sees it (22 wide): the true lock against the estimate, where the tip is, the
        layer's phase; the contact force only for the force variant (the plan's P0 rule)."""
        lk, ly = self._lock, self._layer_ins
        on = self._ins_active.float().unsqueeze(-1)
        out = torch.zeros(self.num_envs, 22, device=self.device)
        out[:, 0:1] = on
        out[:, 1:2] = (ly.phase.float() / 4.0).unsqueeze(-1) * on
        out[:, 2:4] = lk.tip_lateral * 200.0 * on
        out[:, 4:5] = (lk.tip_depth * 100.0).clamp(-3.0, 3.0).unsqueeze(-1) * on
        out[:, 5:6] = lk.tip_inside.float().unsqueeze(-1) * on
        out[:, 6:7] = lk.seated.float().unsqueeze(-1) * on
        rel = lk.origin - self._est_o
        out[:, 7:10] = interface.quat_rotate_inverse(self._base_yaw_quat(), rel) * 100.0 * on
        out[:, 10:11] = (lk.slip_m * 100.0).unsqueeze(-1) * on
        out[:, 11:12] = (lk.mu * 2.0).unsqueeze(-1) * on
        out[:, 12:13] = (lk.play * 2000.0).unsqueeze(-1) * on
        roll = torch.atan2((torch.cross(self._est_u, lk.e_u, dim=-1) * lk.axis).sum(-1), (self._est_u * lk.e_u).sum(-1))
        out[:, 13:14] = (roll * 10.0).unsqueeze(-1) * on
        out[:, 14:15] = (ly.roll * 5.0).unsqueeze(-1) * on
        out[:, 15:16] = (ly.force_cmd * 0.1).unsqueeze(-1) * on
        out[:, 18:19] = self._handle_env.float().unsqueeze(-1)
        if self._force_aware:
            out[:, 19:22] = interface.quat_rotate_inverse(self._base_yaw_quat(), lk.force_w) * 0.05 * on
            out[:, 16:17] = (lk.normal_n * 0.05).unsqueeze(-1) * on
        return out

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        terminated, timed_out = Go2D1LegEnv._get_dones(self)
        if self.cfg.tilt_stop_rad is not None:
            tilt = interface.body_roll_pitch(self._robot.data.root_quat_w).abs().amax(dim=1)
            self._tilt_stopped = (tilt > self.cfg.tilt_stop_rad) & (self._since_reset > START_STEPS) & self._ins_active
            terminated = terminated | self._tilt_stopped
        self._term_lost = self._lost.clone()
        self._term_ins_lost = self._lost.clone()
        terminated = terminated | self._lost
        if self._evaluation is not None:
            terminated = terminated | self._damaged
        # Over once the layer has finished its hold, frozen or not: the dog's next step is the turn.
        timed_out = timed_out | (self._ins_active & (self._done_for >= END_AFTER_DONE_S))
        return terminated, timed_out

    # --- resetting ----------------------------------------------------------------------------------------------------

    def _draw_insert(self, ids: torch.Tensor) -> dict[str, torch.Tensor]:
        k, device = len(ids), self.device
        rand = lambda *shape: torch.rand(*shape, device=device)
        span = lambda r: r[0] + rand(k) * (r[1] - r[0])
        sym = lambda a: (2 * rand(k) - 1) * a
        s = self.cfg.error_scale
        return {"lat_r": ERR_LATERAL_M * s * torch.sqrt(rand(k)), "lat_dir": 2 * math.pi * rand(k),
                "depth": sym(ERR_DEPTH_M * s), "roll": sym(ERR_ROLL_RAD * s), "tilt_u": sym(ERR_TILT_RAD * s),
                "tilt_t": sym(ERR_TILT_RAD * s), "play": span(keylock.PLAY_M), "lock_chamfer": span(keylock.LOCK_CHAMFER_M),
                "key_chamfer": span(keylock.KEY_CHAMFER_M), "mu": span(keylock.MU), "grip_slip": span(keylock.GRIP_SLIP_N),
                "start_off": span(START_OFF_M), "press": span(self.cfg.press_n)}

    def _place_insert(self, ids: torch.Tensor, d: dict[str, torch.Tensor]) -> None:
        """The estimate is the key's placement (the leg task's: the shaft's axis on the door's face, the bar level to
        the robot's right); the true lock is off it by the draws."""
        hd = self._handle
        n_out = hd.n[ids]
        axis = -n_out
        e_u = hd.u0[ids]
        e_t = torch.cross(e_u, axis, dim=-1)
        self._est_a[ids], self._est_u[ids], self._est_t[ids] = axis, e_u, e_t
        self._est_o[ids] = hd.door[ids] + keylock.LOCK_FACE_OFF_DOOR_M * n_out
        lat = torch.stack((torch.cos(d["lat_dir"]), torch.sin(d["lat_dir"])), -1) * d["lat_r"].unsqueeze(-1)
        origin = self._est_o[ids] + lat[:, :1] * e_u + lat[:, 1:2] * e_t + d["depth"].unsqueeze(-1) * axis
        tilt = d["tilt_u"].unsqueeze(-1) * e_u + d["tilt_t"].unsqueeze(-1) * e_t
        true_axis = axis + torch.cross(tilt, axis, dim=-1)
        true_axis = true_axis / true_axis.norm(dim=-1, keepdim=True)
        roll = d["roll"].unsqueeze(-1)
        true_u = torch.cos(roll) * e_u + torch.sin(roll) * e_t
        self._lock.place(ids, origin, true_axis, true_u, d["play"], d["lock_chamfer"], d["key_chamfer"], d["mu"],
                         d["grip_slip"])
        self._start_depth[ids] = -d["start_off"]
        self._press_draw[ids] = d["press"]
        # Until the layer starts, its goal is the start (not the last episode's).
        self._layer_ins.ref_depth[ids] = -d["start_off"]
        self._layer_ins.ref_lateral[ids] = 0.0
        self._layer_ins.force_cmd[ids] = 0.0

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        self._log_insert_episodes(env_ids)
        Go2D1LegEnv._reset_idx(self, env_ids)
        self._z0[env_ids] = self._robot.data.root_pos_w[env_ids, 2] - self.scene.env_origins[env_ids, 2]
        # The foot's turn never starts.
        self._start_delay[env_ids] = 10 ** 9
        for buf in (self._hold_active, self._holding, self._lost, self._ins_active, self._seated_ever, self._damaged,
                    self._layer_started, self._term_ins_lost):
            buf[env_ids] = False
        for buf in (self._hold_err, self._hold_tilt, self._grip_peak, self._hold_err_sum, self._hold_err_max,
                    self._hold_steps, self._done_for, self._press_sum, self._press_steps):
            buf[env_ids] = 0.0
        self._seat_step[env_ids] = -1
        self._touch_true[env_ids] = float("nan")
        self._grip_force_w[env_ids] = 0.0
        self._last_arm_actions[env_ids] = 0.0
        self._miss_b[env_ids] = 0.0
        self._loop[env_ids] = 0.0
        self._ik_fail[env_ids] = 0.0
        self._ik_calls[env_ids] = 0.0
        self._layer_ins.stop(env_ids)
        self._layer_ins.roll[env_ids] = 0.0
        ids = env_ids[self._handle_env[env_ids]]
        if len(ids) == 0:
            return
        if self._insert_eval is not None:
            draw = {k: v[ids] for k, v in self._insert_eval.items()}
        else:
            draw = self._draw_insert(ids)
        self._place_insert(ids, draw)
        q, ok = self._solve_start(ids)
        tries = 0
        while self._evaluation is None and not bool(ok.all()) and tries < leg_cfg.KEYHOLD_REDRAWS:
            again = ids[~ok]
            self._place_objects(again, self._draw_objects(again))
            self._place_insert(again, {k: v[~ok] for k, v in draw.items()})
            q2, ok2 = self._solve_start(again)
            q[~ok], ok[~ok] = q2, ok2
            tries += 1
        command = q + self._sag(q, ids)
        self._ins_ref[ids] = command
        self._ins_geo[ids] = q
        held = ids[ok]
        self._ins_active[held] = True
        self._hold_active[held] = True
        if len(held) > 0:
            joint_pos = self._robot.data.joint_pos[held].clone()
            joint_vel = self._robot.data.joint_vel[held].clone()
            joint_pos[:, self._order[12:18]] = q[ok]
            joint_vel[:, self._order[12:18]] = 0.0
            self._robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=held)
            self._arm_pose[held] = q[ok]
        if self._arm_link is not None:
            self._arm_link.reset(env_ids, self._dof()[0][env_ids, 12:18])

    def _solve_start(self, ids: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """The arm's joints with the key's tip at its start, short of the estimated face on the estimated axis."""
        tip = self._est_o[ids] + self._start_depth[ids].unsqueeze(-1) * self._est_a[ids]
        jaw = tip - keylock.TIP_FROM_JAW_M * self._est_a[ids]
        self._goal_jaw[ids] = jaw
        rot = hand_rotation(self._est_a[ids], self._est_u[ids])
        p_b, r_b = self._in_base(jaw, rot, ids)
        p_b, r_b = p_b.cpu(), r_b.cpu()
        q, ok = self._ik(p_b, r_b, self._kin_nominal.expand(len(ids), 6))
        if not bool(ok.all()):
            miss = (~ok).nonzero().flatten()
            q2, _ = self._kin.inverse(p_b[miss], r_b[miss], self._kin_seeds, soft=False)
            q2, ok2 = self._ik(p_b[miss], r_b[miss], q2)
            q[miss], ok[miss] = q2, ok2
        return q.to(self.device), ok.to(self.device)

    def _ik(self, p_b: torch.Tensor, r_b: torch.Tensor, seed: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """The jaw centre's pose in the base's frame (CPU): `ArmKinematics.track` from `seed`, then polished by damped
        least squares on the whole pose (`POLISH_*`), inside `LIMIT_SHARE` of the hard range."""
        lo, hi = self._lim_lo_cpu, self._lim_hi_cpu
        q, _ = self._kin.track(p_b, r_b, seed, lo, hi, iterations=10)
        eps = 1e-4
        for _ in range(POLISH_ITERATIONS):
            err = _pose_error(self._kin, q, p_b, r_b)                        # (k, 6)
            k = q.shape[0]
            dq = torch.eye(6).unsqueeze(0) * eps                             # (1, 6, 6)
            qq = (q.unsqueeze(1) + dq).reshape(-1, 6)
            e2 = _pose_error(self._kin, qq, p_b.repeat_interleave(6, 0), r_b.repeat_interleave(6, 0)).reshape(k, 6, 6)
            jac = -(e2 - err.unsqueeze(1)).transpose(1, 2) / eps             # d(error)/dq = -J
            jtj = jac.transpose(1, 2) @ jac + POLISH_DAMPING * torch.eye(6)
            step = torch.linalg.solve(jtj, (jac.transpose(1, 2) @ err.unsqueeze(-1))).squeeze(-1)
            q = torch.max(torch.min(q + step, hi), lo)
        err = _pose_error(self._kin, q, p_b, r_b)
        ok = (err[:, :3].norm(dim=-1) < POLISH_TOL_M) & (err[:, 3:].norm(dim=-1) < 0.01)
        return q, ok

    # --- the viewer ---------------------------------------------------------------------------------------------------

    def _set_debug_vis_impl(self, enable: bool) -> None:
        """The insertion's own overlay (`keyinsert_vis`) in place of the key hold's, which draws the key seated in the
        lock where here it is in the jaws."""
        if self.cfg.unifp_markers:
            Go2D1PosForceEnv._set_debug_vis_impl(self, enable)
        if enable and not hasattr(self, "_insert_markers"):
            from .keyinsert_vis import InsertMarkers

            self._insert_markers = InsertMarkers("/Visuals/key_insert", self.num_envs, str(self.device))
        if hasattr(self, "_insert_markers"):
            self._insert_markers.set_visibility(enable)

    def _debug_vis_callback(self, event) -> None:
        if hasattr(self, "_markers"):
            Go2D1PosForceEnv._debug_vis_callback(self, event)
        if hasattr(self, "_insert_markers"):
            self._insert_markers.update(self)

    def _log_insert_episodes(self, env_ids: torch.Tensor) -> None:
        log = self.extras.setdefault("log", {})
        held = env_ids[self._ins_active[env_ids]]
        log["Insert/active_fraction"] = float(self._ins_active[env_ids].float().mean()) if len(env_ids) else 0.0
        if len(held) == 0:
            return
        ly, lk = self._layer_ins, self._lock
        log["Insert/seated_fraction"] = float(lk.seated[held].float().mean())
        log["Insert/seated_ever_fraction"] = float(self._seated_ever[held].float().mean())
        log["Insert/touched_fraction"] = float(ly.touched[held].float().mean())
        log["Insert/done_fraction"] = float((ly.phase[held] == DONE).float().mean())
        log["Insert/searching_fraction"] = float((ly.phase[held] == SEARCH).float().mean())
        log["Insert/peak_n"] = float(lk.peak_n[held].median())
        log["Insert/damaged_fraction"] = float(self._damaged[held].float().mean())
        log["Insert/lost_fraction"] = float(lk.lost[held].float().mean())
        steps = self._press_steps[held]
        if bool((steps > 0).any()):
            log["Insert/press_n"] = float((self._press_sum[held] / steps.clamp(min=1.0))[steps > 0].median())
        log["Insert/err_mm"] = 1000.0 * float(self._hold_err[held].median())
        log["Insert/tilt_stopped_fraction"] = float(self._tilt_stopped[held].float().mean())
        log["Insert/ik_fail_fraction"] = float((self._ik_fail[held] / self._ik_calls[held].clamp(min=1.0)).mean())


def _pose_error(kin, q: torch.Tensor, p_goal: torch.Tensor, r_goal: torch.Tensor) -> torch.Tensor:
    """The jaw centre's position error (m) and the small-angle rotation error (rad), goal less where `q` puts it: (k, 6)."""
    r, p = kin.tool(q)
    re = r_goal @ r.transpose(1, 2)
    w = 0.5 * torch.stack((re[:, 2, 1] - re[:, 1, 2], re[:, 0, 2] - re[:, 2, 0], re[:, 1, 0] - re[:, 0, 1]), -1)
    return torch.cat((p_goal - p, w), -1)
