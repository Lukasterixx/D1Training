"""The key hold as one sequence (Lukas, 2026-11-10): the key drawn from the holster and put in the raised box's lock by the
classical arm, then the trained policy takes over and the front-right foot turns the key while the arm holds it.

On top of `unifp_train.keyhold_env.Go2D1KeyHoldEnv` (the training environment, `run_unifp_train.py play --key_hold
--key_hold_staged`), each episode runs these stages before the policy:

  settle, turn, reach, lower   the arm unfolds from rest to the holster's waiting point in joint space
                               (`holster.draw_plan`'s three goals, each joint paced as the dog program paces it,
                               `holster.joint_move`)
  on, close                    the IK slides the open jaws onto the key's root along its axis; they close
  draw, exit, clear            the key drawn out along the holster's axis, out through its mouth, and lifted clear
  to_lock                      a joint move to the hand 25 mm out on the keyway's axis, level and rolled as the hold is
  insert, seat                 the IK drives the key in along the axis to seated at 10 mm/s; held 0.5 s
  policy                       the hold starts as a training episode does (the reference re-solved from the base's
                               pose with its sag, the grip spring on, the policy's history empty); the foot waits the
                               frozen start delay, then the task layer runs the turn

Through the classical stages the legs stand at UniFP's default targets (zero actions), and the arm's commands go out at
10 Hz with the sag compensated, as the training's `hold_arm="ik"` does. The stance the policy then gets is the one
its episodes started from: `keyhold_settled_arm_out.json` was measured that way, zero actions with the arm out.

Drawn, not simulated (markers): the box (its outline from `demos/combiner/geometry`, on a plinth to the raised height),
the lock adapter, the holster (its boxes, `holster.HOLSTER_BOXES`) and the key (the plant's bar and platform): in the
holster, in the hand from when the jaws close, at the plant's angle once seated. The draw's and the insertion's contacts
are not re-tested here; they are the earlier demos' (F-189, F-208, F-164). The key the plant turns has v4's platform;
the holster's v5 key has none.
"""
from __future__ import annotations

import math

import numpy as np
import torch

from isaaclab.utils import configclass

from unifp_isaaclab import interface
from unifp_train import handle, leg_cfg
from unifp_train.keyhold import bar_at, hand_rotation
from unifp_train.keyhold_env import Go2D1KeyHoldEnv, Go2D1KeyHoldEnvCfg
from unifp_train.leg_env import WAITING

from . import holster

#: Stage names, in order, and the fixed lengths (s) of the ones that are not joint moves (the joint moves take as long as
#: `holster.joint_move` says). The IK lines' lengths are the holster demo's defaults at 2x (`holster_env`).
STAGES = ("settle", "turn", "reach", "lower", "pre", "on", "close", "draw", "exit", "clear", "to_lock", "insert", "seat",
          "policy")
FIXED_S = {"settle": 1.0, "pre": 0.75, "on": 1.5, "close": 1.0, "draw": 2.5, "exit": 0.75, "clear": 1.0, "seat": 0.5}
JOINT_MOVES = ("turn", "reach", "lower", "to_lock")
#: The insertion: from this far out along the keyway's axis to seated, at this speed.
INSERT_FROM_M = 0.025
INSERT_SPEED_M_S = 0.010
#: The jaws' travel per finger, m: open on the way onto the bar, closed on it (`env.JAW_OPEN_M`, `JAW_HOLD_M`).
JAW_OPEN_M, JAW_HOLD_M = 0.012, 0.002
#: The arm's command rate through the classical stages (the D1's).
COMMAND_EVERY_STEPS = 5
BOX_COLOUR, PLINTH_COLOUR, HOLSTER_COLOUR, KEY_COLOUR = (0.73, 0.75, 0.76), (0.35, 0.33, 0.30), (0.15, 0.35, 0.75), (1.0, 0.5, 0.05)


def visual_box_usd() -> str:
    """The d1dogdemo's combiner box model (`demos/combiner/asset.build_combiner_usd`, as the key demos build it), with its
    physics taken out: drawn only, placed each episode on the lock the plant turns. Built once into `generated/`."""
    import dataclasses
    import hashlib
    import json
    from pathlib import Path

    import demos.combiner.asset as box_asset
    from demos.combiner.geometry import DEFAULT_HANDLE_TORQUE_NM, GEOMETRY

    root = Path(__file__).resolve().parents[2]
    built_from = json.dumps(dataclasses.asdict(GEOMETRY), sort_keys=True).encode() + Path(box_asset.__file__).read_bytes()
    key = hashlib.sha256(built_from).hexdigest()[:12]
    src = root / "generated/unifp_demos" / f"combiner_{DEFAULT_HANDLE_TORQUE_NM:.3f}_{key}.usda"
    dst = root / "generated/combiner_key" / f"combiner_visual_{key}.usda"
    if dst.exists():
        return str(dst)
    if not src.exists():
        box_asset.build_combiner_usd(src, handle_torque_nm=DEFAULT_HANDLE_TORQUE_NM)
    from pxr import Usd, UsdPhysics

    stage = Usd.Stage.Open(str(src))
    apis = (UsdPhysics.RigidBodyAPI, UsdPhysics.CollisionAPI, UsdPhysics.MassAPI, UsdPhysics.ArticulationRootAPI,
            UsdPhysics.MeshCollisionAPI)
    for prim in list(stage.Traverse()):
        if prim.IsA(UsdPhysics.Joint):
            prim.SetActive(False)
            continue
        for api in apis:
            if prim.HasAPI(api):
                prim.RemoveAPI(api)
        for name in list(prim.GetAppliedSchemas()):
            if name.startswith("Physx") or name.startswith("Physics"):
                prim.RemoveAppliedSchema(name)
    dst.parent.mkdir(parents=True, exist_ok=True)
    stage.Export(str(dst))
    return str(dst)


@configclass
class Go2D1KeyHoldStagedEnvCfg(Go2D1KeyHoldEnvCfg):
    #: Long enough for the classical stages (~20 s) and the turn (~12 s).
    episode_length_s = 40.0
    #: Draw the d1dogdemo's combiner box model (physics stripped) on the lock; else a plain block of its outline.
    box_model: bool = True


def _frame_quat(x, y, z):
    from unifp_train.leg_vis import _frame_quat as fq

    return fq(x, y, z)


class Go2D1KeyHoldStagedEnv(Go2D1KeyHoldEnv):
    cfg: Go2D1KeyHoldStagedEnvCfg

    def __init__(self, cfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, device = self.num_envs, self.device
        plan = holster.draw_plan()
        self._plan = plan
        self._stage_i = torch.zeros(n, dtype=torch.long, device=device)
        self._stage_t = torch.zeros(n, dtype=torch.long, device=device)
        self._stage_len = torch.zeros(n, dtype=torch.long, device=device)
        self._q_geo = torch.zeros(n, 6, device=device)
        self._q_cmd = torch.zeros(n, 6, device=device)
        self._jaw = torch.full((n,), JAW_OPEN_M, device=device)
        self._move = [None] * n                  # the joint move under way: (k, 6) numpy at 10 Hz
        self._line = [None] * n                  # the IK line under way: (a, b, frame) with frame "base" or "env"
        self._key_in = torch.zeros(n, dtype=torch.long, device=device)   # 0 holster, 1 hand, 2 lock
        self._handover_step = torch.full((n,), -1, dtype=torch.long, device=device)
        rest = holster._rest_q()
        self._rest = torch.as_tensor(rest, dtype=torch.float32, device=device)
        b = lambda v: torch.as_tensor(np.asarray(v, dtype=np.float32), device=device)
        self._h_pts = {k: b(plan[k]) for k in ("pre", "grip", "out", "exit", "clear")}
        self._h_rot = b(plan["rot"])
        self._h_key = b(plan["key"])                 # the holstered key's frame in the base (4x4, mount in)
        frame = holster.holster_in_d1_base().copy()
        from position_only.workspace import MOUNT_B

        frame[:3, 3] += MOUNT_B
        self._holster_frame = b(frame)
        self.stage_log: list[list[tuple[str, float]]] = [[] for _ in range(n)]

    # --- the classical stages ------------------------------------------------------------------------------------------

    def _stage_name(self, i: int) -> str:
        return STAGES[int(self._stage_i[i])]

    def _enter_stage(self, i: int, index: int) -> None:
        """Environment `i` begins stage `index`: plan its joint move or its line."""
        self._stage_i[i], self._stage_t[i] = index, 0
        name = STAGES[index]
        self.stage_log[i].append((name, float(self._since_reset[i]) * interface.POLICY_DT))
        q_now = self._q_geo[i].cpu().numpy()
        goals = {"turn": self._plan["turned"], "reach": self._plan["above"], "lower": self._plan["q_pre"]}
        if name in goals or name == "to_lock":
            goal = goals[name] if name in goals else self._lock_q(i, INSERT_FROM_M)
            path = holster.joint_move(q_now, goal)
            self._move[i] = path
            self._stage_len[i] = len(path) * COMMAND_EVERY_STEPS
        elif name in ("on", "draw", "exit", "clear"):
            a = {"on": "pre", "draw": "grip", "exit": "out", "clear": "exit"}[name]
            b = {"on": "grip", "draw": "out", "exit": "exit", "clear": "clear"}[name]
            self._line[i] = (self._h_pts[a], self._h_pts[b], "base")
            self._stage_len[i] = round(FIXED_S[name] / interface.POLICY_DT)
        elif name == "insert":
            g, a = self._grip_w[i], self._approach_w[i]
            self._line[i] = (g - INSERT_FROM_M * a, g, "env")
            self._stage_len[i] = round(INSERT_FROM_M / INSERT_SPEED_M_S / interface.POLICY_DT)
        elif name == "policy":
            self._handover(i)
            self._stage_len[i] = 10 ** 9
        else:
            self._stage_len[i] = round(FIXED_S[name] / interface.POLICY_DT)
        if name == "close":
            self._jaw[i] = JAW_HOLD_M
            self._key_in[i] = 1
        if name in ("turn", "on"):
            self._jaw[i] = JAW_OPEN_M

    def _lock_q(self, i: int, out_m: float) -> np.ndarray:
        """The joints with the key `out_m` short of seated on the keyway's axis, from the base's pose now."""
        ids = torch.tensor([i], device=self.device)
        point = self._grip_w[ids] - out_m * self._approach_w[ids]
        rot = hand_rotation(self._approach_w[ids], self._bar0_w[ids])
        p_b, r_b = self._in_base(point, rot, ids)
        q, ok = self._kin.track(p_b.cpu(), r_b.cpu(), torch.as_tensor(self._plan["q_pre"], dtype=torch.float32)[None],
                                self._kin.soft_lower, self._kin.soft_upper, iterations=12)
        if not bool(ok[0]):
            q, ok = self._kin.inverse(p_b.cpu(), r_b.cpu(), self._kin_seeds, soft=True)
        return q[0].numpy()

    def _command(self, i: int) -> None:
        """Environment `i`'s 10 Hz arm command in its classical stage."""
        name, t = self._stage_name(i), int(self._stage_t[i])
        if name in JOINT_MOVES and self._move[i] is not None:
            k = min(t // COMMAND_EVERY_STEPS, len(self._move[i]) - 1)
            self._q_geo[i] = torch.as_tensor(self._move[i][k], dtype=torch.float32, device=self.device)
        elif self._line[i] is not None and name in ("on", "draw", "exit", "clear", "insert"):
            a, b, frame = self._line[i]
            u = min(1.0, (t + COMMAND_EVERY_STEPS) / max(1, int(self._stage_len[i])))
            u = u * u * (3 - 2 * u)
            point = (a + u * (b - a)).unsqueeze(0)
            ids = torch.tensor([i], device=self.device)
            if frame == "base":
                p_b, r_b = point, self._h_rot.unsqueeze(0)
            else:
                rot = hand_rotation(self._approach_w[ids], self._bar0_w[ids])
                p_b, r_b = self._in_base(point, rot, ids)
            q, ok = self._kin.track(p_b.cpu(), r_b.cpu(), self._q_geo[ids].cpu(), self._kin.soft_lower,
                                    self._kin.soft_upper, iterations=6)
            if bool(ok[0]):
                self._q_geo[i] = q[0].to(self.device)
        ids = torch.tensor([i], device=self.device)
        self._q_cmd[i] = self._q_geo[i] + self._sag(self._q_geo[ids], ids)[0]

    def _handover(self, i: int) -> None:
        """The policy takes over environment `i`: as a training episode starts, from where the arm put the key."""
        ids = torch.tensor([i], device=self.device)
        q, ok = self._solve_ref(ids)
        if not bool(ok[0]):
            q = self._q_geo[ids]
        self._q_ref[ids] = q + self._sag(q, ids)
        self._ik_ref[ids], self._ik_geo[ids] = self._q_ref[ids], q
        self._hold_active[ids] = True
        self._holding[ids] = True
        self._arm_pose[ids] = q
        self._key_in[i] = 2
        self._handover_step[i] = self._since_reset[i]
        # The foot's task from its frozen start delay, counted from now; the policy's history empty.
        self._stage[ids] = WAITING
        self._start_delay[ids] = self._since_reset[ids] + self.cfg.eval_start_delay_steps
        self._actor_history.reset(ids)
        self._critic_history.reset(ids)
        self._actions[ids] = 0.0
        for buf in (self._hold_err_sum, self._hold_err_max, self._hold_steps, self._grip_peak):
            buf[ids] = 0.0

    # --- the step ------------------------------------------------------------------------------------------------------

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        live = self._stage_i == STAGES.index("policy")
        if bool(live.all()):
            return super()._pre_physics_step(actions)
        # Advance each classical environment's stage, and command its arm at 10 Hz.
        for i in torch.nonzero(~live).flatten().tolist():
            if int(self._stage_t[i]) >= int(self._stage_len[i]):
                self._enter_stage(i, int(self._stage_i[i]) + 1)
                if self._stage_name(i) == "policy":
                    continue
            if int(self._stage_t[i]) % COMMAND_EVERY_STEPS == 0:
                self._command(i)
            self._stage_t[i] += 1
        super()._pre_physics_step(actions)

    def _finish_targets(self) -> None:
        # The classical environments: legs at their default targets, the arm on its 10 Hz command, the jaws as staged;
        # then the link (if any) gets what was decided, as the policy's targets do.
        live = self._stage_i == STAGES.index("policy")
        classical = (~live).unsqueeze(-1)
        self._actions = torch.where(classical, torch.zeros_like(self._actions), self._actions)
        legs = interface.joint_targets(torch.zeros_like(self._actions))[:, :12]
        self._joint_targets[:, :12] = torch.where(classical, legs, self._joint_targets[:, :12])
        self._joint_targets[:, 12:18] = torch.where(classical, self._q_cmd, self._joint_targets[:, 12:18])
        jaw = self._jaw.unsqueeze(-1).expand_as(self._jaw_targets)
        self._jaw_targets = torch.where(classical, jaw, torch.full_like(self._jaw_targets, JAW_HOLD_M))
        super()._finish_targets()

    def _advance_approach(self) -> None:
        # The foot's approach waits until the policy has the robot.
        policy = self._stage_i == STAGES.index("policy")
        self._start_delay = torch.where(policy, self._start_delay, self._since_reset + 10 ** 6)
        super()._advance_approach()

    def _reset_idx(self, env_ids):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        super()._reset_idx(env_ids)
        # The arm starts folded at rest, the hold not yet on; the key in the holster.
        count = len(env_ids)
        self._hold_active[env_ids] = False
        self._holding[env_ids] = False
        joint_pos = self._robot.data.joint_pos[env_ids].clone()
        joint_vel = self._robot.data.joint_vel[env_ids].clone()
        joint_pos[:, self._order[12:18]] = self._rest.expand(count, 6)
        joint_vel[:, self._order[12:18]] = 0.0
        self._robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
        if self._arm_link is not None:
            self._arm_link.reset(env_ids, self._rest.expand(count, 6))
        self._arm_pose[env_ids] = self._rest.expand(count, 6)
        self._q_geo[env_ids] = self._rest.expand(count, 6)
        self._q_cmd[env_ids] = self._rest.expand(count, 6) + self._sag(self._rest.expand(count, 6), env_ids)
        self._jaw[env_ids] = JAW_OPEN_M
        self._key_in[env_ids] = 0
        self._handover_step[env_ids] = -1
        self._start_delay[env_ids] = 10 ** 6
        for i in env_ids.tolist():
            self._move[i], self._line[i] = None, None
            self.stage_log[i] = []
            self._stage_i[i], self._stage_t[i] = 0, 0
            self._stage_len[i] = round(FIXED_S["settle"] / interface.POLICY_DT)
        self._place_box_models()

    def _update_hold(self) -> None:
        super()._update_hold()

    # --- the drawing ---------------------------------------------------------------------------------------------------

    def _setup_scene(self):
        super()._setup_scene()
        self._box_paths = []
        if not self.cfg.box_model:
            return
        import isaaclab.sim as sim_utils

        usd = visual_box_usd()
        for i in range(self.scene.num_envs):
            path = f"/World/envs/env_{i}/KeyholdBox"
            cfg = sim_utils.UsdFileCfg(usd_path=usd)
            cfg.func(path, cfg, translation=(0.0, 0.0, -5.0))
            self._box_paths.append(path)
        self._box_placed = [None] * self.scene.num_envs

    def _place_box_models(self) -> None:
        """Each environment's box model on its lock (world frame), where it moved since it was last placed."""
        if not getattr(self, "_box_paths", None):
            return
        from demos.combiner.geometry import GEOMETRY
        from pxr import Gf
        import isaacsim.core.utils.stage as stage_utils

        stage = stage_utils.get_current_stage()
        hd = self._handle
        door = (hd.door + self.scene.env_origins).cpu().numpy()
        n = hd.n.cpu().numpy()
        for i, path in enumerate(self._box_paths):
            key = (tuple(np.round(door[i], 5)), tuple(np.round(n[i], 5)))
            if self._box_placed[i] == key:
                continue
            out = n[i] / max(1e-9, np.linalg.norm(n[i]))
            right = np.array([-out[1], out[0], 0.0])
            rot = np.column_stack([out, right, [0.0, 0.0, 1.0]])
            root = door[i] - rot @ np.asarray(GEOMETRY.lock, dtype=float)
            yaw = math.atan2(rot[1, 0], rot[0, 0])
            prim = stage.GetPrimAtPath(path)
            translate, orient = prim.GetAttribute("xformOp:translate"), prim.GetAttribute("xformOp:orient")
            translate.Set(Gf.Vec3d(*root.tolist()))
            q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
            quat = Gf.Quatd(q[0], Gf.Vec3d(*q[1:])) if isinstance(orient.Get(), Gf.Quatd) else Gf.Quatf(q[0], Gf.Vec3f(*q[1:]))
            orient.Set(quat)
            self._box_placed[i] = key

    def _set_debug_vis_impl(self, enable: bool) -> None:
        super()._set_debug_vis_impl(enable)
        if enable and not hasattr(self, "_stage_markers"):
            import isaaclab.sim as sim_utils
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

            block = lambda c: sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0),
                                                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=c))
            self._stage_markers = VisualizationMarkers(VisualizationMarkersCfg(
                prim_path="/Visuals/keyhold_staged",
                markers={"box": block(BOX_COLOUR), "plinth": block(PLINTH_COLOUR), "holster": block(HOLSTER_COLOUR),
                         "key": block(KEY_COLOUR)}))
        if hasattr(self, "_stage_markers"):
            self._stage_markers.set_visibility(enable)

    def _debug_vis_callback(self, event) -> None:
        super()._debug_vis_callback(event)
        if not hasattr(self, "_stage_markers"):
            return
        if hasattr(self, "_handle_markers"):
            self._handle_markers.set_visibility(False)          # the box, adapter and key are drawn here instead
        self._place_box_models()
        from demos.combiner.geometry import GEOMETRY

        n, device = self.num_envs, self.device
        origins = self.scene.env_origins
        rows_t, rows_q, rows_s, rows_i = [], [], [], []

        def add(centre, x, y, z, size, kind):
            rows_t.append(centre + origins)
            rows_q.append(_frame_quat(x, y, z))
            rows_s.append(torch.as_tensor(size, dtype=torch.float32, device=device).expand(n, 3))
            rows_i.append(torch.full((n,), kind, dtype=torch.long, device=device))

        # The box behind the door's face, and its plinth, on the lock as the plant has it.
        hd = self._handle
        out, right = hd.n, torch.stack((-hd.n[:, 1], hd.n[:, 0], torch.zeros_like(hd.n[:, 0])), dim=-1)
        right = right / right.norm(dim=-1, keepdim=True).clamp(min=1e-6)
        up = torch.zeros_like(out)
        up[:, 2] = 1.0
        lock_y = -GEOMETRY.hinge_side * (GEOMETRY.width / 2 - GEOMETRY.lock_from_edge)
        middle = hd.door - lock_y * right                  # the door's middle, at the lock's height
        bottom = hd.door[:, 2:3] - (GEOMETRY.height / 2)
        centre = middle - 0.5 * GEOMETRY.depth * out
        centre = torch.cat((centre[:, :2], bottom + GEOMETRY.height / 2), dim=-1)
        block = (GEOMETRY.depth, GEOMETRY.width, GEOMETRY.height) if not getattr(self, "_box_paths", None) else (1e-6,) * 3
        add(centre, out, right, up, block, 0)
        plinth_h = (bottom.squeeze(-1)).clamp(min=1e-3)
        plinth = torch.cat((centre[:, :2], (0.5 * plinth_h).unsqueeze(-1)), dim=-1)
        rows_t.append(plinth + origins)
        rows_q.append(_frame_quat(out, right, up))
        rows_s.append(torch.stack((torch.full_like(plinth_h, GEOMETRY.depth), torch.full_like(plinth_h, GEOMETRY.width),
                                   plinth_h), dim=-1))
        rows_i.append(torch.full((n,), 1, dtype=torch.long, device=device))
        # The lock adapter on the door (`leg_vis.ADAPTER_SIZE_M`, the key's face off it).
        face = leg_cfg.KEY_ADAPTER_FACE_M
        add(hd.door + 0.5 * face * out, right, up, out, (0.060, 0.060, face), 2)
        # The holster's boxes, on the base.
        base_p = self._robot.data.root_pos_w - origins
        base_q = self._robot.data.root_quat_w
        frame = self._holster_frame
        for (lo, hi) in holster.HOLSTER_BOXES.values():
            lo_t, hi_t = torch.tensor(lo, device=device), torch.tensor(hi, device=device)
            c_h = frame[:3, :3] @ (0.5 * (lo_t + hi_t)) + frame[:3, 3]
            c_w = base_p + interface.quat_apply(base_q, c_h.expand(n, 3))
            axes = [interface.quat_apply(base_q, frame[:3, k].expand(n, 3)) for k in range(3)]
            add(c_w, axes[0], axes[1], axes[2], (hi_t - lo_t).clamp(min=1e-3).tolist(), 2)
        # The key: in the holster, in the hand, or at the plant's angle in the lock. (origin, u, n, t)
        k_h = self._h_key
        o_h = base_p + interface.quat_apply(base_q, k_h[:3, 3].expand(n, 3))
        n_h = interface.quat_apply(base_q, k_h[:3, 0].expand(n, 3))
        u_h = interface.quat_apply(base_q, k_h[:3, 1].expand(n, 3))
        t_h = interface.quat_apply(base_q, k_h[:3, 2].expand(n, 3))
        q6 = self._robot.data.body_quat_w[:, self._link6]
        p6 = self._robot.data.body_pos_w[:, self._link6] - origins
        ex = lambda v: interface.quat_apply(q6, torch.tensor(v, dtype=torch.float32, device=device).expand(n, 3))
        o_g, u_g, n_g, t_g = p6 + ex((0.0, 0.0, 0.1266)), ex((1.0, 0.0, 0.0)), -ex((0.0, 0.0, 1.0)), -ex((0.0, 1.0, 0.0))
        u_l = handle.lever_dir(hd.u0, hd.theta)
        t_l = handle.lever_top(hd.u0, hd.theta)
        o_l = hd.door + leg_cfg.KEY_ADAPTER_FACE_M * hd.n
        where = lambda a, b, c: torch.where((self._key_in == 0).unsqueeze(-1), a,
                                            torch.where((self._key_in == 1).unsqueeze(-1), b, c))
        o, u, nn, t = where(o_h, o_g, o_l), where(u_h, u_g, u_l), where(n_h, n_g, hd.n), where(t_h, t_g, t_l)
        for box in (((-0.015, 0.160), (0.0, 0.015), (-0.010, 0.010)), ((0.048, 0.148), (0.015, 0.050), (0.004, 0.010))):
            (a0, a1), (b0, b1), (c0, c1) = box
            centre = o + 0.5 * (a0 + a1) * u + 0.5 * (b0 + b1) * nn + 0.5 * (c0 + c1) * t
            add(centre, u, t, nn, (a1 - a0, c1 - c0, b1 - b0), 3)
        self._stage_markers.visualize(translations=torch.cat(rows_t), orientations=torch.cat(rows_q),
                                      scales=torch.cat(rows_s), marker_indices=torch.cat(rows_i))
