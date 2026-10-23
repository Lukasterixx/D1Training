"""The combiner key demo's environment: the combiner door demo's robot (P2Dingo's walking policy, arm IK at 10 Hz, the
head RealSense and the MID-360 finding the box) inserting Lukas's lever key into the box's cam lock with the arm, then
turning it with the front-right foot under the key-trained leg policy (`unifp_train --task leg --leg_object key`).

On top of `demos.combiner_door.env.CombinerDoorEnv`:
- **The box stands on the floor**, as the real one does, not on the UniFP demos' plinth (that keeps their handle at the
  height their reach was tuned for; the leg policy trained with the lock at its real 0.4175 m). `BoxSite.root_m` is put
  at z 0 for this process, and the plinth is moved aside.
- **The key** (`asset.build_key_usd`) is a free rigid body. Held, it hangs from the jaw centre on a 6-axis spring
  (`GRIP_*`): a compliant grip, the PhysX form of the insertion training's (`insertion.CompliantGrip`); the arm feels its
  reaction. Let go, the spring is off and the key stays where PhysX leaves it. Before the arm has unfolded it is parked
  out of the scene (`parked_key`); it appears in the hand at the unfold's end.
- **The lock adapter** (`asset.build_adapter_usd`) is a kinematic block over the cam lock at the box's true pose, its
  keyway at the insertion angle.
- **The insertion is not learnt**: the jaw centre is put by the arm IK at a pre-insertion point 15 mm short of the
  adapter's face along the estimated keyway axis, the key rolled to 45 degrees, then driven in along that axis to 4 mm
  past seated, so the spring presses the key on; the jaws open, the hand backs straight out along the approach and the
  arm folds to rest. The box's place is the look's estimate; nothing else is told to the arm.
- **The turn is the leg policy's**, in the `turn` phase: the legs switch from the walking policy to it, and its task
  layer, outer loop and approach run as in training -- `LegTurn` runs `unifp_train.leg_env.Go2D1LegEnv`'s own methods
  on its own state, so they are the trained ones, not copies. The key's turn is the training's analytic plant (its
  stick-slip lock, its contact with the foot), placed at the key's true pose; the estimate the layer turns it from is
  the look's. PhysX draws the key where the plant has it. Then the walking policy takes the legs back.
- **The door unlocks** when the plant's key passes 30 degrees down: the box's latch (`demos.combiner.latch`) is released.
  The robot's own check is the turn as its foot measured it (`HandleTaskLayer.theta_meas` at the end of the hold).

Mismatches that belong to the stack, and are in the record: the leg policy trained on UniFP's ideal PD legs with
self-collision on; here the legs are the decoupled stack's DCMotor (the walking policy's, the same 25 / 0.5 gains) with
self-collision off. The leg policy trained with the wrist RealSense's mass on the arm; here the camera is on the head,
drawn only.

Needs the Isaac app: import after `AppLauncher`.
"""
from __future__ import annotations

import math

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObject, RigidObjectCfg

import d1_hardware
import d1_ik
from demos.combiner.geometry import GEOMETRY
from demos.combiner_door import env as door_env
from demos.combiner_door import perception
from demos.unifp import props, script as demo_script
from unifp_isaaclab import interface
from unifp_isaaclab.policy import ObsHistory
from unifp_train import foot_goals, handle, insertion, leg_cfg
from unifp_train.leg_env import ENGAGED, FOOT_DOWN, MISSED, WAITING, Go2D1LegEnv

from . import asset, geometry as kg

# --- the box on the floor ------------------------------------------------------------------------

_PLINTH_ROOT = props.BoxSite.root_m
#: The box's root height in this demo: on the floor.
FLOOR_ROOT_Z_M = 0.0
props.BoxSite.root_m = property(lambda site: (*_PLINTH_ROOT.fget(site)[:2], FLOOR_ROOT_Z_M))

# --- the grip, the phases, the turn --------------------------------------------------------------

#: The spring grip: N/m and N·s/m along, N·m/rad and N·m·s/rad about, the jaw centre's frame, capped (N, N·m) so a key
#: that jams pushes the robot no harder than a stalled insertion should. At 800 N/m and 0.8 N·m/rad (the first build,
#: 2026-10-21) the key pivoted in the hand instead of sliding down the lead-in: its shaft's tip meets the adapter 31 mm
#: ahead of the jaw centre and 25 mm to its side, and 2 N there turned it ~4 degrees. A hand holding a printed bar is
#: stiffer than that.
GRIP_K_N_M = 600.0
GRIP_C_N_S_M = 5.0
GRIP_K_NM_RAD = 5.0
GRIP_C_NM_S_RAD = 0.08
GRIP_MAX_N = 10.0
GRIP_MAX_NM = 0.3
#: The key's centre of mass in its prim frame (x off the door, y along it, z up its top side), and its mass, from the
#: STL; its inertia raised from the STL's ~1e-4 kg·m^2 so the grip's explicit rotational spring stays stable at the 5 ms
#: step (sqrt(k / I) x dt = 0.5). Not its mass: at 0.15 kg (one trial, 2026-10-21) the D1 holding it 0.45 m out sagged
#: 22 degrees off its commanded approach -- the shoulder's 3.33 N·m is spent on the arm itself.
KEY_SIM_MASS_KG = kg.KEY_MASS_KG
KEY_COM_M = (0.0135, 0.069, 0.002)
KEY_INERTIA_KG_M2 = (1.2e-3, 5e-4, 1.2e-3)
#: Where the parked key waits, above its environment.
PARK_Z_M = 6.0
#: The jaw travel holding the 20 mm bar (17.2 mm + 2 x travel: 21.2 mm), and opened to let go (41.2 mm).
JAW_HOLD_M = 0.002
JAW_OPEN_M = 0.012
#: How far past seated the jaw centre is driven, so the grip's spring presses the key onto the adapter (~2.4 N, more or
#: less by the look's error in depth). 15 mm (tried once, 2026-10-21, so that the spring would saturate at its cap
#: whatever the depth error) put the IK's target out of what it solves: it held, and the hand drifted 4-14 degrees.
OVERDRIVE_M = 0.004
#: The look's depth from the lidar as well as its heading. Off: the tag's range is the look's worst error (3-11 mm
#: along the door's normal in the first camera runs, every lateral error under 0.6 mm), but the lidar's face fit put
#: the face a near-constant 1.8-2.6 mm in front of the tag's estimate whatever that error was (`mid360.FaceFit.depth`,
#: one trial of 4), so it does not measure it.
LIDAR_DEPTH = False
#: The pre-insertion stand-off of the key's back face from the adapter's face: its shaft's tip 15 mm short.
PRE_INSERT_M = 0.025
#: Seated: the bar's back face within this of the adapter's face, along the axis, and the shaft inside the channel.
SEATED_M = 0.0015
#: How far the hand backs out along its approach after letting go.
BACK_OUT_M = 0.06
#: The walks: each axis of the base's frame driven on its own at least this fast while it is more than
#: `WALK_ARRIVED_M` out, up to the cap (P2Dingo's clocked gait stands inside 0.1 m/s on each axis, so a diagonal
#: command of 0.15 m/s whose forward share was 0.07 walked sideways only, then crept: 19 cm short after 8 s).
WALK_SPEED_M_S = 0.15
WALK_MAX_M_S = 0.22
WALK_GAIN = 1.5
WALK_ARRIVED_M = 0.012
WALK_YAW_GAIN = 1.0
#: The leg phase's limit, s: its approach, turn, hold, release and return run 11-17 s in training.
TURN_S = 20.0
#: Before the leg policy acts: the legs' targets eased, over this long, from the walking policy's last action (held) to
#: UniFP's default targets (zero actions), so the robot sinks from the walking policy's stand (base ~0.31 m) to the one
#: the leg policy trained from (settled, 0.272 m, the knees bent by gravity); then a short hold, and the policy starts
#: with its history empty, as at a training reset. Switched at once (the first build), the robot dropped to 0.24 m and
#: lurched 22 cm.
CROUCH_S = 1.5
CROUCH_HOLD_S = 0.5
#: The key's lock in the demo (one of the frozen set's middle ones unless given): friction and cam, N·m.
LOCK_FRICTION_NM = 1.5
LOCK_DETENT_NM = 0.5

#: The hand's outer loop through the insertion: the jaw centre as the arm's encoders put it (here Link6's pose, which
#: forward kinematics of the measured joints gives on the real arm) against where it is sent, the miss integrated into
#: the command at this gain (1/s), capped. Only across the keyway's axis once the key is going in, so it never pushes the
#: key harder into a face it has met. The D1 holding the key forward and below its mount sags ~16 mm under gravity
#: (2026-10-21, `--given_box`: the hand 28 mm out at the pre-insertion point, 1.5 degrees off, 16 mm low).
HAND_LOOP_GAIN = 1.5
HAND_LOOP_MAX_M = 0.03
INSERT_PHASES = {"pre", "align", "insert", "seat"}
#: Not while the arm is still on its way to the pre-insertion point: its lag then is not a sag, and integrating it
#: winds the offset up.
INTEGRATE = {"align", "insert", "seat"}
ACROSS_ONLY = {"insert", "seat"}

WALK_PHASES = {"walk_to_insert", "walk_to_turn", "walk_to_key"}
TURN_PHASES = {"turn"}
#: Phases the arm holds still through (not commanded): folded before the unfold and after the fold, and with the key
#: already in, folded throughout. Through the walk to the insertion it holds the home point relative to the base.
PARKED = {"spawn", "look", "no_tag", "walk_to_key", "walk_to_turn", "settle_turn", "turn", "stand"}
POSITION_ONLY = {"walk_to_insert", "settle_insert"}


def key_phases(inserted: bool) -> list[demo_script.Phase]:
    """The attempt after the spawn's settle. Inserting: look, unfold, walk to the insertion stance, insert, let go,
    back out, fold, walk to the turn stance, turn, stand. With the key already in: look, walk to the turn stance, turn,
    stand -- the arm folded throughout. Where the hand goes in each insertion phase is `CombinerKeyEnv._key_point`."""
    hold, open_ = JAW_HOLD_M, JAW_OPEN_M
    home = lambda st, u: door_env.home_at(st["base"])
    if inserted:
        return [demo_script.Phase("look", door_env.LOOK_S, path=home, jaw_m=hold),
                demo_script.Phase("walk_to_key", 12.0, path=home, jaw_m=hold),
                demo_script.Phase("settle_turn", 1.5, path=home, jaw_m=hold),
                demo_script.Phase("turn", TURN_S, path=home, jaw_m=hold),
                demo_script.Phase("stand", 1.5, path=home, jaw_m=hold)]
    key = lambda name: (lambda st, u: st["key_point"](name, u))
    axis = lambda st: st["key_axes"][0]
    bar = lambda st: st["key_axes"][1]
    insert = lambda name, seconds, jaws=hold: demo_script.Phase(name, seconds, path=key(name), jaw_m=jaws, approach=axis,
                                                                object_axis=bar, measure=name == "seat")
    return [demo_script.Phase("look", door_env.LOOK_S, path=home, jaw_m=hold),
            demo_script.Phase("unfold", door_env.UNFOLD_S, path=home, jaw_m=hold),
            demo_script.Phase("walk_to_insert", 10.0, path=home, jaw_m=hold),
            demo_script.Phase("settle_insert", 1.5, path=home, jaw_m=hold),
            insert("pre", 2.5), insert("align", 2.0), insert("insert", 4.0), insert("seat", 1.0),
            insert("release", 0.6, open_), insert("back", 1.5, open_),
            demo_script.Phase("fold", door_env.UNFOLD_S, path=home, jaw_m=open_),
            demo_script.Phase("walk_to_turn", 6.0, path=home, jaw_m=open_),
            demo_script.Phase("settle_turn", 1.5, path=home, jaw_m=open_),
            demo_script.Phase("turn", TURN_S, path=home, jaw_m=open_),
            demo_script.Phase("stand", 1.5, path=home, jaw_m=open_)]


def floor_site(site: props.BoxSite) -> np.ndarray:
    """A `BoxSite`'s enclosure frame in the script frame, on the floor (4x4)."""
    pose = perception.site_pose(site)
    pose[2, 3] = FLOOR_ROOT_Z_M
    return pose


def _quat_mul(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    aw, ax, ay, az = a.unbind(-1)
    bw, bx, by, bz = b.unbind(-1)
    return torch.stack((aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                        aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw), dim=-1)


def _quat_conj(q: torch.Tensor) -> torch.Tensor:
    return q * torch.tensor([1.0, -1.0, -1.0, -1.0], device=q.device, dtype=q.dtype)


def _rotvec(q: torch.Tensor) -> torch.Tensor:
    """The rotation vector of a unit quaternion (w, x, y, z), the short way round."""
    q = torch.where(q[:, :1] < 0.0, -q, q)
    v = q[:, 1:]
    s = v.norm(dim=-1, keepdim=True).clamp(min=1e-9)
    angle = 2.0 * torch.atan2(s, q[:, :1])
    return v / s * angle


def _matrix_quat(m: np.ndarray) -> np.ndarray:
    """(w, x, y, z) of a 3x3 rotation."""
    w = math.sqrt(max(1e-12, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(1e-12, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(1e-12, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(1e-12, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def key_prim_rotation(theta_deg: float) -> np.ndarray:
    """The key prim's axes (columns: x off the door, y along the lever, z up its top side) in the box frame."""
    u, n, t = kg.key_rotation_box(theta_deg)
    return np.column_stack([n, u, t])


#: The key prim's axes in Link6 (columns), from `geometry.KEY_AXES_LINK6` (rows u, n, t).
KEY_PRIM_IN_LINK6 = np.column_stack([np.asarray(kg.KEY_AXES_LINK6[1]), np.asarray(kg.KEY_AXES_LINK6[0]),
                                     np.asarray(kg.KEY_AXES_LINK6[2])])


class LegTurn:
    """The key turn as the leg task runs it, on demo environments: `Go2D1LegEnv`'s approach, task layer, outer loop,
    goal hold and plant, called on this object (which holds their state and lends them the environment's robot)."""

    _advance_approach = Go2D1LegEnv._advance_approach
    _advance_return = Go2D1LegEnv._advance_return
    _run_layer = Go2D1LegEnv._run_layer
    _correct_foot = Go2D1LegEnv._correct_foot
    _hold_goals = Go2D1LegEnv._hold_goals
    _foot_state = Go2D1LegEnv._foot_state
    _ground_under_base = Go2D1LegEnv._ground_under_base
    _goal_centre = Go2D1LegEnv._goal_centre
    _goal_world = Go2D1LegEnv._goal_world
    _to_sphere = Go2D1LegEnv._to_sphere

    def __init__(self, env, variant: str, checkpoint: str):
        from unifp_train.eval import EvaluablePolicy

        n, device = env.num_envs, env.device
        self.env, self.num_envs, self.device = env, n, device
        self._force_aware = variant == "force"
        self.variant = variant
        self.policy = EvaluablePolicy(checkpoint, device=str(device))
        self._key, self._evaluation = True, {"demo": True}       # a frozen evaluation's rules: no deadline engagement
        self._down_goal = leg_cfg.KEY_DOWN_GOAL
        body = env._robot.body_names.index
        self._foot_body, self._knee_body, self._base_body = body(leg_cfg.FOOT_BODY), body(leg_cfg.KNEE_BODY), body("base")
        self._wrench_bodies = torch.tensor([self._foot_body, self._knee_body, self._base_body], dtype=torch.int32,
                                           device=device)
        self._foot_offset = torch.as_tensor(handle.FOOT_CENTRE_LOCAL_M, device=device).expand(n, 3).contiguous()
        self._head_offset = torch.as_tensor(leg_cfg.HEAD_POINT_B, device=device).expand(n, 3).contiguous()
        self._wrench_positions = torch.zeros(n, 3, 3, device=device)
        self._wrench_positions[:, 0] = self._foot_offset
        self._wrench_positions[:, 2] = self._head_offset
        self._goals = foot_goals.FootGoals(n, device, down_goal=self._down_goal, pitch_range=leg_cfg.KEY_GOAL_PITCH_RANGE)
        self._handle = handle.LeverHandle(n, device)
        self._layer = handle.HandleTaskLayer(n, device, kp=leg_cfg.FORCE_LAW_KP_N_PER_M, ki=leg_cfg.FORCE_LAW_KI_N_PER_M_S)
        self._correction = insertion.PoseCorrection(
            n, device, tip_gain=leg_cfg.FOOT_CORRECTION_GAIN, tip_max_m=leg_cfg.FOOT_CORRECTION_MAX_M,
            elevation_gain=0.0, elevation_max_rad=0.0, roll_gain=0.0, roll_max_rad=0.0)
        zeros = lambda *shape: torch.zeros(*shape, device=device)
        flags = lambda: torch.zeros(n, dtype=torch.bool, device=device)
        self.active = flags()
        self._handle_env = flags()
        self._stage = torch.zeros(n, dtype=torch.long, device=device)
        self._stage_step = torch.zeros(n, dtype=torch.long, device=device)
        self._since_reset = torch.zeros(n, dtype=torch.long, device=device)
        self._start_delay = torch.full((n,), 25, dtype=torch.long, device=device)
        self._est = {name: zeros(n, 3) for name in ("pivot", "u0", "door_n")}
        self._est.update({name: zeros(n) for name in ("reach", "clearance", "theta_end", "max_n", "descent",
                                                      "descend_speed", "turn_speed", "theta_start")})
        self._approach_from, self._waypoint, self._pre = zeros(n, 3), zeros(n, 3), zeros(n, 3)
        self._return_from, self._goal_w_held = zeros(n, 3), zeros(n, 3)
        self._down_for = zeros(n)
        self._engage_step = torch.full((n,), -1, dtype=torch.long, device=device)
        self._engage_miss = torch.full((n,), float("nan"), device=device)
        self._history = ObsHistory(n, device=str(device))
        self._actions = zeros(n, interface.NUM_ACTIONS)
        self.force_w = zeros(n, 3)
        self.knee_force_w, self.head_force_w = zeros(n, 3), zeros(n, 3)
        #: Per environment, for the record: peaks, the latch, the foot's own measure of the turn, the steps it ran.
        self.peak_n, self.max_theta = zeros(n), torch.full((n,), -10.0, device=device)
        self.unlock_step = torch.full((n,), -1, dtype=torch.long, device=device)
        self.held_meas = torch.full((n,), float("nan"), device=device)
        self.cmd_max = zeros(n)
        self.steps = torch.zeros(n, dtype=torch.long, device=device)
        self.crouch = torch.zeros(n, dtype=torch.long, device=device)
        self._handover_action = torch.zeros(n, interface.NUM_ACTIONS, device=device)
        self._handover_set = torch.zeros(n, dtype=torch.bool, device=device)
        #: The robot as the leg policy got it, after the crouch: base height, the worst support foot from its place.
        self.handover: list[dict] = [{} for _ in range(n)]

    # what the borrowed methods read off an environment
    @property
    def _robot(self):
        return self.env._robot

    @property
    def scene(self):
        return self.env.scene

    def _base_yaw_quat(self):
        return self.env._base_yaw_quat()

    def start(self, index: int, door_w: np.ndarray, normal_w: np.ndarray, right_w: np.ndarray, est_door_w: np.ndarray,
              est_normal_w: np.ndarray, est_right_w: np.ndarray, friction: float, detent: float) -> None:
        """Begin the turn in environment `index`: the plant at the key's true pose, the layer told the estimate. Env
        frame, numpy: the shaft's point on the door's face, the door's normal toward the robot, the robot's right."""
        c, device = leg_cfg, self.device
        ids = torch.tensor([index], device=device)
        t = lambda v: torch.as_tensor(np.asarray(v, dtype=np.float32), device=device).reshape(1, -1)
        one = lambda v: torch.full((1,), float(v), device=device)
        self._handle.reset(ids)
        self._layer.reset(ids)
        self._correction.reset(ids)
        self._goals.reset(ids)
        self._handle.place(ids, door=t(door_w), normal=t(normal_w), u0=t(right_w), standoff=one(c.KEY_AIM_OFF_DOOR_M),
                           length=one(c.KEY_BAR_BOX[0][1]), inner=one(0.0), radius=one(c.KEY_TOP_M),
                           inertia=one(np.mean(c.KEY_INERTIA_KG_M2)), rest_nm=one(0.0), full_nm=one(0.0),
                           damping=one(np.mean(c.KEY_DAMPING)), friction=one(friction),
                           theta_max=one(c.KEY_THETA_END_RAD), theta_latch=one(c.KEY_THETA_UNLOCK_RAD),
                           mu=one(np.mean(c.KEY_MU)), theta_min=one(c.KEY_THETA_START_RAD), pad=True,
                           detent_nm=one(detent), detent_from=one(c.KEY_THETA_UNLOCK_RAD - c.KEY_DETENT_SPAN_RAD),
                           gravity_nm=one(c.KEY_GRAVITY_NM))
        e = self._est
        e["pivot"][ids] = t(est_door_w) + c.KEY_AIM_OFF_DOOR_M * t(est_normal_w)
        e["u0"][ids], e["door_n"][ids] = t(est_right_w), t(est_normal_w)
        reach = float(np.mean(c.KEY_REACH_M))
        e["reach"][ids], e["clearance"][ids] = reach, handle.FOOT_RADIUS_M + c.KEY_TOP_M
        e["theta_start"][ids], e["theta_end"][ids] = c.KEY_THETA_START_RAD, c.KEY_THETA_END_RAD + c.OVERDRIVE_RAD
        # Told the lock's resistance to within the training's +-30%: here, exactly the middle of the frozen set's.
        e["max_n"][ids] = float(np.clip(c.MAX_MARGIN * (friction + detent) / reach, *c.KEY_MAX_N_LIMITS))
        e["descent"][ids] = float(np.mean(c.KEY_DESCENT_M))
        e["descend_speed"][ids] = float(np.mean(c.DESCEND_SPEED_M_S))
        e["turn_speed"][ids] = float(np.mean(c.TURN_SPEED_RAD_S))
        start = one(c.KEY_THETA_START_RAD)
        u0 = e["u0"][ids]
        rest = handle.foot_on_bar(e["pivot"][ids], u0, start, e["reach"][ids], e["clearance"][ids])
        self._pre[ids] = rest + e["descent"][ids].unsqueeze(-1) * handle.lever_top(u0, start)
        z = torch.tensor([[0.0, 0.0, 1.0]], device=device)
        self._waypoint[ids] = self._pre[ids] + c.KEY_WAYPOINT_BACK_M * e["door_n"][ids] + c.KEY_WAYPOINT_UP_M * z
        self._handle_env[ids] = True
        self._stage[ids], self._stage_step[ids], self._since_reset[ids] = WAITING, 0, 0
        self._goal_w_held[ids] = 0.0
        self._down_for[ids] = 0.0
        self._engage_step[ids], self._engage_miss[ids] = -1, float("nan")
        self._history.reset(ids)
        self._actions[ids] = 0.0
        self.peak_n[ids], self.max_theta[ids], self.cmd_max[ids] = 0.0, c.KEY_THETA_START_RAD, 0.0
        self.unlock_step[ids], self.held_meas[ids], self.steps[ids] = -1, float("nan"), 0
        self.crouch[ids] = round((CROUCH_S + CROUCH_HOLD_S) / interface.POLICY_DT)
        self._handover_set[ids] = False
        self.handover[index] = {}
        self.active[ids] = True

    def finished(self) -> torch.Tensor:
        """Where the turn is over: the foot down on its footfall for a second, or never engaged."""
        return self.active & ((self._down_for > 1.0) | (self._stage == MISSED))

    def stop(self, index: int) -> None:
        self.active[index] = False
        self._handle_env[index] = False

    def step(self, walk: torch.Tensor | None = None) -> torch.Tensor:
        """One policy step for every active environment: the task logic, then the leg policy's 18-wide action. `walk` is
        the walking policy's action this step, which the crouch starts from."""
        if not bool(self.active.any()):
            return self._actions.clone()
        crouching = self.active & (self.crouch > 0)
        crouch_action = torch.zeros_like(self._actions)
        if bool(crouching.any()):
            if walk is not None:
                first = crouching & ~self._handover_set
                self._handover_action[first] = walk[first]
                self._handover_set |= first
            total = round((CROUCH_S + CROUCH_HOLD_S) / interface.POLICY_DT)
            ease = round(CROUCH_S / interface.POLICY_DT)
            s = ((total - self.crouch).float() / ease).clamp(0.0, 1.0)
            s = s * s * (3 - 2 * s)
            crouch_action = (1.0 - s).unsqueeze(-1) * self._handover_action
            self.crouch -= crouching.long()
            done = torch.nonzero(crouching & (self.crouch == 0)).flatten()
            if len(done):
                self._history.reset(done)
                from unifp_train import leg_rewards
                data = self.env._robot.data
                stance = leg_rewards.stance_error(
                    data.body_pos_w[:, self.env._feet_bodies] - self.scene.env_origins.unsqueeze(1),
                    data.root_pos_w - self.scene.env_origins, self._base_yaw_quat())
                for i in done.tolist():
                    rel = interface.quat_rotate_inverse(self._base_yaw_quat()[i:i + 1],
                                                        (self._handle.door[i:i + 1] - self._ground_under_base()[i:i + 1]))[0]
                    self.handover[i] = {"base_z_m": round(float(data.root_pos_w[i, 2] - self.scene.env_origins[i, 2]), 4),
                                        "stance_max_cm": round(100 * float(stance[i].max()), 2),
                                        "door_ahead_m": round(float(rel[0]), 4), "axis_side_m": round(float(rel[1]), 4)}
        live = self.active & (self.crouch == 0)
        if not bool(live.any()):
            return crouch_action
        self._since_reset += live.long()
        self.steps += self.active.long()
        self._advance_approach()
        self._run_layer()
        self._correct_foot()
        self._hold_goals()
        layer = self._layer
        holding = self.active & (self._stage == ENGAGED) & (layer.phase == handle.HOLDING)
        self.held_meas = torch.where(holding, layer.theta_meas, self.held_meas)
        self.cmd_max = torch.maximum(self.cmd_max, layer.force_cmd.norm(dim=-1) * self.active)
        commands = torch.zeros(self.num_envs, interface.NUM_COMMANDS, device=self.device)
        commands[:, interface.CMD_EE_RADIUS:interface.CMD_EE_YAW + 1] = self._goals.step()
        if self._force_aware:
            commands[:, interface.CMD_EE_FORCE] = interface.quat_rotate_inverse(self._base_yaw_quat(), layer.force_cmd)
        dof_pos, dof_vel = self.env._dof()
        data = self.env._robot.data
        obs = interface.single_obs(data.root_quat_w, data.root_ang_vel_b, dof_pos, dof_vel, self._actions,
                                   torch.zeros(self.num_envs, device=self.device), commands)
        obs = torch.clip(obs, -interface.CLIP_OBSERVATIONS, interface.CLIP_OBSERVATIONS)
        history = self._history.append(obs)
        legs = torch.clip(self.policy.act(history), -interface.CLIP_ACTIONS, interface.CLIP_ACTIONS)
        action = torch.zeros(self.num_envs, interface.NUM_ACTIONS, device=self.device)
        action[:, :12] = legs
        self._actions = torch.where(live.unsqueeze(-1), action, torch.zeros_like(action))
        return torch.where(live.unsqueeze(-1), self._actions, crouch_action)

    def physics_step(self, dt: float) -> None:
        """The plant at the physics rate, and its forces on the foot, the knee and the head."""
        robot = self.env._robot
        if bool(self.active.any()):
            foot, foot_v = self._foot_state()
            knee = robot.data.body_pos_w[:, self._knee_body] - self.scene.env_origins
            knee_v = robot.data.body_lin_vel_w[:, self._knee_body]
            arm = interface.quat_apply(robot.data.root_quat_w, self._head_offset)
            head = robot.data.root_pos_w + arm - self.scene.env_origins
            head_v = robot.data.root_lin_vel_w + torch.cross(robot.data.root_ang_vel_w, arm, dim=-1)
            on = self.active.unsqueeze(-1).float()
            self.force_w = self._handle.step(foot, foot_v, dt, knee=knee, knee_v=knee_v, head=head, head_v=head_v,
                                             head_r=leg_cfg.HEAD_RADIUS_M) * on
            self.knee_force_w, self.head_force_w = self._handle.knee_force * on, self._handle.head_force * on
            self.peak_n = torch.maximum(self.peak_n, self._handle.take_peak() * self.active)
            self.max_theta = torch.where(self.active, torch.maximum(self.max_theta, self._handle.theta), self.max_theta)
        else:
            self.force_w.zero_(), self.knee_force_w.zero_(), self.head_force_w.zero_()
        local = torch.stack((
            interface.quat_rotate_inverse(robot.data.body_quat_w[:, self._foot_body], self.force_w),
            interface.quat_rotate_inverse(robot.data.body_quat_w[:, self._knee_body], self.knee_force_w),
            interface.quat_rotate_inverse(robot.data.root_quat_w, self.head_force_w)), dim=1)
        robot.permanent_wrench_composer.set_forces_and_torques(
            forces=local.contiguous(), positions=self._wrench_positions, body_ids=self._wrench_bodies)


class CombinerKeyEnv(door_env.CombinerDoorEnv):
    def __init__(self, cfg, render_mode=None, **kwargs):
        cfg.look_back_m = 0.0
        super().__init__(cfg, render_mode, **kwargs)
        n = self.num_envs
        self.key_inserted = bool(getattr(cfg, "key_inserted", False))
        self.lock_friction = float(getattr(cfg, "lock_friction_nm", LOCK_FRICTION_NM))
        self.lock_detent = float(getattr(cfg, "lock_detent_nm", LOCK_DETENT_NM))
        self.hide_furniture = True
        self.parked_phases = set(PARKED)
        self.position_only_phases = set(POSITION_ONLY)
        self.joint_phases = {"unfold", "fold"}
        self.leg = self._make_turn(cfg)
        self._link6_ids = torch.tensor([self._link6], dtype=torch.int32, device=self.device)
        self._grip = torch.zeros(n, dtype=torch.bool, device=self.device)
        #: How each environment's key is placed this physics step: "parked", "held" (the spring), "free" (PhysX alone)
        #: or "turned" (where the leg's plant has it).
        self.key_mode = ["parked"] * n
        self.records: list[dict] = [{} for _ in range(n)]
        origin = torch.as_tensor(kg.KEY_ORIGIN_LINK6, dtype=torch.float32, device=self.device)
        self._key_in_link6 = origin.expand(n, 3).contiguous()
        self._key_quat_link6 = torch.as_tensor(_matrix_quat(KEY_PRIM_IN_LINK6), dtype=torch.float32,
                                               device=self.device).expand(n, 4).contiguous()
        self._rest_q = None
        self.grip_err_mm, self.grip_err_deg = torch.zeros(n, device=self.device), torch.zeros(n, device=self.device)
        self._hand_offset = np.zeros((n, 3))
        self.handover_states: list[dict] = []

    def _make_turn(self, cfg):
        """What turns the key: the leg policy (`arm_env.ArmTurnEnv` puts the arm's push here instead)."""
        return LegTurn(self, cfg.leg_variant, cfg.leg_checkpoint)

    # --- the feet's friction, by who has the legs ----------------------------------------------------

    def _set_foot_friction(self) -> None:
        """The parent's foot friction for the walking policy (0.6 static, 0.2 dynamic: its training's against UniFP's
        ground), kept with the robot's own, which the leg policy trained on: with the walking policy's, the feet slid
        8 cm as the legs eased into the leg policy's stance, and it could not stand on three of them."""
        view = self._robot.root_physx_view
        self._materials_leg = view.get_material_properties().clone()
        super()._set_foot_friction()
        self._materials_walk = view.get_material_properties().clone()

    def _use_feet(self, index: int, leg_policy: bool) -> None:
        """One environment's feet to the leg policy's friction or the walking policy's. The whole array is written: the
        backend refuses a write of one environment's row among several."""
        view = self._robot.root_physx_view
        if not hasattr(self, "_materials_now"):
            self._materials_now = self._materials_walk.clone()
        self._materials_now[index] = (self._materials_leg if leg_policy else self._materials_walk)[index]
        view.set_material_properties(self._materials_now.clone(), torch.arange(view.count))

    # --- scene ------------------------------------------------------------------------------------

    def _setup_scene(self):
        if getattr(self.cfg, "given_box", False):
            # No camera and no lidar: the look is handed the box's true placement (`_look`). For developing the rest on
            # a GPU with no room for the renderer; the demo proper perceives.
            door_env.DecoupledDoorEnv._setup_scene(self)
        else:
            super()._setup_scene()
        out = kg_dir = door_env.head_camera.ROOT / "generated/combiner_key"
        key_usd = asset.build_key_usd(kg_dir / "key.usda")
        adapter_usd = asset.build_adapter_usd(out / "adapter.usda")
        key_cfg = RigidObjectCfg(
            prim_path="/World/envs/env_.*/Key",
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(key_usd),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=16,
                                                             solver_velocity_iteration_count=4,
                                                             max_depenetration_velocity=0.5, disable_gravity=False),
                mass_props=sim_utils.MassPropertiesCfg(mass=KEY_SIM_MASS_KG)),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, PARK_Z_M)))
        adapter_cfg = RigidObjectCfg(
            prim_path="/World/envs/env_.*/LockAdapter",
            spawn=sim_utils.UsdFileCfg(usd_path=str(adapter_usd),
                                       rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True)),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, PARK_Z_M + 1.0)))
        self._key = RigidObject(key_cfg)
        self._adapter = RigidObject(adapter_cfg)
        self.scene.rigid_objects["key"] = self._key
        self.scene.rigid_objects["lock_adapter"] = self._adapter

    def _set_key_inertia(self) -> None:
        """The key's centre of mass and inertia from the STL, not from its three small colliders."""
        view = self._key.root_physx_view
        coms = view.get_coms().clone()
        coms[:, :3] = torch.tensor(KEY_COM_M)
        view.set_coms(coms, torch.arange(view.count))
        inertias = view.get_inertias().clone()
        inertias[:] = 0.0
        inertias[:, 0], inertias[:, 4], inertias[:, 8] = KEY_INERTIA_KG_M2
        view.set_inertias(inertias, torch.arange(view.count))

    # --- the look, given ----------------------------------------------------------------------------

    def _look(self) -> None:
        if not getattr(self.cfg, "given_box", False):
            return super()._look()
        for index, demo in enumerate(self.scripts):
            if self.estimates[index] is not None or demo.finished or demo.phases[demo.index].name != "look":
                continue
            record = self.look_records[index]
            steps = record.get("steps", 0)
            record["steps"] = steps + 1
            if steps == 0:
                self._anchor(index)
            if demo.elapsed_s + interface.POLICY_DT >= demo.phases[demo.index].duration_s - 1e-9:
                self.estimates[index] = self.truth_sites[index]
                self.estimate_poses[index] = perception.site_pose(self.truth_sites[index])
                record.update({"found": True, "given": True, "detections": 0})

    def _lidar_frame(self) -> None:
        if not getattr(self.cfg, "given_box", False):
            super()._lidar_frame()

    def _finish_look(self, index: int) -> None:
        """The parent's estimate (the tag's position, the lidar's heading), then its depth along the door's normal from
        the lidar's face, where the fit found one (`LIDAR_DEPTH`)."""
        super()._finish_look(index)
        record = self.look_records[index]
        fit = record.get("lidar_heading") or {}
        behind_mm = fit.get("face_behind_tag_mm")
        if not LIDAR_DEPTH or self.estimates[index] is None or behind_mm is None:
            return
        pose = np.array(self.estimate_poses[index], dtype=float)
        out = pose[:3, :3] @ np.asarray(kg.OUT_OF_DOOR_BOX)
        shift = (behind_mm / 1000.0 - GEOMETRY.tag_standoff) * -out
        pose[:3, 3] += shift
        self.estimate_poses[index] = pose
        self.estimates[index] = perception.site_from_pose(pose)
        record["lidar_depth_shift_mm"] = round(1000.0 * float(np.linalg.norm(shift)) * np.sign(behind_mm / 1000.0 - GEOMETRY.tag_standoff), 2)

    # --- frames -----------------------------------------------------------------------------------

    def _box_to_env(self, index: int, site: props.BoxSite) -> np.ndarray:
        """4x4: a box-frame point to the env frame, for a placement given in the script frame (the anchored spawn)."""
        spawn = np.eye(4)
        spawn[:3, :3] = door_env.head_camera.quat_to_matrix(self._spawn_yaw_quat[index].cpu().numpy())
        spawn[:3, 3] = self._spawn_pos[index].cpu().numpy()
        return spawn @ floor_site(site)

    def _truth_env(self, index: int) -> np.ndarray:
        """The true box's frame in the env frame (`sites` are the env's placements, re-made by the anchor)."""
        return floor_site(self.sites[index])

    def key_pose_env(self, index: int, m: np.ndarray, theta_deg: float, off_m: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
        """The key prim's (position, quaternion wxyz) in the env frame, for the box frame `m` (box to env, 4x4)."""
        pos = m[:3, :3] @ np.asarray(kg.key_origin_box(theta_deg, off_m)) + m[:3, 3]
        return pos, _matrix_quat(m[:3, :3] @ key_prim_rotation(theta_deg))

    def _key_point(self, index: int, name: str, progress: float) -> tuple[float, float, float]:
        """The jaw centre for insertion phase `name` at `progress`, script frame, from the box's estimate: the key's
        origin `off` out along the keyway axis, plus the grasp's offset (25 mm along the key, 21.5 mm out)."""
        site = self.estimates[index]
        pose = floor_site(site)
        smooth = progress * progress * (3 - 2 * progress)
        offs = {"pre": (PRE_INSERT_M, PRE_INSERT_M), "align": (PRE_INSERT_M, PRE_INSERT_M),
                "insert": (PRE_INSERT_M, -OVERDRIVE_M), "seat": (-OVERDRIVE_M, -OVERDRIVE_M),
                "release": (-OVERDRIVE_M, -OVERDRIVE_M), "back": (-OVERDRIVE_M, -OVERDRIVE_M + BACK_OUT_M)}
        a, b = offs[name]
        off = a + (b - a) * (smooth if name in ("insert", "back") else 1.0)
        u = np.asarray(kg.lever_dir_box(kg.INSERT_DEG))
        jaw_box = (np.asarray(kg.key_origin_box(kg.INSERT_DEG, off)) + kg.KEY_GRASP_U_M * u
                   + (kg.KEY_ORIGIN_LINK6[2] - kg.JAW_CENTRE_Z_M) * np.asarray(kg.OUT_OF_DOOR_BOX))
        point = pose[:3, :3] @ jaw_box + pose[:3, 3]
        return tuple(float(v) for v in point)

    def _state_of(self, index: int) -> dict:
        state = super()._state_of(index)
        site = self.estimates[index]
        if site is not None:
            pose = floor_site(site)
            into = -pose[:3, :3] @ np.asarray(kg.OUT_OF_DOOR_BOX)
            bar = pose[:3, :3] @ np.asarray(kg.lever_dir_box(kg.INSERT_DEG))
            state["key_axes"] = (tuple(into), tuple(bar))
            state["key_point"] = lambda name, u, i=index: self._key_point(i, name, u)
        return state

    # --- the hand's outer loop ---------------------------------------------------------------------

    def _command_arm(self, due: torch.Tensor) -> None:
        """The parent's 10 Hz arm command, its point moved by the hand's outer loop in the insertion's phases."""
        import dataclasses

        centre_w, _, _ = self._hand_frame()
        period = self._command_period * interface.POLICY_DT
        for index in torch.nonzero(due).flatten().tolist():
            command = self.commands[index]
            if command is None or command.phase not in INSERT_PHASES:
                self._hand_offset[index] = 0.0
                continue
            spawn_rot = door_env.head_camera.quat_to_matrix(self._spawn_yaw_quat[index].cpu().numpy())
            measured = spawn_rot.T @ (centre_w[index] - self.scene.env_origins[index] - self._spawn_pos[index]).cpu().numpy()
            miss = np.asarray(command.point) - measured
            if command.phase not in INTEGRATE:
                miss = np.zeros(3)
            if command.phase in ACROSS_ONLY:
                axis = np.asarray(command.approach) / max(1e-9, float(np.linalg.norm(command.approach)))
                miss = miss - float(miss @ axis) * axis
            offset = self._hand_offset[index] + HAND_LOOP_GAIN * period * miss
            norm = float(np.linalg.norm(offset))
            self._hand_offset[index] = offset * min(1.0, HAND_LOOP_MAX_M / max(norm, 1e-9))
            self.commands[index] = dataclasses.replace(command, point=tuple(np.asarray(command.point) + self._hand_offset[index]))
            self.records[index]["hand_loop_mm"] = round(1000 * float(np.linalg.norm(self._hand_offset[index])), 1)
            self.records[index]["hand_miss_mm"] = round(1000 * float(np.linalg.norm(np.asarray(command.point) - measured)), 1)
        super()._command_arm(due)

    # --- the walks --------------------------------------------------------------------------------

    def _stance_target(self, index: int, phase: str) -> tuple[float, float]:
        """Where a walk phase ends, (x, y) in the script frame: the stance for the estimated box."""
        site = self.estimates[index]
        pose = floor_site(site)
        door_ahead = (kg.INSERT_FACE_AHEAD_M - kg.ADAPTER.face_off_door if phase == "walk_to_insert"
                      else kg.TURN_DOOR_AHEAD_M)
        # The lock's point on the door's face, and the door's normal, in the script frame; the base stands
        # `door_ahead` back along the normal and `LOCK_RIGHT_OF_BASE_M` to the lock's left.
        lock = pose[:3, :3] @ np.asarray(kg.LOCK) + pose[:3, 3]
        normal = pose[:3, :3] @ np.asarray(kg.OUT_OF_DOOR_BOX)
        right = pose[:3, :3] @ np.asarray(kg.LEVER_LEVEL_BOX)
        base = lock + door_ahead * normal - kg.LOCK_RIGHT_OF_BASE_M * right
        return float(base[0]), float(base[1])

    def _walk_base(self) -> None:
        super()._walk_base()
        for index, command in enumerate(self.commands):
            if command is None:
                continue
            if command.phase in WALK_PHASES and self.estimates[index] is not None:
                target = self._stance_target(index, command.phase)
                x, y, yaw = self._base_now[index]
                dx, dy = target[0] - x, target[1] - y
                dist = math.hypot(dx, dy)
                self._base_cmd[index] = 0.0
                c, s = math.cos(yaw), math.sin(yaw)
                ex, ey = c * dx + s * dy, -s * dx + c * dy           # the error in the base's frame
                for axis, error in ((0, ex), (1, ey)):
                    if abs(error) > WALK_ARRIVED_M:
                        speed = min(WALK_MAX_M_S, max(WALK_SPEED_M_S, WALK_GAIN * abs(error)))
                        self._base_cmd[index, axis] = math.copysign(speed, error)
                # Facing the door: the estimated box's normal reversed.
                pose = floor_site(self.estimates[index])
                normal = pose[:3, :3] @ np.asarray(kg.OUT_OF_DOOR_BOX)
                want = math.atan2(-normal[1], -normal[0])
                error = (want - yaw + math.pi) % (2 * math.pi) - math.pi
                self._base_cmd[index, 2] = WALK_YAW_GAIN * error if abs(error) > math.radians(1.5) else 0.0
                self.records[index]["walk_error_m"] = round(dist, 4)
            elif command.phase in TURN_PHASES:
                self._base_cmd[index] = 0.0
        self._commands[:, :3] = self._base_cmd

    # --- the step ---------------------------------------------------------------------------------

    def _scripted_goal(self) -> torch.Tensor:
        goal = super()._scripted_goal()
        for index, command in enumerate(self.commands):
            if command is None:
                continue
            record = self.records[index]
            if record.get("phase") != command.phase:
                self._enter(index, command.phase, record.get("phase"))
                record["phase"] = command.phase
        finished = self.leg.finished()
        for index in torch.nonzero(finished).flatten().tolist():
            self._end_turn(index)
        return goal

    def _enter(self, index: int, phase: str, previous: str | None) -> None:
        """What happens as an environment's script enters `phase`."""
        record = self.records[index]
        if phase == "unfold" or (phase == "look" and self.key_inserted):
            pass
        if phase == "walk_to_insert" or (phase == "walk_to_key"):
            record["estimate"] = self._estimate_record(index)
        if phase == "walk_to_insert":
            self.unfolded(index)
        if phase == "pre":
            record["pre_seed"] = self._seed_arm(index)
        if phase == "insert":
            record["aligned"] = self._seat_record(index)
        if phase == "release":
            record["seat"] = self._seat_record(index)
            self._grip[index] = False
            self.key_mode[index] = "free"
        if phase == "fold":
            record["after_release"] = self._seat_record(index)
            self._joint_goals[index] = ("fold", self._rest_arm())
        if phase == "turn":
            self._start_turn(index)

    def _seed_arm(self, index: int) -> str:
        """Warm-start the arm's IK at the pre-insertion pose from the planner's seeded search (as `choose_grasp` does
        once an attempt): the control loop's IK starts from its last solution, the home point's position-only one."""
        from demos.cup.pick_demo.grasp import _seeds, solve_link6
        from demos.unifp import decoupled_arm

        data = self._robot.data
        base_pos = (data.root_pos_w[index] - self.scene.env_origins[index]).cpu().numpy()
        base_rot = door_env.head_camera.quat_to_matrix(data.root_quat_w[index].cpu().numpy())
        spawn_pos = self._spawn_pos[index].cpu().numpy()
        spawn_rot = door_env.head_camera.quat_to_matrix(self._spawn_yaw_quat[index].cpu().numpy())
        state = self._state_of(index)
        point = np.asarray(self._key_point(index, "pre", 1.0))
        to_base = base_rot.T
        point_b = to_base @ (spawn_pos + spawn_rot @ point - base_pos)
        approach = to_base @ spawn_rot @ np.asarray(state["key_axes"][0])
        bar = to_base @ spawn_rot @ np.asarray(state["key_axes"][1])
        arm = self.arms[index]
        rot = decoupled_arm.hand_rotation(approach, bar, *arm.grasp)
        result = solve_link6(arm.joints, arm.links, point_b, rot, _seeds(point_b, [arm.q]), arm.offset)
        if result is None:
            return "no seeded solution"
        arm.q = np.asarray(result.q, dtype=float)
        return "seeded"

    def _rest_arm(self) -> np.ndarray:
        if self._rest_q is None:
            rest = np.asarray(d1_ik.from_servo_deg(d1_hardware.REST_SERVO_DEG)[0], dtype=float)
            limits = self._robot.data.joint_pos_limits[0][self._arm_ids].cpu().numpy()
            self._rest_q = np.clip(rest, limits[:, 0] + 0.005, limits[:, 1] - 0.005)
        return self._rest_q

    def _start_turn(self, index: int) -> None:
        """Hand the legs to the leg policy, the plant placed at the key's true pose and the layer told the estimate."""
        record = self.records[index]
        seated = self.key_inserted or (record.get("after_release") or {}).get("seated", False)
        record["turn_started"] = bool(seated)
        if not seated:
            return
        out = {}
        for name, m in (("truth", self._truth_env(index)), ("estimate", self._box_to_env(index, self.estimates[index]))):
            out[name] = (m[:3, :3] @ np.asarray(kg.LOCK) + m[:3, 3], m[:3, :3] @ np.asarray(kg.OUT_OF_DOOR_BOX),
                         m[:3, :3] @ np.asarray(kg.LEVER_LEVEL_BOX))
        # The robot as the walking policy leaves it at the turn's stance, for training a leg policy to start from it.
        data = self._robot.data
        dof, _ = self._dof()
        rp = interface.body_roll_pitch(data.root_quat_w[index:index + 1])[0]
        self.handover_states.append({"legs": dof[index, :12].cpu().numpy().tolist(),
                                     "base_z": float(data.root_pos_w[index, 2] - self.scene.env_origins[index, 2]),
                                     "roll": float(rp[0]), "pitch": float(rp[1])})
        if getattr(self.cfg, "handover", "crouch") == "settled":
            self._settle_for_leg(index)
        if getattr(self.cfg, "handover", "crouch") == "direct_still":
            # A diagnostic: the robot where the walking policy left it, its joints' and base's velocities zeroed.
            ids = torch.tensor([index], device=self.device)
            q = self._robot.data.joint_pos[ids].clone()
            self._robot.write_joint_state_to_sim(q, torch.zeros_like(q), env_ids=ids)
            self._robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=self.device), env_ids=ids)
            self.records[index]["handover"] = "direct, velocities zeroed (diagnostic)"
        self.leg.start(index, *out["truth"], *out["estimate"], self.lock_friction, self.lock_detent)
        if getattr(self.cfg, "handover", "crouch") in ("settled", "direct", "direct_still"):
            self.leg.crouch[index] = 0
        self._use_feet(index, leg_policy=True)
        self.key_mode[index] = "turned"
        self.walk_last_action[index] = 0.0

    def _settle_for_leg(self, index: int) -> None:
        """`--handover settled`, a diagnostic, not the demo: the robot put where it stands, the way it faces, in the
        stance the leg policy trained from (`leg_cfg.SETTLED_*`, the arm at rest), at rest -- a teleport, to see the turn
        in this scene apart from the handover from the walking policy's stance."""
        ids = torch.tensor([index], device=self.device)
        robot = self._robot
        q = robot.data.joint_pos[ids].clone()
        q[:, self._order[:12]] = torch.as_tensor(leg_cfg.SETTLED_LEG_POS, device=self.device)
        q[:, self._arm_ids] = torch.as_tensor(self._rest_arm(), dtype=torch.float32, device=self.device)
        robot.write_joint_state_to_sim(q, torch.zeros_like(q), env_ids=ids)
        root = robot.data.root_state_w[ids].clone()
        yaw = interface.yaw_quat(interface.yaw_from_quat(root[:, 3:7]))
        root[:, 2] = self.scene.env_origins[index, 2] + leg_cfg.SETTLED_BASE_HEIGHT_M
        root[:, 3:7] = yaw
        root[:, 7:] = 0.0
        robot.write_root_pose_to_sim(root[:, :7], env_ids=ids)
        robot.write_root_velocity_to_sim(root[:, 7:], env_ids=ids)
        self._planner.reset(ids, q[:, self._arm_ids])
        self.records[index]["handover"] = "settled (teleport, diagnostic)"

    def _end_turn(self, index: int) -> None:
        """Back to the walking policy, and the turn's record."""
        leg = self.leg
        record = self.records[index]
        record["turn"] = {
            "stage": int(leg._stage[index]), "engaged": bool(leg._engage_step[index] >= 0),
            "engage_miss_mm": round(1000 * float(leg._engage_miss[index]), 1),
            "max_theta_deg": round(math.degrees(float(leg.max_theta[index])), 2),
            "final_theta_deg": round(math.degrees(float(leg._handle.theta[index])), 2),
            "unlocked": bool(leg.max_theta[index] >= leg_cfg.KEY_THETA_UNLOCK_RAD),
            "measured_at_hold_deg": round(math.degrees(float(leg.held_meas[index])), 2),
            "verified_unlocked": bool(leg.held_meas[index] >= leg_cfg.KEY_THETA_UNLOCK_RAD),
            "peak_n": round(float(leg.peak_n[index]), 2), "cmd_max_n": round(float(leg.cmd_max[index]), 2),
            "foot_down": bool(leg._stage[index] == FOOT_DOWN), "steps": int(leg.steps[index]),
            "handover": leg.handover[index]}
        leg.stop(index)
        self._use_feet(index, leg_policy=False)
        self.walk_last_action[index] = 0.0
        if self._gait_clock is not None:
            self._gait_clock.reset(torch.tensor([index], device=self.device))

    def leg_action(self, walk: torch.Tensor | None = None) -> torch.Tensor:
        return self.leg.step(walk)

    def legs_from_policy(self) -> torch.Tensor:
        return self.leg.active

    # --- records ----------------------------------------------------------------------------------

    def _estimate_record(self, index: int) -> dict:
        """The look's error in the box's placement, as the key work uses it (on the floor): mm and degrees."""
        if self.estimates[index] is None:
            return {"found": False}
        truth, est = floor_site(self.sites_in_script(index)), floor_site(self.estimates[index])
        lock_t = truth[:3, :3] @ np.asarray(kg.LOCK) + truth[:3, 3]
        lock_e = est[:3, :3] @ np.asarray(kg.LOCK) + est[:3, 3]
        yaw = lambda m: math.degrees(math.atan2(m[1, 0], m[0, 0]))
        return {"found": True, "lock_error_mm": round(1000 * float(np.linalg.norm(lock_e - lock_t)), 2),
                "lock_error_xyz_mm": [round(1000 * float(v), 2) for v in lock_e - lock_t],
                "yaw_error_deg": round((yaw(est) - yaw(truth) + 180) % 360 - 180, 3)}

    def sites_in_script(self, index: int) -> props.BoxSite:
        """The true box placement in the script frame (`truth_sites` are given from the stance, `sites` are the env's
        after the anchor; the script frame is the anchor, so the given one is it)."""
        return self.truth_sites[index]

    def _seat_record(self, index: int) -> dict:
        """Where the key is against the adapter's keyway: depth along it, across it, roll and tilt."""
        pos = (self._key.data.root_pos_w[index] - self.scene.env_origins[index]).cpu().numpy()
        quat = self._key.data.root_quat_w[index].cpu().numpy()
        want_pos, want_quat = self.key_pose_env(index, self._truth_env(index), kg.INSERT_DEG)
        rot = door_env.head_camera.quat_to_matrix(quat)
        want = door_env.head_camera.quat_to_matrix(want_quat)
        delta = want.T @ (pos - want_pos)                  # in the seated key's frame: x off the door, y along, z up
        rel = want.T @ rot
        roll = math.degrees(math.atan2(rel[2, 1], rel[1, 1]))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, rel[0, 0]))))
        seated = bool(delta[0] <= SEATED_M and abs(delta[1]) < 0.003 and abs(delta[2]) < 0.003)
        # The hand: its approach (Link6 z) against the keyway's way in, and the key's axis against the hand's (the grip).
        link_q = self._robot.data.body_quat_w[index, self._link6].cpu().numpy()
        hand = door_env.head_camera.quat_to_matrix(link_q)
        angle = lambda a, b: math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b))))))
        into = -want[:, 0]
        return {"seated": seated, "out_mm": round(1000 * float(delta[0]), 2),
                "across_mm": [round(1000 * float(delta[1]), 2), round(1000 * float(delta[2]), 2)],
                "roll_deg": round(roll, 2), "tilt_deg": round(tilt, 2),
                "hand_approach_error_deg": round(angle(hand[:, 2], into), 2),
                "grip_tilt_deg": round(angle(hand[:, 2], -rot[:, 0]), 2),
                "ik": dict(self.ik_counts[index]), "ik_last": self.ik_status[index]}

    # --- physics ------------------------------------------------------------------------------------

    def _apply_action(self) -> None:
        super()._apply_action()
        self._place_key()
        self.leg.physics_step(self.cfg.sim.dt)
        # The latch: released once the key has turned past the unlock angle.
        for index in torch.nonzero(self.leg.active & (self.leg._handle.theta >= leg_cfg.KEY_THETA_UNLOCK_RAD)).flatten().tolist():
            latch = self._latches[index]
            if latch.latch.locked:
                latch.latch.locked = False
                self.records[index]["unlocked_at_s"] = round(float(self.episode_length_buf[index]) * interface.POLICY_DT, 2)

    def _hand_grasp_frame(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Where the held key's origin should be (world) and its orientation, with the frame's velocities."""
        data = self._robot.data
        pos, quat = data.body_pos_w[:, self._link6], data.body_quat_w[:, self._link6]
        arm = interface.quat_apply(quat, self._key_in_link6)
        target_q = _quat_mul(quat, self._key_quat_link6)
        vel = data.body_lin_vel_w[:, self._link6] + torch.cross(data.body_ang_vel_w[:, self._link6], arm, dim=-1)
        return pos + arm, target_q, vel, data.body_ang_vel_w[:, self._link6]

    def _place_key(self) -> None:
        """Each physics step: the parked key held up out of the way, the held one pulled by the grip's spring (and the
        hand by its reaction), the turned one written where the leg's plant has it."""
        n, device = self.num_envs, self.device
        key = self._key.data
        target_p, target_q, target_v, target_w = self._hand_grasp_frame()
        force = torch.zeros(n, 3, device=device)
        torque = torch.zeros(n, 3, device=device)
        writes = []
        for index, mode in enumerate(self.key_mode):
            if mode == "parked":
                writes.append((index, np.array([0.0, 0.0, PARK_Z_M]), np.array([1.0, 0.0, 0.0, 0.0])))
            elif mode == "seated_start":
                pos, q = self.key_pose_env(index, self._truth_env(index), kg.INSERT_DEG)
                writes.append((index, pos, q))
            elif mode == "appear":
                writes.append((index, None, None))
                self.key_mode[index] = "held"
            elif mode == "turned":
                theta = math.degrees(float(self.leg._handle.theta[index]))
                pos, q = self.key_pose_env(index, self._truth_env(index), theta)
                writes.append((index, pos, q))
        held = torch.tensor([m == "held" for m in self.key_mode], device=device)
        if bool(held.any()):
            dp = target_p - key.root_pos_w
            dv = target_v - key.root_lin_vel_w
            dq = _rotvec(_quat_mul(target_q, _quat_conj(key.root_quat_w)))
            #: The grip's error, for the trace: how far and how much turned the key is from where the hand holds it.
            self.grip_err_mm = 1000.0 * dp.norm(dim=-1) * held
            self.grip_err_deg = torch.rad2deg(dq.norm(dim=-1)) * held
            dw = target_w - key.root_ang_vel_w
            force = (GRIP_K_N_M * dp + GRIP_C_N_S_M * dv) * held.unsqueeze(-1)
            torque = (GRIP_K_NM_RAD * dq + GRIP_C_NM_S_RAD * dw) * held.unsqueeze(-1)
            cap = lambda v, limit: v * (limit / v.norm(dim=-1, keepdim=True).clamp(min=limit))
            force, torque = cap(force, GRIP_MAX_N), cap(torque, GRIP_MAX_NM)
        if writes:
            ids = torch.tensor([w[0] for w in writes], device=device)
            state = self._key.data.default_root_state[ids].clone()
            for row, (index, pos, q) in enumerate(writes):
                if pos is None:      # appearing in the hand: exactly at the grasp frame
                    state[row, :3], state[row, 3:7] = target_p[index], target_q[index]
                else:
                    state[row, :3] = torch.as_tensor(pos, dtype=torch.float32, device=device) + self.scene.env_origins[index]
                    state[row, 3:7] = torch.as_tensor(q, dtype=torch.float32, device=device)
                state[row, 7:] = 0.0
            self._key.write_root_pose_to_sim(state[:, :7], env_ids=ids)
            self._key.write_root_velocity_to_sim(state[:, 7:], env_ids=ids)
        # The grip's spring on the key (its frame), and its reaction on Link6 at the grasp point.
        key_q = key.root_quat_w
        self._key.permanent_wrench_composer.set_forces_and_torques(
            forces=interface.quat_rotate_inverse(key_q, force).unsqueeze(1).contiguous(),
            torques=interface.quat_rotate_inverse(key_q, torque).unsqueeze(1).contiguous())
        link_q = self._robot.data.body_quat_w[:, self._link6]
        lever = target_p - self._robot.data.body_pos_w[:, self._link6]
        reaction_t = -torque - torch.cross(lever, force, dim=-1)
        self._robot.permanent_wrench_composer.set_forces_and_torques(
            forces=interface.quat_rotate_inverse(link_q, -force).unsqueeze(1).contiguous(),
            torques=interface.quat_rotate_inverse(link_q, reaction_t).unsqueeze(1).contiguous(),
            body_ids=self._link6_ids)

    # --- resetting ----------------------------------------------------------------------------------

    def _place_props(self, env_ids: torch.Tensor) -> None:
        super()._place_props(env_ids)
        if not hasattr(self, "_adapter"):
            return
        state = self._adapter.data.default_root_state[env_ids].clone()
        for row, index in enumerate(env_ids.tolist()):
            if self.sites[index] is None:
                continue
            m = self._truth_env(index)
            # The slot frame: +x into the door, +y along the key at the insertion angle, +z across it.
            u, n, t = (np.asarray(v) for v in kg.key_rotation_box(kg.INSERT_DEG))
            frame = m[:3, :3] @ np.column_stack([-n, u, -t])
            pos = m[:3, :3] @ np.asarray(kg.key_origin_box(kg.INSERT_DEG)) + m[:3, 3]
            state[row, :3] = torch.as_tensor(pos, dtype=torch.float32, device=self.device) + self.scene.env_origins[index]
            state[row, 3:7] = torch.as_tensor(_matrix_quat(frame), dtype=torch.float32, device=self.device)
        state[:, 7:] = 0.0
        self._adapter.write_root_pose_to_sim(state[:, :7], env_ids=env_ids)
        self._adapter.write_root_velocity_to_sim(state[:, 7:], env_ids=env_ids)

    def _anchor(self, index: int) -> None:
        super()._anchor(index)
        if self.key_inserted:
            self.key_mode[index] = "seated_start"

    def _reset_idx(self, env_ids) -> None:
        super()._reset_idx(env_ids)
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        if not getattr(self, "_inertia_set", False):
            self._set_key_inertia()
            self._inertia_set = True
        for index in env_ids.tolist():
            self._use_feet(index, leg_policy=False)
            self.key_mode[index] = "parked"
            self.records[index] = {}
            self.arms[index].grasp, self.arms[index].grasp_note = (0.0, kg.GRASP_ROLL), "the key's grasp (roll -1)"
            if index < len(self._latches):
                self._latches[index].latch.locked = True
            self.leg.stop(index)
        self._grip[env_ids] = False

    def unfolded(self, index: int) -> None:
        """The arm has unfolded: the key appears in the hand and the grip holds it."""
        if not self.key_inserted and self.key_mode[index] == "parked":
            self.key_mode[index] = "appear"
            self._grip[index] = True
