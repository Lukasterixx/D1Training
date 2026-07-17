"""Teleop loop for the Go2+D1 flat testbed.

Everything here must be imported *after* `AppLauncher` has started Isaac Sim --
`main.py` is the entry point that guarantees that.

The keybinds are Rescue's, deliberately unchanged, so muscle memory carries
across. What is gone from the loop is Rescue's per-step arm pinning: the weld
does that job in physics now, which is the whole reason this repo exists.
"""
from __future__ import annotations

import os

import carb
import gymnasium as gym
import numpy as np
import torch

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from isaaclab_tasks.utils import get_checkpoint_path
from rsl_rl.runners import OnPolicyRunner

import flat_env_cfg
from agent_cfg import unitree_go2_agent_cfg
from d1_direct import GRIPPER_STROKE_MM, DirectD1
from d1_ik_controller import D1CartesianController, IsaacKinematics
from flat_env_cfg import (
    ARM_BASE_DAMPING,
    ARM_BASE_STIFFNESS,
    ARM_MOUNT_Z,
    D1_ARM_JOINTS,
    D1_GRIPPER_JOINTS,
    GRIPPER_BASE_DAMPING,
    GRIPPER_BASE_STIFFNESS,
    make_env_cfg,
)
from weld import build_welded_robot_usd

# Orientation of the D1's Link6 frame at the arm's zero pose: a 90 deg rotation
# about Y, which points the gripper's approach axis along base +X. This is NOT
# identity -- asking the D1's wrist for identity drives Joint5 into its limit and
# leaves the IK ~13 cm short. Treat this as "pointing straight ahead" and compose
# yaw on top of it rather than using a bare yaw quaternion.
D1_EE_REST_ROT = [0.70710678, 0.0, 0.70710678, 0.0]

DEFAULT_ARM_TELEOP_POS = [0.3, 0.0, 0.4]
DEFAULT_ARM_TELEOP_ROT = list(D1_EE_REST_ROT)

# The D1-550 reaches 550 mm.
ARM_MAX_REACH = 0.55

GRIPPER_STEP_MM = 5.0
ARM_STEP_M = 0.02

# --- Teleop state. Written by the keyboard thread, read by the sim loop. ---
ARM_TELEOP_POS = DEFAULT_ARM_TELEOP_POS.copy()
ARM_TELEOP_ROT = DEFAULT_ARM_TELEOP_ROT.copy()
ARM_TELEOP_YAW = 0.0
ARM_RESUME_REQUESTED = False
RESET_REQUESTED = False
GRIPPER_TELEOP_MM = 0.0

_D1 = None
_CONTROLLER = None


def quat_mul_wxyz(a, b):
    """Hamilton product of two (w, x, y, z) quaternions, as plain lists.

    The teleop layer runs on the keyboard thread and works in lists, not torch
    tensors, so this stays independent of the sim's math helpers.
    """
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


# ===================== Keyboard Handling =====================
# Identical to Rescue, minus T (no lidar in this repo):
#   W/A/S/D/Q/E    : walk  (x, y, yaw)
#   arrows / 1 / 0 : move the Cartesian IK target (x, y, then z)
#   , / .          : close / open the gripper (5 mm per press, 0-65 mm)
#   Z              : return to zero pose (suspends IK; any arm key re-engages)
#   P              : toggle motor power (e-stop)
#   R              : reset the robot
def sub_keyboard_event(event, *args, **kwargs) -> bool:
    global ARM_TELEOP_POS, ARM_TELEOP_ROT, ARM_TELEOP_YAW
    global ARM_RESUME_REQUESTED, RESET_REQUESTED, GRIPPER_TELEOP_MM
    speed = 1.0

    if event.type in (carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT):

        if event.input.name == 'R':
            RESET_REQUESTED = True
            print("[RESET] Requested robot reset.")
            return True

        # -------- , / . : D1 gripper ----------
        # Reads as < / > for close / open. Not G/H: H is Isaac's own hide
        # shortcut and firing both raises a warning in the viewport.
        if event.input.name in ('COMMA', 'PERIOD'):
            if _D1 is None:
                return True
            step = -GRIPPER_STEP_MM if event.input.name == 'COMMA' else GRIPPER_STEP_MM
            GRIPPER_TELEOP_MM = float(np.clip(GRIPPER_TELEOP_MM + step, 0.0, GRIPPER_STROKE_MM))
            _D1.set_gripper(GRIPPER_TELEOP_MM)
            print(f"[D1] Gripper -> {GRIPPER_TELEOP_MM:.1f} mm")
            return True

        # -------- P: motor power / e-stop toggle ----------
        if event.input.name == 'P':
            if _D1 is not None:
                _D1.set_motor_power(not _D1.is_powered())
            return True

        # -------- Z: return to zero pose ----------
        # The Cartesian loop must stand down first: it rewrites every joint at
        # 10 Hz and would overwrite the homing command before the arm moved.
        # Any arm key re-engages it from wherever the arm ended up.
        if event.input.name == 'Z':
            if _D1 is not None and _CONTROLLER is not None:
                _CONTROLLER.suspend()
                _D1.return_to_zero()
                print("[D1] Return-to-zero commanded; IK suspended "
                      "(press an arm key to re-engage).")
            return True

        # -------- Arrow keys & 1/0: D1 arm teleoperation ----------
        is_arm_key = False
        target_yaw = None

        if event.input.name == 'UP':
            ARM_TELEOP_POS[0] += ARM_STEP_M
            target_yaw = 0.0
            is_arm_key = True
        elif event.input.name == 'DOWN':
            ARM_TELEOP_POS[0] -= ARM_STEP_M
            target_yaw = np.pi
            is_arm_key = True
        elif event.input.name == 'LEFT':
            ARM_TELEOP_POS[1] += ARM_STEP_M
            target_yaw = np.pi / 2.0
            is_arm_key = True
        elif event.input.name == 'RIGHT':
            ARM_TELEOP_POS[1] -= ARM_STEP_M
            target_yaw = -np.pi / 2.0
            is_arm_key = True
        # carb names the top-row digits KEY_1/KEY_0, not '1'/'0' -- matching on
        # the bare digit silently limits these to the numpad.
        elif event.input.name in ('KEY_1', 'NUMPAD_1'):
            ARM_TELEOP_POS[2] += ARM_STEP_M
            is_arm_key = True
        elif event.input.name in ('KEY_0', 'NUMPAD_0'):
            ARM_TELEOP_POS[2] -= ARM_STEP_M
            is_arm_key = True

        if is_arm_key and _CONTROLLER is not None:
            # Re-engaging after a suspend has to resync to the arm's real pose
            # first; the sim loop does that, so this press only wakes it up.
            if _CONTROLLER.is_suspended:
                ARM_RESUME_REQUESTED = True
                print("[D1] Re-engaging IK from the arm's current pose.")
                return True

            if target_yaw is not None:
                # Shortest angular distance to the target direction.
                diff = (target_yaw - ARM_TELEOP_YAW + np.pi) % (2 * np.pi) - np.pi
                p_gain = 0.08
                max_step_rad = np.radians(10.0)  # so the base joint can keep up
                ARM_TELEOP_YAW += float(np.clip(diff * p_gain, -max_step_rad, max_step_rad))

                # Yaw about the robot's Z, composed onto the D1's rest
                # orientation. A bare yaw quaternion would command identity at
                # yaw=0, which this wrist cannot reach.
                yaw_q = [
                    float(np.cos(ARM_TELEOP_YAW / 2.0)), 0.0, 0.0,
                    float(np.sin(ARM_TELEOP_YAW / 2.0)),
                ]
                ARM_TELEOP_ROT[:] = quat_mul_wxyz(yaw_q, D1_EE_REST_ROT)

            # --- SAFEGUARDS ---
            # 1. Floor & robot body Z-limit (don't dig into the dog).
            if ARM_TELEOP_POS[2] < 0.05:
                ARM_TELEOP_POS[2] = 0.05

            # 2. Bounding box around the arm's own base.
            if -0.15 < ARM_TELEOP_POS[0] < 0.15 and -0.15 < ARM_TELEOP_POS[1] < 0.15:
                if ARM_TELEOP_POS[2] < 0.2:
                    ARM_TELEOP_POS[2] = 0.2

            # 3. Max reach, measured from the arm's own base -- which sits
            #    ARM_MOUNT_Z above the robot root, not at it.
            dx, dy = ARM_TELEOP_POS[0], ARM_TELEOP_POS[1]
            dz = ARM_TELEOP_POS[2] - ARM_MOUNT_Z
            dist = float(np.sqrt(dx * dx + dy * dy + dz * dz))
            if dist > ARM_MAX_REACH:
                scale = ARM_MAX_REACH / dist
                ARM_TELEOP_POS[0] = dx * scale
                ARM_TELEOP_POS[1] = dy * scale
                ARM_TELEOP_POS[2] = ARM_MOUNT_Z + dz * scale

            print(f"[TELEOP] Arm target: X={ARM_TELEOP_POS[0]:.3f} Y={ARM_TELEOP_POS[1]:.3f} "
                  f"Z={ARM_TELEOP_POS[2]:.3f} | Yaw: {np.degrees(ARM_TELEOP_YAW):.1f} deg")

        # -------- WASD/QE walking teleop ----------
        if len(flat_env_cfg.base_command) > 0:
            if event.input.name == 'W':
                flat_env_cfg.base_command["0"] = [speed, 0, 0]
            if event.input.name == 'S':
                flat_env_cfg.base_command["0"] = [-speed, 0, 0]
            if event.input.name == 'A':
                flat_env_cfg.base_command["0"] = [0, speed, 0]
            if event.input.name == 'D':
                flat_env_cfg.base_command["0"] = [0, -speed, 0]
            if event.input.name == 'Q':
                flat_env_cfg.base_command["0"] = [0, 0, speed]
            if event.input.name == 'E':
                flat_env_cfg.base_command["0"] = [0, 0, -speed]

        if len(flat_env_cfg.base_command) > 1:
            if event.input.name == 'I':
                flat_env_cfg.base_command["1"] = [speed, 0, 0]
            if event.input.name == 'K':
                flat_env_cfg.base_command["1"] = [-speed, 0, 0]
            if event.input.name == 'J':
                flat_env_cfg.base_command["1"] = [0, speed, 0]
            if event.input.name == 'L':
                flat_env_cfg.base_command["1"] = [0, -speed, 0]
            if event.input.name == 'U':
                flat_env_cfg.base_command["1"] = [0, 0, speed]
            if event.input.name == 'O':
                flat_env_cfg.base_command["1"] = [0, 0, -speed]

    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        for i in range(len(flat_env_cfg.base_command)):
            flat_env_cfg.base_command[str(i)] = [0, 0, 0]

    return True


def _subscribe_keyboard():
    """Wire up the teleop keys, if there is a window to receive them.

    `omni.appwindow` does not exist in the headless kit, so --headless (useful
    for a quick smoke test of the weld) must not hard-fail on it.
    """
    try:
        import omni.appwindow
    except ModuleNotFoundError:
        print("[keys] Headless: keyboard teleop unavailable; the robot will stand still.")
        return None, None, None

    iface = carb.input.acquire_input_interface()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    return iface, keyboard, iface.subscribe_to_keyboard_events(keyboard, sub_keyboard_event)


def _resolve_go2_usd() -> str:
    from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

    return f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd"


def _report_articulation(robot, with_arm: bool) -> None:
    """Print what PhysX actually parsed.

    Worth the noise: if the weld silently failed, the arm shows up as its own
    articulation and this list comes back with only 12 joints -- which is the
    difference between "the payload does not affect the gait" and "there is no
    payload".
    """
    names = list(robot.data.joint_names)
    print(f"[robot] {len(names)} joints: {names}")
    print(f"[robot] {len(robot.data.body_names)} bodies: {list(robot.data.body_names)}")
    total_mass = float(robot.root_physx_view.get_masses()[0].sum())
    print(f"[robot] total articulation mass: {total_mass:.3f} kg")
    if with_arm:
        missing = [n for n in D1_ARM_JOINTS + D1_GRIPPER_JOINTS if n not in names]
        if missing:
            raise RuntimeError(
                f"[robot] The weld did not take: {missing} are absent from the Go2's "
                f"articulation. PhysX most likely parsed the arm as a second "
                f"articulation -- check that weld.py stripped its ArticulationRootAPI."
            )


class SelfTest:
    """Drive the robot forward headlessly and report whether the gait survived.

    The point of this repo is a comparison -- bare Go2 versus Go2 + arm, at a
    given arm mass -- and eyeballing a viewport is a poor way to make it. This
    walks a fixed command for a fixed duration and prints numbers you can diff
    across runs.
    """

    SETTLE_S = 1.0
    FALLEN_HEIGHT_M = 0.15

    def __init__(self, duration_s: float, command=(1.0, 0.0, 0.0), kin=None):
        self.duration_s = duration_s
        self.command = list(command)
        self.kin = kin  # IsaacKinematics, if the arm is fitted
        self.t = 0.0
        self.walking = False
        self.start_pos = None
        self.min_height = float("inf")
        self.max_tilt_deg = 0.0
        self.fell = False

    def step(self, robot, dt: float) -> bool:
        """Advance the test. Returns False once it is done."""
        self.t += dt

        # Let the robot settle onto its feet before commanding anything;
        # otherwise the spawn drop pollutes the tilt and height stats.
        if not self.walking:
            if self.t < self.SETTLE_S:
                return True
            self.walking = True
            self.t = 0.0
            self.start_pos = robot.data.root_state_w[0, :3].clone()
            flat_env_cfg.base_command["0"] = list(self.command)
            print(f"[selftest] Walking at {self.command} for {self.duration_s:.0f} s...")
            return True

        pos = robot.data.root_state_w[0, :3]
        self.min_height = min(self.min_height, float(pos[2]))

        # Tilt straight off the gravity vector in the base frame: its z is
        # cos(angle from upright), so a flat dog reads 0 deg regardless of yaw.
        gz = float(robot.data.projected_gravity_b[0, 2])
        self.max_tilt_deg = max(self.max_tilt_deg, float(np.degrees(np.arccos(np.clip(-gz, -1.0, 1.0)))))

        if float(pos[2]) < self.FALLEN_HEIGHT_M:
            self.fell = True

        if self.t < self.duration_s:
            return True

        travelled = (pos - self.start_pos).cpu().numpy()
        dist = float(np.linalg.norm(travelled[:2]))
        speed = dist / self.duration_s
        print("\n" + "=" * 62)
        print(f"[selftest] commanded {self.command[0]:.2f} m/s for {self.duration_s:.0f} s")
        print(f"[selftest] travelled      : {dist:.2f} m  (dx={travelled[0]:+.2f}, dy={travelled[1]:+.2f})")
        print(f"[selftest] mean speed     : {speed:.2f} m/s  ({100.0 * speed / max(self.command[0], 1e-6):.0f}% of command)")
        print(f"[selftest] min base height: {self.min_height:.3f} m")
        print(f"[selftest] max tilt       : {self.max_tilt_deg:.1f} deg")
        print(f"[selftest] verdict        : {'FELL OVER' if self.fell else 'stayed up'}")
        if self.kin is not None:
            # Proves the IK chain actually closed: the weld put the arm in the
            # Go2's articulation, so a mis-sliced Jacobian would still run
            # without error and simply never converge. Tracking error is the
            # only thing that catches that.
            snap = self.kin.snapshot(torch.zeros(1, len(self.kin.joint_ids), device=robot.device))
            rel_pos, _ = self.kin.to_local(snap.ee_pos, snap.ee_quat)
            actual = rel_pos[0].cpu().numpy()
            err = float(np.linalg.norm(actual - np.asarray(ARM_TELEOP_POS)))
            print(f"[selftest] arm EE target  : {np.round(ARM_TELEOP_POS, 3)} (base-relative)")
            print(f"[selftest] arm EE actual  : {np.round(actual, 3)}  -> error {err * 100:.1f} cm")
        print("=" * 62 + "\n")
        return False


def _load_policy(env, agent_cfg):
    log_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "logs", "rsl_rl", agent_cfg["experiment_name"]))
    resume_path = get_checkpoint_path(log_root_path, agent_cfg["load_run"], agent_cfg["load_checkpoint"])
    print(f"[policy] Loading {resume_path}")

    runner = OnPolicyRunner(env, agent_cfg, log_dir=None, device=agent_cfg["device"])
    # Actor and critic only. The default load_cfg also wants an optimizer state
    # and an iteration counter, which the shipped checkpoint does not carry --
    # it was converted from the 2024-era rsl_rl format, and playback has no use
    # for either.
    runner.load(resume_path, load_cfg={"actor": True, "critic": True})
    return runner.get_inference_policy(device=env.unwrapped.device)


def _reset(robot, root_pose, joint_pos, controller):
    global RESET_REQUESTED, ARM_TELEOP_POS, ARM_TELEOP_ROT, ARM_TELEOP_YAW
    robot.write_root_pose_to_sim(root_pose.clone())
    robot.write_root_velocity_to_sim(torch.zeros_like(robot.data.root_state_w[:, 7:13]))
    robot.write_joint_state_to_sim(joint_pos.clone(), torch.zeros_like(robot.data.default_joint_vel))
    for key in flat_env_cfg.base_command:
        flat_env_cfg.base_command[key] = [0.0, 0.0, 0.0]
    if controller is not None:
        controller.reset(DEFAULT_ARM_TELEOP_POS, DEFAULT_ARM_TELEOP_ROT)
        ARM_TELEOP_POS[:] = DEFAULT_ARM_TELEOP_POS
        ARM_TELEOP_ROT[:] = DEFAULT_ARM_TELEOP_ROT
        ARM_TELEOP_YAW = 0.0
    RESET_REQUESTED = False
    print("[RESET] Robot restored to the start pose.")


def run(args_cli, simulation_app):
    global _D1, _CONTROLLER, ARM_RESUME_REQUESTED, ARM_TELEOP_YAW

    _input, _keyboard, _sub_keyboard = _subscribe_keyboard()

    with_arm = not args_cli.no_arm
    if with_arm:
        robot_usd = build_welded_robot_usd(
            go2_usd_path=_resolve_go2_usd(),
            d1_urdf_path=os.path.join(os.path.dirname(__file__), "d1_arm", "d1.urdf"),
            out_usd_path=os.path.join(os.path.dirname(__file__), "generated", "go2_d1.usd"),
            mount_pos=(0.0, 0.0, ARM_MOUNT_Z),
            arm_mass_kg=args_cli.arm_mass,
        )
    else:
        # Baseline: the same flat course with a bare Go2, so the arm's effect on
        # the gait can be seen as a difference rather than guessed at.
        print("[weld] --no_arm: running the bare Go2 as a baseline.")
        robot_usd = _resolve_go2_usd()

    env_cfg = make_env_cfg(robot_usd, num_envs=args_cli.num_envs, with_arm=with_arm)
    for i in range(args_cli.num_envs):
        flat_env_cfg.base_command[str(i)] = [0.0, 0.0, 0.0]

    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    robot = env.unwrapped.scene["robot"]
    _report_articulation(robot, with_arm)

    reset_root_pose = robot.data.root_state_w[:, :7].clone()
    reset_joint_pos = robot.data.joint_pos.clone()

    policy = _load_policy(env, unitree_go2_agent_cfg)
    obs = env.get_observations()

    controller = None
    kin = None
    if with_arm:
        device = env.unwrapped.device
        _D1 = DirectD1(
            robot=robot,
            arm_joint_names=D1_ARM_JOINTS,
            gripper_joint_names=D1_GRIPPER_JOINTS,
            device=device,
            arm_stiffness=ARM_BASE_STIFFNESS,
            arm_damping=ARM_BASE_DAMPING,
            gripper_stiffness=GRIPPER_BASE_STIFFNESS,
            gripper_damping=GRIPPER_BASE_DAMPING,
        )
        kin = IsaacKinematics(robot, D1_ARM_JOINTS, ee_body_name="Link6")
        controller = D1CartesianController(
            arm_client=_D1,
            backend=kin,
            num_envs=args_cli.num_envs,
            device=device,
        )
        controller.reset(DEFAULT_ARM_TELEOP_POS, DEFAULT_ARM_TELEOP_ROT)
        _CONTROLLER = controller

    sim_dt = env.unwrapped.step_dt
    selftest = SelfTest(args_cli.selftest, kin=kin) if args_cli.selftest > 0 else None
    if selftest is None:
        print("\n[keys] W/A/S/D walk, Q/E turn | arrows + 1/0 move the arm | , . gripper "
              "| Z home | P e-stop | R reset\n")

    while simulation_app.is_running():
        with torch.inference_mode():
            if RESET_REQUESTED:
                _reset(robot, reset_root_pose, reset_joint_pos, controller)

            if controller is not None:
                # Re-engaging after a suspend: adopt the arm's actual pose as
                # the target so it does not snap back to a stale one.
                if ARM_RESUME_REQUESTED:
                    pos, rot = controller.resume_from_arm()
                    ARM_TELEOP_POS[:] = pos
                    ARM_TELEOP_ROT[:] = rot
                    w, x, y, z = rot
                    ARM_TELEOP_YAW = float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
                    ARM_RESUME_REQUESTED = False

                _D1.refresh()
                controller.update(ARM_TELEOP_POS, ARM_TELEOP_ROT, sim_dt)
                _D1.apply()

            # The policy sees only the 12 leg joints; the arm rides along as a
            # payload it was never trained to expect. That is the experiment.
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)

            if selftest is not None and not selftest.step(robot, sim_dt):
                break

    if _sub_keyboard is not None:
        _input.unsubscribe_to_keyboard_events(_keyboard, _sub_keyboard)
    env.close()
