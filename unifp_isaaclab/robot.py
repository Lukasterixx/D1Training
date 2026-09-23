"""The welded Go2+D1 in Isaac Lab, controlled the way UniFP controls it.

This is the half of the port that has to be argued for rather than read off. `interface.py` is a
contract the checkpoint imposes and is either met or not; what follows is a claim that a second
simulator has been set up to do the same *physics*, and every deviation below is a reason the
policy might behave differently for reasons that have nothing to do with the policy.

What is matched deliberately:

  * **Explicit PD torques, recomputed at 200 Hz.** UniFP computes
    `tau = kp*(a*0.25 + q_default - q) - kd*qd`, clips it at the URDF effort limit, and writes it
    as a joint force every physics substep -- four substeps per 50 Hz policy step. Isaac Lab's
    `IdealPDActuator` computes exactly that expression and clips it the same way, and
    `Articulation.write_data_to_sim()` runs it once per physics step, so the decimation structure
    is reproduced rather than approximated. An `ImplicitActuator` would hand the same gains to
    PhysX's own implicit solver instead, which is a different integrator and a different answer.
  * **Force drives.** The D1's joints arrive from the URDF importer as *acceleration* drives, which
    scale gains by each joint's effective inertia (F-010). UniFP's gains are N·m/rad. Without this
    the same number means something else on each side, and the arm sags.
  * **The gains, effort limits, default pose and spawn height** are `interface.py`'s, which were
    read out of the running Isaac Gym environment rather than copied from its config.

What is **not** matched, and is a real limit on what this comparison can show:

  * **The robot model is not the same asset.** Isaac Gym loads a URDF that `unifp_go2d1/build_asset.py`
    generates with a reassembled mass model (18.171 kg). Isaac Lab loads the weld of Isaac Lab's
    stock `go2.usd` and the URDF-imported D1. They are built to describe the same robot, and
    `report_model()` prints the mass of what actually loaded so the two can be compared, but they
    are not the same file and the inertias are not guaranteed identical.
  * **Rotor inertia is added to every joint** (`ARM_ARMATURE`), where Isaac Gym runs with none.
    This is the one deliberate departure from the training stack's numbers, it is forced by the
    integrator rather than chosen, and the comment on that constant records what happens without
    it. Read it before comparing arm behaviour across the two stacks.
  * **The ground is a flat plane**, where UniFP trains and plays on a `trimesh` terrain that is
    flat-but-rough (height 0 to 0.05 m). Pass `--flat_terrain` to `unifp_go2d1/play_policy.py`
    for the Isaac Gym side of any comparison, or the terrain difference is folded into the result.
  * **The feet are different colliders.** Isaac Gym builds them from the URDF with
    `replace_cylinder_with_capsule`; Isaac Lab uses whatever `go2.usd` ships. Contact geometry is
    not something either config states, and it is the leading suspect whenever the two stacks
    agree while standing and disagree while walking.
  * **PhysX is configured by two different front ends.** Solver iteration counts, contact offsets
    and friction combine modes are set here to the values below; Isaac Gym's defaults for the same
    quantities are not all discoverable from the config.

None of these make the port wrong. They are the reason a disagreement between the two simulators
is not, on its own, evidence about the policy.
"""
from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.actuators import IdealPDActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

from . import interface

#: Spawn height, from UniFP's `init_state.pos`. Above the Go2's ~0.30 m standing height, as
#: upstream spawns the B2 above its own: the first moments of an episode are a short drop.
SPAWN_HEIGHT_M = 0.35

#: Rotor inertia added to every joint, kg·m².
#:
#: Isaac Gym leaves this at zero and UniFP does not set it, so matching the training stack argues
#: for zero here -- and that is wrong, for a reason that is about the integrator rather than the
#: robot. The D1's wrist links have inertias around 1e-5 kg·m²; UniFP's arm stiffness of 40 N·m/rad
#: on 1e-5 kg·m² is a natural frequency near 1600 rad/s, and an explicit PD evaluated every 5 ms
#: cannot integrate that. Isaac Gym's articulation solver absorbs it; Isaac Lab's does not, and the
#: measured result with `armature = 0` is `Joint4` escaping its +/-2.35 rad limit entirely and
#: reaching -127 rad while its torque sits saturated -- which throws the whole robot over.
#:
#: This is the deviation from the training stack that this port cannot avoid. It is small (it adds
#: 2e-4 kg·m² of rotor inertia, an order of magnitude more than the link itself, but negligible
#: against the 3.3/1.7 N·m the joints can produce at the speeds the task uses) and it is stated
#: rather than hidden, because it does change the arm's dynamics.
ARM_ARMATURE = 2.0e-4

#: PhysX solver iterations (position, velocity).
#:
#: 4/0, which is what the training stack uses (`legged_gym`'s `sim.physx.num_position_iterations`
#: and `num_velocity_iterations`) and, as it happens, also what Isaac Lab's stock Go2 ships. This
#: repository's *own* tasks raise them to 8/4 following unitree_rl_lab, and the temptation is to
#: do the same here -- but the point of this port is to match the stack the policy was trained in,
#: not this repository's conventions.
#:
#: It is worth knowing how much rests on that choice. Measured, same checkpoint, same command,
#: standing on flat ground: at 4/0 the policy holds a stance for the whole 30 s rollout at 8.7 cm
#: median tool-tip error; at 8/4 it falls immediately and spends 100% of the window below 15 cm.
#: The *passive* robot barely notices (zero actions settle at 27.6 cm against 27.2 cm). So this is
#: not a setting that makes the simulation more or less correct -- it decides whether a marginal
#: controller stays up, and it is the reason the standing result must not be read as robust.
SOLVER_ITERATIONS = (4, 0)

#: UniFP's terrain friction (`terrain.static_friction` / `dynamic_friction` / `restitution`).
GROUND_STATIC_FRICTION = 1.0
GROUND_DYNAMIC_FRICTION = 1.0
GROUND_RESTITUTION = 0.0


def _per_joint(values) -> dict[str, float]:
    """Map Isaac Lab joint names to their UniFP value, one exact key per joint.

    Exact names, not regexes: a regex that matches two joints with different gains would take the
    first silently, and `Joint1` is a prefix of nothing here only by luck.
    """
    return {name: float(value) for name, value in zip(interface.ISAACLAB_NAMES, values)}


def make_robot_cfg(usd_path: str, prim_path: str = "/World/envs/env_.*/Robot",
                   spawn_height: float = SPAWN_HEIGHT_M,
                   self_collisions: bool = True,
                   armature: float = ARM_ARMATURE,
                   friction: float = 0.0,
                   solver_iterations: tuple[int, int] = SOLVER_ITERATIONS) -> ArticulationCfg:
    """The welded robot, with UniFP's control law in place of this repository's own.

    Self-collisions are **on**, which is what UniFP trains with. Its config reads
    `self_collisions = 0`, and the temptation is to read that as "off"; legged_gym passes the value
    straight to `create_actor` as a collision *filter* bitmask, and its own comment says
    "1 to disable, 0 to enable". So 0 enables them. It matters here because the policy commands
    arm actions up to 28 (a 7 rad offset at `ACTION_SCALE`), i.e. it deliberately drives the arm
    into its limits and lets the joint and torque limits do the clamping -- with self-collision
    off, the arm swings through the body instead of being stopped by it.
    """
    cfg = UNITREE_GO2_CFG.copy()
    cfg.prim_path = prim_path
    cfg.spawn = cfg.spawn.replace(
        usd_path=usd_path,
        # The gains below are N·m/rad. The URDF importer authors the D1's joints as acceleration
        # drives, where they would not be (F-010).
        joint_drive_props=sim_utils.JointDrivePropertiesCfg(drive_type="force"),
    )
    cfg.spawn.articulation_props.enabled_self_collisions = self_collisions
    # Set explicitly rather than inherited, because the value matters more than it looks like it
    # should -- see SOLVER_ITERATIONS.
    cfg.spawn.articulation_props.solver_position_iteration_count = solver_iterations[0]
    cfg.spawn.articulation_props.solver_velocity_iteration_count = solver_iterations[1]

    cfg.init_state = ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, spawn_height),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos=_per_joint(interface.DEFAULT_DOF_POS),
        joint_vel={".*": 0.0},
    )

    # One actuator group over all 20 joints: the same torque law drives the 18 actioned joints and
    # the 2 held jaws, and only the gains differ. UniFP applies it to every DOF in one expression.
    stiffness = _per_joint(interface.P_GAINS + (interface.GRIPPER_STIFFNESS,) * 2)
    damping = _per_joint(interface.D_GAINS + (interface.GRIPPER_DAMPING,) * 2)
    effort = _per_joint(interface.TORQUE_LIMITS)
    cfg.actuators = {
        "unifp": IdealPDActuatorCfg(
            joint_names_expr=list(interface.ISAACLAB_NAMES),
            stiffness=stiffness,
            damping=damping,
            effort_limit=effort,
            effort_limit_sim=effort,
            armature=armature,
            friction=friction,
        )
    }
    return cfg


def ground_cfg() -> sim_utils.GroundPlaneCfg:
    """A flat plane with UniFP's terrain friction."""
    return sim_utils.GroundPlaneCfg(
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=GROUND_STATIC_FRICTION,
            dynamic_friction=GROUND_DYNAMIC_FRICTION,
            restitution=GROUND_RESTITUTION,
        )
    )


def report_model(robot) -> dict:
    """What actually loaded, in the terms the Isaac Gym side reports.

    The total mass is the number to compare against `articulation_mass_kg` in a `unifp_go2d1`
    run record (18.172 kg there). A difference is not a bug in the port, but it is a difference
    in the robot, and any behavioural comparison has to carry it.
    """
    masses = robot.root_physx_view.get_masses()[0]
    body_names = robot.body_names
    arm_bodies = [i for i, n in enumerate(body_names)
                  if n.startswith("Link") or n == interface.TOOL_BODY]
    # The joint limits are reported because the policy leans on them: it commands arm actions of
    # up to 28, which is a 7 rad offset at ACTION_SCALE, so where the joint actually stops is part
    # of the controller. A limit that differs from the training asset's changes the behaviour even
    # though every gain matches.
    limits = robot.data.joint_pos_limits[0]
    effort = robot.data.joint_effort_limits[0] if hasattr(robot.data, "joint_effort_limits") else None
    return {
        "total_mass_kg": round(float(masses.sum()), 4),
        "arm_mass_kg": round(float(masses[arm_bodies].sum()), 4),
        "num_joints": len(robot.joint_names),
        "num_bodies": len(body_names),
        "joint_names": list(robot.joint_names),
        "body_names": list(body_names),
        "joint_pos_limits": {name: [round(float(limits[i, 0]), 4), round(float(limits[i, 1]), 4)]
                             for i, name in enumerate(robot.joint_names)},
        "joint_effort_limits": (
            {name: round(float(effort[i]), 4) for i, name in enumerate(robot.joint_names)}
            if effort is not None else None),
    }
