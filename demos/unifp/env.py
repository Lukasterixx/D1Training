"""UniFP's trained policy driving a manipulation demo: the task's environment, plus furniture.

`unifp_train.env.Go2D1PosForceEnv` is the environment the deliverable checkpoint was trained in.
This subclasses it rather than rebuilding it, so the robot, the control law, the observation
contract, the gait clock and the goal frame are the trained ones by construction, and the only
things that change are the ones a demo has to change:

  * **A table and a cup, or a post and the combiner box**, added to the scene. The robot stands,
    so the object has to be lifted into the goal sphere -- see `props.py` for the heights and why.
  * **The goal is scripted, not sampled.** UniFP's `EeGoalTrajectory` walks a goal around the
    workspace on its own; here the demo's phase list says where it goes. The generator's `step` is
    replaced rather than its buffers written, because it also runs timers and resamples.
  * **The jaws are commanded.** UniFP holds them at their default with a separate PD and never
    actions them (`interface.NUM_ACTIONS` is 18 of 20 joints), which leaves them free for a demo
    to drive without touching anything the policy does.
  * **The base is told to stand, always.** `_resample_velocity_commands` is overridden to write
    zeros instead of sampling, which also pins the gait phase to 0 (`interface.gait_step`). The
    robot is standing still by command, not by the policy's choice.
  * **The scheduled pushes are off.** UniFP's force curriculum applies random wrenches to the tool
    on a timer; a demo's forces should be the ones the cup and the lever actually exert. The
    curriculum is disabled by `force_start_step`, and what the trace records is contact, not
    schedule.

Two frames are worth keeping straight. UniFP commands the **tool point** -- the tip of the
Link7_1 pincer, one finger -- and a grasp is placed by the **jaw centre**, midway between the
pads. They are ~20 mm apart along the approach and up to 39 mm across it with the jaws open, so
commanding one where the other should go misses the cup entirely. `_tip_goal_from_jaw_centre`
converts, live, from the measured hand pose rather than from an assumed one.
"""
from __future__ import annotations

import math

import torch

from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
import isaaclab.sim as sim_utils
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.utils import configclass

from unifp_isaaclab import interface
from unifp_train.env import Go2D1PosForceEnv
from unifp_train.env_cfg import Go2D1PosForceEnvCfg

from demos.combiner.geometry import DEFAULT_HANDLE_TORQUE_NM, GEOMETRY
from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6

from . import props, wrist
from .script import Command, DemoScript

#: Link6's own axes, in its frame: +z out of the palm toward the fingertips, +y from one finger to
#: the other. Read off `d1_arm/d1.urdf`, where the two finger joints sit at z = +0.0706 and
#: y = -/+0.0296 in Link6 -- so +z is the approach and +y is the axis the jaws open along.
LINK6_APPROACH_LOCAL = (0.0, 0.0, 1.0)
LINK6_JAW_LOCAL = (0.0, 1.0, 0.0)


@configclass
class UniFPDemoEnvCfg(Go2D1PosForceEnvCfg):
    """The trained task's configuration with a demo's furniture bolted on.

    Anything that would change what the policy sees is left exactly as the training config has it.
    The scene is bigger and the episode is longer; the contract is not touched.
    """

    #: **The released checkpoints' controlled point, pinned deliberately.** Since 2026-09-24 the
    #: training task controls the jaw centre (`task_cfg.TOOL_BODY`), and these demos inherit that
    #: config -- but they run `unifp_go2d1_isaaclab_model_56000`, which was trained to put its
    #: *fingertip* on the commanded goal. Letting the default through would move the commanded
    #: point 2.4 cm for a policy that knows nothing about it, and would silently invalidate every
    #: demo number recorded before that date. A policy trained on the jaw centre should have these
    #: two lines deleted, not edited.
    tool_body: str = interface.TOOL_BODY
    tool_offset_m: tuple[float, float, float] = interface.TOOL_OFFSET_M

    #: "cup" or "combiner".
    task: str = "cup"
    #: Built by the launcher inside the Isaac app, which is the only place `pxr` is importable.
    cup_usd: str = ""
    box_usd: str = ""
    handle_torque_nm: float = DEFAULT_HANDLE_TORQUE_NM
    #: The door closer's torque at 45 degrees, N·m (`combiner_cfg`); 0 is a free door.
    door_torque_nm: float = 0.0
    #: Replaced with the demo's own length by the launcher; the trained 20 s would reset mid-attempt.
    episode_length_s: float = 40.0
    #: UniFP's scheduled tool pushes, off: a demo's forces should be the object's.
    force_start_step: int = 10 ** 9

    #: The task's own contact sensor is `/World/envs/env_.*/Robot/.*`, one level deep, and the
    #: weld nests the arm under `Robot/D1/`, so it sees the Go2's bodies and none of the D1's.
    #: The training task only ever asks it about feet and thighs. A demo needs the hand.
    arm_contact: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/D1/Link.*", history_length=3, update_period=0.0,
        track_air_time=False)

    table: RigidObjectCfg = None
    cup: RigidObjectCfg = None
    post: RigidObjectCfg = None
    combiner: ArticulationCfg = None


def _pedestal_cfg(prim_path: str, size_xy, height: float, colour) -> RigidObjectCfg:
    """A solid kinematic block: furniture that collides but is never pushed about.

    Kinematic rather than static so each environment can stand its own table or post where its
    placement wants it -- a static collider is baked at clone time and cannot be moved per
    environment. Solid rather than a top on legs: legs are four more colliders for the dog's feet
    to find and nothing in either demo touches them.
    """
    return RigidObjectCfg(
        prim_path=prim_path,
        spawn=sim_utils.CuboidCfg(
            size=(size_xy[0], size_xy[1], height),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            mass_props=sim_utils.MassPropertiesCfg(mass=20.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.9, dynamic_friction=0.7, restitution=0.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=colour, roughness=0.6)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, height / 2)))


def table_cfg(prim_path="/World/envs/env_.*/Table") -> RigidObjectCfg:
    return _pedestal_cfg(prim_path, props.TABLE_SIZE_M, props.TABLE_TOP_M, (0.42, 0.33, 0.24))


def post_cfg(prim_path="/World/envs/env_.*/Post") -> RigidObjectCfg:
    return _pedestal_cfg(prim_path, props.POST_SIZE_M, props.POST_TOP_M, (0.35, 0.36, 0.38))


def cup_cfg(usd_path: str, prim_path="/World/envs/env_.*/Cup") -> RigidObjectCfg:
    return RigidObjectCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=usd_path,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False, solver_position_iteration_count=8,
                solver_velocity_iteration_count=4, max_depenetration_velocity=1.0)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=props.CUP_NOMINAL_XY_M + (props.TABLE_TOP_M,)))


def combiner_cfg(usd_path: str, handle_torque_nm: float, door_torque_nm: float = 0.0,
                 prim_path="/World/envs/env_.*/CombinerBox") -> ArticulationCfg:
    """The scripted demo's enclosure, with the same drives on its two joints.

    Copied in substance from `demos/combiner/scene.make_combiner_cfg` so the two demos work the
    same mechanism: a free hinge with slight damping, and a lever whose return spring is sized by
    the torque it needs at 45 degrees. The latch is not a joint -- it is the runtime controller in
    `demos/combiner/latch.py`, stepped from `_apply_action` below.

    `door_torque_nm` adds a door closer, sized the same way as the lever's spring: the torque it
    takes to hold the door at 45 degrees, linear from closed (no preload). 0, the default, is the
    free door every result before 2026-09-25 was measured on.
    """
    from isaaclab.actuators import ImplicitActuatorCfg

    door_stiffness = door_torque_nm / math.radians(45.0) if door_torque_nm > 0 else 0.0
    door_effort = max(2.0, 1.5 * door_stiffness * math.radians(GEOMETRY.open_limit_deg))

    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=usd_path,
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=8,
                solver_velocity_iteration_count=4)),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.65, 0.0, props.POST_TOP_M), rot=(0.0, 0.0, 0.0, 1.0),
            joint_pos={"DoorHinge": 0.0, "HandleJoint": 0.0}, joint_vel={".*": 0.0}),
        actuators={
            "hinge": ImplicitActuatorCfg(joint_names_expr=["DoorHinge"], stiffness=door_stiffness,
                                         damping=0.08, effort_limit_sim=door_effort, velocity_limit_sim=2.0),
            "handle": ImplicitActuatorCfg(
                joint_names_expr=["HandleJoint"],
                stiffness=GEOMETRY.spring_stiffness(handle_torque_nm), damping=0.08,
                effort_limit_sim=GEOMETRY.spring_effort_limit(handle_torque_nm),
                velocity_limit_sim=3.0)},
    )


class UniFPDemoEnv(Go2D1PosForceEnv):
    """`Go2D1PosForceEnv` with furniture, a scripted goal and commanded jaws."""

    cfg: UniFPDemoEnvCfg

    def __init__(self, cfg: UniFPDemoEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._link5 = self._robot.body_names.index("Link5")
        self._link6 = self._robot.body_names.index("Link6")
        self._jaw_centre_local = torch.as_tensor(JAW_CENTRE_LINK6, device=self.device)
        self._approach_local = torch.as_tensor(LINK6_APPROACH_LOCAL, device=self.device)
        self._jaw_local = torch.as_tensor(LINK6_JAW_LOCAL, device=self.device)
        self._wrist_axis_local = {
            index: torch.as_tensor(axis, device=self.device)
            for index, axis in wrist.JOINT_LOCAL_AXES.items()}
        #: The link each wrist joint rotates: a URDF joint's axis is given in its child's frame.
        self._wrist_axis_body = {wrist.JOINT5_ACTION_INDEX: self._robot.body_names.index("Link5"),
                                 wrist.JOINT6_ACTION_INDEX: self._robot.body_names.index("Link6")}
        # Contact on the hand and on the forearm, by the *contact sensor's* body order, which is
        # not the articulation's. The fingers say whether a grasp has the object; the forearm says
        # whether the arm is leaning on the table on its way to it.
        self._finger_contacts, _ = self._arm_contact.find_bodies(
            ["Link7_1", "Link7_2"], preserve_order=True)
        self._forearm_contacts, _ = self._arm_contact.find_bodies(
            ["Link4", "Link5", "Link6"], preserve_order=True)

        self.scripts: list[DemoScript] = []
        self.sites: list[object] = []
        self.commands: list[Command | None] = [None] * self.num_envs
        self._goal_sphere = torch.zeros(self.num_envs, 3, device=self.device)
        # The frame each script works in: the robot's position and heading at its last reset.
        # Recorded rather than read live so a demo's straight line stays straight while the base
        # shuffles under it, and so a placement means the same thing for the whole attempt.
        self._spawn_pos = torch.zeros(self.num_envs, 3, device=self.device)
        self._spawn_yaw_quat = torch.zeros(self.num_envs, 4, device=self.device)
        self._spawn_yaw_quat[:, 0] = 1.0
        self._latches = []
        #: Put the cup or the box a metre to one side and run the script anyway. The control that
        #: separates two things a failed grasp confounds: how accurately the arm follows the demo
        #: path, and what happens when it meets the object. Set by the launcher before `reset`.
        self.hide_object = False
        #: Move the furniture aside as well, so the script's path runs in free space. Separates
        #: "the arm cannot follow this path" from "the arm is leaning on the table".
        self.hide_furniture = False
        #: Command the tool point straight at the script's target instead of converting so the
        #: *jaw centre* lands there. Diagnostic only -- a grasp is placed by the jaw centre, and a
        #: tool point 2 cm along the approach and up to 3.9 cm across it is not where the object
        #: has to be. It isolates one thing: the conversion rides on the hand's measured
        #: orientation, so an orientation that wanders makes the commanded goal wander with it.
        self.command_tip = False
        #: Force a constant jaw travel, ignoring the script. Diagnostic: UniFP's controlled point
        #: is the tip of the Link7_1 *finger*, the two jaws are dropped from the observation
        #: (`interface.single_obs`), and the policy trained with them held at their default. A
        #: grasp has to open them, which slides the controlled point up to 30 mm across the hand
        #: for a reason nothing the policy can see accounts for.
        self.jaw_override: float | None = None
        #: Correct the commanded goal for that slide. The policy learnt where its tool point is
        #: from joint angles alone, with the jaws at the default it trained in, so opening them
        #: leaves it tracking to a belief that is `travel` off along the jaw axis. Knowing the
        #: geometry, the goal can be offset by exactly that, which is the task interface doing the
        #: job the observation cannot. Off measures the cost of not doing it.
        self.jaw_compensation = True
        # The generator's own `step` is what writes `commands[3:6]`; replacing it is how a fixed
        # or scripted goal reaches the policy through the path the task already uses.
        self._goals.step = self._scripted_goal          # type: ignore[assignment]

    # --- scene ------------------------------------------------------------------------------------

    def _setup_scene(self):
        """The parent's scene, with the demo's props created before the environments are cloned.

        Order matters: every per-environment prim has to exist under `env_0` before
        `clone_environments` replicates it. The parent method does robot, sensor, terrain, clone,
        light in one go, so it is reproduced here with the props inserted rather than called.
        """
        self._robot = Articulation(self.cfg.robot)
        self.scene.articulations["robot"] = self._robot
        self._contact_sensor = ContactSensor(self.cfg.contact_sensor)
        self.scene.sensors["contact_sensor"] = self._contact_sensor
        self._arm_contact = ContactSensor(self.cfg.arm_contact)
        self.scene.sensors["arm_contact"] = self._arm_contact

        if self.cfg.task == "cup":
            self._table = RigidObject(self.cfg.table)
            self.scene.rigid_objects["table"] = self._table
            self._cup = RigidObject(self.cfg.cup)
            self.scene.rigid_objects["cup"] = self._cup
        else:
            self._post = RigidObject(self.cfg.post)
            self.scene.rigid_objects["post"] = self._post
            self._box = Articulation(self.cfg.combiner)
            self.scene.articulations["combiner"] = self._box

        self.cfg.terrain.spawn.func(self.cfg.terrain.prim_path, self.cfg.terrain.spawn)
        self.scene.clone_environments(copy_from_source=False)
        self.cfg.dome_light.spawn.func(self.cfg.dome_light.prim_path, self.cfg.dome_light.spawn)

    # --- standing ---------------------------------------------------------------------------------

    def _resample_velocity_commands(self, env_ids: torch.Tensor) -> None:
        """Stand. The demos are standing manipulation, so the base is never told to walk.

        Overriding the sampler rather than zeroing the buffer after each step matters: the parent
        resamples inside `_get_rewards`, which runs *before* the observation is built, so a
        command zeroed from outside would still be seen by the policy for one step every five
        seconds -- and a step of commanded walk is a step of gait phase.
        """
        self._commands[env_ids, :3] = 0.0

    # --- the hand ---------------------------------------------------------------------------------

    def _hand_frame(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """(jaw centre, approach axis, jaw axis) in the world, from the measured Link6 pose."""
        pos = self._robot.data.body_pos_w[:, self._link6]
        quat = self._robot.data.body_quat_w[:, self._link6]
        centre = pos + interface.quat_apply(quat, self._jaw_centre_local.expand_as(pos))
        approach = interface.quat_apply(quat, self._approach_local.expand_as(pos))
        jaw = interface.quat_apply(quat, self._jaw_local.expand_as(pos))
        return centre, approach, jaw

    def wrist_axes(self, action_indices) -> torch.Tensor:
        """The servoed joints' rotation axes in the world, as (N, 3, k).

        A URDF joint's axis is given in its child link's frame, so each is that link's measured
        orientation applied to the axis the URDF declares.
        """
        shape = (self.num_envs, 3)
        columns = [interface.quat_apply(
            self._robot.data.body_quat_w[:, self._wrist_axis_body[index]],
            self._wrist_axis_local[index].expand(shape)) for index in action_indices]
        return torch.stack(columns, dim=-1)

    def _tip_goal_from_jaw_centre(self, target: torch.Tensor) -> torch.Tensor:
        """The tool-point goal that puts the *jaw centre* on `target`.

        Two corrections, both from the measured hand rather than an assumed one:

        1. **Tool point to jaw centre.** UniFP commands the tip of the Link7_1 pincer, one finger;
           a grasp is placed by the point midway between the pads. They are about 20 mm apart
           along the approach and, with the jaws open, 39 mm across it.
        2. **The jaws' own travel.** `Joint7_1` slides the controlled point along the hand's jaw
           axis, and the two jaws are dropped from the policy's observation
           (`interface.single_obs` takes the first 18 of 20 joints), so nothing the policy sees
           says they moved. It tracks to where its trained geometry puts the tip -- the jaws shut
           -- and the real tip is `travel` away. Measured: 0.9 cm of tracking error with the jaws
           at their trained stop, 3.5 cm fully open, on the same goal. Adding the travel back
           gives the policy the goal its own belief needs.
        """
        centre, _, jaw_axis = self._hand_frame()
        tip = self._tip_pos() + self.scene.env_origins
        if self.jaw_compensation:
            # Joint7_1 sits at -y in Link6 and opens further along -y, so its travel takes the
            # tool point along the negative jaw axis; the belief is the tip plus that back.
            travel = self._robot.data.joint_pos[:, self._jaw_joint_ids][:, :1]
            tip = tip + travel * jaw_axis
        return target + (tip - centre)

    # --- the scripted goal --------------------------------------------------------------------

    def _scripted_goal(self) -> torch.Tensor:
        """Advance every environment's script one policy step and return the goal, in sphere terms.

        Called by the parent's `_get_rewards`, once per policy step, before the observation that
        carries the command is built. Returning spherical coordinates rather than writing world
        positions keeps the demo inside the task's own goal representation -- the same three
        numbers, in the same frame, with the same scales.
        """
        yaw_quat = self._base_yaw_quat()
        targets = torch.zeros(self.num_envs, 3, device=self.device)
        jaws = torch.zeros(self.num_envs, device=self.device)
        for index, demo in enumerate(self.scripts):
            command = demo.update(interface.POLICY_DT, self._state_of(index))
            self.commands[index] = command
            targets[index] = torch.as_tensor(command.point, device=self.device)
            jaws[index] = command.jaw_m if self.jaw_override is None else self.jaw_override
        # The script works in the robot's yaw frame at its spawn; the base may have shifted under
        # it, so the point is carried back into the world through the *current* yaw and position.
        world = self.scene.env_origins + self._spawn_pos + interface.quat_apply(
            self._spawn_yaw_quat, targets)
        tip_goal = world if self.command_tip else self._tip_goal_from_jaw_centre(world)
        self._jaw_targets = torch.stack((jaws, -jaws), dim=-1)

        centre = self._goal_centre() + self.scene.env_origins
        local = interface.quat_rotate_inverse(yaw_quat, tip_goal - centre)
        self._goal_sphere = interface.cart2sphere(local)
        # `_goal_world()`, the reward and the viewer overlay all read the generator's buffers
        # rather than this return value, so they are kept in step with it. Without this the
        # environment would score the policy against a goal nothing is commanding.
        self._goals.current = self._goal_sphere
        self._goals.start = self._goal_sphere
        self._goals.goal = self._goal_sphere
        return self._goal_sphere

    def _state_of(self, index: int) -> dict:
        """What a script's phase functions may read. Never the thing being worked."""
        state = {"env": index}
        site = self.sites[index]
        if self.cfg.task == "cup":
            state["cup_site"] = site
        else:
            state["box_site"] = site
        return state

    # --- physics ----------------------------------------------------------------------------------

    def _apply_action(self) -> None:
        super()._apply_action()
        for latch in self._latches:
            latch.on_physics_step(self.cfg.sim.dt)

    # --- truth, for the record ------------------------------------------------------------------

    def contact_state(self) -> dict:
        """Peak contact force on the fingers and on the forearm, newtons."""
        forces = self._arm_contact.data.net_forces_w
        peak = lambda ids: forces[:, ids].norm(dim=-1).max(dim=1).values
        return {"finger_n": peak(self._finger_contacts), "forearm_n": peak(self._forearm_contacts)}

    def cup_state(self) -> dict:
        pos = self._cup.data.root_pos_w - self.scene.env_origins
        quat = self._cup.data.root_quat_w
        up = interface.quat_apply(quat, torch.tensor(
            [0.0, 0.0, 1.0], device=self.device).expand_as(pos))
        return {"pos": pos, "lift_m": pos[:, 2] - props.TABLE_TOP_M,
                "tilt_deg": torch.rad2deg(torch.acos(up[:, 2].clamp(-1.0, 1.0)))}

    def box_state(self) -> dict:
        door = self._box.joint_names.index("DoorHinge")
        handle = self._box.joint_names.index("HandleJoint")
        q = self._box.data.joint_pos
        return {"door_deg": torch.rad2deg(q[:, door]), "handle_deg": torch.rad2deg(q[:, handle])}

    # --- resetting --------------------------------------------------------------------------------

    def place(self, sites: list) -> None:
        """Set each environment's placement. Call before `reset()`."""
        self.sites = list(sites)

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        super()._reset_idx(env_ids)
        if not self.sites:
            return
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        root = self._robot.data.root_pos_w - self.scene.env_origins
        self._spawn_pos[env_ids] = torch.stack(
            (root[env_ids, 0], root[env_ids, 1], torch.zeros_like(root[env_ids, 2])), dim=-1)
        self._spawn_yaw_quat[env_ids] = interface.yaw_quat(
            interface.yaw_from_quat(self._robot.data.root_quat_w[env_ids]))

        self._place_props(env_ids)
        centre, _, _ = self._hand_frame()
        for index in env_ids.tolist():
            self.scripts[index].reset(
                tuple((centre[index] - self.scene.env_origins[index]).tolist()),
                float(self._jaw_targets[index, 0]))

    def _place_props(self, env_ids: torch.Tensor) -> None:
        origins = self.scene.env_origins[env_ids]
        aside = 1.5 if self.hide_object else 0.0
        aside_furniture = 1.5 if self.hide_furniture else 0.0
        if self.cfg.task == "cup":
            table = torch.zeros(len(env_ids), 13, device=self.device)
            table[:, 3] = 1.0
            cup = table.clone()
            for row, index in enumerate(env_ids.tolist()):
                site = self.sites[index]
                table[row, :3] = torch.tensor(
                    (site.table_centre_x_m, aside_furniture, props.TABLE_TOP_M / 2),
                    device=self.device)
                base = site.cup_base_m
                cup[row, :3] = torch.tensor(
                    (base[0], base[1] + aside, base[2]), device=self.device)
            table[:, :3] += origins
            cup[:, :3] += origins
            self._table.write_root_pose_to_sim(table[:, :7], env_ids=env_ids)
            self._table.write_root_velocity_to_sim(table[:, 7:], env_ids=env_ids)
            self._cup.write_root_pose_to_sim(cup[:, :7], env_ids=env_ids)
            self._cup.write_root_velocity_to_sim(cup[:, 7:], env_ids=env_ids)
            return

        post = torch.zeros(len(env_ids), 13, device=self.device)
        post[:, 3] = 1.0
        box = post.clone()
        for row, index in enumerate(env_ids.tolist()):
            site = self.sites[index]
            root = site.root_m
            post[row, :3] = torch.tensor(
                (root[0], root[1] + aside_furniture, props.POST_TOP_M / 2), device=self.device)
            box[row, :3] = torch.tensor((root[0], root[1] + aside, root[2]), device=self.device)
            box[row, 3:7] = torch.tensor(site.quaternion, device=self.device)
        post[:, :3] += origins
        box[:, :3] += origins
        self._post.write_root_pose_to_sim(post[:, :7], env_ids=env_ids)
        self._post.write_root_velocity_to_sim(post[:, 7:], env_ids=env_ids)
        self._box.write_root_pose_to_sim(box[:, :7], env_ids=env_ids)
        self._box.write_root_velocity_to_sim(box[:, 7:], env_ids=env_ids)
        joints = torch.zeros(len(env_ids), self._box.num_joints, device=self.device)
        self._box.write_joint_state_to_sim(joints, torch.zeros_like(joints), env_ids=env_ids)
        for latch in self._latches:
            latch.reset(door_deg=0.0, handle_deg=0.0)

    # --- the jaws ---------------------------------------------------------------------------------

    def open_gripper_stop(self, travel_m: float) -> None:
        """Let the finger drives travel past the URDF's stop, as the scripted grip does (F-063).

        The real jaws shut 2 mm past where the URDF says they stop, which is what lets them squeeze
        an 18 mm bar. Only ever widens the limits, so calling it twice changes nothing.
        """
        ids = self._jaw_joint_ids
        limits = self._robot.data.joint_pos_limits[:, ids].clone()
        factor = float(self._robot.cfg.soft_joint_pos_limit_factor)
        span = float(limits[0, 0, 1] - limits[0, 0, 0])
        reach = (2 * (travel_m - 0.0005) - span * (1 - factor)) / (1 + factor)
        limits[..., 0] = torch.minimum(limits[..., 0], torch.full_like(limits[..., 0], reach))
        limits[..., 1] = torch.maximum(limits[..., 1], torch.full_like(limits[..., 1], -reach))
        self._robot.write_joint_position_limit_to_sim(limits, joint_ids=ids)

    def attach_latches(self) -> None:
        """One `LatchController` per environment, stepped at the physics rate from `_apply_action`."""
        from demos.combiner.latch import LatchController

        self._latches = []
        for index in range(self.num_envs):
            latch = LatchController(_SingleBoxView(self._box, index))
            latch.set_spring(self.cfg.handle_torque_nm)
            self._latches.append(latch)


class _SingleBoxView:
    """One environment's slice of the cloned box articulation, shaped as `LatchController` expects.

    `LatchController` was written for the scripted demo's single-environment scene and indexes
    `[0]` throughout. Rather than fork it -- the latch rule is the thing both demos must share,
    or they are not opening the same box -- this presents one environment of the batch under the
    same handful of names.
    """

    def __init__(self, box, index: int):
        self._box, self._index = box, index
        self.joint_names = box.joint_names
        self.device = box.device

    @property
    def data(self):
        return _SingleBoxData(self._box.data, self._index)

    def write_joint_position_limit_to_sim(self, limits, joint_ids=None, warn_limit_violation=True):
        self._box.write_joint_position_limit_to_sim(
            limits, joint_ids=joint_ids, env_ids=torch.tensor([self._index], device=self.device),
            warn_limit_violation=warn_limit_violation)

    def write_joint_stiffness_to_sim(self, value, joint_ids=None):
        self._box.write_joint_stiffness_to_sim(value, joint_ids=joint_ids)

    def write_joint_effort_limit_to_sim(self, value, joint_ids=None):
        self._box.write_joint_effort_limit_to_sim(value, joint_ids=joint_ids)

    def write_joint_state_to_sim(self, position, velocity, joint_ids=None):
        env_ids = torch.tensor([self._index], device=self.device)
        self._box.write_joint_state_to_sim(position, velocity, joint_ids=joint_ids, env_ids=env_ids)

    def set_joint_position_target(self, target, joint_ids=None):
        self._box.set_joint_position_target(
            target, joint_ids=joint_ids, env_ids=torch.tensor([self._index], device=self.device))

    def set_joint_velocity_target(self, target, joint_ids=None):
        self._box.set_joint_velocity_target(
            target, joint_ids=joint_ids, env_ids=torch.tensor([self._index], device=self.device))

    def set_joint_effort_target(self, target, joint_ids=None):
        self._box.set_joint_effort_target(
            target, joint_ids=joint_ids, env_ids=torch.tensor([self._index], device=self.device))

    def reset(self):
        pass

    def write_data_to_sim(self):
        self._box.write_data_to_sim()


class _SingleBoxData:
    def __init__(self, data, index: int):
        self._data, self._index = data, index

    @property
    def joint_pos(self):
        return self._data.joint_pos[self._index:self._index + 1]

    @property
    def joint_vel(self):
        return self._data.joint_vel[self._index:self._index + 1]

    @property
    def default_joint_pos(self):
        return self._data.default_joint_pos[self._index:self._index + 1]
