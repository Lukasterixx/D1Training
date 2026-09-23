"""UniFP's position/force task as an Isaac Lab `DirectRLEnv`.

The parts that decide whether a trained policy is the same policy — the observation contract, the
DOF permutation, the control law, the robot, the goal generator, the gait clock and all 27 reward
terms — are imported from modules that are each checked against the environment that trained
`model_48800` (F-087, and `tests/test_unifp_train.py`). What lives here is the wiring: buffers,
resets, terminations, command sampling, and the order operations happen in.

That order is itself part of the contract, and it is upstream's:

    1. advance the episode clock and refresh derived quantities
    2. resample velocity commands on their timer, advance the gait phase
    3. step the end-effector goal along its trajectory
    4. compute rewards from the **post-step** state
    5. check terminations, reset whoever finished
    6. build observations and push them onto the history rings

Step 4 before step 6 matters: a reward is scored against the state the observation will then
report, and getting it the other way round scores the previous step's behaviour.

External forces follow UniFP's curriculum: nothing for the first `cfg.force_start_step` policy
steps (8,000 iterations by default), then the two gripper push channels of `forces.py` -- one
written into `commands[9:12]` for the policy to obey, one applied as a real wrench for it to
survive. The base push is dead code upstream and is not implemented here; `commands[12:15]` are
zero for the whole run, which is what the trained checkpoint saw.
"""
from __future__ import annotations

import torch

from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sensors import ContactSensor

from unifp_isaaclab import interface, task as goal_task
from unifp_isaaclab.policy import ObsHistory

from . import debug_vis, forces, gait, observations, rewards, task_cfg
from .env_cfg import Go2D1PosForceEnvCfg


class Go2D1PosForceEnv(DirectRLEnv):
    cfg: Go2D1PosForceEnvCfg

    def __init__(self, cfg: Go2D1PosForceEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # UniFP's DOF order against this articulation's. The single most dangerous thing to get
        # wrong in the whole port, so it is resolved by name and raises rather than guessing.
        order = interface.permutation_from(list(self._robot.joint_names))
        self._order = torch.as_tensor(order, device=self.device)
        self._action_joint_ids = self._order[:interface.NUM_ACTIONS]
        self._jaw_joint_ids = self._order[interface.NUM_ACTIONS:]

        # Two different body orderings, and they are not the same list. Kinematics
        # (`robot.data.body_pos_w`) is indexed by the articulation's body order; contact forces
        # (`contact_sensor.data.net_forces_w`) by the sensor's. Using one index against the other
        # reads whatever sits at that slot, and on GPU it surfaces as a device-side assert several
        # calls away from the mistake rather than as an IndexError.
        self._tip_body = self._robot.body_names.index(interface.TOOL_BODY)
        self._feet_bodies = [self._robot.body_names.index(name) for name in gait.FEET]
        self._thigh_bodies = [self._robot.body_names.index(f"{leg}_thigh")
                              for leg in ("FL", "FR", "RL", "RR")]
        self._feet_contacts, _ = self._contact_sensor.find_bodies(
            list(gait.FEET), preserve_order=True)
        self._penalised_contacts, _ = self._contact_sensor.find_bodies(
            list(task_cfg.PENALISED_CONTACT_BODIES))

        zeros = lambda *shape: torch.zeros(*shape, device=self.device)
        self._actions = zeros(self.num_envs, interface.NUM_ACTIONS)
        self._last_actions = zeros(self.num_envs, interface.NUM_ACTIONS)
        self._last_dof_vel = zeros(self.num_envs, interface.NUM_DOF)
        self._commands = zeros(self.num_envs, interface.NUM_COMMANDS)
        self._gait_phase = zeros(self.num_envs)
        self._feet_air_time = zeros(self.num_envs, 4)
        self._last_contacts = torch.zeros(self.num_envs, 4, dtype=torch.bool, device=self.device)

        # The tool takes the external wrench at the point the goal is measured at, not at
        # `Link7_1`'s centre of mass: upstream's asset carries a 1 g `ee_gripper_link` fixed
        # `TOOL_OFFSET_M` from `Link7_1` and pushes *that*. The weld here has no such body, so the
        # offset is passed to the wrench composer instead, which turns it into the same force plus
        # moment on `Link7_1`.
        self._tip_body_ids = torch.tensor([self._tip_body], dtype=torch.int32, device=self.device)
        self._tool_offset = torch.as_tensor(
            interface.TOOL_OFFSET_M, device=self.device).expand(self.num_envs, 1, 3).contiguous()
        self._forces = forces.GripperForces(self.num_envs, self.device)
        self._ee_force_w = zeros(self.num_envs, 3)
        #: Never written -- upstream's base push is commented out of its `step()`. Kept as a named
        #: buffer rather than a literal zero so the reward and observation calls read the same way
        #: as the end-effector ones, and so turning it on is one schedule rather than a rewrite.
        self._base_force_w = zeros(self.num_envs, 3)
        self._wrench_active = False
        #: Degenerate ranges upstream (`[200, 200]`), so constants rather than per-episode draws.
        self._gripper_force_kp = torch.full(
            (self.num_envs, 3), task_cfg.GRIPPER_FORCE_KP, device=self.device)
        self._base_force_kd = torch.full(
            (self.num_envs, 3), task_cfg.BASE_FORCE_KD, device=self.device)

        self._goals = goal_task.EeGoalTrajectory(self.num_envs, device=str(self.device))
        self._actor_history = ObsHistory(self.num_envs, device=str(self.device))
        self._critic_history = ObsHistory(
            self.num_envs, device=str(self.device),
            frame_stack=observations.CRITIC_FRAME_STACK,
            num_single_obs=observations.NUM_PRIVILEGED_OBS)

        self._default_dof_pos = torch.as_tensor(interface.DEFAULT_DOF_POS, device=self.device)
        self._jaw_targets = self._default_dof_pos[interface.NUM_ACTIONS:].expand(
            self.num_envs, -1).clone()

        self._weights = {name: weight for name, weight in task_cfg.scaled_weights().items()}
        self._episode_sums = {name: zeros(self.num_envs) for name in self._weights}

        self._command_interval = int(task_cfg.COMMAND_RESAMPLING_TIME_S / interface.POLICY_DT)

    # --- scene ------------------------------------------------------------------------------------

    def _setup_scene(self):
        self._robot = Articulation(self.cfg.robot)
        self.scene.articulations["robot"] = self._robot
        self._contact_sensor = ContactSensor(self.cfg.contact_sensor)
        self.scene.sensors["contact_sensor"] = self._contact_sensor
        self.cfg.terrain.spawn.func(self.cfg.terrain.prim_path, self.cfg.terrain.spawn)
        self.scene.clone_environments(copy_from_source=False)
        self.cfg.dome_light.spawn.func(self.cfg.dome_light.prim_path, self.cfg.dome_light.spawn)

    # --- acting -----------------------------------------------------------------------------------

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._last_actions = self._actions
        self._actions = torch.clip(actions, -interface.CLIP_ACTIONS, interface.CLIP_ACTIONS)
        self._joint_targets = interface.joint_targets(self._actions)
        self._step_forces()

    def _step_forces(self) -> None:
        """Advance the push schedule and write the wrench, once per policy step.

        `common_step_counter` is incremented after the physics loop, so here it holds the number
        of policy steps already completed -- which is exactly the counter upstream gates on
        (`global_steps > force_start_step * num_steps_per_env`).

        Upstream evaluates its schedule inside the decimation loop and re-applies the wrench every
        physics substep. The wrench composer is permanent, so writing it once per policy step and
        letting `write_data_to_sim` re-apply it each substep is the same thing; see `forces.py`
        for what stepping the *schedule* once instead of four times does and does not change.
        """
        if self.common_step_counter <= self.cfg.force_start_step:
            return
        commanded, external = self._forces.step(self.episode_length_buf)
        self._commands[:, interface.CMD_EE_FORCE] = commanded
        self._ee_force_w = external
        self._write_wrench()

    def _write_wrench(self) -> None:
        """Push the tool with `self._ee_force_w`, expressed in the world frame.

        The composer stores everything in the link frame and only converts for you against link
        poses it caches on first use and refreshes on `reset()` -- which the permanent composer
        never gets, so `is_global=True` would silently freeze the tool's orientation at whatever
        it was the first time this ran. The rotation is done here instead, against the body
        quaternion read this step.
        """
        quat = self._robot.data.body_quat_w[:, self._tip_body]
        local = interface.quat_rotate_inverse(quat, self._ee_force_w)
        self._robot.permanent_wrench_composer.set_forces_and_torques(
            forces=local.unsqueeze(1).contiguous(), positions=self._tool_offset,
            body_ids=self._tip_body_ids)
        self._wrench_active = True

    def _apply_action(self) -> None:
        """Called once per physics step, so the PD actuator recomputes torque at 200 Hz."""
        self._robot.set_joint_position_target(self._joint_targets, joint_ids=self._action_joint_ids)
        self._robot.set_joint_position_target(self._jaw_targets, joint_ids=self._jaw_joint_ids)

    # --- state ------------------------------------------------------------------------------------

    def _dof(self) -> tuple[torch.Tensor, torch.Tensor]:
        return self._robot.data.joint_pos[:, self._order], self._robot.data.joint_vel[:, self._order]

    def _base_yaw_quat(self) -> torch.Tensor:
        return interface.yaw_quat(interface.yaw_from_quat(self._robot.data.root_quat_w))

    def _tip_pos(self) -> torch.Tensor:
        """The controlled point -- `Link7_1` plus the CAD pincer offset -- in the *env* frame.

        Every position this class hands to `rewards` or `observations` has `scene.env_origins`
        taken off it, and the reason is worth stating because getting it wrong is silent. Upstream
        works in one global frame throughout, where the origins cancel inside each difference;
        Isaac Lab spaces its environments on a grid, so a position with the origin still in it and
        one without differ by metres. Mixing the two conventions leaves the tool-tip error at the
        grid spacing rather than the tracking error, which drives `tracking_ee_force_world` --
        `exp(-2 * error)` -- to exactly zero for every environment except one at the origin, and
        nothing else in the task changes at all.
        """
        pos = self._robot.data.body_pos_w[:, self._tip_body]
        quat = self._robot.data.body_quat_w[:, self._tip_body]
        offset = torch.as_tensor(interface.TOOL_OFFSET_M, device=self.device).expand_as(pos)
        return pos + interface.quat_apply(quat, offset) - self.scene.env_origins

    def _contact_forces(self, sensor_bodies) -> torch.Tensor:
        """Net contact force, indexed by the **contact sensor's** body order."""
        return self._contact_sensor.data.net_forces_w[:, sensor_bodies]

    def _task_state(self) -> rewards.TaskState:
        dof_pos, dof_vel = self._dof()
        yaw = self._base_yaw_quat()
        feet_forces = self._contact_forces(self._feet_contacts)
        return rewards.TaskState(
            base_lin_vel_b=self._robot.data.root_lin_vel_b,
            base_ang_vel_b=self._robot.data.root_ang_vel_b,
            root_pos_w=self._robot.data.root_pos_w - self.scene.env_origins,
            dof_pos=dof_pos, dof_vel=dof_vel, last_dof_vel=self._last_dof_vel,
            torques=self._robot.data.applied_torque[:, self._order],
            actions=self._actions, last_actions=self._last_actions,
            commands=self._commands, gait_phase=self._gait_phase,
            ref_leg_pos=gait.reference_leg_pos(self._gait_phase),
            stance_mask=gait.stance_mask(self._gait_phase),
            contact_mask=(feet_forces[:, :, 2] > 5.0).float(),
            feet_pos_w=self._robot.data.body_pos_w[:, self._feet_bodies] - self.scene.env_origins.unsqueeze(1),
            feet_contact_forces=feet_forces,
            feet_air_time=self._feet_air_time,
            penalised_contact_forces=self._contact_forces(self._penalised_contacts),
            ee_pos_w=self._tip_pos(), ee_goal_w=self._goal_world(),
            thigh_pos_w=self._robot.data.body_pos_w[:, self._thigh_bodies] - self.scene.env_origins.unsqueeze(1),
            feet_vel_w=self._robot.data.body_lin_vel_w[:, self._feet_bodies],
            last_contacts=self._last_contacts, base_yaw_quat=yaw,
            # "Measured" is upstream's word for it and it is worth being clear that there is no
            # sensor: the reward reads back the force the task itself applied, not a load cell.
            ee_force_measured=self._ee_force_w,
            ee_force_cmd=self._commands[:, interface.CMD_EE_FORCE],
            base_force_measured=self._base_force_w,
            base_force_cmd=self._commands[:, interface.CMD_BASE_FORCE],
            gripper_force_kp=self._gripper_force_kp,
            base_force_kd=self._base_force_kd)

    def _goal_centre(self) -> torch.Tensor:
        return goal_task.goal_sphere_center(
            self._robot.data.root_pos_w - self.scene.env_origins, self._base_yaw_quat())

    def _goal_world(self) -> torch.Tensor:
        return self._goal_centre() + interface.quat_apply(
            self._base_yaw_quat(), interface.sphere2cart(self._goals.current))

    # --- the step ---------------------------------------------------------------------------------

    def _get_observations(self) -> dict:
        dof_pos, dof_vel = self._dof()
        actor = interface.single_obs(
            self._robot.data.root_quat_w, self._robot.data.root_ang_vel_b,
            dof_pos, dof_vel, self._actions, self._gait_phase, self._commands)
        actor = torch.clip(actor, -interface.CLIP_OBSERVATIONS, interface.CLIP_OBSERVATIONS)

        yaw = self._base_yaw_quat()
        feet_forces = self._contact_forces(self._feet_contacts)
        critic = observations.privileged_obs(
            base_lin_vel_b=self._robot.data.root_lin_vel_b,
            ee_pos_w=self._tip_pos(), ee_goal_w=self._goal_world(),
            goal_centre_w=self._goal_centre(), base_yaw_quat=yaw,
            ee_force_w=self._ee_force_w,
            base_force_w=self._base_force_w,
            gripper_force_kp=self._gripper_force_kp,
            ee_force_cmd=self._commands[:, interface.CMD_EE_FORCE],
            leg_ref_diff=dof_pos[:, :12] - gait.reference_leg_pos(self._gait_phase),
            mass_params=torch.zeros(self.num_envs, 22, device=self.device),
            friction=torch.ones(self.num_envs, 1, device=self.device),
            motor_strength=torch.ones(self.num_envs, interface.NUM_ACTIONS, device=self.device),
            stance_mask=gait.stance_mask(self._gait_phase),
            contact_mask=(feet_forces[:, :, 2] > 5.0).float(),
            projected_gravity=self._robot.data.projected_gravity_b,
            base_ang_vel_b=self._robot.data.root_ang_vel_b,
            dof_pos=dof_pos, dof_vel=dof_vel, actions=self._actions,
            gait_phase=self._gait_phase, commands=self._commands)
        critic = torch.clip(critic, -interface.CLIP_OBSERVATIONS, interface.CLIP_OBSERVATIONS)

        return {
            "policy": self._actor_history.append(actor),
            "critic": self._critic_history.append(critic),
            # The adaptation module's supervised target. It is a *group of the observation* rather
            # than something handed to the algorithm separately because that is how rsl-rl 5.x
            # carries per-transition data: the rollout storage keeps every group the environment
            # returns, so the estimator loss can read it out of the same mini-batch the PPO loss
            # used. No model consumes it -- `obs_groups` maps only "policy" and "critic".
            "estimates": critic[:, :observations.NUM_ESTIMATES],
        }

    def _get_rewards(self) -> torch.Tensor:
        # Resample velocity commands on their timer and advance the gait before scoring, which is
        # the order upstream's `_post_physics_step_callback` runs in.
        due = (self.episode_length_buf % self._command_interval == 0).nonzero(as_tuple=False).flatten()
        if len(due) > 0:
            self._resample_velocity_commands(due)
        self._gait_phase = interface.gait_step(self._gait_phase, self._commands)
        self._commands[:, interface.CMD_EE_RADIUS:interface.CMD_EE_YAW + 1] = self._goals.step()

        state = self._task_state()
        total = torch.zeros(self.num_envs, device=self.device)
        for name, weight in self._weights.items():
            value = rewards.TERMS[name](state) * weight
            total += value
            self._episode_sums[name] += value
        # feet_air_time advanced these in place; keep them for the next step.
        self._feet_air_time, self._last_contacts = state.feet_air_time, state.last_contacts
        self._last_dof_vel = state.dof_vel.clone()
        return total

    # --- the viewer overlay -------------------------------------------------------------------

    def _set_debug_vis_impl(self, enable: bool) -> None:
        """Isaac Lab's hook for `set_debug_vis`. Allocates nothing until something asks for it."""
        if enable:
            if not hasattr(self, "_markers"):
                self._markers = debug_vis.TaskMarkers(
                    "/Visuals/unifp_task", self.num_envs, str(self.device))
            self._markers.set_visibility(True)
        elif hasattr(self, "_markers"):
            self._markers.set_visibility(False)

    def _debug_vis_callback(self, event) -> None:
        """Re-place every marker for the current frame.

        Positions held by this class have `scene.env_origins` taken off them (see `_tip_pos`), and
        markers are placed in the world, so the origins go back on here. Doing that in one place,
        at the point of drawing, is why the rest of the class can stay in one frame.
        """
        origins = self.scene.env_origins
        state = self._task_state()
        goal_w = self._goal_world() + origins
        trajectory = goal_task.trajectory_samples(
            self._goals, debug_vis.TRAJECTORY_SAMPLES, self._base_yaw_quat(), self._goal_centre())
        self._markers.update(
            goal_w=goal_w,
            target_w=rewards.ee_target(state) + origins,
            tip_w=self._tip_pos() + origins,
            centre_w=self._goal_centre() + origins,
            trajectory_w=trajectory + origins.unsqueeze(1),
            tip_force_measured_w=self._ee_force_w,
            tip_force_commanded_w=interface.quat_apply(
                self._base_yaw_quat(), self._commands[:, interface.CMD_EE_FORCE]),
            base_pos_w=self._robot.data.root_pos_w,
            base_force_measured_w=self._base_force_w,
            base_force_commanded_w=interface.quat_apply(
                self._base_yaw_quat(), self._commands[:, interface.CMD_BASE_FORCE]))

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Upstream terminates on orientation alone — its contact-termination list is empty."""
        roll_pitch = interface.body_roll_pitch(self._robot.data.root_quat_w)
        fallen = (roll_pitch[:, 1].abs() > 1.0) | (roll_pitch[:, 0].abs() > 0.8)
        timed_out = self.episode_length_buf >= self.max_episode_length - 1
        return fallen, timed_out

    # --- resetting --------------------------------------------------------------------------------

    def _resample_velocity_commands(self, env_ids: torch.Tensor) -> None:
        """Draw base velocity commands, then zero the ones that amount to standing.

        Two separate zeroings, both upstream's: a flat `ZERO_VEL_CMD_PROB` chance of being told to
        stand outright, and then a dead-zone clip that turns any command too small to be worth
        walking into an exact zero. Without the second, the robot is asked to creep.
        """
        count = len(env_ids)
        draw = lambda span: torch.rand(count, device=self.device) * (span[1] - span[0]) + span[0]
        self._commands[env_ids, 0] = draw(task_cfg.LIN_VEL_X_RANGE)
        self._commands[env_ids, 1] = draw(task_cfg.LIN_VEL_Y_RANGE)
        self._commands[env_ids, 2] = draw(task_cfg.ANG_VEL_YAW_RANGE)

        standing = torch.rand(count, device=self.device) < task_cfg.ZERO_VEL_CMD_PROB
        self._commands[env_ids, :3] *= (~standing).unsqueeze(1)

        moving = (
            (self._commands[env_ids, 0].abs() > interface.LIN_VEL_X_CLIP)
            | (self._commands[env_ids, 1].abs() > interface.LIN_VEL_Y_CLIP)
            | (self._commands[env_ids, 2].abs() > interface.ANG_VEL_YAW_CLIP)
        )
        self._commands[env_ids, :3] *= moving.unsqueeze(1)

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        super()._reset_idx(env_ids)

        joint_pos = self._robot.data.default_joint_pos[env_ids].clone()
        joint_vel = self._robot.data.default_joint_vel[env_ids].clone()
        self._robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)

        root_state = self._robot.data.default_root_state[env_ids].clone()
        root_state[:, :3] += self.scene.env_origins[env_ids]
        self._robot.write_root_pose_to_sim(root_state[:, :7], env_ids)
        self._robot.write_root_velocity_to_sim(root_state[:, 7:], env_ids)

        self._actions[env_ids] = 0.0
        self._last_actions[env_ids] = 0.0
        self._last_dof_vel[env_ids] = 0.0
        self._gait_phase[env_ids] = 0.0
        self._feet_air_time[env_ids] = 0.0
        self._last_contacts[env_ids] = False
        self._commands[env_ids] = 0.0
        self._forces.reset(env_ids)
        self._ee_force_w[env_ids] = 0.0
        if self._wrench_active:
            # The composer holds the last wrench written until it is overwritten, so a reset has
            # to clear it or the new episode starts already being pushed.
            self._write_wrench()
        self._goals.reset(env_ids)
        self._resample_velocity_commands(env_ids)
        # Zeroed rather than seeded with the reset pose: that is what the policy trained against,
        # so the first 32 steps of an episode see a mostly-empty history by design.
        self._actor_history.reset(env_ids)
        self._critic_history.reset(env_ids)

        extras = {}
        for name, total in self._episode_sums.items():
            extras[f"Episode_Reward/{name}"] = torch.mean(
                total[env_ids] / self.max_episode_length_s)
            total[env_ids] = 0.0
        self.extras.setdefault("log", {}).update(extras)
