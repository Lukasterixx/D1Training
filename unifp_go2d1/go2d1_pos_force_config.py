"""UniFP's B2Z1 position/force task, retargeted to the welded Go2 + D1.

Adapted from UniFP `legged_gym/envs/b2/b2z1_pos_force_config.py` at
68847a070f88d731058c3d8476929bc3b205f5bd (BSD-3-Clause; see third_party/UniFP/NOTICE.md).
Structure, reward terms and algorithm settings are upstream's. What changed, and why:

  DOF layout   B2Z1 is 12 legs + 5 controlled arm joints + 2 held joints (z1_wrist_rotate and
               z1_jointGripper) = 19 DOFs, 17 actions. Go2+D1 is 12 legs + 6 arm joints + 2
               prismatic fingers = 20 DOFs, 18 actions. All six D1 revolute joints are actions,
               so this is an 18-action whole-body model, matching the Go2+D1 reference the
               plan audits. That widens every per-DOF observation by one: single obs 73 -> 76,
               single privileged obs 149 -> 153.

  Scale        The Go2 is roughly a quarter of the B2's mass and two thirds of its height, and
               the D1 is a far weaker arm than the Z1 (3.3 N.m on J1/J2, 1.7 N.m on J3-J6,
               against the Z1's tens of N.m). Base height target, spawn height, added-mass and
               CoM randomisation, contact-force penalties and the end-effector goal sphere are
               all resized. See the comments on each.

  Gains        Upstream's leg gains (300/500) are B2 gains. These are the Go2's 25/0.5, the
               same as the Isaac Lab task in this repo. The arm gains are NOT the Isaac Lab
               task's 4000/400: those are acceleration-drive gains authored by the URDF import
               (F-010), while UniFP computes explicit joint torques at 200 Hz, where the same
               numbers are both dimensionally different and unstable.

  Forces       THE IMPORTANT ONE. Upstream pushes the gripper with up to +/-60 N. The D1 cannot
               produce or resist that: at a ~0.45 m moment arm its strongest joint saturates
               near 7 N, so a +/-60 N command would be an instruction the arm can only fail.
               The end-effector force range here is +/-8 N. Force tracking on this arm is a
               few-newton problem, and any comparison with UniFP's reported numbers has to say so.

NOT VALIDATED: every number below is an engineering choice scaled from upstream or from this
repo's Isaac Lab task. None of it has been checked against a trained policy or the real robot.
"""

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO
import numpy as np


class Go2D1PosForceRoughCfg(LeggedRobotCfg):

    class goal_ee:

        num_commands = 3
        traj_time = [1, 3]
        hold_time = [0.5, 2]
        # Keep-out box for the end-effector goal, relative to the sphere centre. Upstream's
        # B2 box scaled by the ratio of reach (D1 ~0.62 m from shoulder, Z1 ~0.9 m).
        collision_upper_limits = [0.17, 0.14, -0.10]
        collision_lower_limits = [-0.48, -0.14, -0.55]
        underground_limit = -0.40
        num_collision_check_samples = 10
        command_mode = 'sphere'
        arm_induced_pitch = 0.38

        class sphere_center:
            # The D1 mounts on the base centreline (arm_mount_joint xyz = 0 0 0.08), not ahead
            # of it as the Z1 does on the B2, so there is no x offset. z is the shoulder
            # (d1_Joint2) height over flat terrain: 0.30 m standing base + 0.19 m of arm base.
            x_offset = 0.0
            y_offset = 0.0
            z_invariant_offset = 0.49

        class ranges:
            # Radius from the sphere centre. The D1's tip sits at r = 0.51 m at its zero pose
            # (from ZERO_ACTION_TIP_M and ZERO_ACTION_BASE_OFFSET_M in position_only/task_space.py)
            # and reaches about 0.62 m fully extended.
            init_pos_start = [0.50, np.pi / 5, 0]
            init_pos_end = [0.50, 0, 0]
            pos_l = [0.30, 0.58]
            pos_p = [-np.pi / 4, np.pi / 3]
            pos_y = [-2 * np.pi / 5, 2 * np.pi / 5]

            delta_orn_r = [-0.5, 0.5]
            delta_orn_p = [-0.5, 0.5]
            delta_orn_y = [-0.5, 0.5]

        sphere_error_scale = [1, 1, 1]
        orn_error_scale = [1, 1, 1]

    class init_state(LeggedRobotCfg.init_state):
        # Go2 stands at ~0.30 m (position_only/task_space.py SPAWN_HEIGHT_M, measured). Spawn a
        # little above it, as upstream spawns the B2 above its own standing height.
        pos = [0.0, 0.0, 0.35]
        default_joint_angles = {
            # Go2's stock stance, the same angles flat_env_cfg.py uses.
            'FL_hip_joint': 0.1,
            'FL_thigh_joint': 0.8,
            'FL_calf_joint': -1.5,

            'FR_hip_joint': -0.1,
            'FR_thigh_joint': 0.8,
            'FR_calf_joint': -1.5,

            'RL_hip_joint': 0.1,
            'RL_thigh_joint': 1.0,
            'RL_calf_joint': -1.5,

            'RR_hip_joint': -0.1,
            'RR_thigh_joint': 1.0,
            'RR_calf_joint': -1.5,

            # The D1's own zero pose, where the real arm homes on return_to_zero. The tip rests
            # 0.42 m ahead of and 0.49 m above the base link.
            'd1_Joint1': 0.0,
            'd1_Joint2': 0.0,
            'd1_Joint3': 0.0,
            'd1_Joint4': 0.0,
            'd1_Joint5': 0.0,
            'd1_Joint6': 0.0,
            # Jaws closed. Held at this angle; not policy actions.
            'd1_Joint7_1': 0.0,
            'd1_Joint7_2': 0.0,
        }
        rand_yaw_range = np.pi / 2
        origin_perturb_range = 0.5
        init_vel_perturb_range = 0.1

    class domain_rand:
        observe_priv = True
        randomize_friction = True
        friction_range = [0.3, 2.0]
        randomize_base_mass = True
        # Upstream adds up to 15 kg to a ~60 kg B2. The Go2 is ~12 kg plus a ~3 kg arm, so the
        # same fraction of body mass is about 3 kg.
        added_mass_range = [0., 3.]
        randomize_base_com = True
        # Upstream's +/-0.15 m is a large fraction of the Go2's 0.38 m trunk; +/-0.05 m is not.
        added_com_range_x = [-0.05, 0.05]
        added_com_range_y = [-0.05, 0.05]
        added_com_range_z = [-0.05, 0.05]
        randomize_leg_mass = False
        leg_mass_scale_range = [-0.20, 0.20]
        randomize_motor = True
        leg_motor_strength_range = [0.85, 1.15]
        arm_motor_strength_range = [0.85, 1.15]

        randomize_rigids_after_start = False
        randomize_restitution = False
        restitution_range = [0.0, 1.0]

        # A payload in the jaws, up to 0.2 kg. Kept at upstream's value: it is already a large
        # fraction of what the D1 can hold out at reach.
        randomize_gripper_mass = True
        gripper_added_mass_range = [0, 0.2]

        push_robots = True
        push_interval_s = 8
        max_push_vel_xy = 0.5

    class env(LeggedRobotCfg.env):

        num_gripper_joints = 2      # d1_Joint7_1, d1_Joint7_2: held, not actions
        num_actions = 18            # 12 legs + 6 D1 revolute joints
        num_torques = 18
        frame_stack = 32
        c_frame_stack = 3
        # 2 body orientation + 3 base ang vel + 18 dof pos + 18 dof vel + 18 actions
        # + 2 gait phase + 15 commands
        num_single_obs = 76
        num_pred_obs = 12
        num_observations = int(frame_stack * num_single_obs)
        # 3 base lin vel + 3 ee sphere + 3 ee force + 3 base force + 12 leg ref diff
        # + 22 mass params + 1 friction + 18 motor strength + 4 stance + 4 contact
        # + 3 gravity + 3 ang vel + 18 dof pos + 18 dof vel + 18 actions + 2 gait phase
        # + 15 commands + 3 force-offset sphere
        single_num_privileged_obs = 153
        num_privileged_obs = int(c_frame_stack * single_num_privileged_obs)

        observe_gait_commands = False
        frequencies = 1.0

        action_delay = 3
        teleop_mode = False

    class commands:
        curriculum = False
        max_curriculum = 1.
        num_commands = 15
        resampling_time = 5.
        heading_command = False

        class ranges:
            lin_vel_x = [-0.6, 0.6]
            lin_vel_y = [-0.4, 0.4]
            ang_vel_yaw = [-0.6, 0.6]
        ang_vel_yaw_clip = 0.2
        ang_vel_pitch_clip = 0.5
        lin_vel_x_clip = 0.1
        lin_vel_y_clip = 0.1

        zero_vel_cmd_prob = 0.3
        zero_vel_cmd_prob_after_force = 0.8

        # Push gripper
        push_gripper_stators = True
        push_gripper_interval_s_cmd = [3.5, 9.0]
        push_gripper_duration_s_cmd = [1.0, 3.0]
        gripper_forced_prob_cmd = 0.8
        push_gripper_interval_s_ext = [3.5, 9.0]
        push_gripper_duration_s_ext = [1.0, 3.0]
        gripper_forced_prob_ext = 0.8
        randomize_gripper_force_gains = True
        gripper_force_kp_range = [200., 200.]
        gripper_force_kd_range = [3.0, 3.0]
        gripper_prop_kd = 0.1

        # +/-8 N, not upstream's +/-60 N. The D1's strongest joint (3.3 N.m) makes about 7 N at
        # a 0.45 m moment arm, so this is the band in which force tracking is a real objective
        # rather than a saturated one. Raising it does not make the arm stronger.
        max_push_force_xyz_gripper_cmd = [-8, 8]
        max_push_force_xyz_gripper_ext = [-8, 8]

        settling_time_force_gripper_s = 1.0

        # Push base. Upstream's released step() calls _push_gripper only -- the base push is
        # commented out there, and is left that way here so the two runs match.
        push_robot_base = True
        push_base_interval_s_cmd = [3.5, 9.0]
        push_base_duration_s_cmd = [1.0, 3.0]
        base_forced_prob_cmd = 0.8
        push_base_interval_s_ext = [6.0, 12.0]
        push_base_duration_s_ext = [1.0, 3.0]
        base_forced_prob_ext = 0.8
        randomize_base_force_gains = True
        base_force_kp_range = [200., 200.]
        base_force_kd_range = [200.0, 200.0]
        base_prop_kd = 0.1

        # Same fraction of body weight as upstream's +/-50 N on a 60 kg B2.
        max_push_force_xyz_base_cmd = [-20, 20]
        max_push_force_xyz_base_ext = [-20, 20]

        force_z_base_ext_scale = 0.1

        settling_time_force_base_s = 3.0

        # Iterations of position-only training before any external force is applied.
        force_start_step = 8000

    class terrain:
        mesh_type = 'trimesh'
        hf2mesh_method = "fast"
        max_error = 0.1
        horizontal_scale = 0.05
        vertical_scale = 0.005
        border_size = 25
        height = [0.00, 0.05]
        gap_size = [0.02, 0.1]
        stepping_stone_distance = [0.02, 0.08]
        downsampled_scale = 0.075
        curriculum = False

        all_vertical = False
        no_flat = True

        static_friction = 1.0
        dynamic_friction = 1.0
        restitution = 0.

        measure_heights = True
        measured_points_x = [-0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]

        selected = False
        terrain_kwargs = None
        max_init_terrain_level = 5
        terrain_length = 8.
        terrain_width = 8.
        num_rows = 10
        num_cols = 20

        terrain_dict = {"smooth slope": 0.,
                        "rough slope up": 0.,
                        "rough slope down": 0.,
                        "rough stairs up": 0.,
                        "rough stairs down": 0.,
                        "discrete": 0.,
                        "stepping stones": 0.,
                        "gaps": 0.,
                        "rough flat": 1.0,
                        "pit": 0.0,
                        "wall": 0.0}
        terrain_proportions = list(terrain_dict.values())
        slope_treshold = None
        origin_zero_z = False

    class control(LeggedRobotCfg.control):
        control_type = 'P'
        # Legs: the Go2's gains, as used by this repo's Isaac Lab task. Arm: explicit-torque
        # gains for a 200 Hz PD loop, sized so gravity droop (torque / stiffness) stays within
        # a couple of degrees at the published effort limits -- 3.3 N.m on J1/J2 against 60
        # N.m/rad is 0.055 rad. PhysX still clips at the URDF effort limit, so stiffness buys
        # tracking and never strength (the same point flat_env_cfg.py makes).
        stiffness = {'hip': 25., 'thigh': 25., 'calf': 25.,
                     'd1_Joint1': 60., 'd1_Joint2': 60., 'd1_Joint3': 40.,
                     'd1_Joint4': 40., 'd1_Joint5': 40., 'd1_Joint6': 40.}
        damping = {'hip': 0.5, 'thigh': 0.5, 'calf': 0.5,
                   'd1_Joint1': 1.0, 'd1_Joint2': 1.0, 'd1_Joint3': 0.8,
                   'd1_Joint4': 0.8, 'd1_Joint5': 0.8, 'd1_Joint6': 0.8}
        # The two finger joints are not actions; they are held at their default by a separate
        # PD, which upstream hardcodes at 64/1.5 for the Z1's revolute gripper. These are
        # N/m and N.s/m for the D1's prismatic jaws: 800 N/m reaches the 15 N effort limit at
        # 19 mm, inside the 30 mm stroke, and stays stable explicitly at dt = 0.005 s.
        gripper_stiffness = 800.
        gripper_damping = 8.
        action_scale = 0.25
        decimation = 4

    class arm:
        init_target_ee_base = [0.2, 0.0, 0.2]
        grasp_offset = 0.08

    class asset(LeggedRobotCfg.asset):
        file = 'resources/robots/go2d1/go2d1.urdf'
        name = "go2d1"
        foot_name = "foot"
        thigh_name = "thigh"
        # Added by unifp_go2d1/build_asset.py at the CAD pincer tip (Link7_1 + TOOL_OFFSET_M),
        # the same controlled point as the Isaac Lab position-only task. CAD, not measured.
        gripper_name = "ee_gripper_link"
        penalize_contacts_on = ["thigh", "calf", "base_link"]
        terminate_after_contacts_on = []
        self_collisions = 0
        # These meshes DO need flipping: with this False the Go2 renders on its side with its
        # legs splayed flat. Measured to be visual-only -- masses identical and every body within
        # 0.0000 mm after a 2 s settle either way, because the Go2's collisions are all box,
        # cylinder and sphere primitives and the flag does not touch the arm's collision meshes.
        # So it changes screenshots and video, never the policy.
        flip_visual_attachments = True

    class rewards(LeggedRobotCfg.rewards):
        gait_vel_sigma = 2.0
        gait_force_sigma = 2.0
        kappa_gait_probs = 0.07

        only_positive_rewards = False
        tracking_sigma = 0.25
        tracking_ee_sigma = 1.0
        soft_dof_pos_limit = 0.8
        soft_dof_vel_limit = 1.
        soft_torque_limit = 0.9
        # Go2 standing height, measured in this repo (task_space.SPAWN_HEIGHT_M).
        base_height_target = 0.30
        # Upstream's 200 N is a B2 number. Scaled by mass: a ~15 kg Go2 carries ~150 N total.
        max_contact_force = 100.

        cycle_time = 0.64
        target_joint_pos_scale = 0.17
        target_joint_pos_thd = 0.5

        sigma_force = 1 / 50

        class scales:

            feet_contact_number = 2.0

            tracking_lin_vel_force_world = 2.0
            tracking_ang_vel = 1.0

            torques = -5.e-6
            stand_still = 0.5
            ref_dof_leg = 1.0
            alive = 1.5
            lin_vel_z = -1.5
            feet_air_time = 1.0
            feet_height = 1.0
            ang_vel_xy = -0.02
            dof_acc = -2.5e-7
            dof_vel = -8.e-4
            dof_acc_arm = -4.5e-7
            dof_vel_arm = -2.e-4
            collision = -5.
            action_rate = -0.02
            action_rate_arm = -0.045
            dof_pos_limits = -10.0
            torque_limits = -0.005
            hip_pos = -0.5
            feet_drag = -0.0008
            feet_contact_forces = -0.001
            base_height = -2.0
            feet_pos_xy = -0.5
            feet_height_high = -15

            arm_termination = 0.
            tracking_ee_sphere = 0.
            tracking_ee_force_world = 2.0
            tracking_ee_sphere_walking = 0.0
            tracking_ee_sphere_standing = 0.0
            tracking_ee_cart = 0.
            arm_orientation = 0.
            arm_energy_abs_sum = 0.
            tracking_ee_orn = 0.
            tracking_ee_orn_ry = 0.


class Go2D1PosForceRoughCfgPPO(LeggedRobotCfgPPO):
    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01

    class policy:
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [512, 256, 128]

    class runner(LeggedRobotCfgPPO.runner):
        run_name = ''
        experiment_name = 'go2d1_pos_force'
