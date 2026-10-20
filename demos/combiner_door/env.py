"""The combiner door demo's environment: the decoupled door pull, finding the box with the head camera and verifying
the door with the MID-360.

On top of `demos.unifp.decoupled.DecoupledDoorEnv` (the walking policy on the legs, inverse kinematics on the arm, the
door pull's script, scene, placements and record):
- **The robot starts `LOOK_BACK_M` behind the door pull's stance**, where the head camera sees the whole tag at every
  placement (from the stance itself it is cut off at a quarter of them). The script's frame is the stance, so the
  placements and everything the script does from the stance on are the door pull's.
- **Before the task: look, then walk in** (`pre_phases`). While standing after the spawn settles, every camera frame of
  the `look` phase is searched for the tag and every lidar frame kept; at its end the box placement the script works
  from (`script_site`, `_state_of`) is made as P2Dingo's docking makes its fix: the tag's centre for the position, the
  door's face in the lidar round it for the heading (`perception.lidar_estimate`). If no camera frame finds the tag, the
  attempt holds where it is (`no_tag`). Then the arm unfolds (`unfold`), and the base walks straight in to the stance
  and stands (`walk_in`, `walk_settle`), the hand at the home point relative to the base, high and out of both sensors'
  view.
- **The arm starts folded**, in the real D1's rest pose against its mechanical stop (`d1_hardware.REST_SERVO_DEG`,
  inside the URDF's limits), and stays there through the spawn and the look: an attempt that finds no tag never unfolds
  it. The unfold is a straight line in joint space to the home point's IK solution, stepped from the measured pose as
  the real arm is (`DecoupledDoorEnv.joint_phases`; F-149 found a folded arm handed to a Cartesian goal turns its hand
  down onto the dog's head).
- **From then on the MID-360 reads the door's angle** at its 10 Hz frame rate (`mid360.door_angle`): the same face fit,
  laid where the tag is at the last reading, in the box frame the look estimated. A check on the door that neither the
  script nor the hand provides; it steers nothing.

What the robot is told, and what it is not: the placement comes from the camera and the lidar, and the base's pose since
the attempt began from the simulator (odometry, as for the walk). The door's angle is never read from the simulator;
the truth is only recorded beside the lidar's reading.

Needs the Isaac app: import after `AppLauncher`.
"""
from __future__ import annotations

import math

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.sensors import TiledCamera
from isaaclab.sensors.ray_caster import MultiMeshRayCaster

import d1_hardware
import d1_ik
from demos.combiner.apriltag import TagDetector
from demos.unifp import props, script as demo_script
from demos.unifp.decoupled import DecoupledDoorEnv
from unifp_isaaclab import interface

from . import head_camera, mid360, perception

LOOK_BACK_M = perception.LOOK_BACK_M
#: The look, the walk in and the pause after it, seconds. The look is long enough for the lidar's frames to add up
#: (ten at 10 Hz).
LOOK_S = 1.0
#: The walk in's cap: it ends when the odometry has the base at the stance (`ARRIVED_M`), which at this policy's
#: actual pace for a 0.06 m/s command (about half that) takes ~5 s.
WALK_IN_S = 6.0
WALK_SETTLE_S = 1.5
#: The unfold from the rest pose to the home point, seconds: 97 deg of J2 at 4 deg a 10 Hz command is 2.4 s, and the arm
#: lags its commands (`DecoupledDoorEnv.joint_step_rad`).
UNFOLD_S = 3.5
#: Close enough to the stance to stop walking, metres.
ARRIVED_M = 0.01
#: The walk in's command, m/s, by walking policy (`decoupled.WALKERS`): see `STRAIGHT_PHASES`. P2Dingo's clocked gait
#: stands still inside its 0.1 m/s dead zone, and stops from 0.15-0.30 m/s within 1.0-1.7 deg of level.
WALK_IN_SPEED = {"rescue_2024": 0.06, "p2dingo_gait": 0.15}
#: Camera frames the look skips after re-placing the box, so the picture has caught up with it (`_look`).
LOOK_SKIP_STEPS = 2
#: The walk in's command: straight ahead at a steady `WALK_IN_SPEED`, nothing else, until the odometry has the base
#: within `ARRIVED_M` of the stance (`_walk_base`). The 2024 walking policy, stopped from 0.08 m/s or faster, keeps
#: turning in place and leans 10-26 deg (stopped from 0.06 it stands, 0.7 deg); a path's feedback, catching up on the
#: base's lag, takes the command to 0.10. Its own playback does it too. And it has to arrive: a pull begun 12 cm short
#: lunged after its path and flung the door off the hand (Week 6 log, 2026-10-20).
STRAIGHT_PHASES = {"walk_in"}
#: Phases the arm holds the home point through, relative to the base as planned, its orientation free
#: (`DecoupledDoorEnv.position_only_phases`): high and forward, above both sensors' fields. The phase it unfolds in, in
#: joint space. And the phases it stays folded through.
STOW_PHASES = {"walk_in", "walk_settle"}
JOINT_PHASES = {"unfold"}
PARKED_PHASES = {"spawn", "look", "no_tag"}
#: Where the camera's mount draws and the lidar's targets are, as prim expressions.
BASE_PRIM = "/World/envs/env_.*/Robot/base"
LIDAR_TARGETS = ("/World/envs/env_.*/CombinerBox/Door", "/World/envs/env_.*/CombinerBox/Handle",
                 "/World/envs/env_.*/CombinerBox/Enclosure", "/World/envs/env_.*/Robot/D1/Link.*",
                 "/World/envs/env_.*/Robot/base/mid360_mount")


def home_behind(back_m: float) -> tuple[float, float, float]:
    """The script's home point for a base `back_m` behind the stance: where the hand waits relative to the robot."""
    x, y, z = demo_script.HOME_POINT
    return (x - back_m, y, z)


def home_at(base) -> tuple[float, float, float]:
    """The home point relative to a base at (x, y, yaw) in the script's frame: where the hand waits as it walks."""
    x, y, z = demo_script.HOME_POINT
    c, s = math.cos(base[2]), math.sin(base[2])
    return (base[0] + c * x - s * y, base[1] + s * x + c * y, z)


def pre_phases(look_back_m: float = LOOK_BACK_M) -> list[demo_script.Phase]:
    """The phases between the spawn's settle and the door script: look (folded), unfold, walk to the stance, settle
    there. From the unfold on the hand goes to the home point relative to the base where it is (`home_at`, the base from
    `_state_of`), so it is where the door script's first phase expects it however far the walk got. The walk's base
    path is for the record only: `_walk_base` commands it."""
    jaws = demo_script.JAW_LEVER_OPEN_M      # the door script's opening on the lever, so its first phase starts there
    home = lambda st, u: home_at(st["base"])
    return [
        demo_script.Phase("look", LOOK_S, path=home, jaw_m=jaws),
        demo_script.Phase("unfold", UNFOLD_S, path=home, jaw_m=jaws),
        demo_script.Phase("walk_in", WALK_IN_S, path=home, jaw_m=jaws,
                          base=lambda st, u: (0.0, 0.0, 0.0)),
        demo_script.Phase("walk_settle", WALK_SETTLE_S, path=home, jaw_m=jaws),
    ]


def shifted_site(site: props.BoxSite, forward_m: float) -> props.BoxSite:
    """`site`, given from a stance `forward_m` ahead, as seen from the spawn behind it."""
    pose = perception.site_pose(site)
    pose[0, 3] += forward_m
    return perception.site_from_pose(pose)


class CombinerDoorEnv(DecoupledDoorEnv):
    def __init__(self, cfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self.parked_phases = set(PARKED_PHASES)
        self.position_only_phases = set(STOW_PHASES)
        self.joint_phases = set(JOINT_PHASES)
        self.look_back_m = float(getattr(cfg, "look_back_m", LOOK_BACK_M))
        self.door_deg = float(getattr(cfg, "door_deg", props.DOOR_PULL_DEG))
        self.camera_model = head_camera.camera_model()
        self.detector = TagDetector(self.camera_model.intrinsic_matrix)
        count = self.num_envs
        self.truth_sites: list = [None] * count
        self.estimates: list = [None] * count
        self.estimate_poses: list = [None] * count
        self.look_records: list[dict] = [{} for _ in range(count)]
        self._look_poses: list[list] = [[] for _ in range(count)]
        #: The look's lidar returns, script frame, added up over its frames.
        self._look_clouds: list[list] = [[] for _ in range(count)]
        #: Where the next door reading lays its window: the last reading's angle; shut at first.
        self._door_prior = [0.0] * count
        self.lidar_readings = [mid360.NO_READING] * count
        self.lidar_times_s = [math.nan] * count
        #: The simulator's door angle at each reading, for the record only: nothing here reads it.
        self.lidar_truth_deg = [math.nan] * count
        self._base_now = np.zeros((count, 3))
        self._noise = torch.Generator(device=self.device)
        self._noise.manual_seed(int(cfg.seed or 0))

    # --- scene ------------------------------------------------------------------------------------

    def _setup_scene(self):
        super()._setup_scene()
        out = head_camera.ROOT / "generated/combiner_door"
        bracket = sim_utils.UsdFileCfg(usd_path=head_camera.build_bracket_usd(out))
        bracket.func(f"{BASE_PRIM}/head_rs_bracket", bracket)
        case = sim_utils.UsdFileCfg(usd_path=head_camera.build_case_usd(head_camera.ROOT / "generated/pick_demo"))
        case.func(f"{BASE_PRIM}/head_rs_body", case, translation=head_camera.COLOUR_SENSOR_BASE_M,
                  orientation=head_camera.OPTICAL_QUAT_BASE_WXYZ)
        self._head_cam = TiledCamera(head_camera.camera_cfg(head_camera.camera_model(), f"{BASE_PRIM}/head_cam"))
        self.scene.sensors["head_cam"] = self._head_cam
        self._lidar = MultiMeshRayCaster(mid360.lidar_cfg(LIDAR_TARGETS, BASE_PRIM))
        self.scene.sensors["mid360"] = self._lidar

    # --- placements: given from the stance, placed from the spawn behind it -------------------------

    def place(self, sites: list) -> None:
        """`sites` are the door pull's placements, from its stance; the box goes `look_back_m` further from the spawn."""
        self.truth_sites = list(sites)
        super().place([shifted_site(site, self.look_back_m) for site in sites])

    def script_site(self, index: int):
        return self.estimates[index]

    def _state_of(self, index: int) -> dict:
        return {"env": index, "box_site": self.estimates[index], "base": tuple(self._base_now[index])}

    def _walk_base(self) -> None:
        """The parent's walk, except through `STRAIGHT_PHASES`: `WALK_IN_SPEED` straight ahead until within `ARRIVED_M`
        of the stance, then nothing."""
        super()._walk_base()
        for index, command in enumerate(self.commands):
            if command is not None and command.phase in STRAIGHT_PHASES:
                self._base_cmd[index] = 0.0
                if -self._base_now[index][0] > ARRIVED_M:
                    self._base_cmd[index, 0] = WALK_IN_SPEED[self.walker]
        self._commands[:, :3] = self._base_cmd

    def _reset_idx(self, env_ids) -> None:
        super()._reset_idx(env_ids)
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        # The script's frame is the stance, `look_back_m` ahead of where the robot spawned.
        ahead = torch.zeros(len(env_ids), 3, device=self.device)
        ahead[:, 0] = self.look_back_m
        self._spawn_pos[env_ids] += interface.quat_apply(self._spawn_yaw_quat[env_ids], ahead)
        self._spawn_pos[env_ids, 2] = 0.0
        self._fold(env_ids)
        for index in env_ids.tolist():
            self.estimates[index] = None
            self.estimate_poses[index] = None
            self.look_records[index] = {}
            self._look_poses[index] = []
            self._look_clouds[index] = []
            self._door_prior[index] = 0.0
            self.lidar_readings[index] = mid360.NO_READING
            self.lidar_times_s[index] = math.nan
            self.lidar_truth_deg[index] = math.nan

    def _fold(self, env_ids) -> None:
        """The arm in the real D1's rest pose, held there: the joints, the planner and the IK's seed."""
        limits = self._robot.data.joint_pos_limits[env_ids][:, self._arm_ids]
        rest = torch.as_tensor(d1_ik.from_servo_deg(d1_hardware.REST_SERVO_DEG)[0], dtype=torch.float32,
                               device=self.device).expand(len(env_ids), 6)
        rest = torch.minimum(torch.maximum(rest, limits[..., 0] + 0.005), limits[..., 1] - 0.005)
        self._robot.write_joint_state_to_sim(rest, torch.zeros_like(rest), joint_ids=self._arm_ids, env_ids=env_ids)
        self._planner.reset(env_ids, rest)
        for row, index in enumerate(env_ids.tolist()):
            self.arms[index].reset(rest[row].cpu().numpy())
            self._joint_goals[index] = None

    # --- the look ---------------------------------------------------------------------------------

    def _base_in_script(self, index: int) -> np.ndarray:
        data = self._robot.data
        return perception.base_in_spawn(
            (data.root_pos_w[index] - self.scene.env_origins[index]).cpu().numpy(), data.root_quat_w[index].cpu().numpy(),
            self._spawn_pos[index].cpu().numpy(), self._spawn_yaw_quat[index].cpu().numpy())

    def _anchor(self, index: int) -> None:
        """Start the attempt from the robot as it stands: the script's frame re-anchored `look_back_m` ahead of where
        the base now is and the way it now faces, and the box placed again, from that, where the attempt's placement
        puts it. The spawn drops the robot from above its standing height, and a walking policy may turn while it
        lands (P2Dingo's ~15 deg over 3-4 s); the real robot starts standing, square to where it was put."""
        data = self._robot.data
        base = (data.root_pos_w[index] - self.scene.env_origins[index]).clone()
        yaw = interface.yaw_quat(interface.yaw_from_quat(data.root_quat_w[index:index + 1]))[0]
        ahead = torch.tensor([self.look_back_m, 0.0, 0.0], device=self.device)
        self._spawn_yaw_quat[index] = yaw
        self._spawn_pos[index] = base + interface.quat_apply(yaw.unsqueeze(0), ahead.unsqueeze(0))[0]
        self._spawn_pos[index, 2] = 0.0
        # The placement, from the stance, into the environment's frame through the new anchor.
        anchor = np.eye(4)
        anchor[:3, :3] = head_camera.quat_to_matrix(yaw.cpu().numpy())
        anchor[:3, 3] = self._spawn_pos[index].cpu().numpy()
        self.sites[index] = perception.site_from_pose(anchor @ perception.site_pose(self.truth_sites[index]))
        self._place_props(torch.tensor([index], device=self.device))
        self._base_now[index] = self.base_pose_in_spawn()[index].cpu().numpy()

    def _look(self) -> None:
        """Search this step's frame for the tag in every environment that is looking; at the look's end, estimate.
        The look's first step anchors the attempt (`_anchor`), and its frames are used from `LOOK_SKIP_STEPS` on."""
        rgb = None
        for index, demo in enumerate(self.scripts):
            if self.estimates[index] is not None or demo.finished or demo.phases[demo.index].name != "look":
                continue
            record = self.look_records[index]
            steps = record.get("steps", 0)
            record["steps"] = steps + 1
            if steps == 0:
                self._anchor(index)
            if steps < LOOK_SKIP_STEPS:
                continue
            record["frames"] = record.get("frames", 0) + 1
            if rgb is None:
                rgb = self._head_cam.data.output["rgb"]
            detections = self.detector.detect(rgb[index, ..., :3].cpu().numpy().astype(np.uint8))
            for detection in detections:
                self._look_poses[index].append(perception.box_in_spawn(detection, self._base_in_script(index)))
                record["side_px"] = round(detection.side_px, 1)
                record["range_m"] = round(detection.range_m, 4)
                record["reprojection_px"] = round(detection.reprojection_px, 3)
            # The script steps after this; if that step ends the look, the estimate must exist before the next phase.
            if demo.elapsed_s + interface.POLICY_DT >= demo.phases[demo.index].duration_s - 1e-9:
                self._finish_look(index)

    def _finish_look(self, index: int) -> None:
        record = self.look_records[index]
        poses = self._look_poses[index]
        record["detections"] = len(poses)
        if not poses:
            # Nothing to work from: hold where it is for the rest of the attempt.
            demo = self.scripts[index]
            demo.phases = demo.phases[:demo.index + 1] + [
                demo_script.Phase("no_tag", 1.0e3, target=lambda st: demo_script.HOME_POINT)]
            record["found"] = False
            return
        record["found"] = True
        truth = self.truth_sites[index]
        # The tag alone, recorded beside what is used: the tag's position with the lidar's heading.
        tag_site, tag_pose = perception.estimate(poses)
        record["tag_only"] = perception.compare(tag_pose, tag_site, truth)
        cloud = np.concatenate(self._look_clouds[index]) if self._look_clouds[index] else np.zeros((0, 3))
        site, pose, fit = perception.lidar_estimate(poses, cloud)
        record["lidar_frames"] = len(self._look_clouds[index])
        record["lidar_heading"] = {"used": fit.ok, "points": fit.points, "span_m": round(fit.span, 3),
                                   "rms_mm": round(1000 * fit.rms, 1) if fit.ok else None,
                                   "face_behind_tag_mm": round(1000 * fit.depth, 1) if fit.ok else None,
                                   "why_not": fit.why or None}
        record.update(perception.compare(pose, site, truth))
        self.estimates[index], self.estimate_poses[index] = site, pose

    # --- the lidar --------------------------------------------------------------------------------

    def commanded_door_deg(self, index: int) -> float:
        command = self.commands[index]
        if command is None:
            return 0.0
        if command.phase == "pull":
            return self.door_deg * command.progress
        return self.door_deg if command.phase in ("hold", "release", "back", "done") else 0.0

    def _lidar_frame(self) -> None:
        """At the lidar's frame rate: its returns kept for the look while looking, and the door read once the box is
        found."""
        if self.common_step_counter % round(mid360.FRAME_PERIOD_S / interface.POLICY_DT):
            return
        looking = [i for i, demo in enumerate(self.scripts)
                   if not demo.finished and demo.phases[demo.index].name == "look" and self.estimates[i] is None
                   and self.look_records[i].get("steps", 0) > LOOK_SKIP_STEPS]
        ready = [i for i in range(self.num_envs) if self.estimates[i] is not None]
        if not looking and not ready:
            return
        data = self._lidar.data
        rays = data.ray_hits_w - data.pos_w.unsqueeze(1)
        ranges = torch.linalg.vector_norm(rays, dim=-1)
        valid = torch.isfinite(ranges) & (ranges > mid360.BLIND_ZONE_M) & (ranges < mid360.MAX_RANGE_M)
        noise = torch.randn(ranges.shape, generator=self._noise, device=self.device) * mid360.RANGE_NOISE_M
        points = data.pos_w.unsqueeze(1) + rays / ranges.clamp(min=1e-6).unsqueeze(-1) * (ranges + noise).unsqueeze(-1)
        origins = self.scene.env_origins.cpu().numpy()
        spawn_pos = self._spawn_pos.cpu().numpy()
        spawn_rot = [head_camera.quat_to_matrix(q) for q in self._spawn_yaw_quat.cpu().numpy()]
        time_s = self.episode_length_buf.cpu().numpy() * interface.POLICY_DT
        truth = self.box_state()["door_deg"].cpu().numpy()
        for index in looking + ready:
            world = points[index][valid[index]].cpu().numpy()
            in_script = (world - origins[index] - spawn_pos[index]) @ spawn_rot[index]
            if index in looking:
                self._look_clouds[index].append(in_script)
                continue
            box = self.box_frame(index)
            in_box = (in_script - box[:3, 3]) @ box[:3, :3]
            reading = mid360.door_angle(in_box, self._door_prior[index])
            if reading.ok:
                self._door_prior[index] = reading.angle_deg
            self.lidar_readings[index] = reading
            self.lidar_times_s[index] = float(time_s[index])
            self.lidar_truth_deg[index] = float(truth[index])

    def box_frame(self, index: int) -> np.ndarray:
        """The box frame the robot works in: the estimate's `BoxSite` (the plinth's height, level), not the raw pose."""
        return perception.site_pose(self.estimates[index])

    # --- the step ---------------------------------------------------------------------------------

    def _scripted_goal(self) -> torch.Tensor:
        self._base_now = self.base_pose_in_spawn().cpu().numpy()
        self._look()
        goal = super()._scripted_goal()
        self._lidar_frame()
        return goal
