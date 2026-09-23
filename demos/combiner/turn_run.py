"""One simulated attempt at the combiner's handle: `TurnSequence` in the loop, and what the simulator says happened.

The sequence sees only what the robot would: sampled joint feedback, the IMU's up, and rendered wrist
frames through the tag detector. Everything it cannot know -- the lever's angle, whether the latch let
go, the box's true pose, contact forces -- is read from the simulator here, traced per step to
`trace.csv`, and summarised in the attempt's row of `run.json`.

What the torque numbers mean. The spring's torque at an angle is known exactly
(`GEOMETRY.spring_torque`), so the lever's angle while the arm holds it is a direct reading of the torque
the arm sustained there: the spring's, less the lever's own weight (`LEVER_GRAVITY_NM` at horizontal).
The arm joint torques are PhysX's own: each joint's transmitted force projected on its axis
(`get_dof_projected_joint_forces`), drive plus any limit. Isaac Lab's `applied_torque` for an implicit drive
is only stiffness x error + damping x velocity, clipped (F-017), and at the arm's 4000 N·m/rad it reads the
limit for any error at all -- the first smoke run showed five of six joints "at their limit". The finger
contact force is the contact sensor's net force on Link7_1 and Link7_2.

For the grasp-and-pull method the jaws shut past the URDF's 17.2 mm stop to squeeze the 18 mm bar
(`sequence.GRIP_SHUT_M`), so the attempt opens the simulated stop as the pick does for a pinch (F-063). The
two fingers are independent drives in the URDF (the real jaws are one servo): `grip.jaw_asymmetry_max_mm`
is how far the pair shifted sideways on the bar, the way the grip fails.
"Slip" is how far the jaw centre (Link6 by the simulator, plus `JAW_CENTRE_LINK6`) is from the point on the
lever it closed on (the Handle body's pose, plus the grasp radius): the grip holding, or not.

Needs the Isaac app: import after `AppLauncher`.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np
import torch

from demos.cup.pick_demo.camera import MeasuredMount, camera_pose, invert, link6_pose, transform
from demos.cup.pick_demo.grasp import JAW_CENTRE_LINK6
from demos.cup.pick_demo.perception import Frame
from demos.cup.pick_demo.sequence import Timing
from position_only.env_cfg import ARM_NAMES

from .apriltag import pose_error
from .geometry import GEOMETRY, LEVER_GRAVITY_NM
from .press import Lever, PressParams
from .pull import PullParams
from .sequence import GRIP_SHUT_M, TurnSequence

FINGER_LINKS = ("Link7_1", "Link7_2")


def quat_to_matrix(q):
    w, x, y, z = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def annotate(rgb, detections, label):
    import cv2

    image = cv2.cvtColor(np.asarray(rgb, np.uint8), cv2.COLOR_RGB2BGR)
    for d in detections:
        corners = d.corners_px.astype(int)
        cv2.polylines(image, [corners.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
        cv2.circle(image, tuple(corners[0]), 4, (0, 0, 255), -1)
        cv2.putText(image, f"id {d.tag_id} {d.range_m:.3f} m", tuple(corners[0] + [0, -8]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(image, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    return image


class TurnAttempt:
    def __init__(self, env, latch, joints, links, mount, detector, out_dir: Path, torque_nm: float, *,
                 mount_calibration: str = "none", press: PressParams | None = None, settle_s: float = 0.5,
                 overview=None, method: str = "pull", pull: PullParams | None = None):
        self.env, self.latch = env, latch
        self.robot, self.box = env.scene["robot"], env.scene["combiner"]
        self.wrist, self.contact = env.scene["wrist_cam"], env.scene["arm_contact"]
        self.joints, self.links, self.torque_nm = joints, links, torque_nm
        self.out_dir = out_dir
        (out_dir / "frames").mkdir(parents=True, exist_ok=True)
        self.mount_calibration = mount_calibration
        self.method = method
        self.sequence = TurnSequence(joints, links, mount, detector, method=method, press=press, pull=pull,
                                     timing=Timing(settle_s=settle_s))
        robot = self.robot
        self.link6 = robot.body_names.index("Link6")
        self.handle_body = self.box.body_names.index("Handle")
        self.arm_ids = [robot.joint_names.index(n) for n in ARM_NAMES]
        self.grip_ids = [robot.joint_names.index(n) for n in ("Joint7_1", "Joint7_2")]
        self.finger_ids = [self.contact.body_names.index(n) for n in FINGER_LINKS]
        self.default_arm = robot.data.default_joint_pos[0, self.arm_ids].cpu().numpy()
        self.limits = robot.data.joint_effort_limits[0, self.arm_ids].cpu().numpy()
        terms = env.observation_manager.active_terms["policy"]
        dims = env.observation_manager.group_obs_term_dim["policy"]
        offsets = np.cumsum([0] + [int(np.prod(d)) for d in dims])
        self.slices = {name: slice(int(offsets[i]), int(offsets[i + 1])) for i, name in enumerate(terms)}
        self.looks, self.rows, self.calibrated = [], [], None
        self.overview, self.captured_holds = overview, set()
        self.extra: dict = {}
        # The box's true pose in the base frame when each look was taken: the push can shove the dog, so
        # an estimate is scored against the frame it was made in, not the one the attempt ends in.
        self.truth_at_look: dict[str, np.ndarray] = {}
        self.base_start = None
        if method == "pull":
            self._open_gripper_stop()

    def _open_gripper_stop(self):
        """Let the jaws travel past the URDF's stop to `GRIP_SHUT_M`, as the pick does for a pinch (F-063).

        Targets are clamped to the soft limits, a factor inside the hard ones, so the hard limit opens further
        than the travel wanted. Only min/max, so doing it again on the next attempt changes nothing."""
        robot, ids = self.robot, self.grip_ids
        limits = robot.data.joint_pos_limits[:, ids].clone()
        factor = float(robot.cfg.soft_joint_pos_limit_factor)
        span = float(limits[0, 0, 1] - limits[0, 0, 0])
        reach = (2 * (GRIP_SHUT_M - 0.0005) - span * (1 - factor)) / (1 + factor)
        limits[..., 0] = torch.minimum(limits[..., 0], torch.full_like(limits[..., 0], reach))
        limits[..., 1] = torch.maximum(limits[..., 1], torch.full_like(limits[..., 1], -reach))
        robot.write_joint_position_limit_to_sim(limits, joint_ids=ids)
        self.gripper_soft_limits = robot.data.soft_joint_pos_limits[0, ids].cpu().numpy().round(5).tolist()
        self.gripper_drive = {"stiffness": robot.data.joint_stiffness[0, ids].cpu().tolist(),
                              "damping": robot.data.joint_damping[0, ids].cpu().tolist(),
                              "effort_limit": robot.data.joint_effort_limits[0, ids].cpu().tolist()}

    # ------------------------------------------------------------------ simulator truth
    def base_pose(self):
        robot = self.robot
        return transform(quat_to_matrix(robot.data.root_quat_w[0].cpu().numpy()),
                         robot.data.root_pos_w[0].cpu().numpy())

    def box_truth_b(self):
        box = self.box
        pose_w = transform(quat_to_matrix(box.data.root_quat_w[0].cpu().numpy()), box.data.root_pos_w[0].cpu().numpy())
        return invert(self.base_pose()) @ pose_w

    def rendered_camera_b(self):
        pose_w = transform(quat_to_matrix(self.wrist.data.quat_w_ros[0].cpu().numpy()),
                           self.wrist.data.pos_w[0].cpu().numpy())
        return invert(self.base_pose()) @ pose_w

    def grasp_offsets(self):
        """At the moment the jaws start to close: where the jaw centre is against the lever point it should
        hold, split into what the plan got wrong (perception) and what the arm got wrong (the simulator's
        Link6 against forward kinematics of its own joint angles). Millimetres, in the planned tool frame
        (x along Link6 x, y the jaw axis, z the approach)."""
        seq, robot = self.sequence, self.robot
        base = invert(self.base_pose())
        link6_w = robot.data.body_link_pose_w[0, self.link6].cpu().numpy()
        jaw_sim = (base @ np.append(link6_w[:3] + quat_to_matrix(link6_w[3:]) @ np.asarray(JAW_CENTRE_LINK6), 1.0))[:3]
        fk = link6_pose(self.joints, robot.data.joint_pos[0, self.arm_ids].cpu().numpy())
        jaw_fk = fk[:3, :3] @ np.asarray(JAW_CENTRE_LINK6) + fk[:3, 3]
        handle = self.box.data.body_link_pose_w[0, self.handle_body].cpu().numpy()
        point_w = handle[:3] + quat_to_matrix(handle[3:]) @ np.array([GEOMETRY.handle_projection, -seq.plan.params.radius_m, 0.0])
        point_true = (base @ np.append(point_w, 1.0))[:3]
        from .pull import Handle

        planned, rot = Handle(seq.box, seq.plan.params).grasp(0.0, 0.0)
        in_tool = lambda v: np.round(1000 * (rot.T @ v), 2).tolist()
        return {"jaw_minus_lever_mm": in_tool(jaw_sim - point_true),
                "plan_minus_lever_mm (perception)": in_tool(planned - point_true),
                "arm_sim_minus_fk_mm (arm model)": in_tool(jaw_sim - jaw_fk),
                "fk_minus_plan_mm (servo and IK)": in_tool(jaw_fk - planned)}

    def grip_slip_m(self):
        """Distance from the jaw centre to the lever point the plan grips, both from the simulator."""
        plan = self.sequence.plan
        if self.method != "pull" or plan is None:
            return math.nan
        link6 = self.robot.data.body_link_pose_w[0, self.link6].cpu().numpy()
        jaw = link6[:3] + quat_to_matrix(link6[3:]) @ np.asarray(JAW_CENTRE_LINK6)
        handle = self.box.data.body_link_pose_w[0, self.handle_body].cpu().numpy()
        point = handle[:3] + quat_to_matrix(handle[3:]) @ np.array([GEOMETRY.handle_projection, -plan.params.radius_m, 0.0])
        return float(np.linalg.norm(jaw - point))

    # ------------------------------------------------------------------ the loop
    def run(self, app, obs, keys=None, on_step=None, max_s: float = 90.0, linger_s: float = 1.0) -> dict:
        env, robot, sequence = self.env, self.robot, self.sequence
        dt = env.step_dt
        actions = torch.zeros(1, env.action_manager.total_action_dim, device=env.device)
        trace_file = (self.out_dir / "trace.csv").open("w", newline="")
        trace = csv.writer(trace_file)
        trace.writerow(["t", "state", "lever_cmd_deg", "door_cmd_deg", "handle_deg", "door_deg", "latched",
                        "gripper_cmd_m", "finger_1_m", "finger_2_m", "grip_slip_mm",
                        *[f"q_cmd_{i}" for i in range(6)], *[f"q_fb_{i}" for i in range(6)],
                        *[f"q_true_{i}" for i in range(6)], *[f"q_drive_{i}" for i in range(6)],
                        *[f"tau_physx_{i}" for i in range(6)],
                        "finger_force_n", "spring_torque_nm", "finger_1_force_n", "finger_2_force_n",
                        "finger_1_target_m", "finger_2_target_m", "arm_contact_max_n", "arm_contact_link"])
        captured = {}

        def frame_source():
            rgb = self.wrist.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8)
            q_true = robot.data.joint_pos[0, self.arm_ids].cpu().numpy()
            rendered = self.rendered_camera_b()
            if self.mount_calibration == "sim" and self.calibrated is None:
                # The simulated equivalent of a perfect hand-eye calibration: where the renderer put the
                # camera, relative to Link6 by forward kinematics. Constant in Link6 (F-052).
                measured = invert(link6_pose(self.joints, q_true)) @ rendered
                self.calibrated = MeasuredMount.from_pose(measured, "simulator render pose at the first look")
                sequence.mount = self.calibrated
            model = camera_pose(self.joints, captured["q_fb"], sequence.mount)
            cos = np.clip((np.trace(model[:3, :3].T @ rendered[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
            captured["check"] = {"position_error_mm": np.round(1000 * (model[:3, 3] - rendered[:3, 3]), 2).tolist(),
                                 "rotation_error_deg": round(math.degrees(math.acos(cos)), 3)}
            captured["rgb"], captured["state"] = rgb, sequence.state
            self.truth_at_look[sequence.state] = self.box_truth_b()
            return Frame(rgb, None, captured["q_fb"].copy(), captured["up"].copy(), captured["t"])

        t, done_at, aborted = 0.0, None, False
        self.base_start = self.base_pose()
        with torch.inference_mode():
            while app.is_running() and t < max_s:
                if keys is not None:
                    events = keys.take()
                    if "R" in events:
                        aborted = True
                        break
                    for key in events:
                        self.latch.command(key)
                policy = obs["policy"][0]
                q_fb = policy[self.slices["arm_joint_pos"]].cpu().numpy() + self.default_arm
                up = -policy[self.slices["projected_gravity"]].cpu().numpy()
                captured.update(q_fb=q_fb, up=up, t=t, rgb=None)
                command = sequence.update(t, q_fb, up, frame_source)
                if command.q is not None:
                    actions[0, 12:] = torch.as_tensor((command.q - self.default_arm) / env.cfg.actions.arm.scale,
                                                      device=env.device, dtype=torch.float32)
                robot.set_joint_position_target(torch.tensor([[command.gripper_m, -command.gripper_m]],
                                                             device=env.device), joint_ids=self.grip_ids)
                if captured["rgb"] is not None:
                    self._record_look(t, captured)
                obs, _, terminated, truncated, _ = env.step(actions)
                if bool(terminated[0] or truncated[0]):
                    raise RuntimeError(f"environment reset during the turn at t={t:.2f} s")
                t += dt
                self._trace(trace, t, command)
                if command.state == "grip" and "grasp_offsets" not in self.extra:
                    self.extra["grasp_offsets"] = self.grasp_offsets()
                self._capture_hold(t)
                if on_step is not None:
                    on_step(sequence.state)
                if sequence.done:
                    done_at = t if done_at is None else done_at
                    if keys is None and t - done_at >= linger_s:
                        break
        trace_file.close()
        return self._summary(t, aborted, obs)

    def _record_look(self, t, captured):
        import cv2

        detections = self.sequence.last_detections
        name = f"{len(self.looks):03d}_{t:05.1f}s_{captured['state']}.png"
        cv2.imwrite(str(self.out_dir / "frames" / name), annotate(captured["rgb"], detections, f"{t:5.1f}s {captured['state']}"))
        self.looks.append({"t": round(t, 3), "state": captured["state"], "image": f"frames/{name}",
                           "detections": [d.as_dict() for d in detections],
                           "camera_model_vs_sim": captured["check"]})

    # Pictures halfway through each hold: the lever pushed down, the lever turned in the jaws, the door open.
    HOLDS = {"hold": ("hold", "hold_s"), "turn_hold": ("turn", "turn_hold_s"), "open_hold": ("open", "open_hold_s")}

    def _capture_hold(self, t):
        seq = self.sequence
        if seq.state not in self.HOLDS or seq.plan is None:
            return
        name, duration = self.HOLDS[seq.state]
        if name in self.captured_holds or t - seq.state_since < getattr(seq.plan.params, duration) / 2:
            return
        import cv2

        self.captured_holds.add(name)
        wrist = self.wrist.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8)
        cv2.imwrite(str(self.out_dir / f"wrist_{name}.png"), cv2.cvtColor(wrist, cv2.COLOR_RGB2BGR))
        if self.overview is not None:
            rgb = self.overview.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8)
            cv2.imwrite(str(self.out_dir / f"overview_{name}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    def _trace(self, trace, t, command):
        robot = self.robot
        door_deg, handle_deg = self.latch.angles()
        tau = robot.root_physx_view.get_dof_projected_joint_forces()[0, self.arm_ids].cpu().numpy()
        forces = self.contact.data.net_forces_w[0, self.finger_ids]
        finger = float(torch.linalg.vector_norm(forces, dim=-1).sum())
        # The largest contact on any arm link, and which: what stops a move the planner thought was clear.
        per_link = torch.linalg.vector_norm(self.contact.data.net_forces_w[0], dim=-1)
        contact_max = float(per_link.max())
        contact_link = self.contact.body_names[int(per_link.argmax())] if contact_max > 0.5 else ""
        q_true = robot.data.joint_pos[0, self.arm_ids].cpu().numpy()
        q_drive = robot.data.joint_pos_target[0, self.arm_ids].cpu().numpy()
        finger_tau = robot.root_physx_view.get_dof_projected_joint_forces()[0, self.grip_ids].cpu().numpy()
        policy_q = command.q if command.q is not None else [math.nan] * 6
        lever_cmd, door_cmd = self.sequence.commanded_lever_deg, self.sequence.commanded_door_deg
        fingers = robot.data.joint_pos[0, self.grip_ids].cpu().numpy()
        slip = self.grip_slip_m()
        row = {"t": t, "state": command.state, "handle_deg": handle_deg, "door_deg": door_deg,
               "latched": self.latch.latch.latched, "tau": tau, "finger_force_n": finger,
               "spring_nm": GEOMETRY.spring_torque(self.torque_nm, handle_deg), "slip_m": slip,
               "fingers": fingers.round(5).tolist()}
        self.rows.append(row)
        trace.writerow([f"{t:.3f}", command.state, "" if lever_cmd is None else f"{lever_cmd:.2f}",
                        "" if door_cmd is None else f"{door_cmd:.2f}",
                        f"{handle_deg:.3f}", f"{door_deg:.3f}", int(row["latched"]),
                        f"{command.gripper_m:.4f}", f"{fingers[0]:.4f}", f"{fingers[1]:.4f}",
                        "" if math.isnan(slip) else f"{1000 * slip:.2f}",
                        *[f"{v:.5f}" for v in policy_q], *[f"{v:.5f}" for v in self.sequence.history[-1][1]],
                        *[f"{v:.5f}" for v in q_true], *[f"{v:.5f}" for v in q_drive], *[f"{v:.4f}" for v in tau], f"{finger:.3f}",
                        f"{row['spring_nm']:.4f}", *[f"{v:.3f}" for v in finger_tau],
                        *[f"{float(v):.4f}" for v in robot.data.joint_pos_target[0, self.grip_ids]],
                        f"{contact_max:.2f}", contact_link])

    def _summary(self, t, aborted, obs) -> dict:
        seq, g = self.sequence, GEOMETRY
        truth = self.box_truth_b()
        moved = invert(self.base_start) @ self.base_pose()
        outcome = seq.state if seq.done else ("aborted" if aborted else "timeout")
        out = {"handle_torque_nm": self.torque_nm, "outcome": outcome,
               "failure": seq.failure, "simulated_s": round(t, 3), "phase_times_s": seq.phase_times,
               "found_by": seq.found_by, "events": seq.events, "looks": self.looks,
               "box_truth_b": np.round(truth, 5).tolist(), "mount_calibration": self.mount_calibration,
               "base_moved": {"position_mm": round(1000 * float(np.linalg.norm(moved[:3, 3])), 2),
                              "yaw_deg": round(math.degrees(math.atan2(moved[1, 0], moved[0, 0])), 3)}}
        if self.calibrated is not None:
            out["calibrated_mount_link6"] = np.round(self.calibrated.pose, 6).tolist()
        radius = (seq.plan.params if seq.plan else PressParams()).radius_m
        for key, estimate, state in (("search_look", seq.box_first, "detect"), ("close_look", seq.box, "refine")):
            if estimate is not None:
                at_look = self.truth_at_look.get(state, truth)
                error = pose_error(estimate, at_look)
                miss = Lever(estimate).top(0.0, radius) - Lever(at_look).top(0.0, radius)
                error["lever_contact_error_mm"] = np.round(1000 * miss, 2).tolist()
                out[f"box_error_{key}"] = error
        out["method"] = self.method
        if seq.plan is not None:
            out["plan"] = seq.plan.as_dict()
            out["static_ceiling_at_45_deg_nm"] = seq.plan.predicted_torque_nm()
        # The lever pushed or turned, and then held there: the torque reading, for either method.
        turning, holding = (("press", "hold"), "hold") if self.method == "press" else (("turn", "turn_hold"), "turn_hold")
        pushing = [r for r in self.rows if r["state"] in turning]
        held = [r for r in self.rows if r["state"] == holding]
        if held:
            tail = [r for r in held if r["t"] >= held[-1]["t"] - 0.5]
            angle = float(np.mean([r["handle_deg"] for r in tail]))
            tau = np.mean([r["tau"] for r in tail], axis=0)
            out["hold"] = {
                "handle_deg": round(angle, 2),
                "spring_torque_nm": round(g.spring_torque(self.torque_nm, angle), 4),
                # What the arm supplied: the spring's torque, less the lever's own weight helping it down.
                "arm_torque_on_lever_nm": round(g.spring_torque(self.torque_nm, angle)
                                                - LEVER_GRAVITY_NM * math.cos(math.radians(angle)), 4),
                "finger_force_n": round(float(np.mean([r["finger_force_n"] for r in tail])), 2),
                "arm_joint_torque_physx_nm": np.round(tau, 3).tolist(),
                "arm_joint_torque_fraction_of_limit": np.round(np.abs(tau) / self.limits, 3).tolist(),
            }
        if pushing:
            peak = max(pushing, key=lambda r: r["handle_deg"])
            out["peak_handle_deg"] = round(peak["handle_deg"], 2)
            out["peak_spring_torque_nm"] = round(g.spring_torque(self.torque_nm, peak["handle_deg"]), 4)
        if self.method == "pull":
            out.update(self._door_summary())
        out.update(self.extra)
        released = [r for r in self.rows if not r["latched"]]
        out["latch_released"] = bool(released)
        out["latch_released_t_s"] = round(released[0]["t"], 3) if released else None
        out["reached_45_deg"] = bool(pushing) and max(r["handle_deg"] for r in pushing) >= g.handle_release_deg
        return out

    def _door_summary(self) -> dict:
        rows, out = self.rows, {}
        gripped = [r for r in rows if r["state"] in ("grip", "turn", "turn_hold", "crack", "unturn", "pull", "open_hold")]
        if not gripped:
            return out
        slips = [r["slip_m"] for r in gripped if not math.isnan(r["slip_m"])]
        # A coupled pair keeps Joint7_2 = -Joint7_1: their sum is how far the independent fingers shifted.
        asymmetry = [abs(r["fingers"][0] + r["fingers"][1]) for r in gripped]
        closed = [r for r in rows if r["state"] == "grip"]
        out["grip"] = {
            "slip_after_closing_mm": round(1000 * closed[-1]["slip_m"], 2) if closed else None,
            "slip_max_mm": round(1000 * max(slips), 2) if slips else None,
            "slip_at_end_mm": round(1000 * gripped[-1]["slip_m"], 2),
            "jaw_asymmetry_max_mm": round(1000 * max(asymmetry), 3),
            "gripper_soft_limits_m": self.gripper_soft_limits,
            "gripper_drive": self.gripper_drive,
            "commanded_m": round(GRIP_SHUT_M, 4),
            # Joint7_1 and Joint7_2 travel where the jaws stopped: +0.0004 each is the 18 mm bar between them.
            "finger_travel_after_closing_m": closed[-1]["fingers"] if closed else None,
        }
        opened = [r for r in rows if r["state"] == "open_hold"]
        out["door_peak_deg"] = round(max(r["door_deg"] for r in rows), 2)
        out["door_planned_deg"] = self.sequence.plan.door_final_deg
        if opened:
            tail = [r for r in opened if r["t"] >= opened[-1]["t"] - 0.5]
            out["door_held_open_deg"] = round(float(np.mean([r["door_deg"] for r in tail])), 2)
        out["door_final_deg"] = round(rows[-1]["door_deg"], 2)
        out["door_opened"] = out.get("door_held_open_deg", 0.0) >= self.sequence.plan.params.door_min_deg
        return out


__all__ = ["TurnAttempt", "annotate", "quat_to_matrix"]
