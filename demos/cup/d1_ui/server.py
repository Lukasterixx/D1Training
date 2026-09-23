"""Browser control surface for the physical D1 on the Go2.

Serves a three.js page that draws the Go2 and the D1 from their URDFs, moves the
joints live from the arm's feedback (and the dog's `rt/lowstate`), and lets the
user click a point on a translucent sphere around the arm to send the tool there.

Runs **on the Jetson payload**, because CycloneDDS needs the arm's subnet:

    python3 demos/cup/d1_ui/server.py --port 8090

then open http://<dog>:8090 from anywhere that can reach it (Tailscale works).

**Sim or hardware** is decided at start-up (`--mode auto`): if a simulator's feed answers on localhost
(`demos/cup/d1_ui/sim_feed.py`, published by `demos/cup/run_pick_demo.py` and `main.py`), the page follows the simulator --
its joints, legs and rendered wrist camera -- and every command that would move an arm is refused. Otherwise,
if this machine has the arm's network interface, it is the dog, and the page drives the real arm as before.
Neither means a workstation with no simulator up yet: sim mode, waiting for one.

The camera window shows the wrist RealSense (the simulator's render in sim mode) with the pick's stock YOLO
boxes drawn on, served as MJPEG from `/camera.mjpg` (`demos/cup/d1_ui/camera_feed.py`).

Everything on the wire is the stdlib: `ThreadingHTTPServer` for the page and
the meshes, Server-Sent Events for the live state (no websocket dependency),
and JSON POSTs for commands. The solver and the arm client are the same
`d1_ik` / `d1_hardware` the CLI uses, so what the page does is exactly what the
`move` and `park` commands do.

Safety, in order of importance:

* The page starts in **dry run**. Nothing is published until the LIVE switch is
  turned on, and that state lives in this process, not in the browser.
* A click never moves the arm. It asks for an IK preview; a second, separate
  action sends it.
* STOP cancels the approach and leaves the arm **holding**. It does not release,
  because on this arm release is a fall (F-028). RELEASE is its own button,
  refused unless the arm is folded (or forced through a confirm dialog).
* One move at a time, every command paced to 10 Hz by the client (F-032). Moves
  are single-shot: one waypoint per pass so the arm runs its own smooth profile
  (F-035), and no re-solving once it arrives (F-033).
* Targets are refused below the mount plane, outside the sphere, if IK does not
  converge, or if the trunk/ground proxy says the solution folds into the dog.
  The proxy assumes the standing trunk box, so it is a coarse guard, not a
  clearance model of a sitting robot.
"""
from __future__ import annotations

import argparse
import json
import math
import queue
import socket
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]      # the repository, three up from demos/cup/d1_ui
sys.path.insert(0, str(ROOT))

import d1_ik  # noqa: E402
import d1_hardware  # noqa: E402
from demos.cup.d1_ui.sim_feed import DEFAULT_URL as SIM_FEED_URL, GO2_MOTOR_ORDER, SimFeedClient  # noqa: E402
from position_only.workspace import BODY_BOX_B, MOUNT_B, clear_of_body  # noqa: E402

HERE = Path(__file__).resolve().parent
STATIC = HERE / "static"
D1_URDF = ROOT / "d1_arm" / "d1.urdf"
GO2_URDF = ROOT / "description" / "go2_d1.urdf"
PICK_ASSETS = HERE.parent / "pick_demo" / "assets"       # this demo's own, beside the console
MESH_DIRS = {"d1": ROOT / "d1_arm" / "meshes", "go2": ROOT / "description" / "meshes" / "go2",
             # Intel's D435 case, for the mount editor. Provenance: third_party/realsense2_description.
             "realsense": PICK_ASSETS / "realsense"}

FOLDED_PARK_DEG = [0.0, -89.9, 89.9, 0.0, 0.0, 0.0]   # servo degrees; the hard limits of J1/J2
SPHERE_MIN_Z_ABOVE_MOUNT_M = 0.03
# Feedback older than this means the page is drawing a memory, not the arm. The D1 reports every 111 ms
# (F-020), so this is several missed cycles rather than a hiccup.
STALE_FEEDBACK_S = 1.0
PROXY_BASE_HEIGHT_M = 0.15   # conservative: the ground closer than standing, since the robot sits


# ------------------------------------------------------------------ model JSON


def _vec(text, default=(0.0, 0.0, 0.0)):
    return [float(v) for v in text.split()] if text else list(default)


def _parse_urdf(path: Path, mesh_key: str):
    """Joints and links as plain numbers: what the page needs to build its scene graph."""
    root = ET.parse(path).getroot()
    joints, links = [], {}
    for joint in root.findall("joint"):
        origin, axis, limit = joint.find("origin"), joint.find("axis"), joint.find("limit")
        joints.append({
            "name": joint.get("name"), "type": joint.get("type"),
            "parent": joint.find("parent").get("link"), "child": joint.find("child").get("link"),
            "xyz": _vec(origin.get("xyz") if origin is not None else None),
            "rpy": _vec(origin.get("rpy") if origin is not None else None),
            "axis": _vec(axis.get("xyz") if axis is not None else None, (0.0, 0.0, 1.0)),
            "limits": ([float(limit.get("lower")), float(limit.get("upper"))] if limit is not None else None),
        })
    for link in root.findall("link"):
        visuals = []
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.get("filename") or ""
            if mesh_key == "go2" and "go2" not in filename:
                continue    # the combined URDF also carries the D1's meshes; served from the D1 spec
            origin = visual.find("origin")
            visuals.append({
                "mesh": f"/meshes/{mesh_key}/{Path(filename).name}",
                "xyz": _vec(origin.get("xyz") if origin is not None else None),
                "rpy": _vec(origin.get("rpy") if origin is not None else None),
                "scale": _vec(mesh.get("scale"), (1.0, 1.0, 1.0)),
            })
        links[link.get("name")] = visuals
    return joints, links


def build_model(sphere_radius_m: float = 0.40) -> dict:
    d1_joints, d1_links = _parse_urdf(D1_URDF, "d1")
    go2_joints, go2_links = _parse_urdf(GO2_URDF, "go2")
    # The combined URDF includes the arm; the page mounts the arm itself from the
    # D1 spec, so drop the arm's joints and the mount from the Go2 chain.
    d1_names = {j["name"] for j in d1_joints}
    go2_joints = [j for j in go2_joints if j["name"] not in d1_names and j["name"] != "arm_mount_joint"]
    go2_links = {k: v for k, v in go2_links.items() if not k.startswith(("Link", "d1_"))}

    joints_model, _ = d1_ik.load_urdf()
    lows, highs = d1_ik.servo_limits_deg(joints_model)
    mount = [float(v) for v in MOUNT_B]
    # Shoulder height above the mount: Joint1 then Joint2 origins along the base z axis.
    shoulder_z = 0.0533 + 0.0563
    return {
        "d1": {"root": "base_link", "mount": mount, "joints": d1_joints, "links": d1_links,
               "arm_joints": d1_ik.ARM_JOINTS,
               "servo_sign": [float(v) for v in d1_ik.SERVO_SIGN],
               "servo_limits_deg": [lows.tolist(), highs.tolist()],
               "gripper_stroke_m": 0.03},
        "go2": {"root": "base_link", "joints": go2_joints, "links": go2_links,
                "motor_order": GO2_MOTOR_ORDER},
        "tool": {"body": d1_ik.TOOL_BODY, "offset": [float(v) for v in d1_ik.TOOL_OFFSET_M]},
        "sphere": {"center": [mount[0], -0.028, mount[2] + shoulder_z], "radius": sphere_radius_m,
                   "min_z": mount[2] + SPHERE_MIN_Z_ABOVE_MOUNT_M},
        "trunk_box": [list(r) for r in BODY_BOX_B],
        "frame": "Go2 base frame: +x forward, +y left, +z up",
    }


# ------------------------------------------------------------------ sim or hardware


def detect_mode(requested: str, sim_url: str, iface: str, net_dir: Path = Path("/sys/class/net")):
    """("sim" | "hardware", why). A simulator feed wins; then the arm's NIC means this is the dog."""
    if requested == "hardware":
        return requested, "--mode hardware"
    health = SimFeedClient(sim_url, timeout_s=0.5).health()
    if requested == "sim":
        return "sim", ("--mode sim; " + (f"simulator feed ({health.get('source')}) answered at {sim_url}" if health
                                         else f"waiting for a simulator feed at {sim_url}"))
    if health is not None:
        return "sim", f"simulator feed ({health.get('source')}) answered at {sim_url}"
    if (net_dir / iface).exists():
        return "hardware", f"no simulator feed, and the arm's interface {iface} is here"
    return "sim", f"no arm interface ({iface}) on this machine; waiting for a simulator feed at {sim_url}"


SIM_REFUSAL = "sim mode: the page follows the simulator, and commands only ever go to the real arm"


# ------------------------------------------------------------------ arm state


class _PollProxy:
    """Hands the mover a client whose `poll` does not touch DDS.

    One thread owns the readers (the state loop); a second poller on the same
    waitset would race it. The mover only needs the cache to be fresh, and the
    state loop keeps it so.
    """

    def __init__(self, client):
        self._client = client

    def poll(self, timeout_s: float = 0.0) -> bool:
        time.sleep(min(max(timeout_s, 0.0), 0.02))
        return True

    def __getattr__(self, name):
        return getattr(self._client, name)


class ArmServer:
    def __init__(self, *, iface: str, legs: bool, sphere_radius_m: float, step_deg: float,
                 log_path: Path | None, level_tool: bool = True, mode: str = "hardware", mode_reason: str = "",
                 client=None, camera=None, pick_cfg=None):
        self.mode, self.mode_reason = mode, mode_reason
        self.camera = camera
        # What a pick would run with: camera model, wrist mount, and the bench frame the operator stated.
        # None means the page's PICK button is off, and `pick_refusal` says why.
        self.pick_cfg = pick_cfg
        # The wrist mount lives here rather than inside `pick_cfg`, because it is worth editing on a
        # workstation with no arm: that is where the simulator runs, and the file is what the two ends
        # share. When a pick *is* configured they are the same object, so editing moves the frame the
        # perception deprojects through.
        if pick_cfg is not None:
            self.mount, self.mount_source = pick_cfg["mount"], pick_cfg.get("mount_source", "")
            self.mount_file = pick_cfg.get("mount_file")
        else:
            self.mount, self.mount_source, self.mount_file = load_default_mount()
        self.pick_state: dict = {"running": False, "last": None}
        self.iface = iface
        self.pick_run_root = ROOT / "logs" / "pick_hardware"
        self.model = build_model(sphere_radius_m)
        self.joints, _ = d1_ik.load_urdf()
        self.step_deg = step_deg
        self.log_path = log_path

        self.live = threading.Event()          # nothing is published until set
        self.cancel = threading.Event()
        self.busy_lock = threading.Lock()
        self.lock = threading.Lock()
        self.subscribers: list[queue.Queue] = []
        self.log: list[str] = []
        self.state: dict = {"connected": False, "busy": None, "progress": None, "last_result": None,
                            "legs_live": False, "live": False, "go2_q_rad": None}

        # The simulator's feed client mirrors D1Client's reads, so everything below is shared.
        self.client = client if client is not None else d1_hardware.D1Client(iface=iface)
        # Level by default: the gripper carries a camera, and a level tool means a
        # level horizon. Targets that cannot be reached levelly are reported as such
        # rather than silently solved tilted (F-037).
        self.mover = d1_hardware.CartesianMover(_PollProxy(self.client), self.joints,
                                                max_joint_step_deg=step_deg, level=level_tool)
        self.level_tool = level_tool
        self.lowstate = self._open_lowstate() if legs and self.mode == "hardware" else None
        threading.Thread(target=self._state_loop, daemon=True, name="d1-ui-state").start()

    # ---- logging

    def _log(self, msg: str) -> None:
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        with self.lock:
            self.log.append(line)
            del self.log[:-60]

    def _record(self, entry: dict) -> None:
        """Every executed command goes to a JSONL file: the evidence of what the page did."""
        if self.log_path is None:
            return
        entry = {"t": time.time(), **entry}
        with self.log_path.open("a") as fh:
            fh.write(json.dumps(entry) + "\n")

    # ---- Go2 legs

    def _open_lowstate(self):
        try:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
            from cyclonedds.sub import DataReader
            from cyclonedds.topic import Topic
            dp = self.client.participant
            return DataReader(dp, Topic(dp, "rt/lowstate", LowState_))
        except Exception as exc:   # the page still works with the legs static
            self._log(f"legs: rt/lowstate unavailable ({exc}); Go2 legs will not be live")
            return None

    # ---- state broadcast

    def _snapshot(self) -> dict:
        try:
            servo = self.client.get_joint_angles()
        except RuntimeError:
            servo = None    # nothing from the arm (or the simulator) yet; the page says so
        age = self.client.feedback_age_s
        snap = {
            "t": time.time(),
            "mode": self.mode, "mode_reason": self.mode_reason,
            # Stale is not connected. Hardware mode used to be exempt from the age check, so a cache
            # that had not been refreshed since the state thread died still read as connected and the
            # page drew six-minute-old joints as though they were live (2026-09-17, F-061). The arm
            # reports every 111 ms (F-020), so a second of silence is already several missed cycles.
            "connected": servo is not None and age < STALE_FEEDBACK_S,
            "servo_deg": None, "q_rad": None, "tool_m": None,
            "gripper_units": None, "finger_m": None, "base_height_m": None, "sim": None,
            "power": self.client.is_powered(), "enable": self.client.is_enabled(),
            "error": self.client.error_status(),
            "feedback_age_s": round(age, 3) if age != float("inf") else None,
            "safe_to_release": self.client.is_safe_to_release(),
            "live": self.live.is_set(),
            "camera": self.camera.status() if self.camera is not None else
                      {"available": False, "message": "camera off (--camera none)", "detections": []},
            "pick": self.pick_status(),
        }
        if servo is not None:
            q = d1_ik.from_servo_deg(servo)[0]
            tool, _ = d1_ik.tool_pose(self.joints, q)
            snap.update(servo_deg=[round(v, 2) for v in servo], q_rad=[round(float(v), 5) for v in q],
                        tool_m=[round(float(v), 4) for v in tool])
        try:
            units = self.client.get_gripper_units()
        except RuntimeError:
            units = None
        if units is not None:
            snap["gripper_units"] = round(units, 1)
        if self.mode == "sim":
            sim = self.client.state or {}
            snap.update(finger_m=sim.get("finger_m"), base_height_m=sim.get("base_height_m"),
                        sim={k: sim.get(k) for k in ("source", "sim_time_s", "status")})
        with self.lock:
            snap.update({k: self.state[k] for k in ("busy", "progress", "last_result", "legs_live", "go2_q_rad")})
            snap["log"] = self.log[-12:]
        return snap

    def _state_loop(self) -> None:
        """The only thing reading the arm. It must not be possible to kill it.

        It was: one malformed DDS sample raised out of `poll` and took this thread with it, after which
        the page went on showing the arm's last known joints as though they were live, for six minutes
        (2026-09-17, F-061). A console that silently freezes is worse than one that says it has lost the
        arm, so anything unexpected is logged once and the loop carries on. `feedback_age_s` and
        `connected` are what tell the page the truth.
        """
        last_servo = None
        go2_q = None
        last_sent = 0.0
        last_error = None
        while True:
            try:
                got = self.client.poll(timeout_s=0.1)
                if self.mode == "sim":
                    legs = (self.client.state or {}).get("legs_q_rad")
                    with self.lock:
                        self.state["go2_q_rad"] = legs
                        self.state["legs_live"] = legs is not None
                if self.lowstate is not None:
                    try:
                        for s in self.lowstate.take(N=20):
                            go2_q = [round(float(s.motor_state[i].q), 4) for i in range(12)]
                        if go2_q is not None:
                            with self.lock:
                                self.state["go2_q_rad"] = go2_q
                                self.state["legs_live"] = True
                    except Exception:
                        pass
                try:
                    servo = self.client.get_joint_angles()
                except RuntimeError:
                    servo = None
                # Unchanged: still send one a second, so feedback age and the camera's status keep moving.
                if not got and servo == last_servo and time.monotonic() - last_sent < 1.0:
                    continue
                last_servo = servo
                last_sent = time.monotonic()
                self._broadcast(self._snapshot())
                last_error = None
            except Exception as exc:      # noqa: BLE001 - deliberately broad; see the docstring
                if repr(exc) != last_error:
                    self._log(f"state loop: {exc!r} -- continuing; watch the feedback age")
                    last_error = repr(exc)
                time.sleep(0.1)

    def _broadcast(self, snap: dict) -> None:
        payload = json.dumps(snap)
        with self.lock:
            subs = list(self.subscribers)
        for q in subs:
            try:
                q.put_nowait(payload)
            except queue.Full:
                pass

    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=8)
        with self.lock:
            self.subscribers.append(q)
        try:
            q.put_nowait(json.dumps(self._snapshot()))
        except Exception:
            pass
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        with self.lock:
            if q in self.subscribers:
                self.subscribers.remove(q)

    # ---- commands

    def validate_target(self, target) -> tuple[bool, str]:
        sph = self.model["sphere"]
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            return False, "target must be three finite numbers"
        if target[2] < sph["min_z"]:
            return False, f"target below the mount plane (z < {sph['min_z']:.2f} m)"
        if np.linalg.norm(target - np.asarray(sph["center"])) > sph["radius"] + 0.02:
            return False, "target outside the reach sphere"
        return True, ""

    def preview(self, target) -> dict:
        ok, why = self.validate_target(target)
        if not ok:
            return {"ok": False, "reason": why}
        try:
            self.client.get_joint_angles()
        except RuntimeError:
            return {"ok": False, "reason": "no joint angles yet: nothing to plan from"}
        plan = self.mover.plan(target)
        clear = bool(clear_of_body(self.joints, plan.q[None, :], PROXY_BASE_HEIGHT_M)[0])
        # The endpoint being clear does not mean the way there is (F-036).
        here = d1_ik.from_servo_deg(self.client.get_joint_angles())[0]
        path_ok, frac = d1_ik.path_clearance(self.joints, here, plan.q, PROXY_BASE_HEIGHT_M)
        ok = plan.converged and clear and path_ok
        reason = ""
        if not plan.converged:
            reason = "IK did not converge"
        elif not clear:
            reason = "solution folds into the trunk/ground proxy"
        elif not path_ok:
            reason = f"the path there sweeps into the trunk/ground proxy {100*frac:.0f}% along"
        elif self.level_tool and max(abs(plan.elevation_deg), abs(plan.roll_deg)) > 1.0:
            ok = False
            reason = (f"reachable, but not with the gripper level "
                      f"(best is {plan.elevation_deg:+.0f}° pitch, {plan.roll_deg:+.0f}° roll)")
        return {
            "ok": ok,
            "reason": reason,
            "converged": plan.converged, "clear": clear, "path_clear": path_ok,
            "position_error_mm": round(plan.position_error_m * 1000, 2),
            "iterations": plan.iterations,
            "elevation_deg": round(plan.elevation_deg, 2),
            "roll_deg": round(plan.roll_deg, 2),
            "level_requested": self.level_tool,
            "level_ok": (not self.level_tool) or max(abs(plan.elevation_deg), abs(plan.roll_deg)) <= 1.0,
            "servo_deg": [round(v, 2) for v in plan.servo_deg],
            "delta_deg": [round(v, 2) for v in (np.asarray(plan.servo_deg) -
                                                 np.asarray(self.client.get_joint_angles()))],
        }

    def _start(self, name: str, fn) -> tuple[bool, str]:
        if not self.busy_lock.acquire(blocking=False):
            return False, "arm is busy"
        self.cancel.clear()
        with self.lock:
            self.state["busy"] = name
            self.state["progress"] = None

        def worker():
            try:
                fn()
            except Exception as exc:
                self._log(f"{name}: ERROR {exc}")
                with self.lock:
                    self.state["last_result"] = {"action": name, "ok": False, "reason": str(exc)}
            finally:
                with self.lock:
                    self.state["busy"] = None
                self.busy_lock.release()
                self._broadcast(self._snapshot())

        threading.Thread(target=worker, daemon=True, name=f"d1-ui-{name}").start()
        return True, ""

    def move_to(self, target) -> tuple[bool, str]:
        if self.mode == "sim":
            return False, SIM_REFUSAL
        pre = self.preview(target)
        if not pre["ok"]:
            return False, pre["reason"]
        live = self.live.is_set()
        target = [float(v) for v in target]

        def on_step(step):
            with self.lock:
                self.state["progress"] = {"step": step.index, "error_mm": round(step.target_error_m * 1000, 1)}

        def job():
            self._log(f"move -> {target} ({'LIVE' if live else 'dry run'}), plan {pre['servo_deg']}")
            # One waypoint per pass, not one per cycle: the arm needs ~2 feedback
            # cycles to reach cruise, so a per-cycle loop restarts its acceleration
            # forever and the motion visibly steps (F-035).
            steps = self.mover.run_oneshot(target, execute=live, passes=2,
                                           on_event=self._log,
                                           should_stop=self.cancel.is_set)
            for st in steps:
                on_step(st)
            final = steps[-1].target_error_m * 1000 if steps else float("nan")
            result = {"action": "move", "ok": True, "live": live, "target": target,
                      "passes": len(steps), "final_error_mm": round(final, 2),
                      "stop_reason": self.mover.last_stop_reason}
            self._log(f"move done: {len(steps)} pass(es), {final:.1f} mm, {self.mover.last_stop_reason}")
            with self.lock:
                self.state["last_result"] = result
            self._record(result)

        return self._start("move", job)

    def park(self) -> tuple[bool, str]:
        if self.mode == "sim":
            return False, SIM_REFUSAL
        live = self.live.is_set()

        def job():
            self._log(f"park -> folded ({'LIVE' if live else 'dry run'})")
            steps = self.mover.approach_joints(FOLDED_PARK_DEG, execute=live,
                                               should_stop=self.cancel.is_set)
            worst = steps[-1].target_error_m if steps else float("nan")
            result = {"action": "park", "ok": True, "live": live, "steps": len(steps),
                      "worst_joint_error_deg": round(worst, 2),
                      "stop_reason": self.mover.last_stop_reason}
            self._log(f"park done: {len(steps)} steps, worst {worst:.2f} deg, {self.mover.last_stop_reason}")
            with self.lock:
                self.state["last_result"] = result
            self._record(result)

        return self._start("park", job)

    # ---- the scripted pick

    def pick_refusal(self) -> str:
        """Why the page's PICK button is unavailable, or "" when it is not."""
        if self.mode == "sim":
            return SIM_REFUSAL
        if self.pick_cfg is None:
            return "the pick needs the arm and the camera, and this console has neither"
        if self.camera is None:
            return "the pick needs the camera window, and it is off (--camera none)"
        if not self.camera.has_depth():
            status = self.camera.status()
            return (f"no depth from the camera ({status.get('message') or status.get('source')}): "
                    "the pick places a cup with depth, so start the server with --pick-depth")
        return ""

    # ----------------------------------------------------------------- the wrist mount editor
    #
    # The mount is where the camera sits on the wrist, and until a bracket is measured it is a guess
    # (F-054's scope note). The console lets an operator move the CAD camera against the wrist in the
    # 3D view and save the result, so the simulator and this controller can be handed the same six
    # numbers instead of each carrying its own default.
    #
    # Setting it takes effect immediately -- `CupPerception` holds the same object, so the next frame
    # is deprojected through the new mount -- but only saving writes a file. A mount changed while a
    # pick is running would move the frame under the sequence, so that is refused.

    def mount_status(self) -> dict:
        from demos.cup.pick_demo import camera_body
        from demos.cup.pick_demo.camera import mount_as_xyz_rpy

        with self.lock:
            mount, source, saved_to = self.mount, self.mount_source, self.mount_file
            busy = bool(self.pick_state.get("running"))
        xyz, rpy = mount_as_xyz_rpy(mount)
        return {
            "available": True,
            "drives_perception": self.pick_cfg is not None,
            "xyz_m": [round(float(v), 6) for v in xyz],
            "rpy_deg": [round(float(v), 4) for v in rpy],
            "source": source,
            "saved_to": saved_to,
            "editable": not busy,
            "mesh": "/meshes/realsense/d435_housing.ply",
            # The mesh is in the CAD's own frame; this 4x4 (row-major) takes it into the colour optical
            # frame, which is what the six numbers above place. Sent rather than repeated in JavaScript
            # so `camera_body` stays the one place the registration is written down.
            "mesh_to_optical": [[round(float(v), 9) for v in row] for row in camera_body.MESH_TO_OPTICAL],
            "mesh_available": (PICK_ASSETS / "realsense" / "d435_housing.ply").is_file(),
            # The same check the simulator's runs report: a mount that buries the camera in the wrist
            # is a number here rather than something to notice later in a render. It needs the case
            # mesh, and so `trimesh`, which the dog's deployment has no reason to carry -- so its
            # absence costs the warning and nothing else. The editor itself needs neither.
            "clearance": _mount_clearance(mount),
            "note": ("aligned by eye against the CAD; this is not a hand-eye calibration and the file "
                     "it writes says so"
                     + ("" if self.pick_cfg is not None
                        else " -- and no pick is configured here, so this edits a file only")),
        }

    def set_mount(self, xyz_m, rpy_deg) -> tuple[bool, str]:
        """Move the camera's optical frame on the wrist. Live for the next frame; nothing is written."""
        from demos.cup.pick_demo.camera import mount_from_xyz_rpy

        try:
            values = [float(v) for v in xyz_m], [float(v) for v in rpy_deg]
            if len(values[0]) != 3 or len(values[1]) != 3:
                raise ValueError("xyz_m and rpy_deg each need three numbers")
            if not all(math.isfinite(v) for v in values[0] + values[1]):
                raise ValueError("xyz_m and rpy_deg must be finite")
            if max(abs(v) for v in values[0]) > 0.5:
                raise ValueError("the camera cannot be half a metre from the wrist; check the units (metres)")
        except (TypeError, ValueError) as exc:
            return False, str(exc)
        with self.lock:
            if self.pick_state.get("running"):
                return False, "a pick is running; the mount cannot move under it"
            mount = mount_from_xyz_rpy(*values, source="console mount editor (unsaved)")
            self.mount, self.mount_source, self.mount_file = mount, "console mount editor, unsaved", None
            if self.pick_cfg is not None:
                self.pick_cfg["mount"] = mount
                self.pick_cfg["perception"].mount = mount
                self.pick_cfg["mount_source"] = self.mount_source
                self.pick_cfg["mount_file"] = None
        return True, ""

    def save_mount(self, name: str = "") -> tuple[bool, str]:
        """Write the current mount to `demos/cup/pick_demo/assets/mounts/`. Returns the path, or why not."""
        from demos.cup.pick_demo.camera import save_mount as write_mount

        stem = "".join(c for c in (name or "wrist_mount") if c.isalnum() or c in "-_") or "wrist_mount"
        path = PICK_ASSETS / "mounts" / f"{stem}.json"
        with self.lock:
            mount = self.mount
            camera = (self.pick_cfg or {}).get("camera_source", "")
        try:
            write_mount(path, mount,
                        source=f"d1_ui mount editor on {socket.gethostname()}",
                        measured=False,
                        method=("the operator aligned the CAD camera against the wrist in the console's "
                                "3D view; no target, no hand-eye procedure, no image evidence"),
                        camera=camera)
        except OSError as exc:
            return False, f"could not write {path}: {exc}"
        with self.lock:
            self.mount_file = str(path)
            self.mount_source = f"console mount editor, saved to {path.name}"
            if self.pick_cfg is not None:
                self.pick_cfg["mount_file"] = self.mount_file
                self.pick_cfg["mount_source"] = self.mount_source
        return True, str(path)

    def pick_status(self) -> dict:
        with self.lock:
            state = dict(self.pick_state)
        refusal = self.pick_refusal()
        cfg = self.pick_cfg or {}
        state.update({
            "available": not refusal,
            "refusal": refusal,
            "configured": self.pick_cfg is not None,
            "up_b": cfg.get("up_b"),
            "mount_source": cfg.get("mount_source"),
            "camera_source": cfg.get("camera_source"),
            "gripper": ("commanded on an unverified scale" if cfg.get("grip_gripper", True)
                        else "not commanded (--pick-no-gripper)"),
            "gripper_units_range": list(d1_hardware.GRIPPER_UNITS_RANGE),
        })
        return state

    def pick(self) -> tuple[bool, str]:
        """Run the scripted pick. Motion only happens when LIVE is on; otherwise it is a dry run."""
        refusal = self.pick_refusal()
        if refusal:
            return False, refusal
        live = self.live.is_set()
        cfg = self.pick_cfg

        def job():
            from demos.cup.pick_demo import hardware as pick_hardware

            with self.lock:
                self.pick_state = {"running": True, "last": None, "started": time.time()}
            self._log(f"pick: {'LIVE -- THE ARM WILL MOVE' if live else 'dry run, nothing is sent'}")
            try:
                result = pick_hardware.run_pick(
                    client=_PollProxy(self.client), joints=self.joints, links=cfg["links"],
                    camera_model=cfg["camera_model"], mount=cfg["mount"], perception=cfg["perception"],
                    pipeline=self.camera, up_b=cfg["up_b"], execute=live, should_stop=self.cancel.is_set, on_event=self._log,
                    grip_gripper=cfg.get("grip_gripper", True),
                    grasp_params=cfg.get("grasp_params"),
                    max_time_s=cfg.get("max_time_s", 180.0))
            finally:
                with self.lock:
                    self.pick_state["running"] = False
            payload = {"action": "pick", "live": live, **result.as_dict()}
            run_dir = self._write_pick_run(cfg, result, live)
            if run_dir is not None:
                payload["run_dir"] = str(run_dir)
                self._log(f"pick: recorded {run_dir}  (./dashboard.py record {run_dir})")
            with self.lock:
                self.pick_state["last"] = payload
                self.state["last_result"] = payload
            self._record({**payload, "events": result.events})

        return self._start("pick", job)

    def _write_pick_run(self, cfg, result, live: bool) -> Path | None:
        """A run directory for the pick, so a hardware pick joins the record like every other run.

        `results/` is the project's evidence and `./dashboard.py record` reads `run.json`, so a pick that
        left only a log line could not be cited. What it does *not* contain is any accuracy figure: there
        is no ground truth on a bench.
        """
        from demos.cup.pick_demo import hardware as pick_hardware

        try:
            stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime()) + f"_{int(time.time() % 1 * 1e6):06d}Z"
            run_dir = self.pick_run_root / f"{stamp}_pick_hw"
            run_dir.mkdir(parents=True, exist_ok=True)
            meta = pick_hardware.run_metadata(
                camera_model=cfg["camera_model"], mount=cfg["mount"], up_b=cfg["up_b"], execute=live,
                grip_gripper=cfg.get("grip_gripper", True),
                grasp_params=cfg.get("grasp_params"),
                extra={"status": "complete" if result.ok else "pick_failed",
                       "camera_source": cfg.get("camera_source"),
                       "arm": {"iface": self.iface, "mode": self.mode},
                       "result": result.as_dict()})
            (run_dir / "run.json").write_text(json.dumps(meta, indent=2) + "\n")
            (run_dir / "events.json").write_text(json.dumps(result.events, indent=2) + "\n")
            return run_dir
        except Exception as exc:
            self._log(f"pick: could not write a run directory ({exc})")
            return None


    def set_gripper(self, units) -> tuple[bool, str]:
        """Jog servo 6 to a value on the protocol's own scale.

        This exists to be measured with, not to grasp with. Nothing in this repository knows what a unit
        of servo 6 is in millimetres of jaw gap: the protocol advertises 65, the arm sits at ~41 at rest,
        and the CAD gripper says the pads cannot come closer than 17.26 mm anyway (F-059). Commanding a
        value and putting a ruler across the fingers is what settles it, and that pair of numbers is
        worth a finding.
        """
        if self.mode == "sim":
            return False, SIM_REFUSAL
        try:
            value = float(units)
        except (TypeError, ValueError):
            return False, "gripper units must be a number"
        low, high = d1_hardware.GRIPPER_UNITS_RANGE
        if not low <= value <= high:
            return False, f"gripper units must be between {low:.0f} and {high:.0f}"
        if not self.live.is_set():
            self._log(f"gripper: dry run, {value:.1f} units not sent")
            return True, "dry run"
        try:
            before = round(float(self.client.get_gripper_units()), 1)
        except RuntimeError:
            before = None
        self.client.set_gripper_units(value)
        self._log(f"gripper -> {value:.1f} units (was {before}); measure the jaw gap to learn what that is")
        self._record({"action": "gripper", "units": value, "units_before": before})
        return True, ""

    def stop(self) -> tuple[bool, str]:
        self.cancel.set()
        self._log("STOP: cancelling; the arm holds where it is")
        return True, ""

    def release(self, force: bool) -> tuple[bool, str]:
        if self.mode == "sim":
            return False, SIM_REFUSAL
        if not self.live.is_set():
            self._log("release: dry run, nothing sent")
            return True, "dry run"
        if not self.client.is_safe_to_release() and not force:
            return False, "arm is not folded: releasing would drop it (pass force to insist)"
        self.client.release()
        self._log(f"RELEASE sent{' (forced, arm not folded)' if force else ''}")
        self._record({"action": "release", "forced": force})
        return True, ""

    def set_live(self, live: bool) -> tuple[bool, str]:
        if self.mode == "sim" and live:
            return False, SIM_REFUSAL
        if live:
            self.live.set()
        else:
            self.live.clear()
        with self.lock:
            self.state["live"] = live
        self._log(f"arm is now {'LIVE' if live else 'in DRY RUN'}")
        self._broadcast(self._snapshot())
        return True, ""


# ------------------------------------------------------------------ HTTP


class Handler(BaseHTTPRequestHandler):
    server_version = "d1-ui/0.1"
    arm: ArmServer = None   # set by main()

    def log_message(self, fmt, *args):   # quieter than the default per-request line
        if any(part in fmt % args for part in ("/events", "/meshes/", "/static/", "/camera")):
            return
        super().log_message(fmt, *args)

    # ---- helpers

    def _json(self, obj, status=HTTPStatus.OK):
        body = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _file(self, path: Path, content_type: str):
        if not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        data = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "max-age=3600" if "/meshes/" in str(path) or "vendor" in str(path) else "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> dict:
        n = int(self.headers.get("Content-Length") or 0)
        return json.loads(self.rfile.read(n) or b"{}")

    # ---- GET

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            return self._file(STATIC / "index.html", "text/html; charset=utf-8")
        if path == "/model.json":
            return self._json(self.arm.model)
        if path == "/state":
            return self._json(self.arm._snapshot())
        if path == "/events":
            return self._events()
        if path == "/camera.mjpg":
            return self._mjpeg()
        if path == "/camera.jpg":
            jpeg = self.arm.camera.latest_jpeg() if self.arm.camera is not None else None
            if jpeg is None:
                return self._json({"ok": False, "reason": "no camera frame yet"}, HTTPStatus.SERVICE_UNAVAILABLE)
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(jpeg)
            return None
        if path.startswith("/static/"):
            name = Path(path).name
            sub = "vendor" if "/vendor/" in path else ""
            ctype = {"js": "application/javascript", "css": "text/css"}.get(name.rsplit(".", 1)[-1], "text/plain")
            return self._file(STATIC / sub / name, ctype)
        if path == "/mount":
            return self._json(self.arm.mount_status())
        if path.startswith("/meshes/"):
            _, _, key, name = path.split("/", 3)
            base = MESH_DIRS.get(key)
            if base is None or "/" in name or ".." in name:
                return self.send_error(HTTPStatus.NOT_FOUND)
            ctype = "model/vnd.collada+xml" if name.lower().endswith(".dae") else "application/octet-stream"
            return self._file(base / name, ctype)
        self.send_error(HTTPStatus.NOT_FOUND)

    def _events(self):
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        q = self.arm.subscribe()
        try:
            while True:
                try:
                    payload = q.get(timeout=5.0)
                    self.wfile.write(f"data: {payload}\n\n".encode())
                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.arm.unsubscribe(q)

    def _mjpeg(self):
        camera = self.arm.camera
        if camera is None:
            return self.send_error(HTTPStatus.NOT_FOUND, "camera off")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        seq = 0
        try:
            while True:
                got = camera.wait_jpeg(seq, timeout_s=5.0)
                if got is None:
                    continue
                seq, jpeg = got
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    # ---- POST

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        try:
            body = self._body()
        except Exception as exc:
            return self._json({"ok": False, "reason": f"bad JSON: {exc}"}, HTTPStatus.BAD_REQUEST)

        if path == "/target":
            target = body.get("target")
            if body.get("preview"):
                return self._json(self.arm.preview(target))
            ok, why = self.arm.move_to(target)
            return self._json({"ok": ok, "reason": why}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
        if path == "/stop":
            ok, why = self.arm.stop()
            return self._json({"ok": ok, "reason": why})
        if path == "/park":
            ok, why = self.arm.park()
            return self._json({"ok": ok, "reason": why}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
        if path == "/pick":
            ok, why = self.arm.pick()
            return self._json({"ok": ok, "reason": why}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
        if path == "/gripper":
            ok, why = self.arm.set_gripper(body.get("units"))
            return self._json({"ok": ok, "reason": why}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
        if path == "/release":
            ok, why = self.arm.release(bool(body.get("force")))
            return self._json({"ok": ok, "reason": why}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
        if path == "/arm":
            ok, why = self.arm.set_live(bool(body.get("live")))
            return self._json({"ok": ok, "reason": why, "live": self.arm.live.is_set()},
                              HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
        if path == "/mount":
            ok, why = self.arm.set_mount(body.get("xyz_m"), body.get("rpy_deg"))
            return self._json({"ok": ok, "reason": why, "mount": self.arm.mount_status()},
                              HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
        if path == "/mount/save":
            ok, detail = self.arm.save_mount(str(body.get("name") or ""))
            return self._json({"ok": ok, "path": detail if ok else None, "reason": "" if ok else detail,
                               "mount": self.arm.mount_status()},
                              HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
        self.send_error(HTTPStatus.NOT_FOUND)


def _mount_clearance(mount):
    """The case against Link6's CAD shell, or None where the mesh cannot be read (the dog)."""
    try:
        from demos.cup.pick_demo import camera_body, grasp

        return camera_body.clearance_report(mount, grasp.PALM_Z_M, grasp.PALM_X_RANGE_M)
    except Exception:
        return None


# The bench's own geometry, remembered between launches for the same reason the mount is: it is a
# property of the table the arm is bolted to, not of a command line. There is no IMU on a bench arm, so
# nothing can work the base height out -- but having typed it once, nobody should have to type it again.
_MOUNT_ARGS = {"file": None, "pos": (-0.055, 0.0, 0.035), "pitch_deg": 20.0}


def load_default_mount():
    """The mount the console starts with, and the source string that says where it came from.

    `pick_demo.camera.resolve_mount` decides: `--pick-mount` if given, else the saved default, else the
    four-number placeholder. Read from `_MOUNT_ARGS`, which `main` fills from the command line, so
    `ArmServer` can be built in a test without a pick, an arm or a camera and still have a mount.
    """
    from demos.cup.pick_demo.camera import WristMount, resolve_mount

    placeholder = WristMount(tuple(_MOUNT_ARGS["pos"]), _MOUNT_ARGS["pitch_deg"])
    return resolve_mount(_MOUNT_ARGS["file"], fallback=placeholder)   # (mount, source, path)


def build_pick_cfg(args, mode: str, camera) -> dict | None:
    """What the scripted pick needs, or None when it cannot run here.

    Built at start-up rather than per request so a misconfiguration is a line in the log at launch, not a
    failure halfway through a motion. Returning None is normal: on a workstation with no arm there is
    nothing to pick with, and the page shows PICK as unavailable and says why.

    The base height is *not* required here. It is the one number nothing can measure on a bench arm, and
    demanding it at launch meant the button was off for anyone who started the console without thinking
    of it -- so it is settable in the page and remembered, and only the button refuses while it is unset.
    """
    if mode != "hardware" or camera is None:
        return None
    try:
        from demos.cup.pick_demo.camera import CAMERAS, WristMount
        from demos.cup.pick_demo.grasp import GraspParams
        from demos.cup.pick_demo.perception import CupPerception
        from demos.cup.d1_ui import camera_feed

        # A stored calibration beats the datasheet preset, which F-053 measured as 14 degrees too wide
        # with cy 14.3 px off centre. Falling back to the preset silently was how every console started
        # without the flag got the wrong camera model, so one stored calibration is now found and used.
        calibration = args.pick_calibration
        if not calibration:
            stored = sorted((PICK_ASSETS / "calibration").glob("*.json"))
            if len(stored) == 1:
                calibration = stored[0]
            elif len(stored) > 1:
                print(f"pick: {len(stored)} calibrations stored; pass --pick-calibration to choose one",
                      flush=True)
        if calibration:
            from demos.cup.pick_demo.realsense import load_camera_model

            model = load_camera_model(calibration)
            camera_source = f"calibration {Path(calibration).name}"
        else:
            model = CAMERAS["d435"]
            camera_source = "d435 datasheet preset -- NOT this camera (F-053)"
        print(f"pick: camera model -- {camera_source}", flush=True)
        # A saved mount file wins over the four-number placeholder: it is what the console's editor
        # writes and what the simulator can be handed, so both ends agree on where the camera is.
        from demos.cup.pick_demo.camera import resolve_mount

        mount, mount_source, mount_file = resolve_mount(
            args.pick_mount, fallback=WristMount(tuple(args.pick_mount_pos), args.pick_mount_pitch_deg))
        print(f"pick: wrist mount -- {mount_source}", flush=True)
        joints, links = d1_ik.load_urdf()
        weights = args.yolo_weights or camera_feed.DEFAULT_WEIGHTS
        detector = camera_feed.make_yolo(weights, device=args.yolo_device)
        grasp_params = (GraspParams(wall_grasp=args.pick_wall_grasp) if args.pick_closed_gap is None else
                        GraspParams(wall_grasp=args.pick_wall_grasp, pinch_closed_gap_m=args.pick_closed_gap))
        print(f"pick: wide cups -- {grasp_params.wall_grasp}, jaws shut to "
              f"{1000 * grasp_params.pinch_closed_gap_m:.1f} mm", flush=True)
        return {
            "camera_model": model, "mount": mount, "links": links,
            "perception": CupPerception(detector, model, joints, mount),
            "up_b": [float(v) for v in args.pick_up],
            "max_time_s": float(args.pick_max_time),
            "grip_gripper": not args.pick_no_gripper,
            # How a cup too wide for the jaws is grasped. The default falls back to pinching its wall,
            # which is only possible because the real gripper shuts far further than the URDF's (F-059).
            "grasp_params": grasp_params,
            "camera_source": camera_source,
            "mount_source": mount_source,
            "mount_file": mount_file,
        }
    except Exception as exc:
        print(f"pick: unavailable ({exc})", flush=True)
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--iface", default=d1_hardware.ARM_IFACE)
    ap.add_argument("--no-legs", action="store_true", help="Do not subscribe to the Go2's rt/lowstate.")
    ap.add_argument("--sphere-radius", type=float, default=0.40, help="Reach sphere radius, metres.")
    ap.add_argument("--max-joint-step-deg", type=float, default=5.0)
    ap.add_argument("--no-level", action="store_true",
                    help="Do not require the gripper to finish level (more of the sphere is reachable).")
    ap.add_argument("--log", default="/tmp/d1_ui_log.jsonl", help="JSONL record of executed commands.")
    ap.add_argument("--mode", choices=("auto", "sim", "hardware"), default="auto",
                    help="auto: a simulator feed answering means sim, the arm's NIC being here means hardware.")
    ap.add_argument("--sim-url", default=SIM_FEED_URL, help="Where a simulator publishes (demos/cup/d1_ui/sim_feed.py).")
    ap.add_argument("--camera", choices=("auto", "none"), default="auto",
                    help="auto: the simulator's wrist camera in sim mode, the RealSense on hardware.")
    ap.add_argument("--detect", default="cup",
                    help="Comma-separated COCO labels to box, or 'all'. 'none' shows frames without YOLO.")
    ap.add_argument("--apriltag", action="store_true",
                    help="Outline tag36h11 AprilTags in the camera window, with their range when the camera's "
                         "intrinsics are known (the combiner box's door carries one).")
    ap.add_argument("--apriltag-size", type=float, default=0.06, metavar="M",
                    help="Black-square edge of the tags, for their range (default: the combiner's 60 mm).")
    ap.add_argument("--yolo-weights", default=None, help="Ultralytics weights (default: the pick demo's).")
    ap.add_argument("--yolo-device", default="auto", help="auto, cpu or cuda:N.")
    ap.add_argument("--camera-fps", type=float, default=15.0, help="Cap on frames detected and streamed.")
    ap.add_argument("--pick-depth", action=argparse.BooleanOptionalAction, default=None,
                    help="Stream depth beside colour. Required by the scripted pick, which places the cup "
                         "with it, and so on by default in hardware mode; --no-pick-depth turns it off "
                         "where the USB bandwidth is wanted for something else.")
    ap.add_argument("--pick-up", type=float, nargs=3, default=(0.0, 0.0, 1.0), metavar=("X", "Y", "Z"),
                    help="World up in the arm's base frame. The default suits an arm standing upright.")
    ap.add_argument("--pick-calibration", default=None,
                    help="A pick_demo.realsense calibration for the wrist camera. Default: the one stored "
                         "in demos/cup/pick_demo/assets/calibration/ when exactly one is. Only with none stored does "
                         "it fall back to the datasheet preset, which is measurably wrong (F-053).")
    ap.add_argument("--pick-mount-pos", type=float, nargs=3, default=(-0.055, 0.0, 0.035), metavar=("X", "Y", "Z"),
                    help="Camera optical origin in the Link6 frame. Assumed until the bracket is measured.")
    ap.add_argument("--pick-mount-pitch-deg", type=float, default=20.0)
    ap.add_argument("--pick-mount", default=None, metavar="FILE",
                    help="A mount file written by the console's mount editor (or pick_demo.camera.save_mount). "
                         "Default: demos/cup/pick_demo/assets/mounts/wrist_mount.json when it exists, so a mount "
                         "saved in the console is used on every launch with nothing to pass. "
                         "'none' forces --pick-mount-pos/--pick-mount-pitch-deg instead.")
    ap.add_argument("--pick-max-time", type=float, default=180.0, help="Seconds before the pick gives up.")
    ap.add_argument("--pick-no-gripper", action="store_true",
                    help="Do not command servo 6 during the pick: reach, descend and lift only. The "
                         "gripper is commanded by default, on a scale nobody has measured (F-059), so a "
                         "run records the units it asked for and claims no millimetres.")
    ap.add_argument("--pick-wall-grasp", choices=("auto", "off", "outside", "wall", "inside_out", "pinch"),
                    default="auto",
                    help="What to do with a cup too wide for the 77.2 mm jaws: auto pinches its wall, and "
                         "failing that goes inside the cup and presses outwards; off refuses the cup.")
    ap.add_argument("--pick-closed-gap", type=float, default=None, metavar="M",
                    help="What the jaws shut to, in metres, which decides whether a wall pinch can touch "
                         "the wall at all. Defaults to grasp.GraspParams (2 mm, stated not measured, "
                         "2026-09-17); the URDF's 17.2 mm would refuse every pinch (F-059).")
    args = ap.parse_args()

    mode, reason = detect_mode(args.mode, args.sim_url, args.iface)
    print(f"mode: {mode.upper()} ({reason})", flush=True)
    # Bind before anything slow. Starting the camera pipeline imports torch and loads the detector,
    # which takes tens of seconds and prints nothing; binding after it meant a port clash surfaced as a
    # traceback long after the terminal had gone quiet, and a second console started over a first one
    # looked like the script doing nothing at all.
    print(f"d1_ui: starting on {args.bind}:{args.port} ({mode} mode)", flush=True)
    try:
        httpd = ThreadingHTTPServer((args.bind, args.port), Handler)
    except OSError as exc:
        holder = ""
        try:
            out = subprocess.run(["ss", "-ltnp", f"sport = :{args.port}"],
                                 capture_output=True, text=True, timeout=5).stdout
            for line in out.splitlines()[1:]:
                if "pid=" in line:
                    who = line.split('users:((', 1)[-1].split('))', 1)[0]
                    name, _, rest = who.partition(",")
                    pid = rest.split("pid=", 1)[-1].split(",", 1)[0]
                    holder = f" -- held by {name.strip(chr(34))} (pid {pid})"
        except (OSError, subprocess.SubprocessError):
            pass
        print(f"d1_ui: cannot listen on {args.bind}:{args.port}: {exc}{holder}", file=sys.stderr, flush=True)
        print(f"d1_ui: stop the console already running there, or pass --port/D1_UI_PORT with a free one.",
              file=sys.stderr, flush=True)
        return 2
    httpd.daemon_threads = True

    client = SimFeedClient(args.sim_url) if mode == "sim" else None
    camera = None
    if args.camera == "auto":
        from demos.cup.d1_ui import camera_feed

        source = (camera_feed.SimFrameSource(SimFeedClient(args.sim_url)) if mode == "sim"
                  else camera_feed.RealSenseSource(
                      want_depth=args.pick_depth if args.pick_depth is not None else True))
        targets = tuple(label.strip() for label in args.detect.split(",") if label.strip())
        weights = args.yolo_weights or camera_feed.DEFAULT_WEIGHTS
        factory = None if targets == ("none",) else (
            lambda: camera_feed.make_yolo(weights, device=args.yolo_device))
        camera = camera_feed.CameraPipeline(source, factory, targets=targets, max_fps=args.camera_fps,
                                            tag_size_m=args.apriltag_size if args.apriltag else None)

    _MOUNT_ARGS.update(file=args.pick_mount, pos=tuple(args.pick_mount_pos), pitch_deg=args.pick_mount_pitch_deg)
    pick_cfg = build_pick_cfg(args, mode, camera)
    Handler.arm = ArmServer(iface=args.iface, legs=not args.no_legs, sphere_radius_m=args.sphere_radius,
                            step_deg=args.max_joint_step_deg, log_path=Path(args.log) if args.log else None,
                            level_tool=not args.no_level, mode=mode, mode_reason=reason, client=client,
                            camera=camera, pick_cfg=pick_cfg)
    if camera is not None:
        camera.log = Handler.arm._log
        camera.start()
    Handler.arm._log(f"{mode.upper()} mode: {reason}")
    Handler.arm._log(f"serving on http://{args.bind}:{args.port}  " +
                     ("(following the simulator; commands refused)" if mode == "sim"
                      else "(DRY RUN until the LIVE switch is on)"))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
