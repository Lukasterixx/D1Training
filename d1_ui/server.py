"""Browser control surface for the physical D1 on the Go2.

Serves a three.js page that draws the Go2 and the D1 from their URDFs, moves the
joints live from the arm's feedback (and the dog's `rt/lowstate`), and lets the
user click a point on a translucent sphere around the arm to send the tool there.

Runs **on the Jetson payload**, because CycloneDDS needs the arm's subnet:

    python3 d1_ui/server.py --port 8090

then open http://<dog>:8090 from anywhere that can reach it (Tailscale works).

**Sim or hardware** is decided at start-up (`--mode auto`): if a simulator's feed answers on localhost
(`d1_ui/sim_feed.py`, published by `run_pick_demo.py` and `main.py`), the page follows the simulator --
its joints, legs and rendered wrist camera -- and every command that would move an arm is refused. Otherwise,
if this machine has the arm's network interface, it is the dog, and the page drives the real arm as before.
Neither means a workstation with no simulator up yet: sim mode, waiting for one.

The camera window shows the wrist RealSense (the simulator's render in sim mode) with the pick's stock YOLO
boxes drawn on, served as MJPEG from `/camera.mjpg` (`d1_ui/camera_feed.py`).

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
import queue
import sys
import threading
import time
import xml.etree.ElementTree as ET
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import d1_ik  # noqa: E402
import d1_hardware  # noqa: E402
from d1_ui.sim_feed import DEFAULT_URL as SIM_FEED_URL, GO2_MOTOR_ORDER, SimFeedClient  # noqa: E402
from position_only.workspace import BODY_BOX_B, MOUNT_B, clear_of_body  # noqa: E402

HERE = Path(__file__).resolve().parent
STATIC = HERE / "static"
D1_URDF = ROOT / "d1_arm" / "d1.urdf"
GO2_URDF = ROOT / "description" / "go2_d1.urdf"
MESH_DIRS = {"d1": ROOT / "d1_arm" / "meshes", "go2": ROOT / "description" / "meshes" / "go2"}

FOLDED_PARK_DEG = [0.0, -89.9, 89.9, 0.0, 0.0, 0.0]   # servo degrees; the hard limits of J1/J2
SPHERE_MIN_Z_ABOVE_MOUNT_M = 0.03
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
                 client=None, camera=None):
        self.mode, self.mode_reason = mode, mode_reason
        self.camera = camera
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
            "connected": servo is not None and (self.mode == "hardware" or age < 1.0),
            "servo_deg": None, "q_rad": None, "tool_m": None,
            "gripper_units": None, "finger_m": None, "base_height_m": None, "sim": None,
            "power": self.client.is_powered(), "enable": self.client.is_enabled(),
            "error": self.client.error_status(),
            "feedback_age_s": round(age, 3) if age != float("inf") else None,
            "safe_to_release": self.client.is_safe_to_release(),
            "live": self.live.is_set(),
            "camera": self.camera.status() if self.camera is not None else
                      {"available": False, "message": "camera off (--camera none)", "detections": []},
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
        last_servo = None
        go2_q = None
        last_sent = 0.0
        while True:
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
        if path == "/release":
            ok, why = self.arm.release(bool(body.get("force")))
            return self._json({"ok": ok, "reason": why}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
        if path == "/arm":
            ok, why = self.arm.set_live(bool(body.get("live")))
            return self._json({"ok": ok, "reason": why, "live": self.arm.live.is_set()},
                              HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
        self.send_error(HTTPStatus.NOT_FOUND)


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
    ap.add_argument("--sim-url", default=SIM_FEED_URL, help="Where a simulator publishes (d1_ui/sim_feed.py).")
    ap.add_argument("--camera", choices=("auto", "none"), default="auto",
                    help="auto: the simulator's wrist camera in sim mode, the RealSense on hardware.")
    ap.add_argument("--detect", default="cup",
                    help="Comma-separated COCO labels to box, or 'all'. 'none' shows frames without YOLO.")
    ap.add_argument("--yolo-weights", default=None, help="Ultralytics weights (default: the pick demo's).")
    ap.add_argument("--yolo-device", default="auto", help="auto, cpu or cuda:N.")
    ap.add_argument("--camera-fps", type=float, default=15.0, help="Cap on frames detected and streamed.")
    args = ap.parse_args()

    mode, reason = detect_mode(args.mode, args.sim_url, args.iface)
    print(f"mode: {mode.upper()} ({reason})", flush=True)
    client = SimFeedClient(args.sim_url) if mode == "sim" else None
    camera = None
    if args.camera == "auto":
        from d1_ui import camera_feed

        source = (camera_feed.SimFrameSource(SimFeedClient(args.sim_url)) if mode == "sim"
                  else camera_feed.RealSenseSource())
        targets = tuple(label.strip() for label in args.detect.split(",") if label.strip())
        weights = args.yolo_weights or camera_feed.DEFAULT_WEIGHTS
        factory = None if targets == ("none",) else (
            lambda: camera_feed.make_yolo(weights, device=args.yolo_device))
        camera = camera_feed.CameraPipeline(source, factory, targets=targets, max_fps=args.camera_fps)

    Handler.arm = ArmServer(iface=args.iface, legs=not args.no_legs, sphere_radius_m=args.sphere_radius,
                            step_deg=args.max_joint_step_deg, log_path=Path(args.log) if args.log else None,
                            level_tool=not args.no_level, mode=mode, mode_reason=reason, client=client,
                            camera=camera)
    if camera is not None:
        camera.log = Handler.arm._log
        camera.start()
    Handler.arm._log(f"{mode.upper()} mode: {reason}")
    Handler.arm._log(f"serving on http://{args.bind}:{args.port}  " +
                     ("(following the simulator; commands refused)" if mode == "sim"
                      else "(DRY RUN until the LIVE switch is on)"))
    httpd = ThreadingHTTPServer((args.bind, args.port), Handler)
    httpd.daemon_threads = True
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
