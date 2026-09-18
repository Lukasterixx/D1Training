"""The reach console's sim mode: a simulator publishes its robot and wrist camera, the console reads them.

A simulator (`demos/cup/run_pick_demo.py`, `main.py`) owns a `SimFeed`, a small HTTP server on localhost that holds
the latest joint positions and wrist-camera frame. The console (`demos/cup/d1_ui/server.py`) finds it at start-up
and reads it through `SimFeedClient`, which mirrors the read half of `d1_hardware.D1Client`, so the page
draws the simulated arm with the same code that draws the real one.

    GET /health           {"sim": true, "source": ..., "camera": bool}
    GET /state            the latest state as JSON (404 until the first publish)
    GET /frame?after=SEQ&feed=ID  the latest RGB frame as raw bytes, shape in the headers
                                  (304 if this feed has none newer than SEQ)

Every feed has its own `id`. Sequence numbers restart with the simulator, so a reader that saw frame 900
from the last run must not wait for this run's frame 901: a different id means "start over".

The simulator only does the work when someone is reading: `wants_state()` and `wants_frame()` stay false
until the console has asked recently, so a sim with no console open copies nothing off the GPU.

Stdlib and numpy only: the publisher runs inside Isaac Sim's Python, the client wherever the console runs.
The publisher never raises into a simulation loop; a port already in use turns the feed off with a warning.
"""
from __future__ import annotations

import json
import os
import threading
import time
import urllib.error
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

DEFAULT_PORT = 8765
DEFAULT_URL = f"http://127.0.0.1:{DEFAULT_PORT}"

ARM_JOINTS = [f"Joint{i}" for i in range(1, 7)]
FINGER_JOINTS = ["Joint7_1", "Joint7_2"]
# unitree_sdk2 LowState motor order (position_only/deploy.py GO2_SDK_JOINT_ORDER).
GO2_MOTOR_ORDER = [f"{leg}_{part}_joint" for leg in ("FR", "FL", "RR", "RL") for part in ("hip", "thigh", "calf")]

# A reader that has not asked for this long is gone; stop copying for it.
_READER_TIMEOUT_S = 3.0


def _pick(names, positions, wanted):
    index = {name: i for i, name in enumerate(names)}
    if not all(name in index for name in wanted):
        return None
    return [round(float(positions[index[name]]), 5) for name in wanted]


class SimFeed:
    """Held by the simulator. Call `publish_joints` / `publish_frame` from the sim loop when `wants_*` says so."""

    def __init__(self, source: str, *, camera: bool, port: int = DEFAULT_PORT, host: str = "127.0.0.1",
                 max_state_hz: float = 30.0, max_frame_hz: float = 15.0, camera_info: dict | None = None, log=print):
        self.source, self.camera, self.camera_info = source, camera, camera_info or {}
        self.id = f"{os.getpid()}-{time.time_ns()}"
        self._state_period, self._frame_period = 1.0 / max_state_hz, 1.0 / max_frame_hz
        self._lock = threading.Lock()
        self._state: bytes | None = None
        self._state_seq = 0
        self._frame: tuple[int, bytes, dict] | None = None
        self._frame_seq = 0
        self._last_state_pub = self._last_frame_pub = 0.0
        self._state_asked = self._frame_asked = -1e9
        self.server = None
        try:
            self.server = ThreadingHTTPServer((host, port), self._handler())
        except OSError as exc:
            log(f"[ui-feed] not publishing for the reach console: {host}:{port} unavailable ({exc})")
            return
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True, name="ui-feed").start()
        log(f"[ui-feed] publishing for the reach console on {self.url}")

    @property
    def active(self) -> bool:
        return self.server is not None

    @property
    def url(self) -> str | None:
        if self.server is None:
            return None
        host, port = self.server.server_address[:2]
        return f"http://{host}:{port}"

    # ---- simulator side

    def wants_state(self) -> bool:
        now = time.monotonic()
        return (self.active and now - self._state_asked < _READER_TIMEOUT_S
                and now - self._last_state_pub >= self._state_period)

    def wants_frame(self) -> bool:
        now = time.monotonic()
        return (self.active and self.camera and now - self._frame_asked < _READER_TIMEOUT_S
                and now - self._last_frame_pub >= self._frame_period)

    def publish_joints(self, names, positions, *, sim_time_s: float, base_height_m: float | None = None,
                       status: str | None = None) -> None:
        """Joint positions (radians / metres) by articulation name; picks out the arm, fingers and legs."""
        positions = np.asarray(positions, dtype=float).reshape(-1)
        state = {
            "source": self.source, "feed": self.id, "wall_time": time.time(), "sim_time_s": round(float(sim_time_s), 3),
            "arm_q_rad": _pick(names, positions, ARM_JOINTS),
            "finger_m": _pick(names, positions, FINGER_JOINTS),
            "legs_q_rad": _pick(names, positions, GO2_MOTOR_ORDER),
            "base_height_m": None if base_height_m is None else round(float(base_height_m), 4),
            "status": status,
        }
        with self._lock:
            self._state_seq += 1
            state["seq"] = self._state_seq
            self._state = json.dumps(state).encode()
            self._last_state_pub = time.monotonic()

    def publish_frame(self, rgb, *, sim_time_s: float) -> None:
        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"expected an HxWx3 RGB frame, got {rgb.shape}")
        headers = {"X-Width": str(rgb.shape[1]), "X-Height": str(rgb.shape[0]), "X-Sim-Time": f"{sim_time_s:.3f}",
                   "X-Feed": self.id}
        with self._lock:
            self._frame_seq += 1
            self._frame = (self._frame_seq, rgb.tobytes(), headers)
            self._last_frame_pub = time.monotonic()

    def close(self) -> None:
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None

    # ---- HTTP

    def _handler(self):
        feed = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def _send(self, status, body=b"", content_type="application/json", headers=None):
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                for key, value in (headers or {}).items():
                    self.send_header(key, value)
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path, _, query = self.path.partition("?")
                if path == "/health":
                    body = {"sim": True, "source": feed.source, "feed": feed.id, "camera": feed.camera,
                            "camera_info": feed.camera_info}
                    return self._send(HTTPStatus.OK, json.dumps(body).encode())
                if path == "/state":
                    feed._state_asked = time.monotonic()
                    with feed._lock:
                        state = feed._state
                    if state is None:
                        return self._send(HTTPStatus.NOT_FOUND, b'{"reason": "no state published yet"}')
                    return self._send(HTTPStatus.OK, state)
                if path == "/frame":
                    if not feed.camera:
                        return self._send(HTTPStatus.NOT_FOUND, b'{"reason": "this simulation has no wrist camera"}')
                    feed._frame_asked = time.monotonic()
                    after, reader_feed = 0, None
                    for part in query.split("&"):
                        key, _, value = part.partition("=")
                        if key == "after":
                            try:
                                after = int(value)
                            except ValueError:
                                pass
                        elif key == "feed":
                            reader_feed = value
                    if reader_feed != feed.id:
                        after = 0       # the reader's sequence numbers belong to another run
                    with feed._lock:
                        frame = feed._frame
                    if frame is None or frame[0] <= after:
                        return self._send(HTTPStatus.NOT_MODIFIED)
                    seq, data, headers = frame
                    return self._send(HTTPStatus.OK, data, "application/octet-stream", {**headers, "X-Seq": str(seq)})
                self._send(HTTPStatus.NOT_FOUND, b'{"reason": "unknown path"}')

        return Handler


class SimFeedClient:
    """Reads a `SimFeed`. Mirrors `D1Client`'s read methods, so the console's server uses it unchanged.

    Angles come out in servo degrees through `d1_ik.to_servo_deg`, exactly as the real arm reports them.
    They are the simulator's joint positions, not a model of the arm's 9 Hz feedback.
    """

    def __init__(self, url: str = DEFAULT_URL, timeout_s: float = 0.5):
        self.url = url.rstrip("/")
        self.timeout_s = timeout_s
        self._lock = threading.Lock()
        self._state: dict | None = None
        self._last_rx = 0.0
        self._frame_seq = 0
        self._frame_feed = ""

    def _get(self, path: str):
        """(status, headers, body); status 0 when nothing answered."""
        try:
            with urllib.request.urlopen(self.url + path, timeout=self.timeout_s) as response:
                return response.status, response.headers, response.read()
        except urllib.error.HTTPError as exc:
            return exc.code, exc.headers, b""
        except (urllib.error.URLError, OSError, ValueError):
            return 0, None, b""

    def health(self) -> dict | None:
        status, _, body = self._get("/health")
        if status != 200:
            return None
        try:
            answer = json.loads(body)
        except ValueError:
            return None
        return answer if answer.get("sim") else None

    # ---- D1Client's read interface

    def poll(self, timeout_s: float = 0.0) -> bool:
        """Fetch the latest state. True if it is newer than the last one; otherwise waits out `timeout_s`."""
        end = time.monotonic() + max(timeout_s, 0.0)
        while True:
            status, _, body = self._get("/state")
            if status == 200:
                try:
                    state = json.loads(body)
                except ValueError:
                    state = None
                if state is not None:
                    with self._lock:
                        fresh = self._state is None or (state.get("feed"), state.get("seq")) != (
                            self._state.get("feed"), self._state.get("seq"))
                        if fresh:
                            self._state = state
                            self._last_rx = time.monotonic()
                    if fresh:
                        return True
            remaining = end - time.monotonic()
            if remaining <= 0:
                return False
            time.sleep(min(0.02 if status == 200 else 0.25, remaining))

    @property
    def state(self) -> dict | None:
        with self._lock:
            return dict(self._state) if self._state is not None else None

    def get_joint_angles(self) -> list[float]:
        import d1_ik

        with self._lock:
            q = None if self._state is None else self._state.get("arm_q_rad")
        if q is None:
            raise RuntimeError("no arm state from the simulator yet")
        return [round(v, 3) for v in d1_ik.to_servo_deg(q)]

    def get_gripper_units(self):
        return None     # the simulator reports finger positions in metres; see `state["finger_m"]`

    @property
    def feedback_age_s(self) -> float:
        with self._lock:
            return float("inf") if self._state is None else time.monotonic() - self._last_rx

    def is_powered(self):
        return None

    def is_enabled(self):
        return None

    def error_status(self):
        return None

    def is_safe_to_release(self) -> bool:
        return False

    @property
    def participant(self):
        return None

    # ---- camera

    def fetch_frame(self):
        """(rgb, sim_time_s) for a frame newer than the last one fetched, or None."""
        status, headers, body = self._get(f"/frame?after={self._frame_seq}&feed={self._frame_feed}")
        if status != 200 or headers is None:
            return None
        try:
            width, height = int(headers["X-Width"]), int(headers["X-Height"])
            rgb = np.frombuffer(body, dtype=np.uint8).reshape(height, width, 3)
            self._frame_seq, self._frame_feed = int(headers["X-Seq"]), headers.get("X-Feed", "")
            return rgb, float(headers.get("X-Sim-Time", "nan"))
        except (KeyError, TypeError, ValueError):
            return None
