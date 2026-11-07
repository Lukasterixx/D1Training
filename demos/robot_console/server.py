#!/usr/bin/env python3
"""The test console: a list of policies, an E-stop, and what the running program reports. Runs ON the dog's host.

    python3 demos/robot_console/server.py             # `d1console` on the PC starts it and tunnels it to localhost:8097

It binds 127.0.0.1 only, so it is reached through the SSH tunnel, key-checked, and is not open on the WiFi. Standard
library only: the dog's host has no Flask, and this must not depend on an image.

The program is `run_press_on_dog.sh` (`press_live.py`) with `--status_port`, as a child: its stdin is the console's way
in, a datagram a tenth of a second its way out. The program reads the remote itself, so **B works with this server, the
browser and the WiFi all gone**. The button's way down, and what stands behind it:

  1. `estop` on the program's stdin: damping, LowCmd stopped, sport mode selected, on its next tick.
  2. If the status does not show it stopping in `ESCALATE_S`: SIGTERM to its container, which goes to damping.
  3. Whenever a live run ends with sport mode not back: `run_sport_on_dog.sh --ensure`. So too the button with no run.

The browser polls every 0.2 s from a worker. A live run that hears nothing for `DEADMAN_S` is E-stopped, and so is one
whose server dies (the program's stdin closes): a console that cannot stop the dog must not leave it running.

Live needs, in this server's lifetime: the remote's B seen in a dry run, and this policy's clean dry run. The word `live`
is then typed at the program's own confirmation, once the program is up and has checked the dog.
"""
from __future__ import annotations

import argparse
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time

REPO = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
PORT, STATUS_PORT = 8097, 8098
#: 3.5 s: past TCP's own recovery on the tunnel (0.2 + 0.4 + 0.8 + 1.6 s of retries). At 2.0 s the WiFi stopped live runs (F-262).
DEADMAN_S, ESCALATE_S = 3.5, 0.6
#: A gap in the page's 5 Hz heartbeat worth writing down, and how old the program's 10 Hz status may be to count as alive.
HEARTBEAT_GAP_S, STATUS_FRESH_S = 0.6, 1.0
#: A dry run has to reach this many policy steps to count (5 s), this many status reports (3 s) with the dog up. A
#: lying dog gives a policy nothing to say about a stand.
DRY_MIN_STEPS, DRY_MIN_STANDING = 250, 30
STANDING_MIN_M = 0.25
LAUNCHERS = {"press": ("demos/combiner_key/robot/run_press_on_dog.sh", "d1t-leg-press"),
             "shadow": ("demos/combiner_key/robot/run_shadow_on_dog.sh", "d1t-leg-shadow"),
             "arm": ("demos/combiner_key/robot/run_holster_on_dog.sh", "d1t-holster")}
SPORT = "demos/combiner_key/robot/run_sport_on_dog.sh"
#: The dog's sport mode, if a run never got to read it.
SPORT_NAME = "mcf"
STOPPING = ("damping", "handback", "done")
#: A whole-body run with its arm out or coming out, and that run on its own way down: see `deadman`.
ARM_OUT, ENDING = ("stand_up", "policy"), ("stand_end", "lie_end")


class Console:
    def __init__(self, repo: Path = REPO, policies_path: Path = HERE / "policies.json", status_port: int = STATUS_PORT,
                 clock=time.monotonic, launchers=LAUNCHERS, sport_cmd=None, kill_cmd=None):
        self.repo, self.clock, self.launchers, self.status_port = repo, clock, launchers, status_port
        self.sport_cmd = sport_cmd or (lambda name: [str(repo / SPORT), "--ensure", name])
        self.kill_cmd = kill_cmd or (lambda container: ["docker", "kill", "-s", "TERM", container])
        self.policies = {p["id"]: p for p in json.loads(policies_path.read_text())}
        self.lock = threading.RLock()
        self.log: deque[str] = deque(maxlen=400)
        self.proc = self.run = None
        self.status, self.status_t = None, 0.0
        self.heartbeat = clock()
        self.polls = 0
        self.b_seen = False
        self.dry_ok: set[str] = set()
        self.history: list[dict] = []
        self.estop_t = None
        self.sport_job = None
        self.sport_result = None
        self.closing = False

    # --- the running program -----------------------------------------------------------------------------------------

    def say(self, line: str) -> None:
        with self.lock:
            self.log.append(f"{time.strftime('%H:%M:%S')} {line}")

    def active(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def can_live(self, policy_id: str) -> tuple[bool, str]:
        if not self.b_seen:
            return False, "press B on the remote during a dry run first"
        if policy_id not in self.dry_ok:
            return False, "this policy needs a clean dry run of at least 5 s, with the dog standing, first"
        return True, ""

    def start(self, policy_id: str, mode: str, word: str = "") -> tuple[bool, str]:
        with self.lock:
            policy = self.policies.get(policy_id)
            if policy is None or mode not in policy["modes"]:
                return False, "no such policy or mode"
            if self.active():
                return False, "a run is going"
            if self.sport_job is not None:
                return False, "sport mode is being given back"
            if mode != "dry":
                ok, why = self.can_live(policy_id)
                if not ok:
                    return False, why
            confirm = policy.get("confirm", {}).get(mode)
            # `quick_start`: a policy proven on the dog takes the page's OK for both words. The Live lock above still stands.
            quick = bool(policy.get("quick_start")) and mode != "dry"
            if confirm and not quick and word.strip().lower() != confirm:
                return False, f"type {confirm} to start this"
            launcher, container = self.launchers[policy["kind"]]
            args = [*(["--policy", policy["file"]] if "file" in policy else []), *policy["modes"][mode]]
            if policy["kind"] == "press":
                args += ["--status_port", str(self.status_port)]
            self.proc = subprocess.Popen([launcher if launcher.startswith("/") else str(self.repo / launcher), *args],
                                         cwd=self.repo, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
            self.run = {"policy": policy_id, "mode": mode, "live": mode != "dry", "container": container,
                        "kind": policy["kind"], "record": None, "started": self.clock(), "estop": None, "standing": 0,
                        "whole_body": "--reach_arm_send" in policy["modes"][mode], "ended_gently": False,
                        "auto_live": quick, "live_sent": False}
            self.status, self.estop_t, self.sport_result = None, None, None
            self.heartbeat = self.clock()
            self.say(f"START {policy['label']} [{mode}]")
            if confirm and policy["kind"] == "press":    # press_live asks on its stdin before it does anything
                self.write(confirm)
            threading.Thread(target=self._read, args=(self.proc, self.run), daemon=True).start()
            return True, ""

    def _read(self, proc, run) -> None:
        for line in proc.stdout:
            line = line.rstrip()
            self.say(line)
            if "record: " in line:
                run["record"] = line.split("record: ", 1)[1].strip()
            if "E-STOP (" in line or "stopped: operator" in line or "stopped on the way back" in line:
                run["heard"] = True                      # its own word: it has heard
            # From the log as well as the status: B stops a run before any status can carry it (2026-11-06).
            if "the remote's B" in line:
                self.b_seen = True
            if run["kind"] == "arm" and "back to the start: " in line:   # folding after a stop: stoppable, watched, again
                with self.lock:
                    self.estop_t, run["heard"] = None, False
        proc.wait()
        self._ended(proc, run)

    def _ended(self, proc, run) -> None:
        summary = self._summary(run)
        with self.lock:
            run["summary"] = summary
            self.say(f"END {run['policy']} [{run['mode']}], exit {proc.returncode}: "
                     f"{(summary or {}).get('abort_reason') or (summary or {}).get('end_reason') or 'no summary'}")
            self.history.append({k: run.get(k) for k in ("policy", "mode", "record", "estop")}
                                | {"abort": (summary or {}).get("abort_reason"), "end": (summary or {}).get("end_reason")})
            why = (summary or {}).get("abort_reason") or ""
            clean = why == "" or "the remote's B" in why
            if run["kind"] == "arm":
                # Finished, or stopped by the operator, and the real arm folded with its report fresh. The last dry run
                # decides: an arm that was folded before is not one that is folded now.
                if run["mode"] == "dry":
                    (self.dry_ok.add if summary is not None and summary["arm_ok"] else self.dry_ok.discard)(run["policy"])
                return                                   # the legs were never touched: sport mode is not this run's
            if (run["mode"] == "dry" and clean and summary is not None and run["standing"] >= DRY_MIN_STANDING
                    and (summary.get("policy_steps") or 0) >= DRY_MIN_STEPS):
                self.dry_ok.add(run["policy"])
            # A live run that cannot show it gave the legs back: ask, and select only if they are still released.
            if run["live"] and (summary is None or summary.get("left_released") or not summary.get("sport_mode_back", True)):
                self.ensure_sport((summary or {}).get("sport_mode") or SPORT_NAME)

    def _summary(self, run) -> dict | None:
        if not run.get("record"):
            return None
        path = str(run["record"]).replace("/root/D1Training/", f"{self.repo}/")
        try:
            if run["kind"] == "arm":
                record = json.loads((Path(path) / "run.json").read_text())
                status, why = record.get("status"), record.get("why") or ""
                return {"abort_reason": None if status == "finished" else f"{status}: {why}", "end_reason": status,
                        "arm_ok": ((status == "finished" or (status == "stopped" and why.startswith("operator: a")))
                                   and str(record.get("real_arm_start_check", "")).startswith("ok"))}
            return json.loads((Path(path) / "summary.json").read_text())
        except (OSError, ValueError):
            return None

    def write(self, line: str) -> bool:
        proc = self.proc
        if proc is None or proc.poll() is not None or proc.stdin is None:
            return False
        try:
            proc.stdin.write(line + "\n")
            proc.stdin.flush()
            return True
        except (OSError, ValueError):
            return False

    # --- stopping ----------------------------------------------------------------------------------------------------

    def estop(self, source: str) -> dict:
        """Everything the button does. Safe to call any number of times."""
        with self.lock:
            active, run = self.active(), self.run
            first = self.estop_t is None
            if active and first:
                self.estop_t = self.clock()
                run["estop"] = source
                self.say(f"E-STOP from {source}")
            if active and run["kind"] in ("press", "arm"):
                # An arm run's stop holds the arm, and the program stays up to fold it: no sport mode, the legs are not its.
                self.write("estop" if run["kind"] == "press" else "a")
                if first:
                    threading.Thread(target=self._escalate, args=(self.proc, run), daemon=True).start()
                return {"stopping": True}
            if active:                                   # a shadow run sends nothing: just end it
                self._kill(run)
                return {"stopping": True}
            self.say(f"E-STOP from {source}, no run: checking sport mode")
            self.ensure_sport(SPORT_NAME)
            return {"stopping": False}

    def _escalate(self, proc, run) -> None:
        time.sleep(ESCALATE_S)
        with self.lock:
            # The program's own line, not only its status: a dry run ends without another status.
            stopping = run.get("heard") or (
                (self.status or {}).get("mode") in STOPPING and self.clock() - self.status_t < ESCALATE_S)
            if proc.poll() is None and not stopping:
                self.say("no sign of it stopping: SIGTERM to its container")
                self._kill(run)

    def _kill(self, run) -> None:
        subprocess.run(self.kill_cmd(run["container"]), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5,
                       check=False)

    def ensure_sport(self, name: str) -> None:
        with self.lock:
            if self.sport_job is not None or self.active():
                return
            self.sport_job = threading.Thread(target=self._sport, args=(name,), daemon=True)
            self.sport_job.start()

    def _sport(self, name: str) -> None:
        try:
            out = subprocess.run(self.sport_cmd(name), cwd=self.repo, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, timeout=25, check=False)
            ok, text = out.returncode == 0, out.stdout.strip().splitlines()
        except (OSError, subprocess.TimeoutExpired) as error:
            ok, text = False, [repr(error)]
        with self.lock:
            self.sport_job = None
            self.sport_result = {"ok": ok, "detail": " / ".join(text[-3:])}
            self.say(("sport mode: " if ok else "SPORT MODE NOT CONFIRMED: use the remote or app. ")
                     + self.sport_result["detail"])

    def deadman(self) -> None:
        """The browser gone, the tunnel down: a live run must not carry on unwatched.

        A whole-body run ends as End gently ends it: the link is not the dog, the run is on the dog and B still works, and the
        E-stop would drop a healthy dog with the arm out (F-262). A run already going down is left to finish its fold. A
        program that has stopped reporting gets the E-stop, as does everything else.
        """
        with self.lock:
            lost = self.clock() - self.heartbeat
            mode = (self.status or {}).get("mode")
            if not self.active() or not self.run["live"] or lost <= DEADMAN_S or self.estop_t is not None or mode in STOPPING:
                return
            alive = self.status is not None and self.clock() - self.status_t < STATUS_FRESH_S
            if self.run["whole_body"] and alive and mode in ARM_OUT + ENDING:
                if mode in ARM_OUT and not self.run["ended_gently"]:
                    self.run["ended_gently"] = True
                    self.say(f"no heartbeat from the browser for {lost:.1f} s: ending gently, the arm folds. B still stops it")
                    self.write("q")
                return
            self.say(f"no heartbeat from the browser for {lost:.1f} s")
            self.estop("the link lost")

    def on_status(self, data: bytes) -> None:
        try:
            status = json.loads(data)
        except ValueError:
            return
        with self.lock:
            self.status, self.status_t = status, self.clock()
            if "B" in status.get("keys", ()):
                self.b_seen = True
            if self.run is not None and status.get("mode") == "policy" and status.get("base_m", 0.0) >= STANDING_MIN_M:
                self.run["standing"] += 1
            # The program has checked the dog and asks for `live`: only then, because it clears anything that came before it asked.
            if (self.run is not None and self.run["auto_live"] and status.get("confirming") and not self.run["live_sent"]
                    and self.estop_t is None and self.active()):
                self.run["live_sent"] = True
                self.say("the page's OK stands for `live`")
                self.write("live")

    def state(self, heartbeat: bool = False) -> dict:
        with self.lock:
            if heartbeat:
                gap = self.clock() - self.heartbeat
                if gap > HEARTBEAT_GAP_S and self.active():
                    self.say(f"heartbeat back after {gap:.1f} s")
                self.heartbeat = self.clock()
                self.polls += 1
            active, run = self.active(), self.run
            policies = []
            for p in self.policies.values():
                ok, why = self.can_live(p["id"])
                policies.append({k: p[k] for k in ("id", "label", "status", "note", "kind")}
                                | {"modes": list(p["modes"]), "confirm": p.get("confirm", {}), "quick_start": bool(p.get("quick_start")), "can_live": ok, "why_not": why,
                                   "dry_ok": p["id"] in self.dry_ok})
            return {"policies": policies, "active": active, "b_seen": self.b_seen, "log": list(self.log)[-160:],
                    "polls": self.polls,
                    "run": None if run is None else {k: run.get(k) for k in ("policy", "mode", "live", "kind", "record", "estop")},
                    "status": self.status, "status_age": round(self.clock() - self.status_t, 2) if self.status else None,
                    "estop": self.estop_t is not None, "sport_busy": self.sport_job is not None,
                    "sport_result": self.sport_result, "history": self.history[-8:]}

    def close(self) -> None:
        """The server is going: stop a run, and let its program hand back."""
        self.closing = True
        if self.active():
            self.estop("the console closing")
            try:
                self.proc.wait(timeout=12)
            except subprocess.TimeoutExpired:
                pass


def listen_status(console: Console, port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", port))
    while True:
        console.on_status(sock.recv(4096))


def watch(console: Console) -> None:
    while True:
        console.deadman()
        time.sleep(0.1)


class Handler(BaseHTTPRequestHandler):
    console: Console = None
    server_version = "RobotConsole/1"

    def log_message(self, format, *args):
        pass

    def _send(self, status: int, body: bytes, kind: str = "application/json") -> None:
        self.send_response(status)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, value, status: int = 200) -> None:
        self._send(status, json.dumps(value).encode())

    def _ours(self) -> bool:
        """Only this page: a local port is open to any web page's POST unless the request proves otherwise. The custom
        header forces a CORS preflight, which this server never answers; the Host check stops DNS rebinding."""
        host = self.headers.get("Host", "").split(":")[0]
        return self.headers.get("X-Console") == "1" and host in ("localhost", "127.0.0.1")

    def do_GET(self):
        if self.path == "/":
            self._send(200, (HERE / "static" / "index.html").read_bytes(), "text/html; charset=utf-8")
        elif self.path.startswith("/api/state"):
            if not self._ours():
                return self._json({"error": "not this page"}, HTTPStatus.FORBIDDEN)
            self._json(self.console.state(heartbeat=True))
        else:
            self._send(404, b"{}")

    def do_POST(self):
        if not self._ours():
            return self._json({"error": "not this page"}, HTTPStatus.FORBIDDEN)
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            body = {}
        c = self.console
        if self.path == "/api/estop":
            self._json(c.estop(str(body.get("source") or "the button")))
        elif self.path == "/api/start":
            ok, why = c.start(str(body.get("policy")), str(body.get("mode")), str(body.get("word") or ""))
            self._json({"ok": ok, "error": why}, 200 if ok else HTTPStatus.CONFLICT)
        elif self.path == "/api/line":
            line = str(body.get("line", ""))
            # An arm run's `q` quits with the arm out, so it has no `q`; its `r` folds it after a stop.
            arm = c.run is not None and c.run["kind"] == "arm"
            if line not in (("", "r") if arm else ("", "q", "live")):
                return self._json({"error": "no such line"}, HTTPStatus.BAD_REQUEST)
            self._json({"ok": c.write(line)})
        elif self.path == "/api/sport":
            c.ensure_sport(SPORT_NAME)
            self._json({"ok": True})
        else:
            self._json({"error": "no such path"}, 404)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--status_port", type=int, default=STATUS_PORT)
    args = parser.parse_args()
    console = Console(status_port=args.status_port)
    Handler.console = console
    threading.Thread(target=listen_status, args=(console, args.status_port), daemon=True).start()
    threading.Thread(target=watch, args=(console,), daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)

    def leave(*_):
        threading.Thread(target=lambda: (console.close(), server.shutdown()), daemon=True).start()

    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(sig, leave)
    print(f"[console] http://127.0.0.1:{args.port}", flush=True)
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
