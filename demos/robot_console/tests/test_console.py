"""The console's stop path and live gates, against a fake program (a shell script): no dog, no docker."""
from __future__ import annotations

import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from demos.robot_console import server as console_server  # noqa: E402

#: Echoes each stdin line to a file, and on `estop` or EOF writes a record the way press_live.py does, then exits.
FAKE = """#!/usr/bin/env python3
import json, os, sys, time
d = os.environ.get("FAKE_DIR", sys.argv[0] + ".d")
os.makedirs(d, exist_ok=True)
steps = 300
print("started", " ".join(sys.argv[1:]), flush=True)
why = None
for line in sys.stdin:
    open(d + "/lines", "a").write(line)
    if line.strip() == "estop":
        why = "E-stop: the console"
        break
    if line.strip() == "q":
        break
    if line.strip() == "b":
        print("[press] E-STOP (the remote's B)", flush=True)
        why = "E-stop: the remote's B"
        break
else:
    why = "E-stop: the console's link closed"
rec = d + "/rec"
os.makedirs(rec, exist_ok=True)
json.dump({"abort_reason": why, "end_reason": None if why else "ended", "policy_steps": steps, "left_released": False,
           "sport_mode": "mcf"}, open(rec + "/summary.json", "w"))
print("[press] record: " + rec, flush=True)
"""

#: `holster_dog.py --console` as the console sees it: `a` stops and stays up, `r` folds and ends. `b`, `bad` and `end` are
#: this fake's own: the remote's B in its log, the real arm's start check failing, the end of a run.
FAKE_ARM = """#!/usr/bin/env python3
import json, os, sys
d = sys.argv[0] + ".d"
os.makedirs(d, exist_ok=True)
print("started", " ".join(sys.argv[1:]), flush=True)
status, why, check = "finished", "", "ok: folded, its report fresh"
for line in sys.stdin:
    line = line.strip()
    open(d + "/lines", "a").write(line + "\\n")
    if line == "a":
        status, why = "stopped", "operator: a"
        print("stopped: operator: a; the arm is held where it is", flush=True)
    elif line == "b":
        print("[holster_dog] the remote's B", flush=True)
    elif line == "bad":
        check = "not folded"
    elif line == "r":
        print("back to the start: home -> tuck -> fold", flush=True)
        status = "stopped, then back at the start"
        break
    elif line == "end":
        break
rec = d + "/rec"
os.makedirs(rec, exist_ok=True)
json.dump({"status": status, "why": why, "real_arm_start_check": check}, open(rec + "/run.json", "w"))
print("[run_holster_on_dog] record: " + rec, flush=True)
"""


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        fake = self.dir / "fake.py"
        fake.write_text(FAKE)
        fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
        arm = self.dir / "fake_arm.py"
        arm.write_text(FAKE_ARM)
        arm.chmod(arm.stat().st_mode | stat.S_IEXEC)
        self.now = [100.0]
        self.sport_calls, self.kills = [], []
        sport = self.dir / "sport.py"
        sport.write_text("#!/usr/bin/env python3\nprint('CheckMode: mcf')\n")
        sport.chmod(sport.stat().st_mode | stat.S_IEXEC)

        def sport_cmd(name):
            self.sport_calls.append(name)
            return [str(sport)]

        def kill_cmd(container):
            self.kills.append(container)
            return ["true"]

        self.c = console_server.Console(repo=REPO, clock=lambda: self.now[0], sport_cmd=sport_cmd, kill_cmd=kill_cmd,
                                        launchers={"press": (str(fake), "fake-press"), "shadow": (str(fake), "fake-shadow"),
                                                   "arm": (str(arm), "fake-arm")})

    def tearDown(self):
        if self.c.active():
            self.c.proc.kill()
        self.tmp.cleanup()

    def wait(self, what, s=5.0):
        end = time.time() + s
        while time.time() < end:
            if what():
                return
            time.sleep(0.01)
        self.fail("timed out; log: " + "\n".join(self.c.log))

    def run_ended(self):
        return self.c.proc is not None and self.c.proc.poll() is None and False or (
            self.c.run is not None and "summary" in self.c.run)

    def dry(self, policy="reach_delay", b=True, standing=True):
        ok, why = self.c.start(policy, "dry")
        self.assertTrue(ok, why)
        self.wait(lambda: self.c.active())
        for _ in range(console_server.DRY_MIN_STANDING if standing else 1):
            self.c.on_status(json.dumps({"mode": "policy", "base_m": 0.33 if standing else 0.07,
                                         "keys": ["B"] if b else []}).encode())
        self.c.write("q")
        self.wait(self.run_ended)


class Gates(Base):
    def test_live_is_locked_until_b_and_a_clean_dry_run(self):
        ok, why = self.c.start("reach_delay", "live_stand")
        self.assertFalse(ok)
        self.assertIn("press B", why)
        self.dry(b=False)
        self.assertFalse(self.c.start("reach_delay", "live_stand")[0])       # B still unseen
        self.dry(b=True)
        self.assertTrue(self.c.start("reach_delay", "live_stand")[0])

    def test_b_ending_a_dry_run_before_any_status_carries_it_still_counts(self):
        self.assertTrue(self.c.start("reach_66200", "dry")[0])
        self.wait(lambda: self.c.active())
        for _ in range(console_server.DRY_MIN_STANDING):
            self.c.on_status(json.dumps({"mode": "policy", "base_m": 0.33, "keys": []}).encode())
        self.assertFalse(self.c.b_seen)
        self.c.write("b")
        self.wait(self.run_ended)
        self.assertTrue(self.c.can_live("reach_66200")[0])

    def test_the_console_stopping_a_run_is_not_b(self):
        self.assertTrue(self.c.start("reach_66200", "dry")[0])
        self.wait(lambda: self.c.active())
        self.c.estop("the button")
        self.wait(self.run_ended)
        self.assertFalse(self.c.b_seen)

    def test_a_dry_run_on_a_lying_dog_does_not_count(self):
        self.dry("press_r4", standing=False)
        self.assertNotIn("press_r4", self.c.dry_ok)
        self.assertFalse(self.c.start("press_r4", "live_stand")[0])

    def test_a_dry_run_counts_for_its_own_policy_only(self):
        self.dry("reach_delay")
        ok, why = self.c.start("press_r4", "live_stand")
        self.assertFalse(ok)
        self.assertIn("dry run", why)

    def test_a_rejected_policy_has_no_live_mode(self):
        self.dry("reach_rate")
        self.assertFalse(self.c.start("reach_rate", "live_stand")[0])

    def test_two_runs_at_once_are_refused(self):
        self.c.start("reach_delay", "dry")
        self.assertFalse(self.c.start("press_r4", "dry")[0])

    def test_every_policy_file_exists_and_only_the_press_kind_goes_live(self):
        for p in json.loads((REPO / "demos/robot_console/policies.json").read_text()):
            self.assertTrue(p["kind"] == "arm" or (REPO / p["file"]).exists(), p.get("file"))
            live = [m for m in p["modes"] if m != "dry"]
            if p["kind"] == "shadow":
                self.assertEqual(live, [])
            for m in live:
                self.assertIn("--live", p["modes"][m])
            self.assertNotIn("--live", p["modes"]["dry"])
            self.assertNotIn("measured", " ".join(sum(p["modes"].values(), [])))

    def test_whatever_moves_the_arm_needs_its_word(self):
        for p in json.loads((REPO / "demos/robot_console/policies.json").read_text()):
            for mode, args in p["modes"].items():
                if p["kind"] == "arm" and "--live" in args or "--reach_arm_send" in args:
                    self.assertEqual(p["confirm"][mode], "arm")

    def test_the_whole_body_needs_the_word_and_passes_it_on(self):
        self.dry("reach_66200")
        ok, why = self.c.start("reach_66200", "live_whole", "live")
        self.assertFalse(ok)
        self.assertIn("arm", why)
        self.assertFalse(self.c.active())
        self.assertTrue(self.c.start("reach_66200", "live_whole", "arm")[0])
        lines = self.dir / "fake.py.d" / "lines"
        self.wait(lines.exists)
        self.wait(lambda: "arm\n" in lines.read_text())
        self.c.write("q")
        self.wait(self.run_ended)


class ArmRuns(Base):
    def lines(self):
        path = self.dir / "fake_arm.py.d" / "lines"
        return path.read_text().split("\n") if path.exists() else []

    def dry_arm(self, *lines):
        self.assertTrue(self.c.start("arm_zero", "dry")[0])
        self.wait(lambda: self.c.active())
        for line in (*lines, "end"):
            self.c.write(line)
        self.wait(self.run_ended)

    def live_arm(self):
        self.dry_arm("b")
        self.assertTrue(self.c.start("arm_zero", "live_zero", "arm")[0])
        self.wait(lambda: self.c.active())

    def test_it_has_no_policy_argument(self):
        self.dry_arm()
        self.assertTrue(any("started --zero_test --console" in line for line in self.c.log))

    def test_live_is_locked_until_b_and_a_dry_run_with_the_arm_folded(self):
        self.assertFalse(self.c.start("arm_zero", "live_zero", "arm")[0])
        self.dry_arm()
        self.assertFalse(self.c.can_live("arm_zero")[0])          # no B yet
        self.dry_arm("b")
        self.assertTrue(self.c.can_live("arm_zero")[0])
        self.dry_arm("bad")
        self.assertFalse(self.c.can_live("arm_zero")[0])          # the arm's start check failed: the last dry run decides

    def test_stopped_by_the_operator_in_a_dry_run_counts(self):
        self.dry_arm("b", "a")
        self.assertTrue(self.c.can_live("arm_zero")[0])

    def test_live_needs_the_word(self):
        self.dry_arm("b")
        self.assertFalse(self.c.start("arm_zero", "live_zero")[0])
        self.assertFalse(self.c.active())

    def test_the_button_holds_the_arm_and_the_program_stays_up_to_fold_it(self):
        self.live_arm()
        self.c.estop("the button")
        self.wait(lambda: self.c.run.get("heard"))
        self.now[0] += 1.0
        time.sleep(console_server.ESCALATE_S + 0.2)
        self.assertEqual(self.kills, [])
        self.assertIn("a", self.lines())
        self.assertTrue(self.c.active())
        self.c.write("r")
        self.wait(self.run_ended)
        self.assertIn("r", self.lines())

    def test_it_never_touches_sport_mode(self):
        self.live_arm()
        self.c.estop("the button")
        self.wait(lambda: self.c.run.get("heard"))
        self.c.write("end")
        self.wait(self.run_ended)
        time.sleep(0.1)
        self.assertEqual(self.sport_calls, [])

    def test_no_sign_of_stopping_escalates(self):
        self.live_arm()
        self.c.write = lambda line: True                           # the program never gets the line
        self.c.estop("the button")
        self.wait(lambda: self.kills == ["fake-arm"])
        del self.c.write
        self.c.write("end")
        self.wait(self.run_ended)

    def test_the_link_lost_holds_the_arm(self):
        self.live_arm()
        self.now[0] += console_server.DEADMAN_S + 0.5
        self.c.deadman()
        self.wait(lambda: "a" in self.lines())
        self.assertEqual(self.sport_calls, [])
        self.c.write("end")
        self.wait(self.run_ended)

    def test_folding_after_a_stop_can_be_stopped_and_watched_again(self):
        self.live_arm()
        self.c.estop("the button")
        self.wait(lambda: self.c.run.get("heard"))
        run = self.c.run
        self.c.write("r")
        self.wait(lambda: run.get("summary") is not None)
        self.assertIsNone(self.c.estop_t)


class Stops(Base):
    def live(self):
        self.dry()
        self.assertTrue(self.c.start("reach_delay", "live_stand")[0])
        self.wait(lambda: self.c.active())

    def test_the_button_reaches_the_program(self):
        self.live()
        self.assertTrue(self.c.estop("the button")["stopping"])
        self.wait(self.run_ended)
        self.assertIn("E-stop", self.c.run["summary"]["abort_reason"])
        self.assertEqual(self.c.run["estop"], "the button")

    def test_it_can_be_pressed_again_and_again(self):
        self.live()
        for _ in range(5):
            self.c.estop("the button")
        self.wait(self.run_ended)

    def test_no_sign_of_stopping_escalates_to_sigterm(self):
        self.live()
        console_server.ESCALATE_S, saved = 0.05, console_server.ESCALATE_S
        self.addCleanup(setattr, console_server, "ESCALATE_S", saved)
        self.c.write = lambda line: True                                # the program never hears it
        self.c.estop("the button")
        self.wait(lambda: self.kills == ["fake-press"])
        del self.c.write

    def test_a_program_that_said_it_heard_is_not_escalated(self):
        self.live()
        console_server.ESCALATE_S, saved = 0.05, console_server.ESCALATE_S
        self.addCleanup(setattr, console_server, "ESCALATE_S", saved)
        self.c.run["heard"] = True
        self.c.write = lambda line: True
        self.c.estop("the button")
        time.sleep(0.2)
        self.assertEqual(self.kills, [])
        del self.c.write

    def test_the_button_with_no_run_checks_sport_mode(self):
        self.assertFalse(self.c.estop("the button")["stopping"])
        self.wait(lambda: self.c.sport_result is not None)
        self.assertEqual(self.sport_calls, ["mcf"])

    def test_a_live_run_that_left_no_record_gets_sport_mode_checked(self):
        self.live()
        self.c.proc.kill()
        self.wait(lambda: self.c.sport_result is not None)
        self.assertEqual(self.sport_calls, ["mcf"])

    def test_the_link_lost_stops_a_live_run(self):
        self.live()
        self.c.state(heartbeat=True)
        self.now[0] += console_server.DEADMAN_S - 0.5
        self.c.deadman()
        self.assertFalse(self.c.estop_t)
        self.now[0] += 1.0
        self.c.deadman()
        self.wait(self.run_ended)
        self.assertEqual(self.c.run["estop"], "the link lost")

    def whole_body(self, mode):
        self.dry("reach_66600")
        self.assertTrue(self.c.start("reach_66600", "live_far_low", "arm")[0])
        self.wait(lambda: self.c.active())
        self.report(mode)

    def report(self, mode):
        self.c.on_status(json.dumps({"mode": mode}).encode())

    def lose_the_link(self, mode):
        self.now[0] += console_server.DEADMAN_S + 0.5
        self.report(mode)
        self.c.deadman()

    def lines(self):
        written = self.dir / "fake.py.d" / "lines"
        return written.read_text().split() if written.exists() else []

    def test_the_link_lost_ends_a_whole_body_run_gently(self):
        for mode in console_server.ARM_OUT:
            with self.subTest(mode):
                self.whole_body(mode)
                self.lose_the_link(mode)
                self.assertIsNone(self.c.estop_t)
                self.assertTrue(self.c.run["ended_gently"])
                self.wait(lambda: self.lines()[-1:] == ["q"])
                self.wait(self.run_ended)

    def test_the_link_lost_asks_once(self):
        self.whole_body("policy")
        self.c.write = lambda line: self.asked.append(line)
        self.asked = []
        self.lose_the_link("policy")
        self.lose_the_link("policy")
        self.assertEqual(self.asked, ["q"])

    def test_the_link_lost_leaves_a_fold_to_finish(self):
        for mode in console_server.ENDING:
            with self.subTest(mode):
                self.whole_body(mode)
                self.lose_the_link(mode)
                self.assertIsNone(self.c.estop_t)
                self.assertFalse(self.c.run["ended_gently"])
                self.c.write("q")
                self.wait(self.run_ended)

    def test_a_whole_body_program_that_stopped_reporting_is_e_stopped(self):
        self.whole_body("policy")
        self.now[0] += console_server.DEADMAN_S + 0.5
        self.c.deadman()
        self.wait(self.run_ended)
        self.assertEqual(self.c.run["estop"], "the link lost")

    def test_a_whole_body_program_stopped_while_folding_is_e_stopped(self):
        self.whole_body("stand_end")
        self.now[0] += console_server.DEADMAN_S + 0.5
        self.c.deadman()
        self.wait(self.run_ended)
        self.assertEqual(self.c.run["estop"], "the link lost")

    def test_before_anything_is_out_the_link_lost_is_the_e_stop(self):
        self.whole_body("lie_down")
        self.lose_the_link("lie_down")
        self.wait(self.run_ended)
        self.assertEqual(self.c.run["estop"], "the link lost")

    def test_a_run_without_the_arm_is_still_e_stopped_with_its_program_alive(self):
        self.live()
        self.lose_the_link("policy")
        self.wait(self.run_ended)
        self.assertEqual(self.c.run["estop"], "the link lost")

    def quick(self):
        self.dry("reach_66600")
        self.assertTrue(self.c.start("reach_66600", "live_far_low")[0])
        self.wait(lambda: self.c.active())
        self.wait(lambda: "arm" in self.lines())

    def test_quick_start_needs_no_word_and_still_tells_the_program(self):
        self.quick()
        self.assertEqual(self.lines().count("live"), 0)                   # not before it asks: it would clear that
        self.c.on_status(json.dumps({"mode": "preflight", "confirming": True}).encode())
        self.wait(lambda: "live" in self.lines())
        self.c.write("q")
        self.wait(self.run_ended)

    def test_quick_start_says_live_once(self):
        self.quick()
        sent = []
        self.c.write = lambda line: sent.append(line)
        for _ in range(5):
            self.c.on_status(json.dumps({"mode": "preflight", "confirming": True}).encode())
        self.assertEqual(sent, ["live"])

    def sent_by(self, status):
        """What the console writes to the program for one status, seen at the write: the program itself may be too slow, or gone."""
        sent = []
        self.c.write, kept = (lambda line: sent.append(line)), self.c.write
        self.c.on_status(json.dumps(status).encode())
        self.c.write = kept
        return sent

    def test_quick_start_waits_until_the_program_asks(self):
        self.quick()
        self.assertEqual(self.sent_by({"mode": "preflight"}), [])
        self.assertEqual(self.sent_by({"mode": "preflight", "confirming": False}), [])
        self.assertEqual(self.sent_by({"mode": "preflight", "confirming": True}), ["live"])

    def test_an_e_stop_while_it_asks_is_not_answered_with_live(self):
        self.quick()
        self.c.estop("the button")
        self.c.active = lambda: True                                       # still up: its own stop can take a moment
        self.assertEqual(self.sent_by({"mode": "preflight", "confirming": True}), [])

    def test_quick_start_does_not_skip_the_live_lock(self):
        ok, why = self.c.start("reach_66600", "live_far_low")
        self.assertFalse(ok)
        self.assertFalse(self.c.active())

    def test_a_dry_run_is_never_answered(self):
        self.c.start("reach_66600", "dry")
        self.wait(lambda: self.c.active())
        self.assertEqual(self.sent_by({"mode": "preflight", "confirming": True}), [])
        self.c.write("q")
        self.wait(self.run_ended)

    def test_only_the_proven_policy_has_it(self):
        self.assertEqual({p["id"] for p in self.c.policies.values() if p.get("quick_start")}, {"reach_66600"})
        self.dry("reach_gentle_67199")
        self.assertFalse(self.c.start("reach_gentle_67199", "live_whole")[0])

    def test_a_gap_in_the_heartbeat_is_written_down(self):
        self.live()
        self.now[0] += 1.2
        self.c.state(heartbeat=True)
        self.assertIn("heartbeat back after 1.2 s", "\n".join(self.c.log))

    def test_the_link_lost_leaves_a_dry_run_alone(self):
        self.c.start("reach_delay", "dry")
        self.wait(lambda: self.c.active())
        self.now[0] += 60.0
        self.c.deadman()
        self.assertIsNone(self.c.estop_t)

    def test_the_server_going_stops_a_live_run(self):
        self.live()
        self.c.close()
        self.assertIn("E-stop", self.c.run.get("summary", {}).get("abort_reason", "E-stop"))
        self.assertEqual(self.c.run["estop"], "the console closing")


class Http(Base):
    def setUp(self):
        super().setUp()
        console_server.Handler.console = self.c
        self.server = console_server.ThreadingHTTPServer(("127.0.0.1", 0), console_server.Handler)
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.shutdown)

    def post(self, path, body=None, headers=None):
        req = urllib.request.Request(self.url + path, json.dumps(body or {}).encode(), method="POST",
                                     headers={"X-Console": "1"} if headers is None else headers)
        return urllib.request.urlopen(req, timeout=3)

    def test_another_page_cannot_start_a_run(self):
        with self.assertRaises(urllib.error.HTTPError) as caught:
            self.post("/api/start", {"policy": "reach_delay", "mode": "dry"}, headers={})
        self.assertEqual(caught.exception.code, 403)
        self.assertFalse(self.c.active())

    def test_only_the_program_lines_it_expects_get_through(self):
        with self.assertRaises(urllib.error.HTTPError):
            self.post("/api/line", {"line": "a"})

    def test_an_arm_run_has_no_q_and_only_it_has_r(self):
        with self.assertRaises(urllib.error.HTTPError):
            self.post("/api/line", {"line": "r"})
        self.assertTrue(self.c.start("arm_zero", "dry")[0])
        self.wait(lambda: self.c.active())
        with self.assertRaises(urllib.error.HTTPError):            # it would quit with the arm out
            self.post("/api/line", {"line": "q"})
        self.post("/api/line", {"line": "r"})
        self.wait(self.run_ended)

    def test_the_word_reaches_the_gate(self):
        self.assertTrue(self.c.start("arm_zero", "dry")[0])
        self.wait(lambda: self.c.active())
        self.c.write("b")
        self.c.write("end")
        self.wait(self.run_ended)
        with self.assertRaises(urllib.error.HTTPError):
            self.post("/api/start", {"policy": "arm_zero", "mode": "live_zero", "word": "nope"})
        self.post("/api/start", {"policy": "arm_zero", "mode": "live_zero", "word": "arm"})
        self.wait(lambda: self.c.active())
        self.c.write("end")
        self.wait(self.run_ended)

    def test_the_page_and_state_are_served(self):
        self.assertIn(b"E-STOP", urllib.request.urlopen(self.url + "/", timeout=3).read())
        state = json.load(urllib.request.urlopen(urllib.request.Request(self.url + "/api/state",
                                                                        headers={"X-Console": "1"}), timeout=3))
        self.assertEqual(len(state["policies"]), 10)


class FromAPcWithoutTheHelpers(unittest.TestCase):
    """`run_on_pc.sh --dog`, with `ssh` and `rsync` replaced by scripts that write down how they were called."""

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.dir = Path(tmp.name)
        for name in ("ssh", "rsync"):
            tool = self.dir / name
            tool.write_text(f'#!/bin/sh\nprintf "%s\\n" "{name} $*" >> "{self.dir}/calls"\nexit "${{FAKE_{name.upper().replace("-", "_")}_EXIT:-0}}"\n')
            tool.chmod(tool.stat().st_mode | stat.S_IEXEC)

    def run_it(self, *args, **env):
        result = subprocess.run(
            [str(REPO / "demos/robot_console/run_on_pc.sh"), "--no_browser", *args],
            env={**os.environ, "PATH": f"{self.dir}:{os.environ['PATH']}", **env},
            capture_output=True, text=True, timeout=20,
        )
        calls = self.dir / "calls"
        return result, calls.read_text().splitlines() if calls.exists() else []

    def test_a_preview_sends_nothing_and_starts_nothing(self):
        result, calls = self.run_it("--dog", "unitree@dog", "--preview")
        self.assertEqual(result.returncode, 0)
        self.assertEqual(len(calls), 1)
        self.assertTrue(calls[0].startswith("rsync "))
        self.assertIn("--dry-run", calls[0])

    def test_it_copies_then_starts_the_console_with_the_host_key_checked(self):
        result, calls = self.run_it("--dog", "unitree@dog")
        self.assertEqual(result.returncode, 0)
        self.assertEqual([c.split()[0] for c in calls], ["rsync", "ssh"])
        self.assertNotIn("--dry-run", calls[0])
        self.assertIn("ssh -o StrictHostKeyChecking=yes", calls[0])
        self.assertIn("-o StrictHostKeyChecking=yes", calls[1])
        self.assertIn("-L 8097:127.0.0.1:8097 unitree@dog", calls[1])
        self.assertIn("-o ControlMaster=no", calls[1])

    def test_the_dogs_logs_and_results_are_not_touched_by_the_delete(self):
        _, calls = self.run_it("--dog", "unitree@dog")
        for kept in ("/logs/", "/results/", "/generated/", "/.git/", "/checkpoints/*"):
            self.assertIn(f"--exclude {kept}", calls[0])
        self.assertNotIn("--delete-excluded", calls[0])

    def test_no_console_starts_when_the_copy_failed(self):
        result, calls = self.run_it("--dog", "unitree@dog", FAKE_RSYNC_EXIT="23")
        self.assertEqual(result.returncode, 1)
        self.assertEqual([c.split()[0] for c in calls], ["rsync"])

    def test_no_update_skips_the_copy(self):
        _, calls = self.run_it("--dog", "unitree@dog", "--no_update")
        self.assertEqual([c.split()[0] for c in calls], ["ssh"])

    def test_it_will_not_guess_the_dog(self):
        for args in (["--dog"], ["--dog", "192.168.1.213"], ["--preview"]):
            result, calls = self.run_it(*args)
            self.assertEqual(result.returncode, 2, args)
            self.assertEqual(calls, [], args)


if __name__ == "__main__":
    unittest.main()
