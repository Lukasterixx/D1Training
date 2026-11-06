"""The console's stop path and live gates, against a fake program (a shell script): no dog, no docker."""
from __future__ import annotations

import json
from pathlib import Path
import stat
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
else:
    why = "E-stop: the console's link closed"
rec = d + "/rec"
os.makedirs(rec, exist_ok=True)
json.dump({"abort_reason": why, "end_reason": None if why else "ended", "policy_steps": steps, "left_released": False,
           "sport_mode": "mcf"}, open(rec + "/summary.json", "w"))
print("[press] record: " + rec, flush=True)
"""


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        fake = self.dir / "fake.py"
        fake.write_text(FAKE)
        fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
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
                                        launchers={"press": (str(fake), "fake-press"), "shadow": (str(fake), "fake-shadow")})

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
            self.assertTrue((REPO / p["file"]).exists(), p["file"])
            live = [m for m in p["modes"] if m != "dry"]
            if p["kind"] != "press":
                self.assertEqual(live, [])
            for m in live:
                self.assertIn("--live", p["modes"][m])
            self.assertNotIn("--live", p["modes"]["dry"])
            self.assertNotIn("measured", " ".join(sum(p["modes"].values(), [])))


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

    def test_the_page_and_state_are_served(self):
        self.assertIn(b"E-STOP", urllib.request.urlopen(self.url + "/", timeout=3).read())
        state = json.load(urllib.request.urlopen(urllib.request.Request(self.url + "/api/state",
                                                                        headers={"X-Console": "1"}), timeout=3))
        self.assertEqual(len(state["policies"]), 6)


if __name__ == "__main__":
    unittest.main()
