"""Results record and dashboard: standard library only, no simulator."""
from datetime import date
import http.client
import json
from pathlib import Path
import shutil
import struct
import tempfile
import threading
import unittest

from evidence import mdrender, record, server, store, tfevents


def varint(n):
    out = bytearray()
    while True:
        byte, n = n & 0x7F, n >> 7
        out.append(byte | (0x80 if n else 0))
        if not n:
            return bytes(out)


def field(number, wire, payload):
    key = varint(number << 3 | wire)
    return key + varint(len(payload)) + payload if wire == 2 else key + payload


def event(step, tag, value=None, double=None, wall=1000.0):
    if value is not None:
        scalar = field(2, 5, struct.pack("<f", value))
    else:  # A one-element DT_DOUBLE tensor, as newer summary writers emit.
        scalar = field(8, 2, field(1, 0, varint(2)) + field(6, 2, struct.pack("<d", double)))
    summary = field(1, 2, field(1, 2, tag.encode()) + scalar)
    return field(1, 1, struct.pack("<d", wall + step)) + field(2, 0, varint(step)) + field(5, 2, summary)


def tfrecord(payload):
    return struct.pack("<Q", len(payload)) + b"\0" * 4 + payload + b"\0" * 4


class MarkdownTests(unittest.TestCase):
    def test_escapes_source_html_and_unsafe_links(self):
        out = mdrender.render('<script>alert(1)</script> [x](javascript:alert(1)) `<b>`')
        self.assertNotIn("<script>", out)
        self.assertIn("&lt;script&gt;", out)
        self.assertIn('href="#"', out)
        self.assertIn("<code>&lt;b&gt;</code>", out)

    def test_tables_keep_pipes_in_code_and_escapes(self):
        out = mdrender.render("| A | B |\n| :-- | --: |\n| `a|b` | c \\| d |\n")
        self.assertIn('<th style="text-align:left">A</th>', out)
        self.assertIn('<td style="text-align:right">c | d</td>', out)
        self.assertIn("<code>a|b</code>", out)

    def test_nested_and_task_lists(self):
        text = "- [x] done\n- [ ] open\n  - child\n    wrapped\n\n1. first\n3. second\n"
        out = mdrender.render(text)
        self.assertIn('<li class="task done" aria-label="done">', out)
        self.assertIn("<ul><li>child\nwrapped</li></ul>", out)
        self.assertIn("<ol><li>first</li><li>second</li></ol>", out)

    def test_loose_list_stays_one_list(self):
        out = mdrender.render("- one\n\n- two\n")
        self.assertEqual(out.count("<ul>"), 1)
        self.assertIn("<li><p>one</p></li>", out)

    def test_fences_comments_emphasis_and_reference_links(self):
        text = ("<!-- hidden -->\n# Title\n\n```bash\na < b && **x**\n```\n\n"
                "**Bold across\nlines** and *em* and snake_case_name and [spec][s].\n\n[s]: https://example.com\n")
        out = mdrender.render(text)
        self.assertNotIn("hidden", out)
        self.assertIn('<h1 id="title">Title</h1>', out)
        self.assertIn('<pre><code class="language-bash">a &lt; b &amp;&amp; **x**</code></pre>', out)
        self.assertIn("<strong>Bold across\nlines</strong>", out)
        self.assertIn("snake_case_name", out)
        self.assertIn('<a href="https://example.com" target="_blank" rel="noopener">spec</a>', out)

    def test_duplicate_headings_get_unique_ids(self):
        out = mdrender.render("## Log\n## Log\n")
        self.assertIn('id="log"', out)
        self.assertIn('id="log-1"', out)


class EventFileTests(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.dir)

    def test_reads_simple_and_tensor_scalars_incrementally(self):
        path = self.dir / "events.out.tfevents.1.host"
        first = tfrecord(event(0, "Train/mean_reward", value=1.5)) + tfrecord(event(0, "Loss/value", double=0.25))
        second = tfrecord(event(1, "Train/mean_reward", value=2.5))
        path.write_bytes(first + second[:10])  # Trailing record still being written.
        reader = tfevents.EventFileReader(path)
        scalars = reader.read()
        self.assertEqual(scalars["Train/mean_reward"], [(0, 1000.0, 1.5)])
        self.assertEqual(scalars["Loss/value"], [(0, 1000.0, 0.25)])
        path.write_bytes(first + second)
        self.assertEqual([p[2] for p in reader.read()["Train/mean_reward"]], [1.5, 2.5])

    def test_resumed_run_merges_files_and_later_file_wins(self):
        (self.dir / "events.out.tfevents.1.a").write_bytes(
            tfrecord(event(0, "x", value=1.0)) + tfrecord(event(1, "x", value=2.0)))
        (self.dir / "events.out.tfevents.2.a").write_bytes(
            tfrecord(event(1, "x", value=20.0)) + tfrecord(event(2, "x", value=3.0)))
        self.assertEqual([(s, v) for s, _, v in tfevents.read_scalars(self.dir)["x"]], [(0, 1.0), (1, 20.0), (2, 3.0)])

    @unittest.skipUnless(__import__("importlib").util.find_spec("tensorboard"), "tensorboard not installed")
    def test_matches_torch_summary_writer(self):
        from torch.utils.tensorboard import SummaryWriter
        writer = SummaryWriter(log_dir=str(self.dir))
        for step in range(5):
            writer.add_scalar("Metrics/error", 0.1 * step, step)
        writer.close()
        values = [v for _, _, v in tfevents.read_scalars(self.dir)["Metrics/error"]]
        for got, want in zip(values, [0.0, 0.1, 0.2, 0.3, 0.4]):
            self.assertAlmostEqual(got, want, places=6)


class RecordFixture(unittest.TestCase):
    """A temporary repository root with a results tree and one run in logs/."""

    def setUp(self):
        self.root = Path(tempfile.mkdtemp())
        results = self.root / "results"
        (results / "week_01" / "screenshots").mkdir(parents=True)
        (results / "config.json").write_text(json.dumps({
            "title": "Test", "term_start": "2026-09-14", "weeks": 10, "log_roots": ["logs/position_only"],
            "key_scalars": ["Train/mean_reward"], "scalar_labels": {},
        }))
        (results / "week_01" / "notes.md").write_text(
            "# Week 1\n\n**Status:** in progress\n**Focus:** Bring-up\n\n![shot](screenshots/01_arm_pose.png)\n"
            "[plan](../../docs/plan.md) [findings](../findings.md) [code](../../run.py)\n")
        (results / "week_01" / "screenshots" / "01_arm_pose.png").write_bytes(b"\x89PNG")
        (results / "findings.md").write_text(
            "# Findings\n\n### F-001 — Arm droop — set by gains\n\n- **Status:** provisional\n- **Week:** 1\n")
        (results / "gates.md").write_text("| Gate | Criterion | Status | Evidence |\n| --- | --- | --- | --- |\n"
                                          "| G0 · Environment | works | in progress | x |\n")
        self.run = self.root / "logs" / "position_only" / "20260915T010203_000000Z_train_seed42"
        self.run.mkdir(parents=True)
        (self.run / "run.json").write_text(json.dumps({
            "mode": "train", "seed": 42, "status": "training_finished", "git_commit": "abc123",
            "git_status": " M file.py", "arguments": {"num_envs": 64, "iterations": 3},
        }))
        (self.run / "smoke.json").write_text(json.dumps({
            "failure_resets": 2, "final_position_error_m": [0.1, 0.3],
            "posture_after_settling": {"base_height_m": {"min": 0.2, "mean": 0.26, "max": 0.42}}}))
        (self.run / "verify.json").write_text(json.dumps({
            "all_passed": False, "failed": ["arm_holds_zero_pose_at_rest"], "interpretation": "mechanism only",
            "checks": [{"name": "a", "passed": True}, {"name": "arm_holds_zero_pose_at_rest", "passed": False}]}))
        (self.run / "model_2.pt").write_bytes(b"weights")
        (self.run / "model_10.pt").write_bytes(b"later weights")
        (self.run / "big.bin").write_bytes(b"not copied")
        (self.run / "events.out.tfevents.1.host").write_bytes(b"".join(
            tfrecord(event(step, tag, value=float(step))) for step in range(3)
            for tag in ("Train/mean_reward", "Train/mean_reward/time")))

    def tearDown(self):
        shutil.rmtree(self.root)


class RecordTests(RecordFixture):
    def test_record_copies_artefacts_and_scalars_into_the_start_week(self):
        dest = record.record_run(self.run, title="Pilot", root=self.root)
        self.assertEqual(dest, self.root / "results/week_01/runs" / self.run.name)
        data = json.loads((dest / "record.json").read_text())
        self.assertEqual((data["week"], data["seed"], data["num_envs"], data["last_iteration"]), (1, 42, 64, 2))
        self.assertTrue(data["git_dirty"])
        self.assertEqual(data["final_checkpoint"]["file"], "model_10.pt")
        self.assertEqual(data["scalar_summary"]["Train/mean_reward"]["last"], 2.0)
        self.assertEqual(data["smoke_summary"]["final_position_error_m"]["mean"], 0.2)
        self.assertEqual(data["smoke_summary"]["posture_after_settling.base_height_m"]["mean"], 0.26)
        self.assertEqual((data["verify_summary"]["passed"], data["verify_summary"]["total"]), (1, 2))
        self.assertEqual(data["verify_summary"]["failed"], ["arm_holds_zero_pose_at_rest"])
        self.assertFalse((dest / "big.bin").exists())
        self.assertEqual(store.scalars_from_csv(dest / "scalars.csv"), {"Train/mean_reward": [[0, 0.0], [1, 1.0], [2, 2.0]]})

        # Re-recording keeps the title; recording into a different week is refused.
        data = json.loads((record.record_run(self.run, root=self.root) / "record.json").read_text())
        self.assertEqual(data["title"], "Pilot")
        with self.assertRaises(SystemExit):
            record.record_run(self.run, week=2, root=self.root)


class StoreTests(RecordFixture):
    def test_weeks_and_dates(self):
        cfg = store.load_config(self.root)
        self.assertEqual(store.week_of(cfg, date(2026, 9, 14)), 1)
        self.assertEqual(store.week_of(cfg, date(2026, 9, 21)), 2)
        self.assertIsNone(store.week_of(cfg, date(2026, 9, 13)))
        self.assertEqual(store.format_range(*store.week_range(cfg, 3)), "28 Sep – 4 Oct 2026")

    def test_week_detail_lists_media_and_resolves_links(self):
        cfg = store.load_config(self.root)
        week = store.week_detail(self.root, cfg, 1, date(2026, 9, 15))
        self.assertEqual((week["status"], week["focus"], week["current"]), ("in progress", "Bring-up", True))
        self.assertEqual([m["caption"] for m in week["screenshot_list"]], ["arm pose"])
        html = week["notes_html"]
        self.assertIn('src="/files/results/week_01/screenshots/01_arm_pose.png"', html)
        self.assertIn('href="#/doc/docs/plan.md"', html)
        self.assertIn('href="#/findings"', html)
        self.assertIn('href="/files/run.py"', html)

    def test_stamp_changes_when_notes_or_logs_change(self):
        cfg = store.load_config(self.root)
        before = store.stamp(self.root, cfg)
        (self.root / "results/week_01/screenshots/02_new.png").write_bytes(b"\x89PNG")
        after_shot = store.stamp(self.root, cfg)
        self.assertNotEqual(before, after_shot)
        with (self.run / "events.out.tfevents.1.host").open("ab") as handle:
            handle.write(tfrecord(event(3, "Train/mean_reward", value=3.0)))
        self.assertNotEqual(after_shot, store.stamp(self.root, cfg))

    def test_findings_gates_and_live_runs(self):
        cfg = store.load_config(self.root)
        overview = store.overview(self.root, cfg, date(2026, 9, 15))
        self.assertEqual(overview["findings"][0]["id"], "F-001")
        self.assertEqual(overview["findings"][0]["title"], "Arm droop — set by gains")
        self.assertEqual(overview["findings"][0]["status"], "provisional")
        self.assertEqual(overview["gates"], [{"id": "G0 · Environment", "name": "works", "status": "in progress"}])
        live = store.live_runs(self.root, cfg)
        self.assertEqual((live[0]["id"], live[0]["recorded_week"]), (self.run.name, None))
        self.assertNotIn("Train/mean_reward/time", store.scalars_from_events(self.run))


class ServerTests(RecordFixture):
    def setUp(self):
        super().setUp()
        self.previous_root = server.Handler.root
        server.Handler.root = self.root
        self.httpd = server.ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def tearDown(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        server.Handler.root = self.previous_root
        super().tearDown()

    def get(self, path, host="localhost"):
        conn = http.client.HTTPConnection("127.0.0.1", self.httpd.server_address[1])
        conn.request("GET", path, headers={"Host": host})
        response = conn.getresponse()
        body = response.read()
        conn.close()
        return response.status, body

    def test_api_and_files(self):
        status, body = self.get("/api/week/1")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["status"], "in progress")
        status, body = self.get(f"/api/scalars?live=logs/position_only/{self.run.name}")
        self.assertEqual(json.loads(body)["tags"]["Train/mean_reward"][-1], [2, 2.0])
        self.assertEqual(self.get("/files/results/week_01/screenshots/01_arm_pose.png"), (200, b"\x89PNG"))
        self.assertEqual(self.get("/")[0], 200)

    def test_rejects_traversal_foreign_hosts_and_unlisted_log_paths(self):
        self.assertEqual(self.get("/files/../etc/passwd")[0], 404)
        self.assertEqual(self.get("/files/%2e%2e/etc/passwd")[0], 404)
        self.assertEqual(self.get("/api/overview", host="evil.example")[0], 403)
        self.assertEqual(self.get("/api/scalars?live=results")[0], 404)
        self.assertEqual(self.get("/api/week/99")[0], 404)


if __name__ == "__main__":
    unittest.main()
