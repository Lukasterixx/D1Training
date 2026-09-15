# Results record

The experimental evidence for Thesis B, kept week by week. Anything a thesis
claim rests on should be traceable from here: the run that produced it, its
configuration and commit, the numbers, and a statement of what it does and does
not show.

View it with `./dashboard.py` from the repository root (it opens
<http://localhost:8765>). The page refreshes itself when files here change.

## Layout

| Path | Contents | Written by |
| --- | --- | --- |
| `config.json` | Term start date (Week 1 Monday), week count, log folders, headline scalars and their labels | Edited by hand |
| `findings.md` | Numbered findings (F-001, …) with status, date, evidence and implication | Claude, reviewed by Lukas |
| `gates.md` | Decision gates G0–G3 from the plan, with current status and evidence | Claude |
| `week_NN/notes.md` | The weekly log: plan, checklist, dated entries, results, issues, next steps | Claude |
| `week_NN/runs/<run id>/` | Recorded runs: `record.json`, copied config/metadata, `scalars.csv` | `dashboard.py record` |
| `week_NN/figures/` | Generated plots for the report (PNG/SVG) | Scripts |
| `week_NN/screenshots/` | Simulator screenshots and clips (PNG/JPG/MP4). Loose images directly in `week_NN/` also show | Lukas |
| `week_NN/external/<name>/` | Evidence from outside this repo (another project's experiments): copied outputs, configs, source snapshots, a provenance README and any script that summarises them | Claude |

Screenshots are optional in any week. They appear in name order, captioned from
the filename: `03_arm_self_collision.png` shows as "arm self collision".

## Recording a run

`logs/` is gitignored and large, so a run is not evidence until it is recorded:

```bash
./dashboard.py record logs/position_only/<run id> --title "64-env PPO pilot, seed 42"
```

This copies `run.json`, `env.yaml`, `agent.json`, `smoke.json`, `smoke_trace.csv`,
`verify.json` and any other small JSON/YAML/CSV outputs. It writes every TensorBoard scalar to
`scalars.csv` (one row per iteration, one column per tag) and stores the final checkpoint's name
and SHA-256. Smoke, verify and playback results are summarised in `record.json`. Playback
self-tests become run folders with `./run_sim.sh ... --selftest 10 --selftest_out logs/playback`.
The weights stay in `logs/`. The week defaults to when the run started. Recording
again refreshes the copy and keeps the title and notes.

Record failed and aborted runs too. A failure that changed a decision is evidence.

## Conventions

- **Numbers, not adjectives.** "Final reach error 4.1 cm (mean of 64 envs)" rather than "reaches well".
- **Name the scope.** A zero-action smoke run checks the interface, not reaching. A diagnostic metric logged
  at reset is not the frozen-manifest evaluation the gates require.
- **Finding status:** `confirmed` (reproduced or measured directly with the method stated),
  `provisional` (single run, unverified source, or method not yet frozen), `superseded` (replaced by a
  later finding, linked), `retracted` (shown wrong, with the reason). Findings are never deleted.
- **Gate status:** `not started`, `in progress`, `passed`, `failed`, `blocked`.
- **Week status:** the `**Status:**` line in each `notes.md`: `not started`, `in progress`, `complete`.
- Dates are absolute (2026-09-14), local time unless marked UTC. Run IDs are UTC stamps.
