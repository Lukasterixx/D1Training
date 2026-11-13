# RSS 2027 Stage 1 draft: tool-mediated force-position control

The alternative to [`../rss2027/`](../rss2027/README.md), the D1 interface paper, written so that the thesis's
force-aware tool work is the subject. Same venue, deadline and format: extended abstract, 5 pages of content plus 1 of
references, double-blind, due **4 December 2026, 11:59 pm AoE**. Pick one: RSS forbids dual submission, and the two
papers share the force-wipe evidence.

- `main.tex`: the paper (the RSS template's `IEEEtran.cls`, unmodified).
- `references.bib`: verified entries only (see the header).
- `figures/make_figures.py`: Fig. 2, from the counts in F-284 and F-285 (typed in from the findings' tables, not
  recomputed from evaluation logs).

## The framing

**The question.** Learned force-position whole-body policies (UniFP) are commanded with a hand position and force. A tool
task gives neither: it gives a state of the tool or mechanism. So who sets the force, and when does force awareness help
with a tool?

**The method.** A task layer turns tool state into force-position references (lag-PI for mechanisms, touch-probe-spiral
for insertion, an arc for the turn). The policy is trained with the layer in the loop and paid to follow it.

**The hypotheses map onto the thesis plan's P-ladder:**
- H3 is P3 against P2: force awareness with tools, as matched twins.
- H4 is P4 against P3: tool conditioning, pre-registered with no result yet.
- H1 and H2 are the task-layer and reward lessons from F-105 to F-109 and F-283 to F-285.
- The D1 interface work appears only as infrastructure in Setting.

## What is and is not new (from the reference check)

A layer issuing force commands to a learned force-position tracker is **not** new on its own:
- UniFP drives its low-level policy with a diffusion policy trained on demonstrations.
- Ctrl-F (arXiv 2611.05507, 3 Nov 2026, humanoid, not peer reviewed) plans forces online from object state for a learned
  motion-force tracker.

The draft cites both and claims only these:
- analytic, inspectable laws on the tool's state, with no demonstrations;
- the low level trained with that layer in the loop and paid to follow it (H2);
- contact through a held tool;
- the matched force-aware / position-only test (H3);
- tool conditioning (H4).

Read Ctrl-F before submitting. If it already trains its tracker with the planner in the loop, H1's framing needs to give
way to H2 and H3.

## Where each number comes from

| Claim | Finding |
| --- | --- |
| Scripted chain on the dog: 5 of 11, three operator stops; once in one run | F-224, F-219 |
| Key 66 mm / 35 mm past seated; 2 mm grip offset gives 33.5-34.3 deg of 45 | F-217, F-221, F-224 |
| Same stack in simulation seats 0 of 12 from its look, 3-11 mm range error | F-164 |
| Pull policy at most ~30 N; outcome-only stalls at ~30 N | F-105 |
| Zero-shot PI law 337 / 480; end-to-end 358 / 480; layer in loop 504 / 512 | F-106, F-107, F-108 |
| 40 N cap treated as advice: 146 / 180 vs 56 / 180 | F-109 |
| Combiner with claw 16 / 16 to 16 N·m; UniFP 9 / 16, none from 8 N·m | F-110 |
| Insertion: 66-81 within the stop vs classical arm 11-25; twins' held-out table; 1/800 and 1/3000 m/N | F-284 |
| Insertion nominal 69 (FA `8200`) and 81 (PO `8600`) | week 9 log, 2026-11-13, round 6 |
| Key hold 104-108 / 108; IK arm 18; arm fixed 0; stiffer-lock lead retracted (79/45 to 104/94) | F-278 and its notes |
| Key-hold training broke at UniFP's learning rate | F-279 |
| Jolt: 45 deg in ~0.1 s, 460-600 deg/s, wrist lead 40-46 deg | F-283 |
| Arc reward: 0 of 911 ahead, 1.2-1.5 s, lead 13-23 deg; FA 81-107, PO 29-68, stalls at ~26 deg | F-285 |
| Door and lever pose policy 32 / 31 vs 18 / 4 | F-274 |
| Force through the cloth on the dog: 7 to 19 N, ~2 N/N, feet ~1 N, 8 wipes 2-21 N | F-269, F-271 (Fig. 2b of the other paper) |

## What a reviewer will push on, and the honest answers already in the text

- **Simulation only for every learned tool result.** Stated in the abstract's last lines, in Results and in Limitations.
  E5 is the hardware plan.
- **Different interface models across tasks:** T1 and the claw demo predate the identified arm model (September, idealized
  50 Hz arm), while T2 to T4 use it. Setting says this.
- **Twins from different parents and seeds** (F-285's scope). Limitations says so, and E3 fixes it with a common parent.
- **Selection on the scored set, one seed.** Stated; Stage 2 uses validation and test splits and three seeds.
- **H4 has no data.** It is pre-registered. Stage 1 allows that, but it is the part the supervisor's tool-conditioning
  question rests on, so it is the first thing to start if this paper is chosen.

## Before 4 December

1. Choose between this and `../rss2027/`.
2. If this one: start E4's tool family in simulation now (even one seed of P3 against P4 on two key lengths would turn
   H4 from a plan into a preliminary result).
3. The rest as in `../rss2027/README.md`: authors, OpenReview profiles, a pdfLaTeX build, an anonymity check.
