# RSS 2027 Stage 1 draft

Extended-abstract submission for RSS 2027, due **4 December 2026, 11:59 pm AoE** on OpenReview
([call for papers](https://roboticsconference.org/information/cfp/)). Stage 1 is at most 5 pages of content plus 1 page
of references, double-blind, with no supplementary material and no external links. Reviewers judge the problem,
hypotheses and methodology; experiments may be preliminary. Invited papers then get an Extension Charter (26 Feb 2027)
and submit the full 8-page paper on 16 April 2027.

- `main.tex`: the paper (the RSS template's `IEEEtran.cls`, unmodified).
- `references.bib`: every entry checked against a primary source on 2026-11-13. RSS desk-rejects hallucinated references,
  so add nothing without the same check.
- `figures/make_figures.py`: Fig. 2, rebuilt from `logs/` and `results/` only.

Build with pdfLaTeX (Overleaf, or any TeX Live) + BibTeX. The length check here used tectonic (XeTeX), which skips
`\pdfinfo`. It gives **6 pages: content ends on page 5, references fill the rest**. RSS requires Type-1 fonts, so build
the submitted PDF with pdfLaTeX, not tectonic.

## Direction chosen

The paper is about deploying a learned force-aware whole-body policy (UniFP) through the D1's slow, closed interface. It
has four hypotheses: H1 report-locked commanding, H2 training through the identified arm model, H3 leg response and the
real-frame replay, H4 the force channel open loop. It deliberately leaves out the combiner box chain (scripted IK on
sport mode), the foot-press and key-turn leg policies, the rope pull, key insertion and the pose policies. Those are
mostly simulation, or not learned whole-body control on the dog, and each would need its own paper to be defended.

## Where each number comes from

| Claim in the paper | Finding |
| --- | --- |
| 9.00 Hz reports, slew 1.2-1.3 rad/s, no brake, gripper angle required, plan restarts per setpoint | F-020, F-033, F-028, F-202, F-045/F-046 |
| Stalls set by command phase (98-106 ms; 36/42; 52 of 590 held, all in window) | F-238; Fig. 2a reproduces the 590 held cycles exactly |
| Locked: 0/716 held, J1 1/894, 0.44-0.98 deg; six joints 2/449, 0.8%, ~10 deg, 50.2 s | F-239, F-240 |
| Lag 77 ms fit, 0.23 deg (0.19-0.32 held out), planner 0.37; six-joint 0.23-0.57 | F-250, F-245 |
| Ideal 1.93 -> 16.6 cm on the fitted model | F-243 status note (F-250 profile) |
| 0.28 of 3.4 N, 87% past a limit, 116 deg, factor ablation | F-241 (the earlier, superseded arm model, which the paper says) |
| Lock 4.02 / 4.90 and the shake; Step 2.15 / 2.83 | F-243 note, F-251, F-253 |
| Step on the dog: 2 runs stopped at 1.5 s, lead 33-43 vs 3.8 simulated | F-255 (+ its 2026-11-07 note) |
| Legs 0.165 / 0.175 vs 0.51 and 0.28-0.32 in sim | F-256, week 8 log 2026-11-06 "Friction is part of it" |
| Held legs: 13 of 27, 7.5 cm (66200) | F-252 scope note |
| Replay 1.4 / 1.8 deg RMS, blind | F-256 |
| 16 checkpoints, none passes; the deployed one missed by 2-6 deg | F-258, week 8 log "Scoring on real frames" |
| Replay conservative / wrong pose / J4 13 vs 23 | F-257 notes, F-258, F-261 |
| Sticky: stand 4/4, far goals, tour, walk, wipes | F-257, F-261, F-263, F-270 log, F-271 |
| Sim far goals 60/60, 3.0 cm | F-259 |
| Scale stalled at 18-26 N; real force vs told; foot sum 0.9 N | F-268, F-269 |
| Wipe estimates 2-21 N, -4 to 28 N within a stroke, zero +-9 N | F-271, F-270 |
| Isaac Lab port within 0.4%, 1.6-2.4 cm, 4,096 envs, 60k iterations | F-089, F-092 |

**One deliberate difference from the record:** Fig. 2b's scale points use every 0.5 s reading at a steady told level
(7.1, 11.3, 14.6, 19.1 N; fit 1.96 x told + 7.1). F-269's table gives 6.4, 10.9, 14.2, 19.0 (2.06 x told + 6.4) from a
selection the CSV does not record. The paper quotes the figure's numbers so the text and figure agree. Same conclusion.

**Naming in the paper:** Ideal = `59600` (corner delay arm), Lock = `62800`, Step = `armft5` `66200`,
Sticky = `armft6b` `66600` (`reach_force_sticky_model_66600`). Note `armft5` also has a `66600`; it is not the one deployed.

## Before 4 December

1. **Lukas:** read for framing and title; decide authors (Stage 1 authorship is effectively fixed); make OpenReview
   profiles now (new profiles without an institutional email take up to two weeks).
2. **The 40 Hz question:** Schperberg et al. (ICRA 2026) report the D1's send/receive rate as 40 Hz. Read how they
   talk to the arm. If it is a different SDK path or firmware, the paper's framing of "a 9 Hz arm" must say "through
   the vendor DDS interface", as it now does, and E1 must test the other path.
3. **Cheap, strengthening evidence if the dog is free** (Stage 1 does not need it, Stage 2 does): an external hand
   measurement on a few far-goal runs (a tag on the gripper and the calibrated front camera); scoring the wipe videos.
4. Anonymity check of the final PDF (no names, no links, no acknowledgements); LLM-use disclosure is encouraged, not
   required.
