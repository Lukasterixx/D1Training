# Week 10 — 16–22 November 2026

**Status:** in progress
**Focus:** Final tagged-box demonstration, Thesis B report and reproducible C handover

## Planned

From the [revised Thesis B plan](../../docs/thesis_b_plan.md#thesis-b-weekly-schedule),
16 September 2026. This replaces the original Appendix A timing; earlier dated logs remain historical evidence.

- **Reuse, simulation and learning:** Finish report, figures and experiment archive; resolve only reproducibility faults.
- **Hardware and camera:** Deliver the final AprilTag-guided real-robot demonstration and its repeated-trial evidence; hand over the calibrated system.
- **Evidence and deliverable:** Prioritise C work: markerless pose validation, then broader trials or learned task policies. Preserve the tagged/scripted reference and any declared B shortfalls.
- **Gate focus:** Final G0–G7 status audit; no implied passes from deadlines.

## Checklist

- [ ] Final real-robot tagged-box demonstration and repeated-trial evidence linked
- [ ] Report figures regenerated from recorded results
- [ ] Archive includes source revisions, configurations, manifests, checkpoints and calibrations
- [ ] G0–G7 final statuses accurately reflect evidence and unresolved issues
- [ ] C handover includes tagged reference, recordings, markerless candidate and comparison protocol
- [ ] Optional learned visual policy and broader task/tool work scoped separately from B recovery

## Log

<!-- Dated entries (### YYYY-MM-DD · topic). For each: the exact command or change, the outcome in numbers,
     and what it does and does not show. Record runs with ./dashboard.py record. -->

### 2026-11-17 · S2's grid and analysis (H3): the twins from one parent do not trade places, and F-284's heavy-leg contrast was the parents'

**The grid's last condition, rerun.** All 192 grid evaluations (12 selected twins × 16 conditions) had run by 14 Nov
18:06. One left no result: `s2t_Ff_s63_8800_g_heavy` (14 Nov 16:39, lane e3) died in the weld. It found no `base_link` in
`generated/d1.usd`, which another evaluation starting in the same second was rewriting. It exited 0 and made no run
directory, so nothing was recorded. Rerun alone today (10:39-10:40, lane e, the same job as `g_07r_s2t_Ff_s63_heavy`):

```
logs/unifp_train/s1tools/tev.sh s2t_Ff_s63_8800_g_heavy "S2 twin force from ki6_force_8200 seed 63, selected 8800: S2 grid heavy (body) on the twins' test locks" \
  logs/unifp_train/s1tools/s2t_Ff_s63_8800.pt test --insert_tool_manifest results/manifests/keyinsert_twins_v1.json \
  --leg_variant force --leg_friction_range 1 1 --leg_armature 0.002 --leg_knee_armature 0.0073
```

[31 of 96](#/week/10/run/20261116T233957_leg_eval_seed1_s2t_Ff_s63_8800_g_heavy). Parallel Isaac starts can race on
`generated/d1.usd`. Every other grid result has its run directory, but a zero-exit "done" is not proof of a result
(the S3 smoke's lesson again).

**The twins** (seeds 61-63, 800 iterations from each family's parent, selected by S1's rule on the 48 validation
locks). Successes inside 15 deg, of 96 test locks, per condition:

| twin | seed | training | selected | nominal | nominal_f2 | zeroarm | heavy | delay0 | d14 | settled | tool mean (9) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Ff | 61 | [train](#/week/9/run/20261113T070414_train_seed61_s2t_Ff_s61) | `8600` | [64](#/week/9/run/20261113T125912_leg_eval_seed1_s2t_Ff_s61_8600_g_nominal) | [55](#/week/9/run/20261113T101154_leg_eval_seed1_s2t_Ff_s61_8600_g_nominal_f2) | [53](#/week/9/run/20261113T140607_leg_eval_seed1_s2t_Ff_s61_8600_g_zeroarm) | [72](#/week/9/run/20261113T100342_leg_eval_seed1_s2t_Ff_s61_8600_g_heavy) | [54](#/week/9/run/20261113T091750_leg_eval_seed1_s2t_Ff_s61_8600_g_delay0) | [57](#/week/9/run/20261113T091346_leg_eval_seed1_s2t_Ff_s61_8600_g_d14) | [45](#/week/9/run/20261113T130954_leg_eval_seed1_s2t_Ff_s61_8600_g_settled) | 52.8 |
| Ff | 62 | [train](#/week/9/run/20261113T103956_train_seed62_s2t_Ff_s62) | `8400` | [64](#/week/9/run/20261113T205508_leg_eval_seed1_s2t_Ff_s62_8400_g_nominal) | [62](#/week/9/run/20261113T205058_leg_eval_seed1_s2t_Ff_s62_8400_g_nominal_f2) | [62](#/week/9/run/20261113T215927_leg_eval_seed1_s2t_Ff_s62_8400_g_zeroarm) | [57](#/week/9/run/20261113T202640_leg_eval_seed1_s2t_Ff_s62_8400_g_heavy) | [73](#/week/9/run/20261113T195847_leg_eval_seed1_s2t_Ff_s62_8400_g_delay0) | [55](#/week/9/run/20261113T195440_leg_eval_seed1_s2t_Ff_s62_8400_g_d14) | [42](#/week/9/run/20261113T212207_leg_eval_seed1_s2t_Ff_s62_8400_g_settled) | 54.7 |
| Ff | 63 | [train](#/week/9/run/20261113T141036_train_seed63_s2t_Ff_s63) | `8800` | [55](#/week/9/run/20261114T055245_leg_eval_seed1_s2t_Ff_s63_8800_g_nominal) | [56](#/week/9/run/20261114T053952_leg_eval_seed1_s2t_Ff_s63_8800_g_nominal_f2) | [58](#/week/9/run/20261114T060324_leg_eval_seed1_s2t_Ff_s63_8800_g_zeroarm) | [31](#/week/10/run/20261116T233957_leg_eval_seed1_s2t_Ff_s63_8800_g_heavy) | [40](#/week/9/run/20261114T053143_leg_eval_seed1_s2t_Ff_s63_8800_g_delay0) | [40](#/week/9/run/20261114T052030_leg_eval_seed1_s2t_Ff_s63_8800_g_d14) | [29](#/week/9/run/20261114T055649_leg_eval_seed1_s2t_Ff_s63_8800_g_settled) | 46.0 |
| Fp | 61 | [train](#/week/9/run/20261113T085214_train_seed61_s2t_Fp_s61) | `8800` | [79](#/week/9/run/20261113T170032_leg_eval_seed1_s2t_Fp_s61_8800_g_nominal) | [81](#/week/9/run/20261113T165029_leg_eval_seed1_s2t_Fp_s61_8800_g_nominal_f2) | [71](#/week/9/run/20261113T175143_leg_eval_seed1_s2t_Fp_s61_8800_g_zeroarm) | [61](#/week/9/run/20261113T164320_leg_eval_seed1_s2t_Fp_s61_8800_g_heavy) | [60](#/week/9/run/20261113T161918_leg_eval_seed1_s2t_Fp_s61_8800_g_delay0) | [69](#/week/9/run/20261113T161607_leg_eval_seed1_s2t_Fp_s61_8800_g_d14) | [27](#/week/9/run/20261113T171639_leg_eval_seed1_s2t_Fp_s61_8800_g_settled) | 70.8 |
| Fp | 62 | [train](#/week/9/run/20261113T122458_train_seed62_s2t_Fp_s62) | `8400` | [80](#/week/9/run/20261114T010148_leg_eval_seed1_s2t_Fp_s62_8400_g_nominal) | [78](#/week/9/run/20261114T005745_leg_eval_seed1_s2t_Fp_s62_8400_g_nominal_f2) | [71](#/week/9/run/20261114T020833_leg_eval_seed1_s2t_Fp_s62_8400_g_zeroarm) | [65](#/week/9/run/20261114T005023_leg_eval_seed1_s2t_Fp_s62_8400_g_heavy) | [73](#/week/9/run/20261114T002012_leg_eval_seed1_s2t_Fp_s62_8400_g_delay0) | [66](#/week/9/run/20261114T001608_leg_eval_seed1_s2t_Fp_s62_8400_g_d14) | [41](#/week/9/run/20261114T013131_leg_eval_seed1_s2t_Fp_s62_8400_g_settled) | 69.1 |
| Fp | 63 | [train](#/week/9/run/20261113T155759_train_seed63_s2t_Fp_s63) | `8400` | [73](#/week/9/run/20261114T063716_leg_eval_seed1_s2t_Fp_s63_8400_g_nominal) | [79](#/week/9/run/20261114T063251_leg_eval_seed1_s2t_Fp_s63_8400_g_nominal_f2) | [66](#/week/9/run/20261114T064629_leg_eval_seed1_s2t_Fp_s63_8400_g_zeroarm) | [48](#/week/9/run/20261114T062701_leg_eval_seed1_s2t_Fp_s63_8400_g_heavy) | [72](#/week/9/run/20261114T062316_leg_eval_seed1_s2t_Fp_s63_8400_g_delay0) | [66](#/week/9/run/20261114T062216_leg_eval_seed1_s2t_Fp_s63_8400_g_d14) | [30](#/week/9/run/20261114T064044_leg_eval_seed1_s2t_Fp_s63_8400_g_settled) | 60.9 |
| Pf | 61 | [train](#/week/9/run/20261113T102534_train_seed61_s2t_Pf_s61) | `9000` | [60](#/week/9/run/20261113T150208_leg_eval_seed1_s2t_Pf_s61_9000_g_nominal) | [74](#/week/9/run/20261113T145847_leg_eval_seed1_s2t_Pf_s61_9000_g_nominal_f2) | [70](#/week/9/run/20261113T160536_leg_eval_seed1_s2t_Pf_s61_9000_g_zeroarm) | [25](#/week/9/run/20261113T143727_leg_eval_seed1_s2t_Pf_s61_9000_g_heavy) | [59](#/week/9/run/20261113T141950_leg_eval_seed1_s2t_Pf_s61_9000_g_delay0) | [74](#/week/9/run/20261113T141014_leg_eval_seed1_s2t_Pf_s61_9000_g_d14) | [45](#/week/9/run/20261113T152753_leg_eval_seed1_s2t_Pf_s61_9000_g_settled) | 55.6 |
| Pf | 62 | [train](#/week/9/run/20261113T135558_train_seed62_s2t_Pf_s62) | `8800` | [72](#/week/9/run/20261113T224628_leg_eval_seed1_s2t_Pf_s62_8800_g_nominal) | [71](#/week/9/run/20261113T224246_leg_eval_seed1_s2t_Pf_s62_8800_g_nominal_f2) | [75](#/week/9/run/20261113T235406_leg_eval_seed1_s2t_Pf_s62_8800_g_zeroarm) | [0](#/week/9/run/20261113T223618_leg_eval_seed1_s2t_Pf_s62_8800_g_heavy) | [58](#/week/9/run/20261113T220710_leg_eval_seed1_s2t_Pf_s62_8800_g_delay0) | [68](#/week/9/run/20261113T220334_leg_eval_seed1_s2t_Pf_s62_8800_g_d14) | [42](#/week/9/run/20261113T231651_leg_eval_seed1_s2t_Pf_s62_8800_g_settled) | 69.4 |
| Pf | 63 | [train](#/week/9/run/20261113T172846_train_seed63_s2t_Pf_s63) | `8800` | [83](#/week/9/run/20261114T061104_leg_eval_seed1_s2t_Pf_s63_8800_g_nominal) | [63](#/week/9/run/20261114T061049_leg_eval_seed1_s2t_Pf_s63_8800_g_nominal_f2) | [77](#/week/9/run/20261114T062207_leg_eval_seed1_s2t_Pf_s63_8800_g_zeroarm) | [0](#/week/9/run/20261114T060811_leg_eval_seed1_s2t_Pf_s63_8800_g_heavy) | [48](#/week/9/run/20261114T060505_leg_eval_seed1_s2t_Pf_s63_8800_g_delay0) | [64](#/week/9/run/20261114T060342_leg_eval_seed1_s2t_Pf_s63_8800_g_d14) | [43](#/week/9/run/20261114T061502_leg_eval_seed1_s2t_Pf_s63_8800_g_settled) | 73.6 |
| Pp | 61 | [train](#/week/9/run/20261113T121002_train_seed61_s2t_Pp_s61) | `9200` | [65](#/week/9/run/20261113T184315_leg_eval_seed1_s2t_Pp_s61_9200_g_nominal) | [67](#/week/9/run/20261113T183931_leg_eval_seed1_s2t_Pp_s61_9200_g_nominal_f2) | [67](#/week/9/run/20261113T194309_leg_eval_seed1_s2t_Pp_s61_9200_g_zeroarm) | [0](#/week/9/run/20261113T181606_leg_eval_seed1_s2t_Pp_s61_9200_g_heavy) | [77](#/week/9/run/20261113T180540_leg_eval_seed1_s2t_Pp_s61_9200_g_delay0) | [69](#/week/9/run/20261113T180201_leg_eval_seed1_s2t_Pp_s61_9200_g_d14) | [30](#/week/9/run/20261113T190931_leg_eval_seed1_s2t_Pp_s61_9200_g_settled) | 61.6 |
| Pp | 62 | [train](#/week/9/run/20261113T154128_train_seed62_s2t_Pp_s62) | `9200` | [72](#/week/9/run/20261114T031151_leg_eval_seed1_s2t_Pp_s62_9200_g_nominal) | [59](#/week/9/run/20261114T030845_leg_eval_seed1_s2t_Pp_s62_9200_g_nominal_f2) | [74](#/week/9/run/20261114T041206_leg_eval_seed1_s2t_Pp_s62_9200_g_zeroarm) | [3](#/week/9/run/20261114T024352_leg_eval_seed1_s2t_Pp_s62_9200_g_heavy) | [72](#/week/9/run/20261114T023314_leg_eval_seed1_s2t_Pp_s62_9200_g_delay0) | [78](#/week/9/run/20261114T023014_leg_eval_seed1_s2t_Pp_s62_9200_g_d14) | [44](#/week/9/run/20261114T033910_leg_eval_seed1_s2t_Pp_s62_9200_g_settled) | 67.6 |
| Pp | 63 | [train](#/week/9/run/20261113T191601_train_seed63_s2t_Pp_s63) | `9399` | [83](#/week/9/run/20261114T065600_leg_eval_seed1_s2t_Pp_s63_9399_g_nominal) | [69](#/week/9/run/20261114T065308_leg_eval_seed1_s2t_Pp_s63_9399_g_nominal_f2) | [83](#/week/9/run/20261114T070313_leg_eval_seed1_s2t_Pp_s63_9399_g_zeroarm) | [20](#/week/9/run/20261114T065136_leg_eval_seed1_s2t_Pp_s63_9399_g_heavy) | [72](#/week/9/run/20261114T064849_leg_eval_seed1_s2t_Pp_s63_9399_g_delay0) | [64](#/week/9/run/20261114T064750_leg_eval_seed1_s2t_Pp_s63_9399_g_d14) | [44](#/week/9/run/20261114T065629_leg_eval_seed1_s2t_Pp_s63_9399_g_settled) | 74.8 |

Family F is fine-tuned from `ki6_force_8200`, family P from `ki6_pos_8600`. The lowercase letter is the twin's variant:
f force-aware, p position-only. Conditions: nominal_f2 is friction 2 N·m; zeroarm is zero leg armature; heavy is 0.002
with knees at 0.0073; delay0 is no leg delay; d14 is 5-20 ms; settled is the settled stance. The tool mean is over the
nine lock and tool conditions.

**Mean over the three seeds** (of 96):

| condition | F force | F position | P force | P position |
|---|---|---|---|---|
| nominal | 61.0 | 77.3 | 71.7 | 73.3 |
| friction 2 N·m | 57.7 | 79.3 | 69.3 | 65.0 |
| zero armature | 57.7 | 69.3 | 74.0 | 74.7 |
| heavy legs | 53.3 | 58.0 | 8.3 | 7.7 |
| delay 0 | 55.7 | 68.3 | 55.0 | 73.7 |
| delay 5-20 ms | 50.7 | 67.0 | 68.7 | 70.3 |
| settled stance | 38.7 | 32.7 | 43.3 | 39.3 |
| sticky (mu 0.45) | 49.7 | 67.7 | 66.0 | 68.7 |
| tight (0.1 mm) | 50.7 | 67.0 | 58.3 | 64.7 |
| no edge breaks | 40.0 | 59.3 | 65.3 | 64.3 |
| errors ×1.3 | 47.3 | 65.7 | 68.3 | 71.3 |
| errors ×1.5 | 47.7 | 61.7 | 59.7 | 63.0 |
| contact ×0.5 | 56.0 | 63.3 | 71.7 | 71.3 |
| contact ×1.5 | 55.0 | 69.0 | 62.3 | 59.0 |
| reach 45 mm | 54.0 | 70.0 | 74.0 | 71.0 |
| reach 24 mm | 60.0 | 78.7 | 70.0 | 78.3 |

**The ×1.5 contact stays in.** The pre-stated rule dropped it only if its peaks showed the penalty contact unstable. Over
all twins (1,152 episodes per condition):

| condition | median peak | p90 peak | p99 peak | max peak | damaged |
|---|---|---|---|---|---|
| nominal | 26.9 N | 36 N | 82 N | 101 N | 44 |
| ×1.5 | 29.1 N | 63 N | 94 N | 104 N | 193 |
| tight | | 43 N | | | 106 |
| sticky | | 51 N | | | 131 |

A heavier tail with the same maximum is what a stiffer lock does, as the tight and sticky locks do. It is not a blow-up.
The analysis is also reported without it.

**The analysis stated before scoring** (`logs/analysis_venv/bin/python logs/unifp_train/s1tools/s2_analysis.py`, and with
`stiff` to drop the ×1.5 contact; `s2_analysis.json`, `s2_analysis_drop_stiff.json`). The model is success inside 15 deg ~
variant × kind (nominal, body, tool) + family, with seed and condition random. It is fitted by variational Bayes, over
18,432 episodes. H3's contrast is (position − force under tool shifts) − (position − force under body shifts), on the
log-odds scale. H3 predicts it positive in both families.

| fit | contrast | ~95% | without ×1.5 |
|---|---|---|---|
| pooled, family fixed | +0.125 | +0.03 to +0.22 | +0.147 (+0.05 to +0.24) |
| family F | +0.250 | +0.12 to +0.38 | +0.258 (+0.12 to +0.39) |
| family P | −0.020 | −0.16 to +0.12 | +0.014 (−0.13 to +0.16) |
| GEE clustered by condition (pre-stated check), pooled | +0.143 | −0.12 to +0.40 (p 0.28) | +0.163 (p 0.22) |

**By the stated rule H3 is not supported:** the contrast is zero in family P.

**Added after seeing the model (post hoc, so labelled), on why the GEE's interval is three times wider.** The model gives
each kind one interaction and treats 18,432 episodes as independent around it. The variant gap, though, differs from
condition to condition far more than between kinds. Tested with the condition as the unit (`s2_analysis.py`'s post hoc
section), the log-odds gap per condition less nominal's:

| family | body (d14, delay0, heavy, f2, settled, zeroarm) | tool | tool − body | Welch p | exact permutation p (5,005) |
|---|---|---|---|---|---|
| F | −0.13, −0.28, −0.65, +0.28, −1.12, −0.31 | −0.53 to +0.13 | +0.23 | 0.31 | 0.23 |
| P | −0.01, +0.80, −0.18, −0.30, −0.26, −0.05 | −0.26 to +0.40 | +0.00 | 0.995 | 0.995 |

The GEE per family agrees: F +0.26 (p 0.15), P +0.00 (p 0.98). A run effect in place of the seed's (the pre-stated seed
label groups a force and a position twin sharing a seed number) moves nothing: +0.122 (sd 0.048). **Family F's
"distinguishable" interaction rests on the model's independence, not on the conditions.**

What the grid does show:
- **Position-only is level or ahead nearly everywhere.** The variant's main effect is +0.49 log-odds pooled: +0.87 in family F (61 against 77 nominal) and +0.10 in P.
  - In family F, position-only is ahead in every condition but the settled stance (38.7 force-aware against 32.7).
  - In family P the twins are within 4 of 96 in most conditions. Force-aware is ahead from the settled stance (43.3 against 39.3), at 2 N·m (69 against 65) and by 1-3 in four tool conditions. Position-only is ahead with no leg delay (74 against 55).
  - The settled stance is the one condition where force-aware leads in both families; per seed it is mixed.
  - Neither variant's latent is force-supervised here (S2's design entry): force-aware means a commanded press and a compliant target.
- **Heavy legs split by parent, not by variant.** Family F's twins keep 31-72 of 96, family P's 0-25, both variants alike. F-284's heavy-leg contrast (force-aware 61 against position-only 4) compared exactly these two parents. It was the parents' difference, not force awareness's.
- **Family F's force-aware twins score lowest of the four** (55-64 nominal), while the position-only twins fine-tuned from the same parent get 73-80. On F-284's frozen 96 the parent got 66-69, but that is a different lock set, so whether the fine-tune cost it anything is not shown.

Finding: [F-291](../findings.md#f-291). F-284 gets a dated note.

**Docs:** `docs/tool_paper_plan.md`'s hypothesis table still showed the 13 Nov evidence. A dated "Results so far" note under it
now gives H2 (F-290), H3 (F-291) and H4 (F-289). The table itself is left as written.

**Not shown:**
- One insertion task, on simulated locks.
- Body conditions are scored on the probe legs at 15-20 ms (the grid's default), not `66600`'s training range.
- With two families of three seeds each, a variant effect that depends on the parent is only seen, not modelled.

### 2026-11-18 · S1c: why the layer believes keys seated that are not; a "located" seated test does not fix it at evaluation time, and the simulated keyway hardly constrains the key's roll

Lukas (17 Nov): "continue with 3", the false seats (F-287 to F-289: 139-264 of 576 in every multi-key arm).

**Code** (2026-11-18):
- `keyinsert_eval` traces the layer's measured lateral, its reference, how it went in, and where its feel started.
- `keylock.InsertLayer.seat_test = "located"` (`--insert_seat_test located`, opt-in, not validated) adds a second way to judge a seat. The old test is unchanged, so every earlier number stands.
- `tests/test_keylock.py` has two new layer tests: a key straight in seats from the located face; a 4 mm jam is refused where the old layer held it.
- Evaluation and training starts now take a lock for their first 25 s (`s1tools/tev.sh`, `train.sh`): the S2 weld race.
- Analysis scripts: `s1tools/false_seats.py`, `seat_features.py`, `slid_rule.py`, `s1c_compare.py`.

**The diagnosis, on S1b's test traces** (20 runs, 4,600 believed seats, 1,615 false):
- 1,454 of the false seats came through the two ways the layer accepts a key stopped short of the floor:
  - the jiggle's time-out ("then the layer holds it where it is"): 880;
  - a key found in by its hold, taken as seated once still: 574.
- The rest were measured "deep" (6.7 mm past the touch) while short.
- 1,050 of the false seats were keys inside the keyway 2-8 mm in.

**The same on validation, with the lateral traced** (the runs are in the table below; S1b's nine selected checkpoints and the parent, v3's 24 validation locks × 6 keys at 1 and 2 N·m): 2,431 believed, 793 false. Where the true tip was at the layer's touch:

| at the touch | true seats | false seats |
|---|---|---|
| inside the keyway (the key went straight in) | 1,432 | 377 |
| on the face | 205 | 336 |
| short of the face, in free air (< -1 mm) | 1 | 80 |

- **87% of true seats went straight in.** The layer then felt the floor or a jam, never the face. Its depth past the touch cannot tell the two apart, and the time-out and hold exits exist so that those keys finish.
- **The probe does not sort them in simulation.** On the first seven of these runs, the 3 mm sideways feel after the touch read "held" for 10 of 568 touches inside the keyway and 1 of 224 on the face. Inside the keyway the measured tip still moved 2.2 mm (median), the key bending in the jaws under the stiff servo. A threshold on the measured slide separated nothing either (`slid_rule.py`).
- **Free-air touches** (81 believed, 80 of them false): the hand fell behind the reference without contact while the body moved. The layer then took the air for the face. No layer can make that key reach; it should only not believe it.

**The located test, three revisions on the same locks** (successes inside 15 deg / false seats, of 144):
- **The idea:** the first touch is made with the wrist rolled 45 deg, where the wards cannot enter, so that touch is the face. A key is believed in only past the face by the keyway's depth less 2 mm, still. A key stuck short is searched again once, then refused, held where it is.
- **v1 (rolled while approaching):** 241 against the old layer's 397 over 5 runs; false seats 105 against 248, refused 343. The layer located the face well: within 0.4 mm (median) of the true one on P4 seed 51 at 1 N·m. But rolling while approaching lagged the arm into touches in free air, and two bugs cost more:
  - the face found rolled and the depth measured unrolled differ by up to 2 mm, so keys on the mouth read as entered;
  - a refused force-aware layer left its reference at the press's lead, and the policy then shoved 13 keys home at 15-28 N.
- **v2 (roll in place, then a slow re-touch at zero roll; a refused key held where it is):** 212 against 629 over 8 runs; false seats 308 against 397.
  - The env's axial outer loop had integrated while the wrist turned with the reference standing.
  - That put the hand about 7-10 mm ahead of its reference, and the rolled approach then read the arm's own lag as a touch: 80 of 144 faces were located more than 1.5 mm short on P4 seed 52 at 2 N·m.
- **v3 (that loop off during the roll):** 222 against 351 over 4 checks; false seats 112 against 137.
  - The hand still runs 8-10 mm ahead while the wrist rolls in place.
  - The jaw centre is on Joint6's axis (`arm_kin`, within 1.1 mm), and forward kinematics of the true joints agrees with the true tip to 0.1 mm rolled or not. So this is the policies answering a wrist 45 deg past anything they trained on (their search wiggles ±8 deg), not geometry.
  - The parent, never fine-tuned on keys, is the exception: [64 against 70](#/week/10/run/20261117T141224_leg_eval_seed1_s1c_loc3_parent_8200_val_f1), p 0.53. Its false seats fell from 32 to 26 and its damaged keys from 41 to 0.

**The keyway hardly constrains the key's roll** (`KeyLock` alone on the CPU, a key pushed in gradually, middling edge breaks and play):

| roll off the keyway | push-back at 2 mm in | at 6 mm in |
|---|---|---|
| 0-4 deg | 0 N | 0.4 N |
| 6 deg | 1.6 N | 8.9 N |
| 8-10 deg | 3.2 N | 17 N |
| 45 deg | 16 N | 43 N |

Points that cross the edge break switch to the wall's sideways normal, and each point is capped at 2 N with a 600 N/m share. So a key rolled 4 deg goes to the floor freely, and at 6-10 deg the 8-12 N press sinks it several millimetres. The wards' clearance is about 1.3 deg (play over ward length). A stiff-wristed real key that far off would wedge on the edge break.

In the old-layer runs the commanded roll at entry was more than 2 deg off the keyway's in 62% of true seats and 75% of false ones (medians 2.7 and 3.7 deg). So the simulated insertion's successes and its jams both rest on this.

| policy | N·m | old layer | located v1 | located v2 | located v3 |
|---|---|---|---|---|---|
| S1b P4 seed 51 (`8799`) | 1 | [101 / 31](#/week/10/run/20261117T131834_leg_eval_seed1_s1c_diag_P4_s51_8799_val_f1) | [65 / 19](#/week/10/run/20261117T134202_leg_eval_seed1_s1c_loc_P4_s51_8799_val_f1) | [69 / 16](#/week/10/run/20261117T135144_leg_eval_seed1_s1c_loc2_P4_s51_8799_val_f1) | [75 / 18](#/week/10/run/20261117T140945_leg_eval_seed1_s1c_loc3_P4_s51_8799_val_f1) |
| S1b P4 seed 51 (`8799`) | 2 | [38 / 101](#/week/10/run/20261117T131720_leg_eval_seed1_s1c_diag_P4_s51_8799_val_f2) | [14 / 31](#/week/10/run/20261117T134435_leg_eval_seed1_s1c_loc_P4_s51_8799_val_f2) | [17 / 27](#/week/10/run/20261117T135217_leg_eval_seed1_s1c_loc2_P4_s51_8799_val_f2) |  |
| S1b P4 seed 52 (`8600`) | 1 | [93 / 37](#/week/10/run/20261117T131949_leg_eval_seed1_s1c_diag_P4_s52_8600_val_f1) | [48 / 26](#/week/10/run/20261117T134518_leg_eval_seed1_s1c_loc_P4_s52_8600_val_f1) | [35 / 40](#/week/10/run/20261117T135406_leg_eval_seed1_s1c_loc2_P4_s52_8600_val_f1) |  |
| S1b P4 seed 52 (`8600`) | 2 | [87 / 45](#/week/10/run/20261117T132104_leg_eval_seed1_s1c_diag_P4_s52_8600_val_f2) | [55 / 12](#/week/10/run/20261117T134659_leg_eval_seed1_s1c_loc_P4_s52_8600_val_f2) | [13 / 59](#/week/10/run/20261117T135645_leg_eval_seed1_s1c_loc2_P4_s52_8600_val_f2) | [21 / 55](#/week/10/run/20261117T140625_leg_eval_seed1_s1c_loc3_P4_s52_8600_val_f2) |
| S1b P4 seed 53 (`8799`) | 1 | [78 / 34](#/week/10/run/20261117T132219_leg_eval_seed1_s1c_diag_P4_s53_8799_val_f1) | [59 / 17](#/week/10/run/20261117T134931_leg_eval_seed1_s1c_loc_P4_s53_8799_val_f1) | [34 / 23](#/week/10/run/20261117T135732_leg_eval_seed1_s1c_loc2_P4_s53_8799_val_f1) |  |
| S1b P4 seed 53 (`8799`) | 2 | [76 / 50](#/week/10/run/20261117T132334_leg_eval_seed1_s1c_diag_P4_s53_8799_val_f2) |  | [19 / 50](#/week/10/run/20261117T135953_leg_eval_seed1_s1c_loc2_P4_s53_8799_val_f2) |  |
| S1b P3 seed 51 (`8799`) | 1 | [80 / 42](#/week/10/run/20261117T132449_leg_eval_seed1_s1c_diag_P3_s51_8799_val_f1) |  | [20 / 45](#/week/10/run/20261117T140256_leg_eval_seed1_s1c_loc2_P3_s51_8799_val_f1) |  |
| S1b P3 seed 51 (`8799`) | 2 | [76 / 57](#/week/10/run/20261117T132514_leg_eval_seed1_s1c_diag_P3_s51_8799_val_f2) |  | [5 / 48](#/week/10/run/20261117T140347_leg_eval_seed1_s1c_loc2_P3_s51_8799_val_f2) |  |
| S1b P3 seed 52 (`8400`) | 1 | [93 / 29](#/week/10/run/20261117T132728_leg_eval_seed1_s1c_diag_P3_s52_8400_val_f1) |  |  | [62 / 13](#/week/10/run/20261117T141011_leg_eval_seed1_s1c_loc3_P3_s52_8400_val_f1) |
| S1b P3 seed 52 (`8400`) | 2 | [81 / 53](#/week/10/run/20261117T132753_leg_eval_seed1_s1c_diag_P3_s52_8400_val_f2) |  |  |  |
| S1b P3 seed 53 (`8799`) | 1 | [91 / 38](#/week/10/run/20261117T132818_leg_eval_seed1_s1c_diag_P3_s53_8799_val_f1) |  |  |  |
| S1b P3 seed 53 (`8799`) | 2 | [95 / 38](#/week/10/run/20261117T132950_leg_eval_seed1_s1c_diag_P3_s53_8799_val_f2) |  |  |  |
| S1b SP seed 51 (`8600`) | 1 | [84 / 22](#/week/10/run/20261117T133035_leg_eval_seed1_s1c_diag_SP_s51_8600_val_f1) |  |  |  |
| S1b SP seed 51 (`8600`) | 2 | [88 / 28](#/week/10/run/20261117T133124_leg_eval_seed1_s1c_diag_SP_s51_8600_val_f2) |  |  |  |
| S1b SP seed 52 (`8400`) | 1 | [73 / 35](#/week/10/run/20261117T133254_leg_eval_seed1_s1c_diag_SP_s52_8400_val_f1) |  |  |  |
| S1b SP seed 52 (`8400`) | 2 | [85 / 30](#/week/10/run/20261117T133430_leg_eval_seed1_s1c_diag_SP_s52_8400_val_f2) |  |  |  |
| S1b SP seed 53 (`8799`) | 1 | [79 / 37](#/week/10/run/20261117T133549_leg_eval_seed1_s1c_diag_SP_s53_8799_val_f1) |  |  |  |
| S1b SP seed 53 (`8799`) | 2 | [85 / 37](#/week/10/run/20261117T133756_leg_eval_seed1_s1c_diag_SP_s53_8799_val_f2) |  |  |  |
| parent `ki6_force_8200` | 1 | [70 / 32](#/week/10/run/20261117T133932_leg_eval_seed1_s1c_diag_parent_8200_val_f1) |  |  | [64 / 26](#/week/10/run/20261117T141224_leg_eval_seed1_s1c_loc3_parent_8200_val_f1) |
| parent `ki6_force_8200` | 2 | [85 / 17](#/week/10/run/20261117T134026_leg_eval_seed1_s1c_diag_parent_8200_val_f2) |  |  |  |


Runs above: [old layer](#/week/10/run/20261117T131834_leg_eval_seed1_s1c_diag_P4_s51_8799_val_f1) (`s1c_diag_*`, 20), located v1 (`s1c_loc_*`, 5; the other 15 cancelled before starting), v2 (`s1c_loc2_*`, 8; 12 cancelled), v3 (`s1c_loc3_*`, 4 checks). All recorded.

**What this does and does not show:**
- The false seats are the layer's: it cannot tell a straight-in key at the floor from one jammed short, and it is built to finish both.
- The keyway model, though, lets keys in at rolls a real lock would refuse, which is where both kinds come from. How many false seats a real lock would produce is not shown.
- The located test is **not validated**: the policies were trained with the old layer. Its rolled first touch is out of their training (v3), and the contact's softness at ward-only contact undermines a rolled touch too.
- S1c's own test set, [`keyinsert_tools_v4`](../manifests/keyinsert_tools_v4.json) (the six held-out keys and v4 on 48 new locks), is written and **not scored**: nothing passed validation.

**Next, in order:**
1. A measurement (Lukas): does the PLA key enter the cam lock rolled 2, 4 and 8 deg off, and how deep does a misrolled key go under 10 N?
2. A contact revision to match it: the edge break keeps a crossing point's axial normal, and a ward-only contact is as stiff as a full-face one.
3. Only then a layer trained in the loop, if keys stay in the paper.

Finding: [F-292](../findings.md#f-292).

### 2026-11-18 · S4 started: T1's held mechanisms on the deployable models, four arms from `66600`, stated before scoring

Lukas (17 Nov): "do 2 as well". S4 of [the tool paper plan](../../docs/tool_paper_plan.md), H1's E1 on T1. F-105 to
F-110 were on the 50 Hz arm with ideal legs; this repeats them through the D1 link on `66600`'s legs.

**The port** (code, 2026-11-18):
- `run_unifp_train.py`:
  - lets the arm link run on `--task mechanism`. The mechanism env keeps UniFP's actions and calls the base env's step, link and observation, as `66600` trained.
  - adds `--mech_tilt_stop_deg`. The episode ends as a failure past the runner's 15 deg stop, counted apart from falls (`mech_env._get_dones`, `mech_eval`'s `tilt_stop`).
  - adds `--mech_set s4_validation|s4_test`.
- New frozen sets, [`results/manifests/mech_t1_v2.json`](../manifests/mech_t1_v2.json) (`mech_cfg.s4_manifest`, written once):
  - placements are drawn on new streams across the old grids' span: height 0.20-0.60 m, bearing ±0.55 rad, reach 0.42-0.45 m;
  - validation is 12 placements at the development conditions, test 16 at the held-out ones;
  - both sets use the four kinds at 10-80 N, so 384 and 512 episodes;
  - the development grid and the 25 Sep test set are spent.
- `mech_eval` now scores the force budget: the drive on the handle above 80 N (the law's cap) on 0.1 s running, from the grasp to the end. It reports the episodes opened within it.

**The stack, `66600`'s own** (its `armft6b_s1` `run.json`):
- the physical payload;
- `unitree_geared` legs drawn as it trained: armature 0-0.002, friction 0-2 N·m, delay 1-4 physics steps;
- the D1 link report-locked, the F-250 lag drawn, the Follower;
- arm step -30, wrist -0.3;
- plus the 15 deg tilt stop.

T1 resets from UniFP's standing pose as `66600` trained, not the key tasks' dog stances: a departure from the plan's defaults. Scoring uses `66600`'s probe legs (armature 2e-4 / 7e-4, delay 3-4 steps) at 1 and 2 N·m, as S1. Wrappers: `logs/unifp_train/s4/recipe.sh`, `mev.sh`, `s1tools/queue_s4.sh`.

**The four arms** (one parent, one budget):

| arm | what | training |
|---|---|---|
| (i) | the layer in the loop: F-108's law v1 (PI on the handle's lag, capped at 80 N) writing the force command, its force-tracking reward | `--force_law --law_variant v1`, 2,000 iterations from `66600`, seeds 81-83 |
| (ii) | the layer zero-shot on a policy not trained with it: `66600` (F-106's design) | none |
| (iii) | outcome-trained, the goal only | 2,000 iterations from `66600`, seeds 81-83 |
| (iv) | the position script: `66600` given the sliding reference as its goal | none |

(i) and (iii) differ only in `--force_law`. The curriculum starts at 15 N for both. Also reported: (iii)'s selected checkpoints with the law zero-shot, as a second (ii).

**Selection rule, stated before any S4 checkpoint is scored:** of `68000`, `68200`, `68400` and `68599`, on the validation set at 1 and 2 N·m (`s1tools/s4_select.py`):
1. fewest falls plus tilt stops;
2. then most opened within the budget;
3. then the earliest.

The test set is scored once, after selection, for all four arms. The measures are opened, opened within the budget, budget violations, peak drive, stops and torn grasps.

**Smoke, on validation at 1 N·m** (not a selection, these arms are not trained):

| arm | opened (of 384) | opened in budget | over budget | tilt stops | falls | peak drive median |
|---|---|---|---|---|---|---|
| (ii) [`66600` + law](#/week/10/run/20261117T132539_mech_eval_seed1_s4_66600_law_val_f1) | 76 | 76 | 0 | 3 | 0 | 21.9 N |
| (iv) [`66600`, goal only](#/week/10/run/20261117T132704_mech_eval_seed1_s4_66600_goal_val_f1) | 9 | 9 | 0 | 0 | 0 | 14.3 N |

`66600` never trained on a mechanism. With the law it opens what needs up to 10-20 N, with the arm saturated (the arm load's 95th percentile 1.0). Given only the goal, it opens 9 latches.

**Training started** 00:32 on lane t1:
- [`s4_law_s81`](#/week/10/run/20261117T133229_train_seed81_s4_law_s81) runs at 6.5 s an iteration beside three evaluation lanes.
- At the start a quarter of mechanism episodes end on the tilt stop, against `66600`'s 15 deg.
- The outcome-trained runs moved to lane t2 at 01:23, once S1c's evaluations were done. Each seed's two arms now train side by side (9.7 GB with one evaluation lane), with an equal budget in iterations. `s4_law_s81` ran its first 600 iterations alone.

### 2026-11-18 · S5: `ki6_force_8200` exported to numpy, and a runner that matches Isaac to ~1e-6; the replay gate's thresholds, stated before scoring

Lukas (17 Nov): "I will trust your judgement for which policy for 1, and there is no additional code on the other pc".

**The policy: `ki6_force_8200`.**
- The plan names it; S1's P4 did not win (F-289).
- It trained on v4, the dog's PLA key.
- It is force-aware, which E5's comparison against the scripted press needs.
- Its family kept 31-72 of 96 on heavy legs in S2, where the position-only parent's family kept 0-25 (F-291).
- Position-only twins scored as high on the nominal legs, but not from this lineage's robustness.

**Built** (with a subagent, checked here; nothing runs on the dog yet):
- `demos/combiner_key/robot/insert_policy.py`, numpy only:
  - the task layer, line for line, in both seated tests;
  - the measured tip from the reported arm joints, the outer loop, goal to jaw;
  - `arm_kin`'s IK with the env's polish and sag model;
  - the insertion's frame and its joint targets;
  - an `InsertRunner` whose every input is explicit (the lock estimate, the believed key, the start depth, the press, the base pose), so the dog program can supply them from perception;
  - a log of the layer's decisions.
  
  It reuses `LegPolicy` and `LegObservation`.
- `export_policy.py --task insert --train_run`: [`checkpoints/insert_force_ki6_8200.npz`](../../checkpoints/insert_force_ki6_8200.npz). Its meta records the layer's and the env's constants, the v4 key, the arm limits and what it trained with. Numpy against torch on 64 random stacks: actions 2.8e-6, estimates 1.7e-6.
- `--insert_shadow_dump N` on `leg_eval --key_insert` (`unifp_train/keyinsert_shadow.py`) writes every raw input and output for N episodes. `demos/combiner_key/robot/check_insert.py` replays the dump through the runner from each reset.
- `tests/test_insert_policy.py`: 14 tests, the numpy layer against the torch one in both modes, and IK/FK. 31 with `tests.test_keylock`, all passing here.

**Parity** ([the dump run](#/week/10/run/20261117T135443_leg_eval_seed1_ki6_force_8200_f1_s5shadow): the frozen 96 at 1 N·m from the dog's stance, the first 8 episodes, 4,875 steps; rerun here from the dump):

| quantity | max abs difference |
|---|---|
| actions | 2.9e-6 |
| leg targets | 7.2e-8 rad |
| arm targets, IK reference | 1.6e-6 rad |
| frames | 2.2e-6 (the commands block, the miss ×10 in float32 on the GPU); other blocks ≤ 2.8e-8 |
| measured depth / lateral | 1.8e-7 / 1.1e-7 m |
| layer: reference, roll, timers | ≤ 1.4e-7 m, 1.5e-8 rad, 0 |
| layer phase and decisions | equal on every step |

F-171's key policy got 1.1e-6 on actions; this is the same order.

The one mismatch is benign in the old seated test, which ki6 uses: the torch layer zeroes three timers (`held_for`, `still_for`, `still_any`) in envs outside the searching or seating phase whenever another env is in it. They agree wherever the layer reads them. In the located test's relocation path a single robot's timer could differ from training's.

**The dump is read-only, and the frozen 96 moves by rounding:**
- [The same evaluation without the dump](#/week/10/run/20261117T141834_leg_eval_seed1_ki6_force_8200_f1_repro1118) is identical in all 96 episodes: 72 inside 15 deg, 22 false seats.
- F-284's 13 Nov run of the same command got [69](#/week/9/run/20261112T221314_leg_eval_seed1_ki6_force_8200_f1), 26 false seats.
- Their traces agree from the first step to 6e-8 m in the measured depth (S1's per-environment key code orders the floats differently), and part from about 1 s in.
- So the frozen 96 moves by about 3 of 96 under a rounding-level change. F-284's 69-81 should be read with that spread.

**The replay gate (gate4's pattern), thresholds stated 2026-11-18 before any scoring:**
- **Frames:** each of `66600`'s four recorded stands (`results/week_08/runs/20261106T1{40642,41120,41455,41945}_reach_stand_live/policy_rows.npz`).
- **Body and legs:** the legs, the body's attitude and the IMU as recorded.
- **Arm:** the arm closed through the real `Follower`, 9 Hz quantised reports and the 77 ms lag.
- **Lock estimate:** placed where the frozen set's box puts it against the dog's arm-out stance.
- **The approach only,** in free space: no face is modelled.

Pass requires all four, on all four runs:
1. J1-J5's target lead over the report ≤ 15 deg from 1 s on (F-256's).
2. Leg targets ≤ 1.2 rad from the stand in the first second, ≤ 0.9 rad after (F-256's).
3. The IK reference solves on every call.
4. The layer feels no touch while its reference is short of the estimated face: no touch in free air on the dog's own body motion (F-292's failure).

Reported without a pass mark: the hand's lag along the axis, J2's torque share, the layer's decisions.

**The gate's result: pass, on every stand and placement.**
- Command: `python3 demos/combiner_key/robot/insert_gate.py --policy checkpoints/insert_force_ki6_8200.npz --out logs/unifp_train/s5gate --workers 6`, the CPU, 19 s, rerun here with the same table.
- Placements: rows 0, 24, 48, 72 of the frozen 96's plan and the median (door 0.50-0.52 m ahead, 0.10-0.12 m to the side, 0.445-0.45 m high, yaw -0.013 to 0.039 rad); estimate exact; start 17.5 mm short; 8 N press; the old seated test.

| stand | J1-J5 lead from 1 s (≤ 15 deg) | leg targets, first s / after (≤ 1.2 / 0.9 rad) | IK failures | touches short of the face | pass |
|---|---|---|---|---|---|
| 140642 | 1.2-1.5 deg | 0.39 / 0.25-0.27 | 0 of 101 | 0 | 5 of 5 |
| 141120 | 3.0-3.6 deg | 0.38-0.39 / 0.29-0.30 | 0 of 101 | 0 | 5 of 5 |
| 141455 | 1.6-1.7 deg | 0.38-0.39 / 0.22-0.24 | 0 of 101 | 0 | 5 of 5 |
| 141945 | 1.5-1.6 deg | 0.39 / 0.23-0.25 | 0 of 101 | 0 | 5 of 5 |

- The reference passes the estimated face at 3.48 s every time. No touch is felt in the 10 s; the layer's only decisions are its start.
- With the recorded yaw held at zero, all 20 also pass (leads 1.7-3.5 deg).
- The base is held at x = y = 0, its height from the support feet (0.287-0.295 m), roll and pitch as recorded, yaw as recorded less the first frame's.
- The legs act open loop on `66600`'s recorded legs, as gate4 does. The arm is closed through the `Follower`, the reports and the lag, from the runner's own start pose.
- `tests/test_insert_policy.py` has four gate tests (18 pass, 1 skipped on the system Python).

**Reported without a pass mark:**
- **The hand runs ahead of its reference:** by -3.3 to -5.0 mm (median) along the axis by the reports, from the policy's arm residual. The Isaac dump shows the same (-1.3 to -6.2 mm, 5-15 mm ahead at the layer's start), as S1c found in its rolled approach. A touch by lag then fires only once the hand has given up that lead plus 3 mm. That may add to the frozen set's touch error, not tested.
- **Joint2's static gravity share at the reports:** 0.87-0.92 of its 3.33 N·m (URDF masses, the wrist payload, the v4 key; the limit is URDF-unverified). The S1 log's 2.66 N·m (0.80) was the arm and RealSense at the insertion's start, without the key; same order. It leaves 8-13% of Joint2 for anything else at this reach.

**What the gate does not show:**
- the press, the search or the seat (no face is modelled);
- the legs answering this policy (they replay `66600`'s);
- the dog's real lock estimate (it is placed exactly);
- anything run on the dog.

Finding: [F-293](../findings.md#f-293). `docs/tool_paper_plan.md`'s "Results so far" note now lists F-292 and F-293.

**Not done:** the box raised to the trained 0.45 m.
- The dog program has no box-height setting. The lock's height comes from `BoxGeometry` (0.4175 m above the box's root) and the box root's height from the tag and lidar fix.
- Training's key hold is at 0.45 m (`leg_cfg.KEYHOLD_AXIS_HEIGHT_M`).
- So it is a 32.5 mm spacer under the box, or a geometry option, which is Lukas's call.

### 2026-11-18 · S4 seed 81, arm (i): trained, and `68400` selected on validation

**The training:** [`s4_law_s81`](#/week/10/run/20261117T133229_train_seed81_s4_law_s81), 00:32-03:25, exit 0, 2,000 iterations from `66600`.
- Command: `logs/unifp_train/s1tools/train.sh s4_law_s81 --task mechanism --force_law --law_variant v1 --seed 81 --iterations 2000 --num_envs 2048 --resume_from checkpoints/unifp_go2d1_isaaclab_sticky_model_66600.pt` plus the recipe's legs, link and tilt stop (the S4 entry above).
- The force curriculum's ceiling reached 80 N at iteration 67,909.
- Training-time `Mechanism/tilt_stopped_fraction` averaged 0.076 over the last 200 logged iterations. That is sampled at reset, not the frozen evaluation.

**Validation, each candidate at 1 and 2 N·m** (`mev.sh <name> <title> <ckpt> s4_validation --force_law --leg_friction_range F F`; 384 episodes each, 768 a candidate):

| checkpoint | f | opened | in budget | over budget | tilt stops | falls | torn | peak drive median | buttons / doors / drawers / latches opened |
|---|---|---|---|---|---|---|---|---|---|
| [`68000`](#/week/10/run/20261117T153224_mech_eval_seed1_s4_law_s81_68000_val_f1) | 1 | 283 | 267 | 16 | 0 | 0 | 0 | 44.5 N | 23 / 74 / 90 / 96 |
| [`68000`](#/week/10/run/20261117T153440_mech_eval_seed1_s4_law_s81_68000_val_f2) | 2 | 286 | 272 | 14 | 0 | 0 | 0 | 48.7 N | 21 / 79 / 90 / 96 |
| [`68200`](#/week/10/run/20261117T155011_mech_eval_seed1_s4_law_s81_68200_val_f1) | 1 | 272 | 245 | 27 | 0 | 0 | 0 | 43.0 N | 26 / 62 / 88 / 96 |
| [`68200`](#/week/10/run/20261117T155224_mech_eval_seed1_s4_law_s81_68200_val_f2) | 2 | 294 | 280 | 14 | 0 | 0 | 0 | 44.3 N | 24 / 81 / 93 / 96 |
| [`68400`](#/week/10/run/20261117T160754_mech_eval_seed1_s4_law_s81_68400_val_f1) | 1 | 378 | 304 | 74 | 0 | 0 | 22 | 71.4 N | 91 / 96 / 95 / 96 |
| [`68400`](#/week/10/run/20261117T161014_mech_eval_seed1_s4_law_s81_68400_val_f2) | 2 | 362 | 316 | 46 | 0 | 0 | 3 | 60.2 N | 75 / 96 / 95 / 96 |
| [`68599`](#/week/10/run/20261117T162834_mech_eval_seed1_s4_law_s81_68599_val_f1) | 1 | 330 | 270 | 60 | 0 | 0 | 0 | 64.2 N | 50 / 91 / 93 / 96 |
| [`68599`](#/week/10/run/20261117T163052_mech_eval_seed1_s4_law_s81_68599_val_f2) | 2 | 296 | 264 | 32 | 1 | 0 | 0 | 51.8 N | 29 / 85 / 86 / 96 |

**Selected by the stated rule: `68400`.**
- Stops: 0, 0, 0, 1.
- Then opened in budget: 539, 525, **620**, 534 of 768.
- Output: `s1tools/s4_select.py s4_law_s81` → `s1tools/s4_law_s81_selected.json`.

**What the rule does not price.** `68400` is also the candidate that tore the most handles out of the claw: 25 (22 of them buttons at 1 N·m, the grasp spring past `GRIP_N` = 150 N). It also had the most episodes over budget, 120 of 768.
- Torn grasps end the episode but are not stops in the rule as stated, so the selection stands.
- The test set reports them beside the opened counts.
- Buttons, at 23 to 91 opened between neighbouring checkpoints, are where the candidates differ most. Doors, drawers and latches stay near their ceilings once the curriculum reaches 80 N.

**Not shown yet:** any comparison between arms. (iii)'s seed 81 is still training; at iteration 68,038 its curriculum was at 40 N, where (i)'s had reached 80 N by 67,909. Seed 82's (i), `s4_law_s82`, started at 03:25 on t1. Its (iii) follows `s4_out_s81` on t2.

**The untrained arms on the test set, scored early, not read.** (ii) `66600` with the law and (iv) `66600` given the goal only have nothing to select. They ran on the idle evaluation lane at 03:35-03:44, at 1 and 2 N·m (`mev.sh s4_66600_<law|goal>_test_fF ... s4_test [--force_law] --leg_friction_range F F`), and are recorded.
- Their numbers are not looked at until all six trained runs are selected, so the test set is still read once.

### 2026-11-18 · S4 seed 81, arm (iii): trained, and `68200` selected on validation

**The training:** [`s4_out_s81`](#/week/10/run/20261117T142403_train_seed81_s4_out_s81), 01:23-04:18, exit 0, 2,000 iterations from `66600`.
- Command: the same as (i) without `--force_law --law_variant v1`: `train.sh s4_out_s81 --task mechanism --seed 81 --iterations 2000 --num_envs 2048 --resume_from checkpoints/unifp_go2d1_isaaclab_sticky_model_66600.pt` plus the recipe.
- The force curriculum's ceiling climbed to 40 N by iteration 67,600 and stayed there to the end. (i)'s reached 80 N.
- Training-time tilt stops averaged 0.062 over the last 200 logged iterations (sampled at reset).

**Validation** (`mev.sh ... s4_validation --leg_friction_range F F`, no law):

| checkpoint | f | opened | in budget | over budget | tilt stops | falls | torn | peak drive median | buttons / doors / drawers / latches opened |
|---|---|---|---|---|---|---|---|---|---|
| [`68000`](#/week/10/run/20261117T162343_mech_eval_seed1_s4_out_s81_68000_val_f1) | 1 | 94 | 94 | 0 | 0 | 0 | 0 | 32.2 N | 9 / 10 / 21 / 54 |
| [`68000`](#/week/10/run/20261117T162605_mech_eval_seed1_s4_out_s81_68000_val_f2) | 2 | 65 | 65 | 0 | 0 | 0 | 0 | 29.5 N | 7 / 5 / 8 / 45 |
| [`68200`](#/week/10/run/20261117T164431_mech_eval_seed1_s4_out_s81_68200_val_f1) | 1 | 91 | 91 | 0 | 0 | 0 | 0 | 30.3 N | 22 / 5 / 21 / 43 |
| [`68200`](#/week/10/run/20261117T164648_mech_eval_seed1_s4_out_s81_68200_val_f2) | 2 | 71 | 71 | 0 | 0 | 0 | 0 | 28.0 N | 19 / 4 / 15 / 33 |
| [`68400`](#/week/10/run/20261117T170216_mech_eval_seed1_s4_out_s81_68400_val_f1) | 1 | 67 | 67 | 0 | 11 | 0 | 0 | 33.3 N | 6 / 3 / 9 / 49 |
| [`68400`](#/week/10/run/20261117T170429_mech_eval_seed1_s4_out_s81_68400_val_f2) | 2 | 37 | 37 | 0 | 0 | 0 | 0 | 25.6 N | 6 / 1 / 2 / 28 |
| [`68599`](#/week/10/run/20261117T171959_mech_eval_seed1_s4_out_s81_68599_val_f1) | 1 | 52 | 52 | 0 | 0 | 0 | 0 | 25.9 N | 15 / 0 / 4 / 33 |
| [`68599`](#/week/10/run/20261117T172214_mech_eval_seed1_s4_out_s81_68599_val_f2) | 2 | 44 | 44 | 0 | 0 | 0 | 0 | 25.5 N | 10 / 3 / 5 / 26 |

**Selected by the stated rule: `68200`.**
- Stops: 0, 0, 11, 0.
- Then opened in budget: 159, **162**, 96 of 768.
- Output: `s1tools/s4_select.py s4_out_s81` → `s4_out_s81_selected.json`.

**What it shows on seed 81, on validation only:**
- Goal-only training opens fewer mechanisms the longer it trains past `68200`. It never goes over the budget, and its median peak drive is 26-33 N.
- It opens fewer than (i)'s selected checkpoint (162 against 620 within budget) on the set both were selected on.
- That is one seed, on the selection set, so it is not E1's result; the test set is.

Seed 82's (iii), `s4_out_s82`, started at 04:18 on t2.

Seed 81's selections are final, so its test scorings are queued behind the validation jobs (`spool_e/e_s4_91_test_*`). They are read only with the rest:
- (i) `68400` with the law;
- (iii) `68200` goal only;
- (iii) `68200` with the law zero-shot.

### 2026-11-18 · S4 seed 82, arm (i): trained, and `68400` selected on validation

**The training:** [`s4_law_s82`](#/week/10/run/20261117T162539_train_seed82_s4_law_s82), 03:25-06:26, exit 0.
- Command: (i)'s with `--seed 82`.
- The ceiling reached 80 N at iteration 67,672.
- Training-time tilt stops averaged 0.053 over the last 200 logged iterations.

**Validation** (as seed 81's):

| checkpoint | f | opened | in budget | over budget | tilt stops | falls | torn | peak drive median | buttons / doors / drawers / latches opened |
|---|---|---|---|---|---|---|---|---|---|
| [`68000`](#/week/10/run/20261117T183243_mech_eval_seed1_s4_law_s82_68000_val_f1) | 1 | 231 | 231 | 0 | 1 | 0 | 0 | 44.6 N | 36 / 50 / 76 / 69 |
| [`68000`](#/week/10/run/20261117T183457_mech_eval_seed1_s4_law_s82_68000_val_f2) | 2 | 235 | 235 | 0 | 6 | 0 | 0 | 42.3 N | 37 / 59 / 65 / 74 |
| [`68200`](#/week/10/run/20261117T185028_mech_eval_seed1_s4_law_s82_68200_val_f1) | 1 | 272 | 268 | 4 | 11 | 0 | 0 | 42.8 N | 28 / 72 / 81 / 91 |
| [`68200`](#/week/10/run/20261117T185246_mech_eval_seed1_s4_law_s82_68200_val_f2) | 2 | 287 | 272 | 15 | 21 | 0 | 0 | 44.2 N | 27 / 75 / 95 / 90 |
| [`68400`](#/week/10/run/20261117T190816_mech_eval_seed1_s4_law_s82_68400_val_f1) | 1 | 293 | 291 | 2 | 1 | 0 | 0 | 40.8 N | 38 / 72 / 87 / 96 |
| [`68400`](#/week/10/run/20261117T191031_mech_eval_seed1_s4_law_s82_68400_val_f2) | 2 | 292 | 288 | 4 | 4 | 0 | 0 | 39.7 N | 28 / 78 / 93 / 93 |
| [`68599`](#/week/10/run/20261117T192635_mech_eval_seed1_s4_law_s82_68599_val_f1) | 1 | 274 | 265 | 9 | 16 | 0 | 0 | 45.3 N | 39 / 60 / 80 / 95 |
| [`68599`](#/week/10/run/20261117T192852_mech_eval_seed1_s4_law_s82_68599_val_f2) | 2 | 293 | 267 | 26 | 19 | 0 | 0 | 46.9 N | 31 / 73 / 93 / 96 |

**Selected by the stated rule: `68400`.**
- Stops: 7, 32, **5**, 35.
- It also opens the most within the budget, 579 of 768.

**Differences from seed 81:**
- This seed tilts: 5-35 stops a candidate, where seed 81's (i) had 0-1.
- It does not tear: 0 torn, where seed 81's `68400` tore 25.
- Buttons stay at 27-39 opened, where seed 81's `68400` opened 91 and 75.
- So seed 81's button count and its tears are one seed's, not the arm's, so far.

Its test scoring ((i) `68400` with the law) is queued behind the validation jobs, unread. Seed 83's (i), `s4_law_s83`, started at 06:26 on t1.

### 2026-11-18 · S4 seed 82, arm (iii): trained, and `68599` selected on validation

**The training:** [`s4_out_s82`](#/week/10/run/20261117T171850_train_seed82_s4_out_s82), 04:18-07:15, exit 0.
- Command: (iii)'s with `--seed 82`.
- The ceiling stopped at 40 N from iteration 68,000, as seed 81's did.
- Training-time tilt stops averaged 0.017 over the last 200 logged iterations.

**Validation:**

| checkpoint | f | opened | in budget | over budget | tilt stops | falls | torn | peak drive median | buttons / doors / drawers / latches opened |
|---|---|---|---|---|---|---|---|---|---|
| [`68000`](#/week/10/run/20261117T192159_mech_eval_seed1_s4_out_s82_68000_val_f1) | 1 | 70 | 70 | 0 | 0 | 0 | 0 | 25.6 N | 2 / 16 / 27 / 25 |
| [`68000`](#/week/10/run/20261117T192412_mech_eval_seed1_s4_out_s82_68000_val_f2) | 2 | 74 | 74 | 0 | 0 | 0 | 0 | 25.8 N | 0 / 20 / 30 / 24 |
| [`68200`](#/week/10/run/20261117T194156_mech_eval_seed1_s4_out_s82_68200_val_f1) | 1 | 77 | 77 | 0 | 0 | 0 | 0 | 25.2 N | 14 / 17 / 22 / 24 |
| [`68200`](#/week/10/run/20261117T194411_mech_eval_seed1_s4_out_s82_68200_val_f2) | 2 | 72 | 72 | 0 | 0 | 0 | 0 | 24.6 N | 13 / 14 / 21 / 24 |
| [`68400`](#/week/10/run/20261117T195838_mech_eval_seed1_s4_out_s82_68400_val_f1) | 1 | 66 | 66 | 0 | 0 | 0 | 0 | 23.2 N | 8 / 11 / 23 / 24 |
| [`68400`](#/week/10/run/20261117T200050_mech_eval_seed1_s4_out_s82_68400_val_f2) | 2 | 60 | 60 | 0 | 0 | 0 | 0 | 23.7 N | 2 / 11 / 23 / 24 |
| [`68599`](#/week/10/run/20261117T201619_mech_eval_seed1_s4_out_s82_68599_val_f1) | 1 | 87 | 87 | 0 | 0 | 0 | 0 | 27.1 N | 17 / 13 / 21 / 36 |
| [`68599`](#/week/10/run/20261117T201833_mech_eval_seed1_s4_out_s82_68599_val_f2) | 2 | 64 | 64 | 0 | 0 | 0 | 0 | 25.1 N | 7 / 9 / 19 / 29 |

**Selected by the stated rule: `68599`.**
- Stops: 0 for every candidate.
- Then opened in budget: 144, 149, 126, **151** of 768.
- The four are within 25 of each other, so the pick among them is nearly arbitrary.

**On validation, seed 82:** goal-only again never goes over the budget, with median peaks of 23-27 N, and opens 60-87 of 384. That is about half of seed 81's (iii).

Its test scorings ((iii) `68599`, and `68599` with the law zero-shot) are queued, unread. Seed 83's (iii), `s4_out_s83`, started at 07:15 on t2.

### 2026-11-18 · S4 seed 83, arm (i): trained, and `68000` selected on validation

**The training:** [`s4_law_s83`](#/week/10/run/20261117T192610_train_seed83_s4_law_s83), 06:26-09:23, exit 0.
- Command: (i)'s with `--seed 83`.
- The ceiling reached 80 N at iteration 68,044.
- Training-time tilt stops averaged 0.048 over the last 200 logged iterations.

**Validation:**

| checkpoint | f | opened | in budget | over budget | tilt stops | falls | torn | peak drive median | buttons / doors / drawers / latches opened |
|---|---|---|---|---|---|---|---|---|---|
| [`68000`](#/week/10/run/20261117T213013_mech_eval_seed1_s4_law_s83_68000_val_f1) | 1 | 283 | 266 | 17 | 4 | 0 | 0 | 46.5 N | 19 / 86 / 94 / 84 |
| [`68000`](#/week/10/run/20261117T213227_mech_eval_seed1_s4_law_s83_68000_val_f2) | 2 | 245 | 226 | 19 | 1 | 0 | 0 | 41.2 N | 19 / 74 / 80 / 72 |
| [`68200`](#/week/10/run/20261117T214755_mech_eval_seed1_s4_law_s83_68200_val_f1) | 1 | 307 | 282 | 25 | 4 | 0 | 0 | 48.9 N | 22 / 93 / 96 / 96 |
| [`68200`](#/week/10/run/20261117T215011_mech_eval_seed1_s4_law_s83_68200_val_f2) | 2 | 299 | 247 | 52 | 8 | 0 | 0 | 52.3 N | 21 / 90 / 93 / 95 |
| [`68400`](#/week/10/run/20261117T220538_mech_eval_seed1_s4_law_s83_68400_val_f1) | 1 | 274 | 273 | 1 | 5 | 0 | 0 | 37.1 N | 31 / 73 / 88 / 82 |
| [`68400`](#/week/10/run/20261117T220754_mech_eval_seed1_s4_law_s83_68400_val_f2) | 2 | 287 | 278 | 9 | 6 | 0 | 0 | 38.1 N | 29 / 82 / 94 / 82 |
| [`68599`](#/week/10/run/20261117T222503_mech_eval_seed1_s4_law_s83_68599_val_f1) | 1 | 308 | 305 | 3 | 31 | 0 | 0 | 40.1 N | 34 / 85 / 94 / 95 |
| [`68599`](#/week/10/run/20261117T222638_mech_eval_seed1_s4_law_s83_68599_val_f2) | 2 | 307 | 303 | 4 | 31 | 0 | 0 | 43.6 N | 33 / 82 / 96 / 96 |

**Selected by the stated rule: `68000`.**
- Stops: **5**, 12, 11, 62.
- `68000` opens the fewest within the budget, 492 of 768. `68599` opens the most, 608, with 62 tilt stops.
- The rule puts stops first, as stated, so `68000` it is.
- As on seed 82, the law-trained policy tilts more the longer it trains at 80 N.

Its test scoring ((i) `68000` with the law) is queued, unread. Lane t1 is idle: S4 has no more training on it.

### 2026-11-18 · S4 seed 83, arm (iii): trained, and `68200` selected on validation

**The training:** [`s4_out_s83`](#/week/10/run/20261117T201554_train_seed83_s4_out_s83), 07:15-09:47, exit 0.
- Command: (iii)'s with `--seed 83`.
- The ceiling stopped at 35 N from iteration 67,400.
- Training-time tilt stops averaged 0.028 over the last 200 logged iterations.

**Validation:**

| checkpoint | f | opened | in budget | over budget | tilt stops | falls | torn | peak drive median | buttons / doors / drawers / latches opened |
|---|---|---|---|---|---|---|---|---|---|
| [`68000`](#/week/10/run/20261117T221822_mech_eval_seed1_s4_out_s83_68000_val_f1) | 1 | 55 | 55 | 0 | 7 | 0 | 0 | 27.0 N | 17 / 2 / 5 / 31 |
| [`68000`](#/week/10/run/20261117T222037_mech_eval_seed1_s4_out_s83_68000_val_f2) | 2 | 51 | 51 | 0 | 0 | 0 | 0 | 25.3 N | 7 / 2 / 11 / 31 |
| [`68200`](#/week/10/run/20261117T223235_mech_eval_seed1_s4_out_s83_68200_val_f1) | 1 | 71 | 71 | 0 | 0 | 0 | 0 | 29.3 N | 13 / 0 / 18 / 40 |
| [`68200`](#/week/10/run/20261117T223408_mech_eval_seed1_s4_out_s83_68200_val_f2) | 2 | 50 | 50 | 0 | 0 | 0 | 0 | 28.3 N | 5 / 0 / 10 / 35 |
| [`68400`](#/week/10/run/20261117T224050_mech_eval_seed1_s4_out_s83_68400_val_f1) | 1 | 64 | 64 | 0 | 0 | 0 | 0 | 28.3 N | 23 / 0 / 6 / 35 |
| [`68400`](#/week/10/run/20261117T224224_mech_eval_seed1_s4_out_s83_68400_val_f2) | 2 | 57 | 57 | 0 | 0 | 0 | 0 | 26.3 N | 23 / 0 / 6 / 28 |
| [`68599`](#/week/10/run/20261117T224908_mech_eval_seed1_s4_out_s83_68599_val_f1) | 1 | 47 | 47 | 0 | 0 | 0 | 0 | 24.9 N | 8 / 1 / 14 / 24 |
| [`68599`](#/week/10/run/20261117T224942_mech_eval_seed1_s4_out_s83_68599_val_f2) | 2 | 39 | 39 | 0 | 0 | 0 | 0 | 26.7 N | 10 / 0 / 4 / 25 |

**Selected by the stated rule: `68200`.**
- Stops: 7, 0, 0, 0.
- Then opened in budget: 106, **121**, **121**, 86 of 768.
- That is a tie broken by the third rule, the earlier checkpoint.

**On validation:** across three seeds, goal-only never went over the budget and never reached the 80 N ceiling in training (40, 40 and 35 N). Its doors stay at 0-2 opened on this seed.

**All six selected:** (i) `68400`, `68400`, `68000`; (iii) `68200`, `68599`, `68200` (seeds 81, 82, 83).

### 2026-11-18 · S4 on the test set (H1, E1 on T1): the layer in training opens 2,268 of 3,072 within the budget; the next best arm 1,049

The test set, `mech_t1_v2.json`'s 16 held-out placements, four kinds at 10-80 N: 512 episodes at each of 1 and 2 N·m on `66600`'s probe legs, so 1,024 per arm and seed. It was read once, at 09:55, after all six selections. Every scoring is recorded (22 runs):
- (ii) [f1](#/week/10/run/20261117T163945_mech_eval_seed1_s4_66600_law_test_f1), [f2](#/week/10/run/20261117T164203_mech_eval_seed1_s4_66600_law_test_f2);
- (iv) [f1](#/week/10/run/20261117T163512_mech_eval_seed1_s4_66600_goal_test_f1), [f2](#/week/10/run/20261117T163731_mech_eval_seed1_s4_66600_goal_test_f2);
- seed 81:
  - (i) [f1](#/week/10/run/20261117T172530_mech_eval_seed1_s4_law_s81_68400_test_f1), [f2](#/week/10/run/20261117T172748_mech_eval_seed1_s4_law_s81_68400_test_f2);
  - (iii) [f1](#/week/10/run/20261117T173449_mech_eval_seed1_s4_out_s81_68200_test_f1), [f2](#/week/10/run/20261117T173707_mech_eval_seed1_s4_out_s81_68200_test_f2);
  - (iii)+law [f1](#/week/10/run/20261117T173009_mech_eval_seed1_s4_out_s81_68200_law_test_f1), [f2](#/week/10/run/20261117T173227_mech_eval_seed1_s4_out_s81_68200_law_test_f2);
- seed 82:
  - (i) [f1](#/week/10/run/20261117T193209_mech_eval_seed1_s4_law_s82_68400_test_f1), [f2](#/week/10/run/20261117T193425_mech_eval_seed1_s4_law_s82_68400_test_f2);
  - (iii) [f1](#/week/10/run/20261117T202625_mech_eval_seed1_s4_out_s82_68599_test_f1), [f2](#/week/10/run/20261117T202842_mech_eval_seed1_s4_out_s82_68599_test_f2);
  - (iii)+law [f1](#/week/10/run/20261117T202149_mech_eval_seed1_s4_out_s82_68599_law_test_f1), [f2](#/week/10/run/20261117T202407_mech_eval_seed1_s4_out_s82_68599_law_test_f2);
- seed 83:
  - (i) [f1](#/week/10/run/20261117T222914_mech_eval_seed1_s4_law_s83_68000_test_f1), [f2](#/week/10/run/20261117T223049_mech_eval_seed1_s4_law_s83_68000_test_f2);
  - (iii) [f1](#/week/10/run/20261117T225233_mech_eval_seed1_s4_out_s83_68200_test_f1), [f2](#/week/10/run/20261117T225309_mech_eval_seed1_s4_out_s83_68200_test_f2);
  - (iii)+law [f1](#/week/10/run/20261117T225118_mech_eval_seed1_s4_out_s83_68200_law_test_f1), [f2](#/week/10/run/20261117T225155_mech_eval_seed1_s4_out_s83_68200_law_test_f2).

The command for each: `logs/unifp_train/s4/mev.sh <name> <title> <ckpt> s4_test [--force_law] --leg_friction_range F F`. Analysis: `python3 logs/unifp_train/s1tools/s4_compare.py test` → `s1tools/s4_compare_test.json`. The script was checked first on the validation runs.

| arm | seed | opened | **in budget** | over budget | tilt stops | falls | torn | peak drive median |
|---|---|---|---|---|---|---|---|---|
| (i) layer in training | 81 | 1007 | **870** | 137 | 1 | 0 | 0 | 70.5 N |
| (i) | 82 | 786 | **783** | 3 | 10 | 0 | 0 | 43.8 N |
| (i) | 83 | 651 | **615** | 36 | 1 | 0 | 0 | 41.9 N |
| (ii) `66600` + layer zero-shot | - | 193 | **193** | 0 | 18 | 0 | 0 | 21.5 N |
| (iii) goal only | 81 | 194 | **194** | 0 | 0 | 0 | 0 | 26.8 N |
| (iii) | 82 | 214 | **214** | 0 | 0 | 0 | 0 | 25.4 N |
| (iii) | 83 | 155 | **155** | 0 | 0 | 0 | 0 | 28.7 N |
| (iii) + layer zero-shot | 81 | 404 | **404** | 0 | 26 | 0 | 0 | 38.5 N |
| (iii) + layer | 82 | 321 | **321** | 0 | 13 | 0 | 0 | 36.5 N |
| (iii) + layer | 83 | 324 | **324** | 0 | 222 | 0 | 0 | 33.8 N |
| (iv) position script | - | 19 | **19** | 0 | 0 | 0 | 0 | 15.7 N |

**Pooled over seeds, within the budget:** (i) 2,268 of 3,072; (iii) 563; (iii)+layer 1,049; (ii) 193 of 1,024; (iv) 19 of 1,024.

**Paired on the same episodes** (exact McNemar on opened within the budget), (i) against each arm:

| (i) against | seed 81 (only i / only other) | seed 82 | seed 83 | pooled |
|---|---|---|---|---|
| (ii) | 677 / 0 | 594 / 4 | 429 / 7 | (one comparator run, no pooled p) |
| (iii) | 679 / 3 | 576 / 7 | 470 / 10 | 1,725 / 20, p < 1e-300 |
| (iii)+layer | 480 / 14 | 473 / 11 | 328 / 37 | 1,281 / 62, p 7e-297 |
| (iv) | 851 / 0 | 764 / 0 | 596 / 0 | (one comparator run) |

Every per-seed p is below 1e-58.

**By the mechanism's resistance**, the fraction opened within the budget, pooled:

| arm | 10 N | 20 N | 30 N | 40 N | 50 N | 60 N | 70 N | 80 N |
|---|---|---|---|---|---|---|---|---|
| (i) | 0.99 | 0.92 | 0.80 | 0.73 | 0.67 | 0.65 | 0.60 | 0.54 |
| (iii)+layer | 0.98 | 0.93 | 0.50 | 0.24 | 0.07 | 0.01 | 0 | 0 |
| (ii) | 0.95 | 0.56 | 0 | 0 | 0 | 0 | 0 | 0 |
| (iii) | 0.73 | 0.54 | 0.16 | 0.03 | 0 | 0 | 0 | 0 |
| (iv) | 0.14 | 0.01 | 0 | 0 | 0 | 0 | 0 | 0 |

By kind, (i) opens 375 of 768 buttons, 597 doors, 621 drawers and 675 latches.

**The budget violations are after the opening.**
- All 176 of (i)'s over-budget episodes opened, and 172 of them went over 80 N only after the mechanism had opened: the drive stays on the end stop. Their peak drive median is 108 N (max 133 N), over 80 N for 1.1 s (median).
- 157 of the 176 are seed 81's.
- No other arm went over the budget. Their median peak drives were 16-39 N.
- This is F-108's integral wind-up on the old arm, again. Law v1 has no bleed, as the plan stated. A bleed (`bleed_s`) is the untested fix.

**The layer added to a policy not trained with it tips the dog:** (iii)+layer has 261 tilt stops, 222 of them seed 83's, mostly on doors (173). (i) has 12.

**What it shows:**
- On the deployable models (D1 link, `66600`'s legs), over three seeds, training with the task layer in the loop is what opens T1's held mechanisms past 30 N. That is H1 for T1 in simulation.
- The layer given zero-shot to `66600` or to (iii)'s policies gets mechanisms to 20-30 N.
- Goal-only training under the same budget and curriculum rule stalled at 35-40 N and opens below that.

**What it does not show:**
- Anything on the dog.
- A handle seen through tags: the layer uses the simulated handle's position.
- That the gap holds with a longer budget: (iii)'s curriculum stalled within 2,000 iterations, and F-107's goal-only policy on the old arm escalated to 70-80 N over about 2,850 iterations with two reward changes.
- That the budget can be kept: (i) is the only arm strong enough to break it, and does so in 5.7% of episodes.
- The claw combiner demo (F-110, F-114) on these models, which is not run.

Finding: [F-294](../findings.md#f-294).
`docs/tool_paper_plan.md`'s "Results so far" note now has H1 on T1 (F-294). Its H1 row is left as written on 13 Nov, as the note says.

### 2026-11-18 · S4 follow-up: does the integral bleed keep (i) inside the budget? Stated before scoring

F-294's violations are law v1's integral pushing on the end stop after opening. The law already has the fix F-108 named: `--force_law_bleed TAU_S` decays the integral while the handle is within 5 mm of its reference (`mech_cfg.FORCE_LAW_ARRIVED_M`).

**The test:**
- (i)'s three selected checkpoints (`s4_law_s81_68400`, `s4_law_s82_68400`, `s4_law_s83_68000`) are scored with the law and `--force_law_bleed 1.0`, at 1 and 2 N·m, on validation and on the test set.
- No retraining: the policies trained on v1 without the bleed.
- τ = 1.0 s is fixed in advance: `mech_cfg.LAW_V2_BLEED_S`, the value v2 trained with. It is not chosen on any S4 result, so nothing is selected, and the test set's pairing with F-294's runs is a single comparison stated now.

**What counts:**
- the change in over-budget episodes and in opened within the budget, paired per episode (exact McNemar) against the same checkpoints without the bleed;
- tilt stops and falls, which must not rise.

**Result** (10:00-10:05, 12 runs, all recorded). Command: `logs/unifp_train/s4/mev.sh s4_<ckpt>_bleed1_<val|test>_fF <title> <ckpt> <set> --force_law --force_law_bleed 1.0 --leg_friction_range F F`.

Each cell is opened within the budget / over budget / tilt stops; the last two columns are the change in opened, and the paired episodes in budget only without / only with the bleed:

| set | checkpoint | f | v1 | v1 + bleed 1.0 s | Δ opened | only v1 / only bleed |
|---|---|---|---|---|---|---|
| val | `law_s81_68400` | 1 | [304](#/week/10/run/20261117T160754_mech_eval_seed1_s4_law_s81_68400_val_f1) / 74 / 0 | [358](#/week/10/run/20261117T225747_mech_eval_seed1_s4_law_s81_68400_bleed1_val_f1) / 21 / 0 | +1 | 3 / 57 |
| val | `law_s81_68400` | 2 | [316](#/week/10/run/20261117T161014_mech_eval_seed1_s4_law_s81_68400_val_f2) / 46 / 0 | [358](#/week/10/run/20261117T225821_mech_eval_seed1_s4_law_s81_68400_bleed1_val_f2) / 3 / 0 | -1 | 4 / 46 |
| val | `law_s82_68400` | 1 | [291](#/week/10/run/20261117T190816_mech_eval_seed1_s4_law_s82_68400_val_f1) / 2 / 1 | [292](#/week/10/run/20261117T230009_mech_eval_seed1_s4_law_s82_68400_bleed1_val_f1) / 0 / 3 | -1 | 7 / 8 |
| val | `law_s82_68400` | 2 | [288](#/week/10/run/20261117T191031_mech_eval_seed1_s4_law_s82_68400_val_f2) / 4 / 4 | [293](#/week/10/run/20261117T230045_mech_eval_seed1_s4_law_s82_68400_bleed1_val_f2) / 0 / 3 | +1 | 2 / 7 |
| val | `law_s83_68000` | 1 | [266](#/week/10/run/20261117T213013_mech_eval_seed1_s4_law_s83_68000_val_f1) / 17 / 4 | [274](#/week/10/run/20261117T230233_mech_eval_seed1_s4_law_s83_68000_bleed1_val_f1) / 9 / 4 | +0 | 2 / 10 |
| val | `law_s83_68000` | 2 | [226](#/week/10/run/20261117T213227_mech_eval_seed1_s4_law_s83_68000_val_f2) / 19 / 1 | [238](#/week/10/run/20261117T230309_mech_eval_seed1_s4_law_s83_68000_bleed1_val_f2) / 8 / 3 | +1 | 2 / 14 |
| test | `law_s81_68400` | 1 | [414](#/week/10/run/20261117T172530_mech_eval_seed1_s4_law_s81_68400_test_f1) / 96 / 0 | [500](#/week/10/run/20261117T225856_mech_eval_seed1_s4_law_s81_68400_bleed1_test_f1) / 10 / 0 | +0 | 2 / 88 |
| test | `law_s81_68400` | 2 | [456](#/week/10/run/20261117T172748_mech_eval_seed1_s4_law_s81_68400_test_f2) / 41 / 1 | [491](#/week/10/run/20261117T225933_mech_eval_seed1_s4_law_s81_68400_bleed1_test_f2) / 5 / 1 | -1 | 4 / 39 |
| test | `law_s82_68400` | 1 | [397](#/week/10/run/20261117T193209_mech_eval_seed1_s4_law_s82_68400_test_f1) / 3 / 4 | [398](#/week/10/run/20261117T230120_mech_eval_seed1_s4_law_s82_68400_bleed1_test_f1) / 1 / 4 | -1 | 3 / 4 |
| test | `law_s82_68400` | 2 | [386](#/week/10/run/20261117T193425_mech_eval_seed1_s4_law_s82_68400_test_f2) / 0 / 6 | [384](#/week/10/run/20261117T230156_mech_eval_seed1_s4_law_s82_68400_bleed1_test_f2) / 0 / 5 | -2 | 3 / 1 |
| test | `law_s83_68000` | 1 | [347](#/week/10/run/20261117T222914_mech_eval_seed1_s4_law_s83_68000_test_f1) / 6 / 1 | [346](#/week/10/run/20261117T230344_mech_eval_seed1_s4_law_s83_68000_bleed1_test_f1) / 2 / 3 | -5 | 6 / 5 |
| test | `law_s83_68000` | 2 | [268](#/week/10/run/20261117T223049_mech_eval_seed1_s4_law_s83_68000_test_f2) / 30 / 0 | [293](#/week/10/run/20261117T230421_mech_eval_seed1_s4_law_s83_68000_bleed1_test_f2) / 5 / 2 | +0 | 2 / 27 |

**Pooled over seeds and frictions:**
- **Test:**
  - over budget 176 → 23;
  - opened within the budget 2,268 → 2,412 of 3,072, paired 20 / 164 (p 2.6e-29);
  - opened 2,444 → 2,435;
  - tilt stops 12 → 15, falls 0 → 0, torn 0 → 0.
- **Validation:**
  - over budget 162 → 41;
  - within the budget 1,691 → 1,813 of 2,304 (p 7.5e-24);
  - tilt stops 10 → 13.
- **Still to note:** seed 81 still tears 15 buttons at 1 N·m on validation (18 before). The bleed acts only within 5 mm of the reference, and a button's 1.5 cm travel leaves the integral little time there.

**What it shows:**
- The violations F-294 found were the law's, not the policy's.
- One fixed evaluation-time change removes 87% of them on the test set, opens as many, and keeps all but 23 of 3,072 inside the budget.
- Tilt stops rise by 3, within the run-to-run spread seen on validation.

**What it does not show:**
- A bound: 23 remain, 21 of them after opening, with no limit of the policy's own.
- Training with the bleed: v2 does, and was not trained here.
- Anything on the dog.

Finding: [F-295](../findings.md#f-295).

### 2026-11-19 · S4b: a T1 mechanism policy for the dog, trained with the bleed and from the dog's stance; design and rule stated before scoring

Lukas (18 Nov): "can you begin working through this list". The list's second item asks for a mechanism policy fit for the dog:
- the law's bleed trained in (F-295 added it only at evaluation);
- starting from the dog's own stance (S4 reset from UniFP's default, a departure from the plan's defaults);
- a new frozen set, then an export and a replay gate as S5's.

**Code (2026-11-19):**
- `--dog_stance_fraction SHARE` (`env_cfg.dog_stance_fraction`, `env._dog_stance`) starts that share of episodes from the dog's arm-out stance.
  - The stance is `task_cfg.DOG_STANCE_*`: leg offsets, base 0.288 m, 3.7 deg nose-up, 0.4 deg roll. These are `run_plan_probe.py --real_stance`'s values from `66600`'s four runs (F-259), held equal by `tests/test_s4b.py`.
  - Training takes 0.75, evaluation 1. It is not for the leg task, which has `--spawn_states`.
  - The base environment's reset only; nothing else changes at 0.
- New frozen sets, [`results/manifests/mech_t1_v3.json`](../manifests/mech_t1_v3.json) (`mech_cfg.s4b_manifest`, written once): S4's draw on new seeds (20261201 validation, 20261202 test), same counts, conditions, kinds, levels and budget. F-294 and F-295 read v2's test set, so it is spent for this. `--mech_set s4b_validation|s4b_test`.
- `--mech_budget_n` scores the budget at another drive, for a law capped lower.
- Fixed on the way: `mech_evaluate` read `mech_cfg` without importing it. The first smoke crashed at its report and left no `run.json`, so it is not on the dashboard.
- `tests/test_s4b.py` (4 tests) passes.

**Smokes** (not selections):
- [`66600` + law from the dog's stance](#/week/10/run/20261118T131714_mech_eval_seed1_s4b_smoke_66600_law_dog_val_f1), S4b validation at 1 N·m: 69 of 384 opened, all within the budget, 9 tilt stops, no falls.
  - The first recorded pitch is 1.2-1.5 deg nose-up, where S4's runs start 1.6-1.9 deg nose-down. The stance is applied.
- [Law v2 training, 3 iterations](#/week/10/run/20261118T131809_train_seed90_s4b_smoke_v2) at 0.75: runs from `66600`, with v2's `mech_overforce` (-1) and `mech_overspeed` (-20) in the weights.

**Two arms, three seeds each (91-93).** Each runs 2,000 iterations from `66600` on S4's recipe (its legs, link, arm prices and the 15 deg tilt stop), with `--dog_stance_fraction 0.75`:

| arm | the law in training | flags |
|---|---|---|
| A | v1 with the bleed (τ 1.0 s): S4's (i) plus the bleed | `--force_law --law_variant v1 --force_law_bleed 1.0` |
| B | v2: the bleed, an over-force price, a cap drawn 30-100 N per episode, the lunge priced ×4 (F-109's design) | `--force_law --law_variant v2` |

**Scoring:**
- From the dog's stance (`--dog_stance_fraction 1`) on `66600`'s probe legs at 1 and 2 N·m.
- The law at 80 N with the bleed (`--force_law --force_law_bleed 1.0`) for every arm.

**Selection rule, per run, stated before any S4b checkpoint is scored:** of `68000`, `68200`, `68400` and `68599`, on S4b validation (`s1tools/s4b_select.py`, S4's rule):
1. fewest falls plus tilt stops over 1 and 2 N·m;
2. then most opened within the budget;
3. then the earliest.

**The test set, read once after all six are selected,** at 1 and 2 N·m from the dog's stance:
- A's and B's selected checkpoints;
- the reference, S4's (i) selected checkpoints with the bleed (F-295's best, never trained from the dog's stance);
- each of those again with the law capped at 40 N (`--force_law_gains 400 1000 40 --mech_budget_n 40`). That is the first setting a dog trial would use, and it asks whether a policy keeps to a lower cap.

Measures: opened, opened within the budget, over budget, tilt stops, falls, torn; paired exact McNemar on opened within the budget, A and B each against the reference and against each other.

**Which goes to the dog** (a deployment choice, not a claim):
- The arm with the most opened within the 40 N budget on the test, among arms with no more falls plus tilt stops than the reference.
- Within that arm, the seed whose selected checkpoint opened the most within the budget on validation.
- That checkpoint is exported and gated as S5's.

### 2026-11-19 · Item 1a: does the insertion still seat keys if the keyway refuses misrolled ones? A strict keyway, stated before scoring

F-292 found that the simulated keyway lets keys in 4-10 deg misrolled, where the lock's wards clear about 1.3 deg. The policy for the dog, `ki6_force_8200`, may rely on that.

**Why the contact lets them in:**
- A ward corner that comes in over the edge break switches, at the break's bottom, to the wall's sideways normal (`keylock.KeyLock.step`, the `mode` rules). Nothing then holds it back along the axis.
- At 6-10 deg the default contact pushes back only 7-14 N at 5 mm, and at 1-4 deg nothing at all.

**The strict keyway** (`--insert_strict_keyway`, `KeyLock.strict_roll`; off by default, so every earlier number stands):
- A key rolled past the wards' clearance, atan(play / ward half-length), cannot pass the break. That is 0.8-1.9 deg over the drawn play, 1.53 at 0.2 mm.
- Its ward points that reach the break's depth from the mouth keep pressing on the break's plane, continued below it.
- They bear together as the flat face does: the contact's 12 kN/m and the face's total cap (40 N, under `DAMAGE_N` 50) shared among them, so a few soft points do not let it sink.
- A ward point that was in the keyway below the break first keeps the wall, so a key that went in aligned and then turned is not stuck.
- Push back at 1.5 / 5 mm deep, against the default contact's: 2 deg 3.3 / 18 N against 0 / 0; 4 deg 4.7 / 20 N against 0 / 0; 10 deg 8.5 / 25 N against 2.4 / 14 N.
- Within the clearance (1 deg at 0.2 mm play) nothing changes.
- `tests/test_keylock.py` has three new tests (20 pass).

What this is not: a measurement. The PLA key's real stiffness and the D1 wrist's roll compliance decide where between the two contacts the real lock sits. The strict keyway is the geometric bound, and Lukas's measurement (2-8 deg misrolled, depth under 10 N) places it.

**The check:**
- Policies on the frozen 96 locks (`keyinsert_eval`), the old seated test as trained, at 1 and 2 N·m:
  - `ki6_force_8200` (`--leg_variant force`);
  - `ki6_pos_8600` (`--leg_variant position`);
  - the classical arm (IK and the layer, no residual, `--hold_arm fixed --leg_variant position`) on `ki6_force_8200`'s legs.
- Each runs with the default contact and with `--insert_strict_keyway`, in the same code today, so the pairs differ only in the contact.
- The 96 are spent as a test set for selection, but nothing is selected here: it is a diagnostic of fixed policies under a contact change.
- Measures: successes inside the 15 deg tilt stop, believed, false seats, damaged; paired exact McNemar on success, default against strict.

**Decision rule, stated now:**
- If `ki6_force_8200` keeps at least three quarters of its successes on the strict keyway (pooled over 1 and 2 N·m), a real trial is worth running as it is.
- If not, it is retrained on the strict keyway from `8200` (three seeds, S1's settings), and that waits for nothing else.
- Wrapper: `logs/unifp_train/s1tools/iev2.sh` (`keyinsert/iev.sh` with the weld lock).

**Revision, 00:45, before any default-contact run was compared:** the first strict version was wrong for seated keys, and the two runs made with it are void.
- [`kw_force_strict_f1`](#/week/10/run/20261118T132906_leg_eval_seed1_kw_force_strict_f1): 54 successes, 35 damaged; [`kw_force_strict_f2`](#/week/10/run/20261118T133349_leg_eval_seed1_kw_force_strict_f2) ran the same version.
- 30 of the 35 damaged keys broke in SEAT, the tip about 8.5 mm in: the layer's ±3 deg jiggle rolled a seated key past the clearance on the soft walls. The key's back ward points then reached the mouth "misrolled" and were held, as if they were not part of a ward already in the slot.
- Fixed with a per-key state instead of the per-point one (`KeyLock.key_in`): the key is in once its tip passes the break with the wards aligned, until the tip is back at the mouth.
- The push-back figures above are unchanged by the fix. A key held misrolled at 2 mm goes in once rolled into alignment, and a seated key jiggled ±3 deg meets no more force than on the default contact (a new test; 21 pass).
- Rerun as `kw_force_strictb_f1`/`f2`. The position-only and classical strict runs had not started and use the fixed code.

**Result for `ki6_force_8200`, 01:18-01:27** (default against strict, the same code; success / false seats / damaged / peak force p90 in N):

| f | default | strict | only default / only strict | p |
|---|---|---|---|---|
| 1 | [72](#/week/10/run/20261118T133826_leg_eval_seed1_kw_force_default_f1) / 22 / 1 / 36.9 | [64](#/week/10/run/20261118T141805_leg_eval_seed1_kw_force_strictb_f1) / 15 / 14 / 51.4 | 24 / 16 | 0.27 |
| 2 | [82](#/week/10/run/20261118T134317_leg_eval_seed1_kw_force_default_f2) / 13 / 1 / 37.2 | [59](#/week/10/run/20261118T142304_leg_eval_seed1_kw_force_strictb_f2) / 12 / 22 / 52.1 | 32 / 9 | 0.0004 |

- **Successes:** pooled, 154 on the default contact and 123 on the strict one, 80% kept (56 / 25 paired, p 0.0008). By the rule stated above, three quarters kept means a real trial is worth running as it is.
- **What the rule did not price:**
  - 36 keys damaged against 2;
  - every damaged key broke in SEAT with the tip 3.5-3.7 mm in (one in APPROACH);
  - every damaged key was on a lock whose estimated roll is off by ±3 deg, past the wards' clearance. On those, the key sits blocked at the break while the layer presses and jiggles.
- A 3 deg roll error is inside what training draws (±4 deg) and what a tag gives.

**Decision:**
- The retrain (below) takes the strict keyway, and damaged keys count in its selection.
- Two reasons: the damage, and that item 1b already needs a retrain with the base's estimate in the loop (F-296).
- Until the retrain is scored, any real trial of `ki6_force_8200` should cap the force on the key well under 50 N, or be ready to lose PLA keys when the roll estimate is a few degrees off.

**The other two controllers, 01:13-02:15** (success / false seats / damaged / peak force p90 N; paired on success):

| controller | f | default | strict | only default / only strict | p |
|---|---|---|---|---|---|
| `ki6_pos_8600` | 1 | [79](#/week/10/run/20261118T143240_leg_eval_seed1_kw_pos_default_f1) / 16 / 0 / 32.0 | [19](#/week/10/run/20261118T141257_leg_eval_seed1_kw_pos_strict_f1) / 14 / 66 / 54.6 | 61 / 1 | 3e-17 |
| `ki6_pos_8600` | 2 | [70](#/week/10/run/20261118T143736_leg_eval_seed1_kw_pos_default_f2) / 21 / 2 / 30.8 | [29](#/week/10/run/20261118T142750_leg_eval_seed1_kw_pos_strict_f2) / 8 / 58 / 55.1 | 47 / 6 | 6e-9 |
| classical arm | 1 | [20](#/week/10/run/20261118T150050_leg_eval_seed1_kw_cls_default_f1) / 73 / 2 / 28.8 | [10](#/week/10/run/20261118T144226_leg_eval_seed1_kw_cls_strict_f1) / 58 / 26 / 53.8 | 16 / 6 | 0.05 |
| classical arm | 2 | [18](#/week/10/run/20261118T150522_leg_eval_seed1_kw_cls_default_f2) / 78 / 0 / 29.1 | [11](#/week/10/run/20261118T145635_leg_eval_seed1_kw_cls_strict_f2) / 55 / 30 / 53.1 | 14 / 7 | 0.19 |

The `kw_pos_strict_*` runs and the classical ones used the fixed strict model (they started after 00:45).

**Across the three controllers, on the strict keyway, pooled over 1 and 2 N·m:**

| controller | seated (default → strict) | damaged (default → strict) |
|---|---|---|
| force-aware (`ki6_force_8200`) | 154 → 123 (80% kept) | 2 → 36 |
| position-only (`ki6_pos_8600`) | 149 → 48 (32%) | 2 → 124 |
| classical arm | 38 → 21 | 2 → 56 |

- Pressing by position into a key the wards block drives the reference deeper and breaks it.
- The force-aware press commands a force, and the layer's lag then stalls it short of breaking most keys.
- This pair comes from different parents (F-291's confound). S2's twins are being run on the strict keyway to test it within a parent (below).

### 2026-11-19 · Item 1b: what the learned insertion needs on the dog, and the order of the work (plan, nothing run)

What exists:
- `insert_policy.InsertRunner` (S5, F-293) runs `ki6_force_8200` in numpy to 1e-6 of Isaac.
- `press_live.py` runs a whole-body policy on the dog: the stand-up, the unfold, report-paced arm goals, `ArmWatch`, the fold, the lie-down and the stops. `66600`'s reach is that path.
- The door program (`combiner_door/robot/node.py --task insert`) inserts the key by a script, lying down, after its look and lidar fit.

What the learned insertion needs that nothing gives yet:
1. **The base's pose against the lock on every step.** The layer measures the key's depth from the arm's forward kinematics placed by the base's pose, and training used the true pose.
   - `press_live` has no odometry: the foot press holds its plate relative to the base.
   - S5's gate held the base at x = y = 0.
   - If the body leans into the press, a fixed base pose reads the lean as the key backing out. How far the base moves is measured first, in simulation. If it matters, the next piece is a support-foot leg odometry (feet planted, IMU attitude), checked against the simulator's base.
2. **The lock's estimate after the stand-up.** The look and fit run in the door program, in sport mode, before the dog lies down. The low-level runner stands up again before any policy, and a stand-up moves the base by centimetres, against the estimate's training errors of 3.5 mm across, 8 mm along and 4 deg in roll. So the tag (and the lidar fit) must be read after the stand-up, from the policy's stand.
3. **A stance the training covers.** The frozen 96's box placements against the base; the runner checks the estimate against them before the press.
4. **The key loaded and the end.**
   - The key goes in at a carry pose, jaws open, and A shuts them, as `--reach_cloth` does.
   - A joint move then takes the arm to the runner's solved start, 17.5 mm short of the estimated face.
   - After the layer's HOLD: the jaws open, leaving the key in the lock, the hand backs out along the axis, then the fold.
5. **The box at 0.45 m (Lukas's call).** The lock is 0.4175 m above the box's root in `BoxGeometry`, and training's hold is at 0.45 m. Built as an offset that defaults to 0, so either a spacer or a setting works.
6. **Stops for this policy:**
   - `ArmWatch`'s thresholds checked against `ki6`'s own commands (S5's gate: lead ≤ 15 deg, leg targets ≤ 1.2 / 0.9 rad);
   - a cap on the layer's press;
   - the tilt stops as the reach has them.

Order: (1)'s measurement in simulation, then (1)'s odometry if needed, then the runner mode (`press_live --insert`, dry first, against `fake_go2`), then (2) and (4), then (6). Everything here is tested against fakes and recordings only. A first live run would be shadow mode (the policy computing, nothing sent), as the leg policy's first was.


**(1) measured, 00:53-01:10** ([`drift_force_f1`](#/week/10/run/20261118T134804_leg_eval_seed1_drift_force_f1): `ki6_force_8200` on the frozen 96, default contact, 1 N·m, `--insert_trace_body`; 72 of 96, as its untraced twin). Checked by `demos/combiner_key/robot/check_odometry.py`.

The base moves a lot during an insertion:

| from | base moved, forward / sideways / down (median, max) | the true feet moved (median, max) |
|---|---|---|
| the episode's start | 59, 96 / 40, 69 / 37, 44 mm | 108, 162 mm |
| 1 s in, after the spawned stance settles | 18, 47 / 43, 67 / 9, 17 mm | 17, 74 mm |

- The first half second is the spawn settling: the rear feet slip back up to 66 mm. A dog already standing does not do that, so 1 s on is the number that matters.
- A fixed base is centimetres wrong.
- Two bugs of mine on the way. The trace's last alive step is the reset's new robot, so it is dropped. The forward kinematics, checked against Isaac's feet, agree to 1-2 mm.

**The leg odometry** (`demos/combiner_key/robot/base_odometry.py`, `LegOdometry`; planted feet, IMU attitude, re-anchored on landing; `tests/test_base_odometry.py`, 3 tests), on the same traces from 1 s on:
- error forward / sideways / height 3.1 / 3.3 / 0.2 mm at the median, 14 / 7.4 / 0.3 at p90, 19 / 8 / 0.3 at worst; 5.7 / 15 / 20 mm horizontally.
- It is what the feet's slip leaves: the simulated feet slide loaded, never lifted (no foot left the ground after 1 s).

**What those errors do to the insertion:** `--insert_base_source true|fixed|legs` (`keyinsert_env.base_source`). The layer's measurement and the reference's frame take the base's position from:
- the simulator (as trained);
- where it was at the layer's start (a runner with no odometry, S5's gate);
- the leg odometry from then. In simulation it is exact kinematics: the true base plus the loaded feet's mean slip since each was anchored.

The attitude stays the simulator's (the IMU's). Scored on the frozen 96 at 1 and 2 N·m against the true-base runs, the same rule as item 4: 10 of 96 with p < 0.05 matters.

**Result, 00:58-01:13** (`ki6_force_8200`, frozen 96, default contact; McNemar on success inside the tilt stop against the true base):

| base | f | success | believed | false seats | damaged | only true / only this | p |
|---|---|---|---|---|---|---|---|
| true | 1 | [72](#/week/10/run/20261118T133826_leg_eval_seed1_kw_force_default_f1) | 94 | 22 | 1 | | |
| fixed | 1 | [0](#/week/10/run/20261118T135748_leg_eval_seed1_base_fixed_f1) | 96 | 96 | 0 | 72 / 0 | 4e-22 |
| legs | 1 | [48](#/week/10/run/20261118T140511_leg_eval_seed1_base_legs_f1) | 96 | 48 | 0 | 32 / 8 | 0.0002 |
| true | 2 | [82](#/week/10/run/20261118T134317_leg_eval_seed1_kw_force_default_f2) | 95 | 13 | 1 | | |
| fixed | 2 | [0](#/week/10/run/20261118T140138_leg_eval_seed1_base_fixed_f2) | 96 | 96 | 0 | 82 / 0 | 4e-25 |
| legs | 2 | [63](#/week/10/run/20261118T140906_leg_eval_seed1_base_legs_f2) | 95 | 32 | 1 | 25 / 6 | 0.0009 |

**What it shows:**
- With the base held where it was when the layer started, the insertion fails every time: 0 of 192, every key believed seated, every one false. That is what a dog runner with no odometry would do. S5's gate held the base fixed in a replay with no contact, so it could not see this.
- The planted-feet odometry recovers 111 of 192, against 154 with the true base.
- What it loses is the simulated feet sliding under load (17 mm at the median, 74 at worst, after the settle).

**What it does not show:**
- Whether the dog's feet slide as the simulator's do. That rests on the simulated ground friction, not validated here. The dog measurement that settles it: the feet's slip during a whole-body press, e.g. a tag on the floor beside a foot, or the leg odometry against the box's tag seen by the head camera.
- A better estimate: the box's tag tracked through the press would measure the lock against the base directly, with no drift.

**Implication for item 1b:** the runner must estimate the base's pose. The policy should be retrained with the estimate it will have (`--insert_base_source legs` in training, or the tag's model), and the next insertion retrain carries it. Finding: [F-296](../findings.md#f-296).

**The runner, built (01:30-02:10; tested against the fakes only, never run on the dog):** [`demos/combiner_key/robot/insert_live.py`](../../demos/combiner_key/robot/insert_live.py), a `press_live.PressRun` with the insertion's arm, policy and end.

```
insert_live.py --policy checkpoints/insert_force_ki6_8200.npz --lock X Y Z YAW_DEG                          # dry
insert_live.py --policy ... --lock ... --live --insert_arm_send                                            # live, typed 'key'
```

- **Dry:** the policy runs on the dog standing in sport mode with a virtual arm started at the solved start, and nothing is sent. The layer meets no face, so it checks timing, IK, the free-air layer and the targets only.
- **Live** (`press_live`'s chain), with the key loaded, from the stand-up to the handback:
  1. The stand-up.
  2. The arm unfolds to the key's hand-in pose, 2 cm back along the axis from the start, jaws open. A shuts them.
  3. A joint move to the runner's start. It is solved again from the stand the legs settle into on the policy's gains (`arm_settle`, at most two corrections): the stand sinks from the stand-up's 0.30 m to 0.27 m, and every training episode starts with the arm at the start solved from the stand it has.
  4. The policy, the base from `LegOdometry` (every foot taken as planted unless `--contact_raw`).
  5. After the layer's hold: the jaws open, the key left in the lock, the hand backs out along the axis as far as the IK solves (up to `--back_out_m`), then the fold and the lie-down.
- **Refused:**
  - a lock estimate outside what the policy trained on (`leg_cfg.KEYHOLD_*`: the door 0.495-0.525 m ahead, the keyway 0.095-0.125 m right, 0.442-0.458 m up, yaw ±0.05 rad, ±5 mm / 1 deg). This also refuses the box as it is, at 0.4175 m, until it is raised.
  - live without the arm, or the arm without live.
  - an observing link.
- **Stops:**
  - `press_live`'s;
  - the reach's whole-body ones (soft tilt, leg bound, `ArmWatch`);
  - the layer's press past `--max_press_n` (20 N by default).
  - a touch felt more than `--air_touch_mm` (15) short of the estimated face, added after item 4 (F-298): a shoulder that cannot track looks like contact to the layer. Training's estimate is off by at most 8 mm along the axis.
  - No key in 90 s folds and lies down instead of damping.
- **Joint5 bounds the hand-in pose:** from the stand-up's stand the level hand reaches its lower limit 3 cm back along the axis.
- `tests/test_insert_live.py` (11 tests): the dry run sends nothing; the live run's order; no key; the press cap; the refusals; the frames. With them `press_live`, `reach_live`, `reach_arm`, `insert_policy` and `base_odometry` pass (217). `press_live.DRIVEN` gained `arm_settle`, so its stale-feed and tilt stops cover it.

**Not done:**
- the lock read from the head camera's tag after the stand-up (`--lock` is typed in);
- `ArmWatch`'s thresholds checked against this policy's commands (they are the reach's);
- a dry run on the dog;
- the box raised (Lukas's call).
- The policy it runs should be `ki7` if it wins its test (below), since `ki6` loses a third of its seats on the leg odometry.
### 2026-11-19 · Item 4: which simulated parameters `ki6_force_8200`'s insertion rests on; stated before scoring

The policy for the dog, on the frozen 96 with the default contact at 1 N·m, against its own baseline run today ([`kw_force_default_f1`](#/week/10/run/20261118T133826_leg_eval_seed1_kw_force_default_f1): 72 of 96 inside the tilt stop). One parameter moved per run:

| run | shift | the flag | why |
|---|---|---|---|
| `sens_j23_080` | Joint2-3's torque limit ×0.8 (2.66 N·m) | `--arm_effort_scale 1 0.8 0.8 1 1 1` | 3.33 N·m is the URDF's, unmeasured, and holding the arm out uses 0.87-0.92 of it (F-293) |
| `sens_j23_090` | ×0.9 | `--arm_effort_scale 1 0.9 0.9 1 1 1` | |
| `sens_j23_120` | ×1.2 | `--arm_effort_scale 1 1.2 1.2 1 1 1` | in case the real joints are stronger |
| `sens_wrist_080` | Joint4-6 ×0.8 | `--arm_effort_scale 1 1 1 0.8 0.8 0.8` | the wrist rolls the key in the search and the jiggle |
| `sens_lag_045` | the arm's lag fixed at 45 ms | `--arm_servo_lag_s 0.045` | training draws 60-90 ms (F-250's fit 77 ms) |
| `sens_lag_120` | fixed at 120 ms | `--arm_servo_lag_s 0.12` | a slower arm than F-250 measured |
| `sens_fric3` | leg friction 3 N·m | `--leg_friction_range 3 3` | past training's 0-2 |
| `sens_settled` | the simulator's settled arm-out stance, not the dog's | `--spawn_states demos/combiner_key/assets/keyhold_settled_arm_out.json --eval_spawn_states` | the dog's stance after a classical draw is unmeasured (F-282's scope) |

**Rule:** a shift matters if success inside the tilt stop moves by 10 or more of 96 with an episode-paired exact McNemar p below 0.05 against the baseline. Damaged keys, falls and peak force are reported beside it. For each shift that matters, the log names the dog measurement that would settle it. Wrapper: `s1tools/iev2.sh`.

**Result, 02:05-02:45** (success inside the stop / damaged / falls / peak force p90 N; paired against the baseline):

| run | success | damaged | falls | peak p90 | Δ | only baseline / only shifted | p |
|---|---|---|---|---|---|---|---|
| baseline | [72](#/week/10/run/20261118T133826_leg_eval_seed1_kw_force_default_f1) | 1 | 0 | 36.9 | | | |
| Joint2-3 ×0.8 | [0](#/week/10/run/20261118T150928_leg_eval_seed1_sens_j23_080) | 0 | 0 | 16.7 | -72 | 72 / 0 | 4e-22, **matters** |
| Joint2-3 ×0.9 | [53](#/week/10/run/20261118T151417_leg_eval_seed1_sens_j23_090) | 5 | 0 | 39.8 | -19 | 33 / 14 | 0.008, **matters** |
| Joint2-3 ×1.2 | [60](#/week/10/run/20261118T151851_leg_eval_seed1_sens_j23_120) | 0 | 0 | 36.5 | -12 | 22 / 10 | 0.05 |
| Joint4-6 ×0.8 | [73](#/week/10/run/20261118T152327_leg_eval_seed1_sens_wrist_080) | 2 | 0 | 37.2 | +1 | 5 / 6 | 1 |
| the arm's lag 45 ms | [57](#/week/10/run/20261118T152752_leg_eval_seed1_sens_lag_045) | 14 | 0 | 66.1 | -15 | 24 / 9 | 0.014, **matters** |
| the arm's lag 120 ms | [67](#/week/10/run/20261118T153224_leg_eval_seed1_sens_lag_120) | 0 | 0 | 28.5 | -5 | 12 / 7 | 0.36 |
| leg friction 3 N·m | [58](#/week/10/run/20261118T153655_leg_eval_seed1_sens_fric3) | 0 | 0 | 32.2 | -14 | 30 / 16 | 0.054 |
| the simulator's settled stance | [51](#/week/10/run/20261118T154127_leg_eval_seed1_sens_settled) | 3 | 0 | 38.3 | -21 | 37 / 16 | 0.006, **matters** |

**The insertion rests on Joint2-3's torque limit above all.**
- At 0.8 of the URDF's 3.33 N·m (2.66) it seats nothing.
- The shoulder cannot hold the arm to its reference. The hand lags, and the layer takes the lag for a touch 70 mm short of the face (median). It then believes 91 keys seated that never entered: F-292's air touch, caused by the arm.
- At 0.9 it loses 19 of 72.
- At 1.2 it loses 12 (p 0.05, under the rule's bar): the policy is tuned to the limit it trained with, not just short of strength.

**The other three that matter:**
- **A faster arm (45 ms against training's 60-90):** presses harder, 14 keys damaged, peak p90 66 N. F-250 measured 77 ms on the dog, inside training, so it is unlikely there.
- **The simulator's settled stance in place of the dog's:** -21. The policy depends on the stance it starts from (F-282's scope).
- **Leg friction 3 N·m:** -14 (p 0.054, under the bar).
- A weaker wrist and a slower arm (120 ms) change nothing.

**The dog measurements, in order:**
1. **Joint2's and Joint3's real torque limit,** e.g. the static load at which the D1 can no longer hold the insertion's start pose with the key, by a known weight at the hand. The URDF's 3.33 N·m is unverified (`motor_model`), and holding the arm out already uses 0.87-0.92 of it (F-293).
2. **The stance the dog hands the policy** (the legs and base at the policy's first step), against the dog's arm-out stance it is scored from. `insert_live` records it.

Finding: [F-298](../findings.md#f-298).

### 2026-11-19 · Item 1, the insertion retrain (`ki7`): the strict keyway and the base's estimate in the loop; design and rule stated before scoring

Why: on the strict keyway `ki6_force_8200` breaks keys when the roll estimate is 3 deg off (item 1a). On the dog it will know the base only by leg odometry, which costs it 43 of 154 seats in simulation (F-296). This is a deployment candidate, not a paper claim: one arm, three seeds, no matched control.

**Training:**
- Round 6's recipe from `ki6_force_8200` (`keyinsert/recipe.sh`: `$KH --leg_variant force`, `$LINKT` with the learning rate pinned at 3e-5, `$LEGS66600`, `$STANCES`, `$STOPS`, the 15 deg tilt stop and `--base_tilt_weight -200`).
- Plus `--insert_strict_keyway --insert_base_source legs --insert_damage_stop`. The damage stop is S1b's (F-287): training past a damaged key paid a light press that left keys short, and damage is why this retrain exists. S1's pilot ran without it.
- 1,500 iterations (S1's budget), seeds 61, 62, 63.

**Sets:** new, [`results/manifests/keyinsert_ki7_v1.json`](../manifests/keyinsert_ki7_v1.json) (`keytools.ki7_manifest`, written once): the v4 key on 48 validation locks and 96 test locks, drawn as the twins' on new seeds (20261203, 20261204). The frozen 96 and the twins' sets are spent for selection.

**Scoring:** the strict keyway and the leg odometry, as the dog would meet them, `66600`'s probe legs at 1 and 2 N·m, the dog's stance, the old seated test.

**Selection rule, per seed, stated before any `ki7` checkpoint is scored:** of `9000`, `9200`, `9400` and `9699` (the last four saves; the trainer saves every 200), on validation:
1. fewest falls plus damaged keys over 1 and 2 N·m;
2. then most successes inside the 15 deg tilt stop;
3. then the earliest.

**Test, read once after all three are selected:**
- the three selected and `ki6_force_8200`, on the test locks at 1 and 2 N·m with the strict keyway and the leg odometry;
- the same four on the default contact with the true base, to see what the retrain cost where `ki6` was trained.

**Measures:** success, damaged, false seats, peak force, falls; paired McNemar against `ki6_force_8200`.

**Which goes to the dog:** a selected `ki7` replaces `ki6_force_8200` if, pooled over seeds, it has fewer damaged keys and no fewer successes on the strict keyway with the leg odometry. Then the seed with the most validation successes is exported and gated as S5's.

*Note, 05:55, after one candidate was scored and before any test run:* "pooled over seeds" is read as the three selected seeds' total against three times `ki6_force_8200`'s count on the same locks, which is the seeds' mean against `ki6`. A tie on validation successes goes to the lower seed. Scripts: `s1tools/ki7_select.py` (the rule above), `queue_ki7_test.sh` and `ki7_compare.py` (the test and this decision).

### 2026-11-19 · Item 3: the box sequence in simulation, the learned insertion handed to the learned hold and turn; stated before scoring

G7 needs "a validated simulated sequence". The staged sequence (F-278's) draws and inserts the key by script before the hold policy, and scored 56 of 108 with `kh16_6800`. This one puts the learned insertion first.

**How:** two linked evaluations, not one environment. The insertion env overrides most of the hold env's methods, so a per-environment switch inside one batch would branch nearly all of it.
1. The insertion: `ki6_force_8200` on the frozen 96 with the base traced (`drift_force_f1`/`f2`, F-296's runs).
2. `demos/combiner_key/insert_handover.py` takes every episode seated inside the tilt stop at the layer's last HOLD step:
   - the legs, base height, roll and pitch as a spawn state;
   - its box re-expressed from the base's pose then, so a hold episode starting at the origin meets the box where the robot is against it;
   - the friction, detent and estimate errors of its key row.
3. The hold and turn: `kh28_7000` (F-285's force-aware candidate) from each, in order:
   - `s1tools/hev.sh`, which is `kev.sh`'s legs, key flags and link;
   - `--key_eval_rows` (`leg_eval.rows_key_plan`) and `--spawn_states_ordered` (`leg_env.spawn_states_ordered`), both new and off by default;
   - at the same friction as the insertion.

**Measure:** the chain's success is the insertion seated inside the tilt stop and then the hold's `all_four` (unlocked, key kept, foot back) inside the stop, out of the 96 started. Each stage is also reported, beside `kh28_7000` on its own frozen keys from the dog's stance.

**What the handover already shows, before any hold is run:** the robot does not end the insertion where the hold trained.

| f | handed over | door ahead (q10 / 50 / 90) | trained | door yaw (q10 / 50 / 90) | trained |
|---|---|---|---|---|---|
| 1 | 72 | 0.534 / 0.567 / 0.600 m | 0.495-0.525 | 0.028 / 0.089 / 0.127 rad | ±0.05 |
| 2 | 82 | 0.534 / 0.568 / 0.596 m | | 0.070 / 0.116 / 0.156 rad | |

- The base at the handover is 52-57 mm back, 33-50 mm left and turned 5-7 deg at the median.
- Most of the backward part is the spawned stance sliding back in its first half second (F-296), which a dog already standing does not do.
- So the chain here is the simulator's. A real chain would start the insertion nearer and end it less displaced, but still turned and still with its own 18 / 43 mm of drift.

**Result, 01:47-01:57** (`hev.sh chain_ki6_kh28_fF ... kh28_7000.pt handover_rows.json handover_spawn.json --leg_variant force --leg_friction_range F F`):

| f | insertions | seated inside the stop | hold and turn `all_four` | **inside the stop** | the hold on its own keys from the dog's stance |
|---|---|---|---|---|---|
| 1 | 96 | 72 | 39 of 72 | **[33](#/week/10/run/20261118T144709_leg_eval_seed1_chain_ki6_kh28_f1)** | 85 of 108, 58 inside ([`kh28_7000_dog_f1`](#/week/9/run/20261112T233429_leg_eval_seed1_kh28_7000_dog_f1)) |
| 2 | 96 | 82 | 36 of 82 | **[29](#/week/10/run/20261118T145156_leg_eval_seed1_chain_ki6_kh28_f2)** | 94, 90 inside ([`kh28_7000_dog_f2`](#/week/9/run/20261112T233835_leg_eval_seed1_kh28_7000_dog_f2)) |

- **The whole chain:** 62 of 192 episodes pass both stages inside the tilt stop, about a third.
- **Given a seated key,** the hold and turn succeed in 46% and 35% of handovers, against 54% and 83% from the dog's stance on its own keys.
- **How the failures go:** at 2 N·m they stall (28) and miss the key (14). At 1 N·m keys are lost (12), the turn stalls (9) or the foot misses (7).
- By the door's distance at the handover the rate is mixed (1 N·m: 14 of 25 under 0.55 m, 7 of 27 beyond 0.58 m; 2 N·m: no trend).
- The own-key runs are 12 Nov's code; the chain is today's.

**What it shows:** in simulation the learned insertion and the learned hold do not chain as they score apart. The insertion leaves the robot outside the hold's trained box range (the door 4-8 cm further, turned 2-9 deg) and in a stance the hold did not train from, and the hold loses most of its 2 N·m success there.

**What it does not show:**
- the real chain, because most of the distance is the spawned stance's slide;
- the scripted draw before the insertion;
- the door after the turn.
- And the hold's own frozen keys are a different set from the handovers.

**Next for the sequence:** train the hold from the insertion's end states (these handovers as spawn states and boxes), or fix the insertion's spawn so it does not slide, then chain again. Finding: [F-297](../findings.md#f-297).

**Note, 04:30, corrected 04:45.** "Inside the stop" above is the tilt stop alone (`all_four`, `max_tilt_deg` ≤ 15). Week 9 scored the hold inside the dog runner's bounds instead (`keyhold/runner_ok.py`): tilt, the replay gate's arm-lead bound (`leg_eval.STOP_LEAD_DEG`, J1-J5 at most 15 deg ahead of the report from 1 s on), the support legs and no fall, with the front-right leg's bound lifted. The note first said week 9 used the tilt stop alone; `select3.py` and `runner_ok.py` show otherwise.
- Scored that way, the chain keeps 3 of 192 (f1 3 of 33, f2 0 of 29). The lead bound removes them.
- `kh28_7000` on its own keys keeps 145 of its 148 ([`kh28_7000_dog_f1`](#/week/9/run/20261112T233429_leg_eval_seed1_kh28_7000_dog_f1) 58 of 58, [`_f2`](#/week/9/run/20261112T233835_leg_eval_seed1_kh28_7000_dog_f2) 87 of 90), its week 9 numbers.
- Across all episodes, 24 of 154 handovers' holds stay within 15 deg, against 189 of 216 on its own keys. The median lead is 17.3-17.7 deg, against 11.7-12.2.
- `leg_eval`'s `all_four_within_all` is 0 for both, because its leg-target stops are the reach runner's and the front-right foot turns the key. It is not the hold's measure.
- Checked with `s1tools/chain_score.py`'s scoring on the recorded runs. Added to F-297 as a dated note.
- `docs/tool_paper_plan.md`'s "Results so far" said the chain was 62 of 192. It now gives both numbers and adds F-299.


### 2026-11-19 · H3 on the strict keyway: S2's twins, stated before scoring

Item 1a's pair (`ki6_force_8200`, `ki6_pos_8600`) split on the strict keyway:
- force-aware keeps 123 of 154 seats with 36 keys damaged;
- position-only keeps 48 of 149 with 124 damaged, pressing a blocked key by position.

Those two come from different parents, the confound F-291 found in F-284. S2's twins come from one parent each.

**The test:**
- S2's twelve selected twins:
  - family F from `ki6_force_8200`: force-aware `s2t_Ff` 8600 / 8400 / 8800, position-only `s2t_Fp` 8800 / 8400 / 8400;
  - family P from `ki6_pos_8600`: force-aware `s2t_Pf` 9000 / 8800 / 8800, position-only `s2t_Pp` 9200 / 9200 / 9399.
- On the twins' 96 test locks, at 1 and 2 N·m from the dog's stance, with the strict keyway and the default contact. Both are run today, so the pairs differ only in the contact.
- The set was read by S2 on the default contact. Nothing is selected here, and the contact is a shift stated now.

**Measures:**
- Each twin's success inside the tilt stop on the default and strict contacts, and its damaged keys.
- The pre-stated contrast: within each family, (force-aware - position-only) of (strict - default) successes, pooled over seeds and frictions.
- H3 as the plan words it (force-aware loses under tool and lock shifts) predicts it negative.
- With seeds as the unit the test is weak (three a family), so the contrast is reported with each seed's.

Wrapper: `s1tools/tev.sh <name> <title> <ckpt> test --insert_tool_manifest results/manifests/keyinsert_twins_v1.json --leg_variant force|position [--insert_strict_keyway] --leg_friction_range F F`.

**Result, 02:45-06:59** (48 runs, all exit 0; `python3 logs/unifp_train/s1tools/tw_compare.py`, out `tw_compare.json`). Success inside the tilt stop and damaged keys, of 192 a twin (both frictions); strict against default paired on the same locks:

| twin | seed | it | default | damaged | strict | damaged | strict - default | only default / only strict | p |
|---|---|---|---|---|---|---|---|---|---|
| Ff | 61 | 8600 | 119 | 3 | 122 | 17 | +3 | 24 / 27 | 0.78 |
| Ff | 62 | 8400 | 126 | 2 | 118 | 18 | -8 | 29 / 21 | 0.32 |
| Ff | 63 | 8800 | 111 | 4 | 118 | 18 | +7 | 24 / 31 | 0.42 |
| Fp | 61 | 8800 | 160 | 1 | 128 | 28 | -32 | 40 / 8 | 3e-6 |
| Fp | 62 | 8400 | 158 | 0 | 146 | 8 | -12 | 24 / 12 | 0.065 |
| Fp | 63 | 8400 | 152 | 0 | 133 | 16 | -19 | 33 / 14 | 0.008 |
| Pf | 61 | 9000 | 134 | 21 | 74 | 97 | -60 | 69 / 9 | 1e-12 |
| Pf | 62 | 8800 | 143 | 17 | 71 | 108 | -72 | 77 / 5 | 1e-17 |
| Pf | 63 | 8800 | 146 | 24 | 74 | 107 | -72 | 77 / 5 | 1e-17 |
| Pp | 61 | 9200 | 132 | 6 | 102 | 41 | -30 | 45 / 15 | 1e-4 |
| Pp | 62 | 9200 | 131 | 18 | 107 | 52 | -24 | 43 / 19 | 0.003 |
| Pp | 63 | 9399 | 152 | 4 | 144 | 22 | -8 | 33 / 25 | 0.36 |

**The pre-stated contrast**, (force-aware - position-only) of (strict - default) successes:

| family | seed 61 | seed 62 | seed 63 | pooled (of 576 a twin) |
|---|---|---|---|---|
| F (from `ki6_force_8200`) | +35 | +4 | +26 | **+65** (force-aware 356 → 358, position-only 470 → 407) |
| P (from `ki6_pos_8600`) | -30 | -48 | -64 | **-142** (force-aware 423 → 219, position-only 415 → 353) |

**What it shows:**
- **The sign follows the parent.** H3 predicts a negative contrast. Family P's is negative in every seed, family F's positive in every seed. As in F-291, the variant's effect under the shift is not one effect of force awareness, so H3 is still not supported.
- **Position-only seats more on the strict keyway in both families,** 407 against 358 in F and 353 against 219 in P. Family F's force-aware twins lose nothing to the strict keyway, but they start 114 lower on the default contact.
- **Family P's force-aware twins break half the keys:** 312 damaged of 576, against 115 for P's position-only twins and 52-53 for family F's.
- **Item 1a's split does not carry over.** `ki6_force_8200` kept 80% of its seats and `ki6_pos_8600` 32%. Here, `ki6_pos_8600`'s position-only fine-tunes keep 85% (353 of 415) and its force-aware fine-tunes 52%. The 32% belonged to that one checkpoint (and the frozen 96), not to position-only control.
- **The strict keyway turns some false seats into damaged keys.** Pooled over every twin, false seats fall from 525 to 425 of 2,304 and damaged keys rise from 100 to 532.

**Exploratory, not stated before scoring:**
- Across the twelve twins, how hard each presses on the default contact (its episodes' median press force) predicts what the strict keyway costs it: Spearman -0.78, permutation p 0.003, n = 12.
- Ordered by press, the median press and pooled change are: Ff 11-13 N (+2), Fp 13-15 N (-63), Pp 15-19 N (-62), Pf 18-19 N (-204). Peak force on the default contact predicts it less well (-0.66).
- This does not explain item 1a's pair: `ki6_force_8200` and `ki6_pos_8600` press alike on the default contact (median 14.0-14.4 N against 14.7-15.4 N).
- What makes a checkpoint fragile on a keyway that refuses misrolled keys is not identified.

**What it does not show:**
- A real keyway. The strict contact is an opt-in model (`KeyLock.strict_roll`), with no measurement of the cam lock's roll tolerance behind it.
- Anything about tools: the key is v4.

`docs/tool_paper_plan.md` corrected: its S5 line called item 1a's split force-aware against position-only, and its H3 line now cites F-300.

Runs (default f1 f2 | strict f1 f2):

| twin | it | default | strict |
|---|---|---|---|
| `s2t_Ff` 61 | 8600 | [f1](#/week/10/run/20261118T160335_leg_eval_seed1_tw_Ff_s61_8600_default_f1) [f2](#/week/10/run/20261118T161718_leg_eval_seed1_tw_Ff_s61_8600_default_f2) | [f1](#/week/10/run/20261118T154459_leg_eval_seed1_tw_Ff_s61_8600_strict_f1) [f2](#/week/10/run/20261118T155849_leg_eval_seed1_tw_Ff_s61_8600_strict_f2) |
| `s2t_Ff` 62 | 8400 | [f1](#/week/10/run/20261118T164035_leg_eval_seed1_tw_Ff_s62_8400_default_f1) [f2](#/week/10/run/20261118T164508_leg_eval_seed1_tw_Ff_s62_8400_default_f2) | [f1](#/week/10/run/20261118T162159_leg_eval_seed1_tw_Ff_s62_8400_strict_f1) [f2](#/week/10/run/20261118T162638_leg_eval_seed1_tw_Ff_s62_8400_strict_f2) |
| `s2t_Ff` 63 | 8800 | [f1](#/week/10/run/20261118T171121_leg_eval_seed1_tw_Ff_s63_8800_default_f1) [f2](#/week/10/run/20261118T171412_leg_eval_seed1_tw_Ff_s63_8800_default_f2) | [f1](#/week/10/run/20261118T164946_leg_eval_seed1_tw_Ff_s63_8800_strict_f1) [f2](#/week/10/run/20261118T170721_leg_eval_seed1_tw_Ff_s63_8800_strict_f2) |
| `s2t_Fp` 61 | 8800 | [f1](#/week/10/run/20261118T172444_leg_eval_seed1_tw_Fp_s61_8800_default_f1) [f2](#/week/10/run/20261118T172802_leg_eval_seed1_tw_Fp_s61_8800_default_f2) | [f1](#/week/10/run/20261118T171758_leg_eval_seed1_tw_Fp_s61_8800_strict_f1) [f2](#/week/10/run/20261118T172122_leg_eval_seed1_tw_Fp_s61_8800_strict_f2) |
| `s2t_Fp` 62 | 8400 | [f1](#/week/10/run/20261118T173917_leg_eval_seed1_tw_Fp_s62_8400_default_f1) [f2](#/week/10/run/20261118T174307_leg_eval_seed1_tw_Fp_s62_8400_default_f2) | [f1](#/week/10/run/20261118T173136_leg_eval_seed1_tw_Fp_s62_8400_strict_f1) [f2](#/week/10/run/20261118T173532_leg_eval_seed1_tw_Fp_s62_8400_strict_f2) |
| `s2t_Fp` 63 | 8400 | [f1](#/week/10/run/20261118T175445_leg_eval_seed1_tw_Fp_s63_8400_default_f1) [f2](#/week/10/run/20261118T175834_leg_eval_seed1_tw_Fp_s63_8400_default_f2) | [f1](#/week/10/run/20261118T174655_leg_eval_seed1_tw_Fp_s63_8400_strict_f1) [f2](#/week/10/run/20261118T175049_leg_eval_seed1_tw_Fp_s63_8400_strict_f2) |
| `s2t_Pf` 61 | 9000 | [f1](#/week/10/run/20261118T180843_leg_eval_seed1_tw_Pf_s61_9000_default_f1) [f2](#/week/10/run/20261118T181225_leg_eval_seed1_tw_Pf_s61_9000_default_f2) | [f1](#/week/10/run/20261118T180208_leg_eval_seed1_tw_Pf_s61_9000_strict_f1) [f2](#/week/10/run/20261118T180536_leg_eval_seed1_tw_Pf_s61_9000_strict_f2) |
| `s2t_Pf` 62 | 8800 | [f1](#/week/10/run/20261118T182245_leg_eval_seed1_tw_Pf_s62_8800_default_f1) [f2](#/week/10/run/20261118T182634_leg_eval_seed1_tw_Pf_s62_8800_default_f2) | [f1](#/week/10/run/20261118T181536_leg_eval_seed1_tw_Pf_s62_8800_strict_f1) [f2](#/week/10/run/20261118T181911_leg_eval_seed1_tw_Pf_s62_8800_strict_f2) |
| `s2t_Pf` 63 | 8800 | [f1](#/week/10/run/20261118T183658_leg_eval_seed1_tw_Pf_s63_8800_default_f1) [f2](#/week/10/run/20261118T185237_leg_eval_seed1_tw_Pf_s63_8800_default_f2) | [f1](#/week/10/run/20261118T182957_leg_eval_seed1_tw_Pf_s63_8800_strict_f1) [f2](#/week/10/run/20261118T183344_leg_eval_seed1_tw_Pf_s63_8800_strict_f2) |
| `s2t_Pp` 61 | 9200 | [f1](#/week/10/run/20261118T190303_leg_eval_seed1_tw_Pp_s61_9200_default_f1) [f2](#/week/10/run/20261118T191743_leg_eval_seed1_tw_Pp_s61_9200_default_f2) | [f1](#/week/10/run/20261118T185543_leg_eval_seed1_tw_Pp_s61_9200_strict_f1) [f2](#/week/10/run/20261118T185923_leg_eval_seed1_tw_Pp_s61_9200_strict_f2) |
| `s2t_Pp` 62 | 9200 | [f1](#/week/10/run/20261118T192743_leg_eval_seed1_tw_Pp_s62_9200_default_f1) [f2](#/week/10/run/20261118T194420_leg_eval_seed1_tw_Pp_s62_9200_default_f2) | [f1](#/week/10/run/20261118T192111_leg_eval_seed1_tw_Pp_s62_9200_strict_f1) [f2](#/week/10/run/20261118T192431_leg_eval_seed1_tw_Pp_s62_9200_strict_f2) |
| `s2t_Pp` 63 | 9399 | [f1](#/week/10/run/20261118T195327_leg_eval_seed1_tw_Pp_s63_9399_default_f1) [f2](#/week/10/run/20261118T195618_leg_eval_seed1_tw_Pp_s63_9399_default_f2) | [f1](#/week/10/run/20261118T194720_leg_eval_seed1_tw_Pp_s63_9399_strict_f1) [f2](#/week/10/run/20261118T195021_leg_eval_seed1_tw_Pp_s63_9399_strict_f2) |

Finding: [F-300](../findings.md#f-300).

### 2026-11-19 · Item 1b: would the dog runner's `ArmWatch` stop the insertion in contact? Stated before scoring

`insert_live` sends every arm goal through `reach_arm.ArmWatch`, the reach runs' stop. It stops sending, softly (the arm held, the
dog folds and lies down), when:
- a joint's goal leads its report by more than 25 deg for 0.5 s, or by more than 60 deg at once;
- a goal steps more than 20 deg in one tick;
- the reports stop for 0.34 s.

The first second after the first goal is a grace. Pressing a key into a lock is a goal held ahead of an arm that cannot follow, and the thresholds were set on free-air reaches (F-255, F-256).

**Measured:**
- `ki6_force_8200` (the dog's insertion until `ki7` is selected) on the frozen 96, at 1 and 2 N·m, under the dog's conditions: the strict keyway, the base from leg odometry, the dog's stance, the D1 link with the Follower.
- The trace gains `arm_goal` (the layer's six goals, as the runner sends them) and `arm_reported` (the simulated link's report, its delay and quantisation) under `--insert_trace_body`. Nothing else in the evaluation changes.
- Job `s1tools/spool_e/e_t_lead_ki6.sh`.
- `demos/combiner_key/robot/check_armwatch.py` runs `ArmWatch.check` itself over each episode, one policy step apart, the grace from step 0, with the report always fresh (the gap stop is not tested).
- Per run it reports:
  - how many episodes it would stop, by rule and phase, and how many of them the layer seated;
  - the worst joint's lead in contact (normal force over 1 N), its median, 90th percentile and maximum;
  - the largest step after the grace.
- `tests/test_check_armwatch.py` (5 tests) checks it on made-up traces.

**Rule, stated now:**
- If `ArmWatch` stops no episode the layer seats inside the tilt stop, and the worst lead in contact stays under 12.5 deg (half the limit), the thresholds stand for the insertion.
- If it stops seated episodes, the runner needs a limit for contact (the lead the press itself asks, from the layer's commanded force), set from these numbers and re-checked on a second run.

**What it cannot show:** the real D1's lead while pressing a key. The lead here is the simulated arm's drive and stiffness, and it is not validated on the arm.

**Result, 03:58-04:07** (`e_t_lead_ki6`: `iev2.sh lead_ki6_strict_legs_fF "<title>" logs/unifp_train/keyinsert/ki6_force_8200.pt --leg_variant force --insert_strict_keyway --insert_base_source legs --insert_trace_body --leg_friction_range F F`, then `python3 demos/combiner_key/robot/check_armwatch.py <run>`):

| f | run | seated inside the stop | damaged | false seats | `ArmWatch` stops | lead in contact, worst joint: median / p90 / max | worst joint | largest step after the grace |
|---|---|---|---|---|---|---|---|---|
| 1 | [`lead_ki6_strict_legs_f1`](#/week/10/run/20261118T165859_leg_eval_seed1_lead_ki6_strict_legs_f1) | 60 of 96 | 23 | 11 | **0** | 3.97 / 5.60 / **6.47 deg** | Joint2 | 3.5 deg |
| 2 | [`lead_ki6_strict_legs_f2`](#/week/10/run/20261118T170309_leg_eval_seed1_lead_ki6_strict_legs_f2) | 60 of 96 | 22 | 11 | **0** | 3.73 / 5.66 / **6.83 deg** | Joint2 | 4.4 deg |

- **By the rule the thresholds stand for the insertion.** No episode would be stopped, and the worst lead in contact is 6.8 deg, against the 12.5 deg bar and the 25 deg limit.
- The largest lead after the grace is the one in contact. Free motion before the touch leads less.
- Steps stay under 4.4 deg a tick, against the 20 deg limit.
- The press is the shoulder's: Joint2 leads most in contact, as F-298's torque result has it.
- On the way, `ki6_force_8200` under all of the dog's conditions at once, the strict keyway and the leg-odometry base, seats 120 of 192, with 45 keys damaged and 22 false seats. That is `ki7`'s baseline condition. These are the frozen 96, not `ki7`'s test set.

**What it does not show:**
- the real arm's lead while pressing;
- a weak shoulder's (F-298: at 0.8× the arm stops short before any press, which the air-touch stop catches, not `ArmWatch`);
- the report-gap stop.

Finding: [F-299](../findings.md#f-299). The simulated margin is about 3.7× under the hold limit. On the dog, the lead is worth logging on the first contact runs (`insert_live` records the goals and reports).

### 2026-11-19 · S4b seed 91: both arms trained, and `68200` selected on validation for each

**Training** (`s1tools/train.sh`, `t_s4b_01` / `t_s4b_02`): 2,000 iterations from `66600`, `--dog_stance_fraction 0.75`. Both ran 13:20-16:49 UTC (3.5 h, beside the eval lane) and exited 0:
- [arm A, seed 91](#/week/10/run/20261118T132002_train_seed91_s4b_A_s91): law v1 with the bleed;
- [arm B, seed 91](#/week/10/run/20261118T132026_train_seed91_s4b_B_s91): law v2.

**Validation** (S4b validation, 384 placements per friction, from the dog's stance, the law at 80 N with the bleed), selected by the rule stated above (`s1tools/s4b_select.py`):

Arm A (v1 + bleed):

| checkpoint | 1 N·m in budget / over / falls / tilt stops | 2 N·m | stops (1+2) | in budget (of 768) |
|---|---|---|---|---|
| 68000 | [268](#/week/10/run/20261118T154954_mech_eval_seed1_s4b_A_s91_68000_val_f1) / 0 / 0 / 11 | [255](#/week/10/run/20261118T155205_mech_eval_seed1_s4b_A_s91_68000_val_f2) / 0 / 0 / 6 | 17 | 523 |
| **68200** | [328](#/week/10/run/20261118T160821_mech_eval_seed1_s4b_A_s91_68200_val_f1) / 0 / 0 / 0 | [314](#/week/10/run/20261118T161033_mech_eval_seed1_s4b_A_s91_68200_val_f2) / 0 / 0 / 1 | 1 | **642** |
| 68400 | [302](#/week/10/run/20261118T163130_mech_eval_seed1_s4b_A_s91_68400_val_f1) / 0 / 0 / 3 | [295](#/week/10/run/20261118T163341_mech_eval_seed1_s4b_A_s91_68400_val_f2) / 0 / 0 / 2 | 5 | 597 |
| 68599 | [322](#/week/10/run/20261118T165130_mech_eval_seed1_s4b_A_s91_68599_val_f1) / 0 / 0 / 1 | [307](#/week/10/run/20261118T165319_mech_eval_seed1_s4b_A_s91_68599_val_f2) / 0 / 0 / 0 | 1 | 629 |

Arm B (v2):

| checkpoint | 1 N·m in budget / over / falls / tilt stops | 2 N·m | stops (1+2) | in budget (of 768) |
|---|---|---|---|---|
| 68000 | [301](#/week/10/run/20261118T155424_mech_eval_seed1_s4b_B_s91_68000_val_f1) / 9 / 0 / 1 | [289](#/week/10/run/20261118T155636_mech_eval_seed1_s4b_B_s91_68000_val_f2) / 0 / 0 / 3 | 4 | 590 |
| **68200** | [309](#/week/10/run/20261118T161254_mech_eval_seed1_s4b_B_s91_68200_val_f1) / 2 / 0 / 0 | [309](#/week/10/run/20261118T161507_mech_eval_seed1_s4b_B_s91_68200_val_f2) / 1 / 0 / 1 | 1 | **618** |
| 68400 | [302](#/week/10/run/20261118T163605_mech_eval_seed1_s4b_B_s91_68400_val_f1) / 2 / 0 / 17 | [295](#/week/10/run/20261118T163820_mech_eval_seed1_s4b_B_s91_68400_val_f2) / 2 / 0 / 15 | 32 | 597 |
| 68599 | [300](#/week/10/run/20261118T165518_mech_eval_seed1_s4b_B_s91_68599_val_f1) / 0 / 0 / 8 | [299](#/week/10/run/20261118T165708_mech_eval_seed1_s4b_B_s91_68599_val_f2) / 3 / 0 / 0 | 8 | 599 |

- **Selected:** A `68200` (1 stop, 642 in budget, none over) and B `68200` (1 stop, 618 in budget, 3 over). Written to `s1tools/s4b_<arm>_s91_selected.json`.
- No falls in any of the 16 runs. The stops are tilt stops.
- A kept every opening within the 80 N budget at all four checkpoints. B went over on 3-9 of 768 at three of them, though v2 prices over-force in training.
- Validation only, one seed. These choose the checkpoint, nothing else. A against B is the test set's question, after all six are selected.

**Queue change, 03:55:** `ki7`'s seed 63 moved from lane t1 to lane t2, ahead of S4b B seed 92 (`spool_t2/t_s4b_035_ki7_s63.sh`; its contents unchanged and not yet started).
- Why: `ki7` can then be selected and tested by about midday rather than tonight.
- Cost: S4b's B seeds 92 and 93 start about 2.5 h later.

### 2026-11-19 · Item 3 follow-up: the hold trained from the insertion's handovers; design and rule stated before scoring

F-297 and its note: from the learned insertion's end, `kh28_7000` keeps 62 of 192 inside the tilt stop and 3 inside the replay gate's 15 deg arm-lead bound as well. The robot ends the insertion outside the hold's trained box (the door 0.52-0.62 m ahead against 0.495-0.525, turned up to 0.19 rad against ±0.05) and in stances it never started from (roll and pitch to ±10 deg).

**Sets:** new, [`results/manifests/keyinsert_chain_v1.json`](../manifests/keyinsert_chain_v1.json) (`keytools.chain_manifest`, written once, `tests/test_keytools_manifests.py`): the v4 key on three lock sets drawn on new seeds, none repeating a lock any earlier set drew:
- `spawn`, 96 locks: scored only by the insertion, only to make the hold's training stances;
- `validation`, 48 locks: the chain selects on it;
- `test`, 96 locks: read once.

**The insertion** is the dog's, item 1's decision: `ki7`'s pick if it replaces `ki6_force_8200`, else `ki6_force_8200`. It is fixed before any chain set is scored. It runs under the dog's conditions (strict keyway, leg-odometry base, the dog's stance, `66600`'s probe legs, the D1 link), at 1 and 2 N·m, with `--insert_trace_body`.

**Handovers:** `insert_handover.py`, now also for tool-set runs. Those carry no key row, so a handover takes the hold plan's row `lock mod 108`, as the frozen 96 do. Its friction, detent and estimate errors are the hold plan's. Its box and stance are the insertion's.

**Training stances:** every handover stance from the `spawn` runs, plus the recipe's four (`keyhold_stances_dog3_settled1.json`) repeated to as many (`s1tools/merge_spawn.py`). So about half the episodes start from a handover, and the other half from the recipe's 3 : 1.

**Training:** round 28's recipe (`keyhold/round28.sh`: `$KH $IMPACT $LINKT $LEGS66600 $STOPS --key_turn_follow_weight -15`), force-aware, from `kh28_7000`, 800 iterations, seeds 71-73, with:
- `--spawn_states <the merged stances> --spawn_states_fraction 1.0`;
- `--key_hold_door_ahead 0.495 0.625 --key_hold_axis_side -0.145 -0.04 --key_hold_door_yaw -0.05 0.19`, covering every handover in F-297's and the `spawn` runs. The box is drawn apart from the stance.

**Measures:**
- A chain success is an insertion seated inside the tilt stop whose hold then does all four inside the tilt stop.
- `chain_gate` also requires the hold inside the dog runner's bounds as week 9 scored them (`keyhold/runner_ok.py`: the lead bound of J1-J5 at most 15 deg ahead from 1 s on, the support legs, no fall).
- Both are out of the insertions started (48 or 96 a friction). Insertions not handed over fail for every candidate alike.
- Scored by `s1tools/chain_score.py`, which reproduces F-297's 33 / 29 and its note's 3 / 0 from the recorded runs, and `kh28_7000`'s week 9 58 / 87 on its own keys.

**Selection per seed,** on `validation` at 1 and 2 N·m: of `7200`, `7400`, `7600` and `7799`:
1. the most `chain_gate` over both;
2. then the most chain successes inside the tilt stop alone;
3. then the fewest falls plus lost keys;
4. then the earliest.

**The test, once:** on `test` at 1 and 2 N·m, from the same handovers:
- each seed's selected checkpoint and `kh28_7000`;
- episode-paired exact McNemar of each seed against `kh28_7000` on `chain_gate`, and pooled;
- each selected checkpoint also on the hold's own frozen keys from the dog's stance (`kh28_7000_dog_f1`/`f2`'s runs), so that a gain on the chain is not paid for there.

**What it will not show:** the real chain. Most of the simulated handover's displacement is the spawned stance's slide (F-296), and no hold runner exists on the dog yet.

*Amended 11:15, before any chain set or `khc` checkpoint is scored:*
- **The own-key check pairs like with like.** `kh28_7000` is rerun on its own frozen 108 keys from the dog's stance at 1 and 2 N·m, today's code, beside each selected `khc` checkpoint. The commands are `kh28_7000_dog_f1`/`f2`'s (`keyhold/lev.sh`, `--hold_arm policy`, the link, `keyhold_dog_arm_out_66600.json`), with the weld lock. The week 9 runs are 12 Nov's code.
- **A regression is** a paired exact McNemar p < 0.05 against today's `kh28_7000` on runner_ok inside the tilt stop, over both frictions. A seed with one is reported and not sent on.
- **Scripts:** `s1tools/queue_chain_ins.sh <stem> <checkpoint>` (the insertion on the three chain sets, named `chainins_<stem>_<set>_f<F>`) and `s1tools/queue_khc.sh <stem>`:
  - spool t1: `khc_s71..73`; the first to start makes the spawn handovers (`logs/unifp_train/chain/<stem>_spawn_f<F>/`) and the merge (`chain/khc_<stem>_stances.json`);
  - spool e: each seed's four candidates on the validation handovers, named `khc_s<seed>_<it>_chainvalidation_f<F>`.

### 2026-11-19 · Item 1b: how good the dog's lock estimate must be; stated before scoring

Why: on the dog, the lock comes from the head camera's tag after the stand-up (`insert_live --lock`, still typed by hand). The policy trained with estimate errors up to 3.5 mm lateral, ±8 mm depth and ±4 deg roll (`keylock.ERR_*`). The frozen 96 puts them on a grid: 0.5-3.5 mm lateral in four directions, -6/0/+6 mm depth, ±3 deg roll. How far past that it still seats is the accuracy the tag must reach. That accuracy can be measured on the dog without moving the arm: the tag's estimate against the box's measured keyway, over several stand-ups.

**Policy:** the dog's, as item 1 decides it (`ki7`'s pick if it replaces `ki6_force_8200`, else `ki6_force_8200`), fixed before this is run.

*Amended 07:40, before any run:* `ki6_force_8200` is run now (stem `ki6_force_8200`), while the eval lane would otherwise stand idle until `ki7` seed 63's checkpoints exist. It is the dog's policy until item 1 decides. If a `ki7` replaces it, that checkpoint is run the same way afterwards, and the two are reported side by side, each against its own baseline.

**Conditions:** the dog's (strict keyway, leg-odometry base, the dog's stance, `66600`'s probe legs, the D1 link), on the frozen 96 at 1 and 2 N·m. A matched baseline at scale 1 is run in the same batch, not taken from an earlier run.

**Shifts:** every lock's error is multiplied on one axis at a time (`--insert_eval_set`), the other axes as frozen:

| shift | the grid's largest error | inside training? |
|---|---|---|
| lateral ×1.5, ×2, ×3 | 5.25, 7, 10.5 mm | no (3.5 mm) |
| depth ×1.5, ×2 | 9, 12 mm | ×1.5 no (8 mm) |
| roll ×1.5, ×2 | 4.5, 6 deg | no (4 deg) |

**Rule:** a shift matters if, pooled over both frictions (192 pairs, the same lock by index), an episode-paired exact McNemar against the baseline gives p below 0.05 and success inside the tilt stop moves by at least 10% of the baseline's count. Damaged keys, false seats, falls and peak force p90 are reported beside it. Per axis, the accuracy needed is the grid's largest error at the largest scale before the first that matters.

**What it will not show:**
- The tag's real accuracy.
- A bias that changes during the run: each lock's error is fixed for the episode.
- An error in the keyway axis's direction (the door's yaw or pitch as the tag sees it). The frozen tilt is ±0.5 deg (training 0.8) and is not scaled; the lateral error, in the face's plane, already covers height.

Scripts: `s1tools/queue_lockerr.sh <stem> <checkpoint>` (names `lockerr_<stem>_<shift>_f<F>`), `lockerr_compare.py <stem>`.

**Result for `ki6_force_8200`, 07:36-08:42** (16 runs, all exit 0; `python3 logs/unifp_train/s1tools/lockerr_compare.py ki6_force_8200`, out `lockerr_ki6_force_8200.json`). Success inside the tilt stop of 192 (both frictions), paired against the baseline:

| shift | rule | success | Δ | only baseline / only shifted, p | damaged | false seats | peak p90 N | runs |
|---|---|---|---|---|---|---|---|---|
| baseline (as frozen) | | 120 | | | 45 | 22 | 53.2 | [f1](#/week/10/run/20261118T203630_leg_eval_seed1_lockerr_ki6_force_8200_base_f1) [f2](#/week/10/run/20261118T204107_leg_eval_seed1_lockerr_ki6_force_8200_base_f2) |
| lateral ×1.5 (5.25 mm) | | 115 | -5 | 38 / 33, p 0.64 | 45 | 32 | 52.5 | [f1](#/week/10/run/20261118T204542_leg_eval_seed1_lockerr_ki6_force_8200_lat150_f1) [f2](#/week/10/run/20261118T204932_leg_eval_seed1_lockerr_ki6_force_8200_lat150_f2) |
| lateral ×2 (7 mm) | | 114 | -6 | 41 / 35, p 0.57 | 32 | 44 | 51.5 | [f1](#/week/10/run/20261118T205255_leg_eval_seed1_lockerr_ki6_force_8200_lat200_f1) [f2](#/week/10/run/20261118T205729_leg_eval_seed1_lockerr_ki6_force_8200_lat200_f2) |
| lateral ×3 (10.5 mm) | **matters** | 92 | -28 | 52 / 24, p 0.002 | 43 | 54 | 52.6 | [f1](#/week/10/run/20261118T210101_leg_eval_seed1_lockerr_ki6_force_8200_lat300_f1) [f2](#/week/10/run/20261118T210536_leg_eval_seed1_lockerr_ki6_force_8200_lat300_f2) |
| depth ×1.5 (9 mm) | | 124 | +4 | 23 / 27, p 0.67 | 44 | 22 | 52.3 | [f1](#/week/10/run/20261118T211010_leg_eval_seed1_lockerr_ki6_force_8200_dep150_f1) [f2](#/week/10/run/20261118T211445_leg_eval_seed1_lockerr_ki6_force_8200_dep150_f2) |
| depth ×2 (12 mm) | **matters** | 103 | -17 | 41 / 24, p 0.046 | 56 | 31 | 53.6 | [f1](#/week/10/run/20261118T211919_leg_eval_seed1_lockerr_ki6_force_8200_dep200_f1) [f2](#/week/10/run/20261118T212355_leg_eval_seed1_lockerr_ki6_force_8200_dep200_f2) |
| roll ×1.5 (4.5 deg) | **matters** | 63 | -57 | 72 / 15, p 4e-10 | 40 | 90 | 53.5 | [f1](#/week/10/run/20261118T212832_leg_eval_seed1_lockerr_ki6_force_8200_roll150_f1) [f2](#/week/10/run/20261118T213209_leg_eval_seed1_lockerr_ki6_force_8200_roll150_f2) |
| roll ×2 (6 deg) | **matters** | 28 | -92 | 101 / 9, p 8e-21 | 27 | 137 | 51.5 | [f1](#/week/10/run/20261118T213520_leg_eval_seed1_lockerr_ki6_force_8200_roll200_f1) [f2](#/week/10/run/20261118T213840_leg_eval_seed1_lockerr_ki6_force_8200_roll200_f2) |

- **The baseline reproduces `lead_ki6_strict_legs_f1`/`f2` field for field** (60 and 60 of 96), as it should: the runs are deterministic.
- **The accuracy needed, by the rule:**
  - lateral: 7 mm (×2 does not matter, ×3 does);
  - depth: 9 mm (×2 matters, at p 0.046, just under the bar);
  - roll: 3 deg, the frozen grid's own. ×1.5 already halves the successes, -57.
- **Roll binds.** Past 3 deg the strict keyway refuses the key at the face, and the layer takes the jam for a seat: false seats go 22 → 90 → 137, and damaged keys fall.
- **The lateral and depth errors are forgiving.** At twice the trained lateral (7 mm) and past the trained depth (9 mm against 8) the policy seats as many.

**Follow-up, stated 08:45 before it ran:** every lock on the frozen grid has a roll error of ±3 deg, so these runs cannot say what 3 deg itself costs. Roll ×0 and ×0.5 (1.5 deg) are run the same way (`spool_e/e_lockerr_09_ki6_force_8200_roll000.sh`, `_roll050.sh`) and scored by the same rule against the same baseline. A gain that matters at ×0 would put the roll the tag must reach below 3 deg.

**Follow-up result, 08:44-08:59** (both exit 0; the same scorer):

| shift | rule | success | Δ | only baseline / only shifted, p | damaged | false seats | peak p90 N | runs |
|---|---|---|---|---|---|---|---|---|
| roll ×0 (0 deg) | | 118 | -2 | 40 / 38, p 0.91 | 4 | 70 | 35.4 | [f1](#/week/10/run/20261118T214354_leg_eval_seed1_lockerr_ki6_force_8200_roll000_f1) [f2](#/week/10/run/20261118T214725_leg_eval_seed1_lockerr_ki6_force_8200_roll000_f2) |
| roll ×0.5 (1.5 deg) | | 136 | +16 | 31 / 47, p 0.09 | 20 | 35 | 50.0 | [f1](#/week/10/run/20261118T215056_leg_eval_seed1_lockerr_ki6_force_8200_roll050_f1) [f2](#/week/10/run/20261118T215419_leg_eval_seed1_lockerr_ki6_force_8200_roll050_f2) |

- **The grid's 3 deg costs no seats that matter.** With the roll exact, the policy seats 118 against 120; at 1.5 deg it seats 136 (p 0.09, under the bar). So 3 deg stands as the roll the tag must reach for seats.
- **The roll decides how a failure ends.** Paired against the baseline (reported, not in the rule):
  - damaged keys: 41 only at 3 deg, 0 only at 0 deg (p 1e-12); at 1.5 deg, 39 against 14 (p 0.0008);
  - false seats move the other way: 11 against 59 at 0 deg (p 5e-9).
  - The episodes that fail, fail either way. At 3 deg the strict keyway refuses the key and the layer presses it to damage; with the roll exact, the same keys stop short and the layer takes them for seated (F-292's jam).
- **Peak force p90** is 35 N at 0 deg against 53 N at 3 deg.

**What it means for the dog:**
- The lock's roll about the keyway's axis is what the estimate must get right: 3 deg for seats, less to spare the keys. The tag's in-plane rotation is usually its best-measured quantity.
- The key's roll in the jaws adds to it, and nothing measures that yet.
- The dog measurements are the tag's roll, lateral and depth error against the box's measured keyway over several stand-ups, and the key's roll in the gripper.

Finding: [F-302](../findings.md#f-302).

### 2026-11-19 · `ki7` seeds 61 and 62 trained, and selected on validation; late checkpoints seat keys the task layer never feels

**Training:** [`ki7_s61`](#/week/10/run/20261118T165011_train_seed61_ki7_s61) and [`ki7_s62`](#/week/10/run/20261118T165036_train_seed62_ki7_s62), 03:50-07:08, both exit 0, 1,500 iterations each from `ki6_force_8200` as stated (`s1tools/spool_t1/t_s4b_025_ki7_s61.sh`, `spool_t2/t_s4b_025_ki7_s62.sh`). Seed 63 started on t2 at 07:08.

**Validation** (the ki7 locks, 48 at each of 1 and 2 N·m; strict keyway, leg-odometry base; `python3 logs/unifp_train/s1tools/ki7_select.py ki7_s6X`). Success inside the tilt stop (f1 / f2), damaged, falls, false seats, and the rule's first key:

| seed | it | success of 96 | damaged | falls | false seats | falls + damaged | runs |
|---|---|---|---|---|---|---|---|
| 61 | 9000 | 72 (38 / 34) | 6 | 0 | 18 | 6 | [f1](#/week/10/run/20261118T184041_leg_eval_seed1_ki7_s61_9000_val_f1) [f2](#/week/10/run/20261118T184334_leg_eval_seed1_ki7_s61_9000_val_f2) |
| 61 | 9200 | 65 (27 / 38) | 8 | 0 | 23 | 8 | [f1](#/week/10/run/20261118T190630_leg_eval_seed1_ki7_s61_9200_val_f1) [f2](#/week/10/run/20261118T190927_leg_eval_seed1_ki7_s61_9200_val_f2) |
| 61 | **9400** | 59 (28 / 31) | 5 | 0 | 22 | 5 | [f1](#/week/10/run/20261118T193102_leg_eval_seed1_ki7_s61_9400_val_f1) [f2](#/week/10/run/20261118T193450_leg_eval_seed1_ki7_s61_9400_val_f2) |
| 61 | 9699 | 19 (3 / 16) | 9 | 0 | 5 | 9 | [f1](#/week/10/run/20261118T200722_leg_eval_seed1_ki7_s61_9699_val_f1) [f2](#/week/10/run/20261118T201119_leg_eval_seed1_ki7_s61_9699_val_f2) |
| 62 | 9000 | 77 (38 / 39) | 5 | 0 | 14 | 5 | [f1](#/week/10/run/20261118T184640_leg_eval_seed1_ki7_s62_9000_val_f1) [f2](#/week/10/run/20261118T184945_leg_eval_seed1_ki7_s62_9000_val_f2) |
| 62 | **9200** | 72 (34 / 38) | 2 | 0 | 22 | 2 | [f1](#/week/10/run/20261118T191216_leg_eval_seed1_ki7_s62_9200_val_f1) [f2](#/week/10/run/20261118T191504_leg_eval_seed1_ki7_s62_9200_val_f2) |
| 62 | 9400 | 70 (35 / 35) | 3 | 0 | 23 | 3 | [f1](#/week/10/run/20261118T193745_leg_eval_seed1_ki7_s62_9400_val_f1) [f2](#/week/10/run/20261118T194137_leg_eval_seed1_ki7_s62_9400_val_f2) |
| 62 | 9699 | 53 (29 / 24) | 9 | 0 | 32 | 9 | [f1](#/week/10/run/20261118T201537_leg_eval_seed1_ki7_s62_9699_val_f1) [f2](#/week/10/run/20261118T201954_leg_eval_seed1_ki7_s62_9699_val_f2) |

**Selected by the stated rule:** seed 61 `9400`, seed 62 `9200` (`s1tools/ki7_s61_selected.json`, `ki7_s62_selected.json`).
- Seed 61's `9000` had 13 more successes for one more damaged key. The rule puts damage first, so `9400` stands.
- `9400` is already partly in the drift below: 9 of 48 at 1 N·m end seated but unfinished.

**Late checkpoints seat keys the task layer never feels.** Seed 61's `9699` touched in 4 of 48 episodes at 1 N·m:
- 24 end with the key seated, and the layer never finishes (`seated_no_finish`). All 24 are seated at 8.7-9.3 mm with 17-42 N of contact, yet the layer never felt a touch.
- 16 end with no touch at all.
- The layer feels a touch as the arm lagging behind its reference (`keylock.InsertLayer.step`: ref minus the measured depth, past `CONTACT_LAG_M` for `CONTACT_HOLD_S`). The measured depth is computed from the reported joints and, here, the leg-odometry base.

**Training's own numbers show it.** Binned per 200 iterations, sampled at reset, so this is not the validation:

| | 9000 | 9200 | 9400 | 9600 |
|---|---|---|---|---|
| seed 61, touched fraction | 0.94 | 0.74 | 0.35 | 0.24 |
| seed 62, touched fraction | 0.94 | 0.88 | 0.84 | 0.86 |
| seed 61, mean reward | 249 | 260 | 281 | 291 |
| seed 61, episode length | 764 | 790 | 847 | 873 |

- The reward pays the true key (`ins_seated`, `ins_depth`), not the layer's finish.
- An episode with no felt touch never reaches the layer's search and seat, so the press and the damage stop that comes with it never happen, and the episode runs longer.
- Seed 61's seated fraction holds at 0.57-0.65 meanwhile.
- So the policy found a way into the keyway that the layer does not register. This is a hole in the ki7 recipe's reward, not something the dog could use: on the dog an unfinished layer never hands over.

**Diagnostic, 07:25-07:33, not for selection:** `ki7_s61_9699` on validation at 1 N·m with the body trace, under the true base and again under the leg odometry (`spool_e/e_ki7d_61_9699_true.sh`). The question was whether the unfelt seats rest on the odometry missing a base slide (F-296) or on the arm's goal masking the lag. Odometry checked with `python3 demos/combiner_key/robot/check_odometry.py <run>`:

| run | success of 48 | touched | seated, unfinished | damaged | feet slide (median) | base forward | odometry error forward |
|---|---|---|---|---|---|---|---|
| [true base](#/week/10/run/20261118T202504_leg_eval_seed1_ki7d_s61_9699_true_val_f1) | 30 | 48 | 0 | 11 | 96 mm | 57 mm | (26 mm, not used) |
| [leg odometry](#/week/10/run/20261118T202922_leg_eval_seed1_ki7d_s61_9699_legs_val_f1) | 3 | 4 | 24 | 4 | 72 mm | 38 mm | 18.6 mm |
| `ki6_force_8200`, frozen 96, leg odometry ([`lead_ki6_strict_legs_f1`](#/week/10/run/20261118T165859_leg_eval_seed1_lead_ki6_strict_legs_f1)) | | | | | 4 mm | 14 mm | 1.0 mm |

- **The unfelt seats rest on the odometry.** With the true base the layer feels every touch and seats 30. With the leg odometry it feels 4.
- **The cause is the feet sliding.** The late checkpoint slides its feet 72-96 mm forward during the insertion, where `ki6` slides them 4 mm. The odometry assumes stance feet stay put, so it is 18.6 mm out along the axis.
- **The layer's measured hand reads deep.** In the leg-odometry run it is 6-12 mm deeper than the true tip and ahead of the layer's own reference, past its 22.7 mm maximum, so the lag that signals a touch never builds. The body meanwhile tilts up to 0.15-0.18 rad.
- **The lean pushes the key.** The key goes in under the body's push, outside the layer's force law, at 17-42 N.
- **The traced run equals the untraced one,** field for field in all 48 episodes (`ki7_s61_9699_val_f1`), so the body trace changes nothing. It is added to `ki7`'s test runs (`queue_ki7_test.sh`), and `ki7_compare.py` reports the feet's slide and the odometry's error beside the stated measures.

**What it means for `ki7`:**
- The recipe pays the true seat. With the leg odometry in the loop, it pays a lean on sliding feet that the dog's estimate cannot see.
- The selected checkpoints come before most of the drift: seed 62's `9200` shows none at validation, seed 61's `9400` 9 unfinished seats of 48 at 1 N·m. Their slide is measured on the test.
- A retrain would pay the seat only when the layer finishes, and price the feet's slide.
- **On the dog:** the real feet's slide under a forward press is not measured. That is the dog measurement this needs, the feet's positions while the body leans into a press (`insert_live` logs the legs).

Finding: [F-301](../findings.md#f-301).

### 2026-11-19 · `ki7` seed 63 trained, and `9699` selected on validation; it does not drift

**Training:** [`ki7_s63`](#/week/10/run/20261118T200851_train_seed63_ki7_s63), 07:08-10:37, exit 0, 1,500 iterations from `ki6_force_8200` as stated (`s1tools/spool_t2/t_s4b_035_ki7_s63.sh`, beside S4b arm A seed 92 on t1).

**Validation**, as for seeds 61 and 62 (`python3 logs/unifp_train/s1tools/ki7_select.py ki7_s63`):

| seed | it | success of 96 | damaged | falls | false seats | falls + damaged | runs |
|---|---|---|---|---|---|---|---|
| 63 | 9000 | 56 (28 / 28) | 7 | 0 | 30 | 7 | [f1](#/week/10/run/20261118T220702_leg_eval_seed1_ki7_s63_9000_val_f1) [f2](#/week/10/run/20261118T221113_leg_eval_seed1_ki7_s63_9000_val_f2) |
| 63 | 9200 | 49 (22 / 27) | 14 | 0 | 31 | 14 | [f1](#/week/10/run/20261118T223412_leg_eval_seed1_ki7_s63_9200_val_f1) [f2](#/week/10/run/20261118T223825_leg_eval_seed1_ki7_s63_9200_val_f2) |
| 63 | 9400 | 57 (27 / 30) | 10 | 0 | 27 | 10 | [f1](#/week/10/run/20261118T230003_leg_eval_seed1_ki7_s63_9400_val_f1) [f2](#/week/10/run/20261118T230408_leg_eval_seed1_ki7_s63_9400_val_f2) |
| 63 | **9699** | 62 (31 / 31) | 5 | 0 | 28 | 5 | [f1](#/week/10/run/20261118T233853_leg_eval_seed1_ki7_s63_9699_val_f1) [f2](#/week/10/run/20261118T234324_leg_eval_seed1_ki7_s63_9699_val_f2) |

**Selected by the stated rule:** `9699` (`s1tools/ki7_s63_selected.json`): the fewest damaged keys, and the most successes too.

**Seed 63 does not show F-301's drift:**
- `9699` has no seated-but-unfinished episodes in 96. Its misses are 28 false seats, 5 damaged keys and 1 still searching. Seed 61's `9699` had 35 unfinished and 28 with no touch.
- In training, seed 63's touched fraction stays at 0.94-0.99 from 8800 to 9699 (sampled at reset, binned per 200 iterations, from `s1tools/ki7_s63.out`). Seed 61's fell to 0.24.
- Seed 63's false seats (27-31 per checkpoint) run higher than seeds 61 and 62 at their selected checkpoints (22 each). False seats are ki6's failure (F-292), not the lean.
- Whether its feet slide is measured on the test (`ki7_compare.py` reports it). Validation ran without the body trace.

**The test is queued, read once** (`bash logs/unifp_train/s1tools/queue_ki7_test.sh`, 10:47). `ki6_force_8200`, `ki7_s61_9400`, `ki7_s62_9200` and `ki7_s63_9699` each run on the ki7 test locks at 1 and 2 N·m under `strict_legs` and `default_true`, 16 runs on lane e. Scored by `python3 logs/unifp_train/s1tools/ki7_compare.py`.

### 2026-11-19 · S4b arm A seed 92 trained, and `68000` selected on validation

**Training:** [arm A, seed 92](#/week/10/run/20261118T200656_train_seed92_s4b_A_s92) (law v1 with the bleed), 07:06-10:01, exit 0, as seed 91 (`spool_t1/t_s4b_03_s4b_A_s92.sh`). Arm B seed 92 started on t2 at 10:37, after `ki7` seed 63; arm A seed 93 started on t1 at 10:01.

**Validation**, as for seed 91 (`python3 logs/unifp_train/s1tools/s4b_select.py s4b_A_s92`):

| checkpoint | 1 N·m in budget / over / falls / tilt stops | 2 N·m | stops (1+2) | in budget (of 768) |
|---|---|---|---|---|
| **68000** | [304](#/week/10/run/20261118T221538_mech_eval_seed1_s4b_A_s92_68000_val_f1) / 0 / 0 / 0 | [299](#/week/10/run/20261118T221741_mech_eval_seed1_s4b_A_s92_68000_val_f2) / 0 / 0 / 7 | 7 | **603** |
| 68200 | [295](#/week/10/run/20261118T222956_mech_eval_seed1_s4b_A_s92_68200_val_f1) / 0 / 0 / 9 | [298](#/week/10/run/20261118T223159_mech_eval_seed1_s4b_A_s92_68200_val_f2) / 0 / 0 / 5 | 14 | 593 |
| 68400 | [299](#/week/10/run/20261118T224638_mech_eval_seed1_s4b_A_s92_68400_val_f1) / 0 / 0 / 5 | [244](#/week/10/run/20261118T224840_mech_eval_seed1_s4b_A_s92_68400_val_f2) / 0 / 0 / 53 | 58 | 543 |
| 68599 | [299](#/week/10/run/20261118T230833_mech_eval_seed1_s4b_A_s92_68599_val_f1) / 0 / 0 / 2 | [296](#/week/10/run/20261118T231036_mech_eval_seed1_s4b_A_s92_68599_val_f2) / 0 / 0 / 10 | 12 | 595 |

- **Selected:** `68000` (7 stops, 603 in budget, none over). Written to `s1tools/s4b_A_s92_selected.json`.
- No falls and nothing over the 80 N budget in any of the 8 runs. Arm A stayed inside the budget at every checkpoint of both seeds so far.
- Seed 92's best (603) is below seed 91's (642), and its stops are all tilt stops. Validation only; this chooses the checkpoint, nothing else.

### 2026-11-19 · Item 2, S4b's dog side: a mechanism export, a numpy layer and a replay gate; the gate's thresholds stated before it runs

Built by a subagent and checked here (CPU only, nothing on the GPU, nothing on the dog). It is the S5 pattern for T1:
- `demos/combiner_key/robot/export_policy.py --task mechanism --train_run <run>` (`--law_gains`, `--law_bleed_s`): the UniFP network plus a meta with the law (evaluation and training, B's drawn 30-100 N cap), the travel plan, the approach, the goal-sphere conventions, the tool, the roll channels, the stance and the tilt stop. It refuses runs trained without the law, position-only ones, and tools other than Link6.
- `demos/combiner_key/robot/mech_layer.py`, numpy only: the approach, the grasp rule, `TravelGoal`'s evaluation plan, the law (anti-windup, bleed, clamp) and the goal on the handle, in the env's step order. Every input is explicit. `MechPath.project`, `measure_travel` and `tool_speed_estimate` are the dog's ways to read the handle's travel and the tool's speed: the hand by forward kinematics of the reports plus `LegOdometry`, projected on the path. Not validated.
- `unifp_train/mech_shadow.py` and `--mech_shadow_dump N` on `mech_eval` (off by default, 9 lines in `run_unifp_train.py`, swapped in by rename while the queue ran): every raw input and output for N environments spread over the plan, with each one's grasp error.
- `demos/combiner_key/robot/check_mech.py` replays a dump through the layer and the policy, and reports what the dog would read for the travel from four sources.
- `demos/combiner_key/robot/mech_gate.py`, the replay gate on `66600`'s four recorded stands.
- `tests/test_mech_layer.py`: 20 tests. The real `Go2D1MechEnv` is compiled onto a CPU stand-in base env with the real `Mechanism`, `TravelGoal` and goals. Run here with `tests.test_insert_policy tests.test_mechanism tests.test_s4b tests.test_insert_live`: 84 tests OK.

**Parity so far:**
- Exports of `s4b_A_s91_68200` and `s4b_B_s91_68200` (scratchpad, not `checkpoints/`). Numpy against torch on 64 random stacks: actions 1.6-1.7e-5, at most 8e-6 relative, since these networks put out up to 68 there. On `66600`'s recorded stand frames: 3.4-3.9e-6, S5's order.
- The layer against the env's code on the stand-in, 16 episodes and 5,264 steps: the reference, the integral and the step counters match bit for bit, and the grasp rule disagrees on no step. Force command ≤ 9.9e-6 N, goals ≤ 1.4e-7. The robot there is scripted, so this shows the layer computes what the env computes from the same state, not what a policy does.

**What the simulator does that the dog cannot (the subagent's reading, checked in `mech_env` here):**
- **The grasp cannot miss.** `_engage_due` builds the mechanism at the tool's position when the settle rule fires, in evaluation too (`self._mech.grasp(ids, point, ...)`, `point = self._tip_pos()`). The planned handle sets only the opening's bearing. This is F-115's limit, and it holds for S4 and S4b as for week 2. So "opened" in F-294, F-295 and S4b counts the opening once attached, not reaching the handle. The arrival error is not scored at evaluation. In training it grows (`Mechanism/grasp_error_m`, medians per 500 iterations from `s1tools/s4b_<arm>_s91.out`): S4b A s91 6.8 → 12.6 cm, B s91 6.1 → 10.1 → 8.2 cm. The subagent read S4 (i) s81 6.3 → 15.0 cm from its tfevents, not checked here. Those are training numbers sampled at the grasp, not the evaluation's.
- **The true travel is not measurable.** Read from the hand, it carries the grasp spring's stretch (about 4 cm at 80 N in the simulator), so the dog's law would ask for less force than training's. The goal changes meaning too: in training it is the handle; read from the hand it becomes the hand's own projection.
- **It is not `66600`'s interface.** This policy controls the jaw centre (Link6 + 0.1051 m, CAD) with the roll channels on. `66600` controls the Link7_1 CAD fingertip with roll off. `press_live`'s reach command must not be reused for it, and no mechanism runner exists on the dog.
- Smaller: training's grasp is instant, where the D1's jaws take about 2 s; the settle rule reads the true tool speed, where the dog has 9 Hz report differences.

**Queued, 11:45:** the shadow dump, 48 environments, on S4b validation at 1 N·m for `s4b_A_s91_68200` and `s4b_B_s91_68200` (`spool_e/e_mshadow_01_s4b_s91_68200.sh`, `s4/mev.sh ... --mech_shadow_dump 48`). For the policy's parity through the layer, and the evaluation's grasp error. The dump is read-only if each run's totals equal its validation run without it (A: 328 in budget, 0 over, 0 falls, 0 tilt stops).

**The replay gate's thresholds, stated before it is run on any candidate** (`mech_gate.py --mode hold`, all four stands × the 12 S4b validation placements; the subagent's proposal, adopted unchanged):
1. J1-J5's target lead over the report ≤ 15 deg from 1 s on (F-256's).
2. Leg targets ≤ 1.2 rad from the stand in the first second, ≤ 0.9 rad after (F-256's).
3. The grasp fires by the settle rule on the reports' tool speed (< 0.10 m/s), not by the 50-step fallback.
4. The hand (forward kinematics of the reports) within 2 cm of the handle when the grasp fires. This comes from what a gripper needs, not from the policy: F-115's lip claw hooked 2-5 mm off and missed 36-42 mm off. It stays at 2 cm whatever the dump shows.
5. The hand within 3 cm of the handle from 1 s after the grasp to the end.

`hold` asks for no travel, so the law asks for nothing. The gate checks the approach, the arm through the link, the grasp rule on the reports, and holding the goal on the handle. It cannot check the law, the opening, the force, a latch, a tear, the jaws, or the legs answering this policy (it replays `66600`'s). `free` (the travel read from the hand with nothing resisting) and `stuck` (the travel read as 0: the law reaches the 80 N cap about 0.93 s after the grasp and asks for the hand about 0.4 m past the handle) are reported without marks.


**The dump's result, 12:00-12:03** ([A](#/week/10/run/20261119T005753_mech_eval_seed1_s4b_A_s91_68200_val_f1_mechshadow), [B](#/week/10/run/20261119T010026_mech_eval_seed1_s4b_B_s91_68200_val_f1_mechshadow)):
- **Read-only:** each run's 384 episodes equal its validation run's without the dump, field for field (A 328 in budget, B 309).
- **Parity**, `python demos/combiner_key/robot/check_mech.py --policy <export> --sim <run>/mech_shadow_dump.npz --out <run>/check_mech.json --brief` (exports re-made in the scratchpad, actions 2.0-2.7e-5 on random stacks). On 48 episodes, 33,552 steps per arm: actions ≤ 2.9e-6, the law's force command ≤ 1.5e-5 N, goals ≤ 1.3e-7, frames ≤ 1.8e-7, the grasp rule disagreeing on no step. The numpy layer and policy reproduce Isaac.

**What the dump shows about these policies on the dog** (S4b validation at 1 N·m, 48 episodes spread over the plan, each arm):

| | A s91 `68200` | B s91 `68200` |
|---|---|---|
| hand to handle goal at the grasp, median (q10-q90) | 6.4 cm (4.2-11.6) | 6.9 cm (2.9-11.1) |
| grasps within 2 cm / 3 cm | 0 / 2 of 48 | 3 / 7 of 48 |
| J1-J5 target lead over the link's report, per step from 1 s: median, p90 | 53 deg, 133 | 65 deg, 134 |
| steps over the runner's 15 deg | 84% | 85% |
| episodes whose 0.5 s running median stays within 15 deg | 0 of 48 | 0 of 48 |
| base moved from the grasp to the end, median (q90) | 85 mm (284) | 78 mm (259) |
| leg odometry's base error, median (q90) | 4.8 cm (11.1) | 4.5 cm (12.3) |
| the handle's travel read by FK of the reports + odometry, error median (q10-q90) | −1.5 cm (−9.9 to 1.5) | −2.3 cm (−9.1 to 1.5) |
| the law's command from that reading, minus the true law's, median | +20 N | +62 N |
| the same with the true base | −6 N | −1 N |

- **No grasp is good enough for a gripper.** The policies settle 6-7 cm from the handle. In simulation the grasp closes there anyway (F-115).
- **The arm runs far ahead of itself.** The targets lead the D1 link by a median 53-65 deg once holding, and 18-27 deg before the grasp. `ArmWatch` would stop every episode. For comparison, the insertion leads by at most 6.8 deg in contact (F-299). S4's recipe prices the arm's step (`--arm_step_weight -30`) but not its lead. The key hold, which week 9 scored inside the runner's bounds, trained with `--arm_lead_weight -30`.
- **The legs move under the hold.** The base travels a median 8 cm while the mechanism is held, so the leg odometry is about 5 cm out. A law fed the travel through it over-commands by 20-62 N at the median. The travel needs another source on the dog: a tag on the handle seen from the body, or the odometry fixed.

**Preview of the gate, not the pick** (`python3 demos/combiner_key/robot/mech_gate.py --policy <export> --out logs/unifp_train/s4bgate_preview_<arm>_s91_68200 --modes hold --workers 6`): both fail on all 48 rows (4 stands × 12 placements). The lead from 1 s is 27-74 deg (A) and 16-86 deg (B). The hand is 5.1-36.5 cm (A) and 8.2-23.5 cm (B) from the handle at the grasp, median 14.6 cm for both. In 30 of 48 rows the grasp comes from the 50-step fallback, because the reports' speed never settles under 0.10 m/s. The leg-target mark passes in all 48, and the lead, the grasp on the handle and the hold on the handle pass in none. This agrees with the dump: the failure is the policies', not the replay's.

**So S4b does not give a T1 policy fit for the dog.** The test still runs as stated, once all six are selected, and answers its own question (A against B, and the 40 N cap). Its pick will be gated as stated and is expected to fail on the lead and the arrival. Finding: [F-304](../findings.md#f-304).

### 2026-11-19 · `ki7`'s test, read once: `ki7_s62_9200` replaces `ki6_force_8200` for the dog; exported, and it passes S5's replay gate

**Runs** (`bash logs/unifp_train/s1tools/queue_ki7_test.sh`, lane e, 10:48-11:57, 16 runs, all exit 0): the three selected seeds and `ki6_force_8200` on the ki7 test locks (96 at each of 1 and 2 N·m), with the body trace. Scored once by `python3 logs/unifp_train/s1tools/ki7_compare.py` (out `s1tools/ki7_test.json`). Its first run stopped before printing anything on a name error in the slide report (`v` for `stems[k]`); fixed, nothing else changed.

**The dog's conditions** (strict keyway, leg-odometry base). Success inside the tilt stop of 192 (f1 / f2), damaged, false seats, peak force median / p90, the feet's slide and the odometry's forward error (medians; one value where f1 and f2 agree, else f1 / f2), and the paired count against `ki6` (only ki7 / only ki6, exact McNemar):

| policy | success | damaged | false seats | peak N | feet slide | odometry error | vs `ki6` | runs |
|---|---|---|---|---|---|---|---|---|
| `ki6_force_8200` | 114 (51 / 63) | 26 | 52 | 35.1 / 50.8 | 4-5 mm | 1.2-1.5 mm |  | [f1](#/week/10/run/20261118T234806_leg_eval_seed1_ki7t_ki6_force_8200_strict_legs_f1) [f2](#/week/10/run/20261118T235659_leg_eval_seed1_ki7t_ki6_force_8200_strict_legs_f2) |
| `ki7_s61_9400` | 110 (55 / 55) | 25 | 41 | 30.6 / 51.4 | 47 / 18 mm | 10.5 / 3.0 mm | 43 / 47, p 0.75 | [f1](#/week/10/run/20261119T000639_leg_eval_seed1_ki7t_ki7_s61_9400_strict_legs_f1) [f2](#/week/10/run/20261119T001535_leg_eval_seed1_ki7t_ki7_s61_9400_strict_legs_f2) |
| **`ki7_s62_9200`** | **140** (66 / 74) | 4 | 48 | 29.8 / 44.5 | 7-8 mm | 1.2-1.3 mm | 51 / 25, p 0.004 | [f1](#/week/10/run/20261119T002449_leg_eval_seed1_ki7t_ki7_s62_9200_strict_legs_f1) [f2](#/week/10/run/20261119T003303_leg_eval_seed1_ki7t_ki7_s62_9200_strict_legs_f2) |
| `ki7_s63_9699` | 130 (63 / 67) | 18 | 44 | 27.6 / 45.5 | 10 / 7 mm | 2.0 / 1.5 mm | 48 / 32, p 0.09 | [f1](#/week/10/run/20261119T004101_leg_eval_seed1_ki7t_ki7_s63_9699_strict_legs_f1) [f2](#/week/10/run/20261119T004926_leg_eval_seed1_ki7t_ki7_s63_9699_strict_legs_f2) |

- Seed 61 also ends 10 keys seated with the layer unfinished, F-301's sign. Seeds 62 and 63 end none.
- Pooled over the seeds: 380 successes against 3 × 114 = 342, damaged 47 against 3 × 26 = 78 (paired 142 / 104, p 0.018).

**The default contact with the true base**, where `ki6` was trained:

| policy | success | damaged | false seats | peak N | feet slide | vs `ki6` | runs |
|---|---|---|---|---|---|---|---|
| `ki6_force_8200` | 136 (63 / 73) | 12 | 43 | 27.4 / 38.7 | 19-21 mm |  | [f1](#/week/10/run/20261118T235210_leg_eval_seed1_ki7t_ki6_force_8200_default_true_f1) [f2](#/week/10/run/20261119T000203_leg_eval_seed1_ki7t_ki6_force_8200_default_true_f2) |
| `ki7_s61_9400` | 136 (73 / 63) | 1 | 55 | 22.7 / 30.8 | 67 / 52 mm | 44 / 44, p 1.0 | [f1](#/week/10/run/20261119T001130_leg_eval_seed1_ki7t_ki7_s61_9400_default_true_f1) [f2](#/week/10/run/20261119T002037_leg_eval_seed1_ki7t_ki7_s61_9400_default_true_f2) |
| `ki7_s62_9200` | 115 (63 / 52) | 1 | 77 | 23.1 / 31.4 | 9-11 mm | 35 / 56, p 0.035 | [f1](#/week/10/run/20261119T002849_leg_eval_seed1_ki7t_ki7_s62_9200_default_true_f1) [f2](#/week/10/run/20261119T003645_leg_eval_seed1_ki7t_ki7_s62_9200_default_true_f2) |
| `ki7_s63_9699` | 132 (60 / 72) | 18 | 42 | 25.0 / 43.6 | 27 / 9 mm | 36 / 40, p 0.73 | [f1](#/week/10/run/20261119T004504_leg_eval_seed1_ki7t_ki7_s63_9699_default_true_f1) [f2](#/week/10/run/20261119T005330_leg_eval_seed1_ki7t_ki7_s63_9699_default_true_f2) |

- Pooled: 383 against 408 (paired 115 / 140, p 0.13).

**The decision, by the rule stated before training** (pooled ki7 damaged below 3 × ki6's and successes no fewer): 47 < 78 and 380 ≥ 342, so **a `ki7` replaces `ki6_force_8200`**. The seed with the most validation successes is 62 (72 of 96, against 59 and 62), so **`ki7_s62_9200` goes to the dog**. It is also the best on the test, which is not why it was chosen.

**What it shows:**
- On the dog's conditions the retrain mostly removes broken keys: `s62` damages 4 of 192 where `ki6` damages 26, and seats 26 more.
- It is not every seed. Seed 61 neither seats more nor damages fewer, and its feet slide 47 mm at 1 N·m, F-301's lean, even at the selected checkpoint.
- The false seats stay: `s62` has 48 against `ki6`'s 52. The retrain does not touch F-292.
- It cost `s62` where `ki6` was trained: 21 fewer seats on the default contact with the true base, its false seats 43 → 77. The dog's keyway is closer to the strict one than to the default, but that is not measured either.
- `s62`'s feet slide 7-11 mm and its odometry is 1.2-1.7 mm out, `ki6`'s order. On the dog the feet's slide is still the measurement this needs (F-301).

**Exported:** `python demos/combiner_key/robot/export_policy.py --task insert --checkpoint logs/unifp_train/keyinsert/ki7_s62_9200.pt --variant force --train_run logs/unifp_train/20261118T165036_train_seed62_ki7_s62 --out checkpoints/insert_force_ki7_s62_9200.npz`. Numpy against torch on 64 random stacks: actions 5.3e-6, estimates 2.3e-6. Its meta records the strict keyway, the leg-odometry base and the damage stop as trained with.

**S5's replay gate, S5's thresholds unchanged:** `python3 demos/combiner_key/robot/insert_gate.py --policy checkpoints/insert_force_ki7_s62_9200.npz --out logs/unifp_train/s5gate_ki7_s62_9200 --workers 6` (CPU, 15 s). **Pass** on all four stands, each at the five placements, with the yaw as recorded and as zero:

| measure | `ki7_s62_9200` | `ki6_force_8200` (S5) | bound |
|---|---|---|---|
| J1-J5 lead from 1 s | 1.7-3.8 deg | 1.2-3.6 deg | ≤ 15 |
| leg targets, first s / after | 0.37-0.39 / 0.24-0.28 rad | 0.38-0.39 / 0.22-0.30 | ≤ 1.2 / 0.9 |
| IK failures | 0 of 101 everywhere | 0 | 0 |
| touches short of the face | 0 | 0 | 0 |

**Queued on the decision, 12:00:**
- Item 3: `queue_chain_ins.sh ki7_s62_9200 ...` (the insertion on the chain's spawn, validation and test sets, 6 runs), then `queue_khc.sh ki7_s62_9200` (`khc_s71..73` on t1 from about 13:30, after S4b A seed 93; their validation on lane e).
- Item 1b: `queue_lockerr.sh ki7_s62_9200 ...` with the roll ×0 and ×0.5 follow-up added to its shifts (20 runs), to report beside `ki6`'s F-302. The script's scale parse failed on `050` (a leading zero in Python) and wrote an empty `roll_scale=`. Fixed in the script and in the queued job before it ran.

**Docs:** `insert_live.py`'s usage now names `insert_force_ki7_s62_9200.npz` (its 11 tests pass), and `docs/tool_paper_plan.md`'s "Results so far" gains F-303 and F-294's grasp note.

Finding: [F-303](../findings.md#f-303).

### 2026-11-19 · S4c-0: do S4b's policies reach the handle if the grasp waits for them? Stated before scoring

F-304: S4b's policies grasp 6-7 cm from the handle. The grasp fires once the tool slows under 0.10 m/s, or 1 s after the engage step whatever it does (`hook_cfg.ENGAGE_*`), so it may close while the hand is still on its way. If the policies reach the handle when the grasp waits for them, the 6-7 cm is the rule's, as F-295's violations were the law's; if they do not, arrival has to be trained.

**Code (12:05-12:17, off by default):**
- `--mech_grasp_within_m R` (`Go2D1MechEnvCfg.grasp_within_m`, `_engage_due`): the grasp closes only with the tool within R of its goal (the handle), the fallback included, and never otherwise.
- `--mech_engage_max_wait_s` sets the fallback's wait. Not used here.
- `mech_eval` records per step the tool's distance from its goal, the env's grasp error and J1-J5's target lead over the D1 link's report. Per episode it reports `grasp_error_m`, `closest_to_goal_m`, `lead_deg_max_after_1s`, `lead_over15_share_after_1s` and `lead_deg_median_held`. Every later mechanism evaluation carries these; none of the stated measures changes.
- `tests/test_s4c.py`, 7 tests: on the stand-in, with the gate off the grasps are bit for bit the old rule's; with it, every grasp closes within R; a radius never reached gives no grasp; old traces score as before. Run with `tests.test_mech_layer tests.test_s4b tests.test_mechanism`: 62 OK. The files were edited as copies and swapped in by rename while the queue ran.

**Runs:** `s4b_A_s91_68200` and `s4b_B_s91_68200` on S4b validation at 1 and 2 N·m from the dog's stance, the law at 80 N with the bleed, as their validation (`s4/mev.sh ... s4b_validation --force_law --force_law_bleed 1.0 --dog_stance_fraction 1`). Two conditions:
- `plain`: unchanged, for the new fields on all 768 episodes. It must reproduce the validation runs' stated measures.
- `gate2`: plus `--mech_grasp_within_m 0.02`. The 2 cm is a gripper's, F-115's, not the policies'.

**Measures:** grasped (arrived within 2 cm under `gate2`), opened within the budget, over the budget, tilt stops, falls; the grasp error's and the closest approach's distributions; the lead's.

**Reading, stated now:**
- Under `gate2`, if most episodes grasp (≥ 3/4 of 768 per arm) and open within the budget at no fewer than 3/4 of `plain`'s count, the arrival is the rule's. Then S4c's retrain needs the lead priced, and the gate kept on at evaluation.
- If fewer, arrival has to be trained: the gate on in training as well.
- Diagnostic, seed 91 only, on validation, so it does not touch the S4b test.

### 2026-11-19 · S4c-0's result: S4b's policies do not reach the handle when the grasp waits; arrival has to be trained

**Runs** (`logs/unifp_train/s1tools/spool_e/e_kz_s4c0_s91_68200.sh`, lane e, 12:26-12:44, 8 runs, all exit 0): [A plain f1](#/week/10/run/20261119T012618_mech_eval_seed1_s4c0_A_s91_68200_plain_f1), [A gate2 f1](#/week/10/run/20261119T012833_mech_eval_seed1_s4c0_A_s91_68200_gate2_f1), [A plain f2](#/week/10/run/20261119T013048_mech_eval_seed1_s4c0_A_s91_68200_plain_f2), [A gate2 f2](#/week/10/run/20261119T013302_mech_eval_seed1_s4c0_A_s91_68200_gate2_f2), [B plain f1](#/week/10/run/20261119T013515_mech_eval_seed1_s4c0_B_s91_68200_plain_f1), [B gate2 f1](#/week/10/run/20261119T013727_mech_eval_seed1_s4c0_B_s91_68200_gate2_f1), [B plain f2](#/week/10/run/20261119T013941_mech_eval_seed1_s4c0_B_s91_68200_plain_f2), [B gate2 f2](#/week/10/run/20261119T014154_mech_eval_seed1_s4c0_B_s91_68200_gate2_f2).

**`plain` reproduces the validation runs** field for field in all 768 episodes per arm (A 642 opened within the budget, B 618), as it must.

**The stated measures** (768 episodes per arm, 1 + 2 N·m):

| | A `plain` | A `gate2` | B `plain` | B `gate2` |
|---|---|---|---|---|
| grasped | 768 | **223** (29%) | 768 | **329** (43%) |
| opened within the budget | 642 | **164** (26% of `plain`'s) | 618 | **209** (34%) |
| over the budget | 0 | 0 | 3 | 1 |
| tilt stops | 1 | 0 | 1 | 10 |
| falls, torn | 0, 0 | 0, 0 | 0, 0 | 0, 0 |
| grasp step, median (q10-q90) | 127 | 323 (150-612) | 132 | 361 (162-625) |

**The reading, as stated:** under `gate2` both arms grasp far fewer than 3/4 of 768 (223 and 329 against 576) and open far fewer than 3/4 of `plain`'s count (164 and 209 against 482 and 464). **The arrival is not the rule's: it has to be trained**, with the gate on in training too.

- The shortfall is spread over every kind and level (A grasps 3-11 of 24 per cell, B 6-14), so it is not one mechanism's geometry.
- The grasps that do come come late: a median 6.5-7.2 s into a 14 s episode against 2.5-2.6 s, so the hand drifts within 2 cm rather than reaching. *(Corrected 15:40: late gated grasps are mostly at UniFP's next goal, not the handle; see "The gate measured the tool against UniFP's next goal" below.)*
- Why the policies stop short, from the code: before the grasp the only pay for the hand's place is UniFP's tracking term, `exp(-2·L1)` (`TRACKING_EE_SIGMA` 1.0), which still pays 0.82 at 6 cm off. After the grasp the mechanism is built where the hand is (`mech_env._engage_due`), so nothing in S4b's training paid the last centimetres. The tracked point is the grasp's point (`_tip_pos`), so it is not an offset between the two.

**What the first runs lost.** The evaluation loop keeps only `mech_eval.FIELDS` of what `record_step` returns (`run_unifp_train.py` `mech_evaluate`), and the new per-step fields were not in it. So these eight traces have no tool-to-goal distance, grasp error or lead, and their episodes no `closest_to_goal_m`, `grasp_error_m` or `lead_*`. `tests/test_s4c.py` tested `score_episode` on a hand-built trace, not the loop. The stated measures above do not use them, and the gate itself acted (it is the env's). Fixed 13:27:
- the three fields are added to `FIELDS`;
- `closest_to_goal_m` is now taken up to the grasp, the handle's approach; after it the goal is the held one on the path;
- two tests: the new keys are in `FIELDS`, and the stand-in's `record_step` returns every key of `FIELDS`.

The change was made on a copy, tested (`tests.test_s4c tests.test_mech_layer tests.test_s4b tests.test_mechanism`, 64 OK) and swapped in by rename with the live file checked unchanged.

**S4c-0b, queued 13:29, started 13:32 (diagnostic, validation only):** `gate2` at 1 N·m again for A and B with the fields kept, for the ungrasped episodes' closest approach and the lead before the grasp. Also the parent `66600` under `gate2` and `plain` at 1 N·m: whether S4b's training lost an arrival `66600` had, or `66600` never had it on this task. The A and B `gate2` re-runs must give the first runs' f1 counts (A 88 grasped and 69 opened within the budget of 384, B 183 and 126).

### 2026-11-19 · Correction to F-304: the dog runner's lead stop is 25 deg held 0.5 s or 60 deg at once, not 15

F-304, its log entry above ("steps over the runner's 15 deg", "0.5 s running median within 15 deg") and my memory notes called 15 deg the runner's. It is the replay gate's bound (`mech_gate.LEAD_MAX_DEG`, `insert_gate`, F-256's): J1-J5 at most 15 deg ahead of the report from 1 s on. Week 9's `keyhold/runner_ok.py` scored the hold against it as "the dog runner's bounds", and the name carried over.

The runner's own `ArmWatch` (`demos/combiner_key/robot/reach_arm.py`, built with its defaults by `press_live` and `insert_live`):
- stops past 25 deg held for 0.5 s (`MAX_LEAD_DEG`, `LEAD_HOLD_S`);
- stops past 60 deg at once (`MAX_LEAD_HARD_DEG`);
- has a 1 s grace after the first goal.

**Applied to S4b's dump** (48 episodes per arm, J1-J5 target against the link's next report, from 1 s): it stops 46 of 48 (A; 38 on the 60 deg hard stop) and 47 of 48 (B; 46 hard), at a median 3.0 s (A) and 3.1 s (B) into the episode. F-304's conclusion stands, that the runner stops these policies' holds. Its "every episode" is 46-47 of 48, and its 15 deg is the gate's. F-304 has a dated note.

**Docs:** `docs/tool_paper_plan.md`'s "Results so far" now says which 15 deg week 9's "dog runner's bounds" means. Its H1 line gains F-304 and S4c-0 (a 2 cm grasp gives 164 and 209 opened within the budget, against 642 and 618).

### 2026-11-19 · S4c-0b: the re-runs with the fields, and the parent `66600` under the gate

**Runs** (`logs/unifp_train/s1tools/spool_e/e_kz_s4c0b_diag.sh`, lane e, 13:32-13:41, 4 runs, all exit 0, S4b validation at 1 N·m, the dog's stance, law + bleed): [A gate2](#/week/10/run/20261119T023210_mech_eval_seed1_s4c0b_A_s91_68200_gate2_f1), [B gate2](#/week/10/run/20261119T023415_mech_eval_seed1_s4c0b_B_s91_68200_gate2_f1), [`66600` gate2](#/week/10/run/20261119T023627_mech_eval_seed1_s4c0b_66600_gate2_f1), [`66600` plain](#/week/10/run/20261119T023840_mech_eval_seed1_s4c0b_66600_plain_f1).

- **A and B reproduce the first runs' f1 counts**: 88 grasped and 69 opened within the budget, and 183 and 126.
- **The approach field was still wrong as first written.** The goal starts at the hand at the reset and runs to the handle, so `closest_to_goal_m` came out near 0 everywhere (median 1.1-1.4 cm for the ungrasped, 0.0 for the grasped). Fixed 13:38: `tool_goal_m` now counts only from the engage step on, when a grasp may close, with a test that it is NaN before and finite after. These four runs predate the fix, so the approach below is read from their traces from 4 s on, past every episode's engage step (at most 3.3 s).

| at 1 N·m, of 384 | A `68200` gate2 | B `68200` gate2 | `66600` gate2 | `66600` plain |
|---|---|---|---|---|
| grasped | 88 | 183 | 106 | 384 |
| opened (within the budget) | 69 (69) | 126 (126) | 18 (18) | 69 (69) |
| grasp error, median (q10-q90) | 1.9 cm | 1.9 cm | 1.9 cm | **3.8 cm** (3.0-5.0) |
| ungrasped: closest from 4 s, median (q10-q90) | 3.8 cm (2.4-6.3) | 3.2 cm (2.2-5.8) | 3.1 cm (2.4-4.3) | |
| J1-J5 lead from 1 s to the grasp, per-episode median (q10-q90) | 8.5 deg (6.0-13.7) | 10.6 deg (7.6-17.2) | **2.4 deg** (1.4-5.2) | 2.3 deg (1.4-3.3) |
| lead held, per-episode median (q10-q90) | 52 deg (18-128) | 56 deg (14-119) | 27 deg (6-47) | 33 deg (8-48) |

Also from the same traces, the ungrasped hands' median distance from the handle grows from 6.2-6.7 cm (2-4 s) to 10.0-10.1 cm (8-14 s) for A and B: they drift off rather than settle. *(Corrected 15:40: these, and the "closest from 4 s" row, are distances to a goal that leaves the handle 0.5-2 s after arriving, not from the handle; "drift" is not shown. See below.)*

**What it shows:**
- `66600` never had a 2 cm arrival on this task either: under the gate it grasps 106 of 384 (A 88, B 183). Ungated, it grasps 3.8 cm from the handle, against S4b's 6-7. S4b's training made the arrival worse.
- `66600`'s lead before the grasp is a median 2.4 deg. S4b's is 8.5-10.6, with no load on the arm. Once holding, even `66600` (never trained on mechanisms) leads 27-33 deg.
- Starting S4c from `66600`, as S4b did, keeps its small approach lead. The arrival has to be trained from either start.

### 2026-11-19 · S4c-1, stated before training

**What it is:** S4b arm A's recipe from `66600` (`s4/recipe.sh`: law v1 + bleed 1.0 in the loop, 2,000 iterations, 2048 envs, 3 of 4 starts from the dog's stance, S4's legs, link, arm step and wrist prices, the 15 deg tilt stop), plus three changes for the three needs in F-304:
1. **`--mech_grasp_within_m 0.02`**: the grasp closes only within 2 cm of the handle, in training as at evaluation.
2. **`--mech_approach_weight 1.0`**, new, `mech_env.Go2D1MechEnv._approach_reward`. It pays exp(−d / 4 cm) per second in mechanism episodes until the grasp, and its full 1 from the grasp on. The width and weight are the pose objective's fine position term's (`task_cfg.TRACKING_EE_POS_SIGMA_FINE`, `POSE_WEIGHTS`). It pays in full after the grasp, so that a term stopping there never pays a hover at 2-3 cm (about 0.5 per second) more than the grasp would earn.
3. **`--arm_lead_weight -30`**, the key hold's (F-255), J1-J6 past the 10 deg free band. On S4b's behaviour it would cost about 54 (A) per episode against tracking's 1.6. From `66600`, whose lead is 2.4 deg before the grasp, it costs nothing until the arm holds. The simulated D1 servo (kp 4000 N·m/rad, limits 1.7-3.3 N·m) saturates within a degree, so holding needs no lead past the free band.

Arm A only: F-294's and F-295's layer. S4b's test, still to run, decides A against B; if it picks B, a B run follows.

**Watched in training (stated now, read from the log, never from evaluation):** `Mechanism/grasped_fraction` is new, 13:42: of the mechanism episodes that ended at least 1 s past the engage step, the share that grasped. `66600` starts near 0.28 under the gate. If by iteration 66900 (300 in) it is under 0.2 and falling while `Episode_Reward/arm_lead` is the largest penalty, the price is teaching the policy not to grasp. The run then stops and restarts at `--arm_lead_weight -3`, logged.

**Evaluation:** checkpoints 68000, 68200, 68400 and 68599 on S4b validation at 1 and 2 N·m, from the dog's stance, law + bleed, with the 2 cm gate (`queue_s4c.sh`). The selection is S4b's rule (fewest falls plus tilt stops, then most opened within the budget, then the earliest).

**The dog-side checks on the selected checkpoint**, as F-304's:
- the shadow dump (48 episodes, 1 N·m) and `check_mech.py` parity;
- `ArmWatch` at its own thresholds (25 deg held 0.5 s, 60 at once);
- the grasp error, the base's motion while holding, and the leg odometry's error with the law read through it;
- the export and `mech_gate.py --modes hold` on `66600`'s four stands.

**Reading, stated now:** S4c-1 gives a T1 policy fit for the dog's runner in simulation if, at the selected checkpoint, all three hold:
- under the 2 cm gate it opens within the budget at least 482 of 768, 3/4 of S4b A `68200`'s ungated 642 (S4b A gated: 164);
- `ArmWatch` stops at most 5 of the 48 dumped episodes;
- the replay gate passes on every row.

The travel's source on the dog is not in this reading. The base's motion, the odometry's error and the law read through it are reported and stay the open dog-side choice (F-304). It is validation only; the S4b test sets are not touched.

*Corrected 14:15, after the run started and before any evaluation:* I first gave S4b's `mech_progress` as 0.03-0.08 per episode, which made the hover look better paid than holding. That range was the bottom of a sorted list of unique values. Over S4b A s91's 2,000 iterations its median is 1.50 (p10 0.39, p90 2.46, zero in 7%), so at S4b's rates a hover would not out-earn holding. Early in S4c-1 it would: its first 75 iterations pay a median 0.37 and nothing in 24%. The design is unchanged; only its reason is corrected.

**Queue:** `bash logs/unifp_train/s1tools/queue_s4c.sh 91` puts it on t2 ahead of S4b B seed 93: S4c is item 2's direct path, and S4b's test compares policies F-304 already found unfit. S4b B s93's job is renamed `t_z_s4b_06_s4b_B_s93.sh` to sort after it, unchanged otherwise, so it runs at about 17:35 and S4b's test moves back about 3.3 h.

**Started 14:04:55** on t2, after S4b B s92 (ended 14:04:54, exit 0). By iteration 66668 every new term is in the log:
- `Episode_Reward/mech_approach` 0.76;
- `Episode_Reward/arm_lead` −0.7 to −3.2 over the last five iterations, against `tracking_ee_force_world` 1.45;
- `Mechanism/grasped_fraction` 0.67-1.0;
- `Mechanism/grasp_error_m` 1.5-2.0 cm, inside the gate as it must be.

The training grasp rate is far above the stop rule's 0.2. It also sits above `66600`'s 0.28 at evaluation, where episodes are 14 s and the placements are S4b validation's; nothing is read from that.

**The stop rule at 66900 (14:35): not met; the run goes on.** Means over 50-iteration blocks from the training log:

| iterations | grasped | opened | grasp error | `arm_lead` | `mech_approach` | `mech_progress` | `tracking_ee_force_world` |
|---|---|---|---|---|---|---|---|
| 66600-66649 | 0.62 | 0.61 | 1.77 cm | −1.78 | 0.24 | 0.38 | 0.87 |
| 66700-66749 | 0.91 | 0.68 | 1.75 cm | −0.68 | 0.49 | 0.90 | 1.35 |
| 66850-66899 | 0.96 | 0.55 | 1.77 cm | −0.46 | 0.56 | 1.08 | 1.39 |

The lead price is still the largest penalty (`action_rate` −0.30 next), but it falls as the grasp rate rises: the policy is learning to hold with the lead down rather than not to grasp. These are training-time metrics sampled at reset, not the validation that decides.

### 2026-11-19 · Item 3: the dog's insertion on the chain sets, and the hold retrains started

**The insertion** `ki7_s62_9200` (item 1's decision, F-303), under the dog's conditions (strict keyway, leg-odometry base, the dog's stance, `66600`'s probe legs, the D1 link), with the body trace. `bash logs/unifp_train/s1tools/queue_chain_ins.sh ki7_s62_9200 logs/unifp_train/keyinsert/ki7_s62_9200.pt`, lane e, 12:02-12:26, 6 runs, all exit 0.

| set | f | started | seated inside the tilt stop | believed seated | false seats | damaged | falls | handovers | run |
|---|---|---|---|---|---|---|---|---|---|
| spawn | 1 | 96 | 64 | 94 | 30 | 2 | 0 | 64 | [f1](#/week/10/run/20261119T010301_leg_eval_seed1_chainins_ki7_s62_9200_spawn_f1) |
| spawn | 2 | 96 | 76 | 92 | 16 | 4 | 0 | 76 | [f2](#/week/10/run/20261119T010718_leg_eval_seed1_chainins_ki7_s62_9200_spawn_f2) |
| validation | 1 | 48 | 37 | 47 | 10 | 1 | 0 | 37 | [f1](#/week/10/run/20261119T011102_leg_eval_seed1_chainins_ki7_s62_9200_validation_f1) |
| validation | 2 | 48 | 38 | 46 | 8 | 2 | 0 | 38 | [f2](#/week/10/run/20261119T011435_leg_eval_seed1_chainins_ki7_s62_9200_validation_f2) |
| test | 1, 2 | 96, 96 | not read until the chain's test | | | | | | [f1](#/week/10/run/20261119T011813_leg_eval_seed1_chainins_ki7_s62_9200_test_f1) [f2](#/week/10/run/20261119T012236_leg_eval_seed1_chainins_ki7_s62_9200_test_f2) |

- The insertion is fixed, so its outcome on `test` cannot steer a choice. It is still read with the chain's test, once.
- **Training stances:** the 140 `spawn` handovers plus the recipe's four repeated to 140 (`merge_spawn.py`): `logs/unifp_train/chain/khc_ki7_s62_9200_stances.json`, 280 rows.
- The false seats (30 and 16 on spawn, 10 and 8 on validation) are F-292's. They are not handed over: `insert_handover.py` hands on only an insertion seated inside the tilt stop (its line 79), so the hold never meets them and they fail the chain for every hold alike. On validation that gives 37 and 38 handovers (`chain/ki7_s62_9200_validation_f<F>/`, made 14:01 by the first `khc` evaluation; the base at the handover a median −34 / 28 mm and −5 deg at 1 N·m, −28 / 23 mm and −5 deg at 2).

**`khc_s71` started 13:18 on t1** (`t_khc_01_khc_s71`, after S4b A s93). Its first iterations show the flags took effect (the silent-flags check):
- `Episode_Reward/key_hold`, `key_hold_coarse`, `arm_lead` (−0.10), `key_turn_ahead` (`--key_turn_follow_weight -15`'s term) and `support_stance` are all present;
- its `env.yaml` has `spawn_states: .../chain/khc_ki7_s62_9200_stances.json`, `spawn_states_fraction: 1.0`, `arm_lead_weight: -30.0`, `turn_follow_weight: -15.0` and `leg_friction_range` 0-2;
- its `run.json` arguments have the box: `key_hold_door_ahead` [0.495, 0.625], `key_hold_axis_side` [−0.145, −0.04], `key_hold_door_yaw` [−0.05, 0.19].

At about 5 iterations a minute, s71 ends near 16:00, s72 near 18:40 and s73 near 21:20. My earlier "t1 free about 17:00" was wrong.

### 2026-11-19 · Item 1b for `ki7_s62_9200`: how good the dog's lock estimate must be for the policy now on the dog

**Runs** (`bash logs/unifp_train/s1tools/queue_lockerr.sh ki7_s62_9200 logs/unifp_train/keyinsert/ki7_s62_9200.pt`, queued 12:00 on item 1's decision; lane e, 12:44-14:18, 20 runs, all exit 0): the stated shifts plus the roll ×0 and ×0.5 follow-up, the same conditions as `ki6`'s (strict keyway, leg-odometry base, the dog's stance, `66600`'s probe legs, the D1 link, the frozen 96 at 1 and 2 N·m). Scored by the same rule: `python3 logs/unifp_train/s1tools/lockerr_compare.py ki7_s62_9200`, out `s1tools/lockerr_ki7_s62_9200.json`.

Success inside the tilt stop of 192, each shift against `ki7`'s own baseline (the rule), beside `ki6`'s same shift. The last column pairs `ki7` against `ki6` on the same locks; it is reported, not in the rule.

| shift | rule (vs own baseline) | `ki7` success | Δ | only base / only shifted, p | damaged | false seats | peak p90 N | `ki6` success (damaged) | `ki7` vs `ki6`: only ki7 / only ki6, p | runs |
|---|---|---|---|---|---|---|---|---|---|---|
| baseline | | 147 | | | 6 | 39 | 47.4 | 120 (45) | 51 / 24, p 0.002 | [f1](#/week/10/run/20261119T014408_leg_eval_seed1_lockerr_ki7_s62_9200_base_f1) [f2](#/week/10/run/20261119T014731_leg_eval_seed1_lockerr_ki7_s62_9200_base_f2) |
| lateral ×1.5 (5.25 mm) | | 135 | −12 | 24 / 12, p 0.065 | 9 | 48 | 48.0 | 115 (45) | 43 / 23, p 0.019 | [f1](#/week/10/run/20261119T015057_leg_eval_seed1_lockerr_ki7_s62_9200_lat150_f1) [f2](#/week/10/run/20261119T015421_leg_eval_seed1_lockerr_ki7_s62_9200_lat150_f2) |
| lateral ×2 (7 mm) | **matters** | 125 | −22 | 37 / 15, p 0.003 | 8 | 57 | 48.4 | 114 (32) | 38 / 27, p 0.22 | [f1](#/week/10/run/20261119T015804_leg_eval_seed1_lockerr_ki7_s62_9200_lat200_f1) [f2](#/week/10/run/20261119T020258_leg_eval_seed1_lockerr_ki7_s62_9200_lat200_f2) |
| lateral ×3 (10.5 mm) | **matters** | 104 | −43 | 57 / 14, p 3e-7 | 19 | 63 | 50.0 | 92 (43) | 42 / 30, p 0.20 | [f1](#/week/10/run/20261119T020610_leg_eval_seed1_lockerr_ki7_s62_9200_lat300_f1) [f2](#/week/10/run/20261119T021109_leg_eval_seed1_lockerr_ki7_s62_9200_lat300_f2) |
| depth ×1.5 (9 mm) | | 149 | +2 | 13 / 15, p 0.85 | 6 | 37 | 46.8 | 124 (44) | 48 / 23, p 0.004 | [f1](#/week/10/run/20261119T021610_leg_eval_seed1_lockerr_ki7_s62_9200_dep150_f1) [f2](#/week/10/run/20261119T021923_leg_eval_seed1_lockerr_ki7_s62_9200_dep150_f2) |
| depth ×2 (12 mm) | | 142 | −5 | 20 / 15, p 0.50 | 8 | 41 | 47.5 | 103 (56) | 59 / 20, p 1e-5 | [f1](#/week/10/run/20261119T022354_leg_eval_seed1_lockerr_ki7_s62_9200_dep200_f1) [f2](#/week/10/run/20261119T022725_leg_eval_seed1_lockerr_ki7_s62_9200_dep200_f2) |
| roll ×1.5 (4.5 deg) | **matters** | 94 | −53 | 58 / 5, p 2e-12 | 10 | 88 | 47.4 | 63 (40) | 52 / 21, p 4e-4 | [f1](#/week/10/run/20261119T024056_leg_eval_seed1_lockerr_ki7_s62_9200_roll150_f1) [f2](#/week/10/run/20261119T024438_leg_eval_seed1_lockerr_ki7_s62_9200_roll150_f2) |
| roll ×2 (6 deg) | **matters** | 61 | −86 | 88 / 2, p 7e-24 | 2 | 129 | 46.7 | 28 (27) | 46 / 13, p 2e-5 | [f1](#/week/10/run/20261119T024817_leg_eval_seed1_lockerr_ki7_s62_9200_roll200_f1) [f2](#/week/10/run/20261119T025138_leg_eval_seed1_lockerr_ki7_s62_9200_roll200_f2) |
| roll ×0 (0 deg) | **matters** (a loss) | 122 | −25 | 49 / 24, p 0.005 | 1 | 69 | 32.2 | 118 (4) | 42 / 38, p 0.74 | [f1](#/week/10/run/20261119T025449_leg_eval_seed1_lockerr_ki7_s62_9200_roll000_f1) [f2](#/week/10/run/20261119T025815_leg_eval_seed1_lockerr_ki7_s62_9200_roll000_f2) |
| roll ×0.5 (1.5 deg) | **matters** (a gain) | 165 | +18 | 13 / 31, p 0.010 | 1 | 25 | 45.4 | 136 (20) | 43 / 14, p 2e-4 | [f1](#/week/10/run/20261119T031019_leg_eval_seed1_lockerr_ki7_s62_9200_roll050_f1) [f2](#/week/10/run/20261119T031337_leg_eval_seed1_lockerr_ki7_s62_9200_roll050_f2) |

**The accuracy needed, by the rule** (the grid's largest error at the largest scale past 1 that does not matter):
- lateral: **5.25 mm** (`ki6`: 7 mm). ×2 matters for `ki7`, −22.
- depth: **12 mm** (`ki6`: 9 mm). Neither depth shift matters.
- roll: **3 deg**, the grid's own (`ki6`: 3 deg).

**What it shows:**
- **`ki7` seats more than `ki6` at every shift** on the same locks, and damages fewer keys at every shift (1-19 against 4-56). Its tighter lateral number is relative to a higher baseline: at 7 mm off it still seats 125, above `ki6`'s 120 with the grid's own errors.
- **The roll still binds, and now it is not monotone.** Seats by roll error: 122 at 0 deg, 165 at 1.5, 147 at 3, 94 at 4.5, 61 at 6. Past 3 deg the strict keyway refuses the key and the layer takes the jam for a seat (false seats 39 → 88 → 129), as for `ki6`. With the roll exact, the false seats also rise (39 → 69) and the damaged keys fall to 1: the keys stop short and are believed home, F-292's jam. `ki6` showed the same shape (118 / 136 / 120) under the bar.
- **For the dog:** a tag good to 3 deg in roll, 5 mm across the keyway and 12 mm along it keeps `ki7`'s seats. A tag better than 3 deg in roll gains seats down to about 1.5 deg. A perfect roll does not, in this simulation.

**What it will not show:** why an exact roll seats fewer. Neither the policy (trained on errors up to ±4 deg) nor the simulated keyway (F-292: it barely constrains roll) was checked for it. It is one policy on the frozen 96, under a contact model with no measurement behind it. The tag's real accuracy is still the dog measurement this asks for: the tag's estimate against the box's measured keyway over several stand-ups.

**Docs:** `insert_live.py`'s usage now gives F-305's tolerances for `ki7_s62_9200` (its tests pass), and `docs/tool_paper_plan.md`'s line on the dog's insertion gives them beside F-302's.

**The runner with the dog's export (14:40, CPU, no simulator):** `tests/test_insert_live.py` ran its 11 tests on `ki6_force_8200`'s export only. They are now run on `checkpoints/insert_force_ki7_s62_9200.npz` by default, and on another export through `INSERT_LIVE_POLICY`: `python -m unittest tests.test_insert_live` gives 11 OK on each, none skipped. The tests run the dry run on the virtual arm, the whole live sequence against the fake robot and arm link, and the refusals (a lock outside training, no key, a press past the cap, a partial live run, an observing link). So `insert_live.py --policy checkpoints/insert_force_ki7_s62_9200.npz` loads, runs and sequences as `ki6`'s did. It is not run on the dog.

Finding: [F-305](../findings.md#f-305).

### 2026-11-19 · S4b arm B seed 92 trained, and `68400` selected on validation

**Training:** [arm B, seed 92](#/week/10/run/20261118T233746_train_seed92_s4b_B_s92) (law v2), 10:37-14:04, exit 0, as seed 91 (`spool_t2/t_s4b_04_s4b_B_s92.sh`).

**Validation**, as for the others (`python3 logs/unifp_train/s1tools/s4b_select.py s4b_B_s92`; lane e, 14:18-14:35). It ran after the evaluation's field fix, so it also carries the grasp error and the lead, reported and not in the selection:

| checkpoint | 1 N·m in budget / over / falls / tilt stops | 2 N·m | stops (1+2) | in budget (of 768) | grasp error, median | grasps within 2 cm | lead held, median |
|---|---|---|---|---|---|---|---|
| 68000 | [164](#/week/10/run/20261119T031816_mech_eval_seed1_s4b_B_s92_68000_val_f1) / 0 / 0 / 0 | [186](#/week/10/run/20261119T032019_mech_eval_seed1_s4b_B_s92_68000_val_f2) / 0 / 0 / 1 | 1 | 350 | 11.2 cm | 5 | 51 deg |
| 68200 | [277](#/week/10/run/20261119T032231_mech_eval_seed1_s4b_B_s92_68200_val_f1) / 0 / 0 / 0 | [280](#/week/10/run/20261119T032434_mech_eval_seed1_s4b_B_s92_68200_val_f2) / 0 / 0 / 4 | 4 | 557 | 10.0 cm | 0 | 65 deg |
| **68400** | [243](#/week/10/run/20261119T032648_mech_eval_seed1_s4b_B_s92_68400_val_f1) / 0 / 0 / 0 | [220](#/week/10/run/20261119T032851_mech_eval_seed1_s4b_B_s92_68400_val_f2) / 0 / 0 / 1 | 1 | **463** | 10.3 cm | 0 | 61 deg |
| 68599 | [266](#/week/10/run/20261119T033104_mech_eval_seed1_s4b_B_s92_68599_val_f1) / 0 / 0 / 1 | [234](#/week/10/run/20261119T033305_mech_eval_seed1_s4b_B_s92_68599_val_f2) / 0 / 0 / 3 | 4 | 500 | 6.6 cm | 21 | 92 deg |

- **Selected:** `68400` (1 stop, 463 in budget, none over), `s1tools/s4b_B_s92_selected.json`. The rule puts stops first, so it passes over `68200`'s 557 for 3 fewer tilt stops.
- Seed 92 is B's weaker seed so far: 463 against seed 91's 618.
- The new fields repeat F-304's picture on another seed: a grasp 7-11 cm from the handle, at most 21 of 768 within 2 cm, and the arm 51-92 deg ahead of its report once holding.
- Validation only; this chooses the checkpoint, nothing else. S4b's test waits for arm B seed 93 (t2 from about 17:35, after S4c-1) and arm A seed 93's validation (queued on lane e).

### 2026-11-19 · Item 3: the test's scripts, and the lead bound beside the runner's own (stated 14:39, after one checkpoint's validation)

**Ready, not queued:** they run once all three seeds are selected.
- `s1tools/queue_khc_test.sh <stem>` refuses to queue until `khc_s71..73_chain_selected.json` exist. For each selected checkpoint and `kh28_7000` it queues on lane e:
  - the hold from the chain-test handovers at 1 and 2 N·m (`hev.sh`, `<cand>_chaintest_f<F>`);
  - the own frozen 108 keys from the dog's stance at 1 and 2 N·m (`<cand>_ownkeys_dog_f<F>`), through `s1tools/lev_lock.sh`. That is `keyhold/lev.sh` with the weld lock, the legs and key flags identical. The flags are `kh28_7000_dog_f1`'s, from its `run.json`.
- `s1tools/khc_test.py` scores the test by the stated rule: chain_gate paired per seed and pooled against `kh28_7000`, and the own-key regression (p < 0.05 with fewer). Its own-key score reproduces week 9's 58 of 108 for `kh28_7000_dog_f1`.

**The lead bound.** `chain_gate`'s lead bound is week 9's `runner_ok`: J1-J5's instantaneous maximum from 1 s on, at most 15 deg. That is the replay gate's bound (F-256), not the runner's `ArmWatch`, which stops past 25 deg held 0.5 s or 60 at once (F-304's note).

`khc_s71_7200`'s validation, the only one read so far:
- it chains 30 of 37 and 30 of 38 handovers inside the tilt stop, and 0 and 3 inside the 15 deg bound;
- its maximum lead from 1 s is a median 17.7 deg (q10-q90 16-20) at 1 N·m, against `kh28_7000`'s 11.7 on its own keys.

So the stated selection and test, which put `chain_gate` first, stand unchanged. Beside them I report, from now on, the chain inside the runner's own lead stop. A maximum under 25 deg cannot trip it, so that count is the tilt-stop chain with the support legs inside their bounds, no fall, and the maximum lead from 1 s at most 25 deg. It is a reported column, not in any rule.

`chain_score.py` now prints it as `chain_runner25`. `khc_s71_7200` gives 60 of 96, its whole tilt-stop chain. Running the script on 7200 alone also wrote `khc_s71_chain_selected.json`, a selection from one candidate. I deleted it at once; nothing had read it. The script now writes a selection only from the four stated candidates.

### 2026-11-19 · S4b arm A seed 93 trained, and `68400` selected on validation

**Training:** [arm A, seed 93](#/week/10/run/20261118T230136_train_seed93_s4b_A_s93) (law v1 with the bleed), 10:01-13:18, exit 0, as seed 91 (`spool_t1/t_s4b_05_s4b_A_s93.sh`).

**Validation** (`python3 logs/unifp_train/s1tools/s4b_select.py s4b_A_s93`; lane e, 14:43-15:01). The grasp error and the lead are reported, not in the selection:

| checkpoint | 1 N·m in budget / over / falls / tilt stops | 2 N·m | stops (1+2) | in budget (of 768) | grasp error, median | grasps within 2 cm | lead held, median |
|---|---|---|---|---|---|---|---|
| 68000 | [253](#/week/10/run/20261119T034408_mech_eval_seed1_s4b_A_s93_68000_val_f1) / 0 / 0 / 0 | [209](#/week/10/run/20261119T034610_mech_eval_seed1_s4b_A_s93_68000_val_f2) / 0 / 0 / 0 | 0 | 462 | 6.3 cm | 5 | 39 deg |
| 68200 | [300](#/week/10/run/20261119T034824_mech_eval_seed1_s4b_A_s93_68200_val_f1) / 0 / 0 / 4 | [258](#/week/10/run/20261119T035027_mech_eval_seed1_s4b_A_s93_68200_val_f2) / 0 / 0 / 0 | 4 | 558 | 11.2 cm | 0 | 55 deg |
| **68400** | [339](#/week/10/run/20261119T035239_mech_eval_seed1_s4b_A_s93_68400_val_f1) / 1 / 0 / 0 | [333](#/week/10/run/20261119T035441_mech_eval_seed1_s4b_A_s93_68400_val_f2) / 0 / 0 / 0 | 0 | **672** | 9.2 cm | 5 | 79 deg |
| 68599 | [354](#/week/10/run/20261119T035654_mech_eval_seed1_s4b_A_s93_68599_val_f1) / 0 / 0 / 1 | [344](#/week/10/run/20261119T035856_mech_eval_seed1_s4b_A_s93_68599_val_f2) / 0 / 0 / 2 | 3 | 698 | 6.2 cm | 15 | 106 deg |

- **Selected:** `68400` (no stops, 672 in budget, 1 over), `s1tools/s4b_A_s93_selected.json`. Of the two with no stops it opens more; `68599` opens 26 more with 3 tilt stops.
- Seed 93 is arm A's best so far, 672 against 642 (s91) and 603 (s92). It is also the first arm A checkpoint over the budget once.
- Its grasps sit 6-11 cm from the handle (5-15 of 768 within 2 cm), and the arm leads its report by a median 39-106 deg once holding, as F-304 found for seed 91.
- Selected so far: A 91 `68200`, A 92 `68000`, A 93 `68400`, B 91 `68200`, B 92 `68400`. B 93 trains on t2 after S4c-1.

### 2026-11-19 · S4c-1 at 67000, a diagnostic, and a second seed next on t2

**Why:** t2 frees at about 17:20 when S4c-1 ends, before its validation exists. The choice then is a second S4c seed or S4b B seed 93.

**Run:** [s4c1diag_A_s91_67000_val_f1](#/week/10/run/20261119T040258_mech_eval_seed1_s4c1diag_A_s91_67000_val_f1) (`spool_e/e_kz_s4c1_diag_67000.sh`, 15:02-15:05, exit 0): `s4c_A_s91` at 67000, 400 iterations in and not a selection candidate. It ran on S4b validation at 1 N·m, from the dog's stance, law + bleed, the 2 cm gate, with the shadow dump of 48 episodes. The dog-side numbers come from `python3 logs/unifp_train/s1tools/mech_dog_check.py <run>`, new. It computes F-304's measures from a dump, and on S4b A s91's dump reproduces F-304's grasp error, `ArmWatch` stops (46 of 48) and base motion (85 mm, q90 284).

| at 1 N·m | S4c-1 `67000` | S4b A s91 `68200` gated (S4c-0b) | S4b A s91 `68200` ungated | `66600` gated |
|---|---|---|---|---|
| grasped, of 384 | **322** | 88 | 384 | 106 |
| opened within the budget | 92 | 69 | 328 | 18 |
| ungrasped: closest once a grasp may close, median (q10-q90) | 2.2 cm (2.0-2.9) | | | |
| dump, 48: grasp error median | 1.9 cm, all 40 within 2 cm | | 6.4 cm, 0 within 2 cm | |
| dump: lead held, per-episode median (q10-q90) | **5.0 deg** (1.4-20.7) | | 65 deg (22-130) | |
| dump: lead before the grasp, median | 1.7 deg | | 8.7 deg | |
| dump: `ArmWatch` stops (25 deg 0.5 s / 60) | **4 of 48**, median 6.1 s | | 46 of 48 | |
| dump: inside the gate's 15 deg from 1 s | 37 of 48 | | 0 of 48 | |
| dump: base moved, grasp to end, median (q90) | 73 mm (105) | | 85 mm (284) | |

Training curves against S4b A s91 at the same iterations (100-iteration means from the logs, 66600 → 67200):
- the curriculum's force ceiling: 21 → 50 N, against S4b's 21 → 55 N;
- `Mechanism/opened_fraction`: 0.53-0.60, against 0.58-0.69;
- `mech_progress`: 0.57-1.14, against 1.02-1.58.

**What it shows:**
- In 400 iterations S4c-1 learned to arrive (84% within 2 cm) and to hold with the arm near its report (median lead 5 deg; `ArmWatch` stops 4 of 48, against 46).
- It opens less so far, 92 of 384, while its curriculum runs a little behind S4b's. Opening may come with the remaining 1,600 iterations; that is for the validation of the four stated candidates, not this.
- The base still moves 7 cm under the hold.

**The dog's layer does not yet know the gate.** `check_mech.py` on this dump (export made in the scratchpad) disagrees with Isaac's grasp on 9,747 of 33,552 steps, where S4b's had none. `mech_layer.MechLayer`'s rule is the settle rule alone. A subagent is adding `grasp_within_m` to the layer, the export, `check_mech`, `mech_gate` and the dump's meta, to the env's decision step for step. S4c's dog-side checks wait for it.

**Decision, 15:08:** S4c seed 92, the same recipe, goes next on t2 (`queue_s4c.sh 92`), ahead of S4b B seed 93. Arrival and the lead are trained, so a second seed is what a finding on S4c needs whichever way its opening turns out. S4b B s93 follows at about 20:40, and S4b's test moves to about 00:30.

### 2026-11-19 · The gate measured the tool against UniFP's next goal: S4c-1 stopped, the env fixed, S4c-2 started

**The dog's layer gains the gate** (a subagent, 14:40-15:30; CPU only, nothing on the dog):
- `demos/combiner_key/robot/mech_layer.py`: `MechLayer.update(..., tool_pos=, tool_to_goal_m=)`. The radius comes from the export's `approach.grasp_within_m`, compared in float32 as the env does. With None it is bit for bit the old rule.
- `export_policy.py --task mechanism` reads the radius from the run's `env.yaml`, cross-checked against `run.json`.
- `check_mech.py` feeds the env's own distance. `mech_gate.py` gates the replay on the reported forward kinematics on its base.
- `unifp_train/mech_shadow.py` records the radius and the wait in the dump's meta (swapped by rename, the live file checked unchanged first).
- Ten new tests in `tests/test_mech_layer.py`.
- On S4c-1's 67000 dump with a re-export (`scratchpad/s4c/mech_s4c_A_s91_67000_v2.npz`, radius 0.02 in its meta): 0 rule-mismatched steps, against 9,747 before. Actions match to 2.3e-6 and the force command to 1.2e-5 N. S4b A's and B's dumps are unchanged, 0 mismatches.

**What it found: the gate was not at the handle.** `_engage_due` measures the tool against `_goal_world()`, UniFP's current goal. A mechanism episode's goal is aimed at the handle (`aim`), but UniFP's generator draws a new one 0.5-2 s after arriving (`EE_GOAL_HOLD_TIME_S`). Under the old rule the grasp always closed within about 1.3 s of the arrival, before that mattered. Under the gate a hand that has not arrived is measured against the next, random goal.

Checked on the 67000 dump myself: 20 of its 40 grasps closed after the goal had left the handle. At those 20 the tool was 2.0-57.8 cm from the handle placement, 15 of them 9 cm or more. The other 20 were within 2.0 cm. `grasp_error_m` and `tool_goal_m` measure against the same moved goal, so they hid it. In training the same happened, and `mech_approach` paid the distance to the moved goal too. The bug is mine: the stated intent was "within 2 cm of the handle".

**Fixed 15:34** (`mech_env.py`, a copy tested and swapped by rename): with `grasp_within_m` set, `_hold_handle_until_grasp` holds a mechanism episode's goal on its handle from the step its approach arrives until the grasp, through `HeldGoals`' hold. `_hold_goals_on_path` now moves only grasped goals onto the path; without the gate every held goal is a grasped one, so nothing changes there. Three tests in `tests/test_s4c.py` (16 in it now):
- with a radius never reached, the goal stays on the handle for 320 steps, past every approach and hold. This test fails on the unfixed file;
- gated grasps are measured against the handle itself;
- with the gate off, the hold mask equals the grasp on every step.

The S4c group gives 81 OK. The dog's layer already held the handle, so env and layer now agree.

**S4c-1 stopped 15:36 at 67605** ([run](#/week/10/run/20261119T030500_train_seed91_s4c_A_s91), re-recorded as stopped). Its training paid grasps and the approach term at random goals, so it is not the arrival that was stated. Its queued second seed and its evaluations were moved to `s1tools/cancelled/`.

**S4c-2, stated now, before it trains:** S4c-1's recipe, evaluation, selection, dog-side checks and reading, unchanged, on the fixed env (`s1tools/queue_s4c2.sh <seed>`). Seeds 91 and 92 run on t2: s91 started 15:36, s92 follows at about 18:55, and S4b B s93 after it at about 22:15. The evaluation's `--mech_grasp_within_m 0.02` now also holds the goal on the handle, so a grasp counts only at the handle. The stop rule from S4c-1 (`Mechanism/grasped_fraction` under 0.2 and falling by 300 iterations in) applies.

**Corrections to earlier entries today** (each was measured against the moving goal):
- *S4c-0's result:* the 223 and 329 gated grasps include grasps at UniFP's next goals. Grasps at the handle are at most these, so the reading (arrival has to be trained) stands, more strongly. "The hand drifts within 2 cm rather than reaching" for the late grasps is wrong: late grasps are where most moved-goal grasps fall.
- *S4c-0b:* the "closest from 4 s" row and "the ungrasped hands' median distance from the handle grows from 6.2-6.7 cm to 10.0-10.1 cm: they drift off" were distances to a goal that moves off the handle 0.5-2 s after arriving. They are not distances from the handle, and "drift" is not shown. `66600`'s 106 gated grasps are likewise at most its arrivals. The ungated grasp errors (`66600` 3.8 cm, S4b 6-7 cm) are unaffected: those grasps close within about 1.3 s of the arrival.
- *S4c-1 at 67000:* its 322 grasps of 384 and 92 openings include grasps at moved goals; in the dump that was 20 of 40. Its lead and `ArmWatch` numbers stand.
- F-304's S4c-0 note gains a dated correction.

**Also from the subagent, open:**
- The dog's own reading of the hand (reported forward kinematics on the odometry base) is about 2 cm off in places: in 8 dumped episodes the dog's gated rule would fire 15-523 steps before Isaac's, with the true tip 2.05-2.56 cm from the goal and the reading under 2 cm. So 2 cm is about the size of the dog's reading error.
- In `mech_gate.py`'s replay the hand stays 16-30 cm from the goal for this export, so no gated grasp closes there (0 of 48). That is being looked into before S4c-2's gate is read, because a replay that cannot bring the hand to the handle cannot pass any policy.

### 2026-11-19 · Why the replay's hand never reaches the handle: S4c reaches with its body

The subagent's second task (15:40-15:50, CPU; scripts in `scratchpad/replaygap/`, not in the repo): why `mech_gate.py`'s replayed hand stays 16-30 cm from the goal for the S4c 67000 export, when Isaac's comes within 2 cm.

**The replay is not wrong.** Its machinery was driven with Isaac's own inputs: the S4c 67000 export on one stand, closest approach from the engage step to Isaac's grasp, three episodes:

| inputs | ep 19 | ep 24 | ep 39 |
|---|---|---|---|
| Isaac's tip | 1.44 cm | 1.47 cm | 1.28 cm |
| the policy and layer on Isaac's arm reports | 1.27 | 1.48 | 1.18 |
| the replay's arm (Follower, 77 ms lag, 9 Hz reports) on Isaac's legs and base | 0.91 | 1.40 | 1.08 |
| the recorded stand's legs and base (what the gate runs) | 20.0 | 25.6 | 23.4 |

- The commanded goal before the grasp is the handle's placement exactly. The frame is the goal sphere of the reset base, as `check_mech` verifies against Isaac. No clamp is active (raw minus clamped arm targets 0-0.015 rad).
- With the replay's arm on Isaac's legs, the arm's joints at the grasp are within 0.03 rad of Isaac's.

**How S4c reaches in Isaac** (the 48 dumped episodes, from the reset to the closest approach):
- the base drops 4.7-9.8 cm (median 7.3), shifts 7-11 cm and rolls 2-9 deg (median 6.2);
- with Isaac's own arm joints but the base frozen at its reset pose, the hand would be 2.7-13.5 cm from the handle (median 6.4), none of 48 within 2 cm;
- S4b leans harder: drops of 6-13 cm, pitch 6-13 deg nose down.

The gate replays `66600`'s recorded stands open loop, so the legs answer nothing. On the dog they answer about 0.17 (F-256); in Isaac, with S4's 0-2 N·m friction, more than 0.28.

**Every gate placement can be reached by the arm alone:** position IK from each of the four stands' reset base reaches all 12 within the trained joint limits. Joint2's static load there is 0.62-0.85 of its limit. Collisions were not checked.

**What it means:** S4c's criterion (c), the replay gate passing on every row, tests a reach with the arm from the dog's stand with legs that answer 0. That is stricter than the dog (0.17), but the dog is close to it. A policy that reaches by sinking its body 7 cm will not reach on the dog. This is F-275's pattern for the pose policies again: in simulation every screen had legs that answer.

**Also changed:** `check_mech.py` now ignores the resampling under a held goal (`goal_hold_mask`) when it counts "goal moved on", so S4c-2's dumps are not misread. S4c-1's dump reports identically. `tests.test_mech_layer` + `test_s4c`: 46 OK against the fixed env.

**Next, stated 15:50:** a leg probe on S4c-1's last checkpoint, `67600` (stopped run, fixed gate at the handle, S4b validation at 1 N·m), twice: the legs as trained, and legs answering 0.17 (`--leg_answer_range 0.17 0.17`, F-275's screen). The rule, stated now:
- If at 0.17 it grasps fewer than half as many as with the legs as trained, the arrival rests on the legs. S4c-2 is then restarted with the legs' answer drawn low in training (`--leg_answer_range 0.1 1`, as `6c` and `pose_s`).
- Otherwise S4c-2 goes on as it is, and its evaluation adds the 0.17 screen.

### 2026-11-19 · Item 3: `khc_s71` trained, and `7600` selected on the chain's validation

**Training:** [`khc_s71`](#/week/10/run/20261119T021825_train_seed71_khc_s71), 13:18-15:42, exit 0, 800 iterations from `kh28_7000` as stated (`spool_t1/t_khc_01_khc_s71.sh`). `khc_s72` started on t1 at 15:42.

**Validation:** the hold from `ki7_s62_9200`'s chain-validation handovers (37 at 1 N·m and 38 at 2, of 48 insertions started each), `python3 logs/unifp_train/s1tools/chain_score.py khc_s71 7200 7400 7600 7799 --set validation`, out `s1tools/khc_s71_chain_selected.json`. Lane e, 14:01-15:52.

| checkpoint | chain_gate (both) | chain inside the tilt stop: f1 / f2 (of 37 / 38 handed over) | runner's own 25 deg (reported) | falls + lost | J1-J5 max lead from 1 s, median f1 / f2 | runs |
|---|---|---|---|---|---|---|
| 7200 | 3 | 60: 30 / 30 | 60 | 0 | 17.8 / 17.6 deg | [f1](#/week/10/run/20261119T030147_leg_eval_seed1_khc_s71_7200_chainvalidation_f1) [f2](#/week/10/run/20261119T030557_leg_eval_seed1_khc_s71_7200_chainvalidation_f2) |
| 7400 | 20 | 64: 31 / 33 | 64 | 0 | 18.1 / 15.7 | [f1](#/week/10/run/20261119T033519_leg_eval_seed1_khc_s71_7400_chainvalidation_f1) [f2](#/week/10/run/20261119T033940_leg_eval_seed1_khc_s71_7400_chainvalidation_f2) |
| **7600** | **44** | **75: 37 / 38** | 75 | 0 | 15.1 / 14.2 | [f1](#/week/10/run/20261119T041031_leg_eval_seed1_khc_s71_7600_chainvalidation_f1) [f2](#/week/10/run/20261119T041448_leg_eval_seed1_khc_s71_7600_chainvalidation_f2) |
| 7799 | 43 | 67: 35 / 32 | 67 | 0 | 14.6 / 13.4 | [f1](#/week/10/run/20261119T044323_leg_eval_seed1_khc_s71_7799_chainvalidation_f1) [f2](#/week/10/run/20261119T044749_leg_eval_seed1_khc_s71_7799_chainvalidation_f2) |

- **Selected:** `7600` by the stated rule: the most `chain_gate`, 44 against 43.
- At `7600` every handover's hold does all four inside the tilt stop. So the chain inside the tilt stop is the insertion's own 75 of 96 seats, against F-297's 62 of 192. The insertion now limits it.
- The lead bound is what still costs. The median maximum lead falls through training from 17.7 to 14-15 deg, and `chain_gate` rises from 3 to 44. Every tilt-stop chain is inside the runner's own lead stop (the reported column).
- Validation only. The test waits for seeds 72 and 73 (t1 until about 21:20) and is run once by `queue_khc_test.sh ki7_s62_9200`.

### 2026-11-19 · The leg probe: S4c's arrival rests on the legs; S4c-2 stopped, S4c-3 trains on legs that answer 0.1-1

**Runs** (`spool_e/e_kz_s4c1_legs_67600.sh`, 15:52-15:56, both exit 0): the stopped S4c-1 at `67600`, the fixed gate (the goal held on the handle), S4b validation at 1 N·m, the dog's stance, law + bleed.

| of 384 | [legs as trained](#/week/10/run/20261119T045212_mech_eval_seed1_s4cleg_A_s91_67600_legs_f1) | [legs answering 0.17](#/week/10/run/20261119T045417_mech_eval_seed1_s4cleg_A_s91_67600_ans017_f1) |
|---|---|---|
| grasped within 2 cm of the handle | 126 | **4** |
| ungrasped: closest once a grasp may close, median (q10-q90) | 2.4 cm (2.1-2.8) | 12.2 cm (6.4-18.2) |
| opened within the budget | 31 | 0 |
| J1-J5 max lead from 1 s, median | 4.1 deg | 11.0 deg |
| falls, tilt stops | 0, 0 | 0, 0 |

**By the rule stated at 15:50:** at 0.17 it grasps fewer than half as many (4 against 126), so the arrival rests on the legs. S4c-2 is restarted with the legs' answer drawn low in training.

**What it shows:**
- With the goal held on the handle, S4c-1's true arrival is 126 of 384 on the legs it trained on. The 322 at `67000` counted grasps at UniFP's next goals.
- On legs that answer as the dog's do, the hand stops a median 12 cm short. This agrees with the replay (16-30 cm, the legs answering nothing) and with how S4c reaches in Isaac, sinking the body 7 cm.
- It is the same lesson as F-256 and F-275, now for the mechanism task.

**S4c-2 stopped 16:03 at about 66920** ([run](#/week/10/run/20261119T043635_train_seed91_s4c2_A_s91), re-recorded as stopped). Its training at the stop: grasped fraction 1.00, `mech_progress` 1.41, lead −0.47 (66850-66899). Its queued second seed and evaluations are in `s1tools/cancelled/`.

**S4c-3, stated now, before it trains** (`s1tools/queue_s4c3.sh <seed>`): S4c-2 plus `--leg_answer_range 0.1 1` in training, as `6c` and `pose_s` did. Seeds 91 (t2 from 16:03, to about 19:25) and 92 (to about 22:45); S4b B s93 after them.
- **Evaluation:** each of 68000, 68200, 68400 and 68599 on S4b validation with the 2 cm gate at the handle:
  - at 1 and 2 N·m on legs answering 0.17 (`--leg_answer_range 0.17 0.17`, F-275's screen). This is the selection's;
  - at 1 N·m on the legs as trained, reported.
- **Selection:** S4b's rule on the 0.17 runs: fewest falls plus tilt stops, then most opened within the budget, then the earliest.
- **Dog-side checks on the selected checkpoint:** the dump on the 0.17 screen, `mech_dog_check.py` and `check_mech.py` (both with the gate now), the export and `mech_gate.py --modes hold`.
- **Reading, as S4c-1's, on the 0.17 screen:** fit for the dog's runner in simulation if all three hold:
  - it opens within the budget at least 482 of 768;
  - `ArmWatch` stops at most 5 of 48 dumped;
  - the replay gate passes on every row.
- **Reported beside it:** the opened count by level, because a 3.3 N·m arm on legs answering 0.17 may not have the force for the heavy levels at all. A failure there is the dog's legs, and the reading says so rather than lowering the bar.
- **Stop rule:** as before. If by 66900 `Mechanism/grasped_fraction` is under 0.2 and falling while `arm_lead` is the largest penalty, the run restarts at `--arm_lead_weight -3`.

**Started 16:03:19**: its `env.yaml` has `leg_answer_range` (0.1, 1.0), `grasp_within_m` 0.02, `approach_weight` 1.0 and `arm_lead_weight` −30.

**The stop rule at 66900 (16:28): not met.** 50-iteration means from the training log, beside the stopped S4c-2's at the same iterations:

| iterations | grasped | opened | `arm_lead` | `mech_progress` | force ceiling | S4c-2: grasped / progress / ceiling |
|---|---|---|---|---|---|---|
| 66600-66649 | 0.23 | 0.67 | −0.61 | 0.09 | 16.7 N | 0.37 / 0.22 / 18.2 N |
| 66700-66749 | 0.91 | 0.60 | −0.83 | 1.09 | 25.0 N | 0.86 / 0.83 / 25.0 N |
| 66850-66899 | 0.97 | 0.57 | −0.38 | 1.28 | 25.0 N | 1.00 / 1.41 / 36.1 N |

On legs drawn 0.1-1 it arrives as often in training. Its curriculum has held at 25 N since 66700, where S4c-2's had climbed to 36 N. The frontier opens less often on weak legs. Training-time metrics sampled at reset, not the validation that decides.

### 2026-11-19 · Item 1: the dog's insertion on legs that answer 0.17, stated before it runs

**Why:** the leg probe showed S4c reaching by sinking its body, which legs answering 0.17 (the dog's, F-256) do not allow. The insertion was never screened that way. Its replay gate (S5, F-303) checks the lead, the leg targets, the IK and touches short of the face, not whether keys seat. In simulation its base moves 18-43 mm (F-296).

**Run** (`spool_e/e_kz_ins017_ki7_s62_9200.sh`, queued 16:31): `ki7_s62_9200` on the frozen 96 at 1 and 2 N·m under the conditions of `lockerr_ki7_s62_9200_base` (strict keyway, leg-odometry base, the dog's stance, `66600`'s probe legs, the D1 link), plus `--leg_answer_range 0.17 0.17`. `keyinsert_env` applies it (`_answer_legs`).

**Baseline:** `lockerr_ki7_s62_9200_base_f1`/`f2` (12:44-12:52 today, the same code path; 147 of 192 inside the tilt stop, 6 damaged, 39 false seats). The two runs differ in the legs' answer alone.

**Rule (F-302's):** the weak legs matter if, pooled over both frictions (192 pairs, the same lock), an exact McNemar gives p < 0.05 and success inside the tilt stop moves by at least 10% of the baseline's, 15 keys. Damaged keys, false seats, falls, the feet's slide and the odometry's error are reported beside it.

**If they matter:** the insertion, like S4c, rests on body motion the dog's legs do not give, and F-303's choice is not dog-ready as it stands. The next step would be a retrain with the legs' answer drawn low.

**Result, 16:29-16:34** (both exit 0: [f1](#/week/10/run/20261119T052913_leg_eval_seed1_ins017_ki7_s62_9200_f1), [f2](#/week/10/run/20261119T053205_leg_eval_seed1_ins017_ki7_s62_9200_f2)). The f in the run names is the legs' joint friction (`--leg_friction_range`), 1 or 2 N·m, as in every evaluation here:

| | baseline (legs as trained) | legs answering 0.17 | only baseline / only 0.17, p |
|---|---|---|---|
| success inside the tilt stop, f1 (of 96) | 68 | 76 | 15 / 23, p 0.26 |
| f2 | 79 | **48** | 40 / 9, p 9e-6 |
| **pooled (of 192)** | **147** | **124** (−23) | 55 / 32, **p 0.018** |
| false seats f1 / f2 | 25 / 14 | 16 / 46 | |
| damaged f1 / f2 | 3 / 3 | 4 / 2 | |
| falls, lost, tilt stops | 0 | 0 | |
| peak force p90 | 47.4 N | 45.5 N | |

**By the stated rule the weak legs matter:** p 0.018 and −23, more than 15. Every miss is a false seat or a damaged key, as with the legs as trained.

**What it shows:**
- On legs as weak as the dog's and as sticky as the stiffer half of training, the insertion seats 48 of 96 against 79, and the layer takes 46 jams for seats against 14.
- On the less sticky legs it seats as many as before or more (76 against 68).
- So part of `ki7_s62_9200`'s seating rests on its legs moving, and how much depends on the legs' friction, which on the dog is not measured.
- F-303's choice stands against `ki6` on the legs it was compared on. On the dog's legs it is not known to beat `ki6`, which was not run on this screen.

**What it will not show:** the dog. 0.17 is F-256's measurement of how much of a leg target the dog's legs carry out, on rug and wood, standing. The two frictions bracket an unknown.

**Next (stated before training):** an insertion retrained with the legs' answer drawn low, `ki8` (below). The `khc` hold's test and the chain keep the insertion as it is (the stated item 3), and this is noted beside them.

Finding: [F-306](../findings.md#f-306).

### 2026-11-19 · `ki8`, the insertion on legs that answer 0.1-1, stated before training

**What it is** (`s1tools/queue_ki8.sh <seed> <lane>`): `ki7`'s training command, resumed from the dog's pick `ki7_s62_9200` for 1,000 iterations, with `--leg_answer_range 0.1 1` added. Otherwise the same flags: diffed against `ki7_s62`'s job, they differ in seed, iterations, resume and the leg answer only.

**Seeds and lanes:** seed 64 on t2 after S4c-3 seed 91 (about 19:25-21:45), seed 65 on t1 after `khc_s73` (about 21:05-23:25). On t2, S4c-3 seed 92 then follows `ki8`, by name order. By then S4c-3 seed 91's validation will say whether a second seed of it is worth the lane.

**Validation per candidate** (9600, 9800, 10000, 10199) on the `ki7` validation locks at 1 and 2 N·m of leg friction, strict keyway, leg-odometry base:
- on legs answering 0.17, `ki8017_s<seed>_<it>_val_f<F>`. This is the selection, by `ki7_select.py`'s rule unchanged: fewest falls plus damaged keys, then most successes inside the tilt stop, then the earliest;
- on the legs as trained, `ki8_s<seed>_<it>_val_f<F>`, reported.

**The decision, stated now.** The seed whose selected checkpoint has more successes on the 0.17 validation (ties: fewer damaged) is compared with `ki7_s62_9200` on the frozen 96 under the dog's conditions at both frictions. It replaces `ki7_s62_9200` for the dog only if all three hold:
1. on legs answering 0.17 it seats more than F-306's 124, by a paired exact McNemar p < 0.05 pooled over 192;
2. on the legs as trained it does not seat fewer than the baseline's 147 at p < 0.05;
3. it damages no more keys than `ki7_s62_9200` on either.

Otherwise `ki7_s62_9200` stays the dog's.

**Docs:** `docs/tool_paper_plan.md`'s line on the dog's insertion gains F-306 and `ki8`.

### 2026-11-20 · The dog's insertion in the Isaac Sim viewer, for the weekly update's screenshots (11:09)

Lukas asked for a viewer of the policy so he can take screenshots for the weekly update; he chose `ki7_s62_9200`, the insertion now on the dog (F-303). The GPU was free (the queue's last job ended 00:13).
- **Command:** `bash logs/unifp_train/keyinsert/view.sh view_ki7_s62_9200_row73 "<title>" logs/unifp_train/keyinsert/ki7_s62_9200.pt 73 --leg_variant force --insert_strict_keyway --insert_base_source legs --leg_friction_range 1 1`.
  - That is `play` with `iev2.sh`'s legs, link and the dog's arm-out stance, one robot, `--steps 60000 --real_time`, under the dog's conditions: the strict keyway and the leg-odometry base, at 1 N·m.
  - The overlay is `keyinsert_vis`'s, as in week 9's shots 7-9. The key is grey before the touch, orange searching, blue seating and holding, green seated, red lost or damaged. Yellow is the estimated keyway centre, magenta the layer's tip reference, and the red bar is the lock's force on the key.
- **Lock 73**, of the 96 frozen locks: the true keyway is 3.5 mm off the estimate at 45 deg, 6 mm nearer than estimated, roll +3 deg.
  - In [the frozen-96 run on the dog's conditions](#/week/10/run/20261119T014408_leg_eval_seed1_lockerr_ki7_s62_9200_base_f1) it was a success: touch at 3.5 s, seated at 8.4 s after a search, peak 19.6 N, tilt at most 7.5 deg.
  - Week 9's lock 84 was not used: `ki7_s62_9200` false-seats it in that run (believed seated at 48.9 N peak).
- **What it shows:** the same frozen lock and policy as that evaluation, replayed for the eye with one environment, so the episode need not match the evaluation's step for step. It is not new evidence.
- **Closed at 11:30:18**, after about 21 min, with exit 137 (stopped by pid, so its `run.json` still reads `running` with no step count, as week 9's did). Recorded as [the viewer run](#/week/10/run/20261120T000902_play_seed1_view_ki7_s62_9200_row73).
  - Lukas took two screenshots from it, at 11:28:35 and 11:28:58: shots 1-2 in `screenshots/` (the entry below).

### 2026-11-20 · The chain's hold in the Isaac Sim viewer, the second policy for the screenshots (11:35)

Lukas asked for the next policy to screenshot. It is item 3's hold, `khc_s71_7600` (selected on the chain's validation, the 2026-11-19 entry above): the arm keeps the key in the lock while the front-right foot turns it, started from where a `ki7_s62_9200` insertion ended.
- **Command:** `bash logs/unifp_train/chain/view_khc_s71_7600_val_f1_row6/view.sh`.
  - That is `play` with `hev.sh`'s legs, key flags and link (`--arm_locked --arm_locked_range --arm_follower --hold_arm policy`), `--leg_variant force --leg_friction_range 1 1`, one robot, `--steps 60000 --real_time`.
  - It uses one-row copies of the chain's validation handover files in that folder (`--key_eval_rows`, `--spawn_states ... --eval_spawn_states --spawn_states_ordered`, `--leg_rows 0`). The copies are row 6 of `logs/unifp_train/chain/ki7_s62_9200_validation_f1/handover_{rows,spawn}.json`, so environment 0 starts from that row's stance, not row 0's.
- **The episode:** handover row 6 is `ki7_s62_9200`'s validation insertion 8 at 1 N·m. The door is 0.531 m ahead and turned 0.065 rad.
  - In [the hold's validation run](#/week/10/run/20261119T041031_leg_eval_seed1_khc_s71_7600_chainvalidation_f1) it did all four: unlocked at 1.74 s, the key turned to 45 deg, the foot back. Tilt was at most 10.7 deg, J1-J5's lead at most 10.6 deg from 1 s on (inside the 15 deg bound) and the peak 19.5 N.
  - It is one of the 18 chain-gate successes at 1 N·m. It was chosen for the lowest lead among them.
- **The overlay** is `leg_vis`'s: the door's face; the lock adapter, a dark block round the shaft; and the key's bar and platform, grey at the start, orange turning, green once unlocked. UniFP's foot goal, force target and force arrows are drawn too.
- **What it shows:** the frozen validation episode replayed for the eye with one environment, so it need not match the evaluation step for step. It is not new evidence, and the chain's test (`queue_khc_test.sh`) has not run.
- **Closed at 11:38:44**, after about 6 min, with exit 137 (stopped by pid at Lukas's word). Recorded as [the viewer run](#/week/10/run/20261120T003135_play_seed1_view_khc_s71_7600_val_f1_row6). Lukas's shots 3-6 are from it.

### 2026-11-20 · Lukas's screenshots from the two viewers, for the weekly update (added 11:45)

Lukas asked for his six screenshots to go in the notes. They are copied from `~/Pictures/Screenshots` into `screenshots/`, renamed in time order; the original times are below.
- **Shots 1-2** are from the insertion viewer: `ki7_s62_9200`, frozen lock 73, under the dog's conditions at 1 N·m. The camera looks from the robot's right, behind the hand.
- **Shots 3-6** are from the chain's hold viewer: `khc_s71_7600` from `ki7_s62_9200`'s chain-validation handover 6, at 1 N·m. The camera is wider, from the robot's right side.
- Both viewers loop their episode, so two shots need not be from the same pass.
- The overlay colours render lighter under the viewer's lights: the orange (1.0, 0.5, 0.05) of searching and turning looks golden yellow, and the green (0.1, 0.75, 0.25) of seated and unlocked looks mint.
- The insertion's millimetre-sized markers (the estimate's yellow dot, the reference's magenta dot, the force bar) are not visible at these camera distances.
- The door, the lock and the key are drawn markers, not simulated bodies: the key's plant is `keylock`'s torch model.

| shot | taken | what it shows |
|---|---|---|
| [`01_keyinsert_ki7_s62_9200_lock73_searching.png`](screenshots/01_keyinsert_ki7_s62_9200_lock73_searching.png) | 11:28:35 | The key at the lock's face in the jaws, its bar and platform orange: the layer is searching for the keyway after the touch. The D1 is stretched forward over the front legs, and the dog leans in toward the door from the arm-out stance. |
| [`02_keyinsert_ki7_s62_9200_lock73_seated.png`](screenshots/02_keyinsert_ki7_s62_9200_lock73_seated.png) | 11:28:58 | The same pose closer to the door, the key green: seated by the plant's test. The robot is in about the same stance as shot 1, all four feet down. |
| [`03_khc_s71_7600_handover6_key_held_at_rest.png`](screenshots/03_khc_s71_7600_handover6_key_held_at_rest.png) | 11:34:52 | The hold's start from the insertion's end: the jaws on the key in the lock, the dark lock adapter round the shaft, and the key's bar grey (not yet turned). All four feet are on the ground. |
| [`04_khc_s71_7600_handover6_foot_turning_key.png`](screenshots/04_khc_s71_7600_handover6_foot_turning_key.png) | 11:35:07 | The front-right foot lifted up to the key's bar and pressing it, the bar orange: turning, short of the latch. The arm still holds the key in the lock and the dog stands on three legs. |
| [`05_khc_s71_7600_handover6_unlocked.png`](screenshots/05_khc_s71_7600_handover6_unlocked.png) | 11:35:14 | The bar turned further under the foot and green: the door has unlocked. The arm is as in shot 4. |
| [`06_khc_s71_7600_handover6_later_pass_start.png`](screenshots/06_khc_s71_7600_handover6_later_pass_start.png) | 11:38:01 | The key grey again and all four feet down, with the arm holding the key: the start of a later pass of the looping episode, not the end of shot 5's. |

### 2026-11-20 · `ki6_force_8200` on the same lock in the viewer, the contrast for F-303 (11:48)

Lukas asked whether other policies should be screenshotted. The contrast for F-303 is ready without new code: `ki6_force_8200`, the insertion `ki7_s62_9200` replaced, on the same lock 73 under the same conditions.
- **Command:** `bash logs/unifp_train/keyinsert/view.sh view_ki6_force_8200_row73_strict_legs "<title>" logs/unifp_train/keyinsert/ki6_force_8200.pt 73 --leg_variant force --insert_strict_keyway --insert_base_source legs --leg_friction_range 1 1`. The flags are the first viewer's apart from the checkpoint.
- **Lock 73 in the evaluations:**
  - In [`ki6`'s frozen-96 run on the dog's conditions](#/week/10/run/20261118T203630_leg_eval_seed1_lockerr_ki6_force_8200_base_f1) it damaged the key at a 58.6 N peak.
  - `ki7_s62_9200` seats the same lock at 19.6 N (shots 1-2).
  - The two runs' arguments are identical apart from the run name and the checkpoint.
  - Over that run's 96 locks at 1 N·m, `ki6` damaged 23 keys and `ki7` 3. On 17 locks `ki6` damaged the key and `ki7` seated it, lock 73 among them.
- **What it shows:** a replay for the eye with one environment, as the other viewers. The key turns red once damaged. It is not new evidence.
- **Closed at 11:57:55**, after about 17 min, with exit 137 (stopped by pid). Recorded as [the viewer run](#/week/10/run/20261120T004023_play_seed1_view_ki6_force_8200_row73_strict_legs).
  - **It did not show the contrast.** In both of Lukas's shots from it (shots 7 and 10 below) the key is green, seated by the plant's test, where the evaluation damaged this key. The entry below checks why.

### 2026-11-20 · The wrist bracket off its screws in the viewer: F-137's 6.8 mm forearm, in the meshes only (12:05)

Lukas sent two close-ups from the viewers: the D1's wrist bracket is not aligned with its mounting screws. He asked whether it affects the work.
- **What it is:** Joint5. The forearm mesh (`d1_arm/meshes/Link4.STL`, Unitree's older `d1_description`) has its servo horn centred 140.0 mm along `Link4`'s z, the old Joint5 origin (0.14018). The circular features' vertex count peaks there, 4,009 at r 2.5 mm, against at most 2,029 elsewhere between 135 and 151 mm. Since F-139 (2026-10-09), Joint5 is at 0.146973, the D1-550 CAD's, and the `Link5` bracket turns about it. So the bracket's hole circle sits 6.8 mm along the forearm from the horn: F-137's "forearm −6.79".
- **What uses the meshes:** the visual and the collision geometry (the URDF gives the same STL for both, at zero origin).
- **What doesn't:**
  - The kinematics: the joint origins, the controlled point, the simulation's IK and `d1_ik.py` on the real arm, all on the CAD's frames. `description/go2_d1.urdf` has the same Joint5.
  - The key, the lock, the door and the mechanisms: analytic plants on the controlled point, drawn as markers. No arm mesh touches them.
- **Where it could matter:** the forearm's collision shape ends 6.8 mm short of the wrist axis. Self-collision is on (`unifp_isaaclab/robot.py`), and adjacent links (Link4-Link5) are filtered. So the residue is a few millimetres of clearance where the forearm's wrist end meets the Go2's body or the ground. Not tested.
- **The open question is the real arm, not the mesh:** F-137 asked for the forearm to be measured between the Joint3 and Joint5 axes (CAD 204.7 mm, old model 197.9) before changing the model. F-139 patched to the CAD without it at Lukas's word, and no caliper reading has been recorded since. If the real forearm is the old length, the controlled point is about 6 mm off on the real arm (F-139: median 6.0, max 8.8). That is the scale of the insertion's lock tolerance (F-305: 5.25 mm across the keyway).
- Nothing changed: `d1.urdf` is untouched, since editing it changes its hash and the generated USD every run records. F-139 has a dated note.

### 2026-11-20 · One environment does not repeat a lock's evaluated episode: lock 73 alone (12:00)

The `ki6` viewer showed lock 73's key seated, where `ki6`'s frozen-96 run damaged it. To tell the viewer apart from the environment count, both policies ran headless on lock 73 alone (`leg_eval --leg_rows 73`, one environment), with the viewers' and the frozen-96 runs' flags otherwise.
- **Commands** (11:58, in parallel, weld lock): `bash logs/unifp_train/s1tools/iev2.sh viewchk_<stem>_row73_strict_legs_f1 "<title>" logs/unifp_train/keyinsert/<stem>.pt --leg_variant force --insert_strict_keyway --insert_base_source legs --leg_friction_range 1 1 --leg_rows 73`, for `ki6_force_8200` and `ki7_s62_9200`.
- **The row is the same lock:** 3.5 mm off at 45 deg, 6 mm nearer, roll +3 deg, as in the 96-lock runs.

| lock 73 | 96 locks at once | alone (one environment) | the viewer (one environment, a later pass) |
|---|---|---|---|
| `ki6_force_8200` | [damaged](#/week/10/run/20261118T203630_leg_eval_seed1_lockerr_ki6_force_8200_base_f1): touch 3.28 s, ended 4.06 s, 58.6 N | [false seat](#/week/10/run/20261120T005914_leg_eval_seed1_viewchk_ki6_force_8200_row73_strict_legs_f1): touch 3.42 s, believed seated, not seated; 33.2 N, no damage | seated (green) in shots 7 and 10 |
| `ki7_s62_9200` | [seated](#/week/10/run/20261119T014408_leg_eval_seed1_lockerr_ki7_s62_9200_base_f1) 8.42 s, 19.6 N | [seated](#/week/10/run/20261120T005850_leg_eval_seed1_viewchk_ki7_s62_9200_row73_strict_legs_f1) 4.66 s, 29.8 N | searching, then seated (shots 1-2) |

- **What it shows:** on the same lock, with the same policy and flags, the episode's outcome changes with the number of environments: `ki6`'s goes from damaged to a false seat to seated. `ki7`'s seats each time but at different times and forces. The contact makes one episode one draw, not a property of the lock. Week 9 saw the frozen 96's totals move about ±3 under such changes.
- **What it does not show:** that the 96-lock comparisons are wrong. F-303's pairs are the same lock in two runs with the same environment count and layout, so the run-to-run draw is in both arms alike. Only the claim "this policy damages this lock" is not repeatable on its own.
- **For the update:** the viewers illustrate how a policy behaves, not a scored episode. Shots 7 and 10 show `ki6` seating lock 73, so they cannot stand for F-303's damaged keys. That contrast is the 96-lock numbers: on the dog's conditions at 1 N·m, `ki6` damaged 23 keys and `ki7` 3.
- F-303 has a dated note.

### 2026-11-20 · Lukas's screenshots, shots 7-10: the `ki6` viewer and the wrist close-ups (added 12:10)

Copied from `~/Pictures/Screenshots` as before, in time order after shots 1-6. All four were taken while the `ki6` viewer (11:40-11:57) was open. Shots 8 and 9 are 480 × 300 crops Lukas made from it.

| shot | taken | what it shows |
|---|---|---|
| [`07_keyinsert_ki6_force_8200_lock73_seated_in_viewer.png`](screenshots/07_keyinsert_ki6_force_8200_lock73_seated_in_viewer.png) | 11:44:20 | `ki6_force_8200` on lock 73, the camera close behind the hand: the key green, seated by the plant's test, held in the honeycomb jaws against the door, with the wrist bracket and forearm at left. Not what the 96-lock evaluation scored for this lock (damaged): see the entry above. |
| [`08_d1_wrist_joint5_mesh_offset_closeup_dark.png`](screenshots/08_d1_wrist_joint5_mesh_offset_closeup_dark.png) | 11:44:47 | A crop of the D1's wrist pitch (Joint5): the servo horn at the forearm's end and the wrist bracket's screw circle visibly off each other. That is F-137's 6.8 mm forearm difference in the meshes only (F-139's note; the entry "The wrist bracket off its screws" above). |
| [`09_d1_wrist_joint5_mesh_offset_closeup_light.png`](screenshots/09_d1_wrist_joint5_mesh_offset_closeup_light.png) | 11:45:45 | The same joint from a slightly different angle against the door: the horn's screws sit off the bracket's holes. |
| [`10_keyinsert_ki6_force_8200_lock73_seated_low_view.png`](screenshots/10_keyinsert_ki6_force_8200_lock73_seated_low_view.png) | 11:56:14 | `ki6` again, from a lower camera (eye z 0.3 m): the key green in the door, the jaws on its bar, and the dog's head at lower left. The same caveat as shot 7. |

### 2026-11-20 · Built: the mechanisms drawn in the viewer, and clean frames captured from the simulator (12:20)

Lukas asked for better screenshots, the opening mechanisms among them, and for plots. Until now S4's mechanisms could not be screenshotted: they exist only as `mechanism.Mechanism`'s plant (a handle on a path), and nothing of them was drawn.
- **`unifp_train/mech_vis.py`**, hooked in by `mech_env`'s `_set_debug_vis_impl` and `_debug_vis_callback` as `keyinsert_vis` is by the insertion. Each part is drawn from the plant once the grasp closes:
  - a slide pulled toward the robot (drawer, latch): the cabinet behind the closed handle, the drawer's front at the handle and its sides between them;
  - a slide pushed away (button): the housing beyond the travel and the cap;
  - a hinge (door, lid): the leaf from the axis to 5 cm past the handle, 40 cm along the axis; the hinge pin; and a dark doorway where the leaf was closed;
  - the handle bar, the path closed to fully open, the travel goal's reference (yellow), and the mechanism's force on the tool (red, 3 mm a newton).
  - The moving parts are orange while held and opening, green from 0.9 of the travel (`mech_eval`'s "opened"), and red while the drive is over the 80 N budget or on the step the handle tears out.
  - The sizes are drawn, not simulated; the plant is still the handle on its path.
  - `tests/test_mech_vis.py` (9 tests): the drawer's parts in order behind the handle, its sides as long as it is open, the colours, the force arrow, the button, the door's leaf through the axis and on the far side of the handle from the robot, the fixed doorway, the arc, finite frames.
- **`unifp_train/capture.py`** and `play --capture_dir DIR [--capture_every N --capture_episodes N --capture_resolution W H]`:
  - Headless with `--enable_cameras`, it saves environment 0's frames from the viewer camera through Isaac Lab's `env.render()` ('rgb_array'): PNGs at 1920 × 1080 with no Isaac Sim window round them.
  - With each frame goes that frame's task state in `frames.jsonl`: the mechanism's kind, level, open fraction, drive, force and "opened"; the insertion's layer phase, seated, damaged and lost; the key's angle and latch. Screenshots are then picked by what happened.
  - Rendering steps no physics. It is still one episode, one draw of the contact (the entry above), so each picture is captioned from its own capture's log, not from an evaluation.
  - UniFP's goal markers are left out of captures; the task's overlay stays.
- **`run_unifp_train.py`**:
  - `--mech_rows` plays chosen rows of the `mech_eval` plan with the force law as `mech_eval` sets it (`set_force_law`, not the training switch).
  - `--view_eye` and `--view_lookat` set the camera.
  - The viewer presets now apply to headless captures, with a mechanism preset from the robot's front right.
  - Nothing changes for training or any evaluation: every new path needs `play` with `--capture_dir` or `--mech_rows`.
- **`logs/unifp_train/capture/cap.sh <name> <title> [play flags]`**: one capture, the weld lock, recorded on the dashboard.
- **Tests:** `tests.test_mech_vis tests.test_s4c tests.test_mech_layer tests.test_s4b tests.test_mechanism` 90 OK; `tests.test_tool_conditioning tests.test_unifp_roll tests.test_force_transmission` (they import the runner) 57 OK.
- **First capture**, [S4's layer-trained `s4_law_s81_68400` on S4 test row 304](#/week/10/run/20261120T011508_play_seed1_cap_s4_law_s81_68400_door40_r304) (a door at 40 N, 1 N·m, the force law in the loop): 139 frames.
  - In that capture the grasp closed between 1.9 and 2.0 s, the door was 0.95 open by 3.1 s and fully open by 8.2 s. The body's tilt was at most 7.9 deg.
  - Once fully open, the drive pulsed: 13 frames from 9.0 s on are above 80 N (peak 94.2 N at 10.2 s), between frames at 20-40 N, about every 0.3 s. That is the door pushed on its end stop, F-294's kind of violation. With frames 0.1 s apart this does not say whether the 5-step budget rule would have counted it.
  - (Corrected 12:30: the first version of these two lines read the numbers off a subsample of the frames and had the grasp at 2.4-2.5 s, full open at 8.5 s, the peak drive 45.3 N and the tilt 7.3 deg.)

### 2026-11-20 · Last night's runs: seven trainings ended, every validation ran, and the picks by the rules stated before training (12:50)

The queue's last job ended at 00:13 and nothing had been read since. Everything below ran on the spool (`s1tools/runner.sh`) with the commands of the entries that stated it. Every run is recorded.
- **Trainings**, all exit 0 (AEDT):
  - [`khc_s72`](#/week/10/run/20261119T044229_train_seed72_khc_s72) 15:42-17:52 and [`khc_s73`](#/week/10/run/20261119T065230_train_seed73_khc_s73) 17:52-19:56;
  - [S4c-3 `s4c3_A_s91`](#/week/10/run/20261119T050323_train_seed91_s4c3_A_s91) 16:03-18:47 and [`s4c3_A_s92`](#/week/10/run/20261119T094837_train_seed92_s4c3_A_s92) 20:48-22:56;
  - [`ki8_s64`](#/week/10/run/20261119T074743_train_seed64_ki8_s64) 18:47-20:48 and [`ki8_s65`](#/week/10/run/20261119T085616_train_seed65_ki8_s65) 19:56-22:10;
  - [S4b `s4b_B_s93`](#/week/10/run/20261119T115636_train_seed93_s4b_B_s93) 22:56-00:11.
- **Item 3, the hold retrain** (`chain_score.py khc_s7X 7200 7400 7600 7799 --set validation`, the rule stated 2026-11-19):
  - `khc_s72` picks `7799`: chain 72 of 96, 33 inside the lead bound as well, 72 inside the runner's own 25 deg ([f1](#/week/10/run/20261119T065338_leg_eval_seed1_khc_s72_7799_chainvalidation_f1), [f2](#/week/10/run/20261119T065801_leg_eval_seed1_khc_s72_7799_chainvalidation_f2)). 7400 had 73 but 23 inside the bound.
  - `khc_s73` picks `7600`: chain 74, 32 inside the bound, 74 inside 25 deg ([f1](#/week/10/run/20261119T082634_leg_eval_seed1_khc_s73_7600_chainvalidation_f1), [f2](#/week/10/run/20261119T083030_leg_eval_seed1_khc_s73_7600_chainvalidation_f2)).
  - With `khc_s71`'s `7600` (75, 44) all three seeds are picked. The test (`queue_khc_test.sh ki7_s62_9200`, read once) has not run; it needs the GPU.
- **`ki8`, the insertion on weak legs** (`ki7_select.py ki8017_s6X 9600 9800 10000 10199`, on the ki7 validation locks with legs answering 0.17, the rule stated before training): both seeds pick `9800`.
  - `ki8_s64`: 66 of 96 (f1 37, f2 29), 4 damaged, 27 false seats ([f1](#/week/10/run/20261119T090531_leg_eval_seed1_ki8017_s64_9800_val_f1), [f2](#/week/10/run/20261119T091204_leg_eval_seed1_ki8017_s64_9800_val_f2)).
  - `ki8_s65`: 73 (37, 36), 4 damaged, 19 false seats ([f1](#/week/10/run/20261119T101636_leg_eval_seed1_ki8017_s65_9800_val_f1), [f2](#/week/10/run/20261119T102406_leg_eval_seed1_ki8017_s65_9800_val_f2)).
  - **Reported, on the legs answering fully** (as `ki7` trained; not in the selection): `ki8_s64` seats 16 of 96 and damages 63 ([f1](#/week/10/run/20261119T090805_leg_eval_seed1_ki8_s64_9800_val_f1), [f2](#/week/10/run/20261119T091443_leg_eval_seed1_ki8_s64_9800_val_f2)); `ki8_s65` seats 15 and damages 35 ([f1](#/week/10/run/20261119T101943_leg_eval_seed1_ki8_s65_9800_val_f1), [f2](#/week/10/run/20261119T102702_leg_eval_seed1_ki8_s65_9800_val_f2)).
  - The decision rule's frozen-96 runs (at 0.17 and on the legs as trained, against `ki7_s62_9200`'s 124 and 147 of 192) have not run. Its conditions 2 and 3 ask `ki8` to keep `ki7`'s seats on full legs and damage no more keys. On validation it damages 35-63 of 96 there, so it will very likely fail. Not decided until those runs; `ki7_s62_9200` stays the dog's insertion.
- **S4b `s4b_B_s93`** (`s4b_select.py`): `68400`, 544 of 768 opened in budget, no stops, none over ([f1](#/week/10/run/20261119T130329_mech_eval_seed1_s4b_B_s93_68400_val_f1), [f2](#/week/10/run/20261119T130459_mech_eval_seed1_s4b_B_s93_68400_val_f2)). All six S4b seeds are picked; the test (`queue_s4b_test.sh`, read once) has not run.
- **S4c-3** (`s4b_select.py s4c3_A_s9X --suffix val017`, S4b's rule on legs answering 0.17, as stated). `--suffix` is new today; it reads `<run>_<it>_val017_f<F>` and writes `<run>_val017_selected.json`. Both seeds pick `68599`:
  - `s4c3_A_s91`: 165 of 768 opened in budget ([f1](#/week/10/run/20261119T074829_mech_eval_seed1_s4c3_A_s91_68599_val017_f1) 80, [f2](#/week/10/run/20261119T075020_mech_eval_seed1_s4c3_A_s91_68599_val017_f2) 85);
  - `s4c3_A_s92`: 100 ([f1](#/week/10/run/20261119T115724_mech_eval_seed1_s4c3_A_s92_68599_val017_f1) 60, [f2](#/week/10/run/20261119T115854_mech_eval_seed1_s4c3_A_s92_68599_val017_f2) 40).
  - No falls, tilt stops, tears or budget violations in either.
  - **The reading** (stated before training: fit if at least 482 of 768 opened in budget, `ArmWatch` stops at most 5 of 48 dumped, and the replay gate passes every row) **fails on its first condition: S4c-3 is not fit for the dog's runner.** The dump, `ArmWatch` and the replay gate were not run, since the verdict does not depend on them.
  - **What it did learn:** on legs answering 0.17 the picks grasp within the 2 cm gate in 301 and 307 of 384 (s91, f1 and f2) and 299 and 210 (s92), with a median grasp error of 1.4-1.75 cm. S4c-1's `67600` grasped 4 of 384 there. The arm's lead, J1-J5, is a median of 3.8-4.8 deg while holding; its maximum after 1 s is a median of 8.1-10.5 deg, and no episode passes 25 deg.
  - **Where it fails:** most grasps do not open the mechanism. The force curriculum's ceiling ended at 40 N (s91) and 35 N (s92); the 25 N noted yesterday evening was mid-training.
  - On the legs answering fully (reported) s91 grasps 140 and opens 38 of 384 ([run](#/week/10/run/20261119T075215_mech_eval_seed1_s4c3_A_s91_68599_val_f1)); s92 grasps 366 and opens 80 ([run](#/week/10/run/20261119T120025_mech_eval_seed1_s4c3_A_s92_68599_val_f1)).
- **F-307.**
- **Still to run (GPU):** item 3's test, `ki8`'s frozen-96 decision runs, S4b's test.
- **Docs (2026-11-20):** `docs/tool_paper_plan.md`'s H1 line now ends with S4c-3's result (F-307) in place of "S4c-2 trains ...", and its insertion line with `ki8`'s picks and its full-leg damage in place of "`ki8` retrains it".

### 2026-11-20 · Figures for the weekly update: F-303, F-294/F-295, F-306 and F-302/F-305 (12:25)

Lukas asked for plots. Four figures in `figures/`, rebuilt by `python3 results/week_10/figures/week10_figures.py` from the recorded runs under `results/` only (each name's latest run, as the analysis scripts read them).
- **Built by a subagent, CPU only. I checked each against the findings:**
  - The script holds the findings' numbers and refuses to write a figure whose plotted values differ (exit 1).
  - None differ, and all 142 episode files it reads are byte-identical to their `logs/` copies.
  - Scoring is copied from `ki7_compare.py`, `s4_compare.py` and `lockerr_compare.py`: a seat counts only inside the 15 deg tilt stop; "within budget" is opened and not over budget; McNemar is exact and paired by lock.

| figure | what it shows | numbers plotted |
|---|---|---|
| [`fig1_f303_insertion_retrain.png`](figures/fig1_f303_insertion_retrain.png) | F-303: `ki6_force_8200` against the three ki7 picks on ki7's test locks, the dog's conditions and the default contact; seated, false seats, damaged | strict_legs `ki6` 114 / 52 / 26, `ki7_s62_9200` 140 / 48 / 4; ki7 pooled 380 seated and 47 damaged against 3 × (114, 26); default contact `ki6` 136, the pick 115 |
| [`fig2_f294_f295_mechanisms.png`](figures/fig2_f294_f295_mechanisms.png) | F-294 and F-295: S4's four arms on T1's test set, per seed, within budget / over / not opened, and arm (i) with the bleed | (i) 870 / 783 / 615 = 2,268 of 3,072, 176 over; with the bleed 2,412, 23 over; zero-shot 1,049; goal-only 563; `66600` with the layer 193 of 1,024; the script 19 |
| [`fig3_f306_weak_legs.png`](figures/fig3_f306_weak_legs.png) | F-306: `ki7_s62_9200` on the frozen 96 with the legs as trained against legs answering 0.17, by friction | 1 N·m 68 → 76 (15 / 23, p 0.26); 2 N·m 79 → 48 (40 / 9, p 9e-6), false seats 14 → 46; pooled 147 → 124 (p 0.018) |
| [`fig4_f302_f305_lock_estimate.png`](figures/fig4_f302_f305_lock_estimate.png) | F-302 and F-305: seats against each axis's lock-estimate error, `ki6` against `ki7_s62_9200`, with the stated "needed" rule and the training range shaded | `ki7` lateral 147 / 135 / 125 / 104, depth 147 / 149 / 142, roll 122 / 165 / 147 / 94 / 61; needed `ki7` 5.25 mm / 12 mm / 3 deg, `ki6` 7 / 9 / 3 |

- **What they are:** the findings' own numbers drawn, simulation only, as each subtitle says. No new evaluation.
- **Fig 1 and fig 3 draw separate bars, not stacks:** an episode can be both damaged and a false seat (4 are), and a stack would change the findings' counts.
- **Left off fig 2:** F-294's "172 of 176 violations after opening" is not on the figure, since the episode fields cannot confirm it.

### 2026-11-20 · Clean renders for the weekly update: S4's mechanisms, the dog's insertion and the chain's hold (12:40)

Lukas asked for better screenshots, the mechanisms among them. Twelve captures (`logs/unifp_train/capture/batch1.sh`, `batch2.sh`, `batch3.sh`, each run by `cap.sh`), one episode each, headless, 1920 × 1080, a frame every 0.1 s (the insertion's every 0.06 s). Every capture is recorded:
- **S4's layer-trained `s4_law_s81_68400`** on S4 test rows (seed 81's pick, F-294), 1 N·m, the force law in the loop as `mev.sh` ran it (`--mech_set s4_test --force_law --mech_rows R`):
  - [door 40 N, row 304](#/week/10/run/20261120T011508_play_seed1_cap_s4_law_s81_68400_door40_r304);
  - [drawer 40 N, row 48](#/week/10/run/20261120T011755_play_seed1_cap_s4_law_s81_68400_drawer40_r48) and [drawer 70 N, row 97](#/week/10/run/20261120T011935_play_seed1_cap_s4_law_s81_68400_drawer70_r97) from the mechanism preset (front right);
  - [door 60 N, row 342](#/week/10/run/20261120T012119_play_seed1_cap_s4_law_s81_68400_door60_r342).
  - The front-right camera put the drawer's cabinet between the camera and the drawer's front, so both drawers were captured again from the robot's right side (`--view_eye 0.40 -1.30 0.60 --view_lookat 0.40 0.0 0.33`): [40 N](#/week/10/run/20261120T012419_play_seed1_cap_s4_law_s81_68400_drawer40_r48_side), [70 N](#/week/10/run/20261120T012606_play_seed1_cap_s4_law_s81_68400_drawer70_r97_side).
- **`ki7_s62_9200`** on frozen lock 73 under the dog's conditions, 1 N·m, `view.sh`'s flags: [close](#/week/10/run/20261120T012259_play_seed1_cap_ki7_s62_9200_lock73_close) (the insertion preset), [wide](#/week/10/run/20261120T012457_play_seed1_cap_ki7_s62_9200_lock73_wide), and [from behind the robot's right](#/week/10/run/20261120T013109_play_seed1_cap_ki7_s62_9200_lock73_behind) (`--view_eye -0.20 -1.25 0.95`, Lukas's own viewer camera in shots 3-6). The wide one's eye is beyond the door, which hides the key; not used.
- **`khc_s71_7600`** from handover 6 with the hold viewer's flags: [the leg preset](#/week/10/run/20261120T012701_play_seed1_cap_khc_s71_7600_handover6) (the door hides the foot; not used) and [from behind the robot's right](#/week/10/run/20261120T013252_play_seed1_cap_khc_s71_7600_handover6_behind).
- **Capturing does not change the episode:**
  - [Row 304 captured every 7 steps](#/week/10/run/20261120T012758_play_seed1_cap_s4_law_s81_68400_door40_r304_every7) matches the every-5 capture in every logged field at all 19 steps both kept (multiples of 35).
  - The three insertion captures' `frames.jsonl` are byte-identical across cameras, as are the two hold captures'. So a one-environment episode repeats exactly, and the camera and the frame rate do not touch it.
  - The insertion's capture seats lock 73 at 4.68 s, as this morning's one-environment `leg_eval` did (4.66 s, frames 0.06 s apart).

**The renders** (`figures/`, picked by `logs/unifp_train/capture/pick.py` from each capture's own state log; every caption is that frame's state, not an evaluation's):

| render | capture, frame | what it shows |
|---|---|---|
| [`render_01_s4_door40_opening.png`](figures/render_01_s4_door40_opening.png) | door 40 N, 2.5 s | The policy pulls a door with a vertical hinge toward itself, 0.69 open. The leaf is orange (held, opening); the grey panel behind is the doorway where it was closed. The drive is 5 N, after 47 N at 2.3 s broke it loose. |
| [`render_02_s4_door40_open_end_stop.png`](figures/render_02_s4_door40_open_end_stop.png) | door 40 N, 8.5 s | Fully open and green. The red bar is the door's 57 N on the hand: the policy keeps pulling on the end stop after opening, with drive pulses to 94 N from 9 s (F-294's kind of violation; the first capture's entry above). |
| [`render_03_s4_door60_peak_drive.png`](figures/render_03_s4_door60_peak_drive.png) | door 60 N, 3.2 s | 0.73 open at the opening's peak drive, 61.6 N; the red bar is 67 N on the hand. The body pitched 6 deg. It opened at 3.3 s. Fully open from 9.0 s, it then pulsed on the end stop to 111 N (14 frames over 80 N, 9.8-13.7 s). |
| [`render_04_s4_drawer70_opening_side.png`](figures/render_04_s4_drawer70_opening_side.png) | drawer 70 N (side), 2.5 s | From the side: the drawer half out of its cabinet (orange) at 47.6 N of drive, the arm stretched forward, the body rolled 7 deg. |
| [`render_05_s4_drawer70_open_side.png`](figures/render_05_s4_drawer70_open_side.png) | drawer 70 N (side), 3.0 s | Fully out and green, 0.1 s after it opened. The body has stepped back from the cabinet. In S4's test (seed 81's pick, 1 N·m) the 10 drawers at 70 N that opened within the budget moved the base 13.8-22.5 cm at the peak drive. |
| [`render_06_ki7_lock73_searching_close.png`](figures/render_06_ki7_lock73_searching_close.png) | insertion close, 4.02 s | The key at the lock's face, orange: the layer searching for the keyway, 3.6 N on the key. The D1's wrist mesh offset (F-139's note) is visible at left. |
| [`render_07_ki7_lock73_seated_close.png`](figures/render_07_ki7_lock73_seated_close.png) | insertion close, 4.68 s | The key green: seated by the plant's test, 1.0 s after the touch, no damage. |
| [`render_08_ki7_lock73_seated_behind.png`](figures/render_08_ki7_lock73_seated_behind.png) | insertion behind, 4.68 s | The same moment with the whole robot: arm-out stance, the D1 reaching to the door, the key seated. |
| [`render_09_khc_hold_key_past_latch_behind.png`](figures/render_09_khc_hold_key_past_latch_behind.png) | hold behind, 4.9 s | The chain's hold from where an insertion ended: the arm keeps the key in the lock while the front-right foot turns its bar past the latch (30 deg, the latch open). Roll 9 deg, pitch 8 deg. |
| [`render_10_khc_hold_foot_back_behind.png`](figures/render_10_khc_hold_foot_back_behind.png) | hold behind, 12.0 s | Unlocked (green, 40 deg), the foot back on the ground, the arm still holding the key. |

- **Scope:** each render is one simulated episode, drawn markers on analytic plants (the mechanisms, the lock and the key have no simulated bodies). The episodes matched their evaluations' outcomes here (opened, seated, unlocked). This morning's check shows that need not hold for every row.

### 2026-11-20 · The full test suite before the commit: 25 failures, all present at the last commit (13:45)

`python -m unittest discover -s . -p "test_*.py"` (the Isaac env, 39 min; log `logs/unifp_train/capture/full_tests_20261120.log`): 1,536 tests, 23 failures, 2 errors, 7 skipped.
- **Where they are:**
  - 21 failures and 1 error in `demos/combiner_key/tests/test_key_robot.py`. The planner finds no arm pose clear of the head camera ("no arm pose in front of the key from which the whole turn solves without a wrist flip and clear of the head camera"), and `key_plan._retime_turn` gets None.
  - One in `test_bezel_image` and one in `test_insert_robot`.
  - `tests.test_payload` "the meshes lie inside their colliders".
  - `tests.test_remote_buttons`: `CombinerDoorNode` has no `odom`.
- **All 25 fail identically on a clean checkout of the last commit** (`3acb837a`, a detached worktree, the same five modules), so they predate this week's uncommitted work. None of the files they test has changed since; the latest commits to `key_plan.py` and `node.py` are from 2026-11-02 and earlier. Not investigated today.
- Every module this week's code touches passes. `tests/test_evidence.py` passes on the system Python.

## Results

<!-- Tables of metrics and links to figures. Recorded runs are listed by the dashboard automatically. -->

## Findings this week

- F-291 (S2: twins from one parent do not trade places; H3 not supported; F-284's heavy-leg contrast was the parents').
- F-292 (S1c: the false seats' cause; the located seated test fails at evaluation time; the simulated keyway barely constrains roll).
- F-293 (S5: `ki6_force_8200` in numpy matches Isaac to ~1e-6 and passes the approach replay on `66600`'s four stands).
- F-294 (S4: on the D1 link and `66600`'s legs, the layer in training opens 2,268 of 3,072 held mechanisms within the budget over three seeds, the next best arm 1,049; it alone breaks the budget, after opening; H1 holds for T1 in simulation).
- F-295 (S4 follow-up: the law's integral bleed at evaluation time cuts (i)'s budget violations 176 → 23 on the test set and raises opened within the budget 2,268 → 2,412).
- F-296 (item 1b: without the base's pose the learned insertion seats 0 of 192 believing all seated; leg odometry 111, the true base 154; the base moves 18/43 mm and the simulated feet slide).
- F-297 (item 3: the learned insertion handed to the learned hold chains in 62 of 192, a third; the hold loses most of its 2 N·m success from the insertion's end state, outside its trained box range).
- F-298 (item 4: the insertion rests on Joint2-3's torque limit, 0 of 96 at 0.8 of the URDF's, 53 at 0.9; a faster arm damages keys; the settled stance costs 21).
- F-299 (item 1b: in simulation the insertion's arm goals lead the link by at most 6.8 deg in contact, Joint2; `ArmWatch` stops none of 192; the reach's thresholds stand).
- F-300 (H3 on the strict keyway: the twins' contrast is +65 in family F and −142 in family P, each sign in every seed; position-only seats more in both; item 1a's 80% against 32% was the two checkpoints')
- F-301 (item 1: `ki7` learns to lean in on sliding feet the leg odometry cannot see, 72 mm; the layer never feels the touch, 3 of 48 against 30 with the true base; the reward paid the true seat)
- F-302 (item 1b: the insertion tolerates a lock estimate off by 7 mm across, 9 mm along and 3 deg in roll; 4.5 deg of roll halves its seats; within 3 deg the roll trades damaged keys for false seats)
- F-303 (item 1: `ki7_s62_9200` replaces `ki6_force_8200` for the dog by the stated rule: 140 of 192 with 4 damaged against 114 with 26 on the dog's conditions, p 0.004, and it passes S5's gate; one seed in three gains nothing, the false seats stay, and the pick seats 21 fewer where `ki6` was trained)
- F-304 (item 2: S4b's policies are not fit for the dog as trained: grasps 6-7 cm off the handle, the arm's targets 53-65 deg ahead of the link, the base moving 8 cm under the hold so the leg odometry is 5 cm out; the dog's numpy code reproduces Isaac to ~1e-5; the gate fails both on every stand)
- F-305 (item 1b for `ki7_s62_9200`: the dog's lock estimate must be within 5.25 mm across, 12 mm along and 3 deg in roll; it seats more than `ki6` at every shift with fewer damaged keys; seats are not monotone in the roll error, 122 at 0 deg, 165 at 1.5, 147 at 3)
- F-306 (item 1: on legs answering 0.17 the dog's insertion seats 124 of 192 against 147, p 0.018, all of the loss on the stickier legs, 48 against 79 with 46 false seats against 14)
- F-307 (item 2: S4c-3 reaches the handle on the dog's weak legs, 210-307 of 384 grasps within 2 cm a friction and the arm's lead under 25 deg, but opens 165 and 100 of 768 within the budget against the 482 required; not fit, force is the gap).

## Issues and risks

- 25 unit tests fail at the last commit as well (the classical key demo's planner, the payload meshes, the door node): see 2026-11-20 "The full test suite before the commit". They predate this week's work and were not investigated.

## Next week
