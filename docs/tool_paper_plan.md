# Tool-mediated force-position control: work plan

Written 13 November 2026 (Week 9) for the training PC, to start the simulations behind the RSS 2027 tool paper
([`paper/rss2027_tools/`](../paper/rss2027_tools/README.md)). **This is a proposal.** It changes Thesis C's priorities
(below), and Lukas has not yet agreed that with his supervisor. Nothing here relaxes
[the Thesis B plan](thesis_b_plan.md)'s rules, gates or evidence standards.

## Where this stands

The paper tests four hypotheses (its Sec. V):

| | Hypothesis | Thesis plan | Evidence now |
| --- | --- | --- | --- |
| H1 | A task layer that turns tool state into force-position references beats position scripting and outcome-trained policies | (the method) | Simulation, one seed. T1 on the old 50 Hz arm (F-105 to F-110); T2 to T4 on the D1 link (F-274, F-278, F-284) |
| H2 | Reward following the layer's reference, not the outcome | (the method) | Simulation, one seed per variant (F-109, F-283, F-285) |
| H3 | Force awareness with a tool is conditional: it wins under body and actuation shifts, loses under tool and lock shifts | P3 vs P2 | Simulation; the twins come from **different parents and seeds** (F-284, F-285) |
| H4 | Tool descriptors improve generalization to held-out tools | P4 vs P3 | **None.** Tool geometry is a constant in the code |

No learned tool policy has run on the dog. Stage 1 is due **4 Dec**. If the paper is invited, its Extension Charter
arrives 26 Feb and the full paper is due 16 Apr 2027.

## Rules for every run here

- [CLAUDE.md](../CLAUDE.md) applies unchanged:
  - check `nvidia-smi` and ask before sharing the GPU;
  - record every run with `./dashboard.py record`;
  - add a dated log entry with the command and its numbers;
  - put findings in `results/findings.md` as the next F-number.
- **State the selection rule in the log before any checkpoint is scored.** Select on a validation set; score once on a
  test set nobody has looked at. The frozen sets used so far (e.g. `keyinsert_eval`'s 96) are spent as test sets for
  anything selected on them. Draw new validation and test sets on new RNG streams.
- **Three seeds** for any result the paper will claim. One seed is a pilot, and is labelled as one.
- **Matched means matched:** the same parent, transition budget, legs, arm link, task layer, evaluation sets and
  selection rule. Only the declared difference changes.
- **Position-only means the plan's P2:** no force command, no force estimate or force-supervised latent in the actor, and
  no force reward. Check what `--leg_variant position` removes before relying on it.
- **Default conditions are the deployable ones:**
  - the D1 link: report-locked, the F-250 lag model, through the Follower;
  - `66600`'s legs: friction 0-2 N·m, delay 1-4 steps, the dog's stance in 3 starts of 4;
  - the runner's 15 deg tilt stop in training (F-284 round 6);
  - a learning rate pinned at 3e-5 or lower for key-family fine-tunes (F-279).

## Simulation work, in priority order

Throughput for reference: insertion training ran at about 2.8 s an iteration (week 9 log, round 1), so 1,000
iterations is about 45 min.

### S1 · Tool conditioning on the key insertion (H4). Pilot before 4 Dec

The biggest gap, and the one the supervisor will ask about.

**Build:**
1. **Make the key a per-environment variable.** In `unifp_train/keylock.py` and `keyinsert_env.py`, these are
   module constants today (`KEY_LENGTH_M`, `SHAFT_R_M`, `KEY_BACK_LINK6_Z_M`, hence `TIP_FROM_JAW_M`). Turn into
   per-environment tensors, drawn at reset:
   - the key's reach beyond the jaws (tip offset along the approach);
   - the shaft's lateral offset in the jaws (F-224 measured 2 mm on the dog);
   - grip compliance and slip force (`grip_slip_n` is already a parameter);
   - the key's mass.

   The lock's hole follows the shaft radius. Keep the keyway's geometry fixed so only the tool varies.
2. **The task layer must use the tool's tip.** The layer and the success test take the tip from the drawn tool, not
   the constant. Update `tests/test_keylock.py` and add a test that two environments with different keys seat at their
   own tips.
3. **Add a descriptor observation** (P4 only): tip offset (3), axis (3), length, lateral offset, grip compliance and
   mass, scaled to about unit range.
   - To keep the comparison matched, give **P3 the same widened network with the descriptor inputs held at zero**.
     Then capacity and warm start are identical, and only the information differs.
   - Widen the input layer of the parent checkpoint with zero-initialized weights, so both can warm-start from
     `logs/unifp_train/keyinsert/ki6_force_8200.pt`.
4. **Tool sets** in a versioned manifest beside `keyinsert_eval`'s locks:
   - *train*: two or three keys spanning the range;
   - *validation*: train keys on new locks;
   - *held-out test*: keys in the gaps of the training range and beyond both ends, on new locks;
   - *wrong descriptor*: a train key with its descriptor off by a stated amount, to test that errors degrade it
     gracefully.

**Runs:**
- **Pilot (one seed each, before 4 Dec):** P3 and P4 from the widened `8200`, about 1,500 iterations each with
  `round6.sh`'s settings, plus a single-key specialist (the current key) as the floor.
  - Score all three on the held-out keys at 1 and 2 N·m.
  - Report successes inside the tilt stop, false seats, peak force and the touch error.
  - Send the numbers back to this PC for the paper's H4 paragraph, labelled as a one-seed pilot.
- **Full (Stage 2):** three seeds each, the same rule.
  - Add a second tool family on another task for generality: the press probe (`unifp_train/pressing.py`'s `Tool`
    already has `bare` and `probe`, so extend it to lengths and radii), or wiping pads (thickness, compliance).

**Selection rule to state first:** no falls, then the most successes inside the tilt stop on the validation set.
Score the held-out test set once.

**H4 is falsified if** P4 is no better than P3 on held-out keys, or no better than the specialist, within seed spread.
Report it either way.

### S2 · Twins from a common parent (H3). Start in Week 10

F-284's and F-285's twins come from different parents and seeds (F-285's scope). That is the first thing a reviewer will
attack.

**Build:**
1. **Choose the common parent and say why in the log.** Either option is acceptable if it is stated:
   - (a) one checkpoint, fine-tuned into both variants. It is cheaper, but position-only inherits force-trained
     features (as round 11 did in F-278's note).
   - (b) a parent from before the force curriculum. It is cleaner and costs more.
2. **A grid evaluation for the insertion:** body shifts crossed with tool shifts, every cell on the same validation
   and test locks.
   - Body shifts: leg armature (light, zero, heavy), delay (0, 5-20, 15-20 ms), stance (dog's, settled).
   - Tool shifts: lock stiffness, tightness, edge breaks, estimate error x1, x1.3, x1.5, and key reach from S1.
3. **An analysis script** (`statsmodels` mixed-effects logistic: success ~ variant x shift-type + (1 | seed) +
   (1 | condition)). Its result answers H3.

**Runs:** three seeds per variant, about 800 iterations each from the parent with round 6's settings.

**H3 is falsified if** the variant-by-shift interaction is not distinguishable from zero.

### S3 · Reference against outcome reward, from one parent (H2)

**Build:** for the key hold's position-only twin, price falling behind the arc as well as running ahead
(F-285's implication: "a band both ways"). Without that, its stalls may be the reward's, not a lack of force sense.

**Runs:** from one key-hold parent on `66600`'s legs (F-282's recipe, `logs/unifp_train/keyhold/round28.sh`), three
seeds each:
- outcome reward;
- following reward (`--key_turn_follow_weight -15`);
- following reward with the band.

**Score:** with `turn_speed.py` and `lead6.py` on new validation and test keys:
- success and keys lost after unlocking;
- the fraction of keys running ahead of the reference;
- peak key speed;
- Joint6's lead;
- stall angle.

**Then the H3 key-turn comparison:** force-aware against position-only under the band, the same seeds.

### S4 · The mechanisms and claw on the deployable models (H1)

F-105 to F-110 predate the D1 link and the leg models. Retrain T1 (held mechanisms) through the D1 link on `66600`'s
legs, three seeds, in four arms on one budget:
- (i) the layer in training;
- (ii) the layer added to a policy not trained with it;
- (iii) outcome-trained, goal only;
- (iv) a position script.

Measure success, force-budget violations and peaks on a new frozen set. The claw combiner demo (F-110, F-114) after
that, if there is time before the Stage 2 deadline.

### S5 · Deployment preparation for the insertion (simulation side of E5)

So that the dog can run `ki6_force_8200`, or S1's P4 if it wins:
- **A numpy export and runner** for the 18-output insertion policy with the IK reference and the task layer.
  - Parity with Isaac to about 1e-6, as F-171 did for the key policy.
  - The layer's decisions logged.
- **A replay gate** for this family on `66600`'s recorded frames (`gate4.py`'s pattern), with its thresholds stated
  first.
- **The box raised** to the trained 0.45 m in the dog program's config.

Hardware stays with Lukas.

## Hardware and measurement (Lukas, this PC)

Each of these unblocks a claim the paper currently has to hedge:
- A **load cell behind the cam lock**: force ground truth for G6 and E5.
- **How hard the real D1 pushes at a few mm of lag.** The layer's 3 mm touch and 0.8 mm feel depend on it (F-284's
  scope).
- **Contact stiffness** of the PLA key in the metal keyway, the lock's turning torque, and the door's load (G2).
- **An external hand measurement** on a few runs (a tag on the gripper, seen by the calibrated front camera).
- **E5:**
  - first, 20 insertion trials across four box placements, learned force-aware against the scripted press;
  - measures: seated, overshoot past seat, peak force, time;
  - then force-aware against position-only wiping on a force plate.

## Timeline

| When | Training PC | This PC / Lukas |
| --- | --- | --- |
| Week 10 (16-22 Nov) | S1 build and pilot; S2 parent chosen | Thesis B report; decide which paper |
| 23 Nov - 4 Dec | S1 pilot numbers back; S2 runs start | Pilot into the draft; read Ctrl-F; submit Stage 1 |
| Dec - Jan | S2, S3 three seeds; S5 export and gate | Load cell; push and stiffness measurements |
| 5-26 Feb | What the Extension Charter asks | Rebuttal (by 12 Feb) |
| Mar - 16 Apr | S1 full, S4 | E5 hardware trials; final paper |

## Decisions pending (Lukas and supervisor)

1. **Thesis C priorities.** The current plan gives early C to markerless perception and broader trials. This direction
   needs those weeks for S1 to S4 and E5.
2. **Which tool family carries H4 past the pilot.** Keys are cheapest, since everything exists. Pads tie to the wipes
   already done on the dog. The claw ties to the box but needs the most rebuilding.
3. **G7.** If the learned insertion replaces the scripted press in the dog's chain, one piece of work serves G7 and
   the paper.
