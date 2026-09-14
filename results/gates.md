# Decision gates

From [the Thesis B plan](../docs/thesis_b_plan.md#gates-and-measurement-definitions). Criteria are
proposed engineering targets until frozen (planned for Week 6). A gate passes only on the evidence
it names. Interface smoke runs and training-time diagnostics do not pass G1.

| Gate | Criterion | Status | Evidence and notes |
| --- | --- | --- | --- |
| G0 · Environment works | Correct articulation/action mapping; finite state; deliberate timeout/fall and partial-reset tests pass; no clone interference; target-frame checks pass | in progress | CPU frame and PPO-API checks pass ([F-003](findings.md)); GPU prerequisite check passes ([Week 1](week_01/notes.md)). The task then gained unitree_rl_lab's leg motor model, split actor/critic observations and a deploy export (F-005, F-006); none has run in simulation yet. Deliberate reset tests, clone isolation and a registered end-effector frame are still to do |
| G1a · Free-space baseline | 100 frozen 10 s episodes per seed: ≥90% reach within 5 cm for ≥1 s continuous; survive to episode end; ≤1% falls | not started | Needs the frozen-manifest evaluator (not yet written) |
| G1b · Moving baseline | G1a criteria on slow trajectories, plus commanded-versus-measured base velocity | not started | |
| G2 · Contact suite | Fixture travel/force signs and contact signals calibrated; repeatable initial states; explicit success/failure definitions; no hidden force inputs to P0/P2 | not started | |
| G3 · Experiment ready | ≥3 independent training seeds; matched budgets and test cases; complete configs, checkpoints and metrics saved | not started | |
