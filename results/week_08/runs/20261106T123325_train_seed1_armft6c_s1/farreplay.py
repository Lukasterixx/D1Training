"""Far goals on the dog's real frames: its legs and body from each 66600 run, held past 10 s, the arm closed on the policy through the real Follower, 9 Hz
reports and the 77 ms lag, the goal sliding as `reach_arm.GoalPath` slides it. A screen, not a forecast (F-258): legs open, so a body that would move does not."""
import glob, math, sys
import numpy as np
from multiprocessing import Pool
sys.path.insert(0, '.'); sys.path.insert(0, 'logs/unifp_train/armlink')
import d1_ik
from demos.combiner_key.robot import leg_policy, reach_arm
from demos.combiner_door.robot import arm_link
dt = .02
P = sys.argv[1] if len(sys.argv) > 1 else 'checkpoints/reach_force_sticky_model_66600.npz'
RUNS = [r for r in sorted(glob.glob('results/week_08/runs/20261106T14*_reach_stand_live/policy_rows.npz')) if any(k in r for k in ('T140642', 'T141120', 'T141455', 'T141945'))]
GOALS = {'low': (.45, 0, 0), 'left': (.45, 36, 40), 'right': (.45, 36, -40), 'forward': (.55, 15, 0), 'low left': (.45, 15, 30), 'low right': (.45, 15, -30), 'wide left': (.5, 25, 60), 'wide right': (.5, 25, -60), 'tour': (.45, 0, 0)}
def one(a):
    name, slide, rows = a; g = GOALS[name]; d = dict(np.load(rows)); F = d['frame']
    pol = leg_policy.LegPolicy(P); m = pol.meta; o = leg_policy.LegObservation(m)
    legs_of = [GOALS[k] for k in ('low', 'left', 'right')] if name == 'tour' else [g]
    path = reach_arm.GoalPath(m['reach_goals']['init_start'], [((x[0], math.radians(x[1]), math.radians(x[2])), slide, 3.0 if name == 'tour' else 4.0) for x in legs_of], 3.0); N = int(path.duration / dt)
    scale = np.asarray(m['commands_scale'])[3:6]
    lim = np.asarray(m['arm_joint_limits'], float); tg = reach_arm.ArmTargets(o.default[12:18], m['action_scale'], m['clip_actions'], lim[:, 0], lim[:, 1])
    fol = arm_link.Follower(np.zeros(6)); arm = np.zeros(6); wire = np.zeros(6); rep = reach_arm.ArmReports(np.zeros(6)); nxt = 0.0; c = 0.0; T, Q, G = [], [], []
    for k in range(N):
        f = np.array(F[min(k, len(F) - 1)], float)
        if c >= nxt:
            rep.report(c, np.round(np.degrees(arm) / .1) * .1 * math.pi / 180); nxt += 1 / 9
            wire = np.ravel(d1_ik.from_servo_deg(np.array(fol.step(np.array(d1_ik.to_servo_deg(rep.q)), 0.0, 1 / 9))))
        f[17:23], f[35:41], f[41:59] = rep.q - o.default[12:18], rep.dq * m['obs_scale_dof_vel'], o.prev_actions
        if k > 0:
            f[64:67] = path.at(k * dt) * scale
        s = o.step(f); a2 = pol.actor(np.concatenate((np.atleast_2d(s)[:, -pol.num_single_obs:], pol.encoder(np.atleast_2d(s))), -1))[0]; o.record_action(a2)
        t = tg.target(o.prev_actions[12:18]); fol.goal = np.array(d1_ik.to_servo_deg(t)); arm = arm + (wire - arm) * (1 - math.exp(-dt / .077)); c += dt
        T.append(np.degrees(t)); Q.append(np.degrees(rep.q)); G.append(np.abs(o.prev_actions[:12]).max() * m['action_scale'])
    T, Q, G = np.array(T), np.array(Q), np.array(G); L = np.abs(T - Q).max(1); run = best = 0
    for v in L[50:]:
        run = run + 1 if v > 25 else 0; best = max(best, run)
    cyc = 6; st = np.abs(T[cyc:] - T[:-cyc]).max(1)[50:].max()
    held = int((3 + slide + 3.5) / dt)
    return name, slide, rows.split('/')[-2][9:15], L[50:].max(), best * dt, st, G[:50].max(), G[50:].max(), Q[held]
if __name__ == '__main__':
    jobs = [(n, s, r) for n in GOALS for s in (3.0, 4.0) for r in RUNS]
    with Pool(8) as p: R = p.map(one, jobs, chunksize=2)
    print(P); print('goal        slide | lead worst, each run          | over 25 s | step | legs 1st after | arm report at hold, run 1')
    for n in GOALS:
        for s in (3.0, 4.0):
            r = [x for x in R if x[0] == n and x[1] == s]
            print(f"{n:11s} {s:3.0f}  | " + ' '.join(f'{x[3]:5.1f}' for x in r) + f" | {max(x[4] for x in r):5.2f} | {max(x[5] for x in r):4.1f} | {max(x[6] for x in r):.2f} {max(x[7] for x in r):.2f} | {[int(round(v)) for v in r[0][8]]}")
