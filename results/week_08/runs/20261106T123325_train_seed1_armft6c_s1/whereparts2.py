"""The replay with the real rhythm: the reports at the ticks they reached the dog (where its recorded arm_obs changes), the arm from its recorded start pose, and the first
command as soon as there is a target. Which of these brings the closed loop back to the dog's settled pose, on all four runs? Nothing here is fitted to the pose."""
import glob, math, sys, itertools
import numpy as np
from multiprocessing import Pool
sys.path.insert(0, '.')
import d1_ik
from demos.combiner_key.robot import leg_policy, reach_arm
from demos.combiner_door.robot import arm_link
P = 'checkpoints/reach_force_sticky_model_66600.npz'; dt = .02
RUNS = [r for r in sorted(glob.glob('results/week_08/runs/20261106T14*_reach_stand_live/policy_rows.npz')) if any(k in r for k in ('T140642', 'T141120', 'T141455', 'T141945'))]
def closed(a):
    r, real_ticks, real_start, early, lag = a
    d = dict(np.load(r)); F = d['frame']; N = len(F); RT = np.degrees(d['arm_target']); RQ = d['arm_obs']
    pol = leg_policy.LegPolicy(P); m = pol.meta; o = leg_policy.LegObservation(m)
    lim = np.asarray(m['arm_joint_limits'], float); tg = reach_arm.ArmTargets(o.default[12:18], m['action_scale'], m['clip_actions'], lim[:, 0], lim[:, 1])
    q0 = RQ[0].copy() if real_start else np.zeros(6)
    fol = arm_link.Follower(np.array(d1_ik.to_servo_deg(q0))); arm = q0.copy(); wire = q0.copy(); rep = reach_arm.ArmReports(q0.copy()); nxt = (dt if early else 0.0); c = 0.0; T, Q = [], []
    ticks = set((np.nonzero(np.abs(np.diff(RQ, axis=0)).max(1) > 1e-9)[0] + 1).tolist())
    for k in range(N):
        f = np.array(F[k], float)
        due = (k in ticks) if real_ticks else (c >= nxt)
        if due:
            rep.report(c, np.round(np.degrees(arm) / .1) * .1 * math.pi / 180); nxt += 1 / 9
            wire = np.ravel(d1_ik.from_servo_deg(np.array(fol.step(np.array(d1_ik.to_servo_deg(rep.q)), 0.0, 1 / 9))))
        f[17:23], f[35:41], f[41:59] = rep.q - o.default[12:18], rep.dq * m['obs_scale_dof_vel'], o.prev_actions
        s = o.step(f); x = pol.actor(np.concatenate((np.atleast_2d(s)[:, -pol.num_single_obs:], pol.encoder(np.atleast_2d(s))), -1))[0]; o.record_action(x)
        t = tg.target(o.prev_actions[12:18]); fol.goal = np.array(d1_ik.to_servo_deg(t)); arm = arm + (wire - arm) * (1 - math.exp(-dt / lag)); c += dt
        T.append(np.degrees(t)); Q.append(np.degrees(rep.q))
    T = np.array(T); return r.split('/')[-2][9:15], real_ticks, real_start, early, lag, float(np.sqrt(((T - RT) ** 2).mean())), T[-1, 1], T[-1, 4], RT[-1, 1], RT[-1, 4], float(np.sqrt(((T[:100] - RT[:100]) ** 2).mean()))
if __name__ == '__main__':
    combos = [(a, b, c, l) for a, b, c in ((0, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), (1, 1, 0), (0, 1, 1)) for l in (.077, .065, .055, .045)]
    with Pool(10) as p: R = p.map(closed, [(r, *c) for c in combos for r in RUNS], chunksize=2)
    print('ticks start early lag  | target RMS each run, first 2 s RMS mean | J2 at 10 s each run (dog 4-5)')
    for c in combos:
        x = [r for r in R if r[1:5] == c]
        print(f'  {c[0]}     {c[1]}     {c[2]}  {c[3]:.3f} | ' + ' '.join(f'{v[5]:5.1f}' for v in x) + f' | {np.mean([v[10] for v in x]):4.1f} | ' + ' '.join(f'{v[6]:4.0f}' for v in x))
