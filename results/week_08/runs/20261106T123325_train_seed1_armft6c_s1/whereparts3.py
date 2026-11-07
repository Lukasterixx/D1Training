"""A play band in the arm: the output stays within +-db of the lagged command and does not move for less, which is what the arm's own command log shows at rest. No pose is fitted:
the band is swept, and it either brings the closed loop to the dog's settled pose on all four runs or it does not."""
import glob, math, sys
import numpy as np
from multiprocessing import Pool
sys.path.insert(0, '.')
import d1_ik
from demos.combiner_key.robot import leg_policy, reach_arm
from demos.combiner_door.robot import arm_link
P = sys.argv[1] if len(sys.argv) > 1 else 'checkpoints/reach_force_sticky_model_66600.npz'; dt = .02
RUNS = [r for r in sorted(glob.glob('results/week_08/runs/20261106T14*_reach_stand_live/policy_rows.npz')) if any(k in r for k in ('T140642', 'T141120', 'T141455', 'T141945'))]
def closed(a):
    r, db, real_ticks = a
    d = dict(np.load(r)); F = d['frame']; N = len(F); RT = np.degrees(d['arm_target']); RQ = d['arm_obs']; RD = np.degrees(RQ)
    pol = leg_policy.LegPolicy(P); m = pol.meta; o = leg_policy.LegObservation(m)
    lim = np.asarray(m['arm_joint_limits'], float); tg = reach_arm.ArmTargets(o.default[12:18], m['action_scale'], m['clip_actions'], lim[:, 0], lim[:, 1])
    q0 = RQ[0].copy(); band = math.radians(db)
    fol = arm_link.Follower(np.array(d1_ik.to_servo_deg(q0))); lagged = q0.copy(); arm = q0.copy(); wire = q0.copy(); rep = reach_arm.ArmReports(q0.copy()); nxt = 0.0; c = 0.0; T, Q = [], []
    ticks = set((np.nonzero(np.abs(np.diff(RQ, axis=0)).max(1) > 1e-9)[0] + 1).tolist())
    for k in range(N):
        f = np.array(F[k], float)
        if (k in ticks) if real_ticks else (c >= nxt):
            rep.report(c, np.round(np.degrees(arm) / .1) * .1 * math.pi / 180); nxt += 1 / 9
            wire = np.ravel(d1_ik.from_servo_deg(np.array(fol.step(np.array(d1_ik.to_servo_deg(rep.q)), 0.0, 1 / 9))))
        f[17:23], f[35:41], f[41:59] = rep.q - o.default[12:18], rep.dq * m['obs_scale_dof_vel'], o.prev_actions
        s = o.step(f); x = pol.actor(np.concatenate((np.atleast_2d(s)[:, -pol.num_single_obs:], pol.encoder(np.atleast_2d(s))), -1))[0]; o.record_action(x)
        t = tg.target(o.prev_actions[12:18]); fol.goal = np.array(d1_ik.to_servo_deg(t))
        lagged = lagged + (wire - lagged) * (1 - math.exp(-dt / .077)); arm = np.clip(arm, lagged - band, lagged + band); c += dt
        T.append(np.degrees(t)); Q.append(np.degrees(rep.q))
    T, Q = np.array(T), np.array(Q)
    return r.split('/')[-2][9:15], db, real_ticks, float(np.sqrt(((T - RT) ** 2).mean())), float(np.sqrt(((Q - RD) ** 2).mean())), T[-1, 1], T[-1, 4], float(np.abs(T - Q)[50:].max())
if __name__ == '__main__':
    combos = [(db, rt) for rt in (1, 0) for db in (0, .1, .2, .3, .4, .5, .6, .8, 1.0, 1.5)]
    with Pool(10) as p: R = p.map(closed, [(r, *c) for c in combos for r in RUNS], chunksize=2)
    print('real ticks, band deg | target RMS each run | report RMS | J2 at 10 s each (dog 4-5) | J5 at 10 s each (dog 22-28) | lead 1 s+ each (dog 6.9 19.5 4.6 4.4)')
    for c in combos:
        x = [r for r in R if r[1:3] == c]
        print(f"{c[1]} {c[0]:4.1f} | " + ' '.join(f'{v[3]:5.1f}' for v in x) + ' | ' + ' '.join(f'{v[4]:4.1f}' for v in x) + ' | ' + ' '.join(f'{v[5]:4.0f}' for v in x) + ' | ' + ' '.join(f'{v[6]:4.0f}' for v in x) + ' | ' + ' '.join(f'{v[7]:5.1f}' for v in x))
