"""Every armft6 export on the four real 66600 runs' frames: the legs and body of a policy that held, the arm closed as in semiclosed.py.
66600 first: the dog's own numbers for it are known (lead 4.4-19.5 from 1 s on, leg target at most 0.56 / 0.24-0.29), so it shows what this test reads."""
import glob, math, os, re, sys
import numpy as np
from multiprocessing import Pool
sys.path.insert(0, '.'); sys.path.insert(0, 'logs/unifp_train/armlink')
import d1_ik
from demos.combiner_key.robot import leg_policy, reach_arm
from demos.combiner_door.robot import arm_link
import handreplay as hr
A = 'logs/unifp_train/armlink/'; dt = .02
RUNS = sorted(glob.glob('results/week_08/runs/20261106T14*_reach_stand_live/policy_rows.npz'))
RUNS = [r for r in RUNS if any(k in r for k in ('T140642', 'T141120', 'T141455', 'T141945'))]
def run(P, F, lag=.077):
    pol = leg_policy.LegPolicy(P); m = pol.meta; o = leg_policy.LegObservation(m)
    lim = np.asarray(m['arm_joint_limits'], float); tg = reach_arm.ArmTargets(o.default[12:18], m['action_scale'], m['clip_actions'], lim[:, 0], lim[:, 1])
    fol = arm_link.Follower(np.zeros(6)); arm = np.zeros(6); wire = np.zeros(6); rep = reach_arm.ArmReports(np.zeros(6)); nxt = 0.0; c = 0.0; T, Q, G = [], [], []
    for k in range(len(F)):
        f = np.array(F[k], float)
        if c >= nxt:
            rep.report(c, np.round(np.degrees(arm) / .1) * .1 * math.pi / 180); nxt += 1 / 9
            wire = np.ravel(d1_ik.from_servo_deg(np.array(fol.step(np.array(d1_ik.to_servo_deg(rep.q)), 0.0, 1 / 9))))
        f[17:23], f[35:41], f[41:59] = rep.q - o.default[12:18], rep.dq * m['obs_scale_dof_vel'], o.prev_actions
        s = o.step(f); a = pol.actor(np.concatenate((np.atleast_2d(s)[:, -pol.num_single_obs:], pol.encoder(np.atleast_2d(s))), -1))[0]; o.record_action(a)
        t = tg.target(o.prev_actions[12:18]); fol.goal = np.array(d1_ik.to_servo_deg(t)); arm = arm + (wire - arm) * (1 - math.exp(-dt / lag)); c += dt
        T.append(t.copy()); Q.append(rep.q.copy()); G.append(np.abs(np.asarray(o.prev_actions[:12], float)) * m['action_scale'])
    return np.array(T), np.array(Q), np.array(G), m
def one(a):
    P, r = a; d = dict(np.load(r)); T, Q, G, m = run(P, d['frame']); e = np.linalg.norm(hr.hand(d, Q, m), axis=1) * 100; L = np.abs(np.degrees(T - Q))
    return os.path.basename(P), r, L[50:].max(), G[:50].max(), G[50:].max(), e[-150:].mean(), np.degrees(Q[-1, 1])
if __name__ == '__main__':
    L = ['checkpoints/reach_force_sticky_model_66600.npz'] + sorted(glob.glob(A + os.environ.get('GATE_GLOB', 'reach_6[bc]_*.npz')), key=lambda p: (p.split('_')[-2], int(re.search(r'_(\d+)\.npz', p).group(1))))
    L = [p for p in L if not p.endswith('reach_6b_66600.npz')]
    with Pool(8) as p: R = p.map(one, [(P, r) for P in L for r in RUNS], chunksize=2)
    print(f'{len(RUNS)} real 66600 runs. Worst over the runs, except hand and J2 (range). Gates: lead <= 15, leg target <= 1.2 first s, <= 0.9 after')
    print(f"{'export':28s} lead 1 s+ | leg first s, after | hand cm last 3 s | J2 at 10 s | passes")
    for P in L:
        r = [x for x in R if x[0] == os.path.basename(P)]; ld = max(x[2] for x in r); g1 = max(x[3] for x in r); g2 = max(x[4] for x in r); h = [x[5] for x in r]; j = [x[6] for x in r]
        print(f"{os.path.basename(P).replace('.npz','').replace('reach_force_','').replace('reach_',''):28s} {ld:7.1f}   | {g1:6.2f} {g2:6.2f}      | {min(h):5.1f}-{max(h):5.1f}      | {min(j):4.0f} to {max(j):4.0f} | {'PASS' if ld <= 15 and g1 <= 1.2 and g2 <= .9 else ''}")
