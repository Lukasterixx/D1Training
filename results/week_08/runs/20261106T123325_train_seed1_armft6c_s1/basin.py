"""Which pose does an export settle in, over small differences a real run has anyway? The dog's frames held to 10 s, the arm closed on the policy
as in semiclosed.py, each trial with its own report phase, arm lag and start pose. The other session found two settled poses for 66600 on the dog's
own frames, 5.7 and 11 cm from the goal, chosen in the first 1-2 s: one replay cannot say which, so a gate has to read the spread, not one trace."""
import glob, math, os, re, sys
import numpy as np
from multiprocessing import Pool
sys.path.insert(0, '.')
import d1_ik
from demos.combiner_key.robot import leg_policy, reach_arm
from demos.combiner_door.robot import arm_link
sys.path.insert(0, 'logs/unifp_train/armlink')
import handreplay as hr
A = 'logs/unifp_train/armlink/'; dt = .02; N = 500; TRIALS = int(os.environ.get('TRIALS', 24))
def run(P, F, seed):
    g = np.random.default_rng(seed); lag = g.uniform(.06, .09); nxt = g.uniform(0, 1 / 9); q0 = np.radians(g.uniform(-1.5, 1.5, 6))
    pol = leg_policy.LegPolicy(P); m = pol.meta; o = leg_policy.LegObservation(m)
    lim = np.asarray(m['arm_joint_limits'], float); tg = reach_arm.ArmTargets(o.default[12:18], m['action_scale'], m['clip_actions'], lim[:, 0], lim[:, 1])
    fol = arm_link.Follower(np.array(d1_ik.to_servo_deg(q0))); arm = q0.copy(); wire = q0.copy(); rep = reach_arm.ArmReports(q0.copy()); c = 0.0; T, Q = [], []
    for k in range(N):
        f = np.array(F[min(k, len(F) - 1)], float)
        if c >= nxt:
            rep.report(c, np.round(np.degrees(arm) / .1) * .1 * math.pi / 180); nxt += 1 / 9
            wire = np.ravel(d1_ik.from_servo_deg(np.array(fol.step(np.array(d1_ik.to_servo_deg(rep.q)), 0.0, 1 / 9))))
        f[17:23], f[35:41], f[41:59] = rep.q - o.default[12:18], rep.dq * m['obs_scale_dof_vel'], o.prev_actions
        s = o.step(f); a = pol.actor(np.concatenate((np.atleast_2d(s)[:, -pol.num_single_obs:], pol.encoder(np.atleast_2d(s))), -1))[0]; o.record_action(a)
        t = tg.target(o.prev_actions[12:18]); fol.goal = np.array(d1_ik.to_servo_deg(t)); arm = arm + (wire - arm) * (1 - math.exp(-dt / lag)); c += dt
        T.append(t.copy()); Q.append(rep.q.copy())
    return np.array(T), np.array(Q), m
def one(a):
    P, rows, seed = a; d = dict(np.load(rows)); T, Q, m = run(P, d['frame'], seed); O = hr.hand(d, Q, m); e = np.linalg.norm(O, axis=1)
    return os.path.basename(P), os.path.basename(rows), e[-150:].mean() * 100, np.degrees(Q[-1]), np.abs(np.degrees(T - Q))[50:].max()
if __name__ == '__main__':
    L = sys.argv[1:] or ['checkpoints/reach_force_sticky_model_66600.npz']
    S = [A + 'policy_rows.npz', A + 'policy_rows_wood.npz']
    with Pool(8) as p: R = p.map(one, [(P, s, k) for P in L for s in S for k in range(TRIALS)], chunksize=2)
    print(f'{TRIALS} trials a stance: lag 60-90 ms, report phase, start pose +-1.5 deg. Hand cm over the last 3 s; J2 at 10 s')
    print(f"{'export':30s} {'frames':7s} hand min  med  max | J2 min  med  max | J5 min  med  max | lead 1 s+ med max | >8 cm")
    for P in L:
        for s in S:
            r = [x for x in R if x[0] == os.path.basename(P) and x[1] == os.path.basename(s)]; h = np.array([x[2] for x in r]); j = np.array([x[3] for x in r]); l = np.array([x[4] for x in r])
            print(f"{os.path.basename(P).replace('.npz','').replace('reach_force_',''):30s} {'wood' if 'wood' in s else 'carpet':7s} {h.min():7.1f} {np.median(h):4.1f} {h.max():4.1f} | {j[:,1].min():5.0f} {np.median(j[:,1]):4.0f} {j[:,1].max():4.0f} | {j[:,4].min():5.0f} {np.median(j[:,4]):4.0f} {j[:,4].max():4.0f} | {np.median(l):6.0f} {l.max():4.0f} | {(h>8).sum():2d}/{len(h)}")
    np.save(A + 'basin_last.npy', np.array([(x[0], x[1], x[2], *x[3], x[4]) for x in R], dtype=object), allow_pickle=True)
