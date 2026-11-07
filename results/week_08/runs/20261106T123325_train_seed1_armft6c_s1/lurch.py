"""The stick-slip question, from the second 66600 run: the carpet frames held to 10 s, a synthetic leg step at 3.5 s, the arm
closed on the policy as in semiclosed.py. Does a step of the dog's size take lead past ArmWatch's 25 deg held 0.5 s?
Synthetic: no real frame of a step is in this replay until run 20261106T141120 lands."""
import itertools, math, os, sys
import numpy as np
from multiprocessing import Pool
sys.path.insert(0, '.')
import d1_ik
from demos.combiner_key.robot import leg_policy, reach_arm
from demos.combiner_door.robot import arm_link
A = 'logs/unifp_train/armlink/'; P = 'checkpoints/reach_force_sticky_model_66600.npz'; dt = .02; N = 500
base = np.load(A + 'policy_rows.npz')['frame']
THIGH = (1, 4, 7, 10)          # hip, thigh, calf per leg, FR FL RR RL
SETS = {'all': (1, 1, 1, 1), 'front': (1, 1, 0, 0), 'rear': (0, 0, 1, 1), 'left': (0, 1, 0, 1), 'right': (1, 0, 1, 0), 'diag': (1, 0, 0, 1)}
def frames(pattern, delta, tau, t0=3.5, roll=0.0, pitch=0.0):
    F = np.repeat(base[-1:], N, 0); F[:len(base)] = base
    w = np.array(SETS[pattern], float); t = np.arange(N) * dt
    s = np.clip((t - t0) / tau, 0, 1); s = s * s * (3 - 2 * s); v = np.gradient(s, dt)
    for k, i in enumerate(THIGH):
        F[:, 5 + i] += delta * w[k] * s; F[:, 23 + i] += delta * w[k] * v * .05
    F[:, 0] += math.radians(roll) * s; F[:, 1] += math.radians(pitch) * s
    F[:, 2] += math.radians(roll) * v * .25; F[:, 3] += math.radians(pitch) * v * .25
    return F
def run(F, lag=.077):
    pol = leg_policy.LegPolicy(P); m = pol.meta; o = leg_policy.LegObservation(m)
    lim = np.asarray(m['arm_joint_limits'], float); tg = reach_arm.ArmTargets(o.default[12:18], m['action_scale'], m['clip_actions'], lim[:, 0], lim[:, 1])
    fol = arm_link.Follower(np.zeros(6)); arm = np.zeros(6); wire = np.zeros(6); rep = reach_arm.ArmReports(np.zeros(6)); nxt = 0.0; c = 0.0; T, Q = [], []
    for k in range(len(F)):
        f = np.array(F[k], float)
        if c >= nxt:
            rep.report(c, np.round(np.degrees(arm) / .1) * .1 * math.pi / 180); nxt += 1 / 9
            wire = np.ravel(d1_ik.from_servo_deg(np.array(fol.step(np.array(d1_ik.to_servo_deg(rep.q)), 0.0, 1 / 9))))
        f[17:23], f[35:41], f[41:59] = rep.q - o.default[12:18], rep.dq * m['obs_scale_dof_vel'], o.prev_actions
        s = o.step(f); a = pol.actor(np.concatenate((np.atleast_2d(s)[:, -pol.num_single_obs:], pol.encoder(np.atleast_2d(s))), -1))[0]; o.record_action(a)
        t = tg.target(o.prev_actions[12:18]); fol.goal = np.array(d1_ik.to_servo_deg(t)); arm = arm + (wire - arm) * (1 - math.exp(-dt / lag)); c += dt
        T.append(np.degrees(t)); Q.append(np.degrees(rep.q))
    return np.array(T), np.array(Q)
def score(c):
    pat, de, tau, ro, pi = c
    T, Q = run(frames(pat, de, tau, roll=ro, pitch=pi)); L = np.abs(T - Q); w = L[175:]
    over = (w.max(1) > 25); run_len = best = 0
    for x in over:
        run_len = run_len + 1 if x else 0; best = max(best, run_len)
    rate = np.abs(np.diff(T[175:, 3])).max() / dt
    return dict(pat=pat, de=de, tau=tau, ro=ro, pi=pi, lead=w.max(), joint=int(w.max(0).argmax()) + 1, over15=(w.max(1) > 15).sum() * dt, over25=best * dt,
                j4a=T[174, 3], j4lo=T[175:, 3].min(), j4hi=T[175:, 3].max(), j4end=T[-1, 3], rate=rate)
if __name__ == '__main__':
    grid = [('all', 0.0, .25, 0, 0)]
    for pat, de, tau in itertools.product(SETS, (-.13, .13, .25, .4, -.3), (.1, .25, .5)):
        ro = {'left': 2.5, 'right': -2.5}.get(pat, 0) * (1 if de > 0 else -1); pi = {'front': -1.5, 'rear': 1.5}.get(pat, 0) * (1 if de > 0 else -1)
        grid.append((pat, de, tau, ro, pi))
    with Pool(int(sys.argv[1]) if len(sys.argv) > 1 else 6) as p: R = p.map(score, grid, chunksize=2)
    b = R[0]; print(f"no step: lead 3.5 s on {b['lead']:.1f}, J4 {b['j4a']:.0f} to {b['j4end']:.0f}")
    print('the dog, run 2: J4 about 0 to -27, up to 157 deg/s, lead 19.5, over 15 for 0.4 s')
    print('pattern  step   tau  roll pitch | lead (J) over15  over25 | J4 before  low  high  end  peak deg/s')
    for r in sorted(R[1:], key=lambda r: -r['lead']):
        print(f"{r['pat']:6s} {r['de']:+.2f} {r['tau']:.2f} {r['ro']:+.1f} {r['pi']:+.1f} | {r['lead']:5.1f} J{r['joint']} {r['over15']:5.2f} {r['over25']:5.2f} | {r['j4a']:5.0f} {r['j4lo']:5.0f} {r['j4hi']:5.0f} {r['j4end']:5.0f} {r['rate']:6.0f}")
    print(f"\n{len(R)-1} steps: past 25 deg in {sum(r['lead']>25 for r in R[1:])}, past 25 held 0.5 s (the stop) in {sum(r['over25']>=.5 for r in R[1:])}; lead above 19.5 in {sum(r['lead']>19.5 for r in R[1:])}")
