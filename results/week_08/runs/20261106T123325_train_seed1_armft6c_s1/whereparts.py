"""Where the replay parts from the dog, one link at a time, on 66600's own frames.
1 open: the recorded frames straight into the policy: it has to give the recorded targets, or the replay was never the dog's policy.
2 arm: the recorded targets through Follower and the lag: it has to give the recorded reports, or the arm model is what is wrong.
3 closed, one input at a time: which of the arm's position, speed and last action, taken from the model not the record, sends it to the other pose."""
import glob, math, sys
import numpy as np
sys.path.insert(0, '.')
import d1_ik
from demos.combiner_key.robot import leg_policy, reach_arm
from demos.combiner_door.robot import arm_link
P = 'checkpoints/reach_force_sticky_model_66600.npz'; dt = .02
RUNS = [r for r in sorted(glob.glob('results/week_08/runs/20261106T14*_reach_stand_live/policy_rows.npz')) if any(k in r for k in ('T140642', 'T141455'))]
def setup():
    pol = leg_policy.LegPolicy(P); m = pol.meta; o = leg_policy.LegObservation(m)
    lim = np.asarray(m['arm_joint_limits'], float); tg = reach_arm.ArmTargets(o.default[12:18], m['action_scale'], m['clip_actions'], lim[:, 0], lim[:, 1])
    return pol, m, o, tg
def act(pol, o, f):
    s = o.step(f); a = pol.actor(np.concatenate((np.atleast_2d(s)[:, -pol.num_single_obs:], pol.encoder(np.atleast_2d(s))), -1))[0]; o.record_action(a); return a
for r in RUNS:
    d = dict(np.load(r)); F = d['frame']; t = d['t'] - d['t'][0]; RT = np.degrees(d['arm_target']); RQ = np.degrees(d['arm_obs']); N = len(F); name = r.split('/')[-2][9:15]
    print('==', name, 'rows', N, 'recorded action cols', d['action'].shape)
    pol, m, o, tg = setup(); T = []
    for k in range(N):
        f = np.array(F[k], float); act(pol, o, f); o.prev_actions = o.prev_actions; T.append(np.degrees(tg.target(o.prev_actions[12:18])))
    T = np.array(T); print('1 open  target RMS per joint', np.round(np.sqrt(((T - RT) ** 2).mean(0)), 2), 'max', round(float(np.abs(T - RT).max()), 2))
    # 2: the recorded targets through the arm model; reports at the recorded rhythm.
    fol = arm_link.Follower(np.zeros(6)); arm = np.zeros(6); wire = np.zeros(6); rep = reach_arm.ArmReports(np.zeros(6)); nxt = 0.0; c = 0.0; Q = []
    for k in range(N):
        if c >= nxt:
            rep.report(c, np.round(np.degrees(arm) / .1) * .1 * math.pi / 180); nxt += 1 / 9
            wire = np.ravel(d1_ik.from_servo_deg(np.array(fol.step(np.array(d1_ik.to_servo_deg(rep.q)), 0.0, 1 / 9))))
        fol.goal = np.array(d1_ik.to_servo_deg(np.radians(RT[k]))); arm = arm + (wire - arm) * (1 - math.exp(-dt / .077)); c += dt
        Q.append(np.degrees(rep.q))
    Q = np.array(Q); e = Q - RQ
    print('2 arm   report RMS per joint', np.round(np.sqrt((e ** 2).mean(0)), 2), 'first 2 s', np.round(np.sqrt((e[:100] ** 2).mean(0)), 2), 'mean offset first 2 s', np.round(e[:100].mean(0), 2))
    for x in (.3, .6, 1.0, 1.5, 2.0):
        i = int(np.argmin(abs(t - x))); print(f'    t {x:.1f} model {np.round(Q[i],1)} dog {np.round(RQ[i],1)}')
    # 3: closed on the policy, with each input from the model or the record.
    def closed(use_q, use_dq, use_prev):
        pol, m, o, tg = setup(); fol = arm_link.Follower(np.zeros(6)); arm = np.zeros(6); wire = np.zeros(6); rep = reach_arm.ArmReports(np.zeros(6)); nxt = 0.0; c = 0.0; TT = []
        for k in range(N):
            f = np.array(F[k], float)
            if c >= nxt:
                rep.report(c, np.round(np.degrees(arm) / .1) * .1 * math.pi / 180); nxt += 1 / 9
                wire = np.ravel(d1_ik.from_servo_deg(np.array(fol.step(np.array(d1_ik.to_servo_deg(rep.q)), 0.0, 1 / 9))))
            if use_q: f[17:23] = rep.q - o.default[12:18]
            if use_dq: f[35:41] = rep.dq * m['obs_scale_dof_vel']
            if use_prev: f[41:59] = o.prev_actions
            act(pol, o, f); x = tg.target(o.prev_actions[12:18]); fol.goal = np.array(d1_ik.to_servo_deg(x)); arm = arm + (wire - arm) * (1 - math.exp(-dt / .077)); c += dt
            TT.append(np.degrees(x))
        return np.array(TT)
    for label, a in (('all from the model', (1, 1, 1)), ('position only', (1, 0, 0)), ('speed only', (0, 1, 0)), ('last action only', (0, 0, 1)), ('position + speed', (1, 1, 0)), ('position + last action', (1, 0, 1))):
        TT = closed(*a); print(f'3 {label:24s} target RMS {np.sqrt(((TT - RT) ** 2).mean()):5.1f}  at 10 s J1-6 {np.round(TT[-1], 0)}  dog {np.round(RT[-1], 0)}')
