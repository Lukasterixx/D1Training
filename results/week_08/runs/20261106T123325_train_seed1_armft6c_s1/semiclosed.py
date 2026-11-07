"""The dog's real legs and body from its frames; the arm closed on the candidate through the real Follower, 9 Hz reports, the 77 ms lag.
Run on 66200 it has to give back the real trace, or it proves nothing."""
import sys, os, math
import numpy as np
sys.path.insert(0, '.')
import d1_ik
from demos.combiner_key.robot import leg_policy, reach_arm
from demos.combiner_door.robot import arm_link
d = np.load(sys.argv[1]); F, t = d['frame'], d['t'] - d['t'][0]; dt = 0.02
idx = [int(np.argmin(abs(t - x))) for x in (.56, .76, .96, 1.16, 1.36, 1.54)]
def run(P, extend=0, lag=0.077):
    pol = leg_policy.LegPolicy(P); m = pol.meta; o = leg_policy.LegObservation(m)
    lim = np.asarray(m['arm_joint_limits'], float); tg = reach_arm.ArmTargets(o.default[12:18], m['action_scale'], m['clip_actions'], lim[:, 0], lim[:, 1])
    fol = arm_link.Follower(np.zeros(6)); arm = np.zeros(6); wire = np.zeros(6); rep = reach_arm.ArmReports(np.zeros(6)); nxt = 0.0; clock = 0.0
    T, Q = [], []
    for k in range(len(F) + extend):
        f = np.array(F[min(k, len(F) - 1)], float)
        if clock >= nxt:
            rep.report(clock, np.round(np.degrees(arm) / 0.1) * 0.1 * math.pi / 180); nxt += 1 / 9
            wire = np.ravel(d1_ik.from_servo_deg(np.array(fol.step(np.array(d1_ik.to_servo_deg(rep.q)), 0.0, 1 / 9))))
        f[17:23], f[35:41] = rep.q - o.default[12:18], rep.dq * m['obs_scale_dof_vel']
        f[41:59] = o.prev_actions
        s = o.step(f)
        a = pol.actor(np.concatenate((np.atleast_2d(s)[:, -pol.num_single_obs:], pol.encoder(np.atleast_2d(s))), -1))[0]
        o.record_action(a)
        target = tg.target(o.prev_actions[12:18]); fol.goal = np.array(d1_ik.to_servo_deg(target))
        arm = arm + (wire - arm) * (1 - math.exp(-dt / lag)); clock += dt
        T.append(np.degrees(target)); Q.append(np.degrees(rep.q))
    return np.array(T), np.array(Q)
print('the dog (66200)                  J4  -30  -58  -69  -78  -85  -89 | rep -13 -21 -26 -35 -45 -55 | lead 33-43')
for P in sys.argv[2:]:
    n = os.path.basename(P).replace('.npz', ''); T, Q = run(P); lead = np.abs(T - Q)
    Te, Qe = run(P, extend=150); le = np.abs(Te - Qe)
    print(f"{n:32s} J4 " + " ".join(f"{T[i,3]:4.0f}" for i in idx) + " | rep " + " ".join(f"{Q[i,3]:3.0f}" for i in idx) + f" | lead 1 s+ {lead[50:].max():3.0f} (J{int(lead[50:].max(0).argmax())+1}); 4.5 s on, last frame held: J4 {Te[-1,3]:.0f}, lead {le[100:].max():.0f}")
