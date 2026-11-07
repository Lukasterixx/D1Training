"""The sport-mode stand's 20 s, real legs and body, the arm closed on the candidate: does J4 stay bounded for a whole run? A second stance, not a second handover."""
import sys, os, math
import numpy as np
sys.path.insert(0, '.')
import d1_ik
from demos.combiner_key.robot import leg_policy, reach_arm
from demos.combiner_door.robot import arm_link
F = np.load(sys.argv[1])['frame']; dt = .02
def run(P, lag=.077):
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
for P in sys.argv[2:]:
    T, Q = run(P); l = np.abs(T - Q); n = os.path.basename(P).replace('.npz', '')
    print(f"{n:32s} lead past 1 s {l[50:].max():3.0f} (J{int(l[50:].max(0).argmax())+1}) | past 5 s {l[250:].max():3.0f} | J4 at 1.5 s {T[75,3]:4.0f}, 5 s {T[250,3]:4.0f}, 20 s {T[-1,3]:4.0f} | at 20 s {np.round(T[-1],0)} | last 5 s moved {np.round(np.ptp(T[-250:],axis=0).max(),1)} deg")
