import json, math, sys
import numpy as np
sys.path.insert(0, '.')
import d1_ik
from position_only.workspace import load_urdf
from mount import ARM_MOUNT_POS_M
from demos.combiner_key.robot import leg_policy, reach_arm
from demos.combiner_door.robot import arm_link
dt = .02; FOOT = 0.022; J = load_urdf()[0]; M = np.array(ARM_MOUNT_POS_M); LEGS = ('FR', 'FL', 'RR', 'RL')
d = dict(np.load(sys.argv[1])); P = sys.argv[2]; F = d['frame']; N = len(F); t = d['t'] - d['t'][0]
def Ry(a): c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def Rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
pol = leg_policy.LegPolicy(P); m = pol.meta; o = leg_policy.LegObservation(m)
lim = np.asarray(m['arm_joint_limits'], float); tg = reach_arm.ArmTargets(o.default[12:18], m['action_scale'], m['clip_actions'], lim[:, 0], lim[:, 1])
fol = arm_link.Follower(np.zeros(6)); arm = np.zeros(6); wire = np.zeros(6); rep = reach_arm.ArmReports(np.zeros(6)); nxt = 0.0; c = 0.0; T, Q = [], []
for k in range(N):
    f = np.array(F[k], float)
    if c >= nxt:
        rep.report(c, np.round(np.degrees(arm) / .1) * .1 * math.pi / 180); nxt += 1 / 9
        wire = np.ravel(d1_ik.from_servo_deg(np.array(fol.step(np.array(d1_ik.to_servo_deg(rep.q)), 0.0, 1 / 9))))
    f[17:23], f[35:41], f[41:59] = rep.q - o.default[12:18], rep.dq * m['obs_scale_dof_vel'], o.prev_actions
    s = o.step(f); a = pol.actor(np.concatenate((np.atleast_2d(s)[:, -pol.num_single_obs:], pol.encoder(np.atleast_2d(s))), -1))[0]; o.record_action(a)
    x = tg.target(o.prev_actions[12:18]); fol.goal = np.array(d1_ik.to_servo_deg(x)); arm = arm + (wire - arm) * (1 - math.exp(-dt / .077)); c += dt
    T.append(x.copy()); Q.append(rep.q.copy())
T, Q = np.degrees(np.array(T)), np.degrees(np.array(Q)); RT, RQ = np.degrees(d['arm_target'][:N]), np.degrees(d['arm_obs'][:N])
print('rows', N, 'to', round(float(t[-1]), 2), 's; recorded first frame arm', np.round(RQ[0], 1), 'target', np.round(RT[0], 1))
print('RMS target, replay vs dog, J1-6:', np.round(np.sqrt(((T - RT) ** 2).mean(0)), 1), 'all', round(float(np.sqrt(((T - RT) ** 2).mean())), 1))
print('RMS report, replay vs dog, J1-6:', np.round(np.sqrt(((Q - RQ) ** 2).mean(0)), 1), 'all', round(float(np.sqrt(((Q - RQ) ** 2).mean())), 1))
L, RL = np.abs(T - Q), np.abs(RT - RQ); b = t >= 1
print(f'lead from 1 s on: replay {L[b].max():.1f} (J{int(L[b].max(0).argmax())+1}) at {t[b][L[b].max(1).argmax()]:.2f} s, dog {RL[b].max():.1f} (J{int(RL[b].max(0).argmax())+1}) at {t[b][RL[b].max(1).argmax()]:.2f} s')
r, pit, yw = m['reach_goals']['init_start']; goal = np.array([r * math.cos(pit) * math.cos(yw), r * math.cos(pit) * math.sin(yw), .49 + r * math.sin(pit)])
def tip(i, q):
    ro, pi = d['rpy'][i, :2]; R = Ry(pi) @ Rx(ro)
    z = [(R @ leg_policy.foot_local(l, d['q'][i, 3 * n:3 * n + 3], m))[2] for n, l in enumerate(LEGS)]
    return np.array([0, 0, FOOT - float(np.mean(z[1:]))]) + R @ d1_ik.tool_pose(J, q)[0] - goal
print('    t | replay target J1-6             | dog target J1-6                | replay hand cm | dog hand cm')
for x in (.28, .58, .98, 1.5, 3, 3.5, 4.5, 6, 6.2, 6.4, 7, 8, 10):
    if x > t[-1] + .05: continue
    i = int(np.argmin(abs(t - x))); a1 = tip(i, np.radians(Q[i])); a2 = tip(i, np.radians(RQ[i]))
    print(f'{t[i]:5.2f} | ' + ' '.join(f'{v:4.0f}' for v in T[i]) + ' | ' + ' '.join(f'{v:4.0f}' for v in RT[i]) + f' | {np.linalg.norm(a1)*100:5.1f} | {np.linalg.norm(a2)*100:5.1f}')
