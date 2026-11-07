"""The replay's hand: the dog's frames (held to 10 s), the arm closed on each export as in semiclosed.py, the CAD tip against the goal
with the frames' own tilt and height. Real 66600, last 3 s: 11.1 cm (-6.9, +2.1, +8.5) and 10.4. Does it say so, unseen?"""
import glob, json, math, os, re, sys
import numpy as np
from multiprocessing import Pool
sys.path.insert(0, '.')
import d1_ik
from position_only.workspace import load_urdf
from mount import ARM_MOUNT_POS_M
from demos.combiner_key.robot import leg_policy, reach_arm
from demos.combiner_door.robot import arm_link
A = 'logs/unifp_train/armlink/'; dt = .02; N = 500; FOOT = 0.022
J = load_urdf()[0]; M = np.array(ARM_MOUNT_POS_M); LEGS = ('FR', 'FL', 'RR', 'RL')
def Ry(a): c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def Rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def run(P, F, lag=.077):
    pol = leg_policy.LegPolicy(P); m = pol.meta; o = leg_policy.LegObservation(m)
    lim = np.asarray(m['arm_joint_limits'], float); tg = reach_arm.ArmTargets(o.default[12:18], m['action_scale'], m['clip_actions'], lim[:, 0], lim[:, 1])
    fol = arm_link.Follower(np.zeros(6)); arm = np.zeros(6); wire = np.zeros(6); rep = reach_arm.ArmReports(np.zeros(6)); nxt = 0.0; c = 0.0; T, Q = [], []
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
def hand(d, Q, m):
    r, pit, yaw = m['reach_goals']['init_start']
    goal = np.array([r * math.cos(pit) * math.cos(yaw), r * math.cos(pit) * math.sin(yaw), 0.49 + r * math.sin(pit)]); W = []
    for k, q in enumerate(Q):
        i = min(k, len(d['t']) - 1); ro, pi = d['rpy'][i, :2]; R = Ry(pi) @ Rx(ro)
        z = [(R @ leg_policy.foot_local(l, d['q'][i, 3 * n:3 * n + 3], m))[2] for n, l in enumerate(LEGS)]
        W.append(np.array([0, 0, FOOT - float(np.mean(z[1:]))]) + R @ d1_ik.tool_pose(J, q)[0])  # tool_pose is in the Go2 base frame
    return np.array(W) - goal
def one(a):
    P, rows = a; d = dict(np.load(rows)); T, Q, m = run(P, d['frame']); O = hand(d, Q, m); e = np.linalg.norm(O, axis=1)
    return (os.path.basename(P).replace('.npz', ''), os.path.basename(rows), e[-150:].mean() * 100, *(O[-150:].mean(0) * 100), e[0] * 100, *np.degrees(Q[-1]), np.abs(np.degrees(T - Q))[50:].max())
if __name__ == '__main__':
    L = ['checkpoints/reach_force_sticky_model_66600.npz', 'checkpoints/reach_force_locked_model_66200.npz'] + sorted(glob.glob(A + 'reach_6[bc]_*.npz'), key=lambda p: (p.split('_')[-2], int(re.search(r'_(\d+)\.npz', p).group(1))))
    S = [A + 'policy_rows.npz', A + 'policy_rows_wood.npz']
    with Pool(8) as p: R = p.map(one, [(P, s) for P in L for s in S])
    print('the dog, 66600: 11.1 cm (-6.9 +2.1 +8.5) and 10.4 cm (-6.2 +2.1 +8.7), last 3 s; 14.4-14.7 at handover; J2 about 4-5')
    print(f"{'export':32s} {'frames':10s} last 3 s  off x     y     z  | start | report at 10 s J1..J6             | lead 1 s+")
    for r in R:
        print(f"{r[0]:32s} {r[1].replace('policy_rows','').replace('.npz','').strip('_') or 'carpet':10s} {r[2]:6.1f}  {r[3]:5.1f} {r[4]:5.1f} {r[5]:5.1f} | {r[6]:5.1f} | " + ' '.join(f'{x:5.0f}' for x in r[7:13]) + f' | {r[13]:4.0f}')
