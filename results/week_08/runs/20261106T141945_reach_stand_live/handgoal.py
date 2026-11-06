import json, math, sys
import numpy as np
sys.path.insert(0, '.')
import d1_ik
from position_only.workspace import load_urdf
from mount import ARM_MOUNT_POS_M
from demos.combiner_key.robot import leg_policy
FOOT_RADIUS_M = 0.022  # press_layer.FOOT_RADIUS_M
rows = sys.argv[1]; P = sys.argv[2] if len(sys.argv) > 2 else 'checkpoints/reach_force_sticky_model_66600.npz'
times = [float(x) for x in sys.argv[3:]] or [0, .5, 1, 1.5, 3, 5, 8, 10]
meta = json.loads(str(np.load(P, allow_pickle=True)['meta']))
d = np.load(rows); t = d['t'] - d['t'][0]; joints = load_urdf()[0]; M = np.array(ARM_MOUNT_POS_M)
r, pit, yaw = meta['reach_goals']['init_start']
goal = np.array([r * math.cos(pit) * math.cos(yaw), r * math.cos(pit) * math.sin(yaw), 0.49 + r * math.sin(pit)])
def Ry(a): c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def Rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
LEGS = ('FR', 'FL', 'RR', 'RL'); SUPPORT = (1, 2, 3)
H, E, C = [], [], []
for i in range(len(t)):
    roll, pitch = d['rpy'][i, :2]; R = Ry(pitch) @ Rx(roll)
    z = [(R @ leg_policy.foot_local(l, d['q'][i, 3 * k:3 * k + 3], meta))[2] for k, l in enumerate(LEGS)]
    h = FOOT_RADIUS_M - float(np.mean([z[k] for k in SUPPORT]))
    p = d1_ik.tool_pose(joints, d['arm_obs'][i])[0]
    w = np.array([0, 0, h]) + R @ p   # tool_pose is already in the Go2 base frame: `forward` starts at MOUNT_B
    H.append(h); C.append(w); E.append(np.linalg.norm(w - goal))
H, E, C = np.array(H), np.array(E), np.array(C)
print('goal (yaw frame, from the ground)', np.round(goal, 3), f'rows {len(t)} to {t[-1]:.2f} s')
print('   t   base  pitch  roll |  hand x      y      z  | off x     y     z  | error cm')
for x in times:
    if x > t[-1] + 0.05: continue
    i = int(np.argmin(abs(t - x))); o = C[i] - goal
    print(f'{t[i]:5.2f} {H[i]:.3f} {math.degrees(d["rpy"][i,1]):6.1f} {math.degrees(d["rpy"][i,0]):5.1f} | {C[i,0]:6.3f} {C[i,1]:6.3f} {C[i,2]:6.3f} | {o[0]*100:5.1f} {o[1]*100:5.1f} {o[2]*100:5.1f} | {E[i]*100:5.1f}')
tail = t >= max(0.0, t[-1] - 3)
print(f'error over the last 3 s: mean {E[tail].mean()*100:.1f} cm, min {E[tail].min()*100:.1f}, max {E[tail].max()*100:.1f}; lowest in the run {E.min()*100:.1f} at {t[E.argmin()]:.2f} s')
