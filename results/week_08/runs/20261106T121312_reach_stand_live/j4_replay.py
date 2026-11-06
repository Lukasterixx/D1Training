import numpy as np
from demos.combiner_key.robot import leg_policy
P = 'checkpoints/reach_force_locked_model_66200.npz'
pol = leg_policy.LegPolicy(P); m = pol.meta
d = np.load('results/week_08/runs/20261106T121312_reach_stand_live/policy_rows.npz')
F, A, t = d['frame'], d['arm_target'], d['t'] - d['t'][0]
print('frame shape', F.shape, 'num_single_obs', pol.num_single_obs)
S = dict(rp=slice(0,2), gyro=slice(2,5), legq=slice(5,17), armq=slice(17,23), legdq=slice(23,35), armdq=slice(35,41), legprev=slice(41,53), armprev=slice(53,59))
settled = np.asarray(m['settled_leg_pos'], float) - np.asarray(m['default_dof_pos'], float)[:12]
def replay(edit=None):
    o = leg_policy.LegObservation(m); out = []
    for k in range(len(F)):
        f = np.array(F[k], float)
        if edit: edit(f, k)
        s = o.step(f)
        a = pol.actor(np.concatenate((np.atleast_2d(s)[:, -pol.num_single_obs:], pol.encoder(np.atleast_2d(s))), -1))[0]
        o.record_action(a); out.append(np.degrees(o.prev_actions[12:18] * m['action_scale'] + o.default[12:18]))
    return np.array(out)
R = replay(); print('reproduces arm_target to', np.abs(R - np.degrees(A)).max(), 'deg')
idx = [np.argmin(abs(t - x)) for x in (.56, .76, .96, 1.16, 1.36, 1.54)]
print('real          J4', np.round(R[idx, 3]))
def show(name, e):
    r = replay(e); print(f'{name:14s}J4', np.round(r[idx, 3]), '| end', np.round(r[-1], 0))
show('legs settled', lambda f, k: f.__setitem__(S['legq'], settled))
show('legs dq 0', lambda f, k: f.__setitem__(S['legdq'], 0.0))
show('gyro 0', lambda f, k: f.__setitem__(S['gyro'], 0.0))
show('rp level', lambda f, k: f.__setitem__(S['rp'], 0.0))
show('arm dq 0', lambda f, k: f.__setitem__(S['armdq'], 0.0))
show('leg prev 0', lambda f, k: f.__setitem__(S['legprev'], 0.0))
show('arm prev 0', lambda f, k: f.__setitem__(S['armprev'], 0.0))
show('arm q 0', lambda f, k: f.__setitem__(S['armq'], 0.0))
def both(f,k):
    f[S['armq']]=0.0; f[S['armprev']]=0.0
show('arm q+prev 0', both)
def still(f,k):
    f[S['legq']]=settled; f[S['legdq']]=0.0; f[S['legprev']]=0.0
show('legs all sim', still)
def frozen(f,k):
    f[S['armq']]=F[0][S['armq']]; f[S['armdq']]=0.0
show('arm q frozen', frozen)
def cmd0(f,k): f[61:76]=F[0][61:76]
show('cmds as row 0', cmd0)
def c64(f,k): f[64]=0.0
def c65(f,k): f[65]=0.0
show('col 64 = 0', c64); show('col 65 = 0', c65)
def cmdarm(f,k):
    f[61:76]=F[0][61:76]; f[S['armq']]=0.0; f[S['armprev']]=0.0
show('cmds+arm', cmdarm)
print('cols 64,65 at idx', np.round(F[idx][:,64:66],3).tolist(), 'first nonzero row', int(np.argmax(np.abs(F[:,61:76]).max(1)>0)), 'cols ever nonzero', (np.where(np.abs(F[:,61:76]).max(0)>0)[0]+61).tolist())
print('frame row 40 rp, gyro', np.round(F[40][:5], 3)); print('leg q - default at start', np.round(F[0][5:17], 3)); print('at end', np.round(F[-1][5:17], 3))
