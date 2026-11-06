import math, sys
import numpy as np
sys.path.insert(0, '.')
import d1_ik
from demos.combiner_key.robot import leg_policy, reach_arm
from demos.combiner_door.robot import arm_link
dt=.02
d=dict(np.load(sys.argv[1])); P=sys.argv[2]; F=d['frame']; N=len(F); t=d['t']-d['t'][0]
pol=leg_policy.LegPolicy(P); m=pol.meta
RT,RQ=np.degrees(d['arm_target'][:N]),np.degrees(d['arm_obs'][:N])
def run(t0):
    o=leg_policy.LegObservation(m); lim=np.asarray(m['arm_joint_limits'],float)
    tg=reach_arm.ArmTargets(o.default[12:18],m['action_scale'],m['clip_actions'],lim[:,0],lim[:,1])
    fol=arm=wire=rep=None; nxt=0.0; c=0.0; T=[]
    for k in range(N):
        f=np.array(F[k],float)
        if t[k]>=t0:
            if rep is None:
                q0=d['arm_obs'][k].astype(float); arm=q0.copy(); wire=q0.copy(); rep=reach_arm.ArmReports(q0.copy())
                fol=arm_link.Follower(np.array(d1_ik.to_servo_deg(q0))); nxt=c
            if c>=nxt:
                rep.report(c,np.round(np.degrees(arm)/.1)*.1*math.pi/180); nxt+=1/9
                wire=np.ravel(d1_ik.from_servo_deg(np.array(fol.step(np.array(d1_ik.to_servo_deg(rep.q)),0.0,1/9))))
            f[17:23],f[35:41],f[41:59]=rep.q-o.default[12:18],rep.dq*m['obs_scale_dof_vel'],o.prev_actions
        s=o.step(f); a=pol.actor(np.concatenate((np.atleast_2d(s)[:,-pol.num_single_obs:],pol.encoder(np.atleast_2d(s))),-1))[0]; o.record_action(a)
        x=tg.target(o.prev_actions[12:18])
        if rep is not None:
            fol.goal=np.array(d1_ik.to_servo_deg(x)); arm=arm+(wire-arm)*(1-math.exp(-dt/.077))
        c+=dt; T.append(x.copy())
    return np.degrees(np.array(T))
T=run(0.0)
e=np.abs(T-RT)
for th in (3,5,10): print(f'first time any joint target parts by {th} deg:',round(float(t[np.argmax(e.max(1)>th)]),2),'s, J',int(e[np.argmax(e.max(1)>th)].argmax())+1)
print('J2 replay/dog at .5 1 1.5 2 2.5 3:',[ (int(round(T[np.argmin(abs(t-x)),1])),int(round(RT[np.argmin(abs(t-x)),1]))) for x in (.5,1,1.5,2,2.5,3)])
print('J5 replay/dog:',[ (int(round(T[np.argmin(abs(t-x)),4])),int(round(RT[np.argmin(abs(t-x)),4]))) for x in (.5,1,1.5,2,2.5,3)])
print('switch to closed loop at t0 | target at 10 s J1-6 | dog at 10 s | J2 at t0+1, +3')
for t0 in (1,2,3,5,8):
    T=run(t0); i=-1; a=np.argmin(abs(t-(t0+1))); b=np.argmin(abs(t-min(t0+3,10)))
    print(f't0 {t0:2d} | '+' '.join(f'{v:4.0f}' for v in T[i])+' | '+' '.join(f'{v:4.0f}' for v in RT[i])+f' | {T[a,1]:.0f} {T[b,1]:.0f}; dog {RT[a,1]:.0f} {RT[b,1]:.0f}')
