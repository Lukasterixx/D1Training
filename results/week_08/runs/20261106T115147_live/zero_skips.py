import json, sys
import numpy as np
from pathlib import Path
sys.path.insert(0,'.')
from demos.combiner_door.robot import arm_gap_phase as g
R=Path('logs/holster_dog/20261106T115147_live')
rows=[json.loads(l) for l in open(R/'arm_link.jsonl')]
cm=[r for r in rows if r['kind']=='command']; tc=np.array([r['t'] for r in cm]); U=np.array([r['command'] for r in cm])
samp=[json.loads(l) for l in open(R/'feedback_timing/samples.jsonl')]; samp=[r for r in samp if r['topic']=='current_servo_angle' and r.get('src') is not None]
tr=g.reports_on_dog_clock(R/'feedback_timing/samples.jsonl'); Q=np.array([r['q'][:6] for r in samp]); t0=rows[0]['t']
steps=np.abs(np.diff(U,axis=0)).max(1); moving=(np.diff(tc)<0.2)
iv=np.diff(tr)
idx=np.where((iv>0.16)&(tr[:-1]>=tc[0])&(tr[:-1]<=tc[-1]))[0]
for i in idx:
    k=np.searchsorted(tc,tr[i]+0.02)   # command about the skipped cycle
    j=int(np.abs(Q[i+1]-Q[i]).argmax())
    print('skip t=%.2f step before cmds %s | joint J%d q before %s across-gap %+.1f after %s'%(tr[i]-t0,np.round(steps[max(0,k-4):k+2],2).tolist(),j+1,np.round(Q[i-2:i+1,j]-Q[i-3:i,j],1).tolist(),Q[i+1,j]-Q[i,j],np.round(Q[i+2:i+4,j]-Q[i+1:i+3,j],1).tolist()))
# step sizes generally
fol=[r['t'] for r in rows if r['kind']=='follow'][0]; hold=[r['t'] for r in rows if r['kind']=='hold' and r['t']>fol][0]
kf=(tc[:-1]>=fol)&(tc[:-1]<=hold)
s=steps[kf]
print('follow steps <2.5:',int((s<2.5).sum()),'of',len(s),'; big(>4.5) at seconds',np.round(tc[:-1][kf][s>4.5]-t0,1).tolist())
# are big steps at a report skip?
skt=[tr[i]-t0 for i in idx]; print('skip seconds',np.round(skt,2).tolist())
# did the arm pause: per-report speed of the fastest joint through follow, reports gap-normal vs skip
sp=np.abs(np.diff(Q,axis=0)).max(1)/np.maximum(iv,1e-3)
k=(tr[:-1]>=fol+2)&(tr[:-1]<=hold-2)
print('follow speed deg/s fastest joint p50 %.1f; across skips %s'%(np.median(sp[k]),np.round(sp[idx],1).tolist()))
# repeated identical reports (arm not moving) during follow
same=(np.abs(np.diff(Q,axis=0)).max(1)<0.05)&k
print('follow reports with no movement at all:',int(same.sum()),'of',int(k.sum()))
# previous locked run comparison available?
