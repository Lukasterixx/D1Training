import json, sys, importlib.util
import numpy as np
from pathlib import Path
sys.path.insert(0,'.')
spec=importlib.util.spec_from_file_location('rm','results/week_08/runs/20261106T023651_live_lock/replay_models.py'); rm=importlib.util.module_from_spec(spec); spec.loader.exec_module(rm)
from demos.combiner_door.robot import arm_gap_phase as g
R=Path('logs/holster_dog/20261106T115147_live')
rows=[json.loads(l) for l in open(R/'arm_link.jsonl')]
cm=[r for r in rows if r['kind']=='command']
tc=np.array([r['t'] for r in cm]); U=np.array([r['command'] for r in cm])
samp=[json.loads(l) for l in open(R/'feedback_timing/samples.jsonl')]
samp=[r for r in samp if r['topic']=='current_servo_angle' and r.get('src') is not None]
tr=g.reports_on_dog_clock(R/'feedback_timing/samples.jsonl'); Q=np.array([r['q'][:6] for r in samp])
same=(np.abs(np.diff(U,axis=0)).max(1)<1e-6)&(np.diff(tc)<0.2)
fol=[r['t'] for r in rows if r['kind']=='follow'][0]; hold=[r['t'] for r in rows if r['kind']=='hold' and r['t']>fol][0]
kf=(tc>=fol)&(tc<=hold)
print('follow commands',kf.sum(),'in',round(hold-fol,1),'s; repeated (no change on any joint) in follow',int(same[kf[1:]].sum()),'; repeated anywhere moving',int(same.sum()))
step=np.abs(np.diff(U[kf],axis=0)); print('follow max step per cmd deg',step.max(0).round(2),'as deg/s at 111ms',(step.max(0)/0.111).round(1))
caps=np.degrees([1.25,1.29,1.23,1.21,1.25,1.25])
keep=(tr>=tc[0])&(tr<=tc[-1]+0.5)
def fit(tau,delay,mask=None):
    out=[]
    for j in range(6):
        x=rm.series(tc,U[:,j],tr[keep],tau,delay,caps[j],Q[keep][0,j])
        e=x-Q[keep][:,j]
        if mask is not None: e=e[mask]
        out.append(e)
    return np.array(out)
trk=tr[keep]
segs={'whole run':None,'circle':(trk>=fol+1)&(trk<=hold)}
for n,m in segs.items():
    e=fit(0.077,0.005,m); print(n,'F-245 (77 ms, 5 ms): rms per joint',np.sqrt((e**2).mean(1)).round(2),'all %.2f'%np.sqrt((e**2).mean()),'max',np.abs(e).max(1).round(1))
m=segs['circle']
best=[]
for j in range(6):
    b=min(((np.sqrt(((rm.series(tc,U[:,j],trk,tau,0.005,caps[j],Q[keep][0,j])-Q[keep][:,j])[m]**2).mean()),tau) for tau in np.arange(0.03,0.15,0.005)))
    best.append((round(float(b[1])*1000),round(float(b[0]),2)))
print('best lag per joint on circle ms, rms',best)
# tracking vs command on circle
print('reports in circle',int(m.sum()))

# skips: when
cy=json.load(open(sys.argv[1]));rowsC=list(cy.values())[0]
mvt=[(r['kind'],r['t']) for r in rows if r['kind'] in('path','follow','hold')]
for r in rowsC:
    if r['skipped']:
        prev=[k for k,t0 in mvt if t0<=r['t']]
        near=tc[(tc>r['t']-0.3)&(tc<r['t']+0.3)]
        print('skip at',round(r['t']-rows[0]['t'],2),'s; last link event',prev[-1] if prev else None,'; interval',round(r['interval_s'],3),'; cmds ms after report',[round(x) for x in r['commands_ms']],'; cmds within 0.3 s',len(near))
print('reports in window cmd phase: median ms after report',np.median([x for r in rowsC for x in r['commands_ms']]).round(1))
st=np.abs(np.diff(U[kf],axis=0)).max(1); print('follow step per cmd: p50 %.2f p90 %.2f max %.2f; at cap(>4.5): %d of %d; first 12:'%(np.median(st),np.percentile(st,90),st.max(),(st>4.5).sum(),len(st)),st[:12].round(2))
gaps=np.diff(tc[kf]); print('follow cmd gaps p50 %.3f max %.3f, gaps>0.15: %d'%(np.median(gaps),gaps.max(),(gaps>0.15).sum()))
