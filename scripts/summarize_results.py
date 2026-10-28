#!/usr/bin/env python3
"""Build a compact, reproducible ledger from retained raw integration evidence."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT=Path(__file__).resolve().parents[1]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def metrics(path):
    rows=[r for line in path.read_text().splitlines() if 'truth' in (r:=json.loads(line))]
    active=[r for r in rows if r['phase']=='navigation']
    if not active:return {}
    pairs=[r for r in active if 'ekf' in r]
    error=[math.dist(r['truth'][:2],r['ekf'][:2]) for r in pairs]
    report={'samples':len(active),'ekf_position_rmse_m':float(np.sqrt(np.mean(np.square(error)))) if error else None,
            'peak_true_speed_mps':max(math.hypot(*r['truth'][4:6]) for r in active),
            'real_time_factor':(active[-1]['sim']-active[0]['sim'])/(active[-1]['wall']-active[0]['wall']) if len(active)>1 else None,
            'true_heave_peak_to_peak_m':float(np.ptp([r['truth'][2] for r in active])),
            'max_scan_source_age_s':max(r.get('scan_age',0) for r in active)}
    if 'truth_rpy' in active[-1]:
        report['true_roll_pitch_peak_to_peak_rad']=np.ptp([r['truth_rpy'][:2] for r in active if 'truth_rpy' in r],axis=0).tolist()
    after=[r for r in rows if r['phase']=='post_goal']
    if after:
        start=after[0]['wall']
        def stable(predicate):
            beginning=None
            for row in after:
                if predicate(row):
                    if beginning is None:beginning=row['wall']
                    if row['wall']-beginning>=.5:return beginning-start
                else:beginning=None
            return None
        report['post_goal']={'stable_exact_zero_thrust_delay_s':stable(lambda r:abs(r.get('left_n',999))+abs(r.get('right_n',999))<1e-6),
                             'stable_speed_below_002_delay_s':stable(lambda r:math.hypot(*r['truth'][4:6])<.02 and abs(r['truth'][6])<.02),
                             'final_thrust_n':[after[-1].get('left_n'),after[-1].get('right_n')],
                             'travel_distance_m':sum(math.dist(a['truth'][:2],b['truth'][:2]) for a,b in zip(after,after[1:]))}
    return report


def plot(item):
    run=Path(item['run_dir']);name=item['case']['name'];trace=run/(name+'.jsonl')
    rows=[r for l in trace.read_text().splitlines() if 'truth' in (r:=json.loads(l)) and r['phase'] in ('navigation','post_goal')]
    if not rows:return
    a=np.array([r['truth'] for r in rows]);e=np.array([r.get('ekf',[np.nan]*7) for r in rows]);t=np.array([r['wall'] for r in rows]);t-=t[0]
    scene=json.loads((run/'generated/scene.json').read_text())
    fig,ax=plt.subplots(1,2,figsize=(11,4.5),layout='constrained')
    ax[0].plot(a[:,0],a[:,1],label='VRX truth');ax[0].plot(e[:,0],e[:,1],'--',label='FCU EKF',alpha=.8)
    for obj in scene['obstacles']:
        sx,sy=obj['size'];ax[0].add_patch(Rectangle((obj['x']-sx/2,obj['y']-sy/2),sx,sy,color='tab:red',alpha=.5))
    for x,y,_ in item['case'].get('goals',[]):ax[0].plot(x,y,'kx')
    ax[0].set(xlabel='East (m)',ylabel='North (m)',title=name,aspect='equal')
    ax[0].set_xlim(a[:,0].min()-4,a[:,0].max()+4);ax[0].set_ylim(a[:,1].min()-4,a[:,1].max()+4)
    ax[0].legend();ax[0].grid(alpha=.3)
    ax[1].plot(t,np.linalg.norm(a[:,4:6],axis=1),label='Actual planar speed')
    ax[1].plot(t,[r.get('preview',[0,0])[0] for r in rows],label='Command speed')
    ax[1].set(xlabel='Wall time since action start (s)',ylabel='Speed (m/s)');ax[1].grid(alpha=.3);ax[1].legend()
    fig.savefig(run/'trajectory.png',dpi=150);plt.close(fig)


def main():
    ledger=json.loads((ROOT/'runs/suite_results.json').read_text());latest={}
    for item in ledger:latest[item['case']['name']]=item
    output={'attempts':ledger,'latest':{}}
    for name,item in latest.items():
        run=Path(item['run_dir']);entry=dict(item);path=run/(name+'.jsonl')
        if path.exists():entry['metrics']=metrics(path)
        entry['evidence_sha256']={p.name:sha(p) for p in [path,run/(name+'_result.json'),run/'generated/usv_nav.sdf',run/'generated/nav2_wamv.yaml'] if p.exists()}
        if path.exists() and 'goals' in item['case']:plot(item)
        output['latest'][name]=entry
    for name in ['single_1','single_2','single_3','multi_1','multi_2','multi_3','static_obstacle','wind','waves','wind_waves']:
        if name in latest:
            result=latest[name].get('result') or {}
            print(name, result.get('passed'),result.get('true_position_error_m'),result.get('true_yaw_error_rad'))
    (ROOT/'docs/results.json').write_text(json.dumps(output,indent=2)+'\n')


if __name__=='__main__':main()
