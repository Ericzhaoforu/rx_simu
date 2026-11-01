#!/usr/bin/env python3
"""Cold-start Task1 cases; keep source snapshots, failures and independent results."""
import argparse
import datetime
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--group',choices=('base','dynamic','all'),default='base')
    p.add_argument('--seed',type=int,choices=(1,2,3));p.add_argument('--changes',type=int,choices=(0,1,2))
    p.add_argument('--dynamic',choices=('none','cross','block'));p.add_argument('--record-maps',action='store_true')
    p.add_argument('--fault',choices=('observation_gap','reordered_observation','mission_exit','cancel','role_change','no_path'))
    p.add_argument('--stop-on-failure',action='store_true')
    p.add_argument('--no-video',action='store_true');a=p.parse_args()
    if (ROOT/'runs/session.json').exists():raise RuntimeError('Stop the current managed session before running a cold-start suite')
    cases=[]
    if a.group in ('base','all'):
        cases += [dict(seed=s,changes=c,dynamic='none') for s in (1,2,3) for c in (0,1,2)]
    if a.group in ('dynamic','all'):
        cases += [dict(seed=s,changes=c,dynamic=d) for s in (1,2,3) for c,d in ((0,'cross'),(0,'block'),(2,'cross'))]
    cases=[c for c in cases if all(getattr(a,k) is None or c[k]==getattr(a,k) for k in ('seed','changes','dynamic'))]
    if a.fault:cases=[dict(seed=1,changes=0,dynamic='none',fault=a.fault)]
    if not cases:raise ValueError('No matching cases')
    ledger=ROOT/'runs'/('task1_suite_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
    results=[]
    for case in cases:
        name=f"task1_s{case['seed']}_c{case['changes']}_{case['dynamic']}"
        if a.fault:name+='_'+a.fault
        command=[sys.executable,str(ROOT/'scripts/sim_session.py'),'start','--task1','--navigation','--control',
                 '--name',name,'--seed',str(case['seed']),'--changes',str(case['changes']),'--dynamic',case['dynamic']]
        if a.fault=='no_path':command.append('--task1-blocked')
        old=(ROOT/'runs/latest.txt').read_text().strip() if (ROOT/'runs/latest.txt').exists() else None
        supervisor=subprocess.Popen(command,stdout=(ROOT/'log'/f'{name}_supervisor.log').open('w'),stderr=subprocess.STDOUT)
        recorder=None;run=None;item={'case':case}
        try:
            deadline=time.monotonic()+90
            while time.monotonic()<deadline:
                current=(ROOT/'runs/latest.txt').read_text().strip() if (ROOT/'runs/latest.txt').exists() else None
                if current and current!=old:run=Path(current);break
                if supervisor.poll() is not None:raise RuntimeError('Simulation supervisor exited during startup')
                time.sleep(.2)
            if run is None:raise TimeoutError('Simulation startup')
            item['run_dir']=str(run);print(json.dumps(item),flush=True)
            if a.record_maps:
                recorder=subprocess.Popen([sys.executable,str(ROOT/'scripts/record_navigation.py'),'--run-dir',str(run)],
                                           stdout=(run/'record_navigation.log').open('w'),stderr=subprocess.STDOUT)
            command=[sys.executable,str(ROOT/'scripts/validate_task1.py')]
            if a.fault:command=[sys.executable,str(ROOT/'scripts/validate_task1_fault.py'),a.fault]
            elif a.no_video:command.append('--no-video')
            result=subprocess.run(command,stdout=(run/'test_console.log').open('w'),stderr=subprocess.STDOUT,timeout=2500)
            item['exit_code']=result.returncode
        except Exception as error:item['error']=repr(error)
        finally:
            if recorder:
                recorder.send_signal(signal.SIGINT)
                try:recorder.wait(timeout=10)
                except subprocess.TimeoutExpired:recorder.terminate();recorder.wait(timeout=5)
            subprocess.run([sys.executable,str(ROOT/'scripts/sim_session.py'),'stop'],check=False)
            try:supervisor.wait(timeout=12)
            except subprocess.TimeoutExpired:supervisor.terminate();supervisor.wait(timeout=5)
        if run and (run/'task1_judge.jsonl').exists():
            from robotx_task1.judge import evaluate
            try:item['result']=evaluate(run)
            except Exception as error:item['judge_error']=repr(error)
        if a.fault and run and (run/'task1_fault_result.json').exists():
            item['fault_result']=json.loads((run/'task1_fault_result.json').read_text())
        if run and case['dynamic']!='none' and a.record_maps:
            from analyze_task1_maps import analyze
            try:item['map_result']={k:v for k,v in analyze(run).items() if k!='samples'}
            except Exception as error:item['map_result']={'passed':False,'error':repr(error)}
        results.append(item);ledger.write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(item),flush=True)
        if a.stop_on_failure and (not item.get('fault_result' if a.fault else 'result',{}).get('passed',False)
                                 or not item.get('map_result',{'passed':True})['passed']):break
    print('Ledger:',ledger,flush=True)
    if not all(r.get('fault_result' if a.fault else 'result',{}).get('passed',False) and
               r.get('map_result',{'passed':True})['passed'] for r in results):raise SystemExit(1)


if __name__=='__main__':main()
