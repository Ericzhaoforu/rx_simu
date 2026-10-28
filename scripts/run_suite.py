#!/usr/bin/env python3
"""Run isolated, real-time integration cases with retained failures and logs."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
CASES=[
    *[{'name':f'single_{i}','goals':[[5,0,0]]} for i in range(1,4)],
    *[{'name':f'multi_{i}','goals':[[3,-3,-.6],[6,-4,0]]} for i in range(1,4)],
    {'name':'static_obstacle','goals':[[18,0,0]]},
    {'name':'cancel','goals':[[5,0,0]],'cancel_after':5,'expected_status':5},
    {'name':'unreachable','goals':[[60,0,0]],'expected_status':6},
    {'name':'wind','goals':[[5,0,0]],'wind':3},
    {'name':'waves','goals':[[5,0,0]],'wave_gain':.3},
    {'name':'wind_waves','goals':[[5,0,0]],'wind':3,'wave_gain':.3},
    {'name':'gps_input_fixed','goals':[[5,0,0]],'gps_input':True},
]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--cases',nargs='*');args=parser.parse_args()
    ledger=ROOT/'runs/suite_results.json'
    results=json.loads(ledger.read_text()) if ledger.exists() else []
    for case in CASES:
        if args.cases and case['name'] not in args.cases:continue
        name=case['name'];print('START',name,flush=True)
        command=[sys.executable,str(ROOT/'scripts/sim_session.py'),'start','--name',name,'--navigation','--control']
        for key,flag in [('wind','--wind'),('wave_gain','--wave-gain')]:
            if key in case:command.extend([flag,str(case[key])])
        if case.get('gps_input'):command.append('--gps-input')
        console=ROOT/'runs'/f'{name}_session_console.log'
        latest=ROOT/'runs/latest.txt'
        previous=latest.read_text() if latest.exists() else ''
        process=subprocess.Popen(command,stdout=console.open('w'),stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+30;run=None
            while time.monotonic()<deadline:
                if process.poll() is not None:raise RuntimeError(console.read_text())
                latest=ROOT/'runs/latest.txt'
                if latest.exists():
                    candidate=Path(latest.read_text().strip())
                    if latest.read_text()!=previous and candidate.name.endswith('_'+name):run=candidate;break
                time.sleep(.2)
            if run is None:raise TimeoutError('Session startup timeout')
            # Let the supervisor attach ROS nodes; validate also waits for fresh data and lifecycle state.
            command=[sys.executable,str(ROOT/'scripts/validate.py'),'goal','--goals',json.dumps(case['goals']),'--label',name]
            if case.get('cancel_after'):command.extend(['--cancel-after',str(case['cancel_after'])])
            command.extend(['--expect-status',str(case.get('expected_status',4))])
            outcome=subprocess.run(command,stdout=(run/'test_console.log').open('w'),stderr=subprocess.STDOUT,timeout=420)
            report=run/(name+'_result.json')
            item={'case':case,'run_dir':str(run),'exit_code':outcome.returncode,
                  'result':json.loads(report.read_text()) if report.exists() else None}
            results.append(item);print(json.dumps(item),flush=True)
            ledger.write_text(json.dumps(results,indent=2)+'\n')
        except Exception as error:
            item={'case':case,'error':str(error),'run_dir':str(run) if run else None}
            results.append(item);ledger.write_text(json.dumps(results,indent=2)+'\n')
            print(json.dumps(item),flush=True)
        finally:
            subprocess.run([sys.executable,str(ROOT/'scripts/sim_session.py'),'stop'],check=False)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=5)
    print('FINISHED',len(results),'cases',flush=True)


if __name__=='__main__':main()
