#!/usr/bin/env python3
"""Summarize retained attempts without replacing failed evidence or inventing passes."""
import hashlib
import json
import tarfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    attempts=[];latest={};models=[];lidars=[]
    for run in sorted((ROOT/'runs').iterdir()):
        if not run.is_dir():continue
        for filename,collection in [('model_validation.json',models),('lidar_validation.json',lidars)]:
            p=run/filename
            if p.exists():
                j=json.loads(p.read_text());collection.append(dict(run=str(run.relative_to(ROOT)),passed=j.get('passed'),sha256=digest(p)))
        source=run/'generated/task1_scenario.json';result=run/'task1_result.json'
        if not source.exists() or not result.exists():continue
        scenario=json.loads(source.read_text());j=json.loads(result.read_text());fault=run/'task1_fault_result.json'
        key=f"s{scenario['seed']}_c{len(scenario['changes'])}_{scenario['dynamic']}"
        verdict=j.get('passed',False);f=None
        map_result=None
        if scenario['dynamic']!='none' and not fault.exists():
            map_file=run/'dynamic_map_result.json'
            if map_file.exists():map_result=json.loads(map_file.read_text())
            else:
                from analyze_task1_maps import analyze
                map_result=analyze(run)
            verdict=verdict and map_result.get('passed',False)
        if fault.exists():
            f=json.loads(fault.read_text());key='fault_'+f['fault'];verdict=f.get('passed',False)
        item=dict(case=key,run=str(run.relative_to(ROOT)),passed=verdict,normal_task_passed=j.get('passed',False),
                  endpoint=j.get('endpoint_error'),clearance_m=j.get('min_clearance_m'),collisions=j.get('collision_contacts'),
                  circles=j.get('circles'),stops=j.get('stops'),fault=f,
                  semantic_timing=j.get('semantic_timing'),drift_m=j.get('post_completion_drift_m'),
                  dynamic_map={k:v for k,v in map_result.items() if k!='samples'} if map_result else None,
                  evidence_sha256={p.name:digest(p) for p in [result,source,run/'provenance.json'] if p.exists()})
        if fault.exists():item['evidence_sha256'][fault.name]=digest(fault)
        mission_log=run/'task1_mission.jsonl'
        if mission_log.exists():
            events=[json.loads(l) for l in mission_log.read_text().splitlines()]
            planning=[]
            for i,e in enumerate(events):
                if e['event']!='planning':continue
                outcome=next((x for x in events[i+1:] if x['generation']==e['generation'] and
                              x['event'] in ('path_ready','plan_failed','stop_requested','aborted')),None)
                if outcome:planning.append(dict(outcome=outcome['event'],sim_s=outcome['sim']-e['sim'],
                                                wall_s=outcome['wall']-e['wall']))
            item['execution_metrics']={
                'planning_attempts':sum(e['event']=='planning' for e in events),
                'planning_failures':sum(e['event']=='plan_failed' for e in events),
                'controller_failures':sum(e['event']=='controller_result' and e['status'] not in (4,5) for e in events),
                'reported_semantic_changes':sum(e['event']=='semantic_update_reported' for e in events),
                'planning_duration_samples':planning,
                'total_planning_wall_s':sum(p['wall_s'] for p in planning),
                'max_planning_wall_s':max((p['wall_s'] for p in planning),default=None),
                'semantic_pause_sim_s':[s['resume_sim']-s['start_sim'] for s in j.get('stops',[])
                                        if s['reason']=='semantic revision changed' and s['resume_sim'] is not None]}
        snapshot=run/'source_snapshot.tar.gz'
        if snapshot.exists():
            with tarfile.open(snapshot) as archive:
                item['runtime_source_sha256']={name:hashlib.sha256(archive.extractfile(name).read()).hexdigest() for name in
                    ['src/robotx_task1/robotx_task1/mission.py','src/robotx_task1/robotx_task1/core.py',
                     'src/robotx_task1/robotx_task1/observer.py','src/usv_nav2/usv_nav2/rover_bridge.py',
                     'models/robotx_2026_robobuoy/model.sdf'] if name in archive.getnames()}
        attempts.append(item);latest[key]=item
    expected=[f's{s}_c{c}_{d}' for s in (1,2,3) for c,d in [(0,'none'),(1,'none'),(2,'none'),(0,'cross'),(0,'block'),(2,'cross')]]
    expected+=['fault_'+f for f in ('observation_gap','reordered_observation','mission_exit','cancel','role_change','no_path','late_change')]
    summary={'expected':expected,'latest':latest,'attempts':attempts,'model_checks':models,'lidar_checks':lidars,
             'scope':'Boat-side simulation; known UAV observation and local reporting substitutes; no official score claimed.'}
    out=ROOT/'docs/task1';(out/'results.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    lines=['# Task 1 实际验证记录','',
           '由 `scripts/report_task1.py` 根据保留的独立裁判结果生成。完整历史（包括调试失败）见 [results.json](results.json)。',
           '每项展示最近一次已完成的尝试，可能来自较早提交；运行目录的 provenance.json 与源码快照标识当时版本。新修改不能继承旧运行的通过结论。',
           '正常任务与故障测试分开判定；程序报告 COMPLETE 不等于独立验收通过。尚未运行的组合明确列出。','',
           '| 用例 | 判定 | 位置 / 航向误差 | 最小净空 | 证据 |',
           '|---|---|---|---|---|']
    for key in expected:
        r=latest.get(key)
        if not r:lines.append(f'| {key} | 未验证 | — | — | — |');continue
        endpoint=r.get('endpoint')
        errors=f"{endpoint['position_m']:.3f} m / {endpoint['yaw_rad']:.3f} rad" if endpoint else '不适用／未完成'
        clearance='—' if r['clearance_m'] is None else f"{r['clearance_m']:.3f} m"
        file='task1_fault_result.json' if r['fault'] else 'task1_result.json'
        lines.append(f"| {key} | {'通过' if r['passed'] else '失败'} | {errors} | {clearance} | [{Path(r['run']).name}](../../{r['run']}/{file}) |")
    lines+=['','模型和感知检查：']
    for name,items,file in [('相机灯态/实体',models,'model_validation.json'),('LiDAR 三层回波',lidars,'lidar_validation.json')]:
        if items:
            r=items[-1];lines+=['',f"- {name}：{'通过' if r['passed'] else '失败'}；[最近证据](../../{r['run']}/{file})。"]
        else:lines+=['',f'- {name}：未验证。']
    lines+=['','固定固件、规则及范围见 [implementation.md](implementation.md)，启动命令见 [runbook.md](runbook.md)，根因与修复见 [debug_log.md](debug_log.md)。',
            'WAM-V、固定系泊模型和本地报告不能代表正式 UAV 协同、RoboCommand 接入或实船比赛得分。']
    (out/'validation.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({'normal_passed':sum(latest[k]['passed'] for k in expected[:18] if k in latest),
                      'fault_passed':sum(latest[k]['passed'] for k in expected[18:] if k in latest),
                      'unverified':[k for k in expected if k not in latest],'attempts':len(attempts)},ensure_ascii=False))


if __name__=='__main__':main()
