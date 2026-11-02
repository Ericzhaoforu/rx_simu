#!/usr/bin/env python3
"""Compare complete recorded tasks without changing their evidence directories.

Truth is used for independent travel distance / progress evaluation only.
Arrival predictions come from the source-stamped route assessment log.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def rows(path):
    return [json.loads(line) for line in path.open()]


def route_switches(run,events,motion):
    """Same-revision accepted-route changes; source-timed FCU pose, no truth."""
    from robotx_task1.route_policy import assess_routes
    raw=run/'visualization/raw/events.jsonl'
    if not raw.exists():return None
    plans=[e for e in rows(raw) if e['topic']=='/task1/reference_path' and e.get('poses')]
    hull=json.loads((run/'generated/scene.json').read_text())['footprint']
    previous=None;changes=[]
    for event in events:
        if (event['event']!='path_ready' or event['stage']!='TRANSIT'
                or event.get('purpose')=='observation'):continue
        plan=min(plans,key=lambda e:abs(e['sim']-event['sim'])) if plans else None
        if plan is None or abs(plan['sim']-event['sim'])>1. or plan['frame']!='map':
            previous=None;changes.append(dict(sim=event['sim'],assessable=False));continue
        candidate=np.asarray(plan['poses']).reshape(-1,3)
        if previous is not None and previous[0]==event['revision']:
            ekf=min(motion,key=lambda e:abs(e['sim']-event['sim']))['ekf']
            decision,_=assess_routes(candidate,previous[1],[ekf[0],ekf[1],ekf[3]],hull,[],event['sim'])
            changes.append(dict(sim=event['sim'],assessable=decision['reference_eta_s'] is not None,
                                mean_deviation_m=decision['mean_route_deviation_m'],
                                corridor_switch=decision['switch_cost_s']>0.))
        previous=event['revision'],candidate
    return changes


def summarize(run):
    run=Path(run).resolve()
    result=json.loads((run/'task1_result.json').read_text())
    events=rows(run/'task1_mission.jsonl')
    start=next(e['sim'] for e in events if e['event']=='started')
    complete=next((e['sim'] for e in events if e['event']=='completed'),None)
    transit=next((e['sim'] for e in events if e['event']=='stage_target' and e['stage']=='TRANSIT'),None)
    finish=next((e['sim'] for e in events if e['event']=='stage_target' and e['stage']=='CIRCLE_EXIT'),None)
    motion=[e for e in rows(run/'task1.jsonl') if 'truth' in e and e['sim']>=start
            and (complete is None or e['sim']<=complete)]
    segment=[e for e in motion if transit is not None and e['sim']>=transit
             and (finish is None or e['sim']<=finish)]
    xy=np.array([e['truth'][:2] for e in motion])
    transit_xy=np.array([e['truth'][:2] for e in segment])
    target=next((e['target'] for e in events if e['event']=='stage_target' and e['stage']=='TRANSIT'),None)
    backward=None
    if len(transit_xy)>1:
        direction=np.array(target[:2])-transit_xy[0];direction/=np.linalg.norm(direction)
        backward=float(-np.minimum(np.diff(transit_xy,axis=0)@direction,0.).sum())
    ready=[e for e in events if e['event']=='path_ready' and e['stage']=='TRANSIT']
    predictions=[]
    for e in events:
        if e['event']!='route_selected' or e['stage']!='TRANSIT':continue
        end=next((p for p in events if p['sim']>e['sim'] and p['event']=='controller_result'),None)
        if end is None or end['status']!=4 or end.get('cancelled'):continue
        execution=next((p for p in events if e['sim']<=p['sim']<=end['sim'] and p['event']=='execution_started'),None)
        if execution:
            actual=end['sim']-execution['sim']
            predictions.append(dict(path_stamp=e['path_stamp'],purpose=e.get('purpose','mission'),predicted_s=e['eta_s'],actual_s=actual,
                                    error_s=actual-e['eta_s']))
    route_path=run/'route_decisions.jsonl'
    route=rows(route_path) if route_path.exists() else []
    maps=run/'dynamic_map_result.json'
    map_result=json.loads(maps.read_text()) if maps.exists() else {}
    evidence=['task1_result.json','task1_mission.jsonl','task1.jsonl']
    if route:evidence.append('route_decisions.jsonl')
    switches=route_switches(run,events,motion)
    return dict(run=str(run),workspace_commit=json.loads((run/'provenance.json').read_text())['workspace_commit'],
                passed=result['passed'],complete=complete is not None,
                duration_s=complete-start if complete is not None else None,
                transit_duration_s=finish-transit if finish is not None and transit is not None else None,
                actual_distance_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()) if len(xy)>1 else 0.,
                transit_distance_m=float(np.linalg.norm(np.diff(transit_xy,axis=0),axis=1).sum()) if len(transit_xy)>1 else 0.,
                transit_reverse_progress_m=backward,
                accepted_transit_paths=len(ready),traffic_yields=sum(e['event']=='traffic_yield' for e in events),
                geometric_transit_switches=(sum(e.get('corridor_switch',False) for e in switches)
                    if switches is not None and all(e['assessable'] for e in switches) else None),
                geometric_switch_details=switches,
                route_wait_decisions=sum(e['event']=='route_wait_selected' for e in events),
                observation_steps=sum(e['event']=='observation_point_reached' for e in events),
                planning_failures=sum(e['event']=='plan_failed' for e in events),
                stale_stops=sum(e['event']=='stop_requested' and 'stale' in e.get('reason','') for e in events),
                collision_contacts=result['collision_contacts'],min_clearance_m=result['min_clearance_m'],
                min_traffic_clearance_m=map_result.get('min_moving_obstacle_clearance_m'),
                endpoint_error=result.get('endpoint_error'),completed_route_eta_errors=predictions,
                advisory_compute_max_s=max((r['evaluation_wall_s'] for r in route),default=None),
                advisory_compute_p95_s=float(np.quantile([r['evaluation_wall_s'] for r in route],.95)) if route else None,
                evidence_sha256={name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in evidence})


def main():
    parser=argparse.ArgumentParser();parser.add_argument('runs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    data=dict(method='VRX truth for distance only. Reverse progress is cumulative negative displacement along the transit start-to-goal axis, not reverse thrust. Whole-task duration includes entry/exit circles and waits. ETA errors include only routes completed without cancellation. Geometric switches compare successive accepted same-revision transit paths with compatible references, using FCU pose and a 0.75 m mean-deviation threshold; semantic changes are excluded. Missing or incompatible evidence makes that count unknown.',
              runs=[summarize(run) for run in args.runs])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps(data,indent=2))


if __name__=='__main__':main()
