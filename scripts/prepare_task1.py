#!/usr/bin/env python3
"""Adapt a newly generated independent VRX world to the 2026 Safe Passage task."""
import argparse
import json
import math
from pathlib import Path
import random
import sys
import xml.etree.ElementTree as E

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/usv_nav2'))
from usv_nav2.make_water_map import write_map


def prepare(out, seed=1, changes=0, dynamic='none'):
    from generate_world import element, set_text, box
    out=Path(out);rng=random.Random(seed)
    defaults=json.loads((ROOT/'config/task1/defaults.json').read_text())
    buoys=[dict(id='entry',x=10.,y=0.,state=3,radius=.36),dict(id='exit',x=60.,y=0.,state=4,radius=.36)]
    for i,x in enumerate((24.,32.,40.,48.)):
        for j,y in enumerate((7.,-7.)):
            # Stable seed 1 is the straight reference; seeds 2/3 are staggered.
            dx=0 if seed==1 else rng.uniform(-1.,1.)
            dy=0 if seed==1 else rng.uniform(-2.,2.)
            buoys.append(dict(id=f'buoy_{i*2+j+1:02}',x=x+dx,y=y+dy,state=2 if j==0 else 1,radius=.36))
    events=[{'after_s':20.,'id':'buoy_03','state':1}, {'after_s':65.,'id':'buoy_06','state':2}][:changes]
    scenario={'seed':seed,'tier':'disruptive','buoys':buoys,'changes':events,'dynamic':dynamic,
              'boundary':defaults['boundary'],'circle_radius':defaults['circle_radius_m']}
    (out/'task1_scenario.json').write_text(json.dumps(scenario,indent=2)+'\n')
    tree=E.parse(out/'usv_nav.sdf');world=tree.getroot().find('world')
    for n in list(world.findall('include')):
        if n.findtext('uri')!='coast_waves':world.remove(n)
    for n in list(world.findall('model')):
        if n.get('name') in ('static_obstacle','dynamic_obstacle'):world.remove(n)
    template=E.parse(ROOT/'models/robotx_2026_robobuoy/model.sdf').getroot().find('model')
    for b in buoys:
        m=E.fromstring(E.tostring(template));m.set('name',b['id'])
        element(m,'pose',f"{b['x']} {b['y']} 0 0 0 0");world.append(m)
    if dynamic!='none':box(world,'task1_moving_obstacle',32,-14,sx=2.,sy=3.,height=.9)
    p=element(world,'plugin',filename='libTask1Scene.so',name='Task1Scene')
    element(p,'scenario',str(out/'task1_scenario.json'))
    observer=world.find("model[@name='overview_camera']")
    set_text(observer,'pose','35 0 85 0 1.57079632679 0')
    set_text(observer.find('.//sensor'),'update_rate',5)
    observer.find('.//camera/horizontal_fov').text='1.3'
    observer.find('.//camera/image/width').text='1280'
    observer.find('.//camera/image/height').text='960'
    close=E.fromstring(E.tostring(observer));close.set('name','beacon_camera')
    set_text(close,'pose','10 0 2 0 1.57079632679 0')
    set_text(close.find('.//sensor'),'topic','/task1/beacon_view')
    world.append(close)
    E.indent(tree);tree.write(out/'usv_nav.sdf',encoding='utf-8',xml_declaration=True)
    scene=json.loads((out/'scene.json').read_text())
    scene['task1']=scenario
    scene['obstacles']=[dict(name=b['id'],x=b['x'],y=b['y'],size=[.508,.508]) for b in buoys]
    if dynamic!='none':scene['obstacles'].append(dict(name='task1_moving_obstacle',x=32.,y=-14.,size=[2.,3.]))
    (out/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    return scenario


def navigation(out):
    out=Path(out);scene=json.loads((out/'scene.json').read_text());scenario=scene['task1']
    params=json.loads((out/'nav2_wamv.yaml').read_text())
    planner=params['planner_server']['ros__parameters']
    planner['planner_plugins']=['GridBased','Task1']
    planner['Task1']={'plugin':'robotx_task1_nav/RulePlanner'}
    controller=params['controller_server']['ros__parameters']
    controller['FollowPath']['critics'].insert(0,'Task1Rule')
    controller['FollowPath']['Task1Rule.class']='robotx_task1_nav::RuleCritic'
    controller['FollowPath']['Task1Rule.scale']=1.0
    controller['failure_tolerance']=.5
    for name in ('local_costmap','global_costmap'):
        costmap=params[name][name]['ros__parameters']
        costmap['plugins'].insert(-1,'known_buoys')
        costmap['known_buoys']={'plugin':'robotx_task1_nav/KnownBuoys'}
    params['rover_bridge']['ros__parameters']['require_task_permission']=True
    (out/'nav2_wamv.yaml').write_text(json.dumps(params,indent=2)+'\n')
    area={'resolution':.2,'padding':1.,'navigable_boundary':scenario['boundary'],'exclusions':[]}
    write_map(area,out/'water',overwrite=True);(out/'water_area.json').write_text(json.dumps(area,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--seed',type=int,default=1)
    p.add_argument('--changes',type=int,choices=(0,1,2),default=0);p.add_argument('--dynamic',choices=('none','cross','block'),default='none')
    a=p.parse_args();prepare(a.output,a.seed,a.changes,a.dynamic)
