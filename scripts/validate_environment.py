#!/usr/bin/env python3
"""Integration checks that keep Gazebo and ArduRover running during failures."""
import argparse
import json
import math
import os
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from validate import ROOT, Runner, yaw


def gz_service(path, request_type, request):
    result=subprocess.run(['gz','service','-s',path,'--reqtype',request_type,
                           '--reptype','gz.msgs.Boolean','--timeout','5000','--req',request],
                          capture_output=True,text=True,timeout=8)
    if result.returncode or 'true' not in result.stdout:
        raise RuntimeError(result.stdout+result.stderr)


def move(runner,x,y):
    gz_service('/world/usv_nav/set_pose','gz.msgs.Pose',
               f'name: "dynamic_obstacle" position {{ x: {x} y: {y} z: 1.5 }} orientation {{ w: 1 }}')
    next(obj for obj in runner.scene['obstacles'] if obj['name']=='dynamic_obstacle').update(x=x,y=y)


def dynamic_map(runner):
    latest=[None]
    runner.node.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda msg:latest.__setitem__(0,msg),
                                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    runner.wait(lambda:latest[0] is not None,10,'No global costmap')
    def occupied():
        m=latest[0];r=m.info.resolution;o=m.info.origin.position
        a=np.asarray(m.data).reshape(m.info.height,m.info.width)
        # Include the visible near faces of the 2 m box, not just its occluded centre.
        return int(np.count_nonzero(a[int((2.8-o.y)/r):int((5.2-o.y)/r)+1,
                                      int((2.8-o.x)/r):int((5.2-o.x)/r)+1]>=99))
    runner.phase='dynamic_baseline';runner.spin(3);before=occupied()
    move(runner,4,4);runner.phase='dynamic_appears';start=time.monotonic()
    runner.wait(lambda:occupied()>0,10,'Moved obstacle not marked in costmap')
    marked=occupied();mark_time=time.monotonic()-start;runner.spin(3)
    move(runner,15,20);runner.phase='dynamic_removed';start=time.monotonic()
    runner.wait(lambda:occupied()==0,15,'Removed obstacle still marked')
    return {'baseline_lethal_cells':before,'marked_lethal_cells':marked,'mark_delay_s':mark_time,
            'clear_delay_s':time.monotonic()-start,'remaining_lethal_cells':occupied(),
            'passed':before==0 and marked>0 and occupied()==0}


def pause(runner):
    runner.arm();runner.phase='before_pause'
    for _ in range(60):runner.command(0,.12);runner.spin(.05)
    runner.phase='paused';before=runner.rows[-1]
    gz_service('/world/usv_nav/control','gz.msgs.WorldControl','pause: true')
    start=time.monotonic();zero=None
    while time.monotonic()-start<2:
        runner.spin(.02)
        preview=runner.data['/usv/setpoint_preview']
        if abs(preview.velocity.x)+abs(preview.yaw_rate)<1e-7 and zero is None:zero=time.monotonic()-start
    frozen=runner.rows[-1]
    gz_service('/world/usv_nav/control','gz.msgs.WorldControl','pause: false')
    runner.phase='resumed_without_command';runner.spin(4)
    rows=[r for r in runner.rows if r['phase']=='resumed_without_command']
    replay=any(abs(r.get('preview',[0,0])[1])>.01 for r in rows)
    return {'zero_output_wall_s':zero,'paused_sim_delta_s':frozen['sim']-before['sim'],
            'resumed_sim_delta_s':runner.rows[-1]['sim']-frozen['sim'],'replayed_old_command':replay,
            'passed':zero is not None and zero<=.6 and not replay,
            'note':'Paused physics is not evidence of physical stopping.'}


def mavros_exit(runner):
    runner.arm();runner.phase='before_mavros_exit'
    for _ in range(100):runner.command(.2,0);runner.spin(.05)
    session=json.loads((ROOT/'runs/session.json').read_text())
    process=next(p for p in session['processes'] if p['name']=='mavros')
    before=runner.rows[-1];start=time.monotonic();os.killpg(process['pid'],signal.SIGKILL)
    runner.phase='mavros_exited';runner.spin(12)
    rows=[r for r in runner.rows if r['phase']=='mavros_exited']
    zero=next((r['wall']-start for r in rows if abs(r.get('left_n',999))+abs(r.get('right_n',999))<1e-7),None)
    stopped=next((r['wall']-start for r in rows if math.hypot(*r['truth'][4:6])<.02),None)
    after=runner.rows[-1]
    return {'thrust_zero_wall_s':zero,'speed_below_002_wall_s':stopped,
            'displacement_m':math.dist(before['truth'][:2],after['truth'][:2]),
            'sim_advanced_s':after['sim']-before['sim'],'final_thrust_n':[after['left_n'],after['right_n']],
            'passed':zero is not None and after['sim']-before['sim']>8}


def gnss(runner):
    client=runner.node.create_client(SetParameters,'/gnss_injector/set_parameters')
    if not client.wait_for_service(timeout_sec=10):raise RuntimeError('Start with --gps-input')
    runner.arm();results=[]
    profiles=[('fixed',6,.02,.02,.1,False,8),('float',5,.3,.3,.1,False,12),
              ('ordinary',3,1.5,1.5,.3,False,15),('lost',1,1.5,1.5,.3,True,15),
              ('recover',6,.02,.02,.1,False,20)]
    for name,fix,sigma,accuracy,delay,drop,duration in profiles:
        values=dict(fix_type=fix,horizontal_sigma=sigma,reported_accuracy=accuracy,delay_s=delay,drop=drop)
        request=SetParameters.Request(parameters=[Parameter(key,value=value).to_parameter_msg() for key,value in values.items()])
        future=client.call_async(request);runner.wait(future.done,5,'GNSS profile update')
        if not all(item.successful for item in future.result().results):raise RuntimeError(str(future.result()))
        runner.phase='gnss_'+name;start=len(runner.rows);end=time.monotonic()+duration
        while time.monotonic()<end:
            # Exercise the current control gate during degradation, without pretending this is a new policy.
            runner.command(.10,0);runner.spin(.05)
        rows=runner.rows[start:]
        errors=[math.dist(r['truth'][:2],r['ekf'][:2]) for r in rows if 'truth' in r and 'ekf' in r]
        results.append({'profile':name,'settings':values,'position_rmse_m':float(np.sqrt(np.mean(np.square(errors)))),
                        'position_error_max_m':max(errors),'last_gate':rows[-1].get('gate'),
                        'nonzero_preview_samples':sum(abs(r.get('preview',[0,0])[0])>.01 for r in rows),
                        'samples':len(rows),'reported_fix_types':sorted(set(r.get('GPS_RAW_INT',{}).get('fix_type',-1) for r in rows)),
                        'last_ekf_status':rows[-1].get('EKF_STATUS_REPORT'),
                        'minimum_clearance_m':min(r['clearance_m'] for r in rows)})
    return results


def motors(runner):
    # Standard ArduRover motor-test command, not direct Gazebo actuator control.
    runner.arm();runner.mav.mav.set_mode_send(1,1,4);runner.spin(1);results=[]
    for side,number in [('left',3),('right',4)]:
        for pwm in (1600,1400,1500):
            runner.phase=f'motor_{side}_{pwm}'
            runner.mav.mav.command_long_send(1,1,209,0,number,1,pwm,3,0,0,0)
            runner.spin(2)
            row=runner.rows[-1];expected=(pwm-1500)*.6
            actual=row['left_n' if side=='left' else 'right_n']
            results.append({'side':side,'pwm':pwm,'expected_n':expected,'actual_n':actual,
                            'other_n':row['right_n' if side=='left' else 'left_n'],'truth_yaw_rate':row['truth'][6],
                            'passed':abs(actual-expected)<1e-6})
            runner.phase='motor_timeout';runner.spin(3)
    return results


def collision_sensor(runner):
    runner.phase='contact_positive_control'
    move(runner,0,.7);runner.spin(3)
    return {'collision_messages':len(runner.contacts),'contact_messages':runner.contact_messages,
            'passed':len(runner.contacts)>0,
            'note':'Intentional overlap in an isolated disarmed scene validates contact detection.'}


def reset(runner):
    runner.phase='before_reset';runner.spin(2);before=runner.rows[-1]
    gz_service('/world/usv_nav/control','gz.msgs.WorldControl','reset { all: true }')
    runner.phase='after_reset';runner.spin(12)
    rows=[r for r in runner.rows if r['phase']=='after_reset']
    latest=rows[-1]
    return {'before_sim_s':before['sim'],'minimum_sim_after_reset_s':min(r['sim'] for r in rows),
            'final_sim_s':latest['sim'],'final_odom_age_s':latest.get('ekf_age'),
            'final_scan_age_s':latest.get('scan_age'),'final_gate':latest.get('gate'),
            'nonzero_preview_samples':sum(sum(abs(v) for v in r.get('preview',[0,0]))>.001 for r in rows),
            'passed':0<=latest.get('ekf_age',999)<.5 and 0<=latest.get('scan_age',999)<.5}


def dynamic_navigation(runner):
    original=runner.spin;start=[None];moved=[False];removed=[False]
    def spin(duration=.05):
        original(duration)
        if runner.phase!='navigation':return
        if start[0] is None:start[0]=time.monotonic()
        elapsed=time.monotonic()-start[0]
        if elapsed>3 and not moved[0]:move(runner,6,4);moved[0]=True
        if elapsed>28 and not removed[0]:move(runner,15,20);removed[0]=True
    runner.spin=spin
    result=runner.goal([[12,8,0]])
    result.update(obstacle_inserted=moved[0],obstacle_removed=removed[0])
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('case',choices=['dynamic_map','dynamic_navigation','pause','mavros_exit','gnss','motors','collision_sensor','reset'])
    args=parser.parse_args();rclpy.init();runner=Runner(args.case)
    try:
        runner.ready();runner.navigation_ready()
        result=globals()[args.case](runner)
        (runner.run/(args.case+'_result.json')).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    finally:runner.close();rclpy.shutdown()


if __name__=='__main__':main()
