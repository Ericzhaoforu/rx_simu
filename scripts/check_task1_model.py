#!/usr/bin/env python3
"""Exercise actual rendered beacon states and save camera evidence without arming."""
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

ROOT=Path(__file__).resolve().parents[1]


def main():
    run=Path((ROOT/'runs/latest.txt').read_text().strip());rclpy.init()
    node=rclpy.create_node('task1_beacon_validation',parameter_overrides=[Parameter('use_sim_time',value=True)])
    pub=node.create_publisher(String,'/task1/scene_command',10)
    state=[None];rows=[];writer=[None];images={};mode=[None]
    node.create_subscription(String,'/task1/scene_state',lambda m:state.__setitem__(0,json.loads(m.data)),10)
    def receive(msg):
        a=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        rgb=a if msg.encoding=='rgb8' else cv2.cvtColor(a,cv2.COLOR_BGR2RGB)
        bgr=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
        if writer[0] is None:writer[0]=cv2.VideoWriter(str(run/'beacon_states.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),5,(msg.width,msg.height))
        writer[0].write(bgr)
        roi=rgb[msg.height//2-50:msg.height//2+50,msg.width//2-50:msg.width//2+50].astype(float)
        counts=[int(np.sum((roi[:,:,i]>100)&(roi[:,:,i]>1.8*roi[:,:,(i+1)%3])&(roi[:,:,i]>1.8*roi[:,:,(i+2)%3]))) for i in range(3)]
        rows.append(dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,requested=mode[0],dominant_rgb_pixels=counts,
                         scene_stamp=state[0]['stamp'] if state[0] else None,
                         entry=next(b for b in state[0]['buoys'] if b['id']=='entry') if state[0] else None))
        key=(mode[0],max(counts)>10)
        if key not in images:
            images[key]=True;cv2.imwrite(str(run/f'beacon_{mode[0]}_{int(max(counts)>10)}.png'),bgr)
    node.create_subscription(Image,'/task1/beacon_view',receive,qos_profile_sensor_data)
    start=time.monotonic()
    while state[0] is None and time.monotonic()-start<20:rclpy.spin_once(node,timeout_sec=.1)
    if state[0] is None:raise RuntimeError('No authoritative scene state')
    for semantic in range(5):
        mode[0]=semantic;pub.publish(String(data=json.dumps({'op':'set_states','states':{'entry':semantic}})))
        end=time.monotonic()+5
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    pub.publish(String(data=json.dumps({'op':'set_states','states':{'entry':3}})))
    for _ in range(5):rclpy.spin_once(node,timeout_sec=.1)
    result={}
    for semantic in range(5):
        samples=[r for r in rows if r['requested']==semantic and r['entry'] and r['entry']['state']==semantic]
        channel=semantic-1 if semantic in (1,2) else 2
        on=sum(r['dominant_rgb_pixels'][channel]>10 for r in samples)
        off=sum(max(r['dominant_rgb_pixels'])<=10 for r in samples)
        result[str(semantic)]=dict(frames=len(samples),on=on,off=off,
            passed=len(samples)>=10 and (off>=10 if semantic==0 else on>=10 if semantic==4 else on>=5 and off>=5))
    report=dict(passed=all(r['passed'] for r in result.values()),states=result,samples=rows)
    (run/'model_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    if writer[0]:writer[0].release()
    node.destroy_node();rclpy.try_shutdown();print(json.dumps(result))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
