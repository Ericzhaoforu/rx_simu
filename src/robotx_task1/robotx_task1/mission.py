"""Task1 orchestration. All vessel pose/velocity comes from MAVROS EKF via TF."""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import ComputePathToPose, FollowPath
from mavros_msgs.msg import State
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from robotx_task1_interfaces.msg import BuoyArray, MissionContext, MotionPermission
from .core import CircleTracker, passage_events, wrap


def seconds(stamp):return stamp.sec+stamp.nanosec/1e9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class Mission(Node):
    def __init__(self):
        super().__init__('task1_mission')
        self.declare_parameter('run_dir','');self.declare_parameter('circle_radius',6.)
        self.radius=self.get_parameter('circle_radius').value
        run=self.get_parameter('run_dir').value
        self.log=(Path(run)/'task1_mission.jsonl').open('w') if run else None
        self.obs=None;self.buoys=[];self.pose=None;self.odom=None;self.pose_stamp=-1.;self.pose_wall=0.
        self.state=None;self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.stage='WAIT_SURVEY';self.motion='IDLE';self.started=False;self.generation=0
        self.bits=0;self.circle=None;self.arc_index=0;self.entry=None;self.exit=None
        self.target=None;self.plan_future=None;self.send_future=None;self.result_future=None
        self.follow_handle=None;self.request_generation=0;self.cancel_requested=False
        self.still_since=None;self.last_retry=-1e9;self.started_at=None;self.last_path=None
        self.previous_pose=None;self.reason='waiting for start';self.last_revision=0
        self.last_event_pass={};self.obs_wall=0.;self.last_source_stamp=0.
        self.plan_client=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.follow_client=ActionClient(self,FollowPath,'/follow_path')
        self.context_pub=self.create_publisher(MissionContext,'/task1/context',10)
        self.permission_pub=self.create_publisher(MotionPermission,'/task1/motion_permission',10)
        self.status_pub=self.create_publisher(String,'/task1/status',10)
        self.events_pub=self.create_publisher(String,'/task1/events',10)
        self.path_pub=self.create_publisher(NavPath,'/task1/reference_path',10)
        self.scene_pub=self.create_publisher(String,'/task1/scene_command',10)
        self.create_subscription(BuoyArray,'/task1/observations',self.on_observation,10)
        self.create_subscription(Odometry,'/usv/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(State,'/mavros/state',lambda m:setattr(self,'state',m),qos_profile_sensor_data)
        self.create_service(Trigger,'/task1/start',self.start)
        self.create_service(Trigger,'/task1/cancel',self.cancel)
        self.create_timer(.1,self.tick)

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def event(self,kind,**values):
        row=dict(sim=self.now(),wall=time.monotonic(),event=kind,stage=self.stage,
                 motion=self.motion,revision=self.last_revision,generation=self.generation,**values)
        text=json.dumps(row,allow_nan=False)
        if self.log:self.log.write(text+'\n');self.log.flush()
        self.events_pub.publish(String(data=text));self.get_logger().info(text)

    def start(self,request,response):
        if self.started:
            response.message='Task already started; restart session for a new attempt';return response
        if not self.state or not self.state.connected or not self.state.armed or self.state.mode!='GUIDED':
            response.message='Connected, normally armed GUIDED SITL required';return response
        self.started=True;self.started_at=self.now();self.motion='STOPPING'
        self.scene_pub.publish(String(data=json.dumps({'op':'activate'})))
        self.event('started',observer='known_state_substitute',robocommand='local_only')
        response.success=True;response.message='Waiting for complete surveyed observations';return response

    def cancel(self,request,response):
        self.abort('task cancelled');response.success=True;response.message=self.reason;return response

    def interrupt(self,reason):
        self.generation+=1;self.motion='STOPPING';self.reason=reason;self.still_since=None
        if self.follow_handle and not self.cancel_requested:
            self.follow_handle.cancel_goal_async();self.cancel_requested=True
        self.permission(False);self.event('stop_requested',reason=reason)

    def abort(self,reason):
        self.interrupt(reason);self.stage='ABORTED';self.motion='ABORTED';self.event('aborted',reason=reason)

    def on_observation(self,msg):
        t=seconds(msg.header.stamp)
        if t<=self.last_source_stamp or msg.revision<self.last_revision:return
        if msg.header.frame_id!='map' or not -.1<=self.now()-t<=.5:return
        data=[dict(id=b.id,x=b.position.x,y=b.position.y,state=b.state,radius=b.collision_radius) for b in msg.buoys]
        if len(data)!=10 or len({b['id'] for b in data})!=10:return
        if any(b['state'] not in range(5) or b['radius']<=0 or not all(math.isfinite(b[k]) for k in ('x','y','radius')) for b in data):return
        entries=[b for b in data if b['state']==3];exits=[b for b in data if b['state']==4]
        if len(entries)!=1 or len(exits)!=1:
            if self.started:self.abort('invalid entry/exit assignment')
            return
        if self.entry and (entries[0]['id']!=self.entry['id'] or exits[0]['id']!=self.exit['id']):
            self.abort('entry/exit reassigned during attempt');return
        changed=self.obs is not None and msg.revision!=self.last_revision
        old_states={b['id']:b['state'] for b in self.buoys}
        self.obs=msg;self.buoys=data;self.last_source_stamp=t;self.last_revision=msg.revision;self.obs_wall=time.monotonic()
        if changed and self.started and self.stage not in ('COMPLETE','ABORTED','WAIT_SURVEY'):
            self.interrupt('semantic revision changed')
            self.event('semantic_update_reported',source_stamp=t,
                       changes=[b for b in data if old_states.get(b['id'])!=b['state']],report_sink='local_only')

    def on_odom(self,msg):
        t=seconds(msg.header.stamp)
        if t<=self.pose_stamp or not -.1<=self.now()-t<=.5:return
        try:
            transform=self.tf.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
        except Exception:return
        a=yaw(transform.rotation);p=msg.pose.pose.position;c,s=math.cos(a),math.sin(a)
        current=[transform.translation.x+c*p.x-s*p.y,transform.translation.y+s*p.x+c*p.y,
                 wrap(a+yaw(msg.pose.pose.orientation))]
        self.previous_pose=self.pose;self.pose=current;self.odom=msg
        self.pose_stamp=t;self.pose_wall=time.monotonic()
        if self.circle and self.stage in ('CIRCLE_ENTRY','CIRCLE_EXIT'):
            was=self.circle.complete;self.circle.update(current)
            if not was and self.circle.complete:self.event('circle_completed',angle=self.circle.angle)
        if self.stage=='TRANSIT' and self.previous_pose and self.obs:
            for e in passage_events(self.previous_pose,current,self.buoys):
                # Do not accumulate repeated boundary jitter at the same marker.
                if self.now()-self.last_event_pass.get(e['id'],-1e9)<2.:continue
                self.last_event_pass[e['id']]=self.now();self.event('ekf_passage',**e)
                if not e['legal']:self.abort('EKF detected wrong-side/ambiguous passage: '+e['id'])
                else:self.bits|=1 if e['state']==1 else 2

    def permission(self,allow):
        m=MotionPermission();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='map'
        m.revision=self.last_revision;m.allow=allow;m.reason=self.reason;self.permission_pub.publish(m)

    def publish_context(self):
        m=MissionContext();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='map'
        m.revision=self.last_revision;m.phase=self.stage;m.enforce_passage=self.stage=='TRANSIT';m.passed_colors=self.bits
        if self.stage in ('CIRCLE_ENTRY','CIRCLE_EXIT') and self.circle:
            m.circle_center.x=float(self.circle.center[0]);m.circle_center.y=float(self.circle.center[1])
            m.circle_radius=self.radius;m.circle_direction=self.circle.direction
        self.context_pub.publish(m)

    def inputs_valid(self):
        return (self.obs is not None and self.obs.survey_complete and self.pose is not None and
                -.1<=self.now()-self.last_source_stamp<=.5 and time.monotonic()-self.obs_wall<=.5 and
                -.1<=self.now()-self.pose_stamp<=.5 and time.monotonic()-self.pose_wall<=.5)

    def select_target(self):
        if self.stage=='APPROACH_ENTRY':self.target=[self.entry['x']-self.radius,self.entry['y'],math.pi/2]
        elif self.stage=='TRANSIT':self.target=[self.exit['x']-self.radius,self.exit['y'],-math.pi/2]
        else:
            marker=self.entry if self.stage=='CIRCLE_ENTRY' else self.exit
            direction=-1 if self.stage=='CIRCLE_ENTRY' else 1
            sweep=min(self.arc_index,4)*math.pi/2+max(0,self.arc_index-4)*.12
            a=math.pi+direction*sweep
            self.target=[marker['x']+self.radius*math.cos(a),marker['y']+self.radius*math.sin(a),wrap(a+direction*math.pi/2)]

    def advance(self):
        if self.stage=='APPROACH_ENTRY':
            self.stage='CIRCLE_ENTRY';self.arc_index=1;self.circle=CircleTracker([self.entry['x'],self.entry['y']],-1,self.radius);self.circle.update(self.pose)
        elif self.stage in ('CIRCLE_ENTRY','CIRCLE_EXIT'):
            if self.circle.complete:
                if self.stage=='CIRCLE_ENTRY':
                    self.stage='TRANSIT';self.circle=None
                    self.scene_pub.publish(String(data=json.dumps({'op':'transit'})))
                else:
                    self.stage='COMPLETE';self.motion='COMPLETE';self.reason='task completed'
                    self.permission(False);self.event('completed',target=self.target,ekf_pose=self.pose);return
            else:
                self.arc_index+=1
                if self.arc_index>10:self.abort('full circle closure was not achieved');return
        elif self.stage=='TRANSIT':
            if self.bits!=3:self.abort('transit finished without both valid colors');return
            self.stage='CIRCLE_EXIT';self.arc_index=1;self.circle=CircleTracker([self.exit['x'],self.exit['y']],1,self.radius);self.circle.update(self.pose)
        self.select_target();self.motion='STOPPING';self.still_since=None
        self.event('stage_target',target=self.target)

    def request_plan(self):
        if not self.plan_client.server_is_ready() or not self.follow_client.server_is_ready():return
        self.publish_context();goal=ComputePathToPose.Goal();goal.planner_id='Task1';goal.use_start=False
        goal.goal=PoseStamped();goal.goal.header.frame_id='map';goal.goal.header.stamp=self.get_clock().now().to_msg()
        goal.goal.pose.position.x=float(self.target[0]);goal.goal.pose.position.y=float(self.target[1])
        goal.goal.pose.orientation.z=math.sin(self.target[2]/2);goal.goal.pose.orientation.w=math.cos(self.target[2]/2)
        self.plan_future=self.plan_client.send_goal_async(goal);self.plan_handle=None;self.plan_result=None
        self.request_generation=self.generation;self.motion='PLANNING';self.last_retry=self.now()
        self.event('planning',target=self.target)

    def poll_actions(self):
        if self.plan_future and self.plan_future.done():
            handle=self.plan_future.result();self.plan_future=None
            if handle.accepted:self.plan_handle=handle;self.plan_result=handle.get_result_async()
            elif self.motion=='PLANNING':self.motion='WAIT_CLEAR';self.reason='planner goal rejected'
        if getattr(self,'plan_result',None) and self.plan_result.done():
            result=self.plan_result.result();self.plan_result=None;self.plan_handle=None
            if self.request_generation==self.generation and self.motion=='PLANNING':
                if result.status!=4 or not result.result.path.poses:
                    self.motion='WAIT_CLEAR';self.reason='no legal path';self.event('plan_failed',status=result.status)
                else:
                    self.last_path=result.result.path;self.path_pub.publish(self.last_path)
                    goal=FollowPath.Goal();goal.path=self.last_path;goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
                    self.send_future=self.follow_client.send_goal_async(goal);self.motion='STARTING'
                    self.event('path_ready',poses=len(goal.path.poses),target=self.target)
        if self.send_future and self.send_future.done():
            self.follow_handle=self.send_future.result();self.send_future=None
            if not self.follow_handle.accepted:
                self.follow_handle=None;self.motion='WAIT_CLEAR';self.reason='controller goal rejected'
            else:
                self.result_future=self.follow_handle.get_result_async();self.cancel_requested=False
                if self.request_generation!=self.generation or self.motion!='STARTING':
                    self.follow_handle.cancel_goal_async();self.cancel_requested=True
                else:self.motion='EXECUTING';self.reason='fresh path executing';self.event('execution_started')
        if self.result_future and self.result_future.done():
            result=self.result_future.result();was_cancel=self.cancel_requested
            self.follow_handle=None;self.result_future=None;self.cancel_requested=False
            self.event('controller_result',status=result.status,cancelled=was_cancel)
            if not was_cancel and self.motion=='EXECUTING':
                if result.status==4:self.advance()
                else:self.interrupt('controller failed; wait and replan')

    def tick(self):
        self.publish_context();self.poll_actions()
        if self.started and self.stage not in ('COMPLETE','ABORTED'):
            if self.now()-self.started_at>1200:self.abort('1200 sim-second task timeout')
            elif not self.inputs_valid():
                if self.motion not in ('STOPPING','IDLE'):self.interrupt('observation or EKF stale')
                self.still_since=None
            else:
                if self.stage=='WAIT_SURVEY':
                    self.entry=next(b for b in self.buoys if b['state']==3);self.exit=next(b for b in self.buoys if b['state']==4)
                    self.event('observations_reported',source_stamp=self.last_source_stamp,buoys=self.buoys,report_sink='local_only')
                    self.stage='APPROACH_ENTRY';self.select_target();self.event('survey_accepted',target=self.target)
                if self.motion in ('STOPPING','WAIT_CLEAR'):
                    v=self.odom.twist.twist.linear;w=self.odom.twist.twist.angular
                    if math.hypot(v.x,v.y)<.05 and abs(w.z)<.05:
                        if self.still_since is None:self.still_since=self.now()
                    else:self.still_since=None
                    free=not any((self.follow_handle,self.send_future,self.plan_future,getattr(self,'plan_result',None)))
                    if free and self.still_since is not None and self.now()-self.still_since>=1. and self.now()-self.last_retry>=2.:
                        self.request_plan()
        allow=self.motion=='EXECUTING' and self.inputs_valid() and self.stage not in ('COMPLETE','ABORTED')
        self.permission(allow)
        status=dict(sim=self.now(),stage=self.stage,motion=self.motion,revision=self.last_revision,
                    target=self.target,pose=self.pose,reason=self.reason,passed_colors=self.bits,
                    circle_angle=self.circle.angle if self.circle else None)
        self.status_pub.publish(String(data=json.dumps(status)))


def main():
    rclpy.init();node=Mission()
    try:rclpy.spin(node)
    except KeyboardInterrupt:node.permission(False)
    finally:
        if node.log:node.log.close()
        node.destroy_node();rclpy.try_shutdown()
