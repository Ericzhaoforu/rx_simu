"""Source-timed LiDAR traffic guard. Outputs permission evidence, never thrust."""
from collections import deque
from array import array
import copy
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Path as NavPath, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener, TransformException
from robotx_task1_interfaces.msg import TrafficSafety, BuoyArray
from usv_sim.cloud_filter import xyz_view
from .traffic import Tracker, StaticMap, corridor, conflict, exclude_surveyed_markers, traffic_grid, tracking_points


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def pose_xyyaw(transform):
    p, q = transform.translation, transform.rotation
    return [p.x, p.y, math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))]


class TrafficGuard(Node):
    def __init__(self):
        super().__init__('traffic_guard')
        self.declare_parameter('footprint', '')
        self.declare_parameter('run_dir', '')
        self.declare_parameter('horizon', 20.)
        self.declare_parameter('speed_bound', .3)
        self.declare_parameter('margin', .6)
        self.declare_parameter('continuation_range', 40.)
        self.footprint = np.asarray(json.loads(self.get_parameter('footprint').value))
        self.horizon = self.get_parameter('horizon').value
        self.speed = self.get_parameter('speed_bound').value
        self.margin = self.get_parameter('margin').value
        self.continuation_range = self.get_parameter('continuation_range').value
        run = self.get_parameter('run_dir').value
        self.log = (Path(run)/'traffic_guard.jsonl').open('w') if run else None
        self.tracker = Tracker(horizon=self.horizon)
        self.static_map = None
        self.map_template = None
        self.last_grid_stamp = -math.inf
        self.markers = None
        self.marker_stamp = -math.inf
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.path = None
        self.path_stamp = Time().to_msg()
        self.pending = deque(maxlen=5)
        self.last_result = None
        self.publisher = self.create_publisher(TrafficSafety, '/task1/traffic_safety', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid,'/task1/traffic_costmap',
                                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PointCloud2, '/usv/lidar/marking', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(NavPath, '/task1/reference_path', self.on_path, 10)
        self.create_subscription(NavPath, '/task1/candidate_path', self.on_path, 10)
        self.create_subscription(BuoyArray, '/task1/observations', self.on_markers, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.02, self.process)

    def on_cloud(self, msg):
        self.pending.append(msg)

    def on_markers(self,msg):
        stamp=seconds(msg.header.stamp)
        markers=np.array([[b.position.x,b.position.y,b.collision_radius] for b in msg.buoys])
        if (msg.header.frame_id=='map' and msg.survey_complete and stamp>self.marker_stamp
                and len(markers)==10 and np.isfinite(markers).all() and np.all(markers[:,2]>0)):
            self.markers=markers
            self.marker_stamp=stamp

    def on_map(self, msg):
        if msg.header.frame_id != 'map' or msg.info.resolution <= 0:
            return
        p, q = msg.info.origin.position, msg.info.origin.orientation
        origin = [p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        self.static_map = StaticMap(np.asarray(msg.data).reshape(msg.info.height,msg.info.width),
                                    msg.info.resolution,origin)
        first=self.map_template is None
        self.map_template=msg
        if first:
            # Initialize the extra layer before Nav2 activation. Other layers
            # still contain all surveyed/measured obstacles; motion needs fresh
            # source-stamped safety evidence after the mission is activated.
            self.publish_grid(msg.header,[])

    def publish_grid(self,header,tracks):
        msg=OccupancyGrid();msg.header=copy.deepcopy(header);msg.header.frame_id='map'
        msg.info=self.map_template.info
        values=traffic_grid(tracks,seconds(header.stamp),self.static_map.occupied.shape,
                            self.static_map.resolution,self.static_map.origin,self.horizon,self.margin)
        msg.data=array('b',values.tobytes())
        self.grid_pub.publish(msg)
        self.last_grid_stamp=seconds(header.stamp)

    def on_path(self, msg):
        if msg.header.frame_id != 'map' or not msg.poses or seconds(msg.header.stamp) <= seconds(self.path_stamp):
            return
        points = []
        for p in msg.poses:
            q = p.pose.orientation
            points.append([p.pose.position.x, p.pose.position.y,
                           math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])
        if not np.isfinite(points).all():
            return
        self.path = np.asarray(points)
        self.path_stamp = copy.deepcopy(msg.header.stamp)

    def process(self):
        if not self.pending or self.static_map is None or self.markers is None:
            return
        msg = self.pending[0]
        stamp = seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds / 1e9
        if not -.05 <= now-self.marker_stamp <= .35:
            return
        if not -.05 <= now - stamp <= .35 or stamp <= self.tracker.stamp:
            self.pending.popleft()
            return
        try:
            source_time = Time.from_msg(msg.header.stamp)
            transform = self.tf.lookup_transform('map', msg.header.frame_id, source_time).transform
            boat = self.tf.lookup_transform('map', 'base_link', source_time).transform
        except TransformException:
            # Wait briefly for the matching FCU transform, without restamping.
            return
        self.pending.popleft()
        p, q = transform.translation, transform.rotation
        points = xyz_view(msg).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        points = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply(points) + [p.x, p.y, p.z]
        points = self.static_map.unmapped(points)
        points = exclude_surveyed_markers(points,self.markers)
        points = tracking_points(points,[p.x,p.y],self.tracker.tracks,stamp,
                                 self.tracker.observation_range,self.continuation_range)
        pose = pose_xyyaw(boat)
        tracks = self.tracker.update(points, stamp, np.asarray(pose[:2]))
        swept = corridor(self.footprint, pose, self.path, self.horizon * self.speed)
        risk = conflict(tracks, stamp, swept, self.horizon, self.margin)
        if stamp-self.last_grid_stamp>=.199:self.publish_grid(msg.header,tracks)
        result = TrafficSafety()
        result.header = copy.deepcopy(msg.header)
        result.header.frame_id = 'map'
        result.path_stamp = copy.deepcopy(self.path_stamp)
        result.clear = risk is None
        result.reason = risk['reason'] if risk else 'observed traffic corridor clear'
        result.track_id = risk['track_id'] if risk else 0
        result.conflict_after_s = risk['future_s'] if risk else -1.
        self.publisher.publish(result)
        row = dict(source_stamp=stamp, evaluated_at=now, path_stamp=seconds(self.path_stamp),
                   horizon=self.horizon,speed_bound=self.speed,margin=self.margin,
                   acquisition_range=self.tracker.observation_range,continuation_range=self.continuation_range,
                   clear=result.clear, risk=risk, pose=pose, corridor=swept.tolist(),
                   merged_tracks=self.tracker.merges, retired_tracks=self.tracker.retired,
                   tracks=[dict(id=t.id, stamp=t.stamp, bounds=t.bounds.tolist(),
                                velocity=t.velocity.tolist(), moving=t.moving) for t in tracks])
        if self.log:
            self.log.write(json.dumps(row, allow_nan=False)+'\n')
            self.log.flush()
        if result.clear != self.last_result:
            self.get_logger().info(f'traffic clear={result.clear}: {result.reason}; source={stamp:.3f}')
            self.last_result = result.clear


def main():
    rclpy.init()
    node = TrafficGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
