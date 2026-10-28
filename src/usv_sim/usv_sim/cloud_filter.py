"""Remove the simulated vessel from LiDAR without rewriting source timestamps."""
import json
from collections import OrderedDict
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation


def xyz_view(msg):
    fields = {field.name: field for field in msg.fields}
    if not all(key in fields and fields[key].datatype == 7 for key in ('x', 'y', 'z')):
        raise ValueError('Expected float32 x/y/z fields')
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                      'formats': [('>' if msg.is_bigendian else '<')+'f4']*3,
                      'offsets': [fields[key].offset for key in ('x', 'y', 'z')],
                      'itemsize': msg.point_step})
    records = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                         strides=(msg.row_step, msg.point_step))
    return np.stack([records[key] for key in ('x', 'y', 'z')], axis=-1)


def inside_polygon(points, polygon, padding=0.1):
    polygon = np.asarray(polygon)
    edges = np.roll(polygon, -1, axis=0)-polygon
    delta = points[..., None, :2]-polygon
    cross = edges[:, 0]*delta[..., 1]-edges[:, 1]*delta[..., 0]
    return np.all(cross >= -padding*np.linalg.norm(edges, axis=1), axis=-1)


class CloudFilter(Node):
    def __init__(self):
        super().__init__('cloud_filter')
        self.declare_parameter('scene_file', '')
        with open(self.get_parameter('scene_file').value) as stream:
            scene = json.load(stream)
        self.polygon = scene['footprint']
        self.offset = np.array(scene['lidar_xyz'])
        self.shape = (scene.get('lidar_beams', 16), scene.get('lidar_samples', 1875))
        self.clouds = OrderedDict()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(PointCloud2, '/usv/lidar/filtered', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/usv/lidar/points', self.on_cloud, qos_profile_sensor_data)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/usv/scan_raw', self.on_scan, qos_profile_sensor_data)

    def on_cloud(self, msg):
        try:
            xyz = xyz_view(msg)
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            self.clouds[key] = (xyz, msg.header.frame_id)
            while len(self.clouds) > 20:
                self.clouds.popitem(last=False)
            finite = np.isfinite(xyz).all(axis=-1)
            # The fixed VRX LiDAR is aligned with base_link; only the known lever arm applies.
            body_points = xyz+self.offset
            remove = np.zeros(finite.shape, dtype=bool)
            remove[finite] = inside_polygon(body_points[finite], self.polygon)
            data = bytearray(msg.data)
            for field in msg.fields:
                if field.name in ('x', 'y', 'z'):
                    values = np.ndarray((msg.height, msg.width),
                                        dtype=('>' if msg.is_bigendian else '<')+'f4',
                                        buffer=data, offset=field.offset,
                                        strides=(msg.row_step, msg.point_step))
                    values[remove] = np.nan
            msg.data = bytes(data)
            msg.is_dense = False
            self.publisher.publish(msg)
        except (ValueError, TypeError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=2)

    def on_scan(self, scan):
        key = (scan.header.stamp.sec, scan.header.stamp.nanosec)
        if key not in self.clouds:
            return
        xyz, source_frame = self.clouds[key]
        # This adapter is deliberately tied to the generated organized 16-beam VRX sensor.
        if xyz.shape[:2] != self.shape:
            self.get_logger().error('Unexpected VRX LiDAR layout; no guarded scan published', throttle_duration_sec=2)
            return
        try:
            transform = self.tf.lookup_transform(scan.header.frame_id, source_frame, Time.from_msg(scan.header.stamp))
        except TransformException:
            return
        q = transform.transform.rotation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        angles = np.linspace(-math.pi, math.pi, xyz.shape[1])
        directions = rotation.apply(np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1))
        projected = np.arctan2(directions[:, 1], directions[:, 0])
        indices = np.floor((projected-scan.angle_min)/scan.angle_increment).astype(int)
        center = xyz[7:9]
        distance = np.linalg.norm(center, axis=-1)
        observed_free = np.all((np.isposinf(distance) | (distance >= scan.range_max)) &
                               ~np.isnan(center).any(axis=-1), axis=0)
        valid_bins = (indices >= 0) & (indices < len(scan.ranges))
        total = np.bincount(indices[valid_bins], minlength=len(scan.ranges))
        clear = np.bincount(indices[valid_bins & observed_free], minlength=len(scan.ranges))
        ranges = np.array(scan.ranges)
        # A converter's empty bin is not proof of free water. Require real no-hit rays.
        unknown = np.isposinf(ranges) & ((total == 0) | (clear != total))
        ranges[unknown] = np.nan
        scan.ranges = ranges.astype(np.float32).tolist()
        self.scan_publisher.publish(scan)


def main():
    rclpy.init()
    node = CloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
