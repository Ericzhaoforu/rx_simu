"""Translate Nav2 Twist to MAVROS BODY_NED velocity + yaw-rate setpoints."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from .core import (BODY_NED_FRAME, VELOCITY_YAW_RATE_MASK, CommandGate,
                   bounded_command, normalise_quaternion, stamp_is_fresh, scan_ranges_valid)
from .task_lease import TaskLease


class RoverBridge(Node):
    def __init__(self):
        super().__init__('rover_bridge')
        defaults = {
            'enable_control': False,
            'command_topic': '/usv/cmd_vel',
            'odom_topic': '/mavros/local_position/odom',
            'scan_topic': '/scan',
            'state_topic': '/mavros/state',
            'setpoint_topic': '/mavros/setpoint_raw/local',
            'expected_parent_frame': 'map',
            'body_frame': 'base_link',
            'max_speed': 0.4,
            'max_yaw_rate': 0.4,
            'command_timeout': 0.5,
            'odom_timeout': 0.5,
            'scan_timeout': 0.5,
            'state_timeout': 2.0,
            'publish_rate': 20.0,
            'require_task_permission': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        for name in ('max_speed', 'max_yaw_rate', 'publish_rate'):
            if not math.isfinite(self.p[name]) or self.p[name] <= 0.0:
                raise ValueError(name + ' must be positive')
        self.gate = CommandGate(**{key: self.p[key] for key in (
            'command_timeout', 'odom_timeout', 'scan_timeout', 'state_timeout')})
        self.sensor_headers = {}
        self.task_lease = TaskLease()
        if self.p['require_task_permission']:
            from robotx_task1_interfaces.msg import MotionPermission
            self.create_subscription(MotionPermission, '/task1/motion_permission', self.on_permission, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Preview is always available. No FCU output publisher in preview mode.
        self.preview_pub = self.create_publisher(PositionTarget, '/usv/setpoint_preview', 10)
        self.status_pub = self.create_publisher(String, '/usv/control_status', 10)
        self.output_pub = (self.create_publisher(PositionTarget, self.p['setpoint_topic'], 10)
                           if self.p['enable_control'] else None)
        self.create_subscription(Twist, self.p['command_topic'], self.on_command, 10)
        self.create_subscription(State, self.p['state_topic'], self.on_state, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['odom_topic'], self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p['scan_topic'], self.on_scan, qos_profile_sensor_data)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / self.p['publish_rate'], self.tick,
                                      clock=self.steady_clock)
        self.last_status = None

    def on_command(self, msg):
        if self.p['require_task_permission'] and not self.task_lease.valid(
                self.get_clock().now().nanoseconds/1e9, time.monotonic()):
            self.gate.invalidate('command')
            return
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        try:
            command = bounded_command(values, self.p['max_speed'], self.p['max_yaw_rate'])
        except ValueError as error:
            self.gate.invalidate('command')
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            return
        self.gate.update_command(command, time.monotonic())

    def on_permission(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        if self.task_lease.update(msg.revision, msg.allow, stamp,
                                  self.get_clock().now().nanoseconds/1e9, time.monotonic(), msg.reason):
            self.gate.invalidate('command')

    def on_state(self, msg):
        self.gate.update_state(msg.connected, msg.armed, msg.mode, time.monotonic())

    def header_fresh(self, header, timeout):
        stamp = header.stamp.sec + header.stamp.nanosec / 1e9
        now = self.get_clock().now().nanoseconds / 1e9
        return stamp_is_fresh(stamp, now, timeout)

    def on_odom(self, msg):
        p, q, v, w = (msg.pose.pose.position, msg.pose.pose.orientation,
                      msg.twist.twist.linear, msg.twist.twist.angular)
        try:
            normalise_quaternion((q.x, q.y, q.z, q.w))
            valid = (self.header_fresh(msg.header, self.p['odom_timeout']) and
                     msg.header.frame_id == self.p['expected_parent_frame'] and
                     msg.child_frame_id == self.p['body_frame'] and
                     all(math.isfinite(x) for x in (p.x, p.y, p.z, v.x, v.y, v.z, w.x, w.y, w.z)))
        except ValueError:
            valid = False
        if valid:
            self.gate.mark('odom', time.monotonic())
            self.sensor_headers['odom'] = msg.header
        else:
            self.gate.invalidate('odom')

    def on_scan(self, msg):
        valid = (self.header_fresh(msg.header, self.p['scan_timeout']) and
                 bool(msg.header.frame_id) and len(msg.ranges) > 0 and
                 math.isfinite(msg.angle_increment) and msg.angle_increment > 0.0 and
                 math.isfinite(msg.range_min) and math.isfinite(msg.range_max) and
                 0.0 <= msg.range_min < msg.range_max and
                 scan_ranges_valid(msg.ranges, msg.range_min, msg.range_max) and
                 self.tf_buffer.can_transform(self.p['body_frame'], msg.header.frame_id,
                                              Time.from_msg(msg.header.stamp)))
        if valid:
            self.gate.mark('scan', time.monotonic())
            self.sensor_headers['scan'] = msg.header
        else:
            self.gate.invalidate('scan')

    def setpoint(self, command):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['body_frame']
        msg.coordinate_frame = BODY_NED_FRAME
        msg.type_mask = VELOCITY_YAW_RATE_MASK
        msg.velocity.x, msg.yaw_rate = command
        msg.velocity.y = 0.0
        # Keep ROS signs: +x forward, +yaw_rate counterclockwise.
        # MAVROS setpoint_raw performs FRD/NED conversion once.
        return msg

    def tick(self):
        for source, header in self.sensor_headers.items():
            if not self.header_fresh(header, self.p[source + '_timeout']):
                self.gate.invalidate(source)
            elif source == 'scan' and not self.tf_buffer.can_transform(
                    self.p['body_frame'], header.frame_id, Time.from_msg(header.stamp)):
                self.gate.invalidate(source)
        command, reason = self.gate.evaluate(time.monotonic())
        if self.p['require_task_permission'] and not self.task_lease.valid(
                self.get_clock().now().nanoseconds/1e9, time.monotonic()):
            command, reason = (0.0, 0.0), 'task stopped: ' + self.task_lease.reason
            self.gate.invalidate('command')
        msg = self.setpoint(command)
        self.preview_pub.publish(msg)
        # Outside GUIDED / armed state, let the pilot / FCU mode own motion.
        if self.output_pub is not None and self.gate.eligible:
            self.output_pub.publish(msg)
        status = ('CONTROL: ' if self.output_pub is not None else 'PREVIEW: ') + reason
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def stop(self):
        if self.output_pub is not None and self.gate.eligible:
            self.output_pub.publish(self.setpoint((0.0, 0.0)))


def main(args=None):
    rclpy.init(args=args)
    node = RoverBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
