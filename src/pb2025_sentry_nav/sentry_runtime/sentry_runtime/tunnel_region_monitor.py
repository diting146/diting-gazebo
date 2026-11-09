#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from sentry_system_interfaces.msg import TunnelRegionState

class TunnelRegionMonitor(Node):
    def __init__(self):
        super().__init__('tunnel_region_monitor')
        self.robot_id = self.declare_parameter('robot_id', 'sentry_robot1').value
        self.tunnel_id = self.declare_parameter('tunnel_id', 'demo_tunnel_unverified').value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.base_frame = self.declare_parameter('base_frame', '').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.entry_x = float(self.declare_parameter('entry_x', 0.0).value)
        self.entry_y = float(self.declare_parameter('entry_y', 0.0).value)
        self.heading = float(self.declare_parameter('heading_rad', 0.0).value)
        self.width = float(self.declare_parameter('width_m', 0.8).value)
        self.length = float(self.declare_parameter('length_m', 2.0).value)
        self.roof_start = float(self.declare_parameter('roof_start_m', 0.825).value)
        self.roof_end = float(self.declare_parameter('roof_end_m', 1.175).value)
        self.pose_timeout = float(self.declare_parameter('pose_timeout_sec', 0.5).value)
        self.pose = None
        self.pose_time = None
        self.pub = self.create_publisher(TunnelRegionState, '~/state', 10)
        self.sub = self.create_subscription(PoseWithCovarianceStamped, '~/pose', self.pose_cb, 10)
        self.timer = self.create_timer(0.1, self.publish_state)

    def pose_cb(self, msg):
        if msg.header.frame_id != self.frame_id:
            self.pose = None
            return
        self.pose = msg.pose.pose
        self.pose_time = self.get_clock().now()

    def publish_state(self):
        if self.base_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id, self.base_frame, Time())
                pose = PoseWithCovarianceStamped()
                pose.header = transform.header
                pose.pose.pose.position.x = transform.transform.translation.x
                pose.pose.pose.position.y = transform.transform.translation.y
                self.pose_cb(pose)
                self.pose_time = Time.from_msg(transform.header.stamp)
            except TransformException:
                self.pose = None
        msg = TunnelRegionState(); msg.robot_id = self.robot_id; msg.tunnel_id = self.tunnel_id
        msg.stamp = self.get_clock().now().to_msg(); msg.frame_id = self.frame_id
        msg.valid = False; msg.region = 'unknown'; msg.invalid_reason = 'pose_unavailable_or_expired'
        if self.pose is not None and (self.get_clock().now() - self.pose_time).nanoseconds <= self.pose_timeout * 1e9:
            dx = self.pose.position.x - self.entry_x; dy = self.pose.position.y - self.entry_y
            along = dx * math.cos(self.heading) + dy * math.sin(self.heading)
            lateral = -dx * math.sin(self.heading) + dy * math.cos(self.heading)
            if abs(lateral) > self.width / 2.0: region = 'outside'
            elif along < 0.0: region = 'approach'
            elif along <= self.roof_start: region = 'inside'
            elif along <= self.roof_end: region = 'central_roof'
            elif along <= self.length: region = 'inside'
            else: region = 'exit'
            msg.valid = True; msg.region = region; msg.invalid_reason = ''
        self.pub.publish(msg)

def main():
    rclpy.init(); node = TunnelRegionMonitor(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
