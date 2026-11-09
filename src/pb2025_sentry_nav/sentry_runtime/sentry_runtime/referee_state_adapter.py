#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sentry_system_interfaces.msg import RefereeRobotState

class RefereeStateAdapter(Node):
    def __init__(self):
        super().__init__('referee_state_adapter')
        self.robot_id = self.declare_parameter('robot_id', 'sentry_robot1').value
        self.source = self.declare_parameter('source', 'simulated_referee').value
        self.expiry = float(self.declare_parameter('expiry_sec', 1.0).value)
        self.hp = None; self.received = None
        self.pub = self.create_publisher(RefereeRobotState, '~/state', 10)
        self.sub = self.create_subscription(Int32, f'/referee/{self.robot_id}/hp', self.cb, 10)
        self.timer = self.create_timer(0.1, self.publish_state)
    def cb(self, msg):
        self.hp = float(msg.data); self.received = self.get_clock().now()
    def publish_state(self):
        now = self.get_clock().now(); msg = RefereeRobotState(); msg.robot_id = self.robot_id
        msg.stamp = now.to_msg(); msg.source = self.source; msg.hp = self.hp or 0.0
        msg.valid = self.received is not None and (now - self.received).nanoseconds <= self.expiry * 1e9
        msg.communicating = msg.valid; msg.invalid_reason = '' if msg.valid else 'expired_or_not_received'; self.pub.publish(msg)
def main():
    rclpy.init(); node = RefereeStateAdapter(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
