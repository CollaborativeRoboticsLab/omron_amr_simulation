#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Empty


class SimBaseCommand(Node):
    def __init__(self):
        super().__init__('sim_base_command')
        self.publisher = self.create_publisher(Twist, '/sim_base/cmd_vel', 1)
        self.create_subscription(Twist, 'cmd_vel', self.command, 1)
        self.create_subscription(Twist, '/sim_base/docking_cmd_vel', self.docking_command, 1)
        self.create_subscription(Empty, 'stop', self.stop, 10)
        self.latest = Twist()
        self.received_at = None
        self.docking_latest = Twist()
        self.docking_received_at = None
        self.create_timer(0.05, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def command(self, message):
        docking_at = getattr(self, 'docking_received_at', None)
        if docking_at is not None and time.monotonic() - docking_at < 0.2:
            return
        components = (message.linear.x, message.linear.y, message.linear.z,
                      message.angular.x, message.angular.y, message.angular.z)
        if not all(math.isfinite(value) for value in components) or any(
                abs(value) > 1e-6 for value in (message.linear.y, message.linear.z,
                                                  message.angular.x, message.angular.y)):
            return
        self.latest = Twist()
        self.latest.linear.x = max(-0.2, min(1.0, message.linear.x))
        self.latest.angular.z = max(-math.pi / 3, min(math.pi / 3, message.angular.z))
        self.received_at = time.monotonic()

    def docking_command(self, message):
        if not math.isfinite(message.linear.x) or not math.isfinite(message.angular.z):
            return
        self.received_at = None
        self.docking_latest = Twist()
        self.docking_latest.linear.x = max(-0.2, min(0.2, message.linear.x))
        self.docking_latest.angular.z = max(-math.pi / 3, min(math.pi / 3, message.angular.z))
        self.docking_received_at = time.monotonic()

    def stop(self, _message):
        self.received_at = None
        self.docking_received_at = None
        self.publisher.publish(Twist())

    def publish(self):
        docking_at = getattr(self, 'docking_received_at', None)
        if docking_at is not None and time.monotonic() - docking_at < 0.2:
            self.publisher.publish(self.docking_latest)
            return
        fresh = self.received_at is not None and time.monotonic() - self.received_at < 0.2
        self.publisher.publish(self.latest if fresh else Twist())


def main():
    rclpy.init()
    node = SimBaseCommand()
    try:
        rclpy.spin(node)
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()