#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import DockRobot, UndockRobot
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty


def dock_coordinates(pose, dock_x, dock_y, dock_yaw):
    position = pose.position
    orientation = pose.orientation
    yaw = math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                     1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z))
    dx, dy = position.x - dock_x, position.y - dock_y
    along = dx * math.cos(dock_yaw) + dy * math.sin(dock_yaw)
    lateral = -dx * math.sin(dock_yaw) + dy * math.cos(dock_yaw)
    heading = math.atan2(math.sin(yaw - dock_yaw), math.cos(yaw - dock_yaw))
    return along, lateral, heading


class SimDocking(Node):
    def __init__(self):
        super().__init__('sim_docking')
        self.declare_parameter('dock_x', 0.0)
        self.declare_parameter('dock_y', 0.0)
        self.declare_parameter('dock_yaw', 0.0)
        self.dock_x = self.get_parameter('dock_x').value
        self.dock_y = self.get_parameter('dock_y').value
        self.dock_yaw = self.get_parameter('dock_yaw').value
        self.lock = threading.Lock()
        self.busy = False
        self.pose = None
        self.pose_at = None
        self.odom_at = None
        self.stationary_since = None
        self.stopped = threading.Event()
        self.group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(Twist, '/sim_base/docking_cmd_vel', 1)
        self.create_subscription(PoseStamped, '/sim_base/ground_truth', self.receive_pose, 10,
                                 callback_group=self.group)
        self.create_subscription(Odometry, 'odom', self.receive_odom, 10, callback_group=self.group)
        self.create_subscription(Empty, 'stop', self.stop, 10, callback_group=self.group)
        self.dock_server = ActionServer(self, DockRobot, 'dock_robot', self.execute_dock,
                                        goal_callback=self.dock_goal, cancel_callback=self.cancel,
                                        callback_group=self.group)
        self.undock_server = ActionServer(self, UndockRobot, 'undock_robot', self.execute_undock,
                                          goal_callback=self.undock_goal, cancel_callback=self.cancel,
                                          callback_group=self.group)

    def receive_pose(self, message):
        with self.lock:
            self.pose = message.pose
            self.pose_at = time.monotonic()

    def stop(self, _message):
        self.stopped.set()

    def receive_odom(self, message):
        now = time.monotonic()
        twist = message.twist.twist
        stationary = (math.isfinite(twist.linear.x) and math.isfinite(twist.angular.z)
                      and abs(twist.linear.x) < 0.01 and abs(twist.angular.z) < 0.02)
        with self.lock:
            if not stationary or self.odom_at is None or now - self.odom_at > 0.5:
                self.stationary_since = None
            elif self.stationary_since is None:
                self.stationary_since = now
            self.odom_at = now

    def reserve(self, docking):
        with self.lock:
            if self.busy or self.pose is None or time.monotonic() - self.pose_at > 1.0:
                return GoalResponse.REJECT
            if (self.odom_at is None or time.monotonic() - self.odom_at > 0.5
                    or self.stationary_since is None or time.monotonic() - self.stationary_since < 0.3):
                return GoalResponse.REJECT
            along, lateral, heading = dock_coordinates(self.pose, self.dock_x, self.dock_y, self.dock_yaw)
            if not all(math.isfinite(value) for value in (along, lateral, heading)):
                return GoalResponse.REJECT
            if abs(lateral) > 0.08 or abs(heading) > 0.18:
                return GoalResponse.REJECT
            if docking and not 0.03 < along <= 0.8:
                return GoalResponse.REJECT
            if not docking and abs(along) > 0.08:
                return GoalResponse.REJECT
            self.busy = True
            self.stopped.clear()
            return GoalResponse.ACCEPT

    def dock_goal(self, goal):
        if not goal.use_dock_id or goal.dock_id not in ('', 'omron') or goal.navigate_to_staging_pose:
            return GoalResponse.REJECT
        return self.reserve(True)

    def undock_goal(self, goal):
        if goal.dock_type not in ('', 'omron') or not math.isfinite(goal.max_undocking_time) or goal.max_undocking_time <= 0:
            return GoalResponse.REJECT
        return self.reserve(False)

    def cancel(self, _goal):
        self.stopped.set()
        return CancelResponse.ACCEPT

    def execute_dock(self, goal):
        return self.execute(goal, True, 30.0)

    def execute_undock(self, goal):
        return self.execute(goal, False, min(goal.request.max_undocking_time, 30.0))

    def execute(self, goal, docking, timeout):
        result = DockRobot.Result() if docking else UndockRobot.Result()
        deadline = time.monotonic() + timeout
        settled_at = None
        error = 'Docking timed out' if docking else 'Undocking timed out'
        try:
            while time.monotonic() < deadline and rclpy.ok():
                if goal.is_cancel_requested or self.stopped.is_set():
                    error = 'Maneuver canceled or stopped'
                    break
                with self.lock:
                    pose, pose_at = self.pose, self.pose_at
                    odom_at, stationary_since = self.odom_at, self.stationary_since
                if pose is None or time.monotonic() - pose_at > 1.0:
                    error = 'Ground-truth pose is unavailable or stale'
                    break
                along, lateral, heading = dock_coordinates(pose, self.dock_x, self.dock_y, self.dock_yaw)
                if not all(math.isfinite(value) for value in (along, lateral, heading)):
                    error = 'Ground-truth pose is invalid'
                    break
                if abs(lateral) > 0.1 or abs(heading) > 0.2 or along < -0.06:
                    error = 'Robot left the dock approach corridor'
                    break
                reached = along <= 0.03 if docking else along >= 0.6
                command = Twist()
                if reached:
                    settled_at = settled_at or time.monotonic()
                    if (time.monotonic() - settled_at >= 0.3 and odom_at is not None
                            and time.monotonic() - odom_at <= 0.5 and stationary_since is not None
                            and time.monotonic() - stationary_since >= 0.3):
                        self.publisher.publish(command)
                        result.success = True
                        result.error_code = result.NONE
                        goal.succeed()
                        return result
                else:
                    settled_at = None
                    command.linear.x = -0.1 if docking else 0.1
                self.publisher.publish(command)
                if docking:
                    feedback = DockRobot.Feedback()
                    feedback.state = feedback.CONTROLLING
                    goal.publish_feedback(feedback)
                else:
                    goal.publish_feedback(UndockRobot.Feedback())
                time.sleep(0.05)
            self.publisher.publish(Twist())
            result.success = False
            result.error_code = result.FAILED_TO_CONTROL
            result.error_msg = error
            if goal.is_cancel_requested:
                goal.canceled()
            else:
                goal.abort()
            return result
        finally:
            self.publisher.publish(Twist())
            with self.lock:
                self.busy = False


def main():
    rclpy.init()
    node = SimDocking()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.publisher.publish(Twist())
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()