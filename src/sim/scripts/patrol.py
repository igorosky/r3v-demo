#!/usr/bin/env python3
"""Drives the robot around a loop of waypoints forever, from /odom (the ground truth pose).

Turns in place towards the next waypoint, then drives to it while correcting the heading.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# Around crate_c, clear of the revolving door and the cart.
DEFAULT_WAYPOINTS = [2.0, -1.5, 2.0, 1.5, -2.0, 1.5, -2.0, -1.5]


class Patrol(Node):
    def __init__(self):
        super().__init__("patrol")
        flat = self.declare_parameter("waypoints", DEFAULT_WAYPOINTS).value
        self.waypoints = list(zip(flat[0::2], flat[1::2]))
        self.speed = self.declare_parameter("speed", 0.4).value
        self.turn_speed = self.declare_parameter("turn_speed", 0.8).value
        self.tolerance = self.declare_parameter("tolerance", 0.15).value
        self.target = 0
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)

    def on_odom(self, odom):
        position = odom.pose.pose.position
        q = odom.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        x, y = self.waypoints[self.target]
        dx, dy = x - position.x, y - position.y
        if math.hypot(dx, dy) < self.tolerance:
            self.target = (self.target + 1) % len(self.waypoints)
            self.get_logger().info(f"next waypoint {self.waypoints[self.target]}")
            return

        error = math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
        cmd = Twist()
        cmd.angular.z = max(-self.turn_speed, min(self.turn_speed, 2.0 * error))
        if abs(error) < 0.3:
            cmd.linear.x = self.speed * min(1.0, math.hypot(dx, dy))
        self.publisher.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(Patrol())


if __name__ == "__main__":
    main()
