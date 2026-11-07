#!/usr/bin/env python3
"""Publishes offset + amplitude * sin(2 pi t / period) as std_msgs/Float64 on `topic`, in sim time.

Drives the cart along its rail and nods the robot's head.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class Oscillator(Node):
    def __init__(self):
        super().__init__("oscillator")
        self.amplitude = self.declare_parameter("amplitude", 1.0).value
        self.period = self.declare_parameter("period", 10.0).value
        self.offset = self.declare_parameter("offset", 0.0).value
        rate = self.declare_parameter("rate", 20.0).value
        self.publisher = self.create_publisher(Float64, self.declare_parameter("topic", "cmd").value, 10)
        self.create_timer(1.0 / rate, self.publish)

    def publish(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        value = self.offset + self.amplitude * math.sin(2 * math.pi * t / self.period)
        self.publisher.publish(Float64(data=value))


def main():
    rclpy.init()
    rclpy.spin(Oscillator())


if __name__ == "__main__":
    main()
