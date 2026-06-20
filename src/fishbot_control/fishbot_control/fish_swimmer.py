#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


class FishSwimmer(Node):

    def __init__(self):
        super().__init__('fish_swimmer')

        self.get_logger().info("Fish swimmer started!")

        # Tail controller publisher
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/tail_position_controller/commands',
            10
        )

        # Speed command subscriber
        self.speed_subscriber = self.create_subscription(
            Float64,
            '/fish_speed',
            self.speed_callback,
            10
        )

        self.start_time = time.time()

        # Oscillator parameters
        self.amplitude = 0.5      # radians
        self.frequency = 1.0      # Hz

        # 50 Hz control loop
        self.timer = self.create_timer(
            0.02,
            self.update_tail
        )

    def speed_callback(self, msg):

        self.frequency = max(0.0, msg.data)

        self.get_logger().info(
            f"Swimming frequency set to {self.frequency:.2f} Hz"
        )

    def update_tail(self):

        t = time.time() - self.start_time

        theta = self.amplitude * math.sin(
            2 * math.pi * self.frequency * t
        )

        msg = Float64MultiArray()
        msg.data = [theta]

        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = FishSwimmer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()