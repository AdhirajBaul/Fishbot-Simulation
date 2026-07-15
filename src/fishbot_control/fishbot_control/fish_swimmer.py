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

        self.get_logger().info(
            "Fish swimmer with tail steering started!"
        )

        # Tail position command
        self.tail_publisher = self.create_publisher(
            Float64MultiArray,
            '/tail_position_controller/commands',
            10
        )

        # Swimming-frequency command
        self.speed_subscriber = self.create_subscription(
            Float64,
            '/fish_speed',
            self.speed_callback,
            10
        )

        # Left/right tail-bias command
        self.turn_subscriber = self.create_subscription(
            Float64,
            '/fish_turn',
            self.turn_callback,
            10
        )

        # Oscillator settings
        self.normal_amplitude = 0.5
        self.frequency = 1.0
        self.tail_bias = 0.0

        # Tail joint has limits of ±0.7 rad.
        self.safe_joint_limit = 0.68
        self.maximum_bias = 0.18

        # Continuous phase variables
        self.phase = 0.0
        self.previous_time = time.monotonic()

        # 50 Hz control loop
        self.timer = self.create_timer(
            0.02,
            self.update_tail
        )

    def speed_callback(self, msg):
        """
        Set the tail oscillation frequency.

        At amplitude 0.5 rad and frequency 1.2 Hz:

            maximum angular velocity
            = 2 * pi * frequency * amplitude
            = approximately 3.77 rad/s

        This remains below the URDF limit of 4 rad/s.
        """

        self.frequency = max(
            0.0,
            min(float(msg.data), 1.2)
        )

        self.get_logger().info(
            f"Swimming frequency set to "
            f"{self.frequency:.2f} Hz"
        )

    def turn_callback(self, msg):
        """Set the centre-point bias of the tail oscillation."""

        self.tail_bias = max(
            -self.maximum_bias,
            min(float(msg.data), self.maximum_bias)
        )

        self.get_logger().info(
            f"Tail steering bias set to "
            f"{self.tail_bias:.2f} rad"
        )

    def update_tail(self):
        """Calculate and publish the next tail position."""

        current_time = time.monotonic()

        dt = current_time - self.previous_time
        self.previous_time = current_time

        # Prevent a large command jump after a delayed callback.
        dt = max(0.0, min(dt, 0.1))

        if self.frequency <= 0.001:
            # Centre the tail when propulsion is stopped.
            theta = 0.0

        else:
            # Integrate phase without discontinuities when the
            # frequency changes.
            self.phase += (
                2.0
                * math.pi
                * self.frequency
                * dt
            )

            self.phase %= 2.0 * math.pi

            # Keep the complete oscillation inside the joint limits.
            available_amplitude = (
                self.safe_joint_limit
                - abs(self.tail_bias)
            )

            amplitude = min(
                self.normal_amplitude,
                available_amplitude
            )

            theta = (
                self.tail_bias
                + amplitude * math.sin(self.phase)
            )

        msg = Float64MultiArray()
        msg.data = [theta]

        self.tail_publisher.publish(msg)

    def centre_tail(self):
        """Return the tail to zero before shutting down."""

        msg = Float64MultiArray()
        msg.data = [0.0]

        self.tail_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = FishSwimmer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.centre_tail()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()