#!/usr/bin/env python3

import sys
import tty
import termios

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


class FishTeleop(Node):

    def __init__(self):
        super().__init__('fish_teleop')

        self.speed_pub = self.create_publisher(
            Float64,
            '/fish_speed',
            10
        )

        self.turn_pub = self.create_publisher(
            Float64,
            '/fish_turn',
            10
        )

        self.left_fin_pub = self.create_publisher(
            Float64MultiArray,
            '/left_fin_controller/commands',
            10
        )

        self.right_fin_pub = self.create_publisher(
            Float64MultiArray,
            '/right_fin_controller/commands',
            10
        )

        self.speed = 1.0
        self.turn_bias = 0.0
        self.pitch_angle = 0.0

        self.get_logger().info(
            'Fish teleop started'
        )

    def publish_speed(self):
        msg = Float64()
        msg.data = self.speed
        self.speed_pub.publish(msg)

    def publish_turn(self):
        msg = Float64()
        msg.data = self.turn_bias
        self.turn_pub.publish(msg)

    def publish_fins(self):
        left_msg = Float64MultiArray()
        left_msg.data = [self.pitch_angle]

        right_msg = Float64MultiArray()
        right_msg.data = [self.pitch_angle]

        self.left_fin_pub.publish(left_msg)
        self.right_fin_pub.publish(right_msg)

    def publish_all(self):
        self.publish_speed()
        self.publish_turn()
        self.publish_fins()

    def faster(self):
        self.speed = min(
            1.2,
            self.speed + 0.1
        )

        self.publish_speed()

    def slower(self):
        self.speed = max(
            0.0,
            self.speed - 0.1
        )

        self.publish_speed()

    def turn_left(self):
        self.turn_bias = -0.2
        self.publish_turn()

    def turn_right(self):
        self.turn_bias = 0.2
        self.publish_turn()

    def swim_straight(self):
        self.turn_bias = 0.0
        self.publish_turn()

    def pitch_up(self):
        self.pitch_angle = 0.20
        self.publish_fins()

    def pitch_down(self):
        self.pitch_angle = -0.20
        self.publish_fins()

    def neutral_fins(self):
        self.pitch_angle = 0.0
        self.publish_fins()

    def stop(self):
        self.speed = 0.0
        self.turn_bias = 0.0
        self.pitch_angle = 0.0

        self.publish_all()


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        return sys.stdin.read(1)

    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings
        )


def main(args=None):
    rclpy.init(args=args)

    node = FishTeleop()
    node.publish_all()

    print(
        "\n"
        "W = Faster\n"
        "S = Slower\n"
        "A = Turn Left\n"
        "D = Turn Right\n"
        "X = Straighten Tail\n"
        "I = Pitch Up\n"
        "K = Pitch Down\n"
        "O = Neutral Pectoral Fins\n"
        "SPACE = Stop\n"
        "Q = Quit\n"
    )

    try:
        while True:
            key = get_key().lower()

            if key == 'w':
                node.faster()
                print(f"Speed: {node.speed:.1f} Hz")

            elif key == 's':
                node.slower()
                print(f"Speed: {node.speed:.1f} Hz")

            elif key == 'a':
                node.turn_left()
                print("Turning left")

            elif key == 'd':
                node.turn_right()
                print("Turning right")

            elif key == 'x':
                node.swim_straight()
                print("Tail centred")

            elif key == 'i':
                node.pitch_up()
                print("Pitching up")

            elif key == 'k':
                node.pitch_down()
                print("Pitching down")

            elif key == 'o':
                node.neutral_fins()
                print("Pectoral fins neutral")

            elif key == ' ':
                node.stop()
                print("Stopped")

            elif key == 'q':
                node.stop()
                break

    except KeyboardInterrupt:
        node.stop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()