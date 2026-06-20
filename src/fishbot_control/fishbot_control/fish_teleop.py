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

        # Toggle states
        self.left_fin_state = 1
        self.right_fin_state = 1

        self.get_logger().info("Fish Teleop Started")

    def publish_speed(self):

        msg = Float64()
        msg.data = self.speed

        self.speed_pub.publish(msg)

    def publish_fins(self, left, right):

        left_msg = Float64MultiArray()
        left_msg.data = [left]

        right_msg = Float64MultiArray()
        right_msg.data = [right]

        self.left_fin_pub.publish(left_msg)
        self.right_fin_pub.publish(right_msg)

    def speed_control(self):

        # Swimming mode
        self.publish_fins(0.0, 0.0)
        self.publish_speed()

    def stop(self):

        self.speed = 0.0

        self.publish_speed()

        # Vertical fins for drag
        self.publish_fins(0.3, 0.3)

    def turn_right(self):

        # Toggle left fin
        self.left_fin_state *= -1

        self.publish_fins(
            0.3 * self.left_fin_state,
            0.0
        )

    def turn_left(self):

        # Toggle right fin
        self.right_fin_state *= -1

        self.publish_fins(
            0.0,
            0.3 * self.right_fin_state
        )

    def pitch(self):

        self.left_fin_state *= -1
        self.right_fin_state *= -1

        self.publish_fins(
            0.3 * self.left_fin_state,
            0.3 * self.right_fin_state
        )

    def roll_clockwise(self):

        self.publish_fins(
            0.3,
            -0.3
        )

    def roll_anticlockwise(self):

        self.publish_fins(
            -0.3,
            0.3
        )


def get_key():

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)

    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings
        )

    return key


def main(args=None):

    rclpy.init(args=args)

    node = FishTeleop()

    print(
        "\n"
        "W = Faster\n"
        "S = Slower\n"
        "A = Turn Left (toggle right fin)\n"
        "D = Turn Right (toggle left fin)\n"
        "P = Pitch\n"
        "E = Roll Clockwise\n"
        "R = Roll Anti-Clockwise\n"
        "SPACE = Stop\n"
        "Q = Quit\n"
    )

    try:

        while True:

            key = get_key()

            if key == 'w':

                node.speed += 0.2

                node.speed_control()

                print(f"Speed: {node.speed:.1f}")

            elif key == 's':

                node.speed = max(
                    0.0,
                    node.speed - 0.2
                )

                node.speed_control()

                print(f"Speed: {node.speed:.1f}")

            elif key == 'a':

                node.turn_left()

                print("Turn Left")

            elif key == 'd':

                node.turn_right()

                print("Turn Right")

            elif key == 'p':

                node.pitch()

                print("Pitch")

            elif key == 'e':

                node.roll_clockwise()

                print("Roll Clockwise")

            elif key == 'r':

                node.roll_anticlockwise()

                print("Roll Anti-Clockwise")

            elif key == ' ':

                node.stop()

                print("Stopped")

            elif key == 'q':

                break

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()