import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SwimCommandPublisher(Node):

    def __init__(self):
        super().__init__('swim_command_publisher')

        self.publisher_ = self.create_publisher(
            String,
            'swim_command',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_command
        )

    def publish_command(self):

        msg = String()
        msg.data = "FORWARD"

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Publishing: {msg.data}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = SwimCommandPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()