import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SwimCommandListener(Node):

    def __init__(self):
        super().__init__('swim_command_listener')

        self.subscription = self.create_subscription(
            String,
            'swim_command',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

def main(args=None):
    rclpy.init(args=args)

    node = SwimCommandListener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()