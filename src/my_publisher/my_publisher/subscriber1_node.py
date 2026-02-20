import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class SubscriberNode(Node):

    def __init__(self):
        super().__init__('subscriber_node')

        self.subscription = self.create_subscription(Int32,'random_number',self.listener_callback,10)
        self.count = 0 
        self.get_logger().info("Subscriber node has started")

    def listener_callback(self, msg):
        self.get_logger().info(f"number recieve ho rhe h: {msg.data}")
        self.count += 1 


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
