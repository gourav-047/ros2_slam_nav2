import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SubscriberNode(Node):

    def __init__(self):
        super().__init__('Subsriber_node')

        self.subscription = self.create_subscription(Float32,'random_number',self.listener_callback,10)
        self.count = 0
        self.get_logger().info('Subscriber hoga')
    
    def listener_callback (self,msg):
        self.get_logger().info(f"subscribe ho rha h : {msg.data}")
        self.count += 1


def main(args=None) :
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
           