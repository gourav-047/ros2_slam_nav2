import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import random 


class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher_node')

        self.publisher_ = self.create_publisher(Int32,'random_number',10)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Random number Publish shuru hua")
        self.count = 0 

    def timer_callback(self):
        msg = Int32()
        msg.data = random.randint(0,100) 
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.count += 1 


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
