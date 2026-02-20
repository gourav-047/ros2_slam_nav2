#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty


class SimpleTeleop(Node):

    def __init__(self):
        super().__init__('simple_teleop')

        # Publisher to cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Speed settings
        self.linear_speed = 0.2     # m/s
        self.angular_speed = 0.8    # rad/s

        print("\n=== SIMPLE TELEOP (GAZEBO) ===")
        print("W : Forward")
        print("S : Backward")
        print("A : Turn Left")
        print("D : Turn Right")
        print("Space : Stop")
        print("Q : Quit")
        print("==============================\n")

        self.keyboard_loop()

    def keyboard_loop(self):
        settings = termios.tcgetattr(sys.stdin)

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)

                tty.setraw(sys.stdin.fileno())
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                key = sys.stdin.read(1) if rlist else ''
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

                cmd = Twist()

                if key == 'q':
                    break
                elif key == 'w':
                    cmd.linear.x = self.linear_speed
                elif key == 's':
                    cmd.linear.x = -self.linear_speed
                elif key == 'a':
                    cmd.angular.z = self.angular_speed
                elif key == 'd':
                    cmd.angular.z = -self.angular_speed
                elif key == ' ':
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                else:
                    continue

                self.cmd_pub.publish(cmd)

        finally:
            self.cmd_pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    rclpy.init()
    node = SimpleTeleop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
