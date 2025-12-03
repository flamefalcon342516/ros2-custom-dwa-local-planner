#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MotionTest(Node):
    def __init__(self):
        super().__init__('motion_test_controller')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.duration =5.4  
        self.start_time = time.time()
        self.state = 0   
        self.timer = self.create_timer(0.1, self.control_loop)  

    def control_loop(self):
        now = time.time()
        elapsed = now - self.start_time
        msg = Twist()

        # --- STATE MACHINE ---
        if self.state == 0:
            # Forward
            msg.linear.x = 0.0
            msg.angular.z = 2.0
            self.get_logger().info("some Forward")
            if elapsed > self.duration:
                self.state = 1
                self.start_time = now

        elif self.state == 1:
            # Forward + turn
            msg.linear.x = 0.0
            msg.angular.z = 1.0
            self.get_logger().info("Turn")
            if elapsed > self.duration:
                self.state = 2
                self.start_time = now

        elif self.state == 2:
            # Reverse
            msg.linear.x = 0.2
            msg.angular.z = 0.0
            self.get_logger().info("front")
            if elapsed > self.duration:
                self.state = 0
                self.start_time = now

        # Publish twist command
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
