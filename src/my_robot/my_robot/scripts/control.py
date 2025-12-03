#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, time

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def getKey(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        print("Use W A S D keys (0.2s pulse). X to stop. Q to quit.")

        while True:
            key = self.getKey()
            msg = Twist()

            if key == 'w':
                msg.linear.x = 0.5
            elif key == 's':
                msg.linear.x = -0.5
            elif key == 'a':
                msg.angular.z = 1.0
            elif key == 'd':
                msg.angular.z = -1.0
            elif key == 'x':
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            elif key == 'q':
                break
            else:
                continue

            # Publish move command
            self.pub.publish(msg)
            time.sleep(0.2)

            # Send stop command
            stop = Twist()
            self.pub.publish(stop)

def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
