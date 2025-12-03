#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Twist
import math
import time
import matplotlib.pyplot as plt

plt.ion()  # enable interactive real-time plotting


class APFAController(Node):
    def __init__(self):
        super().__init__('apfa_controller')

        # Subscribers & Publishers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pc_sub = self.create_subscription(PointCloud2, '/depth_camera/points', self.pointcloud_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Robot state
        self.robot_x = -9.0
        self.robot_y = 7.0
        self.yaw = 0.0

        # Waypoints
        self.goals = [np.array([0.0, 2.0]),
                      np.array([2.0, -4.0]),
                      np.array([0.0, -7.0]),
                      np.array([-3.0, -7.0]),
                      np.array([-8.0, -4.0])]

        self.goal_index = 0
        self.goal = self.goals[self.goal_index]

        # APF parameters
        self.k_att = 1.5
        self.k_rep = 0.8
        self.influence_dist = 3.5
        self.obstacles = []

        # Velocity limits
        self.max_linear_vel = 0.7
        self.max_angular_vel = 2.5

        # Real-time plot data storage
        self.path_x = []
        self.path_y = []
        self.heading_error_log = []

        # ---------- Real-Time Plot Initialization ----------
        self.fig, self.ax = plt.subplots()
        self.robot_path_line, = self.ax.plot([], [], label="Robot Path")
        self.ax.scatter([g[0] for g in self.goals], [g[1] for g in self.goals], c='red', label="Goal Points")
        self.ax.set_xlim(-12, 5)
        self.ax.set_ylim(-10, 10)
        self.ax.set_title("Real-Time Path Tracking")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.grid(True)
        self.ax.legend()

        self.fig2, self.ax2 = plt.subplots()
        self.heading_err_line, = self.ax2.plot([], [], label="Heading Error")
        self.ax2.set_title("Heading Error Over Time")
        self.ax2.set_xlabel("Step")
        self.ax2.set_ylabel("Error (rad)")
        self.ax2.grid(True)
        self.ax2.legend()

        # Control loop timer
        self.timer = self.create_timer(0.05, self.control_loop)

    # ---------------- ODOM CALLBACK ----------------
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(q)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ---------------- POINT CLOUD CALLBACK ----------------
    def pointcloud_callback(self, cloud_msg):
        points = list(point_cloud2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")))

        if len(points) > 2000:
            indices = np.random.choice(len(points), 2000, replace=False)
            points = [points[i] for i in indices]

        self.obstacles = []
        for x, y, z in points:
            if 0.6 < z < 1.2 and abs(x) < 4.0 and abs(y) < 3.0:
                self.obstacles.append([x, y])

    # ---------------- CONTROL LOOP ----------------
    def control_loop(self):
        pos = np.array([self.robot_x, self.robot_y])
        diff = self.goal - pos
        dist = np.linalg.norm(diff)

        # Goal reached
        if dist < 0.5:
            self.publish(0.0, 0.0)
            self.get_logger().info(f"Goal reached: {self.goal_index + 1}")
            time.sleep(0.2)
            if self.goal_index < len(self.goals) - 1:
                self.goal_index += 1
                self.goal = self.goals[self.goal_index]
                self.get_logger().info(f"NEXT GOAL: {self.goal}")
            else:
                self.get_logger().info("ALL GOALS REACHED")
                return

        # Heading control
        desired_heading = math.atan2(diff[1], diff[0])
        heading_err = desired_heading - self.yaw
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

        print("moving to goal:", self.goal)

        # Log heading error for plot
        self.heading_error_log.append(heading_err)

        # ---- CLEAR PATH CASE ----
        if len(self.obstacles) == 0:
            if heading_err > 0.05:
                lin_vel, ang_vel = 0.8, 1.5
            elif heading_err < -0.05:
                lin_vel, ang_vel = 0.8, -1.5
            else:
                lin_vel, ang_vel = 0.8, 0.0

            self.publish(lin_vel, ang_vel)
        else:
            # ---- OBSTACLE AVOIDANCE USING APF ----
            F_att = np.array([math.cos(heading_err), math.sin(heading_err)]) * self.k_att
            F_rep = np.array([0.0, 0.0])
            for obs in self.obstacles:
                obs = np.array(obs)
                d = np.linalg.norm(obs)
                if 0.2 < d < self.influence_dist:
                    rep = self.k_rep * ((1/d - 1/self.influence_dist) * (1/(d*d))) * (obs / d)
                    F_rep += rep

            F = F_att - F_rep
            if np.linalg.norm(F) > 0:
                F /= np.linalg.norm(F)

            angle_F = math.atan2(F[1], F[0])
            heading_err = angle_F - self.yaw
            heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

            ang_vel = -1 * np.clip(1.5 * heading_err, -self.max_angular_vel, self.max_angular_vel)
            lin_vel = np.clip(np.linalg.norm(F) * 0.25, 0.0, self.max_linear_vel)

            print("obstacles detected, avoiding...")
            self.publish(lin_vel, ang_vel)

        # ---------- Real-Time Plot Update ----------
        self.path_x.append(self.robot_x)
        self.path_y.append(self.robot_y)

        self.robot_path_line.set_xdata(self.path_x)
        self.robot_path_line.set_ydata(self.path_y)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        self.heading_err_line.set_xdata(range(len(self.heading_error_log)))
        self.heading_err_line.set_ydata(self.heading_error_log)
        self.ax2.relim()
        self.ax2.autoscale_view()
        self.fig2.canvas.draw()
        self.fig2.canvas.flush_events()

    # ---------------- PUBLISH ----------------
    def publish(self, lin, ang):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = APFAController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
