#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import time
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class DWAController(Node):
    def __init__(self):
        super().__init__("dwa_controller")
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.pc_sub = self.create_subscription(PointCloud2, "/depth_camera/points", self.pc_callback, 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/dwa_path", qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/dwa_trajectories", qos)

        self.x = -9.0
        self.y = 7.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0

        self.goals = [
            np.array([0.0, 2.0]),
            np.array([2.0, -4.0]),
            np.array([0.0, -7.0]),
            np.array([-3.0, -7.0]),
            np.array([-8.0, -4.0])
        ]
        self.goal_index = 0
        self.goal = self.goals[self.goal_index]
        self.obstacles = []

        self.max_v = 1.6
        self.max_w = 3.0
        self.max_acc = 0.6
        self.max_dyaw = 3.0
        self.dt = 0.1
        self.predict_time = 1.0

        self.heading_w = 0.6
        self.clearance_w = 0.6
        self.velocity_w = 0.4

        self.path_msg = Path()
        self.path_msg.header.frame_id = "camera_link"

        self.timer = self.create_timer(0.3, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def pc_callback(self, msg):
        points = list(point_cloud2.read_points(msg, skip_nans=True, field_names=("x","y","z")))
        if len(points) > 2000:
            idx = np.random.choice(len(points), 2000, replace=False)
            points = [points[i] for i in idx]
        self.obstacles = []
        for x, y, z in points:
            if 0.4 < z < 1.3 and abs(x) < 4.0 and abs(y) < 3.0:
                self.obstacles.append([x, y])

    def dynamic_window(self):
        v_min = max(0.0, self.v - self.max_acc * self.dt)
        v_max = min(self.max_v, self.v + self.max_acc * self.dt)
        w_min = max(-self.max_w, self.w - self.max_dyaw * self.dt)
        w_max = min(self.max_w, self.w + self.max_dyaw * self.dt)
        return v_min, v_max, w_min, w_max

    def predict_traj(self, v, w):
        x, y, yaw = 0.0, 0.0, 0.0
        traj = []
        for _ in range(int(self.predict_time / self.dt)):
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            traj.append([x, y])
        return np.array(traj)

    def heading_cost(self, traj):
        last = traj[-1]
        dx = self.goal[0] - (self.x + last[0])
        dy = self.goal[1] - (self.y + last[1])
        desired = math.atan2(dy, dx)
        traj_yaw = math.atan2(last[1], last[0])
        return math.pi - abs(desired - traj_yaw)

    def obstacle_cost(self, traj):
        if len(self.obstacles) == 0:
            return 1.0
        min_dist = 999
        for px, py in traj:
            for ox, oy in self.obstacles:
                d = math.sqrt((px - ox)**2 + (py - oy)**2)
                min_dist = min(min_dist, d)
        return min_dist

    def control_loop(self):
        pos = np.array([self.x, self.y])
        if np.linalg.norm(pos - self.goal) < 0.5:
            self.publish(0.0, 0.0)
            time.sleep(0.3)
            if self.goal_index < len(self.goals) - 1:
                self.goal_index += 1
                self.goal = self.goals[self.goal_index]
                return
            else:
                return

        if len(self.obstacles) == 0:
            dx, dy = self.goal - pos
            desired = math.atan2(dy, dx)
            err = math.atan2(math.sin(desired - self.yaw), math.cos(desired - self.yaw))
            self.publish(0.6, 1.8 * err)
            return

        v_min, v_max, w_min, w_max = self.dynamic_window()
        best_score, best_v, best_w, best_traj = -999, 0, 0, None

        marker_array = MarkerArray()
        marker_id = 0

        for v in np.linspace(v_min, v_max, 6):
            for w in np.linspace(w_min, w_max, 6):
                traj = self.predict_traj(v, w)

                marker = Marker()
                marker.header.frame_id = "camera_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "dwa_traj"
                marker.id = marker_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.02
                marker.color.a = 0.3
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0

                for px, py in traj:
                    p = PoseStamped().pose.position
                    p.x = self.x + px
                    p.y = self.y + py
                    p.z = 0.01
                    marker.points.append(p)

                marker_array.markers.append(marker)
                marker_id += 1

                score = (self.heading_w * self.heading_cost(traj) +
                         self.clearance_w * self.obstacle_cost(traj) +
                         self.velocity_w * v)

                if score > best_score:
                    best_score, best_v, best_w, best_traj = score, v, w, traj

        self.marker_pub.publish(marker_array)
        self.publish(best_v, best_w)

        pose = PoseStamped()
        pose.header.frame_id = "camera_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.w = 1.0

        self.path_msg.header.frame_id = "camera_link"
        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

    def publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DWAController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
