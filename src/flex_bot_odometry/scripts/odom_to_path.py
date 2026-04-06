#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomToPath(Node):
    def __init__(self):
        super().__init__("odom_to_path_simple")

        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("path_topic", "/wheel/path")
        self.declare_parameter("max_poses", 5000)
        self.declare_parameter("min_dist", 0.0)
        self.declare_parameter("min_yaw", 0.0)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.max_poses  = int(self.get_parameter("max_poses").value)
        self.min_dist   = float(self.get_parameter("min_dist").value)
        self.min_yaw    = float(self.get_parameter("min_yaw").value)

        self.pub = self.create_publisher(Path, self.path_topic, 10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.cb, 10)

        self.path = Path()
        self.last = None

        self.get_logger().info(f"SUB  odom: {self.odom_topic}")
        self.get_logger().info(f"PUB path: {self.path_topic}")

    @staticmethod
    def yaw_from_quat(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def ang_diff(a, b):
        d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
        return abs(d)

    def cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.yaw_from_quat(msg.pose.pose.orientation)

        if self.last is not None:
            lx, ly, lyaw = self.last
            if math.hypot(x - lx, y - ly) < self.min_dist and self.ang_diff(yaw, lyaw) < self.min_yaw:
                return

        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose

        self.path.header = msg.header
        self.path.poses.append(ps)
        if len(self.path.poses) > self.max_poses:
            self.path.poses.pop(0)

        self.pub.publish(self.path)
        self.last = (x, y, yaw)

        # Debug print occasionally
        if len(self.path.poses) % 20 == 0:
            self.get_logger().info(f"Path poses: {len(self.path.poses)}  last=({x:.3f},{y:.3f}) yaw={yaw:.3f}")

def main():
    rclpy.init()
    node = OdomToPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
