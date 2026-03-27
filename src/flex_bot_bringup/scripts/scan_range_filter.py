# new node: scan_range_filter.py
# subscribes to /scan_fullframe
# publishes /scan_filtered with ranges > 2.0m set to inf
# obstacle_extractor subscribes to /scan_filtered instead

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanRangeFilter(Node):
    def __init__(self):
        super().__init__('scan_range_filter')
        self.max_range_ = self.declare_parameter('max_range', 2.0).value
        self.sub_ = self.create_subscription(
            LaserScan, '/scan_fullframe', self.cb, 10)
        self.pub_ = self.create_publisher(
            LaserScan, '/scan_filtered', 10)

    def cb(self, msg):
        filtered = LaserScan()
        filtered.header         = msg.header
        filtered.angle_min      = msg.angle_min
        filtered.angle_max      = msg.angle_max
        filtered.angle_increment= msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time      = msg.scan_time
        filtered.range_min      = msg.range_min
        filtered.range_max      = self.max_range_
        filtered.ranges = [
            r if (msg.range_min < r < self.max_range_) else float('inf')
            for r in msg.ranges
        ]
        filtered.intensities = msg.intensities
        self.pub_.publish(filtered)

def main():
    rclpy.init()
    rclpy.spin(ScanRangeFilter())
    rclpy.shutdown()

if __name__ == '__main__':
    main()