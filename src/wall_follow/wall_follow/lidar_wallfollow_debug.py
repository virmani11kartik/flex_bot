#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class LidarWallFollowDebug(Node):
    """
    Subscribes to /scan_fullframe LaserScan.
    Reads a few angle sectors and prints suggested (v, w) to terminal.

    Assumes:
      - LaserScan angles are in radians
      - angle_min..angle_max span ~270 degrees for a Sick 2D lidar
      - 0 rad is "front" in the scan frame (typical for many setups)
    """

    def __init__(self):
        super().__init__('lidar_wallfollow_debug')

        self.declare_parameter('desired_wall_dist', 0.6)     # meters
        self.declare_parameter('k_p', 1.2)                   # steering gain
        self.declare_parameter('v_nom', 0.25)                # m/s (suggested)
        self.declare_parameter('w_max', 1.2)                 # rad/s (suggested)
        self.declare_parameter('stop_front_dist', 0.35)      # meters
        self.declare_parameter('sector_width_deg', 6.0)      # how wide each sector is

        self.sub = self.create_subscription(
            LaserScan,
            '/scan_fullframe',
            self.cb_scan,
            10
        )

        self.get_logger().info("Subscribed to /scan_fullframe. Printing suggested velocity commands...")

    def angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        # Convert an angle to the closest index in the scan array
        idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
        return clamp(idx, 0, len(scan.ranges) - 1)

    def sector_min(self, scan: LaserScan, center_angle_rad: float, width_deg: float) -> float:
        half = math.radians(width_deg) / 2.0
        a0 = center_angle_rad - half
        a1 = center_angle_rad + half

        i0 = self.angle_to_index(scan, a0)
        i1 = self.angle_to_index(scan, a1)
        if i1 < i0:
            i0, i1 = i1, i0

        best = float('inf')
        for i in range(i0, i1 + 1):
            r = scan.ranges[i]
            # Filter invalids
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                if r < best:
                    best = r
        return best

    def cb_scan(self, scan: LaserScan):
        desired_wall_dist = float(self.get_parameter('desired_wall_dist').value)
        k_p = float(self.get_parameter('k_p').value)
        v_nom = float(self.get_parameter('v_nom').value)
        w_max = float(self.get_parameter('w_max').value)
        stop_front_dist = float(self.get_parameter('stop_front_dist').value)
        sector_width_deg = float(self.get_parameter('sector_width_deg').value)

        # Probe sectors
        d_front = self.sector_min(scan, math.radians(0.0), sector_width_deg)
        d_frontr = self.sector_min(scan, math.radians(-45.0), sector_width_deg)
        d_right = self.sector_min(scan, math.radians(-90.0), sector_width_deg)

        # ---- WALL DETECTION LOGIC ----
        wall_threshold = 2.0  # meters (tune this)

        if math.isfinite(d_right) and d_right < wall_threshold:
            wall_msg = f"Wall detected at distance {d_right:.2f} m (right side)"
        else:
            wall_msg = "No wall detected on right"

        if math.isfinite(d_front) and d_front < stop_front_dist:
            front_msg = f"Obstacle ahead at {d_front:.2f} m"
        else:
            front_msg = "Front clear"

        # If no valid right or front readings, skip control
        if not math.isfinite(d_right) or not math.isfinite(d_front):
            self.get_logger().warn("No valid scan data in key sectors.")
            return

        # ---- WALL FOLLOW CONTROL ----
        error = d_right - desired_wall_dist
        w = -k_p * error
        w = clamp(w, -w_max, w_max)

        v = v_nom
        if d_front < stop_front_dist:
            v = 0.0
            w = +0.6  # turn left

        # ---- PRINT EVERYTHING CLEANLY ----
        self.get_logger().info(
            f"{wall_msg} | {front_msg}\n"
            f"Ranges: front={d_front:.2f}  right={d_right:.2f} | "
            f"error={error:+.2f} -> suggest v={v:.2f}, w={w:+.2f}"
        )



def main():
    rclpy.init()
    node = LidarWallFollowDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
