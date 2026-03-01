#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class LidarWallFollow(Node):
    """
    Subscribes to /scan_fullframe LaserScan.
    Computes (v,w) then converts to wheel angular velocity (rad/s)
    and publishes to /left_motor and /right_motor (std_msgs/Float64).

    Always follows the LEFT wall:
      - left wall uses sector at +90 deg
      - positive error (too far) -> positive w (turn left)
    """

    def __init__(self):
        super().__init__('lidar_wallfollow_left')

        # Control params
        self.declare_parameter('desired_wall_dist', 0.6)     # meters
        self.declare_parameter('k_p', 1.2)                   # steering gain
        self.declare_parameter('v_nom', 0.25)                # m/s
        self.declare_parameter('w_max', 1.2)                 # rad/s
        self.declare_parameter('stop_front_dist', 0.35)      # meters
        self.declare_parameter('sector_width_deg', 6.0)      # sector width

        # Robot geometry for converting (v,w) -> wheel rad/s
        self.declare_parameter('wheel_radius', 0.05)         # meters (TUNE)
        self.declare_parameter('wheel_base', 0.30)           # meters (TUNE)

        # Optional limits on wheel angular speed
        self.declare_parameter('max_wheel_rad_s', 20.0)       # rad/s

        self.sub = self.create_subscription(
            LaserScan,
            '/scan_fullframe',
            self.cb_scan,
            10
        )

        self.pub_left = self.create_publisher(Float64, '/left_motor/vel_radps', 10)
        self.pub_right = self.create_publisher(Float64, '/right_motor/vel_radps', 10)

        self.get_logger().info(
            "Subscribed to /scan_fullframe. Following LEFT wall. "
            "Publishing wheel commands to /left_motor/vel_radps and /right_motor/vel_radps."
        )

    def angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
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
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                if r < best:
                    best = r
        return best

    def publish_wheel_cmds(self, wl: float, wr: float):
        msg_l = Float64()
        msg_r = Float64()
        msg_l.data = float(wl)
        msg_r.data = float(wr)
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

    def cb_scan(self, scan: LaserScan):
        desired_wall_dist = float(self.get_parameter('desired_wall_dist').value)
        k_p = float(self.get_parameter('k_p').value)
        v_nom = float(self.get_parameter('v_nom').value)
        w_max = float(self.get_parameter('w_max').value)
        stop_front_dist = float(self.get_parameter('stop_front_dist').value)
        sector_width_deg = float(self.get_parameter('sector_width_deg').value)

        wheel_radius = float(self.get_parameter('wheel_radius').value)
        wheel_base = float(self.get_parameter('wheel_base').value)
        max_wheel = float(self.get_parameter('max_wheel_rad_s').value)

        # Probe sectors
        d_front = self.sector_min(scan, math.radians(0.0), sector_width_deg)
        d_wall = self.sector_min(scan, math.radians(+90.0), sector_width_deg)  # LEFT wall

        # ---- WALL DETECTION LOGIC ----
        wall_threshold = 2.0  # meters (tune this)
        if math.isfinite(d_wall) and d_wall < wall_threshold:
            wall_msg = f"Wall detected at distance {d_wall:.2f} m (left side)"
        else:
            wall_msg = "No wall detected on left"

        if math.isfinite(d_front) and d_front < stop_front_dist:
            front_msg = f"Obstacle ahead at {d_front:.2f} m"
        else:
            front_msg = "Front clear"

        # Validate
        if not math.isfinite(d_wall) or not math.isfinite(d_front):
            self.get_logger().warn("No valid scan data in key sectors.")
            return

        if wheel_radius <= 0.0 or wheel_base <= 0.0:
            self.get_logger().error("wheel_radius and wheel_base must be > 0 to convert to rad/s.")
            return

        # ---- WALL FOLLOW CONTROL (v,w) ----
        # error is positive if we're too far from wall
        error = d_wall - desired_wall_dist

        # LEFT wall: turning left is positive w, so w = +k_p * error
        w = +k_p * error
        w = clamp(w, -w_max, w_max)

        v = v_nom

        # Obstacle avoidance: if something ahead, stop and turn away from the LEFT wall (turn right => negative w)
        if d_front < stop_front_dist:
            v = 0.0
            w = -0.6

        # ---- Convert (v,w) -> wheel angular velocities (rad/s) ----
        v_left = v - (w * wheel_base / 2.0)
        v_right = v + (w * wheel_base / 2.0)

        wl = v_left / wheel_radius
        wr = v_right / wheel_radius

        # Optional clamp
        wl = clamp(wl, -max_wheel, max_wheel)
        wr = clamp(wr, -max_wheel, max_wheel)

        # l sign flip
        self.publish_wheel_cmds(-1 * wl, -1 * wr)

        # ---- Log ----
        self.get_logger().info(
            f"{wall_msg} | {front_msg} | follow_side=left_only\n"
            f"Ranges: front={d_front:.2f}  wall={d_wall:.2f} | "
            f"error={error:+.2f} -> v={v:.2f}, w={w:+.2f} | "
            f"cmd_rad_s: left={wl:+.2f}, right={wr:+.2f}"
        )


def main():
    rclpy.init()
    node = LidarWallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()