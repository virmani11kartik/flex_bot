#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class WallFollow(Node):

    MODE_FOLLOW = 0
    MODE_TURN_LEFT = 1

    def __init__(self):
        super().__init__("wall_follow")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("scan_topic", "/scan_fullframe")
        self.declare_parameter("follow_side", "left")  # keep "left" for this behavior

        self.declare_parameter("desired_dist", 0.50)   # m
        self.declare_parameter("lookahead", 0.60)      # m

        self.declare_parameter("v_nom", 0.08)          # m/s
        self.declare_parameter("v_min", 0.04)          # m/s
        self.declare_parameter("omega_max", 1.0)       # rad/s

        self.declare_parameter("front_stop_dist", 0.45)  # m (enter corner turn)
        self.declare_parameter("front_slow_dist", 0.70)  # m

        # Robot geometry
        self.declare_parameter("wheel_radius", 0.0765)   # m
        self.declare_parameter("wheel_base", 0.5000)     # m

        # PID
        self.declare_parameter("kp", 1.3)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.18)

        # Beams
        self.declare_parameter("beam_deg_a", 60.0)
        self.declare_parameter("beam_deg_b", 90.0)
        self.declare_parameter("beam_window_deg", 2.0)
        self.declare_parameter("front_window_deg", 10.0)

        # Output direction / mapping fixes
        self.declare_parameter("invert_forward", True)
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)
        self.declare_parameter("swap_wheels", False)

        # Shutdown safety
        self.declare_parameter("stop_on_shutdown", True)
        self.declare_parameter("stop_publish_count", 10)
        self.declare_parameter("stop_publish_dt", 0.02)
        self.declare_parameter("shutdown_flush_s", 0.15)
        self.declare_parameter("scan_timeout", 0.25)

        self.declare_parameter("enable_corner_turn", True)

        # When corner detected
        self.declare_parameter("corner_turn_omega", 0.7)   # rad/s
        self.declare_parameter("corner_turn_v", 0.00)      # m/s 

        self.declare_parameter("corner_clear_dist", 0.45)      # m
        self.declare_parameter("left_reacquire_dist", 0.8)    # m

        # Minimum time to stay in turn mode
        self.declare_parameter("corner_hold_s", 0.40)

        self.scan_topic = str(self.get_parameter("scan_topic").value)

        # QoS (match bridge)
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Pub/Sub
        self.pub_l = self.create_publisher(Float64, "/left_wheel/cmd_vel", self.qos)
        self.pub_r = self.create_publisher(Float64, "/right_wheel/cmd_vel", self.qos)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, self.qos)

        # Watchdog
        self.last_scan_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.05, self._watchdog)

        # PID state
        self.e_int = 0.0
        self.e_prev = 0.0
        self.t_prev = self.get_clock().now()

        # Mode state
        self.mode = self.MODE_FOLLOW
        self.mode_enter_time = self.get_clock().now()

        self.get_logger().info(
            f"[wall_follow] Subscribed: {self.scan_topic} | mode=FOLLOW | keep-left-wall with corner turning"
        )

    def stop_robot(self):
        if not bool(self.get_parameter("stop_on_shutdown").value):
            return
        n = int(self.get_parameter("stop_publish_count").value)
        dt = float(self.get_parameter("stop_publish_dt").value)
        ml = Float64(); mr = Float64()
        ml.data = 0.0; mr.data = 0.0
        for _ in range(max(1, n)):
            self.pub_l.publish(ml)
            self.pub_r.publish(mr)
            time.sleep(max(0.0, dt))

    def _watchdog(self):
        timeout = float(self.get_parameter("scan_timeout").value)
        age = (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9
        if age > timeout:
            self.stop_robot()

    def _min_range_in_sector(self, scan: LaserScan, center_rad: float, half_width_rad: float) -> float:
        a0 = max(center_rad - half_width_rad, scan.angle_min)
        a1 = min(center_rad + half_width_rad, scan.angle_max)
        if a1 <= a0:
            return float("inf")

        i0 = int((a0 - scan.angle_min) / scan.angle_increment)
        i1 = int((a1 - scan.angle_min) / scan.angle_increment)
        i0 = int(clamp(i0, 0, len(scan.ranges) - 1))
        i1 = int(clamp(i1, 0, len(scan.ranges) - 1))
        if i1 < i0:
            i0, i1 = i1, i0

        best = float("inf")
        for i in range(i0, i1 + 1):
            r = scan.ranges[i]
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                best = min(best, r)
        return best

    def _publish_wheels(self, v: float, omega: float, r_wheel: float, wheel_base: float):
        if bool(self.get_parameter("invert_forward").value):
            v = -v

        v_l = v - omega * (wheel_base / 2.0)
        v_r = v + omega * (wheel_base / 2.0)

        w_l = v_l / max(1e-6, r_wheel)
        w_r = v_r / max(1e-6, r_wheel)

        if bool(self.get_parameter("swap_wheels").value):
            w_l, w_r = w_r, w_l
        if bool(self.get_parameter("invert_left").value):
            w_l = -w_l
        if bool(self.get_parameter("invert_right").value):
            w_r = -w_r

        ml = Float64(); mr = Float64()
        ml.data = float(w_l)
        mr.data = float(w_r)
        self.pub_l.publish(ml)
        self.pub_r.publish(mr)

    def on_scan(self, scan: LaserScan):
        self.last_scan_time = self.get_clock().now()

        # We implement "follow left" + corner turn left
        side = "left"

        desired = float(self.get_parameter("desired_dist").value)
        L = float(self.get_parameter("lookahead").value)

        v_nom = float(self.get_parameter("v_nom").value)
        v_min = float(self.get_parameter("v_min").value)
        omega_max = float(self.get_parameter("omega_max").value)

        front_stop = float(self.get_parameter("front_stop_dist").value)
        front_slow = float(self.get_parameter("front_slow_dist").value)

        r_wheel = float(self.get_parameter("wheel_radius").value)
        wheel_base = float(self.get_parameter("wheel_base").value)

        kp = float(self.get_parameter("kp").value)
        ki = float(self.get_parameter("ki").value)
        kd = float(self.get_parameter("kd").value)

        beam_deg_a = float(self.get_parameter("beam_deg_a").value)
        beam_deg_b = float(self.get_parameter("beam_deg_b").value)

        beam_win = math.radians(float(self.get_parameter("beam_window_deg").value))
        front_win = math.radians(float(self.get_parameter("front_window_deg").value))

        enable_corner = bool(self.get_parameter("enable_corner_turn").value)
        corner_omega = float(self.get_parameter("corner_turn_omega").value)
        corner_v = float(self.get_parameter("corner_turn_v").value)
        corner_clear = float(self.get_parameter("corner_clear_dist").value)
        left_reacq = float(self.get_parameter("left_reacquire_dist").value)
        corner_hold = float(self.get_parameter("corner_hold_s").value)

        # Angles for left wall
        theta_a = math.radians(+beam_deg_a)
        theta_b = math.radians(+beam_deg_b)

        # Ranges for wall + front + left
        ra = self._min_range_in_sector(scan, theta_a, beam_win)
        rb = self._min_range_in_sector(scan, theta_b, beam_win)
        r_front = self._min_range_in_sector(scan, 0.0, front_win)

        r_left = rb  # rb is around +90deg already

        now = self.get_clock().now()
        mode_age = (now - self.mode_enter_time).nanoseconds * 1e-9

        if enable_corner:
            # Enter TURN_LEFT if front blocked
            if self.mode == self.MODE_FOLLOW and math.isfinite(r_front) and r_front < front_stop:
                self.mode = self.MODE_TURN_LEFT
                self.mode_enter_time = now
                # reset PID integrator a bit to avoid lurch after turning
                self.e_int = 0.0
                self.e_prev = 0.0
                self.get_logger().warn("[wall_follow] Corner detected -> TURN_LEFT")

            # Exit TURN_LEFT when front clear and left wall seen again (after hold time)
            if self.mode == self.MODE_TURN_LEFT and mode_age > corner_hold:
                front_ok = (math.isfinite(r_front) and r_front > corner_clear)
                left_ok = (math.isfinite(r_left) and r_left < left_reacq)
                if front_ok and left_ok:
                    self.mode = self.MODE_FOLLOW
                    self.mode_enter_time = now
                    self.e_int = 0.0
                    self.e_prev = 0.0
                    self.get_logger().warn("[wall_follow] Exit TURN_LEFT -> FOLLOW")

 
        # Execute mode
        if self.mode == self.MODE_TURN_LEFT:
            # Turn left until we can continue
            omega = clamp(abs(corner_omega), 0.0, omega_max)  # ensure positive magnitude
            omega = +omega  # left turn
            v = clamp(corner_v, -v_nom, v_nom)
            self._publish_wheels(v=v, omega=omega, r_wheel=r_wheel, wheel_base=wheel_base)
            self.get_logger().info(f"[wall_follow] mode=TURN_LEFT front={r_front:.2f} left={r_left:.2f} v={v:.2f} omega={omega:.2f}")
            return


        # FOLLOW mode 
        if not math.isfinite(ra) or not math.isfinite(rb):
            self._publish_wheels(v=v_min, omega=0.0, r_wheel=r_wheel, wheel_base=wheel_base)
            self.get_logger().warn("[wall_follow] Invalid wall beams (ra/rb).")
            return

        dtheta = abs(theta_b - theta_a)
        if dtheta < 1e-3:
            self._publish_wheels(v=v_min, omega=0.0, r_wheel=r_wheel, wheel_base=wheel_base)
            self.get_logger().warn("[wall_follow] dtheta too small; check beam angles.")
            return

        alpha = math.atan2(ra * math.cos(dtheta) - rb, ra * math.sin(dtheta))
        dist_to_wall = rb * math.cos(alpha)
        dist_ahead = dist_to_wall + L * math.sin(alpha)

        e = desired - dist_ahead

        t = self.get_clock().now()
        dt = (t - self.t_prev).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-3
        self.t_prev = t

        self.e_int += e * dt
        de = (e - self.e_prev) / dt
        self.e_prev = e

        omega = kp * e + ki * self.e_int + kd * de
        omega = clamp(omega, -omega_max, omega_max)

        # speed shaping
        v = v_nom * clamp(1.0 - 0.5 * abs(omega) / max(1e-6, omega_max), 0.4, 1.0)
        v = max(v, v_min)

        # front safety
        if math.isfinite(r_front):
            if r_front < front_stop:
                if not enable_corner:
                    v = 0.0
                    omega = 0.0
            elif r_front < front_slow:
                scale = (r_front - front_stop) / max(1e-6, (front_slow - front_stop))
                v = max(v_min, v * clamp(scale, 0.0, 1.0))

        self._publish_wheels(v=v, omega=omega, r_wheel=r_wheel, wheel_base=wheel_base)

        self.get_logger().info(
            f"[wall_follow] mode=FOLLOW ra={ra:.2f} rb={rb:.2f} front={r_front:.2f} "
            f"alpha={math.degrees(alpha):.1f} pred={dist_ahead:.2f} e={e:.2f} v={v:.2f} omega={omega:.2f}"
        )


def main():
    rclpy.init()
    node = WallFollow()
    rclpy.get_default_context().on_shutdown(node.stop_robot)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_robot()
            time.sleep(float(node.get_parameter("shutdown_flush_s").value))
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
