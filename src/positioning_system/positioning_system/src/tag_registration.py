#!/usr/bin/env python3
"""
Topics consumed:
    /pgv/position    (geometry_msgs/PointStamped)
    /pgv/tag_id      (std_msgs/Float64)
    /pgv/angle_deg   (std_msgs/Float64)
    /amcl_pose       (geometry_msgs/PoseWithCovarianceStamped)

Topics published:
    /positioning/markers_viz  (visualization_msgs/MarkerArray) 
"""

import json
import math
import os
import sys
import select
import termios
import threading
import tty
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray

try:
    from ament_index_python.packages import get_package_share_directory
    _PKG_SHARE = get_package_share_directory("positioning_system")
except Exception:
    _PKG_SHARE = None

DEFAULT_MARKERS_FILE = os.path.join(
    _PKG_SHARE, "config", "markers.json"
) if _PKG_SHARE else os.path.join(
    os.path.expanduser("~"), "flex_bot", "src",
    "positioning_system", "config", "markers.json"
)


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def get_key():
    """Read one keypress, non-blocking (returns None if no key ready)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        return sys.stdin.read(1) if rlist else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class TagRegistration(Node):
    def __init__(self, markers_file: str):
        super().__init__("tag_registration")
        self.markers_file = markers_file

        # Load existing markers
        self.markers: dict = {}
        if os.path.exists(self.markers_file):
            with open(self.markers_file) as f:
                self.markers = json.load(f)
            self.get_logger().info(
                f"Loaded {len(self.markers)} existing markers from {self.markers_file}")

        # Live state
        self._lock       = threading.Lock()
        self.pgv_x_m     = 0.0
        self.pgv_y_m     = 0.0
        self.pgv_angle   = 0.0
        self.pgv_tag_id  = 0
        self.amcl_x      = None
        self.amcl_y      = None
        self.amcl_yaw    = None
        self.have_amcl   = False

        # Subscribers
        self.create_subscription(PointStamped, "/pgv/position",   self._cb_pos,   10)
        self.create_subscription(Float64,      "/pgv/tag_id",     self._cb_tag,   10)
        self.create_subscription(Float64,      "/pgv/angle_deg",  self._cb_angle, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self._cb_amcl, 10)

        # Latched MarkerArray publisher for RViz
        latched_qos = QoSProfile(depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/positioning/markers_viz", latched_qos)

        # Status line at 2 Hz
        self.create_timer(0.5, self._print_status)

        # Publish existing markers immediately
        self._publish_all_markers()

        print("\n" + "="*62)
        print("  PGV Tag Registration")
        print("="*62)
        print("  SPACEBAR  — lock current reading and register marker")
        print("  Q / Ctrl+C — quit and save")
        print("="*62 + "\n")

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _cb_pos(self, msg):
        with self._lock:
            self.pgv_x_m = msg.point.x
            self.pgv_y_m = msg.point.y

    def _cb_tag(self, msg):
        with self._lock:
            self.pgv_tag_id = int(msg.data)

    def _cb_angle(self, msg):
        with self._lock:
            self.pgv_angle = msg.data

    def _cb_amcl(self, msg):
        with self._lock:
            self.amcl_x   = msg.pose.pose.position.x
            self.amcl_y   = msg.pose.pose.position.y
            self.amcl_yaw = math.degrees(quat_to_yaw(msg.pose.pose.orientation))
            self.have_amcl = True

    # ── Status line ───────────────────────────────────────────────────────────
    def _print_status(self):
        with self._lock:
            tag = self.pgv_tag_id
            px  = self.pgv_x_m * 1000.0
            py  = self.pgv_y_m * 1000.0
            pa  = self.pgv_angle
            ax, ay, ayaw = self.amcl_x, self.amcl_y, self.amcl_yaw
            have = self.have_amcl

        tag_str  = f"tag={tag}" if tag else "tag=NONE"
        pgv_str  = f"pgv x={px:+7.1f}mm y={py:+7.1f}mm angle={pa:+7.2f}°"
        amcl_str = (f"amcl x={ax:.3f}m y={ay:.3f}m yaw={ayaw:.2f}°"
                    if have else "amcl WAITING...")
        print(f"\r  [{tag_str:12s}]  {pgv_str}  |  {amcl_str}    ",
              end="", flush=True)

    # ── Lock and register ─────────────────────────────────────────────────────
    def lock_and_register(self):
        with self._lock:
            tag_id  = self.pgv_tag_id
            pgv_x   = self.pgv_x_m
            pgv_y   = self.pgv_y_m
            pgv_ang = self.pgv_angle
            ax, ay, ayaw = self.amcl_x, self.amcl_y, self.amcl_yaw
            have = self.have_amcl

        print()

        if not have:
            print("  ✗  No AMCL pose — ensure localization is running.")
            return
        if tag_id == 0:
            print("  ✗  No tag visible (tag_id=0) — drive over a tag first.")
            return

        key = str(tag_id)

        print(f"\n  ── Locked reading ──────────────────────────────────────")
        print(f"  Tag ID      : {tag_id}")
        print(f"  PGV offset  : x={pgv_x*1000:.1f}mm  y={pgv_y*1000:.1f}mm")
        print(f"  PGV angle   : {pgv_ang:.2f}°")
        print(f"  AMCL pose   : x={ax:.4f}m  y={ay:.4f}m  yaw={ayaw:.2f}°")

        if key in self.markers:
            mk = self.markers[key]
            print(f"\n  ⚠  Tag {tag_id} already registered:")
            print(f"     map x={mk['map_x']:.4f}  y={mk['map_y']:.4f}"
                  f"  yaw={mk['map_yaw_deg']:.2f}°"
                  f"  dock_angle={mk['pgv_dock_angle_deg']:.2f}°")
            overwrite = input("  Overwrite? [y/N]: ").strip().lower()
            if overwrite != "y":
                print("  Skipped.")
                return

        self.markers[key] = {
            "tag_id"             : tag_id,
            "map_x"              : round(ax,      6),
            "map_y"              : round(ay,      6),
            "map_yaw_deg"        : round(ayaw,    4),
            "pgv_dock_angle_deg" : round(pgv_ang, 4),
            "pgv_offset_x_mm"    : round(pgv_x * 1000.0, 2),
            "pgv_offset_y_mm"    : round(pgv_y * 1000.0, 2),
            "registered_at"      : datetime.now().isoformat(),
            "note"               : ""
        }

        self._save()
        self._publish_all_markers()
        print(f"\n  ✓  Tag {tag_id} registered, saved, and visible in RViz.\n")

    def _save(self):
        os.makedirs(os.path.dirname(self.markers_file), exist_ok=True)
        with open(self.markers_file, "w") as f:
            json.dump(self.markers, f, indent=2)
        print(f"  Saved → {self.markers_file}")

    # ── Publish all markers to RViz ───────────────────────────────────────────
    def _publish_all_markers(self):
        arr = MarkerArray()
        mid = 0

        for key, mk in self.markers.items():
            stamp = self.get_clock().now().to_msg()
            tag_id = mk["tag_id"]
            mx, my = mk["map_x"], mk["map_y"]
            yaw_rad = math.radians(mk["map_yaw_deg"])

            # Cyan cylinder
            cyl = Marker()
            cyl.header.frame_id = "map"
            cyl.header.stamp    = stamp
            cyl.ns, cyl.id      = "tag_markers", mid; mid += 1
            cyl.type            = Marker.CYLINDER
            cyl.action          = Marker.ADD
            cyl.pose.position.x = mx
            cyl.pose.position.y = my
            cyl.pose.position.z = 0.05
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = cyl.scale.y = 0.3
            cyl.scale.z = 0.1
            cyl.color.r = 0.0; cyl.color.g = 0.8
            cyl.color.b = 1.0; cyl.color.a = 0.8
            arr.markers.append(cyl)

            # White text label
            txt = Marker()
            txt.header          = cyl.header
            txt.ns, txt.id      = "tag_labels", mid; mid += 1
            txt.type            = Marker.TEXT_VIEW_FACING
            txt.action          = Marker.ADD
            txt.pose.position.x = mx
            txt.pose.position.y = my
            txt.pose.position.z = 0.45
            txt.pose.orientation.w = 1.0
            txt.scale.z         = 0.22
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text            = f"TAG {tag_id}"
            arr.markers.append(txt)

            # Yellow docking arrow
            arr_m = Marker()
            arr_m.header        = cyl.header
            arr_m.ns, arr_m.id  = "tag_arrows", mid; mid += 1
            arr_m.type          = Marker.ARROW
            arr_m.action        = Marker.ADD
            arr_m.pose.position.x = mx
            arr_m.pose.position.y = my
            arr_m.pose.position.z = 0.05
            arr_m.pose.orientation.z = math.sin(yaw_rad / 2.0)
            arr_m.pose.orientation.w = math.cos(yaw_rad / 2.0)
            arr_m.scale.x = 0.4
            arr_m.scale.y = arr_m.scale.z = 0.06
            arr_m.color.r = 1.0; arr_m.color.g = 0.8
            arr_m.color.b = 0.0; arr_m.color.a = 1.0
            arr.markers.append(arr_m)

        self.marker_pub.publish(arr)

    def print_summary(self):
        print("\n" + "="*62)
        print(f"  Registered markers ({len(self.markers)}):")
        print("="*62)
        for k, v in sorted(self.markers.items(), key=lambda x: int(x[0])):
            print(f"  Tag {k:>3s}  map=({v['map_x']:.3f}, {v['map_y']:.3f})m"
                  f"  yaw={v['map_yaw_deg']:.1f}°"
                  f"  dock_angle={v['pgv_dock_angle_deg']:.1f}°")
        print("="*62)


def main():
    rclpy.init()
    node = TagRegistration(DEFAULT_MARKERS_FILE)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            key = get_key()
            if key is None:
                continue
            if key == " ":
                node.lock_and_register()
            elif key in ("q", "Q", "\x03"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.print_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
