#!/usr/bin/env python3
"""
Usage (interactive):
    ros2 run positioning_system waypoint_cli.py

Usage (direct):
    ros2 run positioning_system waypoint_cli.py --tag 5

Publishes:
    /positioning/go_to_tag  (std_msgs/String)

Subscribes:
    /positioning/docked     (std_msgs/String) — "tag:N" on dock complete
    /positioning/status     (std_msgs/String) — human-readable state
"""

import argparse
import json
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_MARKERS_FILE = os.path.join(
    os.path.expanduser("~"), "flex_bot", "src",
    "positioning_system", "config", "markers.json"
)


def load_markers():
    if os.path.exists(DEFAULT_MARKERS_FILE):
        with open(DEFAULT_MARKERS_FILE) as f:
            return json.load(f)
    return {}


class WaypointCli(Node):
    def __init__(self):
        super().__init__("waypoint_cli")

        self.pub_ = self.create_publisher(String, "/positioning/go_to_tag", 1)

        self.create_subscription(String, "/positioning/docked",
            lambda m: print(f"\n  ✓  DOCKED — {m.data}\n> ", end="", flush=True), 10)

        self.create_subscription(String, "/positioning/status",
            lambda m: print(f"\n  [{m.data}]\n> ", end="", flush=True), 10)

    def send(self, tag_id: str):
        msg = String()
        msg.data = str(tag_id)
        self.pub_.publish(msg)


def print_markers(markers: dict):
    if not markers:
        print("  No markers registered yet.")
        print("  Run: ros2 run positioning_system tag_registration_tool.py")
        return
    print("\n  Registered markers:")
    print("  " + "-"*56)
    for k, v in sorted(markers.items(), key=lambda x: int(x[0])):
        note = f"  ({v['note']})" if v.get("note") else ""
        print(f"  [{k:>3s}]  map=({v['map_x']:.3f}, {v['map_y']:.3f})m"
              f"  dock_angle={v['pgv_dock_angle_deg']:.1f}°{note}")
    print("  " + "-"*56)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tag", type=str, default=None,
                        help="Tag ID to navigate to directly (non-interactive)")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = WaypointCli()
    markers = load_markers()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Non-interactive
    if args.tag is not None:
        if args.tag not in markers:
            print(f"  ✗  Tag {args.tag} not in markers.json ({DEFAULT_MARKERS_FILE})")
            sys.exit(1)
        print(f"  Sending robot to tag {args.tag}...")
        node.send(args.tag)
        input("  Press Enter to exit...\n")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Interactive
    print("\n" + "="*56)
    print("  flex_bot Positioning CLI")
    print("="*56)
    print(f"  Markers file: {DEFAULT_MARKERS_FILE}")
    print_markers(markers)
    print("\n  Commands:")
    print("    <tag_id>   — navigate + dock at tag (e.g. '5')")
    print("    list       — show registered markers")
    print("    q / exit   — quit")
    print("="*56 + "\n")

    try:
        while rclpy.ok():
            try:
                raw = input("> ").strip()
            except EOFError:
                break

            if not raw:
                continue
            if raw.lower() in ("q", "quit", "exit"):
                break
            if raw.lower() == "list":
                markers = load_markers()
                print_markers(markers)
                continue

            try:
                tag_id = str(int(raw))
            except ValueError:
                print(f"  Unknown command: '{raw}' — type a tag ID or 'list'")
                continue

            markers = load_markers()  # reload in case just registered
            if tag_id not in markers:
                print(f"  ✗  Tag {tag_id} not registered. Type 'list' to see available markers.")
                continue

            mk = markers[tag_id]
            print(f"  → Navigating to tag {tag_id} "
                  f"at map({mk['map_x']:.3f}, {mk['map_y']:.3f})m "
                  f"dock_angle={mk['pgv_dock_angle_deg']:.1f}° ...")
            node.send(tag_id)

    except KeyboardInterrupt:
        pass
    finally:
        print("\n  Bye.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
