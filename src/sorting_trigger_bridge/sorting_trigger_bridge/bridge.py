#!/usr/bin/env python3
import threading
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn

from fastapi.middleware.cors import CORSMiddleware


app = FastAPI()
# TARGET_POSE_TOPIC = "/target_pose"
TARGET_POSE_TOPIC = "/goal_pose"
SHELF_READING_FLAG_TOPIC = "/shelf_reading_flag"

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # your web app origin
    allow_credentials=True,
    allow_methods=["*"],   # allow POST, OPTIONS, etc.
    allow_headers=["*"],   # allow Content-Type, Authorization, etc.
)


class TriggerRequest(BaseModel):
    reason: str = "batch_ready"
    batch_id: str | None = None
    count: int | None = None


class StandbyPoseRequest(BaseModel):
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    frame_id: str = "map"


class ShelfReadingTriggerRequest(BaseModel):
    running: bool = True

_ros_node = None
_pub = None
_goal_pub = None
_shelf_reading_pub = None
_shelf_reading_running = False

class TriggerBridge(Node):
    def __init__(self):
        super().__init__("sorting_trigger_bridge")
        self.publisher = self.create_publisher(String, "/sorting/trigger", 10)
        self.goal_publisher = self.create_publisher(PoseStamped, TARGET_POSE_TOPIC, 10)
        self.shelf_reading_publisher = self.create_publisher(Bool, SHELF_READING_FLAG_TOPIC, 10)
        self.shelf_reading_subscription = self.create_subscription(
            Bool,
            SHELF_READING_FLAG_TOPIC,
            self.handle_shelf_reading_flag,
            10,
        )
        self.get_logger().info(
            "Sorting Trigger Bridge ready: POST http://localhost:8080/trigger_sort "
            "and POST http://localhost:8080/standby_pose "
            "and POST http://localhost:8080/shelf_reading_trigger"
        )

    def handle_shelf_reading_flag(self, msg: Bool):
        global _shelf_reading_running
        _shelf_reading_running = bool(msg.data)

@app.post("/trigger_sort")
def trigger_sort(req: TriggerRequest):
    # Publish into ROS
    msg = String()
    # Put whatever you want here — JSON string is a simple start
    msg.data = f"reason={req.reason}, batch_id={req.batch_id}, count={req.count}"
    _pub.publish(msg)
    return {"ok": True, "published_to": "/sorting/trigger", "data": msg.data}


@app.post("/standby_pose")
def standby_pose(req: StandbyPoseRequest):
    goal = PoseStamped()
    goal.header.stamp = _ros_node.get_clock().now().to_msg()
    goal.header.frame_id = req.frame_id
    goal.pose.position.x = req.x
    goal.pose.position.y = req.y
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = math.sin(req.theta / 2.0)
    goal.pose.orientation.w = math.cos(req.theta / 2.0)

    _goal_pub.publish(goal)

    return {
        "ok": True,
        "published_to": TARGET_POSE_TOPIC,
        "pose": {
            "x": req.x,
            "y": req.y,
            "theta": req.theta,
            "frame_id": req.frame_id,
        },
    }


@app.get("/shelf_reading_status")
def shelf_reading_status():
    return {
        "ok": True,
        "topic": SHELF_READING_FLAG_TOPIC,
        "running": _shelf_reading_running,
    }


@app.post("/shelf_reading_trigger")
def shelf_reading_trigger(req: ShelfReadingTriggerRequest):
    global _shelf_reading_running

    if _shelf_reading_running and req.running:
        return {
            "ok": True,
            "topic": SHELF_READING_FLAG_TOPIC,
            "running": _shelf_reading_running,
            "already_running": True,
        }

    msg = Bool()
    msg.data = req.running
    _shelf_reading_pub.publish(msg)
    _shelf_reading_running = bool(req.running)

    return {
        "ok": True,
        "topic": SHELF_READING_FLAG_TOPIC,
        "running": _shelf_reading_running,
        "already_running": False,
    }

def main():
    global _ros_node, _pub, _goal_pub, _shelf_reading_pub
    rclpy.init()
    _ros_node = TriggerBridge()
    _pub = _ros_node.publisher
    _goal_pub = _ros_node.goal_publisher
    _shelf_reading_pub = _ros_node.shelf_reading_publisher

    # Spin ROS in a background thread
    t = threading.Thread(target=rclpy.spin, args=(_ros_node,), daemon=True)
    t.start()

    # Run the web server (blocking)
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="info")

    # Cleanup (if uvicorn exits)
    _ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
