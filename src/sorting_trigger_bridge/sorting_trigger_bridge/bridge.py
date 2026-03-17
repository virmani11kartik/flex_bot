#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn

from fastapi.middleware.cors import CORSMiddleware


app = FastAPI()

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

_ros_node = None
_pub = None

class TriggerBridge(Node):
    def __init__(self):
        super().__init__("sorting_trigger_bridge")
        self.publisher = self.create_publisher(String, "/sorting/trigger", 10)
        self.get_logger().info("Sorting Trigger Bridge ready: POST http://localhost:8080/trigger_sort")

@app.post("/trigger_sort")
def trigger_sort(req: TriggerRequest):
    # Publish into ROS
    msg = String()
    # Put whatever you want here — JSON string is a simple start
    msg.data = f"reason={req.reason}, batch_id={req.batch_id}, count={req.count}"
    _pub.publish(msg)
    return {"ok": True, "published_to": "/sorting/trigger", "data": msg.data}

def main():
    global _ros_node, _pub
    rclpy.init()
    _ros_node = TriggerBridge()
    _pub = _ros_node.publisher

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
