import pyrealsense2 as rs
import numpy as np
import cv2
import time

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

pipeline.start(config)

# Let auto-exposure settle
for _ in range(30):
    pipeline.wait_for_frames()

frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()

if not color_frame:
    raise RuntimeError("No color frame")

color_image = np.asanyarray(color_frame.get_data())

cv2.imwrite("d455_snapshot.jpg", color_image)
print("Saved d455_snapshot.jpg")

pipeline.stop()
