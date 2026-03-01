# Companion Computer/Jetson Setup



![Demo](demo.gif)

The system publishes the standard TF chain:

map → odom → base_link → laser_1
└→ xsens_imu


> **Important**
>
> RViz can only show the scan and map moving in the world if
> `odom → base_link` changes over time.
> If no real odometry source exists (wheel odom / VIO / scan-matching odom),
> and the LiDAR is moved by hand, the scan and map will appear “stuck” in `map`.
> This is expected behavior.

---

## Prerequisites

- ROS 2 (Humble / Jazzy)
- fastapi
- uvicorn
- Packages installed:
  - `sick_scan_xd`
  - `robot_localization`
  - `slam_toolbox`
  - `tf2_ros`
  - `rviz2`

Network example (adjust as needed):

- Lidar IP: `192.168.0.1`
- ROS PC (UDP receiver): `192.168.0.20`

---

## Build Workspace

```bash
cd ~/kr_flexbot
colcon build --symlink-install
source install/setup.bash
```

Publish Static TF (base_link → IMU)

Declare the IMU as rigidly mounted to the robot body.
```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link xsens_imu

    Replace the zeros with the real mounting transform when known.
```
Publish Static TF (base_link → LiDAR)

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link laser_1

    Do not publish duplicate TFs.
    Verify existing TFs using:

    ros2 run tf2_tools view_frames
```
Step 4: Start EKF (robot_localization)

The EKF publishes odom → base_link.
```bash
ros2 run robot_localization ekf_node --ros-args --params-file \
  ~/li_bot/src/picoscan_launch/config/ekf_imu.yaml
```
EKF Notes

    For 2D SLAM, two_d_mode: true is strongly recommended.

    If only IMU data is available, fusing linear acceleration will cause drift.

    A stable configuration fuses yaw and yaw-rate only until wheel odometry
    or VIO is added.


SLAM TF responsibilities

    slam_toolbox: publishes map → odom

    robot_localization: publishes odom → base_link

    Static TFs: publish base_link → laser_1, base_link → sick_imu

Step 6: RViz2 Visualization

Start RViz:

rviz2

Fixed Frame

    Set Fixed Frame = map

Displays to add

    TF

    LaserScan

        Topic: /scan_fullframe

