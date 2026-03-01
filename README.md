# FlexBot Companion Computer Setup

This repository contains the companion computer (Jetson/Ubuntu PC) side of the FlexBot stack. It is designed to work alongside the IMX7 firmware — see the `imx7` branch for the embedded side.

The packages covered here provide a minimal framework for:
- **Teleoperation** — sending UDP commands to the IMX7 and receiving state feedback
- **Wheel Odometry** — estimating robot motion from encoder data
- **State Estimation** — fusing odometry and IMU via an EKF to produce a stable `odom → base_link` transform

> **Note:** The `particle_filter` and full `bringup` packages are not part of this guide. Only the packages listed above are needed for a working teleop + odometry setup.

---

## Prerequisites

- ROS 2 (Humble or Jazzy)
- `robot_localization` — for EKF state estimation
- `tf2_ros`
- `rviz2` (optional, for visualization)

Network configuration (adjust to your setup):
- IMX7 IP: e.g. `192.168.0.10`
- Companion Computer IP (UDP receiver): e.g. `192.168.0.20`

---

## Build the Workspace

```bash
cd ~/flex_bot
colcon build --symlink-install
source install/setup.bash
```

---

## Package Overview

### `flex_bot_teleop`

Handles bidirectional UDP communication with the IMX7.

- `udp_bridge_node` — bridges ROS 2 topics to/from UDP packets exchanged with the IMX7
- `teleop_node` — converts joystick or keyboard input into velocity commands

Configuration is in `config/flex_bot_udp.yaml` and `config/teleop.yaml`. Adjust UDP ports and IP addresses there to match your network.

Launch teleop:
```bash
ros2 launch flex_bot_teleop flex_bot.launch.py
```

---

### `flex_bot_odometry`

Computes wheel odometry from encoder feedback received over UDP from the IMX7.

Publishes `odom → base_link` as an odometry source (later fused by the EKF).

Configuration is in `config/wheel_odom.yaml`.

Launch wheel odometry:
```bash
ros2 launch flex_bot_odometry wheel_odom.launch.py
```

To visualize the odometry path:
```bash
ros2 run flex_bot_odometry odom_to_path.py
```

---

### State Estimation (`flex_bot_bringup` — EKF only)

The EKF (via `robot_localization`) fuses wheel odometry and IMU data to produce a smooth, stable `odom → base_link` transform.

Configuration is in `flex_bot_bringup/config/ekf_imu.yaml`. Key settings:

- `two_d_mode: true` — recommended for ground robots
- Fuse yaw and yaw-rate from IMU; avoid fusing raw linear acceleration until a reliable odometry source is confirmed, as it will cause drift

Launch state estimation:
```bash
ros2 launch flex_bot_bringup bringup_state_estimation.launch.py
```

Or run the EKF node directly:
```bash
ros2 run robot_localization ekf_node --ros-args --params-file \
  ~/flex_bot/src/flex_bot_bringup/config/ekf_imu.yaml
```

---

## Static TF Setup

The robot requires static transforms declaring sensor mounting positions relative to `base_link`. Publish these before starting any other nodes.

**base_link → IMU:**
```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link xsens_imu
```

> Replace the zeros with the real mounting offset once measured.

Verify the full TF tree at any time:
```bash
ros2 run tf2_tools view_frames
```

---

## TF Chain

The expected transform chain during normal operation:

```
odom → base_link        # published by robot_localization EKF
         └→ xsens_imu  # static TF
```

> `map → odom` is not published in this minimal setup. It would require a SLAM or localization node, which is out of scope here.

---

## Recommended Launch Order

1. Static TFs
2. Teleop (UDP bridge + command input)
3. Wheel Odometry
4. State Estimation (EKF)

```bash
# Terminal 1 — Static TFs
ros2 launch flex_bot_bringup static_tfs.launch.py

# Terminal 2 — Teleop + UDP bridge
ros2 launch flex_bot_teleop flex_bot.launch.py

# Terminal 3 — Wheel odometry
ros2 launch flex_bot_odometry wheel_odom.launch.py

# Terminal 4 — EKF state estimation
ros2 launch flex_bot_bringup bringup_state_estimation.launch.py
```

---

## IMX7 Counterpart

The embedded firmware running on the IMX7 handles low-level motor control, encoder reading, and UDP packet formatting. See the `imx7` branch of this repository for that code.

Make sure the UDP IP/port settings in `flex_bot_teleop/config/flex_bot_udp.yaml` match what is configured on the IMX7 side.
