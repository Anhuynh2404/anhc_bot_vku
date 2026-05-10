# ANHC_BOT

ROS 2 Jazzy + Gazebo Harmonic mobile robot simulation package.

Forked and professionally restructured from [bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot).

## Features

- Differential-drive mobile robot with 6 wheels (2 traction + 4 caster)
- 2D LiDAR (360°, 16m range)
- Depth camera (Intel D415)
- Stereo camera support
- Conveyor belt option
- SLAM Toolbox integration
- Nav2 autonomous navigation

## Environment

- **OS:** Ubuntu 24.04 LTS
- **ROS:** ROS 2 Jazzy Jalisco
- **Simulator:** Gazebo Harmonic

## Quick Start

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/anhc_ws/install/setup.bash

# Launch simulation
ros2 launch anhc_bot simulation.launch.py \
  camera_enabled:=True \
  two_d_lidar_enabled:=True \
  world_file:=small_warehouse.sdf

# Launch SLAM mapping
ros2 launch anhc_bot mapping.launch.py

# Launch Nav2 navigation
ros2 launch anhc_bot navigation.launch.py

# Launch RViz visualization only
ros2 launch anhc_bot visualization.launch.py
```

## Package Structure

```
anhc_bot/
├── urdf/           # Robot description (URDF/xacro)
├── launch/         # Launch files
├── config/
│   ├── slam/       # SLAM Toolbox params
│   ├── nav2/       # Nav2 params
│   └── maps/       # Pre-built maps
├── worlds/         # Gazebo world files
├── models/         # Third-party Gazebo models (AWS)
├── meshes/         # Robot mesh assets
├── rviz/           # RViz2 configurations
├── scripts/        # Utility scripts
└── docs/           # Documentation
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/anhc_bot/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/anhc_bot/odom` | `nav_msgs/Odometry` | Odometry |
| `/anhc_bot/scan` | `sensor_msgs/LaserScan` | 2D LiDAR |
| `/anhc_bot/imu` | `sensor_msgs/Imu` | IMU |
| `/anhc_bot/kinect_camera` | `sensor_msgs/Image` | Depth image |
| `/anhc_bot/kinect_camera/points` | `sensor_msgs/PointCloud2` | Point cloud |
