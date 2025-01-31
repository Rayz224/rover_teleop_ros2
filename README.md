# Rover Teleop ROS2

A ROS2 package for keyboard teleoperation of a robot using WASD or arrow keys.

## Description

This package provides keyboard-based teleoperation functionality, publishing commands as Twist messages. It supports variable linear and angular speeds with smooth control.

## Installation
Install `colcon` if not already installed: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

### Dependencies
```bash
pip install -U keyboard
```

### Node
```bash
cd ~/ros2_ws/src
git clone https://github.com/Rayz224/rover_teleop_ros2.git

cd ~/ros2_ws
colcon build --packages-select rover_teleop --symlink-install

source install/setup.bash # ./install/local_setup.ps1 for Windows
```

## Usage

Run the teleop node:

```bash
ros2 run rover_teleop rover_teleop
```
with custom parameters:
```bash
ros2 run rover_teleop rover_teleop --ros-args -p cmd_topic:=turtle1/cmd_vel
```
### Controls

| Control | Action |
|----------|---------|
| W or ↑ | Forward |
| S or ↓ | Backward |
| A or ← | Rotate left |
| D or → | Rotate right |
| Q or Ctrl+C | Quit |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| linear_max | 1.0 | Maximum linear velocity (m/s) |
| angular_max | 1.0 | Maximum angular velocity (rad/s) |
| publish_rate | 10.0 | Command publish rate (Hz) |
| cmd_topic | cmd_vel | Topic name for velocity commands |

## Troubleshooting

If you get build errors about setuptools, try installing the 58.2.0 version with `pip install setuptools==58.2.0`

The node works only when the terminal is focused.