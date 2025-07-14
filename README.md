# ROS 2 Robotic Arm Control System

This repository contains the ROS 2 packages for controlling a robotic arm system.

## Packages

- **motor**: Motor controller interface and drivers
- **servo**: Servo controller for wrist and vacuum end-effector
- **trajectory**: Trajectory planning and path generation
- **vision**: Computer vision and object detection
- **gui**: Qt-based user interface for system control
- **arm_bringup**: Launch files and configuration for system startup

## Prerequisites

- ROS 2 Rolling
- Ubuntu 22.04 or newer
- Qt 5
- Python 3.8+

## Building

```bash
# Clone the repository
git clone https://github.com/yourusername/rosarm.git
cd rosarm

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
```

## Running

You can use the start script which automatically detects devices and sets permissions:

```bash
./start.sh
```

Or launch manually:

```bash
# Source ROS 2
source /opt/ros/rolling/setup.bash
source install/setup.bash

# Launch the system
ros2 launch arm_bringup arm_control.launch.py
```

## Configuration

The system can be configured using parameters specified in the launch files or via the command line:

```bash
ros2 launch arm_bringup arm_control.launch.py motor_port:=/dev/ttyUSB0 servo_port:=/dev/ttyUSB1
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.