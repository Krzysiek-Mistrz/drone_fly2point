# Drone Control (ROS2 & PX4)

A ROS2 package for flying a PX4-based drone to specified 3D points via MAVROS.  
This project demonstrates a simple point-to-point navigation using ROS2, MAVROS, and PX4 SITL.

---

## Table of Contents

- [Overview](#overview)  
- [Features](#features)  
- [Prerequisites](#prerequisites)  
- [Installation & Build](#installation--build)  
- [Configuration](#configuration)  
- [Launching the System](#launching-the-system)  
- [Running the Flight Node](#running-the-flight-node)  
- [Package Structure](#package-structure)  
- [Contributing](#contributing)  
- [License](#license)  
- [Maintainer](#maintainer)  

---

## Overview

This package sets up:
1. A MAVROS bridge connecting to PX4 SITL gazebo.  
2. A ROS2 node (`drone_fly_to_point`) that sends position setpoints (`geometry_msgs/PoseStamped`) to the flight stack.  

By combining these, you can command the drone to take off, navigate to a target coordinate, and land.

---

## Features

- Connect to PX4 SITL over UDP via MAVROS  
- Publish setpoints for autonomous point-to-point flight  
- Uses only standard ROS2 and MAVROS messages  
- Easy-to-extend Python node (`rclpy`)  

---

## Prerequisites

- Linux OS (Ubuntu 20.04+ recommended)  
- ROS2 distribution (Foxy / Galactic / Humble)  
- PX4 Autopilot (with SITL support)  
- MAVROS installed (`sudo apt install ros-<distro>-mavros ros-<distro>-mavros-extras`)  
- Python3, `colcon` build tools  

---

## Installation & Build

1. Clone this repository into your ROS2 workspace:
   ```
   cd ~/ros2_ws/src
   git clone git@github.com:Krzysiek-Mistrz/drone_fly2point.git
   ```
2. Install any missing ROS2 dependencies:
   ```
   cd ~/ros2_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -y
   ```
3. Build the workspace:
   ```
   colcon build --symlink-install
   ```
4. Source the install setup:
   ```
   source install/setup.bash
   ```  
> **IMPORTANT**  
> After cloning u need to change package name (main folder name so "drone_fly2point" 2 drone_fly_to_point) in order for this package 2 work. Otherwise you would need to change setup.py 

---

## Configuration

- **Launch file**:  
  `launch/my_mavros_launch.launch.py`  
  - Configures MAVROS to connect to PX4 SITL at `udp://:14540@localhost:14557`  
  - Launches the `drone_fly_to_point` node  
> **NOTE**
> U need 2 have mavros installed `sudo apt install ros-<ur_ros_distro>-mavros ros-<ur_ros_distro>-mavros-extras`

- **Parameters**:  
  - In the launch file, adjust `fcu_url`, `system_id`, and `component_id` as needed.  
  - The Python node may accept command-line flags or ROS2 parameters for the target point (see code).

---

## Launching the System

1. Start PX4 SITL in a separate terminal:
   ```
   cd PX4-Autopilot
   make px4_sitl_default gazebo
   ```
2. In your ROS2 workspace:
   ```
   source install/setup.bash
   ros2 launch drone_control my_mavros_launch.launch.py
   ```
3. You should see MAVROS and your flight node come up, connecting to the simulator.

---

## Running the Flight Node

Once launched, the drone'll flight to certain point characterized in node:

---

## Package Structure

```
drone_control/
├── launch/
│   └── my_mavros_launch.launch.py   # Start MAVROS + flight node
├── drone_control/                   # Python module (rclpy node)
│   └── drone_fly_to_point.py        # Main node implementation
├── package.xml                      # ROS2 package descriptor
└── setup.py                         # Python install/setup script
```

---

## Contributing

1. Fork the repository  
2. Create a feature branch (`git checkout -b feature/foo`)  
3. Commit your changes (`git commit -am 'Add foo'`)  
4. Push to the branch (`git push origin feature/foo`)  
5. Open a Pull Request  

Please follow ROS2 style guidelines and include tests where possible.

---

## License

GNU GPL V3 @ Krzychu 2025