
# ss24-solo12-control

This repository supports the development and control of the SOLO12 robot from the Open Dynamic Robot Initiative, leveraging the *ros2_control* framework. It includes essential packages for robot description, control, and simulation, providing a foundational structure to work with SOLO12 in ROS 2.

## What is *ros2_control*?

In short, *ros2_control* is a control framework for ROS 2. But actually, it is much more—it's the kernel of the ROS 2 system that controls robots:

- It abstracts hardware and low-level control for other frameworks such as MoveIt2 and Nav2.
- It provides resource access management.
- It controls the lifecycle of robots and their hardware.

For more details, check [this presentation](https://control.ros.org/master/doc/resources/resources.html#ros-world-2021).

## Installation

To install the required dependencies, run the following command in the root directory of your workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

This command installs all necessary dependencies for the project.

## Usage

To launch the system, execute the following command:

```bash
ros2 launch solo12_bringup solo12.launch.py
```

This will bring up the SOLO12 system. For more detailed information, including available launch arguments, please refer to the `solo12_bringup` package.

### View the Robot in Rviz2

To view the SOLO12 robot model and move its joints using the *Joint State Publisher* GUI, execute the following command:

```bash
ros2 launch solo12_description view_solo12.launch.py
```

This will load the robot's URDF and allow interaction with the robot's joints through Rviz2.

## Repository Structure

The repository is structured around the following main components:

### 1. Robot Description (`solo12_description`)
This package contains the URDF and XACRO files, defining the kinematics, visual, and collision properties of the SOLO12 robot.

### 2. Bringup Package (`solo12_bringup`)
This package is responsible for launching the system, loading robot description files, and starting the necessary controllers.

### 3. Control Package (`solo12_control`)
This package contains the control logic for the robot, interfacing with the *ros2_control* framework. It includes the setup for sending commands and managing robot behavior.

### 4. Example Applications (`solo12_examples`)
This package includes example use cases and demonstrations to help test and interact with the robot using *ros2_control*.

## Using *ros2_control* with SOLO12

The *ros2_control* framework abstracts hardware and low-level control, providing resource management and lifecycle control for the robot. Key components of the framework include:

- **Controller Manager**: Manages and loads controllers.
- **Controllers**: Send commands to control the robot's joints.
- **Resource Manager**: Manages hardware interfaces and resources.
- **Hardware Interface**: Interfaces with physical or simulated hardware.

### Mock Hardware Plugin

For simpler testing and development, you can use the Mock Hardware Plugin to simulate robot hardware. This allows testing controllers and setup configurations without needing real hardware or simulation.

To run the system with mock hardware, use:

```bash
ros2 launch solo12_bringup solo12.launch.py 
```

### Testing Controllers

By default, the system starts with the Joint Trajectory Controller. If you want to load the Forward Command Controller instead, use the following command:

```bash
ros2 launch solo12_bringup solo12.launch.py robot_controller:=forward_command_controller

```
To test the controllers, run the following:

```bash
ros2 launch solo12_bringup controllers.launch.py

```

This will launch the joint_trajectory_controller and publish goals using trajectory_msgs.msg.JointTrajectoryPoint.

**NOTE:** Delay between spawning controllers is usually not necessary, but may be useful when starting a complex setup. Adjust this for your specific use case.


## Packages Overview

- **solo12_bringup**: Handles the system startup and controller management.
- **solo12_control**: Contains control logic and configuration for the controllers.
- **solo12_description**: Defines the robot's URDF and XACRO files.
- **solo12_examples**: Provides examples for testing and interacting with the robot.

For more detailed information about each package, please refer to the README files within the respective directories.

---
