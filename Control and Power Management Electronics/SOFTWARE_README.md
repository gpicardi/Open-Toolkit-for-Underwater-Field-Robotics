# ROS2 Software Stack

## Overview

This repository documents the **ROS2 Software Stack** used in the **Open Toolkit for Underwater Robotics**. The software provides modular control for underwater robotic actuators, sensors, and communication interfaces, leveraging ROS2 for distributed and scalable operation.

## Software Architecture

The ROS2 software stack includes:
- **Sensor Nodes** for real-time data acquisition.
- **Motor Control using ros2_control** for efficient motion execution.
- **Communication Interfaces** for networked operations with a surface buoy.

## Sensor Nodes

### 1. **Humidity and Temperature Sensor Node**
- **Sensor Used**: DHT11
- **Topic Published**: `/environment/humidity_temperature`
- **Description**: Monitors the internal canister conditions to detect early leakage or overheating.
- **Data Format**:
  ```json
  {
    "humidity": 45.3,
    "temperature": 22.5
  }
  ```

### 2. **IMU Node (Inertial Measurement Unit)**
- **Sensor Used**: MPU9250
- **Topic Published**: `/imu/data`
- **Description**: Provides real-time orientation, acceleration, and angular velocity for state estimation.
- **Data Format**:
  ```json
  {
    "orientation": {"x": 0.1, "y": 0.2, "z": 0.3, "w": 1.0},
    "angular_velocity": {"x": 0.01, "y": -0.02, "z": 0.005},
    "linear_acceleration": {"x": 0.0, "y": 9.81, "z": 0.0}
  }
  ```

### 3. **Pressure Sensor Node**
- **Sensor Used**: Blue Robotics Bar30
- **Topic Published**: `/environment/pressure`
- **Description**: Measures underwater depth and ambient pressure.
- **Data Format**:
  ```json
  {
    "pressure": 1013.25,
    "depth": 5.2
  }
  ```

## Motor Control with `ros2_control`

The **ros2_control** framework is used for real-time motor actuation.

### 1. **Hardware Interface**
- **Driver Used**: Dynamixel Workbench (RS485 communication)
- **Controller Interface**: `/cmd_vel`, `/joint_states`
- **Motor Topics**:
  - `/motor/position`
  - `/motor/velocity`
  - `/motor/effort`

### 2. **Controller Configuration**
A sample `ros2_control` configuration for the motor driver:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

velocity_controller:
  ros__parameters:
    joints:
      - leg_joint_1
      - leg_joint_2
      - leg_joint_3
    gains:
      leg_joint_1: {p: 5.0, i: 0.1, d: 0.01}
      leg_joint_2: {p: 5.0, i: 0.1, d: 0.01}
      leg_joint_3: {p: 5.0, i: 0.1, d: 0.01}
```

### 3. **Motor Control Execution**
- Commands are sent via `/cmd_vel` for velocity control.
- Position-based control can be implemented via `/motor/position`.
- State feedback is provided via `/joint_states`.

## Future Improvements
- Integration of **ROS2 Lifecycle Nodes** for better resource management.
- Adding **sensor fusion** between IMU and pressure sensors for improved localization.
- Implementing **adaptive motor control** for energy efficiency in different environments.

---
For more information, contact: **[Your Name/Email]**
