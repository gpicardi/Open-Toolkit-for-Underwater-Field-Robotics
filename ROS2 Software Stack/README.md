# ROS2 Software Stack

## Disclaimer
To be ported from BluE repository 

## Overview

This repository documents the **ROS2 Software Stack** used in the **Open Toolkit for Underwater Robotics**. The software provides modular control for underwater robotic joints, sensors, and communication interfaces, leveraging ROS2 for distributed and scalable operation.

## Software Architecture

The ROS2 software stack includes:
- **Sensor Nodes** for real-time data acquisition.
- **Motor Control and State Feedback using ros2_control** for efficient motion execution.
- **Communication Interfaces** for networked operations (e.g. with a control PC, surface buoy, or fixed platform). 

## Sensor Nodes

### 1. **Humidity and Temperature Sensor Node**
- **Sensor Used**: DHT11
- **Description**: Monitors the internal canister conditions to detect early leakage or overheating.
- **Topic Published**: `/dht11_sensor_data` (silver_interfaces/msg/DhtSensorData)
- **Data Format**:
  ```\
  std_msgs/String location
  std_msgs/Float64 temperature
  std_msgs/Float64 humidity
  ```

### 2. **IMU Node (Inertial Measurement Unit)**
- **Sensor Used**: BNO055
- **Description**: Provides real-time orientation, acceleration, and angular velocity for state estimation.
- **Topic Published**: `/imu/data` (sensor_msgs/msg/Imu)
- **Reference Repository**: https://github.com/flynneva/bno055

### 3. **Internal Pressure, Temperature and Humidity Sensor Node**
- **Sensor Used**: MS8607
- **Description**: Measures pressure, temperature and humidity inside the control canister.
- **Topic Published**: 
  - `/ms8607/pressure` (std_msgs/Float64)
  - `/ms8607/temperature` (std_msgs/Float64)
  - `/ms8607/humidity` (std_msgs/Float64)

### 4. **External Pressure, Temperature and Depth Sensor Node**
- **Sensor Used**: MS8537
- **Description**: Measures underwater pressure, temperature and depth.
- **Topic Published**:
  - `/ms8537/pressure` (std_msgs/Float64)
  - `/ms8537/temperature` (std_msgs/Float64)
  - `/ms8537/depth` (std_msgs/Float64)

### 5. **Powering Monitor Sensor Node**
- **Sensor Used**: LTC2945
- **Description**: Measures power monitoring.
- **Topic Published**:
  - `/ltc2945/vin` (std_msgs/Float64)
  - `/ltc2945/power` (std_msgs/Float64)
  - `/ltc2945/current` (std_msgs/Float64)

## Motor Control with `ros2_control`

The **ros2_control** framework is used for real-time motor actuation.

### 1. **Hardware Interface**
- **Driver Used**: Dynamixel Workbench (RS485 communication) https://github.com/dynamixel-community/dynamixel_hardware.
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
For more information, contact: **Giacomo Picardi giacomo.picardi1991@gmail.com or gpicardi@icm.csic.es** or **Jorge Aguirregomezcorta mail@mail**
