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
- **Topic Published**: `/dht11_sensor_data` (rrr_interfaces/msg/DhtSensorData)
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
- **XML RRR_Manipulator Xacro File**: Simple transmission mechanism added to set offset and reduction of the Dynamixel Actuators
```xml
<transmission name="coxa_transmission">
  <plugin>transmission_interface/SimpleTransmission</plugin>
  <actuator name="coxa_actuator" role="coxa_actuator"/>
  <joint name="coxa_joint" role="coxa_joint">
      <mechanical_reduction>${coxa_reduction}</mechanical_reduction>
      <offset>${coxa_offset}</offset>
  </joint>
</transmission>
```
### 2. **Controller Configuration**
A sample `ros2_control` configuration for the motor driver:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 30

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    forward_position_controller:
      type: forward_command_controller/ForwardCommandController

    joint_trajectory_position_controller:
      type: joint_trajectory_controller/JointTrajectoryController

forward_position_controller:
  ros__parameters:
    joints:
      - coxa_joint
      - femur_joint
      - tibia_joint

    interface_name: position

    action_monitor_rate: 20.0

    allow_partial_joints_goal: false
    open_loop_control: true
    allow_integration_in_goal_trajectories: true
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0

```

### 3. **Motor Control Execution**
- Position commands are sent via `/forward_position_controller/commands`. I.e.:
  ```
  ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: - 0.75 - 0.8 - 2.3"
  ```
- Position-based control can be implemented via `/motor/position`.
- State feedback (position, velocity, effort) is provided via `/joint_states`.

### 4. RRR Manipulator Application
- The toolkit is applied to the case of an RRR serial manipulator.
- Robot description and controller configuration are contained in the /rrr_manipulator package.
- Replace such directory with your own description in order to test your own robot implementation.
---
For more information, contact:
- **Giacomo Picardi: giacomo.picardi1991@gmail.com or gpicardi@icm.csic.es**
- **Jorge Aguirregomezcorta Aina: jaguirregomezcorta@gmail.com or jaguirregomezcorta@icm.csic.es**
