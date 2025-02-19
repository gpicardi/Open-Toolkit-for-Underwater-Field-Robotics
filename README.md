# Open Toolkit for Underwater Robotics

## Overview

The **Open Toolkit for Underwater Robotics** is designed to facilitate the development of underwater robotic manipulators and legged systems. It provides modular, cost-effective, and open-source solutions for waterproofing, control, power management, and software integration, enabling researchers and developers to prototype and test underwater robotic systems more efficiently.

### Key Features
- **Waterproof Canisters**: Designed for Dynamixel actuators, tested up to 30m depth.
- **Control & Power Management**: Custom electronics enabling efficient underwater operations.
- **Leakage Detection System**: Early warning system for water ingress.
- **ROS 2-Based Software Stack**: Facilitates control, sensing, and actuation.
- **Open-Source Design**: Hardware and software available for community contributions.

## System Components

### 1. Waterproof Actuation Module
- Uses **Dynamixel XM430-W350** servo motors enclosed in an aluminum canister.
- Includes a **capacitive humidity sensor** and **DHT11 temperature sensor** for early leakage detection.
- Designed with **O-ring sealing** and a **pressure relief plug** to maintain internal pressure stability.

![Waterproof Actuation Module](git_images/CannisterV1.jpg)

### 2. Control & Power Management
- Based on a **Raspberry Pi 4** architecture.
- Supports **RS485 communication** for Dynamixel servos.
- Includes **DCDC converters** for stable power delivery.
- Features integrated **humidity monitoring** for early fault detection.
- Compatible with Blue Robotics Canisters for integration.

![Control & Power Management](path/to/image2.jpg)

### 3. ROS 2 Software Stack
- Modular software framework for motor control, sensing, and data acquisition.
- Supports real-time monitoring and logging.
- Facilitates integration with **networking and surface communication**.

![ROS 2 Software Stack](path/to/image3.jpg)

## Experimental Validation

### 1. Leakage Detection Tests
- Humidity sensor successfully detected small water ingress during controlled tests.
- **Failure-depth tests**: Several trials conducted at different depths, with minor failures recorded at **Calabria 2021** and **La Spezia 2021**.

![Leakage Detection Test](path/to/image4.jpg)

### 2. Structural and Environmental Tests
- FEM simulations in **SolidWorks** validated the canister’s buckling resistance.
- Field deployments in **real underwater conditions** for long-term monitoring.
- Potential future **hyperbaric chamber testing** to assess deeper water resistance.

![Structural Testing](path/to/image5.jpg)

## Applications

### 1. **Leg Module for SILVER2**
- An **RRR serial manipulator** based on three underwater robotic joints.
- Integrated as the **legged system** for SILVER2.

![SILVER2 Leg Module](path/to/image6.jpg)

### 2. **Tendon-Driven Actuation**
- Used for **soft robotic grippers** requiring flexible motion.
- Features a **spooling system** for precise cable-driven control.

![Tendon-Driven Actuation](path/to/image7.jpg)

### 3. **Underactuated Mechanisms for Sampling**
- Designed for **sediment collection** in underwater environments.
- Utilizes a **four-bar linkage mechanism** for efficient grasping and storage.

![Underactuated Sampling System](path/to/image8.jpg)

## Open Science & Reproducibility

To foster collaboration and reproducibility, all hardware designs, software, and datasets are shared under open licenses:
- **GitHub Repository**: [Link to repository]
- **Zenodo Archive**: [DOI for dataset & experiments]

We encourage contributions, modifications, and feedback to improve the toolkit and extend its capabilities for broader underwater robotics applications.

## Citing this Repository

If you use this toolkit in your research or project, please cite it as follows:

```
@misc{YourRepo,
  author = {Your Name and Contributors},
  title = {Open Toolkit for Underwater Robotics},
  year = {2025},
  url = {https://github.com/your-repo},
  note = {Accessed: YYYY-MM-DD}
}
```

## Future Work
- Scaling for deeper environments beyond 30m.
- Integration of **AI-driven control strategies** for adaptive locomotion.
- Exploring bio-inspired legged movement for improved seabed interaction.
- Enhancing modularity for broader use cases in oceanographic research.

---
For more information, contact: **[Your Name/Email]**
