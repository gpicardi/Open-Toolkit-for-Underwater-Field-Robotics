# Mechanical Design Repository for Underwater Robotics

## Overview

This repository documents the mechanical design of the **Open Toolkit for Underwater Robotics**, providing details on the waterproof canisters, actuator integration, and material selection. The goal is to offer a reproducible, modular, and cost-effective approach to underwater robotic systems.

## Mechanical Subsystems

### 1. Waterproof Actuation Module
- Designed for **Dynamixel XM430-W350** actuators.
- Enclosed in a **6000-series aluminum** canister for corrosion resistance.
- Features a **double O-ring seal** and **pressure relief plug**.
- Includes **early leakage detection** with humidity and temperature sensors.

![Waterproof Actuation Module](path/to/image1.jpg)

### 2. Structural Components
- Frame and mounting brackets designed for **submersion stability**.
- Components are **CNC machined** for precision.
- Stress analysis performed using **FEM simulations** in SolidWorks.

![Structural Components](path/to/image2.jpg)

## Material Selection

| Component           | Material             | Justification                                  |
|--------------------|---------------------|----------------------------------------------|
| Canister          | 6000-series Aluminum | Corrosion resistance, lightweight, machinable |
| Shaft             | 316 Stainless Steel  | Prevents corrosion under prolonged exposure   |
| Seals             | Nitrile Rubber (NBR) | High resistance to water ingress              |
| Front Cap         | ABS Plastic          | Good adhesion properties with aluminum       |
| Structural Frame  | Anodized Aluminum    | Increased durability in marine environments  |

## Discussion & Tips

### 1. **Canister Sealing Considerations**
- Ensure **O-rings are lubricated** before assembly.
- Avoid excessive tightening of **rear cap** to maintain proper compression.
- Periodically inspect the **humidity sensor readings** for early leakage detection.

### 2. **Structural Strength & Buoyancy**
- Consider **reducing metal thickness** to minimize weight without compromising depth resistance.
- Add **buoyancy foam** for neutral buoyancy in deeper deployments.
- FEM analysis suggests **reinforcement** at stress points in the leg modules.

### 3. **Corrosion Prevention**
- **Rinse components with fresh water** after deployment.
- Use **anodized finishes** or protective coatings on aluminum surfaces.
- Regularly inspect and replace **rubber seals** to maintain watertight integrity.

## Future Improvements
- **Optimizing shaft seals** to reduce potential lateral leaks.
- **Exploring new materials** like titanium for deeper applications.
- **Hyperbaric testing** to validate performance beyond 30m depth.

---
For more information, contact: **[Your Name/Email]**
