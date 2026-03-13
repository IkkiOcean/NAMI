# NAMI – Navigational Autonomous Mapping Intelligence

NAMI is a LiDAR-based autonomous robot designed for **indoor navigation and logistics automation**.  
The system can **map unknown indoor environments, localize itself, plan routes, and avoid obstacles** without requiring any external infrastructure such as magnetic tracks or markers.

This project was developed as a **Major Academic Project (2025–2026)** for the B.Tech Computer Science & Engineering (Artificial Intelligence) program.

---

## Overview

Many indoor environments such as offices, hospitals, and warehouses still rely on manual transportation of files, supplies, and materials. This consumes time and diverts staff from their core responsibilities.

NAMI addresses this problem by introducing an **autonomous navigation system** that can move within indoor spaces, detect obstacles, and reach target locations efficiently.

Key goals of the system include:

- Infrastructure-free indoor navigation
- Autonomous environment mapping
- Real-time obstacle avoidance
- Reliable operation in dynamic environments

---

## Features

- **LiDAR-based environment scanning**
- **SLAM-based mapping and localization**
- **A* path planning algorithm**
- **Dynamic obstacle detection and avoidance**
- **Autonomous indoor navigation**

---

## System Components

### Hardware

- **LiDAR Sensor** – 360° environmental scanning
- **Raspberry Pi 4** – Main processing unit
- **DC Motors (4)** – Robot locomotion
- **L298N Motor Drivers** – Motor control
- **Ultrasonic Sensors** – Additional obstacle detection
- **12V Battery** – Power supply

### Software

- **SLAM (Simultaneous Localization and Mapping)** for map generation and localization
- **A\* Algorithm** for optimal path planning
- **Obstacle avoidance algorithms** for safe navigation
- **Motor control system** for movement execution

---

## How It Works

1. The **LiDAR sensor scans the environment** and collects spatial data.
2. A **SLAM algorithm builds a map** while determining the robot’s position.
3. The **A\* algorithm calculates the optimal path** to a destination.
4. The robot **moves using motor control commands**.
5. If obstacles appear, the system **updates the route dynamically**.

---

## Applications

NAMI can be applied in environments where indoor transport automation is needed.

Examples include:

- **Healthcare Facilities** – Transporting lab samples, medicines, or documents
- **Corporate Offices** – Internal document and mail distribution
- **Warehouses** – Moving inventory and materials
- **Hotels** – Delivery of luggage or room service

---

## Expected Impact

- Reduction in manual logistics workload
- Improved operational efficiency
- Continuous autonomous operation
- High navigation accuracy within indoor environments

## License

This project is licensed under the **Creative Commons Attribution–NonCommercial 4.0 International License (CC BY-NC 4.0)**.

You are free to:
- Share and adapt the material
- Use it for research or educational purposes

Under the following conditions:
- Proper attribution must be given to the original authors.
- The material **may not be used for commercial purposes**.

Full license text: https://creativecommons.org/licenses/by-nc/4.0/