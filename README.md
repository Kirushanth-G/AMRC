# AMR Controller - Autonomous Mobile Robot

This project involves the development of an **Autonomous Mobile Robot (AMR)** capable of mapping a closed environment and navigating through it using SLAM (Simultaneous Localization and Mapping). The robot is equipped with custom-built hardware, including a motor driver PCB and a designed frame and enclosure, and utilizes **ROS2** and the **Cartographer SLAM algorithm** for mapping and navigation.

## Features
- **Mapping**: Uses **Cartographer SLAM** for real-time mapping of the environment.
- **Navigation**: Utilizes **ROS2 Humble NAV2** for path planning and autonomous navigation.
- **Custom Motor Driver PCB**: Designed and fabricated a motor driver circuit to control the robot's wheels.
- **Robot Frame & Enclosure**: A robust frame to house the motors and sensors, with a custom-designed plastic enclosure to protect the motor driver PCB and development board.
- **LiDAR Integration**: For accurate obstacle detection and environment mapping.

---

## Table of Contents
- [Features](#features)
- [Hardware](#hardware)
- [Software](#software)
- [Installation](#installation)
- [Usage](#usage)
- [Acknowledgments](#acknowledgments)

---

## Hardware

### Components
- **Microcontroller**: ESP32 WROOM 32E (or similar)
- **LiDAR Sensor**: For SLAM and obstacle detection
- **Motor Driver PCB**: Custom-designed PCB to control the motors.
- **Motors**: 12V DC motors with encoders for precise movement.
- **Frame**: Aluminum/plastic frame for structural integrity.
- **Enclosure**: Custom 3D-printed plastic enclosure for housing the PCB and development board.
- **Batteries**: Li-ion batteries for powering the robot.

### Motor Driver PCB
The motor driver PCB was designed to efficiently control the robot's motors and was developed using **Altium Designer**. The design features:
- **H-Bridge circuits** for motor control.
- **Overcurrent protection**.
- **PWM control** for speed adjustment.

### Frame & Enclosure
The robot's frame was built to support the motors, LiDAR, and other sensors. A custom-designed 3D-printed plastic enclosure houses the motor driver PCB and microcontroller, ensuring that all electronics are securely protected during operation.

---

## Software

### ROS2 and SLAM
The software is built around the **ROS2 Humble** middleware, using the following key packages:
- **Cartographer**: For 2D SLAM and real-time environment mapping.
- **Nav2**: For navigation and path planning.
  
### Key Algorithms
- **SLAM**: We use Cartographer SLAM to generate a real-time 2D map of the robot's environment based on LiDAR data.
- **Path Planning**: Nav2 is used for efficient and reliable navigation through the mapped environment.

---

## Installation

### Prerequisites
- Install **ROS2 Humble** and necessary dependencies for your system. You can follow the [ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).
- Set up your workspace with the following required packages:
  ```bash
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src
  git clone https://github.com/ros2/cartographer.git
  git clone https://github.com/ros2/navigation2.git
  cd ~/ros2_ws
  colcon build
