# ROS2 UR5 PIE – Pick and Place System

This project implements a pick-and-place robotic system using a UR5 manipulator, integrating MATLAB-based control with ROS2 communication.

Developed at ENSTA Paris (U2IS) as part of the PIE 2025/2026 project.

---

## Description

The objective of this project is to develop a complete pipeline for controlling a UR5 robotic arm to perform pick-and-place tasks.

The system combines:
- Robot configuration and setup  
- MATLAB-based control and inverse kinematics  
- ROS2 communication between systems  

The control pipeline follows a Cartesian-to-joint approach:
1. A target position is defined in Cartesian space  
2. A corresponding end-effector orientation is computed  
3. A homogeneous transformation is constructed  
4. Inverse kinematics is solved  
5. Joint commands are sent to the robot  

This architecture enables modular and extensible control of the robot for structured manipulation tasks.

---

## Project Status

This repository is under development.  
Further documentation and setup instructions will be added progressively.
