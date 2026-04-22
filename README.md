# UR5 ROS2 Control & IK System

This project provides a **MATLAB-based framework** to control a Universal Robots UR5 using ROS 2, inverse kinematics, and external perception inputs (camera or cage tracking).

### It supports:
* Real-time robot state publishing
* ROS2-based communication
* IK-based motion planning
* Debug/manual control mode
* Camera or cage-driven target execution
* Tool output control (gripper)

---

### BEFORE RUNNING ANY MATLAB SCRIPT
In a terminal of your PC, you **MUST** first run:

```bash
source /opt/ros/humble/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5 \
    robot_ip:=147.250.35.40 \
    launch_rviz:=false

```
### Important

This step must be done before MATLAB scripts.

It starts the UR driver and enables robot communication.

---

## SYSTEM OVERVIEW

### OPTION 1 — Full ROS2 Pipeline (Camera / Cage Control)

### Run in MATLAB#1:
publisher.m

### Function:
- Publishes robot pose (TransformStamped) in /robot_pose

---

### Run in MATLAB#2:
main2.m

### Function:
- Subscribes to:
  - /instruction_cage (cage tracking)
  - OR /object_positions (camera tracking)
- Receives target positions
- Computes inverse kinematics (IK)
- Sends joint commands to UR5 robot

---

### How to Operate (Option 1)

- Open terminal → start UR driver (see above)
- Open MATLAB #1 → run publisher.m
- Open MATLAB #2 → run main2.m
- Wait for incoming target point (camera or cage)

When a point is received:
- Press ENTER → robot moves to target
- Stop execution anytime with CTRL + C

---

### Behavior Rules

- Robot only moves after user confirmation (ENTER)
- Incoming points are throttled (prevents spam commands)
- Robot is locked during execution (prevents collisions)

---

### OPTION 2 — Debug / Manual IK Mode

### Run in MATLAB:
ur5_script.m

### Function:

This script:
- Loads UR5 robot model
- Computes IK for a given position
- Sends robot using computed joint angles (qSol)
- Returns robot to home before execution

---

### Workflow

- Robot is placed or measured at a known pose
- IK is computed for a target position
- Robot is optionally returned to home configuration
- User manually sends joint configuration:

sendJointConfiguration(ur, qSol, 'EndTime', 5);

---

### Purpose

This mode is used for:
- IK debugging
- Reachability testing
- Calibration validation
- Motion planning verification

---

### Difference from Option 1

- No camera or cage input
- No ROS2 topic subscription
- Fully manual and deterministic
