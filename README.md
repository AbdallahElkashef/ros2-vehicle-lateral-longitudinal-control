# 🚗 ROS 2 PID-Based Autonomous Lane Keeping & Precise Stopping

This repository contains a ROS 2 implementation of a closed-loop autonomous driving controller that performs lane keeping, speed regulation, and precise stopping using PID control.

The system was developed as part of the Assiut Motorsport Autonomous Team Newcomer Program, focusing on real-world autonomous vehicle control software.

---

## 🧠 System Architecture

Two independent real-time PID controllers operate simultaneously:

| Controller | Function |
|------------|------------|
| Lateral PID | Minimizes lateral deviation using steering angle |
| Longitudinal PID | Controls throttle and braking for speed tracking and stopping |

Both controllers rely on live odometry feedback.

---

## 🎯 Control Strategy

### Lateral Control

The lateral error is computed as:

e_x = x_desired − x_current

A PID controller continuously adjusts the steering angle to keep the vehicle centered in the lane.

---

### Longitudinal Control

The longitudinal controller operates in two phases:

| Phase | Behavior |
|--------|-------------|
| Cruising | Track target speed using throttle PID |
| Terminal | Disable throttle and apply full braking |

The mission only terminates when the vehicle reaches the target position and its velocity becomes zero, guaranteeing a true physical stop.

---

## ⚙️ Key Technical Features

- Dual real-time PID control loops
- Anti-windup protection
- Quaternion to yaw conversion
- Velocity magnitude estimation
- ROS 2 Action Server for mission control
- Hard stop enforcement logic for zero-velocity convergence

---

## 🗂️ ROS 2 Interfaces

| Type | Name |
|------|--------|
| Throttle | /cmd_vel |
| Brakes | /brakes |
| Steering | /SteeringAngle |
| Odometry | /odom |
| Action Server | /reach_target |

---

## 📁 Package Structure

.
├── coppelia
│   ├── c_drive0.py
│   ├── c_drive1.py
│   ├── c_drive2.py
│   ├── task_0.py
│   └── task2_pid.py
├── package.xml
├── setup.py
├── setup.cfg
└── resource
    └── coppelia

---

## ▶️ Running the Controller

Build workspace:

colcon build  
source install/setup.bash

Run node:

ros2 run <your_package_name> task2_pid.py

Send mission goal:

ros2 action send_goal /reach_target my_own_action/action/ReachTarget "{target_distance: -80.0}"

---

## 🏁 Skills Demonstrated

- Autonomous vehicle control systems
- PID-based lateral and longitudinal control
- ROS 2 real-time robotics software
- Closed-loop feedback system design
- Autonomous stopping logic
- Team-based engineering (Assiut Motorsport)

---

## 👨‍💻 Author

Abdallah Elkashef  
Autonomous Systems Engineer — Assiut Motorsport

