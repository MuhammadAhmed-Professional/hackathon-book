---
id: weekly-breakdown
title: 13-Week Course Schedule
sidebar_label: Weekly Breakdown
sidebar_position: 11
description: Detailed 13-week schedule mapping topics to modules, weekly learning objectives, hands-on projects, and assessment milestones for the Physical AI & Humanoid Robotics course.
keywords: [course schedule, 13 weeks, learning path, robotics curriculum, assessment, projects]
---

# 13-Week Course Schedule

This chapter provides a detailed week-by-week breakdown of the Physical AI & Humanoid Robotics course, mapping topics to modules, learning objectives, and hands-on projects.

## Course Timeline Overview

| **Weeks** | **Module** | **Focus** | **Assessment** |
|-----------|------------|-----------|----------------|
| 1-2 | Introduction | Physical AI fundamentals, setup | None |
| 3-5 | Module 1: ROS 2 | Nodes, topics, services, URDF | ROS 2 package build |
| 6-7 | Module 2: Gazebo/Unity | Simulation, physics, sensors | Gazebo simulation project |
| 8-10 | Module 3: NVIDIA Isaac | Isaac Sim, Isaac ROS, VSLAM, Nav2 | Isaac pipeline project |
| 11-12 | Humanoid Development | Integration, bipedal locomotion | Mid-term integration check |
| 13 | Module 4: VLA | Voice, LLM planning, capstone | Autonomous humanoid capstone |

---

## Weeks 1-2: Introduction to Physical AI

### Week 1: Physical AI Fundamentals

**Topics**:
- What is Physical AI? Transition from digital to embodied intelligence
- The humanoid robotics landscape (Unitree, Boston Dynamics, Figure, Tesla)
- Course structure and learning outcomes
- Hardware requirements and setup

**Activities**:
- Install Ubuntu 22.04 (native or VM)
- Set up development environment (Python, Git, text editor)
- Review prerequisites (Python, Linux, basic robotics concepts)

**Deliverables**: None (orientation week)

### Week 2: Software Setup & Prerequisites

**Topics**:
- Install ROS 2 Humble Hawksbill
- Verify ROS 2 installation (run turtlesim, explore topics)
- Introduction to coordinate frames and transformations
- Review of linear algebra for robotics

**Activities**:
- Complete ROS 2 installation tutorial
- Run basic ROS 2 examples (talker/listener nodes)
- Install VS Code with Python and ROS 2 extensions

**Deliverables**: Verified ROS 2 installation (screenshot of working turtlesim)

---

## Weeks 3-5: Module 1 - ROS 2

### Week 3: ROS 2 Architecture & Nodes

**Topics**:
- ROS 2 architecture and DDS foundation
- ROS 2 vs ROS 1 comparison
- Nodes: independent processes in ROS 2
- Creating your first ROS 2 node in Python (rclpy)

**Activities**:
- Read Module 1 chapters: ROS 2 Architecture, Nodes/Topics/Services
- Create a simple publisher node (publishes string messages)
- Create a simple subscriber node (receives and prints messages)

**Deliverables**: Python publisher and subscriber nodes

### Week 4: Topics, Services, and Actions

**Topics**:
- Pub/sub pattern with topics
- Request/response pattern with services
- Long-running tasks with actions (feedback mechanism)
- ROS 2 message types and custom messages

**Activities**:
- Build a ROS 2 package with multiple nodes
- Implement service client/server (example: add two numbers)
- Explore ROS 2 command-line tools (ros2 topic, ros2 service)

**Deliverables**: ROS 2 package with topics, services, and actions

### Week 5: Python Integration (rclpy) & URDF

**Topics**:
- rclpy (ROS Client Library for Python)
- Launch files (Python vs XML)
- Parameter management
- URDF basics: links, joints, kinematic chains
- URDF for humanoid robots

**Activities**:
- Write a Python launch file to start multiple nodes
- Define a simplified humanoid URDF (torso, arms, legs)
- Visualize URDF in RViz

**Assessment**: Submit ROS 2 package with Python nodes, launch file, and humanoid URDF

---

## Weeks 6-7: Module 2 - Gazebo & Unity

### Week 6: Gazebo Simulation & URDF/SDF

**Topics**:
- Gazebo environment setup
- Spawning robots in Gazebo
- URDF vs SDF: when to use each
- Robot descriptions with visual, collision, and inertial properties

**Activities**:
- Read Module 2 chapters: Gazebo Simulation, URDF vs SDF
- Spawn your humanoid URDF in Gazebo
- Add collision and inertial properties to URDF

**Deliverables**: Gazebo world with spawned humanoid robot

### Week 7: Physics & Sensor Simulation

**Topics**:
- Rigid body dynamics, collision detection
- Physics parameters (gravity, friction, damping)
- Sensor simulation: LiDAR, depth cameras, IMUs
- Unity for photorealistic rendering (optional)

**Activities**:
- Simulate LiDAR and depth camera in Gazebo
- Process sensor data in ROS 2 nodes
- Tune physics parameters for realistic behavior

**Assessment**: Submit Gazebo simulation with robot, sensors, and ROS 2 integration

---

## Weeks 8-10: Module 3 - NVIDIA Isaac

### Week 8: Isaac Sim & Omniverse

**Topics**:
- NVIDIA Isaac Sim overview
- Omniverse foundation and USD (Universal Scene Description)
- Photorealistic simulation vs Gazebo
- Synthetic data generation for perception training

**Activities**:
- Read Module 3 chapter: Isaac Sim
- Install Isaac Sim (via Omniverse)
- Import your humanoid robot into Isaac Sim
- Explore Isaac Sim sensors (cameras, LiDAR)

**Deliverables**: Isaac Sim scene with humanoid and sensors

### Week 9: Isaac ROS & VSLAM

**Topics**:
- Isaac ROS GEMs (GPU-accelerated modules)
- Hardware-accelerated VSLAM (Visual SLAM)
- Camera-based mapping and localization
- Comparison: VSLAM vs LiDAR SLAM

**Activities**:
- Install Isaac ROS packages
- Set up Isaac ROS VSLAM with RealSense camera (simulated or real)
- Build a map of environment using VSLAM
- Visualize map and robot pose in RViz

**Deliverables**: VSLAM pipeline with map visualization

### Week 10: Nav2 Path Planning

**Topics**:
- Nav2 navigation stack overview
- Path planning algorithms (A*, DWA, etc.)
- Obstacle avoidance for bipedal robots
- Configuring Nav2 for humanoids

**Activities**:
- Read Module 3 chapter: Nav2 Planning
- Configure Nav2 for your humanoid robot
- Send navigation goals in simulation
- Test obstacle avoidance in cluttered environment

**Assessment**: Submit Isaac Sim + Isaac ROS + Nav2 pipeline (video demo required)

---

## Weeks 11-12: Humanoid Development (Integration)

### Week 11: Bipedal Locomotion & Integration

**Topics**:
- Bipedal locomotion challenges (balance, gait planning)
- Integrating VSLAM + Nav2 for humanoid navigation
- Sensor fusion (cameras, IMU, force sensors)
- Multi-robot coordination (optional)

**Activities**:
- Test humanoid navigation in complex environments (stairs, narrow spaces)
- Implement sensor fusion for improved localization
- Review and refactor code for modularity

**Deliverables**: Integrated navigation system (VSLAM + Nav2) on humanoid

### Week 12: Manipulation & Object Interaction

**Topics**:
- Humanoid manipulation (grasping, object detection)
- MoveIt 2 for motion planning (optional)
- Object detection using Isaac ROS (AprilTags, object segmentation)
- Human-robot interaction scenarios

**Activities**:
- Add object detection to humanoid (detect and localize objects)
- Plan grasping motions (simplified or using MoveIt 2)
- Test manipulation in simulation

**Deliverables**: Manipulation demo (detect object, navigate, approach)

---

## Week 13: Module 4 - Vision-Language-Action (Capstone)

### Week 13: VLA Capstone Project

**Topics**:
- OpenAI Whisper for voice-to-text
- LLM-based cognitive planning (GPT-4 for task decomposition)
- Translating natural language to ROS 2 actions
- End-to-end autonomous humanoid system

**Activities**:
- Read Module 4 chapters: Voice-to-Action, Cognitive Planning, Capstone
- Integrate Whisper for voice commands
- Use GPT-4 to plan tasks (e.g., "Go to kitchen, get water bottle")
- Execute plan using Nav2 + manipulation
- Provide voice feedback (text-to-speech)

**Assessment (Final Capstone)**:
Submit a complete autonomous humanoid system that:
1. Receives voice command
2. Plans task using LLM
3. Navigates using Nav2 + VSLAM
4. Detects and manipulates object (optional)
5. Provides status feedback

**Deliverables**:
- Source code (ROS 2 packages, launch files)
- Video demo (max 5 minutes showing voice command → execution → completion)
- Project report (3-5 pages: architecture, design decisions, challenges, results)

---

## Assessment Summary

| **Week** | **Assessment Type** | **Weight** |
|----------|---------------------|------------|
| 5 | ROS 2 Package (nodes, launch, URDF) | 15% |
| 7 | Gazebo Simulation (sensors, physics) | 15% |
| 10 | Isaac Pipeline (VSLAM + Nav2) | 20% |
| 11-12 | Integration Check (navigation + manipulation) | 15% |
| 13 | **Capstone Project** (VLA autonomous humanoid) | **35%** |

---

## Time Commitment

- **Lectures/Readings**: 3-4 hours/week
- **Hands-On Labs**: 4-6 hours/week
- **Projects/Assignments**: 5-8 hours/week (more for capstone week)

**Total**: ~12-18 hours/week

---

## Tips for Success

1. **Start early on installations**: ROS 2, Gazebo, and Isaac Sim can be time-consuming to set up
2. **Use simulation first**: Test algorithms in simulation before deploying to hardware
3. **Document as you go**: Keep notes on design decisions for the final report
4. **Leverage community resources**: ROS 2 forums, NVIDIA Isaac forums, Stack Overflow
5. **Iterate quickly**: Don't aim for perfection in early weeks; iterate and improve

---

**Related**: [Introduction](./intro.md) | [Hardware Requirements](./hardware.md)
