---
id: module1-index
title: "Module 1 - ROS 2: The Robotic Nervous System"
sidebar_label: Overview
sidebar_position: 1
description: "Introduction to ROS 2 (Robot Operating System 2) as the foundational software architecture for modern robotics. Learn nodes, topics, services, Python integration, and URDF for humanoid robots."
keywords: [ros 2, robot operating system, nodes, topics, services, rclpy, urdf, humanoid robotics]
---

# Module 1: ROS 2 - The Robotic Nervous System

**Duration**: Weeks 3-5 (3 weeks)

## Module Overview

ROS 2 (Robot Operating System 2) is the de facto standard software framework for building robot applications. It provides a distributed architecture where independent **nodes** communicate via **topics** (pub/sub), **services** (request/response), and **actions** (long-running tasks with feedback). This module introduces you to ROS 2's core concepts and teaches you to build Python-based robot software for humanoid platforms.

Unlike ROS 1, ROS 2 is built on **DDS (Data Distribution Service)**, enabling real-time communication, improved security, and multi-robot coordination. It's used by leading robotics companies including Boston Dynamics, NASA, and NVIDIA Isaac.

## Learning Objectives

By the end of this module, you will be able to:

1. Understand ROS 2 architecture and how it differs from ROS 1
2. Create ROS 2 nodes that communicate via topics and services
3. Build Python-based ROS 2 packages using rclpy (ROS Client Library for Python)
4. Write launch files to orchestrate multiple nodes
5. Define humanoid robot structures using URDF (Unified Robot Description Format)
6. Visualize robot models in RViz

## Prerequisites

- Python 3.8+ programming experience
- Basic understanding of object-oriented programming
- Linux command-line familiarity (Ubuntu 22.04 recommended)
- Completed course introduction and setup

## Module Structure

This module consists of 4 chapters:

1. **ROS 2 Architecture**: DDS foundation, real-time communication, comparison to ROS 1
2. **Nodes, Topics, Services, Actions**: Core communication patterns in ROS 2
3. **Python Integration (rclpy)**: Building ROS 2 packages with Python, launch files, parameters
4. **URDF for Humanoids**: Defining robot kinematic chains, links, joints, visualization

## Hands-On Project

**Project**: Build a ROS 2 package for a simulated humanoid robot that:
- Creates a publisher node sending joint position commands
- Creates a subscriber node receiving sensor data
- Defines a humanoid robot URDF with at least 10 joints (torso, arms, legs)
- Launches multiple nodes using a Python launch file

## Hardware/Software Requirements

- Ubuntu 22.04 LTS (native or VM)
- ROS 2 Humble Hawksbill (or later)
- Python 3.10+
- 8GB RAM minimum
- RViz for visualization

**Installation Guide**: Follow the official ROS 2 installation guide at [docs.ros.org](https://docs.ros.org/en/humble/Installation.html)

## Assessment

You will be evaluated on:
- Correct implementation of ROS 2 nodes with pub/sub communication
- Proper package structure following ROS 2 conventions
- URDF accuracy (kinematic chain correctness, joint limits, inertial properties)
- Code quality (proper error handling, documentation, ROS 2 best practices)

## Additional Resources

- [ROS 2 Official Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Python Client Library (rclpy) API](https://docs.ros2.org/latest/api/rclpy/)
- [URDF XML Specification](http://wiki.ros.org/urdf/XML)
- [ROS 2 Design Principles](https://design.ros2.org/)

---

**Next Chapter**: [ROS 2 Architecture](./ros2-architecture.md)
