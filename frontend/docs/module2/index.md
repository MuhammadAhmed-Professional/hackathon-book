---
id: module2-index
title: "Module 2 - Gazebo & Unity: The Digital Twin"
sidebar_label: Overview
sidebar_position: 1
description: "Learn to simulate robot behaviors in Gazebo physics engine and Unity for photorealistic rendering. Master URDF/SDF formats, physics simulation, and sensor simulation for safe testing before real-world deployment."
keywords: [gazebo, unity, simulation, digital twin, urdf, sdf, physics simulation, robotics]
---

# Module 2: Gazebo & Unity - The Digital Twin

**Duration**: Weeks 6-7 (2 weeks)

## Module Overview

Simulation is critical for robotics development, allowing you to test algorithms safely, iterate quickly, and generate synthetic training data before deploying to expensive hardware. This module introduces **Gazebo** (open-source physics simulator) and **Unity** (game engine for photorealistic rendering) as complementary tools for creating digital twins of humanoid robots.

**Gazebo** excels at physics-based simulation (gravity, collisions, friction) and sensor simulation (LiDAR, depth cameras, IMUs). **Unity** provides high-fidelity 3D rendering for human-robot interaction scenarios and synthetic data generation.

## Learning Objectives

By the end of this module, you will be able to:

1. Set up Gazebo simulation environments with custom worlds
2. Understand URDF vs SDF (Simulation Description Format) and when to use each
3. Simulate physics (rigid body dynamics, collision detection, friction)
4. Simulate sensors (LiDAR, depth cameras, IMUs) and process simulated data
5. Integrate Unity with ROS 2 using Unity Robotics Hub
6. Compare Gazebo vs Unity for different robotics use cases

## Prerequisites

- Completed Module 1 (ROS 2 fundamentals, URDF basics)
- Familiarity with 3D geometry and coordinate frames
- Ubuntu 22.04 with ROS 2 Humble installed

## Module Structure

This module consists of 4 chapters:

1. **Gazebo Simulation**: Environment setup, spawning robots, plugin architecture, ROS 2 integration
2. **URDF vs SDF Formats**: When to use each, robot descriptions with visual/collision/inertial properties
3. **Physics Simulation**: Rigid body dynamics, collision detection, friction models, tuning parameters
4. **Unity Rendering**: Unity for photorealism, Unity Robotics Hub, human-robot interaction scenarios

## Hands-On Project

**Project**: Create a Gazebo simulation of a humanoid robot that:
- Spawns in a custom world with obstacles (stairs, narrow corridors, uneven terrain)
- Has realistic physics properties (mass, inertia, joint limits)
- Simulates LiDAR and depth camera sensors
- Publishes sensor data to ROS 2 topics
- Includes a launch file to start simulation and sensor nodes

## Hardware/Software Requirements

- Ubuntu 22.04 LTS
- Gazebo Fortress (or Garden)
- ROS 2 Humble with ros_gz bridge
- Unity 2021.3+ (for Unity chapter, optional)
- Unity Robotics Hub (for Unity chapter)
- 16GB RAM recommended for Gazebo + ROS 2

## Assessment

You will be evaluated on:
- Correct Gazebo world setup with realistic physics
- Accurate robot SDF with collision/visual/inertial properties
- Functional sensor simulation (LiDAR, depth camera)
- Integration with ROS 2 (sensor data published correctly)

## Additional Resources

- [Gazebo Official Documentation](https://gazebosim.org/docs)
- [SDF Format Specification](http://sdformat.org/)
- [Unity Robotics Hub GitHub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-Gazebo Integration Guide](https://github.com/ros-simulation/ros_gz)

---

**Next Chapter**: [Gazebo Simulation](./gazebo-simulation.md)
