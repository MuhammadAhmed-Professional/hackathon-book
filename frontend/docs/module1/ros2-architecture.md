---
id: ros2-architecture
title: "ROS 2 Architecture and Core Concepts"
sidebar_label: "ROS 2 Architecture"
sidebar_position: 2
description: "Deep dive into ROS 2 architecture, DDS foundation, real-time communication, and comparison to ROS 1 for humanoid robotics applications."
keywords: [ros2, architecture, dds, middleware, real-time, ros 1 vs ros 2]
---

# ROS 2 Architecture and Core Concepts

## Introduction

ROS 2 (Robot Operating System 2) represents a complete architectural redesign of the original ROS, built from the ground up to address the limitations of ROS 1 for modern robotics applications. Understanding ROS 2's architecture is crucial for building reliable, real-time humanoid robot systems that can operate safely in dynamic environments.

**Learning Objectives**:
- Understand the DDS (Data Distribution Service) foundation of ROS 2
- Recognize key architectural differences between ROS 1 and ROS 2
- Grasp how ROS 2 enables real-time communication for humanoid robots
- Learn the role of middleware in robot software architecture
- Understand quality of service (QoS) policies for reliable communication

## Why ROS 2? The Evolution from ROS 1

ROS 1, released in 2007, revolutionized robotics by providing a flexible framework for robot software. However, as robotics applications evolved—particularly for humanoid robots requiring **real-time control**, **multi-robot coordination**, and **production-grade reliability**—ROS 1's limitations became apparent:

**ROS 1 Limitations**:
- **Single Master Architecture**: Centralized rosmaster creates a single point of failure
- **No Real-Time Support**: Best-effort communication unsuitable for safety-critical control loops
- **Limited Security**: No built-in authentication, encryption, or access control
- **TCP-Only by Default**: High latency for time-sensitive data like IMU readings or motor commands
- **Python 2.x Dependency**: End-of-life Python version with security vulnerabilities

ROS 2 addresses these issues with a modernized architecture designed for production robotics.

## The DDS Foundation

At the heart of ROS 2 is **DDS (Data Distribution Service)**, an industry-standard middleware specification from the Object Management Group (OMG). DDS provides:

### 1. Decentralized Architecture

Unlike ROS 1's rosmaster, DDS uses **automatic peer discovery**. Nodes find each other dynamically without a central broker, eliminating single points of failure. When you launch multiple ROS 2 nodes on a humanoid robot:

- Nodes broadcast their presence on the network
- Other nodes discover them automatically via multicast
- Communication happens directly between nodes (peer-to-peer)
- No master process to crash and bring down the system

### 2. Quality of Service (QoS) Policies

DDS introduces **QoS policies** that allow fine-grained control over communication reliability and performance. For humanoid robots, this means:

**Reliable QoS** (for critical commands):
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# For motor commands - must not be lost
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

**Best-Effort QoS** (for high-frequency sensor data):
```python
# For camera frames - occasional loss acceptable
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1  # Only keep latest frame
)
```

For a humanoid robot's balance controller, motor commands use **RELIABLE** (every command must arrive), while camera feeds use **BEST_EFFORT** (occasional dropped frames won't cause falls).

### 3. Real-Time Communication

DDS supports **deterministic, low-latency communication** essential for real-time control. ROS 2 can run on real-time operating systems (RTOS) like RT-PREEMPT Linux, enabling:

- Predictable loop times for biped

al locomotion controllers
- Sub-millisecond response times for obstacle avoidance
- Hard real-time guarantees for safety-critical systems

## ROS 2 Architecture Layers

ROS 2 consists of three architectural layers:

### 1. DDS Layer (Middleware)

The foundation providing communication infrastructure. ROS 2 supports multiple DDS implementations:

- **Fast DDS** (default): Optimized for performance, widely used
- **CycloneDDS**: Lightweight, excellent for embedded systems (Jetson Orin)
- **RTI Connext DDS**: Commercial-grade, certified for safety-critical applications

You can switch DDS implementations without changing application code:

```bash
# Use Fast DDS (default)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Use CycloneDDS (better for Jetson)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 2. ROS 2 Core (rcl/rclcpp/rclpy)

The client libraries providing ROS 2 APIs:

- **rcl**: ROS Client Library in C (language-agnostic core)
- **rclcpp**: C++ client library (high performance)
- **rclpy**: Python client library (ease of development)

For humanoid robots, **rclpy** is ideal for high-level planning and perception, while **rclcpp** handles low-level motor control requiring tight timing.

### 3. Application Layer

Your robot application code: nodes, topics, services, actions. This is where you implement humanoid-specific logic like:

- VSLAM for navigation
- Biped locomotion controllers
- Manipulation planning with MoveIt 2
- Voice command processing

## Key Architectural Benefits for Humanoid Robots

### 1. Multi-Robot Support

DDS's decentralized architecture enables seamless multi-robot systems. Imagine a warehouse with multiple humanoid robots:

```python
# Each robot has its own namespace
robot1_node = Node('perception', namespace='/robot1')
robot2_node = Node('perception', namespace='/robot2')

# Robots can discover and communicate automatically
# /robot1/camera/image_raw
# /robot2/camera/image_raw
```

### 2. Security

ROS 2 supports **DDS Security** with:
- Authentication (verify node identity)
- Encryption (protect data in transit)
- Access control (restrict who can publish/subscribe to topics)

For commercial humanoid robots handling sensitive data, this is critical.

### 3. Cross-Platform Support

ROS 2 runs on:
- Linux (Ubuntu, RHEL)
- Windows 10/11
- macOS
- Embedded systems (Jetson, Raspberry Pi)

This flexibility allows developing on Windows/macOS and deploying on Linux/Jetson.

## Communication Patterns in ROS 2

ROS 2 inherits ROS 1's communication paradigms with improvements:

1. **Topics**: Publish/subscribe for continuous data streams (sensor readings, motor commands)
2. **Services**: Request/response for discrete operations (calibrate sensor, get robot state)
3. **Actions**: Long-running tasks with feedback (navigate to goal, manipulate object)

We'll explore these in detail in the next chapter.

## Practical Example: Creating Your First ROS 2 Node

Here's a minimal ROS 2 node demonstrating the architecture:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        # Initialize node with name 'minimal_node'
        super().__init__('minimal_node')

        # Log to console - DDS handles distribution
        self.get_logger().info('ROS 2 node initialized with DDS!')

        # Create a timer (1 Hz) - shows ROS 2's event-driven architecture
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        # Called every second by ROS 2's executor
        self.get_logger().info(f'Heartbeat: {self.counter}')
        self.counter += 1

def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create node instance
    node = MinimalNode()

    # Spin node - ROS 2 executor handles callbacks
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Architectural Elements**:
1. `rclpy.init()`: Initializes DDS middleware
2. `Node` class: Wraps DDS participant (entity in DDS network)
3. `create_timer()`: ROS 2 executor manages callback scheduling
4. `rclpy.spin()`: Event loop processing DDS messages and timers

## Summary

ROS 2's architecture represents a fundamental shift from ROS 1, built on the robust DDS middleware for real-time, distributed robotics. Key takeaways:

- **DDS Foundation**: Decentralized, real-time communication without single points of failure
- **QoS Policies**: Fine-grained control over reliability vs. performance
- **Multiple DDS Implementations**: Choose the best middleware for your hardware
- **Production-Ready**: Security, real-time support, and multi-platform compatibility

For humanoid robots requiring tight sensor-motor control loops, multi-robot coordination, and safety-critical operation, ROS 2's architecture provides the foundation for success.

**Next Chapter**: [Nodes, Topics, and Services](./nodes-topics-services.md) - Learn how ROS 2 nodes communicate

**Additional Resources**:
- [ROS 2 Design Documentation](https://design.ros2.org/)
- [DDS Specification](https://www.omg.org/spec/DDS/)
- [ROS 2 QoS Documentation](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
