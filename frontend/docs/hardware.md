---
id: hardware
title: Hardware Requirements
sidebar_label: Hardware Requirements
sidebar_position: 10
description: Detailed hardware specifications for Physical AI & Humanoid Robotics course including workstation requirements, Jetson Orin kits, RealSense cameras, robot platforms, and cloud-based alternatives.
keywords: [hardware requirements, rtx gpu, jetson orin, realsense camera, unitree robot, isaac sim, cloud alternatives]
---

# Hardware Requirements

This chapter provides comprehensive hardware specifications for the Physical AI & Humanoid Robotics course. While the course focuses on simulation for safety and accessibility, having access to the recommended hardware enables hands-on deployment and edge computing experiments.

## Workstation Requirements

### Minimum Specifications (Simulation-Focused)

For running ROS 2, Gazebo, and basic Isaac Sim scenarios:

- **CPU**: Intel Core i7-12700K or AMD Ryzen 7 5800X (8+ cores)
- **GPU**: NVIDIA RTX 3060 (12GB VRAM)
- **RAM**: 32GB DDR4
- **Storage**: 500GB NVMe SSD
- **OS**: Ubuntu 22.04 LTS (native installation recommended, not WSL)
- **Network**: 1Gbps Ethernet

**Estimated Cost**: $1,500 - $2,000 USD

### Recommended Specifications (Full Isaac Sim + Real-Time Perception)

For photorealistic Isaac Sim, synthetic data generation, and hardware-accelerated Isaac ROS:

- **CPU**: Intel Core i9-13900K or AMD Ryzen 9 7950X (16+ cores)
- **GPU**: NVIDIA RTX 4070 Ti or RTX 4080 (16GB+ VRAM)
- **RAM**: 64GB DDR5
- **Storage**: 1TB NVMe SSD (Gen4)
- **OS**: Ubuntu 22.04 LTS
- **Network**: 1Gbps Ethernet or Wi-Fi 6

**Estimated Cost**: $2,500 - $3,500 USD

### Why NVIDIA RTX GPUs?

NVIDIA RTX GPUs are essential for:
- **Isaac Sim**: Ray-tracing for photorealistic simulation requires RTX cores
- **Isaac ROS**: Hardware-accelerated VSLAM and perception use CUDA cores and Tensor Cores
- **Synthetic Data Generation**: Training perception models on GPU-generated data
- **Real-Time Performance**: Nav2 + VSLAM at 30+ FPS requires GPU acceleration

**Minimum VRAM**: 12GB for basic Isaac Sim, 16GB+ recommended for complex scenes

## Edge Computing: Jetson Orin Kits

For deploying AI perception on humanoid robots, NVIDIA Jetson Orin kits provide edge inference:

### Jetson Orin Nano (8GB)

- **CPU**: 6-core Arm Cortex-A78AE
- **GPU**: 1024-core NVIDIA Ampere (up to 40 TOPS AI)
- **RAM**: 8GB LPDDR5
- **Power**: 7W - 15W
- **Use Case**: Lightweight VSLAM, object detection on quadrupeds (Unitree Go2)

**Cost**: ~$500 USD

### Jetson Orin NX (16GB)

- **CPU**: 8-core Arm Cortex-A78AE
- **GPU**: 1024-core NVIDIA Ampere (up to 100 TOPS AI)
- **RAM**: 16GB LPDDR5
- **Power**: 10W - 25W
- **Use Case**: Full Isaac ROS stack, VSLAM + Nav2, humanoid perception (Unitree G1)

**Cost**: ~$900 USD

### Jetson AGX Orin (32GB/64GB)

- **CPU**: 12-core Arm Cortex-A78AE
- **GPU**: 2048-core NVIDIA Ampere (up to 275 TOPS AI)
- **RAM**: 32GB or 64GB LPDDR5
- **Power**: 15W - 60W
- **Use Case**: Multi-sensor fusion, advanced perception, manipulation planning

**Cost**: $1,600 (32GB) - $2,000 (64GB) USD

## Vision Sensors: RealSense Cameras

### Intel RealSense D435i

- **Depth Technology**: Stereo vision (active IR)
- **Depth Range**: 0.3m - 3m
- **RGB Resolution**: 1920x1080 @ 30fps
- **IMU**: Built-in accelerometer and gyroscope
- **Interface**: USB 3.0
- **Use Case**: VSLAM, obstacle detection, object recognition

**Cost**: ~$300 USD

### Intel RealSense D455

- **Depth Range**: 0.4m - 6m (longer range than D435i)
- **Field of View**: 86° x 57° (wider than D435i)
- **Use Case**: Navigation in larger spaces, outdoor scenarios

**Cost**: ~$400 USD

## Robot Platforms

### Unitree Go2 (Quadruped)

- **DOF**: 12 (3 per leg)
- **Payload**: 5kg
- **Speed**: 5 m/s
- **Battery**: 2-4 hours
- **Sensors**: LiDAR, depth cameras, IMU
- **Use Case**: Learning locomotion, Nav2 testing, outdoor navigation

**Cost**: ~$2,000 - $3,000 USD (varies by model: Air, Pro, Edu)

### Unitree G1 (Humanoid)

- **DOF**: 23+ (arms, legs, torso, head)
- **Height**: ~1.3m
- **Payload**: Arms can carry ~2kg each
- **Sensors**: RealSense cameras, LiDAR, IMU, force sensors
- **Use Case**: Full humanoid development, manipulation, VLA capstone

**Cost**: ~$16,000 - $20,000 USD (educational pricing may vary)

### Alternatives

- **Boston Dynamics Spot** (Quadruped): ~$75,000 (enterprise-focused)
- **Figure 01** (Humanoid): Research platform, pricing not public
- **Robotic Arms** (e.g., UR5e, Franka Emika): $20,000 - $50,000 (for manipulation-only projects)

## Cloud-Based Alternatives

For students without access to RTX GPUs or physical robots:

### AWS RoboMaker

- **Service**: Cloud-based simulation and deployment platform
- **Features**: Gazebo simulation, ROS 2 support, fleet management
- **Pricing**: Pay-as-you-go (simulation: ~$0.45/hour, deployment: varies)
- **Limitations**: No Isaac Sim support, limited to Gazebo

**Use Case**: Gazebo simulation, ROS 2 testing without local GPU

### NVIDIA Omniverse Cloud

- **Service**: Cloud-based Isaac Sim and Omniverse access
- **Features**: Full Isaac Sim with RTX rendering, synthetic data generation
- **Pricing**: Free tier available, enterprise pricing for extended use
- **Requirements**: High-speed internet (1Gbps+ recommended for streaming)

**Use Case**: Isaac Sim without local RTX GPU

### Google Cloud / Azure for Compute

- **VMs with NVIDIA T4/V100 GPUs**: $0.50 - $3.00/hour depending on GPU
- **Use Case**: Isaac ROS training, perception model training, batch simulations

## Summary: Hardware Recommendations by Course Phase

| **Course Phase** | **Required Hardware** | **Optional Hardware** |
|------------------|-----------------------|-----------------------|
| **Weeks 1-5 (ROS 2)** | Ubuntu 22.04 VM or native (8GB RAM) | None |
| **Weeks 6-7 (Gazebo/Unity)** | Ubuntu 22.04, 16GB RAM, GTX 1660+ | RealSense camera |
| **Weeks 8-10 (Isaac)** | Ubuntu 22.04, RTX 4070 Ti+, 64GB RAM | Jetson Orin NX, RealSense D435i |
| **Week 13 (VLA Capstone)** | All of the above + OpenAI API | Unitree Go2/G1, microphone, speaker |

## Cost Breakdown

| **Configuration** | **Total Cost** | **What You Get** |
|-------------------|----------------|------------------|
| **Simulation-Only** | $1,500 - $2,000 | Workstation (RTX 3060, 32GB RAM) |
| **Simulation + Edge** | $2,500 - $3,500 | Workstation (RTX 4070 Ti+, 64GB RAM) + Jetson Orin NX + RealSense D435i |
| **Full Hardware Setup** | $20,000 - $25,000 | Simulation + Edge + Unitree G1 humanoid |
| **Cloud Alternative** | $0 - $500/month | AWS RoboMaker + Omniverse Cloud (pay-as-you-go) |

## Recommendations

1. **Start with simulation**: Use a workstation or cloud for Modules 1-3
2. **Add edge compute**: Get Jetson Orin NX for Module 3 (Isaac ROS deployment)
3. **Physical robot (optional)**: Only if budget allows and you want real-world deployment

Most learning outcomes can be achieved through simulation. Physical hardware is valuable for final deployment but not required for course completion.

---

**Related**: [Introduction](./intro.md) | [Weekly Breakdown](./weekly-breakdown.md)
