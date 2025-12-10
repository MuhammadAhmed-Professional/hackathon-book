---
sidebar_position: 6
title: "Capstone Project"
description: "Build a complete autonomous humanoid system integrating perception, planning, control, and execution"
keywords: [capstone project, system integration, autonomous navigation, manipulation, humanoid robotics, end-to-end system]
---

# Chapter 6: Capstone Project - Autonomous Humanoid System

## Introduction

This capstone project synthesizes all knowledge from Modules 1-4 into a **production-ready autonomous humanoid robot system**. You will build a robot that:

1. **Localizes itself** using sensor fusion (IMU + odometry + vision)
2. **Navigates autonomously** through cluttered environments
3. **Detects and recognizes objects** using computer vision
4. **Plans manipulation tasks** with collision-free motion planning
5. **Executes precise control** with feedback from sensors
6. **Handles failures gracefully** with error recovery and diagnostics

This is a **team-based project** (3-4 students) spanning 3 weeks. You will document your design, implement the system, test rigorously, and present results.

## 6.1 Project Specification

### 6.1.1 Mission Scenario

**Scenario**: A humanoid robot must navigate to a table, identify a target object (red cup), pick it up, and deliver it to a designated drop-off location.

**Tasks**:
1. **Initialization**: Robot starts at origin, loads map of environment
2. **Navigation**: Plan path to table location (3 meters away) and execute
3. **Perception**: Detect all objects on table using YOLO, identify red cup
4. **Pose Estimation**: Compute 6D pose of cup for grasping
5. **Motion Planning**: Plan collision-free arm trajectory to grasp pose
6. **Grasping**: Execute grasp with force control, verify grasp success
7. **Return Navigation**: Navigate back to drop-off location
8. **Placement**: Place cup on target surface with gentle contact
9. **Completion**: Return to home position, report mission success

**Success Criteria**:
- Complete task in < 5 minutes
- Zero collisions (self-collision or environment collision)
- Grasp success rate > 90% over 10 trials
- Position accuracy: < 5cm for navigation, < 1cm for placement
- Real-time performance: all processing < 200ms latency

### 6.1.2 Hardware Requirements

**Minimum Configuration**:
- **Mobile base**: Differential drive or omnidirectional (e.g., Unitree B2, TurtleBot4)
- **Manipulator**: 6+ DOF arm with gripper (e.g., Kinova Gen3, UR5e, or custom)
- **Sensors**:
  - RGB-D camera (RealSense D435i or ZED 2)
  - IMU (9-axis, e.g., BNO055)
  - Wheel encoders (integrated in base)
  - Optional: LiDAR for outdoor environments
- **Compute**: NVIDIA Jetson AGX Orin (32GB) or desktop GPU (RTX 3060+)
- **Power**: Battery with 2+ hour runtime

**Alternative**: Full simulation in Isaac Sim with digital twin of Unitree G1 or custom humanoid

### 6.1.3 Software Stack

**ROS 2 Packages**:
```yaml
Core:
  - ros2_control: Hardware interface and controller manager
  - robot_localization: EKF sensor fusion
  - nav2: Navigation stack
  - moveit2: Motion planning and control

Perception:
  - realsense_ros: Camera driver
  - yolov8_ros: Object detection
  - vision_msgs: Detection message types

Custom Packages:
  - humanoid_description: URDF/SRDF robot model
  - humanoid_bringup: Launch files
  - task_planner: High-level mission sequencer
  - manipulation_controller: Grasp and placement logic
```

## 6.2 System Architecture

### 6.2.1 Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                     Mission Planner                         │
│        (State Machine: Init → Nav → Perceive → Grasp)       │
└──────────────┬──────────────────────────────────────────────┘
               │
       ┌───────┴────────┐
       │                │
       ▼                ▼
┌─────────────┐  ┌─────────────┐
│ Navigation  │  │Manipulation │
│  Controller │  │  Planner    │
└──────┬──────┘  └──────┬──────┘
       │                │
       │        ┌───────┴────────┐
       │        │                │
       ▼        ▼                ▼
┌──────────┐ ┌──────────┐ ┌──────────┐
│  Sensor  │ │ Computer │ │  Motion  │
│  Fusion  │ │  Vision  │ │ Planning │
│  (EKF)   │ │  (YOLO)  │ │ (MoveIt) │
└────┬─────┘ └────┬─────┘ └────┬─────┘
     │            │            │
┌────┴────────────┴────────────┴─────┐
│         Robot Hardware Interface   │
│  (ros2_control, sensor drivers)    │
└────────────────────────────────────┘
```

### 6.2.2 State Machine

```python
from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

class MissionState(Enum):
    IDLE = 0
    NAVIGATING_TO_TABLE = 1
    DETECTING_OBJECTS = 2
    PLANNING_GRASP = 3
    EXECUTING_GRASP = 4
    NAVIGATING_TO_DROPOFF = 5
    PLACING_OBJECT = 6
    RETURNING_HOME = 7
    MISSION_COMPLETE = 8
    ERROR = 9

class MissionPlanner(Node):
    """
    High-level mission planner using state machine.
    """
    def __init__(self):
        super().__init__('mission_planner')

        self.state = MissionState.IDLE

        # Service clients
        self.navigate_client = self.create_client(NavigateToPose, '/navigate_to_pose')
        self.detect_client = self.create_client(DetectObjects, '/detect_objects')
        self.grasp_client = self.create_client(GraspObject, '/grasp_object')

        # Timer for state machine updates
        self.timer = self.create_timer(0.1, self.update_state_machine)

        self.get_logger().info("Mission Planner initialized")

    def update_state_machine(self):
        """Main state machine logic."""
        if self.state == MissionState.IDLE:
            self.get_logger().info("Starting mission...")
            self.state = MissionState.NAVIGATING_TO_TABLE
            self.send_navigation_goal(x=3.0, y=0.0, theta=0.0)

        elif self.state == MissionState.NAVIGATING_TO_TABLE:
            # Wait for navigation to complete (checked via action feedback)
            pass

        elif self.state == MissionState.DETECTING_OBJECTS:
            self.get_logger().info("Detecting objects on table...")
            # Call detection service
            self.state = MissionState.PLANNING_GRASP

        # ... (implement other states)

    def send_navigation_goal(self, x: float, y: float, theta: float):
        """Send navigation goal to Nav2."""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        # ... set orientation from theta

        # Call Nav2 action server (implementation details omitted)
```

## 6.3 Implementation Guide

### 6.3.1 Phase 1: Setup and Testing (Week 1)

**Day 1-2: Hardware Assembly and Bringup**
```bash
# 1. Install dependencies
sudo apt install ros-humble-nav2-bringup ros-humble-moveit
pip install ultralytics opencv-python

# 2. Clone and build packages
cd ~/ros2_ws/src
git clone https://github.com/your_team/humanoid_capstone.git
cd ~/ros2_ws
colcon build --symlink-install

# 3. Test hardware interfaces
ros2 launch humanoid_bringup test_hardware.launch.py

# Check topics
ros2 topic list
# Expected: /camera/image_raw, /imu/data, /odom, /joint_states
```

**Day 3-4: Sensor Calibration**
- Calibrate camera intrinsics using `camera_calibration` package
- Tune EKF covariances in `robot_localization`
- Verify odometry accuracy (drive 1m straight, measure error)

**Day 5-7: Individual Component Testing**
```bash
# Test navigation
ros2 launch humanoid_bringup navigation.launch.py
# Use RViz to send 2D Nav Goals

# Test object detection
ros2 run humanoid_perception yolo_node.py
ros2 run rqt_image_view rqt_image_view /detections/image

# Test motion planning
ros2 launch humanoid_moveit demo.launch.py
# Plan to pose in MoveIt GUI
```

### 6.3.2 Phase 2: Integration (Week 2)

**Mission Planner Development**:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray
import numpy as np

class CapstoneMissionNode(Node):
    """Complete capstone mission controller."""

    def __init__(self):
        super().__init__('capstone_mission')

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.moveit_client = ActionClient(self, MoveGroup, '/move_action')

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.latest_detections = []
        self.get_logger().info("Capstone mission node ready")

    def detection_callback(self, msg):
        """Store latest object detections."""
        self.latest_detections = msg.detections

    def run_mission(self):
        """Execute full mission sequence."""
        try:
            # Step 1: Navigate to table
            self.get_logger().info("Step 1: Navigating to table...")
            table_pose = self.create_pose(x=3.0, y=0.0, theta=0.0)
            if not self.navigate_to_pose(table_pose):
                raise RuntimeError("Navigation failed")

            # Step 2: Detect objects
            self.get_logger().info("Step 2: Detecting objects...")
            rclpy.spin_once(self, timeout_sec=2.0)  # Wait for detections
            target = self.find_target_object("cup")
            if target is None:
                raise RuntimeError("Target object not found")

            # Step 3: Estimate grasp pose
            self.get_logger().info("Step 3: Computing grasp pose...")
            grasp_pose = self.compute_grasp_pose(target)

            # Step 4: Plan and execute grasp
            self.get_logger().info("Step 4: Executing grasp...")
            if not self.execute_grasp(grasp_pose):
                raise RuntimeError("Grasp failed")

            # Step 5: Navigate to drop-off
            self.get_logger().info("Step 5: Navigating to drop-off...")
            dropoff_pose = self.create_pose(x=0.0, y=0.0, theta=np.pi)
            if not self.navigate_to_pose(dropoff_pose):
                raise RuntimeError("Return navigation failed")

            # Step 6: Place object
            self.get_logger().info("Step 6: Placing object...")
            if not self.place_object():
                raise RuntimeError("Placement failed")

            self.get_logger().info("✓ Mission completed successfully!")
            return True

        except Exception as e:
            self.get_logger().error(f"Mission failed: {e}")
            return False

    def navigate_to_pose(self, goal_pose: PoseStamped) -> bool:
        """Navigate to target pose using Nav2."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return result_future.result().status == 4  # SUCCEEDED

    def find_target_object(self, class_name: str):
        """Find object with given class name in detections."""
        for detection in self.latest_detections:
            if detection.results[0].hypothesis.class_id == class_name:
                return detection
        return None

    def compute_grasp_pose(self, detection) -> PoseStamped:
        """Compute 6D grasp pose from detection."""
        # Simplified: use detection bbox center + fixed approach
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "camera_link"
        grasp_pose.pose.position.x = detection.bbox.center.position.x / 1000.0
        grasp_pose.pose.position.y = detection.bbox.center.position.y / 1000.0
        grasp_pose.pose.position.z = 0.5  # 50cm above table
        # Set orientation for top-down grasp
        grasp_pose.pose.orientation.w = 1.0
        return grasp_pose

    def execute_grasp(self, grasp_pose: PoseStamped) -> bool:
        """Plan and execute grasp motion."""
        # Use MoveIt to plan to grasp pose
        # (Implementation uses MoveIt Python API)
        return True  # Placeholder

    def place_object(self) -> bool:
        """Place object at drop-off location."""
        # Plan placement motion and open gripper
        return True  # Placeholder

    def create_pose(self, x: float, y: float, theta: float) -> PoseStamped:
        """Create PoseStamped from x, y, theta."""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        # Convert theta to quaternion
        pose.pose.orientation.z = np.sin(theta / 2)
        pose.pose.orientation.w = np.cos(theta / 2)
        return pose

def main():
    rclpy.init()
    node = CapstoneMissionNode()

    # Run mission
    success = node.run_mission()

    if success:
        print("\n=== MISSION SUCCESS ===")
    else:
        print("\n=== MISSION FAILED ===")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6.3.3 Phase 3: Testing and Optimization (Week 3)

**Testing Protocol**:

1. **Unit Tests** (each component):
   ```bash
   # Test navigation accuracy
   ros2 run humanoid_testing test_navigation.py
   # Expected: < 5cm error over 3m distance

   # Test detection precision
   ros2 run humanoid_testing test_detection.py
   # Expected: > 90% precision on test dataset

   # Test grasp success rate
   ros2 run humanoid_testing test_grasping.py
   # Expected: > 90% success rate
   ```

2. **Integration Tests** (end-to-end):
   ```bash
   # Run full mission 10 times
   for i in {1..10}; do
       ros2 run humanoid_capstone mission_node.py
   done

   # Analyze results
   ros2 run humanoid_capstone analyze_results.py
   # Outputs: Success rate, avg time, failure modes
   ```

3. **Performance Profiling**:
   ```bash
   # Record execution trace
   ros2 bag record -a

   # Analyze latencies
   ros2 run humanoid_capstone profile_latency.py
   # Check: Perception < 50ms, Planning < 500ms, Total < 200ms
   ```

## 6.4 Deliverables

### 6.4.1 Code Repository

**Structure**:
```
humanoid_capstone/
├── README.md                    # Setup instructions
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── bringup.launch.py       # Full system launch
│   └── simulation.launch.py    # Isaac Sim launch
├── config/
│   ├── ekf.yaml                # Sensor fusion params
│   ├── nav2_params.yaml        # Navigation tuning
│   └── moveit_config.yaml      # Motion planning config
├── src/
│   ├── mission_planner.py      # State machine
│   ├── navigation_controller.py
│   ├── perception_node.py
│   └── manipulation_controller.py
├── tests/
│   ├── test_navigation.py
│   ├── test_detection.py
│   └── test_integration.py
└── docs/
    ├── architecture.md
    ├── api_reference.md
    └── user_guide.md
```

### 6.4.2 Technical Report

**Required Sections** (10-15 pages):

1. **Executive Summary** (1 page)
   - Mission objective and success criteria
   - Summary of results and key metrics

2. **System Architecture** (2 pages)
   - Block diagram with component interactions
   - Technology stack justification

3. **Design Decisions** (3 pages)
   - Sensor fusion: EKF vs. particle filter
   - Motion planning: RRT* vs. other algorithms
   - Control: PID vs. MPC trade-offs

4. **Implementation** (3 pages)
   - Key algorithms with pseudocode
   - ROS 2 integration architecture
   - Hardware-software interface

5. **Experimental Results** (3 pages)
   - Quantitative metrics (table format)
   - Success rate analysis
   - Failure mode classification
   - Performance profiling (latency breakdown)

6. **Discussion** (2 pages)
   - Comparison to state-of-the-art systems
   - Limitations and future work

7. **References** (1 page)
   - Academic papers, ROS packages, hardware datasheets

### 6.4.3 Video Demonstration

**Requirements**:
- 3-5 minute video showing:
  1. System overview (30 seconds)
  2. Hardware setup (30 seconds)
  3. Full mission execution (2-3 minutes)
  4. Quantitative results overlay (30 seconds)
- Upload to YouTube with public link
- Include in README

## 6.5 Evaluation Rubric

**Total: 100 points**

| Category | Points | Criteria |
|----------|--------|----------|
| **Functionality** | 40 | - Mission completion (20 pts)<br />- Grasp success rate (10 pts)<br />- Collision avoidance (10 pts) |
| **Technical Quality** | 30 | - Code quality (10 pts)<br />- System architecture (10 pts)<br />- Real-time performance (10 pts) |
| **Documentation** | 20 | - Technical report quality (10 pts)<br />- Code documentation (5 pts)<br />- Video demonstration (5 pts) |
| **Innovation** | 10 | - Advanced features beyond requirements<br />- Novel solutions to challenges |

**Grading Scale**:
- A (90-100): Exceeds all requirements, publication-quality work
- B (80-89): Meets all requirements with minor issues
- C (70-79): Meets most requirements, significant issues present
- D (60-69): Partial completion, major functionality missing
- F (&lt;60): Does not meet minimum requirements

## 6.6 Advanced Challenges (Optional)

For teams seeking additional challenge:

1. **Multi-Object Manipulation**: Pick and place 5 different objects in sequence
2. **Dynamic Obstacles**: Navigate around moving obstacles (simulated humans)
3. **Learning-Based Grasping**: Train deep RL policy for grasp pose estimation
4. **Human-Robot Interaction**: Accept voice commands using speech recognition
5. **Outdoor Navigation**: Deploy on campus with GPS + LiDAR SLAM

**Bonus Points**: +10 points for each advanced challenge successfully completed

## Summary

This capstone project integrates:
- **Sensor fusion** (Chapter 2): EKF for robust localization
- **Motion planning** (Chapter 3): RRT* for navigation, MoveIt for manipulation
- **Control systems** (Chapter 4): PID for tracking, force control for grasping
- **Computer vision** (Chapter 5): YOLO for detection, pose estimation

**Key Takeaway**: Building a complete autonomous robot requires not just implementing algorithms, but **system engineering**—managing complexity, handling failures, optimizing performance, and rigorous testing.

**Congratulations** on completing this textbook! You now have the skills to build production humanoid robots used in industry and research.

## References

1. Thrun, S., et al. (2005). *Probabilistic Robotics*. MIT Press.
2. LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.
3. Spong, M. W., et al. (2020). *Robot Modeling and Control*. Wiley.
4. Murphy, R. R. (2019). *Introduction to AI Robotics*. MIT Press.
5. ROS 2 Documentation: https://docs.ros.org/
6. MoveIt 2 Tutorials: https://moveit.picknik.ai/
7. Nav2 Documentation: https://navigation.ros.org/
