---
id: testing-validation
title: "Testing & Validation for Robotics Systems"
sidebar_label: "Testing & Validation"
sidebar_position: 1
description: "Comprehensive guide to testing ROS 2 robotics systems: unit testing with pytest, integration testing, simulation-based testing in Gazebo, hardware-in-the-loop (HIL), and CI/CD pipelines for autonomous humanoid robots."
keywords: [ros2 testing, pytest, integration testing, gazebo testing, hardware-in-loop, ci/cd robotics, launch_testing, pytest-cov]
---

# Testing & Validation for Robotics Systems

## Introduction

Testing robotics software is fundamentally different from testing traditional software. A bug in a web application might crash a browser; a bug in a humanoid robot control system could cause physical damage or injury. This chapter teaches you industry-standard testing methodologies for ROS 2 robotics systems, from unit tests to full hardware-in-the-loop validation.

**Why Robotics Testing is Hard**:
- **Physical Constraints**: Robots interact with the real world—gravity, friction, collisions
- **Multi-Component Systems**: Dozens of nodes, sensors, actuators must work in concert
- **Non-Deterministic Behavior**: Sensor noise, network latency, environment variability
- **Safety-Critical**: Failures can damage hardware or harm humans
- **Expensive Hardware**: Can't test every scenario on real robots

**Learning Objectives**:
- Write unit tests for ROS 2 nodes using pytest and unittest
- Implement integration tests for multi-node systems
- Use Gazebo for simulation-based regression testing
- Deploy hardware-in-the-loop (HIL) testing strategies
- Build CI/CD pipelines with GitHub Actions
- Achieve >80% code coverage with pytest-cov
- Apply safety validation techniques (collision testing, workspace limits)

## The Robotics Testing Pyramid

Traditional software uses the testing pyramid (many unit tests, fewer integration tests, minimal E2E tests). Robotics adds a fourth layer:

```
         ┌─────────────────┐
         │  HIL Testing    │  (Real hardware + real environment)
         │   (Hours/Days)  │
         ├─────────────────┤
         │  Sim Testing    │  (Gazebo/Isaac Sim)
         │   (Minutes)     │
         ├─────────────────┤
         │ Integration     │  (Multi-node, launch_testing)
         │   (Seconds)     │
         ├─────────────────┤
         │   Unit Tests    │  (Individual nodes, mocks)
         │ (Milliseconds)  │
         └─────────────────┘
```

**Robotics-Specific Testing Strategy**:
1. **Unit Tests (70%)**: Test individual nodes with mocked sensors/actuators
2. **Integration Tests (20%)**: Test node communication and coordination
3. **Simulation Tests (8%)**: Validate behavior in Gazebo with physics
4. **HIL Tests (2%)**: Final validation on real hardware

## Unit Testing ROS 2 Nodes

Unit tests verify individual ROS 2 nodes in isolation using mocks for sensors, actuators, and communication.

### Testing Framework: pytest + unittest

ROS 2 supports both Python `unittest` (built-in) and `pytest` (preferred for modern robotics). We'll use **pytest** for cleaner syntax and better fixtures.

**Installation**:
```bash
pip3 install pytest pytest-cov pytest-mock
sudo apt install ros-humble-launch-testing
```

### Example 1: Testing a Simple Publisher Node

**Node Under Test** (`joint_state_publisher.py`):
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        self.joint_names = ['shoulder_pitch', 'elbow', 'wrist']
        self.positions = [0.0, 0.0, 0.0]

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        self.publisher.publish(msg)
```

**Unit Test** (`test_joint_state_publisher.py`):
```python
#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from joint_state_publisher import JointStatePublisher

@pytest.fixture
def joint_publisher():
    """Fixture to create and destroy node for each test"""
    rclpy.init()
    node = JointStatePublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_node_name(joint_publisher):
    """Test node has correct name"""
    assert joint_publisher.get_name() == 'joint_state_publisher'

def test_publisher_created(joint_publisher):
    """Test publisher is created on correct topic"""
    publishers = joint_publisher.get_publishers_info_by_topic('/joint_states')
    assert len(publishers) == 1
    assert publishers[0].topic_type == 'sensor_msgs/msg/JointState'

def test_joint_names(joint_publisher):
    """Test joint names are correctly initialized"""
    assert len(joint_publisher.joint_names) == 3
    assert 'shoulder_pitch' in joint_publisher.joint_names
    assert 'elbow' in joint_publisher.joint_names

def test_message_publication():
    """Test that messages are published with correct format"""
    rclpy.init()
    publisher_node = JointStatePublisher()

    # Create subscriber to capture messages
    received_messages = []

    class TestSubscriber(Node):
        def __init__(self):
            super().__init__('test_subscriber')
            self.subscription = self.create_subscription(
                JointState,
                '/joint_states',
                lambda msg: received_messages.append(msg),
                10
            )

    subscriber_node = TestSubscriber()

    # Spin until message received (with timeout)
    import time
    start_time = time.time()
    while len(received_messages) == 0 and (time.time() - start_time) < 2.0:
        rclpy.spin_once(publisher_node, timeout_sec=0.1)
        rclpy.spin_once(subscriber_node, timeout_sec=0.1)

    # Assertions
    assert len(received_messages) > 0, "No messages received"
    msg = received_messages[0]
    assert len(msg.name) == 3
    assert len(msg.position) == 3
    assert msg.name == ['shoulder_pitch', 'elbow', 'wrist']

    # Cleanup
    publisher_node.destroy_node()
    subscriber_node.destroy_node()
    rclpy.shutdown()
```

**Run Tests**:
```bash
pytest test_joint_state_publisher.py -v
```

### Example 2: Testing with Mocked Sensors

For nodes that depend on sensors (cameras, LiDAR, IMU), use **mocks** to avoid hardware dependencies.

**Node Under Test** (`obstacle_detector.py`):
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.publisher = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.min_safe_distance = 0.5  # meters

    def scan_callback(self, msg):
        # Check if any laser reading is below threshold
        min_distance = min(msg.ranges)
        obstacle_detected = min_distance < self.min_safe_distance

        detection_msg = Bool()
        detection_msg.data = obstacle_detected
        self.publisher.publish(detection_msg)
```

**Unit Test with Mocked LiDAR** (`test_obstacle_detector.py`):
```python
#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from obstacle_detector import ObstacleDetector

def create_mock_scan(min_range, max_range, num_readings=360):
    """Helper to create mock LaserScan messages"""
    msg = LaserScan()
    msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    msg.angle_min = 0.0
    msg.angle_max = 6.28  # 2*pi
    msg.angle_increment = 6.28 / num_readings
    msg.range_min = 0.1
    msg.range_max = 10.0
    msg.ranges = [max_range] * num_readings
    msg.ranges[0] = min_range  # Put obstacle at 0 degrees
    return msg

def test_obstacle_detected():
    """Test obstacle is detected when LiDAR reads distance < threshold"""
    rclpy.init()
    detector = ObstacleDetector()

    received_detections = []

    class DetectionSubscriber(Node):
        def __init__(self):
            super().__init__('detection_subscriber')
            self.subscription = self.create_subscription(
                Bool, '/obstacle_detected',
                lambda msg: received_detections.append(msg.data),
                10
            )

    subscriber = DetectionSubscriber()

    # Create publisher to send mock LiDAR data
    class MockLidarPublisher(Node):
        def __init__(self):
            super().__init__('mock_lidar')
            self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        def publish_scan(self, scan_msg):
            self.publisher.publish(scan_msg)

    mock_lidar = MockLidarPublisher()

    # Test case 1: Obstacle at 0.3m (below 0.5m threshold)
    close_scan = create_mock_scan(min_range=0.3, max_range=5.0)
    mock_lidar.publish_scan(close_scan)

    # Spin to process message
    for _ in range(10):
        rclpy.spin_once(detector, timeout_sec=0.1)
        rclpy.spin_once(subscriber, timeout_sec=0.1)
        rclpy.spin_once(mock_lidar, timeout_sec=0.1)

    assert len(received_detections) > 0
    assert received_detections[-1] is True, "Obstacle should be detected at 0.3m"

    # Test case 2: No obstacle at 2.0m (above threshold)
    received_detections.clear()
    far_scan = create_mock_scan(min_range=2.0, max_range=5.0)
    mock_lidar.publish_scan(far_scan)

    for _ in range(10):
        rclpy.spin_once(detector, timeout_sec=0.1)
        rclpy.spin_once(subscriber, timeout_sec=0.1)
        rclpy.spin_once(mock_lidar, timeout_sec=0.1)

    assert len(received_detections) > 0
    assert received_detections[-1] is False, "No obstacle at 2.0m"

    # Cleanup
    detector.destroy_node()
    subscriber.destroy_node()
    mock_lidar.destroy_node()
    rclpy.shutdown()
```

**Key Testing Patterns**:
- **Fixtures**: Use `@pytest.fixture` to setup/teardown nodes
- **Mock Messages**: Create helper functions to generate sensor messages
- **Timeouts**: Use `spin_once()` with timeouts to avoid hanging tests
- **Assertions**: Test both expected and edge cases (no obstacle, close obstacle)

## Integration Testing with launch_testing

Integration tests validate that multiple ROS 2 nodes work correctly together. ROS 2 provides `launch_testing` for this purpose.

### Example: Testing Multi-Node Navigation System

**Launch File** (`test_navigation.launch.py`):
```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import pytest

def generate_test_description():
    """Launch nodes required for navigation test"""

    # Launch obstacle detector
    obstacle_detector_node = Node(
        package='my_robot_pkg',
        executable='obstacle_detector',
        name='obstacle_detector'
    )

    # Launch path planner
    path_planner_node = Node(
        package='my_robot_pkg',
        executable='path_planner',
        name='path_planner'
    )

    # Launch mock LiDAR
    mock_lidar_node = Node(
        package='my_robot_pkg',
        executable='mock_lidar',
        name='mock_lidar'
    )

    return (
        LaunchDescription([
            obstacle_detector_node,
            path_planner_node,
            mock_lidar_node,
            ReadyToTest()
        ]),
        {
            'obstacle_detector': obstacle_detector_node,
            'path_planner': path_planner_node,
            'mock_lidar': mock_lidar_node,
        }
    )

@pytest.mark.launch_test
def test_obstacle_triggers_replanning():
    """Test that obstacle detection triggers path replanning"""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool
    from nav_msgs.msg import Path

    rclpy.init()

    received_paths = []

    class PathSubscriber(Node):
        def __init__(self):
            super().__init__('path_subscriber')
            self.subscription = self.create_subscription(
                Path, '/planned_path',
                lambda msg: received_paths.append(msg),
                10
            )

    subscriber = PathSubscriber()

    # Wait for initial path
    import time
    timeout = 5.0
    start_time = time.time()
    while len(received_paths) == 0 and (time.time() - start_time) < timeout:
        rclpy.spin_once(subscriber, timeout_sec=0.1)

    assert len(received_paths) > 0, "No initial path received"
    initial_path_length = len(received_paths[0].poses)

    # Simulate obstacle detection
    obstacle_pub = Node('obstacle_simulator')
    detection_publisher = obstacle_pub.create_publisher(Bool, '/obstacle_detected', 10)

    obstacle_msg = Bool()
    obstacle_msg.data = True
    detection_publisher.publish(obstacle_msg)

    # Wait for replanned path
    received_paths.clear()
    start_time = time.time()
    while len(received_paths) == 0 and (time.time() - start_time) < timeout:
        rclpy.spin_once(subscriber, timeout_sec=0.1)
        rclpy.spin_once(obstacle_pub, timeout_sec=0.1)

    assert len(received_paths) > 0, "No replanned path received"
    replanned_path_length = len(received_paths[0].poses)

    # Replanned path should be different (longer to avoid obstacle)
    assert replanned_path_length != initial_path_length, "Path unchanged after obstacle"

    subscriber.destroy_node()
    obstacle_pub.destroy_node()
    rclpy.shutdown()
```

**Run Integration Tests**:
```bash
launch_test test_navigation.launch.py
```

## Simulation-Based Testing with Gazebo

Simulation testing validates robot behavior in realistic physics environments before deploying to hardware.

### Example: Gazebo Regression Test for Balance

**Test Script** (`test_balance_gazebo.py`):
```python
#!/usr/bin/env python3
"""
Test humanoid balance in Gazebo simulation
Validates that robot maintains upright posture under disturbances
"""
import pytest
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, ApplyBodyWrench
from geometry_msgs.msg import Wrench, Vector3
from sensor_msgs.msg import Imu
import subprocess
import time
import math

class GazeboBalanceTest(Node):
    def __init__(self):
        super().__init__('gazebo_balance_test')

        # Service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.wrench_client = self.create_client(ApplyBodyWrench, '/apply_body_wrench')

        # Subscribe to IMU
        self.imu_data = []
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data',
            lambda msg: self.imu_data.append(msg),
            10
        )

    def spawn_humanoid(self, urdf_path):
        """Spawn humanoid robot in Gazebo"""
        with open(urdf_path, 'r') as urdf_file:
            robot_description = urdf_file.read()

        request = SpawnEntity.Request()
        request.name = 'test_humanoid'
        request.xml = robot_description
        request.initial_pose.position.z = 1.0  # Spawn 1m above ground

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        return future.result().success

    def apply_push(self, force_x=10.0, duration=0.5):
        """Apply horizontal force to robot torso (simulate push)"""
        request = ApplyBodyWrench.Request()
        request.body_name = 'test_humanoid::torso'
        request.wrench.force = Vector3(x=force_x, y=0.0, z=0.0)
        request.duration.sec = int(duration)
        request.duration.nanosec = int((duration % 1) * 1e9)

        future = self.wrench_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        return future.result().success

    def get_tilt_angle(self):
        """Calculate robot tilt from IMU orientation (quaternion to euler)"""
        if not self.imu_data:
            return None

        imu = self.imu_data[-1]
        q = imu.orientation

        # Convert quaternion to roll angle (tilt around x-axis)
        roll = math.atan2(
            2 * (q.w * q.x + q.y * q.z),
            1 - 2 * (q.x * q.x + q.y * q.y)
        )

        return math.degrees(roll)

@pytest.fixture
def gazebo_world():
    """Start Gazebo simulation before tests"""
    gazebo_process = subprocess.Popen([
        'gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'
    ])

    time.sleep(3)  # Wait for Gazebo to start
    yield gazebo_process

    gazebo_process.terminate()
    gazebo_process.wait()

def test_humanoid_balance_after_push(gazebo_world):
    """Test: Robot should recover balance after 10N push"""
    rclpy.init()
    test_node = GazeboBalanceTest()

    # Spawn robot
    urdf_path = '/path/to/humanoid.urdf'
    assert test_node.spawn_humanoid(urdf_path), "Failed to spawn robot"

    # Wait for robot to stabilize
    time.sleep(2.0)
    for _ in range(50):
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Record initial tilt
    initial_tilt = test_node.get_tilt_angle()
    assert initial_tilt is not None
    assert abs(initial_tilt) < 5.0, f"Robot not upright initially: {initial_tilt}°"

    # Apply 10N push
    assert test_node.apply_push(force_x=10.0, duration=0.5), "Failed to apply force"

    # Wait for recovery
    time.sleep(3.0)
    for _ in range(100):
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Check final tilt
    final_tilt = test_node.get_tilt_angle()
    assert final_tilt is not None
    assert abs(final_tilt) < 10.0, f"Robot fell over: {final_tilt}°"

    get_logger().info(f"Balance test passed: {initial_tilt}° → {final_tilt}°")

    test_node.destroy_node()
    rclpy.shutdown()

def test_humanoid_falls_with_strong_push(gazebo_world):
    """Test: Robot should fall with 100N push (negative test)"""
    rclpy.init()
    test_node = GazeboBalanceTest()

    assert test_node.spawn_humanoid('/path/to/humanoid.urdf')
    time.sleep(2.0)

    # Apply strong push
    assert test_node.apply_push(force_x=100.0, duration=0.5)

    time.sleep(3.0)
    for _ in range(100):
        rclpy.spin_once(test_node, timeout_sec=0.1)

    final_tilt = test_node.get_tilt_angle()
    assert abs(final_tilt) > 45.0, "Robot should have fallen with 100N push"

    test_node.destroy_node()
    rclpy.shutdown()
```

**Run Gazebo Tests**:
```bash
pytest test_balance_gazebo.py -v -s
```

## Hardware-in-the-Loop (HIL) Testing

HIL testing validates software on real hardware with controlled inputs. This is the final testing stage before deployment.

### Example: HIL Test for Motor Controller

**Setup**: Real robot motors connected via CAN bus, simulated sensor inputs

```python
#!/usr/bin/env python3
"""
Hardware-in-the-Loop test for motor controller
Tests actual motor hardware with scripted sensor inputs
"""
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import can  # python-can library for CAN bus

class MotorHILTest(Node):
    def __init__(self):
        super().__init__('motor_hil_test')

        # Publisher for desired joint positions
        self.cmd_pub = self.create_publisher(
            Float64, '/joint/shoulder/command', 10
        )

        # Subscriber for actual joint positions
        self.actual_positions = []
        self.state_sub = self.create_subscription(
            JointState, '/joint_states',
            lambda msg: self.actual_positions.append(msg.position[0]),
            10
        )

        # CAN bus interface for direct motor communication
        self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan')

    def send_position_command(self, position_rad):
        """Send position command to motor"""
        msg = Float64()
        msg.data = position_rad
        self.cmd_pub.publish(msg)

    def read_motor_current(self):
        """Read actual motor current from CAN bus"""
        # Example: Read CAN message (motor-specific protocol)
        msg = self.can_bus.recv(timeout=1.0)
        if msg and msg.arbitration_id == 0x201:  # Motor status message
            current_amps = int.from_bytes(msg.data[0:2], 'little') * 0.01
            return current_amps
        return None

@pytest.fixture
def motor_hardware():
    """Setup/teardown for motor hardware"""
    # Enable motor drivers
    import os
    os.system('sudo ip link set can0 type can bitrate 1000000')
    os.system('sudo ip link set up can0')

    yield

    # Disable motors after test
    os.system('sudo ip link set down can0')

def test_motor_position_accuracy(motor_hardware):
    """Test: Motor reaches commanded position within 0.05 rad"""
    rclpy.init()
    test_node = MotorHILTest()

    # Command shoulder to 45 degrees (0.785 rad)
    target_position = 0.785
    test_node.send_position_command(target_position)

    # Wait for motor to reach position
    import time
    time.sleep(2.0)

    for _ in range(50):
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Check actual position
    assert len(test_node.actual_positions) > 0, "No position feedback"
    actual_position = test_node.actual_positions[-1]

    error = abs(actual_position - target_position)
    assert error < 0.05, f"Position error too large: {error} rad"

    test_node.destroy_node()
    rclpy.shutdown()

def test_motor_current_limit(motor_hardware):
    """Test: Motor current never exceeds 10A"""
    rclpy.init()
    test_node = MotorHILTest()

    # Command rapid movements
    positions = [0.0, 1.57, -1.57, 0.0]  # 0°, 90°, -90°, 0°
    max_current = 0.0

    for pos in positions:
        test_node.send_position_command(pos)

        import time
        for _ in range(20):
            current = test_node.read_motor_current()
            if current:
                max_current = max(max_current, current)
            time.sleep(0.05)

    assert max_current < 10.0, f"Motor current exceeded limit: {max_current}A"

    test_node.destroy_node()
    rclpy.shutdown()
```

**Safety Considerations for HIL Testing**:
- Always test in a **safety enclosure** (cage, barriers)
- Implement **emergency stop** (physical button, software watchdog)
- Start with **low speeds and forces**, gradually increase
- Monitor **temperature, current, voltage** in real-time
- Use **soft limits** before mechanical stops

## CI/CD for Robotics with GitHub Actions

Continuous integration ensures every code commit is tested automatically. For robotics, this includes unit tests, integration tests, and simulation tests.

### Complete CI/CD Pipeline

**.github/workflows/ros2_ci.yml**:
```yaml
name: ROS 2 Humble CI

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  test:
    runs-on: ubuntu-22.04
    container:
      image: osrf/ros:humble-desktop-full

    steps:
      - name: Checkout repository
        uses: actions/checkout@v3

      - name: Install dependencies
        run: |
          apt-get update
          apt-get install -y python3-pip python3-pytest python3-pytest-cov
          apt-get install -y ros-humble-gazebo-ros-pkgs
          pip3 install pytest-mock

      - name: Build ROS 2 workspace
        shell: bash
        run: |
          source /opt/ros/humble/setup.bash
          mkdir -p /ros2_ws/src
          cp -r . /ros2_ws/src/my_robot_pkg
          cd /ros2_ws
          colcon build --symlink-install
          source install/setup.bash

      - name: Run unit tests
        shell: bash
        run: |
          source /opt/ros/humble/setup.bash
          source /ros2_ws/install/setup.bash
          cd /ros2_ws/src/my_robot_pkg
          pytest tests/ -v --cov=my_robot_pkg --cov-report=xml

      - name: Run integration tests
        shell: bash
        run: |
          source /opt/ros/humble/setup.bash
          source /ros2_ws/install/setup.bash
          cd /ros2_ws
          colcon test --packages-select my_robot_pkg
          colcon test-result --verbose

      - name: Run Gazebo simulation tests
        shell: bash
        run: |
          source /opt/ros/humble/setup.bash
          source /ros2_ws/install/setup.bash
          # Start Xvfb for headless Gazebo
          export DISPLAY=:99
          Xvfb :99 -screen 0 1024x768x24 &
          sleep 3
          pytest /ros2_ws/src/my_robot_pkg/tests/simulation/ -v

      - name: Upload coverage to Codecov
        uses: codecov/codecov-action@v3
        with:
          files: ./coverage.xml
          flags: unittests
          name: ros2-coverage

      - name: Static analysis (ruff)
        run: |
          pip3 install ruff
          ruff check /ros2_ws/src/my_robot_pkg

      - name: Check ROS 2 coding standards
        run: |
          pip3 install ament_lint
          source /opt/ros/humble/setup.bash
          cd /ros2_ws
          ament_cpplint src/my_robot_pkg
          ament_flake8 src/my_robot_pkg
```

### Docker-Based CI for Reproducibility

**Dockerfile**:
```dockerfile
FROM osrf/ros:humble-desktop-full

# Install testing tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-launch-testing \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install pytest-mock ruff

# Create workspace
WORKDIR /ros2_ws/src
COPY . /ros2_ws/src/my_robot_pkg

# Build workspace
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Set entrypoint
COPY docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
```

**docker-entrypoint.sh**:
```bash
#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"
```

**Run Tests in Docker**:
```bash
docker build -t my-robot-tests .
docker run --rm my-robot-tests pytest /ros2_ws/src/my_robot_pkg/tests -v
```

## Code Coverage with pytest-cov

Code coverage measures what percentage of your code is executed during tests. Industry standard for robotics: **>80% coverage**.

**Run Coverage Analysis**:
```bash
pytest tests/ --cov=my_robot_pkg --cov-report=html --cov-report=term
```

**Example Coverage Report**:
```
---------- coverage: platform linux, python 3.10 -----------
Name                              Stmts   Miss  Cover
-----------------------------------------------------
my_robot_pkg/__init__.py              2      0   100%
my_robot_pkg/joint_publisher.py      45      3    93%
my_robot_pkg/obstacle_detector.py    38      5    87%
my_robot_pkg/path_planner.py         92     18    80%
-----------------------------------------------------
TOTAL                               177     26    85%
```

**Coverage Configuration** (`.coveragerc`):
```ini
[run]
omit =
    */tests/*
    */setup.py
    */launch/*

[report]
exclude_lines =
    pragma: no cover
    def __repr__
    raise NotImplementedError
    if __name__ == .__main__.:
```

## Performance Regression Testing

For real-time robotics, performance is critical. Test that code changes don't degrade latency or throughput.

**Example: Path Planning Latency Test**:
```python
#!/usr/bin/env python3
import pytest
import time
from path_planner import PathPlanner

def test_path_planning_latency():
    """Test: Path planning completes within 100ms"""
    planner = PathPlanner()

    start = (0.0, 0.0)
    goal = (10.0, 10.0)
    obstacles = [(5.0, 5.0, 1.0)]  # (x, y, radius)

    # Measure planning time
    start_time = time.perf_counter()
    path = planner.plan(start, goal, obstacles)
    end_time = time.perf_counter()

    latency_ms = (end_time - start_time) * 1000

    assert path is not None, "Planning failed"
    assert latency_ms < 100.0, f"Planning too slow: {latency_ms:.2f}ms"

    print(f"✓ Path planning latency: {latency_ms:.2f}ms")

@pytest.mark.benchmark
def test_vslam_throughput():
    """Test: VSLAM processes at least 10 FPS"""
    from vslam_node import VSLAMNode
    import rclpy

    rclpy.init()
    vslam = VSLAMNode()

    # Simulate 100 camera frames
    num_frames = 100
    start_time = time.perf_counter()

    for i in range(num_frames):
        # Create mock camera frame
        frame = create_mock_frame(width=640, height=480)
        vslam.process_frame(frame)

    end_time = time.perf_counter()

    fps = num_frames / (end_time - start_time)
    assert fps >= 10.0, f"VSLAM too slow: {fps:.1f} FPS"

    print(f"✓ VSLAM throughput: {fps:.1f} FPS")

    vslam.destroy_node()
    rclpy.shutdown()
```

**Track Performance Over Time** (GitHub Actions):
```yaml
- name: Run performance benchmarks
  run: |
    pytest tests/benchmarks/ --benchmark-json=benchmark.json

- name: Store benchmark result
  uses: benchmark-action/github-action-benchmark@v1
  with:
    tool: 'pytest'
    output-file-path: benchmark.json
    github-token: ${{ secrets.GITHUB_TOKEN }}
    auto-push: true
```

## Safety Validation Testing

For humanoid robots, safety is paramount. Validate workspace limits, collision avoidance, and emergency stops.

### Example: Collision Detection Test

```python
#!/usr/bin/env python3
"""
Test collision detection and avoidance
Validates that robot stops before hitting obstacles
"""
import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def test_emergency_stop_on_collision():
    """Test: Robot stops when obstacle detected within 0.3m"""
    rclpy.init()

    # Mock robot controller
    class RobotController(Node):
        def __init__(self):
            super().__init__('robot_controller')
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.emergency_stop = False

            self.scan_sub = self.create_subscription(
                LaserScan, '/scan',
                self.scan_callback, 10
            )

        def scan_callback(self, msg):
            min_distance = min(msg.ranges)
            if min_distance < 0.3:
                self.emergency_stop = True
                # Publish zero velocity
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)

        def move_forward(self):
            if not self.emergency_stop:
                cmd = Twist()
                cmd.linear.x = 0.5  # 0.5 m/s
                self.cmd_pub.publish(cmd)

    controller = RobotController()

    # Simulate obstacle at 0.2m
    mock_scan = LaserScan()
    mock_scan.ranges = [0.2] * 360

    # Publish scan
    scan_pub = Node('mock_lidar')
    scan_publisher = scan_pub.create_publisher(LaserScan, '/scan', 10)
    scan_publisher.publish(mock_scan)

    # Process
    for _ in range(10):
        rclpy.spin_once(controller, timeout_sec=0.1)
        rclpy.spin_once(scan_pub, timeout_sec=0.1)

    assert controller.emergency_stop is True, "Emergency stop not triggered"

    controller.destroy_node()
    scan_pub.destroy_node()
    rclpy.shutdown()
```

## Summary: Best Practices for Robotics Testing

**1. Test Pyramid**:
- 70% unit tests (fast, isolated)
- 20% integration tests (multi-node)
- 8% simulation tests (Gazebo)
- 2% HIL tests (real hardware)

**2. Testing Tools**:
- **pytest**: Unit and integration tests
- **launch_testing**: Multi-node integration
- **Gazebo**: Physics-based simulation
- **pytest-cov**: Code coverage analysis
- **GitHub Actions**: CI/CD automation

**3. What to Test**:
- **Functional**: Does it do what it's supposed to?
- **Performance**: Is it fast enough? (latency, throughput)
- **Safety**: Does it stop/avoid collisions?
- **Robustness**: Does it handle sensor noise, edge cases?

**4. CI/CD Pipeline**:
```
Code Commit → Unit Tests → Integration Tests → Simulation Tests → HIL Tests → Deploy
    ↓             ↓              ↓                   ↓                 ↓          ↓
  <1 min        <5 min         <15 min            <1 hour          <1 day   Manual
```

**5. Coverage Goals**:
- Critical safety code: **100% coverage**
- Core algorithms (planning, control): **>90% coverage**
- Utilities, helpers: **>80% coverage**
- Overall project: **>85% coverage**

**Additional Resources**:
- [ROS 2 Testing Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
- [pytest Documentation](https://docs.pytest.org/)
- [launch_testing Examples](https://github.com/ros2/launch/tree/humble/launch_testing)
- [Gazebo Testing Best Practices](https://gazebosim.org/api/gazebo/6/test_fixture.html)
- [GitHub Actions for ROS 2](https://github.com/ros-tooling/action-ros-ci)

**Next Chapter**: [Course Review & Next Steps](./next-steps.md)
