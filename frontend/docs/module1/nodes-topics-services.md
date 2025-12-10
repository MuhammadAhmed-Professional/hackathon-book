---
id: nodes-topics-services
title: "Nodes, Topics, Services, and Actions"
sidebar_label: "Nodes & Communication"
sidebar_position: 3
description: "Master ROS 2 communication patterns: nodes, topics (pub/sub), services (request/response), and actions (long-running tasks) with practical Python examples for humanoid robotics."
keywords: [ros2, nodes, topics, services, actions, publisher, subscriber, rclpy]
---

# Nodes, Topics, Services, and Actions

## Introduction

ROS 2's power comes from its distributed architecture where independent **nodes** communicate via three primary patterns: **topics** (publish/subscribe), **services** (request/response), and **actions** (long-running tasks with feedback). Understanding when and how to use each pattern is fundamental to building robust humanoid robot systems.

**Learning Objectives**:
- Create ROS 2 nodes that perform specific computational tasks
- Implement publish/subscribe communication using topics
- Use services for synchronous request/response operations
- Deploy actions for long-running tasks with progress feedback
- Choose the right communication pattern for different scenarios

## ROS 2 Nodes: The Building Blocks

A **node** is an independent process that performs a specific computation. In a humanoid robot system, you might have:

- **Perception Node**: Processes camera images for object detection
- **Localization Node**: Estimates robot position using VSLAM
- **Planning Node**: Computes navigation paths
- **Control Node**: Sends motor commands to actuators

### Why Multiple Nodes?

**Modularity**: Each node has a single responsibility, making code easier to test and debug.

**Parallel Execution**: Nodes run as separate processes, leveraging multi-core CPUs. A humanoid robot's perception and control can execute simultaneously.

**Fault Isolation**: If one node crashes (e.g., perception), others (e.g., motor control) continue running.

**Language Flexibility**: Perception in Python (for AI libraries), control in C++ (for real-time performance).

### Creating a Basic Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class HumanoidPerception(Node):
    def __init__(self):
        # Node name: 'humanoid_perception'
        super().__init__('humanoid_perception')
        self.get_logger().info('Humanoid perception node started')

def main():
    rclpy.init()
    node = HumanoidPerception()
    rclpy.spin(node)  # Keep node alive
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: Publish/Subscribe Communication

**Topics** implement the **publish/subscribe pattern** for continuous data streams. Publishers send messages to a topic; subscribers receive them. Multiple publishers and subscribers can connect to the same topic.

### When to Use Topics

- **Sensor data**: Camera images, IMU readings, joint positions (continuous streams)
- **Motor commands**: Velocity commands, joint trajectories (continuous control)
- **Status updates**: Battery level, robot state (periodic broadcasts)

### Topic Example: Publishing Joint States

For a humanoid robot, joint states must be published continuously for visualization and control:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher: topic='/joint_states', message type=JointState, queue=10
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Publish at 100 Hz (typical for joint states)
        self.timer = self.create_timer(0.01, self.publish_joint_states)

        # Humanoid joint names (simplified)
        self.joint_names = [
            'neck_yaw', 'neck_pitch',
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_elbow',
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_elbow',
            'left_hip_pitch', 'left_knee', 'left_ankle',
            'right_hip_pitch', 'right_knee', 'right_ankle'
        ]

        self.positions = [0.0] * len(self.joint_names)
        self.get_logger().info(f'Publishing {len(self.joint_names)} joint states')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions

        # In real robot, read from hardware here
        # self.positions = read_joint_encoders()

        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topic Example: Subscribing to Camera Data

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10  # QoS queue size
        )

        self.bridge = CvBridge()
        self.get_logger().info('Subscribed to /camera/image_raw')

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process image (e.g., object detection)
        # For demo, just show resolution
        height, width = cv_image.shape[:2]
        self.get_logger().info(f'Received image: {width}x{height}')

        # In real application: run YOLO, segment objects, etc.

def main():
    rclpy.init()
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Topic Concepts**:
- **Decoupling**: Publisher doesn't know who subscribes; subscriber doesn't know who publishes
- **Many-to-Many**: Multiple publishers and subscribers can use the same topic
- **QoS**: Configure reliability (RELIABLE vs. BEST_EFFORT) based on data importance

## Services: Request/Response Communication

**Services** implement synchronous **request/response** communication for discrete, on-demand operations. A service client sends a request and blocks until receiving a response.

### When to Use Services

- **Configuration**: Set parameters, calibrate sensors
- **Query State**: Get current robot pose, battery level
- **Discrete Actions**: Take a photo, save map, reset odometry

Services are **not** suitable for continuous control (use topics) or long-running tasks (use actions).

### Service Example: Robot Status Query

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # Simple service: no input, string output

class StatusService(Node):
    def __init__(self):
        super().__init__('status_service')

        # Create service server
        self.service = self.create_service(
            Trigger,
            '/get_robot_status',
            self.get_status_callback
        )

        self.get_logger().info('Robot status service ready')

    def get_status_callback(self, request, response):
        # Service logic: check robot health
        battery_level = 87  # In real robot: read from hardware
        balance_stable = True

        if battery_level < 20:
            response.success = False
            response.message = f'Low battery: {battery_level}%'
        elif not balance_stable:
            response.success = False
            response.message = 'Balance unstable'
        else:
            response.success = True
            response.message = f'Robot OK, battery: {battery_level}%'

        return response

def main():
    rclpy.init()
    node = StatusService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class StatusClient(Node):
    def __init__(self):
        super().__init__('status_client')

        # Create service client
        self.client = self.create_client(Trigger, '/get_robot_status')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for status service...')

        self.get_logger().info('Status service found')

    def check_status(self):
        # Create request (empty for Trigger service)
        request = Trigger.Request()

        # Call service (blocking)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Get response
        response = future.result()
        if response.success:
            self.get_logger().info(f'Robot status: {response.message}')
        else:
            self.get_logger().warn(f'Robot issue: {response.message}')

def main():
    rclpy.init()
    node = StatusClient()
    node.check_status()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Service Concepts**:
- **Synchronous**: Client blocks waiting for response
- **One-to-One**: Each request has exactly one response
- **Not Continuous**: Services are for discrete queries, not streaming data

## Actions: Long-Running Tasks with Feedback

**Actions** are designed for **long-running, goal-oriented tasks** that provide:
- **Feedback**: Progress updates during execution
- **Cancellation**: Ability to abort the task mid-execution
- **Result**: Final outcome when task completes

### When to Use Actions

- **Navigation**: "Navigate to kitchen" (takes seconds/minutes, provides progress)
- **Manipulation**: "Pick up object" (multi-step, can fail, provides status)
- **Calibration**: "Calibrate IMU" (takes time, reports progress)

### Action Example: Simple Navigation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci  # Using built-in action for demo

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,  # In real robot: use nav2_msgs.action.NavigateToPose
            'navigate_to_goal',
            self.execute_callback
        )

        self.get_logger().info('Navigation action server ready')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing navigation goal...')

        # Simulate navigation with feedback
        feedback_msg = Fibonacci.Feedback()

        for i in range(10):  # Simulate 10 steps of navigation
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Navigation cancelled')
                return Fibonacci.Result()

            # Publish feedback (distance to goal, etc.)
            feedback_msg.sequence = [i, i+1]  # In real robot: [current_x, current_y]
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Navigation progress: {i*10}%')

            # Simulate time to reach goal
            import time
            time.sleep(0.5)

        # Goal reached - send result
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = [10, 10]  # Final position
        self.get_logger().info('Navigation succeeded!')

        return result

def main():
    rclpy.init()
    node = NavigationActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Action Concepts**:
- **Asynchronous**: Client doesn't block while goal executes
- **Feedback**: Server sends progress updates
- **Cancellable**: Client can abort goal mid-execution
- **Stateful**: Actions have states (ACCEPTED, EXECUTING, SUCCEEDED, CANCELED, ABORTED)

## Choosing the Right Communication Pattern

| **Pattern** | **Use Case** | **Example** |
|-------------|--------------|-------------|
| **Topic** | Continuous data streams | Camera images, IMU readings, joint states |
| **Service** | Discrete queries or commands | Get robot pose, calibrate sensor, save map |
| **Action** | Long-running tasks with feedback | Navigate to goal, grasp object, charge battery |

**Rule of Thumb**:
- Data published continuously at high rate? → **Topic**
- Quick query with immediate response? → **Service**
- Task takes >1 second and needs progress updates? → **Action**

## Summary

ROS 2 nodes communicate via three patterns, each optimized for different scenarios:

- **Nodes**: Independent processes with single responsibilities
- **Topics**: Publish/subscribe for continuous data streams (sensors, commands)
- **Services**: Request/response for discrete operations (configuration, queries)
- **Actions**: Long-running tasks with feedback and cancellation (navigation, manipulation)

For humanoid robots, a typical architecture uses:
- Topics for sensor data (cameras, IMU) and motor commands
- Services for configuration (calibrate sensors, set parameters)
- Actions for high-level tasks (navigate, grasp, balance)

**Next Chapter**: [Python Integration with rclpy](./python-integration.md) - Build complete ROS 2 packages

**Additional Resources**:
- [ROS 2 Topics Tutorial](https://docs.ros.org/en/humble/Tutorials/Topics/Understanding-ROS2-Topics.html)
- [ROS 2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Services/Understanding-ROS2-Services.html)
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Actions/Understanding-ROS2-Actions.html)
