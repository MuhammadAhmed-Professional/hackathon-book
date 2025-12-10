---
id: actions-and-services
title: "ROS 2 Actions and Advanced Services"
sidebar_label: "Actions & Services"
sidebar_position: 5
description: "Master ROS 2 actions for long-running tasks and advanced service patterns for humanoid robot control."
tags: [ros2, actions, services, feedback, goal-based-control]
---

# ROS 2 Actions and Advanced Services

**In this chapter**, you'll master ROS 2 actions for long-running tasks like navigation and manipulation, plus advanced service patterns for synchronous robot control.

## Learning Objectives

- ✅ Understand when to use actions vs topics vs services
- ✅ Implement action servers for goal-based tasks
- ✅ Create action clients with feedback handling
- ✅ Design service patterns for discrete robot commands
- ✅ Handle cancellation and preemption in action workflows

**Prerequisites**: Python integration (Chapter 4), understanding of publishers/subscribers
**Estimated Time**: 50 minutes

---

## Actions vs Topics vs Services: When to Use Each

| Pattern | Use Case | Example | Bidirectional? | Feedback? |
|---------|----------|---------|----------------|-----------|
| **Topic** | Continuous streams | Camera images, sensor data | No | No |
| **Service** | Discrete requests | Get current pose, trigger action | Yes | No |
| **Action** | Long-running goals | Navigate to waypoint, grasp object | Yes | Yes |

:::info Real-World Example
Tesla Optimus uses:
- **Topics**: IMU data (100 Hz), joint states (50 Hz)
- **Services**: "Get battery level", "Emergency stop"
- **Actions**: "Walk to kitchen", "Pick up cup" (with progress feedback)
:::

**Action Workflow**:
```
Client                    Server
  │                         │
  │──Goal──────────────────>│ 1. Send goal
  │                         │
  │<──────────────Feedback──│ 2. Periodic progress updates
  │<──────────────Feedback──│
  │                         │
  │<──────────────Result────│ 3. Final result (success/failure)
```

---

## Action Definition: Navigation Example

ROS 2 actions are defined in `.action` files with three parts:

```action title="NavigateToPoint.action"
# Goal: Target position
geometry_msgs/Point target
---
# Result: Final outcome
bool success
float32 distance_traveled
string error_message
---
# Feedback: Periodic updates
geometry_msgs/Point current_position
float32 distance_remaining
float32 estimated_time_remaining
```

**Compilation**: Action definitions compile to 3 message types:
- `NavigateToPoint_Goal`
- `NavigateToPoint_Result`
- `NavigateToPoint_Feedback`

---

## Hands-On: Action Server for Arm Movement

Let's create an action server that moves a humanoid arm to a target joint configuration with progress feedback.

### Step 1: Define the Action

```action title="arm_control_interfaces/action/MoveArm.action"
# Goal: Target joint angles (radians)
float64[] target_joint_angles
---
# Result
bool success
string message
float64 final_error  # RMS error between target and actual
---
# Feedback: Progress updates
float64[] current_joint_angles
float64 progress_percentage
float64 time_elapsed
```

### Step 2: Implement Action Server

```python title="arm_controller/arm_action_server.py"
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from arm_control_interfaces.action import MoveArm
import numpy as np
import time

class ArmActionServer(Node):
    def __init__(self):
        super().__init__('arm_action_server')

        self._action_server = ActionServer(
            self,
            MoveArm,
            'move_arm',
            self.execute_callback
        )

        # Simulated current arm state
        self.current_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 6-DOF arm

        self.get_logger().info('Arm action server started')

    def execute_callback(self, goal_handle):
        """Execute the arm movement goal"""
        self.get_logger().info('Executing goal...')

        target = np.array(goal_handle.request.target_joint_angles)

        # Validate target
        if len(target) != 6:
            goal_handle.abort()
            result = MoveArm.Result()
            result.success = False
            result.message = f'Invalid target: expected 6 angles, got {len(target)}'
            return result

        # Check joint limits
        if np.any(np.abs(target) > np.pi):
            goal_handle.abort()
            result = MoveArm.Result()
            result.success = False
            result.message = 'Target exceeds joint limits (±π rad)'
            return result

        # Movement simulation (in production, send to robot controller)
        feedback_msg = MoveArm.Feedback()
        total_distance = np.linalg.norm(target - self.current_angles)
        start_time = time.time()

        # Simulate gradual movement
        for step in range(100):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = MoveArm.Result()
                result.success = False
                result.message = 'Goal canceled by client'
                return result

            # Interpolate to target
            alpha = (step + 1) / 100.0
            self.current_angles = (1 - alpha) * self.current_angles + alpha * target

            # Publish feedback
            feedback_msg.current_joint_angles = self.current_angles.tolist()
            feedback_msg.progress_percentage = alpha * 100.0
            feedback_msg.time_elapsed = time.time() - start_time
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.05)  # 20 Hz update rate

        # Goal succeeded
        goal_handle.succeed()

        result = MoveArm.Result()
        result.success = True
        result.message = 'Arm reached target position'
        result.final_error = np.linalg.norm(target - self.current_angles)

        self.get_logger().info(f'Goal succeeded: error={result.final_error:.4f} rad')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ArmActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Action Client with Feedback

```python title="arm_controller/arm_action_client.py"
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from arm_control_interfaces.action import MoveArm
import numpy as np

class ArmActionClient(Node):
    def __init__(self):
        super().__init__('arm_action_client')

        self._action_client = ActionClient(self, MoveArm, 'move_arm')

    def send_goal(self, target_angles):
        """Send goal to action server"""
        goal_msg = MoveArm.Goal()
        goal_msg.target_joint_angles = target_angles

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Process feedback during execution"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Progress: {feedback.progress_percentage:.1f}% | '
            f'Time: {feedback.time_elapsed:.2f}s'
        )

    def get_result_callback(self, future):
        """Handle final result"""
        result = future.result().result

        if result.success:
            self.get_logger().info(
                f'Goal succeeded! Final error: {result.final_error:.4f} rad'
            )
        else:
            self.get_logger().error(f'Goal failed: {result.message}')

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = ArmActionClient()

    # Target: 45° on first 3 joints
    target = [np.pi/4, np.pi/4, np.pi/4, 0.0, 0.0, 0.0]
    client.send_goal(target)

    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

### Step 4: Test the Action

```bash
# Terminal 1: Start server
python3 arm_action_server.py

# Terminal 2: Send goal
python3 arm_action_client.py

# Terminal 3: Monitor action
ros2 action list
ros2 action info /move_arm
```

**Expected Output**:
```
[INFO] Waiting for action server...
[INFO] Sending goal...
[INFO] Goal accepted
[INFO] Progress: 10.0% | Time: 0.50s
[INFO] Progress: 20.0% | Time: 1.00s
...
[INFO] Progress: 100.0% | Time: 5.00s
[INFO] Goal succeeded! Final error: 0.0001 rad
```

---

## Advanced Service Patterns

Services are best for discrete, synchronous operations that complete quickly (&lt;1 second).

### Example: Robot Pose Service

```python title="pose_service/get_pose_server.py"
from geometry_msgs.srv import GetPose
from geometry_msgs.msg import Pose
from rclpy.node import Node
import rclpy

class PoseServer(Node):
    def __init__(self):
        super().__init__('pose_server')

        self.srv = self.create_service(
            GetPose,
            'get_robot_pose',
            self.get_pose_callback
        )

        # Simulated robot state
        self.pose = Pose()
        self.pose.position.x = 1.0
        self.pose.position.y = 2.0
        self.pose.position.z = 0.5

    def get_pose_callback(self, request, response):
        """Return current robot pose"""
        response.pose = self.pose
        self.get_logger().info('Pose request served')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PoseServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

:::warning Service Limitations
Services block the client until completion. For tasks >1 second, use actions instead.
:::

---

## Cancellation and Preemption

Action clients can cancel goals mid-execution:

```python
# In client code
goal_handle = await self._action_client.send_goal_async(goal_msg)

# Cancel after 2 seconds
await asyncio.sleep(2.0)
cancel_future = goal_handle.cancel_goal_async()
cancel_response = await cancel_future

if cancel_response.goals_canceling:
    self.get_logger().info('Goal successfully canceled')
```

**Server handling**:
```python
if goal_handle.is_cancel_requested:
    goal_handle.canceled()
    result = MoveArm.Result()
    result.success = False
    result.message = 'Canceled by user'
    return result
```

---

## Exercises

### Easy
1. Modify the arm action server to use 7 DOF instead of 6
2. Add a parameter for movement speed (duration)
3. Create a service that returns battery level (float)

### Medium
4. Implement goal preemption: new goals cancel old ones
5. Add velocity limits to arm movement (max 0.5 rad/s per joint)
6. Create a "home position" service that moves all joints to zero

### Hard
7. Build a navigation action with obstacle avoidance feedback
8. Implement action composition: chain multiple arm movements
9. Add trajectory planning: smooth acceleration/deceleration curves

---

## Summary

You've mastered:

- ✅ Choosing between topics, services, and actions for different use cases
- ✅ Implementing action servers with goal validation and feedback
- ✅ Creating action clients with async callbacks
- ✅ Handling cancellation and errors gracefully
- ✅ Designing service patterns for synchronous operations

**Next Chapter**: [Parameter Management](./parameters) - Configure robots at runtime

## References

- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [action_msgs Package](https://docs.ros.org/en/humble/p/action_msgs/)
- [Action Design Patterns](https://design.ros2.org/articles/actions.html)
