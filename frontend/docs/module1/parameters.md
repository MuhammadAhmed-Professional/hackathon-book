---
id: parameters
title: "ROS 2 Parameter Management"
sidebar_label: "Parameters"
sidebar_position: 6
description: "Configure robot behavior at runtime with ROS 2 parameters and dynamic reconfiguration."
tags: [ros2, parameters, configuration, dynamic-reconfigure, yaml]
---

# ROS 2 Parameter Management

**In this chapter**, you'll learn to configure robot behavior using ROS 2 parameters, enabling runtime tuning without recompilation.

## Learning Objectives

- ✅ Declare and use parameters in ROS 2 nodes
- ✅ Load parameters from YAML configuration files
- ✅ Implement parameter callbacks for dynamic reconfiguration
- ✅ Use parameter validation and constraints
- ✅ Organize multi-robot configurations with namespaces

**Prerequisites**: Python integration (Chapter 4), basic YAML syntax
**Estimated Time**: 40 minutes

---

## Why Parameters?

Parameters allow runtime configuration without code changes:

❌ **Without Parameters** (Hardcoded):
```python
CAMERA_FPS = 30  # Must recompile to change
MAX_VELOCITY = 1.5
```

✅ **With Parameters** (Configurable):
```python
self.camera_fps = self.declare_parameter('camera_fps', 30).value
self.max_velocity = self.declare_parameter('max_velocity', 1.5).value
```

**Benefits**:
- Test different values without recompiling
- Share configurations across robots (YAML files)
- Tune PID controllers in real-time
- Enable/disable features dynamically

:::info Real-World Use
Boston Dynamics Spot uses parameters for gait tuning, allowing engineers to adjust walking speed, step height, and balance parameters on the fly.
:::

---

## Parameter Types

ROS 2 supports these parameter types:

| Type | Python | Example |
|------|--------|---------|
| bool | `bool` | `enable_safety_limits: true` |
| integer | `int` | `control_frequency: 100` |
| double | `float` | `max_velocity: 2.5` |
| string | `str` | `robot_name: "optimus"` |
| byte_array | `bytes` | (rarely used) |
| bool_array | `list[bool]` | `[true, false, true]` |
| integer_array | `list[int]` | `joint_ids: [1, 2, 3]` |
| double_array | `list[float]` | `pid_gains: [1.0, 0.1, 0.01]` |
| string_array | `list[str]` | `sensor_names: ["imu", "camera"]` |

---

## Hands-On: Parameter-Driven Robot Controller

Let's create a velocity controller with tunable parameters.

### Step 1: Declare Parameters

```python title="velocity_controller/controller_node.py"
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        # Declare parameters with defaults and constraints
        self.declare_parameter('max_linear_velocity', 2.0,
            ParameterDescriptor(
                description='Maximum linear velocity (m/s)',
                floating_point_range=[FloatingPointRange(
                    from_value=0.1,
                    to_value=5.0,
                    step=0.1
                )]
            ))

        self.declare_parameter('max_angular_velocity', 1.5,
            ParameterDescriptor(
                description='Maximum angular velocity (rad/s)',
                floating_point_range=[FloatingPointRange(
                    from_value=0.1,
                    to_value=3.0,
                    step=0.1
                )]
            ))

        self.declare_parameter('control_frequency', 50,
            ParameterDescriptor(
                description='Control loop frequency (Hz)',
                integer_range=[IntegerRange(
                    from_value=10,
                    to_value=200,
                    step=1
                )]
            ))

        self.declare_parameter('enable_safety_limits', True,
            ParameterDescriptor(
                description='Enable velocity safety limits'
            ))

        # Get parameter values
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.safety_enabled = self.get_parameter('enable_safety_limits').value

        # Add parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer at control frequency
        period = 1.0 / self.control_freq
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(f'Controller started:')
        self.get_logger().info(f'  Max linear velocity: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  Max angular velocity: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'  Control frequency: {self.control_freq} Hz')

    def parameter_callback(self, params):
        """Handle parameter changes at runtime"""
        for param in params:
            if param.name == 'max_linear_velocity':
                old_value = self.max_linear_vel
                self.max_linear_vel = param.value
                self.get_logger().info(
                    f'Updated max_linear_velocity: {old_value} → {param.value} m/s'
                )

            elif param.name == 'max_angular_velocity':
                old_value = self.max_angular_vel
                self.max_angular_vel = param.value
                self.get_logger().info(
                    f'Updated max_angular_velocity: {old_value} → {param.value} rad/s'
                )

            elif param.name == 'control_frequency':
                old_freq = self.control_freq
                self.control_freq = param.value
                # Update timer
                self.timer.cancel()
                period = 1.0 / self.control_freq
                self.timer = self.create_timer(period, self.control_loop)
                self.get_logger().info(
                    f'Updated control_frequency: {old_freq} → {param.value} Hz'
                )

            elif param.name == 'enable_safety_limits':
                self.safety_enabled = param.value
                self.get_logger().info(
                    f'Safety limits: {"enabled" if param.value else "disabled"}'
                )

        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def control_loop(self):
        """Publish velocity commands"""
        cmd = Twist()

        # Simple forward motion (replace with actual control logic)
        cmd.linear.x = 1.0
        cmd.angular.z = 0.5

        # Apply safety limits
        if self.safety_enabled:
            cmd.linear.x = min(cmd.linear.x, self.max_linear_vel)
            cmd.angular.z = min(abs(cmd.angular.z), self.max_angular_vel) * \
                           (1 if cmd.angular.z >= 0 else -1)

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()

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

### Step 2: Load Parameters from YAML

Create a configuration file:

```yaml title="config/velocity_controller_params.yaml"
/velocity_controller:
  ros__parameters:
    max_linear_velocity: 3.0
    max_angular_velocity: 2.0
    control_frequency: 100
    enable_safety_limits: true
```

**Launch with parameters**:
```bash
ros2 run velocity_controller controller_node \
  --ros-args --params-file config/velocity_controller_params.yaml
```

### Step 3: Change Parameters at Runtime

```bash
# List all parameters
ros2 param list /velocity_controller

# Get current value
ros2 param get /velocity_controller max_linear_velocity

# Set new value (dynamic update)
ros2 param set /velocity_controller max_linear_velocity 2.5

# Dump all parameters to YAML
ros2 param dump /velocity_controller > params_backup.yaml
```

**Expected Output**:
```
Setting parameter successful
[INFO] Updated max_linear_velocity: 3.0 → 2.5 m/s
```

---

## Parameter Validation

Prevent invalid parameter values:

```python
def parameter_callback(self, params):
    for param in params:
        if param.name == 'max_linear_velocity':
            if param.value < 0 or param.value > 5.0:
                self.get_logger().error(
                    f'Invalid max_linear_velocity: {param.value} (must be 0-5 m/s)'
                )
                return SetParametersResult(successful=False)

            self.max_linear_vel = param.value

    return SetParametersResult(successful=True)
```

**Test invalid value**:
```bash
ros2 param set /velocity_controller max_linear_velocity 10.0
# Output: Set parameter failed
```

---

## Multi-Robot Configuration with Namespaces

Use namespaces to configure multiple robots independently:

```yaml title="config/dual_robot.yaml"
/robot1/velocity_controller:
  ros__parameters:
    max_linear_velocity: 2.0
    robot_name: "robot1"

/robot2/velocity_controller:
  ros__parameters:
    max_linear_velocity: 3.5
    robot_name: "robot2"
```

**Launch with namespace**:
```bash
# Robot 1
ros2 run velocity_controller controller_node \
  --ros-args -r __ns:=/robot1 --params-file config/dual_robot.yaml

# Robot 2
ros2 run velocity_controller controller_node \
  --ros-args -r __ns:=/robot2 --params-file config/dual_robot.yaml
```

**Nodes appear as**:
- `/robot1/velocity_controller`
- `/robot2/velocity_controller`

Each has independent parameters!

---

## Parameter Organization Best Practices

### 1. Group Related Parameters

```python
# ❌ Flat structure
self.declare_parameter('camera_width', 640)
self.declare_parameter('camera_height', 480)
self.declare_parameter('camera_fps', 30)

# ✅ Grouped (in YAML)
```

```yaml
/camera_node:
  ros__parameters:
    camera:
      resolution:
        width: 640
        height: 480
      fps: 30
```

### 2. Use Descriptive Names

```python
# ❌ Unclear
self.declare_parameter('threshold', 0.5)

# ✅ Clear
self.declare_parameter('object_detection_confidence_threshold', 0.5)
```

### 3. Provide Descriptions

```python
self.declare_parameter('pid_kp', 1.0,
    ParameterDescriptor(
        description='Proportional gain for PID controller',
        floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0)]
    ))
```

---

## Read-Only vs Dynamic Parameters

Some parameters should only be set at startup:

```python
# Read-only parameter (no callback)
self.declare_parameter('robot_model', 'optimus_v2',
    ParameterDescriptor(
        description='Robot model (cannot change at runtime)',
        read_only=True
    ))

self.robot_model = self.get_parameter('robot_model').value
```

**Attempting to change read-only parameter**:
```bash
ros2 param set /node robot_model "optimus_v3"
# Output: Parameter 'robot_model' is read-only
```

---

## Exercises

### Easy
1. Add a `robot_name` parameter (string) to the velocity controller
2. Create a YAML file with 5 different parameter sets
3. Use `ros2 param dump` to save current parameters

### Medium
4. Implement PID controller parameters (Kp, Ki, Kd) with validation
5. Create a parameter for joint limits (array of 6 floats)
6. Add parameter callback that logs changes to a file

### Hard
7. Build a parameter manager node that monitors and logs all parameter changes
8. Implement parameter profiles: "sport mode" vs "eco mode" configurations
9. Create dynamic parameter bounds that change based on robot state

---

## Summary

You've mastered:

- ✅ Declaring parameters with types, defaults, and constraints
- ✅ Loading configurations from YAML files
- ✅ Implementing dynamic parameter updates with callbacks
- ✅ Validating parameter values at runtime
- ✅ Managing multi-robot configurations with namespaces

**Next Module**: [Gazebo Simulation](../module2/gazebo-simulation) - Test robots in virtual environments

## References

- [ROS 2 Parameters Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [Parameter API](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html)
- [YAML Configuration Guide](https://docs.ros.org/en/humble/How-To-Guides/Using-ros2-param-YAML-files.html)
