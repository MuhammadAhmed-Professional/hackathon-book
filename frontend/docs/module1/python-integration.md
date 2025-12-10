---
id: python-integration
title: "Python Integration with ROS 2"
sidebar_position: 4
description: "Master rclpy for building Python-based ROS 2 nodes with publishers, subscribers, and timers"
---

# Python Integration with ROS 2

Python has become the lingua franca of robotics and AI development. Its readability, extensive ecosystem (NumPy, OpenCV, PyTorch), and rapid prototyping capabilities make it ideal for robotics research and development. ROS 2's **rclpy** library provides a Pythonic interface to the ROS 2 middleware, enabling you to build sophisticated robotic systems without sacrificing the performance benefits of DDS.

In this chapter, you'll learn to harness rclpy's power for creating publishers, subscribers, timers, and complex node architectures that integrate seamlessly with the ROS 2 ecosystem.

## Why Python for ROS 2?

While C++ offers maximum performance for time-critical control loops, Python excels in several robotics domains:

- **Perception pipelines**: OpenCV, scikit-image, and PIL integration for computer vision
- **AI/ML inference**: PyTorch, TensorFlow, and ONNX Runtime for neural network deployment
- **Rapid prototyping**: Test algorithms quickly before C++ optimization
- **Data science**: Pandas, Matplotlib, and Jupyter for experiment analysis
- **High-level coordination**: State machines, mission planning, and behavior trees

**Performance consideration**: rclpy uses Python's C extension API for message serialization, minimizing overhead. For hard real-time control (e.g., 1 kHz motor control), use C++ nodes; for perception and planning (10-100 Hz), Python is perfectly adequate.

## The rclpy Node Lifecycle

Every ROS 2 Python node follows this initialization pattern:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')  # Node name for ROS 2 graph
        self.get_logger().info('Node initialized')

        # Create publishers, subscribers, timers here
        self.setup_communication()

    def setup_communication(self):
        # Setup method for clarity
        pass

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 Python client library
    node = MyNode()

    try:
        rclpy.spin(node)  # Process callbacks indefinitely
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key components**:
- `rclpy.init()`: Must be called before creating nodes (initializes DDS middleware)
- `Node` inheritance: Provides logging, parameter server, and communication factory methods
- `rclpy.spin()`: Processes callbacks in an event loop (blocks until Ctrl+C)
- Cleanup: Always destroy nodes and shutdown rclpy in `finally` block

## Building a Python Publisher

Let's create a node that publishes IMU (Inertial Measurement Unit) data at 100 Hz:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
import math
import time

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Create publisher: topic name, message type, queue size
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)

        # Timer: 0.01 seconds = 100 Hz
        self.timer = self.create_timer(0.01, self.publish_imu_data)

        self.sequence = 0
        self.get_logger().info('IMU publisher started at 100 Hz')

    def publish_imu_data(self):
        msg = Imu()

        # Header with timestamp and frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulated orientation (quaternion)
        angle = self.sequence * 0.01  # Slow rotation for demo
        msg.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(angle / 2),
            w=math.cos(angle / 2)
        )

        # Simulated angular velocity (rad/s)
        msg.angular_velocity = Vector3(x=0.1, y=0.0, z=0.05)

        # Simulated linear acceleration (m/s²)
        msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)

        self.publisher.publish(msg)
        self.sequence += 1

        if self.sequence % 100 == 0:  # Log every second
            self.get_logger().info(f'Published IMU message {self.sequence}')

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()

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

**Publisher parameters**:
- **Topic name**: `/imu/data` follows ROS 2 naming convention (absolute path)
- **Queue size**: 10 messages buffered if subscriber can't keep up
- **QoS profile**: Default is RELIABLE (all messages delivered) for sensor data

**Timestamp best practices**: Always use `self.get_clock().now()` instead of Python's `time.time()` for ROS 2 time compatibility (supports simulation time in Gazebo).

## Building a Python Subscriber

Now let's create a node that subscribes to camera images and processes them with OpenCV:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        # CvBridge converts between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

        # Subscriber: topic, message type, callback function, queue size
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for processed images
        self.publisher = self.create_publisher(Image, '/camera/image_processed', 10)

        self.get_logger().info('Image processor node started')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Apply edge detection (Canny algorithm)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, threshold1=50, threshold2=150)

            # Convert back to BGR for publishing
            edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

            # Convert OpenCV image back to ROS Image message
            processed_msg = self.bridge.cv2_to_imgmsg(edges_bgr, encoding='bgr8')
            processed_msg.header = msg.header  # Preserve timestamp

            self.publisher.publish(processed_msg)

            self.get_logger().info(
                f'Processed image {msg.header.stamp.sec}.{msg.header.stamp.nanosec}',
                throttle_duration_sec=1.0  # Log max once per second
            )

        except Exception as e:
            self.get_logger().error(f'Image processing failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()

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

**Callback best practices**:
- Keep callbacks fast (< 10 ms for 100 Hz topics)
- Use `throttle_duration_sec` for logging to avoid spam
- Always use try-except for robustness (malformed messages shouldn't crash the node)
- Preserve message headers when republishing (maintains timestamp chain)

## Timers and Periodic Tasks

Timers are essential for control loops and periodic state updates:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # Publisher for battery voltage
        self.voltage_pub = self.create_publisher(Float32, '/battery/voltage', 10)

        # Timer: check battery every 5 seconds
        self.timer = self.create_timer(5.0, self.check_battery)

        self.voltage = 12.6  # Initial voltage (full charge)
        self.discharge_rate = 0.01  # Volts per check

        self.get_logger().info('Battery monitor started')

    def check_battery(self):
        # Simulate battery discharge
        self.voltage -= self.discharge_rate

        # Publish current voltage
        msg = Float32()
        msg.data = self.voltage
        self.voltage_pub.publish(msg)

        # Log warnings at specific thresholds
        if self.voltage < 11.1:
            self.get_logger().error(f'CRITICAL: Battery at {self.voltage:.2f}V!')
        elif self.voltage < 11.5:
            self.get_logger().warn(f'LOW: Battery at {self.voltage:.2f}V')
        else:
            self.get_logger().info(f'Battery: {self.voltage:.2f}V')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()

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

**Timer precision**: ROS 2 timers use the system clock by default. For sub-millisecond precision, use a real-time kernel (PREEMPT_RT patch) and set thread priorities.

## Parameters and Reconfiguration

ROS 2 parameters enable runtime configuration without recompiling:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

class ConfigurablePublisher(Node):
    def __init__(self):
        super().__init__('configurable_publisher')

        # Declare parameters with default values and descriptions
        self.declare_parameter(
            'publish_rate',
            10.0,  # Default: 10 Hz
            ParameterDescriptor(description='Publishing frequency in Hz')
        )

        self.declare_parameter(
            'topic_name',
            '/data',
            ParameterDescriptor(description='Topic to publish on')
        )

        # Read parameters
        rate = self.get_parameter('publish_rate').value
        topic = self.get_parameter('topic_name').value

        self.get_logger().info(f'Publishing to {topic} at {rate} Hz')

        # Use parameters in node setup
        self.publisher = self.create_publisher(Float32, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        msg.data = 42.0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConfigurablePublisher()

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

**Set parameters at launch**:
```bash
ros2 run my_package configurable_publisher --ros-args -p publish_rate:=50.0 -p topic_name:=/custom_topic
```

**Change parameters at runtime**:
```bash
ros2 param set /configurable_publisher publish_rate 100.0
```

## Integration with NumPy and AI Libraries

Python's true power emerges when integrating scientific libraries:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.min_distance = 0.5  # meters

    def scan_callback(self, msg):
        # Convert ranges to NumPy array
        ranges = np.array(msg.ranges)

        # Filter out invalid readings (inf, nan)
        valid_ranges = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]

        if valid_ranges.size == 0:
            return

        # Find closest obstacle
        min_range = np.min(valid_ranges)
        min_index = np.argmin(ranges)

        # Calculate angle of closest obstacle
        angle = msg.angle_min + min_index * msg.angle_increment
        angle_deg = np.degrees(angle)

        if min_range < self.min_distance:
            self.get_logger().warn(
                f'OBSTACLE: {min_range:.2f}m at {angle_deg:.1f}°',
                throttle_duration_sec=0.5
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()

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

**NumPy advantages**:
- Vectorized operations (10-100x faster than Python loops)
- Statistical functions (mean, std, percentile) for sensor fusion
- Linear algebra (transformations, inverse kinematics)

## Practical Exercise

**Challenge**: Create a `VelocityController` node that:
1. Subscribes to `/odom` (nav_msgs/Odometry) to get current robot velocity
2. Publishes `/cmd_vel` (geometry_msgs/Twist) to command velocity
3. Uses a parameter `target_speed` (default 0.5 m/s)
4. Implements a simple proportional controller to reach target speed

**Starter template**:
```python
class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        # TODO: Declare parameter, create subscriber and publisher

    def odom_callback(self, msg):
        current_speed = msg.twist.twist.linear.x
        # TODO: Calculate error, publish cmd_vel
```

## Key Takeaways

✅ **rclpy lifecycle**: init → create node → spin → destroy → shutdown
✅ **Publishers**: `create_publisher(MessageType, topic, qos)`
✅ **Subscribers**: `create_subscription(MessageType, topic, callback, qos)`
✅ **Timers**: `create_timer(period_sec, callback)` for periodic tasks
✅ **CvBridge**: Bridge between ROS Image and OpenCV (install `ros-humble-cv-bridge`)
✅ **NumPy integration**: Convert ROS messages to arrays for scientific computing
✅ **Parameters**: `declare_parameter()` for runtime configuration

**Next steps**: Learn URDF (Unified Robot Description Format) to model your robot's kinematics and visualize it in RViz. Understanding robot descriptions is essential before diving into simulation with Gazebo and Isaac Sim.

---

**Related chapters**:
- [ROS 2 Architecture](./ros2-architecture.md) - Understand the DDS foundation
- [Nodes, Topics, and Services](./nodes-topics-services.md) - Communication patterns
- [URDF for Humanoids](./urdf-for-humanoids.md) - Next chapter: robot modeling
