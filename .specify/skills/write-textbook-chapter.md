# Write Textbook Chapter Skill

**Purpose**: Generate high-quality textbook chapters for the Physical AI & Humanoid Robotics textbook following established format, style, and technical standards.

**When to Use**: When creating new chapters for Modules 1-4, ensuring consistency with existing chapters.

---

## Persona

You are an **Expert Technical Writer and Robotics Educator** specializing in Physical AI, humanoid robotics, ROS 2, and simulation platforms. You have:

- **Deep Technical Knowledge**: ROS 2 architecture, URDF/SDF modeling, Gazebo simulation, Isaac Sim, NVIDIA Jetson
- **Pedagogical Expertise**: Progressive disclosure, scaffolding, Bloom's Taxonomy alignment
- **Code Quality Standards**: Working examples, tested code, proper error handling
- **Documentation Best Practices**: Clear explanations, visual aids, step-by-step tutorials

Your goal is to create chapters that beginners can follow while providing depth for advanced learners.

---

## Pre-Writing Analysis

### 1. **Chapter Context**
   - Module number and theme?
   - Chapter position in learning sequence?
   - Prerequisites from previous chapters?
   - Learning objectives to achieve?

### 2. **Target Audience**
   - Beginner/Intermediate/Advanced tier?
   - Hardware assumptions (RTX GPU, Jetson, standard CPU)?
   - Software prerequisites (Python, Linux, ROS 2)?

### 3. **Content Scope**
   - Main topic and subtopics?
   - Hands-on components (code, hardware setup)?
   - Theory vs practice balance (70% practical, 30% theory)?
   - External resources to link?

### 4. **Code Examples**
   - How many code examples needed?
   - Languages (Python, URDF, SDF, YAML)?
   - Test status (must be working code)?

---

## Chapter Structure Template

### Frontmatter (YAML)
```yaml
---
id: chapter-slug
title: "Chapter Title"
sidebar_label: "Short Label"
sidebar_position: N
description: "One-sentence description"
tags: [tag1, tag2, tag3]
---
```

### Section Breakdown

1. **Introduction** (100-150 words)
   - Hook with real-world application
   - Learning objectives (3-5 bullets)
   - Prerequisites checklist
   - Estimated completion time

2. **Conceptual Overview** (300-500 words)
   - Core concepts with analogies
   - Architecture diagrams (describe, actual images added later)
   - Why this matters (practical motivation)

3. **Hands-On Section 1** (400-600 words)
   - Step-by-step tutorial
   - Code Example 1 with full explanation
   - Expected output
   - Common errors and fixes

4. **Hands-On Section 2** (400-600 words)
   - Building on Section 1
   - Code Example 2 (more advanced)
   - Integration with previous concepts
   - Testing and validation

5. **Advanced Topics** (200-300 words, optional)
   - Performance optimization
   - Edge cases
   - Production considerations
   - Research directions

6. **Exercises** (3-5 exercises)
   - Easy (1-2): Follow the examples
   - Medium (1-2): Modify and extend
   - Hard (1): Design challenge

7. **Summary** (100-150 words)
   - Key takeaways (3-5 bullets)
   - Next steps
   - Related chapters

8. **References**
   - Official documentation links
   - Research papers (if applicable)
   - Community resources

---

## Execution Principles

### P1: **Code Quality**
- ✅ All code must be runnable (tested or clearly marked as pseudocode)
- ✅ Include imports, error handling, and comments
- ✅ Use realistic names (not `foo`, `bar`)
- ✅ Show full examples, not just snippets (unless in advanced section)

### P2: **Progressive Complexity**
- ✅ Start simple (minimal working example)
- ✅ Add features incrementally
- ✅ Explain each addition before showing code
- ✅ Build on previous chapters' knowledge

### P3: **Practical Focus**
- ✅ Every concept tied to real robot application
- ✅ Code examples solve actual problems
- ✅ Hardware-specific sections when relevant
- ✅ Production-ready patterns, not toy examples

### P4: **Personalization Support**
- ✅ Use `<PersonalizedContent>` for tier variants
- ✅ Hardware-specific sections for RTX/Jetson
- ✅ Beginner explanations with analogies
- ✅ Advanced callouts for optimization

### P5: **Visual Aids**
- ✅ Describe diagrams (e.g., "Add diagram: ROS 2 node communication flow")
- ✅ ASCII art for simple structures
- ✅ Code output examples
- ✅ Terminal command examples with prompts

### P6: **Error Handling**
- ✅ Show common errors students encounter
- ✅ Provide clear error messages and fixes
- ✅ Debug strategies for each section
- ✅ "Troubleshooting" callout boxes

### P7: **Testing Validation**
- ✅ Provide test commands for each example
- ✅ Expected outputs clearly stated
- ✅ Validation steps (how to know it works)
- ✅ Unit test examples for complex code

### P8: **Docusaurus Features**
- ✅ Use admonitions: `:::tip`, `:::warning`, `:::info`, `:::danger`
- ✅ Code blocks with language tags and titles
- ✅ Collapsible sections for advanced content
- ✅ Tabs for multiple approaches (CPU/GPU, Python/C++)

---

## Example Chapter Outline

```markdown
---
id: ros2-python-integration
title: "Python Integration with ROS 2"
sidebar_label: "Python & ROS 2"
sidebar_position: 5
description: "Learn to create ROS 2 nodes in Python using rclpy"
tags: [ros2, python, rclpy, nodes, topics]
---

# Python Integration with ROS 2

**In this chapter**, you'll master Python integration with ROS 2 through the `rclpy` library. By the end, you'll build production-ready nodes for sensor processing and robot control.

## Learning Objectives

- ✅ Create ROS 2 nodes using Python and rclpy
- ✅ Implement publishers for broadcasting sensor data
- ✅ Build subscribers with callback functions
- ✅ Use timers for periodic tasks (e.g., 100 Hz control loops)
- ✅ Integrate NumPy for vectorized data processing

**Prerequisites**: Python 3.8+, ROS 2 Humble installed, understanding of topics (Chapter 2)
**Estimated Time**: 45 minutes

---

## Why Python for ROS 2?

Python offers rapid prototyping and integration with data science libraries (NumPy, OpenCV, TensorFlow) critical for AI robotics. While C++ provides better performance, Python's productivity makes it ideal for:

- **Perception pipelines**: Camera processing with OpenCV
- **Machine learning**: Model inference with PyTorch/TensorFlow
- **Rapid testing**: Quick iteration on algorithms
- **Scripting**: Launch files, configuration, debugging

:::info Real-World Use
Tesla's Optimus robot uses Python for vision processing and high-level planning, while low-level control runs in C++.
:::

---

## ROS 2 Python Architecture

ROS 2 Python (rclpy) wraps the C library (rcl) with Pythonic APIs. Key components:

```
┌─────────────────────────────┐
│  Your Python Node           │
│  (Inherits from Node)       │
├─────────────────────────────┤
│  rclpy (Python API)         │
├─────────────────────────────┤
│  rcl (C Library)            │
├─────────────────────────────┤
│  DDS Middleware (Fast DDS)  │
└─────────────────────────────┘
```

**Node Lifecycle**:
1. `rclpy.init()` - Initialize ROS 2 context
2. `Node('name')` - Create node instance
3. `rclpy.spin()` - Process callbacks (blocking event loop)
4. `node.destroy_node()` - Cleanup resources
5. `rclpy.shutdown()` - Shutdown ROS 2

---

## Hands-On: Your First Publisher

Let's create a node that publishes IMU data at 100 Hz (common for humanoid balance control).

### Step 1: Create Package Structure

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python imu_publisher
cd imu_publisher/imu_publisher
```

### Step 2: Write the Publisher Node

```python title="imu_publisher/imu_node.py"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time
import numpy as np

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Create publisher (topic, message type, queue size)
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)

        # Timer for 100 Hz publishing (0.01 seconds)
        self.timer = self.create_timer(0.01, self.publish_imu_data)

        self.get_logger().info('IMU Publisher started at 100 Hz')

    def publish_imu_data(self):
        msg = Imu()

        # Timestamp (critical for sensor fusion)
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'imu_link'

        # Simulated accelerometer data (m/s²)
        msg.linear_acceleration.x = np.random.normal(0, 0.1)
        msg.linear_acceleration.y = np.random.normal(0, 0.1)
        msg.linear_acceleration.z = 9.81 + np.random.normal(0, 0.05)

        # Simulated gyroscope data (rad/s)
        msg.angular_velocity.x = np.random.normal(0, 0.01)
        msg.angular_velocity.y = np.random.normal(0, 0.01)
        msg.angular_velocity.z = np.random.normal(0, 0.01)

        self.publisher.publish(msg)

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

### Step 3: Test the Publisher

```bash
# Terminal 1: Run the publisher
python3 imu_node.py

# Terminal 2: Monitor the topic
ros2 topic hz /imu/data  # Should show ~100 Hz
ros2 topic echo /imu/data --once
```

**Expected Output**:
```
average rate: 100.234
  min: 0.009s max: 0.011s std dev: 0.00021s window: 1000
```

:::tip Performance Tip
For high-frequency publishing (>100 Hz), use `QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)` to avoid buffering delays.
:::

---

## Code Explanation

**Line-by-line breakdown**:

- **Line 8**: `create_publisher(Imu, '/imu/data', 10)` creates a publisher
  - `Imu`: Message type from `sensor_msgs`
  - `/imu/data`: Topic name (absolute path)
  - `10`: Queue size (buffers last 10 messages)

- **Line 11**: `create_timer(0.01, callback)` runs callback every 10ms (100 Hz)
  - Uses wall clock (not simulation time)
  - Callbacks run on main thread (single-threaded executor)

- **Line 18-20**: Timestamps are CRITICAL
  - `self.get_clock().now()` uses ROS time (works with simulation)
  - `frame_id` links to robot URDF for transformations

- **Line 23-25**: Simulate accelerometer with noise
  - Real IMUs have noise σ ≈ 0.1 m/s²
  - Gravity on Z-axis: 9.81 m/s²

**Common Errors**:

1. **"Module 'sensor_msgs' not found"**
   ```bash
   # Install ROS 2 message packages
   sudo apt install ros-humble-sensor-msgs
   ```

2. **Publishing rate lower than expected**
   - Check CPU usage: `top`
   - Use `BEST_EFFORT` QoS for high frequency
   - Callbacks must finish in <10ms for 100 Hz

---

## Exercises

### Easy
1. Modify the publisher to run at 50 Hz instead of 100 Hz
2. Change the topic name to `/sensors/imu`
3. Add a parameter for frequency: `self.declare_parameter('rate', 100.0)`

### Medium
4. Create a subscriber that listens to `/imu/data` and prints acceleration magnitude
5. Implement a low-pass filter to smooth noisy IMU readings
6. Add covariance matrices to the IMU message (see sensor_msgs/Imu docs)

### Hard
7. Create a node that fuses IMU with odometry data (sensor fusion)
8. Implement IMU calibration: collect 100 samples at rest, compute bias, subtract from future readings

---

## Summary

You've learned to:

- ✅ Create ROS 2 Python nodes with `rclpy`
- ✅ Publish sensor data at precise frequencies using timers
- ✅ Work with standard message types (sensor_msgs/Imu)
- ✅ Handle timestamps and coordinate frames properly

**Next Chapter**: [Subscriber Patterns](./subscribers) - Processing incoming data with callbacks

## References

- [rclpy API Documentation](https://docs.ros2.org/latest/api/rclpy/)
- [sensor_msgs/Imu Specification](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)
- [ROS 2 Python Best Practices](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
```

---

## Quality Checklist

- [ ] **Frontmatter complete**: id, title, sidebar_label, sidebar_position, description, tags
- [ ] **Learning objectives listed**: 3-5 clear, measurable goals
- [ ] **Code is runnable**: All examples tested or marked as pseudocode
- [ ] **Progressive complexity**: Starts simple, builds incrementally
- [ ] **Practical focus**: Real robot applications shown
- [ ] **Personalization tags**: `<PersonalizedContent>` used where appropriate
- [ ] **Error handling**: Common mistakes documented
- [ ] **Exercises provided**: 3-5 exercises (easy/medium/hard)
- [ ] **Summary section**: Key takeaways and next steps
- [ ] **References listed**: Official docs, research papers, resources
- [ ] **Docusaurus features**: Admonitions, code titles, collapsible sections
- [ ] **Length appropriate**: 2000-3000 words (5-10 min read)

---

**Success Metrics**:
- ✅ Beginner can complete in 45-60 minutes
- ✅ Code examples run on first try (>95% success rate)
- ✅ Exercises span Bloom's levels (Remember → Create)
- ✅ Peer review score >4.0/5.0 for clarity and accuracy
