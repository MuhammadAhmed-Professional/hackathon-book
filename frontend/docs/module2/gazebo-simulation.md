---
id: gazebo-simulation
title: "Gazebo Simulation Fundamentals"
sidebar_position: 1
description: "Master Gazebo Classic and Gazebo Sim for realistic robot simulation with physics, sensors, and ROS 2 integration"
---

# Gazebo Simulation Fundamentals

**Gazebo** is the industry-standard open-source robot simulator, powering research labs, universities, and companies worldwide. It provides photorealistic 3D environments with accurate physics simulation, enabling you to test algorithms before deploying to expensive hardware. Whether you're prototyping a wheeled robot or a 50-DOF humanoid, Gazebo accelerates development and reduces risk.

In this chapter, you'll learn to spawn robots in Gazebo, configure physics engines (ODE, Bullet, DART), add sensors (cameras, LIDAR, IMU), and integrate with ROS 2 for closed-loop control.

## Gazebo Classic vs Gazebo Sim (Ignition)

The Gazebo ecosystem has two active versions:

| Feature | Gazebo Classic (11) | Gazebo Sim (Ignition/Harmonic) |
|---------|-------------------|-------------------------------|
| **ROS 2 Support** | Via `gazebo_ros_pkgs` bridge | Native integration |
| **Physics Engines** | ODE, Bullet, DART, Simbody | DART (primary), Bullet, TPE |
| **Rendering** | OGRE 1.x | OGRE 2.x (PBR materials) |
| **Plugin System** | C++ only | C++, Python (future) |
| **Performance** | Single-threaded | Multi-threaded physics |
| **Status** | Maintenance mode | Active development |

**Recommendation**: Use **Gazebo Sim (Harmonic)** for new ROS 2 projects. It's included in ROS 2 Humble/Iron and offers better performance. However, many tutorials still reference Gazebo Classic, so understanding both is valuable.

## Installing Gazebo Sim on Ubuntu 22.04

```bash
# Install Gazebo Harmonic (latest stable as of 2024)
sudo apt-get update
sudo apt-get install gazebo-sim gz-tools

# Install ROS 2 Humble integration
sudo apt-get install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge

# Verify installation
gz sim --version
# Expected: Gazebo Sim, version 8.x.x
```

**Test Gazebo**:
```bash
gz sim shapes.sdf
```

You should see a window with geometric shapes (sphere, box, cylinder) falling under gravity.

## Launching Your First Robot in Gazebo

Let's spawn the humanoid arm from our previous URDF chapter:

**Step 1**: Convert URDF to SDF (Gazebo's native format)
```bash
# SDF supports more features than URDF (multiple models, lights, sensors)
gz sdf -p my_humanoid_arm.urdf > my_humanoid_arm.sdf
```

**Step 2**: Create a ROS 2 launch file

```python
# launch/gazebo_humanoid.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Path to robot description
    pkg_share = FindPackageShare('my_robot_description').find('my_robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'humanoid_arm.urdf')

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Gazebo world file (empty world by default)
    world_file = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'worlds',
        'empty.world'
    ])

    return LaunchDescription([
        # Launch Gazebo Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_args': '-r empty.sdf'  # -r starts paused
            }.items()
        ),

        # Spawn robot at origin
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'humanoid_arm',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '1.0'  # Spawn 1m above ground
            ],
            output='screen'
        ),

        # Robot State Publisher (TF tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True  # CRITICAL: Use Gazebo's clock
            }]
        ),

        # ROS-Gazebo bridge for clock
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        )
    ])
```

**Launch**:
```bash
ros2 launch my_robot_description gazebo_humanoid.launch.py
```

**Expected result**: Gazebo window opens with your humanoid arm floating 1m above ground plane. It will fall due to gravity and land on the ground.

## Understanding SDF (Simulation Description Format)

While URDF describes kinematics, SDF extends this for simulation:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="default">
    <!-- Physics configuration -->
    <physics name="ode_physics" type="ode">
      <max_step_size>0.001</max_step_size>  <!-- 1ms timestep -->
      <real_time_factor>1.0</real_time_factor>  <!-- 1x speed -->
      <real_time_update_rate>1000</real_time_update_rate>  <!-- 1000 Hz -->
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/ground_plane</uri>
    </include>

    <!-- Robot model -->
    <include>
      <uri>model://my_humanoid_arm</uri>
      <pose>0 0 1.0 0 0 0</pose>  <!-- x y z roll pitch yaw -->
    </include>
  </world>
</sdf>
```

**Key SDF features**:
- **Multiple models**: Unlike URDF, SDF can contain entire worlds
- **Plugins**: C++ extensions for sensors, actuators, custom physics
- **Fuel models**: Cloud repository of pre-built models (`fuel.gazebosim.org`)
- **Physics parameters**: Timestep, solver iterations, contact properties

## Configuring Physics Engines

Gazebo supports multiple physics engines, each with trade-offs:

### ODE (Open Dynamics Engine)
```xml
<physics name="ode_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>

  <ode>
    <solver>
      <type>quick</type>  <!-- or 'world' for more accuracy -->
      <iters>50</iters>  <!-- Constraint solver iterations -->
      <sor>1.3</sor>  <!-- Successive Over-Relaxation -->
    </solver>

    <constraints>
      <cfm>0.0</cfm>  <!-- Constraint Force Mixing (softness) -->
      <erp>0.2</erp>  <!-- Error Reduction Parameter -->
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

**Pros**: Fast, stable for wheeled robots and simple manipulators
**Cons**: Less accurate for stiff contacts (humanoid feet on ground)

### DART (Dynamic Animation and Robotics Toolkit)
```xml
<physics name="dart_physics" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>

  <dart>
    <solver>
      <solver_type>dantzig</solver_type>  <!-- LCP solver -->
    </solver>

    <collision_detector>bullet</collision_detector>  <!-- or 'ode', 'fcl' -->
  </dart>
</physics>
```

**Pros**: Best for legged robots, accurate contact forces
**Cons**: Slower than ODE for large scenes

### Bullet
```xml
<physics name="bullet_physics" type="bullet">
  <max_step_size>0.001</max_step_size>

  <bullet>
    <solver>
      <type>sequential_impulse</type>
      <iters>50</iters>
    </solver>
  </bullet>
</physics>
```

**Pros**: Good collision detection, used in games/movies
**Cons**: Less mature in Gazebo than ODE/DART

**Recommendation**: Use **DART** for humanoid robots (better contact handling), **ODE** for mobile robots (faster).

## Adding Sensors: Camera Plugin

Equip your robot with a camera:

```xml
<!-- Add to robot's head link in SDF -->
<sensor name="camera" type="camera">
  <pose>0.05 0 0 0 0 0</pose>  <!-- 5cm in front of head -->
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>  <!-- Realistic sensor noise -->
    </noise>
  </camera>

  <update_rate>30</update_rate>  <!-- 30 FPS -->
  <always_on>true</always_on>
  <visualize>true</visualize>  <!-- Show camera frustum in Gazebo -->

  <!-- Publish to ROS 2 topic -->
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
</sensor>
```

**Bridge camera to ROS 2**:
```bash
# In launch file, add parameter_bridge node
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
    ],
    output='screen'
)
```

**View camera feed**:
```bash
ros2 run rqt_image_view rqt_image_view /camera
```

## Adding Sensors: LIDAR (Laser Range Finder)

For navigation and SLAM:

```xml
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0.2 0 0 0</pose>  <!-- 20cm above base -->
  <topic>lidar</topic>
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>  <!-- -180° -->
        <max_angle>3.14159</max_angle>   <!-- +180° -->
      </horizontal>
      <vertical>
        <samples>16</samples>  <!-- 16 vertical beams -->
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>  <!-- -15° -->
        <max_angle>0.261799</max_angle>   <!-- +15° -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </lidar>

  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
</sensor>
```

**Bridge to ROS 2**:
```python
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
    output='screen'
)
```

**Visualize in RViz2**:
```bash
rviz2
# Add -> By topic -> /lidar -> LaserScan
# Set Fixed Frame to 'base_link'
```

## Joint Control: Effort (Torque) Plugin

Control your robot's joints with PID controllers:

```xml
<!-- Add to each actuated joint in SDF -->
<joint name="shoulder_pitch" type="revolute">
  <parent>torso</parent>
  <child>upper_arm</child>
  <!-- ... joint definition ... -->

  <plugin filename="gz-sim-joint-position-controller-system"
          name="gz::sim::systems::JointPositionController">
    <joint_name>shoulder_pitch</joint_name>
    <topic>/shoulder_pitch/cmd_pos</topic>
    <p_gain>100.0</p_gain>
    <i_gain>0.1</i_gain>
    <d_gain>10.0</d_gain>
    <i_max>1.0</i_max>
    <i_min>-1.0</i_min>
    <cmd_max>50.0</cmd_max>  <!-- Max effort (Nm) -->
    <cmd_min>-50.0</cmd_min>
  </plugin>
</joint>
```

**Command joint position from ROS 2**:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')

        self.publisher = self.create_publisher(
            Float64,
            '/shoulder_pitch/cmd_pos',
            10
        )

        self.timer = self.create_timer(1.0, self.send_command)
        self.position = 0.0

    def send_command(self):
        msg = Float64()
        self.position += 0.1  # Increment 0.1 rad each second

        if self.position > 1.57:  # 90 degrees
            self.position = -1.57

        msg.data = self.position
        self.publisher.publish(msg)
        self.get_logger().info(f'Commanded position: {self.position:.2f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Tuning Physics for Stability

Unstable simulations (robots exploding, jittering) often result from:

**Problem 1: Timestep too large**
```xml
<!-- Solution: Reduce timestep for stiff systems -->
<max_step_size>0.0001</max_step_size>  <!-- 0.1ms instead of 1ms -->
```

**Problem 2: Insufficient solver iterations**
```xml
<ode>
  <solver>
    <iters>100</iters>  <!-- Increase from default 50 -->
  </solver>
</ode>
```

**Problem 3: Heavy links on weak joints**
```xml
<!-- Solution: Increase joint damping -->
<joint name="shoulder_pitch" type="revolute">
  <dynamics>
    <damping>1.0</damping>  <!-- Nm/(rad/s) -->
    <friction>0.1</friction>
  </dynamics>
</joint>
```

**Problem 4: Collision geometry too complex**
```xml
<!-- Solution: Use convex decomposition or primitive shapes -->
<collision name="collision">
  <geometry>
    <cylinder radius="0.05" length="0.3"/>  <!-- Simpler than mesh -->
  </geometry>
</collision>
```

## Performance Optimization

For real-time humanoid simulation (50+ DOF):

**1. Reduce rendering quality**:
```bash
gz sim --render-engine ogre2 --render-quality low
```

**2. Disable GUI for headless runs**:
```bash
gz sim -s  # Server-only mode (no GUI)
```

**3. Use multi-threading** (DART only):
```xml
<physics type="dart">
  <dart>
    <collision_detector>bullet</collision_detector>
    <solver>
      <solver_type>pgs</solver_type>  <!-- Parallel Gauss-Seidel -->
    </solver>
  </dart>
</physics>
```

**4. Profile physics performance**:
```bash
gz topic -e -t /stats
# Monitor real_time_factor (should be ≥ 1.0 for real-time)
```

## Practical Exercise

**Challenge**: Create a humanoid reaching task in Gazebo:
1. Spawn your 7-DOF arm from the URDF chapter
2. Add a target sphere at position (0.5, 0.3, 1.0)
3. Write a Python node that publishes joint positions to reach the target
4. Add a camera sensor to the robot's end effector

**Success criteria**:
- [ ] Robot spawns without physics errors
- [ ] Camera feed visible in RViz2
- [ ] End effector reaches within 5cm of target
- [ ] Simulation runs at ≥ 0.5x real-time

## Key Takeaways

✅ **Gazebo versions**: Classic (maintenance) vs Sim/Harmonic (active)
✅ **SDF format**: World description with physics, lighting, models
✅ **Physics engines**: DART (legged robots), ODE (mobile robots), Bullet (general)
✅ **Sensor plugins**: Camera, LIDAR, IMU with realistic noise models
✅ **ROS 2 bridge**: `ros_gz_bridge` for topic/service integration
✅ **Joint control**: PID controllers with effort/position/velocity commands
✅ **Performance**: Reduce timestep, increase iterations, simplify collision

**Next steps**: Compare Gazebo with NVIDIA Isaac Sim to understand when to use photorealistic ray-tracing (synthetic data generation) vs fast physics (reinforcement learning training).

---

**Related chapters**:
- [URDF for Humanoids](../module1/urdf-for-humanoids.md) - Robot modeling prerequisite
- [URDF vs SDF](./urdf-vs-sdf.md) - Deep dive into format differences
- [Isaac Sim](../module3/isaac-sim.md) - GPU-accelerated alternative
