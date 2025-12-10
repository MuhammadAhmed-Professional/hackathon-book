---
id: urdf-for-humanoids
title: "URDF for Humanoid Robots"
sidebar_position: 5
description: "Model humanoid robot kinematics with URDF: joints, links, and visualization in RViz"
---

# URDF for Humanoid Robots

The **Unified Robot Description Format (URDF)** is the lingua franca for describing robot kinematics in ROS 2. It defines the structure of your robot—from a simple wheeled platform to a 50-DOF humanoid—as an XML tree of links (rigid bodies) and joints (connections between links). Mastering URDF is essential for simulation, motion planning, and visualization.

In this chapter, you'll learn to model a humanoid robot's kinematic chain, define collision geometry, set up visual meshes, and validate your description in RViz2.

## Why URDF for Humanoid Robots?

Humanoid robots present unique challenges compared to mobile manipulators:

- **High DOF count**: 20-50+ joints (head, torso, arms, hands, legs, feet)
- **Closed kinematic chains**: Parallel mechanisms in hands and feet
- **Complex mass distribution**: Balancing requires accurate inertial properties
- **Realistic visualization**: Human-like appearance for HRI (Human-Robot Interaction) studies

URDF provides:
✅ **Kinematic tree**: Parent-child relationships for forward kinematics
✅ **Dynamic properties**: Mass, inertia, center of mass for simulation
✅ **Collision geometry**: Simplified shapes for fast collision checking
✅ **Visual meshes**: STL/DAE/OBJ files for photorealistic rendering

**URDF vs MJCF vs SDF**: URDF is the ROS standard, MuJoCo XML (MJCF) is used in reinforcement learning (e.g., Isaac Gym), and SDF (Simulation Description Format) extends URDF for Gazebo. We'll focus on URDF here and cover SDF conversion in Module 2.

## URDF Structure: Links and Joints

Every URDF file is an XML tree with two primary elements:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>  <!-- Torso: 30cm x 20cm x 40cm -->
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>  <!-- Same as visual for simplicity -->
      </geometry>
    </collision>

    <inertial>
      <mass value="5.0"/>  <!-- 5 kg torso -->
      <origin xyz="0 0 0.2"/>  <!-- CoM at torso center -->
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="shoulder_pitch_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0.15 0 0.35" rpy="0 0 0"/>  <!-- Shoulder position -->
    <axis xyz="0 1 0"/>  <!-- Rotate around Y-axis (pitch) -->
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <link name="upper_arm_link">
    <!-- Upper arm definition -->
  </link>
</robot>
```

**Key concepts**:
- **Links**: Define geometry (visual/collision) and dynamics (mass/inertia)
- **Joints**: Connect links with motion constraints (revolute, prismatic, fixed)
- **Origin**: Translation (xyz) and rotation (rpy: roll-pitch-yaw in radians)
- **Limits**: Joint range, max effort (torque in Nm), max velocity (rad/s)

## Building a 7-DOF Humanoid Arm

Let's model a realistic humanoid arm with 7 degrees of freedom:

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">
  <!-- Base link (torso attachment) -->
  <link name="torso"/>

  <!-- 1. Shoulder Yaw Joint -->
  <joint name="shoulder_yaw" type="revolute">
    <parent link="torso"/>
    <child link="shoulder_link"/>
    <origin xyz="0.15 0.25 0.35" rpy="0 0 0"/>  <!-- Right shoulder -->
    <axis xyz="0 0 1"/>  <!-- Yaw around Z -->
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <link name="shoulder_link">
    <visual>
      <geometry>
        <sphere radius="0.05"/>  <!-- Shoulder joint housing -->
      </geometry>
      <material name="black"><color rgba="0.1 0.1 0.1 1.0"/></material>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- 2. Shoulder Pitch Joint -->
  <joint name="shoulder_pitch" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Pitch around Y -->
    <limit lower="-3.14" upper="0.785" effort="50" velocity="2.0"/>
  </joint>

  <link name="upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>  <!-- Cylinder center offset -->
      <geometry>
        <cylinder radius="0.04" length="0.3"/>  <!-- 30cm upper arm -->
      </geometry>
      <material name="white"><color rgba="0.9 0.9 0.9 1.0"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>  <!-- Slightly larger for collision -->
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15"/>
      <mass value="1.2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- 3. Shoulder Roll Joint -->
  <joint name="shoulder_roll" type="revolute">
    <parent link="upper_arm"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>  <!-- Roll around X -->
    <limit lower="-1.57" upper="1.57" effort="30" velocity="2.0"/>
  </joint>

  <link name="elbow_link">
    <visual>
      <geometry>
        <sphere radius="0.04"/>  <!-- Elbow joint -->
      </geometry>
      <material name="black"/>
    </visual>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- 4. Elbow Pitch Joint -->
  <joint name="elbow_pitch" type="revolute">
    <parent link="elbow_link"/>
    <child link="forearm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.618" effort="30" velocity="2.0"/>  <!-- 0° to 150° -->
  </joint>

  <link name="forearm">
    <visual>
      <origin xyz="0 0 -0.125"/>
      <geometry>
        <cylinder radius="0.035" length="0.25"/>  <!-- 25cm forearm -->
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.125"/>
      <mass value="0.8"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- 5-7. Wrist Joints (Yaw, Pitch, Roll) -->
  <!-- Omitted for brevity - follow same pattern -->

</robot>
```

**Design principles**:
- **Joint naming**: `{body_part}_{motion_type}` (e.g., `shoulder_pitch`)
- **Coordinate frames**: ROS uses right-handed XYZ (X forward, Y left, Z up)
- **Inertia tensors**: Use CAD software (SolidWorks, Fusion 360) to calculate accurate values
- **Collision geometry**: Use simpler shapes (spheres, cylinders) than visual meshes for speed

## Calculating Inertia Tensors

The inertia tensor describes how mass is distributed around the center of mass:

```
I = | Ixx  Ixy  Ixz |
    | Iyx  Iyy  Iyz |
    | Izx  Izy  Izz |
```

**For common shapes**:

**Solid cylinder** (radius r, length l, mass m, axis along Z):
```
Ixx = Iyy = (1/12) * m * (3r² + l²)
Izz = (1/2) * m * r²
```

**Solid sphere** (radius r, mass m):
```
Ixx = Iyy = Izz = (2/5) * m * r²
```

**Solid box** (dimensions dx, dy, dz, mass m):
```
Ixx = (1/12) * m * (dy² + dz²)
Iyy = (1/12) * m * (dx² + dz²)
Izz = (1/12) * m * (dx² + dy²)
```

**Python helper for cylinder inertia**:
```python
import numpy as np

def cylinder_inertia(mass, radius, length):
    """Calculate inertia tensor for solid cylinder along Z-axis."""
    ixx = iyy = (1/12) * mass * (3 * radius**2 + length**2)
    izz = 0.5 * mass * radius**2
    return {
        'ixx': ixx, 'ixy': 0.0, 'ixz': 0.0,
        'iyy': iyy, 'iyz': 0.0, 'izz': izz
    }

# Example: Upper arm (1.2 kg, 4cm radius, 30cm length)
inertia = cylinder_inertia(1.2, 0.04, 0.30)
print(f"<inertia ixx='{inertia['ixx']:.6f}' ixy='0' ixz='0'")
print(f"         iyy='{inertia['iyy']:.6f}' iyz='0' izz='{inertia['izz']:.6f}'/>")
```

**Output**:
```xml
<inertia ixx='0.009096' ixy='0' ixz='0'
         iyy='0.009096' iyz='0' izz='0.000960'/>
```

## Using Mesh Files for Realistic Visuals

For production robots, use CAD-exported meshes:

```xml
<link name="head">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://my_robot_description/meshes/head.stl" scale="1.0 1.0 1.0"/>
    </geometry>
    <material name="skin_tone">
      <color rgba="0.95 0.8 0.7 1.0"/>
    </material>
  </visual>

  <collision>
    <!-- Simplified collision geometry (sphere approximation) -->
    <geometry>
      <sphere radius="0.12"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="3.5"/>  <!-- Human head ~3-4 kg -->
    <inertia ixx="0.02" ixy="0" ixz="0"
             iyy="0.02" iyz="0" izz="0.015"/>
  </inertial>
</link>
```

**Mesh file formats**:
- **STL**: Binary/ASCII triangle mesh (most common)
- **DAE (Collada)**: Supports textures and materials
- **OBJ**: Lightweight, plain text
- **URDF package paths**: `package://package_name/meshes/file.stl` resolves to ROS 2 package location

**Mesh preparation tips**:
1. Decimate meshes to < 10,000 triangles for real-time rendering
2. Export collision meshes with convex hull approximation
3. Use binary STL (smaller, faster to parse)
4. Apply scale in CAD software, not URDF (scale="1 1 1" ideal)

## Validating URDF with check_urdf

Before launching in RViz, validate your URDF syntax:

```bash
# Install URDF tools
sudo apt install liburdfdom-tools

# Check URDF syntax and kinematic tree
check_urdf my_humanoid.urdf
```

**Expected output**:
```
robot name is: humanoid_arm
---------- Successfully Parsed XML ---------------
root Link: torso has 1 child(ren)
    child(1):  shoulder_link
        child(1):  upper_arm
            child(1):  elbow_link
                child(1):  forearm
```

**Common errors**:
- `link 'X' is not connected to the root` → Missing joint chain
- `joint 'Y' has invalid parent/child` → Link name typo
- `inertia matrix not positive definite` → Invalid inertia values

## Visualizing in RViz2

Create a launch file to view your URDF:

```python
# launch/view_humanoid.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to URDF file
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'humanoid_arm.urdf'
    )

    # Load URDF into robot_description parameter
    robot_description = Command(['cat ', urdf_file])

    return LaunchDescription([
        # Robot State Publisher (publishes TF transforms)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),

        # Joint State Publisher GUI (control joints with sliders)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('my_robot_description'),
                'rviz',
                'view_robot.rviz'
            )]
        )
    ])
```

**Launch**:
```bash
ros2 launch my_robot_description view_humanoid.launch.py
```

**RViz2 configuration**:
1. Set **Fixed Frame** to `torso`
2. Add **RobotModel** display
3. Add **TF** display to visualize coordinate frames
4. Use **Joint State Publisher GUI** sliders to move joints

## Xacro: Macros for Complex Robots

For 50+ joint humanoids, writing raw XML becomes tedious. **Xacro** (XML Macros) adds:
- **Variables**: `<xacro:property name="arm_length" value="0.3"/>`
- **Macros**: Reusable templates for repeated structures
- **Math**: `${arm_length * 0.5}`

**Example xacro macro**:
```xml
<xacro:macro name="humanoid_arm" params="prefix reflect">
  <joint name="${prefix}_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="${prefix}_upper_arm"/>
    <origin xyz="0.15 ${reflect * 0.25} 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="0.785" effort="50" velocity="2.0"/>
  </joint>

  <link name="${prefix}_upper_arm">
    <!-- Link definition -->
  </link>

  <!-- More joints and links -->
</xacro:macro>

<!-- Instantiate left and right arms -->
<xacro:humanoid_arm prefix="left" reflect="1"/>
<xacro:humanoid_arm prefix="right" reflect="-1"/>
```

**Convert xacro to URDF**:
```bash
xacro my_robot.urdf.xacro > my_robot.urdf
```

## Practical Exercise

**Challenge**: Create a simplified humanoid torso with:
1. **Base link**: Torso (box: 0.3m x 0.2m x 0.4m, 5 kg)
2. **Head joint**: Fixed joint connecting head sphere (radius 0.12m, 3 kg) at torso top
3. **Left arm**: 3-DOF arm (shoulder pitch, shoulder roll, elbow pitch) using cylinders
4. **Joint limits**: Realistic human range of motion

**Validation checklist**:
- [ ] `check_urdf` passes without errors
- [ ] RViz2 shows robot without collision warnings
- [ ] All joints move within specified limits
- [ ] TF tree has no broken chains

## Key Takeaways

✅ **URDF structure**: Links (geometry + dynamics) + Joints (constraints)
✅ **Humanoid challenges**: High DOF, complex inertia, mesh files
✅ **Joint types**: Revolute (rotation), prismatic (translation), fixed
✅ **Inertia calculation**: Use CAD or analytical formulas
✅ **Mesh workflows**: STL/DAE for visual, simplified shapes for collision
✅ **Validation**: `check_urdf` + RViz2 + Joint State Publisher GUI
✅ **Xacro**: Macros for maintainable multi-DOF robots

**Next steps**: Take your URDF into Gazebo and Isaac Sim for physics simulation. You'll learn how to add actuators, sensors, and run control algorithms on your virtual humanoid.

---

**Related chapters**:
- [Python Integration](./python-integration.md) - Control URDF joints with rclpy
- [Gazebo Simulation](../module2/gazebo-simulation.md) - Next chapter: simulate your robot
- [Isaac Sim](../module3/isaac-sim.md) - Photorealistic rendering with RTX
