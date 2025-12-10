---
id: urdf-vs-sdf
title: "URDF vs SDF: Choosing the Right Format"
sidebar_position: 2
description: "Understand when to use URDF vs SDF for robot descriptions, with conversion workflows"
---

# URDF vs SDF: Choosing the Right Format

As you dive deeper into robot simulation, you'll encounter two primary robot description formats: **URDF (Unified Robot Description Format)** and **SDF (Simulation Description Format)**. While both describe robot geometry and kinematics, they serve different purposes and have distinct capabilities. Understanding when to use each—and how to convert between them—is essential for effective robotics development.

In this chapter, you'll learn the technical differences between URDF and SDF, discover their respective strengths and limitations, and master the conversion workflows that connect ROS 2 ecosystems with Gazebo simulations.

## The Origins: Why Two Formats?

**URDF** was created as part of ROS 1 (circa 2009) to standardize robot descriptions for visualization (RViz) and motion planning (MoveIt). Its design prioritizes:
- Kinematic tree representation
- ROS tool compatibility
- Human-readable XML
- TF (Transform) tree generation

**SDF** emerged from Gazebo's need for more expressive simulation capabilities (circa 2012). It extends beyond URDF to support:
- Complete world descriptions (multiple robots, lights, physics)
- Advanced sensor models (GPU LIDAR, depth cameras)
- Plugin architecture for custom behaviors
- Multiple physics engines with detailed configurations

**Key insight**: URDF is a **robot description format**, while SDF is a **world description format**. Think of URDF as a blueprint for one robot, and SDF as a blueprint for an entire simulated environment.

## URDF: Strengths and Limitations

### Strengths

**1. ROS Ecosystem Integration**
```xml
<!-- URDF seamlessly integrates with ROS 2 tools -->
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry><box size="0.5 0.3 0.2"/></geometry>
    </visual>
  </link>
</robot>
```

- **robot_state_publisher**: Automatically publishes TF transforms
- **joint_state_publisher**: GUI for manual joint control
- **MoveIt 2**: Motion planning requires URDF input
- **RViz2**: Native URDF visualization

**2. Simplicity and Readability**
URDF files are concise for basic robots. A wheeled platform can be defined in < 100 lines of XML.

**3. Xacro Support**
Xacro (XML Macros) adds programmability:
```xml
<xacro:property name="wheel_radius" value="0.1"/>
<xacro:macro name="wheel" params="prefix reflect">
  <link name="${prefix}_wheel">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="0.05"/>
      </geometry>
    </visual>
  </link>
</xacro:macro>

<xacro:wheel prefix="left" reflect="1"/>
<xacro:wheel prefix="right" reflect="-1"/>
```

### Limitations

**1. No Multi-Robot Support**
URDF describes **one robot at a time**. You cannot define multiple robots or environmental objects (tables, walls) in a single URDF file.

**2. Limited Physics Configuration**
```xml
<!-- URDF has minimal physics control -->
<joint name="elbow" type="revolute">
  <dynamics damping="0.5" friction="0.1"/>
  <!-- That's it - no solver config, contact parameters, etc. -->
</joint>
```

**3. No Sensor Plugins**
URDF has no native plugin system. To add a camera, you must:
1. Define the sensor link in URDF
2. Add Gazebo-specific tags (wrapped in `<gazebo>` tags)
3. Hope the Gazebo parser recognizes your plugin

**4. No World Elements**
URDF cannot specify:
- Lighting (sun, spotlights, ambient)
- Ground planes
- Gravity direction/magnitude
- Atmospheric effects

## SDF: Strengths and Limitations

### Strengths

**1. Complete World Descriptions**
```xml
<sdf version="1.9">
  <world name="humanoid_lab">
    <!-- Physics engine configuration -->
    <physics type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
    </light>

    <!-- Ground plane -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/ground_plane</uri>
    </include>

    <!-- Multiple robots -->
    <model name="robot1">
      <pose>0 0 0 0 0 0</pose>
      <!-- Robot definition -->
    </model>

    <model name="robot2">
      <pose>5 0 0 0 0 0</pose>
      <!-- Another robot -->
    </model>
  </world>
</sdf>
```

**2. Advanced Sensor Modeling**
```xml
<sensor name="rgbd_camera" type="rgbd_camera">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
    <distortion>
      <k1>-0.25</k1>  <!-- Barrel distortion -->
      <k2>0.12</k2>
      <k3>-0.03</k3>
      <p1>-0.0005</p1>  <!-- Tangential distortion -->
      <p2>0.0002</p2>
      <center>0.5 0.5</center>
    </distortion>
  </camera>

  <!-- Native plugin support -->
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
</sensor>
```

**3. Detailed Physics Configuration**
```xml
<physics type="dart">
  <dart>
    <solver>
      <solver_type>dantzig</solver_type>  <!-- LCP solver choice -->
      <solver_iterations>50</solver_iterations>
    </solver>

    <collision_detector>bullet</collision_detector>

    <!-- Contact surface properties -->
    <contact>
      <friction>
        <ode>
          <mu>1.0</mu>  <!-- Coefficient of friction -->
          <mu2>0.8</mu2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.3</restitution_coefficient>
        <threshold>0.01</threshold>
      </bounce>
    </contact>
  </dart>
</physics>
```

**4. Plugin System**
SDF plugins extend functionality without recompiling Gazebo:
- Model plugins: Custom robot behaviors
- Sensor plugins: Custom sensor processing
- World plugins: Environmental effects (wind, water currents)
- System plugins: Core simulation modifications

### Limitations

**1. Complexity**
SDF files can grow to thousands of lines for realistic worlds. A humanoid robot in a furnished room might be 5,000+ lines.

**2. Limited ROS Tool Support**
- MoveIt 2 requires URDF (no native SDF support)
- Most ROS visualization tools expect URDF
- `robot_state_publisher` only reads URDF

**3. Version Fragmentation**
SDF has multiple versions (1.4, 1.6, 1.7, 1.9), with breaking changes between them. URDF is more stable.

## When to Use URDF vs SDF

| Use Case | Recommended Format | Rationale |
|----------|-------------------|-----------|
| **Motion Planning (MoveIt 2)** | URDF | MoveIt requires URDF kinematic descriptions |
| **Gazebo Simulation** | SDF | Native format, full feature support |
| **RViz Visualization** | URDF | Direct compatibility with robot_state_publisher |
| **Multi-Robot Simulation** | SDF | URDF cannot describe multiple robots |
| **Custom Sensors (LIDAR, Depth Camera)** | SDF | Advanced sensor models and plugins |
| **ROS Package Distribution** | URDF | Standard for sharing robot descriptions |
| **Physics Research** | SDF | Detailed physics engine configuration |
| **Quick Prototyping** | URDF | Simpler syntax, faster iteration |

**Practical recommendation**: Start with URDF for robot modeling, then convert to SDF when you need advanced Gazebo features.

## Converting URDF to SDF

Gazebo provides built-in conversion tools:

```bash
# Method 1: Using gz command-line tool
gz sdf -p my_robot.urdf > my_robot.sdf

# Method 2: Programmatic conversion in launch files
from launch_ros.actions import Node

Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['--urdf-file', 'my_robot.urdf', '--output-sdf', 'my_robot.sdf']
)
```

**Conversion behavior**:
- ✅ Links and joints are preserved
- ✅ Visual and collision geometry converted
- ✅ Inertial properties transferred
- ⚠️ Gazebo-specific `<gazebo>` tags extracted to SDF plugins
- ⚠️ Xacro macros must be pre-processed (`xacro my_robot.urdf.xacro > my_robot.urdf`)

**Example conversion**:

**Input URDF**:
```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <link name="base_link">
    <visual>
      <geometry><box size="0.2 0.2 0.1"/></geometry>
    </visual>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
    </inertial>
  </link>

  <link name="arm_link">
    <visual>
      <geometry><cylinder radius="0.03" length="0.5"/></geometry>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.04" ixy="0" ixz="0" iyy="0.04" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>
</robot>
```

**Output SDF** (simplified):
```xml
<sdf version="1.9">
  <model name="simple_arm">
    <link name="base_link">
      <pose>0 0 0 0 0 0</pose>
      <visual name="visual">
        <geometry><box><size>0.2 0.2 0.1</size></box></geometry>
      </visual>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.1</iyy><iyz>0</iyz><izz>0.05</izz>
        </inertia>
      </inertial>
    </link>

    <link name="arm_link">
      <pose relative_to="shoulder">0 0 0 0 0 0</pose>
      <visual name="visual">
        <geometry><cylinder><radius>0.03</radius><length>0.5</length></cylinder></geometry>
      </visual>
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.04</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.04</iyy><iyz>0</iyz><izz>0.001</izz>
        </inertia>
      </inertial>
    </link>

    <joint name="shoulder" type="revolute">
      <parent>base_link</parent>
      <child>arm_link</child>
      <pose relative_to="base_link">0 0 0.05 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>50</effort>
          <velocity>2.0</velocity>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
```

**Key differences**:
- SDF wraps everything in `<model>` tag
- `<pose>` replaces URDF's `<origin>`
- More verbose inertia syntax
- `relative_to` attribute for frame relationships

## Converting SDF to URDF (Limited)

SDF → URDF conversion is **not officially supported** because SDF contains features that cannot be represented in URDF (world elements, plugins, advanced sensors). However, you can extract a single model:

```python
#!/usr/bin/env python3
import xml.etree.ElementTree as ET

def sdf_model_to_urdf(sdf_file, model_name, output_urdf):
    """Extract single model from SDF and convert to URDF."""
    tree = ET.parse(sdf_file)
    root = tree.getroot()

    # Find the model element
    model = root.find(f".//model[@name='{model_name}']")
    if model is None:
        raise ValueError(f"Model '{model_name}' not found in SDF")

    # Create URDF root
    urdf_root = ET.Element('robot', name=model_name)

    # Convert links
    for sdf_link in model.findall('link'):
        urdf_link = ET.SubElement(urdf_root, 'link', name=sdf_link.get('name'))

        # Copy visual elements
        for visual in sdf_link.findall('visual'):
            urdf_visual = ET.SubElement(urdf_link, 'visual')
            # Copy geometry, etc.

        # Copy collision elements
        # Copy inertial elements
        # ... (implementation details omitted)

    # Convert joints
    for sdf_joint in model.findall('joint'):
        urdf_joint = ET.SubElement(urdf_root, 'joint',
                                    name=sdf_joint.get('name'),
                                    type=sdf_joint.get('type'))
        # ... (convert joint properties)

    # Write URDF file
    tree = ET.ElementTree(urdf_root)
    tree.write(output_urdf, encoding='utf-8', xml_declaration=True)

# Usage
sdf_model_to_urdf('humanoid_world.sdf', 'my_humanoid', 'my_humanoid.urdf')
```

**Warning**: This conversion loses SDF-specific features (plugins, sensors, physics config). Use only when you need a URDF for ROS tools.

## Hybrid Workflow: Best of Both Worlds

**Recommended approach** for complex projects:

1. **Author robot in URDF + Xacro**:
   ```bash
   my_robot_description/
   ├── urdf/
   │   ├── my_robot.urdf.xacro  # Main robot file
   │   ├── materials.xacro       # Colors and textures
   │   ├── sensors.xacro         # Sensor definitions
   │   └── gazebo.xacro          # Gazebo-specific tags
   ```

2. **Generate URDF from Xacro**:
   ```bash
   xacro my_robot.urdf.xacro > my_robot.urdf
   ```

3. **Convert to SDF for Gazebo**:
   ```bash
   gz sdf -p my_robot.urdf > ../sdf/my_robot.sdf
   ```

4. **Enhance SDF with advanced features**:
   ```xml
   <!-- Add to generated SDF -->
   <physics type="dart">
     <max_step_size>0.0001</max_step_size>
   </physics>

   <plugin filename="my_custom_plugin" name="MyPlugin">
     <parameter>value</parameter>
   </plugin>
   ```

5. **Use URDF for ROS tools**, **SDF for Gazebo**:
   - Motion planning (MoveIt): Use URDF
   - Simulation (Gazebo): Use SDF
   - Visualization (RViz): Use URDF
   - Multi-robot scenarios: Use SDF world file

## Practical Exercise

**Challenge**: Convert your humanoid arm from Module 1 to SDF and enhance it:

1. Convert URDF to SDF using `gz sdf`
2. Add a realistic world with:
   - Ground plane
   - Directional lighting (sun)
   - Physics engine (DART with 0.001s timestep)
3. Add a camera sensor to the end effector
4. Add joint position controller plugins
5. Launch in Gazebo and verify camera feed in RViz

**Validation checklist**:
- [ ] SDF parses without errors (`gz sdf --check my_robot.sdf`)
- [ ] Robot spawns in Gazebo
- [ ] Camera publishes to `/camera` topic
- [ ] Joint controllers respond to commands
- [ ] Physics runs at ≥ 1.0x real-time

## Key Takeaways

✅ **URDF**: Robot descriptions for ROS ecosystem (MoveIt, RViz, TF)
✅ **SDF**: World descriptions for Gazebo simulation (physics, sensors, plugins)
✅ **Conversion**: URDF → SDF (supported), SDF → URDF (limited, model extraction only)
✅ **Workflow**: Author in URDF, convert to SDF, enhance with simulation features
✅ **Tool compatibility**: URDF for ROS tools, SDF for Gazebo simulation
✅ **Complexity trade-off**: URDF is simpler, SDF is more powerful

**Next steps**: Dive deeper into physics simulation parameters to understand how to tune contact forces, friction, and solver settings for stable humanoid simulation.

---

**Related chapters**:
- [URDF for Humanoids](../module1/urdf-for-humanoids.md) - Robot modeling foundation
- [Gazebo Simulation](./gazebo-simulation.md) - Using SDF in practice
- [Physics Simulation](./physics-simulation.md) - Next chapter: tuning physics engines
