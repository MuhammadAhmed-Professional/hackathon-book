---
id: physics-simulation
title: "Physics Simulation Deep Dive"
sidebar_position: 3
description: "Master rigid body dynamics, collision detection, contact forces, and physics tuning for humanoid robots"
---

# Physics Simulation Deep Dive

Physics simulation is the heart of any robotics simulator. It transforms static 3D models into dynamic systems that respond to gravity, collisions, and actuator forces. For humanoid robots—with their high degree-of-freedom articulated bodies, complex contact dynamics, and balance requirements—understanding physics simulation is not optional; it's essential.

In this chapter, you'll learn the fundamentals of rigid body dynamics, explore collision detection algorithms, master contact force modeling, and discover how to tune physics parameters for stable, real-time simulation of humanoid robots.

## Rigid Body Dynamics: The Foundation

Every link in your robot is a **rigid body**—an object whose shape doesn't deform under forces. Rigid body dynamics governs how these bodies move and interact.

### Newton-Euler Equations

The motion of each rigid body is determined by:

**Translation** (linear motion):
```
F = m * a
```
Where:
- `F`: Total force vector (N)
- `m`: Mass (kg)
- `a`: Linear acceleration (m/s²)

**Rotation** (angular motion):
```
τ = I * α + ω × (I * ω)
```
Where:
- `τ`: Total torque vector (Nm)
- `I`: Inertia tensor (3x3 matrix, kg·m²)
- `α`: Angular acceleration (rad/s²)
- `ω`: Angular velocity (rad/s)
- `×`: Cross product

**The second term** `ω × (I * ω)` represents gyroscopic effects—crucial for spinning objects like flywheels or rotating joints.

### State Vector

At each timestep, the simulator tracks:
```python
state = {
    'position': [x, y, z],           # m
    'orientation': [qw, qx, qy, qz], # quaternion (avoids gimbal lock)
    'linear_velocity': [vx, vy, vz], # m/s
    'angular_velocity': [wx, wy, wz] # rad/s
}
```

**Why quaternions?** Euler angles (roll, pitch, yaw) suffer from gimbal lock at ±90° pitch. Quaternions avoid this and enable smooth interpolation.

### Integration: From Forces to Motion

Physics engines use **numerical integration** to update the state over time:

**Explicit Euler** (simplest, least stable):
```python
def step(state, forces, dt):
    """One timestep of explicit Euler integration."""
    acceleration = forces / mass
    state.velocity += acceleration * dt
    state.position += state.velocity * dt
    return state
```

**Semi-Implicit Euler** (better stability):
```python
def step(state, forces, dt):
    """Semi-implicit Euler (update velocity first)."""
    state.velocity += (forces / mass) * dt  # Update velocity first
    state.position += state.velocity * dt    # Then update position
    return state
```

**Runge-Kutta 4th Order (RK4)** (high accuracy, computationally expensive):
```python
def step(state, forces, dt):
    """4th-order Runge-Kutta integration."""
    k1 = dt * derivative(state, forces)
    k2 = dt * derivative(state + 0.5*k1, forces)
    k3 = dt * derivative(state + 0.5*k2, forces)
    k4 = dt * derivative(state + k3, forces)

    state += (k1 + 2*k2 + 2*k3 + k4) / 6
    return state
```

**Gazebo default**: Semi-implicit Euler (good balance of speed and stability)

**For humanoid robots**: Use small timesteps (0.0001-0.001s) with semi-implicit Euler. RK4 is overkill for most scenarios.

## Collision Detection: Finding Contacts

Before computing contact forces, the simulator must detect which objects are colliding.

### Broad Phase: Spatial Partitioning

**Goal**: Quickly eliminate pairs of objects that are far apart.

**Axis-Aligned Bounding Box (AABB)**:
```python
class AABB:
    def __init__(self, min_point, max_point):
        self.min = min_point  # [x_min, y_min, z_min]
        self.max = max_point  # [x_max, y_max, z_max]

    def intersects(self, other):
        """Check if two AABBs overlap."""
        return (self.min[0] <= other.max[0] and self.max[0] >= other.min[0] and
                self.min[1] <= other.max[1] and self.max[1] >= other.min[1] and
                self.min[2] <= other.max[2] and self.max[2] >= other.min[2])
```

**Algorithms**:
- **Sweep and Prune**: Sort AABBs along one axis, detect overlaps
- **Spatial Hashing**: Divide space into grid cells, check neighbors
- **Bounding Volume Hierarchy (BVH)**: Tree structure for fast queries

**Gazebo uses**: BVH for complex meshes, spatial hashing for simple scenes

### Narrow Phase: Precise Contact Points

Once a potential collision is found, compute exact contact geometry:

**Primitive Shapes** (sphere, box, cylinder):
```python
def sphere_sphere_collision(sphere1, sphere2):
    """Detect collision between two spheres."""
    distance = np.linalg.norm(sphere1.position - sphere2.position)
    combined_radius = sphere1.radius + sphere2.radius

    if distance < combined_radius:
        penetration_depth = combined_radius - distance
        contact_normal = (sphere2.position - sphere1.position) / distance
        contact_point = sphere1.position + contact_normal * sphere1.radius

        return {
            'penetration': penetration_depth,
            'normal': contact_normal,
            'point': contact_point
        }
    return None
```

**Convex Meshes** (arbitrary shapes):
- **GJK (Gilbert-Johnson-Keerthi)**: Iterative algorithm to find closest points
- **EPA (Expanding Polytope Algorithm)**: Extends GJK to compute penetration depth

**Concave Meshes** (e.g., terrain):
- **Decomposition**: Break into convex pieces
- **Heightmap**: Special case for terrains (fast ray-mesh intersection)

**Performance tip**: Use primitive shapes (spheres, cylinders) for collision geometry, even if visual geometry is a complex mesh. A humanoid foot can be approximated as 3-4 spheres for fast contact detection.

## Contact Forces: Making Objects Interact

When two objects collide, the simulator must compute contact forces to prevent interpenetration.

### Normal Force: Preventing Penetration

The normal force pushes objects apart:

```
F_normal = k_p * penetration_depth + k_d * relative_velocity_normal
```

Where:
- `k_p`: Spring stiffness (N/m)
- `k_d`: Damping coefficient (N·s/m)

**In Gazebo SDF**:
```xml
<contact>
  <ode>
    <kp>1000000.0</kp>  <!-- Spring stiffness -->
    <kd>100.0</kd>      <!-- Damping -->
    <max_vel>0.01</max_vel>  <!-- Max penetration velocity -->
    <min_depth>0.001</min_depth>  <!-- Minimum penetration to register -->
  </ode>
</contact>
```

**Tuning guidelines**:
- **High kp** (10⁶-10⁸): Stiff contacts (metal on metal)
- **Medium kp** (10⁴-10⁶): Soft contacts (rubber, foam)
- **kd ≈ 0.1 * kp**: Critical damping for stable contacts

### Friction: Tangential Forces

Friction prevents sliding:

**Coulomb Friction Model**:
```
F_friction ≤ μ * F_normal
```

Where:
- `μ`: Coefficient of friction (dimensionless)
- `F_friction`: Tangential force parallel to contact surface

**Typical friction coefficients**:
- Steel on steel: μ = 0.15-0.25
- Rubber on concrete: μ = 0.7-1.0
- Teflon on Teflon: μ = 0.04
- Ice on ice: μ = 0.02-0.05

**In Gazebo**:
```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>   <!-- Friction along primary direction -->
      <mu2>0.8</mu2> <!-- Friction along secondary direction (anisotropic) -->
      <fdir1>1 0 0</fdir1>  <!-- Primary friction direction -->
    </ode>
  </friction>
</surface>
```

**For humanoid feet**: Use μ = 0.8-1.0 to prevent slipping during walking.

### Restitution: Bounciness

Restitution models energy loss in collisions:

```
coefficient_of_restitution = relative_velocity_after / relative_velocity_before
```

- `e = 0.0`: Perfectly inelastic (no bounce)
- `e = 1.0`: Perfectly elastic (full bounce)
- `e = 0.3`: Typical for metal on metal
- `e = 0.9`: Bouncy ball

**In Gazebo**:
```xml
<surface>
  <bounce>
    <restitution_coefficient>0.3</restitution_coefficient>
    <threshold>0.01</threshold>  <!-- Minimum velocity to bounce -->
  </bounce>
</surface>
```

**For humanoids**: Use e = 0.1-0.3 for stable foot contacts (low bounce).

## Constraint Solvers: Joint Dynamics

Joints (revolute, prismatic, fixed) are implemented as **constraints** that limit relative motion between links.

### Lagrange Multipliers

Joints are enforced using constraint forces:

```
C(q) = 0  (constraint equation, e.g., "distance between joint origins = 0")
```

The solver finds forces `λ` that satisfy:
```
M * a + J^T * λ = F_external
J * a = -Ċ  (time derivative of constraint)
```

Where:
- `M`: Mass matrix
- `J`: Constraint Jacobian matrix
- `λ`: Constraint forces (Lagrange multipliers)

### Solver Types

**Sequential Impulse (SI)** (used in Bullet):
```python
def sequential_impulse_solver(contacts, iterations):
    """Gauss-Seidel style iterative solver."""
    for _ in range(iterations):
        for contact in contacts:
            # Compute impulse to satisfy contact constraint
            impulse = compute_impulse(contact)
            apply_impulse(contact.body1, contact.point1, impulse)
            apply_impulse(contact.body2, contact.point2, -impulse)
```

**Pros**: Fast, handles large contact counts
**Cons**: Requires many iterations for stiff systems

**Projected Gauss-Seidel (PGS)** (used in ODE):
- Similar to SI but with projection step to handle inequality constraints (e.g., friction cone)

**Dantzig LCP Solver** (used in DART):
- Solves Linear Complementarity Problem (LCP) exactly
- More accurate for stiff contacts (humanoid foot on ground)
- Slower than iterative methods

**In Gazebo**:
```xml
<physics type="dart">
  <dart>
    <solver>
      <solver_type>dantzig</solver_type>  <!-- or 'pgs' -->
    </solver>
  </dart>
</physics>
```

**Recommendation for humanoids**: Use DART with Dantzig solver for accurate contact forces during walking.

## Timestep and Real-Time Factor

### Choosing Timestep

**Timestep trade-off**:
- **Small timestep** (0.0001s = 10 kHz): Accurate, stable, slow
- **Large timestep** (0.01s = 100 Hz): Fast, unstable, inaccurate

**Rule of thumb**: Timestep < 1/10 of fastest oscillation in system.

For a humanoid with:
- Joint natural frequency: ~100 Hz (typical for motors)
- Contact stiffness: 1 MHz → natural frequency ~1 kHz

**Recommended timestep**: 0.001s (1 kHz) for balance, 0.0001s (10 kHz) for high-fidelity contact.

**In Gazebo**:
```xml
<physics type="dart">
  <max_step_size>0.001</max_step_size>  <!-- 1ms timestep -->
  <real_time_update_rate>1000</real_time_update_rate>  <!-- 1000 Hz -->
  <real_time_factor>1.0</real_time_factor>  <!-- Target 1x real-time -->
</physics>
```

### Real-Time Factor (RTF)

```
RTF = simulation_time_elapsed / wall_clock_time_elapsed
```

- **RTF = 1.0**: Real-time (1 second simulated = 1 second wall clock)
- **RTF = 0.5**: Half speed (common for complex robots)
- **RTF = 2.0**: Faster than real-time (reinforcement learning training)

**Monitor RTF**:
```bash
gz topic -e -t /stats
# Look for <real_time_factor> value
```

**Optimization tips**:
1. Simplify collision geometry (spheres > cylinders > meshes)
2. Reduce joint count (merge fixed joints)
3. Increase timestep (if stable)
4. Disable GUI rendering (`gz sim -s` for headless)
5. Use multi-threading (DART only)

## Tuning Physics for Stable Humanoid Simulation

### Problem 1: Jittering Feet

**Symptom**: Humanoid feet vibrate rapidly when standing.

**Cause**: Contact stiffness too high, timestep too large.

**Solution**:
```xml
<contact>
  <ode>
    <kp>500000.0</kp>  <!-- Reduce from 1e6 -->
    <kd>50.0</kd>      <!-- Increase damping -->
    <max_vel>0.001</max_vel>  <!-- Limit penetration velocity -->
  </ode>
</contact>

<physics>
  <max_step_size>0.0005</max_step_size>  <!-- Reduce timestep -->
</physics>
```

### Problem 2: Exploding Robot

**Symptom**: Links fly apart explosively at simulation start.

**Cause**: Joint limits violated, constraint solver fails.

**Solution**:
```xml
<!-- Increase solver iterations -->
<physics type="ode">
  <ode>
    <solver>
      <iters>100</iters>  <!-- Increase from 50 -->
    </solver>
  </ode>
</physics>

<!-- Add joint damping -->
<joint name="shoulder_pitch">
  <dynamics>
    <damping>5.0</damping>  <!-- Nm/(rad/s) -->
  </dynamics>
</joint>
```

### Problem 3: Foot Slip During Walking

**Symptom**: Feet slide backward during stance phase.

**Cause**: Friction coefficient too low.

**Solution**:
```xml
<surface>
  <friction>
    <ode>
      <mu>1.2</mu>   <!-- Increase friction -->
      <mu2>1.0</mu2>
      <slip1>0.0</slip1>  <!-- Zero slip (perfect friction) -->
      <slip2>0.0</slip2>
    </ode>
  </friction>
</surface>
```

### Problem 4: Slow Simulation (RTF < 0.5)

**Symptom**: Simulation can't keep up with real-time.

**Solution**:
```python
# 1. Profile physics
gz topic -e -t /stats
# Check <sim_time> vs <real_time> ratio

# 2. Simplify collision geometry
<collision name="foot_collision">
  <geometry>
    <sphere radius="0.05"/>  <!-- Use sphere instead of mesh -->
  </geometry>
</collision>

# 3. Increase timestep (if stable)
<max_step_size>0.002</max_step_size>  <!-- 2ms instead of 1ms -->

# 4. Reduce rendering quality
gz sim --render-engine ogre2 --render-quality low
```

## Practical Exercise

**Challenge**: Create a humanoid balance test:

1. Spawn your humanoid standing on one leg
2. Apply lateral forces (10-50 N) to the torso
3. Tune physics to achieve:
   - No foot slip (friction)
   - Stable standing (damping)
   - RTF ≥ 0.8 (performance)
4. Log joint torques and contact forces

**Success criteria**:
- [ ] Robot stands for ≥ 5 seconds without falling
- [ ] Foot remains stationary (< 1 cm drift)
- [ ] Simulation runs at ≥ 0.8x real-time
- [ ] No jittering or instability

**Starter code**:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench

class ForceApplier(Node):
    def __init__(self):
        super().__init__('force_applier')
        self.publisher = self.create_publisher(Wrench, '/apply_force', 10)
        self.timer = self.create_timer(2.0, self.apply_lateral_force)

    def apply_lateral_force(self):
        wrench = Wrench()
        wrench.force.y = 30.0  # 30 N lateral force
        self.publisher.publish(wrench)
        self.get_logger().info('Applied 30 N lateral force')

def main(args=None):
    rclpy.init(args=args)
    node = ForceApplier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Key Takeaways

✅ **Rigid body dynamics**: Newton-Euler equations govern motion
✅ **Integration methods**: Semi-implicit Euler for speed, RK4 for accuracy
✅ **Collision detection**: Broad phase (AABBs) + narrow phase (GJK/EPA)
✅ **Contact forces**: Normal (kp, kd), friction (μ), restitution (e)
✅ **Constraint solvers**: PGS (fast), Dantzig (accurate)
✅ **Timestep rule**: < 1/10 of fastest oscillation
✅ **Tuning for humanoids**: High friction (μ > 0.8), low restitution (e < 0.3), small timestep (1ms)

**Next steps**: Explore Unity's rendering engine for photorealistic visualization of humanoid robots in human environments, complementing Gazebo's physics-focused simulation.

---

**Related chapters**:
- [Gazebo Simulation](./gazebo-simulation.md) - Applying physics in Gazebo
- [URDF vs SDF](./urdf-vs-sdf.md) - Configuring physics in SDF
- [Unity Rendering](./unity-rendering.md) - Next chapter: high-fidelity visualization
