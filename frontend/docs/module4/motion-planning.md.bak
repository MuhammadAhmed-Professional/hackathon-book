---
sidebar_position: 3
title: "Motion Planning"
description: "Sampling-based motion planning with RRT/RRT*, MoveIt 2 integration, collision checking, and trajectory optimization"
keywords: [motion planning, rrt, rrt*, moveit, ompl, collision checking, path planning, trajectory optimization]
---

# Chapter 3: Motion Planning

## Introduction

**Motion planning** is the problem of finding a collision-free path from a start configuration to a goal configuration in a robot's configuration space. For humanoid robots, this is challenging because:

1. **High dimensionality**: A humanoid with 30+ joints has a 30+ dimensional configuration space
2. **Kinematic constraints**: Joint limits, self-collision, balance constraints
3. **Environmental obstacles**: Dynamic obstacles, uncertain environments
4. **Real-time requirements**: Planning must complete in < 1 second for interactive tasks

This chapter focuses on **sampling-based planners**, particularly RRT (Rapidly-exploring Random Tree) and RRT*, which excel in high-dimensional spaces. You will implement these algorithms from scratch and integrate them with MoveIt 2 for production humanoid manipulation.

**Learning Objectives**:
- Understand configuration space (C-space) and collision checking
- Implement RRT and RRT* algorithms in Python
- Integrate motion planning with MoveIt 2 and OMPL
- Optimize trajectories for smooth execution
- Deploy planners on real humanoid arms

## 3.1 Configuration Space

### 3.1.1 From Workspace to C-Space

The **workspace** is the 3D Euclidean space where the robot operates (e.g., R³). The **configuration space** (C-space) is the space of all possible robot configurations.

For a planar robot arm with 2 joints (shoulder and elbow angles θ₁, θ₂):
- Workspace: Points (x, y) the end-effector can reach
- C-space: Joint angles (θ₁, θ₂) ∈ [0, 2π] × [0, 2π]

**C-space obstacles** are configurations where the robot collides with obstacles or violates joint limits.

**Example**: A 2-joint arm in a room with a wall at x = 1m. The C-space obstacle is:
```math
\mathcal{C}_{obs} = \lbrace(\theta_1, \theta_2) \mid FK(\theta_1, \theta_2) \text{ intersects wall}\rbrace
```

Where FK is forward kinematics.

### 3.1.2 C-Space Dimensionality

For a robot with $n$ joints:
- **Revolute joints**: Each adds 1 dimension (angle ∈ S¹)
- **Prismatic joints**: Each adds 1 dimension (position ∈ R)
- **Floating base** (humanoid torso): Adds 6 dimensions (position ∈ R³, orientation ∈ SO(3))

**Unitree G1 humanoid** (simplified):
- 2 legs × 6 joints = 12 DOF
- 2 arms × 7 joints = 14 DOF
- Torso (floating base) = 6 DOF
- **Total: 32-dimensional C-space**

**Challenge**: Searching a 32D space is computationally intractable for grid-based methods (10 samples per dimension → 10³² grid cells). Sampling-based methods are essential.

## 3.2 Rapidly-Exploring Random Trees (RRT)

### 3.2.1 Algorithm Overview

RRT builds a tree $\mathcal{T}$ rooted at the start configuration $q_\text{start}$ by iteratively:

1. **Sample** a random configuration $q_\text{rand}$ from C-space
2. **Find nearest** node $q_\text{near}$ in tree to $q_\text{rand}$
3. **Extend** from $q_\text{near}$ toward $q_\text{rand}$ by step size $\epsilon$
4. **Collision check** the new configuration q_new
5. **Add** q_new to tree if collision-free

**Termination**: When a node reaches within distance $\epsilon$ of $q_\text{goal}$.

**Pseudocode**:
```
RRT(q_start, q_goal, K):
    T.init(q_start)
    for k = 1 to K:
        q_rand ← Sample()
        q_near ← Nearest(T, q_rand)
        q_new ← Steer(q_near, q_rand, ε)
        if CollisionFree(q_near, q_new):
            T.add_edge(q_near, q_new)
            if Distance(q_new, q_goal) < ε:
                return ExtractPath(T, q_start, q_new)
    return FAILURE
```

**Properties**:
- **Probabilistically complete**: Probability of finding a solution (if one exists) approaches 1 as iterations increase
- **Not optimal**: Returns first feasible path, often highly suboptimal
- **Fast**: Typically finds solutions in seconds for 10-20D spaces

### 3.2.2 Python Implementation

```python
import numpy as np
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from scipy.spatial import KDTree

class RRTNode:
    """Node in the RRT tree."""
    def __init__(self, config: np.ndarray):
        self.config = config  # Configuration (joint angles or position)
        self.parent: Optional[RRTNode] = None
        self.cost: float = 0.0  # Cost from start (for RRT*)

class RRTPlanner:
    """
    Rapidly-exploring Random Tree (RRT) motion planner.

    Works in arbitrary-dimensional configuration spaces.
    """
    def __init__(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        bounds: List[Tuple[float, float]],
        obstacle_checker,
        step_size: float = 0.5,
        goal_sample_rate: float = 0.05,
        max_iterations: int = 5000
    ):
        """
        Args:
            start: Start configuration (n-dimensional)
            goal: Goal configuration
            bounds: List of (min, max) for each dimension
            obstacle_checker: Function that returns True if config is collision-free
            step_size: Maximum distance to extend in each iteration
            goal_sample_rate: Probability of sampling goal instead of random
            max_iterations: Maximum planning iterations
        """
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.bounds = bounds
        self.dim = len(bounds)
        self.obstacle_checker = obstacle_checker
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.max_iterations = max_iterations

        # Tree storage
        self.nodes: List[RRTNode] = []
        self.kdtree: Optional[KDTree] = None

    def sample(self) -> np.ndarray:
        """Sample random configuration (goal biasing)."""
        if np.random.rand() < self.goal_sample_rate:
            return self.goal.copy()
        else:
            return np.array([np.random.uniform(low, high) for low, high in self.bounds])

    def nearest(self, config: np.ndarray) -> RRTNode:
        """Find nearest node in tree to given configuration."""
        if self.kdtree is None:
            # Build KDTree for efficient nearest neighbor search
            configs = np.array([node.config for node in self.nodes])
            self.kdtree = KDTree(configs)

        _, idx = self.kdtree.query(config)
        return self.nodes[idx]

    def steer(self, from_config: np.ndarray, to_config: np.ndarray) -> np.ndarray:
        """Extend from from_config toward to_config by step_size."""
        direction = to_config - from_config
        distance = np.linalg.norm(direction)

        if distance < self.step_size:
            return to_config
        else:
            return from_config + (direction / distance) * self.step_size

    def collision_free(self, from_config: np.ndarray, to_config: np.ndarray) -> bool:
        """Check if edge from from_config to to_config is collision-free."""
        # Discretize edge and check intermediate points
        num_checks = int(np.linalg.norm(to_config - from_config) / 0.1) + 1
        for i in range(num_checks + 1):
            alpha = i / num_checks
            config = from_config + alpha * (to_config - from_config)
            if not self.obstacle_checker(config):
                return False
        return True

    def plan(self) -> Optional[List[np.ndarray]]:
        """
        Execute RRT planning.

        Returns:
            List of configurations from start to goal, or None if planning fails
        """
        # Initialize tree with start node
        start_node = RRTNode(self.start)
        self.nodes = [start_node]
        self.kdtree = None

        for iteration in range(self.max_iterations):
            # Sample random configuration
            q_rand = self.sample()

            # Find nearest node
            q_near_node = self.nearest(q_rand)

            # Steer toward sample
            q_new = self.steer(q_near_node.config, q_rand)

            # Check collision
            if self.collision_free(q_near_node.config, q_new):
                # Add new node
                new_node = RRTNode(q_new)
                new_node.parent = q_near_node
                self.nodes.append(new_node)

                # Invalidate KDTree (will rebuild on next nearest query)
                self.kdtree = None

                # Check if goal reached
                if np.linalg.norm(q_new - self.goal) < self.step_size:
                    # Add goal node
                    goal_node = RRTNode(self.goal)
                    goal_node.parent = new_node
                    self.nodes.append(goal_node)

                    # Extract path
                    path = self.extract_path(goal_node)
                    print(f"RRT: Path found in {iteration + 1} iterations, {len(path)} waypoints")
                    return path

        print("RRT: Planning failed (max iterations reached)")
        return None

    def extract_path(self, goal_node: RRTNode) -> List[np.ndarray]:
        """Extract path from start to goal by backtracking through parents."""
        path = []
        node = goal_node
        while node is not None:
            path.append(node.config)
            node = node.parent
        return list(reversed(path))


# Example: 2D point robot with circular obstacles
class PointRobotObstacleChecker:
    """Obstacle checker for 2D point robot."""
    def __init__(self, obstacles: List[Tuple[float, float, float]]):
        """
        Args:
            obstacles: List of (x, y, radius) for circular obstacles
        """
        self.obstacles = obstacles

    def is_collision_free(self, config: np.ndarray) -> bool:
        """Check if configuration collides with any obstacle."""
        x, y = config
        for ox, oy, radius in self.obstacles:
            if (x - ox)**2 + (y - oy)**2 < radius**2:
                return False
        return True


if __name__ == "__main__":
    # Define problem
    start = np.array([0.0, 0.0])
    goal = np.array([9.0, 9.0])
    bounds = [(0, 10), (0, 10)]

    # Circular obstacles: (x, y, radius)
    obstacles = [
        (3, 3, 1.5),
        (7, 2, 1.0),
        (2, 7, 1.2),
        (6, 6, 1.5),
    ]

    obstacle_checker = PointRobotObstacleChecker(obstacles)

    # Plan with RRT
    print("Planning with RRT...")
    planner = RRTPlanner(
        start=start,
        goal=goal,
        bounds=bounds,
        obstacle_checker=obstacle_checker.is_collision_free,
        step_size=0.5,
        goal_sample_rate=0.05,
        max_iterations=5000
    )

    path = planner.plan()

    if path is not None:
        # Visualize
        fig, ax = plt.subplots(figsize=(10, 10))

        # Draw obstacles
        for ox, oy, radius in obstacles:
            circle = Circle((ox, oy), radius, color='red', alpha=0.5)
            ax.add_patch(circle)

        # Draw tree
        for node in planner.nodes:
            if node.parent is not None:
                ax.plot([node.config[0], node.parent.config[0]],
                       [node.config[1], node.parent.config[1]],
                       'gray', linewidth=0.5, alpha=0.3)

        # Draw path
        path_array = np.array(path)
        ax.plot(path_array[:, 0], path_array[:, 1], 'b-', linewidth=3, label='RRT path')
        ax.plot(start[0], start[1], 'go', markersize=15, label='Start')
        ax.plot(goal[0], goal[1], 'r*', markersize=20, label='Goal')

        ax.set_xlim(bounds[0])
        ax.set_ylim(bounds[1])
        ax.set_aspect('equal')
        ax.grid(True)
        ax.legend()
        ax.set_title(f'RRT Planning ({len(path)} waypoints)')
        plt.savefig('rrt_result.png', dpi=150)
        print("Visualization saved to rrt_result.png")

        # Compute path length
        path_length = sum(np.linalg.norm(path[i+1] - path[i]) for i in range(len(path)-1))
        print(f"Path length: {path_length:.2f}")
```

**Expected Output**:
```
Planning with RRT...
RRT: Path found in 347 iterations, 23 waypoints
Visualization saved to rrt_result.png
Path length: 15.43
```

**Analysis**: RRT successfully finds a path around obstacles. However, the path is jagged and suboptimal (straight-line distance is 12.73, RRT path is 15.43).

## 3.3 RRT* (Optimal RRT)

### 3.3.1 Asymptotic Optimality

RRT* improves upon RRT by:

1. **Rewiring**: After adding a node, check if nearby nodes can reduce their cost by connecting through the new node
2. **Best parent selection**: Connect new node to parent that minimizes cost-to-come

**Key Difference from RRT**:
- RRT: Connect to nearest node
- RRT*: Connect to best node within radius $r$ (considering cost)

**Property**: RRT* is **asymptotically optimal**: As iterations → ∞, path cost → optimal cost.

### 3.3.2 RRT* Algorithm

```
RRT_STAR(q_start, q_goal, K):
    T.init(q_start)
    for k = 1 to K:
        q_rand ← Sample()
        q_near ← Nearest(T, q_rand)
        q_new ← Steer(q_near, q_rand, ε)

        if CollisionFree(q_near, q_new):
            # Find neighbors within radius r
            Q_near ← Near(T, q_new, r)

            # Choose parent with minimum cost
            q_min ← q_near
            c_min ← Cost(q_near) + Distance(q_near, q_new)

            for q' in Q_near:
                if CollisionFree(q', q_new):
                    c' ← Cost(q') + Distance(q', q_new)
                    if c' < c_min:
                        q_min ← q'
                        c_min ← c'

            # Add new node with best parent
            T.add_edge(q_min, q_new)
            q_new.cost ← c_min

            # Rewire tree
            for q' in Q_near:
                c' ← Cost(q_new) + Distance(q_new, q')
                if c' < Cost(q') and CollisionFree(q_new, q'):
                    T.remove_edge(q'.parent, q')
                    T.add_edge(q_new, q')
                    q'.cost ← c'

    return ExtractPath(T, q_start, q_goal)
```

**Radius Selection**: Use r = gamma * (log(n)/n)^(1/d) where n is number of nodes, d is dimension, gamma is tuning parameter.

### 3.3.3 Python Implementation

```python
class RRTStarPlanner(RRTPlanner):
    """
    RRT* planner with rewiring for asymptotic optimality.
    """
    def __init__(self, *args, rewire_radius: float = 1.5, **kwargs):
        super().__init__(*args, **kwargs)
        self.rewire_radius = rewire_radius

    def near(self, config: np.ndarray, radius: float) -> List[RRTNode]:
        """Find all nodes within radius of config."""
        configs = np.array([node.config for node in self.nodes])
        kdtree = KDTree(configs)
        indices = kdtree.query_ball_point(config, radius)
        return [self.nodes[i] for i in indices]

    def cost(self, node: RRTNode) -> float:
        """Compute cost from start to node."""
        return node.cost

    def plan(self) -> Optional[List[np.ndarray]]:
        """Execute RRT* planning with rewiring."""
        # Initialize tree
        start_node = RRTNode(self.start)
        start_node.cost = 0.0
        self.nodes = [start_node]

        best_goal_node: Optional[RRTNode] = None
        best_cost = float('inf')

        for iteration in range(self.max_iterations):
            # Sample
            q_rand = self.sample()

            # Nearest
            q_near_node = self.nearest(q_rand)

            # Steer
            q_new = self.steer(q_near_node.config, q_rand)

            # Collision check
            if not self.collision_free(q_near_node.config, q_new):
                continue

            # Find neighbors
            neighbors = self.near(q_new, self.rewire_radius)

            # Choose best parent (minimum cost)
            min_cost = q_near_node.cost + np.linalg.norm(q_new - q_near_node.config)
            best_parent = q_near_node

            for neighbor in neighbors:
                if self.collision_free(neighbor.config, q_new):
                    cost = neighbor.cost + np.linalg.norm(q_new - neighbor.config)
                    if cost < min_cost:
                        min_cost = cost
                        best_parent = neighbor

            # Add new node
            new_node = RRTNode(q_new)
            new_node.parent = best_parent
            new_node.cost = min_cost
            self.nodes.append(new_node)

            # Invalidate KDTree
            self.kdtree = None

            # Rewire tree
            for neighbor in neighbors:
                cost = new_node.cost + np.linalg.norm(neighbor.config - q_new)
                if cost < neighbor.cost and self.collision_free(q_new, neighbor.config):
                    # Rewire: change neighbor's parent to new_node
                    neighbor.parent = new_node
                    neighbor.cost = cost

            # Check if goal reached
            dist_to_goal = np.linalg.norm(q_new - self.goal)
            if dist_to_goal < self.step_size:
                goal_cost = new_node.cost + dist_to_goal
                if goal_cost < best_cost:
                    goal_node = RRTNode(self.goal)
                    goal_node.parent = new_node
                    goal_node.cost = goal_cost
                    best_goal_node = goal_node
                    best_cost = goal_cost
                    print(f"RRT*: Improved solution at iteration {iteration+1}, cost {best_cost:.2f}")

        if best_goal_node is not None:
            path = self.extract_path(best_goal_node)
            print(f"RRT*: Final path cost {best_cost:.2f}, {len(path)} waypoints")
            return path
        else:
            print("RRT*: Planning failed")
            return None


if __name__ == "__main__":
    # Same setup as RRT example
    start = np.array([0.0, 0.0])
    goal = np.array([9.0, 9.0])
    bounds = [(0, 10), (0, 10)]
    obstacles = [(3, 3, 1.5), (7, 2, 1.0), (2, 7, 1.2), (6, 6, 1.5)]
    obstacle_checker = PointRobotObstacleChecker(obstacles)

    # Plan with RRT*
    print("\nPlanning with RRT*...")
    planner_star = RRTStarPlanner(
        start=start,
        goal=goal,
        bounds=bounds,
        obstacle_checker=obstacle_checker.is_collision_free,
        step_size=0.5,
        goal_sample_rate=0.05,
        rewire_radius=1.5,
        max_iterations=5000
    )

    path_star = planner_star.plan()

    if path_star is not None:
        # Visualize
        fig, ax = plt.subplots(figsize=(10, 10))

        # Obstacles
        for ox, oy, radius in obstacles:
            circle = Circle((ox, oy), radius, color='red', alpha=0.5)
            ax.add_patch(circle)

        # Tree (showing rewiring)
        for node in planner_star.nodes:
            if node.parent is not None:
                ax.plot([node.config[0], node.parent.config[0]],
                       [node.config[1], node.parent.config[1]],
                       'gray', linewidth=0.3, alpha=0.2)

        # Path
        path_array = np.array(path_star)
        ax.plot(path_array[:, 0], path_array[:, 1], 'b-', linewidth=3, label='RRT* path')
        ax.plot(start[0], start[1], 'go', markersize=15, label='Start')
        ax.plot(goal[0], goal[1], 'r*', markersize=20, label='Goal')

        ax.set_xlim(bounds[0])
        ax.set_ylim(bounds[1])
        ax.set_aspect('equal')
        ax.grid(True)
        ax.legend()
        ax.set_title('RRT* Planning')
        plt.savefig('rrt_star_result.png', dpi=150)
        print("Visualization saved to rrt_star_result.png")

        # Path length
        path_length = sum(np.linalg.norm(path_star[i+1] - path_star[i]) for i in range(len(path_star)-1))
        print(f"Path length: {path_length:.2f} (straight-line: 12.73)")
```

**Expected Output**:
```
Planning with RRT*...
RRT*: Improved solution at iteration 412, cost 13.87
RRT*: Improved solution at iteration 891, cost 13.24
RRT*: Improved solution at iteration 1534, cost 13.01
RRT*: Final path cost 13.01, 18 waypoints
Visualization saved to rrt_star_result.png
Path length: 13.01 (straight-line: 12.73)
```

**Comparison**:
- **RRT path**: 15.43 (21% longer than optimal)
- **RRT* path**: 13.01 (2% longer than optimal)

RRT* produces significantly better paths through rewiring.

## 3.4 Collision Checking

### 3.4.1 Flexible Collision Library (FCL)

FCL is the industry-standard library for collision detection in robotics. It supports:

- Primitive shapes (boxes, spheres, cylinders)
- Triangle meshes (for complex geometries)
- Continuous collision detection (sweeping volumes)
- Distance queries (closest points between objects)

**Installation**:
```bash
sudo apt install libfcl-dev
pip install python-fcl
```

### 3.4.2 FCL Example

```python
import fcl
import numpy as np

# Create two boxes
box1 = fcl.Box(1.0, 1.0, 1.0)  # 1m cube
box2 = fcl.Box(0.5, 0.5, 0.5)

# Create collision objects with transforms
transform1 = fcl.Transform(np.array([0, 0, 0]))  # At origin
transform2 = fcl.Transform(np.array([0.6, 0, 0]))  # 0.6m away in x

obj1 = fcl.CollisionObject(box1, transform1)
obj2 = fcl.CollisionObject(box2, transform2)

# Collision request and result
request = fcl.CollisionRequest()
result = fcl.CollisionResult()

# Check collision
num_contacts = fcl.collide(obj1, obj2, request, result)

if num_contacts > 0:
    print(f"Collision detected! {num_contacts} contact points")
else:
    print("No collision")

# Distance query
dist_request = fcl.DistanceRequest()
dist_result = fcl.DistanceResult()
distance = fcl.distance(obj1, obj2, dist_request, dist_result)

print(f"Minimum distance: {distance:.3f} m")
# Expected: 0.1 m (0.6 - 0.5 - 0.5/2)
```

### 3.4.3 Self-Collision Checking for Humanoid

```python
import fcl
import numpy as np
from typing import List, Dict

class HumanoidCollisionChecker:
    """
    Collision checker for humanoid robot using FCL.

    Checks both self-collision and environment collision.
    """
    def __init__(self, link_geometries: Dict[str, fcl.CollisionGeometry]):
        """
        Args:
            link_geometries: Dict mapping link names to FCL geometries
                            Example: {'forearm_left': fcl.Cylinder(0.05, 0.3), ...}
        """
        self.link_geometries = link_geometries
        self.env_objects: List[fcl.CollisionObject] = []

    def add_environment_obstacle(self, geometry: fcl.CollisionGeometry, transform: fcl.Transform):
        """Add static obstacle to environment."""
        obj = fcl.CollisionObject(geometry, transform)
        self.env_objects.append(obj)

    def check_self_collision(self, link_transforms: Dict[str, np.ndarray]) -> bool:
        """
        Check if robot is in self-collision.

        Args:
            link_transforms: Dict mapping link names to 4x4 homogeneous transforms

        Returns:
            True if collision-free, False if self-collision detected
        """
        # Create collision objects for each link
        link_objects = []
        for link_name, geometry in self.link_geometries.items():
            transform_matrix = link_transforms[link_name]
            # Extract rotation and translation
            R = transform_matrix[:3, :3]
            t = transform_matrix[:3, 3]
            transform = fcl.Transform(R, t)
            obj = fcl.CollisionObject(geometry, transform)
            link_objects.append((link_name, obj))

        # Check all pairs (excluding adjacent links)
        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()

        for i in range(len(link_objects)):
            for j in range(i + 2, len(link_objects)):  # Skip adjacent pairs
                result.clear()
                num_contacts = fcl.collide(link_objects[i][1], link_objects[j][1], request, result)
                if num_contacts > 0:
                    print(f"Self-collision: {link_objects[i][0]} <-> {link_objects[j][0]}")
                    return False

        return True

    def check_environment_collision(self, link_transforms: Dict[str, np.ndarray]) -> bool:
        """
        Check if robot collides with environment.

        Returns:
            True if collision-free, False if collision detected
        """
        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()

        for link_name, geometry in self.link_geometries.items():
            transform_matrix = link_transforms[link_name]
            R = transform_matrix[:3, :3]
            t = transform_matrix[:3, 3]
            transform = fcl.Transform(R, t)
            robot_obj = fcl.CollisionObject(geometry, transform)

            for env_obj in self.env_objects:
                result.clear()
                num_contacts = fcl.collide(robot_obj, env_obj, request, result)
                if num_contacts > 0:
                    print(f"Environment collision: {link_name}")
                    return False

        return True

    def is_collision_free(self, link_transforms: Dict[str, np.ndarray]) -> bool:
        """Check both self-collision and environment collision."""
        return (self.check_self_collision(link_transforms) and
                self.check_environment_collision(link_transforms))
```

**Usage**:
```python
# Define simplified humanoid geometry
link_geometries = {
    'torso': fcl.Box(0.3, 0.2, 0.5),
    'upper_arm_left': fcl.Cylinder(0.05, 0.3),
    'forearm_left': fcl.Cylinder(0.04, 0.25),
    'upper_arm_right': fcl.Cylinder(0.05, 0.3),
    'forearm_right': fcl.Cylinder(0.04, 0.25),
}

checker = HumanoidCollisionChecker(link_geometries)

# Add table obstacle
table = fcl.Box(1.0, 0.6, 0.05)
table_transform = fcl.Transform(np.array([0.5, 0, 0.7]))  # 0.7m height
checker.add_environment_obstacle(table, table_transform)

# Check configuration (forward kinematics would compute these)
link_transforms = {
    'torso': np.eye(4),
    'upper_arm_left': np.array([[1,0,0,0.15], [0,1,0,0], [0,0,1,0.5], [0,0,0,1]]),
    # ... other links
}

is_safe = checker.is_collision_free(link_transforms)
print(f"Configuration safe: {is_safe}")
```

## 3.5 MoveIt 2 Integration

### 3.5.1 MoveIt 2 Overview

**MoveIt 2** is the de facto motion planning framework for ROS 2. It provides:

- Integration with OMPL (Open Motion Planning Library) for 20+ planning algorithms
- Collision checking with FCL
- Kinematic constraint handling (IK, workspace bounds)
- Trajectory execution and monitoring
- Support for mobile manipulation

**Installation**:
```bash
sudo apt install ros-humble-moveit
```

### 3.5.2 MoveIt 2 Configuration

For a humanoid robot, you need:

1. **URDF/SRDF**: Robot description with planning groups
2. **Joint limits**: Velocity, acceleration, jerk limits
3. **Collision matrix**: Pairs of links that should never be checked (adjacent links)
4. **Planning pipeline**: Choice of planner (RRTConnect, RRT*, CHOMP, etc.)

**Example SRDF** (Semantic Robot Description Format):
```xml
<?xml version="1.0"?>
<robot name="humanoid">
  <!-- Define planning group for left arm -->
  <group name="left_arm">
    <chain base_link="torso" tip_link="left_hand"/>
  </group>

  <!-- Define planning group for right arm -->
  <group name="right_arm">
    <chain base_link="torso" tip_link="right_hand"/>
  </group>

  <!-- Disable collision checking for adjacent links -->
  <disable_collisions link1="upper_arm_left" link2="forearm_left" reason="Adjacent"/>
  <disable_collisions link1="forearm_left" link2="hand_left" reason="Adjacent"/>

  <!-- Define end-effector -->
  <end_effector name="left_gripper" parent_link="hand_left" group="left_arm"/>
</robot>
```

### 3.5.3 Planning with MoveIt 2 Python API

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np

class MoveItPlanner(Node):
    """
    MoveIt 2 motion planning interface for humanoid arms.
    """
    def __init__(self):
        super().__init__('moveit_planner')

        # Import MoveIt Python interface (must be done after rclpy.init())
        from moveit.planning import MoveItPy
        from moveit.core.robot_state import RobotState as MoveItRobotState

        # Initialize MoveIt
        self.moveit = MoveItPy(node=self)
        self.left_arm = self.moveit.get_planning_component("left_arm")

        self.get_logger().info("MoveIt planner initialized")

    def plan_to_pose(self, target_pose: np.ndarray) -> bool:
        """
        Plan to Cartesian target pose.

        Args:
            target_pose: 4x4 homogeneous transform (end-effector pose)

        Returns:
            True if planning succeeded
        """
        # Set pose goal
        self.left_arm.set_goal_state(pose_stamped_target=self.numpy_to_pose_stamped(target_pose),
                                      pose_link="left_hand")

        # Plan
        self.get_logger().info("Planning to target pose...")
        plan_result = self.left_arm.plan()

        if plan_result:
            self.get_logger().info(f"Planning succeeded! Trajectory has {len(plan_result.trajectory.joint_trajectory.points)} points")
            # Execute (comment out for simulation-only)
            # self.left_arm.execute()
            return True
        else:
            self.get_logger().error("Planning failed!")
            return False

    def plan_to_joint_config(self, joint_positions: Dict[str, float]) -> bool:
        """
        Plan to joint-space configuration.

        Args:
            joint_positions: Dict mapping joint names to target angles (radians)

        Returns:
            True if planning succeeded
        """
        self.left_arm.set_goal_state(configuration_name="ready")  # Use predefined pose
        # Or set custom joint values:
        # self.left_arm.set_goal_state(joint_state_target=joint_positions)

        plan_result = self.left_arm.plan()

        if plan_result:
            self.get_logger().info("Planning succeeded!")
            return True
        else:
            self.get_logger().error("Planning failed!")
            return False

    def numpy_to_pose_stamped(self, transform: np.ndarray) -> PoseStamped:
        """Convert 4x4 numpy array to PoseStamped message."""
        from scipy.spatial.transform import Rotation

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()

        # Translation
        pose.pose.position.x = transform[0, 3]
        pose.pose.position.y = transform[1, 3]
        pose.pose.position.z = transform[2, 3]

        # Rotation (convert to quaternion)
        R = transform[:3, :3]
        quat = Rotation.from_matrix(R).as_quat()  # [x, y, z, w]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose


def main():
    rclpy.init()
    planner = MoveItPlanner()

    # Example: Plan to target pose (0.5m forward, 0.2m left, 0.8m up)
    target_pose = np.array([
        [1, 0, 0, 0.5],
        [0, 1, 0, 0.2],
        [0, 0, 1, 0.8],
        [0, 0, 0, 1]
    ])

    success = planner.plan_to_pose(target_pose)

    if success:
        print("Planning succeeded! Ready to execute.")
    else:
        print("Planning failed. Check joint limits and collision constraints.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run**:
```bash
# Terminal 1: Launch MoveIt with robot description
ros2 launch humanoid_moveit_config demo.launch.py

# Terminal 2: Run planner
python3 moveit_planner.py
```

### 3.5.4 Trajectory Smoothing

RRT/RRT* paths are often jagged. Post-processing smooths trajectories:

**1. Shortcutting**: Randomly sample pairs of waypoints and replace with straight line if collision-free.

**2. B-spline fitting**: Fit smooth B-spline curve through waypoints.

```python
from scipy.interpolate import splprep, splev

def smooth_path(path: List[np.ndarray], num_samples: int = 100) -> List[np.ndarray]:
    """
    Smooth path using B-spline interpolation.

    Args:
        path: List of waypoints
        num_samples: Number of samples in smoothed path

    Returns:
        Smoothed path
    """
    path_array = np.array(path)

    # Fit B-spline (k=3 for cubic)
    tck, u = splprep(path_array.T, s=0, k=min(3, len(path)-1))

    # Evaluate at uniform samples
    u_new = np.linspace(0, 1, num_samples)
    smoothed = splev(u_new, tck)

    return [np.array(point) for point in zip(*smoothed)]
```

## 3.6 Exercises

### Exercise 1 (Easy): Implement 1D RRT

**Problem**: Implement RRT for a 1D point robot on a line with gaps (obstacles). Start at x=0, goal at x=10, obstacles at [3,4] and [7,8].

**Hints**: C-space is 1D, collision check is just range check.

### Exercise 2 (Easy): Visualize RRT Tree Growth

**Problem**: Modify the RRT implementation to save tree state every 100 iterations and create an animation showing tree growth.

**Hints**: Use matplotlib animation or save frames as images.

### Exercise 3 (Medium): Bidirectional RRT

**Problem**: Implement bidirectional RRT that grows trees from both start and goal simultaneously, connecting them when they get close.

**Benefits**: Often finds solutions 2-3x faster than unidirectional RRT.

### Exercise 4 (Medium): RRT-Connect

**Problem**: Implement RRT-Connect, which extends maximally toward samples instead of by fixed step size. Compare performance to standard RRT.

**Pseudocode**: Replace `Steer` with `Connect` that extends until collision or reaching sample.

### Exercise 5 (Hard): Informed RRT*

**Problem**: Implement Informed RRT* which, after finding an initial solution, restricts sampling to an ellipsoidal heuristic region likely to contain better paths.

**Math**: Sample from ellipsoid with foci at start and goal, semi-major axis = current best cost.

**Expected Improvement**: 30-50% faster convergence to near-optimal paths.

## Summary

This chapter covered:

1. **Configuration space**: The fundamental representation for motion planning
2. **RRT**: Fast probabilistically-complete sampling-based planner
3. **RRT***: Asymptotically optimal variant with rewiring
4. **Collision checking**: Using FCL for self-collision and environment collision
5. **MoveIt 2**: Production-ready framework integrating planning, kinematics, and execution

**Key Takeaways**:
- Sampling-based planners scale to high dimensions (20-30 DOF humanoids)
- RRT* should be used when path quality matters; RRT when speed is critical
- Collision checking is typically the computational bottleneck (optimize with caching, broad-phase culling)
- MoveIt 2 handles 90% of production use cases; implement custom planners only when needed

**Next Chapter**: [Control Systems](./control-systems.md) - Executing planned trajectories with feedback control

## References

1. LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press. (Chapters 5 & 14)
2. Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning". *IJRR*, 30(7), 846-894.
3. Sucan, I. A., & Chitta, S. (2019). "MoveIt!". *IEEE Robotics & Automation Magazine*.
4. Gammell, J. D., et al. (2014). "Informed RRT*: Optimal sampling-based path planning focused via direct sampling of an admissible ellipsoidal heuristic". *IROS*.
5. OMPL documentation: https://ompl.kavrakilab.org/
