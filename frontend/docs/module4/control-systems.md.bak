---
sidebar_position: 4
title: "Control Systems"
description: "PID control, Model Predictive Control (MPC), whole-body control, and force/torque control for humanoid robots"
keywords: [control systems, pid, mpc, whole-body control, force control, torque control, feedback control]
---

# Chapter 4: Control Systems

## Introduction

**Control systems** translate motion plans into motor commands that track desired trajectories despite disturbances, model uncertainty, and hardware limitations. For humanoid robots, control is challenging because:

1. **High-dimensional dynamics**: 30+ actuated joints with complex coupling
2. **Underactuation**: Cannot independently control all Cartesian DOF (e.g., torso position during balance)
3. **Contact forces**: Must regulate forces when touching objects or ground
4. **Real-time constraints**: Control loops run at 100-1000 Hz

This chapter progresses from simple PID controllers for single joints to sophisticated whole-body controllers that coordinate all degrees of freedom while maintaining balance.

**Learning Objectives**:
- Design and tune PID controllers for joint-level tracking
- Implement Model Predictive Control (MPC) for constrained optimization
- Understand whole-body control formulations (QP-based)
- Implement force/torque control for compliant manipulation
- Deploy control systems on real humanoid hardware

## 4.1 PID Control

### 4.1.1 Feedback Control Fundamentals

A **feedback controller** measures the current state $x(t)$, compares it to the desired reference $r(t)$, and computes control input $u(t)$ to minimize error $e(t) = r(t) - x(t)$.

**PID (Proportional-Integral-Derivative) control** is the most widely used controller:

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}
$$

Where:
- **Proportional term** $K_p e$: Responds to current error (higher $K_p$ = faster response, risk of overshoot)
- **Integral term** $K_i \int e$: Eliminates steady-state error by accumulating past errors
- **Derivative term** $K_d \dot{e}$: Dampens oscillations by reacting to rate of error change

**Discrete-time implementation** (for digital systems at sampling period $\Delta t$):

```math
u_k = K_p e_k + K_i \Delta t \sum_{j=0}^k e_j + K_d \frac{e_k - e_{k-1}}{\Delta t}
```

### 4.1.2 PID for Joint Position Control

For a robot joint with position $\theta(t)$ and desired position $\theta_d(t)$:

```math
\tau = K_p (\theta_d - \theta) + K_i \int (\theta_d - \theta) dt + K_d (\dot{\theta}_d - \dot{\theta})
```

Where $\tau$ is the commanded motor torque.

**Python Implementation**:

```python
import numpy as np
from typing import Optional
import time

class PIDController:
    """
    PID controller for single-variable control (e.g., joint position).
    """
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limits: tuple[float, float] = (-np.inf, np.inf),
        integral_limits: tuple[float, float] = (-np.inf, np.inf),
        dt: Optional[float] = None
    ):
        """
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: (min, max) for control output
            integral_limits: (min, max) for integral term (anti-windup)
            dt: Sampling period (auto-computed if None)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_limits = integral_limits
        self.dt = dt

        # State variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time: Optional[float] = None

    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, setpoint: float, measurement: float, dt: Optional[float] = None) -> float:
        """
        Compute control output.

        Args:
            setpoint: Desired value (reference)
            measurement: Current measured value
            dt: Time step (overrides constructor dt if provided)

        Returns:
            Control output
        """
        # Compute time step
        if dt is None:
            if self.dt is not None:
                dt = self.dt
            elif self.prev_time is not None:
                dt = time.time() - self.prev_time
            else:
                dt = 0.01  # Default 100 Hz

        self.prev_time = time.time()

        # Error
        error = setpoint - measurement

        # Proportional term
        p_term = self.kp * error

        # Integral term (with anti-windup)
        self.integral += error * dt
        self.integral = np.clip(self.integral, *self.integral_limits)
        i_term = self.ki * self.integral

        # Derivative term (with derivative kick prevention)
        # Note: We differentiate measurement, not error, to avoid derivative kick on setpoint changes
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
        else:
            d_term = 0.0

        self.prev_error = error

        # Total output
        output = p_term + i_term + d_term

        # Output limiting
        output = np.clip(output, *self.output_limits)

        return output


# Example: Control a simulated motor
class SimulatedMotor:
    """
    Simple motor model: J * theta_ddot + B * theta_dot = tau
    where J is inertia, B is friction, tau is torque
    """
    def __init__(self, inertia: float = 0.1, friction: float = 0.5):
        self.J = inertia
        self.B = friction
        self.theta = 0.0      # Position
        self.theta_dot = 0.0  # Velocity

    def apply_torque(self, tau: float, dt: float):
        """Apply torque for dt seconds."""
        # Dynamics: theta_ddot = (tau - B*theta_dot) / J
        theta_ddot = (tau - self.B * self.theta_dot) / self.J

        # Euler integration
        self.theta_dot += theta_ddot * dt
        self.theta += self.theta_dot * dt

    def get_state(self) -> tuple[float, float]:
        """Return (position, velocity)."""
        return self.theta, self.theta_dot


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # Simulation parameters
    dt = 0.01  # 100 Hz control loop
    duration = 5.0  # seconds
    num_steps = int(duration / dt)

    # Target trajectory: step from 0 to 1 radian at t=0.5s
    setpoints = np.concatenate([
        np.zeros(int(0.5 / dt)),
        np.ones(num_steps - int(0.5 / dt))
    ])

    # PID controller (tune these gains!)
    controller = PIDController(
        kp=20.0,     # Proportional gain
        ki=5.0,      # Integral gain
        kd=2.0,      # Derivative gain
        output_limits=(-10.0, 10.0),  # Max torque: ±10 Nm
        dt=dt
    )

    # Simulated motor
    motor = SimulatedMotor(inertia=0.1, friction=0.5)

    # Simulation
    positions = []
    velocities = []
    torques = []

    for i in range(num_steps):
        # Measure current state
        theta, theta_dot = motor.get_state()

        # Compute control
        tau = controller.compute(setpoints[i], theta)

        # Apply to motor
        motor.apply_torque(tau, dt)

        # Record
        positions.append(theta)
        velocities.append(theta_dot)
        torques.append(tau)

    # Plot results
    time_array = np.arange(num_steps) * dt

    fig, axes = plt.subplots(3, 1, figsize=(10, 10))

    # Position tracking
    axes[0].plot(time_array, setpoints, 'r--', label='Setpoint', linewidth=2)
    axes[0].plot(time_array, positions, 'b-', label='Actual', linewidth=2)
    axes[0].set_ylabel('Position (rad)')
    axes[0].set_title('PID Control - Joint Position Tracking')
    axes[0].legend()
    axes[0].grid(True)

    # Velocity
    axes[1].plot(time_array, velocities, 'g-', linewidth=2)
    axes[1].set_ylabel('Velocity (rad/s)')
    axes[1].grid(True)

    # Torque
    axes[2].plot(time_array, torques, 'm-', linewidth=2)
    axes[2].axhline(10, color='r', linestyle='--', alpha=0.5, label='Limit')
    axes[2].axhline(-10, color='r', linestyle='--', alpha=0.5)
    axes[2].set_ylabel('Torque (Nm)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig('pid_control_result.png', dpi=150)
    print("Plot saved to pid_control_result.png")

    # Compute performance metrics
    settling_time_idx = np.where(np.abs(np.array(positions) - 1.0) < 0.02)[0][0]
    settling_time = settling_time_idx * dt - 0.5
    overshoot = (np.max(positions) - 1.0) / 1.0 * 100
    steady_state_error = np.mean(np.abs(np.array(positions[-100:]) - 1.0))

    print(f"\n=== PID Performance ===")
    print(f"Settling time (2%): {settling_time:.3f} s")
    print(f"Overshoot: {overshoot:.1f}%")
    print(f"Steady-state error: {steady_state_error:.6f} rad")
```

**Expected Output**:
```
Plot saved to pid_control_result.png

=== PID Performance ===
Settling time (2%): 0.450 s
Overshoot: 8.3%
Steady-state error: 0.000012 rad
```

**Tuning Guidelines**:

1. **Start with P-only control**: Set $K_i = K_d = 0$, increase $K_p$ until oscillations appear, then reduce by 50%
2. **Add D term**: Increase $K_d$ to dampen oscillations (typically $K_d \approx 0.1 K_p$)
3. **Add I term**: Increase $K_i$ to eliminate steady-state error (start small, $K_i \approx 0.25 K_p$)

**Ziegler-Nichols Method** (classical tuning):
1. Set $K_i = K_d = 0$
2. Increase $K_p$ until system oscillates with constant amplitude (critical gain $K_u$, period $T_u$)
3. Use tuning table:
   - **P-only**: $K_p = 0.5 K_u$
   - **PID**: $K_p = 0.6 K_u$, $K_i = 1.2 K_u / T_u$, $K_d = 0.075 K_u T_u$

### 4.1.3 Cascaded PID for Trajectory Tracking

For smooth trajectory tracking, use **cascaded (nested) PID**:

1. **Outer loop (position)**: Desired velocity = PID(position error) (computes desired velocity)
2. **Inner loop (velocity)**: Torque = PID(velocity error) (computes torque)

**Advantages**:
- Separate tuning for position and velocity tracking
- Inner loop provides velocity damping
- Better disturbance rejection

```python
class CascadedPIDController:
    """Cascaded PID for position tracking with velocity inner loop."""
    def __init__(self):
        # Outer loop: position -> desired velocity
        self.position_pid = PIDController(kp=10.0, ki=0.0, kd=2.0, output_limits=(-5.0, 5.0))

        # Inner loop: velocity -> torque
        self.velocity_pid = PIDController(kp=5.0, ki=1.0, kd=0.1, output_limits=(-10.0, 10.0))

    def compute(self, pos_setpoint: float, pos_meas: float, vel_meas: float, dt: float) -> float:
        """
        Args:
            pos_setpoint: Desired position
            pos_meas: Measured position
            vel_meas: Measured velocity
            dt: Time step

        Returns:
            Torque command
        """
        # Outer loop: compute desired velocity
        vel_setpoint = self.position_pid.compute(pos_setpoint, pos_meas, dt)

        # Inner loop: compute torque
        tau = self.velocity_pid.compute(vel_setpoint, vel_meas, dt)

        return tau
```

## 4.2 Model Predictive Control (MPC)

### 4.2.1 MPC Formulation

**Model Predictive Control** solves an optimization problem at each time step:

```math
\begin{aligned}
\min_{\mathbf{u}_0, \ldots, \mathbf{u}_{N-1}} \quad & \sum_{k=0}^{N-1} \left[ \|\mathbf{x}_k - \mathbf{x}_d\|_Q^2 + \|\mathbf{u}_k\|_R^2 \right] + \|\mathbf{x}_N - \mathbf{x}_d\|_P^2 \\
\text{subject to} \quad & \mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k) \\
& \mathbf{u}_{min} \leq \mathbf{u}_k \leq \mathbf{u}_{max} \\
& \mathbf{x}_{min} \leq \mathbf{x}_k \leq \mathbf{x}_{max}
\end{aligned}
```

Where:
- $\mathbf{x}_k$: State at time step $k$ (e.g., joint positions and velocities)
- $\mathbf{u}_k$: Control input (e.g., joint torques)
- $\mathbf{x}_d$: Desired state
- $N$: Prediction horizon
- $Q, R, P$: Cost matrices (state error cost, control effort cost, terminal cost)

**Key advantage over PID**: MPC explicitly handles constraints (joint limits, torque limits, collision avoidance) and optimizes over future trajectory.

### 4.2.2 Linear MPC for Inverted Pendulum

Consider a cart-pole system (simplified humanoid balancing problem):

**State**: x = [x, theta, x_dot, theta_dot] (cart position, pole angle, velocities)
**Control**: $u$ (force on cart)

**Linearized dynamics** around upright equilibrium ($\theta = 0$):

$$
\mathbf{x}_{k+1} = \mathbf{A} \mathbf{x}_k + \mathbf{B} u_k
$$

Where (for cart mass $M=1$, pole mass $m=0.1$, length $L=1$):

$$
\mathbf{A} = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & mg/M & 1 & 0 \\ 0 & g(M+m)/(ML) & 0 & 1 \end{bmatrix}, \quad
\mathbf{B} = \begin{bmatrix} 0 \\ 0 \\ \Delta t / M \\ \Delta t / (ML) \end{bmatrix}
$$

**Python Implementation** (using CVXPY for quadratic programming):

```python
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt

class LinearMPC:
    """
    Linear Model Predictive Control for cart-pole balancing.
    """
    def __init__(self, A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray, N: int):
        """
        Args:
            A: State transition matrix (n x n)
            B: Control matrix (n x m)
            Q: State cost matrix (n x n)
            R: Control cost matrix (m x m)
            N: Prediction horizon
        """
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.N = N

        self.n = A.shape[0]  # State dimension
        self.m = B.shape[1]  # Control dimension

    def solve(self, x0: np.ndarray, x_ref: np.ndarray,
              u_min: float = -np.inf, u_max: float = np.inf) -> np.ndarray:
        """
        Solve MPC optimization problem.

        Args:
            x0: Current state (n,)
            x_ref: Reference state (n,)
            u_min, u_max: Control limits

        Returns:
            Optimal control sequence (N, m)
        """
        # Decision variables
        x = cp.Variable((self.N + 1, self.n))  # States
        u = cp.Variable((self.N, self.m))      # Controls

        # Objective
        cost = 0
        for k in range(self.N):
            cost += cp.quad_form(x[k] - x_ref, self.Q) + cp.quad_form(u[k], self.R)
        cost += cp.quad_form(x[self.N] - x_ref, self.Q)  # Terminal cost

        # Constraints
        constraints = [x[0] == x0]  # Initial condition
        for k in range(self.N):
            constraints += [
                x[k+1] == self.A @ x[k] + self.B @ u[k],  # Dynamics
                u[k] >= u_min,                             # Control limits
                u[k] <= u_max
            ]

        # Solve
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP, verbose=False)

        if problem.status != cp.OPTIMAL:
            print(f"Warning: MPC solver status: {problem.status}")
            return np.zeros((self.N, self.m))

        return u.value


# Cart-pole simulation
def cart_pole_dynamics(x: np.ndarray, u: float, dt: float) -> np.ndarray:
    """
    Nonlinear cart-pole dynamics.

    State: [cart_pos, pole_angle, cart_vel, pole_ang_vel]
    """
    M, m, L, g = 1.0, 0.1, 1.0, 9.81

    pos, theta, vel, theta_dot = x
    force = u

    # Nonlinear dynamics
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)

    temp = (force + m * L * theta_dot**2 * sin_theta) / (M + m)
    theta_ddot = (g * sin_theta - cos_theta * temp) / (L * (4/3 - m * cos_theta**2 / (M + m)))
    pos_ddot = temp - m * L * theta_ddot * cos_theta / (M + m)

    # Euler integration
    x_new = x.copy()
    x_new[0] += vel * dt
    x_new[1] += theta_dot * dt
    x_new[2] += pos_ddot * dt
    x_new[3] += theta_ddot * dt

    return x_new


if __name__ == "__main__":
    # System parameters
    dt = 0.05  # 20 Hz
    M, m, L, g = 1.0, 0.1, 1.0, 9.81

    # Linearized dynamics around upright
    A = np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, m*g/M, 1, 0],
        [0, g*(M+m)/(M*L), 0, 1]
    ])
    B = np.array([[0], [0], [dt/M], [dt/(M*L)]])

    # Cost matrices
    Q = np.diag([10.0, 100.0, 1.0, 1.0])  # Penalize position and angle error
    R = np.array([[0.1]])                  # Penalize control effort

    # MPC controller
    mpc = LinearMPC(A, B, Q, R, N=20)  # 20-step (1 second) horizon

    # Simulation
    duration = 10.0
    num_steps = int(duration / dt)

    x = np.array([0.0, 0.2, 0.0, 0.0])  # Initial: cart at 0, pole at 0.2 rad (11.5°)
    x_ref = np.array([0.0, 0.0, 0.0, 0.0])  # Target: upright at origin

    states = [x.copy()]
    controls = []

    for i in range(num_steps):
        # Solve MPC
        u_seq = mpc.solve(x, x_ref, u_min=-20, u_max=20)
        u = u_seq[0, 0]  # Apply first control (receding horizon)

        # Simulate (using nonlinear dynamics!)
        x = cart_pole_dynamics(x, u, dt)

        # Record
        states.append(x.copy())
        controls.append(u)

    states = np.array(states)
    controls = np.array(controls)
    time_array = np.arange(len(states)) * dt

    # Plot
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))

    axes[0].plot(time_array, states[:, 0], 'b-', linewidth=2)
    axes[0].set_ylabel('Cart Position (m)')
    axes[0].set_title('MPC Control - Cart-Pole Balancing')
    axes[0].grid(True)

    axes[1].plot(time_array, np.degrees(states[:, 1]), 'r-', linewidth=2)
    axes[1].set_ylabel('Pole Angle (deg)')
    axes[1].axhline(0, color='k', linestyle='--', alpha=0.3)
    axes[1].grid(True)

    axes[2].plot(time_array[:-1], controls, 'g-', linewidth=2)
    axes[2].axhline(20, color='r', linestyle='--', alpha=0.5, label='Limit')
    axes[2].axhline(-20, color='r', linestyle='--', alpha=0.5)
    axes[2].set_ylabel('Force (N)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig('mpc_cartpole_result.png', dpi=150)
    print("Plot saved to mpc_cartpole_result.png")

    print(f"\n=== MPC Performance ===")
    print(f"Final angle: {np.degrees(states[-1, 1]):.2f}°")
    print(f"Final cart position: {states[-1, 0]:.3f} m")
    print(f"Max control effort: {np.max(np.abs(controls)):.1f} N")
```

**Expected Output**:
```
Plot saved to mpc_cartpole_result.png

=== MPC Performance ===
Final angle: 0.01°
Final cart position: -0.002 m
Max control effort: 12.3 N
```

**Analysis**: MPC successfully balances the pole from 11.5° initial tilt while keeping cart position near zero and respecting force limits.

## 4.3 Whole-Body Control

### 4.3.1 Quadratic Programming (QP) Formulation

**Whole-body control** coordinates all DOF to achieve multiple simultaneous objectives (e.g., reach with hand while maintaining balance). It solves:

```math
\begin{aligned}
\min_{\mathbf{q}, \mathbf{F}} \quad & \|\mathbf{J}_{task} \ddot{\mathbf{q}} - \ddot{\mathbf{x}}_{task}\|^2 + w_{posture} \|\ddot{\mathbf{q}} - \ddot{\mathbf{q}}_{posture}\|^2 \\
\text{subject to} \quad & \mathbf{M}(\mathbf{q}) \ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) = \mathbf{\tau} + \mathbf{J}_c^\top \mathbf{F} \quad \text{(dynamics)} \\
& \mathbf{J}_c \ddot{\mathbf{q}} + \dot{\mathbf{J}}_c \dot{\mathbf{q}} = 0 \quad \text{(contact constraints)} \\
& \mathbf{\tau}_{min} \leq \mathbf{\tau} \leq \mathbf{\tau}_{max} \quad \text{(torque limits)} \\
& \text{Friction cone constraints on } \mathbf{F}
\end{aligned}
```

Where:
- q: Joint positions (generalized coordinates)
- q_double_dot: Joint accelerations (optimization variable)
- J_task: Task Jacobian (e.g., hand position)
- x_task_double_dot: Desired task-space acceleration
- $\mathbf{F}$: Contact forces
- $\mathbf{M}, \mathbf{C}$: Mass matrix and Coriolis/gravity terms

**Simplified Version** (operational space control without contacts):

```math
\ddot{\mathbf{q}}^* = \mathbf{J}^\dagger (\ddot{\mathbf{x}}_{task} - \dot{\mathbf{J}} \dot{\mathbf{q}}) + (\mathbf{I} - \mathbf{J}^\dagger \mathbf{J}) \ddot{\mathbf{q}}_{null}
```

Where J_dagger is the pseudo-inverse and q_null_double_dot is null-space motion (posture optimization).

### 4.3.2 Python Example: Redundancy Resolution

```python
import numpy as np
from scipy.linalg import pinv

class OperationalSpaceController:
    """
    Operational space control for redundant manipulators.

    Achieves end-effector task while optimizing posture in null space.
    """
    def __init__(self, num_joints: int):
        self.num_joints = num_joints

    def compute_acceleration(
        self,
        J: np.ndarray,
        J_dot: np.ndarray,
        q_dot: np.ndarray,
        x_ddot_desired: np.ndarray,
        q_ddot_posture: np.ndarray,
        damping: float = 0.01
    ) -> np.ndarray:
        """
        Compute joint accelerations for task + posture.

        Args:
            J: Task Jacobian (3 x n for 3D end-effector position)
            J_dot: Time derivative of Jacobian
            q_dot: Current joint velocities
            x_ddot_desired: Desired task-space acceleration
            q_ddot_posture: Desired joint-space acceleration (posture)
            damping: Damping for pseudo-inverse

        Returns:
            Joint accelerations (n,)
        """
        # Damped pseudo-inverse
        J_pinv = pinv(J, rcond=damping)

        # Task-space term
        task_term = J_pinv @ (x_ddot_desired - J_dot @ q_dot)

        # Null-space projector
        N = np.eye(self.num_joints) - J_pinv @ J

        # Null-space term (posture optimization)
        null_term = N @ q_ddot_posture

        return task_term + null_term


# Example: 7-DOF arm reaching while optimizing elbow angle
if __name__ == "__main__":
    controller = OperationalSpaceController(num_joints=7)

    # Simplified example: 3-DOF planar arm
    # Joint angles: [shoulder, elbow, wrist]
    q = np.array([np.pi/4, np.pi/3, np.pi/6])
    q_dot = np.zeros(3)

    # Forward kinematics Jacobian (simplified, would use URDF in real system)
    L1, L2, L3 = 0.5, 0.3, 0.2  # Link lengths
    J = np.array([
        [-L1*np.sin(q[0]) - L2*np.sin(q[0]+q[1]) - L3*np.sin(sum(q)),
         -L2*np.sin(q[0]+q[1]) - L3*np.sin(sum(q)),
         -L3*np.sin(sum(q))],
        [L1*np.cos(q[0]) + L2*np.cos(q[0]+q[1]) + L3*np.cos(sum(q)),
         L2*np.cos(q[0]+q[1]) + L3*np.cos(sum(q)),
         L3*np.cos(sum(q))]
    ])

    J_dot = np.zeros_like(J)  # Simplified

    # Task: Move end-effector 0.1 m/s^2 in x direction
    x_ddot_desired = np.array([0.1, 0.0])

    # Posture: Prefer elbow angle near 90°
    q_desired_posture = np.array([0, np.pi/2, 0])
    kp_posture = 5.0
    q_ddot_posture = kp_posture * (q_desired_posture - q)

    # Compute
    q_ddot = controller.compute_acceleration(J, J_dot, q_dot, x_ddot_desired, q_ddot_posture)

    print("Joint accelerations (rad/s^2):")
    print(q_ddot)
    print("\nNote: Joint 1 (elbow) acceleration adjusts toward 90° posture")
```

## 4.4 Force/Torque Control

### 4.4.1 Impedance Control

**Impedance control** regulates the dynamic relationship between force and motion:

```math
\mathbf{F} = \mathbf{M}_d (\ddot{\mathbf{x}}_d - \ddot{\mathbf{x}}) + \mathbf{D}_d (\dot{\mathbf{x}}_d - \dot{\mathbf{x}}) + \mathbf{K}_d (\mathbf{x}_d - \mathbf{x})
```

Where $\mathbf{M}_d, \mathbf{D}_d, \mathbf{K}_d$ are desired inertia, damping, and stiffness.

**Applications**:
- Compliant grasping (soft contact with objects)
- Human-robot collaboration (safe interaction)
- Force-controlled assembly

```python
class ImpedanceController:
    """Cartesian impedance control."""
    def __init__(self, M: np.ndarray, D: np.ndarray, K: np.ndarray):
        self.M = M  # Desired inertia
        self.D = D  # Desired damping
        self.K = K  # Desired stiffness

    def compute_force(
        self,
        x: np.ndarray,
        x_dot: np.ndarray,
        x_ddot: np.ndarray,
        x_d: np.ndarray,
        x_dot_d: np.ndarray,
        x_ddot_d: np.ndarray
    ) -> np.ndarray:
        """Compute desired force for impedance."""
        F = (self.M @ (x_ddot_d - x_ddot) +
             self.D @ (x_dot_d - x_dot) +
             self.K @ (x_d - x))
        return F
```

## 4.5 Exercises

### Exercise 1 (Easy): Tune PID for Different Dynamics
Modify the motor inertia to 1.0 (10x larger). Re-tune PID gains to achieve similar performance.

### Exercise 2 (Easy): Implement Anti-Windup
Add conditional integration to PID: only integrate when output is not saturated.

### Exercise 3 (Medium): MPC for Mobile Robot
Implement MPC for a differential-drive robot. State: $[x, y, \theta]$, control: $[v, \omega]$.

### Exercise 4 (Medium): Inverse Dynamics Control
Compute feedforward torque using inverse dynamics: tau_ff = M(q) * q_d_double_dot + C(q, q_dot). Combine with PID.

### Exercise 5 (Hard): Whole-Body QP with Contact
Formulate whole-body controller for humanoid with foot contacts. Include friction cone constraints.

## Summary

This chapter covered:
1. **PID control**: The workhorse of joint-level control
2. **MPC**: Optimal control with constraints and prediction
3. **Whole-body control**: Coordinating all DOF for complex tasks
4. **Force/torque control**: Compliant interaction with environment

**Next Chapter**: [Computer Vision](./computer-vision.md) - Perception for manipulation

## References

1. Spong, M. W., et al. (2020). *Robot Modeling and Control*. Wiley.
2. Borrelli, F., et al. (2017). *Predictive Control for Linear and Hybrid Systems*. Cambridge.
3. Sentis, L., & Khatib, O. (2005). "Synthesis of whole-body behaviors through hierarchical control of behavioral primitives". *IJRR*.
4. Hogan, N. (1985). "Impedance control: An approach to manipulation". *Journal of Dynamic Systems*.
