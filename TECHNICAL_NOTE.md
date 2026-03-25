# Technical Note — Constrained Manipulation Benchmark for Space-Inspired Robotics

## 1. Objective

This project studies a simplified robotic manipulation problem representative of constrained interaction tasks that arise in space robotics, including capture-point approach, docking alignment, and manipulation near fragile hardware.

The technical objective is to compare:
- a **Cartesian impedance controller**, and
- a **joint-space PD baseline**

on the same constrained manipulation task.

## 2. System model

We consider a planar 2-link manipulator with generalized coordinates:

\[
q = [q_1, q_2]^T, \qquad \dot{q} = [\dot{q}_1, \dot{q}_2]^T
\]

### Forward kinematics

For link lengths \(l_1\) and \(l_2\), the end-effector position is:

\[
x(q)=
\begin{bmatrix}
l_1\cos q_1 + l_2\cos(q_1+q_2) \\
l_1\sin q_1 + l_2\sin(q_1+q_2)
\end{bmatrix}
\]

### Jacobian

The differential kinematics are:

\[
\dot{x} = J(q)\dot{q}
\]

with:

\[
J(q)=
\begin{bmatrix}
-l_1\sin q_1 - l_2\sin(q_1+q_2) & -l_2\sin(q_1+q_2) \\
l_1\cos q_1 + l_2\cos(q_1+q_2) & l_2\cos(q_1+q_2)
\end{bmatrix}
\]

### Simplified joint dynamics

A reduced joint-space model is used:

\[
M\ddot{q} + B\dot{q} + Kq = \tau
\]

where \(M\) is diagonal inertia, \(B\) is viscous damping, and \(K\) is a nominal stiffness matrix.

This simplified model keeps the benchmark lightweight while still supporting meaningful controller comparison.

## 3. Task definition

The manipulator is asked to move from an initial configuration toward a target capture point while remaining inside a corridor:

- target position: \((0.78, 0.42)\) m,
- wall boundary: \(x \leq 0.82\) m,
- vertical corridor: \(0.30 \leq y \leq 0.54\) m.

The reference trajectory is a smooth cubic interpolation between the initial end-effector position and the target.

## 4. Contact / constraint model

Virtual compliant walls emulate interaction with sensitive surrounding hardware. When the end-effector penetrates a boundary, a penalty force is applied:

\[
F_c = -k_c \delta - b_c \dot{\delta}
\]

where \(\delta\) is the penetration depth. The resulting joint disturbance torque is:

\[
\tau_{env} = J(q)^T F_c
\]

This model is intentionally simple but useful for testing safe motion under geometric constraints.

## 5. Controllers

### 5.1 Cartesian impedance control

The main controller is a task-space spring-damper law mapped into joint torques:

\[
F_{cmd} = K_p(x_d-x) + K_d(\dot{x}_d-\dot{x})
\]

\[
\tau = J(q)^T F_{cmd}
\]

This controller is well-suited for constrained manipulation because it directly regulates end-effector behavior.

### 5.2 Joint-space PD baseline

The comparison baseline tracks a fixed inverse-kinematics target:

\[
\tau = K_p(q^*-q) - K_d\dot{q}
\]

This controller can reach the goal but does not explicitly account for task-space constraints during the transient response.

## 6. Simulation setup

- time step: 2 ms
- horizon: 4 s
- link lengths: 0.7 m and 0.5 m
- torque saturation included
- deterministic simulation

## 7. Evaluation metrics

The benchmark reports:
- RMS end-effector tracking error,
- final tracking error,
- peak contact force,
- settling time below 2 cm after 0.5 s,
- number of constraint-violation samples,
- control effort surrogate based on \(\int ||\tau||^2 dt\).

## 8. Results

### Quantitative results

| Controller | RMS pos. error [m] | Final error [m] | Peak contact [N] | Settling [s] | Constraint violations |
|---|---:|---:|---:|---:|---:|
| impedance | 0.0013 | 0.0003 | 0.00 | 0.50 | 0 |
| joint_pd | 0.0884 | 0.0003 | 0.00 | 2.38 | 0 |

### Discussion

The Cartesian impedance controller clearly outperforms the joint-space baseline in transient task-space behavior. Even though both controllers eventually converge close to the target, the impedance controller provides:
- near-zero RMS tracking error,
- much faster settling,
- behavior that is naturally compatible with constrained manipulation.

This is consistent with the intuition that interaction-centric tasks should be regulated directly in task space rather than only through joint coordinates.

## 9. Relevance to space robotics

Although simplified, this benchmark reflects several important characteristics of robotic manipulation for space applications:
- geometric constraints around servicing interfaces,
- need for smooth and low-impact motion,
- task-space control importance,
- controller benchmarking under safety-related constraints.

It therefore forms a credible starting point for more advanced work involving:
- free-floating dynamics,
- hybrid motion-force control,
- tactile sensing,
- vision-guided servoing,
- learning-based adaptation,
- multi-modal manipulation.

## 10. Recommended research extensions

To evolve this benchmark toward a stronger research platform, the next steps would be:
1. Add a 6-DoF manipulator and orientation control.
2. Introduce floating-base spacecraft dynamics.
3. Replace penalty walls with complementarity-based contact.
4. Add uncertainty and robustness analysis.
5. Integrate model predictive control.
6. Add camera-based target estimation.
7. Test sim-to-real style domain randomization.
8. Wrap the benchmark into ROS 2 / Gazebo / Isaac Sim.

## 11. Reproducibility

All figures and metrics in this technical note are generated by the included script:

```bash
python scripts/run_benchmark.py
```

The outputs are stored in `results/`.

## 12. Honest positioning

This repository should be presented as a **compact research prototype** rather than a full publication-ready system. Its value lies in the clarity of the formulation, the relevance of the control comparison, and the reproducibility of the benchmark.
