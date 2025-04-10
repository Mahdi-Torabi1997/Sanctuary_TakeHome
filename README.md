# 3-DOF Planar Manipulator Kinematics Solver

### Author: Mahdi Torabi  
This project is part of the take-home assignment of Sanctuary AI interview for solving both **Forward Kinematics** and **Inverse Kinematics** of a 3-link planar manipulator using different analytical and numerical methods.

---

## Tasks Overview

- **Task 1:** Implement a kinematic model for a planar RRR robot.
- **Task 2:** Implement the **Forward Kinematics (FK)** function.
- **Task 3:** Implement the **Inverse Kinematics (IK)** using:
  -  **Analytical solution**
  -  **Gradient Descent (GD)**
  -  **Levenberg-Marquardt (LM)**
  -  **Quadratic Programming (QP)**
  -  **Null-space optimization** (Extra)

---

## Manipulator Description

The robot is a 3R planar manipulator defined by:
- Link lengths: `L1 = 0.3`, `L2 = 0.3`, `L3 = 0.1` (default in meters)
- Joint angles: `θ1`, `θ2`, `θ3` (in radians)

---

## Mathematical Foundations

###  Forward Kinematics

Forward kinematics determines the end-effector pose $(x, y, \phi)$ based on joint angles and link lengths.

---

###  1. Denavit–Hartenberg (DH) Parameters

| $i$    | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$   |
|--------|----------------|-----------|-------|--------------|
| 1      | $0$            | $0$       | $0$   | $\theta_1$   |
| 2      | $0$            | $L_1$     | $0$   | $\theta_2$   |
| 3      | $0$            | $L_2$     | $0$   | $\theta_3$   |
| e.e.   | $0$            | $L_3$     | $0$   | $0$          |

---

###  2. Transformation Matrix Definition

The standard homogeneous transformation between frames is:

$$
^{i-1}T_i =
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i & 0 & a_{i-1} \\
\sin\theta_i &  \cos\theta_i & 0 & 0 \\
0            & 0             & 1 & d_i \\
0            & 0             & 0 & 1
\end{bmatrix}
$$

---

###  3. Compute Each Transformation

- $^0T_1$:

$$
^0T_1 =
\begin{bmatrix}
\cos\theta_1 & -\sin\theta_1 & 0 & 0 \\
\sin\theta_1 &  \cos\theta_1 & 0 & 0 \\
0            & 0             & 1 & 0 \\
0            & 0             & 0 & 1
\end{bmatrix}
$$

- $^1T_2$:

$$
^1T_2 =
\begin{bmatrix}
\cos\theta_2 & -\sin\theta_2 & 0 & L_1 \\
\sin\theta_2 &  \cos\theta_2 & 0 & 0 \\
0            & 0             & 1 & 0 \\
0            & 0             & 0 & 1
\end{bmatrix}
$$

- $^2T_3$:

$$
^2T_3 =
\begin{bmatrix}
\cos\theta_3 & -\sin\theta_3 & 0 & L_2 \\
\sin\theta_3 &  \cos\theta_3 & 0 & 0 \\
0            & 0             & 1 & 0 \\
0            & 0             & 0 & 1
\end{bmatrix}
$$

- $^3T_{e.e.}$:

$$
^3T_{e.e.} =
\begin{bmatrix}
1 & 0 & 0 & L_3 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

###  4. Full Forward Kinematics

Total transformation:

$$
^0T_{e.e.} = ^0T_1 \cdot ^1T_2 \cdot ^2T_3 \cdot ^3T_{e.e.}
$$

Multiplied to get the final transformation matrix from base to end-effector.

---

###  5. Final Pose Equations

From the matrix result, extract:

- **End-effector position** $(x, y)$:


$$
x = L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2) + L_3 \cos(\theta_1 + \theta_2 + \theta_3)
$$

$$
y = L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2) + L_3 \sin(\theta_1 + \theta_2 + \theta_3)
$$

$$
\phi = \theta_1 + \theta_2 + \theta_3
$$

---

###  Inverse Kinematics (Analytical)

#### Approach Overview:
This method computes joint angles $\theta_1$, $\theta_2$, $\theta_3$ for a desired end-effector pose $(x, y, \phi)$, assuming a 3R planar manipulator.

---

#### Approach Overview

1. **Compute the wrist center**:

$$
x_w = x - L_3 \cos(\phi), \quad y_w = y - L_3 \sin(\phi)
$$

2. **Compute squared distance to wrist center**:

$$
r^2 = x_w^2 + y_w^2
$$

3. **Apply the Law of Cosines**:

$$
\cos(\theta_2) = \frac{r^2 - L_1^2 - L_2^2}{2 L_1 L_2}
$$

- Check feasibility: $\left|\cos(\theta_2)\right| \leq 1$

- Elbow-down configuration (positive root):

$$
\sin(\theta_2) = \sqrt{1 - \cos^2(\theta_2)}
$$

$$
\theta_2 = \text{atan2}(\sin(\theta_2), \cos(\theta_2))
$$

4. **Solve for $\theta_1$**:

$$
\theta_1 = \text{atan2}(y_w, x_w) - \text{atan2}(L_2 \sin(\theta_2), L_1 + L_2 \cos(\theta_2))
$$

5. **Solve for $\theta_3$**:

$$
\theta_3 = \phi - (\theta_1 + \theta_2)
$$

---
####  Key Notes:
- There are **two solutions**: elbow-up and elbow-down. Only **elbow-down** is implemented in the current function.
- The method uses **geometric intuition** and **trigonometric identities** to derive closed-form joint angles.
- The robot has **3 degrees of freedom (DOF)**, but the **task only requires 2 DOF** — specifically, the **\( x, y \)** position of the end-effector in the 2D plane.

- This means the system is **kinematically redundant**:  
  So the solution lies on a **1-dimensional null space**.
- In practice, this means **more than one joint configuration** can achieve the same end-effector position. 
- To resolve this, **additional constraints** can be imposed. Examples include:
  - Minimizing joint velocities or torques,
  - Biasing toward a desired posture (e.g. "elbow-down"),
  - Minimizing deviation from a rest configuration.
---


## Numerical IK Methods

This section outlines the numerical inverse kinematics methods implemented in this project. These methods are useful when analytical solutions are unavailable, unstable, or need to be generalized for more complex robots.

---

###  1. Gradient Descent (GD)

Gradient Descent minimizes the error between the desired and current end-effector pose through iterative updates.

#### Update Rule

$$
\theta_{t+1} = \theta_t - \eta \nabla f(\theta_t)
$$

Where:
- $\theta_t$ is the joint vector at iteration $t$
- $\eta$ is the learning rate
- $f(\theta) = \frac{1}{2} \| \mathbf{x}(\theta) - \mathbf{x}_{\text{target}} \|^2$ is the cost function
- $\nabla f(\theta)$ is computed via Jacobian transpose:

$$
\nabla f(\theta) = J(\theta)^T \left( \mathbf{x}(\theta) - \mathbf{x}_{\text{target}} \right)
$$

#### Pros
- Simple and intuitive to implement
- Low memory and compute cost

#### Cons
- Sensitive to learning rate
- May converge slowly
- Prone to local minima

---

###  2. Adaptive Gradient Descent *(not implemented)*

Vanilla GD can be improved with a decaying learning rate:

#### Adaptive Learning Rate

$$
\eta_t = \frac{\eta_0}{1 + \alpha t}
$$

Update rule becomes:

$$
\theta_{t+1} = \theta_t - \eta_t \nabla f(\theta_t)
$$

#### Benefits
- Faster convergence far from the target
- Better stability near solution
- Easy to extend from basic GD

---

###  3. Levenberg-Marquardt (LM)

A hybrid of **Gauss-Newton** and **Gradient Descent** with a damping term for stability.

#### Update Rule

$$
\Delta \theta = -\left( J^T J + \lambda I \right)^{-1} J^T \left( \mathbf{x}(\theta) - \mathbf{x}_{\text{target}} \right)
$$

Where:
- $J$ is the Jacobian matrix
- $\lambda$ is a damping factor
- $I$ is the identity matrix

#### Pros
- Faster convergence than GD
- Robust near singularities
- Handles mild non-linearity well

#### Cons
- Requires matrix inversion
- Damping factor tuning needed

---

###  4. Quadratic Programming (QP)

Formulates inverse kinematics as a constrained optimization problem.

#### Objective

Minimize:

$$
\frac{1}{2} \Delta \theta^T P \Delta \theta + q^T \Delta \theta
$$

Regularization terms:

- **Velocity penalty**: encourages smaller updates  
- **Delta velocity penalty**: smoothens convergence  
- **Position penalty**: biases toward minimizing task error directly  

These are tuned adaptively during optimization depending on the magnitude of the error.

---

#### Constraint Formulation

In this implementation:

- Only joint \( \theta_1 \) is constrained between [−70°, +70°]  
- The constraint is imposed on the **change in \( \theta_1 \)**, i.e.:

$$
\theta_1^{\text{min}} - \theta_1 \leq \Delta \theta_1 \leq \theta_1^{\text{max}} - \theta_1
$$

This is enforced as a linear constraint:

$$
A \Delta \theta \leq u,\quad A \Delta \theta \geq l
$$

Where:

$$
A = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}
$$

 Solved using the [OSQP](https://osqp.org/) solver.

---

#### Pros

- Handles joint constraints  
- Stable even near singularities or redundancy  
- Supports regularization and prioritization

#### Cons

- Requires a QP solver (e.g., OSQP)  
- Slower per iteration than basic GD or LM

---

### Summary

| Method               | Advantages                         | Limitations                       |
|----------------------|------------------------------------|------------------------------------|
| Gradient Descent     | Simple, lightweight                | Sensitive to $\eta$, local minima  |
| Adaptive GD          | Better stability, dynamic steps    | Needs decay tuning                 |
| Levenberg-Marquardt  | Robust & fast convergence          | Damping parameter tuning           |
| Quadratic Programming| Constraint handling, generalizable | Slower, requires solver            |

> For redundancy-aware control (3 DOF manipulator, 2 DOF task), QP and null-space optimization allow for additional posture objectives like joint comfort or collision avoidance.


## Null-Space Optimization

To resolve the redundancy of a 3-DoF planar manipulator performing a 2D task (position only), we use **null-space projection** to bias the solution toward a preferred joint configuration (elbow-up).

---

####  Joint Update Rule:

$$
\Delta \theta = J^+ \mathbf{e} + (I - J^+ J)(-\lambda(\theta - \theta_{\text{rest}}))
$$

Where:
- $\theta$ is the current joint vector  
- $J$ is the Jacobian matrix (2×3 for position)  
- $J^+$ is the damped pseudoinverse of $J$  
- $\theta_{\text{rest}}$ is a preferred joint posture (e.g., elbow-up)  
- $\lambda$ is a gain controlling how strongly we pull toward $\theta_{\text{rest}}$

---

####  Virtual Spring Logic:

The term:

$$\lambda (\theta - \theta_{\text{rest}})
$$

acts as a **virtual spring**, pulling the configuration toward $\theta_{\text{rest}}$ **within the null space** of the task. This encourages elbow-up poses without affecting end-effector accuracy.

---

This technique enables redundancy resolution, posture control, and safer motion in real-time applications.

## Testing

All components are tested using **GoogleTest**:
- Forward Kinematics
- Inverse Kinematics (valid and invalid poses)
- Integration tests (IK + FK round-trip)

To run:

```bash
cd build
cmake ..
make
ctest --output-on-failure
```

---

## Project Structure

```
├── include/
│   └── Sanctuary/         # All header files
├── src/                   # All implementation files
├── tests/                 # GTest-based unit tests
├── osqp-eigen/            # External solver for QP
├── osqp-cpp/              # C++ binding for OSQP
├── CMakeLists.txt         # Build configuration
└── README.md              # This file
```


## Notes

-  You can set custom link lengths and joint angles via API.
-  QP implementation uses [OSQP](https://osqp.org/) with Eigen support.
-  `.gitignore` excludes `build/`, but **helper libs like OSQP** are included for portability as some files have been altered after cloning and their cmake has been changed to disable internal testing.

---

##  Running with Docker

This project is Dockerized to ensure consistent results 

### Build and Run the Container

```bash
# Pull from Docker Hub (already built and tested)
docker pull mahditorabi/cpp_takehome:latest

# Run interactively
sudo docker run -it mahditorabi/cpp_takehome:latest
```

Once inside the container:

```bash
cd /root/Sanctuary/build

./main_exec      # Run your main IK/FK executable
./unit_tests     # Run your GTest unit tests
```

The container includes:
- OSQP built from source
- `osqp-cpp` C++ wrapper
- `Eigen3`, `GTest`, and `QuIK` installed
- All build steps already completed
- 
## Bonus Task: ROS 2 Integration

As a bonus, this project includes a complete **ROS 2 package** demonstrating the use of forward and inverse kinematics through inter-node communication.

### Goal:
- Accept joint angles `θ₁`, `θ₂`, `θ₃` via launch parameters.
- Compute forward kinematics and publish the end-effector pose.
- Subscribe to that pose, run inverse kinematics, and print the recovered joint angles.

---

### Architecture

- **`publisher_node`**  
  - Takes `L1`, `L2`, `L3`, and joint angles as parameters.
  - Computes **Forward Kinematics** using `getEndEffectorPose()`.
  - Publishes end-effector pose (`x, y, φ`) on topic `/ee_pose`.

- **`subscriber_node`**  
  - Subscribes to `/ee_pose`.
  - Uses **Inverse Kinematics** to recover joint angles `θ₁`, `θ₂`, `θ₃`.
  - Prints recovered angles to the terminal.

---

### Launch File

Runs both nodes with parameterized joint values:


### Package Structure

```text
cpp_pkg/
├── src/
│   ├── publisher_node.cpp       # FK computation and publishing
│   └── subscriber_node.cpp      # IK computation and logging
├── launch/
│   └── cpp_pkg_launch.py
├── CMakeLists.txt
└── package.xml
```
### How to Run
```text
-colcon build --packages-select cpp_pkg
-source install/setup.bash
```


Then launch the nodes:
```text
-ros2 launch cpp_pkg cpp_pkg_launch.py
```

---



