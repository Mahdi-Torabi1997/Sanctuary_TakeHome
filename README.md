# 3-DOF Planar Manipulator Kinematics Solver

### Author: Mahdi Torabi  
This project is part of a take-home assignment of Sanctuary AI interview for solving both **Forward Kinematics** and **Inverse Kinematics** of a 3-link planar manipulator using different analytical and numerical methods.

---

## 📝 Task Overview

- **Task 1:** Implement a kinematic model for a planar RRR robot.
- **Task 2:** Implement the **Forward Kinematics (FK)** function.
- **Task 3:** Implement the **Inverse Kinematics (IK)** using:
  - ✅ **Analytical solution**
  - ✅ **Gradient Descent (GD)**
  - ✅ **Levenberg-Marquardt (LM)**
  - ✅ **Quadratic Programming (QP)**
  - ✅ **Null-space optimization** (Extra)

---

## 🤖 Robot Description

The robot is a 3R planar manipulator defined by:
- Link lengths: `L1 = 0.3`, `L2 = 0.3`, `L3 = 0.1` (default in meters)
- Joint angles: `θ1`, `θ2`, `θ3` (in radians)

---

## 🧠 Mathematical Foundations

### 🔹 Forward Kinematics

Forward kinematics determines the end-effector pose $(x, y, \phi)$ based on joint angles and link lengths.

---

### 🔸 1. Denavit–Hartenberg (DH) Parameters

| $i$    | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$   |
|--------|----------------|-----------|-------|--------------|
| 1      | $0$            | $0$       | $0$   | $\theta_1$   |
| 2      | $0$            | $L_1$     | $0$   | $\theta_2$   |
| 3      | $0$            | $L_2$     | $0$   | $\theta_3$   |
| e.e.   | $0$            | $L_3$     | $0$   | $0$          |

---

### 🔸 2. Transformation Matrix Definition

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

### 🔸 3. Compute Each Transformation

- $^0T_1$:
$$
\begin{bmatrix}
\cos\theta_1 & -\sin\theta_1 & 0 & 0 \\
\sin\theta_1 &  \cos\theta_1 & 0 & 0 \\
0            & 0             & 1 & 0 \\
0            & 0             & 0 & 1
\end{bmatrix}
$$

- $^1T_2$:
$$
\begin{bmatrix}
\cos\theta_2 & -\sin\theta_2 & 0 & L_1 \\
\sin\theta_2 &  \cos\theta_2 & 0 & 0 \\
0            & 0             & 1 & 0 \\
0            & 0             & 0 & 1
\end{bmatrix}
$$

- $^2T_3$:
$$
\begin{bmatrix}
\cos\theta_3 & -\sin\theta_3 & 0 & L_2 \\
\sin\theta_3 &  \cos\theta_3 & 0 & 0 \\
0            & 0             & 1 & 0 \\
0            & 0             & 0 & 1
\end{bmatrix}
$$

- $^3T_{e.e.}$:
$$
\begin{bmatrix}
1 & 0 & 0 & L_3 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

### 🔸 4. Full Forward Kinematics

Total transformation:

$$
^0T_{e.e.} = ^0T_1 \cdot ^1T_2 \cdot ^2T_3 \cdot ^3T_{e.e.}
$$

Multiply these to get the final transformation matrix from base to end-effector.

---

### 🔸 5. Final Pose Equations

From the matrix result, extract:

- **End-effector position** $(x, y)$:
$$
\begin{aligned}
x &= L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2) + L_3 \cos(\theta_1 + \theta_2 + \theta_3) \\
y &= L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2) + L_3 \sin(\theta_1 + \theta_2 + \theta_3)
\end{aligned}
$$

- **Orientation** $\phi$:
$$
\phi = \theta_1 + \theta_2 + \theta_3
$$

---

### 🔹 Inverse Kinematics (Analytical)

Using geometry:
- Solve for the wrist center
- Apply Law of Cosines for `θ2`
- Solve for `θ1` using triangle geometry
- Compute `θ3 = φ - θ1 - θ2`

✅ **Two solutions exist** (elbow-up, elbow-down). By default, we use the elbow-down configuration.

---

## ⚙️ Numerical IK Methods

### 🔸 1. Gradient Descent (GD)

- Minimizes position error by iteratively updating `θ` using:

```
Δθ = α * Jᵀ * (target_pos - current_pos)
```

- Fast but sensitive to:
  - Initial guess
  - Learning rate
  - Local minima

---

### 🔸 2. Levenberg-Marquardt (LM)

- Combines GD and Gauss-Newton for better convergence:

```
Δθ = (Jᵀ * J + λ * I)⁻¹ * Jᵀ * error
```

- Handles ill-conditioned Jacobians with damping.

---

### 🔸 3. Quadratic Programming (QP)

- Formulated as:

```
min (1/2) Δθᵀ * P * Δθ + qᵀ * Δθ
```

With:
- `P = Jᵀ * J + regularization`
- Joint constraints (e.g., θ₁ ∈ [-70°, +70°])

✅ Solved using **OSQP**.

---

## 🌀 Null-Space Optimization

We implemented **null-space control** to:
- Prefer secondary objectives (like posture comfort)
- Avoid joint limit saturation

The update is:

```
Δθ = J⁺ * e + (I - J⁺ * J) * (-λ * (θ - θ_rest))
```

Where:
- `J⁺`: damped pseudoinverse of the Jacobian
- `(I - J⁺ * J)`: null-space projector
- `θ_rest`: preferred rest posture (e.g., elbow-up)

This helps select the **elbow-up** solution via a virtual spring pull toward a desired configuration.

---

## 🧪 Testing

All components are tested using **GoogleTest**:
- ✅ Forward Kinematics
- ✅ Inverse Kinematics (valid and invalid poses)
- ✅ Integration tests (IK + FK round-trip)

To run:

```bash
cd build
cmake ..
make
ctest --output-on-failure
```

---

## 🗂️ Project Structure

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

---

## 🔍 Learnings and Evolution

| Method         | Math Basis            | Strength            | Limitation            |
|----------------|------------------------|----------------------|------------------------|
| Analytical     | Trig geometry          | Fast, exact          | Ambiguous solutions   |
| Gradient Descent | Optimization (1st-order) | Simple, intuitive     | Local minima          |
| Levenberg-Marquardt | Gauss-Newton variant   | Stable, fast convergence | Requires damping tune |
| QP             | Convex Optimization    | Constraints supported | Slower setup          |
| Null-space     | Redundancy control     | Custom posture goals  | Extra computation     |

---

## 📌 Notes

- ✅ You can set custom link lengths and joint angles via API.
- ✅ Code is object-oriented for easy extensibility.
- ✅ QP implementation uses [OSQP](https://osqp.org/) with Eigen support.
- 🔧 `.gitignore` excludes `build/`, but **helper libs like OSQP** are included for portability.

---

## 🙋‍♂️ About Me

I'm **Mahdi Torabi**, a mechatronics engineer passionate about robotics, motion control, and simulation.  
This repo demonstrates my ability to:
- Analyze robotic systems
- Implement optimization-based control
- Write clean, tested C++ code

---

## 🚀 Future Ideas

- Visualize manipulator using matplotlib or 3D engines
- Animate trajectory IK
- Extend to 6DOF or SCARA arms
- Add velocity and torque control modules
