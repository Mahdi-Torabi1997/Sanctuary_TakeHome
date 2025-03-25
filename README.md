Inverse Kinematics Assignment - Sanctuary Take-Home Project

Author: Mahdi TorabiGitHub: Mahdi-Torabi1997

Overview

This project demonstrates the implementation of forward and inverse kinematics for a 3-DOF planar manipulator. It includes algebraic and numerical approaches to inverse kinematics, unit testing, and an object-oriented structure. This work was completed as part of a take-home assignment.

🦾 Robot Description

The robot consists of three revolute joints and three links with default lengths:

L1 = 0.3 m

L2 = 0.3 m

L3 = 0.1 m

The joint angles are:

θ1 - base rotation

θ2 - shoulder joint

θ3 - elbow joint

The robot operates in a 2D plane, and the forward and inverse kinematics map between joint space (θi) and task space (x, y, φ).

✅ Tasks Implemented

✅ Task 1: Kinematic Model

RRRManipulator class encapsulates link lengths and joint angles.

Provides setters/getters for clean, modular code.

✅ Task 2: Forward Kinematics

Goal: Compute (x, y, φ) from given θ1, θ2, θ3

Transformation matrix:

T_0^ee = T_01 * T_12 * T_23 * T_3ee

where each T is a 4x4 homogeneous transformation matrix based on Denavit-Hartenberg parameters (see image).

Final result:

x = L1*cos(θ1) + L2*cos(θ1 + θ2) + L3*cos(θ1 + θ2 + θ3)
y = L1*sin(θ1) + L2*sin(θ1 + θ2) + L3*sin(θ1 + θ2 + θ3)
φ = θ1 + θ2 + θ3

✅ Task 3: Inverse Kinematics

Goal: Solve for θi from a given target (x, y, φ).

✅ Algebraic Solution

Uses geometry and trigonometry.

Based on triangle construction and Law of Cosines:

c2 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2)
s2 = √(1 - c2^2)
θ2 = atan2(s2, c2)

k1 = L1 + L2*cos(θ2)
k2 = L2*sin(θ2)
θ1 = atan2(y, x) - atan2(k2, k1)
θ3 = φ - (θ1 + θ2)

✨ Bonus: Numerical IK Solvers

1. Gradient Descent (GD)

Minimizes the error between current and target (x, y, φ) iteratively:

∂E/∂θ = J^T * error

Sensitive to learning rate (ALPHA)

May converge slowly or fall into local minima.

2. Levenberg-Marquardt (LM)

A hybrid between GD and Gauss-Newton:

dθ = (J^T*J + λI)^(-1) * J^T * error

More stable and faster convergence.

λ (damping factor) balances speed vs. stability.

3. Quadratic Programming (QP)

Minimizes:

min || J*dθ - error ||^2 + regularization terms

Subject to joint constraints.

Uses OSQP solver.

More robust and flexible.

Allows velocity/position/step penalties.

💡 Bonus: Null Space Optimization (Elbow-Up Preference)

Implemented using null space projection to bias toward a desired configuration (e.g., elbow-up):

Given:

grad_task = J_pinv * error

grad_null = -λ*(θ - θ_rest)

Final update:

dθ = grad_task + (I - J_pinv*J)*grad_null

This allows executing the primary task while optimizing a secondary objective (like posture, energy, or safety).

→ Similar to adding a virtual spring pulling the solution to θ_rest

🧪 Testing

Implemented using Google Test (gtest)

Includes unit tests for:

Forward kinematics (test_forward_kinematics.cpp)

Algebraic IK (test_inverse_kinematics.cpp)

End-to-end FK-IK consistency (test_ik_fk_integration.cpp)

Run all tests with:

cd build
ctest --output-on-failure

📁 Folder Structure

Sanctuary_TakeHome/
├── src/                  # All implementation files
├── include/Sanctuary/    # All header files
├── tests/                # Unit tests using gtest
├── osqp-cpp/ & osqp-eigen/ # QP solver libraries
├── CMakeLists.txt        # CMake build config
├── .gitignore            # Ignores build files, etc.

🚀 How to Build

git clone https://github.com/Mahdi-Torabi1997/Sanctuary_TakeHome.git
cd Sanctuary_TakeHome
mkdir build && cd build
cmake ..
make
ctest --output-on-failure

🙋‍♂️ About Me

I’m Mahdi Torabi, a passionate engineer and researcher interested in robotics, control, and AI. This project combines my strengths in:

Robot modeling

Optimization and numerical solvers

Clean and modular C++ code

Hands-on testing and debugging

Feel free to explore the repo and connect if you have questions!

📬 Contact

📧 mahdi.torabi1997@gmail.com🔗 LinkedIn

⭐ Acknowledgments

OSQP library for QP optimization

Google Test for unit testing

Sanctuary Robotics for the assignment prompt


