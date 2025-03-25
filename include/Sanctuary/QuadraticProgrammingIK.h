#ifndef QUADRATIC_PROGRAMMING_IK_H
#define QUADRATIC_PROGRAMMING_IK_H

#include "RRRManipulator.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "osqp++.h"

namespace IK {

    /**
     * @brief Solves the inverse kinematics problem for a 3-DOF planar RRR manipulator
     *        using Quadratic Programming
     *
     * This approach minimizes the task-space error between the current and target
     * end-effector poses while optionally incorporating joint limits and penalties for
     * large joint movements. It uses OSQP (Operator Splitting Quadratic Program) as the QP solver.
     * 
     * https://github.com/google/osqp-cpp
     *
     * The cost function includes:
     *  - Task-space error
     *  - Penalization of joint velocities (for smoother solutions)
     *  - Joint limit constraints (e.g., for θ₁)
     *
     * The solution is computed iteratively and converges toward the desired (x, y, φ) pose.
     *
     * @param robot        Reference to the RRRManipulator instance with link parameters.
     *                     The initial joint angles inside this object are used as the starting point.
     * @param target_x     Desired end-effector x position in meters.
     * @param target_y     Desired end-effector y position in meters.
     * @param target_phi   Desired end-effector orientation in radians.
     * @param theta1       Output: computed joint angle θ₁ (radians).
     * @param theta2       Output: computed joint angle θ₂ (radians).
     * @param theta3       Output: computed joint angle θ₃ (radians).
     * 
     * @return true if a valid IK solution is found; false if optimization fails or does not converge.
     *
     * @note Only θ₁ is constrained with joint limits in the current implementation as a demonstration.
     *       Other joint limits can be added similarly by extending the constraint matrix A.
     */
    bool quadraticProgrammingIK(RRRManipulator& robot, double target_x, double target_y, double target_phi,
                                double &theta1, double &theta2, double &theta3);

} // namespace IK

#endif // QUADRATIC_PROGRAMMING_IK_H
