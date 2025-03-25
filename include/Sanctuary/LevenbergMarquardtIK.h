#ifndef LEVENBERG_MARQUARDT_IK_H
#define LEVENBERG_MARQUARDT_IK_H

#include "Sanctuary/RRRManipulator.h"
#include <Eigen/Dense>

namespace IK {

    /**
     * @brief Solves the inverse kinematics problem for a 3-DOF planar RRR manipulator
     *        using the Levenberg-Marquardt optimization algorithm.
     *
     * This method is a hybrid between Gauss-Newton and gradient descent. It minimizes
     * the task-space error by iteratively updating the joint angles using a damped least squares update.
     * It's particularly useful for handling singularities or when the Jacobian is ill-conditioned.
     *
     * @param robot        Reference to the RRRManipulator instance with current joint configuration.
     * @param target_x     Desired end-effector x position in meters.
     * @param target_y     Desired end-effector y position in meters.
     * @param target_phi   Desired end-effector orientation (phi) in radians.
     * @param theta1       Output: computed joint angle θ₁ (in radians).
     * @param theta2       Output: computed joint angle θ₂ (in radians).
     * @param theta3       Output: computed joint angle θ₃ (in radians).
     * @return true if convergence is successful within the iteration limit; false otherwise.
     *
     * @note Uses a fixed initial damping value (lambda) and does not dynamically adapt it.
     *       can improve robustness by adjusting lambda based on update success.
     */
    bool levenbergMarquardtIK(RRRManipulator& robot, double target_x, double target_y, double target_phi,
                              double &theta1, double &theta2, double &theta3);

    /**
     * @brief Computes the Jacobian matrix of the RRR manipulator for the current configuration.
     * 
     * @return Eigen::Matrix3d 3x3 Jacobian matrix representing partial derivatives of end-effector pose
     *                         with respect to joint angles.
     *
     */
    Eigen::Matrix3d computeJacobian(const RRRManipulator& robot);

} // namespace IK

#endif // LEVENBERG_MARQUARDT_IK_H
