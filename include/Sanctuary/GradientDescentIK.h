#ifndef GRADIENT_DESCENT_IK_H
#define GRADIENT_DESCENT_IK_H

#include "Sanctuary/RRRManipulator.h"
#include "Sanctuary/ForwardKinematics.h"
#include <Eigen/Dense>

namespace IK {

    /**
     * @brief Solves the inverse kinematics problem using a basic gradient descent algorithm.
     *
     * This function iteratively updates the joint angles of a 3-DOF planar RRR manipulator
     * by descending the gradient of the task-space error. It uses the transpose of the Jacobian
     * to compute the gradient direction.
     *
     * @param robot        Reference to the RRRManipulator object with initial joint angles.
     * @param target_x     Desired x position of the end-effector.
     * @param target_y     Desired y position of the end-effector.
     * @param target_phi   Desired orientation (phi) of the end-effector in radians.
     * @param theta1       Output: Computed joint angle θ₁ in radians.
     * @param theta2       Output: Computed joint angle θ₂ in radians.
     * @param theta3       Output: Computed joint angle θ₃ in radians.
     * @return true if the solver converges to a solution within the specified tolerance; false otherwise.
     *
     * @note This method is sensitive to learning rate and initial guess. It may get stuck in local minima
     *       or diverge if the parameters are not tuned properly. A btter choice can be using an adaptive learning rate.
     */
    bool gradientDescentIK(RRRManipulator& robot, double target_x, double target_y, double target_phi,
                           double &theta1, double &theta2, double &theta3);

    /**
     * @brief Computes the 3x3 Jacobian matrix for the RRR manipulator.
     *
     * @return Eigen::Matrix3d The Jacobian matrix representing partial derivatives of (x, y, phi)
     *                         with respect to θ₁, θ₂, θ₃.
     */
    Eigen::Matrix3d computeJacobian(const RRRManipulator& robot);

} // namespace IK

#endif // GRADIENT_DESCENT_IK_H
