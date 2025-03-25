#ifndef JACOBIAN_H
#define JACOBIAN_H

/// @brief Declares the function to compute the analytical Jacobian for a 3-DOF RRR manipulator.

#include "Sanctuary/RRRManipulator.h"
#include <Eigen/Dense>
#include <cmath>

namespace IK {

    /**
     * @brief Computes the Jacobian matrix for a 3-DOF planar RRR manipulator.
     *
     * The Jacobian relates joint velocities [θ̇1, θ̇2, θ̇3] to the end-effector
     * spatial velocity [ẋ, ẏ, φ̇]. It's a 3x3 matrix composed of the partial derivatives
     * of the end-effector position and orientation with respect to each joint angle.
     *
        J = [ ∂x/∂θ1   ∂x/∂θ2   ∂x/∂θ3 ]
            [ ∂y/∂θ1   ∂y/∂θ2   ∂y/∂θ3 ]
            [   1        1        1   ]   <-- Rotation φ = θ1 + θ2 + θ3

        Where:
        - L1, L2, L3 -> lengths
        - θ1, θ2, θ3 -> joint angles
     *
     * @param robot The RRRManipulator object containing link lengths and current joint angles.
     * @return Eigen::Matrix3d The computed 3x3 Jacobian matrix.
     */
    Eigen::Matrix3d computeJacobian(const RRRManipulator& robot);

}

#endif // JACOBIAN_H
