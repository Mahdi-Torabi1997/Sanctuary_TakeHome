#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "Sanctuary/RRRManipulator.h"

namespace IK {

    /**
     * @brief Computes the analytical inverse kinematics solution for a planar 3-DOF RRR manipulator.
     *
     * This function analytically solves the inverse kinematics problem for a desired end-effector pose (x, y, φ),
     * assuming an "elbow-down" configuration. The method uses geometric relationships and trigonometry
     * to compute joint angles θ₁, θ₂, and θ₃.
     *
     * @param robot    The RRRManipulator object containing link lengths.
     * @param x        Target x-coordinate of the end-effector.
     * @param y        Target y-coordinate of the end-effector.
     * @param phi      Desired orientation (in radians) of the end-effector.
     * @param theta1   Output: Joint angle θ₁ in radians.
     * @param theta2   Output: Joint angle θ₂ in radians.
     * @param theta3   Output: Joint angle θ₃ in radians.
     * @return true if a valid solution exists; false if the target is unreachable.
     *
     * @note The function does not account for joint limits.
     */
    bool inverseKinematicsAlgebraic(const RRRManipulator &robot,
                                    double x, double y, double phi,
                                    double &theta1, double &theta2, double &theta3);

}

#endif // INVERSEKINEMATICS_H
