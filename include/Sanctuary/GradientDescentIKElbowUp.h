#ifndef GRADIENT_DESCENT_IK_ELBOWUP_H
#define GRADIENT_DESCENT_IK_ELBOWUP_H

#include "RRRManipulator.h"

namespace IK_ElbowUp {

    /**
     * @brief inverse kinematics using gradient descent with null-space optimization
     *        to favor an elbow-up configuration for a 3-DOF planar manipulator
     *
     * This function computes joint angles (theta1, theta2, theta3) that achieve a desired
     * end-effector pose in 2D space (x, y) and orientation (phi), while biasing the solution
     * toward an elbow-up posture by adding a secondary optimization in the null space
     *
     * @param robot        Reference to the RRRManipulator object (robot model).
     * @param target_x     Desired end-effector x position.
     * @param target_y     Desired end-effector y position.
     * @param target_phi   Desired end-effector orientation (radians). May be ignored in some strategies.
     * @param theta1       Output: computed angle for joint 1 (radians).
     * @param theta2       Output: computed angle for joint 2 (radians).
     * @param theta3       Output: computed angle for joint 3 (radians).
     * @return true if the solution converges within a set number of iterations and tolerance; false otherwise.
     *
     */
    bool gradientDescentIK(RRRManipulator& robot,
                           double target_x, double target_y, double target_phi,
                           double &theta1, double &theta2, double &theta3);

}

#endif // GRADIENT_DESCENT_IK_ELBOWUP_H
