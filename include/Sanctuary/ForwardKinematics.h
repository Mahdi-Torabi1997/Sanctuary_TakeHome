#ifndef FORWARDKINEMATICS_H
#define FORWARDKINEMATICS_H

#include "Sanctuary/RRRManipulator.h"
#include <Eigen/Dense>

/// @brief Provides functions to compute the forward kinematics for a planar 3-DOF RRR manipulator.
/// @brief Computes the full transformation matrix from the base to the end-effector (T0ee).
///
/// This function uses the homogeneous transformation matrices for each link (T01, T12, T23, T3ee)
/// and multiplies them to compute the pose of the end-effector in the world frame.
///
/// @param robot The RRRManipulator object with joint angles and link lengths set.
/// @return A 4x4 homogeneous transformation matrix representing T0ee (base to end-effector).

Eigen::Matrix4d computeForwardKinematics(const RRRManipulator &robot);

/// @brief Extracts the 2D pose (x, y, phi) of the end-effector from the transformation matrix.
///
/// This function calls `computeForwardKinematics()` and extracts:
/// - x, y position from the translation part
/// - phi (orientation) as atan2(T0ee(1,0), T0ee(0,0))
///
/// @param robot The RRRManipulator object.
/// @param x Output x-position of the end-effector.
/// @param y Output y-position of the end-effector.
/// @param phi Output orientation angle (in radians) of the end-effector.

void getEndEffectorPose(const RRRManipulator &robot, double &x, double &y, double &phi);

#endif // FORWARDKINEMATICS_H
