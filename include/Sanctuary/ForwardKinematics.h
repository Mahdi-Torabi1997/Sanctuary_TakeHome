#ifndef FORWARDKINEMATICS_H
#define FORWARDKINEMATICS_H

#include "Sanctuary/RRRManipulator.h"
#include <Eigen/Dense>

// Extracts the end-effector pose (x, y, phi) from T0->ee

Eigen::Matrix4d computeForwardKinematics(const RRRManipulator &robot);
void getEndEffectorPose(const RRRManipulator &robot, double &x, double &y, double &phi);

#endif

