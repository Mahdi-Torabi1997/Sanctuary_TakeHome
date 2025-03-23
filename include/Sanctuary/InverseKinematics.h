#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "Sanctuary/RRRManipulator.h"

// Computes the inverse; Analytical solution
namespace IK {
    bool inverseKinematicsAlgebraic(const RRRManipulator &robot, double x, double y, double phi,
                                    double &theta1, double &theta2, double &theta3);
}

#endif
