#ifndef QUADRATIC_PROGRAMMING_IK_H
#define QUADRATIC_PROGRAMMING_IK_H

#include "Sanctuary/RRRManipulator.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "osqp++.h"

namespace IK {
    bool quadraticProgrammingIK(RRRManipulator& robot, double target_x, double target_y, double target_phi,
                                double &theta1, double &theta2, double &theta3);
}

#endif
