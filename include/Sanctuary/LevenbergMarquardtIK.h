#ifndef LEVENBERG_MARQUARDT_IK_H
#define LEVENBERG_MARQUARDT_IK_H

#include "Sanctuary/RRRManipulator.h"
#include <Eigen/Dense>

namespace IK {

    bool levenbergMarquardtIK(RRRManipulator& robot, double target_x, double target_y, double target_phi,
                              double &theta1, double &theta2, double &theta3);

    Eigen::Matrix3d computeJacobian(const RRRManipulator& robot);

}

#endif
