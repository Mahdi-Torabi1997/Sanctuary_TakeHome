#ifndef JACOBIAN_H
#define JACOBIAN_H

#include "Sanctuary/RRRManipulator.h"
#include <Eigen/Dense>
#include <cmath>

namespace IK {
    Eigen::Matrix3d computeJacobian(const RRRManipulator& robot);
}

#endif
