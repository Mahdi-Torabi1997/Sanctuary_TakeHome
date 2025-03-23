#include "Sanctuary/ForwardKinematics.h"
#include <cmath>


// Computes the homogeneous transformation matrix
// The transforms used are:

// T01 = [ c1 -s1 0 0;
//         s1  c1 0 0;
//         0   0  1 0;
//         0   0  0 1]

// T12 = [ c2 -s2 0 L1;
//         s2  c2 0 0;
//         0   0  1 0;
//         0   0  0 1]

// T23 = [ c3 -s3 0 L2;
//         s3  c3 0 0;
//         0   0  1 0;
//         0   0  0 1]

// T3ee = [ 1 0 0 L3;
//          0 1 0 0;
//          0 0 1 0;
//          0 0 0 1]
//
// Overall: T0ee = T01*T12*T23*T3ee


Eigen::Matrix4d computeForwardKinematics(const RRRManipulator &robot) {
    double L1 = robot.getL1();
    double L2 = robot.getL2();
    double L3 = robot.getL3();
    double theta1 = robot.getTheta1();
    double theta2 = robot.getTheta2();
    double theta3 = robot.getTheta3();

    // compute cosines and sines
    double c1 = std::cos(theta1), s1 = std::sin(theta1);
    double c2 = std::cos(theta2), s2 = std::sin(theta2);
    double c3 = std::cos(theta3), s3 = std::sin(theta3);

    // T01: from frame 0 to 1.
    Eigen::Matrix4d T01;
    T01 <<  c1, -s1, 0, 0,
            s1,  c1, 0, 0,
            0,   0, 1, 0,
            0,   0, 0, 1;

    // T12: from frame 1 to 2.
    Eigen::Matrix4d T12;
    T12 <<  c2, -s2, 0, L1,
            s2,  c2, 0,  0,
            0,   0, 1,  0,
            0,   0, 0,  1;

    // T23: from frame 2 to 3.
    Eigen::Matrix4d T23;
    T23 <<  c3, -s3, 0, L2,
            s3,  c3, 0,  0,
            0,   0, 1,  0,
            0,   0, 0,  1;

    // T3ee: from frame 3 to end-effector.
    Eigen::Matrix4d T3ee;
    T3ee << 1, 0, 0, L3,
            0, 1, 0,  0,
            0, 0, 1,  0,
            0, 0, 0,  1;

    return T01 * T12 * T23 * T3ee;
}

void getEndEffectorPose(const RRRManipulator &robot, double &x, double &y, double &phi) {
    Eigen::Matrix4d T0ee = computeForwardKinematics(robot);
    x = T0ee(0,3);
    y = T0ee(1,3);
    // Here in this assignment it is moving planar so Phi is not important as it can be calculated knowing (x,y), but in an angel Phi will be required
    phi = std::atan2(T0ee(1,0), T0ee(0,0));
    Eigen::MatrixXd computeJacobian(const RRRManipulator &robot);

}
