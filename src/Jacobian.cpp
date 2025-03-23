#include "Sanctuary/computeJacobian.h"
#include <Eigen/Dense>

// Computes the Jacobian matrix usig joint angles and links' lengths

namespace IK {
    /*
    Mathematically J Matrix will be as below (refer to readme file for plot)

    joint angular velocities [θ̇1, θ̇2, θ̇3]T
    end-effector linear and angular velocity [ẋ, ẏ, φ̇]T

    J = [ ∂x/∂θ1   ∂x/∂θ2   ∂x/∂θ3 ]
        [ ∂y/∂θ1   ∂y/∂θ2   ∂y/∂θ3 ]
        [   1        1        1   ]   <-- Rotation φ = θ1 + θ2 + θ3

    Where:
      - L1, L2, L3 -> lengths
      - θ1, θ2, θ3 -> joint angles

    Elements:
    J11 = -L1*sin(θ1) - L2*sin(θ1 + θ2) - L3*sin(θ1 + θ2 + θ3)
    J12 =             - L2*sin(θ1 + θ2) - L3*sin(θ1 + θ2 + θ3)
    J13 =                                 - L3*sin(θ1 + θ2 + θ3)

    J21 =  L1*cos(θ1) + L2*cos(θ1 + θ2) + L3*cos(θ1 + θ2 + θ3)
    J22 =               L2*cos(θ1 + θ2) + L3*cos(θ1 + θ2 + θ3)
    J23 =                                  L3*cos(θ1 + θ2 + θ3)

    J31 = J32 = J33 = 1 (since φ = θ1 + θ2 + θ3)
    */


    Eigen::Matrix3d computeJacobian(const RRRManipulator& robot) {
        double L1 = robot.getL1(), L2 = robot.getL2(), L3 = robot.getL3();
        double t1 = robot.getTheta1(), t2 = robot.getTheta2(), t3 = robot.getTheta3();

        double c1 = cos(t1), s1 = sin(t1);
        double c12 = cos(t1 + t2), s12 = sin(t1 + t2);
        double c123 = cos(t1 + t2 + t3), s123 = sin(t1 + t2 + t3);

        Eigen::Matrix3d J;
        J << -L1*s1 - L2*s12 - L3*s123, -L2*s12 - L3*s123, -L3*s123,
                L1*c1 + L2*c12 + L3*c123,  L2*c12 + L3*c123,  L3*c123,
                1, 1, 1;

        return J;
    }

}
