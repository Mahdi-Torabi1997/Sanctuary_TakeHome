#include "Sanctuary/InverseKinematics.h"
#include <cmath>

namespace IK {

    bool inverseKinematicsAlgebraic(const RRRManipulator &robot, double x, double y, double phi,
                                    double &theta1, double &theta2, double &theta3) {
        // get link lengths
        double L1 = robot.getL1();
        double L2 = robot.getL2();
        double L3 = robot.getL3();

        // Compute the wrist center
        double xw = x - L3 * std::cos(phi);
        double yw = y - L3 * std::sin(phi);

        // Compute squared distance to wrist center
        double r2 = xw * xw + yw * yw;

        // calculate for theta2
        double cosTheta2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        if (std::fabs(cosTheta2) > 1.0) {
            return false; // No solution exists
        }

        //AS there are 3DoF and task needs 2Dof so there will be more than one answer I assumed elbow down
        double sinTheta2 = std::sqrt(1 - cosTheta2 * cosTheta2); // Elbow-down solution
        theta2 = std::atan2(sinTheta2, cosTheta2);

        // Solve for theta1
        theta1 = std::atan2(yw, xw) - std::atan2(L2 * std::sin(theta2), L1 + L2 * std::cos(theta2));

        // Solve for theta3
        theta3 = phi - (theta1 + theta2);

        return true;
    }

}
