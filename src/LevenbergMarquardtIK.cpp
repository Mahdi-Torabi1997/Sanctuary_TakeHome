#include "Sanctuary/LevenbergMarquardtIK.h"
#include "Sanctuary/computeJacobian.h"
#include "Sanctuary/ForwardKinematics.h"
#include <Eigen/Dense>

#define LAMBDA_INIT 0.01   // Initial damping factor
#define MAX_ITER 10000
#define TOLERANCE 1e-4

namespace IK {

    bool levenbergMarquardtIK(RRRManipulator& robot, double target_x, double target_y, double target_phi,
                              double &theta1, double &theta2, double &theta3) {


        // Start with current joint values
        theta1 = robot.getTheta1();
        theta2 = robot.getTheta2();
        theta3 = robot.getTheta3();

        double lambda = LAMBDA_INIT;

        for (int iter = 0; iter < MAX_ITER; iter++) {

            // Get current end-effector pose
            double x_curr, y_curr, phi_curr;
            getEndEffectorPose(robot, x_curr, y_curr, phi_curr);

            // Compute error between current and target pose
            Eigen::Vector3d error(target_x - x_curr, target_y - y_curr, target_phi - phi_curr);
            if (error.norm() < TOLERANCE) return true;

            // Compute damped update step
            Eigen::Matrix3d J = computeJacobian(robot);
            Eigen::Matrix3d JtJ = J.transpose() * J;
            Eigen::Matrix3d damping = lambda * Eigen::Matrix3d::Identity();
            Eigen::Vector3d dTheta = (JtJ + damping).inverse() * J.transpose() * error;

            // Update joint angles
            theta1 += dTheta(0);
            theta2 += dTheta(1);
            theta3 += dTheta(2);

            robot.setJointAngles(theta1, theta2, theta3);
        }

        return false; // No convergence
    }

}
