#include "Sanctuary/GradientDescentIKElbowUp.h"
#include "Sanctuary/computeJacobian.h"
#include "Sanctuary/ForwardKinematics.h"
#include <Eigen/Dense>
#include <cmath>

#define ALPHA 0.2             // Learning rate
#define MAX_ITER 10000
#define TOLERANCE 1e-3
#define LAMBDA 1               // Regularization 
#define MAX_STEP 0.2          // Maximum step size

namespace IK_ElbowUp {

    bool gradientDescentIK(RRRManipulator& robot,
                           double target_x, double target_y, double target_phi,
                           double &theta1, double &theta2, double &theta3) {

        theta1 = robot.getTheta1();
        theta2 = robot.getTheta2();
        theta3 = robot.getTheta3();

        Eigen::Vector3d theta_rest(0.0, M_PI / 2.0, 0.0); // rest pose

        for (int iter = 0; iter < MAX_ITER; ++iter) {
            double x_curr, y_curr, phi_curr;
            getEndEffectorPose(robot, x_curr, y_curr, phi_curr);

            // error
            Eigen::Vector2d error(target_x - x_curr, target_y - y_curr);
            if (error.norm() < TOLERANCE) return true;

            // position Jacobian
            Eigen::MatrixXd J_full = IK::computeJacobian(robot); // 3x3
            Eigen::MatrixXd J_pos = J_full.topRows(2); // 2x3

            Eigen::Vector3d theta_vec(theta1, theta2, theta3);

            // Compute damped pseudoinverse
            double damping = 1e-6;
            Eigen::Matrix2d JJt = J_pos * J_pos.transpose(); // 2x2
            Eigen::Matrix2d JJt_damped = JJt + damping * Eigen::Matrix2d::Identity();
            Eigen::MatrixXd J_pos_pinv = J_pos.transpose() * JJt_damped.inverse(); // 3x2

            // Null-space projection
            Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - J_pos_pinv * J_pos;

            // Primary task
            Eigen::Vector3d grad_task = J_pos_pinv * error;

            // Null-space optimization
            Eigen::Vector3d grad_null = -LAMBDA * (theta_vec - theta_rest);

            Eigen::Vector3d dTheta = grad_task + P * grad_null;

            if (dTheta.norm() > MAX_STEP) {
                dTheta *= MAX_STEP / dTheta.norm();
            }

            theta_vec += ALPHA * dTheta;
            theta1 = theta_vec(0);
            theta2 = theta_vec(1);
            theta3 = theta_vec(2);
            robot.setJointAngles(theta1, theta2, theta3);
        }

        return false;
    }

}