#include "Sanctuary/GradientDescentIK.h"
#include "Sanctuary/computeJacobian.h"
#include "Sanctuary/ForwardKinematics.h"
#include <Eigen/Dense>

// learning Hyperparams, the problem with GD is that if not tune enough will get stuck in local minima or even diverge
#define ALPHA 0.1  // Learning rate
#define MAX_ITER 10000
#define TOLERANCE 1e-4

namespace IK {

    bool gradientDescentIK(RRRManipulator& robot, double target_x, double target_y, double target_phi,
                           double &theta1, double &theta2, double &theta3) {
        // Initialize joint angles from robot state

        theta1 = robot.getTheta1();
        theta2 = robot.getTheta2();
        theta3 = robot.getTheta3();

        for (int iter = 0; iter < MAX_ITER; iter++) {
            // Get current end-effector pose
            double x_curr, y_curr, phi_curr;
            getEndEffectorPose(robot, x_curr, y_curr, phi_curr);

            // Compute pose error
            Eigen::Vector3d error(target_x - x_curr, target_y - y_curr, target_phi - phi_curr);
            if (error.norm() < TOLERANCE) return true;

            // Update joint angles using gradient descent
            Eigen::Matrix3d J = computeJacobian(robot);
            Eigen::Vector3d dTheta = ALPHA * J.transpose() * error;

            theta1 += dTheta(0);
            theta2 += dTheta(1);
            theta3 += dTheta(2);

            // Apply updated angles to robot
            robot.setJointAngles(theta1, theta2, theta3);
        }

        // If no convergence after max iterations
        return false;
    }

}
