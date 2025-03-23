#include "Sanctuary/QuadraticProgrammingIK.h"
#include "Sanctuary/computeJacobian.h"
#include "Sanctuary/ForwardKinematics.h"
#include <iostream>
#include <osqp++.h>
#include <Eigen/Sparse>

namespace IK {

    bool quadraticProgrammingIK(RRRManipulator& robot, double target_x, double target_y, double target_phi,
                                double &theta1, double &theta2, double &theta3) {

        // Joint limit for θ₁ in radians (-70° to 70°) just as an example
        const double theta1_min = -1.2217;
        const double theta1_max =  1.2217;

        // Penalty parameters
        const double velocity_penalty = 1e-2;
        const double delta_velocity_penalty = 1e-3;
        const double position_penalty = 1e-1;

        // Increase iterations to allow more gradual convergence
        const int max_iterations = 1000;

        // Get initial joint angles
        Eigen::Vector3d q = {robot.getTheta1(), robot.getTheta2(), robot.getTheta3()};

        for (int iter = 0; iter < max_iterations; ++iter) {

            // Compute current FK & Jacobian
            double x_current, y_current, phi_current;
            getEndEffectorPose(robot, x_current, y_current, phi_current);
            Eigen::Matrix3d J = computeJacobian(robot);

            // Compute task-space error ΔX
            Eigen::Vector3d delta_X(target_x - x_current, target_y - y_current, target_phi - phi_current);

            // Print error for debugging
            //std::cout << "Iteration " << iter << " error norm: " << delta_X.norm() << std::endl;

            // If the error is small enough, break early
            if(delta_X.norm() < 1e-3) {
                std::cout << "Converged after " << iter << " iterations." << std::endl;
                break;
            }

            // QP objective formulation
            Eigen::Matrix3d P = J.transpose() * J;
            Eigen::Vector3d q_vec = -J.transpose() * delta_X;

            // Adjust weights
            double error_magnitude = delta_X.norm();
            // Here, you might want to refine or remove the scaling factor if it overcompensates:
            double scaling_factor = std::max(1.0, error_magnitude * 5.0);
            P += velocity_penalty * scaling_factor * Eigen::Matrix3d::Identity();
            P += delta_velocity_penalty * Eigen::Matrix3d::Identity();
            P += position_penalty * Eigen::Matrix3d::Identity();

            // Constraint for θ₁: Δq₁ ∈ [θ₁_min - q(0), θ₁_max - q(0)]
            Eigen::SparseMatrix<double> A(1, 3);
            A.insert(0, 0) = 1.0;

            Eigen::VectorXd l(1), u(1);
            l << theta1_min - q(0);
            u << theta1_max - q(0);

            // Set up OSQP instance
            osqp::OsqpInstance instance;
            instance.objective_matrix = P.sparseView();
            instance.objective_vector = q_vec;
            instance.constraint_matrix = A;
            instance.lower_bounds = l;
            instance.upper_bounds = u;

            osqp::OsqpSettings settings;
            settings.alpha = 1.8; // Step size parameter
            settings.verbose = false;
            osqp::OsqpSolver solver;
            solver.Init(instance, settings);
            osqp::OsqpExitCode status = solver.Solve();

            if (status != osqp::OsqpExitCode::kOptimal) {
                std::cout << "QP failed to find a solution." << std::endl;
                return false;
            }

            // Get the joint increments
            Eigen::Vector3d delta_q = solver.primal_solution();

            // Adaptive step limit: larger steps when error is high, smaller as we converge
            double step_limit = std::min(0.5, error_magnitude * 0.5);
            for (int i = 0; i < 3; i++) {
                if (delta_q(i) > step_limit) delta_q(i) = step_limit;
                if (delta_q(i) < -step_limit) delta_q(i) = -step_limit;
            }

            // Update joint angles
            q += delta_q;
            robot.setJointAngles(q(0), q(1), q(2));
        }

        // Return final joint angles
        theta1 = q(0);
        theta2 = q(1);
        theta3 = q(2);
        return true;
    }

} // namespace IK
