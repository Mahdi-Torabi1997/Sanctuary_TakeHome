#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include "quik/Robot.hpp"
#include "quik/IKSolver.hpp"
#include "Sanctuary/RRRManipulator.h"
#include "Sanctuary/ForwardKinematics.h"
#include "Sanctuary/InverseKinematics.h"
#include "Sanctuary/GradientDescentIK.h"
#include "Sanctuary/LevenbergMarquardtIK.h"
#include "Sanctuary/QuadraticProgrammingIK.h"
#include "Sanctuary/GradientDescentIKElbowUp.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct TestCase {
    double x;
    double y;
    double phi;
};

using namespace IK;


int main() {

    cout << "\n \nTask two \n";
    cout << "Forward Kinematic Using Transformation Matrices \n";

    // Initialize the robot with link lengths
    RRRManipulator r(0.3, 0.3, 0.1);

    // Set joint angles  in radians
    double theta1 = 0;  
    double theta2 = M_PI/2;  
    double theta3 = 0;    
    r.setJointAngles(theta1, theta2, theta3);

    // Compute forward kinematics
    Eigen::Matrix4d T0ee_manual = computeForwardKinematics(r);

    // Print the transformation matrix
    std::cout << "Forward Kinematics:\nT0ee:\n" << T0ee_manual << "\n\n";

    // Extract and print end-effector pose
    double x, y, phi;
    getEndEffectorPose(r, x, y, phi);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "End-effector Pose:\n";
    std::cout << "x = " << x << ", y = " << y << ", phi = " << phi << " rad\n";

    //_______________________________________________________________________________________
    //_______________________________________________________________________________________

    cout << "\n \nTask Three \n";
    cout << "Forward Kinematic Using Transformation Matrices \n\n";



    TestCase testCases[] = {
        {0.5, 0.2, M_PI/4},
        {0.3, 0.3, M_PI/2},
        {0.2, -0.1, -M_PI/4}
    };
    int numTests = sizeof(testCases) / sizeof(TestCase);

    for (int i = 0; i < numTests; i++) {
        double solTheta1, solTheta2, solTheta3;
        cout << "Test Case " << (i+1) << ":" << endl;
        cout << "Desired Pose: x = " << testCases[i].x << ", y = " << testCases[i].y
             << ", phi = " << testCases[i].phi << endl;

        // === Algebraic IK
        if (IK::inverseKinematicsAlgebraic(r, testCases[i].x, testCases[i].y, testCases[i].phi,
                                           solTheta1, solTheta2, solTheta3)) {
            cout << "\n Algebraic IK Solution:" << endl;
            cout << "theta1 = " << solTheta1 << ", theta2 = " << solTheta2
                 << ", theta3 = " << solTheta3 << endl;
            r.setJointAngles(solTheta1, solTheta2, solTheta3);
            double x_sol, y_sol, phi_sol;
            getEndEffectorPose(r, x_sol, y_sol, phi_sol);
            cout << "Verification (FK): x = " << x_sol << ", y = " << y_sol
                 << ", phi = " << phi_sol << endl;
        } else {
            cout << "Algebraic IK failed." << endl;
        }

        // === Gradient Descent IK
        if (IK::gradientDescentIK(r, testCases[i].x, testCases[i].y, testCases[i].phi,
                                  solTheta1, solTheta2, solTheta3)) {
            cout << "\n Gradient Descent IK Solution:" << endl;
            cout << "theta1 = " << solTheta1 << ", theta2 = " << solTheta2
                 << ", theta3 = " << solTheta3 << endl;
        } else {
            cout << "Gradient Descent IK failed." << endl;
        }

        // === Levenberg-Marquardt IK
        if (IK::levenbergMarquardtIK(r, testCases[i].x, testCases[i].y, testCases[i].phi,
                                     solTheta1, solTheta2, solTheta3)) {
            cout << "\n Levenberg-Marquardt IK Solution:" << endl;
            cout << "theta1 = " << solTheta1 << ", theta2 = " << solTheta2
                 << ", theta3 = " << solTheta3 << endl;
        } else {
            cout << "Levenberg-Marquardt IK failed." << endl;
        }

        // === Quadratic Programming IK
        if (IK::quadraticProgrammingIK(r, testCases[i].x, testCases[i].y, testCases[i].phi,
                                       solTheta1, solTheta2, solTheta3)) {
            cout << "\n Quadratic Programming IK Solution:" << endl;
            cout << "theta1 = " << solTheta1 << ", theta2 = " << solTheta2
                 << ", theta3 = " << solTheta3 << endl;
        } else {
            cout << "Quadratic Programming IK failed." << endl;
        }

        cout << endl;
    }

    //_______________________________________________________________________________________
    //_______________________________________________________________________________________

    cout << "\n \n" << "Forward and Inverse Kinematics Using QuIK Library" << "\n \n";
    // REFERENCE: https://github.com/steffanlloyd/quik?utm_source=chatgpt.com

    constexpr int DOF = 3;


    // DH Table Format: [a_i-1, alpha_i-1, d_i, theta_i]
    // same as the example they provided last link length was not in DH table and was in a form of ofset

    Eigen::Matrix<double, DOF, 4> DH;
    DH << 0.0, 0.0, 0.0, 0.0,   // θ₁ (base joint)
          0.3, 0.0, 0.0, 0.0,   // θ₂ (link 1 length = 0.3m)
          0.3, 0.0, 0.0, 0.0;   // θ₃ (link 2 length = 0.3m)

    // Joint Types; All joints are revolute 
    Eigen::Matrix<quik::JOINTTYPE_t, DOF, 1> linkTypes;
    linkTypes << quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE;

    // Joint Sign Vector for positive direction of rotation/translation for each joint
    Eigen::Matrix<double, DOF, 1> Qsign;
    Qsign << 1.0, 1.0, 1.0;

    // Transformation Matrices
    Eigen::Matrix4d Tbase = Eigen::Matrix4d::Identity(); // Base frame at world origin
    Eigen::Matrix4d Ttool = Eigen::Matrix4d::Identity(); // Tool frame offset
    Ttool(0, 3) = 0.1; // Tool offset of 0.1m 

    // Create Model
    // Encapsulates the kinematics using the QuIK library
    auto robot = std::make_shared<quik::Robot<DOF>>(DH, linkTypes, Qsign, Tbase, Ttool);

    // Inverse Kinematics Solver
    const quik::IKSolver<DOF> solver(
        robot,
        100,                     // max_iter
        quik::ALGORITHM_QUIK,    // algorithm
        1e-6,                    // exit_tolerance
        1e-10,                   // min_step_size
        0.05,                    // relative_improvement_tolerance
        10,                      // exit if this many steps fail to improve
        20,                      // total allowed gradient failures
        1e-10,                   // damping parameter
        0.1,                     // max_linear_step_size
        1.0                      // max_angular_step_size
    );

    // Forward Kinematics
    Eigen::Vector3d q_actual;
    q_actual << 0.0, M_PI / 2.0, 0.0;

    Eigen::Matrix<double, 4 * (DOF + 1), 4> T_all;
    robot->FK(q_actual, T_all); // Compute full transform chain
    Eigen::Matrix4d T0ee = T_all.block<4, 4>(4 * DOF, 0); // Extract T from base to end-effector

    std::cout << "Forward Kinematics (T0ee):\n" << T0ee << "\n\n";

    // Inverse Kinematics
    Eigen::Vector3d q_guess = Eigen::Vector3d::Zero(); // Initial guess (all zeros)
    Eigen::Vector3d q_solution;                        // Output solution
    Eigen::Matrix<double, 6, 1> err;                   // Pose error vector
    int iter;                                          // Iteration count
    quik::BREAKREASON_t reason;                        // Exit condition

    solver.IK(T0ee, q_guess, q_solution, err, iter, reason);

    std::cout << std::fixed << std::setprecision(6);

    if (reason == quik::BREAKREASON_TOLERANCE) {
        std::cout << "Solved:\n";
        std::cout << "q_solution = [" << q_solution.transpose() << "]\n";
        std::cout << "Iterations: " << iter << "\n";
        std::cout << "Final error: " << err.transpose() << "\n";
    } else {
        std::cout << "failed after " << iter << " iterations.\n";
        std::cout << "Break reason" << static_cast<int>(reason) << "\n";
    }

    std::cout << "\n\n[Gradient Descent IK vs Elbow-Up IK with Null-Space Optimization]\n";

    // Target pose (reachable and ambiguous configuration)
    double target_x = 0.4;
    double target_y = 0.0;
    double target_phi = 0.0;

    // initial guess
    double init_theta1 = 0.0;
    double init_theta2 = -M_PI / 4;
    double init_theta3 = M_PI / 4;

    // Normal Gradient Descent (Elbow-Down)
    RRRManipulator robot_gd(0.3, 0.3, 0.1);
    robot_gd.setJointAngles(init_theta1, init_theta2, init_theta3);
    double gd_t1, gd_t2, gd_t3;

    std::cout << "\n[Gradient Descent (Elbow-Down)]\n";
    if (gradientDescentIK(robot_gd, target_x, target_y, target_phi, gd_t1, gd_t2, gd_t3)) {
        std::cout << "Thetas: " << gd_t1 << ", " << gd_t2 << ", " << gd_t3 << "\n";
        double x, y, phi;
        getEndEffectorPose(robot_gd, x, y, phi);
        std::cout << "FK Pose: x = " << x << ", y = " << y << ", phi = " << phi << "\n";
    } else {
        std::cout << "Gradient Descent IK failed.\n";
    }

    // Gradient Descent with Null-Space (Elbow-Up)
    RRRManipulator robot_ns(0.3, 0.3, 0.1);
    robot_ns.setJointAngles(init_theta1, init_theta2, init_theta3);
    double ns_t1, ns_t2, ns_t3;

    std::cout << "\n[Gradient Descent with Null-Space (Elbow-Up)]\n";
    if (IK_ElbowUp::gradientDescentIK(robot_ns, target_x, target_y, target_phi, ns_t1, ns_t2, ns_t3)) {
        std::cout << "Thetas: " << ns_t1 << ", " << ns_t2 << ", " << ns_t3 << "\n";
        double x, y, phi;
        getEndEffectorPose(robot_ns, x, y, phi);
        std::cout << "FK Pose: x = " << x << ", y = " << y << ", phi = " << phi << "\n";
    } else {
        std::cout << "Elbow-Up IK failed.\n";
    }

    std::cout << "\n[Comparison Done]\n";
    

    return 0;
}
