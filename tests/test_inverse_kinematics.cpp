#include <gtest/gtest.h>
#include "Sanctuary/InverseKinematics.h"
#include "Sanctuary/RRRManipulator.h"
#include "Sanctuary/ForwardKinematics.h"
#include <cmath>

// Tolerance for floating-point comparisons
const double EPS = 1e-6;

TEST(InverseKinematicsTest, SolvesForValidPose) {
    // Given robot and target pose
    RRRManipulator robot(0.3, 0.3, 0.1);
    double expected_x = 0.3;
    double expected_y = 0.3;
    double expected_phi = M_PI / 4;

    double theta1, theta2, theta3;

    // Solve IK
    bool success = IK::inverseKinematicsAlgebraic(robot, expected_x, expected_y, expected_phi,
                                                  theta1, theta2, theta3);

    EXPECT_TRUE(success);

    // Set solved angles
    robot.setJointAngles(theta1, theta2, theta3);

    // Compute forward kinematics to check solution
    double actual_x, actual_y, actual_phi;
    getEndEffectorPose(robot, actual_x, actual_y, actual_phi);

    EXPECT_NEAR(actual_x, expected_x, EPS);
    EXPECT_NEAR(actual_y, expected_y, EPS);
    EXPECT_NEAR(actual_phi, expected_phi, EPS);
}

TEST(InverseKinematicsTest, RejectsUnreachablePose) {
    RRRManipulator robot(0.3, 0.3, 0.1);

    // Target pose that is unreachable (too far)
    double x = 10.0, y = 10.0, phi = 0.0;
    double theta1, theta2, theta3;

    bool success = IK::inverseKinematicsAlgebraic(robot, x, y, phi, theta1, theta2, theta3);
    EXPECT_FALSE(success);
}
