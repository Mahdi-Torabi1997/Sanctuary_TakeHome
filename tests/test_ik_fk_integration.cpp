#include <gtest/gtest.h>
#include "Sanctuary/InverseKinematics.h"
#include "Sanctuary/ForwardKinematics.h"
#include "Sanctuary/RRRManipulator.h"
#include <cmath>

const double EPS = 1e-6;

TEST(IKFKIntegrationTest, PoseRecoveredAfterIKandFK) {
    // Original pose
    double target_x = 0.25;
    double target_y = 0.3;
    double target_phi = M_PI / 6;

    // Robot model
    RRRManipulator robot(0.3, 0.3, 0.1);

    // Solve IK
    double theta1, theta2, theta3;
    bool ik_success = IK::inverseKinematicsAlgebraic(robot, target_x, target_y, target_phi,
                                                     theta1, theta2, theta3);
    ASSERT_TRUE(ik_success);

    // Apply IK result to robot
    robot.setJointAngles(theta1, theta2, theta3);

    // Run FK to compute resulting end-effector pose
    double actual_x, actual_y, actual_phi;
    getEndEffectorPose(robot, actual_x, actual_y, actual_phi);

    // Compare recovered pose with original target pose
    EXPECT_NEAR(actual_x, target_x, EPS);
    EXPECT_NEAR(actual_y, target_y, EPS);
    EXPECT_NEAR(actual_phi, target_phi, EPS);
}
