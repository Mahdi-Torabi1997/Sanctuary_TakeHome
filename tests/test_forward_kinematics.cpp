#include <gtest/gtest.h>
#include "Sanctuary/ForwardKinematics.h"
#include "Sanctuary/RRRManipulator.h"

// Test forward kinematics at zero position
TEST(ForwardKinematicsTest, ZeroPosition) {
    RRRManipulator robot(0.3, 0.3, 0.1); // L1, L2, L3
    robot.setJointAngles(0, 0, 0); // theta1, theta2, theta3

    Eigen::Matrix4d T0ee = computeForwardKinematics(robot);

    double expected_x = 0.3 + 0.3 + 0.1;
    double expected_y = 0.0;

    EXPECT_NEAR(T0ee(0, 3), expected_x, 1e-6);
    EXPECT_NEAR(T0ee(1, 3), expected_y, 1e-6);
}

// Add more tests for non-zero angles, etc.
