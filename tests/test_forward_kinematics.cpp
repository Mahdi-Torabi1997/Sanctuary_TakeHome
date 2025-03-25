#include <gtest/gtest.h>
#include "Sanctuary/ForwardKinematics.h"
#include "Sanctuary/RRRManipulator.h"
#include <cmath>

constexpr double TOL = 1e-6; // Global test tolerance
constexpr double L1 = 0.3;
constexpr double L2 = 0.3;
constexpr double L3 = 0.1;

// Reusable test fixture
class ForwardKinematicsTest : public ::testing::Test {
protected:
    RRRManipulator robot;

    ForwardKinematicsTest() : robot(L1, L2, L3) {}

    void SetJointAngles(double t1, double t2, double t3) {
        robot.setJointAngles(t1, t2, t3);
    }

    Eigen::Matrix4d T() {
        return computeForwardKinematics(robot);
    }

    void CheckPose(double expected_x, double expected_y, double expected_phi) {
        double x, y, phi;
        getEndEffectorPose(robot, x, y, phi);
        EXPECT_NEAR(x, expected_x, TOL);
        EXPECT_NEAR(y, expected_y, TOL);
        EXPECT_NEAR(phi, expected_phi, TOL);
    }
};

// Test FK at zero configuration
TEST_F(ForwardKinematicsTest, ZeroAngles) {
    SetJointAngles(0.0, 0.0, 0.0);
    CheckPose(L1 + L2 + L3, 0.0, 0.0);
}

// Test FK when all joints are at π/2
TEST_F(ForwardKinematicsTest, AllJointsPiOver2) {
    SetJointAngles(M_PI/2, 0.0, 0.0);
    CheckPose(0.0, L1 + L2 + L3, M_PI/2);
}

// Test FK when joints bend negatively
TEST_F(ForwardKinematicsTest, NegativeAngles) {
    SetJointAngles(-M_PI/2, 0.0, 0.0);
    CheckPose(0.0, -(L1 + L2 + L3), -M_PI/2);
}

// Test FK with mixed joint angles
TEST_F(ForwardKinematicsTest, MixedAngles) {
    SetJointAngles(M_PI/4, -M_PI/4, 0.0);
    // Compute FK manually
    double phi = M_PI/4 - M_PI/4 + 0.0; // = 0
    double x = L1 * cos(M_PI/4) + L2 * cos(M_PI/4 - M_PI/4) + L3 * cos(0);
    double y = L1 * sin(M_PI/4) + L2 * sin(M_PI/4 - M_PI/4) + L3 * sin(0);
    CheckPose(x, y, phi);
}


// Test FK with all joints contributing to φ
TEST_F(ForwardKinematicsTest, CumulativeRotation) {
    SetJointAngles(M_PI/6, M_PI/6, M_PI/6); // Total φ = π/2
    CheckPose(L1 * std::cos(M_PI/6) +
              L2 * std::cos(M_PI/3) +
              L3 * std::cos(M_PI/2),
              L1 * std::sin(M_PI/6) +
              L2 * std::sin(M_PI/3) +
              L3 * std::sin(M_PI/2),
              M_PI/2);
}
