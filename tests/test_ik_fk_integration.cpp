#include <gtest/gtest.h>
#include "Sanctuary/InverseKinematics.h"
#include "Sanctuary/ForwardKinematics.h"
#include "Sanctuary/RRRManipulator.h"
#include <cmath>

constexpr double TOL = 1e-6;
constexpr double L1 = 0.3;
constexpr double L2 = 0.3;
constexpr double L3 = 0.1;

class IKFKIntegrationTest : public ::testing::Test {
protected:
    RRRManipulator robot;

    IKFKIntegrationTest() : robot(L1, L2, L3) {}

    void SolveAndValidate(double x, double y, double phi) {
        double t1, t2, t3;
        bool ik_success = IK::inverseKinematicsAlgebraic(robot, x, y, phi, t1, t2, t3);

        ASSERT_TRUE(ik_success) << "IK failed for pose (" << x << ", " << y << ", " << phi << ")";

        robot.setJointAngles(t1, t2, t3);

        double x_fk, y_fk, phi_fk;
        getEndEffectorPose(robot, x_fk, y_fk, phi_fk);

        EXPECT_NEAR(x_fk, x, TOL);
        EXPECT_NEAR(y_fk, y, TOL);
        EXPECT_NEAR(phi_fk, phi, TOL);
    }
};

// === Test Cases ===

TEST_F(IKFKIntegrationTest, BasicPose) {
    SolveAndValidate(0.25, 0.3, M_PI / 6);
}

TEST_F(IKFKIntegrationTest, StraightReachPose) {
    SolveAndValidate(0.3 + 0.3 + 0.1 - .01, 0.0, 0.0); // Slightly inside the edge
}

TEST_F(IKFKIntegrationTest, UpwardPose) {
    SolveAndValidate(0.1, 0.55, M_PI / 2);
}

TEST_F(IKFKIntegrationTest, NegativePhiPose) {
    SolveAndValidate(0.1, -0.55, -M_PI / 2);
}

TEST_F(IKFKIntegrationTest, DiagonalPose) {
    SolveAndValidate(0.35, 0.35, M_PI / 4);
}
