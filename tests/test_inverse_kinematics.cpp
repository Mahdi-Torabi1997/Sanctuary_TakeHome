#include <gtest/gtest.h>
#include "Sanctuary/InverseKinematics.h"
#include "Sanctuary/ForwardKinematics.h"
#include "Sanctuary/RRRManipulator.h"
#include <cmath>

constexpr double TOL = 1e-6;
constexpr double L1 = 0.3;
constexpr double L2 = 0.3;
constexpr double L3 = 0.1;

class InverseKinematicsTest : public ::testing::Test {
protected:
    RRRManipulator robot;

    InverseKinematicsTest() : robot(L1, L2, L3) {}

    void SolveAndCheck(double x, double y, double phi) {
        double t1, t2, t3;

        bool success = IK::inverseKinematicsAlgebraic(robot, x, y, phi, t1, t2, t3);
        EXPECT_TRUE(success) << "IK solution failed for a reachable pose.";

        robot.setJointAngles(t1, t2, t3);

        double x_sol, y_sol, phi_sol;
        getEndEffectorPose(robot, x_sol, y_sol, phi_sol);

        EXPECT_NEAR(x_sol, x, TOL);
        EXPECT_NEAR(y_sol, y, TOL);
        EXPECT_NEAR(phi_sol, phi, TOL);
    }
};

// === Test Cases ===

TEST_F(InverseKinematicsTest, ValidPoseSimple) {
    SolveAndCheck(0.3, 0.3, M_PI / 4);
}

TEST_F(InverseKinematicsTest, ValidPoseStraightArm) {
    SolveAndCheck(0.3 + 0.3 + 0.1, 0.0, 0.0);
}

TEST_F(InverseKinematicsTest, ValidPoseBentUp) {
    SolveAndCheck(0.1, 0.5, M_PI / 2);
}

TEST_F(InverseKinematicsTest, ValidPoseBentDown) {
    SolveAndCheck(0.1, -0.5, -M_PI / 2);
}

TEST_F(InverseKinematicsTest, RejectsUnreachablePose) {
    double t1, t2, t3;
    bool success = IK::inverseKinematicsAlgebraic(robot, 10.0, 10.0, 0.0, t1, t2, t3);
    EXPECT_FALSE(success);
}
