#include "Sanctuary/RRRManipulator.h"

// Initialize the manipulator with given link lengths and joint angles==0
RRRManipulator::RRRManipulator(double L1, double L2, double L3)
        : L1(L1), L2(L2), L3(L3), theta1(0.0), theta2(0.0), theta3(0.0) {}

// Updates the lengths of the manipulator's links
// Not used in this challenge but it can be helpful in general to make the code more generalized
void RRRManipulator::setLinkLengths(double l1, double l2, double l3) {
    L1 = l1;
    L2 = l2;
    L3 = l3;
}

// Same as prev Function ,Sets the current joint angles
void RRRManipulator::setJointAngles(double t1, double t2, double t3) {
    theta1 = t1;
    theta2 = t2;
    theta3 = t3;
}

// assistive functions to return length, used for F and IK
double RRRManipulator::getL1() const { return L1; }

double RRRManipulator::getL2() const { return L2; }

double RRRManipulator::getL3() const { return L3; }

double RRRManipulator::getTheta1() const { return theta1; }

double RRRManipulator::getTheta2() const { return theta2; }

double RRRManipulator::getTheta3() const { return theta3; }
