#ifndef RRRMANIPULATOR_HPP
#define RRRMANIPULATOR_HPP

// RRRManipulator class includes link lengths and joint angles, also some methods to set and access these parameters that can be useful in long run
class RRRManipulator {
private:
    double L1, L2, L3;
    // Joint angles are in radians
    double theta1, theta2, theta3;

public:
    // Initialize the manipulator
    // Default values are provided if no arguments are passed
    // Joint angles are initialized to zero
    RRRManipulator(double L1 = 0.3, double L2 = 0.3, double L3 = 0.1);

    // Useful for modifying the robot's geometry during simulation or testing
    void setLinkLengths(double l1, double l2, double l3);

    // For updating the robot's configuration for forward or inverse kinematics
    void setJointAngles(double t1, double t2, double t3);

    // For kinematics functions
    double getL1() const;
    double getL2() const;
    double getL3() const;
    double getTheta1() const;
    double getTheta2() const;
    double getTheta3() const;
};

#endif