#ifndef RRRMANIPULATOR_HPP
#define RRRMANIPULATOR_HPP

/// @brief Defines a class representing a planar 3-DOF RRR manipulator for representing and manipulating the state of a planar 3-link manipulator.
/// and provides accessors and mutators for use in kinematic algorithms like forward or inverse kinematics.
class RRRManipulator {
private:
    double L1, L2, L3;             ///< Lengths of the three links
    double theta1, theta2, theta3; ///< Joint angles (in radians)

public:
    /// @brief Constructor for initializing link lengths and joint angles.
    /// @param L1 Length of the first link (default = 0.3)
    /// @param L2 Length of the second link (default = 0.3)
    /// @param L3 Length of the third link (default = 0.1)
    ///
    /// Joint angles are initialized to zero by default.
    RRRManipulator(double L1 = 0.3, double L2 = 0.3, double L3 = 0.1);

    /// @brief Set new link lengths for the manipulator.
    /// @param l1 Length of link 1
    /// @param l2 Length of link 2
    /// @param l3 Length of link 3
    void setLinkLengths(double l1, double l2, double l3);

    /// @brief Set the joint angles of the manipulator (in radians).
    /// @param t1 Angle of joint 1
    /// @param t2 Angle of joint 2
    /// @param t3 Angle of joint 3
    void setJointAngles(double t1, double t2, double t3);

    /// @brief Get the length of link 1.
    /// @return Length of L1
    double getL1() const;

    /// @brief Get the length of link 2.
    /// @return Length of L2
    double getL2() const;

    /// @brief Get the length of link 3.
    /// @return Length of L3
    double getL3() const;

    /// @brief Get the current angle of joint 1.
    /// @return Angle theta1 in radians
    double getTheta1() const;

    /// @brief Get the current angle of joint 2.
    /// @return Angle theta2 in radians
    double getTheta2() const;

    /// @brief Get the current angle of joint 3.
    /// @return Angle theta3 in radians
    double getTheta3() const;
};

#endif // RRRMANIPULATOR_HPP
