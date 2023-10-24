#include <iostream>
#include <vector>
#include "DHParameters.hpp"

/**
 * @file robot.hpp
 *
 * @brief Represents a robot with Denavit-Hartenberg parameters.
 *
 * This class extends the DHParameters class to create a robot with additional
 * properties and methods.
 *
 * @author Krishna Rajesh Hundekari
 */
 
class robot : public DHParameters {
public:
    /**
     * @brief Constructor to initialize the robot with DH parameters and custom properties.
     * @param links The number of links in the robot.
     * @param DOF The degrees of freedom of the robot.
     * @param vel_param A vector containing velocity profile information.
     * @param length The length of the link.
     * @param twist The twist angle of the link.
     * @param offset The offset along the link's z-axis.
     * @param angle The joint angle, which can be considered as a rotation about the z-axis.
     */
    robot(int links, int DOF, const std::vector<double>& vel_param,
          double length, double twist, double offset, double angle)
        : DHParameters(length, twist, offset, angle),
          num_link(links),
          degrees_of_freedom(DOF),
          velocity_profile(vel_param) {}

    /**
     * @brief Compute inverse kinematics for the robot.
     * @param target A vector representing the target configuration.
     * @return A vector representing the computed inverse kinematics solution.
     *
     * This function computes the inverse kinematics for the robot based on the
     * given target configuration and returns the result as a vector.
     */
    std::vector<double> computeIK(const std::vector<double>& target);

    
private:
    int num_link;              /**< The number of links in the robot. */
    int degrees_of_freedom;   /**< The degrees of freedom of the robot. */
    std::vector<double> velocity_profile;  /**< A vector containing velocity profile information. */
};
