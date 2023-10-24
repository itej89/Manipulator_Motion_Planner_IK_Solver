#pragma once
#include <iostream>
#include <vector>

/**
 * @file DHParameters.hpp
 *
 * @brief Represents Denavit-Hartenberg parameters for a robotic link.
 *
 * The Denavit-Hartenberg parameters are used to describe the geometry and joint
 * configurations of robotic manipulators. This class provides a convenient way
 * to store and access these parameters.
 *
 * @author Krishna Rajesh Hundekari
 */

class DHParameters {
private:
    double linkLength;   /**< The length of the link. */
    double linkTwist;    /**< The twist angle of the link. */
    double linkOffset;   /**< The offset along the link's z-axis. */
    double jointAngle;   /**< The joint angle, which can be considered as a rotation about the z-axis. */

public:
    /**
     * @brief Default constructor. Initializes all parameters to zero.
     */
    DHParameters() : linkLength(0.0), linkTwist(0.0), linkOffset(0.0), jointAngle(0.0) {}

    /**
     * @brief Parameterized constructor to initialize the DH parameters.
     * @param length The length of the link.
     * @param twist The twist angle of the link.
     * @param offset The offset along the link's z-axis.
     * @param angle The joint angle, which can be considered as a rotation about the z-axis.
     */
    DHParameters(double length, double twist, double offset, double angle)
        : linkLength(length), linkTwist(twist), linkOffset(offset), jointAngle(angle) {}

    /**
     * @brief Get the length of the link.
     * @return The length of the link.
     */
    double getLinkLength() const;

    /**
     * @brief Set the length of the link.
     * @param length The length of the link.
     */
    void setLinkLength(double length);

    /**
     * @brief Get the twist angle of the link.
     * @return The twist angle of the link.
     */
    double getLinkTwist() const;

    /**
     * @brief Set the twist angle of the link.
     * @param twist The twist angle of the link.
     */
    void setLinkTwist(double twist);

    /**
     * @brief Get the offset along the link's z-axis.
     * @return The offset along the link's z-axis.
     */
    double getLinkOffset() const;

    /**
     * @brief Set the offset along the link's z-axis.
     * @param offset The offset along the link's z-axis.
     */
    void setLinkOffset(double offset);

    /**
     * @brief Get the joint angle, which is a rotation about the z-axis.
     * @return The joint angle.
     */
    double getJointAngle() const;

    /**
     * @brief Set the joint angle, which is a rotation about the z-axis.
     * @param angle The joint angle.
     */
    void setJointAngle(double angle);

    /**
     * @brief Calculate the transformation matrix based on the DH parameters.
     * @param matrix A vector containing the transformation matrix.
     * @return A vector representing the resulting transformation matrix.
     *
     * This function calculates and returns a transformation matrix based on the
     * DH parameters. The resulting matrix is a vector of values.
     */
    std::vector<double> transformationMatrixCal(std::vector<double> matrix);
};
