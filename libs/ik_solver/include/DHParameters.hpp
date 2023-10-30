#pragma once

/**
 * @file DHParameters.cpp
 *
 * @brief Represents Denavit-Hartenberg parameters for a robotic link.
 *
 * The Denavit-Hartenberg parameters are used to describe the geometry and joint
 * configurations of robotic manipulators. This class provides a convenient way
 * to store and access these parameters.
 *
 * @author Krishna Rajesh Hundekari
 */

#include <iostream>
#include <vector>

class DHParameters {
 private:
  // double linkLength; /**< The length of the link. */ 
  // double linkTwist;  /**< The twist angle of the link. */
  // double linkOffset; /**< The offset along the link's z-axis. */
  // double jointAngle; /**< The joint angle, which can be considered as a rotation
  //                       about the z-axis. */


  double d;
  double alpha;
  double a;
  double theta;

 public:

  /**
   * @brief Parameterized constructor to initialize the DH parameters.
   * @param d The length of the link.
   * @param alpha The twist angle of the link.
   * @param a The offset along the link's z-axis.
   */
  DHParameters(double length, double twist, double offset)
      : d(length),
        alpha(twist),
        a(offset) {}

  /**
   * @brief Default Constructor for DHParameters
   * 
   */
  DHParameters() {}

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
