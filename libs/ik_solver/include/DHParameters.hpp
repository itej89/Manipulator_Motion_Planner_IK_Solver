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
  double d;
  double alpha;
  double a;
  double theta;

 public:

  friend class Robot;

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
   * @brief Get the twist angle of the link.
   * @return The twist angle of the link.
   */
  double getLinkTwist() const;

  /**
   * @brief Get the offset along the link's z-axis.
   * @return The offset along the link's z-axis.
   */
  double getLinkOffset() const;

};
