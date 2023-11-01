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

#include "DHParameters.hpp"

// Define the member function to get the length of the link
double DHParameters::getLinkLength() const {
  return d;  // Return the value of the link length
}

// Define the member function to get the twist angle of the link
double DHParameters::getLinkTwist() const {
  return alpha;  // Return the value of the twist angle
}

// Define the member function to get the offset along the link's z-axis
double DHParameters::getLinkOffset() const {
  return a;  // Return the value of the link offset
}
