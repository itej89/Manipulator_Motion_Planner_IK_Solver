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

// Define the member function to set the length of the link
void DHParameters::setLinkLength(double length) {
  d = length;  // Set the link length to the provided value
}

// Define the member function to get the twist angle of the link
double DHParameters::getLinkTwist() const {
  return alpha;  // Return the value of the twist angle
}

// Define the member function to set the twist angle of the link
void DHParameters::setLinkTwist(double twist) {
  alpha = twist;  // Set the twist angle to the provided value
}

// Define the member function to get the offset along the link's z-axis
double DHParameters::getLinkOffset() const {
  return a;  // Return the value of the link offset
}

// Define the member function to set the offset along the link's z-axis
void DHParameters::setLinkOffset(double offset) {
  a = offset;  // Set the link offset to the provided value
}

// Define the member function to get the joint angle
double DHParameters::getJointAngle() const {
  return theta;  // Return the value of the joint angle
}

// Define the member function to set the joint angle
void DHParameters::setJointAngle(double angle) {
  theta = angle;  // Set the joint angle to the provided value
}

// Define the member function to calculate the transformation matrix based on DH
// parameters
std::vector<double> DHParameters::transformationMatrixCal(
    std::vector<double> matrix) {
  // Placeholder implementation, should contain actual logic for transformation
  // matrix calculation
  return {1.0,
          0.0};  // Placeholder return value, replace with actual calculation
}
