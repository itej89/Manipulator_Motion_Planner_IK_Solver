/**
 * @file my_robot_test.cpp
 *
 * @brief This file contains unit tests for the inverse kinematics (IK)
 * trajectory calculation of a robot using Google Test and Google Mock.
 *
 * These tests verify that the `robot` class can correctly compute the inverse
 * kinematics trajectory for a given input vector.
 *
 * @author [Abrarudiin Syed]
 */

#include <gtest/gtest.h>

#include <vector>

#include "DHParameters.hpp"
#include "gmock/gmock.h"
#include "robot.hpp"

/**
 * @brief Test case for the IK trajectory calculation.
 *
 * This test verifies that the `robot` class can accurately compute the inverse
 * kinematics trajectory for a given input vector.
 */
TEST(IK_TRAJECTORY_TEST, this_should_pass) {
  // Define an input vector representing a position in 3D space.
  std::vector<double> input_vector = {1.0, 3.0, 2.5};

  // Create an instance of the robot for testing.
  robot trial_1;

  // Compute the IK trajectory for the input vector.
  std::vector<double> output_vector = trial_1.computeIK(input_vector);

  // Verify that the output vector matches the expected values.
  ASSERT_THAT(output_vector, testing::ElementsAre(5, 10, 15));
}

/**
 * @brief The main function for running the tests.
 *
 * This function sets up and runs the unit tests defined in this file.
 *
 * @param argc The number of command-line arguments.
 * @param argv An array of command-line argument strings.
 * @return The exit code of the test program.
 */
int main(int argc, char** argv) {
  // Initialize Google Test and Google Mock.
  ::testing::InitGoogleTest(&argc, argv);
  // Run the tests.
  return RUN_ALL_TESTS();
}
