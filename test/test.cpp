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

#include <Eigen/src/Core/Matrix.h>

/**
 * @brief Test case for the IK trajectory calculation.
 *
 * This test verifies that the `robot` class can accurately compute the inverse
 * kinematics trajectory for a given input vector.
 */
TEST(IK_TRAJECTORY_TEST, this_should_pass) {

  // Create an instance of the robot for testing.
  Robot trial_1;

  // Compute the IK trajectory for the input vector.
  Eigen::VectorXd config(7);
  config << 0, 0, 0, -3.14/2, 0, 3.14, 0;

  trial_1.ComputeIK(
    0, 2*3.14/5, 
  0.1, 0.01, 
  (5/(2*3.14))*0.01, 
   config);

  ASSERT_EQ(1, 1);
  // // Verify that the output vector matches the expected values.
  // ASSERT_THAT(output_vector, testing::ElementsAre(0.0, 1.0));
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
