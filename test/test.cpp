/**
 * @file my_robot_test.cpp
 *
 * @brief This file contains unit tests for the inverse kinematics (IK)
 * trajectory calculation of a robot using Google Test and Google Mock.
 *
 * These tests verify that the `robot` class can correctly compute the inverse
 * kinematics trajectory for a given input vector.
 *
 * @author Abrarudiin Syed

 * Apache License Version 2.0, January 2004

 * Licensed to the Apache Software Foundation (ASF) under one
* or more contributor license agreements.  See the NOTICE file
* distributed with this work for additional information
* regarding copyright ownership.  The ASF licenses this file
* to you under the Apache License, Version 2.0 (the
* "License"); you may not use this file except in compliance
* with the License.  You may obtain a copy of the License at
* 
*   http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing,
* software distributed under the License is distributed on an
* "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
* KIND, either express or implied.  See the License for the
* specific language governing permissions and limitations
* under the License.
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
TEST(IK_TRAJECTORY_TEST, circle_radius_one_check) {
  // Create an instance of the robot for testing.
  Robot trial_1;

  // Compute the IK trajectory for the input vector.
  Eigen::VectorXd config(7);
  config << 0, 0, 0, -3.14/2, 0, 3.14, 0;

    /**
     * @brief 
     * 
     */
    double radius = 0.1;
    double theta_dot = 2*M_PI/5;
    double delta_theta = 0.01;
    double delta_t = (5/(2*M_PI))*delta_theta;

    std::pair<double, double> theta_0;
    std::pair<double, double> theta_pi;


    for (double theta = 0; theta <=  M_PI; theta = theta + delta_theta) {
      Eigen::VectorXd V(6);
      V << 0, radius * theta_dot * cos(theta),
       -1 * radius * theta_dot * sin(theta), 0, 0, 0;
      Eigen::VectorXd  q_dot = trial_1.ComputeIK(V, config);

      if (theta == 0)
        theta_0 = trial_1.getCirclePoints().back();

      config += q_dot * delta_t;
    }

    theta_pi = trial_1.getCirclePoints().back();

    double computed_diameter = sqrt(pow(theta_0.first-theta_pi.first, 2) +
     pow(theta_0.second-theta_pi.second, 2) );

    double computed_radius = computed_diameter/2;\

    EXPECT_THAT(computed_radius,
     ::testing::AllOf(::testing::Ge(radius-0.01), ::testing::Le(radius+0.01)));
}


/**
 * @brief Test case for the IK trajectory calculation.
 *
 * This test verifies that the `robot` class can accurately compute the inverse
 * kinematics trajectory for a given input vector.
 */
TEST(IK_TRAJECTORY_TEST, circle_radius_two_check) {
  // Create an instance of the robot for testing.
  Robot trial_1;

  // Compute the IK trajectory for the input vector.
  Eigen::VectorXd config(7);
  config << 0, 0, 0, -3.14/2, 0, 3.14, 0;

    /**
     * @brief 
     * 
     */
    double radius = 0.15;
    double theta_dot = 2*M_PI/5;
    double delta_theta = 0.01;
    double delta_t = (5/(2*M_PI))*delta_theta;

    std::pair<double, double> theta_0;
    std::pair<double, double> theta_pi;


    for (double theta = 0; theta <=  M_PI; theta = theta + delta_theta) {
      Eigen::VectorXd V(6);
      V << 0, radius * theta_dot * cos(theta),
       -1 * radius * theta_dot * sin(theta), 0, 0, 0;

      Eigen::VectorXd  q_dot = trial_1.ComputeIK(V, config);

      if (theta == 0)
        theta_0 = trial_1.getCirclePoints().back();

      config += q_dot * delta_t;
    }

    theta_pi = trial_1.getCirclePoints().back();

    double computed_diameter = sqrt(pow(theta_0.first-theta_pi.first, 2) +
     pow(theta_0.second-theta_pi.second, 2) );

    double computed_radius = computed_diameter/2;\

    EXPECT_THAT(computed_radius,
     ::testing::AllOf(::testing::Ge(radius-0.01), ::testing::Le(radius+0.01)));
}


/**
 * @brief Test case for the IK trajectory calculation.
 *
 * This test verifies that the `robot` class can accurately compute the inverse
 * kinematics trajectory for a given input vector.
 */
TEST(IK_TRAJECTORY_TEST, circle_radius_three_check) {
  // Create an instance of the robot for testing.
  Robot trial_1;

  // Compute the IK trajectory for the input vector.
  Eigen::VectorXd config(7);
  config << 0, 0, 0, -3.14/2, 0, 3.14, 0;

    /**
     * @brief 
     * 
     */
    double radius = 0.05;
    double theta_dot = 2*M_PI/5;
    double delta_theta = 0.01;
    double delta_t = (5/(2*M_PI))*delta_theta;

    std::pair<double, double> theta_0;
    std::pair<double, double> theta_pi;


    for (double theta = 0; theta <=  M_PI; theta = theta + delta_theta) {
      Eigen::VectorXd V(6);
      V << 0, radius * theta_dot * cos(theta), -1 * radius
       * theta_dot * sin(theta), 0, 0, 0;

      Eigen::VectorXd  q_dot = trial_1.ComputeIK(V, config);

      if (theta == 0)
        theta_0 = trial_1.getCirclePoints().back();

      config += q_dot * delta_t;
    }

    theta_pi = trial_1.getCirclePoints().back();

    double computed_diameter = sqrt(pow(theta_0.first-theta_pi.first, 2) +
     pow(theta_0.second-theta_pi.second, 2) );

    double computed_radius = computed_diameter/2;\

    EXPECT_THAT(computed_radius, ::testing::AllOf(::testing::Ge(radius-0.01),
    ::testing::Le(radius+0.01)));
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
