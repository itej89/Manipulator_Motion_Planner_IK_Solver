/**
 * @file main.cpp
 *
 * @brief Demo application for a robotic arm tracking a circular trajectory.
 *
 * This application demonstrates the use of the robotic arm class to compute the
 * inverse kinematic (IK) trajectory for tracking a circular path and visualizes
 * the end-effector's pose.
 *
 * @author Tej, Krishna, Abraruddin 

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

#include <iostream>
#include "../../thirdparty_libraries/matplotlib-cpp/matplotlibcpp.h"
#include "robot.hpp"

namespace plt = matplotlibcpp;

/**
 * @brief The main function to execute the robotic arm's circular trajectory.
 *
 * This function initializes a robotic arm, computes its inverse kinematic (IK) trajectory
 * to follow a circular path, and then plots the end-effector's pose as it tracks the circle.
 *
 * @return 0 upon successful execution.
 */
int main() {
    // Create a robotic arm instance.
    Robot trial_1;

    // Define the initial robot configuration.
    Eigen::VectorXd config(7);
    config << 0, 0, 0, -3.14/2, 0, 3.14, 0;

    // Define circular trajectory parameters.
    double radius = 0.05;
    double theta_dot = 2 * M_PI / 5;
    double delta_theta = 0.01;
    double delta_t = (5 / (2 * M_PI)) * delta_theta;

    // Generate the IK trajectory to track the circular path.
    for (double theta = 0; theta < 2 * M_PI; theta += delta_theta) {
        Eigen::VectorXd V(6);
        V << 0, radius * theta_dot *
        cos(theta), -1 * radius * theta_dot * sin(theta), 0, 0, 0;
        Eigen::VectorXd q_dot = trial_1.ComputeIK(V, config);
        config += q_dot * delta_t;
    }

    // Get the final circular trajectory points.
    const auto CIRCLE_POINTS_Final = trial_1.getCirclePoints();

    // Extract the y and z components of the circular points.
    std::vector<double> y;
    std::vector<double> z;
    for (int i = 0; i < CIRCLE_POINTS_Final.size(); i++) {
        y.push_back(CIRCLE_POINTS_Final[i].first);
        z.push_back(CIRCLE_POINTS_Final[i].second);
    }
    // Plot the end-effector's pose as it tracks the circle.
    plt::plot(y, z);
    plt::xlabel("y");
    plt::ylabel("z");
    plt::title(
      "Robotic Arm End-Effector Pose for Tracking a Circle of Radius 0.05");
    plt::show();
    return 0;
}
