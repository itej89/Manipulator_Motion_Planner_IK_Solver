#pragma once
/**
 * @file robot.cpp
 *
 * @brief Represents a robot with Denavit-Hartenberg parameters.
 *
 * This class extends the DHParameters class to create a robot with additional
 * properties and methods.
 *
 * @author Krishna Rajesh Hundekari

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


#include "DHParameters.hpp"
#include <iostream>
#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include <map>
#include <algorithm>
#include <Eigen/Dense>


class Robot {
public:

    /**
     * @brief Constructor to initialize the robot with DH parameters and custom properties.
     *
     * @param links The number of links in the robot.
     * @param DOF The degrees of freedom of the robot.
     * @param vel_param A vector containing velocity profile information.
     * @param length The length of the link.
     * @param twist The twist angle of the link.
     * @param offset The offset along the link's z-axis.
     * @param angle The joint angle, which can be considered as a rotation about the z-axis.
     */
    Robot();

    /**
     * @brief Calculate the transformation matrices for each joint in the robot.
     *
     * @param config A vector representing the joint configuration.
     * @return A vector of transformation matrices for each joint.
     */
    std::vector<Eigen::MatrixXd> CalculateTransformations(const Eigen::VectorXd& config);

    /**
     * @brief Compute the Jacobian matrix for the robot.
     *
     * @param config A vector representing the joint configuration.
     * @return The Jacobian matrix.
     */
    Eigen::MatrixXd GetJacobian(const Eigen::VectorXd& config); 

    /**
     * @brief Compute inverse kinematics for the robot.
     *
     * @param theta Initial angle.
     * @param theta_dot Angular velocity.
     * @param radius Radius.
     * @param delta_theta Incremental angle.
     * @param delta_t Time step.
     * @param config A vector representing the current joint configuration.
     */
    Eigen::VectorXd ComputeIK(Eigen::VectorXd V, Eigen::VectorXd config);

      /**
      * @brief Get the vector of points representing the end-effector's circular trajectory.
      * 
      * This function returns a constant reference to the vector of points that represent
      * the end-effector's positions along a circular trajectory.
      *
      * @return A constant reference to the vector of (x, y) points.
      */
      const std::vector<std::pair<double, double>>& getCirclePoints() const;
  
private:
    std::map<std::string, DHParameters> Robot_DHParam; /**< Created a map with strings as keys & object of 
    DH parameter class as its corresponding value. */
    std::vector<std::pair<double, double>> CIRCLE_POINTS;/**< Vector of Circle points to plot */
    int num_link;           /**< The number of links in the robot. */
    int degrees_of_freedom; /**< The degrees of freedom of the robot. */
    std::vector<double> velocity_profile; /**< A vector containing velocity profile information. */
};

