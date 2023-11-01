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
#include <algorithm>
#include "robot.hpp"
#include "DHParameters.hpp"
#include <Eigen/src/Core/Matrix.h>



/**
 * @brief Constructor to initialize the robot with DH parameters and custom properties.
 */
Robot::Robot() {
    Robot_DHParam["J1"] = DHParameters(0.333, M_PI/2, 0);
    Robot_DHParam["J2"] = DHParameters(0.0,   -M_PI/2, 0);
    Robot_DHParam["J3"] = DHParameters(0.316, M_PI/2, 0.088);
    Robot_DHParam["J4"] = DHParameters(0.0,   -M_PI/2, -0.088);
    Robot_DHParam["J5"] = DHParameters(0.384, M_PI/2, 0);
    Robot_DHParam["J6"] = DHParameters(0.0,   -M_PI/2, 0.088);
    Robot_DHParam["J7"] = DHParameters(-0.107, 0, 0);
}

/**
 * @brief Calculate the transformation matrices for each joint in the robot.
 *
 * @param config A vector representing the joint configuration.
 * @return A vector of transformation matrices for each joint.
 */
std::vector<Eigen::MatrixXd> Robot::CalculateTransformations(
    const Eigen::VectorXd& config) {
    std::vector<Eigen::MatrixXd> TF_list;
    Eigen::MatrixXd TF_FixedToEndEffector = Eigen::MatrixXd::Identity(4, 4);

    for (int i = 1; i <= config.size(); i++) {
        std::string Joint = 'J' + std::to_string(i);

        double d = Robot_DHParam[Joint].getLinkLength();
        double alpha = Robot_DHParam[Joint].getLinkTwist();
        double a = Robot_DHParam[Joint].getLinkOffset();
        double theta = config[i - 1];

        Eigen::MatrixXd TF{
            { cos(theta), -1 * sin(theta) * cos(alpha),
            sin(theta) * sin(alpha), a * cos(theta) },
            { sin(theta), cos(theta) * cos(alpha),
            -1 * cos(theta) * sin(alpha), a * sin(theta) },
            { 0, sin(alpha), cos(alpha), d },
            { 0, 0, 0, 1 }
        };

        TF_FixedToEndEffector = TF_FixedToEndEffector * TF;
        Eigen::MatrixXd TF_FixedToJoint = TF_FixedToEndEffector;

        TF_list.push_back(TF_FixedToJoint);
    }

    CIRCLE_POINTS.push_back(std::pair<double, double>(
        TF_FixedToEndEffector(1, 3),
        TF_FixedToEndEffector(2, 3)));

    TF_list.insert(TF_list.begin(), Eigen::MatrixXd::Identity(4, 4));

    return TF_list;
}

/**
 * @brief Compute the Jacobian matrix for the robot.
 *
 * @param config A vector representing the joint configuration.
 * @return The Jacobian matrix.
 */
Eigen::MatrixXd Robot::GetJacobian(const Eigen::VectorXd& config) {
    std::vector<Eigen::MatrixXd> TF_list = CalculateTransformations(config);
    Eigen::MatrixXd Jacobian(6, Robot_DHParam.size());

    for (int i = 1; i <= Robot_DHParam.size(); i++) {
        std::string Joint = 'J' + std::to_string(i);

        Eigen::Vector3d Zi_1(TF_list[i - 1](0, 2),
         TF_list[i - 1](1, 2), TF_list[i - 1](2, 2));
        Eigen::Vector3d Oi_1(TF_list[i - 1](0, 3),
         TF_list[i - 1](1, 3), TF_list[i - 1](2, 3));
        Eigen::Vector3d On(TF_list[TF_list.size() - 1](0, 3),
        TF_list[TF_list.size() - 1](1, 3), TF_list[TF_list.size() - 1](2, 3));

        Eigen::Vector3d O_diff = On - Oi_1;

        Eigen::Vector3d Ji_linear = Zi_1.cross(O_diff);
        Eigen::Vector3d Ji_angular(TF_list[i](0, 2),
         TF_list[i](1, 2), TF_list[i](2, 2));

        Eigen::VectorXd Ji(6);
        Ji << Ji_linear[0], Ji_linear[1], Ji_linear[2],
         Ji_angular[0], Ji_angular[1], Ji_angular[2];

        Jacobian.col(i - 1) = Ji;
    }
    return Jacobian;
}

/**
 * @brief Compute inverse kinematics for the robot.
 *
 * @param thetaf Initial angle.
 * @param theta_dot Angular velocity.
 * @param radius Radius.
 * @param delta_theta Incremental angle.
 * @param delta_t Time step.
 * @param config A vector representing the current joint configuration.
 */
// void Robot::ComputeIK(double thetaf, double theta_dot, double radius,
//     double delta_theta, double delta_t, Eigen::VectorXd config) {

Eigen::VectorXd Robot::ComputeIK(Eigen::VectorXd V, Eigen::VectorXd config) {
        Eigen::MatrixXd J = GetJacobian(config);

        Eigen::MatrixXd J_inv =
        J.completeOrthogonalDecomposition().pseudoInverse();

        Eigen::VectorXd q_dot = J_inv * V;

        return q_dot;
}

/**
      * @brief Get the vector of points representing the end-effector's circular trajectory.
      * @return A constant reference to the vector of (x, y) points.
      */
const std::vector<std::pair<double, double>>&
                Robot::getCirclePoints() const {
    return CIRCLE_POINTS;
  }

