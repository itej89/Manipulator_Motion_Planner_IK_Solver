#pragma once 


#include <iostream>
#include <math.h> 

#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include <Eigen/Dense>

using Eigen::MatrixXd;

#include "DHParameters.hpp"

#define PI 3.14

/**
 * @file robot.hpp
 *
 * @brief Represents a robot with Denavit-Hartenberg parameters.
 *
 * This class extends the DHParameters class to create a robot with additional
 * properties and methods.
 *
 * @author Krishna Rajesh Hundekari
 */

class robot{
 public:
  /**
   * @brief Constructor to initialize the robot with DH parameters and custom
   * properties.
   * @param links The number of links in the robot.
   * @param DOF The degrees of freedom of the robot.
   * @param vel_param A vector containing velocity profile information.
   * @param length The length of the link.
   * @param twist The twist angle of the link.
   * @param offset The offset along the link's z-axis.
   * @param angle The joint angle, which can be considered as a rotation about
   * the z-axis.
   */

  std::map<std::string,DHParameters> Robot_DHParam;

  robot() {

      Robot_DHParam["J1"] = DHParameters(0.333, PI/2, 0);
      Robot_DHParam["J2"] = DHParameters(0.0,   -PI/2, 0);
      Robot_DHParam["J3"] = DHParameters(0.316, PI/2, 0.088);
      Robot_DHParam["J4"] = DHParameters(0.0,   -PI/2, -0.088);
      Robot_DHParam["J5"] = DHParameters(0.384, PI/2, 0);
      Robot_DHParam["J6"] = DHParameters(0.0,   -PI/2, 0.088);
      Robot_DHParam["J7"] = DHParameters(0.107, 0, 0);

  }


  MatrixXd GetJacobian(std::vector<double> config) {

    std::vector<MatrixXd> TF_list;
    MatrixXd TF_FixedToEndEffector = MatrixXd::Identity(4,4);


    for (int i=0; i< config.size(); i++) {

      std::string Joint = 'J'+std::to_string(i);

      double d = Robot_DHParam[Joint].getLinkLength();
      double alpha = Robot_DHParam[Joint].getLinkTwist();
      double a = Robot_DHParam[Joint].getLinkOffset();
      double theta = config[i];

      MatrixXd TF {
        { cos(theta), -1*sin(theta)*cos(alpha),    sin(theta)*sin(alpha), a*cos(theta) },
        { sin(theta),    cos(theta)*cos(alpha), -1*cos(theta)*sin(alpha), a*sin(theta) },
        { 0, sin(alpha), cos(alpha), d },
        { 0, 0, 0, 1}
      };

      TF_FixedToEndEffector = TF_FixedToEndEffector*TF;
      MatrixXd TF_FixedToJoint = TF_FixedToEndEffector;
      TF_list.push_back(TF_FixedToJoint);

    }

    // TF_list.insert(0, MatrixXd::Identity(4,4));
    MatrixXd Jacobian;


    return Jacobian;
  }



  /**
   * @brief Compute inverse kinematics for the robot.
   * @param target A vector representing the target configuration.
   * @return A vector representing the computed inverse kinematics solution.
   *
   * This function computes the inverse kinematics for the robot based on the
   * given target configuration and returns the result as a vector.
   */
  std::vector<double> computeIK(const std::vector<double>& target);

 private:
  int num_link;           /**< The number of links in the robot. */
  int degrees_of_freedom; /**< The degrees of freedom of the robot. */
  std::vector<double> velocity_profile; /**< A vector containing velocity
                                           profile information. */
};
