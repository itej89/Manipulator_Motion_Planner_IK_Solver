#pragma once 


#include <iostream>
#include <math.h> 

#include <string>
#include <utility>
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
  std::vector<std::pair<double, double>> CIRCLE_POINTS;

  robot() {

      Robot_DHParam["J1"] = DHParameters(0.333, M_PI/2, 0);
      Robot_DHParam["J2"] = DHParameters(0.0,   -M_PI/2, 0);
      Robot_DHParam["J3"] = DHParameters(0.316, M_PI/2, 0.088);
      Robot_DHParam["J4"] = DHParameters(0.0,   -M_PI/2, -0.088);
      Robot_DHParam["J5"] = DHParameters(0.384, M_PI/2, 0);
      Robot_DHParam["J6"] = DHParameters(0.0,   -M_PI/2, 0.088);
      Robot_DHParam["J7"] = DHParameters(-0.107, 0, 0);
  }


  MatrixXd GetJacobian(Eigen::VectorXd config) {

    std::vector<MatrixXd> TF_list;
    MatrixXd TF_FixedToEndEffector = MatrixXd::Identity(4,4);


    for (int i=1; i<= config.size(); i++) {

      std::string Joint = 'J'+std::to_string(i);

      double d = Robot_DHParam[Joint].getLinkLength();
      double alpha = Robot_DHParam[Joint].getLinkTwist();
      double a = Robot_DHParam[Joint].getLinkOffset();
      double theta = config[i-1];

      MatrixXd TF {
        { cos(theta), -1*sin(theta)*cos(alpha),    sin(theta)*sin(alpha), a*cos(theta) },
        { sin(theta),    cos(theta)*cos(alpha), -1*cos(theta)*sin(alpha), a*sin(theta) },
        { 0, sin(alpha), cos(alpha), d },
        { 0, 0, 0, 1}
      };

      TF_FixedToEndEffector = TF_FixedToEndEffector*TF;\
      MatrixXd TF_FixedToJoint = TF_FixedToEndEffector;

      TF_list.push_back(TF_FixedToJoint);

    }

    CIRCLE_POINTS.push_back(std::pair<double, double>(
      TF_FixedToEndEffector(1,3),
      TF_FixedToEndEffector(2,3)));

    TF_list.insert(TF_list.begin(), MatrixXd::Identity(4,4));
    MatrixXd Jacobian (6, Robot_DHParam.size());

    for (int i=1; i<= Robot_DHParam.size(); i++) {
      
      std::string Joint = 'J'+std::to_string(i);

      Eigen::Vector3d Zi_1(TF_list[i-1](0, 2),TF_list[i-1](1, 2),TF_list[i-1](2, 2));
      Eigen::Vector3d Oi_1(TF_list[i-1](0, 3),TF_list[i-1](1, 3),TF_list[i-1](2, 3));
      Eigen::Vector3d On(TF_list[TF_list.size()-1](0, 3),TF_list[TF_list.size()-1](1, 3),TF_list[TF_list.size()-1](2, 3));

      Eigen::Vector3d O_diff = On - Oi_1;
            
      Eigen::Vector3d  Ji_linear = Zi_1.cross(O_diff);
      Eigen::Vector3d  Ji_angular (TF_list[i](0, 2),TF_list[i](1, 2),TF_list[i](2, 2));

      Eigen::VectorXd  Ji(6);
      Ji <<  Ji_linear[0], Ji_linear[1], Ji_linear[2],
        Ji_angular[0],   Ji_angular[1],    Ji_angular[2]; 

      Jacobian.col(i-1) =  Ji;
    }


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
  void computeIK(double theta, double theta_dot, double radius, 
  double detlta_theta, double delta_t, Eigen::VectorXd config );

 private:
  int num_link;           /**< The number of links in the robot. */
  int degrees_of_freedom; /**< The degrees of freedom of the robot. */
  std::vector<double> velocity_profile; /**< A vector containing velocity
                                           profile information. */
};
