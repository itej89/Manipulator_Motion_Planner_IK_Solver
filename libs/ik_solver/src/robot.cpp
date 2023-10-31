/**
 * @file robot.cpp
 *
 * @brief Represents a robot with Denavit-Hartenberg parameters.
 *
 * This class extends the DHParameters class to create a robot with additional
 * properties and methods.
 *
 * @author Krishna Rajesh Hundekari
 */

#include "robot.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>

//  This function computes the inverse kinematics for the robot based on the
//  given target configuration and returns the result as a vector.
  void robot::computeIK(double thetaf, double theta_dot, double radius, 
  double detlta_theta, double delta_t,  Eigen::VectorXd config ) {

    for(double theta=0; theta<2*M_PI; theta=theta+detlta_theta) {

      std::cout << ("computeIK looping-------------------") << theta << "\n";

      Eigen::VectorXd V(6);

      V <<  0, radius*theta_dot*cos(theta), 
      -1*radius*theta_dot*sin(theta), 0, 0, 0 ;


      // std::cout << ("Computing Jacobian") << "\n";

      MatrixXd J = GetJacobian(config);

      std::cout << "Computed config ";
      for (int i=0; i < config.size(); i++) {
        std::cout << config[i] << " ";
      }
      std::cout << " \n";


      std::cout << "Computed Jacobian : \n";
      std::cout << J <<std::endl;

      MatrixXd J_inv = J.completeOrthogonalDecomposition()
                                            .pseudoInverse();


      // std::cout << "Computed J_inv : " << J_inv.rows() << "; " << J_inv.cols() << "\n";

      Eigen::VectorXd q_dot = J_inv * V;

      config += q_dot * delta_t;
      
    

     
      std::cout << ("------------------------") << "\n";

    }

    std::for_each(CIRCLE_POINTS.begin(), 
    CIRCLE_POINTS.end(), [](std::pair<double, double>& point) {
      std::cout << point.first << " " << point.second << "\n";
    });

  }
