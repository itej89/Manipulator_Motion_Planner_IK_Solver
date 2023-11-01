#include <iostream>

#include "../../thirdparty_libraries/matplotlib-cpp/matplotlibcpp.h"

#include "robot.hpp"


namespace plt = matplotlibcpp;


int main() {

    Robot trial_1;

    // Compute the IK trajectory for the input vector.
    Eigen::VectorXd config(7);
    config << 0, 0, 0, -3.14/2, 0, 3.14, 0;

    double radius = 0.05;
    double theta_dot = 2*M_PI/5;
    double delta_theta = 0.01;
    double delta_t = (5/(2*M_PI))*delta_theta;

    for (double theta = 0; theta < 2 * M_PI; theta = theta + delta_theta) {
      Eigen::VectorXd V(6);
      V << 0, radius * theta_dot * cos(theta), -1 * radius * theta_dot * sin(theta), 0, 0, 0;
      
      Eigen::VectorXd  q_dot = trial_1.ComputeIK(V, config);
      config += q_dot * delta_t;
    }


    auto CIRCLE_POINTS_Final = trial_1.getCirclePoints();

    std::vector<double> y;
    std::vector<double>  z;
    for(int i=0; i< CIRCLE_POINTS_Final.size(); i++) {
        y.push_back(CIRCLE_POINTS_Final[i].first);
        z.push_back(CIRCLE_POINTS_Final[i].second);
    }
    
    plt::plot(y, z);
    plt::xlabel("y");
    plt::ylabel("z");
    plt::title("Robotic Arm endeffector pose for tracking a circle of radius 0.05");
    plt::show();
}