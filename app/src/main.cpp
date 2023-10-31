#include <iostream>

#include "../../thirdparty_libraries/matplotlib-cpp/matplotlibcpp.h"

#include "robot.hpp"


namespace plt = matplotlibcpp;


int main() {

  Robot trial_1;

  // Compute the IK trajectory for the input vector.
  Eigen::VectorXd config(7);
  config << 0, 0, 0, -3.14/2, 0, 3.14, 0;

  trial_1.ComputeIK(
    0, 2*3.14/5, 
  0.1, 0.01, 
  (5/(2*3.14))*0.01, 
   config);
  
    auto CIRCLE_POINTS_Final = trial_1.getCirclePoints();

    std::vector<double> y;
    std::vector<double>  z;
    for(int i=0; i< CIRCLE_POINTS_Final.size(); i++) {
        y.push_back(CIRCLE_POINTS_Final[i].first);
        z.push_back(CIRCLE_POINTS_Final[i].second);
    }
    
    plt::plot(y, z);
    plt::show();
}