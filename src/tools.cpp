#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  // ... your code here
    if (estimations.size() <= 0) {
        std::cout << "Calculate CalculateRMSE - Division by zero ERROR" << std::endl;
        return rmse;
    }
    else if (estimations.size() != ground_truth.size()) {
        std::cout << "Calculate CalculateRMSE - Estimation size and ground_truth size differ ERROR" << std::endl;
        return rmse;
    }
  //accumulate squared residuals
  float error_sum = 0.0;
  VectorXd e(4);
  VectorXd g(4);
  for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        e = estimations[i];
        g = ground_truth[i];
        VectorXd residual = e - g;
      for(int i=0; i < rmse.size(); ++i){
          rmse(i) += residual(i) * residual(i);
      }
  }

  //calculate the mean
  // ... your code here
    rmse /= estimations.size();
  //calculate the squared root
  // ... your code here
  for(int i=0; i < rmse.size(); ++i){
      rmse(i) = sqrt(rmse(i));
  }

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float vxpy = (vx * py) - (vy * px);

  // square sum
  float ss = pow(px, 2) + pow(py, 2);
  // square root sum
  float srs = sqrt(ss);
  // square root cube sum
  float srcs = ss * srs;
  if (fabs(ss) < 0.0001) {
    std::cout << "KalmanFilter::UpdateEKF - Division by zero error" << std::endl;
    return Hj;
  }

  Hj << (px / srs), (py / srs), 0, 0,
        (-py / ss), (px / ss), 0, 0,
        (py * vxpy / srcs), (px * (-vxpy) / srcs), (px / srs), (py / srs);
  return Hj;
}
