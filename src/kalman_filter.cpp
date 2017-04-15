#define _USE_MATH_DEFINES

#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = (F_ * P_ * F_.transpose()) + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = (H_ * P_ * Ht) + R_;
  MatrixXd K = (P_ * Ht) * S.inverse();
  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  P_ = ((MatrixXd::Identity(x_size,x_size)) - (K * H_)) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  if (px == 0) {
    return;
  }
  // square root sum
  float rho = sqrt(pow(px, 2) + pow(py, 2));
  float phi = atan2(py, px);
  float rho_dot = ((px * vx) + (py * vy)) / rho;

  VectorXd hx = VectorXd(3);
  hx << rho,
        phi,
        rho_dot;

  VectorXd z_pred = hx;
  while (z_pred(1) < - M_PI) {
    z_pred(1) += M_PI;
  }
  while (z_pred(1) > M_PI) {
    z_pred(1) -= M_PI;
  }
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = (H_ * P_ * Ht) + R_;
  MatrixXd K = (P_ * Ht) * S.inverse();
  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  P_ = ((MatrixXd::Identity(x_size,x_size)) - (K * H_)) * P_;
}
