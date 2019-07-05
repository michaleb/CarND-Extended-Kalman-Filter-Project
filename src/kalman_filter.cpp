#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  
  MatrixXd F_t = F_.transpose();
  x_ = F_ * x_;
  P_ = F_ * P_ * F_t + Q_;
} 
//The prediction for Radar is made in cartesian space, however the update is in polar space...hence must 
//convert back to polar prior to update

void KalmanFilter::Update(const VectorXd &z) {
  
  VectorXd y = z - H_ * x_;
  MatrixXd H_t = H_.transpose();
  MatrixXd S = H_ * P_ * H_t + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * H_t * Si;

  // new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
  //Convert predicted state of x_ from Cartesian to polar coordinates
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = pow(pow(px,2) + pow(py,2),0.5);
  float phi = atan2(py, px);
  float rho_dot;

  // check division by zero
  if (fabs(rho) < 0.0001) { 
    rho_dot = 0;
  }
  else {
    rho_dot = (px*vx + py*vy)/rho;
  }

  //x_prime a 3x1 vector is used to store polar equivalent of predicted state x_
  //which can then be compared with the radar measurement z also a 3x1 vector to find the error, y,
  // between prediction and actual measurement in the current update.

  VectorXd x_prime(3);
  x_prime << rho,
             phi,
             rho_dot;  

  VectorXd y = z - x_prime;
  
  //adjust phi to range -pi to +pi
  if (y(1) < -M_PI) { 
    y(1) += 2*M_PI;
  } 
  else if (y(1) > M_PI) {
    y(1) -= 2*M_PI;
  }
  
  MatrixXd H_t = H_.transpose();
  MatrixXd S = H_ * P_ * H_t + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * H_t * Si;           
  
  // new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}


