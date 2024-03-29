#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

/* name: Init()
 * usage: Set Kalman Filter Parametters
 */
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

/* name: Predict()
 * usage: Predict next state after a time dt
 */
void KalmanFilter::Predict() {

  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_=F_*P_*Ft+Q_;
}

/* name: Update()
 * usage: Update current state with observation from laser sensor data
 */
void KalmanFilter::Update(const VectorXd &z) {

  // Calculate Kalman Filter Gain K
  VectorXd z_predict = H_*x_;
  VectorXd y = z-z_predict;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht+R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_*Ht*Si;
  
  // New position after sensor feedback predict position + (Gain*error)
  x_ = x_+(K*y);
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  P_ = (I-K*H_)*P_;
}

/* name: UpdateEKF()
 * usage: Update current state with observation from radar sensor data
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {

  // Sanity check
  float c1 = x_[0]*x_[0]+x_[1]*x_[1];
  if (c1 < 0.000001){
    return;
  }

  // Calculate E Kalman Filter Gain K
  VectorXd z_predict(3);
  
  // Precalculations
  float rho = sqrt(c1);
  float phi = atan2(x_[1],x_[0]);
  float rho_dot = (x_[0]*x_[2]+x_[1]*x_[3])/rho;
  
  // Asign calculated data to vector
  z_predict << rho,
               phi,
               rho_dot;
  
  VectorXd y = z-z_predict;
  // Normalize the phi error
  // Error phi has to be <= M_PI and > -M_PI
  while (y[1]>M_PI){
    y[1] -= 2*M_PI;
  }
  while (y[1]<=-M_PI){
    y[1] += 2*M_PI;
  }
  
  // Kalman filter calculation
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht+R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_*Ht*Si;
  
  // New position after sensor feedback predict position + (Gain*error)
  x_ = x_+(K*y);
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  P_ = (I-K*H_)*P_;
}
