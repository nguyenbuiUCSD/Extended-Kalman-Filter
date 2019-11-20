#include "kalman_filter.h"
#include <math.h>

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
  /**
   * TODO: predict the state
   */
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_=F_*P_*Ft+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  // Sanity check
  float c1 = z[0]*z[0]+z[1]*z[1];
  if ((fabs(c1) < 0.0001) || (z[0]<0.001)){
    return;
  }
    
  // Calculate E Kalman Filter Gain K
  VectorXd z_predict(3);
  // Precalculations
  float rho = sqrt(c1);
  float phi = atan(z[1]/z[0]);
  // Normalize phi
  while (phi>M_PI){
    phi -= M_PI;
  }
  while (phi<=-M_PI){
    phi += M_PI;
  }
    
  float rho_dot = (z[0]*z[2]+z[1]*z[3])/rho;
  z_predict << rho,
               phi,
               rho_dot;
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
