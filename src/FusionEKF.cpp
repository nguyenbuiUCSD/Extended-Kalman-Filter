#include "FusionEKF.h"
#include <iostream>
#include <math.h>
#include "Eigen/Dense"
#include "tools.h"

#define NOISE_AX 9
#define NOISE_AY 9


using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);


  /**
   * Initializing Extended Kalman Filter Matrices
   * x_, F_, P_, Q_ : fixed size
   * H_, R_ : dynamic size, depend on sensor type
   */
  ekf_.x_ = VectorXd(4);
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    
    // first measurement
    cout << "Initializing first EKF ... " << endl;
    
    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
              0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
    
    // Mesurement matrix - Laser
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    
    /*
    * INITIALIZING EXTENDED KALMAN FILTER (FusionEKF.ekf_)
    * x_ : current state
    * P_ : The current uncertainty covariance
    * F_ : transition matrix
    * H_ : Measurment Transformation
    * R_ : Measurement uncertainty (Measurement noise)
    * Q_ : Process covariance
    */
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;
    ekf_.Q_ << 0, 0, 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;

    // Init x_, H_, R_
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

      // init ekf.x_
      ekf_.x_ << measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]), 
                 measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]), 
                 0, 
                 0;
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    
      // init ekf.x_
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
                 measurement_pack.raw_measurements_[1], 
                 0, 
                 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    
    cout << "Done ... " << endl;
    return;
  }

  /*
   * TEST:
   * Use this to skip one data type
   */
  /*
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //return;
  } else {
    //return;
  }
  */
  
  /**
   * Prediction
   */
  
  // Calculate dt in second and save current timestamp
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Precalculation
  float dt_p2 = dt * dt;
  float dt_p3 = dt_p2 * dt;
  float dt_p4 = dt_p3 * dt;
    
  // Update Transition matrix
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Calculate process covariance matrix Q
  ekf_.Q_ <<  dt_p4/4*NOISE_AX, 0, dt_p3/2*NOISE_AX, 0,
              0, dt_p4/4*NOISE_AY, 0, dt_p3/2*NOISE_AY,
              dt_p3/2*NOISE_AX, 0, dt_p2*NOISE_AX, 0,
              0, dt_p3/2*NOISE_AY, 0, dt_p2*NOISE_AY;

  ekf_.Predict();

  /**
   * UPDATE
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Precalculation for H_j
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    // Change ekf before update
    ekf_.H_ = Hj_;
    ekf_.R_ =  R_radar_;
    // Update with Laser data
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  
  } else {
    // Change ekf before update
    ekf_.H_ = H_laser_;
    ekf_.R_ =  R_laser_;
    // Update with Laser data
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ (" << measurement_pack.sensor_type_ << ") : " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
