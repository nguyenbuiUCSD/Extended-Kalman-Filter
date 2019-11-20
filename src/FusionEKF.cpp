#include "FusionEKF.h"
#include <iostream>
#include <math.h>
#include "Eigen/Dense"
#include "tools.h"

#define NOISE_AX 100
#define NOISE_AY 100


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
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

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
    
    // TODO: Mesurement matrix - Radar
    
    
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
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
            // init ekf.x_
      ekf_.x_ << measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]), 
                 measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]), 
                 0, 
                 0;
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
          
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

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
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
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    
    // Precalculation for H_radar_j
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    
    // Change ekf before update
    ekf_.H_ = Hj_;
    ekf_.R_ =  R_radar_;
    
    // Update with Laser data
    ekf_.Update(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates

    
    // Change ekf before update
    ekf_.H_ = H_laser_;
    ekf_.R_ =  R_laser_;
    
    // Update with Laser data
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
