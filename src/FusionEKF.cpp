#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */


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
     * TODO: Create the covariance matrix.
     */
    
    cout << "Measurement: " << measurement_pack.raw_measurements_ << endl;

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Initialize state
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rhodot = measurement_pack.raw_measurements_[2];
      
      // Convert from polar to cartesian coordinates
      float cartesian_x = rho * cos(phi);
      float cartesian_y = rho * sin(phi);
      
      // Velocity in x and y
      // TODO: Should this be absolute?
      float v_x = rhodot * cos(phi);
      float v_y = rhodot * sin(phi);
      
      cout << "Radar | rho: " << rho << ", phi: " << phi << ", x: " << cartesian_x
           << ", y: " << cartesian_y << ", rhodot: " << rhodot
           << ", v_x: " << v_x << ", v_y: " << v_y << endl;
      
      ekf_.x_ << cartesian_x, cartesian_y, v_x, v_y;

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      float cartesian_x = measurement_pack.raw_measurements_[0];
      float cartesian_y = measurement_pack.raw_measurements_[1];
      
      cout << "Lidar | x: " << cartesian_x << ", y: " << cartesian_y << endl;
      
      ekf_.x_ << cartesian_x, cartesian_y, 0, 0; // No velocity for LIDAR
    }
    
    // ekf_.P_ = ???

    previous_timestamp_ = measurement_pack.timestamp_ / 1000000.0;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  // Get elapsed time between timestamps. Division to convert to seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // Update state transition matrix
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;
  
  // Update process noise covariance matrix
  //ekf_.Q_ = ??

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

  } else {
    // TODO: Laser updates

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
