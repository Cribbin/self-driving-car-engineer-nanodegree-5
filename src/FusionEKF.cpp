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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  // measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;

  // State covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  
  // State transition matrix
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  ekf_.Q_ = MatrixXd(4, 4);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    RunInitialization(measurement_pack);
    // done initializing, no need to predict or update
    return;
  }

  RunPrediction(measurement_pack);
  RunUpdate(measurement_pack);

  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

void FusionEKF::RunInitialization(const MeasurementPackage &measurement_pack) {
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
    float x = rho * cos(phi);
    float y = rho * sin(phi);

    // Velocity in x and y
    float v_x = rhodot * cos(phi);
    float v_y = rhodot * sin(phi);

    ekf_.x_ << x, y, v_x, v_y;

  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // Initialize state.
    float x = measurement_pack.raw_measurements_[0];
    float y = measurement_pack.raw_measurements_[1];

    ekf_.x_ << x, y, 0, 0; // No velocity for LIDAR
  }

  previous_timestamp_ = measurement_pack.timestamp_;

  // done initializing, no need to predict or update
  is_initialized_ = true;
}

void FusionEKF::RunPrediction(const MeasurementPackage &measurement_pack) {
  // Get elapsed time between timestamps. Division to convert to seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // Update state transition matrix
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Update process noise covariance matrix
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  float noise_ax = 9.0;
  float noise_ay = 9.0;

  ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
             0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();
}

void FusionEKF::RunUpdate(const MeasurementPackage &measurement_pack) {
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    // Update Measurement matrix and measurement noise covariance matrix
    ekf_.H_ = H_laser_;      
    ekf_.R_ = R_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
  }
}