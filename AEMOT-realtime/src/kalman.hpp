// This file is licensed under the Apache License, Version 2.0,
// with additional restrictions under the Commons Clause.
// See the LICENSE file for more details.

#include <eigen3/Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <iomanip>
#include <iostream>
#pragma once

class KalmanFilter
{

public:
  /**
   * Create a Kalman filter with the specified matrices.
   *   F - System dynamics matrix
   *   C - Output matrix
   *   Q - Process noise covariance
   *   R - Measurement noise covariance
   *   P - Estimate error covariance
   */
  KalmanFilter(double dt, const Eigen::MatrixXd &F, const Eigen::MatrixXd &C,
               const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, const Eigen::MatrixXd &P,
               const Eigen::MatrixXd &A, const Eigen::MatrixXd &x0, int n_state,
               int ring_buffer_len);

  /**
   * Create a blank estimator.
   */
  KalmanFilter();

  /**
   * Initialize the filter with initial states as zero.
   */
  void init();

  /**
   * Initialize the filter with a guess for initial states.
   */
  void init(double t0, std::string input_folder_nam);

  /**
   * Update the estimated state based on measured values. The
   * time step is assumed to remain constant.
   */
  void update(const Eigen::Vector3d &y, int id, int p);

  void update_start_point(const Eigen::MatrixXd &x0) { x_hat = x0; }

  void update_distance_threshold(double distance_threshold);

  void update_ts_last_for_gamma(double ts);

  void update_gyro(const Eigen::MatrixXd vx_gyro_update, const Eigen::MatrixXd vy_gyro_update)
  {
    x_hat.block(0, 0, n_target, 1) = x_hat.block(0, 0, n_target, 1) + vx_gyro_update;
    x_hat.block(0, 1, n_target, 1) = x_hat.block(0, 1, n_target, 1) + vy_gyro_update;
  }

  void update_gyro_single_id(double vx_gyro_update, double vy_gyro_update)
  {
    x_hat(0) = x_hat(0) + vx_gyro_update;
    x_hat(1) = x_hat(1) + vy_gyro_update;
    // std::cout << vx_gyro_update << " " << vy_gyro_update << std::endl;
  }

  void update_matrix(const Eigen::MatrixXd &Fx, const Eigen::MatrixXd &Cx,
                     const Eigen::MatrixXd &Qx, const Eigen::MatrixXd &Rx, const Eigen::MatrixXd &Px,
                     const Eigen::MatrixXd &Ax, const Eigen::MatrixXd &x0x)
  {
    F = Fx;
    C = Cx;
    Q = Qx;
    R = Rx;
    P = Px;
    A = Ax;
    x_hat = x0x;
  }

  void reset(const Eigen::MatrixXd &x0_new, double ts);

  //void initialiseDecoder(double dt, std::string data_name, std::string input_event_path, double contrast_threshold, int id);

  // Comms
  //Decoding *decoder;

  /**
   * Return the current state and time.
   */
  Eigen::MatrixXd state() { return x_hat; };
  double time() { return t; };
  double get_ts_last() { return ts_last[0]; };
  double get_ts_last_for_gamma() { return ts_last_for_gamma; };
  double get_dist_threshold() { return dist_threshold; };

  double get_t0() {return t0; };

  int getID() {return track_id; };
  void setID(int track_id) { this->track_id = track_id; };

  void openOutputFile();

  double P_lambda() { return P(4, 4); };
  Eigen::MatrixXd P_x() { return P; };

  bool validated = 0;
  bool active = false;
  double dt_moving_avg = 1;

  // for optimisation:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int N_STATE = 10;

  // for fast processing.cpp lookup:
  inline double pos_x() const { return x_hat(0, 0); }
  inline double pos_y() const { return x_hat(0, 1); }

  // constant-velocity forward prediction to an arbitrary timestamp 'ts'
  inline void predicted_position(double ts, double& px, double& py) const {
    const double dt_since_update = std::max(0.0, ts - ts_last[0]);
    px = x_hat(0, 0) + x_hat(0, 2) * dt_since_update;
    py = x_hat(0, 1) + x_hat(0, 3) * dt_since_update;
  }

  // for logging purposes
  inline int getID() const { return track_id; } // returns the track's persistent unique ID
  inline const double* state_data() const { return x_hat.data(); } // easy fast way to access the kalman state data for logging

  //destructor
  ~KalmanFilter();

  // --- Event accumulator for batching updates ---
  // Collects raw event coordinates between explicit update() calls, so
  // update() (and downstream logging) runs on a decimated timescale
  // instead of once per associated event.
  double accum_sum_x = 0.0;
  double accum_sum_y = 0.0;
  int accum_polarity_sum = 0;   // +1 per ON event, -1 per OFF - majority vote used at flush
  int accum_count = 0;
  double accum_latest_ts = 0.0;         // ts of the most recent accumulated event
  double accum_window_start_ts = 0.0;   // ts of the first event in the current window

  // Lightweight "last seen" timestamp - updated on EVERY associated event immediately, independent of ts_last
  double last_seen_ts = 0.0;
  inline double get_last_seen_ts() const { return last_seen_ts; }

  // lightweight last seen position
  double last_seen_x = 0.0;
  double last_seen_y = 0.0;
  inline double get_last_seen_x() const { return last_seen_x; }
  inline double get_last_seen_y() const { return last_seen_y; }

  // Thresholds injected from outside
  int accum_count_threshold = 8;
  double accum_time_threshold  = 0.002;  // seconds
  inline void update_accumulator_thresholds(int count_thresh, double time_thresh) {
    accum_count_threshold = count_thresh;
    accum_time_threshold = time_thresh;
  }

  // Adds one event into the accumulator. Returns true when the caller
  // should flush (call update() with the accumulated centroid) - either
  // the count threshold or the time threshold was hit, whichever first.
  inline bool accumulate_event(double x, double y, double ts, int p) {
    if (accum_count == 0) accum_window_start_ts = ts;
    accum_sum_x += x;
    accum_sum_y += y;
    accum_polarity_sum += (p > 0) ? 1 : -1;
    accum_count += 1;
    accum_latest_ts = ts;
    
    if (last_seen_ts > 0.0) {
      const double raw_dt = ts - last_seen_ts;
      const double dt_alpha = 0.95;
      dt_moving_avg = dt_alpha * dt_moving_avg + (1 - dt_alpha) * raw_dt;
    }
    last_seen_ts = ts;
    last_seen_x = x;
    last_seen_y = y;

    return (accum_count >= accum_count_threshold) || (ts - accum_window_start_ts >= accum_time_threshold);
  }

  // Centroid (mean x, mean y) at the window's latest ts, plus majority
  // polarity - written into e_out/p_out ready to hand straight to
  // update(). Call reset_accumulator() right after.
  inline void get_accumulated_measurement(Eigen::Vector3d& e_out, int& p_out) const {
      e_out << accum_sum_x / accum_count, accum_sum_y / accum_count, accum_latest_ts;
      p_out = (accum_polarity_sum >= 0) ? 1 : 0;
  }

  inline void reset_accumulator() {
      accum_sum_x = 0.0;
      accum_sum_y = 0.0;
      accum_polarity_sum = 0;
      accum_count = 0;
      // accum_window_start_ts / accum_latest_ts get reinitialized on the
      // next accumulate_event() call (the accum_count==0 branch above)
  }

private:
  std::ofstream blob_measurements_txt_, states_txt_;

  int track_id = -1;;

  int n_state;
  int n_target = 1;
  // Matrices for computation
  Eigen::Matrix<double, N_STATE, N_STATE> F, P, P0, Q, I, P_pred;
  Eigen::Matrix<double, 3, N_STATE>       C;
  Eigen::Matrix3d                          R, S;
  Eigen::Matrix2d                          A;
  Eigen::Matrix<double, 1, N_STATE>        x_hat, x_hat_pred;
  Eigen::Matrix<double, N_STATE, 3>        K;

  Eigen::Matrix2d rotation_m, rotation_m_buf, A_rotation, Omiga, C_lambda, temp;
  Eigen::Vector2d e_tilda, e_tilda_buffer, y_hat, y_hat_sum, y_hat_buffer, C_lambda_diag, temp_diag, y_lambda, y_lambda_sum;
  Eigen::Vector3d y_true, y_hat_fuse;

  // System dimensions
  int m, n;

  // Distance threshold
  double dist_threshold;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  std::vector<double> ts_last;
  double ts_last_for_gamma;

  double y_square_sum, theta_buf;

  int ring_buffer_len;
  double vx_gyro = 0;
  double vy_gyro = 0;

  typedef boost::circular_buffer<Eigen::MatrixXd> CircularBuffer;
  std::vector<boost::circular_buffer<std::tuple<double, double, double, double>>> ring_buffer_e;
  std::vector<boost::circular_buffer<std::pair<double, double>>> ring_buffer_x;
  std::vector<boost::circular_buffer<Eigen::Matrix2d>> ring_buffer_rot;
  std::vector<boost::circular_buffer<std::pair<double, double>>> ring_buffer_delta;

  bool f_equivalent_measurement_init = 0;
  int equiv_measurement_step = 1;
  int equiv_measurement_count = 0;

};
