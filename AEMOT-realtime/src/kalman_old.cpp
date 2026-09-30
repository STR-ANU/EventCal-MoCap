// This file is licensed under the Apache License, Version 2.0,
// with additional restrictions under the Commons Clause.
// See the LICENSE file for more details.

#include <boost/circular_buffer.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "kalman.hpp"

double new_ts_count = 0;
double total_count = 0;


KalmanFilter::KalmanFilter(double dt, const Eigen::MatrixXd &F, const Eigen::MatrixXd &C,
                           const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                           const Eigen::MatrixXd &P, const Eigen::MatrixXd &A,
                           const Eigen::MatrixXd &x0, int n_state,
                           int ring_buffer_len)
    : F(F), C(C), Q(Q), R(R), P0(P), A(A), dt(dt), initialized(false),
      x_hat(x0), n_state(n_state), ring_buffer_len(ring_buffer_len)
{
  I.setIdentity();
  x_hat = x0;

  //decoder = nullptr; //instanciate this here

  // Important for re-init different targets
  ts_last.clear();
  ring_buffer_e.clear();
  ring_buffer_x.clear();
  ring_buffer_rot.clear();
  ring_buffer_delta.clear();

  for (int i = 0; i < n_target; ++i)
  {
    ring_buffer_e.emplace_back(ring_buffer_len);
    ring_buffer_x.emplace_back(ring_buffer_len);
    ring_buffer_rot.emplace_back(ring_buffer_len);
    ring_buffer_delta.emplace_back(ring_buffer_len);
  }


}

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter(){
  //delete decoder;
}

void KalmanFilter::init(double t0, std::string input_folder_name)
{
  P = P0;
  this->t0 = t0;
  initialized = true;
  Omiga << 0, -1, 1, 0;
  y_true << 0, 0, 2 * ring_buffer_len;

  // Important for re-init different targets
  ts_last.clear();
  ring_buffer_e.clear();
  ring_buffer_x.clear();
  ring_buffer_rot.clear();
  ring_buffer_delta.clear();
  for (int i = 0; i < n_target; i++)
  {
    ts_last.push_back(t0);
    ring_buffer_e.emplace_back(ring_buffer_len);
    ring_buffer_x.emplace_back(ring_buffer_len);
    ring_buffer_rot.emplace_back(ring_buffer_len);
    ring_buffer_delta.emplace_back(ring_buffer_len);
  }
}

void KalmanFilter::init()
{
  P = P0;
  t0 = 0;
  initialized = true;

  //accumulator
  accum_sum_x = 0.0;
  accum_sum_y = 0.0;
  accum_polarity_sum = 0;
  accum_count = 0;
  last_seen_ts = t0;
}


void KalmanFilter::openOutputFile(){ // HACK : comment these lines to stop files opening..

  // // // std::cout << "Opening output files for " << track_id << "...\t\t";

  // std::string master_dir = "/Users/angus/data/comms/comms_3d_pyramid/intrinsic_calibration/old_camera/raw/";

  // // // // std::string blob_file_name = master_dir + "/equivalent/state_measurements_" + std::to_string(track_id) + ".csv";
  // // // // blob_measurements_txt_.open(blob_file_name);

  // std::string state_file_name = master_dir + "state_" + std::to_string(track_id) + ".csv";
  // states_txt_.open(state_file_name);

  // std::cout << states_txt_.is_open() << "\t" << blob_measurements_txt_.is_open() << std::endl;
 
}


//void KalmanFilter::initialiseDecoder(double dt, std::string data_name, std::string input_event_path, double contrast_threshold, int id)
//{
//  delete decoder; // Delete any existing decoder to avoid memory leaks
//  decoder = new Decoding(dt, data_name, input_event_path, contrast_threshold, id);
//}

void KalmanFilter::reset(const Eigen::MatrixXd &x0_new, double ts)
{
  x_hat = x0_new; // reset state vector
  P = P0;         // reset covariance matrix
  t0 = ts;        // reset initial time
  ts_last_for_gamma = ts; // reset last timestamp for gamma calculation
  dt_moving_avg = 1e-3; // reset moving average of dt
  validated = 0; // reset validation flag
  initialized = true;

  // clear the accumulator for the reused slot
  accum_sum_x = 0.0;
  accum_sum_y = 0.0;
  accum_polarity_sum = 0;
  accum_count = 0;
  last_seen_ts = ts;

  f_equivalent_measurement_init = 0; // reset equivalent measurement initialization flag
  equiv_measurement_count = 0; // reset equivalent measurement count

  // Important for re-init different targets
  ts_last.clear();
  ring_buffer_e.clear();
  ring_buffer_x.clear();
  ring_buffer_rot.clear();
  ring_buffer_delta.clear();
  for (int i = 0; i < n_target; i++)
  {
    ts_last.push_back(ts);
    ring_buffer_e.emplace_back(ring_buffer_len);
    ring_buffer_x.emplace_back(ring_buffer_len);
    ring_buffer_rot.emplace_back(ring_buffer_len);
    ring_buffer_delta.emplace_back(ring_buffer_len);
  }
}

void KalmanFilter::update_distance_threshold(double distance_threshold)
{
  dist_threshold = distance_threshold;
}

void KalmanFilter::update_ts_last_for_gamma(double ts)
{
  ts_last_for_gamma = ts;
}

void KalmanFilter::update(const Eigen::Vector3d &e, int id, int p)
{

  bool f_new_equiv_measurement = 0;

  id = 0;
  if (!initialized)
    throw std::runtime_error("Filter is not initialized!");

  if (ts_last[id] <= 0)
  {
    dt = 0; // First event triggered at this pixel
  }
  else
  {
    dt = (e(2) - ts_last[id]);
  }

  if (!f_equivalent_measurement_init){
    x_hat_pred = x_hat;
    P_pred = P;
    f_equivalent_measurement_init = 1;
  }

  //------------------------------------------------//
  //------------------------------------------------//
  //------------------------------------------------//
  // this below is now handled in kalman.hpp for batch kalman updating
  //double dt_alpha = 0.95;
  //dt_moving_avg = (dt_alpha) * dt_moving_avg + (1-dt_alpha) * dt;

  ts_last[id] = e(2);

  F(0, 2) = dt;
  F(1, 3) = dt;
  F(6, 7) = dt;

  //------------------------------------------------//
  //-----              PREDICTION              -----//
  //------------------------------------------------//

  // Propagate updated state (predict and update)
  x_hat = (F * x_hat.transpose()).transpose();
  P = F * P * F.transpose() + Q * dt;

  x_hat_pred = (F * x_hat_pred.transpose()).transpose();
  P_pred = F * P_pred * F.transpose() + Q * dt;
  
  rotation_m << cos(x_hat(0, 6)), -sin(x_hat(0, 6)), sin(x_hat(0, 6)), cos(x_hat(0, 6));
  const Eigen::Matrix2d rotation_m_T = rotation_m.transpose();

  // A is 1 on lambda
  A << 1 / x_hat(id, 4), 0, 0, 1 / x_hat(id, 5);
  A_rotation = rotation_m * A * rotation_m_T;

  if (p <= 0)
  {
    e_tilda << e(0) + x_hat(0, 8) - x_hat(0, 0), e(1) + x_hat(0, 9) - x_hat(0, 1);
    ring_buffer_delta[id].push_back({x_hat(0, 8), x_hat(0, 9)});
  }
  else
  {
    e_tilda << e(0) - x_hat(0, 8) - x_hat(0, 0), e(1) - x_hat(0, 9) - x_hat(0, 1);
    ring_buffer_delta[id].push_back({-x_hat(0, 8), -x_hat(0, 9)});
  }

  y_hat = A_rotation * e_tilda;
  y_hat_sum << 0, 0;
  y_square_sum = 0;
  C.block<2, 2>(0, 0) << -A_rotation;
  C.block<2, 2>(0, 2) << 0, 0, 0, 0;
  C_lambda_diag = A * A * rotation_m_T * e_tilda;
  C_lambda << C_lambda_diag(0), 0, 0, C_lambda_diag(1);
  C.block<2, 2>(0, 4) << -rotation_m * C_lambda;
  C.block<2, 1>(0, 6) << rotation_m * (Omiga * A - A * Omiga) * rotation_m_T * e_tilda;
  C.block<2, 1>(0, 7) << 0, 0;

  if (p <= 0)
  {
    C.block<2, 2>(0, 8) << A_rotation;
  }
  else
  {
    C.block<2, 2>(0, 8) << -A_rotation;
  }

  C.block<1, N_STATE>(2, 0) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  ring_buffer_e[id].push_back({e(0), e(1), e(2), p});
  ring_buffer_x[id].push_back({x_hat(0, 0), x_hat(0, 1)});
  ring_buffer_rot[id].push_back(rotation_m);

  // if (ring_buffer_e[id].size() >= 2)
  if (ring_buffer_e[id].size() >= ring_buffer_len)
  {
    // t1, t2, t3, ..., buf_dt[last] is the latest for buffer e, x and m
    // here use t1, t2, t3, ..., buf_dt[last]-1
    y_lambda << 0, 0;
    y_lambda_sum << 0, 0;
    for (int i = (ring_buffer_e[id].size() - 2); i >= 0; --i)
    {
      // Use the same time states and events
      e_tilda_buffer << std::get<0>(ring_buffer_e[id][i]) + ring_buffer_delta[id][i].first - ring_buffer_x[id][i].first,
          std::get<1>(ring_buffer_e[id][i]) + ring_buffer_delta[id][i].second - ring_buffer_x[id][i].second;

      rotation_m_buf = ring_buffer_rot[id][i];
      y_hat_buffer = rotation_m_buf * A * rotation_m_buf.transpose() * e_tilda_buffer;
      y_hat_sum += y_hat_buffer;
      y_square_sum += y_hat_buffer.squaredNorm();

      temp_diag = A * A * rotation_m_buf.transpose() * e_tilda_buffer;
      temp << temp_diag(0), 0, 0, temp_diag(1);

      y_lambda = -2 * y_hat_buffer.transpose() * rotation_m_buf * temp;
      y_lambda_sum += y_lambda;
    }
    C.block<1, 2>(2, 4) += y_lambda_sum;
  }

  // Innovation covariance
  R(2, 2) = 4 * ring_buffer_e[id].size();
  S = C * P * C.transpose() + R;
  K = P * C.transpose() * S.inverse();
  y_hat_fuse << y_hat, y_square_sum;
  y_true.coeffRef(2, 0) = 2 * ring_buffer_e[id].size();
  x_hat.noalias() += (K * (y_true - y_hat_fuse)).transpose();
  P = (I - K * C) * P;
      
}
