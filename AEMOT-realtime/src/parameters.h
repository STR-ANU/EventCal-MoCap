#ifndef PARAMETERS_H
#define PARAMETERS_H
#include "yaml-cpp/yaml.h"
#include <string>

struct Parameters
{
  // General
  int width;
  int height;
  double dt;
  int show_display;
  int save_files;
  int create_event_log_file;

  // camera
  int camera_id;

  // Track Manager
  int pool_size;
  int use_dt_detector;
  double dist_threshold;
  int ring_buffer_len;
  int f_evaluate;
  double evaluate_ts_age;
  double evaluate_dt_terminate;
  double rate_per_area;
  double event_rate_threshold;
  double evaluate_low_activity_factor;

  // Renderer, high-pass, and display
  int publish_framerate;
  double contrast_threshold;
  int alpha;
  int f_show_candidates;
  int disp_covariance_flag;

  // Kalman filters
  int n_state;
  double lambda_init;
  double var_x;
  double var_y;
  double var_vx;
  double var_vy;
  double var_lambda_1;
  double var_lambda_2;
  double var_theta;
  double var_q;
  double q_x;
  double q_y;
  double q_vx;
  double q_vy;
  double q_lambda_1;
  double q_lambda_2;
  double q_theta;
  double q_q;
  int accumulator_count_thresh;
  double accumulator_time_thresh;

  // SAE detector
  int SAE_operation_rate;
  double detector_dist_threshold;
  int SAE_ksize;
  double SAE_alpha;
  double SAE_detection_threshold;
  double SAE_min_active_pixels;
  double SAE_min_contributions;
  double SAE_recency_window;
  double detector_dt_threshold;

  // Corroboration grid
  double corroboration_cell_size;
  double corroboration_window;
  double corroboration_direction_cos_threshold;

};

Parameters loadParametersFromYAML(const std::string &yaml_file_path);

#endif // PARAMETERS_H