#include "parameters.h"

Parameters loadParametersFromYAML(const std::string &yaml_file_path)
{
    const YAML::Node config = YAML::LoadFile(yaml_file_path);

    Parameters params;

    // General
    params.width = config["width"].as<int>();
    params.height = config["height"].as<int>();
    params.dt = config["dt"].as<double>();
    params.show_display = config["show_display"].as<int>();
    params.save_files = config["save_files"].as<int>();
    params.create_event_log_file = config["create_event_log_file"].as<int>();


    // Track Manager
    params.pool_size = config["pool_size"] ? config["pool_size"].as<int>() : 10;
    params.use_dt_detector = config["use_dt_detector"] ? config["use_dt_detector"].as<int>() : 0;
    params.dist_threshold = config["dist_threshold"].as<double>();
    params.ring_buffer_len = config["ring_buffer_len"].as<int>();

    params.f_evaluate = config["f_evaluate"].as<int>();
    params.evaluate_ts_age = config["evaluate_ts_age"].as<double>();
    params.evaluate_dt_terminate = config["evaluate_dt_terminate"].as<double>();
    params.rate_per_area = config["rate_per_area"] ? config["rate_per_area"].as<double>() : 1.0;
    params.event_rate_threshold = config["event_rate_threshold"].as<double>();
    params.evaluate_low_activity_factor = config["evaluate_low_activity_factor"].as<double>();


    // Renderer, high-pass, and sisplay
    params.publish_framerate = config["publish_framerate"].as<int>();
    params.contrast_threshold = config["contrast_threshold"].as<double>();
    params.alpha = config["alpha"].as<int>();
    params.f_show_candidates = config["f_show_candidates"].as<int>();
    params.disp_covariance_flag = config["disp_covariance_flag"].as<int>();


    // Kalman filters
    params.n_state = config["n_state"].as<int>();
    params.lambda_init = config["lambda_init"].as<double>();

    params.var_x = config["var_x"].as<double>();
    params.var_y = config["var_y"].as<double>();
    params.var_vx = config["var_vx"].as<double>();
    params.var_vy = config["var_vy"].as<double>();
    params.var_lambda_1 = config["var_lambda_1"].as<double>();
    params.var_lambda_2 = config["var_lambda_2"].as<double>();
    params.var_theta = config["var_theta"].as<double>();
    params.var_q = config["var_q"].as<double>();

    params.q_x = config["q_x"].as<double>();
    params.q_y = config["q_y"].as<double>();
    params.q_vx = config["q_vx"].as<double>();
    params.q_vy = config["q_vy"].as<double>();
    params.q_lambda_1 = config["q_lambda_1"].as<double>();
    params.q_lambda_2 = config["q_lambda_2"].as<double>();
    params.q_theta = config["q_theta"].as<double>();
    params.q_q = config["q_q"].as<double>();

    params.accumulator_count_thresh = config["accumulator_count_thresh"].as<int>();
    params.accumulator_time_thresh = config["accumulator_time_thresh"].as<double>();


    // SAE detector
    params.SAE_operation_rate = config["SAE_operation_rate"] ? config["SAE_operation_rate"].as<int>() : 500;
    params.detector_dist_threshold = config["detector_dist_threshold"].as<double>();

    params.SAE_ksize = config["SAE_ksize"].as<int>();
    params.SAE_alpha = config["SAE_alpha"].as<double>();
    params.SAE_detection_threshold = config["SAE_detection_threshold"].as<double>();
    params.SAE_min_active_pixels = config["SAE_min_active_pixels"].as<double>();
    params.SAE_min_contributions = config["SAE_min_contributions"].as<double>();
    params.SAE_recency_window = config["SAE_recency_window"].as<double>();
    params.detector_dt_threshold = config["detector_dt_threshold"].as<double>();


    // Corroboration grid
    params.corroboration_cell_size = config["corroboration_cell_size"].as<double>();
    params.corroboration_window = config["corroboration_window"].as<double>();
    params.corroboration_direction_cos_threshold = config["corroboration_direction_cos_threshold"].as<double>();
    
    // camera
    params.camera_id = config["camera_id"].as<int>();

    return params;
}