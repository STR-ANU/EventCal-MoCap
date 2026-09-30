#ifndef TRACKMANAGER_HPP
#define TRACKMANAGER_HPP

#include <vector>
// #include <opencv4/opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>

#include "kalman.hpp"
#include "track_logger.hpp"
#include "track_summary_logger.hpp"

class KalmanFilter;

struct NewTrackResult {
        KalmanFilter* track = nullptr;
        int slot = -1;
    };

class TrackManager
{

public:
    //------------------------------------------//
    //-----        PUBLIC FUNCTIONS        -----//
    //------------------------------------------//
    // Constructor Functions
    TrackManager(double dt, Eigen::MatrixXd F, Eigen::MatrixXd C,
                 Eigen::MatrixXd Q, Eigen::MatrixXd R,
                 Eigen::MatrixXd P, Eigen::MatrixXd A,
                 Eigen::MatrixXd x0, int ring_buffer_len, int n_state, 
                 int pool_size, int accumulator_count_thresh, double accumulator_time_thresh);

    // Add track to output
    NewTrackResult createNewTrack(Eigen::Vector<double,3> new_point);

    // Evaluation
    std::vector<int> evaluateTracks(double ts);
    std::vector<int> evaluateTracksPosition();
    std::vector<int> evaluateDoubleTracks();

    // Initialise Decoding
    //void initialiseDecoder(double dt, std::string data_name, std::string input_event_path, double contrast_threshold);

    // Access the track objects
    std::vector<KalmanFilter *> getTracks();
    KalmanFilter *getTrack(int id);

    // Destructor
    ~TrackManager();

    bool hasAvailableSlot();

    int len(){ return output_tracks.size(); };

    void update_default_dist_threshold(double dist_threshold);
    void update_frame_dimensions(int height, int width);
    void update_event_rate_threshold(double threshold);
    void update_rate_per_area(double rate_per_area);

    void store_parameters(double ts_age, double dt_terminate, double low_activity_factor);

    //void setDecodeFlag(bool flag){f_decode = flag;};
    //void storeDecodeVariables(double dt, std::string data_name, std::string input_event_path, double contrast_threshold);

    std::vector<int> getValidTrackIds();

    int activeCount();

    inline KalmanFilter* getTrackUnchecked(int id) { return output_tracks[id]; }

    //logging
    void setLogger(TrackLogger* logger, bool save_files);
    void logTrackUpdate(int slot, double ts, const double* x_hat_data);

    void setSummaryLogger(TrackSummaryLogger* summary_logger);
    void flushAllSummaries();


private:
    //------------------------------------------//
    //-----        PRIVATE FUNCTIONS       -----//
    //------------------------------------------//
    void deleteTrack(std::vector<KalmanFilter *> &track_array, int track_id, uint32_t delete_reason);

    //logging
    void flushBacklog(int slot);

    
    void recordValidationTime(int slot, double ts_now);

    // Builds and writes one TrackSummaryRecord for `slot`, using its stored
    // validation snapshot plus its current (end-of-life) state/covariance.
    // Only called for tracks that are validated - see deleteTrack() and
    // flushAllSummaries().
    void writeSummary(int slot, uint32_t delete_reason);

    //------------------------------------------//
    //-----        PRIVATE VARIABLES       -----//
    //------------------------------------------//
    // Vector to store the filters: dynamic list of Kalman filter pointers
    std::vector<KalmanFilter *> output_tracks;

    double dt;
    Eigen::MatrixXd F, C, R, P, Q, A, x0_template;
    int ring_buffer_len = 1;
    int n_state = 10;

    double default_dist_threshold;
    int height = 720;
    int width = 1280;

    int pool_size = 10; // set the number of available kalman objects here!!

    double event_rate_threshold;

    int next_unique_id = 0; 

    bool save_files = true;

    //bool f_decode = 0;
    //double decoder_dt;
    //std::string decoder_data_name;
    //std::string decoder_input_event_path;
    //double decoder_contrast_threshold;

    double evaluate_ts_age;
    double evaluate_dt_terminate;
    double evaluate_low_activity_factor;

    double rate_per_area = 1.0;

    int active_track_count_ = 0;

    //logging
    TrackLogger* logger_ = nullptr;
    std::vector<std::vector<KalmanLogRecord>> log_backlogs_;

    // Running count of records logged (backlog + post-validation) per slot,
    // reset whenever the slot is reused for a new track. Feeds
    // TrackSummaryRecord::num_records.
    std::vector<uint32_t> log_counts_;

    TrackSummaryLogger* summary_logger_ = nullptr;
    
    // One entry per pool slot: ts at which that slot's track validated,
    // or -1 if not yet validated / not holding a validated track.
    std::vector<double> t_validated_;
};

#endif
