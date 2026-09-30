
#include <vector>
#include <fstream>
#include <filesystem>
#include <iostream>

#include "TrackManager.hpp"
#include "kalman.hpp"

//------------------------------------------//
//-----        PUBLIC FUNCTIONS        -----//
//------------------------------------------//

//************************************************************************************//
//**********                          CONSTRUCTOR                           **********//
//************************************************************************************//
TrackManager::TrackManager(double dt, Eigen::MatrixXd F, Eigen::MatrixXd C,
                           Eigen::MatrixXd Q, Eigen::MatrixXd R,
                           Eigen::MatrixXd P, Eigen::MatrixXd A,
                           Eigen::MatrixXd x0, int ring_buffer_len, int n_state,
                           int pool_size, int accumulator_count_thresh, double accumulator_time_thresh)
    : dt(dt), F(F), C(C), Q(Q), R(R), P(P), A(A),
      x0_template(x0), ring_buffer_len(ring_buffer_len), n_state(n_state), pool_size(pool_size)
{
    // Build the entire kalman filter line up on init
    output_tracks.reserve(pool_size);
    for (int i = 0; i < pool_size; ++i) 
    {
        KalmanFilter *kf = new KalmanFilter(dt, F, C, Q, R, P, A, x0_template, n_state, ring_buffer_len);
        kf->active = false;

        //injest tunable threshholds
        kf->accum_count_threshold = accumulator_count_thresh;
        kf->accum_time_threshold = accumulator_time_thresh;
        
        output_tracks.push_back(kf);
    }
    active_track_count_ = 0;

    // one backlog / counter / validation-snapshot slot per pool slot
    log_backlogs_.resize(pool_size);
    log_counts_.assign(pool_size, 0u);
    t_validated_.resize(pool_size, -1.0);
};

void TrackManager::update_default_dist_threshold(double dist_threshold)
{
    default_dist_threshold = dist_threshold;
}

void TrackManager::update_frame_dimensions(int height, int width)
{
    this->height = height;
    this->width = width;
}

void TrackManager::update_event_rate_threshold(double threshold)
{
    this->event_rate_threshold = threshold;
}

void TrackManager::update_rate_per_area(double rate_per_area)
{
    this->rate_per_area = rate_per_area;
}

void TrackManager::store_parameters(double ts_age, double dt_terminate, double low_activity_factor){
    this->evaluate_ts_age = ts_age;
    this->evaluate_dt_terminate = dt_terminate;
    this->evaluate_low_activity_factor = low_activity_factor;
}

void TrackManager::setLogger(TrackLogger* logger, bool save_files)
{
    logger_ = logger;
    this->save_files = save_files;
}
void TrackManager::setSummaryLogger(TrackSummaryLogger* summary_logger)
{
    summary_logger_ = summary_logger;
}

TrackManager::~TrackManager()
{
    for (auto *kf : output_tracks)
    {
        delete kf;
    }
}

//************************************************************************************//
//**********                         ACCESS TRACKS                          **********//
//************************************************************************************//
// Get the vector of all output/candidate tracks
std::vector<KalmanFilter *> TrackManager::getTracks()
{
    std::vector<KalmanFilter *> active_tracks;
    active_tracks.reserve(active_track_count_);
    for (auto *kf : output_tracks)
    {
        if (kf->active) active_tracks.push_back(kf);
    }
    return active_tracks;
}

// Access individual tracks
KalmanFilter *TrackManager::getTrack(int id)
{
    if (id >= output_tracks.size() || id < 0)
    {
        throw std::runtime_error("Requested ID out of range of Output Tracks");
    }
    return output_tracks[id];
}

// Get valid track IDs
std::vector<int> TrackManager::getValidTrackIds()
{
    std::vector<int> valid_ids;
    valid_ids.reserve(active_track_count_);
    for (int i = 0; i < output_tracks.size(); ++i)
    {
        if (output_tracks[i]->active) // or add more checks if needed
        {
            valid_ids.push_back(i);
        }
    }
    return valid_ids;
}

//************************************************************************************//
//**********                        SPAWN CANDIDATES                        **********//
//************************************************************************************//
// Create new candidate tracker
NewTrackResult TrackManager::createNewTrack(Eigen::Vector<double,3> new_point)
{
    for (int i = 0; i < output_tracks.size(); ++i)
    {
        if (!output_tracks[i]->active)
        {
            // Reuse an inactive Kalman filter
            KalmanFilter *kf = output_tracks[i];

            Eigen::MatrixXd x0_new_track = x0_template;
            x0_new_track(0) = new_point(0);
            x0_new_track(1) = new_point(1);

            // reset the filter
            kf->reset(x0_new_track, new_point(2));

            kf->update_distance_threshold(default_dist_threshold);
            kf->update_ts_last_for_gamma(new_point(2));
            kf->setID(next_unique_id);
            kf->validated = 0;
            kf->active = true;
            active_track_count_++;

            //debug
            //std::cerr << "[new] slot=" << i
            //            << " xy=(" << new_point(0) << "," << new_point(1) << ")"
            //            << " t0=" << new_point(2)
            //            << " activeCount=" << activeCount() << std::endl;

            //if (f_decode){
            //    kf->initialiseDecoder(decoder_dt, decoder_data_name, decoder_input_event_path, decoder_contrast_threshold, next_unique_id);
            //}

            next_unique_id++;

            if (save_files) {
                // logging: clear out any leftover backlog from whatever track used this slot last
                log_backlogs_[i].clear();
                log_counts_[i] = 0;
                t_validated_[i] = -1.0;
                // Log this track's initial state - buffered until (if) it validates.
                logTrackUpdate(i, new_point(2), kf->state_data());
            }

            return {kf, i}; // stop after filling one inactive slot
        }
    }
    
    std::cout << "Warning! Kalman lineup is exhausted - no free slot for an new track! Consider increasing pool_size.\n";
    return {nullptr, -1}; // returns a nullptr if no track was actively created
}

bool TrackManager::hasAvailableSlot()
{
    return active_track_count_ < pool_size;
}
//************************************************************************************//
//**********                        EVALUATE TRACKS                         **********//
//************************************************************************************//
// Evaluate to 'validate' or 'delete' tracks
std::vector<int> TrackManager::evaluateTracks(double ts_now){
    std::vector<int> deleted_IDs; 

    // Iterate through all tracks and evaluate
    for (int i = output_tracks.size()-1; i >= 0; i--){

        if(!output_tracks[i]->active) continue; // Skip inactive tracks

        auto* kf = output_tracks[i];
        Eigen::VectorXd x = kf->state().transpose();

        const double min_area = 50.0; // prevent division by zero for tiny blobs
        double area = std::max(std::abs(x(4) * x(5)), min_area); // area = width * height

        double required_rate = rate_per_area * area; // threshold scales with area
        required_rate = std::max(required_rate, evaluate_low_activity_factor * event_rate_threshold); // ensure it doesn't go below the base threshold so we don't squash small blobs' hopes and dreams

        bool out_of_frame = (x(0) < 10 || x(0) > width-11 || x(1) < 10 || x(1) > height-11);
        bool inactive     = (ts_now - kf->get_last_seen_ts() > evaluate_dt_terminate);
        bool low_activity = (1.0 / kf->dt_moving_avg) < required_rate;
        bool bad_shape    = (x(4) <= 0 || x(5) <= 0);

        // Always allow deletion for these hard failures
        if (out_of_frame || inactive || bad_shape || low_activity) {
            //debug
            //std::cerr << "[del] id=" << i
            //                << " age=" << (ts_now - output_tracks[i]->get_t0())
            //                << " out=" << out_of_frame
            //                << " inact=" << inactive
            //                << " low=" << low_activity
            //                << " shape=" << bad_shape
            //                << " rate=" << (1.0 / output_tracks[i]->dt_moving_avg)
            //                << " thr=" << required_rate
            //                << " val=" << output_tracks[i]->validated
            //                << std::endl;

            //for summary logging:
            uint32_t reason = 0;
            if (out_of_frame) reason |= TrackDeleteReason::OUT_OF_FRAME;
            if (inactive)     reason |= TrackDeleteReason::INACTIVE;
            if (bad_shape)    reason |= TrackDeleteReason::BAD_SHAPE;
            if (low_activity) reason |= TrackDeleteReason::LOW_ACTIVITY;

            deleted_IDs.push_back(i);
            deleteTrack(output_tracks, i, reason);
            continue;
        }

        // Validation waits for age
        if (ts_now - kf->get_t0() >= evaluate_ts_age) {
            bool rate_ok = (1.0 / kf->dt_moving_avg) > event_rate_threshold; // keep standard event_rate_threshold here for validation, not the area-scaled one
            if (rate_ok && kf->validated == 0) {
                kf->validated = 1;
                kf->openOutputFile();
                //if (f_decode){
                //   output_tracks[i]->decoder->open_G_file();
                //}

                // Logging: track just got promoted - write out everything in its buffer and then log straight through from now on
                if (save_files) {
                    flushBacklog(i);
                    recordValidationTime(i, ts_now);
                }
            }
        }
    }

    return deleted_IDs;
}


// Evaluate tracks just by position
std::vector<int> TrackManager::evaluateTracksPosition(){
    std::vector<int> deleted_IDs; 

    // Iterate through all tracks and evaluate
    for (int i = output_tracks.size()-1; i >= 0; i--){

        if(!output_tracks[i]->active) continue; // Skip inactive tracks

        Eigen::Vector<double, 10> x_hat_i = output_tracks[i]->state().transpose();
                   
        // Conditions
        bool out_of_frame = ((x_hat_i(0) < 10) || (x_hat_i(0) > width-11)) || ((x_hat_i(1) < 10) || (x_hat_i(1) > height-11));

        if(out_of_frame){
            deleted_IDs.push_back(i);
            deleteTrack(output_tracks, i, TrackDeleteReason::OUT_OF_FRAME);        
            continue;
        }
    }
    return deleted_IDs;
}



// Evaluate if tracks are tracking the same object
std::vector<int> TrackManager::evaluateDoubleTracks(){
    std::vector<int> deleted_IDs, deleted_IDs_temp; 

    // Iterate through all pairs of tracks
    for (int i = 0; i < output_tracks.size(); i++){

        if(!output_tracks[i]->active) continue; // Skip inactive tracks

        for (int j = 0; j < output_tracks.size(); j++){
            // Skip if it is the same track in both loops
            if (i == j){
                continue;
            }
            else {
                if(!output_tracks[j]->active) {
                    continue; // Skip inactive tracks
                } 
                else {
                    // Read the state
                    Eigen::Vector<double, 10> x_hat_1 = output_tracks[i]->state().transpose();
                    Eigen::Vector<double, 10> x_hat_2 = output_tracks[j]->state().transpose();

                    // Computer x and y distance
                    double x_dist = abs(x_hat_1(0) - x_hat_2(0));
                    double y_dist = abs(x_hat_1(1) - x_hat_2(1));

                    if ((x_dist < 15) & (y_dist < 15)){
                        Eigen::Vector2d v1_norm = x_hat_1.segment(2,3).normalized();
                        Eigen::Vector2d v2_norm = x_hat_2.segment(2,3).normalized();

                        if ((v1_norm.transpose() * v2_norm) < 0.95){
                            bool i_validated = output_tracks[i]->validated;
                            bool j_validated = output_tracks[j]->validated;
                            // only mark i for deletion if it's NOT validated, or if BOTH are validated.
                            if (!(i_validated && !j_validated)){
                                deleted_IDs_temp.push_back(i);
                            }
                        }                    
                    }
                }
            }
        }  
    }

    // Deleted tracks
    for (int i = deleted_IDs_temp.size() - 1; i >= 0; i--){
        deleted_IDs.push_back(deleted_IDs_temp[i]);
        deleteTrack(output_tracks, deleted_IDs_temp[i], TrackDeleteReason::DUPLICATE_TRACK); 
        // std::cout << "Uh oh\n";       
    }

    return deleted_IDs;
}

int TrackManager::activeCount()
{
    return active_track_count_;
}

//************************************************************************************//
//**********                         COMMUNICATION                          **********//
//************************************************************************************//
//void TrackManager::initialiseDecoder(double dt, std::string data_name, std::string input_event_path, double contrast_threshold)
//{
//    for (int id = 0; id < output_tracks.size(); id++)
//    {
//        output_tracks[id]->initialiseDecoder(dt, data_name, input_event_path, contrast_threshold, id);
//    }
//}

//void TrackManager::storeDecodeVariables(double dt, std::string data_name, std::string input_event_path, double contrast_threshold)
//{
//    decoder_dt = dt;
//    decoder_data_name = data_name;
//    decoder_input_event_path = input_event_path;
//    decoder_contrast_threshold = contrast_threshold;
//}


//************************************************************************************//
//**********                            LOGGING                             **********//
//************************************************************************************//
void TrackManager::logTrackUpdate(int slot, double ts, const double* x_hat_data)
{
    if (!logger_) return; // logging not wired up - nothing to do
    if (!save_files) return; // logging disabled - nothing to do

    KalmanFilter* kf = output_tracks[slot];

    if (kf->validated) {
        // Already validated (and therefore already flushed) - log straight through.
        logger_->log(static_cast<uint64_t>(kf->getID()), ts, x_hat_data);
        return;
    }

    // Not validated (yet) - hold onto it in this slot's backlog. If the
    // track later validates, evaluateTracks() flushes this in order via
    // flushBacklog(). If it gets deleted first, deleteTrack() drops it.
    KalmanLogRecord rec;
    rec.track_id = static_cast<uint64_t>(kf->getID());
    rec.ts = ts;
    std::memcpy(rec.x_hat, x_hat_data, sizeof(rec.x_hat));
    log_backlogs_[slot].push_back(rec);
}

void TrackManager::flushBacklog(int slot)
{
    if (!logger_) {
        log_backlogs_[slot].clear();
        return;
    }
    if (!save_files) {
        return;
    }

    auto& backlog = log_backlogs_[slot];
    for (const auto& rec : backlog) {
        logger_->log(rec.track_id, rec.ts, rec.x_hat);
    }
    backlog.clear();
}

// for track summary logging
void TrackManager::recordValidationTime(int slot, double ts_now)
{
    if(!save_files) return;
    t_validated_[slot] = ts_now;
}

void TrackManager::writeSummary(int slot, uint32_t delete_reason)
{
    if (!summary_logger_) return;
    if (!save_files) return;

    KalmanFilter* kf = output_tracks[slot];

    TrackSummaryRecord rec{};
    rec.track_id = static_cast<uint64_t>(kf->getID());
    rec.t_created = kf->get_t0();
    rec.t_validated = t_validated_[slot];
    rec.t_deleted = kf->get_ts_last();
    rec.num_records = log_counts_[slot];
    rec.delete_reason = delete_reason;
    rec.event_rate_at_deletion = (kf->dt_moving_avg > 0.0) ? (1.0 / kf->dt_moving_avg) : 0.0;

    // End-of-life state/covariance, captured right now.
    Eigen::VectorXd x_final = kf->state().transpose();
    Eigen::MatrixXd P_final = kf->P_x();
    for (int k = 0; k < kSummaryStateDim; ++k) {
        rec.x_hat_at_deletion[k] = x_final(k);
    }
    for (int r = 0; r < kSummaryStateDim; ++r) {
        for (int c = 0; c < kSummaryStateDim; ++c) {
            rec.P_at_deletion[r * kSummaryStateDim + c] = P_final(r, c);
        }
    }

    summary_logger_->log(rec);
}

void TrackManager::flushAllSummaries()
{
    if (!save_files) return;

    for (int i = 0; i < static_cast<int>(output_tracks.size()); ++i) {
        if (output_tracks[i]->active && output_tracks[i]->validated) {
            writeSummary(i, TrackDeleteReason::STILL_ACTIVE_AT_SHUTDOWN);
        }
    }
}

//------------------------------------------//
//-----        PRIVATE FUNCTIONS       -----//
//------------------------------------------//
void TrackManager::deleteTrack(std::vector<KalmanFilter *> &track_array, int track_id, uint32_t delete_reason)
{
    KalmanFilter *kf = track_array[track_id];
    if (kf->active) {
        
        // if this was a real validated track, capture its summary for file writting
        if (kf->validated) {
            writeSummary(track_id, delete_reason);
        }

        kf->active = false; // Mark the Kalman filter as inactive
        active_track_count_--;

        // logging: clear out this track's log
        if (save_files) {
            log_backlogs_[track_id].clear();
            log_counts_[track_id] = 0;
            t_validated_[track_id] = -1.0;
        }
    }
}