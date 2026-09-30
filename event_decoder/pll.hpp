#ifndef PLL_HPP
#define PLL_HPP

#include <deque>
#include <vector>
#include <cmath>
#include <optional>
#include <array>

struct PLLUpdateResult {
    double onset_time;
    std::optional<double> snapped_time;
    std::optional<double> snap_error;
    bool was_snapped;
    std::vector<double> new_ticks;
    std::optional<double> phase;
    std::optional<double> error;
    std::optional<double> freq_adj;
};

class RealTimePLL {
    public:
        RealTimePLL(
            double bit_rate,
            double target_phase = 0.0,
            double alpha = 0.1,
            double beta = 0.001,
            std::optional<double> max_snap_error = std::nullopt,
            int tick_buffer_size = 32,
            bool store_history = false
        )
            : bit_rate_(bit_rate),
              target_phase_(std::fmod(target_phase, 1.0)),
              alpha_(alpha),
              beta_(beta),
              max_snap_error_(max_snap_error),
              tick_buffer_size_(tick_buffer_size),
              store_history_(store_history),
              T_bit_(1.0 / bit_rate),
              prev_onset_(std::nullopt),
              phase_(0.0),
              freq_adj_(0.0) {
        }
        
        PLLUpdateResult update(double onset_time) {
            double t = onset_time;
            
            // First onset just initializes timing state
            if (!prev_onset_.has_value()) {
                prev_onset_ = t;
                return PLLUpdateResult{
                    .onset_time = t,
                    .snapped_time = std::nullopt,
                    .snap_error = std::nullopt,
                    .was_snapped = false,
                    .new_ticks = {},
                    .phase = std::nullopt,
                    .error = std::nullopt,
                    .freq_adj = std::nullopt
                };
            }
            
            double dt = t - prev_onset_.value();
            
            // Invalid interval
            if (dt <= 0) {
                return PLLUpdateResult{
                    .onset_time = t,
                    .snapped_time = std::nullopt,
                    .snap_error = std::nullopt,
                    .was_snapped = false,
                    .new_ticks = {},
                    .phase = phase_,
                    .error = std::nullopt,
                    .freq_adj = freq_adj_
                };
            }
            
            double phase_before = phase_;
            double gap_in_bits = gapInBits(dt);
            double phase_unwrapped = phase_before + gap_in_bits;
            
            std::vector<double> new_ticks = interpolateTicks(
                prev_onset_.value(), t, dt, phase_before, phase_unwrapped, gap_in_bits
            );
            
            storeTicks(new_ticks);
            
            double peak_phase = phase_unwrapped;
            double phase_wrapped = std::fmod(phase_unwrapped, 1.0);
            if (phase_wrapped < 0) phase_wrapped += 1.0;
            
            double error = phaseError(phase_wrapped);
            freq_adj_ -= beta_ * error;
            phase_ = std::fmod(phase_wrapped - alpha_ * error, 1.0);
            if (phase_ < 0) phase_ += 1.0;
            
            auto [snapped_time, snap_error, was_snapped] = snapOnset(t);
            
            if (store_history_) {
                peak_phase_t_.push_back(peak_phase);
                residual_t_.push_back(phase_);
                error_t_.push_back(error);
                freq_adj_t_.push_back(freq_adj_);
                snapped_onsets_.push_back(snapped_time.value_or(t));
                snap_errors_.push_back(snap_error.value_or(0.0));
                snapped_mask_.push_back(was_snapped);
            }
            
            prev_onset_ = t;
            
            return PLLUpdateResult{
                .onset_time = t,
                .snapped_time = snapped_time,
                .snap_error = snap_error,
                .was_snapped = was_snapped,
                .new_ticks = new_ticks,
                .phase = phase_,
                .error = error,
                .freq_adj = freq_adj_
            };
        }
        
        std::vector<double> getClock() const {
            return recovered_clock_times_;
        }
        
        std::tuple<std::vector<double>, std::vector<double>, std::vector<bool>> getSnappedOnsets() const {
            return {snapped_onsets_, snap_errors_, snapped_mask_};
        }
        
        std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> getDiagnostics() const {
            return {peak_phase_t_, residual_t_, error_t_, freq_adj_t_};
        }
        
        void clear() {
            prev_onset_ = std::nullopt;
            phase_ = 0.0;
            freq_adj_ = 0.0;
            recent_ticks_.clear();
            recovered_clock_times_.clear();
            snapped_onsets_.clear();
            snap_errors_.clear();
            snapped_mask_.clear();
            peak_phase_t_.clear();
            residual_t_.clear();
            error_t_.clear();
            freq_adj_t_.clear();
        }
    
    private:
        double gapInBits(double dt) const {
            double T_bit_adj = T_bit_ / (1.0 + freq_adj_);
            return dt / T_bit_adj;
        }
        
        std::vector<double> interpolateTicks(
            double t0, double t1, double dt,
            double phase_before, double phase_unwrapped, double gap_in_bits
        ) const {
            int start_tick_index = static_cast<int>(std::floor(phase_before)) + 1;
            int end_tick_index = static_cast<int>(std::floor(phase_unwrapped));
            
            std::vector<double> ticks;
            ticks.reserve(static_cast<size_t>(end_tick_index - start_tick_index + 1));
            
            for (int tick_index = start_tick_index; tick_index <= end_tick_index; ++tick_index) {
                double frac = (static_cast<double>(tick_index) - phase_before) / gap_in_bits;
                double t_tick = t0 + frac * dt;
                ticks.push_back(t_tick);
            }
            
            return ticks;
        }
        
        void storeTicks(const std::vector<double>& ticks) {
            for (double t_tick : ticks) {
                recent_ticks_.push_back(t_tick);
                if (recent_ticks_.size() > static_cast<size_t>(tick_buffer_size_)) {
                    recent_ticks_.pop_front();
                }
                if (store_history_) {
                    recovered_clock_times_.push_back(t_tick);
                }
            }
        }
        
        double phaseError(double phase_wrapped) const {
            double error = phase_wrapped - target_phase_;
            error = std::fmod(error + 0.5, 1.0);
            if (error < 0) error += 1.0;
            return error - 0.5;
        }
        
        double predictNextTick(double now) const {
            double T_bit_adj = T_bit_ / (1.0 + freq_adj_);
            double phase_to_next_tick = 1.0 - phase_;
            return now + phase_to_next_tick * T_bit_adj;
        }
        
        std::tuple<std::optional<double>, std::optional<double>, bool> snapOnset(double onset_time) {
            std::vector<double> candidates;
            
            if (!recent_ticks_.empty()) {
                candidates.push_back(recent_ticks_.back());
            }
            
            candidates.push_back(predictNextTick(onset_time));
            
            double nearest_tick = candidates[0];
            double min_err = std::abs(candidates[0] - onset_time);
            
            for (size_t i = 1; i < candidates.size(); ++i) {
                double err = std::abs(candidates[i] - onset_time);
                if (err < min_err) {
                    min_err = err;
                    nearest_tick = candidates[i];
                }
            }
            
            double err = nearest_tick - onset_time;
            
            if (max_snap_error_.has_value() && std::abs(err) > max_snap_error_.value()) {
                return {std::nullopt, err, false};
            }
            
            return {nearest_tick, err, true};
        }
    
    private:
        // Parameters
        double bit_rate_;
        double target_phase_;
        double alpha_;
        double beta_;
        std::optional<double> max_snap_error_;
        int tick_buffer_size_;
        bool store_history_;
        
        // Derived
        double T_bit_;
        
        // State
        std::optional<double> prev_onset_;
        double phase_;
        double freq_adj_;
        
        // Buffers
        std::deque<double> recent_ticks_;
        
        // History (optional)
        std::vector<double> recovered_clock_times_;
        std::vector<double> snapped_onsets_;
        std::vector<double> snap_errors_;
        std::vector<bool> snapped_mask_;
        std::vector<double> peak_phase_t_;
        std::vector<double> residual_t_;
        std::vector<double> error_t_;
        std::vector<double> freq_adj_t_;
    };
    
    #endif // PLL_HPP