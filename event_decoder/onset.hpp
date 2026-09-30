#ifndef ONSET_HPP
#define ONSET_HPP

#include <algorithm>
#include <cstddef>
#include <deque>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

class RealTimeOnset {
public:
    RealTimeOnset(
        double gradient_threshold = 200000.0,
        double amplitude_threshold = 5.0,
        double min_refractory = 100e-6,
        double drop_fraction = 0.30,
        double background_noise = 2.0,
        double reset_holdoff = 20e-6,
        double min_rebound = 2.0,
        int safegap = 3,
        bool store_history = false
    )
        : gradient_threshold_(gradient_threshold),
          amplitude_threshold_(amplitude_threshold),
          min_refractory_(min_refractory),
          drop_fraction_(drop_fraction),
          background_noise_(background_noise),
          reset_holdoff_(reset_holdoff),
          min_rebound_(min_rebound),
          safegap_(safegap),
          store_history_(store_history),
          armed_(true),
          peak_since_onset_(0.0),
          last_detection_time_(negative_infinity()),
          reset_candidate_time_(std::nullopt),
          valley_after_drop_(positive_infinity()),
          seen_drop_(false) {
        if (safegap_ < 0) {
            safegap_ = 0;
        }
    }

    std::optional<double> update(double t, double v) {
        append_sample(t, v);

        // Match Python: zero samples enter the buffer but cannot trigger.
        if (v == 0.0) {
            return std::nullopt;
        }

        const std::size_t required_size = buffer_size();
        if (buffer_.size() < required_size) {
            return std::nullopt;
        }

        const auto [t_start, v_start] = buffer_.front();
        const auto [t_end, v_end] = buffer_.back();

        // Match Python ordering exactly.
        if (!armed_) {
            update_rearm_state();
        }

        if ((t_start - last_detection_time_) < min_refractory_) {
            return std::nullopt;
        }

        if (!armed_) {
            return std::nullopt;
        }

        if (v_start < amplitude_threshold_) {
            return std::nullopt;
        }

        const double dt = t_end - t_start;
        if (dt <= 0.0) {
            return std::nullopt;
        }

        const double gradient = (v_end - v_start) / dt;
        if (gradient < gradient_threshold_) {
            return std::nullopt;
        }

        const double onset_time = t_start;

        if (store_history_) {
            onset_times_.push_back(onset_time);
        }

        armed_ = false;
        peak_since_onset_ = v_start;
        last_detection_time_ = t_start;
        reset_candidate_time_ = std::nullopt;
        valley_after_drop_ = positive_infinity();
        seen_drop_ = false;

        return onset_time;
    }

    const std::vector<double>& getOnsets() const noexcept {
        return onset_times_;
    }

    void clear() {
        buffer_.clear();
        armed_ = true;
        peak_since_onset_ = 0.0;
        last_detection_time_ = negative_infinity();
        reset_candidate_time_ = std::nullopt;
        valley_after_drop_ = positive_infinity();
        seen_drop_ = false;
        onset_times_.clear();
    }

private:
    static constexpr double negative_infinity() noexcept {
        return -std::numeric_limits<double>::infinity();
    }

    static constexpr double positive_infinity() noexcept {
        return std::numeric_limits<double>::infinity();
    }

    std::size_t buffer_size() const noexcept {
        return static_cast<std::size_t>(safegap_) + 1;
    }

    void append_sample(double t, double v) {
        buffer_.emplace_back(t, v);

        while (buffer_.size() > buffer_size()) {
            buffer_.pop_front();
        }
    }

    void update_rearm_state() {
        const auto [t_start, v_start] = buffer_.front();

        peak_since_onset_ = std::max(peak_since_onset_, v_start);
        const double drop_threshold =
            peak_since_onset_ * (1.0 - drop_fraction_);

        if (v_start < drop_threshold || v_start <= background_noise_) {
            seen_drop_ = true;

            if (!reset_candidate_time_.has_value()) {
                reset_candidate_time_ = t_start;
            }

            valley_after_drop_ = std::min(valley_after_drop_, v_start);

            if ((t_start - reset_candidate_time_.value()) >= reset_holdoff_) {
                const double rebound = v_start - valley_after_drop_;
                if (rebound >= min_rebound_) {
                    rearm();
                }
            }
        } else if (!seen_drop_) {
            reset_candidate_time_ = std::nullopt;
            valley_after_drop_ = positive_infinity();
        }
    }

    void rearm() {
        armed_ = true;
        peak_since_onset_ = 0.0;
        reset_candidate_time_ = std::nullopt;
        valley_after_drop_ = positive_infinity();
        seen_drop_ = false;
    }

private:
    double gradient_threshold_;
    double amplitude_threshold_;
    double min_refractory_;
    double drop_fraction_;
    double background_noise_;
    double reset_holdoff_;
    double min_rebound_;
    int safegap_;
    bool store_history_;

    std::deque<std::pair<double, double>> buffer_;

    bool armed_;
    double peak_since_onset_;
    double last_detection_time_;

    std::optional<double> reset_candidate_time_;
    double valley_after_drop_;
    bool seen_drop_;

    std::vector<double> onset_times_;
};

#endif // ONSET_HPP