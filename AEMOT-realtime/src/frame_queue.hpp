// Code genrated by Claude AI for this setup.

#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>

#include <condition_variable>
#include <deque>
#include <mutex>
#include <vector>

// One track's state, captured BY VALUE at the moment a frame is published.
// KalmanFilter::state()/P_x() already return copies (not references), so
// this is safe to hand across threads even though the pool slot it came
// from can be reset() and reused for a completely different physical
// track before the render thread gets around to drawing it.
struct track_snapshot {
    Eigen::MatrixXd state;
    Eigen::MatrixXd P;
    bool validated;
};

// Everything the render thread needs to draw and save one frame, entirely
// self-contained. The render thread never touches shared tracking state
// (log_intensity_state, the TrackManager, etc) directly - only this.
struct frame_job {
    cv::Mat log_intensity_snapshot; // a .clone() of the SAE image at publish time, not a view of it
    double ts = 0.0;
    std::vector<track_snapshot> tracks;
};

// A bounded queue of frame_job's between the processing thread (producer)
// and the render thread (consumer). Small and drop-oldest-on-full, on
// purpose: frames are for a human to look at, so if rendering falls
// behind, showing the latest frame (dropping stale ones) is more useful
// than buffering a backlog and drawing an increasingly out-of-date view.
// This is a DIFFERENT policy from event_queue's push_blocking() - frames
// are disposable, events aren't.
class frame_queue {
    public:
    explicit frame_queue(std::size_t max_frames) : _max_frames(max_frames) {}

    void push(frame_job&& job) {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_frames.size() >= _max_frames) {
                _frames.pop_front();
            }
            _frames.push_back(std::move(job));
        }
        _condition_variable.notify_all();
    }

    // Blocks until a frame is available or the queue is stopped. Returns
    // false only when stopped and empty.
    bool pop(frame_job& job) {
        std::unique_lock<std::mutex> lock(_mutex);
        _condition_variable.wait(lock, [&] { return !_frames.empty() || _stopped; });
        if (_frames.empty()) {
            return false;
        }
        job = std::move(_frames.front());
        _frames.pop_front();
        return true;
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _stopped = true;
        }
        _condition_variable.notify_all();
    }

    private:
    std::size_t _max_frames;
    std::deque<frame_job> _frames;
    std::mutex _mutex;
    std::condition_variable _condition_variable;
    bool _stopped = false;
};