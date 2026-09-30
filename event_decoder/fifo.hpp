#ifndef FIFO_HPP
#define FIFO_HPP

#include <cstdint>
#include <deque>
#include <utility>

struct Event {
    std::uint64_t t{};
    std::uint16_t x{};
    std::uint16_t y{};
    bool p{};
};

class fifo {
public:
    explicit fifo(std::uint64_t window_size_us = 100)
        : window_size_us_(window_size_us) {}

    std::pair<std::uint64_t, int> update(const Event& event) {
        const int polarity = event.p ? 1 : 0;

        q_.emplace_back(event.t, polarity);
        running_sum_ += polarity;

        const std::uint64_t cutoff =
            event.t >= window_size_us_ ? event.t - window_size_us_ : 0;

        while (!q_.empty() && q_.front().first < cutoff) {
            running_sum_ -= q_.front().second;
            q_.pop_front();
        }

        return {event.t, running_sum_};
    }

    int get_sum() const noexcept {
        return running_sum_;
    }

    void clear() noexcept {
        q_.clear();
        running_sum_ = 0;
    }

private:
    std::uint64_t window_size_us_;
    std::deque<std::pair<std::uint64_t, int>> q_;
    int running_sum_ = 0;
};

#endif // FIFO_HPP