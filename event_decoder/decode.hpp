#ifndef DECODE_HPP
#define DECODE_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

class MFMHammingDecoder {
public:
    enum class Status : std::uint8_t {
        OK,
        CORRECTED,
        UNCORRECTABLE
    };

    struct DecodeResult {
        std::array<std::uint8_t, 3> bytes{};
        Status status = Status::UNCORRECTABLE;
        // Decoder-input time, in seconds, of the sentinel/start bit belonging
        // to this packet. Its zero point is the zero point of the input event
        // stream; the calibration pipeline uses the tracker-relative clock.
        double ts_start_bit_s = -1.0;
    };

    explicit MFMHammingDecoder(
        double unitT = 200e-6,
        int data_length = 32,
        int pkt_length = 35
    )
        : unitT_(unitT),
          data_length_(data_length),
          pkt_length_(pkt_length) {
        mfm_code_.reserve(256);
        binv_.reserve(128);
        binv_start_times_.reserve(128);
    }

    std::optional<DecodeResult> process_onset(double t) {
        if (!std::isfinite(t)) {
            return std::nullopt;
        }

        if (!last_t_.has_value()) {
            last_t_ = t;
            return std::nullopt;
        }

        const double dt = t - last_t_.value();
        last_t_ = t;
        current_packet_duration_ += dt;

        classify_gap(dt, t);
        pair_symbols();

        const auto result = extract_packet();
        if (result.has_value()) {
            current_packet_duration_ = 0.0;
        }
        return result;
    }

    void reset() {
        mfm_code_.clear();
        binv_.clear();
        binv_start_times_.clear();
        bin_start_found_ = false;
        mfm_read_head_ = 0;
        binv_read_head_ = 0;
        last_t_ = std::nullopt;
        ts_start_bit_ = -1.0;
        current_packet_duration_ = 0.0;
    }

    // Alias for callers that use the same lifecycle name as FIFO/PLL.
    void clear() {
        reset();
    }

    std::uint32_t packets_ok() const noexcept { return packets_ok_; }
    std::uint32_t packets_corrected() const noexcept { return packets_corrected_; }
    std::uint32_t packets_discarded() const noexcept { return packets_discarded_; }

private:
    void classify_gap(double dt, double t_now) {
        if (!(dt > 0.0) || !(unitT_ > 0.0)) {
            return;
        }

        if (dt > unitT_ * 0.85 && dt <= unitT_ * 1.15) {
            mfm_code_.emplace_back(0, t_now - 0.5 * dt);
            mfm_code_.emplace_back(1, t_now);
            return;
        }

        if (dt > unitT_ * 1.35 && dt <= unitT_ * 1.65) {
            const double prev_t = t_now - dt;
            const double delta = dt / 3.0;
            mfm_code_.emplace_back(0, prev_t + delta);
            mfm_code_.emplace_back(0, prev_t + 2.0 * delta);
            mfm_code_.emplace_back(1, t_now);
            return;
        }

        if (dt > unitT_ * 1.85 && dt <= unitT_ * 2.15) {
            const double prev_t = t_now - dt;
            const double delta = dt / 4.0;
            mfm_code_.emplace_back(0, prev_t + delta);
            mfm_code_.emplace_back(0, prev_t + 2.0 * delta);
            mfm_code_.emplace_back(0, prev_t + 3.0 * delta);
            mfm_code_.emplace_back(1, t_now);
            return;
        }

        if (dt > unitT_ * 2.35 && dt <= unitT_ * 2.65) {
            mfm_code_.emplace_back(-2, t_now);
            return;
        }

        snap_to_nearest_symbol(t_now);
    }

    void snap_to_nearest_symbol(double t_now) {
        if (mfm_code_.empty()) {
            return;
        }

        const double last_sym_t = mfm_code_.back().second;
        const std::array<double, 4> expected_one = {
            last_sym_t + unitT_,
            last_sym_t + 1.5 * unitT_,
            last_sym_t + 2.0 * unitT_,
            last_sym_t + 2.5 * unitT_
        };
        const std::array<double, 4> expected_zero = {
            last_sym_t + 0.5 * unitT_,
            last_sym_t + unitT_,
            last_sym_t + 1.5 * unitT_,
            last_sym_t + 2.0 * unitT_
        };

        int index = 0;
        double best_diff = std::abs(expected_one[0] - t_now);
        for (int k = 1; k < 4; ++k) {
            const double diff = std::abs(expected_one[k] - t_now);
            if (diff < best_diff) {
                best_diff = diff;
                index = k;
            }
        }

        if (index == 3) {
            mfm_code_.emplace_back(-2, expected_one[3]);
        } else if (index == 2) {
            mfm_code_.emplace_back(0, expected_zero[0]);
            mfm_code_.emplace_back(0, expected_zero[1]);
            mfm_code_.emplace_back(0, expected_zero[2]);
            mfm_code_.emplace_back(1, expected_one[2]);
        } else if (index == 1) {
            mfm_code_.emplace_back(0, expected_zero[0]);
            mfm_code_.emplace_back(0, expected_zero[1]);
            mfm_code_.emplace_back(1, expected_one[1]);
        } else {
            mfm_code_.emplace_back(0, expected_zero[0]);
            mfm_code_.emplace_back(1, expected_one[0]);
        }
    }

    void pair_symbols() {
        const std::size_t n = mfm_code_.size();

        if (!bin_start_found_) {
            find_packet_sync();
            if (!bin_start_found_) {
                return;
            }
        }

        std::size_t i = mfm_read_head_;
        while (i < n) {
            if (i == 0 || i + 1 >= n) {
                mfm_read_head_ = i;
                return;
            }

            const std::int8_t prev_value = mfm_code_[i - 1].first;
            const std::int8_t curr_value = mfm_code_[i].first;

            if (prev_value == -2 || curr_value == -2) {
                const std::size_t sentinel_i =
                    prev_value == -2 ? i - 1 : i;
                ts_start_bit_ = mfm_code_[sentinel_i].second;
                binv_.push_back(-2);
                binv_start_times_.push_back(ts_start_bit_);
                ++i;
                mfm_read_head_ = i;
                continue;
            }

            binv_.push_back(
                static_cast<std::int8_t>(prev_value == 0 && curr_value == 1)
            );
            // Only sentinel entries have a packet-start timestamp.
            binv_start_times_.push_back(-1.0);
            i += 2;
            mfm_read_head_ = i;
        }
    }

    void find_packet_sync() {
        const std::size_t n = mfm_code_.size();
        if (n < 65) {
            return;
        }

        const std::size_t limit = n - 65;
        for (std::size_t i = 1; i <= limit; ++i) {
            const std::size_t candidate = i + 1;
            const auto word = pairs_to_bits(candidate);
            if (!word.has_value()) {
                continue;
            }

            const auto [syndrome, overall_ok] = check_parity(word.value());
            if (syndrome == 0 && overall_ok) {
                bin_start_found_ = true;
                mfm_read_head_ = candidate;
                return;
            }
        }
    }

    std::optional<std::uint32_t> pairs_to_bits(std::size_t start) const {
        if (start + 64 > mfm_code_.size()) {
            return std::nullopt;
        }

        std::uint32_t word = 0;
        int bit_index = 0;
        for (std::size_t i = start; i < start + 64; i += 2) {
            const std::int8_t first = mfm_code_[i].first;
            const std::int8_t second = mfm_code_[i + 1].first;
            if (first == -2 || second == -2) {
                return std::nullopt;
            }
            if (first == 0 && second == 1) {
                word |= (1u << bit_index);
            }
            ++bit_index;
        }
        return word;
    }

    std::optional<DecodeResult> extract_packet() {
        const std::size_t n = binv_.size();
        std::size_t i = binv_read_head_;

        while (i < n) {
            if (binv_[i] != -2) {
                ++i;
                continue;
            }

            const std::size_t data_start = i + 1;
            if (i + static_cast<std::size_t>(pkt_length_) > n ||
                data_start + static_cast<std::size_t>(data_length_) > n) {
                binv_read_head_ = i;
                return std::nullopt;
            }
            // Save the timestamp associated with this particular sentinel.
            // Do not read the class-wide ts_start_bit_ later: pair_symbols()
            // may already have encountered the start of a newer packet.
            const double packet_start_bit_s = binv_start_times_[i];

            std::uint32_t word = 0;
            bool valid = true;
            for (int k = 0; k < data_length_; ++k) {
                const int value = binv_[data_start + static_cast<std::size_t>(k)];
                if (value < 0) {
                    binv_read_head_ = data_start + static_cast<std::size_t>(k);
                    i = binv_read_head_;
                    valid = false;
                    break;
                }
                word |= static_cast<std::uint32_t>(value) << k;
            }

            if (!valid) {
                continue;
            }

            const auto [bytes, status] = hamming_decode(word);
            binv_read_head_ = data_start + static_cast<std::size_t>(data_length_);

            if (status == Status::UNCORRECTABLE) {
                ++packets_discarded_;
                i = binv_read_head_;
                continue;
            }

            if (status == Status::CORRECTED) {
                ++packets_corrected_;
            } else {
                ++packets_ok_;
            }

            return DecodeResult{bytes, status, packet_start_bit_s};
        }

        binv_read_head_ = i;
        return std::nullopt;
    }

    std::pair<std::array<std::uint8_t, 3>, Status>
    hamming_decode(std::uint32_t word) const {
        auto [syndrome, overall_ok] = check_parity(word);
        Status status;

        if (syndrome == 0 && overall_ok) {
            status = Status::OK;
        } else if (syndrome != 0 && !overall_ok) {
            if (syndrome < 1 || syndrome > 31) {
                return {{}, Status::UNCORRECTABLE};
            }
            word ^= (1u << (syndrome - 1));
            status = Status::CORRECTED;
        } else if (syndrome == 0 && !overall_ok) {
            status = Status::CORRECTED;
        } else {
            return {{}, Status::UNCORRECTABLE};
        }

        // const std::uint8_t c1 = static_cast<std::uint8_t>(
        //     ((word >> 5) & 0x03u) | (((word >> 8) & 0x3Fu) << 2)
        // );
        // const std::uint8_t c2 = static_cast<std::uint8_t>(
        //     ((word >> 14) & 0x01u) | (((word >> 16) & 0x3Fu) << 1)
        // );
        // const std::uint8_t c3 = static_cast<std::uint8_t>((word >> 22) & 0xFFu);

        const std::uint8_t c1 = static_cast<std::uint8_t>(
            ((word >> 5) & 0x03u) | (((word >> 8) & 0x3Fu) << 2)
        );

        const std::uint8_t c2 = static_cast<std::uint8_t>(
            ((word >> 14) & 0x01u) |
            (((word >> 16) & 0x7Fu) << 1)
        );

        const std::uint8_t c3 = static_cast<std::uint8_t>(
            (word >> 23) & 0xFFu
        );



        return {{c1, c2, c3}, status};
    }

    static const std::array<std::array<int, 256>, 4>& syndrome_tables() {
        static const auto tables = [] {
            std::array<std::array<int, 256>, 4> result{};
            for (int byte_index = 0; byte_index < 4; ++byte_index) {
                const int base_position = byte_index * 8;
                for (int value = 0; value < 256; ++value) {
                    int syndrome = 0;
                    for (int bit = 0; bit < 8; ++bit) {
                        if (value & (1 << bit)) {
                            const int position = base_position + bit + 1;
                            if (position <= 31) {
                                syndrome ^= position;
                            }
                        }
                    }
                    result[byte_index][value] = syndrome;
                }
            }
            result[3][0x80] = 0;
            return result;
        }();
        return tables;
    }

    std::pair<int, bool> check_parity(std::uint32_t word) const {
        const auto& tables = syndrome_tables();
        const int syndrome =
            tables[0][word & 0xFFu] ^
            tables[1][(word >> 8) & 0xFFu] ^
            tables[2][(word >> 16) & 0xFFu] ^
            tables[3][(word >> 24) & 0xFFu];

        std::uint32_t value = word;
        bool odd_parity = false;
        while (value != 0u) {
            odd_parity = !odd_parity;
            value &= value - 1u;
        }

        return {syndrome, !odd_parity};
    }

private:
    double unitT_;
    int data_length_;
    int pkt_length_;

    double current_packet_duration_ = 0.0;
    std::optional<double> last_t_;
    double ts_start_bit_ = -1.0;
    std::size_t mfm_read_head_ = 0;
    std::size_t binv_read_head_ = 0;
    bool bin_start_found_ = false;

    std::uint32_t packets_ok_ = 0;
    std::uint32_t packets_corrected_ = 0;
    std::uint32_t packets_discarded_ = 0;

    std::vector<std::pair<std::int8_t, double>> mfm_code_;
    std::vector<std::int8_t> binv_;
    // Parallel to binv_: timestamp for -2 sentinel entries, -1 otherwise.
    std::vector<double> binv_start_times_;
};

#endif // DECODE_HPP