#include "processing.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <vector>
#include <iomanip>

#include "yaml-cpp/yaml.h"      

#include "event_queue.hpp"
#include "fifo.hpp"
#include "onset.hpp"
#include "pll.hpp"
#include "decode.hpp"
#include "sepia.hpp"
#include "parameters.h"     


namespace processing {
namespace {

Parameters params;

std::uint64_t total_events = 0;
std::uint64_t roi_events = 0;

double t_start_s = -1.0;
double t_end_s = -1.0;
std::int64_t t_start_us = -1;
bool first_event_seen = false;

std::ofstream csv_file;
bool csv_open = false;

// The FIFO uses integer microseconds. 100 us is therefore passed as 100.
fifo roi_fifo(100);
double fifo_sleep_s = 100e-6;
double skip_fifo_until_s = -1.0;

RealTimeOnset onset_detector;
std::optional<double> prev_onset;

RealTimePLL digital_pll(1e4, 0.0, 0.1, 0.001,
    std::nullopt, 32, true);

MFMHammingDecoder decoder(
    200e-6,          // unitT
    32,              // data length
    35               // packet length
);

const char* status_string(MFMHammingDecoder::Status status) {
    switch (status) {
    case MFMHammingDecoder::Status::OK:
        return "ok";
    case MFMHammingDecoder::Status::CORRECTED:
        return "corrected";
    case MFMHammingDecoder::Status::UNCORRECTABLE:
        return "uncorrectable";
    }
    return "unknown";
}

void print_decoded_packet(
    const MFMHammingDecoder::DecodeResult& result
) {
    const auto& bytes = result.bytes;

    std::cout << "[DECODE] bytes=["
              << static_cast<unsigned int>(bytes[0]) << ", "
              << static_cast<unsigned int>(bytes[1]) << ", "
              << static_cast<unsigned int>(bytes[2]) << "]"
              << " status=" << status_string(result.status)
              << '\n';
}

void write_csv(
    std::uint64_t t_us,
    int sum,
    const std::optional<double>& onset_time = std::nullopt,
    const std::optional<double>& snapped_time = std::nullopt,
    bool was_snapped = false
) {   
    if (!csv_open || !csv_file.is_open()) {
        return;
    }

    csv_file << t_us << ',' << sum << ',';
    if (onset_time.has_value()) {
        csv_file << onset_time.value();
    }
    csv_file << ',';
    if (snapped_time.has_value()) {
        csv_file << snapped_time.value();
    }
    csv_file << ',' << (was_snapped ? 1 : 0) << '\n';
}

} // namespace

bool setup(const std::string& config_name) {
    // std::cout << "[processing] setup called with config: "
    //           << config_name << '\n';

    total_events = 0;
    roi_events = 0;
    roi_fifo.clear();
    onset_detector.clear();
    digital_pll.clear();
    decoder.reset();
    first_event_seen = false;
    t_start_us = -1;
    skip_fifo_until_s = -1.0;
    prev_onset = std::nullopt;

    // Reset defaults on every setup call.
    t_start_s = -1.0;
    t_end_s = -1.0;

    // std::cout << "[processing] time window: t_start="
    //           << (t_start_s < 0.0 ? "none" : std::to_string(t_start_s))
    //           << " s, t_end="
    //           << (t_end_s < 0.0 ? "none" : std::to_string(t_end_s))
    //           << " s\n";

    // csv_file.open("roi_fifo.csv", std::ios::out | std::ios::trunc);
    // if (!csv_file.is_open()) {
    //     std::cerr << "[processing] failed to open roi_fifo.csv for writing\n";
    //     csv_open = false;
    // } else {
    //     csv_file << "t_us,sum,onset_time,snapped_time,was_snapped\n";
    //     csv_open = true;
    // }   

    csv_file.open("decoder_output.csv", std::ios::out | std::ios::trunc);
    if (!csv_file.is_open()) {
        std::cerr << "[processing] failed to open roi_fifo.csv for writing\n";
        csv_open = false;
    } else {
        csv_file << "t_us,ts_start_bit_s,byte1,byte2,byte3\n";
        csv_open = true;
    }   

    // PARSE THE CONFIG //
    try {
        // std::cout << "configs/" + config_name + ".yaml" << std::endl;
        // params = loadParametersFromYAML("configs/" + config_name + ".yaml");
        params = loadParametersFromYAML(config_name);
    } catch (const std::exception &ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return false; // Fail fast if config is missing/invalid
    }

    return true;
}
void process_batch(const std::vector<sepia::dvs_event>& events) {
    total_events += events.size();

    int cx = params.led_px;
    int cy = 720 - params.led_py;
    int r = params.led_r;


    for (const auto& e : events) {
        if (!first_event_seen) {
            t_start_us = static_cast<std::int64_t>(e.t);
            first_event_seen = true;
        }

        const double relative_t_s =
            static_cast<double>(static_cast<std::int64_t>(e.t) - t_start_us)
            * 1e-6;

        if (t_start_s >= 0.0 && relative_t_s < t_start_s) {
            continue;
        }

        if (t_end_s >= 0.0 && relative_t_s > t_end_s) {
            // std::cout << "[processing] reached t_end="
            //           << t_end_s << " s, stopping\n";
            return;
        }

        if (params.f_fixed_coordinates){
            const int dx = std::abs(static_cast<int>(e.x) - cx);
            const int dy = std::abs(static_cast<int>(e.y) - cy);
            if (dx > r || dy > r) {
                continue;
            }
        }
        // This matches the Python pipeline: only ON events reach the FIFO.
        if (!e.on) {
            continue;
        }
        
        ++roi_events;

        Event event{
            static_cast<std::uint64_t>(e.t),
            static_cast<std::uint16_t>(e.x),
            static_cast<std::uint16_t>(e.y),
            static_cast<bool>(e.on)
        };

        // Seconds in the input event stream's timebase are used consistently
        // by the onset detector, PLL, and decoder. In the calibration pipeline,
        // the per-track .es converter preserves the tracker's t1-relative
        // clock, so ts_start_bit_s can be compared directly with tracker ts.
        const double t_sec = static_cast<double>(e.t) * 1e-6;

        if (t_sec < skip_fifo_until_s) {
            onset_detector.update(t_sec, 0.0);
            // write_csv(event.t, 0);
            continue;
        }

        const auto [t_us, sum] = roi_fifo.update(event);
        const auto onset_time =
            onset_detector.update(t_sec, static_cast<double>(sum));

        if (!onset_time.has_value()) {
            // write_csv(t_us, sum);
            continue;
        }

        skip_fifo_until_s = onset_time.value() + fifo_sleep_s;
        prev_onset = onset_time;
        roi_fifo.clear();

        // Update the PLL only for a genuine onset.
        const PLLUpdateResult pll_result =
            digital_pll.update(onset_time.value());

        // Decode using the snapped onset when available. On the first PLL
        // onset, snapped_time is empty, so use the original onset to preserve
        // the decoder's continuous timing stream.
        const double decoder_time =
            pll_result.snapped_time.value_or(onset_time.value());

        const auto decode_result = decoder.process_onset(decoder_time);
        if (decode_result.has_value()) {
            // print_decoded_packet(decode_result.value());

            // Write decoder output to file
            const auto& bytes = decode_result.value().bytes;

            csv_file << t_us << ','
                     << std::setprecision(17)
                     << decode_result.value().ts_start_bit_s << ','
                     << static_cast<unsigned int>(bytes[0]) << ','
                     << static_cast<unsigned int>(bytes[1]) << ','
                     << static_cast<unsigned int>(bytes[2]) << '\n';
        }

        // write_csv(
        //     t_us,
        //     sum,
        //     onset_time,
        //     pll_result.snapped_time,
        //     pll_result.was_snapped
        // );

        // std::cout << "[ONSET] t=" << onset_time.value();
        // if (pll_result.snapped_time.has_value()) {
        //     std::cout << " snapped_t="
        //               << pll_result.snapped_time.value();
        // } else {
        //     std::cout << " snapped_t=none";
        // }
        // std::cout << " sum=" << sum
        //           << " sleep_until=" << skip_fifo_until_s
        //           << " new_ticks=" << pll_result.new_ticks.size()
        //           << '\n';
    }
}

void teardown() {
    if (csv_open && csv_file.is_open()) {
        csv_file.close();
    }
    csv_open = false;

    // std::cout << "[processing] total events processed: "
    //           << total_events << '\n';
    // std::cout << "[processing] ROI events (x~=741, y~=198, r=7): "
    //           << roi_events << '\n';
    // std::cout << "[processing] output written to roi_fifo.csv\n";
    // std::cout << "[processing] recovered clock ticks: "
    //           << digital_pll.getClock().size() << '\n';
    // std::cout << "[processing] decoded packets: ok="
    //           << decoder.packets_ok()
    //           << ", corrected=" << decoder.packets_corrected()
    //           << ", discarded=" << decoder.packets_discarded()
    //           << '\n';
}
} // namespace processing
