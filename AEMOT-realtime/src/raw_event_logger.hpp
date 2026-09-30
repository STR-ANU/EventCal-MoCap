#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Binary log record for one raw associated event - [t, x, y, p].
// This is a plain intermediate format for speed - will need to be 
// converted to .es or another file type for processing off-line.

struct RawEventRecord {
    double t;
    uint32_t track_id;
    uint16_t x;
    uint16_t y;
    uint8_t p; // 1 = ON, 0 = OFF
    uint8_t reserved[7] = {0, 0, 0, 0, 0, 0, 0};
};

struct RawEventLogFileHeader {
    char magic[8] = {'A', 'E', 'M', 'O', 'T', 'E', 'V', 'T'};
    uint32_t version = 1;
    uint32_t reserved = 0;
    uint32_t record_size = sizeof(RawEventRecord);
    uint32_t reserved2 = 0;
};
// sizeof(RawEventLogFileHeader) == 24 - same as the other AEMOT log files

class RawEventLogger {
public:
    // buffer_capacity_records: default 16384 -> 256KB per buffer
    explicit RawEventLogger(const std::string& output_path,
                            size_t buffer_capacity_records = 16384,
                            int num_buffers = 4);
    ~RawEventLogger();

    RawEventLogger(const RawEventLogger&) = delete;
    RawEventLogger& operator=(const RawEventLogger&) = delete;

    // Appends one record
    inline void log(double t, double x, double y, int p, int track_id) {
        RawEventRecord rec;
        rec.t = t;
        rec.track_id = static_cast<uint32_t>(track_id);
        rec.x = static_cast<uint16_t>(x);
        rec.y = static_cast<uint16_t>(y);
        rec.p = static_cast<uint8_t>(p > 0 ? 1 : 0);

        current_buffer_->push_back(rec);
        if (current_buffer_->size() >= buffer_capacity_records_) {
            enqueue_current_buffer();
        }
    }

    // flushes any partially-filled buffer and stops the writer thread
    void close();

private:
    void writer_thread_main();
    void enqueue_current_buffer();
    std::vector<RawEventRecord>* acquire_free_buffer();

    std::string output_path_;
    size_t buffer_capacity_records_;
    int num_buffers_;

    std::vector<std::vector<RawEventRecord>*> owned_buffers_;
    std::vector<RawEventRecord>* current_buffer_ = nullptr;


    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<std::vector<RawEventRecord>*> filled_queue_;
    std::deque<std::vector<RawEventRecord>*> free_queue_;
    bool stopping_ = false;

    std::thread writer_thread_;
    bool closed_ = false;
};