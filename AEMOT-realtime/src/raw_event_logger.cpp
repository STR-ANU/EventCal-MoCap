#include "raw_event_logger.hpp"

#include <stdexcept>

RawEventLogger::RawEventLogger(const std::string& output_path,
                                size_t buffer_capacity_records,
                                int num_buffers)
    : output_path_(output_path),
      buffer_capacity_records_(buffer_capacity_records),
      num_buffers_(num_buffers)
{
    {
        std::ofstream test(output_path_, std::ios::binary | std::ios::trunc);
        if (!test.is_open()) {
            throw std::runtime_error("[RawEventLogger]: could not open output file: " + output_path_);
        }
    }

    owned_buffers_.reserve(num_buffers_);
    for (int i = 0; i < num_buffers_; i++) {
        auto* buf = new std::vector<RawEventRecord>();
        buf->reserve(buffer_capacity_records_);
        owned_buffers_.push_back(buf);
        if (i == 0) {
            current_buffer_ = buf;
        } else {
            free_queue_.push_back(buf);
        }
    }

    writer_thread_ = std::thread(&RawEventLogger::writer_thread_main, this);
}

RawEventLogger::~RawEventLogger() {
    close();
    for (auto* buf : owned_buffers_) {
        delete buf;
    }
}

void RawEventLogger::enqueue_current_buffer() {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        filled_queue_.push_back(current_buffer_);
    }
    queue_cv_.notify_all();
    current_buffer_ = acquire_free_buffer();
}

std::vector<RawEventRecord>* RawEventLogger::acquire_free_buffer() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_cv_.wait(lock, [&] { return !free_queue_.empty(); });
    auto* buf = free_queue_.front();
    free_queue_.pop_front();
    return buf;
}

void RawEventLogger::writer_thread_main() {
    std::ofstream file(output_path_, std::ios::binary | std::ios::trunc);
    if (!file.is_open()) {
        return;
    }
 
    RawEventLogFileHeader header;
    file.write(reinterpret_cast<const char*>(&header), sizeof(header));
 
    while (true) {
        std::vector<RawEventRecord>* buf = nullptr;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [&] { return !filled_queue_.empty() || stopping_; });
            if (filled_queue_.empty() && stopping_) {
                break;
            }
            buf = filled_queue_.front();
            filled_queue_.pop_front();
        }
 
        if (!buf->empty()) {
            file.write(reinterpret_cast<const char*>(buf->data()),
                       static_cast<std::streamsize>(buf->size() * sizeof(RawEventRecord)));
            buf->clear();
        }
 
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            free_queue_.push_back(buf);
        }
        queue_cv_.notify_all();
    }
 
    file.flush();
    file.close();
}
 
void RawEventLogger::close() {
    if (closed_) return;
    closed_ = true;
 
    if (current_buffer_ != nullptr) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!current_buffer_->empty()) {
            filled_queue_.push_back(current_buffer_);
        } else {
            free_queue_.push_back(current_buffer_);
        }
        current_buffer_ = nullptr;
    }
 
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        stopping_ = true;
    }
    queue_cv_.notify_all();
 
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
}
