#pragma once

#include <nlohmann/json.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace awakening::record {

// A bounded, asynchronous diagnostic sink. It never publishes control messages.
class DiagnosticsRecorder {
public:
    DiagnosticsRecorder() = default;
    DiagnosticsRecorder(const DiagnosticsRecorder&) = delete;
    DiagnosticsRecorder& operator=(const DiagnosticsRecorder&) = delete;
    ~DiagnosticsRecorder() { close(); }

    bool open(const std::filesystem::path& parent, bool enable, size_t capacity = 4096) {
        close();
        if (!enable) return false;
        std::error_code error;
        const auto run_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        auto directory = parent / ("diagnostics_" + std::to_string(run_ns));
        if (!std::filesystem::create_directories(directory, error) || error) return false;
        path_ = directory / "events.jsonl";
        output_.open(path_, std::ios::out | std::ios::trunc);
        if (!output_) return false;
        capacity_ = capacity > 0 ? capacity : 1;
        dropped_ = 0;
        written_ = 0;
        stopping_ = false;
        enabled_ = true;
        try {
            worker_ = std::thread([this] { run(); });
        } catch (const std::exception&) {
            enabled_ = false;
            stopping_ = true;
            output_.close();
            return false;
        }
        return true;
    }

    bool enabled() const noexcept { return enabled_.load(); }
    const std::filesystem::path& path() const noexcept { return path_; }
    uint64_t dropped() const noexcept { return dropped_.load(); }
    uint64_t written() const noexcept { return written_.load(); }

    bool write(const nlohmann::json& event) {
        if (!enabled()) return false;
        std::string line = event.dump();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_ || !enabled()) return false;
            if (queue_.size() >= capacity_) {
                ++dropped_;
                return false;
            }
            queue_.push_back(std::move(line));
        }
        ready_.notify_one();
        return true;
    }

    // Producers stop accepting events, then the writer drains accepted events.
    void close() {
        enabled_ = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        ready_.notify_one();
        if (worker_.joinable()) worker_.join();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            dropped_ += queue_.size();
            queue_.clear();
        }
        if (output_.is_open()) output_.close();
    }

private:
    void run() {
        auto last_flush = std::chrono::steady_clock::now();
        auto last_status = last_flush;
        while (true) {
            std::deque<std::string> batch;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait_for(lock, std::chrono::milliseconds(250),
                                [this] { return stopping_ || !queue_.empty(); });
                batch.swap(queue_);
                if (batch.empty() && stopping_) break;
            }
            for (size_t i = 0; i < batch.size(); ++i) {
                output_ << batch[i] << '\n';
                if (!output_) {
                    enabled_ = false;
                    dropped_ += batch.size() - i;
                    return;
                }
                ++written_;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now - last_status >= std::chrono::seconds(1)) {
                status();
                last_status = now;
            }
            if (now - last_flush >= std::chrono::milliseconds(250)) {
                output_.flush();
                last_flush = now;
            }
        }
        status();
        output_.flush();
    }

    void status() {
        output_ << nlohmann::json({{"kind", "recorder_status"},
                                   {"written", written()}, {"dropped", dropped()}}).dump()
                << '\n';
    }

    std::atomic<bool> enabled_ {false};
    std::atomic<uint64_t> dropped_ {0};
    std::atomic<uint64_t> written_ {0};
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<std::string> queue_;
    std::thread worker_;
    std::ofstream output_;
    std::filesystem::path path_;
    size_t capacity_ = 4096;
    bool stopping_ = true;
};

inline DiagnosticsRecorder& diagnostics() {
    static DiagnosticsRecorder recorder;
    return recorder;
}

} // namespace awakening::record
