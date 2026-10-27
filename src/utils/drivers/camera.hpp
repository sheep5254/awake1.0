#pragma once

#include "utils/common/image.hpp"
#include "utils/logger.hpp"
#include "utils/scheduler/scheduler.hpp"
#include <atomic>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>

namespace awakening {

class Camera {
public:
    explicit Camera(Scheduler* scheduler = nullptr): scheduler_(scheduler) {}
    virtual ~Camera() {
        stop_source_thread();
    }

    virtual void init() {}
    virtual void stop() {
        stop_source_thread();
    }
    virtual void restart() {}
    virtual bool is_running() const = 0;
    virtual ImageFrame read() = 0;
    virtual bool start_capture() {
        return true;
    }
    virtual double get_ExposureTime() const {
        return 0.0;
    }
    virtual void set_ExposureTime(double) {
        AWAKENING_WARN("camera does not support exposure control");
    }

    template<typename Tag>
    void start(const std::string& source_name) {
        if (!scheduler_) {
            throw std::runtime_error("camera source start requires a Scheduler");
        }
        using IO = IOPair<Tag, ImageFrame>;
        source_snapshot_id_ = scheduler_->register_source<IO>(source_name);
        if (!start_capture()) {
            AWAKENING_ERROR("camera failed to start capture");
            return;
        }
        source_thread_running_ = true;
        source_thread_ = std::thread(&Camera::run_source_loop<IO>, this);
    }

protected:
    void stop_source_thread() {
        source_thread_running_ = false;
        if (source_thread_.joinable()) {
            source_thread_.join();
        }
    }

    template<typename IO>
    void run_source_loop() {
        int fail_count = 0;
        int restart_count = 0;
        while (source_thread_running_ && is_running()) {
            auto frame = read();
            if (frame.src_img.empty()) {
                if (++fail_count > 10) {
                    ++restart_count;
                    AWAKENING_WARN(
                        "camera read failed too many times, restarting（第 {} 次）", restart_count);
                    restart();
                    // 重启退避：1.5s、3s、4.5s…最多 4.5s，避免相机卡死时疯狂重开
                    std::this_thread::sleep_for(std::chrono::milliseconds(
                        std::min(1500 * restart_count, 4500)));
                    if (restart_count == 5) {
                        AWAKENING_ERROR(
                            "相机连续 {} 次重启仍无图像：请检查 USB3 线/换一个 USB3 口/供电"
                            "（可用 sudo sh -c 'echo 0 > /sys/bus/usb/devices/4-2/authorized; "
                            "sleep 1; echo 1 > /sys/bus/usb/devices/4-2/authorized' 复位）",
                            restart_count);
                    }
                    fail_count = 0;
                }
                continue;
            }

            fail_count = 0;
            restart_count = 0;
            scheduler_->runtime_push_source<IO>(
                source_snapshot_id_,
                [f = std::move(frame)]() mutable {
                    return std::make_tuple(std::optional<typename IO::second_type>(std::move(f)));
                }
            );
        }
    }

    Scheduler* scheduler_ = nullptr;
    size_t source_snapshot_id_ {};
    std::atomic<bool> source_thread_running_ { false };
    std::thread source_thread_;
};

} // namespace awakening
