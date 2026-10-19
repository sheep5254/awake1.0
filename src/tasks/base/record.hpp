#pragma once
// 自瞄逐帧诊断记录器（纯写文件，不改任何处理逻辑）
//
//   * 每次启动自瞄都以 trunc 方式打开**同一个文件** => 上一次的记录被自动覆盖，
//     所以永远只有最新一次运行的记录（路径 record/auto_aim/run.jsonl）。
//   * 每个处理帧写一行 JSON（JSONL），可以直接丢给分析脚本/我。
//   * 记录内容包括：本帧检测到的每块板（号码的两种来源、置信度、像素框、PnP 位置）、
//     跟踪器状态（状态机/目标号码/车心位置/速度/vyaw/yaw/r1/r2/h/jumped）、
//     预测装甲板（odom 位置 + 投影像素）、图像时刻的云台姿态（IMU）、
//     以及最近一次解算出的云台指令（含与指令的时间差 age_ms）。
#include <nlohmann/json.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

namespace awakening::record {

    class RunRecorder {
    public:
        void open(const std::string& path, bool enable, int flush_every = 20) {
            std::lock_guard<std::mutex> lk(mutex_);
            path_ = path;
            enable_ = enable;
            flush_every_ = flush_every > 0 ? flush_every : 1;
            lines_ = 0;
            if (!enable_) {
                return;
            }
            std::error_code ec;
            const auto parent = std::filesystem::path(path).parent_path();
            if (!parent.empty()) {
                std::filesystem::create_directories(parent, ec);
            }
            // trunc：每次启动覆盖旧记录
            ofs_.open(path, std::ios::out | std::ios::trunc);
            if (!ofs_.is_open()) {
                enable_ = false;
            }
        }

        [[nodiscard]] bool enabled() const noexcept {
            return enable_;
        }

        [[nodiscard]] const std::string& path() const noexcept {
            return path_;
        }

        void write(const nlohmann::json& j) {
            if (!enable_) {
                return;
            }
            std::lock_guard<std::mutex> lk(mutex_);
            ofs_ << j.dump() << '\n';
            if (++lines_ % flush_every_ == 0) {
                ofs_.flush();
            }
        }

        void close() {
            std::lock_guard<std::mutex> lk(mutex_);
            if (ofs_.is_open()) {
                ofs_.flush();
                ofs_.close();
            }
        }

    private:
        mutable std::mutex mutex_;
        std::ofstream ofs_;
        std::string path_;
        bool enable_ = false;
        int flush_every_ = 20;
        int lines_ = 0;
    };

    inline RunRecorder& run_recorder() {
        static RunRecorder r;
        return r;
    }

    // NaN/Inf -> null（nlohmann 序列化 NaN 会抛异常，必须过滤）
    inline nlohmann::json num(double v) {
        if (!std::isfinite(v)) {
            return nlohmann::json(nullptr);
        }
        return nlohmann::json(v);
    }

    // 一包 IMU（云台四元数）的原始快照，用来检查"IMU 本身是否有问题"：
    //   wall_ns   = 收到这包的 host 时刻
    //   packet_ns = 写进 tf 的时间戳（= wall - serial_send_to_image_microseconds）
    //   dq_deg / rate_dps = 与上一包之间的姿态差 / 换算出的角速度
    //                       （用来抓跳变、卡包、尖峰，以及和真实角速度对不上）
    //   seq / bad = 累计解析成功 / 失败包数（看丢包）
    struct ImuSample {
        int64_t wall_ns = 0;
        int64_t packet_ns = 0;
        double w = 1.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double dq_deg = 0.0;
        double rate_dps = 0.0;
        uint64_t seq = 0;
        uint64_t bad = 0;
        int protocol = 1; // 1 = ascii, 0 = binary
    };

} // namespace awakening::record
