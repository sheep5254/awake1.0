#pragma once
// ASCII 文本帧协议（serial.send_protocol / serial.recv_protocol = 1 时启用）
//
//   视觉 -> 下位机:  AAP<pitch>Y<yaw>F<fire>N<frame>C<4位hex>%<LF>
//                    例: AAP1.23Y-4.56F0N120Cc328%\n
//                    pitch/yaw 单位度，fire 0/1，frame 为自增帧号
//   下位机 -> 视觉:  ARQ<w>X<x>Y<y>Z<z>C<4位hex>%<LF>   （末尾 '%' 可省）
//                    例: ARQ0.7071X0.0000Y0.7071Z0.0000C2dd9%\n
//                    四元数 wxyz，单位化
//
// CRC 覆盖 'C' 之前的整段 ASCII 字节。
// 注意：**固件实际用的是 CRC16-MCRF4XX**（初值 0xFFFF、反射多项式 0x8408，
// 也就是 0x1021 的反射）；早期文档和我们最初的实现写的是 Modbus
// （初值 0xFFFF、反射多项式 0xA001），两者互不兼容。因此：
//   * 发送：默认 MCRF4XX（与固件一致），可用 serial.crc_mode 切回 modbus；
//   * 接收：两种都接受（先试 MCRF4XX 再试 Modbus），避免文档/固件不一致时全丢帧。
//
// 另外：下位机经常把别的协议的二进制数据混在同一个串口上发（实测帧间夹着
// 约 70 字节的 0x5A 开头二进制包），而串口驱动交给任务的是**任意长度的原始
// 字节块**，所以取帧必须用 RecvFrameScanner：它会在字节流里搜 "ARQ" 起点、
// 按 '%'/'\n' 切帧、跨读块拼接，并丢弃中间的垃圾字节。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace awakening::ascii_serial {

enum class CrcMode : int {
    Mcrf4xx = 0,  // 固件实际使用（0x8408）
    Modbus = 1,   // 早期文档/实现（0xA001）
};

inline uint16_t crc16_mcrf4xx(const uint8_t* data, size_t len) noexcept {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x0001) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408)
                                 : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

inline uint16_t crc16_modbus(const uint8_t* data, size_t len) noexcept {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x0001) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001)
                                 : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

inline uint16_t crc16_of(CrcMode mode, const uint8_t* data, size_t len) noexcept {
    return mode == CrcMode::Modbus ? crc16_modbus(data, len) : crc16_mcrf4xx(data, len);
}

// 视觉 -> 下位机
inline std::string build_send_frame(
    double pitch_deg,
    double yaw_deg,
    int fire,
    int frame_count,
    CrcMode crc_mode = CrcMode::Mcrf4xx
) {
    char body[80];
    const int n = std::snprintf(
        body, sizeof(body), "AAP%.2fY%.2fF%dN%d", pitch_deg, yaw_deg, fire, frame_count
    );
    if (n <= 0) return {};

    char tail[16];
    const uint16_t crc =
        crc16_of(crc_mode, reinterpret_cast<const uint8_t*>(body), static_cast<size_t>(n));
    const int m = std::snprintf(tail, sizeof(tail), "C%04x%%\n", crc);
    if (m <= 0) return {};

    std::string frame(body, static_cast<size_t>(n));
    frame.append(tail, static_cast<size_t>(m));
    return frame;
}

// 下位机 -> 视觉：解析单帧四元数。CRC 不符（两种算法都不符）或格式不对返回 nullopt
inline std::optional<Eigen::Quaterniond> parse_recv_frame(const std::vector<uint8_t>& raw) {
    size_t n = raw.size();
    while (n > 0 && (raw[n - 1] == '\n' || raw[n - 1] == '\r' || raw[n - 1] == ' ')) --n;
    if (n > 0 && raw[n - 1] == '%') --n;  // 末尾 '%' 可省
    if (n < 8) return std::nullopt;

    size_t cpos = std::string::npos;  // 从后往前找 CRC 的 'C'
    for (size_t i = n; i-- > 0;) {
        if (raw[i] == 'C') {
            cpos = i;
            break;
        }
    }
    if (cpos == std::string::npos || n - cpos < 5) return std::nullopt;

    char hex[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    const size_t hl = std::min<size_t>(4, n - cpos - 1);
    for (size_t i = 0; i < hl; ++i) hex[i] = static_cast<char>(raw[cpos + 1 + i]);
    const uint16_t got = static_cast<uint16_t>(std::strtoul(hex, nullptr, 16));
    // 固件用 MCRF4XX，旧文档用 Modbus —— 两种都认，谁的都行
    if (got != crc16_mcrf4xx(raw.data(), cpos) && got != crc16_modbus(raw.data(), cpos)) {
        return std::nullopt;
    }

    const std::string line(reinterpret_cast<const char*>(raw.data()), cpos);
    double w = 0, x = 0, y = 0, z = 0;
    if (std::sscanf(line.c_str(), "ARQ%lfX%lfY%lfZ%lf", &w, &x, &y, &z) != 4) return std::nullopt;

    Eigen::Quaterniond q(w, x, y, z);
    const double norm = q.norm();
    if (!std::isfinite(norm) || norm < 1e-6) return std::nullopt;
    q.normalize();
    return q;
}

// 从任意字节流中提取 ARQ 帧：搜帧头、按 '%'/'\n' 切帧、跨读块拼接、丢弃垃圾。
class RecvFrameScanner {
public:
    static constexpr size_t kMaxFrame = 256;

    // 喂入新到的字节；每凑齐一帧就回调一次（frame 不含尾部 '%'/'\n'）
    template<typename Fn>
    void feed(const uint8_t* data, size_t len, Fn&& on_frame) {
        if (len > 0) buf_.insert(buf_.end(), data, data + len);

        while (true) {
            // 1) 找 "ARQ" 起点，丢掉前面的垃圾（但保留末尾 2 字节，可能是半截帧头）
            auto head = std::search(buf_.begin(), buf_.end(), kHead, kHead + 3);
            if (head == buf_.end()) {
                const size_t keep = std::min<size_t>(buf_.size(), 2);
                buf_.erase(buf_.begin(), buf_.end() - static_cast<std::ptrdiff_t>(keep));
                return;
            }
            if (head != buf_.begin()) buf_.erase(buf_.begin(), head);

            // 2) 找帧尾
            auto tail = std::find_if(buf_.begin(), buf_.end(), [](uint8_t c) {
                return c == '%' || c == '\n';
            });
            if (tail == buf_.end()) {
                if (buf_.size() > kMaxFrame) buf_.clear();  // 畸形数据，别让它无限长
                return;
            }

            std::vector<uint8_t> frame(buf_.begin(), tail);
            buf_.erase(buf_.begin(), tail + 1);
            on_frame(std::move(frame));

            if (buf_.size() < 3) return;
        }
    }

    void clear() { buf_.clear(); }
    size_t pending() const { return buf_.size(); }

private:
    static constexpr uint8_t kHead[3] = {'A', 'R', 'Q'};
    std::vector<uint8_t> buf_;
};

}  // namespace awakening::ascii_serial
