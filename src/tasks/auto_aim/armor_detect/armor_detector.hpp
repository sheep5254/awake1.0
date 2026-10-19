#pragma once
#include "tasks/auto_aim/type.hpp"
#include "utils/impl.hpp"
#include <optional>
#include <vector>
namespace awakening::auto_aim {
class ArmorDetector {
public:
    using Ptr = std::unique_ptr<ArmorDetector>;
    ArmorDetector(const YAML::Node& config);
    [[nodiscard]] std::tuple<std::vector<Light>, std::vector<Armor>> detect(
        const CommonFrame& frame,
        const cv::Rect& net_focus,
        const std::optional<cv::Rect>& detect_light = std::nullopt
    );
    double get_net_wh_ratio() const noexcept;

    // 诊断用：检测各阶段计数（读取后清零）
    struct DetectStats {
        int net_empty = 0;    // 网络无输出次数
        int over_conf = 0;    // 过 conf 阈值的 anchor 总数（NMS 前）
        int kept = 0;         // NMS 后保留的装甲板总数
        int extract_fail = 0; // 数字裁剪失败次数
        int out = 0;          // 最终输出的装甲板总数
    };
    [[nodiscard]] DetectStats take_detect_stats() const;
    AWAKENING_IMPL_DEFINITION(ArmorDetector)
};
} // namespace awakening::auto_aim