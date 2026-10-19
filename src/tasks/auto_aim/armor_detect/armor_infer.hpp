#pragma once
#include "tasks/auto_aim/type.hpp"
#include "utils/impl.hpp"
namespace awakening::auto_aim {
class ArmorInfer {
public:
    using Ptr = std::unique_ptr<ArmorInfer>;
    ArmorInfer(const YAML::Node& config);
    AWAKENING_IMPL_DEFINITION(ArmorInfer)
    static Ptr create(const YAML::Node& config) {
        return std::make_unique<ArmorInfer>(config);
    }

    [[nodiscard]] std::vector<Armor> process(const cv::Mat& output_buffer) const;

    // 诊断用：置信度过滤 / NMS 保留数量（读取后清零）
    struct Stats {
        int over_conf = 0; // 超过 conf_threshold 的 anchor 数（NMS 前）
        int kept = 0;      // NMS 之后保留的装甲板数
    };
    [[nodiscard]] Stats take_stats() const noexcept;

    int input_w() const noexcept;
    int input_h() const noexcept;
    bool use_norm() const noexcept;
    PixelFormat target_format() const noexcept;
};

} // namespace awakening::auto_aim