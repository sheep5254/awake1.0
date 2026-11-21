#pragma once
#include "utils/common/image.hpp"
#include "utils/common/type_common.hpp"
#include "utils/utils.hpp"
namespace awakening {
static const Mat3 R_CV2PHYSICS =
    (Mat3() << 0.0, 0.0, 1.0, -1.0, -0.0, 0.0, 0.0, -1.0, 0.0).finished();
struct CommonFrame {
    ImageFrame img_frame;
    int id;
    int frame_id;
    // cv::Rect2f expanded;
};
enum class EnemyColor : bool {
    RED = 0,
    BLUE = 1,
};
inline EnemyColor enemy_color_from_string(const std::string& str) {
    auto key = utils::to_upper(str);
    if (key == "RED")
        return EnemyColor::RED;
    else if (key == "BLUE") {
        return EnemyColor::BLUE;
    } else {
        throw std::runtime_error("Invalid enemy color: " + key);
    }
}
struct CameraInfo {
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    void load(const YAML::Node& config) {
        std::vector<double> camera_k = config["camera_matrix"]["data"].as<std::vector<double>>();
        std::vector<double> camera_d =
            config["distortion_coefficients"]["data"].as<std::vector<double>>();

        assert(camera_k.size() == 9);
        assert(camera_d.size() == 5);

        cv::Mat K(3, 3, CV_64F);
        std::memcpy(K.data, camera_k.data(), 9 * sizeof(double));

        cv::Mat D(1, 5, CV_64F);
        std::memcpy(D.data, camera_d.data(), 5 * sizeof(double));

        camera_matrix = K.clone();
        distortion_coefficients = D.clone();
    }
    double camera_fx() const noexcept {
        return camera_matrix.at<double>(0, 0);
    }

    double camera_fy() const noexcept {
        return camera_matrix.at<double>(1, 1);
    }

    double camera_cx() const noexcept {
        return camera_matrix.at<double>(0, 2);
    }

    double camera_cy() const noexcept {
        return camera_matrix.at<double>(1, 2);
    }

    CameraInfo clone() const {
        return { .camera_matrix = this->camera_matrix.clone(),
                 .distortion_coefficients = this->distortion_coefficients.clone() };
    }
};
struct AimPoint {
    ISO3 pose;
    int frame_id;
    static AimPoint lerp(const AimPoint& a, const AimPoint& b, double t) {
        AimPoint p;

        Vec3 trans = (1.0 - t) * a.pose.translation() + t * b.pose.translation();
        Quaternion qa(a.pose.rotation());
        Quaternion qb(b.pose.rotation());
        Quaternion q = qa.slerp(t, qb);

        p.pose = ISO3::Identity();
        p.pose.linear() = q.toRotationMatrix();
        p.pose.translation() = trans;
        return p;
    }
    void transform(const ISO3& old_in_new, int new_frame_id) {
        pose = old_in_new * pose;
        frame_id = new_frame_id;
    }
};
struct GimbalCmd {
    TimePoint timestamp;
    double pitch = 0;
    double yaw = 0;
    double enable_yaw_diff = 0;
    double enable_pitch_diff = 0;
    double target_yaw = 0;
    double target_pitch = 0;
    double v_yaw = 0;
    double v_pitch = 0;
    double a_yaw = 0;
    double a_pitch = 0;
    bool fire_advice = false;
    double fly_time = 0;
    bool appear = false;
    AimPoint aim_point;
    int select_id = 0;
    // 【诊断】当前时刻控制点的弹道输入/输出（由 get_control_point 在 cp0 那次调用时填）
    double dbg_pz = 0;
    double dbg_d = 0;
    double dbg_alpha_deg = 0;
    int dbg_valid = 0;      // 1 = 本拍重建了轨迹（诊断量是本拍算的）；0 = 复用，量来自上次重建
    double dbg_age_ms = 0;  // 复用轨迹时诊断量的年龄（ms）
    int dbg_aim_id = -1;    // 选中的装甲板编号
    double obs_pixel_elev = 0;  // 观测 LOS：检测框中心 + 内参 + 曝光时刻相机姿态 → 世界系仰角
    double obs_pnp_elev = 0;    // 观测 LOS：PnP 三维点相对相机中心的世界系仰角
    double img_age_ms = 0;      // 图像时刻 → 当前解算时刻 的间隔
    // 【诊断】参考板与 4 块板高度：定位"换板导致 LOS 跳变"发生在哪一步
    int dbg_ref_id = -1;        // aim_z_from_observed 选中的参考板索引
    double dbg_ref_z = 0;       // 该参考板的 z（就是覆盖到瞄准点上的值）
    double dbg_z[4] = {0, 0, 0, 0};  // 当前状态下 4 块模型板的 z
    double dbg_h = 0;           // 状态里的 H（奇数板高度差）
    double dbg_car_roll = 0;    // 车体 roll（度，来自 C_ROT_X）
    double dbg_car_pitch = 0;   // 车体 pitch（度，来自 C_ROT_Y）
    inline bool is_valid() const noexcept {
        auto bad = [](double x) { return std::isnan(x) || std::isinf(x); };

        // 这些字段会直接进"增量下发"和画面叠加，任一 NaN/Inf 都不能放行。
        // （原来漏了 a_yaw / a_pitch / fly_time，且 target_pitch 判了两次。）
        if (bad(pitch) || bad(yaw) || bad(target_yaw) || bad(target_pitch) || bad(v_yaw)
            || bad(v_pitch) || bad(a_yaw) || bad(a_pitch) || bad(fly_time)
            || bad(enable_yaw_diff) || bad(enable_pitch_diff))
            return false;

        return true;
    }
    inline void no_shoot() {
        fire_advice = false;
        enable_yaw_diff = 0;
        enable_pitch_diff = 0;
    }
};
} // namespace awakening