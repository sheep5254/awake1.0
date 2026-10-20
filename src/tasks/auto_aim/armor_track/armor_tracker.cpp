#include "armor_tracker.hpp"
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/auto_aim/type.hpp"
#include "tasks/base/dta_utils.hpp"
#include "utils/logger.hpp"
#include <algorithm>
#include <array>
#include <limits>
#include <utility>
#include <vector>
namespace awakening::auto_aim {
struct ArmorTracker::Impl {
    Impl(const YAML::Node& config) {
        cfg_.load(config);
    }
    ArmorTarget track(
        Armors& armors,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom,
        int frame_id
    ) {
        auto& cur = target_buf_[cur_target_idx_];
        auto& pre = target_buf_[pre_target_idx_];
        auto process = [&](int idx) {
            auto& t = target_buf_[idx];
            bool found = (t.track_state.tracker_state == ArmorTarget::TrackState::LOST)
                ? init_target(t, armors, frame_id, camera_info, camera_cv_in_odom)
                : update_target(t, armors, camera_info, camera_cv_in_odom);
            // 【重捕获】板子明明检到了，却连续 relock_time_thres 秒更新不上（匹配被门限
            // 全拒 / 内部预测已经偏了）：不要再空推到 lost_time_thres(1.5s) 才恢复，
            // 直接把这个检测当新目标重新初始化。否则这段时间预测框、车坐标系都钉在旧
            // 位置，就是"画面里明明识别到装甲板，可画出来的小车坐标系飞了"。
            if (!found && t.track_state.tracker_state != ArmorTarget::TrackState::LOST
                && dta_utils::elapsed_sec(t.last_update, armors.timestamp)
                    > cfg_.relock_time_thres)
            {
                t.track_state.tracker_state = ArmorTarget::TrackState::LOST;
                found = init_target(t, armors, frame_id, camera_info, camera_cv_in_odom);
            }
            update_fsm(found, idx, armors.timestamp);
            return found;
        };
        // 双缓冲，方便异常丢失恢复和操作手换目标。
        process(cur_target_idx_);

        if (cur.track_state.tracker_state == ArmorTarget::TrackState::TEMP_LOST) {
            process(pre_target_idx_);

            if (pre.track_state.tracker_state == ArmorTarget::TrackState::TRACKING) {
                std::swap(cur, pre);
                pre.track_state.tracker_state = ArmorTarget::TrackState::LOST;
            }
        } else if (cur.track_state.tracker_state == ArmorTarget::TrackState::TRACKING) {
            pre.track_state.tracker_state = ArmorTarget::TrackState::LOST;
        }
        armors.lights.erase(
            std::remove_if(
                armors.lights.begin(),
                armors.lights.end(),
                [](const Light& l) { return l.laji; }
            ),
            armors.lights.end()
        );

        return target_buf_[cur_target_idx_].fast_copy_without_ekf(); //下游不让用ekf
    }
    bool init_target(
        ArmorTarget& target,
        Armors& armors,
        int frame_id,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    ) noexcept {
        if (armors.armors.empty()) {
            return false;
        }
        const auto valid_target = [&](Armor& armor) {
            const bool valid_color =
                armor.color != ArmorColor::NONE && armor.color != ArmorColor::PURPLE;
            // 【修源码 bug】号码决定板尺寸（no1/outpost 走大板几何）：分类器明确判 UNKNOWN
            // 的板不能拿来 init，否则会按错误的板尺寸解 PnP，位置直接错到相机旁边。
            const bool number_ok = !armor.number_classifier
                || armor.number_classifier->number != ArmorClass::UNKNOWN;
            const bool keep_outpost =
                target_buf_[cur_target_idx_].target_number == ArmorClass::OUTPOST
                && armor.number != ArmorClass::OUTPOST && target_buf_[cur_target_idx_].check();
            const bool pnp_ok = ArmorTarget::armor_pnp(armor, camera_info, camera_cv_in_odom);
            // 【物理合理性】init 没有任何匹配门限，一次误检/退化 PnP（两灯条乱配、板只有
            // 几像素）就会把状态直接设到几十米外（实测 p=[-75, 40, -20]，|p|≈88 m），
            // 之后所有测量都在门限外、只能空推。这里只允许"相机前方 0.5~8 m"的板 init。
            bool geometry_ok = true;
            if (pnp_ok) {
                const Vec3 p_cam = camera_cv_in_odom.inverse() * armor.pose.translation();
                const double d = p_cam.norm();
                geometry_ok = (d > 0.5) && (d < 8.0) && (p_cam.z() > 0.3);
            }
            return valid_color && number_ok && !keep_outpost && pnp_ok && geometry_ok;
        };
        Armor* selected_armor = nullptr;
        auto min_dis = std::numeric_limits<double>::max();
        for (auto& armor: armors.armors) {
            if (valid_target(armor)) {
                auto bbox = armor.key_points.bounding_box();
                auto dis = utils::calculate_distance_to_img_center(
                    (bbox.tl() + bbox.br()) / 2.0,
                    camera_info.camera_matrix
                );
                if (dis < min_dis) {
                    min_dis = dis;
                    selected_armor = &armor;
                }
            }
        }
        if (!selected_armor) {
            return false;
        }

        Armor init_target = *selected_armor;
        AWAKENING_INFO("init target: {}", string_by_armor_class(init_target.number));
        target.reset(init_target, cfg_, armors.timestamp, frame_id, camera_info, camera_cv_in_odom);
        target.track_state.tracker_state = ArmorTarget::TrackState::DETECTING;
        return true;
    }
    bool update_target(
        ArmorTarget& target,
        Armors& armors,
        const CameraInfo& camera_info,
        const ISO3& camera_cv_in_odom
    ) noexcept {
        std::vector<Armor> candidates;
        candidates.reserve(armors.armors.size());
        for (const auto& a: armors.armors) {
            if (a.color == ArmorColor::NONE || a.color == ArmorColor::PURPLE) {
                continue;
            }
            if (a.number == target.target_number) {
                candidates.emplace_back(a);
            } else if (a.number == ArmorClass::UNKNOWN) {
                // 【别因为号码没判出来就丢观测】暗光/斜看时分类器频繁给 UNKNOWN（实测占
                // 被拒观测的 97%），而更新路径原来要求 number 严格相等 → 这些帧全部丢掉
                // → last_update 不变 → 输出状态被冻结，表现为"板子跑走了、识别坐标系还在
                // 原地"。号码只用来定装甲板尺寸，这里按**已锁定的目标号码**解释这块板，
                // 几何/运动照常参与更新；target_number 本身不会被改写。
                Armor a_known = a;
                a_known.number = target.target_number;
                candidates.emplace_back(std::move(a_known));
            }
            // if (a.number == target.target_number)
            // {
            //     candidates.emplace_back(a);
            // }
        }
        target.predict_ekf(armors.timestamp);
        auto matched_armors =
            target.match_armor(candidates, armors.timestamp, camera_info, camera_cv_in_odom);
        auto matched_lights = target.match_light(
            armors.lights,
            matched_armors,
            armors.timestamp,
            camera_info,
            camera_cv_in_odom
        );
        const int updated = target.update(
            matched_armors,
            matched_lights,
            armors.timestamp,
            camera_info,
            camera_cv_in_odom
        );
        return updated > 0;
    }
    void update_fsm(bool found, size_t i, const TimePoint& now) noexcept {
        auto& target = target_buf_[i];
        auto& s = target.track_state;
        if (found)
            ++found_count_;
        dta_utils::update_fsm(
            found,
            s,
            cfg_.tracking_thres,
            dta_utils::elapsed_sec(target.last_update, now),
            lost_time_thres(target),
            cfg_.detecting_miss_tol
        );
    }
    double lost_time_thres(const ArmorTarget& target) const noexcept {
        return (target.target_number == ArmorClass::OUTPOST) ? cfg_.lost_time_thres_outpost
                                                             : cfg_.lost_time_thres;
    }
    void set_sentry(bool is_sentry) {
        iam_sentry = is_sentry;
    }

    int found_count_ = 0;

    size_t cur_target_idx_ = 0;
    size_t pre_target_idx_ = 1;
    std::array<ArmorTarget, 2> target_buf_;
    ArmorTrackerCfg cfg_;
    bool iam_sentry = false;
};
ArmorTracker::ArmorTracker(const YAML::Node& config): _impl(std::make_unique<Impl>(config)) {}
ArmorTracker::~ArmorTracker() noexcept = default;

ArmorTarget ArmorTracker::track(
    Armors& armors,
    const CameraInfo& camera_info,
    const ISO3& camera_cv_in_odom,
    int frame_id
) {
    return _impl->track(armors, camera_info, camera_cv_in_odom, frame_id);
}
int ArmorTracker::get_count() {
    return _impl->found_count_;
}
void ArmorTracker::reset_count() {
    _impl->found_count_ = 0;
}
void ArmorTracker::set_sentry(bool is_sentry) {
    _impl->set_sentry(is_sentry);
}
} // namespace awakening::auto_aim
