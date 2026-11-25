#include "very_aimer.hpp"
#include "angles.h"
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/base/ballistic_trajectory.hpp"
#include "tasks/base/dta_utils.hpp"
#include "utils/common/type_common.hpp"
#include "utils/logger.hpp"
#include "utils/utils.hpp"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
namespace awakening::auto_aim {
struct VeryAimer::Impl {
    struct Params {
        double sample_total_time;
        int sample_horizon;
        double fire_delay_min;
        double fire_delay_max;
        double max_yaw_acc;
        double max_pitch_acc;
        double prediction_delay;
        double aim_center_more_prediction_time;
        double shooting_range_h;
        double shooting_range_w_small;
        double shooting_range_w_large;
        double min_enable_pitch_deg;
        double min_enable_yaw_deg;
        double fire_actual_tol_deg = 1.5;
        // 【弹道高度常数】只给弹道解算用的"板中心相对枪口的高度差"(m)。
        // 不影响画图/匹配/瞄准点记录（那些仍用原始 p）。
        bool ballistic_h_const_enable = false;
        double ballistic_h_const = 0.0;
        bool ballistic_h_const_log = false;
        // 【俯仰符号】solve_pitch 返回物理仰角(正=上)，但 rpy2matrix 用的是标准 Ry(pitch)，
        // 正 pitch 让 +X 朝下 → 直接把 α 填进去会让枪管压低 α。这里取负修正。
        // 2026-11-25 实车最终结论：必须取负(-1)，并且【同时】把发布出去的 pitch 也取负
        // （nav_pitch_sign: -1），两个成对才正确；只改一个会落点偏低/云台反向跑。
        double ballistic_pitch_sign = -1.0;
        // 【pitch 用观测射线】true = 弹道解算的"高度"取『检测像素方向』而不是模型的板高。
        // 模型里板高/tilt 本质不可观（实测静目标板高乱跳 ±10cm、四块板极差 0.22m），
        // 用它算 pitch 会抖+下垂；像素方向实测稳定 ±0.2°。
        // 注意：只替换"高度"分量，水平方向仍用模型（含超前量），所以不影响打提前量。
        bool use_observed_pitch = false;
        // 瞄准点高度改用"当前可见那块板"的 z（而不是超前预测那块板的 z）
        bool aim_z_from_observed = false;

        void load(const YAML::Node& config) {
            sample_total_time = config["sample_total_time"].as<double>();
            sample_horizon = config["sample_horizon"].as<int>();
            fire_delay_min = config["fire_delay_min"].as<double>();
            fire_delay_max = config["fire_delay_max"].as<double>();
            max_yaw_acc = config["max_yaw_acc"].as<double>();
            max_pitch_acc = config["max_pitch_acc"].as<double>();
            prediction_delay = config["prediction_delay"].as<double>();
            aim_center_more_prediction_time =
                config["aim_center_more_prediction_time"].as<double>();
            shooting_range_h = config["shooting_range_h"].as<double>();
            shooting_range_w_small = config["shooting_range_w_small"].as<double>();
            shooting_range_w_large = config["shooting_range_w_large"].as<double>();
            min_enable_pitch_deg = config["min_enable_pitch_deg"].as<double>();
            min_enable_yaw_deg = config["min_enable_yaw_deg"].as<double>();
            fire_actual_tol_deg = config["fire_actual_tol_deg"]
                ? config["fire_actual_tol_deg"].as<double>()
                : 1.5;
            aim_z_from_observed = config["aim_z_from_observed"]
                ? config["aim_z_from_observed"].as<bool>()
                : false;
            ballistic_h_const_enable = config["ballistic_h_const_enable"]
                ? config["ballistic_h_const_enable"].as<bool>()
                : false;
            ballistic_h_const = config["ballistic_h_const"]
                ? config["ballistic_h_const"].as<double>()
                : 0.0;
            ballistic_h_const_log = config["ballistic_h_const_log"]
                ? config["ballistic_h_const_log"].as<bool>()
                : false;
            ballistic_pitch_sign = config["ballistic_pitch_sign"]
                ? config["ballistic_pitch_sign"].as<double>()
                : -1.0;
            use_observed_pitch = config["use_observed_pitch"]
                ? config["use_observed_pitch"].as<bool>()
                : false;
        }
    } params_;
    using MPCType = dta_utils::TinyMpcTrajectory;
    Impl(const YAML::Node& config) {
        params_.load(config);
        ballistic_trajectory_ = BallisticTrajectory::create(config["ballistic_trajectory"]);
        base_yaw_offset_rad_ = angles::from_degrees(config["base_yaw_offset"].as<double>());
        base_pitch_offset_rad_ = angles::from_degrees(config["base_pitch_offset"].as<double>());
        auto type = config["type"].as<std::string>();
        if (type == "mpc" || type == "MPC") {
            MPCType::Params mpc_p {
                .yaw_max_acc = (float)params_.max_yaw_acc,
                .pitch_max_acc = (float)params_.max_pitch_acc,
                .horizon = params_.sample_horizon,
                .dt = params_.sample_total_time / params_.sample_horizon,
                .max_iter = 10,
            };
            mpc_traj_ = MPCType::create(mpc_p);
            AWAKENING_INFO("very_aimer: using  MpcTrajectory");
        }
    }
    [[nodiscard]] int
    select_armor(const ArmorTarget& target, const AutoAimFsm& auto_aim_fsm) const noexcept {
        static int lock_id = -1;
        const auto& target_state = target.get_target_state();
        const auto armors_pose = target_state.get_armors_pose(target.target_number);
        const int armor_num = static_cast<int>(armors_pose.size());
        int i_chosen = 0;
        const double center_yaw = std::atan2(target_state.pos().y(), target_state.pos().x());
        auto delta_angle = [&](int i) {
            auto rpy = utils::matrix2rpy<double>(armors_pose[i].linear());
            // return angles::normalize_angle(
            //     rpy[2]
            //     - std::atan2(armors_pose[i].translation().y(), armors_pose[i].translation().x())
            // );
            return angles::normalize_angle(rpy[2] - center_yaw);
        };
        auto abs_delta_angle = [&](int i) { return std::abs(delta_angle(i)); };
        auto pick_best_range = [&](int first, int last, auto&& accept) -> int {
            int best = -1;
            double best_val = std::numeric_limits<double>::infinity();
            for (int i = first; i < last; ++i) {
                if (!accept(i))
                    continue;
                const double val = abs_delta_angle(i);
                if (val < best_val) {
                    best_val = val;
                    best = i;
                }
            }
            return best;
        };
        auto accept_all = [](int) { return true; };
        auto accept_pair = [&](int i) { return (i == 0 || i == 2); };

        if (auto_aim_fsm == AutoAimFsm::AIM_SINGLE_ARMOR
            && target.target_number != ArmorClass::OUTPOST && armor_num > 0)
        {
            int candidates[2] = { -1, -1 };
            int candidate_count = 0;
            for (int i = 0; i < armor_num && candidate_count < 2; ++i) {
                if (std::abs(delta_angle(i)) <= 60.0 / 57.3) {
                    candidates[candidate_count++] = i;
                }
            }

            if (candidate_count > 0) {
                int pick = -1;

                if (candidate_count == 1) {
                    pick = candidates[0];
                    lock_id = -1;
                } else {
                    if (lock_id < 0 || (lock_id != candidates[0] && lock_id != candidates[1])) {
                        lock_id = (abs_delta_angle(candidates[0]) < abs_delta_angle(candidates[1]))
                            ? candidates[0]
                            : candidates[1];
                    }
                    pick = lock_id;
                }

                if (pick >= 0 && pick < armor_num) {
                    i_chosen = pick;
                }
            }

            return i_chosen;
        }
        if (armor_num > 0) {
            int best_idx = -1;

            if (auto_aim_fsm
                    == AutoAimFsm::AIM_WHOLE_CAR_PAIR //4选2,本质提升控制轨迹与目标轨迹重合窗口
                && target.target_number != ArmorClass::OUTPOST)
            {
                best_idx = pick_best_range(0, armor_num, accept_pair);
            }
            if (best_idx < 0) {
                best_idx = pick_best_range(0, armor_num, accept_all);
            }

            i_chosen = best_idx;
        }

        return i_chosen;
    }
    [[nodiscard]] dta_utils::ControlPoint get_control_point(
        const ISO3& armor_pose,
        const ISO3& shoot_in_gimbal_odom,
        const ISO3& gimbal_in_gimbal_odom,
        double bullet_speed,
        int aim_id
    ) const noexcept {
        dta_utils::ControlPoint cp;
        auto p = armor_pose.translation() - shoot_in_gimbal_odom.translation();
        // 【弹道高度常数】瞄准点的 z 有两个用途：
        //   ① 画图/匹配/瞄准点记录 —— 必须和图像观测一致（所以绝不能钉状态里的 CZ）；
        //   ② 弹道解算 —— 它只要"板中心相对枪口高多少"，这个物理上是常数。
        // 这里只替换 ②：原始 p 继续用于 yaw / cp.aim_point / 其它几何用途，
        // 另建 ballistic_p 只改 z 后交给 solve_pitch。这样预测框不会飞、
        // 弹道仰角也不再随 CZ 漂；剩余固定偏差由 base_pitch_offset 一次标定吸收。
        Vec3 ballistic_p = p;
        if (params_.ballistic_h_const_enable) {
            ballistic_p.z() = params_.ballistic_h_const;
        }
        // 【pitch 用观测射线】把"板中心相对枪口的高度"换成观测值：
        //   tan(观测仰角) = (板高 - 相机高) / 水平距离
        //   => 板高 - 枪口高 = (相机高 - 枪口高) + 水平距离 * tan(观测仰角)
        // 水平方向（含超前量）仍用模型，所以打提前量不受影响。
        if (params_.use_observed_pitch && obs_valid_) {
            const double d_h = std::hypot(p.x(), p.y());
            const double dz_cam_muzzle = obs_cam_z_ - shoot_in_gimbal_odom.translation().z();
            ballistic_p.z() = dz_cam_muzzle + d_h * std::tan(obs_elev_cam_);
        }
        if (params_.ballistic_h_const_log) {
            static TimePoint last_log_tp {};
            const auto now_log = Clock::now();
            if (last_log_tp.time_since_epoch().count() == 0
                || now_log - last_log_tp >= std::chrono::seconds(1))
            {
                last_log_tp = now_log;
                AWAKENING_INFO(
                    "ballistic p.z()={:+.3f} m | p=[{:+.2f},{:+.2f},{:+.2f}] 恒定值={:+.3f} 启用={}",
                    p.z(), p.x(), p.y(), p.z(), params_.ballistic_h_const,
                    params_.ballistic_h_const_enable ? 1 : 0);
            }
        }
        auto desired_pitch_opt = ballistic_trajectory_->solve_pitch(ballistic_p, bullet_speed);
        if (desired_pitch_opt) {
            dbg_pz_ = p.z();
            dbg_d_ = std::hypot(p.x(), p.y());
            dbg_alpha_deg_ = angles::to_degrees(*desired_pitch_opt);
        }
        if (!desired_pitch_opt) {
            cp.valid = false;
            // 限流（最多 1 条/秒）：状态被带偏的那几帧会连续触发，原来每拍一条把日志刷爆。
            // 同时打印距离/高度，方便区分是"状态错"（|p| 不对）还是"真超范围"。
            static TimePoint last_warn_tp {};
            const auto now_wp = Clock::now();
            if (last_warn_tp.time_since_epoch().count() == 0
                || now_wp - last_warn_tp >= std::chrono::seconds(1))
            {
                last_warn_tp = now_wp;
                AWAKENING_WARN(
                    "very_aimer: solve_pitch failed | p=[{:.2f},{:.2f},{:.2f}] |p|={:.2f} "
                    "h={:.2f} d={:.2f} v0={:.1f}",
                    p.x(),
                    p.y(),
                    p.z(),
                    p.norm(),
                    p.z(),
                    std::hypot(p.x(), p.y()),
                    bullet_speed
                );
            }
            return cp;
        }
        const auto [yaw_offset, pitch_offset] = get_yaw_pitch_offset();
        const double desired_control_yaw = std::atan2(p.y(), p.x());
        // 【俯仰符号修正】离线自检验证：solve_pitch 返回物理仰角 α(正=上)，
        // 而 rpy2matrix 的标准 Ry(pitch) 使 +X 朝下 → 直接填 α 会让枪管压低 α。
        // 实测（固定靶 1.2m）: α=+0.764° → 前向轴实际 -0.764°；改用 -α 后为 +0.764° ✓
        // 这也解释了过去"自旋时 pitch 慢慢往下、落点偏低 3~5cm"（α 变大 → 被压得更低）。
        const double pitch_for_matrix =
            params_.ballistic_pitch_sign * (desired_pitch_opt.value() + pitch_offset);
        auto desired_shoot = utils::rpy2matrix(Vec3(
            0.0,
            pitch_for_matrix,
            angles::normalize_angle(desired_control_yaw + yaw_offset)
        ));
        const Mat3 R_gimbal_shoot =
            gimbal_in_gimbal_odom.linear().transpose() * shoot_in_gimbal_odom.linear();
        auto desired_gimbal = desired_shoot * R_gimbal_shoot.transpose();
        auto rpy = utils::matrix2rpy<double>(desired_gimbal);
        cp.yaw = rpy[2];
        cp.pitch = rpy[1];
        cp.valid = true;

        cp.aim_point.pose = armor_pose;
        cp.aim_id = aim_id;
        return cp;
    };
    [[nodiscard]] dta_utils::ControlPoint select_and_get_control_point(
        const ArmorTarget& target,
        double bullet_speed,
        const AutoAimFsm& fsm,
        const ISO3& shoot_in_gimbal_odom,
        const ISO3& gimbal_in_gimbal_odom
    ) const noexcept {
        const auto& target_state = target.get_target_state();
        const int selected_armor = select_armor(target, fsm);
        auto armors_pose = target_state.get_armors_pose(target.target_number);
        if (params_.aim_z_from_observed && obs_plate_z_ && selected_armor >= 0
            && selected_armor < static_cast<int>(armors_pose.size())) {
            armors_pose[selected_armor].translation().z() = *obs_plate_z_;
        }
        if (fsm == AutoAimFsm::AIM_WHOLE_CAR_CENTER) { //瞄准中间把目标推回原来板子的位置
            auto p = target_state.pos() - shoot_in_gimbal_odom.translation();
            double center_xy_dis = std::hypot(p.x(), p.y());
            double center_yaw = std::atan2(p.y(), p.x());
            center_xy_dis -= target_state.get_armor_r(selected_armor, target.target_number);
            armors_pose[selected_armor].translation().x() = center_xy_dis * std::cos(center_yaw);
            armors_pose[selected_armor].translation().y() = center_xy_dis * std::sin(center_yaw);
        }
        return get_control_point(
            armors_pose[selected_armor],
            shoot_in_gimbal_odom,
            gimbal_in_gimbal_odom,
            bullet_speed,
            selected_armor
        );
    }
    struct HitCtx {
        ArmorTarget hit_time_target;
        double fly_time;
    };
    std::optional<HitCtx> get_hit(
        const ArmorTarget& target_ready_to_aim,
        double bullet_speed,
        const AutoAimFsm& fsm,
        const ISO3& shoot_in_gimbal_odom
    ) const noexcept {
        auto hit_time_target = target_ready_to_aim.fast_copy_without_ekf();
        const int roughly_select = select_armor(hit_time_target, fsm);
        const auto __armors_pose =
            hit_time_target.get_target_state().get_armors_pose(hit_time_target.target_number);
        auto prev_pitch_and_fly_time_opt = ballistic_trajectory_->solve_pitch_and_flytime(
            __armors_pose[roughly_select].translation() - shoot_in_gimbal_odom.translation(),
            bullet_speed
        );
        if (!prev_pitch_and_fly_time_opt) {
            return std::nullopt;
        }
        auto prev_fly_time = prev_pitch_and_fly_time_opt.value().second;

        for (int iter = 0; iter < 10; ++iter) {
            auto i_target = hit_time_target.fast_copy_without_ekf();
            i_target.set_target_state([&](armor_point_motion_model::State& state) {
                state.predict(prev_fly_time, i_target.target_number);
            });
            // auto iter_select = select_armor(i_target, fsm);//不知道哪个最好
            const auto iter_armors_pose =
                i_target.get_target_state().get_armors_pose(i_target.target_number);
            // auto iter_pitch_and_fly_time_opt = ballistic_trajectory_->solve_pitch_and_flytime(
            //     iter_armors_xyza[iter_select].head<3>(),
            //     bullet_speed
            // );
            auto iter_pitch_and_fly_time_opt = ballistic_trajectory_->solve_pitch_and_flytime(
                iter_armors_pose[roughly_select].translation() - shoot_in_gimbal_odom.translation(),
                bullet_speed
            );
            if (!iter_pitch_and_fly_time_opt) {
                return std::nullopt;
            }
            if (std::abs(iter_pitch_and_fly_time_opt.value().second - prev_fly_time) < 1e-3) {
                prev_fly_time = iter_pitch_and_fly_time_opt.value().second;
                break;
            }
            prev_fly_time = iter_pitch_and_fly_time_opt.value().second;
        }
        const double predict_time = prev_fly_time + params_.prediction_delay
            + (fsm == AutoAimFsm::AIM_WHOLE_CAR_CENTER ? params_.aim_center_more_prediction_time : 0
            ); //瞄准中间只能预测一段发弹延迟
        hit_time_target.set_target_state([&](auto& state) {
            state.predict(predict_time, hit_time_target.target_number);
        });
        return HitCtx {
            .hit_time_target = hit_time_target,
            .fly_time = prev_fly_time,
        };
    }
    struct InputCtx {
        ArmorTarget target;
        double bullet_speed;
        AutoAimFsm fsm;
        ISO3 shoot_in_gimbal_odom;
        ISO3 gimbal_in_gimbal_odom;
        bool valid = false;
        bool is_same(
            const ArmorTarget& _target,
            double _bullet_speed,
            AutoAimFsm _fsm,
            const ISO3& _shoot_in_gimbal_odom,
            const ISO3& _gimbal_in_gimbal_odom
        ) {
            if (!valid)
                return false;
            bool same_target = _target.this_id == target.this_id;
            return same_target && _fsm == fsm && std::abs(_bullet_speed - bullet_speed) < 1e-2
                && _shoot_in_gimbal_odom.isApprox(shoot_in_gimbal_odom, 1e-2)
                && _gimbal_in_gimbal_odom.isApprox(gimbal_in_gimbal_odom, 1e-2);
        }
        void
        set(const ArmorTarget& _target,
            double _bullet_speed,
            AutoAimFsm _fsm,
            const ISO3& _shoot_in_gimbal_odom,
            const ISO3& _gimbal_in_gimbal_odom) {
            target = _target.fast_copy_without_ekf();
            bullet_speed = _bullet_speed;
            fsm = _fsm;
            shoot_in_gimbal_odom = _shoot_in_gimbal_odom;
            gimbal_in_gimbal_odom = _gimbal_in_gimbal_odom;
            valid = true;
        }

    } last_input_;
    TimePoint last_time_;
    dta_utils::LimitTrajectory limit_traj_;
    dta_utils::ControlPoint limit_traj_cp0_;
    Trajectory<AimPoint, double> aim_traj_;
    Trajectory<AimPoint, double> aim_center_aim_traj_;
    Trajectory<dta_utils::GimbalState, double> aim_center_target_traj_;
    dta_utils::ControlPoint aim_center_target_traj_cp0_;
    double last_fly_time_;
    int last_select_;
    [[nodiscard]] GimbalCmd very_aim(
        const ArmorTarget& _target,
        double bullet_speed,
        AutoAimFsm fsm,
        const ISO3& shoot_in_gimbal_odom,
        const ISO3& gimbal_in_gimbal_odom
    ) noexcept {
        if (_target.target_number == ArmorClass::BASE) {
            fsm = AutoAimFsm::AIM_SINGLE_ARMOR;
        }
        GimbalCmd cmd;
        cmd.appear = false;
        bool is_same =
            last_input_
                .is_same(_target, bullet_speed, fsm, shoot_in_gimbal_odom, gimbal_in_gimbal_odom);
        double time_in_traj = 0.0;
        const auto now = Clock::now();
        if (is_same) { //状态完全未更新，使用第一次构建的轨迹基于时间进行推进减少重复计算
            time_in_traj = std::chrono::duration<double>(now - last_time_).count();
        } else {
            last_input_
                .set(_target, bullet_speed, fsm, shoot_in_gimbal_odom, gimbal_in_gimbal_odom);
            last_time_ = now;
        }
        auto target = _target.fast_copy_without_ekf();
        // 【观测仰角】从 tracker 带过来的本帧观测（像素方向）
        obs_elev_cam_ = target.obs_elev_cam;
        obs_cam_z_ = target.obs_cam_z;
        obs_valid_ = target.obs_valid;
        target.set_target_state([&](armor_point_motion_model::State& state) {
            state.predict(now, target.target_number);
        });
        // 【瞄准点高度用"当前可见那块板"】4 块装甲板物理等高，但模型里它们的高度
        // 并不一致（来自不可观的车体倾斜/H，实测极差 ~0.1m ≈ 4°）。超前量会把瞄准点
        // 换到"下一块板"，于是瞄准点高度跟着换 → 枪口偏低/偏高。这里记下"现在正对相机
        // 那块板"的高度，供 select_and_get_control_point 覆盖预测结果。
        obs_plate_z_.reset();
        if (params_.aim_z_from_observed) {
            const auto& st_now = target.get_target_state();
            const auto poses_now = st_now.get_armors_pose(target.target_number);
            if (!poses_now.empty()) {
                int best = -1;
                // 优先用"本帧真正匹配到的那块板"的高度（GPT 指出：原来只按模型朝向挑，
                // 没有和 det_plate_id 绑定，自旋换板时参考板可能与实际观测的板不是同一块）。
                if (target.dbg_match_num >= 0
                    && target.dbg_match_num < static_cast<int>(poses_now.size())) {
                    best = target.dbg_match_num;
                } else {
                    const double cy = std::atan2(st_now.pos().y(), st_now.pos().x());
                    double best_v = std::numeric_limits<double>::infinity();
                    best = 0;
                    for (int i = 0; i < static_cast<int>(poses_now.size()); ++i) {
                        const auto rpy = utils::matrix2rpy<double>(poses_now[i].linear());
                        const double v = std::abs(angles::normalize_angle(rpy[2] - cy));
                        if (v < best_v) {
                            best_v = v;
                            best = i;
                        }
                    }
                }
                obs_plate_z_ = poses_now[best].translation().z();
                dbg_ref_id_ = best;
                dbg_ref_z_ = *obs_plate_z_;
            }
            // 4 块模型板的高度 + H + 车体 roll/pitch（诊断用）
            for (int i = 0; i < 4 && i < static_cast<int>(poses_now.size()); ++i) {
                dbg_z_[i] = poses_now[i].translation().z();
            }
            const auto& xs = st_now.x;
            dbg_h_ = xs[armor_point_motion_model::idx::P2];
            // 【注意】原来这里记的是旋转向量分量 C_ROT_X/C_ROT_Y，它不是 roll/pitch（会误导，
            // 实测读出来 40°+ 其实只是旋转向量分量）。改成真欧拉角（ZYX 分解）：
            //   car_roll/car_pitch = 车体相对 odom 的真实横滚/俯仰
            // 这个量才是"倾斜×旋转导致板高起伏"机制里的 tilt。
            const Vec3 car_rotvec(
                xs[armor_point_motion_model::idx::C_ROT_X],
                xs[armor_point_motion_model::idx::C_ROT_Y],
                xs[armor_point_motion_model::idx::C_ROT_Z]
            );
            const auto car_rpy = utils::matrix2rpy<double>(
                utils::so3_exp<double>(car_rotvec), utils::RPYOrder::ZYX
            );
            dbg_roll_ = angles::to_degrees(car_rpy[0]);
            dbg_pitch_ = angles::to_degrees(car_rpy[1]);
        }
        auto make_even = [](int x) { return x % 2 == 0 ? x : x + 1; };
        const int horizon = make_even(params_.sample_horizon);
        const double dt = params_.sample_total_time / horizon;
        auto sample_once = [&](double t,
                               const ArmorTarget& base_target,
                               AutoAimFsm fsm_mode,
                               const dta_utils::ControlPoint& _cp0,
                               dta_utils::GimbalState& out_gs,
                               AimPoint& out_ap) -> bool {
            auto tmp_target = base_target.fast_copy_without_ekf();
            tmp_target.set_target_state([&](armor_point_motion_model::State& state) {
                state.predict(t, tmp_target.target_number);
            }); //保证轨迹完全为不同时间对同一状态的瞄准控制点
            auto hit_opt = get_hit(tmp_target, bullet_speed, fsm_mode, shoot_in_gimbal_odom);
            if (!hit_opt)
                return false;
            auto cp = select_and_get_control_point(
                hit_opt->hit_time_target,
                bullet_speed,
                fsm_mode,
                shoot_in_gimbal_odom,
                gimbal_in_gimbal_odom
            );
            if (!cp.valid)
                return false;
            out_gs.aim_id = cp.aim_id;
            out_gs.yaw_state.p = angles::normalize_angle(cp.yaw - _cp0.yaw);
            out_gs.pitch_state.p = angles::normalize_angle(cp.pitch - _cp0.pitch);
            out_ap = cp.aim_point;
            return true;
        };
        auto push_sample = [&](auto& traj_gs,
                               auto& traj_ap,
                               double t,
                               const dta_utils::ControlPoint& _cp0,
                               AutoAimFsm fsm_mode) -> bool {
            dta_utils::GimbalState gs;
            AimPoint ap;
            if (!sample_once(t, target, fsm_mode, _cp0, gs, ap)) {
                return false;
            }
            traj_gs.push_back(gs, t);
            traj_ap.push_back(ap, t);
            return true;
        };
        if (!is_same) {
            auto hit_ctx_opt =
                get_hit(target, bullet_speed, fsm, shoot_in_gimbal_odom); //直接算到击中目标时间
            if (!hit_ctx_opt) {
                return cmd;
            }
            const auto& hit_ctx = *hit_ctx_opt;
            auto cp0 = select_and_get_control_point(
                hit_ctx.hit_time_target,
                bullet_speed,
                fsm,
                shoot_in_gimbal_odom,
                gimbal_in_gimbal_odom
            );
            cmd.dbg_pz = dbg_pz_;
            cmd.dbg_d = dbg_d_;
            cmd.dbg_alpha_deg = dbg_alpha_deg_;
            cmd.dbg_aim_id = cp0.valid ? cp0.aim_id : dbg_id_;
            cmd.dbg_ref_id = dbg_ref_id_;
            cmd.dbg_ref_z = dbg_ref_z_;
            for (int i = 0; i < 4; ++i) cmd.dbg_z[i] = dbg_z_[i];
            cmd.dbg_h = dbg_h_;
            cmd.dbg_car_roll = dbg_roll_;
            cmd.dbg_car_pitch = dbg_pitch_;
            cmd.dbg_valid = 1;
            cmd.dbg_age_ms = 0.0;
            dbg_id_ = cmd.dbg_aim_id;
            dbg_tp_ = now;
            if (!cp0.valid) {
                return cmd;
            }
            last_fly_time_ = hit_ctx.fly_time;
            last_select_ = cp0.aim_id;
            auto build_traj = [&](auto& traj_gs,
                                  auto& traj_ap,
                                  const dta_utils::ControlPoint& _cp0,
                                  AutoAimFsm fsm_mode,
                                  int horizon,
                                  double dt) -> bool {
                int half = horizon / 2;
                for (int i = half; i >= 1; --i) {
                    if (!push_sample(traj_gs, traj_ap, -i * dt, _cp0, fsm_mode)) {
                        return false;
                    }
                }

                dta_utils::GimbalState gs0;
                gs0.aim_id = _cp0.aim_id;
                gs0.yaw_state.p = 0.0;
                gs0.pitch_state.p = 0.0;
                traj_gs.push_back(gs0, 0.0);
                traj_ap.push_back(_cp0.aim_point, 0.0);

                for (int i = 1; i <= half; ++i) {
                    if (!push_sample(traj_gs, traj_ap, i * dt, _cp0, fsm_mode)) {
                        return false;
                    }
                }

                return true;
            };
            limit_traj_cp0_ = cp0;
            limit_traj_.clear();
            aim_traj_.clear();
            limit_traj_.reserve(horizon + 1);
            aim_traj_.reserve(horizon + 1);

            if (!build_traj(limit_traj_, aim_traj_, limit_traj_cp0_, fsm, horizon, dt)) {
                return cmd;
            }
            if (!mpc_traj_) {
                limit_traj_.build_limit(params_.max_yaw_acc, params_.max_pitch_acc, time_in_traj);
            } else {
                if (!mpc_traj_->solve(limit_traj_, time_in_traj)) {
                    AWAKENING_ERROR("very_aimer: mpc solve failed");
                    return cmd;
                }
            }

            if (fsm == AutoAimFsm::AIM_WHOLE_CAR_CENTER) { //瞄准中间的目标和控制不一样
                aim_center_target_traj_.clear();
                aim_center_target_traj_.reserve(horizon + 1);
                auto aim_center_target_hit_ctx_opt = get_hit(
                    target,
                    bullet_speed,
                    AutoAimFsm::AIM_WHOLE_CAR_ARMOR,
                    shoot_in_gimbal_odom
                );
                if (!aim_center_target_hit_ctx_opt) {
                    return cmd;
                }
                const auto& aim_center_target_hit_ctx = *aim_center_target_hit_ctx_opt;
                aim_center_target_traj_cp0_ = select_and_get_control_point(
                    aim_center_target_hit_ctx.hit_time_target,
                    bullet_speed,
                    AutoAimFsm::AIM_WHOLE_CAR_ARMOR,
                    shoot_in_gimbal_odom,
                    gimbal_in_gimbal_odom
                );
                aim_center_aim_traj_.clear();
                aim_center_aim_traj_.reserve(horizon + 1);
                if (!build_traj(
                        aim_center_target_traj_,
                        aim_center_aim_traj_,
                        aim_center_target_traj_cp0_,
                        AutoAimFsm::AIM_WHOLE_CAR_ARMOR,
                        horizon,
                        dt
                    ))
                {
                    return cmd;
                }
            }
        } else {
            const int target_forward_idx = ((int)(horizon / 2) + time_in_traj / dt);
            auto replenish_traj = [&](auto& traj_gs,
                                      auto& traj_ap,
                                      const dta_utils::ControlPoint& _cp0,
                                      AutoAimFsm fsm_mode,
                                      int target_forward_idx,
                                      double dt) -> bool {
                if (traj_gs.empty())
                    return false;
                int last_forward_idx =
                    static_cast<int>(std::floor(traj_gs.time_at(traj_gs.size() - 1) / dt));
                for (int i = last_forward_idx + 1; i <= target_forward_idx; ++i) {
                    if (!push_sample(traj_gs, traj_ap, i * dt, _cp0, fsm_mode)) {
                        return false;
                    }
                }
                return true;
            };
            const size_t old_limit_size = limit_traj_.size();
            if (!replenish_traj(
                    limit_traj_,
                    aim_traj_,
                    limit_traj_cp0_,
                    fsm,
                    target_forward_idx,
                    dt
                )) {
                return cmd;
            }
            if (limit_traj_.size() > old_limit_size) {
                if (!mpc_traj_) {
                    limit_traj_.update_limit_after_append(
                        params_.max_yaw_acc,
                        params_.max_pitch_acc,
                        time_in_traj,
                        old_limit_size
                    );
                } else {
                    if (!mpc_traj_->solve(limit_traj_, time_in_traj)) {
                        AWAKENING_ERROR("very_aimer: mpc solve failed");
                        return cmd;
                    }
                }
            }
            if (fsm == AutoAimFsm::AIM_WHOLE_CAR_CENTER) { //瞄准中间的目标和控制不一样
                if (!replenish_traj(
                        aim_center_target_traj_,
                        aim_center_aim_traj_,
                        aim_center_target_traj_cp0_,
                        AutoAimFsm::AIM_WHOLE_CAR_ARMOR,
                        target_forward_idx,
                        dt
                    ))
                {
                    return cmd;
                }
            }
        }

        const bool use_center_target = fsm == AutoAimFsm::AIM_WHOLE_CAR_CENTER;
        const dta_utils::ControlPoint& target_traj_cp0 =
            use_center_target ? aim_center_target_traj_cp0_ : limit_traj_cp0_;
        const Trajectory<dta_utils::GimbalState, double>& target_traj = use_center_target
            ? static_cast<const Trajectory<dta_utils::GimbalState, double>&>(aim_center_target_traj_
            )
            : static_cast<const Trajectory<dta_utils::GimbalState, double>&>(limit_traj_);
        const Trajectory<AimPoint, double>& aim_traj =
            use_center_target ? aim_center_aim_traj_ : aim_traj_;
        auto target_gimbal_state = target_traj.Trajectory::state_at(time_in_traj);
        //目标轨迹，一定击中目标
        auto get_control = [&](double t) {
            if (!mpc_traj_) {
                return limit_traj_.dta_utils::LimitTrajectory::state_at(t);
            } else {
                return mpc_traj_->state_at(t);
            }
        };
        auto control = get_control(time_in_traj);
        //控制轨迹，轨迹优化后最优控制（并非最优，下位机实际vel acc 可以基于error和上位机规划叠加）
        double control_yaw = angles::normalize_angle(control.yaw_state.p + limit_traj_cp0_.yaw);
        double control_pitch =
            angles::normalize_angle(control.pitch_state.p + limit_traj_cp0_.pitch);
        double target_yaw =
            angles::normalize_angle(target_gimbal_state.yaw_state.p + target_traj_cp0.yaw);
        double target_pitch =
            angles::normalize_angle(target_gimbal_state.pitch_state.p + target_traj_cp0.pitch);
        cmd.timestamp = Clock::now();
        cmd.yaw = angles::to_degrees(control_yaw);
        cmd.v_yaw = angles::to_degrees(control.yaw_state.v);
        cmd.a_yaw = angles::to_degrees(control.yaw_state.a);
        cmd.pitch = angles::to_degrees(control_pitch);
        cmd.v_pitch = angles::to_degrees(control.pitch_state.v);
        cmd.a_pitch = angles::to_degrees(control.pitch_state.a);
        cmd.target_yaw = angles::to_degrees(target_yaw);
        cmd.target_pitch = angles::to_degrees(target_pitch);
        cmd.fly_time = last_fly_time_;
        cmd.appear = true;
        // 复用轨迹（本拍没重建）：诊断量沿用上次重建时的值，并标注年龄，避免统计混入零值
        if (cmd.dbg_valid == 0) {
            cmd.dbg_pz = dbg_pz_;
            cmd.dbg_d = dbg_d_;
            cmd.dbg_alpha_deg = dbg_alpha_deg_;
            cmd.dbg_aim_id = dbg_id_;
            cmd.dbg_age_ms =
                std::chrono::duration<double, std::milli>(now - dbg_tp_).count();
        }
        cmd.aim_point = aim_traj.state_at(time_in_traj);
        cmd.aim_point.frame_id = target.get_target_state().frame_id;
        cmd.select_id = last_select_;
        const bool is_big = target.target_number == ArmorClass::NO1;
        const double half_w =
            (is_big ? params_.shooting_range_w_large : params_.shooting_range_w_small) * 0.5;
        const double half_h = params_.shooting_range_h * 0.5;
        const double min_enable_yaw = angles::from_degrees(params_.min_enable_yaw_deg);
        const double min_enable_pitch = angles::from_degrees(params_.min_enable_pitch_deg);
        const double pitch_factor = std::cos(FIFTTEN_DEGREE_RAD);
        auto cal_enbale_diff = [&](double _t) {
            auto aim_point = aim_traj.state_at(_t);
            const auto& pos = aim_point.pose.translation();
            const double distance = pos.norm();
            auto pose1_in_aim_point = ISO3::Identity();
            pose1_in_aim_point.translation() = Vec3(0, -half_w, 0);
            auto pose2_in_aim_point = ISO3::Identity();
            pose2_in_aim_point.translation() = Vec3(0, half_w, 0);
            auto pose1_in_odom = aim_point.pose * pose1_in_aim_point;
            auto pose2_in_odom = aim_point.pose * pose2_in_aim_point;
            auto yaw1 =
                std::atan2(pose1_in_odom.translation().y(), pose1_in_odom.translation().x());
            auto yaw2 =
                std::atan2(pose2_in_odom.translation().y(), pose2_in_odom.translation().x());
            auto aim_yaw = std::atan2(pos.y(), pos.x());
            double shooting_range_yaw = std::min(
                std::abs(angles::normalize_angle(yaw1 - aim_yaw)),
                std::abs(angles::normalize_angle(yaw2 - aim_yaw))
            ); //直接算两个边缘yaw
            double shooting_range_pitch = std::abs(std::atan2(half_h, distance));
            auto rpy = utils::matrix2rpy<double>(aim_point.pose.linear(), utils::RPYOrder::XYZ);
            shooting_range_pitch *= std::cos(rpy[1]);
            shooting_range_yaw = std::max(shooting_range_yaw, min_enable_yaw);
            shooting_range_pitch = std::max(shooting_range_pitch, min_enable_pitch);
            return std::make_pair(std::abs(shooting_range_yaw), std::abs(shooting_range_pitch));
        };
        auto enable_diff = cal_enbale_diff(time_in_traj);
        cmd.enable_yaw_diff = angles::to_degrees(enable_diff.first);
        cmd.enable_pitch_diff = angles::to_degrees(enable_diff.second);
        auto abs_angle_error = [](double from, double to) {
            return std::abs(angles::shortest_angular_distance(from, to));
        };
        cmd.fire_advice =
            abs_angle_error(angles::from_degrees(cmd.target_yaw), angles::from_degrees(cmd.yaw))
                < cmd.enable_yaw_diff
            && abs_angle_error(
                   angles::from_degrees(cmd.target_pitch),
                   angles::from_degrees(cmd.pitch)
               ) < cmd.enable_pitch_diff;
        if (fsm != AutoAimFsm::AIM_WHOLE_CAR_CENTER) {
            auto delay_fire = [&](double delay) {
                auto delay_control = get_control(time_in_traj + delay);
                auto delay_target = target_traj.Trajectory::state_at(time_in_traj + delay);
                auto delay_enable = cal_enbale_diff(time_in_traj + delay);
                const double control_yaw =
                    angles::normalize_angle(delay_control.yaw_state.p + limit_traj_cp0_.yaw);
                const double target_yaw =
                    angles::normalize_angle(delay_target.yaw_state.p + target_traj_cp0.yaw);
                const double control_pitch =
                    angles::normalize_angle(delay_control.pitch_state.p + limit_traj_cp0_.pitch);
                const double target_pitch =
                    angles::normalize_angle(delay_target.pitch_state.p + target_traj_cp0.pitch);

                return abs_angle_error(control_yaw, target_yaw) < delay_enable.first
                    && abs_angle_error(control_pitch, target_pitch) < delay_enable.second;
            };
            {
                bool has_can = false;

                double t_check = 0 + params_.fire_delay_min; //发射延迟内不让打弹
                while (t_check < (0 + params_.fire_delay_max) && t_check <= horizon / 2.0) {
                    if (!delay_fire(+t_check)) {
                        cmd.no_shoot();
                    } else {
                        has_can = true;
                    }
                    t_check += (dt / 2.0);
                }
                // if (!has_can) {
                //     cmd.no_shoot();
                // }
            }
        }

        // 【开火前再核一次"云台实际角"】上面的 fire_advice 比较的是 target_traj 与
        // control_traj —— 两者都在解算内部、只差 ~0.01°，等于恒真；而云台实际角(IMU)可能
        // 还差好几度。实测开火帧里 31% 的 |cmd.pitch - act| > 2.4°（最大 17.9°），也就是
        // "云台没跟上就已经开火"。这里要求"云台实际角 ≈ 理想瞄准角"才允许开火：
        //   fire_actual_tol_deg = 0 关闭（回到旧行为）；建议 1.5（≈ 命中窗口 2.4° 的 2/3）。
        if (params_.fire_actual_tol_deg > 0.0) {
            const auto cur_rpy = utils::matrix2rpy<double>(
                gimbal_in_gimbal_odom.linear(), utils::RPYOrder::ZYX);
            const double cur_pitch_deg = angles::to_degrees(cur_rpy[1]);
            const double cur_yaw_deg = angles::to_degrees(cur_rpy[2]);
            const auto err_deg = [](double a, double b) {
                return std::abs(angles::shortest_angular_distance_degrees(a, b));
            };
            if (err_deg(cmd.target_pitch, cur_pitch_deg) > params_.fire_actual_tol_deg
                || err_deg(cmd.target_yaw, cur_yaw_deg) > params_.fire_actual_tol_deg) {
                cmd.no_shoot();
            }
        }

        return cmd;
    }
    void set_operator_offset(std::pair<double, double> offset) {
        operator_offset_ = offset;
    }
    std::pair<double, double> get_yaw_pitch_offset() const noexcept {
        return std::make_pair(
            base_yaw_offset_rad_ + operator_offset_.first, //操作手在线调偏置
            base_pitch_offset_rad_ + operator_offset_.second
        );
    }
    BallisticTrajectory::Ptr ballistic_trajectory_;
    double base_yaw_offset_rad_;
    double base_pitch_offset_rad_;
    std::pair<double, double> operator_offset_ = std::make_pair(0, 0);
    // 当前(未超前)状态下、正对相机那块板的高度；每次 very_aim 刷新一次
    mutable std::optional<double> obs_plate_z_;
    // 【诊断】最近一次 get_control_point 的弹道输入/输出（cp0 调用后立即取走）
    mutable double dbg_pz_ = 0.0;
    mutable double dbg_d_ = 0.0;
    mutable double dbg_alpha_deg_ = 0.0;
    mutable int dbg_id_ = -1;
    mutable TimePoint dbg_tp_ {};
    mutable int dbg_ref_id_ = -1;
    // 【观测仰角】本帧从 target 取来，供 get_control_point 使用
    mutable double obs_elev_cam_ = 0.0;
    mutable double obs_cam_z_ = 0.0;
    mutable bool obs_valid_ = false;
    mutable double dbg_ref_z_ = 0.0;
    mutable double dbg_z_[4] = {0, 0, 0, 0};
    mutable double dbg_h_ = 0.0;
    mutable double dbg_roll_ = 0.0;
    mutable double dbg_pitch_ = 0.0;
    MPCType::Ptr mpc_traj_;
};
VeryAimer::VeryAimer(const YAML::Node& config) {
    _impl = std::make_unique<Impl>(config);
}
VeryAimer::~VeryAimer() noexcept {
    _impl.reset();
}
GimbalCmd VeryAimer::very_aim(
    const ArmorTarget& target,
    double bullet_speed,
    const AutoAimFsm& fsm,
    const ISO3& shoot_in_gimbal_odom,
    const ISO3& gimbal_in_gimbal_odom
) {
    return _impl->very_aim(target, bullet_speed, fsm, shoot_in_gimbal_odom, gimbal_in_gimbal_odom);
}
std::pair<double, double> VeryAimer::get_yaw_pitch_offset() {
    return _impl->get_yaw_pitch_offset();
}
void VeryAimer::set_operator_offset(std::pair<double, double> offset) {
    _impl->set_operator_offset(offset);
}
} // namespace awakening::auto_aim
