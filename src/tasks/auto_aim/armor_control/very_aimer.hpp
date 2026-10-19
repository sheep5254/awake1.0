#pragma once
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/auto_aim/auto_aim_fsm.hpp"
#include "tasks/base/common.hpp"
#include "utils/impl.hpp"
#include <memory>
#include <yaml-cpp/node/node.h>

namespace awakening::auto_aim {

// very_aim() 各条出口的计数（纯诊断，logger 每秒读取并清零）。
//   用途：看"自瞄到底从哪条路退出去、退了多频繁"——原来的日志只能看到
//   invalid cmd（NaN 护栏），看不到 get_hit / build_traj / replenish 这些静默出口。
struct AimPathStats {
    int calls = 0;        // very_aim 调用次数
    int ok = 0;           // 正常出指令（appear 保持 true）
    int hit0 = 0;         // get_hit() 失败（主目标）
    int cp0_invalid = 0;  // 当前控制点 cp0.valid == false
    int build_fail = 0;   // build_traj() 失败（某个采样点控制点无效）
    int mpc_fail = 0;     // mpc_traj_->solve() 返回 false
    int center_hit0 = 0;  // 瞄车心时 get_hit() 失败
    int center_build = 0; // 瞄车心时 build_traj() 失败
    int replenish = 0;    // replenish_traj() 失败
    int nan_cmd = 0;      // 算出的 cmd 含 NaN/Inf，被护栏拦下
    int state_bad = 0;    // 状态可信度门命中（位置/高度离谱）→ 输出上一条有效指令
    int clamp_v = 0;      // 平移速度超限被限幅（削弱前馈）
    int clamp_vyaw = 0;   // 转速超限被限幅（削弱前馈）
    void reset() {
        *this = {};
    }
};
inline AimPathStats& aim_path_stats() {
    static AimPathStats s;
    return s;
}

class VeryAimer {
public:
    VeryAimer(const YAML::Node& config);
    AWAKENING_IMPL_DEFINITION(VeryAimer)
    [[nodiscard]] GimbalCmd very_aim(
        const ArmorTarget& target,
        double bullet_speed,
        const AutoAimFsm& fsm,
        const ISO3& shoot_in_gimbal_odom,
        const ISO3& gimbal_in_gimbal_odom
    );
    std::pair<double, double> get_yaw_pitch_offset();
    void set_operator_offset(std::pair<double, double> offset);
};
} // namespace awakening::auto_aim