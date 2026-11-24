# 发布端实现指南（给要"另写一份视觉"的人/AI）

> 配套文档：`docs/comms-protocol.md`（协议细节）
> 本文件回答三个问题：**要哪个 msg 包**、**怎么编**、**发布代码怎么写**。

---

## 1. 三个话题分别需要什么消息包

| 话题 | 消息类型 | 需要额外安装/编译吗 |
|---|---|---|
| `cmd_gimbal_joint` | `sensor_msgs/msg/JointState` | **不需要**，ROS 2 自带 |
| `cmd_shoot` | `example_interfaces/msg/UInt8` | **不需要**，ROS 2 自带（`ros-humble-example-interfaces`） |
| `tracker/target` | `auto_aim_interfaces/msg/Target` | **需要**，自定义包 → 已在本仓库 `interfaces/auto_aim_interfaces/` |

**结论：只有 `tracker/target` 需要自定义包**，另外两个用标准消息。
如果暂时不接 `tracker/target`（导航只做云台+开火），**完全不需要 `auto_aim_interfaces`**。

---

## 2. `auto_aim_interfaces` 怎么用

包里只有 8 个 msg，依赖仅 `std_msgs` + `geometry_msgs`（都是标准包）：

```
interfaces/auto_aim_interfaces/
├── CMakeLists.txt
├── package.xml
└── msg/{Target,Armor,Armors,DebugArmor,DebugArmors,DebugLight,DebugLights,TrackerInfo}.msg
```

`msg/Target.msg` 全文：

```
std_msgs/Header header
bool tracking
string id
int32 armors_num
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 yaw
float64 v_yaw
float64 radius_1
float64 radius_2
float64 dz
```

**编译**（放在你自己的 ws/src 下）：

```bash
cp -r interfaces/auto_aim_interfaces ~/your_ws/src/
cd ~/your_ws && colcon build --packages-select auto_aim_interfaces
source ~/your_ws/install/setup.bash
ros2 interface show auto_aim_interfaces/msg/Target    # 验证
```

`package.xml` 里需要：

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
```

---

## 3. 发布端完整示例（可直接用）

下面是**最小可用**的发布端，语义与真实实现一致（增量、±3° 限幅、丢目标 0/0、开火 0/1）。

### CMakeLists.txt 片段

```cmake
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(auto_aim_interfaces REQUIRED)   # 只要 tracker/target 才需要

add_executable(my_publisher src/my_publisher.cpp)
ament_target_dependencies(my_publisher rclcpp sensor_msgs example_interfaces auto_aim_interfaces)
```

### package.xml 片段

```xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>example_interfaces</depend>
<depend>auto_aim_interfaces</depend>
```

### src/my_publisher.cpp

```cpp
#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <example_interfaces/msg/u_int8.hpp>
#include <auto_aim_interfaces/msg/target.hpp>

class AimPublisher : public rclcpp::Node {
public:
    AimPublisher() : Node("aim_publisher") {
        // QoS 与真实实现一致：gimbal/shoot 用 reliable + KeepLast(10)
        gimbal_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "cmd_gimbal_joint", rclcpp::QoS(rclcpp::KeepLast(10)));
        shoot_pub_ = create_publisher<example_interfaces::msg::UInt8>(
            "cmd_shoot", rclcpp::QoS(rclcpp::KeepLast(10)));
        // tracker/target 用 SensorDataQoS（best_effort）
        target_pub_ = create_publisher<auto_aim_interfaces::msg::Target>(
            "tracker/target", rclcpp::SensorDataQoS());

        // 以 ~200 Hz 模拟"每收到一包 IMU 回一包"，实际请在你的 IMU 回调里调用 publish_frame()
        timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] { tick(); });
    }

private:
    static constexpr double MAX_STEP_DEG = 3.0;   // 单次增量限幅
    bool use_deg_ = false;                        // false = rad（默认）
    bool delta_mode_ = true;                      // delta / absolute

    // ---- 你的解算结果，填这里 ----
    double cmd_pitch_deg_ = 0.0;     // 规划后的控制角（度）
    double cmd_yaw_deg_ = 0.0;
    double imu_pitch_deg_ = 0.0;     // 云台当前实际角（来自 IMU，度）
    double imu_yaw_deg_ = 0.0;
    bool appear_ = false;            // 是否锁定目标
    bool fire_advice_ = false;       // 是否允许开火

    static double wrap180(double a) {
        while (a > 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }

    void tick() {
        // ===== 1) 云台角 =====
        bool send_gimbal = false;
        double pitch_cmd = 0.0, yaw_cmd = 0.0;

        if (appear_) {
            if (delta_mode_) {
                double dp = std::clamp(cmd_pitch_deg_ - imu_pitch_deg_, -MAX_STEP_DEG, MAX_STEP_DEG);
                double dy = std::clamp(wrap180(cmd_yaw_deg_ - imu_yaw_deg_), -MAX_STEP_DEG, MAX_STEP_DEG);
                pitch_cmd = use_deg_ ? dp : dp * M_PI / 180.0;
                yaw_cmd   = use_deg_ ? dy : dy * M_PI / 180.0;
            } else {                                  // absolute：世界系绝对角，恒为 rad
                pitch_cmd = cmd_pitch_deg_ * M_PI / 180.0;
                yaw_cmd   = cmd_yaw_deg_   * M_PI / 180.0;
            }
            send_gimbal = std::isfinite(pitch_cmd) && std::isfinite(yaw_cmd);  // NaN 不发
        } else if (delta_mode_) {
            pitch_cmd = 0.0;                          // 丢目标：0/0 = 别动
            yaw_cmd = 0.0;
            send_gimbal = true;
        }
        // absolute 模式丢目标：什么都不发（由 nav 锁存上一条）

        if (send_gimbal) {
            sensor_msgs::msg::JointState m;
            m.name = {"gimbal_pitch_joint", "gimbal_yaw_joint"};
            m.position = {pitch_cmd, yaw_cmd};
            gimbal_pub_->publish(m);
        }

        // ===== 2) 开火：每拍都发，只有 0 / 1 =====
        example_interfaces::msg::UInt8 s;
        s.data = (appear_ && fire_advice_) ? 1 : 0;
        shoot_pub_->publish(s);

        // ===== 3) 目标状态：>=15Hz 或 tracking 变化时发 =====
        static auto last = std::chrono::steady_clock::now();
        static bool last_tracking = false;
        static bool has_last = false;
        const auto now = std::chrono::steady_clock::now();
        const bool due = (now - last) >= std::chrono::milliseconds(66);
        if (due || !has_last || appear_ != last_tracking) {
            auto_aim_interfaces::msg::Target t;
            t.header.stamp = this->now();
            t.tracking = appear_;
            if (appear_) {
                // TODO: 填 id / armors_num / position / velocity / yaw / v_yaw / radius_1 / radius_2 / dz
            }
            target_pub_->publish(t);
            last = now;
            last_tracking = appear_;
            has_last = true;
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_pub_;
    rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr shoot_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AimPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

---

## 4. 接收端（nav）必须怎么写

```cpp
// 收到就转发！不要用 1Hz 定时器转发（实测：云台每秒只能被纠正 3°，自旋完全追不上）
void cmdGimbalJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "gimbal_pitch_joint") gimbal.pitch = msg->position[i];
        if (msg->name[i] == "gimbal_yaw_joint")   gimbal.yaw   = msg->position[i];
    }
    last_cmd_time = now();
    sendData();                      // 立刻下发
}

void cmdShootCallback(const example_interfaces::msg::UInt8::SharedPtr msg) {
    shoot.fric_on = true;
    shoot.fire = msg->data;          // 0 / 1
    last_cmd_time = now();
    sendData();
}

// 超时（1s 无新指令）：三个都要清
if (now() - last_cmd_time > 1s) {
    gimbal.pitch = 0.0f;   // ← 漏掉这句，云台会一直被旧增量推着漂
    gimbal.yaw   = 0.0f;
    shoot.fire   = 0;      // ← 漏掉这句，视觉挂了还会一直开火
}
```

---

## 5. 三个最容易踩的错

| 错误 | 后果 |
|---|---|
| 把 `position` 当**绝对角**（而发送端发的是增量） | 云台甩飞 / 抖成锯齿 |
| 发布频率 < 100 Hz | 自旋跟不上（视觉端是 200 Hz 的设计） |
| `cmd_shoot` 不用 0/1 或用"不发=停火" | 下位机锁存旧值 → 一直开火 |

对照检查：见 `docs/comms-protocol.md` 第 5 节"自检清单"。
