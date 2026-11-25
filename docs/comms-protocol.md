# 自瞄对外通信协议（话题 + 串口）

> 适用代码：本仓库 `src/runtime/standard.cpp`、`src/tasks/base/ascii_serial.hpp`
> 目标读者：需要"另写一份视觉/导航"并保持兼容的实现者（人或 AI）。
> 所有角度单位都在下面逐条写明：**度 = deg、弧度 = rad**，不要混用。

---

## 0. 一句话总览

```
[视觉] --(ROS 话题 cmd_gimbal_joint/cmd_shoot/tracker/target)--> [导航 nav] --(串口)--> [下位机 MCU]
[视觉] <--(串口 ARQ 四元数，收 IMU)-------------------------------------------- [下位机 MCU]
```

两种接法任选其一：

| 接法 | 谁发云台指令 | 需要改哪里 |
|---|---|---|
| **A（当前使用）** | 视觉发 ROS 话题，**nav 转串口** | 只需实现第 1 节的话题订阅 |
| **B** | 视觉**直接**发串口（AAP 帧） | 需要实现第 2 节的 AAP 帧 + 关掉 nav 的云台下发 |

---

## 1. ROS 话题（接法 A，推荐）

### 1.1 `cmd_gimbal_joint` — 云台角

| 项 | 值 |
|---|---|
| 消息类型 | `sensor_msgs/msg/JointState` |
| QoS | `reliable`，`KeepLast(10)` |
| 发布频率 | **跟随 IMU 包率 ~200 Hz**（每收到一包 IMU 回一包） |
| `name[0]` | `"gimbal_pitch_joint"` |
| `name[1]` | `"gimbal_yaw_joint"` |
| `position[0]` | pitch（语义见下） |
| `position[1]` | yaw（语义见下） |

**语义**由两个参数决定（视觉侧配置）：

| `nav_gimbal_mode` | 含义 | 丢目标时 |
|---|---|---|
| **`delta`（默认，推荐）** | **增量**：相对"当前云台角"的偏移量，下位机自己叠加 | **显式发 0/0**（= 别动） |
| `absolute` | 世界系绝对角 | **不发布**（由 nav 锁存上一条，避免 0/0 被当成 yaw=0 甩头） |

**单位**：`nav_gimbal_units: rad`（默认）或 `deg`；`absolute` 模式恒为 rad。

**delta 模式的两个硬约束**（视觉侧已实现，接收端要一致）：

```cpp
constexpr double MAX_STEP_DEG = 3.0;                       // 单次增量限幅
d_pitch = clamp(cmd.pitch - 当前IMU pitch, -3.0, +3.0);     // 度
d_yaw   = clamp(wrap180(cmd.yaw - 当前IMU yaw), -3.0, +3.0);
position[0] = 单位换算(d_pitch);  position[1] = 单位换算(d_yaw);
```

> 解释：视觉按 ~200 Hz 发布，±3° 限幅等效最高 600°/s，正常跟踪时增量远小于 3°。

**丢目标**（`appear == false`）：delta 模式下 `position = {0.0, 0.0}`（"别动"），
并且配合 `cmd_shoot = 0`，下位机可以用"fire/pitch/yaw 三零"判断丢失。

**异常保护**：若解算出 NaN/Inf，**这一帧不发布**（宁可少发，也不发 NaN）。

### 1.2 `cmd_shoot` — 开火

| 项 | 值 |
|---|---|
| 消息类型 | `example_interfaces/msg/UInt8` |
| QoS | `reliable`，`KeepLast(10)` |
| 发布频率 | **每拍都发**（不丢帧） |
| `data` | **`1` = 开火（连发）**，**`0` = 停火**（只有这两个值） |

开火条件（视觉侧已实现）：`appear && fire_advice`，其中 `fire_advice` 需要
"理想瞄准角 vs 规划器"和"理想瞄准角 vs **云台 IMU 实际角**"都在门限内
（`min_enable_*_deg`、`fire_actual_tol_deg`）。

### 1.3 `tracker/target` — 目标状态（给导航/决策用，可不接）

| 项 | 值 |
|---|---|
| 消息类型 | `auto_aim_interfaces/msg/Target` |
| QoS | `SensorDataQoS`（best_effort） |
| 发布频率 | **≥15 Hz**（66 ms 定时），且 `tracking` 变化时立即发 |
| 关键字段 | `tracking`(bool)、`id`(string)、`armors_num`、`position`、`velocity`、`yaw`、`v_yaw`、`radius_1`、`radius_2`、`dz` |

`tracking = appear`（是否锁定目标）。

### 1.4 编译/运行前提

- 视觉必须**编译时开 `USE_ROS2`**，并 source 含 `auto_aim_interfaces` 的工作空间；
- 配置里 `publish_to_nav: true`；
- 若只想记录、不想驱动云台：`publish_to_nav: false`（视觉完全不碰云台）。

---

## 2. 串口 ASCII 协议（接法 B，视觉直连下位机）

**物理层**：115200 8N1，无流控。当前设备 `/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0-port0` → `/dev/ttyUSB0`。

### 2.1 视觉 → 下位机：`AAP` 帧

```
AAP<pitch>Y<yaw>F<fire>N<frame>C<crc>%<LF>
例: AAP1.23Y-4.56F0N120Cc328%\n
```

| 字段 | 格式 | 含义 |
|---|---|---|
| `pitch` | `%.2f` | **度**，增量（相对当前云台角），建议 ±3° 内 |
| `yaw` | `%.2f` | **度**，增量，建议 ±3° 内 |
| `fire` | `%d` | `0` = 停火，`1` = 开火（连发） |
| `frame` | `%d` | 自增帧号（用于丢帧统计，可回绕） |
| `crc` | `%04x` | **小写 16 进制 4 位**，覆盖 `C` 之前的**整段 ASCII 字节**（含 `AAP...N<frame>`） |
| `%` | 字面量 | 帧尾标记 |
| `\n` | 0x0A | 结束符 |

**CRC 算法（固件实际使用）**：CRC16-MCRF4XX

```c
uint16_t crc16_mcrf4xx(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= d[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
    }
    return crc;
}
```

> ⚠️ 早期文档写的是 **Modbus**（`0xA001`），两者**互不兼容**。
> 视觉**发送**默认 MCRF4XX（与固件一致，可用 `serial.crc_mode: modbus` 切回）；
> 视觉**接收**两种都认（先试 MCRF4XX 再试 Modbus），避免文档/固件不一致时全丢帧。

### 2.2 下位机 → 视觉：`ARQ` 帧（IMU 四元数）

```
ARQ<w>X<x>Y<y>Z<z>C<crc>%<LF>      （末尾 '%' 可省）
例: ARQ0.7071X0.0000Y0.7071Z0.0000C2dd9%\n
```

| 字段 | 含义 |
|---|---|
| `w x y z` | 四元数，**wxyz 顺序**，接收端会归一化 |
| `crc` | 同 2.1，覆盖 `C` 之前整段（`ARQ...Z<z>`） |

**取帧要点（重要）**：串口上常混有别的协议的二进制包（实测夹着 ~70 字节 `0x5A` 开头的包），
且驱动交给上层的是**任意长度字节块**，所以必须：

1. 在字节流里搜 `"ARQ"` 作为帧头，丢掉前面的垃圾（保留末尾 2 字节，可能是半截帧头）；
2. 按 `'%'` 或 `'\n'` 切帧，跨读块拼接（单帧最长 256 字节）；
3. CRC 不符（两种算法都不符）就丢弃。

实测该串口 **~200 帧/秒、7.6 KB/s**（115200 的 68%）。

### 2.3 坐标系/符号约定（四元数）

```
+x 前、+y 左、+z 上；+yaw 左、+pitch 下、+roll 右倾；旋转顺序 ZYX
```

`pitch` 与代码里的 `rpy[1]`（ZYX 分解）同一套约定。

---

## 3. 串口二进制协议（旧版，接法 B 的另一选项）

`serial.send_protocol / recv_protocol = 0` 时启用。**小端**。

**帧头（4 字节）**：

| 偏移 | 类型 | 含义 |
|---|---|---|
| 0 | uint8 | `sof = 0x5A` |
| 1 | uint8 | `len`（数据段长度） |
| 2 | uint8 | `id`（`0x01` 发送 / `0x02` 接收） |
| 3 | uint8 | 帧头 CRC8 |

**视觉 → 下位机（`id=0x01`，`SendRobotCmdData`）**：

```
uint8  cmd_ID(0x01)
uint32 time_stamp
uint8  appear
float  pitch, yaw                 // 增量，rad
float  target_yaw, target_pitch   // 度
float  enable_yaw_diff, enable_pitch_diff
float  v_yaw, v_pitch
float  a_yaw, a_pitch
uint8  detect_color               // 0=红 1=蓝
... 末尾 CRC16
```

**下位机 → 视觉（`id=0x02`，`ReceiveRobotData`）**：

```
uint8  cmd_ID(0x02)
uint32 time_stamp_pc
uint32 time_stamp_receive_micro
uint32 time_stamp_send_micro
float  yaw, pitch, roll           // rad，+yaw 左/+pitch 下/+roll 右倾，ZYX
float  vyaw, vpitch, vroll
float  v_x, v_y, v_z
float  bullet_speed
uint8  detect_color
uint32 bullet_count
float  operator_yaw_offset, operator_pitch_offset
... 末尾 CRC16
```

> 二进制路径当前**未使用**（配置是 `send_protocol: 1 / recv_protocol: 1`）。
> 若要用，请以 `src/tasks/base/packet_typedef_send.hpp` /
> `packet_typedef_receive.hpp` 的字段顺序为准（结构体已 `packed`）。

---

## 3.5 ⚠️ pitch 符号：**两处必须成对取负**（2026-11-25 实车验证）

这是踩了最久的一个坑，复刻时务必照抄：

| 位置 | 参数 | 正确值 | 作用 |
|---|---|---|---|
| 视觉内部（`very_aimer`） | `ballistic_pitch_sign` | **-1.0** | 弹道仰角 α 进 `rpy2matrix` 时取负 |
| 视觉发布端（`standard.cpp`） | `nav_pitch_sign` | **-1.0** | 发出去的 pitch 增量再取负 |

**只改一个的后果**：
- 只改 `ballistic_pitch_sign` → 落点偏低（约 2α·d：1.2 m 低 2.7 cm、3 m 低 17 cm）
- 只改 `nav_pitch_sign` → 闭环变正反馈，云台朝反方向跑
- **两个都改 → 实车验证正常**

**原因**：`solve_pitch()` 返回的 α 是"物理仰角（向上为正）"，而 `rpy2matrix`/`matrix2rpy`
用的是"向上为负"的约定；发布端既要满足这个约定、又要与下位机的增量方向一致，
因此需要两次取负。

**验证方法**：固定靶 1.2 m 打 3 发，落点应居中（改动前会低 2~3 cm）；
3 m 处改动前后差异约 17 cm，最容易判断。

---

## 4. 导航侧（nav）应实现的契约

如果另写 nav，最小实现如下（参考 `standard_robot_pp_ros2`）：

```cpp
// 1) 订阅 cmd_gimbal_joint：直接当作"相对当前角度的偏移量"透传给下位机
void cmdGimbalJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "gimbal_pitch_joint") gimbal.pitch = msg->position[i];  // 单位按配置
    if (msg->name[i] == "gimbal_yaw_joint")   gimbal.yaw   = msg->position[i];
  }
  last_cmd_time = now();          // 刷新超时
  sendData();                     // 【重要】收到就转发，不要用 1 Hz 定时器转发
}

// 2) 订阅 cmd_shoot：0/1
void cmdShootCallback(const example_interfaces::msg::UInt8::SharedPtr msg) {
  shoot.fric_on = true;           // 摩擦轮
  shoot.fire    = msg->data;      // 0/1
  last_cmd_time = now();
  sendData();
}

// 3) 超时保护（1 s 没新指令）：pitch / yaw / fire 必须**全部**清零
if (now() - last_cmd_time > 1s) {
  gimbal.pitch = 0.0f;            // ← 少清一个，云台会一直被旧增量推着走
  gimbal.yaw   = 0.0f;
  shoot.fire   = 0;
}
```

**几个坑（实测踩过）**：

| 坑 | 现象 | 正确做法 |
|---|---|---|
| 用 1 Hz 定时器转发 | 云台每秒只被纠正 3° → 自旋完全追不上 | **收到话题就发**（视觉 200 Hz） |
| 超时只清 yaw | 视觉挂了云台还按旧 pitch 漂、并且仍在开火 | 三个量一起清 |
| 把 delta 当绝对角用 | 云台甩飞/抖动 | delta 就按增量叠加 |
| CRC 用 Modbus | 下位机全部拒帧（或视觉全丢） | 用 MCRF4XX（2.1 节） |

---

## 5. 自检清单（换实现后逐条对）

1. `ros2 topic hz /cmd_gimbal_joint` → **~200 Hz**
2. `ros2 topic hz /cmd_shoot` → 与上面同频（每拍都发）
3. `ros2 topic echo /cmd_gimbal_joint` → `name` 必须是 `gimbal_pitch_joint` / `gimbal_yaw_joint`
4. 静止锁定固定靶：`position` 应**接近 0**（±0.01 rad 级抖动）
5. 丢目标：`cmd_shoot=0`，`cmd_gimbal_joint` 在 delta 模式下为 0/0
6. 串口侧：`serial: 200`（收包率），且无 `running WITHOUT serial`
7. 串口抓包：能看到 `ARQ...Cxxxx%` 连续帧；发出去的是 `AAP...Cxxxx%\n`
8. 用下面这段 python 可直接验证一帧 CRC：

```python
def crc16_mcrf4xx(b: bytes) -> int:
    crc = 0xFFFF
    for x in b:
        crc ^= x
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc

body = b"AAP1.23Y-4.56F0N120"
print(f"C{crc16_mcrf4xx(body):04x}%")   # 与帧里的 C 后 4 位对比
```
