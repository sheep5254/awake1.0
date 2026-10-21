# IMU / 俯仰动态跟随诊断

修改前检查点：`7b6d6b1`（`何总codex开始修改代码`）。

此补丁增加观测字段，不调整 PID、MPC、pitch 控制符号、串口帧格式、CRC、
姿态时间补偿值或开火逻辑。先前发现的开火单位混用问题仍需单独处理。

## 启用和运行

`config/omni.yaml`：

```yaml
frame_log:
  enable: true
  diagnostics: true
```

`enable` 控制原有 `run.jsonl`，`diagnostics` 控制新诊断文件，两个开关独立。
新程序启动后会创建：

```text
record/auto_aim/diagnostics_<启动时主机纳秒时间>/events.jsonl
```

每次启动使用新目录，不覆盖旧诊断记录。原有 `run.jsonl` 的覆盖行为保持原样。
需要保留旧 `run.jsonl` 时，先自行备份。

诊断版与正在运行的正式程序隔离：

```text
build-codex-diagnostics/bin/standard
build-codex-diagnostics/libawakening_*.so
```

需要重新编译时使用完整环境脚本，确保 CMake 重配置时 ROS/相机 SDK 环境齐全：

```bash
bash ./build_diagnostics.sh
```

默认以两个较低优先级的编译任务构建并运行无硬件测试。

确认旧实例已由操作者停止、避免多个实例占用同一相机/串口后，在工程目录执行：

```bash
bash ./run_diagnostics.sh config/omni.yaml true
```

脚本沿用原启动脚本的 ROS/SDK 环境设置，但只启动诊断版并优先加载对应的新共享库。
不能只把新 `standard` 二进制复制到旧 `bin/`：`ImageFrame` 增加元数据后，必须使用
同一版本的共享库。

结束本次测试时，在运行该程序的终端按 Ctrl+C，让后台写盘队列排空。
这只结束测试程序，不会退出 SSH。无需关闭或重启 SSH 服务。

## events.jsonl 字段

所有 `*_ns` 主机时间以 Unix 纳秒表示，只有明确命名为 `*_steady_ns` 的字段除外。
`steady_ns` 仅用于持续时间/间隔分析，不与 Unix 时间或相机设备计数直接相减。

### session

- `schema`：本诊断格式版本，目前为 1。
- `serial_send_to_image_microseconds`：实际使用的固定补偿值。
- `command_pitch_convention`：控制侧使用 `+ZYX rpy[1]`，单位度。
- `legacy_cam_pitch_convention`：旧逐帧日志的 `cam.pitch` 使用负号。
- `mcu_sample_timestamp_available: false`：当前 ARQ 协议没有 MCU 采样时刻。
- `device_timestamp_clock_calibrated: false`：相机时钟尚未与主机/MCU 时钟标定。

### imu

- `raw_wxyz`、`raw_norm`：CRC 通过、数值解析后，归一化之前的四元数和模长。
- `normalized_wxyz`：实际供姿态变换使用的单位四元数；顺序均为 wxyz。
- `host_received_ns`、`host_received_steady_ns`：完成该帧所在最后一个读块的 Asio 回调时刻。
  这是软件收包回调时刻，不是 UART 硬件采样或 MCU 采样时刻。
- `host_processed_ns`：调度后的解析任务时刻，兼容现有控制时间定义。
- `tf_assigned_ns`：沿用原逻辑写入 TF 的时间；本补丁不调整时间补偿或融合行为。
- `chunk_seq`、`index_in_chunk`：读块序号及本读块完成的第几个候选帧。
- `seq`：主机解析成功帧计数；不是 MCU 序号，无法用它精确计算串口丢包。
- `parse_rejects`：累计解析失败数，不等同于传输丢包数。

`imu_rejected` 记录候选帧拒收事件及 CRC/原始数值是否已解析。

### serial_chunk

- `bytes`、`frames`、`rejected`：本次读块字节数、完成的候选帧数、拒绝数。
- `host_received_*`：Asio 收包回调时刻。
- `host_processed_ns`、`host_parse_end_ns`：解析任务开始/完成时刻。

一帧可跨读块，一次读块也可包含多帧。日志保留此事实，不为合包帧虚构均匀采样时间。

### camera

- `frame_id`：本进程相机帧计数。
- `image_ns`：原有算法使用的图像时间戳，可与 `run.jsonl` 的 `t_ns` 精确关联。
- `pipeline_ns`：相机帧进入主处理数据流的主机时间。
- `host_buffer_ns`、`host_buffer_steady_ns`：海康 SDK 取帧返回时的主机时间。
- `exposure_us`：原有时间戳计算使用的曝光参数，单位微秒。
- `sdk_exposure_us`：海康帧信息中报告的原始曝光值。
- `hardware_frame_id`：SDK 帧号。
- `device_timestamp_ticks`：SDK 高低 32 位合并的原始设备时间计数。
- `sdk_host_timestamp_raw`：SDK 自带主机时间字段原值，未假定单位或纪元。
- `sdk_metadata_available`：当前帧是否携带海康硬件元数据。

当前活跃的海康同步/异步取帧路径都填充这些字段。其他相机/视频/共享内存来源若无
这类元数据，将保持 `sdk_metadata_available=false`，不能把默认 0 当成有效硬件时间。
`ImageFrame::clone()` 保留元数据；原有图像网络序列化格式不变，不新增线缆字段。

### nav_command

- `planner_pitch_deg`、`planner_yaw_deg`：规划器的绝对角，单位度。
- `current_pitch_rpy_deg`、`current_pitch_up_deg`：同一姿态的控制符号和上仰符号两个版本。
- `current_yaw_deg`：该次增量计算采用的 yaw 基准。
- `raw_pitch_delta_deg`、`raw_yaw_delta_deg`：限幅前的增量。
- `pitch`、`yaw`、`units`：实际传给 `cmd_gimbal_joint.position` 的值及单位。
- `mode`：`delta` 或 `absolute`；绝对模式输出始终为 rad。
- `pitch_limited`、`yaw_limited`：此次增量是否触发原有 ±3° 限幅。
- `appear`、`gimbal_published`：是否有目标及此次是否实际调用了云台消息发布。
  `gimbal_published=false` 时，`pitch/yaw` 不是一条已发布指令，应忽略。
- `planner_ns`：规划指令生成时刻。
- `pose_lookup_ns`：本次控制所用云台姿态的查询时刻。
- `latest_imu_seq_at_record`：记录事件时最新解析的 IMU 序号，辅助关联收包；
  它不是 TF 插值所使用的两个样本序号，不应当作精确插值来源。
- `gimbal_publish_begin_ns`、`gimbal_publish_end_ns`：ROS 云台 publish 调用前后时刻。
  未发布时为 0；它们不是导航实际收到或电机实际执行时刻。
- `host_published_ns`：云台/开火发布调用后的主机时刻。
- `shoot_request`：最终发出的开火建议数值；不表示实际发弹。

### recorder_status

- `written`：已经写入的数据事件数。
- `dropped`：诊断队列满时丢弃的数据事件数。

写盘在独立线程完成，队列容量 4096。队列满时丢弃诊断事件，不等待磁盘；
采集时需要检查 dropped，不能把诊断丢事件误认为串口丢包。
状态事件约每秒记录一次，并在正常结束时再记录最终统计。

## 测试后提供的文件

1. 本次 `diagnostics_*/events.jsonl`。
2. 对应的原有 `run.jsonl` 和使用的配置文件。
3. 另一个电控工程导出的编码器/目标角/速度/电流记录。

电控记录请保留时间戳、单位和正方向，并记录与上位机共有的测试起始标记。
没有独立编码器基准时，仍不能仅凭四元数日志区分机械未跟随和 AHRS 动态失真。

## 无硬件回归测试

```bash
ctest --test-dir build-codex-diagnostics --output-on-failure
```

测试覆盖归一化前模长、CRC、非法四元数、分包/合包、相机元数据 clone、
多生产者诊断队列、溢出计数、关闭排空和跨次启动日志隔离；不会连接相机或电机。
