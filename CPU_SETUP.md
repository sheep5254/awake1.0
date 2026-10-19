# 本机自瞄（auto_aim）运行说明（无 GPU / 无相机）

本机为 Ubuntu 22.04 + Intel i7-13620H（无 NVIDIA GPU、无相机、无 `sudo`）。
以下配置让 `standard` 自瞄链路以 **OpenVINO (CPU)** 推理、**视频文件** 作为图像源运行。

## 一键运行

```bash
# 构建（首次或改动源码后）
./build_cpu.sh

# 只跑自瞄算法
./run_autoaim.sh                      # 默认 config/video_test.yaml，开启 debug

# 带 Web 可视化（http://127.0.0.1:9000）
./run_web.sh &                        # 另开一个终端
./run_autoaim.sh
```

Web 页面：`http://127.0.0.1:9000`（`/video` 标注流，`/data` 曲线，`/log` 目标状态）。

> 注意：C++ 侧通过 `pgrep -x wust_vision_web` 判断 Web 是否在运行，
> 因此 `web.py` 与 `standard` 必须在同一个 PID 命名空间/终端环境里启动。

## 与默认部署的差异

| 项目 | 默认（机器人） | 本机 |
| :--- | :--- | :--- |
| 推理后端 | TensorRT/CUDA | OpenVINO CPU |
| 编译器 | clang + C++23 | **gcc-12 + C++23**（clang-14 不接受 `-std=c++23`） |
| 相机 | Hik 工业相机 | 视频文件 `testdata/armor_loop.mp4` |
| 串口 | `/dev/ttyACM0` | 关闭 |
| ROS2 | 开启 | 关闭（缺少雷达所需的 Open3D） |

## 关键配置：`config/video_test.yaml`

由 `config/omni.yaml` 派生，改动如下：

- `camera.type: video`，新增 `camera.video.{path,fps,loop,start_frame}`
- `serial.enable: false`、`auto_exposure.enable: false`
- OpenVINO `device_name: "CPU"`，并删除 `priority: HIGH`
  （OpenVINO 2025.0 的 CPU 插件不支持 `MODEL_PRIORITY` 属性）
- `camera_info` 内参按 1440x1080 → 1280x720 缩放

## 为使工程可编译/可运行所做的修复

1. `CMakeLists.txt` — `relink` 未链接任何依赖，无法预编译 PCH，对其设置
   `DISABLE_PRECOMPILE_HEADERS ON`。
2. `3rdparty/backward-cpp/` 为空子模块目录，已放入单头文件 `backward.hpp`。
3. `src/utils/drivers/video_player.hpp` — `standard` 在 `init()` 后立即检查
   `is_running()`，而 `VideoPlayer` 只在 `start_capture()` 中打开文件；补充
   `init()` 覆写并让 `start_capture()` 幂等。
4. `src/tasks/auto_aim/armor_track/armor_target.cpp` — `DIFFZ_N == 1`，但
   `r(1, 1)` 越界写入 1x1 的 Eigen 矩阵，单装甲板观测时会触发
   **stack smashing** 崩溃；已删除该行。

## 实测结果

- 处理帧率 ≈ 60 FPS（受视频源限制），端到端平均延迟 ≈ 12–13 ms
- 识别 → 跟踪 → 状态估计 → 弹道解算全链路正常
- 稳定锁定 `no4` 装甲板（见 `testdata/web_frame.jpg`）
