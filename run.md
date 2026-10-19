# 自瞄（auto_aim）运行说明

本文只讲**怎么单独跑自瞄**以及怎么看结果。构建流程、环境差异、代码修复原因见
[`CPU_SETUP.md`](CPU_SETUP.md)。
- 主程序：`bin/standard`，入口源码 `src/runtime/standard.cpp`
- 当前默认配置：**`config/omni.yaml`（海康相机，1280x720）**

## 1. 一键启动

```bash
cd ~/Downloads/awakening-main
./run_autoaim.sh
```

## 9. 重新构建

改过源码或换机器后：

```bash
./build_cpu.sh
```


cd ~/Downloads/awakening-main
./plotjuggler.sh


停止：`Ctrl-C`。

- **插了相机**：正常出帧、识别、跟踪。
- **没插相机**：会每秒打印一次 `No USB cameras found` 并一直重试，**不会崩**，按 `Ctrl-C` 干净退出。
- **想用视频测试**（不需要相机）：

```bash
./run_autoaim.sh config/video_test.yaml
```

> 单独跑自瞄**不需要** Python、不需要 venv、不需要 ROS2、不需要 Web 调试器。

---

## 2. 相机配置（`config/omni.yaml`）

```yaml
camera:
  type: hik
  hik_camera:
    target_sn: DA4973922      # 你相机的 SN（启动日志会列出所有已连接相机）
    pixel_format: BayerRG8
    width: 1280
    height: 720
    center_roi: true          # ROI 由 WidthMax/HeightMax 自动居中，不写死 offset
    offset_x: 0               # 仅 center_roi=false 时生效
    offset_y: 0
    flip: "180"               # 相机倒装补偿：none / vertical / horizontal / 180
    acquisition_frame_rate: 250
    exposure_time: 1500
    gain: 16.9
    gamma: 0.7
    trigger_mode: Off
    format: "bgr"             # BayerRG8 -> BGR
```

### 相机倒装（flip）

相机如果倒着装，用 `flip` 在**识别之前**把画面翻转正过来。软件翻转在 Bayer 解码之后做，
不会影响颜色（硬件 ReverseX/Y 翻转可能改变 Bayer 相位，这里不用）：

| 值 | 效果 | flipCode |
| :--- | :--- | :--- |
| `none`（默认） | 不翻转 | — |
| `vertical` | 上下翻转 | 0 |
| `horizontal` | 左右翻转 | 1 |
| `180` | 上下 + 左右（相机绕光轴转 180° 装） | -1 |

启动日志会打印 `hik_camera: flip = 180 (code -1)`。

> ⚠️ 绕光轴转 180° 装必须用 `180`。只翻上下的话左右会镜像，装甲板数字会反向、
> 瞄准偏航也会反。

翻转只改图像、不改内参。若倒装后测距偏差明显，标定时一并处理（主点按图像镜像修正）。

### SN 怎么查

`target_sn` 必须**精确匹配**，否则会一直重试。直接把相机插上跑一次，日志会把所有相机列出来：

```
Found 1 USB camera(s):
[ 0 ] SN = DA4973922
```

把 `[ 0 ] SN = ...` 后面的值填进 `target_sn` 即可。

### 换分辨率

只需改 `width`/`height`，ROI 居中偏移由相机满幅分辨率自动计算，不再写死：

```
OffsetX = (WidthMax  - width ) / 2
OffsetY = (HeightMax - height) / 2
```

对齐增量由相机节点自己处理（`center_roi: true`）。启动日志会打印实际值，例如
3072x2048 的相机裁 1280x720：

```
hik_camera: OffsetX centered = 896 (WidthMax=3072, size=1280)
hik_camera: OffsetY centered = 664 (HeightMax=2048, size=720)
```

内参仍需按新分辨率重新标定。

---

## 3. 内参（`camera_info`）

```yaml
camera_info:
  image_width: 1280
  image_height: 720
  camera_matrix:
    data: [1814.19888, 0., 650.27359,
           0., 1807.37177, 349.57031,
           0., 0., 1.]
```

> ⚠️ **这套内参目前是占位值**：它来自 1440x1080 传感器 + 6mm 镜头的旧标定，而本机相机是
> **MV-CS060-10UC-PRO（6MP，满幅约 3072x2048）**，两者不通用。
>
> 在 1280x720 下**必须重新标定**，标定后把 `image_width/image_height/camera_matrix/
> distortion_coefficients/projection_matrix` 一起换成新值，否则测距和弹道有固定偏差。

---

## 4. 本机（笔记本）专属设置

`omni.yaml` 原本是给机器人用的，针对这台笔记本改了三处：

| 项 | 机器人 | 本机 | 原因 |
| :--- | :--- | :--- | :--- |
| OpenVINO `device_name` | `GPU` | **`CPU`** | 本机 `available_devices = ['CPU']`，无可用 GPU |
| `priority` | `HIGH` | **注释掉** | OpenVINO 2025 的 CPU 插件不支持 `MODEL_PRIORITY` |
| `serial.enable` | `true` | **`false`** | 本机没接单片机 `/dev/ttyACM0` |

**同步回机器人时**：把这三项改回 `GPU` / `HIGH` / `true`（机器人若要 TensorRT，还需
`backend: tensorrt` 并用 `-DBUILD_WITH_TRT=ON` 重新编译）。

相机权限不足（提示打开设备失败）时，需要装 udev 规则：

```bash
sudo ./script/setup_udev.sh
```

---

## 5. 命令与参数

```bash
./run_autoaim.sh [配置文件] [debug]
```

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| 配置文件 | `config/omni.yaml` | 海康相机，标准自瞄 |
| debug | 自动判断 | 检测到 Web 调试器在跑 → `true`；否则 → `false` |

```bash
./run_autoaim.sh                                  # 海康相机
./run_autoaim.sh config/video_test.yaml           # 视频测试
./run_autoaim.sh config/omni.yaml true            # 强制开 debug（配 Web 看画面）
```

脚本会自动准备 `MVCAM_*`、`LD_LIBRARY_PATH` 等变量，在任意目录调用都可以。

---

## 6. 输出日志怎么看

正常运行时每秒打印一行：

```
detect: 60  track: 60  found: 10  solve: 999  serial: 0  camera: 60  avg_latency: 12.5 ms
```

| 字段 | 含义 | 正常值 |
| :--- | :--- | :--- |
| `detect` | 每秒完成推理的帧数 | ≈ `camera` |
| `track` | 每秒进入跟踪的帧数 | ≈ `detect` |
| `found` | 每秒匹配到目标的次数 | 0～60 波动都正常 |
| `solve` | 每秒弹道解算次数（1000Hz 定时） | ≈ 1000 |
| `serial` | 每秒收到的串口包数 | 接 MCU 才有；本机为 0 |
| `camera` | 每秒采集帧数 | ≈ 相机/视频帧率 |
| `avg_latency` | 采集 → 识别完成的平均耗时 | 个位数 ~ 十几 ms |

出现 `init target: no4` 表示已锁定 4 号装甲板，是**正常且期望**的输出。

---

## 7. 常见情况

| 现象 | 原因 | 处理 |
| :--- | :--- | :--- |
| 一直刷 `No USB cameras found` | 相机没插 / 没上电 / USB 线不是 3.0 / 驱动 | 检查连接；`lsusb` 应能看到相机 |
| `Camera with serial xxx not found` | `target_sn` 不对 | 用日志里列出的 `[ i ] SN = ...` 改配置 |
| 启动就 `terminate` / `abort` | OpenVINO 设备不对（GPU 不存在） | 确认 `device_name: "CPU"` |
| `serial` 相关报错刷屏 | 本机没接 MCU 但 `serial.enable` 为 true | 本机改成 `false` |
| 启动 4 条 `[error]`：Gamma / ADCBitDepth / TriggerSource / TriggerActivation | 该相机型号不支持这些节点（TriggerMode=Off 时触发节点禁用） | **无害**，关键参数都已生效 |
| 实际帧率低于 `acquisition_frame_rate` | 相机在该分辨率/像素格式下的上限 | 正常，如 MV-CS060 在 1280x720 约 165fps |
| 画面全黑、曝光被拉到上限 | 镜头盖没摘 / 没装镜头 / 环境太暗 | 用 imshow 或 Web 看画面确认 |
| 没看到画面窗口 | 无显示环境，或窗口被强制关闭 | 见第 8 节；`AWAKENING_IMSHOW=1` 强制开 |
| 某几秒 `found: 0` | 目标被遮挡 / 短暂丢失 | 正常，跟踪器会重新捕获 |
| `Ctrl-C` 后刷 `[critical] 改了东西...` | 作者的关机提醒 | 正常，无需处理 |

后台运行并保存日志：

```bash
nohup ./run_autoaim.sh > autoaim.log 2>&1 &
tail -f autoaim.log
```

---

## 8. 可视化

两种看画面的方式，可以同时用。

### 8.1 原生窗口 imshow（本机推荐）

```bash
./run_autoaim.sh
```

- 检测到显示环境（`DISPLAY` / `WAYLAND_DISPLAY`）时**自动开启**
- 窗口标题 `awakening auto_aim`，显示的是**处理后的画面**（识别框、目标、FSM、延迟），
  以及左上角绿色的 **`FPS(detect): xx.x`** —— 这是**实际完成推理的帧率**，不是相机出图帧率。
  两者不一致（如相机 241、detect 157）说明推理跟不上，有帧被丢弃
- 按 `ESC` 或 `q` 关闭
- 强制开关：`AWAKENING_IMSHOW=1` 强制开，`AWAKENING_NO_IMSHOW=1` 强制关
- 没有可用显示时只打印一条错误日志，算法照常运行，不会崩

### 8.2 Web 可视化（远程 / 手机看）

**必须先起 Web，再起自瞄**：

```bash
# 终端 1（保持运行）
./run_web.sh

# 终端 2
./run_autoaim.sh
```

浏览器打开 **http://127.0.0.1:9000**：`/` 调试页面，`/video` 是带识别框的 MJPEG 流。
`run_web.sh` 会自动选一个能 `import flask` 的 Python（venv 里没有也没关系）。

---

## 9. 重新构建

改过源码或换机器后：

```bash
./build_cpu.sh
```

产物：`bin/standard`。构建参数说明见 [`CPU_SETUP.md`](CPU_SETUP.md)。

---

## 10. 相关文件

| 文件 | 作用 |
| :--- | :--- |
| `run_autoaim.sh` | 启动自瞄（本机主用，默认 `config/omni.yaml`） |
| `build_cpu.sh` | 无 GPU 环境的构建脚本 |
| `run_web.sh` | 启动 Web 可视化（可选） |
| `config/omni.yaml` | **当前默认**：海康相机 1280x720，标准自瞄 |
| `config/leg.yaml` | 舵轮兵种的海康配置（1440x1080） |
| `config/video_test.yaml` | 视频源配置（无相机时测试用） |
| `testdata/armor_loop.mp4` | 内置演示视频（含装甲板） |
| `bin/standard` | 自瞄可执行程序（主程序） |
| `src/runtime/standard.cpp` | 主程序入口源码 |
| `log/` | 运行日志 |
