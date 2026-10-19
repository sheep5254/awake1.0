#include "angles.h"
#include "ascii_banner.hpp"
#include "daedalus_interface/shm_layout.hpp"
#include "tasks/auto_aim/armor_track/motion_model.hpp"
#include "tasks/auto_buff/rune_control/rune_aimer.hpp"
#include "tasks/auto_buff/rune_detect/rune_detector.hpp"
#include "tasks/auto_buff/rune_track/rune_target.hpp"
#include "tasks/auto_buff/rune_track/rune_tracker.hpp"
#include "tasks/auto_buff/type.hpp"
#include "tasks/base/ascii_serial.hpp"
#include "tasks/base/ballistic_trajectory.hpp"
#include "tasks/base/wheel_odometry.hpp"
#include "utils/drivers/camera_factory.hpp"
#include "utils/io/video_save.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <optional>
#include <string>
#include <utility>
#ifdef USE_RERUN
    #include "_rerun/recorder.hpp"
    #include "_rerun/tf.hpp"
#endif
#ifdef USE_ROS2
    #include "_rcl/node.hpp"
    #include "_rcl/tf.hpp"
    #include "_rcl/visual/armor.hpp"
    #include "_rcl/visual/armor_target.hpp"
    // 发给导航的话题类型
    #include "auto_aim_interfaces/msg/target.hpp"
    #include "example_interfaces/msg/u_int8.hpp"
    #include "rclcpp/qos.hpp"
    #include "sensor_msgs/msg/joint_state.hpp"
    #include "sensor_msgs/msg/camera_info.hpp"
    #include "sensor_msgs/msg/image.hpp"
    #include <rclcpp/qos.hpp>
#endif
#include "backward-cpp/backward.hpp"
#ifdef USE_Daedalus
    #include "daedalus_interface/shm_client.hpp"
#endif
#include "param_deliver.h"
#include "runtime/config.hpp"
#include "tasks/auto_aim/armor_control/very_aimer.hpp"
#include "tasks/auto_aim/armor_detect/armor_detector.hpp"
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/auto_aim/armor_track/armor_tracker.hpp"
#include "tasks/auto_aim/auto_aim_fsm.hpp"
#include "tasks/auto_aim/type.hpp"
#include "tasks/base/common.hpp"
#include "tasks/base/packet_typedef_receive.hpp"
#include "tasks/base/packet_typedef_send.hpp"
#include "tasks/base/record.hpp"
#include "tasks/base/web.hpp"
#include "utils/buffer.hpp"
#include "utils/common/image.hpp"
#include "utils/common/type_common.hpp"
#include "utils/drivers/serial_driver.hpp"
#include "utils/logger.hpp"
#include "utils/runtime_tf.hpp"
#include "utils/scheduler/scheduler.hpp"
#include "utils/semaphore_guard.hpp"
#include "utils/signal_guard.hpp"
#include "utils/utils.hpp"
namespace backward {
static backward::SignalHandling sh;
}
using namespace awakening;
enum class Mode : int { AutoAim = 0, AutoBuff = 1 };
Mode str_to_Mode(std::string str) {
    str = utils::to_upper(str);
    if (str == "AUTO_AIM") {
        return Mode::AutoAim;
    } else if (str == "AUTO_BUFF") {
        return Mode::AutoBuff;
    } else {
        throw std::invalid_argument("Invalid mode string");
    }
}
enum class SimpleFrame : int { ODOM, GIMBAL_ODOM, GIMBAL, CAMERA, CAMERA_CV, SHOOT, N };

using SimpleRobotTF = utils::tf::RobotTF<SimpleFrame, static_cast<size_t>(SimpleFrame::N), false>;

std::string SimpleFrame_to_str(int frame) {
    constexpr const char* details[] = { "odom",   "gimbal_odom", "gimbal",
                                        "camera", "camera_cv",   "shoot" };
    return std::string(details[frame]);
}
std::string SimpleFrame_to_str(SimpleFrame frame) {
    return SimpleFrame_to_str(std::to_underlying(frame));
}
struct CameraTag {};
struct SerialTag {};
struct DetectTag {};
struct FrameTag {};

using CameraIO = IOPair<CameraTag, ImageFrame>;
using SerialIO = IOPair<SerialTag, std::vector<uint8_t>>;
using CommonFrameIo = IOPair<FrameTag, CommonFrame>;
struct Detection {
    std::optional<std::vector<auto_aim::Armors>> armors;
    std::optional<std::vector<auto_buff::RuneDetection>> rune;
    std::optional<TimePoint> detect_start;
};
using DetIo = IOPair<DetectTag, Detection>;
struct LogCtx {
    int camera_count = 0;
    int detect_count = 0;
    int track_count = 0;
    int solve_count = 0;
    int serial_count = 0;
    int serial_bad_frame = 0; // ASCII 帧解析/CRC 失败次数
    int found_count = 0;
    double latency_ms_total = 0.0;
    void reset() {
        *this = {};
    }
};
struct AutoExposureCfg {
    double target_brightness = 0.0;
    double step_gain = 0.0;
    double decay_step = 0.0;
    double tolerance = 0.0;
    double exposure_min = 0.0;
    double exposure_max = 0.0;
    double control_interval_ms = 0.0;

    explicit AutoExposureCfg(const YAML::Node& c):
        target_brightness(c["target_brightness"].as<double>()),
        step_gain(c["step_gain"].as<double>()),
        decay_step(c["decay_step"].as<double>()),
        tolerance(c["tolerance"].as<double>()),
        exposure_min(c["exposure_min"].as<double>()),
        exposure_max(c["exposure_max"].as<double>()),
        control_interval_ms(c["control_interval_ms"].as<double>()) {}
};
bool is_web_running() {
    static std::atomic<bool> cached { true };
    utils::dt_once(
        [&]() {
            const int ret = std::system("pgrep -x wust_vision_web > /dev/null 2>&1");
            cached = (ret == 0);
        },
        std::chrono::duration<double>(1.0)
    );
    bool running = cached.load();
#ifdef USE_RERUN
    running = running || rerun_visual::Recorder::instance().enabled();
#endif
    return running;
}
static constexpr auto RECORD_FOLDER_PATH_ARR = utils::concat(ROOT_DIR, "/record/auto_aim");
static constexpr std::string_view RECORD_FOLDER_PATH(RECORD_FOLDER_PATH_ARR.data());

int main(int argc, char** argv) {
    auto start_tp = Clock::now();
    print_banner();
    auto& signal = utils::SignalGuard::instance();
    logger::init(spdlog::level::trace);

    bool debug = false;
    std::string config_path;
    std::string robot_name;
    auto first_arg = utils::get_arg(1, argc, argv);
    if (first_arg) {
        robot_name = first_arg.value();
        config_path = get_robot_config_path(robot_name).value_or(robot_name);
    } else {
        return 1;
    }
    auto second_arg = utils::get_arg(2, argc, argv);
    if (second_arg) {
        debug = second_arg.value() == "true";
    }
    // 原生 imshow 窗口：默认在有显示环境(DISPLAY/WAYLAND_DISPLAY)时开启。
    //   AWAKENING_IMSHOW=1     强制开启
    //   AWAKENING_NO_IMSHOW=1  强制关闭
    const bool show_window = [] {
        if (std::getenv("AWAKENING_NO_IMSHOW") != nullptr) {
            return false;
        }
        if (std::getenv("AWAKENING_IMSHOW") != nullptr) {
            return true;
        }
        return std::getenv("DISPLAY") != nullptr
            || std::getenv("WAYLAND_DISPLAY") != nullptr;
    }();
    // 诊断开关：AWAKENING_DIAG_ARMOR=1 时每秒打印一次检测各阶段数量
    const bool diag_armor = std::getenv("AWAKENING_DIAG_ARMOR") != nullptr;
    // 诊断开关：AWAKENING_NO_ROI=1 时忽略目标 ROI、始终全图检测（A/B 验证 ROI 影响）
    const bool diag_no_roi = std::getenv("AWAKENING_NO_ROI") != nullptr;
    auto config = YAML::LoadFile(config_path);
    // 逐帧诊断记录：固定文件名 + trunc => 每次启动自瞄都会覆盖上一次的记录。
    //   键名用 frame_log —— 不能用 record：那个键是"录屏(video)"，
    //   重名会被 YAML 覆盖、把录屏一起打开（之前那几十~几百 MB 的 avi 就是这么来的）。
    {
        const bool record_enable = config["frame_log"] && config["frame_log"]["enable"]
            ? config["frame_log"]["enable"].as<bool>()
            : false;
        const auto record_path = std::string(RECORD_FOLDER_PATH) + "/run.jsonl";
        record::run_recorder().open(record_path, record_enable);
        if (record::run_recorder().enabled()) {
            AWAKENING_INFO(
                "record enabled: {} (每次启动自瞄都会覆盖这个文件)", record::run_recorder().path()
            );
        }
    }
    Scheduler s;
    std::atomic<Mode> mode = str_to_Mode(config["mode"].as<std::string>());
    std::atomic<EnemyColor> enemy_color =
        enemy_color_from_string(config["enemy_color"].as<std::string>());
    std::atomic<double> bullet_speed = config["bullet_speed"].as<double>();
#ifdef USE_ROS2
    rcl::RclcppNode rcl_node("awakening");
    rcl::TF rcl_tf(rcl_node);
    // 视觉不再直接串口下发，改为发布话题、由导航统一下发到底层板。
    //   publish_to_nav: true 时发布下面三个话题
    // QoS 必须对：导航侧 cmd_gimbal_joint / cmd_shoot 用默认 reliable 订阅，
    //             tracker/target 用 SensorDataQoS 订阅（best-effort）。
    const bool publish_to_nav =
        config["publish_to_nav"] ? config["publish_to_nav"].as<bool>() : false;
    // cmd_gimbal_joint 的语义：
    //   nav_gimbal_mode: delta（增量，下位机自己叠加）/ absolute（世界系绝对角）
    //   nav_gimbal_units: delta 模式下的单位 rad / deg（absolute 恒为 rad）
    const bool nav_gimbal_delta = [&] {
        const std::string mode =
            config["nav_gimbal_mode"] ? config["nav_gimbal_mode"].as<std::string>() : "delta";
        return mode != "absolute";
    }();
    const bool nav_gimbal_units_deg = [&] {
        const std::string units =
            config["nav_gimbal_units"] ? config["nav_gimbal_units"].as<std::string>() : "rad";
        return units == "deg";
    }();
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_cmd_pub;
    rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr shoot_cmd_pub;
    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub;
    if (publish_to_nav) {
        gimbal_cmd_pub = rcl_node.make_pub<sensor_msgs::msg::JointState>(
            "cmd_gimbal_joint", rclcpp::QoS(rclcpp::KeepLast(10))
        );
        shoot_cmd_pub = rcl_node.make_pub<example_interfaces::msg::UInt8>(
            "cmd_shoot", rclcpp::QoS(rclcpp::KeepLast(10))
        );
        target_pub = rcl_node.make_pub<auto_aim_interfaces::msg::Target>(
            "tracker/target", rclcpp::SensorDataQoS()
        );
        AWAKENING_INFO(
            "publish_to_nav: cmd_gimbal_joint / cmd_shoot / tracker/target"
        );
    }
#endif
    bool use_daedalus = false;
#ifdef USE_Daedalus
    std::unique_ptr<talos::ipc::ShmClient> daedalus_shm_client;
    if (config["use_sim"].as<bool>()) {
        auto client = talos::ipc::ShmClient::connect();
        if (!client) {
            AWAKENING_ERROR("Failed to connect to talos::ipc::ShmClient");
            return 1;
        } else {
            use_daedalus = true;
            daedalus_shm_client = std::make_unique<talos::ipc::ShmClient>(std::move(*client));
        }
    }
#endif
    std::unique_ptr<SerialDriver> serial;
    if (!use_daedalus && config["serial"]["enable"].as<bool>()) {
        serial = std::make_unique<SerialDriver>(config["serial"], s);
    }
    // 串口协议版本（两个方向独立）：0 = 原有二进制包；1 = ASCII 文本帧。缺省为 1。
    const int send_protocol =
        config["serial"]["send_protocol"] ? config["serial"]["send_protocol"].as<int>() : 1;
    const int recv_protocol =
        config["serial"]["recv_protocol"] ? config["serial"]["recv_protocol"].as<int>() : 1;
    const bool ascii_send = (send_protocol == 1);
    const bool ascii_recv = (recv_protocol == 1);
    // ASCII 帧的 CRC 算法：
    //   mcrf4xx（默认）= 固件实际使用的 CRC16-MCRF4XX（初值 0xFFFF、反射多项式 0x8408）
    //   modbus         = 早期文档的 CRC16-Modbus（初值 0xFFFF、反射多项式 0xA001）
    // 接收方向两种都认（见 ascii_serial::parse_recv_frame），这里只影响**发送**。
    const ascii_serial::CrcMode ascii_crc_mode = [&] {
        const std::string mode = config["serial"]["crc_mode"]
            ? config["serial"]["crc_mode"].as<std::string>()
            : std::string("mcrf4xx");
        return mode == "modbus" ? ascii_serial::CrcMode::Modbus
                                : ascii_serial::CrcMode::Mcrf4xx;
    }();
    // 是否允许视觉**直接**通过串口下发指令（AAP 增量帧）。
    //   true（默认）：和原来一样，收到一包回一包；
    //   false       ：只收下位机的 IMU，指令仍走 ROS/导航（避免和导航抢云台）。
    const bool serial_send_enable =
        config["serial"]["send_enable"] ? config["serial"]["send_enable"].as<bool>() : true;
    ascii_serial::RecvFrameScanner ascii_recv_scanner;

    auto camera_config = config["camera"];
    std::unique_ptr<Camera> camera;
    utils::SignalGuard::add_callback([&]() {
        if (camera) {
            camera->stop();
        }
    });
    if (!use_daedalus) {
        camera = create_camera(camera_config, s, "daheng");
        camera->init();
        if (!camera->is_running()) {
            return 0;
        }
    }
    std::unique_ptr<VideoSaver> video_saver;
    if (config["record"]["enable"].as<bool>()) {
        video_saver = std::make_unique<VideoSaver>(
            VideoSaver::generate_record_filename(RECORD_FOLDER_PATH.data()),
            VideoSaver::Mode::NonBlocking
        );
    }
    CameraInfo camera_info;
    camera_info.load(camera_config["camera_info"]);
    auto_aim::ArmorDetector armor_detector(config["armor_detector"]);
    auto_aim::ArmorTracker armor_tracker(config["armor_tracker"]);
    auto_aim::AutoAimFsmController auto_aim_fsm_controller(config["auto_aim_fsm"]);
    auto_aim::VeryAimer very_aimer(config["very_aimer"]);
    auto_buff::RuneDetector rune_detector(config["rune_detector"]);
    auto_buff::RuneTracker rune_tracker(config["rune_tracker"]);
    auto_buff::RuneAimer rune_aimer(config["rune_aimer"]);
    utils::OrderedQueue<auto_aim::Armors> armors_queue;
    utils::OrderedQueue<auto_buff::RuneDetection> rune_detections_queue;
    // 队列排序用的 id 只给「真正跑过推理的帧」自增。
    // 被推理背压（max_infer_num）跳过的帧直接丢弃、不入队，
    // 否则跟踪器会把「没跑推理」误判成「这一帧没找到目标」。
    std::atomic<uint64_t> armor_detect_seq { 0 };
    std::atomic<uint64_t> rune_detect_seq { 0 };
    // 串口接收序号：每成功解析一包下位机数据 +1。
    // solver 只在它变化后才回发一包（收到一次 → 用最新解算结果发一次）。
    std::atomic<uint32_t> serial_rx_seq { 0 };
    utils::SWMR<auto_aim::ArmorTarget> armor_target;
    utils::SWMR<auto_buff::RuneTarget> rune_target;
    BulletPickUp bullet_pick_up(config["bullet_pick_up"]);
    LogCtx log_ctx;
    std::optional<VisionDebugCtx> dbg;
    // 最近一次解算出的云台指令快照（只给逐帧记录用）
    utils::Locked<GimbalCmd> record_cmd;
    // 最近一次检测用的 ROI（只给逐帧记录用）
    utils::Locked<cv::Rect> record_roi;
    // 最近一包 IMU 的原始快照（只给逐帧记录用；用来判定 IMU 本身是否有问题）
    utils::Locked<record::ImuSample> record_imu;
    std::pair<double, double> operator_offset = std::make_pair(0, 0);
    // imshow 由独立 GUI 线程执行：scheduler 回调跑在 TBB 线程池上，直接操作 GTK 不安全
    std::mutex show_mutex;
    cv::Mat show_img;
    std::atomic<bool> show_running { false };
    std::atomic<uint64_t> camera_frame_count { 0 };
    std::atomic<uint64_t> detect_frame_count { 0 };
    std::thread show_thread;
    if (debug || show_window) {
        dbg.emplace();
        dbg->camera_info_ = camera_info;
    }
    WheelOdometry wheel_odometry(config["wheel_odometry"], Clock::now());
    auto tf = SimpleRobotTF::create();
    {
        for (auto [from, to]: std::array {
                 std::pair { SimpleFrame::ODOM, SimpleFrame::GIMBAL_ODOM },
                 std::pair { SimpleFrame::GIMBAL_ODOM, SimpleFrame::GIMBAL },
                 std::pair { SimpleFrame::GIMBAL, SimpleFrame::CAMERA },
                 std::pair { SimpleFrame::GIMBAL, SimpleFrame::SHOOT },
                 std::pair { SimpleFrame::CAMERA, SimpleFrame::CAMERA_CV },
             })
        {
            tf->add_edge(from, to);
        }
        ISO3 cv_in_camera = ISO3::Identity();
        cv_in_camera.linear() = R_CV2PHYSICS;
        tf->push(SimpleFrame::CAMERA, SimpleFrame::CAMERA_CV, Clock::now(), cv_in_camera);
        tf->push(
            SimpleFrame::GIMBAL,
            SimpleFrame::CAMERA,
            Clock::now(),
            utils::load_isometry3(config["tf"]["camera_in_gimbal"])
        );
        tf->push(
            SimpleFrame::GIMBAL,
            SimpleFrame::SHOOT,
            Clock::now(),
            utils::load_isometry3(config["tf"]["shoot_in_gimbal"])
        );
    }
    auto serial_send_to_image_microseconds = config["serial_send_to_image_microseconds"].as<int>();
#ifdef USE_Daedalus
    if (daedalus_shm_client) {
        auto daedalus_imgs = s.register_source<CameraIO>("daedalus_img");

        auto push_pose = [&](talos::ipc::PoseIndex idx, SimpleFrame parent, SimpleFrame child) {
            if (auto pose = daedalus_shm_client->recv_pose(idx)) {
                ISO3 T = ISO3::Identity();
                T.translation() << pose->x, pose->y, pose->z;
                T.linear() =
                    Quaternion { pose->qw, pose->qx, pose->qy, pose->qz }.toRotationMatrix();

                tf->push(parent, child, TimePoint(std::chrono::nanoseconds(pose->timestamp_ns)), T);
            }
        };

        s.add_rate_source<>("daedalus_tick", 300.0, [&]() {
            static bool has_camera_info = false;

            if (!has_camera_info) {
                auto info = daedalus_shm_client->camera_info();
                has_camera_info = true;
                camera_info.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
                camera_info.camera_matrix.at<double>(0, 0) = info.fx;
                camera_info.camera_matrix.at<double>(1, 1) = info.fy;
                camera_info.camera_matrix.at<double>(0, 2) = info.cx;
                camera_info.camera_matrix.at<double>(1, 2) = info.cy;
                camera_info.distortion_coefficients = cv::Mat(1, 5, CV_64F);
                std::memcpy(
                    camera_info.distortion_coefficients.ptr<double>(),
                    info.distortion,
                    5 * sizeof(double)
                );
            }

            if (auto frame = daedalus_shm_client->recv_image()) {
                ImageFrame img_frame {
                    .src_img = std::move(frame->image),
                    .format = PixelFormat::RGB,
                    .timestamp = TimePoint(std::chrono::nanoseconds(frame->timestamp_ns)),
                };
                s.runtime_push_source<CameraIO>(
                    daedalus_imgs,
                    [f = std::move(img_frame)]() mutable {
                        return std::make_tuple(std::optional<CameraIO::second_type>(std::move(f)));
                    }
                );
            }
            if (auto ground_truth = daedalus_shm_client->recv_ground_truth()) {
                if (dbg) {
                    dbg->ground_truth.set(ground_truth.value());
                }
            }

            push_pose(talos::ipc::PoseIndex::POSE_CAMERA, SimpleFrame::GIMBAL, SimpleFrame::CAMERA);
            push_pose(talos::ipc::PoseIndex::POSE_MUZZLE, SimpleFrame::GIMBAL, SimpleFrame::SHOOT);
            push_pose(
                talos::ipc::PoseIndex::POSE_GIMBAL,
                SimpleFrame::GIMBAL_ODOM,
                SimpleFrame::GIMBAL
            );
        });
    }
#endif
    if (video_saver) {
        s.register_task<CameraIO>("save_video", [&](CameraIO::second_type&& f) {
            if (!f.src_img.empty()) {
                video_saver->write_frame(f.src_img);
            }
            return std::make_tuple(std::optional<CameraIO::second_type>(std::nullopt));
        });
    }
    s.register_task<CameraIO, CommonFrameIo>("push_common_frame", [&](CameraIO::second_type&& f) {
        static int current_id = 0;
        if (f.src_img.empty()) {
            return std::make_tuple(std::optional<CommonFrameIo::second_type>(std::nullopt));
        }
        log_ctx.camera_count++;
        camera_frame_count.fetch_add(1, std::memory_order_relaxed);
        CommonFrame frame {
            .img_frame = std::move(f),
            .id = current_id++,
            .frame_id = std::to_underlying(SimpleFrame::CAMERA_CV),
        };

        return std::make_tuple(std::optional<CommonFrameIo::second_type>(std::move(frame)));
    });

    if (serial) {
        s.register_task<SerialIO>("receive_serial", [&](SerialIO::second_type&& data) {
            static std::mutex mutex;
            std::lock_guard<std::mutex> lock(mutex);
            auto now = Clock::now();

            if (ascii_recv) {
                // ASCII 帧（serial.recv_protocol = 1）：ARQ<w>X<x>Y<y>Z<z>C<4位hex>%
                // 只带四元数，所以二进制包里的时间戳/角速度/车体速度/弹速/弹数/
                // 颜色/操作手偏移都无从获取 —— 这里保持原值不动，延迟估计
                // (time_stamp_*) 与发弹拾取 (bullet_count) 在 recv_protocol=1 下不生效。
                //
                // 用 RecvFrameScanner 而不是直接把整块丢给 parse_recv_frame：
                // 下位机常在同一个串口混发其它协议的二进制包（实测帧间夹着 70 字节
                // 的 0x5A 二进制包），而这里拿到的是任意长度的读块 —— 必须自己
                // 搜 "ARQ" 起点、按 '%'/'\n' 切帧、跨读块拼接。
                ascii_recv_scanner.feed(data.data(), data.size(), [&](std::vector<uint8_t>&& frame) {
                    auto q_opt = ascii_serial::parse_recv_frame(frame);
                    if (!q_opt.has_value()) {
                        log_ctx.serial_bad_frame++;
                        return;
                    }

                    serial_rx_seq.fetch_add(1, std::memory_order_relaxed);
                    log_ctx.serial_count++; // ASCII 路径按"解析成功的帧数"计
                    auto packet_time =
                        now - std::chrono::microseconds(serial_send_to_image_microseconds);
                    // IMU 健康快照：原始四元数 + 与上一包的姿态差/角速度（逐帧记录会用）
                    {
                        static Eigen::Quaterniond last_q = Eigen::Quaterniond::Identity();
                        static TimePoint last_q_t {};
                        static bool has_last_q = false;
                        record::ImuSample s;
                        s.wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        now.time_since_epoch()
                        )
                                        .count();
                        s.packet_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          packet_time.time_since_epoch()
                        )
                                          .count();
                        s.w = q_opt->w();
                        s.x = q_opt->x();
                        s.y = q_opt->y();
                        s.z = q_opt->z();
                        if (has_last_q) {
                            const double dt =
                                std::chrono::duration<double>(packet_time - last_q_t).count();
                            if (dt > 1e-6) {
                                const double dot =
                                    std::clamp(std::abs(last_q.dot(*q_opt)), 0.0, 1.0);
                                s.dq_deg = angles::to_degrees(2.0 * std::acos(dot));
                                s.rate_dps = s.dq_deg / dt;
                            }
                        }
                        last_q = *q_opt;
                        last_q_t = packet_time;
                        has_last_q = true;
                        s.seq = serial_rx_seq.load(std::memory_order_relaxed);
                        s.bad = static_cast<uint64_t>(log_ctx.serial_bad_frame);
                        s.protocol = 1;
                        record_imu.set(s);
                    }
                    ISO3 gimbal_in_gimbal_odom = ISO3::Identity();
                    gimbal_in_gimbal_odom.linear() = q_opt->toRotationMatrix();
                    tf->push(
                        SimpleFrame::GIMBAL_ODOM,
                        SimpleFrame::GIMBAL,
                        packet_time,
                        gimbal_in_gimbal_odom
                    );
                    Web::write_log("robo", [&](auto& j) {
                        j["protocol"] = "ascii";
                        j["quat_w"] = Web::val(q_opt->w());
                        j["quat_x"] = Web::val(q_opt->x());
                        j["quat_y"] = Web::val(q_opt->y());
                        j["quat_z"] = Web::val(q_opt->z());
                    });
                });
                return;
            }

            if (auto robo_opt = ReceiveRobotData::create(data); robo_opt.has_value()) {
                auto robo = robo_opt.value();
                serial_rx_seq.fetch_add(1, std::memory_order_relaxed);
                log_ctx.serial_count++; // 二进制路径：一个读块 = 一包
                static uint32_t last_pc = -1, delay = 0, last_bullet_count = 0;
                if (robo.time_stamp_pc != last_pc) {
                    last_pc = robo.time_stamp_pc;
                    delay = (std::chrono::duration_cast<std::chrono::microseconds>(now - start_tp)
                                 .count()
                             - robo.time_stamp_pc
                             - (robo.time_stamp_send_micro - robo.time_stamp_receive_micro))
                        / 2.0;
                }

                auto packet_time =
                    now - std::chrono::microseconds(serial_send_to_image_microseconds);
                ISO3 gimbal_in_gimbal_odom = ISO3::Identity();
                gimbal_in_gimbal_odom.linear() = utils::rpy2matrix(
                    Vec3(
                        angles::from_degrees(robo.roll),
                        angles::from_degrees(robo.pitch),
                        angles::from_degrees(robo.yaw)
                    ),
                    utils::RPYOrder::ZYX
                );
                // IMU 健康快照（二进制路径：四元数由 roll/pitch/yaw 换算）
                {
                    static Eigen::Quaterniond last_q = Eigen::Quaterniond::Identity();
                    static TimePoint last_q_t {};
                    static bool has_last_q = false;
                    const Eigen::Quaterniond q(gimbal_in_gimbal_odom.linear());
                    record::ImuSample s;
                    s.wall_ns =
                        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch())
                            .count();
                    s.packet_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      packet_time.time_since_epoch()
                    )
                                      .count();
                    s.w = q.w();
                    s.x = q.x();
                    s.y = q.y();
                    s.z = q.z();
                    if (has_last_q) {
                        const double dt =
                            std::chrono::duration<double>(packet_time - last_q_t).count();
                        if (dt > 1e-6) {
                            const double dot = std::clamp(std::abs(last_q.dot(q)), 0.0, 1.0);
                            s.dq_deg = angles::to_degrees(2.0 * std::acos(dot));
                            s.rate_dps = s.dq_deg / dt;
                        }
                    }
                    last_q = q;
                    last_q_t = packet_time;
                    has_last_q = true;
                    s.seq = serial_rx_seq.load(std::memory_order_relaxed);
                    s.bad = static_cast<uint64_t>(log_ctx.serial_bad_frame);
                    s.protocol = 0;
                    record_imu.set(s);
                }
                tf->push(
                    SimpleFrame::GIMBAL_ODOM,
                    SimpleFrame::GIMBAL,
                    packet_time,
                    gimbal_in_gimbal_odom
                );
                operator_offset = { angles::from_degrees(robo.operator_yaw_offset),
                                    angles::from_degrees(robo.operator_pitch_offset) };
                auto gimbal_odom_in_odom = ISO3::Identity();
                gimbal_odom_in_odom.translation() = Vec3(robo.v_x, robo.v_y, robo.v_z);
                // tf->push(
                //     SimpleFrame::ODOM,
                //     SimpleFrame::GIMBAL_ODOM,
                //     packet_time,
                //     gimbal_odom_in_odom
                // );
                enemy_color = EnemyColor(robo.detect_color);
                bullet_speed = robo.bullet_speed;
                robo.update_log(delay);
                if (robo.bullet_count > last_bullet_count) {
                    bullet_pick_up.push_back(Bullet {
                        .fire_time = Clock::now(),
                        .fire_time_shoot_in_odom =
                            tf->pose_a_in_b(SimpleFrame::SHOOT, SimpleFrame::ODOM, Clock::now()),
                        .speed_in_odom = bullet_speed,
                    });
                }
                last_bullet_count = robo.bullet_count;
            }
        });
    }
    if (camera) {
        s.register_task<CommonFrameIo>("auto_exposure", [&](CommonFrameIo::second_type&& frame) {
            static std::mutex mutex;
            std::lock_guard<std::mutex> lock(mutex);
            static std::optional<AutoExposureCfg> auto_exposure_cfg =
                config["auto_exposure"]["enable"].as<bool>()
                ? std::optional<AutoExposureCfg>(std::in_place, config["auto_exposure"])
                : std::nullopt;
            if (auto_exposure_cfg) { // 平均亮度pid
                auto& cfg = auto_exposure_cfg.value();
                utils::dt_once(
                    [&]() {
                        cv::Mat gray;
                        cv::cvtColor(frame.img_frame.src_img, gray, cv::COLOR_BGR2GRAY);
                        double exposure_time = camera->get_ExposureTime();
                        static double last_exposure_time = 0.0;
                        const double diff = cv::mean(gray)[0] - cfg.target_brightness;
                        if (std::fabs(diff) > cfg.tolerance && exposure_time > 0.0) {
                            exposure_time -= diff * cfg.step_gain;
                        } else {
                            exposure_time -= cfg.decay_step;
                        }
                        exposure_time =
                            std::clamp(exposure_time, cfg.exposure_min, cfg.exposure_max);
                        if (std::abs(exposure_time - last_exposure_time) > 10) {
                            camera->set_ExposureTime(exposure_time);
                            last_exposure_time = exposure_time;
                        }
                    },
                    std::chrono::milliseconds((int)cfg.control_interval_ms)
                );
            }
        });
    }

    s.register_task<CommonFrameIo, DetIo>("detector", [&](CommonFrameIo::second_type&& frame) {
        Detection r;
        cv::Rect net_focus =
            cv::Rect(0, 0, frame.img_frame.src_img.cols, frame.img_frame.src_img.rows);
        if (mode == Mode::AutoAim) {
            static std::unique_ptr<std::counting_semaphore<>> detector_sem;
            if (!detector_sem) {
                detector_sem = std::make_unique<std::counting_semaphore<>>(
                    config["armor_detector"]["max_infer_num"].as<int>()
                );
            }
            std::optional<cv::Rect> detect_light = std::nullopt;
            auto target = armor_target.read();
            if (!diag_no_roi && target.need_focus()) {
                auto camera_cv_in_old = tf->pose_a_in_b(
                    SimpleFrame(frame.frame_id),
                    SimpleFrame(target.get_target_state().frame_id),
                    frame.img_frame.timestamp
                );
                net_focus = target.get_net_focus_roi(
                    frame.img_frame.timestamp,
                    camera_cv_in_old,
                    camera_info,
                    frame.img_frame.src_img.size(),
                    armor_detector.get_net_wh_ratio()
                );
                if (target.need_detect_lights()) {
                    detect_light = target.expanded( // 送给传统越小越好
                        frame.img_frame.timestamp,
                        camera_cv_in_old,
                        camera_info,
                        frame.img_frame.src_img.size()
                    );
                    detect_light = utils::expand_and_clip_rect(
                        detect_light.value(),
                        1.6,
                        frame.img_frame.src_img.size()
                    );
                }
            }

            auto_aim::Armors armors { .timestamp = frame.img_frame.timestamp,
                                      .id = 0,
                                      .frame_id = frame.frame_id };
            bool inferred = false;
            {
                inferred = detector_sem->try_acquire();
                utils::SemaphoreGuard guard(*detector_sem, inferred); //并发控制
                if (inferred) {
                    auto start = Clock::now();
                    r.detect_start = start;
                    record_roi.set(net_focus);
                    auto [ls, as] = armor_detector.detect(frame, net_focus, detect_light);
                    armors.armors = as;
                    armors.lights = ls;
                    log_ctx.detect_count++;
                    detect_frame_count.fetch_add(1, std::memory_order_relaxed);

                    // ---- 诊断：每秒统计检测各阶段数量（AWAKENING_DIAG_ARMOR=1）
                    if (diag_armor) {
                        static std::mutex diag_mtx;
                        static std::map<std::string, int> diag_cnt;
                        static int st_inf = 0;
                        static int st_net_empty = 0, st_over_conf = 0, st_kept = 0, st_zero = 0;
                        static int st_extract_fail = 0, st_out = 0;
                        static auto diag_t0 = Clock::now();
                        std::lock_guard<std::mutex> lk(diag_mtx);
                        ++st_inf;
                        for (const auto& a: as) {
                            diag_cnt[string_by_armor_color(a.color) + "_"
                                     + string_by_armor_class(a.number)]++;
                        }
                        const auto st = armor_detector.take_detect_stats();
                        st_net_empty += st.net_empty;
                        st_over_conf += st.over_conf;
                        st_kept += st.kept;
                        if (st.kept == 0) {
                            ++st_zero;
                        }
                        st_extract_fail += st.extract_fail;
                        st_out += st.out;
                        const auto now = Clock::now();
                        if (now - diag_t0 >= std::chrono::seconds(1)) {
                            std::string s;
                            for (const auto& [k, v]: diag_cnt) {
                                s += fmt::format("{}({}) ", k, v);
                            }
                            AWAKENING_INFO(
                                "[DIAG] armor/s: {}| inf={} net_empty={} over_conf={} kept={} "
                                "zero={} extract_fail={} out={}",
                                s.empty() ? std::string("(none) ") : s,
                                st_inf,
                                st_net_empty,
                                st_over_conf,
                                st_kept,
                                st_zero,
                                st_extract_fail,
                                st_out
                            );
                            diag_cnt.clear();
                            st_inf = st_net_empty = st_over_conf = st_kept = st_zero = 0;
                            st_extract_fail = st_out = 0;
                            diag_t0 = now;
                        }
                    }
                    // ---- 诊断结束
                }
            }
            if (inferred) {
                // 只有推理过的帧才入队；id 用检测序号，保证队列 id 连续不卡
                armors.id = static_cast<int>(armor_detect_seq.fetch_add(1) + 1);
                armors_queue.enqueue(armors);
            }
            auto batch_armors = armors_queue.dequeue_batch(); // 根据id有序输出
            r.armors = batch_armors;
        } else {
            static std::unique_ptr<std::counting_semaphore<>> detector_sem;
            if (!detector_sem) {
                detector_sem = std::make_unique<std::counting_semaphore<>>(
                    config["rune_detector"]["max_infer_num"].as<int>()
                );
            }
            auto target = rune_target.read();
            if (target.need_focus()) {
                auto camera_cv_in_old = tf->pose_a_in_b(
                    SimpleFrame(frame.frame_id),
                    SimpleFrame(target.get_target_state().frame_id),
                    frame.img_frame.timestamp
                );
                net_focus = target.get_net_focus_roi(
                    frame.img_frame.timestamp,
                    camera_cv_in_old,
                    camera_info,
                    frame.img_frame.src_img.size(),
                    armor_detector.get_net_wh_ratio()
                );
            }
            auto_buff::RuneDetection rune_detection;
            bool inferred = false;
            {
                inferred = detector_sem->try_acquire();
                utils::SemaphoreGuard guard(*detector_sem, inferred); //并发控制
                if (inferred) {
                    auto start = Clock::now();
                    r.detect_start = start;
                    rune_detection = rune_detector.detect(frame, net_focus, enemy_color);
                    log_ctx.detect_count++;
                    detect_frame_count.fetch_add(1, std::memory_order_relaxed);
                }
            }
            if (inferred) {
                // 同 auto_aim：只有推理过的帧才入队，id 用检测序号
                rune_detection.frame_id = frame.frame_id;
                rune_detection.id = static_cast<int>(rune_detect_seq.fetch_add(1) + 1);
                rune_detection.timestamp = frame.img_frame.timestamp;
                rune_detections_queue.enqueue(rune_detection);
            }
            auto batch_runes = rune_detections_queue.dequeue_batch();
            r.rune = batch_runes;
        }

        if (dbg && (is_web_running() || show_window)) {
            dbg->expanded.set(net_focus);
            dbg->img_frame.set(std::move(frame.img_frame));
        }

        return std::make_tuple(std::optional<DetIo::second_type>(std::move(r)));
    });

    s.register_task<DetIo>("tracker", [&](DetIo::second_type&& io) {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);
        if (io.armors) {
            for (const auto& armors_raw: io.armors.value()) {
                auto armors = armors_raw;
                auto is_ally = [&](const auto& obj) {
                    return (enemy_color == EnemyColor::BLUE
                            && obj.color == auto_aim::ArmorColor::RED)
                        || (enemy_color == EnemyColor::RED
                            && obj.color == auto_aim::ArmorColor::BLUE);
                };
                armors.armors.erase(
                    std::remove_if(armors.armors.begin(), armors.armors.end(), is_ally),
                    armors.armors.end()
                );
                armors.lights.erase(
                    std::remove_if(armors.lights.begin(), armors.lights.end(), is_ally),
                    armors.lights.end()
                );
                auto camera_cv_in_odom = tf->pose_a_in_b(
                    SimpleFrame(armors.frame_id),
                    SimpleFrame::ODOM,
                    armors.timestamp
                ); //转到odom
                armors.frame_id = std::to_underlying(SimpleFrame::ODOM);
                auto __armor_target =
                    armor_tracker.track(armors, camera_info, camera_cv_in_odom, armors.frame_id);
                // 【丢观测时冻结输出状态】last_update 只在 init / 真正 EKF 更新时才变；
                // 它没变 = 这一帧没有可用观测（板子没检出/没匹配上）。原来的 EKF 会按
                // 上一次的速度继续外推：实测丢 1.9 s 就把目标中心从 2.5 m 推到 9.9 m，
                // 预测的装甲板"飞走"（pred 与实测差 898 px），ROI 也跟着飞 → 越丢越抓不回来。
                // 这里只对**输出**刹车：车心钉在最后一次观测处、平移速度清零（转速保留，
                // 装甲板仍绕冻结的车心转）。瞄准/ROI/显示都走这个输出；跟踪器内部状态不动。
                bool record_braked = false;
                {
                    static TimePoint last_seen_update {};
                    static Vec3 last_seen_pos = Vec3::Zero();
                    // 判据：这一帧 EKF 有没有更新过（含**灯条观测**）。
                    //   试过"必须检测到装甲板才算有观测"的版本，结果灯条带来的真实位置
                    //   修正被丢掉，匹配率明显下降（found/detect 掉到 16~53%）、整体更差，
                    //   已回退。代价是"只有灯条更新、没有装甲板"的那几十帧仍可能外推偏掉。
                    const bool has_obs = __armor_target.last_update != last_seen_update;
                    if (has_obs) {
                        last_seen_update = __armor_target.last_update;
                        last_seen_pos = __armor_target.get_target_state().pos();
                    } else if (__armor_target.check()) {
                        __armor_target.set_target_state_keep_id([&](auto& s) {
                            namespace idx = auto_aim::armor_point_motion_model::idx;
                            s.x[idx::CX] = last_seen_pos.x();
                            s.x[idx::CY] = last_seen_pos.y();
                            s.x[idx::CZ] = last_seen_pos.z();
                            s.x[idx::VCX] = 0.0;
                            s.x[idx::VCY] = 0.0;
                            s.x[idx::VCZ] = 0.0;
                        });
                        record_braked = true;
                    }
                }
                auto_aim_fsm_controller.update(
                    __armor_target.get_target_state().vyaw(),
                    __armor_target.jumped,
                    __armor_target.get_target_state().timestamp
                );
                armor_target.write(__armor_target);
                // ---- 逐帧诊断记录（纯观察，不改逻辑）----
                //   固定文件 + 启动时 trunc => 每次开自瞄都覆盖上一次的记录。
                if (record::run_recorder().enabled()) {
                    using nlohmann::json;
                    json j = json::object();
                    j["wall_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       std::chrono::system_clock::now().time_since_epoch()
                    )
                                       .count();
                    j["t_ns"] = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    armors.timestamp.time_since_epoch()
                    )
                                    .count();
                    j["enemy"] = (enemy_color.load() == EnemyColor::RED) ? "red" : "blue";
                    j["brake"] = record_braked; // 这一帧是否因无观测被冻结（丢观测刹车）
                    {
                        const auto roi = record_roi.get();
                        j["roi"] = json::array({ roi.x, roi.y, roi.width, roi.height });
                    }

                    // 本帧检测到的每一块板（号码的两种来源 / 置信度 / 像素框 / PnP 位置）
                    json det = json::array();
                    for (auto& a: armors.armors) {
                        json d = json::object();
                        d["color"] = string_by_armor_color(a.color);
                        d["num"] = string_by_armor_class(a.number);
                        d["e2e"] =
                            a.net ? string_by_armor_class(a.net->number) : std::string("cv");
                        d["clf"] = a.number_classifier
                            ? string_by_armor_class(a.number_classifier->number)
                            : std::string("none");
                        d["conf"] = record::num(a.net ? a.net->confidence : 0.0);
                        const auto bbox = a.key_points.bounding_box();
                        d["xy"] = json::array(
                            { record::num((bbox.tl().x + bbox.br().x) * 0.5),
                              record::num((bbox.tl().y + bbox.br().y) * 0.5) }
                        );
                        d["wh"] =
                            json::array({ record::num(bbox.width), record::num(bbox.height) });
                        // PnP 出来的装甲板位置（odom 系）—— 手眼/姿态链判定就靠它。
                        //   用副本算，绝不影响跟踪器。
                        auto a_pnp = a;
                        const bool pnp_ok =
                            auto_aim::ArmorTarget::armor_pnp(a_pnp, camera_info, camera_cv_in_odom);
                        d["pnp_ok"] = pnp_ok;
                        if (pnp_ok) {
                            d["p_odom"] = json::array({ record::num(a_pnp.pose.translation().x()),
                                                        record::num(a_pnp.pose.translation().y()),
                                                        record::num(a_pnp.pose.translation().z()) });
                        }
                        d["p_raw"] = json::array({ record::num(a.pose.translation().x()),
                                                   record::num(a.pose.translation().y()),
                                                   record::num(a.pose.translation().z()) });
                        det.push_back(std::move(d));
                    }
                    j["det"] = std::move(det);

                    // 跟踪器状态
                    const auto st = __armor_target.get_target_state();
                    json trk = json::object();
                    trk["state"] = auto_aim::ArmorTarget::TrackState::string_by_state(
                        __armor_target.track_state.tracker_state
                    );
                    trk["num"] = string_by_armor_class(__armor_target.target_number);
                    trk["check"] = __armor_target.check();
                    trk["jumped"] = __armor_target.jumped;
                    trk["pos"] = json::array({ record::num(st.pos().x()),
                                               record::num(st.pos().y()),
                                               record::num(st.pos().z()) });
                    trk["vel"] = json::array({ record::num(st.vel().x()),
                                               record::num(st.vel().y()),
                                               record::num(st.vel().z()) });
                    trk["vyaw"] = record::num(st.vyaw());
                    trk["yaw"] = record::num(st.yaw());
                    trk["r1"] = record::num(st.r1());
                    trk["r2"] = record::num(st.r2());
                    trk["h"] = record::num(st.h());
                    // 完整 13 维 EKF 状态（NaN 会变成 null）——用来查"哪个维度坏了"
                    {
                        json xv = json::array();
                        for (Eigen::Index i = 0; i < st.x.size(); ++i) {
                            xv.push_back(record::num(st.x[i]));
                        }
                        trk["x"] = std::move(xv);
                    }
                    j["trk"] = std::move(trk);

                    // 预测装甲板：odom 位置 + 投影像素（用来对比"预测 vs 实测"）
                    // 只在真的有目标时记录，避免 LOST 帧写一堆无意义的默认值。
                    auto pred_state = st;
                    const bool has_target = __armor_target.check();
                    if (has_target) {
                        pred_state.predict(armors.timestamp, __armor_target.target_number);
                    }
                    const auto pred_poses = has_target
                        ? pred_state.get_armors_pose(__armor_target.target_number)
                        : std::vector<ISO3> {};
                    const auto odom_in_camera_cv = camera_cv_in_odom.inverse();
                    json pred = json::array();
                    for (const auto& pose: pred_poses) {
                        json e = json::object();
                        e["p"] = json::array({ record::num(pose.translation().x()),
                                               record::num(pose.translation().y()),
                                               record::num(pose.translation().z()) });
                        const auto pose_in_camera_cv = odom_in_camera_cv * pose;
                        if (pose_in_camera_cv.translation().z() > 0.1) {
                            const auto pts = utils::reprojection(
                                camera_info.camera_matrix,
                                camera_info.distortion_coefficients,
                                { cv::Point3f(0, 0, 0) },
                                pose_in_camera_cv
                            );
                            if (!pts.empty()) {
                                e["xy"] = json::array(
                                    { record::num(pts[0].x), record::num(pts[0].y) }
                                );
                            }
                        }
                        pred.push_back(std::move(e));
                    }
                    j["pred"] = std::move(pred);

                    // 图像时刻的云台姿态（IMU 回传，经 tf 插值/外推）
                    const auto gimbal_in_gimbal_odom = tf->pose_a_in_b(
                        SimpleFrame::GIMBAL,
                        SimpleFrame::GIMBAL_ODOM,
                        armors.timestamp
                    );
                    const auto rpy = utils::matrix2rpy<double>(
                        gimbal_in_gimbal_odom.linear(),
                        utils::RPYOrder::ZYX
                    );
                    json cam = json::object();
                    cam["yaw"] = record::num(angles::to_degrees(rpy[2]));
                    cam["pitch"] = record::num(-angles::to_degrees(rpy[1]));
                    cam["roll"] = record::num(angles::to_degrees(rpy[0]));
                    j["cam"] = std::move(cam);
                    // 同一时刻的"当前"云台姿态（瞄准用的那一个），和 cam（图像时刻，
                    // 经 tf 插值/外推）对比，能看出姿态链的滞后/跳变。
                    {
                        const auto g_now = tf->pose_a_in_b(
                            SimpleFrame::GIMBAL,
                            SimpleFrame::GIMBAL_ODOM,
                            Clock::now()
                        );
                        const auto rpy_now = utils::matrix2rpy<double>(
                            g_now.linear(),
                            utils::RPYOrder::ZYX
                        );
                        json cn = json::object();
                        cn["yaw"] = record::num(angles::to_degrees(rpy_now[2]));
                        cn["pitch"] = record::num(-angles::to_degrees(rpy_now[1]));
                        cn["roll"] = record::num(angles::to_degrees(rpy_now[0]));
                        j["cam_now"] = std::move(cn);
                    }

                    // IMU 原始快照：四元数 / |q| / 与上一包的姿态差和角速度 / 丢包计数
                    {
                        const auto s = record_imu.get();
                        json imu = json::object();
                        imu["wall_ms"] = s.wall_ns / 1000000;
                        imu["packet_ms"] = s.packet_ns / 1000000;
                        imu["age_ms"] = s.seq == 0
                            ? nlohmann::json(nullptr)
                            : record::num(
                                  std::chrono::duration<double, std::milli>(
                                      armors.timestamp
                                      - TimePoint(std::chrono::nanoseconds(s.packet_ns))
                                  )
                                      .count()
                              );
                        imu["w"] = record::num(s.w);
                        imu["x"] = record::num(s.x);
                        imu["y"] = record::num(s.y);
                        imu["z"] = record::num(s.z);
                        imu["norm"] = record::num(std::sqrt(
                            s.w * s.w + s.x * s.x + s.y * s.y + s.z * s.z
                        ));
                        imu["dq_deg"] = record::num(s.dq_deg);
                        imu["rate_dps"] = record::num(s.rate_dps);
                        imu["seq"] = s.seq;
                        imu["bad"] = s.bad;
                        imu["proto"] = s.protocol;
                        j["imu"] = std::move(imu);
                    }

                    // 最近一次解算出的云台指令（含与指令的时间差 age_ms）
                    const auto c = record_cmd.get();
                    json cj = json::object();
                    cj["appear"] = c.appear;
                    cj["fire"] = c.fire_advice;
                    cj["yaw"] = record::num(c.yaw);
                    cj["pitch"] = record::num(c.pitch);
                    cj["target_yaw"] = record::num(c.target_yaw);
                    cj["target_pitch"] = record::num(c.target_pitch);
                    cj["v_yaw"] = record::num(c.v_yaw);
                    cj["v_pitch"] = record::num(c.v_pitch);
                    cj["age_ms"] = record::num(
                        c.timestamp.time_since_epoch().count() == 0
                            ? 0.0
                            : std::chrono::duration<double, std::milli>(
                                  Clock::now() - c.timestamp
                              )
                                  .count()
                    );
                    cj["aim"] = json::array({ record::num(c.aim_point.pose.translation().x()),
                                              record::num(c.aim_point.pose.translation().y()),
                                              record::num(c.aim_point.pose.translation().z()) });
                    cj["aim_frame"] = c.aim_point.frame_id;
                    j["cmd"] = std::move(cj);

                    record::run_recorder().write(j);
                }
                if (io.detect_start) {
                    auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          Clock::now() - io.detect_start.value()
                    )
                                          .count();
                    log_ctx.latency_ms_total += latency_ms;
                }

                log_ctx.found_count += armor_tracker.get_count();
                armor_tracker.reset_count();
                if (dbg) {
                    dbg->armors.set(armors);
                }

                log_ctx.track_count++;
            }
        } else if (io.rune) {
            for (auto& rune: io.rune.value()) {
                auto camera_cv_in_odom = tf->pose_a_in_b(
                    SimpleFrame(rune.frame_id),
                    SimpleFrame::ODOM,
                    rune.timestamp
                ); //转到odom
                rune.frame_id = std::to_underlying(SimpleFrame::ODOM);

                auto __rune_target =
                    rune_tracker.track(rune, camera_info, camera_cv_in_odom, rune.frame_id);

                rune_target.write(__rune_target);
                if (io.detect_start) {
                    auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          Clock::now() - io.detect_start.value()
                    )
                                          .count();
                    log_ctx.latency_ms_total += latency_ms;
                }
                log_ctx.found_count += rune_tracker.get_count();
                rune_tracker.reset_count();

                if (dbg) {
                    dbg->rune.set(rune);
                }
                log_ctx.track_count++;
            }
        }
    });
    s.add_rate_source<>("solver", 1000.0, [&]() {
        log_ctx.solve_count++;
        auto a_target = armor_target.read(); //需要转为相对gimbal_odom
        int old_this_id = a_target.this_id;
        auto gimbal_odom_state_in_odom = wheel_odometry.state;
        gimbal_odom_state_in_odom.predict(Clock::now());
        a_target.set_target_state([&](auto& s) {
            namespace idx = auto_aim::armor_point_motion_model::idx;
            s.frame_id = std::to_underlying(SimpleFrame::GIMBAL_ODOM);
            const auto pos = gimbal_odom_state_in_odom.pos();
            const auto vel = gimbal_odom_state_in_odom.vel();
            s.x[idx::CX] -= pos.x();
            s.x[idx::CY] -= pos.y();
            s.x[idx::CZ] -= pos.z();
            s.x[idx::VCX] -= vel.x();
            s.x[idx::VCY] -= vel.y();
            s.x[idx::VCZ] -= vel.z();
            // s.x[idx::VCX] =0;
            // s.x[idx::VCY] =0;
            // s.x[idx::VCZ] =0;
            // s.x[idx::VYAW] =12.0;
        });
        a_target.this_id = old_this_id;
        auto r_target = rune_target.read();
        old_this_id = r_target.this_id;
        r_target.set_target_state([&](auto& s) {
            namespace idx = auto_buff::motion_model::idx;
            s.frame_id = std::to_underlying(SimpleFrame::GIMBAL_ODOM);
            const auto pos = gimbal_odom_state_in_odom.pos();
            const auto vel = gimbal_odom_state_in_odom.vel();
            s.x[idx::CX] -= pos.x();
            s.x[idx::CY] -= pos.y();
            s.x[idx::CZ] -= pos.z();
        });
        r_target.this_id = old_this_id;
        GimbalCmd cmd {
            .appear = false,
        };
        auto shoot_in_gimbal_odom =
            tf->pose_a_in_b(SimpleFrame::SHOOT, SimpleFrame::GIMBAL_ODOM, Clock::now());
        auto gimbal_in_gimbal_odom =
            tf->pose_a_in_b(SimpleFrame::GIMBAL, SimpleFrame::GIMBAL_ODOM, Clock::now());
        static ShootControlSend shoot_send;
        if (a_target.check()) {
            very_aimer.set_operator_offset(operator_offset);
            cmd = very_aimer.very_aim(
                a_target,
                bullet_speed,
                auto_aim_fsm_controller.get_state(),
                shoot_in_gimbal_odom,
                gimbal_in_gimbal_odom
            );

        } else if (r_target.check()) {
            rune_aimer.set_operator_offset(operator_offset);
            cmd =
                rune_aimer.aim(r_target, bullet_speed, shoot_in_gimbal_odom, gimbal_in_gimbal_odom);
            bool can_fire = rune_aimer.tick_fire(r_target, cmd);
            if (can_fire) {
                AWAKENING_INFO("you can fire!");
                cmd.fire_advice = true;
                shoot_send.shoot_count++;
            } else {
                cmd.fire_advice = false;
            }
        }

        // 发送节奏：由接收驱动 —— 每收到下位机一包数据，就用最新解算结果回一包。
        // 这既符合“计算完发一次”，也和 ASCII 的增量语义一致：
        //   send_pitch/send_yaw 是相对“当前云台角度”的增量，而当前角度来自最近收到的
        //   那一帧四元数；不等新包就发的话，用的是过期姿态，增量会算错。
        // 另外加最小发送间隔（≤500Hz），防止下位机突发多包时把串口写队列越堆越长。
        const uint32_t rx_seq = serial_rx_seq.load(std::memory_order_relaxed);
        static uint32_t sent_rx_seq = 0;
        static TimePoint last_send_tp {};
        const auto now_send = Clock::now();
        constexpr auto kMinSendInterval = std::chrono::milliseconds(2);
        const bool send_now =
            (rx_seq != sent_rx_seq) && (now_send - last_send_tp) >= kMinSendInterval;
        if (send_now) {
            sent_rx_seq = rx_seq;
            last_send_tp = now_send;
        }
        if (serial && serial_send_enable && send_now) {
          if (ascii_send) {
            // ASCII 帧（serial.send_protocol = 1）：
            //   AAP<pitch>Y<yaw>F<fire>N<frame>C<4位hex>%<LF>
            //
            // pitch / yaw 是**相对当前云台角度的增量（度）**，下位机在此基础上叠加，
            // 语义与 sp_vision 的 simple_serial 完全一致：
            //     send_pitch = cmd.pitch - cur_pitch_deg
            //     send_yaw   = wrap180(cmd.yaw - cur_yaw_deg)
            // 没有目标（cmd.appear == false）时增量为 0 且不允许开火，云台保持不动。
            static int ascii_frame_count = 0;

            const bool control = cmd.appear;
            // matrix2rpy(ZYX) 返回 {roll, pitch, yaw}；.linear() 是 Block 表达式，
            // 先落成 Matrix3d 才能匹配模板。
            const Eigen::Matrix3d cur_R = gimbal_in_gimbal_odom.linear();
            const Eigen::Vector3d cur_rpy = utils::matrix2rpy(cur_R, utils::RPYOrder::ZYX);
            const double cur_pitch_deg = angles::to_degrees(cur_rpy[1]);
            const double cur_yaw_deg = angles::to_degrees(cur_rpy[2]);

            const auto wrap180 = [](double d) {
                while (d > 180.0) d -= 360.0;
                while (d < -180.0) d += 360.0;
                return d;
            };

            const double send_pitch = control ? (cmd.pitch - cur_pitch_deg) : 0.0;
            const double send_yaw = control ? wrap180(cmd.yaw - cur_yaw_deg) : 0.0;

            // 与 sp_vision 一致：增量总误差小于 1.0° 才允许开火
            constexpr double AIM_PRECISION_THRESH = 1.0;
            const double aim_error = std::hypot(send_pitch, send_yaw);
            const int fire =
                (control && cmd.fire_advice && aim_error < AIM_PRECISION_THRESH) ? 1 : 0;

            auto frame = ascii_serial::build_send_frame(
                send_pitch, send_yaw, fire, ascii_frame_count++, ascii_crc_mode
            );
            if (!frame.empty()) {
                std::vector<uint8_t> bytes(frame.begin(), frame.end());
                serial->write(std::move(bytes));
            }
          } else {
            SendRobotCmdData send;
            send.cmd_ID = SendRobotCmdData::ID;
            send.time_stamp =
                std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start_tp)
                    .count();
            send.appear = cmd.appear, send.detect_color = std::to_underlying(enemy_color.load());
            send.yaw = cmd.yaw, send.pitch = cmd.pitch, send.v_yaw = cmd.v_yaw;
            send.target_yaw = cmd.target_yaw, send.target_pitch = cmd.target_pitch;
            send.v_pitch = cmd.v_pitch, send.a_yaw = cmd.a_yaw, send.a_pitch = cmd.a_pitch;
            send.enable_yaw_diff = cmd.enable_yaw_diff;
            send.enable_pitch_diff = cmd.enable_pitch_diff;
            if (utils::is_finite(send)) {
                serial->write(std::move(utils::to_vector(send)));
            }
            if (utils::is_finite(shoot_send)) {
                serial->write(std::move(utils::to_vector(shoot_send)));
            }
          }
        }
#ifdef USE_ROS2
        // 视觉 -> 导航（由导航统一下发到底层板，视觉不再直接串口下发）
        //   cmd_gimbal_joint : 云台角，语义由 nav_gimbal_mode 决定
        //                      delta    = 相对当前云台角的增量，丢目标发 0/0（"三零"=丢失）
        //                      absolute = 世界系绝对角(rad)，丢目标不发（由导航锁存）
        //   cmd_shoot        : 0 停火 / >0 开火，与云台同频；停火必须显式发 0
        //   tracker/target   : 只填 tracking，低频 ~15Hz；放开控制要显式发 false
        if (publish_to_nav) {
            // 用 detect_frame_count（只在真跑过推理的帧自增），
            // 不用 camera_frame_count —— 后者会把被背压跳过的帧也算进来。
            const uint64_t frame_now = detect_frame_count.load(std::memory_order_relaxed);
            static uint64_t last_pub_frame = std::numeric_limits<uint64_t>::max();
            if (frame_now != last_pub_frame) {
                last_pub_frame = frame_now;

                // 当前云台角：来自下位机回传的四元数（GIMBAL -> GIMBAL_ODOM）。
                // 没有 IMU 时 tf 返回单位阵，"当前角" = 0，此时算出来的增量恰好等于
                // "目标相对相机的角度"，短时间可用，但基准会漂。
                const Eigen::Matrix3d cur_R = gimbal_in_gimbal_odom.linear();
                const Eigen::Vector3d cur_rpy = utils::matrix2rpy(cur_R, utils::RPYOrder::ZYX);
                const double cur_pitch_deg = angles::to_degrees(cur_rpy[1]);
                const double cur_yaw_deg = angles::to_degrees(cur_rpy[2]);
                const auto wrap180 = [](double d) {
                    while (d > 180.0) d -= 360.0;
                    while (d < -180.0) d += 360.0;
                    return d;
                };
                // 增量模式的单位可配（rad / deg）；绝对角恒为 rad
                const auto to_nav_units = [&](double deg) {
                    return nav_gimbal_units_deg ? deg : angles::from_degrees(deg);
                };

                double pitch_cmd = 0.0;
                double yaw_cmd = 0.0;
                bool send_gimbal = false;
                if (cmd.appear) {
                    if (nav_gimbal_delta) {
                        pitch_cmd = to_nav_units(cmd.pitch - cur_pitch_deg);
                        yaw_cmd = to_nav_units(wrap180(cmd.yaw - cur_yaw_deg));
                    } else {
                        pitch_cmd = angles::from_degrees(cmd.pitch);
                        yaw_cmd = angles::from_degrees(cmd.yaw);
                    }
                    // 解算异常时宁可这一帧不发，也不要发 NaN 出去
                    send_gimbal = std::isfinite(pitch_cmd) && std::isfinite(yaw_cmd);
                } else if (nav_gimbal_delta) {
                    // 丢目标：显式发 0/0。增量 0 = 别动，同时让下位机能用
                    // "fire/pitch/yaw 三零"判断丢失目标。
                    pitch_cmd = 0.0;
                    yaw_cmd = 0.0;
                    send_gimbal = true;
                }
                // absolute 模式丢目标时**不发云台角**：发 0/0 会被当成"世界系 yaw=0"，
                // 云台会甩到正前方；导航侧会锁存上一条指令。

                if (send_gimbal) {
                    sensor_msgs::msg::JointState gimbal_msg;
                    gimbal_msg.name = { "gimbal_pitch_joint", "gimbal_yaw_joint" };
                    gimbal_msg.position = { pitch_cmd, yaw_cmd };
                    gimbal_cmd_pub->publish(gimbal_msg);
                }

                // 停火必须显式发 0（导航侧会锁存）
                example_interfaces::msg::UInt8 shoot_msg;
                shoot_msg.data = (cmd.appear && cmd.fire_advice) ? 2 : 0; // 2 = 连发
                shoot_cmd_pub->publish(shoot_msg);
            }

            static TimePoint last_target_pub {};
            static bool last_tracking = false;
            static bool has_last_tracking = false;
            const auto now_target = Clock::now();
            const bool due = (now_target - last_target_pub) >= std::chrono::milliseconds(66);
            const bool changed = !has_last_tracking || (cmd.appear != last_tracking);
            if (due || changed) {
                auto_aim_interfaces::msg::Target target_msg;
                target_msg.tracking = cmd.appear;
                target_pub->publish(target_msg);
                last_tracking = cmd.appear;
                has_last_tracking = true;
                last_target_pub = now_target;
            }
        }
#endif
#ifdef USE_Daedalus
        if (daedalus_shm_client) {
            auto gimbal_in_gimbal_odom =
                tf->pose_a_in_b(SimpleFrame::GIMBAL, SimpleFrame::GIMBAL_ODOM, Clock::now());
            auto rpy =
                utils::matrix2rpy<double>(gimbal_in_gimbal_odom.linear(), utils::RPYOrder::ZYX);
            static TimePoint last_fire;
            bool fire = cmd.fire_advice
                && (std::abs(angles::shortest_angular_distance_degrees(
                        angles::to_degrees(rpy[2]),
                        cmd.target_yaw
                    )) < cmd.enable_yaw_diff
                    && std::abs(angles::shortest_angular_distance_degrees(
                           angles::to_degrees(-rpy[1]),
                           cmd.target_pitch
                       ))
                        < cmd.enable_pitch_diff);
            if (fire) {
                last_fire = Clock::now();
            }
            bool send_fire = false;
            if (Clock::now() - last_fire < std::chrono::milliseconds(100)) {
                send_fire = true;
            }
            daedalus_shm_client
                ->send_gimbal_cmd(cmd.yaw, -cmd.pitch, cmd.appear ? 1.0 : -1.0, send_fire);
            // daedalus_shm_client
            //     ->send_gimbal_cmd(cmd.yaw, -cmd.pitch, cmd.appear ? 1.0 : -1.0, false);
        }
#endif
        auto old_in_camera_cv = tf->pose_a_in_b(
            SimpleFrame(cmd.aim_point.frame_id),
            SimpleFrame::CAMERA_CV,
            cmd.timestamp
        );
        cmd.aim_point.transform(old_in_camera_cv, std::to_underlying(SimpleFrame::CAMERA_CV));
        record_cmd.set(cmd); // 逐帧记录用的指令快照
        if (dbg && (is_web_running() || show_window)) {
            dbg->gimbal_cmd.set(cmd);
        }
    });
    s.add_rate_source<>("logger", 1.0, [&]() {
        const double avg_latency_ms = log_ctx.latency_ms_total / log_ctx.detect_count;
        // 号码来源对比（诊断）：E2E 检测头 (armor_infer CLASSES 表) vs 数字分类器 (label.txt)。
        //   正常 = 全落在对角线（no2->no2）；若频繁出现 sentry->no1 / no5->outpost 这类
        //   "e2e 比 clf 低一档"，说明 E2E 的类别表相对 label.txt 整体错位，
        //   这会让 init 瞬间把 no2 读成 sentry/outpost 并冻结进 target_number。
        auto& ns = auto_aim::number_source_stats();
        std::string num_diag;
        int num_pairs = 0;
        int num_off_by_one = 0;
        for (int e = 0; e < auto_aim::NumberSourceStats::N; ++e) {
            for (int c = 0; c < auto_aim::NumberSourceStats::N; ++c) {
                const int cnt = ns.pair[e][c];
                if (cnt <= 0)
                    continue;
                num_pairs += 1;
                if (c == e + 1)
                    num_off_by_one += cnt;
                if (num_pairs <= 5) {
                    if (!num_diag.empty())
                        num_diag += ' ';
                    num_diag +=
                        auto_aim::string_by_armor_class(static_cast<auto_aim::ArmorClass>(e)) + "->"
                        + auto_aim::string_by_armor_class(static_cast<auto_aim::ArmorClass>(c)) + ":"
                        + std::to_string(cnt);
                }
            }
        }
        if (num_diag.empty())
            num_diag = "none";
        else if (num_pairs > 5)
            num_diag += " ...";
        const int num_unknown = ns.clf_unknown;
        const int num_total = ns.total;
        ns.reset();
        // 自瞄出口计数：very_aim() 从哪条路退出去（含 get_hit/build_traj/replenish
        // 这些静默出口，原来日志里看不到）
        auto& ast = auto_aim::aim_path_stats();
        const int a_calls = ast.calls;
        const int a_ok = ast.ok;
        const int a_hit0 = ast.hit0;
        const int a_cp0 = ast.cp0_invalid;
        const int a_build = ast.build_fail;
        const int a_mpc = ast.mpc_fail;
        const int a_chit0 = ast.center_hit0;
        const int a_cbuild = ast.center_build;
        const int a_repl = ast.replenish;
        const int a_nan = ast.nan_cmd;
        const int a_bad = ast.state_bad;
        const int a_cv = ast.clamp_v;
        const int a_cy = ast.clamp_vyaw;
        ast.reset();
        AWAKENING_INFO(
            "detect: {} track: {} found: {} solve: {} serial: {} bad_frame: {} camera: {} "
            "number(e2e->clf): {} off_by_one: {}/{} clf_unknown: {} "
            "aimer(calls/ok/hit/cp0/build/mpc/chit/cbuild/repl/nan/bad/cv/cy): "
            "{}/{}/{}/{}/{}/{}/{}/{}/{}/{}/{}/{}/{} avg_latency: {:.3} ms",
            log_ctx.detect_count,
            log_ctx.track_count,
            log_ctx.found_count,
            log_ctx.solve_count,
            log_ctx.serial_count,
            log_ctx.serial_bad_frame,
            log_ctx.camera_count,
            num_diag,
            num_off_by_one,
            num_total,
            num_unknown,
            a_calls,
            a_ok,
            a_hit0,
            a_cp0,
            a_build,
            a_mpc,
            a_chit0,
            a_cbuild,
            a_repl,
            a_nan,
            a_bad,
            a_cv,
            a_cy,
            avg_latency_ms
        );
        if (dbg) {
            dbg->avg_latency_ms.set(avg_latency_ms);
        }
        log_ctx.reset();
    });
    if (dbg) {
        s.add_rate_source<>("debug", 45.0, [&]() {
            auto __armor_target = armor_target.read();
            if (is_web_running() || show_window) {
                static Web web;
                dbg->type =
                    (mode == Mode::AutoAim) ? VisionDebugCtx::AUTO_AIM : VisionDebugCtx::AUTO_BUFF;
                auto __rune_target = rune_target.read();
                __armor_target.write_log();
                __rune_target.write_log();
                wheel_odometry.write_log();
                auto img_now = dbg->img_frame.get().timestamp;
                dbg->armor_target.set(__armor_target);
                dbg->rune_target.set(__rune_target);
                dbg->auto_aim_fsm_state.set(auto_aim_fsm_controller.get_state());
                auto gimbal_in_gimbal_odom =
                    tf->pose_a_in_b(SimpleFrame::GIMBAL, SimpleFrame::GIMBAL_ODOM, Clock::now());
                auto rpy =
                    utils::matrix2rpy<double>(gimbal_in_gimbal_odom.linear(), utils::RPYOrder::ZYX);
                auto gimbal_yaw_pitch =
                    std::make_pair(angles::to_degrees(rpy[2]), -angles::to_degrees(rpy[1]));
                dbg->gimbal_yaw_pitch.set(gimbal_yaw_pitch);
                bullet_pick_up.update(
                    Clock::now(),
                    dbg->gimbal_cmd.get().appear ? dbg->gimbal_cmd.get().fly_time : 0.4
                );
                auto bullet_poss =
                    bullet_pick_up.get_bullet_positions(img_now, very_aimer.get_yaw_pitch_offset());
                auto odom_in_camera_cv =
                    tf->pose_a_in_b(SimpleFrame::ODOM, SimpleFrame::CAMERA_CV, img_now);
                for (auto& pos: bullet_poss) {
                    pos = odom_in_camera_cv * pos;
                }
                dbg->odom_in_camera_cv.set(odom_in_camera_cv);
                dbg->bullet_positions.set(bullet_poss);
                web.write_debug_data(dbg.value());
                auto img = dbg->img_frame.get();
                auto debug_img = img.src_img.clone();
                if (img.format == PixelFormat::RGB) {
                    cv::cvtColor(debug_img, debug_img, cv::COLOR_RGB2BGR);
                }
                if (!debug_img.empty()) {
                    web.draw(debug_img, dbg.value());
                    if (is_web_running()) {
                        web.write_shm(debug_img);
                    }
                    if (show_window) {
                        std::lock_guard<std::mutex> lock(show_mutex);
                        show_img = debug_img;
                    }
                }
            }

#ifdef USE_ROS2
            auto armors = dbg->armors.get();
            rcl::pub_armor_target_marker(
                rcl_node,
                SimpleFrame_to_str(__armor_target.get_target_state().frame_id),
                __armor_target
            );
#endif
        });
#ifdef USE_ROS2
        if (debug) {
            s.add_rate_source<>("tf_pub", 100.0, [&]() {
                rcl_tf.pub_robot_tf(*tf, [](SimpleFrame frame) {
                    return SimpleFrame_to_str(frame);
                });
            });
        }

#endif
#ifdef USE_RERUN
        if (debug) {
            s.add_rate_source<>("rerun_tf", 15.0, [&]() {
                rerun_visual::log_robot_tf(*tf, [](SimpleFrame frame) {
                    return SimpleFrame_to_str(frame);
                });
            });
        }
#endif
    }

    if (camera) {
        camera->start<CameraTag>("hik");
    }
    if (serial) {
        serial->start<SerialTag>("serial");
    }
    if (show_window) {
        show_running.store(true);
        show_thread = std::thread([&] {
            AWAKENING_INFO("debug window enabled: press ESC or 'q' to close it");
            try {
                auto last_tp = Clock::now();
                uint64_t last_frames = detect_frame_count.load(std::memory_order_relaxed);
                double fps = 0.0;
                while (show_running.load()) {
                    cv::Mat img;
                    {
                        std::lock_guard<std::mutex> lock(show_mutex);
                        img = show_img;
                    }

                    // 统计实际输入帧率（相机帧计数）
                    const auto now = Clock::now();
                    const uint64_t frames = detect_frame_count.load(std::memory_order_relaxed);
                    const double dt = std::chrono::duration<double>(now - last_tp).count();
                    if (dt >= 0.5) {
                        fps = static_cast<double>(frames - last_frames) / dt;
                        last_tp = now;
                        last_frames = frames;
                    }

                    if (!img.empty()) {
                        cv::Mat shown = img.clone();
                        cv::putText(
                            shown,
                            fmt::format("FPS(detect): {:.1f}", fps),
                            cv::Point(10, 100),
                            cv::FONT_HERSHEY_SIMPLEX,
                            1.4,
                            cv::Scalar(0, 255, 0),
                            3,
                            cv::LINE_AA
                        );
                        cv::imshow("awakening auto_aim", shown);
                    }
                    const int key = cv::waitKey(1);
                    if (key == 27 || key == 'q') {
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(15));
                }
            } catch (const std::exception& e) {
                AWAKENING_ERROR("debug window disabled ({}), auto-aim keeps running", e.what());
            }
            cv::destroyAllWindows();
        });
    }
    s.build();
    s.run();

#ifdef USE_ROS2
    std::thread([&]() { rcl_node.spin(); }).detach();
#endif
    utils::SignalGuard::spin(std::chrono::milliseconds(1000));
    s.stop();
    if (show_thread.joinable()) {
        show_running.store(false);
        show_thread.join();
    }
#ifdef USE_ROS2
    rcl_node.shutdown();
#endif
    for (int i = 0; i < 10; ++i) {
        AWAKENING_CRITICAL("改了东西记得同步其他有关的exe的src");
    }
    return 0;
}
