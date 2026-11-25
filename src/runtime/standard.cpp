#include "angles.h"
#include "ascii_banner.hpp"
#include "daedalus_interface/shm_layout.hpp"
#include "tasks/auto_aim/armor_track/motion_model.hpp"
#include "tasks/auto_buff/rune_control/rune_aimer.hpp"
#include "tasks/auto_buff/rune_detect/rune_detector.hpp"
#include "tasks/auto_buff/rune_track/rune_target.hpp"
#include "tasks/auto_buff/rune_track/rune_tracker.hpp"
#include "tasks/auto_buff/type.hpp"
#include "tasks/base/ballistic_trajectory.hpp"
#include "tasks/base/serial_ascii.hpp"
#include "tasks/base/wheel_odometry.hpp"
#include "utils/drivers/camera_factory.hpp"
#include "utils/io/video_save.hpp"
#ifdef USE_NAV_CMD_PUB
    #include <example_interfaces/msg/u_int8.hpp>
    #include <sensor_msgs/msg/joint_state.hpp>
#endif
#ifdef USE_NAV_TARGET_PUB
    #include <auto_aim_interfaces/msg/target.hpp>
#endif
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
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
            // 优先匹配 web.py 设置的进程名；没装 setproctitle 时退化为匹配命令行，
            // 这样 python3 web.py 也能被识别到。
            const int ret = std::system(
                "pgrep -x wust_vision_web > /dev/null 2>&1"
                " || pgrep -f '[w]eb\\.py' > /dev/null 2>&1"
            );
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
    auto config = YAML::LoadFile(config_path);
    Scheduler s;
    std::atomic<Mode> mode = str_to_Mode(config["mode"].as<std::string>());
    std::atomic<EnemyColor> enemy_color =
        enemy_color_from_string(config["enemy_color"].as<std::string>());
    std::atomic<double> bullet_speed = config["bullet_speed"].as<double>();

    // 串口协议：ascii = AAP/ARQ 文本协议（新）；缺省/其它 = 旧的二进制包
    const auto serial_config = config["serial"];
    const bool use_ascii_protocol =
        serial_config["protocol"]
        && utils::to_upper(serial_config["protocol"].as<std::string>()) == "ASCII";
    // 只收不发：串口归导航（接法 A）时，视觉仍可以读 IMU，但不能下发云台指令
    const bool serial_send_enable =
        !(serial_config["send_enable"] && !serial_config["send_enable"].as<bool>());
    const auto ascii_config = serial_config["ascii"];
    const double ascii_send_hz =
        ascii_config["send_hz"] ? ascii_config["send_hz"].as<double>() : 200.0;
    const double ascii_max_step_deg =
        ascii_config["max_step_deg"] ? std::abs(ascii_config["max_step_deg"].as<double>()) : 3.0;
    // 增量基准：feedback = 用 ARQ 四元数解出的当前云台角；command = 用上一拍下发的角度
    const bool ascii_use_feedback = !(
        ascii_config["increment_source"]
        && utils::to_upper(ascii_config["increment_source"].as<std::string>()) == "COMMAND"
    );
    std::atomic<double> gimbal_imu_yaw_deg { 0.0 };
    std::atomic<double> gimbal_imu_pitch_deg { 0.0 };
    std::atomic<bool> gimbal_imu_valid { false };
    std::atomic<int64_t> gimbal_imu_last_ns { 0 };
    double ascii_last_cmd_yaw_deg = 0.0;
    double ascii_last_cmd_pitch_deg = 0.0;
    uint32_t ascii_frame_id = 0;
    TimePoint ascii_last_send = Clock::now();

    // 接法 A（ROS 话题）：publish_to_nav 为 true 时发布
    //   cmd_gimbal_joint (sensor_msgs/JointState, reliable, rad 增量)
    //   cmd_shoot        (example_interfaces/UInt8, reliable, 0/1)
    //   tracker/target   (auto_aim_interfaces/Target, SensorDataQoS, >=15Hz)
    bool publish_to_nav = false;
    bool nav_gimbal_delta = true;
    bool nav_gimbal_units_deg = false;
#ifdef USE_ROS2
    publish_to_nav =
        config["publish_to_nav"] && config["publish_to_nav"].as<bool>();
    nav_gimbal_delta = !(
        config["nav_gimbal_mode"]
        && utils::to_upper(config["nav_gimbal_mode"].as<std::string>()) == "ABSOLUTE"
    );
    nav_gimbal_units_deg =
        config["nav_gimbal_units"]
        && utils::to_upper(config["nav_gimbal_units"].as<std::string>()) == "DEG";
#endif

#ifdef USE_ROS2
    rcl::RclcppNode rcl_node("awakening");
    rcl::TF rcl_tf(rcl_node);
#ifdef USE_NAV_CMD_PUB
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_cmd_pub;
    rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr shoot_cmd_pub;
#endif
#ifdef USE_NAV_TARGET_PUB
    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_cmd_pub;
#endif
    if (publish_to_nav) {
#ifdef USE_NAV_CMD_PUB
        gimbal_cmd_pub = rcl_node.make_pub<sensor_msgs::msg::JointState>(
            "cmd_gimbal_joint",
            rclcpp::QoS(rclcpp::KeepLast(10))
        );
        shoot_cmd_pub = rcl_node.make_pub<example_interfaces::msg::UInt8>(
            "cmd_shoot",
            rclcpp::QoS(rclcpp::KeepLast(10))
        );
#endif
#ifdef USE_NAV_TARGET_PUB
        target_cmd_pub = rcl_node.make_pub<auto_aim_interfaces::msg::Target>(
            "tracker/target",
            rclcpp::SensorDataQoS()
        );
#endif
        AWAKENING_INFO("publish_to_nav: cmd_gimbal_joint / cmd_shoot / tracker/target");
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
    utils::SWMR<auto_aim::ArmorTarget> armor_target;
    utils::SWMR<auto_buff::RuneTarget> rune_target;
    BulletPickUp bullet_pick_up(config["bullet_pick_up"]);
    LogCtx log_ctx;
    // imshow 上显示的实时统计（由 1Hz 的 logger 源更新）
    std::atomic<double> ui_camera_fps { 0.0 };
    std::atomic<double> ui_detect_fps { 0.0 };
    std::atomic<double> ui_avg_latency_ms { 0.0 };
    std::optional<VisionDebugCtx> dbg;
    std::pair<double, double> operator_offset = std::make_pair(0, 0);
    if (debug) {
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

            log_ctx.serial_count++;
            static bool dumped_first_chunk = false;
            if (!dumped_first_chunk && !data.empty()) {
                dumped_first_chunk = true;
                char hex[3 * 48 + 1] = {};
                const size_t n = std::min<size_t>(data.size(), 48);
                for (size_t i = 0; i < n; ++i) {
                    std::snprintf(hex + i * 3, 4, "%02x ", static_cast<unsigned>(data[i]));
                }
                AWAKENING_INFO("serial first chunk ({} bytes): {}", data.size(), hex);
            }
            if (use_ascii_protocol) {
                // ARQ<w>X<x>Y<y>Z<z>C<crc>%：四元数即云台姿态，直接用来更新 TF
                static serial_ascii::ArqFramer framer;
                framer.feed(
                    std::string_view(reinterpret_cast<const char*>(data.data()), data.size()),
                    [&](const serial_ascii::ArqFrame& frame) {
                        Eigen::Quaterniond q(frame.w, frame.x, frame.y, frame.z);
                        const double norm = q.norm();
                        if (!std::isfinite(norm) || norm < 1e-6) {
                            return;
                        }
                        q.normalize();
                        ISO3 gimbal_in_gimbal_odom = ISO3::Identity();
                        gimbal_in_gimbal_odom.linear() = q.toRotationMatrix();
                        tf->push(
                            SimpleFrame::GIMBAL_ODOM,
                            SimpleFrame::GIMBAL,
                            now,
                            gimbal_in_gimbal_odom
                        );
                        const auto rpy = utils::matrix2rpy<double>(
                            gimbal_in_gimbal_odom.linear(),
                            utils::RPYOrder::ZYX
                        );
                        gimbal_imu_yaw_deg = angles::to_degrees(rpy[2]);
                        gimbal_imu_pitch_deg = angles::to_degrees(rpy[1]);
                        gimbal_imu_valid = true;
                        gimbal_imu_last_ns = now.time_since_epoch().count();
                    }
                );
                // 每秒打印一次解析统计，便于确认这个串口上到底有没有 IMU/ARQ
                utils::dt_once(
                    [&]() {
                        static uint64_t last_good = 0;
                        static uint64_t last_bad = 0;
                        const uint64_t good = framer.good_frames();
                        const uint64_t bad = framer.bad_frames();
                        AWAKENING_INFO(
                            "serial ascii rx: ARQ ok/s={} bad/s={} (total ok={} bad={})",
                            good - last_good,
                            bad - last_bad,
                            good,
                            bad
                        );
                        last_good = good;
                        last_bad = bad;
                    },
                    std::chrono::duration<double>(1.0)
                );
                return;
            }
            if (auto robo_opt = ReceiveRobotData::create(data); robo_opt.has_value()) {
                auto robo = robo_opt.value();
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
                                      .id = frame.id,
                                      .frame_id = frame.frame_id };
            {
                bool got = detector_sem->try_acquire();
                utils::SemaphoreGuard guard(*detector_sem, got); //并发控制
                if (got) {
                    auto start = Clock::now();
                    r.detect_start = start;
                    auto [ls, as] = armor_detector.detect(frame, net_focus, detect_light);
                    armors.armors = as;
                    armors.lights = ls;
                    log_ctx.detect_count++;
                }
            }
            armors_queue.enqueue(armors);
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
            {
                bool got = detector_sem->try_acquire();
                utils::SemaphoreGuard guard(*detector_sem, got); //并发控制
                if (got) {
                    auto start = Clock::now();
                    r.detect_start = start;
                    rune_detection = rune_detector.detect(frame, net_focus, enemy_color);
                    log_ctx.detect_count++;
                }
            }
            rune_detection.frame_id = frame.frame_id;
            rune_detection.id = frame.id;
            rune_detection.timestamp = frame.img_frame.timestamp;
            rune_detections_queue.enqueue(rune_detection);
            auto batch_runes = rune_detections_queue.dequeue_batch();
            r.rune = batch_runes;
        }

        if (dbg) {
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
                auto_aim_fsm_controller.update(
                    __armor_target.get_target_state().vyaw(),
                    __armor_target.jumped,
                    __armor_target.get_target_state().timestamp
                );
                armor_target.write(__armor_target);
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

        if (serial && serial_send_enable) {
            if (use_ascii_protocol) {
                // AAP<pitch>Y<yaw>F<fire>N<frame>C<crc>%：增量角，单次限幅 max_step_deg
                const auto send_now = Clock::now();
                const double period = 1.0 / std::max(ascii_send_hz, 1.0);
                if (std::chrono::duration<double>(send_now - ascii_last_send).count() >= period) {
                    ascii_last_send = send_now;

                    double pitch_step = 0.0;
                    double yaw_step = 0.0;
                    bool fire = false;
                    if (cmd.appear) {
                        const int64_t imu_last_ns = gimbal_imu_last_ns.load();
                        const bool feedback_fresh =
                            imu_last_ns != 0
                            && std::chrono::duration<double>(
                                   send_now - TimePoint(std::chrono::nanoseconds(imu_last_ns))
                               )
                                       .count()
                                < 0.2;
                        const bool from_feedback =
                            ascii_use_feedback && gimbal_imu_valid.load() && feedback_fresh;
                        const double base_yaw =
                            from_feedback ? gimbal_imu_yaw_deg.load() : ascii_last_cmd_yaw_deg;
                        const double base_pitch =
                            from_feedback ? gimbal_imu_pitch_deg.load() : ascii_last_cmd_pitch_deg;
                        yaw_step = std::clamp(
                            angles::shortest_angular_distance_degrees(base_yaw, cmd.yaw),
                            -ascii_max_step_deg,
                            ascii_max_step_deg
                        );
                        pitch_step = std::clamp(
                            angles::shortest_angular_distance_degrees(base_pitch, cmd.pitch),
                            -ascii_max_step_deg,
                            ascii_max_step_deg
                        );
                        // 记录“已下发到的角度”，作为没有 ARQ 反馈时的增量基准
                        ascii_last_cmd_yaw_deg = base_yaw + yaw_step;
                        ascii_last_cmd_pitch_deg = base_pitch + pitch_step;
                        fire = cmd.fire_advice;
                    }
                    if (std::isfinite(pitch_step) && std::isfinite(yaw_step)) {
                        auto frame = serial_ascii::encode_aap(
                            pitch_step,
                            yaw_step,
                            fire,
                            ascii_frame_id++
                        );
                        serial->write(std::vector<uint8_t>(frame.begin(), frame.end()));
                    }
                }
            } else {
                SendRobotCmdData send;
                send.cmd_ID = SendRobotCmdData::ID;
                send.time_stamp =
                    std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start_tp)
                        .count();
                send.appear = cmd.appear,
                send.detect_color = std::to_underlying(enemy_color.load());
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
        if (publish_to_nav) {
            // 云台/开火：约 200 Hz 发布（跟随 IMU 包率）
            const auto pub_now = Clock::now();
            static TimePoint last_gimbal_pub {};
            if (std::chrono::duration<double>(pub_now - last_gimbal_pub).count()
                >= 1.0 / 200.0) {
                last_gimbal_pub = pub_now;

                const auto rpy = utils::matrix2rpy<double>(
                    gimbal_in_gimbal_odom.linear(),
                    utils::RPYOrder::ZYX
                );
                const double current_pitch_deg = angles::to_degrees(rpy[1]);
                const double current_yaw_deg = angles::to_degrees(rpy[2]);
                const auto to_nav_units = [&](double deg) {
                    return nav_gimbal_units_deg ? deg : angles::from_degrees(deg);
                };

                double pitch_cmd = 0.0;
                double yaw_cmd = 0.0;
                bool send_gimbal = false;
                if (cmd.appear) {
                    if (nav_gimbal_delta) {
                        // 增量：相对当前云台角，单次限幅 ±3°（~200Hz 等效 600°/s）
                        constexpr double MAX_STEP_DEG = 3.0;
                        const double d_pitch = std::clamp(
                            cmd.pitch - current_pitch_deg,
                            -MAX_STEP_DEG,
                            MAX_STEP_DEG
                        );
                        const double d_yaw = std::clamp(
                            angles::shortest_angular_distance_degrees(
                                current_yaw_deg,
                                cmd.yaw
                            ),
                            -MAX_STEP_DEG,
                            MAX_STEP_DEG
                        );
                        pitch_cmd = to_nav_units(d_pitch);
                        yaw_cmd = to_nav_units(d_yaw);
                    } else {
                        // 绝对角恒为 rad
                        pitch_cmd = angles::from_degrees(cmd.pitch);
                        yaw_cmd = angles::from_degrees(cmd.yaw);
                    }
                    send_gimbal = std::isfinite(pitch_cmd) && std::isfinite(yaw_cmd);
                } else if (nav_gimbal_delta) {
                    // 丢目标：显式发 0/0（= 别动）
                    send_gimbal = true;
                }

#ifdef USE_NAV_CMD_PUB
                if (gimbal_cmd_pub && send_gimbal) {
                    sensor_msgs::msg::JointState gimbal_msg;
                    // PlotJuggler 按 header.stamp 作为横轴；不打时间戳会被画到 1970 年，
                    // 表现为"话题有、曲线看不到"。
                    gimbal_msg.header.stamp = rcl_node.get_node()->now();
                    gimbal_msg.name = { "gimbal_pitch_joint", "gimbal_yaw_joint" };
                    gimbal_msg.position = { pitch_cmd, yaw_cmd };
                    gimbal_cmd_pub->publish(gimbal_msg);
                }
                if (shoot_cmd_pub) {
                    // 每拍都发，只有 0/1；停火必须显式发 0
                    example_interfaces::msg::UInt8 shoot_msg;
                    shoot_msg.data = (cmd.appear && cmd.fire_advice) ? 1 : 0;
                    shoot_cmd_pub->publish(shoot_msg);
                }
#endif
            }

#ifdef USE_NAV_TARGET_PUB
            // 目标状态：>=15 Hz，tracking 变化时立即发
            static TimePoint last_target_pub {};
            static bool last_tracking = false;
            static bool has_last_tracking = false;
            const bool due =
                (pub_now - last_target_pub) >= std::chrono::milliseconds(66);
            const bool tracking_changed =
                !has_last_tracking || (cmd.appear != last_tracking);
            if (due || tracking_changed) {
                auto_aim_interfaces::msg::Target target_msg;
                target_msg.tracking = cmd.appear;
                if (cmd.appear && a_target.check()) {
                    using auto_aim::ArmorClass;
                    const auto number = a_target.target_number;
                    target_msg.header.stamp = rcl_node.get_node()->now();
                    target_msg.header.frame_id = "gimbal_odom";
                    switch (number) {
                        case ArmorClass::SENTRY:
                            target_msg.id = "7";
                            break;
                        case ArmorClass::NO1:
                            target_msg.id = "1";
                            break;
                        case ArmorClass::NO2:
                            target_msg.id = "2";
                            break;
                        case ArmorClass::NO3:
                            target_msg.id = "3";
                            break;
                        case ArmorClass::NO4:
                            target_msg.id = "4";
                            break;
                        case ArmorClass::NO5:
                            target_msg.id = "5";
                            break;
                        case ArmorClass::OUTPOST:
                            target_msg.id = "outpost";
                            break;
                        case ArmorClass::BASE:
                            target_msg.id = "base";
                            break;
                        default:
                            target_msg.id = "unknown";
                            break;
                    }
                    const auto& state = a_target.get_target_state();
                    const auto pos = state.pos();
                    const auto vel = state.vel();
                    target_msg.armors_num = a_target.armor_num();
                    target_msg.position.x = pos.x();
                    target_msg.position.y = pos.y();
                    target_msg.position.z = pos.z();
                    target_msg.velocity.x = vel.x();
                    target_msg.velocity.y = vel.y();
                    target_msg.velocity.z = vel.z();
                    target_msg.yaw = state.yaw();
                    target_msg.v_yaw = state.vyaw();
                    target_msg.radius_1 = state.r1();
                    target_msg.radius_2 = state.r2();
                    target_msg.dz = state.h();
                }
                if (target_cmd_pub) {
                    target_cmd_pub->publish(target_msg);
                }
                last_tracking = cmd.appear;
                has_last_tracking = true;
                last_target_pub = pub_now;
            }
#endif
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
        if (dbg) {
            dbg->gimbal_cmd.set(cmd);
        }
    });
    s.add_rate_source<>("logger", 1.0, [&]() {
        double avg_latency_ms = log_ctx.latency_ms_total / log_ctx.detect_count;
        AWAKENING_INFO(
            "detect: {} track: {} found: {} solve: {} serial: {} camera: {} avg_latency: {:.3} ms",
            log_ctx.detect_count,
            log_ctx.track_count,
            log_ctx.found_count,
            log_ctx.solve_count,
            log_ctx.serial_count,
            log_ctx.camera_count,
            avg_latency_ms
        );
        if (dbg) {
            dbg->avg_latency_ms.set(avg_latency_ms);
        }
        ui_camera_fps = log_ctx.camera_count;
        ui_detect_fps = log_ctx.detect_count;
        ui_avg_latency_ms = std::isfinite(avg_latency_ms) ? avg_latency_ms : 0.0;
        log_ctx.reset();
    });
    if (dbg) {
        s.add_rate_source<>("debug", 45.0, [&]() {
            auto __armor_target = armor_target.read();
            {
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
                    // 画出识别到的装甲板框，以及跟踪目标预测出的 4 块装甲板的重投影框
                    web.draw(debug_img, dbg.value());

                    // imshow 左上角帧率/延迟（由 1Hz 的 logger 源更新）
                    {
                        char fps_text[96];
                        std::snprintf(
                            fps_text,
                            sizeof(fps_text),
                            "CAM %.0f FPS | DET %.0f FPS | %.1f ms",
                            ui_camera_fps.load(),
                            ui_detect_fps.load(),
                            ui_avg_latency_ms.load()
                        );
                        const cv::Point org { 10, 28 };
                        cv::putText(
                            debug_img,
                            fps_text,
                            org,
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.7,
                            cv::Scalar(0, 0, 0),
                            4,
                            cv::LINE_AA
                        );
                        cv::putText(
                            debug_img,
                            fps_text,
                            org,
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.7,
                            cv::Scalar(0, 255, 255),
                            2,
                            cv::LINE_AA
                        );
                    }

                    // 推送到 web.py（浏览器打开 http://<ip>:9000 即可看到带框画面）
                    if (is_web_running()) {
                        web.write_shm(debug_img);
                    }

                    // 有图形界面时同时弹本地窗口；无 DISPLAY（纯 SSH）时跳过，避免 highgui 报错
                    if (std::getenv("DISPLAY") != nullptr) {
                        cv::imshow("Auto Aim - Reprojection", debug_img);
                        cv::waitKey(1);
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
    s.build();
    s.run();

#ifdef USE_ROS2
    std::thread([&]() { rcl_node.spin(); }).detach();
#endif
    utils::SignalGuard::spin(std::chrono::milliseconds(1000));
    s.stop();
#ifdef USE_ROS2
    rcl_node.shutdown();
#endif
    for (int i = 0; i < 10; ++i) {
        AWAKENING_CRITICAL("改了东西记得同步其他有关的exe的src");
    }
    return 0;
}
