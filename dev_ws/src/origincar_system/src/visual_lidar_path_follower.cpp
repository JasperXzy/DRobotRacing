#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

#include "follower_utils.hpp"

using origincar_system::route_direction_name;

// namespace
// 构造函数完成参数声明、参数读取、航点加载、话题订阅/发布和控制定时器创建。
VisualLidarPathFollower::VisualLidarPathFollower()
: rclcpp::Node("visual_lidar_path_follower"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  read_parameters();

  // 启动时始终预载 Task1 路线。物理底盘开关关闭期间节点会持续完成
  // 静态绿色候选确认；开关打开后同一条路线立即可用，不需要等待二维码。
  // 顺/逆时针文件同时各保留一份无偏移保护线，仅用于限制运行绿/橙点
  // 坐标更新不得跨越原始路线；它们不参与此刻的控制和路径偏移。
  logic3_clockwise_update_guard_route_ = load_waypoints(
    route_waypoints_clockwise_file_, "clockwise");
  logic3_counterclockwise_update_guard_route_ = load_waypoints(
    route_waypoints_counterclockwise_file_, "counterclockwise");
  waypoints_ = load_waypoints(task1_waypoints_file_, "task1");
  reset_waypoint_shift_state();
  task1_original_route_ = original_waypoints_;
  selected_route_direction_ = "task1";
  waiting_for_qr_start_ = false;
  following_qr_path_ = true;
  set_task_phase(TaskPhase::TASK1);

  // 发布速度命令，最终由主线底盘驱动订阅 /cmd_vel 执行。
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  tts_pub_ = create_publisher<std_msgs::msg::String>(tts_topic_, 10);

  // 发布图像分析触发。transient_local 让后启动的订阅者也能收到最后一次触发值。
  image_analysis_trigger_pub_ =
    create_publisher<std_msgs::msg::Int32>(image_analysis_trigger_topic_, rclcpp::QoS(10).transient_local());
  visual_control_suspend_pub_ = create_publisher<std_msgs::msg::Int32>(visual_control_suspend_topic_, 3);
  logic3_snapshot_pub_ = create_publisher<std_msgs::msg::String>(
    "/visual_lidar_path_follower/route_snapshot",
    rclcpp::QoS(32).transient_local().reliable());
  if (enable_debug_visualization_) {
    const auto transient_qos = rclcpp::QoS(1).transient_local().reliable();
    debug_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/visual_lidar_path_follower/debug_markers", transient_qos);
    debug_route_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/visual_lidar_path_follower/debug_route_markers", transient_qos);
    debug_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/visual_lidar_path_follower/debug_path", transient_qos);
  }

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS().keep_last(1),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
      latest_scan_ = msg;
      latest_scan_received_at_ = std::chrono::steady_clock::now();
      ++scan_sequence_;
      // The diagnostic can arrive before or after this node receives /scan.
      // Latch it only when both messages carry the same scan timestamp.
      latch_logic3_match_diagnostics_to_latest_scan();
    });
  if (!logic3_match_diagnostics_topic_.empty()) {
    logic3_match_diagnostics_sub_ =
      create_subscription<geometry_msgs::msg::Vector3Stamped>(
      logic3_match_diagnostics_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
        latest_match_diagnostics_stamp_nanoseconds_ = rclcpp::Time(
          msg->header.stamp, get_clock()->get_clock_type()).nanoseconds();
        latest_lidar_match_score_ = msg->vector.x;
        latest_map_odom_translation_correction_ = msg->vector.y;
        latest_map_odom_yaw_correction_ = msg->vector.z;
        latest_match_diagnostics_valid_ =
          std::isfinite(msg->vector.x) && std::isfinite(msg->vector.y) &&
          std::isfinite(msg->vector.z);
        // The values above are pending until their timestamp matches the
        // follower's current /scan. This also handles diagnostics arriving
        // before the corresponding scan callback.
        latest_lidar_match_score_valid_ = false;
        latch_logic3_match_diagnostics_to_latest_scan();
      });
  }
  logic3_match_score_sub_ = create_subscription<std_msgs::msg::Float32>(
    logic3_match_score_topic_, rclcpp::QoS(10),
    [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
      // A configured stamped diagnostic is the authoritative score source.
      // Keep this legacy scalar path only for configurations that explicitly
      // disable the diagnostic topic.
      if (!logic3_match_diagnostics_topic_.empty()) {
        return;
      }
      latest_lidar_match_score_ = static_cast<double>(msg->data);
      latest_lidar_match_score_valid_ = std::isfinite(latest_lidar_match_score_);
      // match_score由lidar_location处理完一帧/scan后发布。消息本身没有
      // Header，因此锁存回调到达时最新的scan序号，让运行绿点只处理
      // 已经得到匹配分数的雷达帧，而不是抢先使用上一帧分数。
      latest_match_score_scan_sequence_ = scan_sequence_;
    });
  logic3_cmd_vel_history_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::QoS(100),
    [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
      const std::int64_t stamp_nanoseconds = now().nanoseconds();
      logic3_command_history_.push_back(
        Logic3CommandSample{stamp_nanoseconds, msg->angular.z, msg->linear.x});
      // Two seconds comfortably covers a lidar scan and inter-process
      // diagnostic latency. Keep one older sample as the command active at
      // the beginning of the retained interval.
      constexpr std::int64_t kHistoryDurationNanoseconds = 2000000000LL;
      const std::int64_t oldest_stamp =
        stamp_nanoseconds - kHistoryDurationNanoseconds;
      while (logic3_command_history_.size() > 1 &&
        logic3_command_history_[1].stamp_nanoseconds < oldest_stamp)
      {
        logic3_command_history_.pop_front();
      }
    });
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {latest_map_ = msg;});
  restriction_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    restriction_map_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      latest_restriction_map_ = msg;
    });
  logic3_static_restriction_map_sub_ =
    create_subscription<nav_msgs::msg::OccupancyGrid>(
      logic3_static_restriction_map_topic_,
      rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        latest_logic3_static_restriction_map_ = msg;
      });
  logic3_runtime_restriction_map_sub_ =
    create_subscription<nav_msgs::msg::OccupancyGrid>(
      logic3_runtime_restriction_map_topic_,
      rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        latest_logic3_runtime_restriction_map_ = msg;
      });
  logic3_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    logic3_odom_topic_, rclcpp::SensorDataQoS().keep_last(10),
    std::bind(
      &VisualLidarPathFollower::logic3_odom_callback, this,
      std::placeholders::_1));

  return_visual_detection_sub_ =
    create_subscription<ai_msgs::msg::PerceptionTargets>(
    return_visual_detection_topic_, rclcpp::SensorDataQoS().keep_last(1),
    [this](ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
      latest_return_visual_targets_ = msg;
      latest_return_visual_received_at_ = std::chrono::steady_clock::now();
    });

  // 只需要一个有效 QR；选定方向后会取消订阅，避免后续误切路线。
  if (enable_qr_direction_select_) {
    qr_sub_ = create_subscription<std_msgs::msg::String>(
      qr_topic_, 10, std::bind(&VisualLidarPathFollower::qr_callback, this, std::placeholders::_1));
  }

  if (waypoints_.empty() && !waiting_for_qr_start_) {
    RCLCPP_ERROR(get_logger(), "未加载到航点，巡线节点将停车。");
    publish_stop();
    return;
  }

  const auto reverse_count = std::count_if(
    waypoints_.begin(), waypoints_.end(), [](const Waypoint & w) { return w.reverse; });
  const std::string route_name = following_qr_path_ ? "Task1" :
    route_direction_name(selected_route_direction_);
  RCLCPP_INFO(
    get_logger(),
    "VisualLidarPathFollower 初始化完成：%zu 个航点，路线=%s，跟踪器=%s，路线变形=%s。",
    waypoints_.size(), route_name.c_str(), tracking_controller_mode_.c_str(),
    route_deformation_mode_.c_str());
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(),
      "debug: 倒车点=%zu，Task1=%s，顺时针=%s，逆时针=%s，"
      "动态前瞻=%.2f+%.2f*|v|（上限%.2f），"
      "避障=全阶段静态+动态雷达控制，线速度=%.2f。",
      static_cast<std::size_t>(reverse_count), task1_waypoints_file_.c_str(),
      route_waypoints_clockwise_file_.c_str(), route_waypoints_counterclockwise_file_.c_str(),
      lookahead_base_, lookahead_speed_gain_, lookahead_max_, linear_speed_);
  }
  publish_debug_route();

  // 定时控制循环：每次读取 TF、更新路线进度并计算离散前瞻 waypoint 速度。
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(control_frequency_, 1.0)),
    std::bind(&VisualLidarPathFollower::control_once, this));
}

int main(int argc, char ** argv)
{
  // 标准 ROS 2 节点入口：初始化、创建节点、进入事件循环，退出前发布零速度。
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualLidarPathFollower>();
  rclcpp::spin(node);
  if (node->should_publish_stop_on_exit()) {
    node->publish_stop();
  }
  rclcpp::shutdown();
  return 0;
}
