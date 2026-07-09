#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <cctype>

#include <yaml-cpp/yaml.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

/*
 * smooth_path_follower.cpp
 *
 * 作用：
 *   这是 test/dev_ws 里的 91 点路径跟随节点。它不调用 Nav2 action，
 *   而是直接根据 map 坐标系下的航点、当前机器人位姿和雷达障碍物信息，
 *   计算 geometry_msgs/Twist 并发布到 /cmd_vel。
 *
 * 输入：
 *   1. YAML 航点文件：包含顺时针/逆时针路线，每个点至少有 x、y。
 *   2. TF：frame_id -> base_frame_id，默认 map -> base_link。
 *   3. /scan：LaserScan，用于前方障碍减速、避让、急停和倒车脱困。
 *   4. /filter_mask：可选 keepout 虚拟障碍地图。
 *   5. /qr_info 或 /qr_code：可选 QR 数字，用于选择顺/逆时针路线。
 *
 * 输出：
 *   1. /cmd_vel：底盘速度命令。
 *   2. /person_trigger 或 /sign4return：到指定航点时触发图像分析/图生文。
 *
 * 坐标单位：
 *   ROS map 坐标默认单位为米；速度单位 m/s，角速度单位 rad/s。
 */

namespace
{

// 将 value 限制在 [low, high] 区间内，常用于限速、限角速度和比例系数裁剪。
double clamp_value(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

// 将任意角度归一化到 [-pi, pi]，方便比较雷达扇区和航向误差。
double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// 从 ROS 参数里读取布尔值；兼容 bool 参数和 "true"/"1"/"yes" 这类字符串参数。
bool as_bool(const rclcpp::Parameter & parameter)
{
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    return parameter.as_bool();
  }
  const auto text = parameter.value_to_string();
  return text == "1" || text == "true" || text == "True" || text == "TRUE" ||
         text == "yes" || text == "on" || text == "ON";
}

// 从 ROS 参数里读取 double；兼容 double、int、字符串，失败时返回默认值。
double as_double(const rclcpp::Parameter & parameter, double default_value = 0.0)
{
  try {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return parameter.as_double();
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(parameter.as_int());
    }
    return std::stod(parameter.value_to_string());
  } catch (const std::exception &) {
    return default_value;
  }
}

// 从 ROS 参数里读取 int；兼容 int、double、字符串，失败时返回默认值。
int as_int(const rclcpp::Parameter & parameter, int default_value = 0)
{
  try {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<int>(parameter.as_int());
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return static_cast<int>(parameter.as_double());
    }
    return std::stoi(parameter.value_to_string());
  } catch (const std::exception &) {
    return default_value;
  }
}

// 从 YAML 节点里读取布尔值；兼容 YAML bool 和字符串形式。
bool yaml_bool(const YAML::Node & node, bool default_value = false)
{
  if (!node) {
    return default_value;
  }
  try {
    return node.as<bool>();
  } catch (const YAML::Exception &) {
    try {
      const auto text = node.as<std::string>();
      return text == "1" || text == "true" || text == "True" || text == "TRUE" ||
             text == "yes" || text == "on" || text == "ON";
    } catch (const YAML::Exception &) {
      return default_value;
    }
  }
}

// 单个航点。当前跟随器只使用 x/y/reverse/pass_radius，不使用 YAML 里的 yaw。
struct Waypoint
{
  // 航点在 map 坐标系下的 x 坐标，单位米。
  double x{0.0};
  // 航点在 map 坐标系下的 y 坐标，单位米。
  double y{0.0};
  // true 表示到这个点时倒车跟随；false 表示正向前进。
  bool reverse{false};
  // 正向通过半径，机器人离该点小于等于该值就认为通过，单位米。
  double pass_radius{0.0};
  // 倒车通过半径；倒车点优先使用它，单位米。
  double reverse_pass_radius{0.0};
};

// 实际追踪目标。它通常等于某个航点，也可能是平滑路径采样出来的临时目标点。
struct TrackingTarget
{
  // 追踪目标 x，单位米，map 坐标系。
  double x{0.0};
  // 追踪目标 y，单位米，map 坐标系。
  double y{0.0};
  // true 表示对这个目标执行倒车控制。
  bool reverse{false};
};

// 将字符串转成小写，用于 route_direction 等参数的宽松匹配。
std::string to_lower(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return text;
}

}  // namespace

// 路径跟随主节点：读取航点，监听 TF/雷达/QR，周期性计算并发布 /cmd_vel。
class SmoothPathFollower : public rclcpp::Node
{
public:
  // 构造函数完成参数声明、参数读取、航点加载、话题订阅/发布和控制定时器创建。
  SmoothPathFollower()
  : Node("smooth_path_follower"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 基础输入/输出参数：航点文件、坐标系、速度输出话题、雷达话题和 keepout 地图话题。
    // waypoints_file：航点 YAML 文件路径。为空时无法加载路线，节点会停车并报错。
    declare_parameter<std::string>("waypoints_file", "");
    // frame_id：全局坐标系名称。航点 x/y 必须属于这个坐标系，默认 map。
    declare_parameter<std::string>("frame_id", "map");
    // base_frame_id：机器人本体坐标系名称。节点通过 TF 查询 frame_id -> base_frame_id 得到当前位置。
    declare_parameter<std::string>("base_frame_id", "base_link");
    // cmd_vel_topic：速度命令输出话题。发布 geometry_msgs/Twist，默认给底盘的 /cmd_vel。
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    // scan_topic：激光雷达输入话题。订阅 sensor_msgs/LaserScan，用于真实障碍避让。
    declare_parameter<std::string>("scan_topic", "/scan");
    // keepout_mask_topic：虚拟禁行区地图话题。订阅 nav_msgs/OccupancyGrid，用于 keepout 避让。
    declare_parameter<std::string>("keepout_mask_topic", "/filter_mask");
    // control_frequency：控制循环频率，单位 Hz。默认 20 Hz，即每 0.05 秒计算一次速度。
    declare_parameter<double>("control_frequency", 20.0);

    // 路径跟随速度和前瞻参数。
    // linear_speed：普通路段目标线速度，单位 m/s。最终速度还会被转弯和避障逻辑缩放。
    declare_parameter<double>("linear_speed", 0.9);
    // channel_linear_speed：指定航点范围内的线速度，单位 m/s，用于窄通道/特殊区域单独调速。
    declare_parameter<double>("channel_linear_speed", 0.9);
    // channel_waypoint_ranges：使用 channel_linear_speed 的航点编号范围，如 "3-10,15,20-22"。
    declare_parameter<std::string>("channel_waypoint_ranges", "");
    // lookahead_distance：前瞻距离，单位米。越大越早朝远处点转向，路径更顺但可能切弯。
    declare_parameter<double>("lookahead_distance", 0.55);
    // enable_lookahead_waypoint_selection：是否允许从当前航点往后选一个更远的前瞻航点。
    declare_parameter<bool>("enable_lookahead_waypoint_selection", false);
    // enable_lookahead_progress_commit：前瞻选到后续航点时，是否把中间航点直接标记为通过。
    declare_parameter<bool>("enable_lookahead_progress_commit", true);
    // pass_radius：默认航点通过半径，单位米。进入该半径就认为当前点完成；YAML 单点可覆盖。
    declare_parameter<double>("pass_radius", 0.35);

    // 某些 launch 参数可能以字符串形式传入，这里允许动态类型，避免参数类型不一致时报错。
    rcl_interfaces::msg::ParameterDescriptor dynamic_parameter;
    dynamic_parameter.dynamic_typing = true;

    // 平滑 keepout 路径参数：在指定航点范围内，用 Catmull-Rom 曲线和 keepout clearance 选更平滑的追踪点。
    // enable_smooth_keepout_path：是否启用局部平滑目标点。开启后不会直接追航点，而是在曲线上选临时点。
    declare_parameter("enable_smooth_keepout_path", rclcpp::ParameterValue(false), dynamic_parameter);
    // smooth_path_min_waypoint：启用平滑路径的起始航点编号，从 1 开始。
    declare_parameter("smooth_path_min_waypoint", rclcpp::ParameterValue(4), dynamic_parameter);
    // smooth_path_max_waypoint：启用平滑路径的结束航点编号，从 1 开始。
    declare_parameter("smooth_path_max_waypoint", rclcpp::ParameterValue(9), dynamic_parameter);
    // smooth_path_samples_per_segment：每两个航点之间曲线采样数量；越大候选点越密，计算量也越大。
    declare_parameter("smooth_path_samples_per_segment", rclcpp::ParameterValue(10), dynamic_parameter);
    // smooth_path_keepout_clearance：候选目标点希望离 keepout 栅格的距离，单位米。
    declare_parameter("smooth_path_keepout_clearance", rclcpp::ParameterValue(0.22), dynamic_parameter);
    // smooth_path_keepout_weight：keepout 距离不足时的评分惩罚权重。越大越倾向远离禁行区。
    declare_parameter("smooth_path_keepout_weight", rclcpp::ParameterValue(0.9), dynamic_parameter);

    // 冲过头跳点参数：当下一个点明显更近时，允许跳过当前点。
    // enable_skip_overshot_waypoints：是否允许“冲过当前点但没进 pass_radius”时跳到下一个点。
    declare_parameter<bool>("enable_skip_overshot_waypoints", true);
    // overshot_waypoint_margin：下一个点至少比当前点近多少才判定为冲过，单位米。
    declare_parameter("overshot_waypoint_margin", rclcpp::ParameterValue(0.05), dynamic_parameter);
    // overshot_waypoint_max_distance：当前点距离超过该值时不允许跳点，避免大范围误跳，单位米。
    declare_parameter("overshot_waypoint_max_distance", rclcpp::ParameterValue(0.80), dynamic_parameter);
    // overshot_next_waypoint_max_distance：下一个点必须小于该距离才允许跳点，单位米。
    declare_parameter("overshot_next_waypoint_max_distance", rclcpp::ParameterValue(0.45), dynamic_parameter);
    // skip_overshot_min_waypoint：允许冲过头跳点的起始航点编号，从 1 开始。
    declare_parameter("skip_overshot_min_waypoint", rclcpp::ParameterValue(5), dynamic_parameter);
    // skip_overshot_max_waypoint：允许冲过头跳点的结束航点编号，从 1 开始。
    declare_parameter("skip_overshot_max_waypoint", rclcpp::ParameterValue(9), dynamic_parameter);

    // 终点判定、正向/倒车控制增益和速度缩放参数。
    // goal_tolerance：最后一个航点的到达容差，单位米。到终点小于该距离就停车并关闭节点。
    declare_parameter<double>("goal_tolerance", 0.25);
    // turn_angular_gain：正向行驶时巡线式角度误差转角速度的增益；越大转向越积极。
    declare_parameter<double>("turn_angular_gain", 1.4);
    // turn_min_speed_scale：正向大转角时的最低速度比例，防止一转弯就降到太慢。
    declare_parameter<double>("turn_min_speed_scale", 0.45);
    // reverse_angular_gain：倒车时巡线式角度误差转角速度的增益。
    declare_parameter<double>("reverse_angular_gain", 1.1);
    // reverse_min_speed_scale：倒车大转角时的最低速度比例。
    declare_parameter<double>("reverse_min_speed_scale", 0.30);
    // reverse_goal_lookahead_distance：倒车当前点使用的短前瞻距离，单位米，防止倒车提前切远点。
    declare_parameter<double>("reverse_goal_lookahead_distance", 0.25);
    // reverse_pass_radius：默认倒车点通过半径，单位米；YAML 中 reverse_pass_radius 可覆盖。
    declare_parameter<double>("reverse_pass_radius", 0.25);
    // enable_reverse_approach_nudge：是否在指定倒车点附近额外给一个角速度修正。
    declare_parameter("enable_reverse_approach_nudge", rclcpp::ParameterValue(true), dynamic_parameter);
    // reverse_approach_nudge_waypoint：需要倒车姿态微调的航点编号，从 1 开始。
    declare_parameter("reverse_approach_nudge_waypoint", rclcpp::ParameterValue(2), dynamic_parameter);
    // reverse_approach_nudge_distance：离该航点小于这个距离时开始微调，单位米。
    declare_parameter("reverse_approach_nudge_distance", rclcpp::ParameterValue(0.35), dynamic_parameter);
    // reverse_approach_nudge_angular_z：倒车微调角速度，单位 rad/s；正负号决定打角方向。
    declare_parameter("reverse_approach_nudge_angular_z", rclcpp::ParameterValue(-0.35), dynamic_parameter);

    // 真实雷达避障参数：减速、避让、急停和倒车脱困。
    // enable_obstacle_avoidance：是否启用基于 /scan 的真实障碍避让。
    declare_parameter<bool>("enable_obstacle_avoidance", true);
    // obstacle_enable_from_waypoint：从哪个航点编号开始启用避障；0 表示从一开始就启用。
    declare_parameter<int>("obstacle_enable_from_waypoint", 0);
    // obstacle_stop_distance：真实障碍急停距离，单位米。前方小于该值会停车或倒车脱困。
    declare_parameter<double>("obstacle_stop_distance", 0.22);
    // obstacle_slow_distance：真实障碍减速开始距离，单位米。小于该值会按距离缩放线速度。
    declare_parameter<double>("obstacle_slow_distance", 0.80);
    // obstacle_avoid_distance：真实障碍转向避让开始距离，单位米。小于该值会叠加角速度。
    declare_parameter<double>("obstacle_avoid_distance", 0.60);
    // min_obstacle_range：雷达点最小有效距离，单位米。更近的点按噪声忽略。
    declare_parameter<double>("min_obstacle_range", 0.10);
    // front_angle_deg：前方检测扇区半角，单位度。30 表示检测车头左右各 30 度。
    declare_parameter<double>("front_angle_deg", 30.0);
    // side_angle_deg：左右侧检测扇区角度，单位度，用来比较哪边更空。
    declare_parameter<double>("side_angle_deg", 90.0);
    // avoid_angular_z：避障叠加角速度大小，单位 rad/s。
    declare_parameter<double>("avoid_angular_z", 0.55);

    // keepout 虚拟障碍参数：读取 OccupancyGrid，把禁行区作为软/硬避障依据。
    // enable_keepout_avoidance：是否启用 keepout 虚拟障碍避让。
    declare_parameter("enable_keepout_avoidance", rclcpp::ParameterValue(false), dynamic_parameter);
    // keepout_occupied_threshold：OccupancyGrid 占用值阈值，达到该值的格子视为禁行，范围 1-100。
    declare_parameter("keepout_occupied_threshold", rclcpp::ParameterValue(50), dynamic_parameter);
    // keepout_stop_distance：虚拟障碍硬停距离，单位米；只有 enable_keepout_hard_stop=true 时才硬停。
    declare_parameter("keepout_stop_distance", rclcpp::ParameterValue(0.25), dynamic_parameter);
    // keepout_slow_distance：虚拟障碍减速开始距离，单位米。
    declare_parameter("keepout_slow_distance", rclcpp::ParameterValue(0.70), dynamic_parameter);
    // keepout_avoid_distance：虚拟障碍软转向避让开始距离，单位米。
    declare_parameter("keepout_avoid_distance", rclcpp::ParameterValue(0.55), dynamic_parameter);
    // keepout_side_sample_distance：判断左右哪边被 keepout 挡住时的侧向采样距离，单位米。
    declare_parameter("keepout_side_sample_distance", rclcpp::ParameterValue(0.35), dynamic_parameter);
    // enable_keepout_hard_stop：true 时 keepout_stop_distance 内直接停车；false 时主要做软减速/软避让。
    declare_parameter("enable_keepout_hard_stop", rclcpp::ParameterValue(false), dynamic_parameter);
    // keepout_min_speed_scale：keepout 软减速时的最低速度比例，范围会被限制到 0.05-1.0。
    declare_parameter("keepout_min_speed_scale", rclcpp::ParameterValue(0.45), dynamic_parameter);

    // 急停后倒车脱困参数。
    // backup_trigger_time：真实障碍急停持续多久后尝试倒车，单位秒；<=0 表示立即尝试。
    declare_parameter<double>("backup_trigger_time", 0.0);
    // backup_duration：一次倒车脱困动作持续多久，单位秒。
    declare_parameter<double>("backup_duration", 0.5);
    // backup_speed：倒车脱困线速度绝对值，单位 m/s；发布时会变成负 linear.x。
    declare_parameter<double>("backup_speed", 0.9);
    // backup_angular_z：倒车脱困时叠加的角速度绝对值，单位 rad/s。
    declare_parameter<double>("backup_angular_z", 0.35);
    // backup_stop_distance：后方障碍小于该距离时禁止倒车，单位米。
    declare_parameter<double>("backup_stop_distance", 0.18);

    // QR 路线选择参数：可根据二维码奇偶或指定数值选择顺/逆时针。
    // enable_qr_skip_first：第一次收到 QR 后，如果还在第 1 点，是否直接切到第 2 点。
    declare_parameter<bool>("enable_qr_skip_first", true);
    // qr_topic：QR 结果话题，消息类型 std_msgs/String。主线集成时通常可设为 /qr_code。
    declare_parameter<std::string>("qr_topic", "/qr_info");
    // route_direction：启动默认路线方向。auto 表示读 YAML 顶层 route_direction；也可写 clockwise/counterclockwise。
    declare_parameter<std::string>("route_direction", "auto");
    // enable_qr_direction_select：是否允许 QR 数字在运行中选择顺/逆时针路线。
    declare_parameter<bool>("enable_qr_direction_select", true);
    // use_qr_parity_direction：true 时奇数 QR=clockwise，偶数 QR=counterclockwise。
    declare_parameter<bool>("use_qr_parity_direction", true);
    // clockwise_qr_value：不用奇偶规则时，QR 数值等于它就选择顺时针。
    declare_parameter<int>("clockwise_qr_value", 1);
    // counterclockwise_qr_value：不用奇偶规则时，QR 数值等于它就选择逆时针。
    declare_parameter<int>("counterclockwise_qr_value", 2);

    // 图像分析/图生文触发参数：到指定航点时发布 Int32 触发消息。
    // enable_image_analysis_trigger：是否在经过指定航点时发布一次图像分析触发。
    declare_parameter<bool>("enable_image_analysis_trigger", true);
    // image_analysis_waypoint：触发图像分析的航点编号，从 1 开始。
    declare_parameter<int>("image_analysis_waypoint", 7);
    // image_analysis_trigger_topic：图像分析触发话题，消息类型 std_msgs/Int32；主线可设为 /sign4return。
    declare_parameter<std::string>("image_analysis_trigger_topic", "/person_trigger");

    // 读取基础配置。
    waypoints_file_ = get_parameter("waypoints_file").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();

    // 读取路径跟随速度和前瞻配置。
    linear_speed_ = get_parameter("linear_speed").as_double();
    channel_linear_speed_ = get_parameter("channel_linear_speed").as_double();
    channel_waypoint_ranges_ = parse_waypoint_ranges(get_parameter("channel_waypoint_ranges").as_string());
    lookahead_distance_ = get_parameter("lookahead_distance").as_double();
    enable_lookahead_waypoint_selection_ = as_bool(get_parameter("enable_lookahead_waypoint_selection"));
    enable_lookahead_progress_commit_ = as_bool(get_parameter("enable_lookahead_progress_commit"));
    pass_radius_ = get_parameter("pass_radius").as_double();

    // 读取平滑 keepout 路径配置，并对范围做基本保护。
    enable_smooth_keepout_path_ = as_bool(get_parameter("enable_smooth_keepout_path"));
    smooth_path_min_waypoint_ =
      std::max(1, as_int(get_parameter("smooth_path_min_waypoint"), 4));
    smooth_path_max_waypoint_ =
      std::max(smooth_path_min_waypoint_, as_int(get_parameter("smooth_path_max_waypoint"), 9));
    smooth_path_samples_per_segment_ =
      std::max(3, as_int(get_parameter("smooth_path_samples_per_segment"), 10));
    smooth_path_keepout_clearance_ =
      std::max(0.0, as_double(get_parameter("smooth_path_keepout_clearance"), 0.22));
    smooth_path_keepout_weight_ =
      std::max(0.0, as_double(get_parameter("smooth_path_keepout_weight"), 0.9));

    // 读取冲过头跳点配置。
    enable_skip_overshot_waypoints_ = as_bool(get_parameter("enable_skip_overshot_waypoints"));
    overshot_waypoint_margin_ =
      std::max(0.0, as_double(get_parameter("overshot_waypoint_margin"), 0.05));
    overshot_waypoint_max_distance_ =
      std::max(0.0, as_double(get_parameter("overshot_waypoint_max_distance"), 0.80));
    overshot_next_waypoint_max_distance_ =
      std::max(0.0, as_double(get_parameter("overshot_next_waypoint_max_distance"), 0.45));
    skip_overshot_min_waypoint_ =
      std::max(1, as_int(get_parameter("skip_overshot_min_waypoint"), 5));
    skip_overshot_max_waypoint_ =
      std::max(skip_overshot_min_waypoint_, as_int(get_parameter("skip_overshot_max_waypoint"), 9));

    // 读取正向、倒车和终点控制参数。
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    turn_angular_gain_ = get_parameter("turn_angular_gain").as_double();
    turn_min_speed_scale_ = get_parameter("turn_min_speed_scale").as_double();
    reverse_angular_gain_ = get_parameter("reverse_angular_gain").as_double();
    reverse_min_speed_scale_ = get_parameter("reverse_min_speed_scale").as_double();
    reverse_goal_lookahead_distance_ = get_parameter("reverse_goal_lookahead_distance").as_double();
    reverse_pass_radius_ = get_parameter("reverse_pass_radius").as_double();
    enable_reverse_approach_nudge_ = as_bool(get_parameter("enable_reverse_approach_nudge"));
    reverse_approach_nudge_waypoint_ =
      std::max(1, as_int(get_parameter("reverse_approach_nudge_waypoint"), 2));
    reverse_approach_nudge_distance_ =
      std::max(0.0, as_double(get_parameter("reverse_approach_nudge_distance"), 0.35));
    reverse_approach_nudge_angular_z_ =
      as_double(get_parameter("reverse_approach_nudge_angular_z"), -0.35);

    // 读取真实雷达避障参数；角度参数从 degree 转成 rad。
    enable_obstacle_avoidance_ = as_bool(get_parameter("enable_obstacle_avoidance"));
    obstacle_enable_from_waypoint_ = std::max(0, static_cast<int>(get_parameter("obstacle_enable_from_waypoint").as_int()));
    obstacle_stop_distance_ = get_parameter("obstacle_stop_distance").as_double();
    obstacle_slow_distance_ = get_parameter("obstacle_slow_distance").as_double();
    obstacle_avoid_distance_ = get_parameter("obstacle_avoid_distance").as_double();
    min_obstacle_range_ = get_parameter("min_obstacle_range").as_double();
    front_angle_ = get_parameter("front_angle_deg").as_double() * M_PI / 180.0;
    side_angle_ = get_parameter("side_angle_deg").as_double() * M_PI / 180.0;
    avoid_angular_z_ = get_parameter("avoid_angular_z").as_double();

    // 读取 keepout 虚拟障碍配置。
    enable_keepout_avoidance_ = as_bool(get_parameter("enable_keepout_avoidance"));
    keepout_occupied_threshold_ =
      std::max(1, std::min(100, as_int(get_parameter("keepout_occupied_threshold"), 50)));
    keepout_stop_distance_ =
      std::max(0.0, as_double(get_parameter("keepout_stop_distance"), 0.25));
    keepout_slow_distance_ =
      std::max(keepout_stop_distance_, as_double(get_parameter("keepout_slow_distance"), 0.70));
    keepout_avoid_distance_ =
      std::max(keepout_stop_distance_, as_double(get_parameter("keepout_avoid_distance"), 0.55));
    keepout_side_sample_distance_ =
      std::max(0.05, as_double(get_parameter("keepout_side_sample_distance"), 0.35));
    enable_keepout_hard_stop_ = as_bool(get_parameter("enable_keepout_hard_stop"));
    keepout_min_speed_scale_ = clamp_value(
      as_double(get_parameter("keepout_min_speed_scale"), 0.45), 0.05, 1.0);

    // 读取倒车脱困参数。
    backup_trigger_time_ = get_parameter("backup_trigger_time").as_double();
    backup_duration_ = get_parameter("backup_duration").as_double();
    backup_speed_ = get_parameter("backup_speed").as_double();
    backup_angular_z_ = get_parameter("backup_angular_z").as_double();
    backup_stop_distance_ = get_parameter("backup_stop_distance").as_double();

    // 读取 QR 路线选择参数。
    enable_qr_skip_first_ = as_bool(get_parameter("enable_qr_skip_first"));
    qr_topic_ = get_parameter("qr_topic").as_string();
    route_direction_ = to_lower(get_parameter("route_direction").as_string());
    enable_qr_direction_select_ = as_bool(get_parameter("enable_qr_direction_select"));
    use_qr_parity_direction_ = as_bool(get_parameter("use_qr_parity_direction"));
    clockwise_qr_value_ = static_cast<int>(get_parameter("clockwise_qr_value").as_int());
    counterclockwise_qr_value_ = static_cast<int>(get_parameter("counterclockwise_qr_value").as_int());

    // 读取图像分析触发参数；在当前主线集成中可把 topic 改成 /sign4return。
    enable_image_analysis_trigger_ = as_bool(get_parameter("enable_image_analysis_trigger"));
    image_analysis_waypoint_ = std::max(1, static_cast<int>(get_parameter("image_analysis_waypoint").as_int()));
    image_analysis_trigger_topic_ = get_parameter("image_analysis_trigger_topic").as_string();

    // 控制循环频率，单位 Hz。
    const double control_frequency = get_parameter("control_frequency").as_double();

    // 按 route_direction_ 从 YAML 中加载初始路线；auto 会读取 YAML 顶层 route_direction。
    waypoints_ = load_waypoints(route_direction_);

    // 发布速度命令，最终由主线底盘驱动订阅 /cmd_vel 执行。
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(get_parameter("cmd_vel_topic").as_string(), 10);

    // 发布图像分析触发。transient_local 让后启动的订阅者也能收到最后一次触发值。
    image_analysis_trigger_pub_ =
      create_publisher<std_msgs::msg::Int32>(image_analysis_trigger_topic_, rclcpp::QoS(10).transient_local());

    // 保存最近一帧雷达数据，控制循环中用于避障。
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      get_parameter("scan_topic").as_string(), 10,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { latest_scan_ = msg; });

    // 保存最近一张 keepout mask，占用栅格值用于虚拟障碍避让。
    keepout_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("keepout_mask_topic").as_string(), rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { latest_keepout_mask_ = msg; });

    // 只有需要 QR 选方向或扫到 QR 后跳过第一点时，才订阅 QR 话题。
    if (enable_qr_skip_first_ || enable_qr_direction_select_) {
      qr_sub_ = create_subscription<std_msgs::msg::String>(
        qr_topic_, 10, std::bind(&SmoothPathFollower::qr_callback, this, std::placeholders::_1));
    }

    if (waypoints_.empty()) {
      RCLCPP_ERROR(get_logger(), "No waypoints loaded; smooth path follower will stop.");
      publish_stop();
      return;
    }

    // 只用于启动日志，帮助确认倒车点数量和关键参数。
    const auto reverse_count = std::count_if(
      waypoints_.begin(), waypoints_.end(), [](const Waypoint & w) { return w.reverse; });
    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu waypoint(s) (%zu reverse), route=%s, lookahead=%.2fm, waypoint_lookahead=%s, pass_radius=%.2fm, outside_speed=%.2fm/s, channel_speed=%.2fm/s, obstacle_from_waypoint=%d, obstacle_avoidance=%s.",
      waypoints_.size(), static_cast<std::size_t>(reverse_count), selected_route_direction_.c_str(), lookahead_distance_,
      enable_lookahead_waypoint_selection_ ? "true" : "false", pass_radius_,
      linear_speed_, channel_linear_speed_, obstacle_enable_from_waypoint_,
      enable_obstacle_avoidance_ ? "true" : "false");

    // 定时控制循环：每次读取 TF、更新航点进度、算 cmd_vel、应用避障，然后发布速度。
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(control_frequency, 1.0)),
      std::bind(&SmoothPathFollower::control_once, this));
  }

  // 立即发布零速度。用于启动失败、定位暂不可用、到达终点或程序退出时停车。
  void publish_stop()
  {
    if (cmd_pub_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

private:
  // 解析航点范围字符串，例如 "3-10,15,20-22" -> [(3,10), (15,15), (20,22)]。
  // 这些范围用于 channel_linear_speed_，让某些航点段使用通道速度。
  std::vector<std::pair<int, int>> parse_waypoint_ranges(std::string value)
  {
    // 兼容中文逗号。
    for (std::size_t pos = value.find("，"); pos != std::string::npos; pos = value.find("，", pos + 1)) {
      value.replace(pos, std::string("，").size(), ",");
    }
    std::vector<std::pair<int, int>> ranges;
    std::size_t start_pos = 0;
    while (start_pos <= value.size()) {
      const auto comma = value.find(',', start_pos);
      std::string part = value.substr(start_pos, comma == std::string::npos ? std::string::npos : comma - start_pos);
      part.erase(std::remove_if(part.begin(), part.end(), ::isspace), part.end());
      if (!part.empty()) {
        try {
          int first = 0;
          int last = 0;
          const auto dash = part.find('-');
          if (dash == std::string::npos) {
            first = last = std::stoi(part);
          } else {
            first = std::stoi(part.substr(0, dash));
            last = std::stoi(part.substr(dash + 1));
          }
          if (first > last) {
            std::swap(first, last);
          }
          ranges.emplace_back(std::max(first, 1), std::max(last, 1));
        } catch (const std::exception &) {
          RCLCPP_WARN(get_logger(), "Ignoring invalid channel waypoint range: %s", part.c_str());
        }
      }
      if (comma == std::string::npos) {
        break;
      }
      start_pos = comma + 1;
    }
    return ranges;
  }

  // QR 回调：
  //   1. 根据 QR 数字判断顺/逆时针并切换路线；
  //   2. 如果 enable_qr_skip_first_ 为 true，第一次收到 QR 时直接跳到当前路线第 2 点。
  void qr_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string direction = direction_from_qr(msg->data);
    if (!direction.empty()) {
      switch_route(direction);
    }

    if (enable_qr_skip_first_) {
      skip_first_waypoint_from_qr(msg->data);
    }
  }

  // 将 QR 字符串转换为路线方向。
  // use_qr_parity_direction_=true 时：奇数=clockwise，偶数=counterclockwise。
  // 否则使用 clockwise_qr_value_ 和 counterclockwise_qr_value_ 做精确匹配。
  std::string direction_from_qr(const std::string & data) const
  {
    if (!enable_qr_direction_select_) {
      return "";
    }
    int value = 0;
    try {
      value = std::stoi(data);
    } catch (const std::exception &) {
      return "";
    }
    if (use_qr_parity_direction_) {
      return value % 2 == 0 ? "counterclockwise" : "clockwise";
    }
    if (value == clockwise_qr_value_) {
      return "clockwise";
    }
    if (value == counterclockwise_qr_value_) {
      return "counterclockwise";
    }
    return "";
  }

  // 切换路线。current_index_ 不清零，而是继续沿用当前进度，避免 QR 重复触发后回到起点。
  void switch_route(const std::string & direction)
  {
    if (direction == selected_route_direction_) {
      return;
    }

    const auto new_waypoints = load_waypoints(direction);
    if (new_waypoints.empty()) {
      RCLCPP_WARN(get_logger(), "QR selected route \"%s\", but no waypoints were loaded.", direction.c_str());
      return;
    }

    waypoints_ = new_waypoints;
    selected_route_direction_ = direction;
    current_index_ = std::min(current_index_, waypoints_.size() - 1);
    emergency_start_time_ = std::numeric_limits<double>::quiet_NaN();
    backup_until_time_ = std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(
      get_logger(), "QR selected %s route; continuing from waypoint %zu/%zu.",
      selected_route_direction_.c_str(), current_index_ + 1, waypoints_.size());
  }

  // 第一次收到 QR 时，如果还在第 1 个点，就直接把当前目标改为第 2 个点。
  // 这个逻辑常用于比赛中“扫码后开始倒车去 2 号点”的动作。
  void skip_first_waypoint_from_qr(const std::string & data)
  {
    if (qr_skip_done_) {
      return;
    }
    if (waypoints_.size() < 2) {
      qr_skip_done_ = true;
      RCLCPP_WARN(get_logger(), "QR skip ignored: fewer than 2 waypoints loaded.");
      return;
    }
    if (current_index_ == 0) {
      current_index_ = 1;
      qr_skip_done_ = true;
      emergency_start_time_ = std::numeric_limits<double>::quiet_NaN();
      backup_until_time_ = std::numeric_limits<double>::quiet_NaN();
      RCLCPP_INFO(
        get_logger(), "QR info received \"%s\"; switching directly to waypoint 2 (%s).",
        data.c_str(), waypoints_[current_index_].reverse ? "reverse" : "forward");
      return;
    }
    qr_skip_done_ = true;
    RCLCPP_INFO(get_logger(), "QR info received \"%s\", but waypoint 1 is already passed; no skip needed.", data.c_str());
  }

  // 从 YAML 文件加载航点。
  // 支持两种格式：
  //   1. routes.clockwise / routes.counterclockwise：当前 91 点文件使用这种；
  //   2. waypoints：没有路线分组时的兜底格式。
  std::vector<Waypoint> load_waypoints(const std::string & requested_direction)
  {
    std::vector<Waypoint> waypoints;
    std::string resolved_direction = requested_direction;
    if (waypoints_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter waypoints_file is empty.");
      return waypoints;
    }
    try {
      const YAML::Node data = YAML::LoadFile(waypoints_file_);
      std::string direction = to_lower(requested_direction);
      if (direction.empty() || direction == "auto") {
        // auto 时使用 YAML 顶层 route_direction；如果没有，就默认 counterclockwise。
        direction = data["route_direction"] ? to_lower(data["route_direction"].as<std::string>()) : "counterclockwise";
      }
      resolved_direction = direction;
      const YAML::Node routes = data["routes"];
      const YAML::Node items = routes && routes[direction] ? routes[direction] : data["waypoints"];
      if (!items || !items.IsSequence()) {
        return waypoints;
      }
      for (std::size_t i = 0; i < items.size(); ++i) {
        try {
          Waypoint w;
          // 当前跟随器不使用 YAML yaw；方向由当前位姿和目标点实时计算。
          w.x = items[i]["x"].as<double>();
          w.y = items[i]["y"].as<double>();
          w.reverse = yaml_bool(items[i]["reverse"], false);
          w.pass_radius = items[i]["pass_radius"] ? items[i]["pass_radius"].as<double>() : pass_radius_;
          w.reverse_pass_radius =
            items[i]["reverse_pass_radius"] ? items[i]["reverse_pass_radius"].as<double>() : reverse_pass_radius_;
          waypoints.push_back(w);
        } catch (const YAML::Exception & exc) {
          RCLCPP_WARN(get_logger(), "Skipping invalid waypoint #%zu: %s", i, exc.what());
        }
      }
    } catch (const YAML::Exception & exc) {
      RCLCPP_ERROR(get_logger(), "Failed to read waypoint YAML: %s", exc.what());
    }
    if (!waypoints.empty()) {
      selected_route_direction_ = resolved_direction;
    }
    return waypoints;
  }

  // 主控制循环。每个周期执行：
  //   1. 查询机器人在 map 下的位姿；
  //   2. 判断是否到终点；
  //   3. 更新已通过航点；
  //   4. 选择前瞻/平滑追踪目标；
  //   5. 用类似视觉巡线的目标角度误差计算线速度和角速度；
  //   6. 叠加真实障碍和 keepout 避障；
  //   7. 发布 /cmd_vel。
  void control_once()
  {
    // robot_x / robot_y / robot_yaw：机器人当前在 frame_id_ 坐标系下的位置和航向。
    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    if (!lookup_robot_pose(robot_x, robot_y, robot_yaw)) {
      publish_stop();
      return;
    }

    // 终点使用 goal_tolerance_ 判定，避免最后一个点因为 pass_radius 逻辑继续向后切。
    const auto & final = waypoints_.back();
    const double final_distance = std::hypot(final.x - robot_x, final.y - robot_y);
    if (current_index_ >= waypoints_.size() - 1 && final_distance <= goal_tolerance_) {
      RCLCPP_INFO(get_logger(), "Final waypoint reached; stopping.");
      publish_stop();
      rclcpp::shutdown();
      return;
    }

    // 更新航点进度，再选择实际追踪目标。
    advance_passed_waypoints(robot_x, robot_y);
    const std::size_t target_index = select_lookahead_waypoint(robot_x, robot_y);
    commit_lookahead_progress(target_index);
    const auto tracking_target = select_tracking_target(robot_x, robot_y, target_index);
    const double active_speed = active_linear_speed(target_index);

    // dx/dy：目标点相对机器人在 map 坐标系下的向量。
    const double dx = tracking_target.x - robot_x;
    const double dy = tracking_target.y - robot_y;

    // 将目标向量从 map 坐标系旋转到 base_link 坐标系。
    // target_x_base > 0 表示目标在车头前方，target_y_base 表示目标在车体左/右侧。
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    const double target_x_base = cos_yaw * dx + sin_yaw * dy;
    const double target_y_base = -sin_yaw * dx + cos_yaw * dy;
    const double distance = std::max(std::hypot(target_x_base, target_y_base), 0.001);

    // heading_error：车头朝向与目标方向的夹角；越大越需要减速转向。
    double heading_error = std::atan2(target_y_base, target_x_base);
    double speed_scale = clamp_value(1.0 - std::abs(heading_error) / 1.8, turn_min_speed_scale_, 1.0);
    if (target_x_base < 0.0) {
      // 正向行驶但目标落在车后方时强制降速，避免急转过猛。
      speed_scale = 0.25;
    }

    geometry_msgs::msg::Twist cmd;

    if (tracking_target.reverse) {
      // 倒车时目标方向按车尾方向处理，linear.x 为负。
      heading_error = std::atan2(-target_y_base, -target_x_base);
      speed_scale = clamp_value(1.0 - std::abs(heading_error) / 1.8, reverse_min_speed_scale_, 1.0);
      cmd.linear.x = -active_speed * speed_scale;
      // 巡线式转向：只看目标在车体系下形成的角度误差，不再使用曲率公式。
      cmd.angular.z = reverse_angular_gain_ * heading_error;
      cmd = apply_reverse_approach_nudge(cmd, target_index, distance);
    } else {
      cmd.linear.x = active_speed * speed_scale;
      // 巡线式转向：目标在左侧 heading_error 为正，目标在右侧 heading_error 为负。
      cmd.angular.z = turn_angular_gain_ * heading_error;
    }

    // 最后叠加避障逻辑，可能修改线速度、角速度，甚至改成倒车脱困命令。
    cmd = apply_obstacle_avoidance(cmd, tracking_target.reverse);
    cmd_pub_->publish(cmd);
  }

  // 倒车接近指定航点时额外打一小段方向，用来修正倒车入点姿态。
  geometry_msgs::msg::Twist apply_reverse_approach_nudge(
    geometry_msgs::msg::Twist cmd, std::size_t target_index, double target_distance) const
  {
    const int waypoint_number = static_cast<int>(target_index) + 1;
    if (!enable_reverse_approach_nudge_ || waypoint_number != reverse_approach_nudge_waypoint_) {
      return cmd;
    }
    if (target_distance > reverse_approach_nudge_distance_) {
      return cmd;
    }

    cmd.angular.z = reverse_approach_nudge_angular_z_;
    return cmd;
  }

  // 根据当前目标航点编号选择速度。
  // 如果航点编号落在 channel_waypoint_ranges_ 中，就使用 channel_linear_speed_。
  double active_linear_speed(std::size_t target_index)
  {
    const int waypoint_number = static_cast<int>(target_index) + 1;
    const bool in_channel = std::any_of(
      channel_waypoint_ranges_.begin(), channel_waypoint_ranges_.end(),
      [waypoint_number](const auto & range) { return range.first <= waypoint_number && waypoint_number <= range.second; });
    const std::string zone = in_channel ? "channel" : "outside";
    const double speed = in_channel ? channel_linear_speed_ : linear_speed_;
    if (zone != last_speed_zone_) {
      RCLCPP_INFO(get_logger(), "Speed zone: %s, waypoint %d, speed=%.2fm/s.", zone.c_str(), waypoint_number, speed);
      last_speed_zone_ = zone;
    }
    return speed;
  }

  // 从 TF 查询 frame_id_ -> base_frame_id_，默认 map -> base_link。
  // 返回 x/y/yaw，其中 yaw 是机器人在 map 坐标系下的航向角。
  bool lookup_robot_pose(double & x, double & y, double & yaw)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(frame_id_, base_frame_id_, tf2::TimePointZero);
      const auto & translation = transform.transform.translation;
      const auto & rotation = transform.transform.rotation;
      x = translation.x;
      y = translation.y;
      yaw = std::atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
      return true;
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN(get_logger(), "Waiting for TF %s->%s: %s", frame_id_.c_str(), base_frame_id_.c_str(), exc.what());
      return false;
    }
  }

  // 按 pass_radius 更新 current_index_。
  // 只要机器人进入当前航点通过半径，就认为该点完成并切到下一个点。
  void advance_passed_waypoints(double robot_x, double robot_y)
  {
    while (current_index_ < waypoints_.size() - 1) {
      const auto & waypoint = waypoints_[current_index_];
      const double distance = std::hypot(waypoint.x - robot_x, waypoint.y - robot_y);
      // 倒车点使用 reverse_pass_radius，但不会小于 reverse_goal_lookahead_distance_。
      const double pass_radius = waypoint.reverse ?
        std::max(waypoint.reverse_pass_radius, reverse_goal_lookahead_distance_) : waypoint.pass_radius;
      if (distance > pass_radius) {
        if (should_skip_overshot_waypoint(robot_x, robot_y, distance)) {
          const auto & next = waypoints_[current_index_ + 1];
          const double next_distance = std::hypot(next.x - robot_x, next.y - robot_y);
          RCLCPP_WARN(
            get_logger(),
            "Skipping overshot waypoint %zu/%zu: current_distance=%.2fm, next_distance=%.2fm.",
            current_index_ + 1, waypoints_.size(), distance, next_distance);
          trigger_image_analysis_if_needed(current_index_ + 1);
          ++current_index_;
          continue;
        }
        break;
      }
      RCLCPP_INFO(get_logger(), "Passing waypoint %zu/%zu at distance %.2fm.", current_index_ + 1, waypoints_.size(), distance);
      trigger_image_analysis_if_needed(current_index_ + 1);
      ++current_index_;
    }
  }

  // 判断机器人是否已经“冲过”当前航点。
  // 典型场景：车因为速度/避障绕行没进入当前点 pass_radius，但已经离下一个点更近。
  // 返回 true 时 advance_passed_waypoints() 会跳过当前点，避免车辆回头找点。
  bool should_skip_overshot_waypoint(double robot_x, double robot_y, double current_distance) const
  {
    // 没开功能，或当前点已经是最后一个点，就不能跳。
    if (!enable_skip_overshot_waypoints_ || current_index_ + 1 >= waypoints_.size()) {
      return false;
    }

    // 只允许在配置的航点编号范围内跳点，避免普通路段误跳。
    const int waypoint_number = static_cast<int>(current_index_) + 1;
    if (waypoint_number < skip_overshot_min_waypoint_ || waypoint_number > skip_overshot_max_waypoint_) {
      return false;
    }

    // 当前点太远时不认为是“刚刚错过”，否则可能把路径大段跳掉。
    if (current_distance > overshot_waypoint_max_distance_) {
      return false;
    }

    // next_distance：机器人到下一个航点的距离。
    // 只有下一个点足够近，并且比当前点近 overshot_waypoint_margin_ 以上，才认为已经冲过当前点。
    const auto & next = waypoints_[current_index_ + 1];
    const double next_distance = std::hypot(next.x - robot_x, next.y - robot_y);
    if (next_distance > overshot_next_waypoint_max_distance_) {
      return false;
    }
    return next_distance + overshot_waypoint_margin_ < current_distance;
  }

  // 到达指定航点时发布一次图像分析触发消息。
  // waypoint_number 是人看的航点编号，从 1 开始；current_index_ 是数组下标，从 0 开始。
  void trigger_image_analysis_if_needed(std::size_t waypoint_number)
  {
    // 只触发一次，避免机器人在同一个航点附近反复发布。
    if (!enable_image_analysis_trigger_ || image_analysis_triggered_) {
      return;
    }
    if (waypoint_number != static_cast<std::size_t>(image_analysis_waypoint_)) {
      return;
    }

    std_msgs::msg::Int32 msg;
    msg.data = image_analysis_waypoint_;
    image_analysis_trigger_pub_->publish(msg);
    image_analysis_triggered_ = true;
    RCLCPP_INFO(
      get_logger(), "Reached waypoint %d; triggered Aliyun image analysis on %s.",
      image_analysis_waypoint_, image_analysis_trigger_topic_.c_str());
  }

  // 选择本周期要追踪的航点下标。
  // 如果不开前瞻，就追当前 current_index_。
  // 如果开前瞻，就从 current_index_ 往后找第一个距离达到 lookahead_distance_ 的点，
  // 让车提前朝远一点的点转向，从而减少折线感。
  std::size_t select_lookahead_waypoint(double robot_x, double robot_y) const
  {
    if (!enable_lookahead_waypoint_selection_) {
      return current_index_;
    }

    // 精准点需要真正贴近，例如起步点、二维码后倒车点和终点，不做前瞻跳过。
    if (current_index_ < waypoints_.size() && is_precision_waypoint(current_index_)) {
      return current_index_;
    }

    // target_index：当前控制周期最终选中的追踪航点下标。
    std::size_t target_index = current_index_;
    for (std::size_t index = current_index_; index < waypoints_.size(); ++index) {
      const auto & waypoint = waypoints_[index];
      const double distance = std::hypot(waypoint.x - robot_x, waypoint.y - robot_y);

      // lookahead：本点使用的前瞻距离。倒车当前点会用更短的前瞻，避免倒车入点时过早切远点。
      double lookahead = lookahead_distance_;
      if (waypoint.reverse && index == current_index_) {
        lookahead = std::min(lookahead_distance_, reverse_goal_lookahead_distance_);
      }

      // 前方如果遇到精准点，直接把它作为目标，让车辆认真到位。
      if (index > current_index_ && is_precision_waypoint(index)) {
        target_index = index;
        break;
      }

      // 找到距离满足前瞻长度的点后停止搜索。
      if (distance >= lookahead) {
        target_index = index;
        break;
      }

      // 如果后续点也都还没达到前瞻距离，就至少把 target_index 推到当前遍历点。
      target_index = index;
    }
    return target_index;
  }

  // 哪些点必须精确通过，不允许被前瞻逻辑提前跳过。
  // 当前策略：前两个点和最后一个点为精准点。
  bool is_precision_waypoint(std::size_t index) const
  {
    if (index >= waypoints_.size()) {
      return false;
    }
    return index < 2 || index == waypoints_.size() - 1;
  }

  // 当前瞻目标已经超过 current_index_ 时，把中间航点直接标记为完成。
  // 这会让“看远一点”的目标也推进路线进度，否则车辆会一直被旧 current_index_ 拉回。
  void commit_lookahead_progress(std::size_t target_index)
  {
    if (!enable_lookahead_progress_commit_ || target_index <= current_index_) {
      return;
    }

    target_index = std::min(target_index, waypoints_.size() - 1);
    while (current_index_ < target_index) {
      RCLCPP_INFO(
        get_logger(), "Lookahead committed waypoint %zu/%zu; continuing toward waypoint %zu/%zu.",
        current_index_ + 1, waypoints_.size(), target_index + 1, waypoints_.size());
      trigger_image_analysis_if_needed(current_index_ + 1);
      ++current_index_;
    }
  }

  // 选择真正用于转向控制的目标点。
  // 默认直接返回目标航点；如果开启平滑 keepout 路径，则会在航点之间的曲线上采样，
  // 并选择“距离合适、离 keepout 较远”的临时目标点。
  TrackingTarget select_tracking_target(double robot_x, double robot_y, std::size_t target_index) const
  {
    const auto & target = waypoints_[target_index];
    TrackingTarget result{target.x, target.y, target.reverse};

    // 倒车点不做平滑采样；航点太少也无法构造 Catmull-Rom 曲线。
    if (!enable_smooth_keepout_path_ || target.reverse || waypoints_.size() < 4) {
      return result;
    }

    // 只在配置的航点编号范围内启用平滑路径，常用于狭窄/禁行区附近的局部优化。
    const int waypoint_number = static_cast<int>(target_index) + 1;
    if (waypoint_number < smooth_path_min_waypoint_ || waypoint_number > smooth_path_max_waypoint_) {
      return result;
    }

    // best_score：当前找到的最佳候选点评分，越小越好。
    // desired_distance：希望候选点离机器人多远，避免目标点太近造成抖动。
    double best_score = std::numeric_limits<double>::infinity();
    TrackingTarget best = result;
    const double desired_distance = std::max(lookahead_distance_, 0.20);

    // max_segment：只采样到 smooth_path_max_waypoint_ 对应的局部范围，不扫描整条 91 点路径。
    const std::size_t max_segment =
      std::min<std::size_t>(waypoints_.size() - 2, static_cast<std::size_t>(smooth_path_max_waypoint_ - 1));

    for (std::size_t segment = current_index_; segment <= max_segment; ++segment) {
      for (int sample = 1; sample <= smooth_path_samples_per_segment_; ++sample) {
        // t 是当前线段内的插值比例，0 到 1；point 是曲线采样点，next_point 用于估计切线方向。
        const double t = static_cast<double>(sample) / static_cast<double>(smooth_path_samples_per_segment_);
        const auto point = catmull_rom_point(segment, t);
        const auto next_point = catmull_rom_point(segment, std::min(1.0, t + 0.05));
        const double distance = std::hypot(point.first - robot_x, point.second - robot_y);
        if (distance < desired_distance * 0.55) {
          continue;
        }

        // tangent 是曲线切线；normal 是切线的法向，用来在曲线左右少量偏移候选点。
        const double tangent_x = next_point.first - point.first;
        const double tangent_y = next_point.second - point.second;
        const double tangent_norm = std::max(std::hypot(tangent_x, tangent_y), 0.001);
        const double normal_x = -tangent_y / tangent_norm;
        const double normal_y = tangent_x / tangent_norm;

        // offsets：在曲线中心线、左偏、右偏各试一个候选点，选 keepout clearance 更好的那个。
        const double offsets[] = {0.0, smooth_path_keepout_clearance_ * 0.5, -smooth_path_keepout_clearance_ * 0.5};
        for (const double offset : offsets) {
          const double candidate_x = point.first + normal_x * offset;
          const double candidate_y = point.second + normal_y * offset;
          const double candidate_distance = std::hypot(candidate_x - robot_x, candidate_y - robot_y);

          // clearance：候选点到 keepout 占用栅格的近似距离。
          // keepout_penalty：如果 clearance 小于期望安全距离，就给评分加惩罚。
          const double clearance = keepout_clearance_distance(
            candidate_x, candidate_y, std::max(smooth_path_keepout_clearance_, 0.01));
          const double keepout_penalty = std::max(0.0, smooth_path_keepout_clearance_ - clearance);
          const double offset_penalty = std::abs(offset) * 0.25;

          // score 由三部分组成：前瞻距离误差、keepout 太近惩罚、偏离中心线惩罚。
          const double score =
            std::abs(candidate_distance - desired_distance) +
            keepout_penalty * smooth_path_keepout_weight_ +
            offset_penalty;
          if (score < best_score) {
            best_score = score;
            best.x = candidate_x;
            best.y = candidate_y;
            best.reverse = false;
          }
        }
      }
    }

    return best;
  }

  // Catmull-Rom 曲线插值。
  // segment 表示从 waypoints_[segment] 到 waypoints_[segment+1] 这一段；
  // t 为段内比例 [0,1]。返回平滑曲线上的 map 坐标点。
  std::pair<double, double> catmull_rom_point(std::size_t segment, double t) const
  {
    // p0/p1/p2/p3 是曲线计算需要的四个控制点；边界处用端点重复，避免越界。
    const auto p0 = waypoint_point(segment == 0 ? 0 : segment - 1);
    const auto p1 = waypoint_point(segment);
    const auto p2 = waypoint_point(std::min(segment + 1, waypoints_.size() - 1));
    const auto p3 = waypoint_point(std::min(segment + 2, waypoints_.size() - 1));

    // t2/t3 是 t 的二次和三次项，Catmull-Rom 公式使用三次多项式。
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double x = 0.5 * (
      2.0 * p1.first +
      (-p0.first + p2.first) * t +
      (2.0 * p0.first - 5.0 * p1.first + 4.0 * p2.first - p3.first) * t2 +
      (-p0.first + 3.0 * p1.first - 3.0 * p2.first + p3.first) * t3);
    const double y = 0.5 * (
      2.0 * p1.second +
      (-p0.second + p2.second) * t +
      (2.0 * p0.second - 5.0 * p1.second + 4.0 * p2.second - p3.second) * t2 +
      (-p0.second + 3.0 * p1.second - 3.0 * p2.second + p3.second) * t3);
    return {x, y};
  }

  // 安全读取某个航点的 x/y，index 超过范围时钳制到最后一个点。
  std::pair<double, double> waypoint_point(std::size_t index) const
  {
    const auto & waypoint = waypoints_[std::min(index, waypoints_.size() - 1)];
    return {waypoint.x, waypoint.y};
  }

  // 叠加真实雷达避障和 keepout 虚拟障碍避让。
  // 输入 cmd 是路径跟随算出的原始速度；返回值可能被减速、转向、停车或替换成倒车命令。
  // reverse 表示当前路径点是否倒车，倒车时“前方雷达扇区”会切到车尾方向。
  geometry_msgs::msg::Twist apply_obstacle_avoidance(geometry_msgs::msg::Twist cmd, bool reverse)
  {
    if (!enable_obstacle_avoidance_) {
      return cmd;
    }

    // obstacle_enable_from_waypoint_ 允许前几个点不启用避障，常用于起步/扫码等特殊动作。
    if (static_cast<int>(current_index_) + 1 < obstacle_enable_from_waypoint_) {
      return cmd;
    }

    // 先处理 keepout mask，因为它是地图上的虚拟禁行区，不依赖激光雷达是否扫到。
    cmd = apply_keepout_avoidance(cmd, reverse);

    if (!latest_scan_) {
      return cmd;
    }

    // now：当前 ROS 时间，单位秒，用于计算急停持续多久后进入倒车脱困。
    const double now = this->now().seconds();

    // front_min / left_min / right_min：对应运动方向前方、左侧、右侧扇区内最近障碍距离。
    // 没有有效雷达点时为 infinity。
    double front_min = 0.0;
    double left_min = 0.0;
    double right_min = 0.0;
    if (reverse) {
      // 倒车时，车尾方向才是“运动前方”，所以采样角度移动到 pi 附近。
      front_min = min_range(M_PI - front_angle_, M_PI + front_angle_);
      left_min = min_range(M_PI, M_PI + side_angle_);
      right_min = min_range(M_PI - side_angle_, M_PI);
    } else {
      // 正向时，0 度附近是车头前方，正角度为左侧，负角度为右侧。
      front_min = min_range(-front_angle_, front_angle_);
      left_min = min_range(0.0, side_angle_);
      right_min = min_range(-side_angle_, 0.0);
    }

    // 如果前方完全没有有效雷达点，就清掉急停/倒车状态，沿用原始跟随速度。
    if (!std::isfinite(front_min)) {
      emergency_start_time_ = std::numeric_limits<double>::quiet_NaN();
      backup_until_time_ = std::numeric_limits<double>::quiet_NaN();
      return cmd;
    }

    // 如果已经进入倒车脱困窗口，则在 backup_until_time_ 前持续发布倒车命令。
    if (std::isfinite(backup_until_time_)) {
      if (now < backup_until_time_) {
        return backup_cmd();
      }
      // 倒车窗口结束后清状态，下一周期重新按雷达距离判断。
      backup_until_time_ = std::numeric_limits<double>::quiet_NaN();
      emergency_start_time_ = std::numeric_limits<double>::quiet_NaN();
    }

    // 急停区：前方障碍小于 obstacle_stop_distance_。
    // 处理优先级：能立即倒车就倒车；不能倒车就停车并朝更空的一侧转。
    if (front_min <= obstacle_stop_distance_) {
      const double avoid_direction = avoid_direction_from_ranges(left_min, right_min);
      if (backup_trigger_time_ <= 0.0 && rear_is_clear()) {
        backup_until_time_ = now + backup_duration_;
        backup_turn_direction_ = avoid_direction;
        log_obstacle_warning("Obstacle emergency at " + std::to_string(front_min) + "m; backing up.");
        return backup_cmd();
      }
      if (!std::isfinite(emergency_start_time_)) {
        emergency_start_time_ = now;
      } else if (now - emergency_start_time_ >= backup_trigger_time_) {
        if (rear_is_clear()) {
          backup_until_time_ = now + backup_duration_;
          backup_turn_direction_ = avoid_direction;
          log_obstacle_warning("Obstacle stuck at " + std::to_string(front_min) + "m; backing up.");
          return backup_cmd();
        }
        emergency_start_time_ = now;
      }

      // 后方不安全或未达到倒车触发时间时，原地停下，并保留一点转向避让趋势。
      cmd.linear.x = 0.0;
      cmd.angular.z = avoid_direction * avoid_angular_z_;
      log_obstacle_warning("Obstacle emergency stop: " + std::to_string(front_min) + "m.");
      return cmd;
    }

    // 离开急停区后重置急停计时。
    emergency_start_time_ = std::numeric_limits<double>::quiet_NaN();

    // 减速区：距离越接近 stop_distance，速度缩放越小。
    if (front_min <= obstacle_slow_distance_) {
      const double slow_scale = clamp_value(
        (front_min - obstacle_stop_distance_) /
          std::max(obstacle_slow_distance_ - obstacle_stop_distance_, 0.01),
        0.20, 1.0);
      cmd.linear.x *= slow_scale;
    }

    // 避让区：在原始跟随角速度上叠加朝空旷侧转的角速度。
    if (front_min <= obstacle_avoid_distance_) {
      const double avoid_direction = avoid_direction_from_ranges(left_min, right_min);
      const double avoid_gain = clamp_value(
        (obstacle_avoid_distance_ - front_min) /
          std::max(obstacle_avoid_distance_ - obstacle_stop_distance_, 0.01),
        0.0, 1.0);
      cmd.angular.z += avoid_direction * avoid_angular_z_ * avoid_gain;
      log_obstacle_warning(
        "Obstacle avoid: front=" + std::to_string(front_min) + "m, left=" +
        std::to_string(left_min) + "m, right=" + std::to_string(right_min) + "m.");
    }
    return cmd;
  }

  // keepout 虚拟障碍避让。
  // latest_keepout_mask_ 是 OccupancyGrid，占用值超过 keepout_occupied_threshold_ 的格子视为禁行区。
  // 这段逻辑会沿运动方向向前采样，检测最近虚拟障碍距离，再做减速/转向/硬停。
  geometry_msgs::msg::Twist apply_keepout_avoidance(geometry_msgs::msg::Twist cmd, bool reverse)
  {
    if (!enable_keepout_avoidance_ || !latest_keepout_mask_) {
      return cmd;
    }

    // keepout 需要知道机器人当前位置和朝向，才能在地图坐标下向前采样。
    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    if (!lookup_robot_pose(robot_x, robot_y, robot_yaw)) {
      return cmd;
    }

    // direction 用于修正倒车时的角速度符号。
    // heading 是机器人实际运动方向：正向为车头方向，倒车为车尾方向。
    const double direction = reverse ? -1.0 : 1.0;
    const double heading = normalize_angle(robot_yaw + (reverse ? M_PI : 0.0));
    const double front_min = keepout_distance_along(robot_x, robot_y, heading, keepout_slow_distance_);
    if (!std::isfinite(front_min)) {
      return cmd;
    }

    // 在运动方向左右两侧各采一个点，判断哪边被 keepout 挡住，用于决定避让方向。
    const double left_heading = normalize_angle(heading + M_PI / 2.0);
    const double right_heading = normalize_angle(heading - M_PI / 2.0);
    const bool left_blocked = keepout_cell_occupied(
      robot_x + std::cos(left_heading) * keepout_side_sample_distance_,
      robot_y + std::sin(left_heading) * keepout_side_sample_distance_);
    const bool right_blocked = keepout_cell_occupied(
      robot_x + std::cos(right_heading) * keepout_side_sample_distance_,
      robot_y + std::sin(right_heading) * keepout_side_sample_distance_);
    const double avoid_direction = right_blocked && !left_blocked ? 1.0 : -1.0;

    // 硬停模式：虚拟障碍进入 keepout_stop_distance_ 就直接停止线速度。
    if (enable_keepout_hard_stop_ && front_min <= keepout_stop_distance_) {
      cmd.linear.x = 0.0;
      cmd.angular.z = avoid_direction * avoid_angular_z_ * direction;
      log_obstacle_warning("Keepout stop: virtual obstacle at " + std::to_string(front_min) + "m.");
      return cmd;
    }

    // 软减速：越靠近 keepout_stop_distance_，线速度越低，但不低于 keepout_min_speed_scale_。
    const double slow_scale = clamp_value(
      (front_min - keepout_stop_distance_) /
        std::max(keepout_slow_distance_ - keepout_stop_distance_, 0.01),
      keepout_min_speed_scale_, 1.0);
    cmd.linear.x *= slow_scale;

    // 软避让：在角速度上叠加一个偏离 keepout 的转向量。
    if (front_min <= keepout_avoid_distance_) {
      const double avoid_gain = clamp_value(
        (keepout_avoid_distance_ - front_min) /
          std::max(keepout_avoid_distance_ - keepout_stop_distance_, 0.01),
        0.0, 1.0);
      cmd.angular.z += avoid_direction * avoid_angular_z_ * avoid_gain * direction;
      log_obstacle_warning("Keepout soft avoid: virtual obstacle at " + std::to_string(front_min) + "m.");
    }

    return cmd;
  }

  // 从 origin 沿 heading 方向向前采样 OccupancyGrid，返回第一个 occupied cell 的距离。
  // 如果 max_distance 内没有碰到 keepout，返回 infinity。
  double keepout_distance_along(double origin_x, double origin_y, double heading, double max_distance) const
  {
    // step 是采样步长：优先跟地图分辨率相关，最低 2 cm，避免漏扫窄格子。
    const double step = latest_keepout_mask_->info.resolution > 0.0 ?
      std::max(0.02, static_cast<double>(latest_keepout_mask_->info.resolution) * 0.5) : 0.03;
    for (double distance = 0.0; distance <= max_distance; distance += step) {
      const double x = origin_x + std::cos(heading) * distance;
      const double y = origin_y + std::sin(heading) * distance;
      if (keepout_cell_occupied(x, y)) {
        return distance;
      }
    }
    return std::numeric_limits<double>::infinity();
  }

  // 估算 world_x/world_y 到最近 keepout 占用栅格的距离，最大只查 max_distance。
  // 用于平滑路径候选点评分：离禁行区越远，候选点越好。
  double keepout_clearance_distance(double world_x, double world_y, double max_distance) const
  {
    if (!latest_keepout_mask_ || max_distance <= 0.0) {
      return max_distance;
    }
    if (keepout_cell_occupied(world_x, world_y)) {
      return 0.0;
    }

    const auto & map = *latest_keepout_mask_;
    const double resolution = map.info.resolution > 0.0 ? map.info.resolution : 0.05;
    const double step = std::max(0.02, resolution);

    // 从小半径到大半径画圆采样，第一次碰到 occupied cell 的半径就是近似 clearance。
    for (double radius = step; radius <= max_distance; radius += step) {
      const int samples = std::max(8, static_cast<int>(std::ceil(2.0 * M_PI * radius / step)));
      for (int i = 0; i < samples; ++i) {
        const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(samples);
        const double x = world_x + std::cos(angle) * radius;
        const double y = world_y + std::sin(angle) * radius;
        if (keepout_cell_occupied(x, y)) {
          return radius;
        }
      }
    }
    return max_distance;
  }

  // 判断 world 坐标下某个点是否落在 keepout 占用栅格里。
  // 坐标换算：world 坐标 -> map origin 偏移 -> resolution 转 cell 下标 -> data 一维数组。
  bool keepout_cell_occupied(double world_x, double world_y) const
  {
    if (!latest_keepout_mask_) {
      return false;
    }
    const auto & map = *latest_keepout_mask_;
    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;
    const double resolution = map.info.resolution;
    if (resolution <= 0.0) {
      return false;
    }

    // cell_x/cell_y：该 world 点对应的栅格坐标。
    const int cell_x = static_cast<int>(std::floor((world_x - origin_x) / resolution));
    const int cell_y = static_cast<int>(std::floor((world_y - origin_y) / resolution));
    if (cell_x < 0 || cell_y < 0 ||
      cell_x >= static_cast<int>(map.info.width) || cell_y >= static_cast<int>(map.info.height))
    {
      return false;
    }

    // OccupancyGrid data 是按行展开的一维数组，index = y * width + x。
    const auto index = static_cast<std::size_t>(cell_y) * map.info.width + static_cast<std::size_t>(cell_x);
    if (index >= map.data.size()) {
      return false;
    }
    return map.data[index] >= keepout_occupied_threshold_;
  }

  // 生成倒车脱困速度命令。
  // linear.x 为负；angular.z 的方向由进入倒车前的左右空旷程度决定。
  geometry_msgs::msg::Twist backup_cmd() const
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = -std::abs(backup_speed_);
    cmd.angular.z = backup_turn_direction_ * backup_angular_z_;
    return cmd;
  }

  // 检查车后方是否足够安全，只有后方最近障碍大于 backup_stop_distance_ 才允许倒车。
  bool rear_is_clear()
  {
    // LaserScan 的 pi 和 -pi 附近共同组成后方扇区，所以分成左右两段取最小值。
    const double rear_left = min_range(M_PI - front_angle_, M_PI);
    const double rear_right = min_range(-M_PI, -M_PI + front_angle_);
    const double rear_min = std::min(rear_left, rear_right);
    return !std::isfinite(rear_min) || rear_min > backup_stop_distance_;
  }

  // 根据左右最近障碍距离选择避让方向。
  // 返回值会乘到 avoid_angular_z_ 上：1.0 叠加正角速度，-1.0 叠加负角速度。
  // 注意这里保留当前代码原有策略；若实车左右转向不符合预期，优先核查这几个分支。
  double avoid_direction_from_ranges(double left_min, double right_min) const
  {
    // 两边都没看到障碍时默认正角速度方向。
    if (!std::isfinite(left_min) && !std::isfinite(right_min)) {
      return 1.0;
    }
    // 只有左侧没有有效点时，按原策略返回负角速度方向。
    if (!std::isfinite(left_min)) {
      return -1.0;
    }
    // 只有右侧没有有效点时，按原策略返回正角速度方向。
    if (!std::isfinite(right_min)) {
      return 1.0;
    }
    // 两边都有读数时，左侧距离更大返回正角速度，否则返回负角速度。
    return left_min >= right_min ? 1.0 : -1.0;
  }

  // 读取 LaserScan 在 [min_angle, max_angle] 扇区内的最近有效距离。
  // 角度单位是弧度，0 表示车头方向；返回 infinity 表示该扇区没有有效点。
  double min_range(double min_angle, double max_angle) const
  {
    if (!latest_scan_) {
      return std::numeric_limits<double>::infinity();
    }
    const auto & scan = *latest_scan_;
    double result = std::numeric_limits<double>::infinity();
    double angle = scan.angle_min;

    // 小于雷达自身 range_min 或 min_obstacle_range_ 的点都忽略，过滤近距离噪声。
    const double min_valid_range = std::max(static_cast<double>(scan.range_min), min_obstacle_range_);
    min_angle = normalize_angle(min_angle);
    max_angle = normalize_angle(max_angle);
    for (const auto value : scan.ranges) {
      const double scan_angle = normalize_angle(angle);

      // 扇区可能跨越 -pi/pi 边界；跨界时用 “>= min 或 <= max” 判断。
      const bool in_sector = min_angle <= max_angle ?
        (min_angle <= scan_angle && scan_angle <= max_angle) :
        (scan_angle >= min_angle || scan_angle <= max_angle);
      if (in_sector && std::isfinite(value) && min_valid_range <= value && value <= scan.range_max) {
        result = std::min(result, static_cast<double>(value));
      }
      angle += scan.angle_increment;
    }
    return result;
  }

  // 避障日志节流。控制循环频率较高，直接每周期 WARN 会刷屏，所以每 10 次打印 1 次。
  void log_obstacle_warning(const std::string & message)
  {
    ++obstacle_warning_count_;
    if (obstacle_warning_count_ % 10 == 1) {
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }
  }

  // waypoints_file_：航点 YAML 文件绝对路径，由参数 waypoints_file 传入。
  std::string waypoints_file_;
  // frame_id_：全局路径坐标系，默认 map。
  std::string frame_id_;
  // base_frame_id_：机器人本体坐标系，默认 base_link。
  std::string base_frame_id_;
  // qr_topic_：二维码识别结果话题，消息类型 std_msgs/String。
  std::string qr_topic_;
  // route_direction_：启动时请求的方向，取值 auto/clockwise/counterclockwise。
  std::string route_direction_;
  // selected_route_direction_：当前实际已加载的方向，用于日志和防止重复切换。
  std::string selected_route_direction_;
  // clockwise_qr_value_：不用奇偶规则时，匹配该数值选择顺时针。
  int clockwise_qr_value_{1};
  // counterclockwise_qr_value_：不用奇偶规则时，匹配该数值选择逆时针。
  int counterclockwise_qr_value_{2};

  // linear_speed_：普通路段线速度，单位 m/s。
  double linear_speed_{0.9};
  // channel_linear_speed_：指定航点范围内使用的通道线速度，单位 m/s。
  double channel_linear_speed_{0.45};
  // channel_waypoint_ranges_：通道速度航点范围，例如 (3,10) 表示 3 到 10 号点。
  std::vector<std::pair<int, int>> channel_waypoint_ranges_;
  // lookahead_distance_：前瞻目标距离，单位米，越大路径越顺但越容易切弯。
  double lookahead_distance_{0.55};
  // enable_lookahead_waypoint_selection_：是否允许从 current_index_ 往后挑更远的追踪航点。
  bool enable_lookahead_waypoint_selection_{false};
  // enable_lookahead_progress_commit_：前瞻选到后方点时，是否同步推进 current_index_。
  bool enable_lookahead_progress_commit_{true};
  // pass_radius_：默认航点通过半径，单位米；YAML 单点配置可覆盖它。
  double pass_radius_{0.35};

  // enable_smooth_keepout_path_：是否启用 Catmull-Rom 平滑路径和 keepout clearance 选点。
  bool enable_smooth_keepout_path_{false};
  // smooth_path_min_waypoint_：平滑路径启用的起始航点编号，从 1 开始。
  int smooth_path_min_waypoint_{4};
  // smooth_path_max_waypoint_：平滑路径启用的结束航点编号，从 1 开始。
  int smooth_path_max_waypoint_{9};
  // smooth_path_samples_per_segment_：每个航点段上的曲线采样数量。
  int smooth_path_samples_per_segment_{10};
  // smooth_path_keepout_clearance_：希望平滑候选点离 keepout 的最小距离，单位米。
  double smooth_path_keepout_clearance_{0.22};
  // smooth_path_keepout_weight_：keepout 距离不足时在评分里的惩罚权重。
  double smooth_path_keepout_weight_{0.9};

  // enable_skip_overshot_waypoints_：是否允许检测到冲过头时跳过当前航点。
  bool enable_skip_overshot_waypoints_{true};
  // overshot_waypoint_margin_：下一个点至少要比当前点近多少才认为冲过，单位米。
  double overshot_waypoint_margin_{0.05};
  // overshot_waypoint_max_distance_：当前点距离超过该值时不做冲过头跳点，单位米。
  double overshot_waypoint_max_distance_{0.80};
  // overshot_next_waypoint_max_distance_：下一个点必须小于该距离才允许跳点，单位米。
  double overshot_next_waypoint_max_distance_{0.45};
  // skip_overshot_min_waypoint_：允许冲过头跳点的起始航点编号，从 1 开始。
  int skip_overshot_min_waypoint_{5};
  // skip_overshot_max_waypoint_：允许冲过头跳点的结束航点编号，从 1 开始。
  int skip_overshot_max_waypoint_{9};

  // goal_tolerance_：最终点到达容差，单位米。
  double goal_tolerance_{0.25};
  // turn_angular_gain_：正向巡线式角度误差转角速度的增益。
  double turn_angular_gain_{1.4};
  // turn_min_speed_scale_：正向转弯时线速度最低缩放比例。
  double turn_min_speed_scale_{0.45};
  // reverse_angular_gain_：倒车巡线式角度误差转角速度的增益。
  double reverse_angular_gain_{1.1};
  // reverse_min_speed_scale_：倒车转向时线速度最低缩放比例。
  double reverse_min_speed_scale_{0.30};
  // reverse_goal_lookahead_distance_：倒车当前点使用的较短前瞻距离，单位米。
  double reverse_goal_lookahead_distance_{0.25};
  // reverse_pass_radius_：默认倒车点通过半径，单位米；YAML 单点配置可覆盖它。
  double reverse_pass_radius_{0.25};
  // enable_reverse_approach_nudge_：是否在指定倒车点附近额外叠加角速度修正。
  bool enable_reverse_approach_nudge_{true};
  // reverse_approach_nudge_waypoint_：需要倒车姿态修正的航点编号，从 1 开始。
  int reverse_approach_nudge_waypoint_{2};
  // reverse_approach_nudge_distance_：进入该距离后开始倒车姿态修正，单位米。
  double reverse_approach_nudge_distance_{0.35};
  // reverse_approach_nudge_angular_z_：倒车姿态修正角速度，单位 rad/s。
  double reverse_approach_nudge_angular_z_{-0.35};

  // enable_obstacle_avoidance_：是否启用真实雷达避障。
  bool enable_obstacle_avoidance_{true};
  // obstacle_enable_from_waypoint_：从哪个航点编号开始启用避障；0 表示一开始就启用。
  int obstacle_enable_from_waypoint_{0};
  // obstacle_stop_distance_：真实障碍急停距离，单位米。
  double obstacle_stop_distance_{0.22};
  // obstacle_slow_distance_：真实障碍减速开始距离，单位米。
  double obstacle_slow_distance_{0.80};
  // obstacle_avoid_distance_：真实障碍转向避让开始距离，单位米。
  double obstacle_avoid_distance_{0.60};
  // min_obstacle_range_：雷达点最小有效距离，低于该值认为是噪声，单位米。
  double min_obstacle_range_{0.10};
  // front_angle_：前方扇区半角，单位 rad；默认 30 度。
  double front_angle_{M_PI / 6.0};
  // side_angle_：左右侧扇区角度，单位 rad；默认 90 度。
  double side_angle_{M_PI / 2.0};
  // avoid_angular_z_：避障叠加角速度大小，单位 rad/s。
  double avoid_angular_z_{0.55};

  // enable_keepout_avoidance_：是否启用 keepout OccupancyGrid 虚拟障碍避让。
  bool enable_keepout_avoidance_{false};
  // keepout_occupied_threshold_：栅格占用值达到该阈值就视为 keepout，范围 1-100。
  int keepout_occupied_threshold_{50};
  // keepout_stop_distance_：虚拟障碍硬停距离，单位米。
  double keepout_stop_distance_{0.25};
  // keepout_slow_distance_：虚拟障碍减速开始距离，单位米。
  double keepout_slow_distance_{0.70};
  // keepout_avoid_distance_：虚拟障碍软转向避让开始距离，单位米。
  double keepout_avoid_distance_{0.55};
  // keepout_side_sample_distance_：检测左右哪边被 keepout 挡住时的侧向采样距离，单位米。
  double keepout_side_sample_distance_{0.35};
  // enable_keepout_hard_stop_：true 时 keepout_stop_distance_ 内直接停车。
  bool enable_keepout_hard_stop_{false};
  // keepout_min_speed_scale_：keepout 软减速时最低速度比例。
  double keepout_min_speed_scale_{0.45};

  // backup_trigger_time_：急停持续多久后尝试倒车脱困，单位秒；<=0 表示立即倒车。
  double backup_trigger_time_{0.0};
  // backup_duration_：一次倒车脱困持续时间，单位秒。
  double backup_duration_{0.5};
  // backup_speed_：倒车脱困线速度绝对值，单位 m/s。
  double backup_speed_{0.12};
  // backup_angular_z_：倒车脱困角速度绝对值，单位 rad/s。
  double backup_angular_z_{0.35};
  // backup_stop_distance_：后方小于该距离时禁止倒车，单位米。
  double backup_stop_distance_{0.18};

  // enable_qr_skip_first_：第一次收到 QR 后是否直接跳过第 1 个航点。
  bool enable_qr_skip_first_{true};
  // enable_qr_direction_select_：是否允许 QR 数字切换顺/逆时针路线。
  bool enable_qr_direction_select_{true};
  // use_qr_parity_direction_：true 时奇数 QR=顺时针，偶数 QR=逆时针。
  bool use_qr_parity_direction_{true};
  // enable_image_analysis_trigger_：是否在指定航点发布图像分析触发。
  bool enable_image_analysis_trigger_{true};
  // image_analysis_waypoint_：触发图像分析的航点编号，从 1 开始。
  int image_analysis_waypoint_{7};
  // image_analysis_trigger_topic_：图像分析触发话题，消息类型 std_msgs/Int32。
  std::string image_analysis_trigger_topic_{"/person_trigger"};

  // waypoints_：当前实际路线的航点数组，已按 selected_route_direction_ 加载。
  std::vector<Waypoint> waypoints_;
  // current_index_：当前正在处理的航点下标，从 0 开始；日志里显示时会 +1。
  std::size_t current_index_{0};
  // latest_scan_：最近一次收到的 LaserScan，用于真实障碍避让。
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  // latest_keepout_mask_：最近一次收到的 keepout OccupancyGrid，用于虚拟障碍避让。
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_keepout_mask_;
  // obstacle_warning_count_：避障 WARN 日志计数器，用于节流打印。
  int obstacle_warning_count_{0};
  // emergency_start_time_：进入真实障碍急停区的开始时间，NaN 表示当前不在急停计时中。
  double emergency_start_time_{std::numeric_limits<double>::quiet_NaN()};
  // backup_until_time_：倒车脱困结束时间，NaN 表示当前没有倒车脱困动作。
  double backup_until_time_{std::numeric_limits<double>::quiet_NaN()};
  // backup_turn_direction_：倒车脱困时的转向方向，1.0 或 -1.0。
  double backup_turn_direction_{1.0};
  // qr_skip_done_：是否已经执行过“QR 后跳过第 1 点”，防止重复跳。
  bool qr_skip_done_{false};
  // image_analysis_triggered_：图像分析触发是否已经发布过，防止重复触发。
  bool image_analysis_triggered_{false};
  // last_speed_zone_：上一周期所在速度区，用于只在速度区切换时打印日志。
  std::string last_speed_zone_;

  // tf_buffer_：TF 缓存，用于查询 map 到 base_link 的变换。
  tf2_ros::Buffer tf_buffer_;
  // tf_listener_：TF 监听器，持续把系统 TF 写入 tf_buffer_。
  tf2_ros::TransformListener tf_listener_;
  // cmd_pub_：速度发布器，发布 geometry_msgs/Twist 到 cmd_vel_topic。
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  // image_analysis_trigger_pub_：图像分析触发发布器。
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr image_analysis_trigger_pub_;
  // scan_sub_：LaserScan 订阅器，只缓存最近一帧。
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  // keepout_sub_：keepout OccupancyGrid 订阅器，只缓存最近一帧。
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr keepout_sub_;
  // qr_sub_：QR 字符串订阅器，用于路线选择和起步跳点。
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
  // timer_：控制循环定时器，按 control_frequency 周期调用 control_once()。
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  // 标准 ROS 2 节点入口：初始化、创建节点、进入事件循环，退出前发布零速度。
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SmoothPathFollower>();
  rclcpp::spin(node);
  node->publish_stop();
  rclcpp::shutdown();
  return 0;
}
