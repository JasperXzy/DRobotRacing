#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "follower_utils.hpp"

using origincar_system::normalize_angle;

bool VisualLidarPathFollower::is_static_map_point(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  const VisualLidarPathFollower::Point2D & base_point) const
{
  return is_static_map_point(
    robot_pose, base_point, latest_restriction_map_,
    static_map_inflation_cells_);
}

bool VisualLidarPathFollower::is_static_map_point(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  const VisualLidarPathFollower::Point2D & base_point,
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & restriction_map,
  int restriction_inflation_cells) const
{
  if (!enable_static_map_filter_) {
    return false;
  }

  const Point2D map_point = base_to_map(robot_pose, base_point);
  auto occupied_in_map = [&](
      const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & map_ptr,
      int occupied_threshold, int inflation_cells,
      bool outside_is_occupied) {
      if (!map_ptr) {
        return false;
      }
      const auto & map = *map_ptr;
      const int width = static_cast<int>(map.info.width);
      const int height = static_cast<int>(map.info.height);
      if (width <= 0 || height <= 0 || map.info.resolution <= 0.0 ||
        map.data.size() < static_cast<std::size_t>(width) * static_cast<std::size_t>(height))
      {
        return false;
      }

      const auto & origin = map.info.origin;
      const auto & q = origin.orientation;
      const double origin_yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      const double dx = map_point.x - origin.position.x;
      const double dy = map_point.y - origin.position.y;
      const double grid_x_m = std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy;
      const double grid_y_m = -std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy;
      const int grid_x = static_cast<int>(std::floor(grid_x_m / map.info.resolution));
      const int grid_y = static_cast<int>(std::floor(grid_y_m / map.info.resolution));
      if (grid_x < 0 || grid_y < 0 || grid_x >= width || grid_y >= height) {
        return outside_is_occupied;
      }

      for (int offset_y = -inflation_cells;
        offset_y <= inflation_cells; ++offset_y)
      {
        for (int offset_x = -inflation_cells;
          offset_x <= inflation_cells; ++offset_x)
        {
          if (offset_x * offset_x + offset_y * offset_y >
            inflation_cells * inflation_cells)
          {
            continue;
          }
          const int check_x = grid_x + offset_x;
          const int check_y = grid_y + offset_y;
          if (check_x < 0 || check_y < 0 || check_x >= width || check_y >= height) {
            if (outside_is_occupied) {
              return true;
            }
            continue;
          }
          const auto value = map.data[
            static_cast<std::size_t>(check_y) * map.info.width + check_x];
          if (value >= occupied_threshold) {
            return true;
          }
        }
      }
      return false;
    };

  const bool occupied_in_physical_map = occupied_in_map(
    latest_map_, static_map_occupied_threshold_, static_map_inflation_cells_, true);
  const bool occupied_in_restriction_map = occupied_in_map(
    restriction_map, waypoint_shift_occupied_threshold_,
    restriction_inflation_cells, false);
  return occupied_in_physical_map || occupied_in_restriction_map;
}

std::vector<VisualLidarPathFollower::ScanPointCluster>
VisualLidarPathFollower::collect_non_static_scan_clusters_map(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  int minimum_cluster_points,
  bool use_logic3_static_restriction_map,
  bool filter_logic3_green_mask,
  VisualLidarPathFollower::RobotPose * observation_pose_out) const
{
  if (!latest_scan_) {
    return {};
  }
  const bool use_logic3_runtime_restriction_map =
    logic3_planning_pose_anchored_ && !use_logic3_static_restriction_map;
  const auto & active_restriction_map = use_logic3_static_restriction_map ?
    latest_logic3_static_restriction_map_ :
    (use_logic3_runtime_restriction_map ?
    latest_logic3_runtime_restriction_map_ : latest_restriction_map_);
  const int active_restriction_inflation_cells =
    use_logic3_static_restriction_map ?
    logic3_static_restriction_inflation_cells_ : 0;
  if (enable_static_map_filter_ && require_map_for_lidar_assist_ &&
    (!latest_map_ || !active_restriction_map))
  {
    return {};
  }

  const auto & scan = *latest_scan_;
  const rclcpp::Time scan_stamp(scan.header.stamp, get_clock()->get_clock_type());
  const rclcpp::Time lookup_stamp = scan_stamp.nanoseconds() == 0 ?
    rclcpp::Time(0, 0, get_clock()->get_clock_type()) : scan_stamp;
  RobotPose observation_pose = robot_pose;
  // 发车后车辆在运动，不能用控制周期“现在”的base_link位姿给稍早的
  // 雷达帧投影。按scan采样时刻取得map->base_link，避免12Hz雷达在高速
  // 行驶和转弯时给障碍坐标附加一段车身位移。
  if (logic3_planning_pose_anchored_ &&
    !lookup_pose(frame_id_, base_frame_id_, lookup_stamp, observation_pose))
  {
    return {};
  }
  if (observation_pose_out) {
    *observation_pose_out = observation_pose;
  }
  RobotPose scan_to_base;
  if (!scan.header.frame_id.empty() && scan.header.frame_id != base_frame_id_ &&
    !lookup_pose(
      base_frame_id_, scan.header.frame_id,
      lookup_stamp, scan_to_base))
  {
    return {};
  }

  const double cos_scan = std::cos(scan_to_base.yaw);
  const double sin_scan = std::sin(scan_to_base.yaw);
  std::vector<ScanPointCluster> clusters;
  bool cluster_active = false;
  ScanPointCluster current_cluster;
  Point2D previous_base_point;
  double previous_range = 0.0;
  const std::size_t required_points = static_cast<std::size_t>(
    std::max(1, minimum_cluster_points));

  auto finish_cluster = [&]() {
      if (cluster_active &&
        current_cluster.points.size() >= required_points)
      {
        for (const auto & point : current_cluster.points) {
          current_cluster.center.x += point.x;
          current_cluster.center.y += point.y;
        }
        const double point_count = static_cast<double>(current_cluster.points.size());
        current_cluster.center.x /= point_count;
        current_cluster.center.y /= point_count;
        clusters.push_back(std::move(current_cluster));
      }
      current_cluster = ScanPointCluster();
      cluster_active = false;
    };

  double angle = scan.angle_min;
  for (const auto range : scan.ranges) {
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      finish_cluster();
      angle += scan.angle_increment;
      continue;
    }
    const double scan_x = static_cast<double>(range) * std::cos(angle);
    const double scan_y = static_cast<double>(range) * std::sin(angle);
    const Point2D base_point{
      scan_to_base.x + cos_scan * scan_x - sin_scan * scan_y,
      scan_to_base.y + sin_scan * scan_x + cos_scan * scan_y};
    if (enable_static_map_filter_ && is_static_map_point(
        observation_pose, base_point, active_restriction_map,
        active_restriction_inflation_cells))
    {
      finish_cluster();
      angle += scan.angle_increment;
      continue;
    }
    const Point2D map_point = base_to_map(observation_pose, base_point);
    if (filter_logic3_green_mask && logic3_green_mask_contains(map_point)) {
      finish_cluster();
      angle += scan.angle_increment;
      continue;
    }

    const double point_range = std::hypot(base_point.x, base_point.y);
    const bool connected = cluster_active &&
      std::abs(point_range - previous_range) <= cluster_max_range_jump_ &&
      std::hypot(
      base_point.x - previous_base_point.x,
      base_point.y - previous_base_point.y) <= cluster_max_point_gap_;
    if (cluster_active && !connected) {
      finish_cluster();
    }
    cluster_active = true;
    current_cluster.points.push_back(map_point);
    previous_base_point = base_point;
    previous_range = point_range;
    angle += scan.angle_increment;
  }
  finish_cluster();
  return clusters;
}

bool VisualLidarPathFollower::latest_lidar_scan_is_fresh() const
{
  if (!latest_scan_) {
    return false;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - latest_scan_received_at_).count();
  return age >= 0.0 && age <= lidar_scan_timeout_;
}

// 为 QR 倒车和 channel 全程避障准备同一帧雷达的坐标变换。
// 墙体、静态+动态方案运行限制图或绿色掩码需要 map 坐标时，采样时刻 TF 缺失视为数据未就绪。
bool VisualLidarPathFollower::prepare_lidar_scan(
  VisualLidarPathFollower::RobotPose & scan_to_base,
  VisualLidarPathFollower::RobotPose & scan_to_map,
  bool require_map_transform,
  bool require_logic3_runtime_map) const
{
  scan_to_base = RobotPose();
  scan_to_map = RobotPose();
  if (!latest_lidar_scan_is_fresh()) {
    return false;
  }

  const auto & scan = *latest_scan_;
  const std::string source_frame =
    scan.header.frame_id.empty() ? base_frame_id_ : scan.header.frame_id;
  const rclcpp::Time scan_stamp(scan.header.stamp, get_clock()->get_clock_type());
  const rclcpp::Time lookup_stamp = scan_stamp.nanoseconds() == 0 ?
    rclcpp::Time(0, 0, get_clock()->get_clock_type()) : scan_stamp;

  if (source_frame != base_frame_id_ &&
    !lookup_pose(base_frame_id_, source_frame, lookup_stamp, scan_to_base))
  {
    return false;
  }
  const bool needs_map_transform =
    lidar_enable_static_map_filter_ || require_map_transform;
  if (!needs_map_transform) {
    return true;
  }
  if (lidar_enable_static_map_filter_) {
    if (!latest_map_) {
      return false;
    }
    const auto & map = *latest_map_;
    if (map.info.width == 0 || map.info.height == 0 || map.info.resolution <= 0.0 ||
      map.data.size() < static_cast<std::size_t>(map.info.width) * map.info.height)
    {
      return false;
    }
  }
  if (require_logic3_runtime_map) {
    if (!latest_logic3_runtime_restriction_map_) {
      return false;
    }
    const auto & map = *latest_logic3_runtime_restriction_map_;
    if (map.info.width == 0 || map.info.height == 0 || map.info.resolution <= 0.0 ||
      map.data.size() < static_cast<std::size_t>(map.info.width) * map.info.height)
    {
      return false;
    }
  }
  if (source_frame == frame_id_) {
    return true;
  }
  return lookup_pose(frame_id_, source_frame, lookup_stamp, scan_to_map);
}

bool VisualLidarPathFollower::task1_static_avoid_region_contains(
  const VisualLidarPathFollower::Point2D & map_point) const
{
  return map_point.x > task1_static_avoid_min_x_ &&
         map_point.x < task1_static_avoid_max_x_ &&
         map_point.y > task1_static_avoid_min_y_ &&
         map_point.y < task1_static_avoid_max_y_;
}

bool VisualLidarPathFollower::logic3_green_mask_contains(
  const VisualLidarPathFollower::Point2D & map_point) const
{
  if (tracking_controller_mode_ == "smac_pid") {
    // smac_pid障碍策略下，正常完成静态偏移的绿/橙永久点保持完整掩码。
    // 只有circle_envelope最终硬验收仍失败的ID临时解除掩码，让实时雷达
    // 接续偏移；静态路线本身继续保留最后一次最外推结果，不回退原路线。
    const double radius_squared =
      logic3_green_red_filter_radius_ * logic3_green_red_filter_radius_;
    for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
      if (index < logic3_green_route_clearance_failed_.size() &&
        logic3_green_route_clearance_failed_[index])
      {
        continue;
      }
      const auto & recorded_cluster = logic3_green_clusters_[index];
      const double dx = map_point.x - recorded_cluster.center.x;
      const double dy = map_point.y - recorded_cluster.center.y;
      if (dx * dx + dy * dy <= radius_squared) {
        return true;
      }
    }
    return false;
  }
  const double radius_squared =
    logic3_green_red_filter_radius_ * logic3_green_red_filter_radius_;
  for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
    if (index < logic3_green_route_clearance_failed_.size() &&
      logic3_green_route_clearance_failed_[index])
    {
      continue;
    }
    const auto & green_cluster = logic3_green_clusters_[index];
    // Task1静态矩形外的冻结绿色簇不做静态路线偏移，因此也不能被
    // 绿圈掩码过滤；保留给实时雷达动态避障。A→C→B动态专用阶段由调用方
    // 完全关闭本掩码，进入Task2后再恢复。
    if ((following_qr_path_ || task_phase_ == TaskPhase::TASK1) &&
      !task1_static_avoid_region_contains(green_cluster.center))
    {
      continue;
    }
    // 进入返程段后，分界线以下的整个绿/橙永久障碍簇交给实时雷达，不能继续
    // 被冻结绿色掩码吃掉。按簇中心分类，避免同一个锥桶跨线时被拆开。
    // 上方障碍仍完整保留掩码，即使其点云或偏移尾段延伸到分界线以下。
    if (task3_route_active_ &&
      green_cluster.center.y <= return_obstacle_split_y_)
    {
      continue;
    }
    const double dx = map_point.x - green_cluster.center.x;
    const double dy = map_point.y - green_cluster.center.y;
    if (dx * dx + dy * dy <= radius_squared) {
      return true;
    }
  }
  return false;
}

// 静态+动态方案运行限制图已直接画出需排除的区域，这里只查询点所在栅格，
// 不再叠加物理地图的15cm墙体膨胀。限制图外仍交给物理地图和绿色掩码判断。
bool VisualLidarPathFollower::logic3_runtime_map_point_is_filtered(
  const VisualLidarPathFollower::Point2D & map_point) const
{
  if (!latest_logic3_runtime_restriction_map_) {
    return false;
  }

  const auto & map = *latest_logic3_runtime_restriction_map_;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double resolution = map.info.resolution;
  if (width <= 0 || height <= 0 || resolution <= 0.0 ||
    map.data.size() < static_cast<std::size_t>(width) * static_cast<std::size_t>(height))
  {
    return false;
  }

  const auto & origin = map.info.origin;
  const auto & q = origin.orientation;
  const double origin_yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double dx = map_point.x - origin.position.x;
  const double dy = map_point.y - origin.position.y;
  const double grid_x_m = std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy;
  const double grid_y_m = -std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy;
  const int cell_x = static_cast<int>(std::floor(grid_x_m / resolution));
  const int cell_y = static_cast<int>(std::floor(grid_y_m / resolution));
  if (cell_x < 0 || cell_y < 0 || cell_x >= width || cell_y >= height) {
    return false;
  }

  const auto index = static_cast<std::size_t>(cell_y) * map.info.width +
    static_cast<std::size_t>(cell_x);
  return map.data[index] >= waypoint_shift_occupied_threshold_;
}

// 只用物理定位地图过滤墙体，不把 restriction_map 的软禁行区混入这两段判断。
bool VisualLidarPathFollower::lidar_map_point_is_wall(
  const VisualLidarPathFollower::Point2D & map_point) const
{
  if (!lidar_enable_static_map_filter_ || !latest_map_) {
    return false;
  }

  const auto & map = *latest_map_;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double resolution = map.info.resolution;
  if (width <= 0 || height <= 0 || resolution <= 0.0 ||
    map.data.size() < static_cast<std::size_t>(width) * static_cast<std::size_t>(height))
  {
    return false;
  }

  const auto & origin = map.info.origin;
  const auto & q = origin.orientation;
  const double origin_yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double dx = map_point.x - origin.position.x;
  const double dy = map_point.y - origin.position.y;
  const double grid_x_m = std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy;
  const double grid_y_m = -std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy;
  const int cell_x = static_cast<int>(std::floor(grid_x_m / resolution));
  const int cell_y = static_cast<int>(std::floor(grid_y_m / resolution));
  const int padding_cells = static_cast<int>(
    std::ceil(lidar_static_map_wall_padding_ / resolution));
  if (cell_x < 0 || cell_y < 0 || cell_x >= width || cell_y >= height) {
    return true;
  }

  for (int y = cell_y - padding_cells; y <= cell_y + padding_cells; ++y) {
    for (int x = cell_x - padding_cells; x <= cell_x + padding_cells; ++x) {
      const int offset_x = x - cell_x;
      const int offset_y = y - cell_y;
      if (std::hypot(offset_x, offset_y) * resolution >
        lidar_static_map_wall_padding_ + 1.0e-9)
      {
        continue;
      }
      if (x < 0 || y < 0 || x >= width || y >= height) {
        return true;
      }
      const auto index = static_cast<std::size_t>(y) * map.info.width +
        static_cast<std::size_t>(x);
      if (map.data[index] >= lidar_static_map_occupied_threshold_) {
        return true;
      }
    }
  }
  return false;
}

// 在指定朝向上取最近障碍。常规模式要求总视野扇形、最大半径和矩形
// 走廊同时命中；二维码倒车前方模式改用近宽远窄的直边梯形。
// center_angle=0 检查车前，center_angle=pi 检查车后。
// restrict_to_qr_reverse_front_trapezoid=true时，maximum_distance作为梯形
// 纵向长度，near/far半宽沿forward线性插值，不再使用fov或固定走廊。
// restrict_to_qr_reverse_arc=true时，额外用二维码倒车的固定参考曲率生成
// 车体左外缘圆弧，只保留圆弧以下、也就是右后倒实际扫过一侧的点。
VisualLidarPathFollower::LidarObstacle
VisualLidarPathFollower::find_lidar_obstacle(
  double center_angle, double maximum_distance, double fov_deg,
  double corridor_half_width,
  const VisualLidarPathFollower::RobotPose & scan_to_base,
  const VisualLidarPathFollower::RobotPose & scan_to_map,
  bool filter_logic3_green_mask,
  bool filter_logic3_runtime_map,
  bool restrict_to_qr_reverse_front_trapezoid,
  bool restrict_to_qr_reverse_arc,
  const VisualLidarPathFollower::Point2D * locked_target_map_point,
  double locked_target_radius) const
{
  LidarObstacle result;
  if (!latest_scan_ || maximum_distance <= 0.0) {
    return result;
  }

  const auto & scan = *latest_scan_;
  const double half_fov = std::max(fov_deg * M_PI / 360.0, 0.01);
  const double cos_base = std::cos(scan_to_base.yaw);
  const double sin_base = std::sin(scan_to_base.yaw);
  const double cos_map = std::cos(scan_to_map.yaw);
  const double sin_map = std::sin(scan_to_map.yaw);
  double angle = scan.angle_min;
  for (const auto value : scan.ranges) {
    const double scan_angle = angle;
    angle += scan.angle_increment;
    if (!std::isfinite(value) || value < scan.range_min || value > scan.range_max) {
      continue;
    }

    const double scan_x = static_cast<double>(value) * std::cos(scan_angle);
    const double scan_y = static_cast<double>(value) * std::sin(scan_angle);
    const Point2D base_point{
      scan_to_base.x + cos_base * scan_x - sin_base * scan_y,
      scan_to_base.y + sin_base * scan_x + cos_base * scan_y};
    const double point_range = std::hypot(base_point.x, base_point.y);
    const double relative_angle = normalize_angle(
      std::atan2(base_point.y, base_point.x) - center_angle);
    const double forward = point_range * std::cos(relative_angle);
    const double lateral = point_range * std::sin(relative_angle);
    if (restrict_to_qr_reverse_front_trapezoid) {
      if (forward <= 0.0 || forward > maximum_distance) {
        continue;
      }
      const double distance_ratio = std::clamp(
        forward / maximum_distance, 0.0, 1.0);
      const double allowed_half_width =
        qr_reverse_front_near_half_width_ +
        distance_ratio *
        (qr_reverse_front_far_half_width_ - qr_reverse_front_near_half_width_);
      if (std::abs(lateral) > allowed_half_width) {
        continue;
      }
    } else {
      if (point_range > maximum_distance ||
        std::abs(relative_angle) > half_fov || forward <= 0.0 ||
        std::abs(lateral) > corridor_half_width)
      {
        continue;
      }
    }

    if (restrict_to_qr_reverse_arc) {
      // 当前倒车为linear.x<0、angular.z>0，瞬时转动中心位于车体右侧
      // (base_link.y<0)。车体左外缘因此扫过以(0,-R)为圆心、
      // R+left_offset为半径的外圆弧：
      //   y_arc(x) = -R + sqrt((R+left_offset)^2 - x^2)
      // 后方硬保护只保留y<=y_arc的点。参考角速度为0时退化为
      // y<=left_offset的直线裁剪。
      double arc_boundary_y = qr_reverse_rear_arc_left_offset_;
      if (qr_reverse_rear_arc_angular_speed_ > 1.0e-6 &&
        qr_reverse_linear_speed_ > 1.0e-6)
      {
        const double turn_radius =
          qr_reverse_linear_speed_ / qr_reverse_rear_arc_angular_speed_;
        const double outer_radius =
          turn_radius + qr_reverse_rear_arc_left_offset_;
        const double radicand =
          outer_radius * outer_radius - base_point.x * base_point.x;
        if (radicand < 0.0) {
          continue;
        }
        arc_boundary_y = -turn_radius + std::sqrt(std::max(0.0, radicand));
      }
      if (base_point.y > arc_boundary_y) {
        continue;
      }
    }

    const Point2D map_point{
      scan_to_map.x + cos_map * scan_x - sin_map * scan_y,
      scan_to_map.y + sin_map * scan_x + cos_map * scan_y};
    const bool locked_target_active = locked_target_map_point != nullptr;
    const bool belongs_to_locked_target = locked_target_active &&
      std::hypot(
      map_point.x - locked_target_map_point->x,
      map_point.y - locked_target_map_point->y) <= locked_target_radius;
    // 一次动作开始后只允许首次锁定目标继续提供控制点。其他实时障碍和
    // 永久锥桶即使随后进入扇形，也不能把本轮避障无限续命；待本轮目标
    // 真正离开检测区后，它们才可在下一轮作为新目标重新判断方向。
    if (locked_target_active && !belongs_to_locked_target) {
      continue;
    }
    if (lidar_enable_static_map_filter_ || filter_logic3_runtime_map ||
      filter_logic3_green_mask)
    {
      if ((lidar_enable_static_map_filter_ &&
        lidar_map_point_is_wall(map_point)) ||
        (filter_logic3_runtime_map &&
        logic3_runtime_map_point_is_filtered(map_point)) ||
        (filter_logic3_green_mask && !belongs_to_locked_target &&
        logic3_green_mask_contains(map_point)))
      {
        continue;
      }
    }

    if (!result.found || point_range < result.range) {
      result.found = true;
      result.range = point_range;
      result.relative_angle = relative_angle;
      result.map_point = map_point;
    }
  }
  return result;
}

// 为四个阶段的第x+1轮及以后提供“障碍相对无偏移路线方位”。
// 障碍在路线左侧时右绕、右侧时左绕；若几乎压在线上，则与上一轮反向，
// 避免连续同方向动作把车越推越远。投影只搜索current_index附近的局部弧长，
// 防止闭环路线把障碍匹配到空间接近但进度很远的另一条支路。
bool VisualLidarPathFollower::find_lidar_route_side_direction(
  const VisualLidarPathFollower::Point2D & obstacle_map_point,
  const VisualLidarPathFollower::StageControlProfile & profile,
  int last_avoid_direction, int & avoid_direction, double & route_heading,
  double & signed_lateral_distance) const
{
  avoid_direction = 0;
  route_heading = 0.0;
  signed_lateral_distance = 0.0;
  if (original_waypoints_.size() < 2 ||
    current_index_ >= original_waypoints_.size() - 1)
  {
    return false;
  }

  const std::size_t last_segment = original_waypoints_.size() - 2;
  std::size_t start_segment = std::min(current_index_, last_segment);
  double backward_length = 0.0;
  while (start_segment > 0 &&
    backward_length < profile.lidar_route_side_search_backward_distance)
  {
    --start_segment;
    backward_length += std::hypot(
      original_waypoints_[start_segment + 1].x -
      original_waypoints_[start_segment].x,
      original_waypoints_[start_segment + 1].y -
      original_waypoints_[start_segment].y);
  }

  std::size_t end_segment = std::min(current_index_, last_segment);
  double forward_length = 0.0;
  while (end_segment < last_segment &&
    forward_length < profile.lidar_route_side_search_forward_distance)
  {
    forward_length += std::hypot(
      original_waypoints_[end_segment + 1].x -
      original_waypoints_[end_segment].x,
      original_waypoints_[end_segment + 1].y -
      original_waypoints_[end_segment].y);
    ++end_segment;
  }

  bool projection_valid = false;
  double best_distance = std::numeric_limits<double>::infinity();
  double best_segment_x = 0.0;
  double best_segment_y = 0.0;
  double best_segment_length = 0.0;
  const Waypoint * best_segment_start = nullptr;
  for (std::size_t index = start_segment; index <= end_segment; ++index) {
    const auto & start = original_waypoints_[index];
    const auto & finish = original_waypoints_[index + 1];
    const double segment_x = finish.x - start.x;
    const double segment_y = finish.y - start.y;
    const double segment_length_squared =
      segment_x * segment_x + segment_y * segment_y;
    if (segment_length_squared < 1.0e-8) {
      continue;
    }
    const double point_x = obstacle_map_point.x - start.x;
    const double point_y = obstacle_map_point.y - start.y;
    const double t = std::clamp(
      (point_x * segment_x + point_y * segment_y) /
      segment_length_squared, 0.0, 1.0);
    const double projected_x = start.x + t * segment_x;
    const double projected_y = start.y + t * segment_y;
    const double distance = std::hypot(
      obstacle_map_point.x - projected_x,
      obstacle_map_point.y - projected_y);
    if (distance < best_distance) {
      projection_valid = true;
      best_distance = distance;
      best_segment_x = segment_x;
      best_segment_y = segment_y;
      best_segment_length = std::sqrt(segment_length_squared);
      best_segment_start = &start;
    }
  }
  if (!projection_valid || best_segment_start == nullptr ||
    best_segment_length < 1.0e-8)
  {
    return false;
  }

  route_heading = std::atan2(best_segment_y, best_segment_x);
  signed_lateral_distance =
    (best_segment_x * (obstacle_map_point.y - best_segment_start->y) -
    best_segment_y * (obstacle_map_point.x - best_segment_start->x)) /
    best_segment_length;
  if (signed_lateral_distance > profile.lidar_route_side_deadband) {
    // 障碍在路线行进方向左侧：向右避。
    avoid_direction = -1;
  } else if (
    signed_lateral_distance < -profile.lidar_route_side_deadband)
  {
    // 障碍在路线行进方向右侧：向左避。
    avoid_direction = 1;
  } else {
    // 障碍几乎压在线上时没有稳定左右方位：优先与上一轮反向。
    avoid_direction = last_avoid_direction != 0 ? -last_avoid_direction : 1;
  }
  return true;
}

// 四阶段实时雷达避障：smac_pid策略始终屏蔽全部已确认的绿/橙永久点，
// 只让掩码外未记录新障碍进入实时控制；legacy_lookahead在配置
// transition_dynamic_avoid_only时允许过渡段雷达覆盖全部障碍，但这只影响
// 实时雷达过滤，不再清除已经在完整A→C→B→Task2路线中计算出的静态偏移。
bool VisualLidarPathFollower::apply_channel_lidar_obstacle_avoidance(
  geometry_msgs::msg::Twist & cmd, bool * observation_valid,
  bool * obstacle_found)
{
  if (observation_valid) {
    *observation_valid = false;
  }
  if (obstacle_found) {
    *obstacle_found = false;
  }
  const bool filter_logic3_runtime_map = logic3_planning_pose_anchored_;
  const bool before_or_at_channel_entry = !waypoints_.empty() &&
    channel_entry_waypoint_index_ < waypoints_.size() &&
    current_index_ <= channel_entry_waypoint_index_;
  const bool transition_lidar_scope =
    task_phase_ == TaskPhase::TRANSITION ||
    (task_phase_ == TaskPhase::TASK2 && before_or_at_channel_entry);
  // B点附近即使Task2参数已经提前接管，仍按过渡阶段计数与策略继续，
  // 直到真正越过A→C→B入口段；其余三个阶段各自独立计数。
  const TaskPhase policy_phase = transition_lidar_scope ?
    TaskPhase::TRANSITION : task_phase_;
  const StageControlProfile * policy_profile = nullptr;
  std::size_t * policy_avoid_count = nullptr;
  int * policy_last_direction = nullptr;
  switch (policy_phase) {
    case TaskPhase::TASK1:
      policy_profile = &task1_profile_;
      policy_avoid_count = &task1_lidar_avoid_count_;
      policy_last_direction = &task1_lidar_last_avoid_direction_;
      break;
    case TaskPhase::TRANSITION:
      policy_profile = &transition_profile_;
      policy_avoid_count = &transition_lidar_avoid_count_;
      policy_last_direction = &transition_lidar_last_avoid_direction_;
      break;
    case TaskPhase::TASK2:
      policy_profile = &task2_profile_;
      policy_avoid_count = &task2_lidar_avoid_count_;
      policy_last_direction = &task2_lidar_last_avoid_direction_;
      break;
    case TaskPhase::TASK3:
      policy_profile = &task3_profile_;
      policy_avoid_count = &task3_lidar_avoid_count_;
      policy_last_direction = &task3_lidar_last_avoid_direction_;
      break;
    case TaskPhase::FINISHED:
    case TaskPhase::UNKNOWN:
      break;
  }
  if (!policy_profile || !policy_avoid_count || !policy_last_direction) {
    return false;
  }
  const std::size_t heading_direction_max_count = static_cast<std::size_t>(
    policy_profile->lidar_heading_direction_max_count);
  const std::size_t route_side_direction_max_count = static_cast<std::size_t>(
    policy_profile->lidar_route_side_direction_enabled ?
    policy_profile->lidar_route_side_direction_max_count : 0);
  const std::size_t policy_total_max_count =
    heading_direction_max_count + route_side_direction_max_count;
  if (!channel_lidar_obstacle_active_ &&
    (*policy_avoid_count >= policy_total_max_count))
  {
    return false;
  }
  // 倒车结束时立下scan_sequence栅栏。必须收到严格晚于倒车退出时刻的
  // 新雷达帧，才允许首次正向动态避障判断；禁止复用倒车转向过程中的旧帧。
  if (channel_lidar_waiting_for_post_reverse_scan_) {
    if (!latest_scan_ || scan_sequence_ <= channel_lidar_post_reverse_scan_sequence_) {
      return false;
    }
    const std::int64_t scan_stamp_nanoseconds = rclcpp::Time(
      latest_scan_->header.stamp, get_clock()->get_clock_type()).nanoseconds();
    // 正常带时间戳的雷达必须确实采样于倒车退出之后；时间戳为0的驱动无法
    // 做采样时刻比较，只退化为上面的新回调序号闸门。
    if (scan_stamp_nanoseconds != 0 &&
      scan_stamp_nanoseconds <= channel_lidar_post_reverse_time_nanoseconds_)
    {
      return false;
    }
    channel_lidar_waiting_for_post_reverse_scan_ = false;
  }
  const bool transition_dynamic_only =
    tracking_controller_mode_ != "smac_pid" && transition_dynamic_avoid_only_ &&
    transition_lidar_scope;
  const bool track_locked_target =
    channel_lidar_obstacle_active_ && channel_lidar_locked_target_valid_;
  const double locked_target_tracking_radius =
    channel_lidar_locked_target_is_permanent_ ?
    logic3_green_red_filter_radius_ : logic3_red_takeover_radius_;
  const bool filter_logic3_green_mask =
    logic3_planning_pose_anchored_ &&
    (!transition_dynamic_only || channel_lidar_obstacle_active_) &&
    !logic3_green_clusters_.empty();
  // 同一次避障动作锁定触发阶段的检测区、控制参数和首次目标。这样即使
  // 动作中越过B点切换Task2参数，也只会在首次目标离开原检测区后结束。
  const double lidar_avoid_distance = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_avoid_distance :
    channel_lidar_avoid_distance_;
  const double lidar_avoid_fov_deg = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_avoid_fov_deg :
    channel_lidar_avoid_fov_deg_;
  const double lidar_corridor_half_width = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_corridor_half_width :
    channel_lidar_corridor_half_width_;
  const double lidar_avoid_linear_speed = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_avoid_linear_speed :
    channel_lidar_avoid_linear_speed_;
  double lidar_avoid_angular_gain = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_avoid_angular_gain :
    channel_lidar_avoid_angular_gain_;
  const double lidar_direction_upper_angle_deg = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_direction_upper_angle_deg :
    channel_lidar_direction_upper_angle_deg_;
  const double lidar_direction_lower_angle_deg = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_direction_lower_angle_deg :
    channel_lidar_direction_lower_angle_deg_;
  const double lidar_route_side_reverse_exit_hysteresis_deg =
    channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_route_side_reverse_exit_hysteresis_deg :
    policy_profile->lidar_route_side_reverse_exit_hysteresis_deg;
  RobotPose scan_to_base;
  RobotPose scan_to_map;
  if (!prepare_lidar_scan(
      scan_to_base, scan_to_map,
      filter_logic3_runtime_map || filter_logic3_green_mask ||
      track_locked_target,
      filter_logic3_runtime_map))
  {
    if (!channel_lidar_waiting_for_inputs_) {
      RCLCPP_WARN(
        get_logger(),
        "channel雷达避障输入暂不可用：跳过本周期雷达覆盖，继续当前路线跟踪，不停车。");
      channel_lidar_waiting_for_inputs_ = true;
    }
    channel_lidar_obstacle_active_ = false;
    channel_lidar_logged_direction_ = 0;
    channel_lidar_avoid_direction_ = 0;
    channel_lidar_route_side_action_active_ = false;
    channel_lidar_route_reverse_phase_active_ = false;
    channel_lidar_route_toward_direction_ = 0;
    channel_lidar_route_trigger_obstacle_valid_ = false;
    channel_lidar_locked_target_valid_ = false;
    channel_lidar_locked_target_is_permanent_ = false;
    channel_lidar_locked_target_is_task1_clearance_fallback_ = false;
    return false;
  }
  channel_lidar_waiting_for_inputs_ = false;
  if (observation_valid) {
    *observation_valid = true;
  }

  const auto obstacle = find_lidar_obstacle(
    0.0, lidar_avoid_distance, lidar_avoid_fov_deg,
    lidar_corridor_half_width, scan_to_base, scan_to_map,
    filter_logic3_green_mask, filter_logic3_runtime_map, false, false,
    track_locked_target ? &channel_lidar_locked_target_map_point_ : nullptr,
    locked_target_tracking_radius);
  // 二次防护：即使上游过滤开关未按预期生效，PID模式也绝不允许
  // 已记录绿/橙掩码内的雷达点覆盖路线跟踪命令。
  if (obstacle.found && !track_locked_target &&
    tracking_controller_mode_ == "smac_pid" &&
    logic3_green_mask_contains(obstacle.map_point))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "实时雷达候选map(%.3f, %.3f)位于已记录绿/橙点%.2fm掩码内；"
      "二次防护已拦截，保持smac_pid路线跟踪。",
      obstacle.map_point.x, obstacle.map_point.y, logic3_green_red_filter_radius_);
    return false;
  }
  if (!obstacle.found) {
    if (channel_lidar_obstacle_active_) {
      const char * completed_target_type =
        channel_lidar_locked_target_is_task1_clearance_fallback_ ?
        "Task1静态净距失败专用兜底" :
        (channel_lidar_locked_target_is_permanent_ ?
        "绿/橙永久点动态接管" : "普通实时动态障碍");
      RCLCPP_INFO(
        get_logger(),
        "channel雷达避障结束：首次锁定目标=%s，中心=(%.3f, %.3f)"
        "已离开检测区；其他障碍未参与本轮续接，恢复当前阶段路线跟踪。",
        completed_target_type,
        channel_lidar_locked_target_map_point_.x,
        channel_lidar_locked_target_map_point_.y);
      channel_lidar_obstacle_active_ = false;
      channel_lidar_logged_direction_ = 0;
      channel_lidar_avoid_direction_ = 0;
      channel_lidar_route_side_action_active_ = false;
      channel_lidar_route_reverse_phase_active_ = false;
      channel_lidar_route_toward_direction_ = 0;
      channel_lidar_route_trigger_obstacle_valid_ = false;
      channel_lidar_locked_target_valid_ = false;
      channel_lidar_locked_target_is_permanent_ = false;
      channel_lidar_locked_target_is_task1_clearance_fallback_ = false;
    }
    return false;
  }
  if (obstacle_found) {
    *obstacle_found = true;
  }

  const double half_fov = std::max(
    lidar_avoid_fov_deg * M_PI / 360.0, 0.01);
  const double center_weight = std::clamp(
    1.0 - std::abs(obstacle.relative_angle) / half_fov, 0.0, 1.0);
  const double upper_angle =
    lidar_direction_upper_angle_deg * M_PI / 180.0;
  const double lower_angle =
    lidar_direction_lower_angle_deg * M_PI / 180.0;
  // 由同一雷达采样时刻的map<-scan和base_link<-scan恢复map<-base_link，
  // 供路线方位动作判断车头是否已经越过“本轮触发障碍”的固定map方向。
  const double vehicle_map_yaw = normalize_angle(
    scan_to_map.yaw - scan_to_base.yaw);
  const double vehicle_map_cos = std::cos(vehicle_map_yaw);
  const double vehicle_map_sin = std::sin(vehicle_map_yaw);
  const Point2D vehicle_map_point{
    scan_to_map.x -
    (vehicle_map_cos * scan_to_base.x - vehicle_map_sin * scan_to_base.y),
    scan_to_map.y -
    (vehicle_map_sin * scan_to_base.x + vehicle_map_cos * scan_to_base.y)};
  const bool avoidance_action_started = !channel_lidar_obstacle_active_;
  double route_signed_lateral_distance = 0.0;
  if (avoidance_action_started) {
    channel_lidar_route_side_action_active_ = false;
    channel_lidar_route_reverse_phase_active_ = false;
    channel_lidar_route_toward_direction_ = 0;
    channel_lidar_route_trigger_obstacle_valid_ = false;
    const std::size_t policy_action_number = *policy_avoid_count + 1;
    const bool request_route_side_direction =
      policy_profile->lidar_route_side_direction_enabled &&
      policy_action_number > heading_direction_max_count;
    if (request_route_side_direction &&
      find_lidar_route_side_direction(
        obstacle.map_point, *policy_profile, *policy_last_direction,
        channel_lidar_avoid_direction_,
        channel_lidar_route_heading_, route_signed_lateral_distance))
    {
      channel_lidar_route_side_action_active_ = true;
      channel_lidar_route_trigger_obstacle_map_point_ = obstacle.map_point;
      channel_lidar_route_trigger_obstacle_valid_ = true;
      lidar_avoid_angular_gain = policy_profile->lidar_route_side_angular_gain;

      // T_map_scan = T_map_base * T_base_scan，由两份同帧TF恢复车头map角。
      const double route_relative_angle = normalize_angle(
        channel_lidar_route_heading_ - vehicle_map_yaw);
      if (std::abs(route_relative_angle) > 1.0e-6) {
        channel_lidar_route_toward_direction_ =
          route_relative_angle > 0.0 ? 1 : -1;
      }
      const bool obstacle_between_vehicle_and_route =
        channel_lidar_route_toward_direction_ != 0 &&
        obstacle.relative_angle * route_relative_angle > 0.0 &&
        std::abs(obstacle.relative_angle) <=
        std::abs(route_relative_angle) + 1.0e-9;
      channel_lidar_route_reverse_phase_active_ =
        policy_profile->lidar_route_side_reverse_phase_enabled &&
        obstacle_between_vehicle_and_route;
      if (channel_lidar_route_reverse_phase_active_) {
        // 精确落实反相规则：障碍角夹在车头与路线之间时，不沿普通障碍
        // 左右侧继续外绕，而是直接锁定朝路线方向转，并使用最大转角。
        channel_lidar_avoid_direction_ = channel_lidar_route_toward_direction_;
      }
    } else {
      // 第1轮以及路线局部投影失败时，保持原有车头相对角方向算法。
      if (obstacle.relative_angle > upper_angle) {
        // 障碍明确在左侧：右绕。
        channel_lidar_avoid_direction_ = -1;
      } else if (obstacle.relative_angle < lower_angle) {
        // 障碍明确在右侧：左绕。
        channel_lidar_avoid_direction_ = 1;
      } else {
        // 首次遇到正中障碍时沿用原策略：默认左绕。
        channel_lidar_avoid_direction_ = 1;
      }
    }
    ++(*policy_avoid_count);
    *policy_last_direction = channel_lidar_avoid_direction_;
  }
  // 路线方位动作的方向从触发到清空始终锁定。若触发时障碍方向夹在
  // 车头与路线方向之间，就先按最大转角穿过障碍方向；车头沿锁定方向
  // 越过障碍至少配置迟滞角后，才恢复原扇形中心权重。切换时障碍几乎
  // 位于车头中心，因此最大值可平滑衔接到原公式，不会出现先小后大的反相。
  if (channel_lidar_route_side_action_active_ &&
    channel_lidar_route_reverse_phase_active_)
  {
    const double exit_hysteresis =
      lidar_route_side_reverse_exit_hysteresis_deg * M_PI / 180.0;
    const double trigger_obstacle_relative_angle =
      channel_lidar_route_trigger_obstacle_valid_ ? normalize_angle(
      std::atan2(
        channel_lidar_route_trigger_obstacle_map_point_.y - vehicle_map_point.y,
        channel_lidar_route_trigger_obstacle_map_point_.x - vehicle_map_point.x) -
      vehicle_map_yaw) : obstacle.relative_angle;
    const bool crossed_obstacle_direction =
      (channel_lidar_route_toward_direction_ > 0 &&
      trigger_obstacle_relative_angle <= -exit_hysteresis) ||
      (channel_lidar_route_toward_direction_ < 0 &&
      trigger_obstacle_relative_angle >= exit_hysteresis);
    if (crossed_obstacle_direction) {
      channel_lidar_route_reverse_phase_active_ = false;
    }
  }
  const double turn_weight =
    channel_lidar_route_side_action_active_ &&
    channel_lidar_route_reverse_phase_active_ ? 1.0 : center_weight;
  const double avoid_direction =
    static_cast<double>(channel_lidar_avoid_direction_);
  cmd.linear.x = lidar_avoid_linear_speed;
  cmd.angular.z = avoid_direction * lidar_avoid_angular_gain * turn_weight;
  if (avoidance_action_started) {
    // 固定本轮唯一目标。优先关联绿/橙永久ID并锁定其稳定中心；未关联时
    // 锁定首次红点。中心在整轮动作中绝不逐帧更新，避免雷达抖动把目标
    // 沿相邻点云拖走，或由其他锥桶接力延长本轮动作。
    channel_lidar_locked_target_map_point_ = obstacle.map_point;
    channel_lidar_locked_target_valid_ = true;
    channel_lidar_locked_target_is_permanent_ = false;
    channel_lidar_locked_target_is_task1_clearance_fallback_ = false;
    channel_lidar_locked_target_permanent_index_ = logic3_green_clusters_.size();
    double nearest_permanent_distance = logic3_red_takeover_radius_;
    for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
      const double distance = std::hypot(
        obstacle.map_point.x - logic3_green_clusters_[index].center.x,
        obstacle.map_point.y - logic3_green_clusters_[index].center.y);
      if (distance <= nearest_permanent_distance) {
        nearest_permanent_distance = distance;
        channel_lidar_locked_target_permanent_index_ = index;
      }
    }
    if (channel_lidar_locked_target_permanent_index_ <
      logic3_green_clusters_.size())
    {
      const std::size_t permanent_index =
        channel_lidar_locked_target_permanent_index_;
      channel_lidar_locked_target_map_point_ =
        logic3_green_clusters_[permanent_index].center;
      channel_lidar_locked_target_is_permanent_ = true;
      channel_lidar_locked_target_is_task1_clearance_fallback_ =
        task_phase_ == TaskPhase::TASK1 &&
        task1_static_avoid_region_contains(
        logic3_green_clusters_[permanent_index].center) &&
        permanent_index < logic3_green_route_clearance_failed_.size() &&
        logic3_green_route_clearance_failed_[permanent_index];
    }
    const char * locked_target_type =
      channel_lidar_locked_target_is_task1_clearance_fallback_ ?
      "Task1静态净距失败专用兜底" :
      (channel_lidar_locked_target_is_permanent_ ?
      "绿/橙永久点动态接管" : "普通实时动态障碍");

    // 每次完整动作只在首次真正覆盖路线命令时记录一个红点；连续控制周期
    // 不重复生成抖动红点，也不重复生成before_red图片。
    record_logic3_lidar_avoidance_obstacle(obstacle, scan_to_base, scan_to_map);
    channel_lidar_action_profile_.lidar_avoid_distance = lidar_avoid_distance;
    channel_lidar_action_profile_.lidar_avoid_fov_deg = lidar_avoid_fov_deg;
    channel_lidar_action_profile_.lidar_corridor_half_width = lidar_corridor_half_width;
    channel_lidar_action_profile_.lidar_avoid_linear_speed = lidar_avoid_linear_speed;
    channel_lidar_action_profile_.lidar_avoid_angular_gain = lidar_avoid_angular_gain;
    channel_lidar_action_profile_.lidar_direction_upper_angle_deg =
      lidar_direction_upper_angle_deg;
    channel_lidar_action_profile_.lidar_direction_lower_angle_deg =
      lidar_direction_lower_angle_deg;
    channel_lidar_action_profile_.lidar_route_side_reverse_exit_hysteresis_deg =
      lidar_route_side_reverse_exit_hysteresis_deg;
    // 只在本次避障真正触发的这一刻关联永久绿/橙身份：从原始路线重算并
    // 取消该ID的静态偏移；绿/橙雷达掩码始终保留。后续只对上面锁定的
    // 唯一目标开放掩码例外，并在该目标离开检测区时结束本轮。
    if (activate_logic3_dynamic_takeover(obstacle.map_point)) {
      rebuild_logic3_route_from_green_clusters("实际雷达避障接管永久点");
    }
    channel_lidar_obstacle_active_ = true;
    channel_lidar_logged_direction_ = channel_lidar_avoid_direction_;

    // 每次完整避障动作只在false->true的触发瞬间留一张图；动作中的连续
    // 控制周期以及方向保持不重复出图。
    if (task_phase_ == TaskPhase::TASK1) {
      ++logic3_task1_before_red_snapshot_count_;
      pending_control_snapshot_event_ = "task1_before_red";
    } else if (task_phase_ == TaskPhase::TRANSITION ||
      task_phase_ == TaskPhase::TASK2 || task_phase_ == TaskPhase::TASK3)
    {
      ++logic3_final_before_red_snapshot_count_;
      pending_control_snapshot_event_ = "final_before_red";
    }

    if (channel_lidar_route_side_action_active_) {
      RCLCPP_WARN(
        get_logger(),
        "channel雷达避障开始：阶段=%s，第%zu/%zu轮[x=%zu,y=%zu]，"
        "目标=%s，锁定中心=(%.3f, %.3f)，"
        "方向算法=路线方位，障碍map=(%.3f, %.3f)，"
        "相对无偏移路线横距=%+.3fm，路线角=%.1fdeg，"
        "车头相对障碍角=%.1fdeg，反相最大转角=%s，保持%s绕，输出=%.2f/%.2f。",
        task_phase_name(policy_phase), *policy_avoid_count, policy_total_max_count,
        heading_direction_max_count, route_side_direction_max_count,
        locked_target_type,
        channel_lidar_locked_target_map_point_.x,
        channel_lidar_locked_target_map_point_.y,
        obstacle.map_point.x, obstacle.map_point.y,
        route_signed_lateral_distance,
        channel_lidar_route_heading_ * 180.0 / M_PI,
        obstacle.relative_angle * 180.0 / M_PI,
        channel_lidar_route_reverse_phase_active_ ? "是" : "否",
        channel_lidar_avoid_direction_ > 0 ? "左" : "右",
        cmd.linear.x, cmd.angular.z);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "channel雷达避障开始：阶段=%s，第%zu/%zu轮[x=%zu,y=%zu]，"
        "目标=%s，锁定中心=(%.3f, %.3f)，"
        "方向算法=车头相对角，距离=%.3fm，相对角=%.1fdeg，"
        "滞环=[%.1f, %.1f]deg，保持%s绕，输出=%.2f/%.2f。",
        task_phase_name(policy_phase), *policy_avoid_count, policy_total_max_count,
        heading_direction_max_count, route_side_direction_max_count,
        locked_target_type,
        channel_lidar_locked_target_map_point_.x,
        channel_lidar_locked_target_map_point_.y,
        obstacle.range, obstacle.relative_angle * 180.0 / M_PI,
        lidar_direction_lower_angle_deg,
        lidar_direction_upper_angle_deg,
        channel_lidar_avoid_direction_ > 0 ? "左" : "右",
        cmd.linear.x, cmd.angular.z);
    }
  }
  return true;
}

// 用簇内二维雷达点拟合圆：
//   2*u*cx + 2*v*cy + c = u^2 + v^2
// 先减去点集均值改善数值稳定性，再用 3x3 高斯消元解最小二乘法方程。
bool VisualLidarPathFollower::fit_cone_circle(
  VisualLidarPathFollower::LidarCluster & cluster) const
{
  if (cluster.points.size() < static_cast<std::size_t>(cluster_min_points_)) {
    return false;
  }

  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const auto & point : cluster.points) {
    mean_x += point.x;
    mean_y += point.y;
  }
  const double point_count = static_cast<double>(cluster.points.size());
  mean_x /= point_count;
  mean_y /= point_count;

  double system[3][4] = {};
  for (const auto & point : cluster.points) {
    const double u = point.x - mean_x;
    const double v = point.y - mean_y;
    const double rhs = u * u + v * v;
    const double row[3] = {2.0 * u, 2.0 * v, 1.0};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        system[r][c] += row[r] * row[c];
      }
      system[r][3] += row[r] * rhs;
    }
  }

  for (int column = 0; column < 3; ++column) {
    int pivot_row = column;
    for (int row = column + 1; row < 3; ++row) {
      if (std::abs(system[row][column]) > std::abs(system[pivot_row][column])) {
        pivot_row = row;
      }
    }
    if (std::abs(system[pivot_row][column]) < 1e-10) {
      return false;
    }
    if (pivot_row != column) {
      for (int c = column; c < 4; ++c) {
        std::swap(system[column][c], system[pivot_row][c]);
      }
    }

    const double pivot = system[column][column];
    for (int c = column; c < 4; ++c) {
      system[column][c] /= pivot;
    }
    for (int row = 0; row < 3; ++row) {
      if (row == column) {
        continue;
      }
      const double factor = system[row][column];
      for (int c = column; c < 4; ++c) {
        system[row][c] -= factor * system[column][c];
      }
    }
  }

  const double local_center_x = system[0][3];
  const double local_center_y = system[1][3];
  const double radius_squared =
    local_center_x * local_center_x + local_center_y * local_center_y + system[2][3];
  if (!std::isfinite(radius_squared) || radius_squared <= 0.0) {
    return false;
  }

  const double radius = std::sqrt(radius_squared);
  const Point2D center{mean_x + local_center_x, mean_y + local_center_y};
  if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
    radius < cone_circle_min_radius_ || radius > cone_circle_max_radius_)
  {
    return false;
  }

  double squared_error_sum = 0.0;
  for (const auto & point : cluster.points) {
    const double radial_error = std::hypot(point.x - center.x, point.y - center.y) - radius;
    squared_error_sum += radial_error * radial_error;
  }
  const double rmse = std::sqrt(squared_error_sum / point_count);
  if (!std::isfinite(rmse) || rmse > cone_circle_max_rmse_) {
    return false;
  }

  cluster.center = center;
  cluster.fitted_radius = radius;
  cluster.fit_rmse = rmse;
  return true;
}

std::vector<VisualLidarPathFollower::LidarCluster>
VisualLidarPathFollower::collect_cone_clusters(
  const VisualLidarPathFollower::RobotPose & robot_pose) const
{
  if (!latest_scan_) {
    return {};
  }
  if (enable_static_map_filter_ && require_map_for_lidar_assist_ &&
    (!latest_map_ || !latest_restriction_map_))
  {
    return {};
  }

  const auto & scan = *latest_scan_;

  RobotPose scan_to_base;
  if (!scan.header.frame_id.empty() && scan.header.frame_id != base_frame_id_ &&
    !lookup_pose(
      base_frame_id_, scan.header.frame_id,
      rclcpp::Time(0, 0, get_clock()->get_clock_type()), scan_to_base))
  {
    return {};
  }

  std::vector<LidarCluster> clusters;
  bool cluster_active = false;
  LidarCluster current_cluster;
  Point2D previous_point;
  double previous_range = 0.0;

  auto finish_cluster = [&]() {
      if (cluster_active && current_cluster.point_count >= cluster_min_points_ &&
        fit_cone_circle(current_cluster))
      {
        const bool static_map_cluster =
          enable_static_map_filter_ && is_static_map_point(robot_pose, current_cluster.center);
        if (!static_map_cluster) {
          clusters.push_back(current_cluster);
        }
      }
      current_cluster = LidarCluster();
      cluster_active = false;
    };

  double angle = scan.angle_min;
  for (const auto value : scan.ranges) {
    bool valid = std::isfinite(value) && value >= scan.range_min && value <= scan.range_max;
    Point2D base_point;
    if (valid) {
      const double scan_x = static_cast<double>(value) * std::cos(angle);
      const double scan_y = static_cast<double>(value) * std::sin(angle);
      const double cos_scan = std::cos(scan_to_base.yaw);
      const double sin_scan = std::sin(scan_to_base.yaw);
      base_point.x = scan_to_base.x + cos_scan * scan_x - sin_scan * scan_y;
      base_point.y = scan_to_base.y + sin_scan * scan_x + cos_scan * scan_y;
    }
    if (!valid) {
      finish_cluster();
      angle += scan.angle_increment;
      continue;
    }

    const double base_range = std::hypot(base_point.x, base_point.y);
    const bool connected = cluster_active &&
      std::abs(base_range - previous_range) <= cluster_max_range_jump_ &&
      std::hypot(base_point.x - previous_point.x, base_point.y - previous_point.y) <=
      cluster_max_point_gap_;
    if (cluster_active && !connected) {
      finish_cluster();
    }
    if (!cluster_active) {
      cluster_active = true;
    }

    current_cluster.points.push_back(base_point);
    ++current_cluster.point_count;
    previous_point = base_point;
    previous_range = base_range;
    angle += scan.angle_increment;
  }
  finish_cluster();
  return clusters;
}
