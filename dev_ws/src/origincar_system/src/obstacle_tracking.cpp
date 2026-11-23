#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "follower_utils.hpp"

using origincar_system::to_lower;

bool VisualLidarPathFollower::logic3_runtime_green_steering_is_stable(
  const sensor_msgs::msg::LaserScan & scan, double * steering_span) const
{
  if (steering_span) {
    *steering_span = std::numeric_limits<double>::infinity();
  }
  if (logic3_command_history_.empty()) {
    return false;
  }

  const std::int64_t scan_start_nanoseconds = rclcpp::Time(
    scan.header.stamp, get_clock()->get_clock_type()).nanoseconds();
  double scan_duration_seconds = 0.0;
  if (scan.ranges.size() > 1 && std::isfinite(scan.time_increment) &&
    scan.time_increment > 0.0F)
  {
    scan_duration_seconds = static_cast<double>(scan.ranges.size() - 1) *
      static_cast<double>(scan.time_increment);
  } else if (std::isfinite(scan.scan_time) && scan.scan_time > 0.0F) {
    scan_duration_seconds = static_cast<double>(scan.scan_time);
  } else {
    return false;
  }
  const std::int64_t scan_end_nanoseconds = scan_start_nanoseconds +
    static_cast<std::int64_t>(std::llround(scan_duration_seconds * 1.0e9));

  bool have_start_command = false;
  double minimum_steering = 0.0;
  double maximum_steering = 0.0;
  for (const auto & sample : logic3_command_history_) {
    if (sample.stamp_nanoseconds <= scan_start_nanoseconds) {
      if (!std::isfinite(sample.steering_angle)) {
        return false;
      }
      minimum_steering = sample.steering_angle;
      maximum_steering = sample.steering_angle;
      have_start_command = true;
      continue;
    }
    if (sample.stamp_nanoseconds > scan_end_nanoseconds) {
      break;
    }
    if (!have_start_command || !std::isfinite(sample.steering_angle)) {
      return false;
    }
    minimum_steering = std::min(minimum_steering, sample.steering_angle);
    maximum_steering = std::max(maximum_steering, sample.steering_angle);
  }
  if (!have_start_command) {
    return false;
  }

  const double measured_span = maximum_steering - minimum_steering;
  if (steering_span) {
    *steering_span = measured_span;
  }
  // Only the change is limited: a constant non-zero steering command (a
  // stable arc) passes exactly like a constant zero command (a straight line).
  return measured_span <= logic3_runtime_green_max_steering_span_;
}

bool VisualLidarPathFollower::logic3_runtime_green_speed_is_stable(
  const sensor_msgs::msg::LaserScan & scan,
  double * minimum_abs_speed,
  double * maximum_speed_step) const
{
  if (minimum_abs_speed) {
    *minimum_abs_speed = std::numeric_limits<double>::infinity();
  }
  if (maximum_speed_step) {
    *maximum_speed_step = std::numeric_limits<double>::infinity();
  }
  if (logic3_command_history_.empty()) {
    return false;
  }

  const std::int64_t scan_start_nanoseconds = rclcpp::Time(
    scan.header.stamp, get_clock()->get_clock_type()).nanoseconds();
  double scan_duration_seconds = 0.0;
  if (scan.ranges.size() > 1 && std::isfinite(scan.time_increment) &&
    scan.time_increment > 0.0F)
  {
    scan_duration_seconds = static_cast<double>(scan.ranges.size() - 1) *
      static_cast<double>(scan.time_increment);
  } else if (std::isfinite(scan.scan_time) && scan.scan_time > 0.0F) {
    scan_duration_seconds = static_cast<double>(scan.scan_time);
  } else {
    return false;
  }
  const std::int64_t scan_end_nanoseconds = scan_start_nanoseconds +
    static_cast<std::int64_t>(std::llround(scan_duration_seconds * 1.0e9));

  bool have_start_command = false;
  double previous_speed = 0.0;
  double measured_minimum_abs_speed = std::numeric_limits<double>::infinity();
  double measured_maximum_speed_step = 0.0;
  for (const auto & sample : logic3_command_history_) {
    if (sample.stamp_nanoseconds <= scan_start_nanoseconds) {
      if (!std::isfinite(sample.linear_speed)) {
        return false;
      }
      previous_speed = sample.linear_speed;
      have_start_command = true;
      continue;
    }
    if (sample.stamp_nanoseconds > scan_end_nanoseconds) {
      break;
    }
    if (!have_start_command || !std::isfinite(sample.linear_speed)) {
      return false;
    }
    measured_minimum_abs_speed = std::min(
      measured_minimum_abs_speed, std::abs(previous_speed));
    measured_maximum_speed_step = std::max(
      measured_maximum_speed_step,
      std::abs(sample.linear_speed - previous_speed));
    previous_speed = sample.linear_speed;
    measured_minimum_abs_speed = std::min(
      measured_minimum_abs_speed, std::abs(previous_speed));
  }
  if (!have_start_command) {
    return false;
  }
  if (!std::isfinite(measured_minimum_abs_speed)) {
    // 一圈雷达内没有新的速度消息时，使用扫描开始时仍在生效的指令；
    // 相邻发布跳变量为0。
    measured_minimum_abs_speed = std::abs(previous_speed);
  }

  if (minimum_abs_speed) {
    *minimum_abs_speed = measured_minimum_abs_speed;
  }
  if (maximum_speed_step) {
    *maximum_speed_step = measured_maximum_speed_step;
  }
  return measured_minimum_abs_speed + 1.0e-9 >=
         logic3_runtime_green_min_speed_ &&
         measured_maximum_speed_step <=
         logic3_runtime_green_max_speed_step_ + 1.0e-9;
}

void VisualLidarPathFollower::record_logic3_runtime_green_gate_log_sample(
  double steering_span, bool steering_measurement_valid,
  double minimum_abs_speed, double maximum_speed_step,
  bool speed_measurement_valid,
  double map_odom_translation_correction_abs,
  double map_odom_yaw_correction_abs_deg,
  bool steering_within_limit, bool speed_within_limits,
  bool translation_within_limit, bool yaw_within_limit)
{
  if (logic3_runtime_green_gate_log_rate_hz_ <= 0.0) {
    return;
  }

  const auto sample_time = std::chrono::steady_clock::now();
  auto & stats = logic3_runtime_green_gate_log_stats_;
  if (!stats.window_started) {
    stats.window_started = true;
    stats.window_started_at = sample_time;
  }

  ++stats.scan_count;
  stats.translation_correction_abs_sum += map_odom_translation_correction_abs;
  stats.yaw_correction_abs_deg_sum += map_odom_yaw_correction_abs_deg;
  if (!steering_measurement_valid) {
    ++stats.steering_invalid_count;
  } else {
    ++stats.steering_valid_count;
    stats.steering_span_sum += steering_span;
  }
  if (!speed_measurement_valid) {
    ++stats.speed_invalid_count;
  } else {
    ++stats.speed_valid_count;
    stats.minimum_abs_speed_sum += minimum_abs_speed;
    stats.maximum_speed_step_sum += maximum_speed_step;
  }
  if (steering_measurement_valid && speed_measurement_valid) {
    const int rejection_count =
      static_cast<int>(!steering_within_limit) +
      static_cast<int>(!speed_within_limits) +
      static_cast<int>(!translation_within_limit) +
      static_cast<int>(!yaw_within_limit);
    if (rejection_count == 0) {
      ++stats.motion_within_limits_count;
    } else if (rejection_count > 1) {
      ++stats.both_reject_count;
    } else if (!steering_within_limit) {
      ++stats.steering_reject_count;
    } else if (!speed_within_limits) {
      ++stats.speed_reject_count;
    } else if (!translation_within_limit) {
      ++stats.translation_reject_count;
    } else {
      ++stats.yaw_reject_count;
    }
  }

  const double elapsed_seconds =
    std::chrono::duration<double>(sample_time - stats.window_started_at).count();
  const double log_period_seconds = 1.0 / logic3_runtime_green_gate_log_rate_hz_;
  if (elapsed_seconds < log_period_seconds) {
    return;
  }

  const double average_steering_span = stats.steering_valid_count > 0 ?
    stats.steering_span_sum / static_cast<double>(stats.steering_valid_count) :
    std::numeric_limits<double>::quiet_NaN();
  const double average_minimum_abs_speed = stats.speed_valid_count > 0 ?
    stats.minimum_abs_speed_sum / static_cast<double>(stats.speed_valid_count) :
    std::numeric_limits<double>::quiet_NaN();
  const double average_maximum_speed_step = stats.speed_valid_count > 0 ?
    stats.maximum_speed_step_sum / static_cast<double>(stats.speed_valid_count) :
    std::numeric_limits<double>::quiet_NaN();
  const double average_translation_correction_abs =
    stats.translation_correction_abs_sum / static_cast<double>(stats.scan_count);
  const double average_yaw_correction_abs_deg =
    stats.yaw_correction_abs_deg_sum / static_cast<double>(stats.scan_count);
  const char * steering_gate_result = !steering_measurement_valid ? "无数据" :
    (steering_within_limit ? "通过" : "拒绝");
  const char * speed_gate_result = !speed_measurement_valid ? "无数据" :
    (speed_within_limits ? "通过" : "拒绝");
  RCLCPP_INFO(
    get_logger(),
    "运行绿/橙点运动统计[%.2fs]：阶段=%s，雷达帧=%llu，"
    "转角跨度当前/平均=%.3f/%.3frad（上限%.3f），"
    "最低速度幅值当前/平均=%.3f/%.3fm/s（下限%.3f），"
    "相邻发布速度跳变当前/平均=%.3f/%.3fm/s（上限%.3f），"
    "map->odom平移修正当前/平均=%.3f/%.3fm（上限%.3f），"
    "map->odom航向修正当前/平均=%.2f/%.2fdeg（上限%.2f），"
    "本帧[转向=%s，速度=%s，平移=%s，航向=%s]，"
    "阈值内=%llu/%llu，拒绝[转向=%llu，速度=%llu，平移=%llu，"
    "航向=%llu，多项=%llu，转向无数据=%llu，速度无数据=%llu]，"
    "运动门槛启用=%s。",
    elapsed_seconds, task_phase_name(task_phase_),
    static_cast<unsigned long long>(stats.scan_count),
    steering_span, average_steering_span,
    logic3_runtime_green_max_steering_span_,
    minimum_abs_speed, average_minimum_abs_speed,
    logic3_runtime_green_min_speed_,
    maximum_speed_step, average_maximum_speed_step,
    logic3_runtime_green_max_speed_step_,
    map_odom_translation_correction_abs, average_translation_correction_abs,
    logic3_runtime_green_max_map_odom_translation_correction_,
    map_odom_yaw_correction_abs_deg, average_yaw_correction_abs_deg,
    logic3_runtime_green_max_map_odom_yaw_correction_deg_,
    steering_gate_result,
    speed_gate_result,
    translation_within_limit ? "通过" : "拒绝",
    yaw_within_limit ? "通过" : "拒绝",
    static_cast<unsigned long long>(stats.motion_within_limits_count),
    static_cast<unsigned long long>(stats.scan_count),
    static_cast<unsigned long long>(stats.steering_reject_count),
    static_cast<unsigned long long>(stats.speed_reject_count),
    static_cast<unsigned long long>(stats.translation_reject_count),
    static_cast<unsigned long long>(stats.yaw_reject_count),
    static_cast<unsigned long long>(stats.both_reject_count),
    static_cast<unsigned long long>(stats.steering_invalid_count),
    static_cast<unsigned long long>(stats.speed_invalid_count),
    logic3_runtime_green_motion_gate_enabled_ ? "是" : "否");

  stats = Logic3RuntimeGreenGateLogStats{};
  // Keep the rate-limit clock continuous across statistic windows. Otherwise
  // the first scan after every log only restarts the clock and high log rates
  // can never reach the actual diagnostic scan rate.
  stats.window_started = true;
  stats.window_started_at = sample_time;
}

void VisualLidarPathFollower::update_logic3_static_cluster_tracks(
  const std::vector<VisualLidarPathFollower::ScanPointCluster> & clusters,
  const VisualLidarPathFollower::RobotPose & robot_pose,
  double match_score, std::uint64_t score_scan_sequence)
{
  const double now_seconds = now().seconds();
  bool formal_collection_changed = false;
  const double window_start =
    now_seconds - logic3_static_confirmation_window_seconds_;
  logic3_static_scan_times_.push_back(now_seconds);
  while (!logic3_static_scan_times_.empty() &&
    logic3_static_scan_times_.front() < window_start)
  {
    logic3_static_scan_times_.pop_front();
  }
  for (auto & track : logic3_static_cluster_tracks_) {
    while (!track.rolling_hit_times.empty() &&
      track.rolling_hit_times.front() < window_start)
    {
      track.rolling_hit_times.pop_front();
      track.rolling_hit_centers.pop_front();
    }
  }

  std::vector<bool> track_matched(logic3_static_cluster_tracks_.size(), false);
  for (const auto & cluster : clusters) {
    std::size_t best_track = logic3_static_cluster_tracks_.size();
    double best_distance = logic3_static_cluster_match_radius_;
    for (std::size_t index = 0; index < logic3_static_cluster_tracks_.size(); ++index) {
      if (track_matched[index]) {
        continue;
      }
      const auto & track = logic3_static_cluster_tracks_[index];
      if (track.entered_state1 && track.loss_pending &&
        now_seconds - track.loss_started_seconds > logic3_static_loss_grace_seconds_)
      {
        continue;
      }
      const double match_radius = track.entered_state1 && track.loss_pending ?
        logic3_static_reacquire_radius_ : logic3_static_cluster_match_radius_;
      const auto & center = track.stability_anchor;
      const double distance = std::hypot(
        cluster.center.x - center.x, cluster.center.y - center.y);
      if (distance <= match_radius && distance <= best_distance) {
        best_distance = distance;
        best_track = index;
      }
    }
    if (best_track == logic3_static_cluster_tracks_.size()) {
      // 一个障碍在同一帧仍可能残留两个碎片。若最近ID已经被本帧第一个
      // 碎片占用，旧代码会跳过它并为第二个碎片创建新ID。这里允许第二个
      // 碎片在身份半径内并回本帧已匹配ID，但同一雷达帧仍只计一次命中。
      std::size_t matched_track = logic3_static_cluster_tracks_.size();
      double matched_distance = logic3_static_cluster_match_radius_;
      for (std::size_t index = 0; index < logic3_static_cluster_tracks_.size(); ++index) {
        if (!track_matched[index]) {
          continue;
        }
        const auto & current_center = logic3_static_cluster_tracks_[index].cluster.center;
        const double distance = std::hypot(
          cluster.center.x - current_center.x,
          cluster.center.y - current_center.y);
        if (distance <= matched_distance) {
          matched_distance = distance;
          matched_track = index;
        }
      }
      if (matched_track < logic3_static_cluster_tracks_.size()) {
        auto & track = logic3_static_cluster_tracks_[matched_track];
        const Point2D existing_center = track.cluster.center;
        track.cluster.points.insert(
          track.cluster.points.end(), cluster.points.begin(), cluster.points.end());
        track.cluster.center = Point2D();
        if (track.cluster.points.empty()) {
          track.cluster.center.x = 0.5 * (existing_center.x + cluster.center.x);
          track.cluster.center.y = 0.5 * (existing_center.y + cluster.center.y);
        } else {
          for (const auto & point : track.cluster.points) {
            track.cluster.center.x += point.x;
            track.cluster.center.y += point.y;
          }
          const double point_count = static_cast<double>(track.cluster.points.size());
          track.cluster.center.x /= point_count;
          track.cluster.center.y /= point_count;
        }
        track.last_seen_seconds = now_seconds;
        if (!track.rolling_hit_centers.empty()) {
          track.rolling_hit_centers.back() = track.cluster.center;
        }
        continue;
      }

      Logic3StaticClusterTrack track;
      track.cluster = cluster;
      track.stability_anchor = cluster.center;
      track.last_seen_seconds = now_seconds;
      track.rolling_hit_times.push_back(now_seconds);
      track.rolling_hit_centers.push_back(cluster.center);
      logic3_static_cluster_tracks_.push_back(std::move(track));
      track_matched.push_back(true);
      continue;
    }

    auto & track = logic3_static_cluster_tracks_[best_track];
    const double stability_distance = std::hypot(
      cluster.center.x - track.stability_anchor.x,
      cluster.center.y - track.stability_anchor.y);
    const double obstacle_range = std::hypot(
      cluster.center.x - robot_pose.x, cluster.center.y - robot_pose.y);
    const double effective_stability_radius =
      logic3_static_stability_radius_for_range(obstacle_range);
    const bool recovered_during_grace = track.entered_state1 && track.loss_pending;

    // 状态1后的失联重捕只检查固定锚点0.20m；成功出现一次就恢复原ID，
    // 不再要求该时刻的2秒窗口立刻重新达到10帧。
    if (recovered_during_grace) {
      track.loss_pending = false;
      track.loss_started_seconds = 0.0;
    } else if (stability_distance > effective_stability_radius) {
      // 坐标越出其距离自适应稳定半径，视为该ID位置发生更新。只让该ID
      // 作废并从新位置重新进行2秒窗口确认，其他ID继续采样或保持READY。
      const std::uint64_t invalidated_id = track.identity_id;
      const bool invalidated_formal_track = track.confirmed;
      track.identity_id = 0;
      track.confirmed = false;
      track.entered_state1 = false;
      track.loss_pending = false;
      track.ready = false;
      track.rolling_hit_times.clear();
      track.rolling_hit_centers.clear();
      track.best_score_samples.clear();
      track.ready_obstacle = Logic3StaticObstacle();
      track.stability_anchor = cluster.center;
      if (invalidated_formal_track) {
        formal_collection_changed = true;
        if (debug_control_logs_) {
          RCLCPP_INFO(
            get_logger(),
            "debug: 静态ID%llu位置越出稳定半径，回到2秒窗口重新确认。",
            static_cast<unsigned long long>(invalidated_id));
        }
      }
    }
    track.cluster = cluster;
    track.last_seen_seconds = now_seconds;
    track.rolling_hit_times.push_back(now_seconds);
    track.rolling_hit_centers.push_back(cluster.center);
    track_matched[best_track] = true;
  }

  std::vector<bool> remove_track(logic3_static_cluster_tracks_.size(), false);
  for (std::size_t index = 0; index < logic3_static_cluster_tracks_.size(); ++index) {
    auto & track = logic3_static_cluster_tracks_[index];
    const int rolling_hits = static_cast<int>(track.rolling_hit_times.size());

    if (!track.confirmed &&
      rolling_hits >= logic3_static_confirmation_min_hits_)
    {
      Point2D averaged_center;
      for (const auto & center : track.rolling_hit_centers) {
        averaged_center.x += center.x;
        averaged_center.y += center.y;
      }
      const double hit_count =
        static_cast<double>(track.rolling_hit_centers.size());
      averaged_center.x /= hit_count;
      averaged_center.y /= hit_count;
      track.stability_anchor = averaged_center;
      track.identity_id = logic3_static_next_identity_id_++;
      track.confirmed = true;
      track.entered_state1 = false;
      track.loss_pending = false;
      track.loss_started_seconds = 0.0;
      track.ready = false;
      track.best_score_samples.clear();
      track.ready_obstacle = Logic3StaticObstacle();
      formal_collection_changed = true;
    }

    if (track.confirmed && track.entered_state1) {
      if (!track_matched[index] &&
        rolling_hits < logic3_static_confirmation_min_hits_ &&
        !track.loss_pending)
      {
        track.loss_pending = true;
        track.loss_started_seconds = now_seconds;
      }
      if (track.loss_pending &&
        now_seconds - track.loss_started_seconds > logic3_static_loss_grace_seconds_)
      {
        // 状态1后每个正式ID独立使用1秒失联宽限；超时才真正删除该ID
        // 并让整体退出状态1，但不清空其他ID的数据。
        remove_track[index] = true;
        formal_collection_changed = true;
        RCLCPP_WARN(
          get_logger(),
          "静态+动态方案静态ID%llu失联超过%.2fs：删除该ID并退出整体状态1；"
          "其他ID的命中窗口、采样和READY结果保持不变。",
          static_cast<unsigned long long>(track.identity_id),
          logic3_static_loss_grace_seconds_);
      }
    } else if (track.confirmed &&
      rolling_hits < logic3_static_confirmation_min_hits_)
    {
      // 正式ID还没参加过状态1时，不启用1秒失联宽限；它仍必须持续满足
      // 状态1前的2秒/10帧规则。
      remove_track[index] = true;
      formal_collection_changed = true;
      RCLCPP_INFO(
        get_logger(),
        "静态+动态方案静态ID%llu在进入状态1前不再满足2秒/%d帧：删除该ID。",
        static_cast<unsigned long long>(track.identity_id),
        logic3_static_confirmation_min_hits_);
    } else if (!track.confirmed && track.rolling_hit_times.empty()) {
      // 状态1前的临时候选完全由2秒窗口维持，不另设1秒内部ID保留。
      remove_track[index] = true;
    }
  }

  for (std::size_t index = remove_track.size(); index > 0; --index) {
    if (remove_track[index - 1]) {
      logic3_static_cluster_tracks_.erase(
        logic3_static_cluster_tracks_.begin() + static_cast<std::ptrdiff_t>(index - 1));
      track_matched.erase(
        track_matched.begin() + static_cast<std::ptrdiff_t>(index - 1));
    }
  }

  // 第三道硬保护：已有静态轨迹也必须满足身份半径内唯一。绿色坐标是
  // 最高分样本平均值，两个早期相距较远的碎片ID可能在稳定后收敛到一起；
  // 因此不能只在“新建ID”瞬间检查，还要在每帧READY计算前两两去重。
  auto representative_center = [](const Logic3StaticClusterTrack & track) {
      if (track.ready) {
        return track.ready_obstacle.cluster.center;
      }
      if (!track.best_score_samples.empty()) {
        Point2D center;
        for (const auto & sample : track.best_score_samples) {
          center.x += sample.cluster.center.x;
          center.y += sample.cluster.center.y;
        }
        const double count = static_cast<double>(track.best_score_samples.size());
        center.x /= count;
        center.y /= count;
        return center;
      }
      if (!track.rolling_hit_centers.empty()) {
        return track.rolling_hit_centers.back();
      }
      return track.cluster.center;
    };
  for (std::size_t first = 0; first < logic3_static_cluster_tracks_.size(); ++first) {
    std::size_t second = first + 1;
    while (second < logic3_static_cluster_tracks_.size()) {
      const Point2D first_center = representative_center(
        logic3_static_cluster_tracks_[first]);
      const Point2D second_center = representative_center(
        logic3_static_cluster_tracks_[second]);
      const double duplicate_distance = std::hypot(
        first_center.x - second_center.x, first_center.y - second_center.y);
      if (duplicate_distance > logic3_static_cluster_match_radius_) {
        ++second;
        continue;
      }

      auto & first_track = logic3_static_cluster_tracks_[first];
      auto & second_track = logic3_static_cluster_tracks_[second];
      const bool second_is_better_keeper =
        (!first_track.confirmed && second_track.confirmed) ||
        (first_track.confirmed == second_track.confirmed &&
        first_track.identity_id != 0 && second_track.identity_id != 0 &&
        second_track.identity_id < first_track.identity_id);
      if (second_is_better_keeper) {
        std::swap(first_track, second_track);
        std::swap(track_matched[first], track_matched[second]);
      }

      auto & keeper = logic3_static_cluster_tracks_[first];
      auto & duplicate = logic3_static_cluster_tracks_[second];
      const bool duplicate_was_formal = duplicate.confirmed;
      const bool either_matched = track_matched[first] || track_matched[second];

      if (duplicate.last_seen_seconds > keeper.last_seen_seconds ||
        (std::abs(duplicate.last_seen_seconds - keeper.last_seen_seconds) <= 1e-9 &&
        duplicate.cluster.points.size() > keeper.cluster.points.size()))
      {
        keeper.cluster = duplicate.cluster;
      }
      keeper.last_seen_seconds = std::max(
        keeper.last_seen_seconds, duplicate.last_seen_seconds);

      std::vector<std::pair<double, Point2D>> combined_hits;
      combined_hits.reserve(
        keeper.rolling_hit_times.size() + duplicate.rolling_hit_times.size());
      for (std::size_t index = 0;
        index < keeper.rolling_hit_times.size() &&
        index < keeper.rolling_hit_centers.size(); ++index)
      {
        combined_hits.emplace_back(
          keeper.rolling_hit_times[index], keeper.rolling_hit_centers[index]);
      }
      for (std::size_t index = 0;
        index < duplicate.rolling_hit_times.size() &&
        index < duplicate.rolling_hit_centers.size(); ++index)
      {
        combined_hits.emplace_back(
          duplicate.rolling_hit_times[index], duplicate.rolling_hit_centers[index]);
      }
      std::sort(
        combined_hits.begin(), combined_hits.end(),
        [](const auto & lhs, const auto & rhs) {return lhs.first < rhs.first;});
      keeper.rolling_hit_times.clear();
      keeper.rolling_hit_centers.clear();
      for (const auto & hit : combined_hits) {
        if (!keeper.rolling_hit_times.empty() &&
          std::abs(hit.first - keeper.rolling_hit_times.back()) <= 1e-6)
        {
          auto & center = keeper.rolling_hit_centers.back();
          center.x = 0.5 * (center.x + hit.second.x);
          center.y = 0.5 * (center.y + hit.second.y);
          continue;
        }
        if (hit.first >= window_start) {
          keeper.rolling_hit_times.push_back(hit.first);
          keeper.rolling_hit_centers.push_back(hit.second);
        }
      }

      std::vector<Logic3StaticScoredSample> combined_samples =
        keeper.best_score_samples;
      combined_samples.insert(
        combined_samples.end(), duplicate.best_score_samples.begin(),
        duplicate.best_score_samples.end());
      std::sort(
        combined_samples.begin(), combined_samples.end(),
        [](const Logic3StaticScoredSample & lhs,
        const Logic3StaticScoredSample & rhs) {
          if (std::abs(lhs.match_score - rhs.match_score) > 1e-9) {
            return lhs.match_score > rhs.match_score;
          }
          return lhs.scan_sequence > rhs.scan_sequence;
        });
      keeper.best_score_samples.clear();
      for (const auto & sample : combined_samples) {
        const bool duplicate_scan = std::any_of(
          keeper.best_score_samples.begin(), keeper.best_score_samples.end(),
          [&](const Logic3StaticScoredSample & selected) {
            return selected.scan_sequence == sample.scan_sequence;
          });
        if (duplicate_scan) {
          continue;
        }
        keeper.best_score_samples.push_back(sample);
        if (keeper.best_score_samples.size() >=
          static_cast<std::size_t>(logic3_static_best_score_samples_))
        {
          break;
        }
      }

      keeper.confirmed = keeper.confirmed || duplicate.confirmed;
      keeper.entered_state1 = keeper.entered_state1 || duplicate.entered_state1;
      keeper.loss_pending = either_matched ? false :
        (keeper.loss_pending && duplicate.loss_pending);
      if (!keeper.loss_pending) {
        keeper.loss_started_seconds = 0.0;
      }
      track_matched[first] = either_matched;

      if (keeper.best_score_samples.size() >=
        static_cast<std::size_t>(logic3_static_best_score_samples_))
      {
        ScanPointCluster averaged_cluster;
        for (const auto & sample : keeper.best_score_samples) {
          averaged_cluster.center.x += sample.cluster.center.x;
          averaged_cluster.center.y += sample.cluster.center.y;
        }
        const double sample_count =
          static_cast<double>(keeper.best_score_samples.size());
        averaged_cluster.center.x /= sample_count;
        averaged_cluster.center.y /= sample_count;
        const auto strongest_sample = std::max_element(
          keeper.best_score_samples.begin(), keeper.best_score_samples.end(),
          [](const Logic3StaticScoredSample & lhs,
          const Logic3StaticScoredSample & rhs) {
            if (std::abs(lhs.match_score - rhs.match_score) > 1e-9) {
              return lhs.match_score < rhs.match_score;
            }
            return lhs.scan_sequence < rhs.scan_sequence;
          });
        averaged_cluster.points = strongest_sample->cluster.points;
        if (averaged_cluster.points.empty()) {
          averaged_cluster.points.push_back(averaged_cluster.center);
        } else {
          const double translate_x =
            averaged_cluster.center.x - strongest_sample->cluster.center.x;
          const double translate_y =
            averaged_cluster.center.y - strongest_sample->cluster.center.y;
          for (auto & point : averaged_cluster.points) {
            point.x += translate_x;
            point.y += translate_y;
          }
        }
        keeper.ready_obstacle = Logic3StaticObstacle{averaged_cluster};
        keeper.ready = true;
      } else if (!keeper.ready && duplicate.ready) {
        keeper.ready_obstacle = duplicate.ready_obstacle;
        keeper.ready = true;
      }

      formal_collection_changed = formal_collection_changed || duplicate_was_formal;
      logic3_static_cluster_tracks_.erase(
        logic3_static_cluster_tracks_.begin() + static_cast<std::ptrdiff_t>(second));
      track_matched.erase(
        track_matched.begin() + static_cast<std::ptrdiff_t>(second));
    }
  }

  const bool has_formal_id = std::any_of(
    logic3_static_cluster_tracks_.begin(), logic3_static_cluster_tracks_.end(),
    [](const Logic3StaticClusterTrack & track) {
      return track.confirmed;
    });

  // 状态1是整个正式ID集合的统一状态，不再由单个ID独立进入。正式ID
  // 新增、删除或位置越界更新都会重新开始0.5秒稳定计时并退出状态1；
  // 未受影响ID的采样与READY平均结果全部保留。
  if (!has_formal_id) {
    logic3_static_state1_active_ = false;
    logic3_static_ready_ = false;
    logic3_static_collection_change_time_valid_ = false;
  } else if (formal_collection_changed) {
    logic3_static_state1_active_ = false;
    logic3_static_ready_ = false;
    logic3_static_collection_change_time_valid_ = true;
    logic3_static_collection_last_change_seconds_ = now_seconds;
  } else if (!logic3_static_collection_change_time_valid_) {
    logic3_static_collection_change_time_valid_ = true;
    logic3_static_collection_last_change_seconds_ = now_seconds;
  } else if (!logic3_static_state1_active_ &&
    now_seconds - logic3_static_collection_last_change_seconds_ >=
    logic3_static_collection_stable_seconds_)
  {
    logic3_static_state1_active_ = true;
    for (auto & track : logic3_static_cluster_tracks_) {
      if (track.confirmed) {
        track.entered_state1 = true;
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "静态+动态方案静态正式ID数量和位置连续%.2fs无更新：整体进入状态1。",
      logic3_static_collection_stable_seconds_);
  }

  // 只有整体状态1有效时才维护最高置信度样本池。当前帧未匹配、失联
  // 宽限或整帧地图匹配分数不足时只暂停采样，不影响ID的存在性判断。
  // READY之后仍允许更高分的新帧替换池内最低分帧，因此绿色中心会继续
  // 变准；identity_id和stability_anchor始终不随该平均中心移动。
  for (std::size_t index = 0; index < logic3_static_cluster_tracks_.size(); ++index) {
    auto & track = logic3_static_cluster_tracks_[index];
    if (!logic3_static_state1_active_ || !track.entered_state1 ||
      !track.confirmed || !track_matched[index] || track.loss_pending ||
      !std::isfinite(match_score) ||
      match_score + 1.0e-9 < logic3_static_min_match_score_)
    {
      continue;
    }

    Logic3StaticScoredSample new_sample;
    new_sample.match_score = match_score;
    new_sample.scan_sequence = score_scan_sequence;
    new_sample.cluster = track.cluster;
    bool sample_pool_changed = false;
    if (track.best_score_samples.size() <
      static_cast<std::size_t>(logic3_static_best_score_samples_))
    {
      track.best_score_samples.push_back(std::move(new_sample));
      sample_pool_changed = true;
    } else {
      const auto weakest_sample = std::min_element(
        track.best_score_samples.begin(), track.best_score_samples.end(),
        [](const Logic3StaticScoredSample & left,
        const Logic3StaticScoredSample & right) {
          if (std::abs(left.match_score - right.match_score) > 1.0e-9) {
            return left.match_score < right.match_score;
          }
          return left.scan_sequence < right.scan_sequence;
        });
      const bool higher_score =
        match_score > weakest_sample->match_score + 1.0e-9;
      const bool equal_score_but_newer =
        std::abs(match_score - weakest_sample->match_score) <= 1.0e-9 &&
        score_scan_sequence > weakest_sample->scan_sequence;
      if (higher_score || equal_score_but_newer) {
        *weakest_sample = std::move(new_sample);
        sample_pool_changed = true;
      }
    }

    if (!sample_pool_changed || track.best_score_samples.size() <
      static_cast<std::size_t>(logic3_static_best_score_samples_))
    {
      continue;
    }

    ScanPointCluster averaged_cluster;
    for (const auto & sample : track.best_score_samples) {
      averaged_cluster.center.x += sample.cluster.center.x;
      averaged_cluster.center.y += sample.cluster.center.y;
    }
    const double sample_count =
      static_cast<double>(track.best_score_samples.size());
    averaged_cluster.center.x /= sample_count;
    averaged_cluster.center.y /= sample_count;

    // 障碍几何只取最高分（同分取最新）帧，再整体平移到10帧平均中心；
    // 不把多帧原始点叠成一个虚假的宽障碍。
    const auto strongest_sample = std::max_element(
      track.best_score_samples.begin(), track.best_score_samples.end(),
      [](const Logic3StaticScoredSample & left,
      const Logic3StaticScoredSample & right) {
        if (std::abs(left.match_score - right.match_score) > 1.0e-9) {
          return left.match_score < right.match_score;
        }
        return left.scan_sequence < right.scan_sequence;
      });
    averaged_cluster.points = strongest_sample->cluster.points;
    if (averaged_cluster.points.empty()) {
      averaged_cluster.points.push_back(averaged_cluster.center);
    } else {
      const double translate_x =
        averaged_cluster.center.x - strongest_sample->cluster.center.x;
      const double translate_y =
        averaged_cluster.center.y - strongest_sample->cluster.center.y;
      for (auto & point : averaged_cluster.points) {
        point.x += translate_x;
        point.y += translate_y;
      }
    }

    // 静态阶段只冻结障碍身份和几何，不把某条参考路线的世界方向固化到
    // 障碍物。扫码选路后，每个连续局部路线命中段独立决定偏移方向。
    track.ready_obstacle = Logic3StaticObstacle{averaged_cluster};
    track.ready = true;
  }

  logic3_static_ready_ = logic3_static_state1_active_ && std::all_of(
    logic3_static_cluster_tracks_.begin(), logic3_static_cluster_tracks_.end(),
    [](const Logic3StaticClusterTrack & track) {
      return !track.confirmed || track.ready;
    });
}

double VisualLidarPathFollower::logic3_static_stability_radius_for_range(
  double obstacle_range) const
{
  if (obstacle_range <= logic3_static_stability_near_distance_) {
    return logic3_static_stability_radius_;
  }
  if (obstacle_range >= logic3_static_stability_far_distance_) {
    return logic3_static_far_stability_radius_;
  }
  const double interpolation =
    (obstacle_range - logic3_static_stability_near_distance_) /
    (logic3_static_stability_far_distance_ -
    logic3_static_stability_near_distance_);
  return logic3_static_stability_radius_ + interpolation *
         (logic3_static_far_stability_radius_ - logic3_static_stability_radius_);
}

std::vector<VisualLidarPathFollower::Logic3StaticObstacle>
VisualLidarPathFollower::logic3_ready_static_obstacles() const
{
  std::vector<Logic3StaticObstacle> obstacles;
  for (const auto & track : logic3_static_cluster_tracks_) {
    if (track.confirmed && track.entered_state1 && track.ready) {
      obstacles.push_back(track.ready_obstacle);
    }
  }
  return obstacles;
}

void VisualLidarPathFollower::maybe_log_logic3_static_obstacles(
  std::size_t current_frame_cluster_count,
  const std::vector<Logic3StaticObstacle> & obstacles)
{
  (void)current_frame_cluster_count;
  (void)obstacles;
  if (logic3_planning_pose_anchored_ || logic3_motion_candidate_ ||
    logic3_start_zone_motion_pending_)
  {
    return;
  }

  const double now_seconds = now().seconds();
  const double log_interval_seconds = 1.0 / logic3_static_status_log_rate_hz_;
  if (logic3_static_status_log_started_ &&
    now_seconds - logic3_last_static_status_log_seconds_ < log_interval_seconds)
  {
    return;
  }
  logic3_static_status_log_started_ = true;
  logic3_last_static_status_log_seconds_ = now_seconds;

  std::vector<const Logic3StaticClusterTrack *> fixed_tracks;
  for (const auto & track : logic3_static_cluster_tracks_) {
    if (track.entered_state1 && track.confirmed) {
      fixed_tracks.push_back(&track);
    }
  }
  std::sort(
    fixed_tracks.begin(), fixed_tracks.end(),
    [](const Logic3StaticClusterTrack * left,
    const Logic3StaticClusterTrack * right) {
      return left->identity_id < right->identity_id;
    });

  std::ostringstream status;
  status << "状态1=" << (logic3_static_state1_active_ ? 1 : 0)
         << "\n静态ID坐标：";
  status << std::fixed << std::setprecision(3);
  const std::size_t rolling_scan_count = logic3_static_scan_times_.size();
  std::size_t ready_track_count = 0;
  for (const auto * track : fixed_tracks) {
    if (track->ready) {
      ++ready_track_count;
    }
    const Point2D & display_center = track->ready ?
      track->ready_obstacle.cluster.center : track->stability_anchor;
    status << "\nID" << track->identity_id << ":("
           << display_center.x << ',' << display_center.y << ") 命中="
           << track->rolling_hit_times.size() << '/' << rolling_scan_count
           << " 最高分采样=" << track->best_score_samples.size()
           << '/' << logic3_static_best_score_samples_ << "；";
  }
  status << "\nREADY=" << (logic3_static_ready_ ? 1 : 0)
         << "（独立就绪=" << ready_track_count << '/' << fixed_tracks.size()
         << "；可信发车冻结最后一次完整READY快照）";
  RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
}

void VisualLidarPathFollower::record_logic3_lidar_avoidance_obstacle(
  const VisualLidarPathFollower::LidarObstacle & obstacle,
  const VisualLidarPathFollower::RobotPose & scan_to_base,
  const VisualLidarPathFollower::RobotPose & scan_to_map)
{
  if (!obstacle.found || !std::isfinite(obstacle.map_point.x) ||
    !std::isfinite(obstacle.map_point.y))
  {
    return;
  }
  // 红点入库的最后一道不变式：PID模式下已记录绿/橙掩码内
  // 不得产生实时避障红点。正常情况应已被find_lidar_obstacle过滤。
  if (tracking_controller_mode_ == "smac_pid" &&
    logic3_green_mask_contains(obstacle.map_point))
  {
    return;
  }

  // 保留原有红图ROI语义。scan_to_map=T(map<-scan)，
  // scan_to_base=T(base<-scan)，由两者还原采样时刻的base_link地图位置。
  const double map_to_base_yaw = scan_to_map.yaw - scan_to_base.yaw;
  const double cos_map_to_base = std::cos(map_to_base_yaw);
  const double sin_map_to_base = std::sin(map_to_base_yaw);
  const Point2D robot_map_point{
    scan_to_map.x -
    (cos_map_to_base * scan_to_base.x - sin_map_to_base * scan_to_base.y),
    scan_to_map.y -
    (sin_map_to_base * scan_to_base.x + cos_map_to_base * scan_to_base.y)};
  const double half_width = 0.5 * logic3_runtime_roi_width_;
  const double half_height = 0.5 * logic3_runtime_roi_height_;
  if ((logic3_runtime_roi_width_ > 0.0 &&
    std::abs(obstacle.map_point.x - robot_map_point.x) > half_width) ||
    (logic3_runtime_roi_height_ > 0.0 &&
    std::abs(obstacle.map_point.y - robot_map_point.y) > half_height) ||
    obstacle.map_point.y < logic3_runtime_channel_min_y_)
  {
    return;
  }

  // 红点只忠实记录实际雷达避障动作，不建立身份、不更新坐标、不去重。
  // 本函数只在避障动作由未激活切换为激活时调用，因此一次动作只记录一次；
  // 即使多次动作命中了相同位置，也分别保留并绘制。
  Logic3RedAvoidanceRecord record;
  record.record_id = logic3_next_red_record_id_++;
  record.center = obstacle.map_point;
  record.seen_in_task1 = task_phase_ == TaskPhase::TASK1;
  record.seen_after_task1 = !record.seen_in_task1;
  logic3_red_avoidance_records_.push_back(record);
}

bool VisualLidarPathFollower::activate_logic3_dynamic_takeover(
  const VisualLidarPathFollower::Point2D & red_point)
{
  if (!std::isfinite(red_point.x) || !std::isfinite(red_point.y) ||
    logic3_green_clusters_.empty())
  {
    return false;
  }
  if (logic3_green_dynamic_takeover_.size() > logic3_green_clusters_.size()) {
    logic3_green_dynamic_takeover_.resize(logic3_green_clusters_.size());
  }
  logic3_green_dynamic_takeover_.resize(logic3_green_clusters_.size(), false);
  if (logic3_green_route_clearance_failed_.size() > logic3_green_clusters_.size()) {
    logic3_green_route_clearance_failed_.resize(logic3_green_clusters_.size());
  }
  logic3_green_route_clearance_failed_.resize(
    logic3_green_clusters_.size(), false);

  std::size_t nearest_index = logic3_green_clusters_.size();
  double nearest_distance = logic3_red_takeover_radius_;
  for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
    const auto & center = logic3_green_clusters_[index].center;
    const double distance = std::hypot(
      red_point.x - center.x, red_point.y - center.y);
    if (distance <= nearest_distance) {
      nearest_distance = distance;
      nearest_index = index;
    }
  }
  if (nearest_index >= logic3_green_clusters_.size() ||
    logic3_green_dynamic_takeover_[nearest_index])
  {
    return false;
  }

  logic3_green_dynamic_takeover_[nearest_index] = true;
  const bool is_orange = logic3_orange_green_boundary_locked_ &&
    nearest_index >= logic3_orange_green_begin_index_;
  RCLCPP_WARN(
    get_logger(),
    "实际雷达避障动态接管%s点ID%zu：红点map(%.3f, %.3f)，"
    "永久点map(%.3f, %.3f)，距离=%.3fm<=%.2fm；"
    "本次发车只取消该ID静态偏移，绿/橙雷达掩码继续保留。",
    is_orange ? "橙" : "绿", nearest_index + 1,
    red_point.x, red_point.y,
    logic3_green_clusters_[nearest_index].center.x,
    logic3_green_clusters_[nearest_index].center.y,
    nearest_distance, logic3_red_takeover_radius_);
  return true;
}

bool VisualLidarPathFollower::logic3_identity_update_crosses_protected_route(
  const Point2D & identity_anchor, const Point2D & candidate_center) const
{
  const double move_x = candidate_center.x - identity_anchor.x;
  const double move_y = candidate_center.y - identity_anchor.y;
  if (move_x * move_x + move_y * move_y <= 1.0e-12) {
    return false;
  }

  // 检查“固定锚点→候选中心”是否真正穿过保护折线。只使用有限路线段，
  // 不把局部切线无限延伸，避免在弯角外侧误拒绝更新。
  auto crosses_route = [&](const std::vector<Waypoint> & route) {
      if (route.size() < 2) {
        return false;
      }
      constexpr double kSideTolerance = 1.0e-8;
      constexpr double kSegmentTolerance = 1.0e-7;
      for (std::size_t index = 0; index + 1 < route.size(); ++index) {
        const double route_x = route[index + 1].x - route[index].x;
        const double route_y = route[index + 1].y - route[index].y;
        const double route_length_squared = route_x * route_x + route_y * route_y;
        if (route_length_squared <= 1.0e-12) {
          continue;
        }
        const double anchor_side =
          route_x * (identity_anchor.y - route[index].y) -
          route_y * (identity_anchor.x - route[index].x);
        const double candidate_side =
          route_x * (candidate_center.y - route[index].y) -
          route_y * (candidate_center.x - route[index].x);

        // 锚点恰在线上时允许观测离开，否则该ID将永远无法更新。
        if (std::abs(anchor_side) <= kSideTolerance) {
          continue;
        }
        if (std::abs(candidate_side) > kSideTolerance &&
          anchor_side * candidate_side >= 0.0)
        {
          continue;
        }
        const double denominator = anchor_side - candidate_side;
        if (std::abs(denominator) <= 1.0e-12) {
          continue;
        }
        const double move_ratio = anchor_side / denominator;
        if (move_ratio <= kSegmentTolerance ||
          move_ratio > 1.0 + kSegmentTolerance)
        {
          continue;
        }
        const double intersection_x = identity_anchor.x + move_ratio * move_x;
        const double intersection_y = identity_anchor.y + move_ratio * move_y;
        const double route_ratio =
          ((intersection_x - route[index].x) * route_x +
          (intersection_y - route[index].y) * route_y) / route_length_squared;
        if (route_ratio >= -kSegmentTolerance &&
          route_ratio <= 1.0 + kSegmentTolerance)
        {
          return true;
        }
      }
      return false;
    };

  // Task1阶段也保护其未偏移基准路线，避免绿/橙/红永久中心因更新穿过
  // Task1巡线轨迹换到另一侧。
  if (task_phase_ == TaskPhase::TASK1) {
    return crosses_route(task1_original_route_);
  }

  const std::string direction = to_lower(selected_route_direction_);
  if (direction == "clockwise") {
    if (crosses_route(logic3_clockwise_update_guard_route_)) {
      return true;
    }
  } else if (direction == "counterclockwise") {
    if (crosses_route(logic3_counterclockwise_update_guard_route_)) {
      return true;
    }
  } else if (crosses_route(logic3_clockwise_update_guard_route_) ||
    crosses_route(logic3_counterclockwise_update_guard_route_))
  {
    // 二维码方向尚未确定时，两条原始路线都作为保护线。
    return true;
  }

  // 每次进入过渡或倒车后重规划都会替换这条最新无偏移ACB前缀。
  return crosses_route(logic3_latest_channel_entry_update_guard_route_);
}

int VisualLidarPathFollower::logic3_runtime_green_required_confirm_scans(
  std::size_t point_count) const
{
  if (logic3_runtime_green_confirm_scans_by_point_count_.empty()) {
    return 7;
  }
  // 参数的第5项同时覆盖5个及以上点；空簇按最严格的1点处理，避免
  // 异常输入绕过确认门槛。
  const std::size_t normalized_point_count = std::max<std::size_t>(1U, point_count);
  const std::size_t parameter_index = std::min<std::size_t>(
    normalized_point_count, logic3_runtime_green_confirm_scans_by_point_count_.size()) - 1U;
  return static_cast<int>(std::max<std::int64_t>(
      3, logic3_runtime_green_confirm_scans_by_point_count_[parameter_index]));
}

bool VisualLidarPathFollower::update_logic3_runtime_green_tracks(
  const std::vector<VisualLidarPathFollower::ScanPointCluster> & clusters,
  double speed, double speed_step,
  double map_odom_translation_correction,
  double map_odom_yaw_correction_deg)
{
  // 身份锚点与logic3_green_clusters_永远按索引对齐。旧状态或
  // 异常路径缺失锚点时，仅用当前中心补齐一次；后续绝不随
  // cluster.center更新。
  if (logic3_green_identity_anchors_.size() > logic3_green_clusters_.size()) {
    logic3_green_identity_anchors_.resize(logic3_green_clusters_.size());
  }
  while (logic3_green_identity_anchors_.size() < logic3_green_clusters_.size()) {
    logic3_green_identity_anchors_.push_back(
      logic3_green_clusters_[logic3_green_identity_anchors_.size()].center);
  }
  if (logic3_green_dynamic_takeover_.size() > logic3_green_clusters_.size()) {
    logic3_green_dynamic_takeover_.resize(logic3_green_clusters_.size());
  }
  logic3_green_dynamic_takeover_.resize(logic3_green_clusters_.size(), false);
  if (logic3_green_route_clearance_failed_.size() > logic3_green_clusters_.size()) {
    logic3_green_route_clearance_failed_.resize(logic3_green_clusters_.size());
  }
  logic3_green_route_clearance_failed_.resize(
    logic3_green_clusters_.size(), false);

  std::vector<bool> track_matched(logic3_runtime_green_tracks_.size(), false);
  bool route_replan_required = false;

  auto refresh_recent_cluster = [&](Logic3RuntimeGreenTrack & track) {
      Point2D averaged_center;
      for (const auto & sample : track.recent_samples) {
        averaged_center.x += sample.center.x;
        averaged_center.y += sample.center.y;
      }
      const double sample_count = static_cast<double>(track.recent_samples.size());
      averaged_center.x /= sample_count;
      averaged_center.y /= sample_count;

      // 位置使用最近高分帧的中心平均，点云形状只取最新帧并整体平移；
      // 这样不会把车辆运动期间的多帧点云拖成一个很宽的障碍物。
      track.cluster = track.recent_samples.back();
      const double translate_x = averaged_center.x - track.cluster.center.x;
      const double translate_y = averaged_center.y - track.cluster.center.y;
      for (auto & point : track.cluster.points) {
        point.x += translate_x;
        point.y += translate_y;
      }
      track.cluster.center = averaged_center;
    };

  auto find_green_identity = [&](const Point2D & center) {
      std::size_t best_index = logic3_green_clusters_.size();
      double best_distance = logic3_runtime_green_match_radius_;
      for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
        // 身份ID关联只和永久库当前中心比较；固定锚点在身份确定后再
        // 独立决定这个ID是否允许接受新坐标。
        const auto & current_center = logic3_green_clusters_[index].center;
        const double distance = std::hypot(
          center.x - current_center.x,
          center.y - current_center.y);
        if (distance <= best_distance) {
          best_distance = distance;
          best_index = index;
        }
      }
      return best_index;
    };
  auto find_current_permanent_coverage = [&](const Point2D & center) {
      std::size_t best_index = logic3_green_clusters_.size();
      double best_distance = logic3_old_cluster_radius_;
      for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
        const auto & current_center = logic3_green_clusters_[index].center;
        const double distance = std::hypot(
          center.x - current_center.x, center.y - current_center.y);
        if (distance <= best_distance) {
          best_distance = distance;
          best_index = index;
        }
      }
      return best_index;
  };

  for (const auto & cluster : clusters) {
    const int observation_required_confirm_scans =
      logic3_runtime_green_required_confirm_scans(cluster.points.size());
    std::size_t best_track = logic3_runtime_green_tracks_.size();
    double best_distance = logic3_runtime_green_match_radius_;
    for (std::size_t index = 0; index < logic3_runtime_green_tracks_.size(); ++index) {
      if (track_matched[index]) {
        continue;
      }
      const auto & candidate_track = logic3_runtime_green_tracks_[index];
      const bool has_permanent_identity = candidate_track.linked_to_green &&
        candidate_track.green_index < logic3_green_clusters_.size();
      const auto & center = has_permanent_identity ?
        logic3_green_clusters_[candidate_track.green_index].center :
        candidate_track.latest_center;
      const double distance = std::hypot(
        cluster.center.x - center.x, cluster.center.y - center.y);
      if (distance <= best_distance) {
        best_distance = distance;
        best_track = index;
      }
    }

    if (best_track == logic3_runtime_green_tracks_.size()) {
      Logic3RuntimeGreenTrack track;
      track.cluster = cluster;
      track.latest_center = cluster.center;
      track.consecutive_hits = 1;
      track.required_confirm_scans = observation_required_confirm_scans;
      track.recent_samples.push_back(cluster);
      const std::size_t green_index = find_green_identity(cluster.center);
      if (green_index < logic3_green_clusters_.size()) {
        track.linked_to_green = true;
        track.green_index = green_index;
      }
      logic3_runtime_green_tracks_.push_back(std::move(track));
      track_matched.push_back(true);
      best_track = logic3_runtime_green_tracks_.size() - 1;
    } else {
      auto & track = logic3_runtime_green_tracks_[best_track];
      if (track.missing_count > 0) {
        // 中断后重新出现必须重新积累连续高分帧；永久绿点本身不受影响。
        track.confirmed = false;
        track.consecutive_hits = 0;
        track.required_confirm_scans = observation_required_confirm_scans;
        track.recent_samples.clear();
      } else {
        // 同一段连续观测内，点数偶尔变少时采用见过的最严格门槛，不能
        // 因后一帧点数重新变多就立刻放宽。
        track.required_confirm_scans = std::max(
          track.required_confirm_scans, observation_required_confirm_scans);
      }
      track.latest_center = cluster.center;
      track.missing_count = 0;
      ++track.consecutive_hits;
      track.recent_samples.push_back(cluster);
      const int retained_sample_count = std::max(
        logic3_runtime_green_average_samples_, track.required_confirm_scans);
      while (track.recent_samples.size() >
        static_cast<std::size_t>(retained_sample_count))
      {
        track.recent_samples.pop_front();
      }
      track_matched[best_track] = true;
    }

    auto & track = logic3_runtime_green_tracks_[best_track];
    // 若连续段后半程才出现更严格的低点数观测，缓存还需要补满新的完整
    // 确认窗口；不能只凭已经很大的consecutive_hits立即采用该帧。
    if (track.consecutive_hits < track.required_confirm_scans ||
      track.recent_samples.size() <
      static_cast<std::size_t>(track.required_confirm_scans))
    {
      continue;
    }
    refresh_recent_cluster(track);
    track.confirmed = true;

    // 候选确认时再查一次全局绿色集合，防止两个临时候选先后确认后
    // 为同一实体追加两个永久绿点。
    if (!track.linked_to_green ||
      track.green_index >= logic3_green_clusters_.size())
    {
      const std::size_t green_index = find_green_identity(track.cluster.center);
      if (green_index < logic3_green_clusters_.size()) {
        track.linked_to_green = true;
        track.green_index = green_index;
      } else {
        // READY锚点只负责约束既有绿色身份不能被连续小步拖走。判断一个
        // 候选能否成为“新增橙点”时，必须再与所有绿/橙永久点的当前更新
        // 中心比较。0.25m只负责严格身份关联；无法关联身份后，只要候选仍在
        // 任一永久点配置过滤覆盖范围内，就视为该永久障碍的点云抖动，禁止
        // 新增橙点，也不把原ID拉出固定身份锚点。
        const std::size_t current_center_index =
          find_current_permanent_coverage(track.cluster.center);
        if (current_center_index < logic3_green_clusters_.size()) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "运行绿/橙点候选map(%.3f, %.3f)无法通过%.2fm身份半径关联，"
            "但位于永久点%zu当前中心%.2fm过滤覆盖内，不新增橙点。",
            track.cluster.center.x, track.cluster.center.y,
            logic3_runtime_green_match_radius_, current_center_index + 1,
            logic3_old_cluster_radius_);
          continue;
        }
        track.green_index = logic3_green_clusters_.size();
        track.linked_to_green = true;
        logic3_green_clusters_.push_back(track.cluster);
        // 发车后新橙点没有READY坐标，以首次连续确认的
        // 中心作为它的固定身份锚点。
        logic3_green_identity_anchors_.push_back(track.cluster.center);
        logic3_green_dynamic_takeover_.push_back(false);
        logic3_green_route_clearance_failed_.push_back(false);
        route_replan_required = true;
        const double seconds_since_trusted_departure =
          task1_departure_timer_started_ ?
          std::max(
            0.0,
            std::chrono::duration<double>(
              std::chrono::steady_clock::now() - task1_departure_started_at_).count()) :
          0.0;
        RCLCPP_WARN(
          get_logger(),
          "@@@@@@@橙点：(%.3f，%.3f)@@@@@@@此时匹配score为：%.3f；"
          "此时距离可信发车的时间为：%.3f秒；"
          "speed=%.3fm/s；speed_step=%.3fm/s；"
          "map_odom_translation_correction=%.3fm；"
          "map_odom_yaw_correction_deg=%.3fdeg。",
          track.cluster.center.x, track.cluster.center.y,
          latest_lidar_match_score_, seconds_since_trusted_departure,
          speed, speed_step, map_odom_translation_correction,
          map_odom_yaw_correction_deg);
        continue;
      }
    }

    const std::size_t green_index = track.green_index;
    const std::size_t departure_green_count =
      logic3_orange_green_boundary_locked_ ?
      std::min(logic3_orange_green_begin_index_, logic3_green_clusters_.size()) :
      logic3_green_clusters_.size();
    if (logic3_runtime_orange_only_update_enabled_ &&
      green_index < departure_green_count)
    {
      // 橙点专用更新模式：可信发车时的READY绿色身份、坐标和点云保持冻结。
      // 它们发车时已经产生的静态绕行及配置半径雷达掩码继续有效；这里只禁止
      // 运行高分观测移动绿色中心并触发新的绿色路线偏移。
      continue;
    }

    // 身份已经确定后才进入固定锚点更新约束。锚点只保存READY绿色或
    // 橙点首次确认的位置，绝不随永久库中心更新；越界只拒绝本次坐标，
    // 不解除ID关联，也绝不能因此新增一个橙点。
    const auto & identity_anchor = logic3_green_identity_anchors_[green_index];
    const double anchor_distance = std::hypot(
      track.cluster.center.x - identity_anchor.x,
      track.cluster.center.y - identity_anchor.y);
    if (anchor_distance > logic3_runtime_green_anchor_update_radius_ + 1.0e-9) {
      continue;
    }

    // 可选的路线换边保护。允许跨线时完全跳过几何检查，身份关联半径、
    // 固定锚点更新半径、永久库写入阈值和重规划阈值仍照常生效。
    if (!logic3_runtime_green_allow_route_crossing_ &&
      logic3_identity_update_crosses_protected_route(
        identity_anchor, track.cluster.center))
    {
      continue;
    }

    const auto & previous_center = logic3_green_clusters_[green_index].center;
    const double distance_from_permanent = std::hypot(
      track.cluster.center.x - previous_center.x,
      track.cluster.center.y - previous_center.y);
    // 小于写入阈值时永久库保持原值；临时高分轨迹仍持续观察，因此多帧
    // 小幅同向变化可以累计，最终达到阈值后再一次性写入。
    if (distance_from_permanent + 1.0e-9 <
      logic3_runtime_green_update_min_distance_)
    {
      continue;
    }
    logic3_green_clusters_[green_index] = track.cluster;
    const bool has_planned_center =
      green_index < logic3_green_last_planned_centers_.size();
    const double distance_from_planned = has_planned_center ? std::hypot(
      track.cluster.center.x - logic3_green_last_planned_centers_[green_index].x,
      track.cluster.center.y - logic3_green_last_planned_centers_[green_index].y) :
      std::numeric_limits<double>::infinity();
    if (!has_planned_center ||
      distance_from_planned + 1.0e-9 >= logic3_runtime_green_replan_min_distance_)
    {
      route_replan_required = true;
    }
  }

  for (std::size_t index = 0; index < logic3_runtime_green_tracks_.size(); ++index) {
    if (!track_matched[index]) {
      auto & track = logic3_runtime_green_tracks_[index];
      ++track.missing_count;
      track.consecutive_hits = 0;
    }
  }
  logic3_runtime_green_tracks_.erase(
    std::remove_if(
      logic3_runtime_green_tracks_.begin(), logic3_runtime_green_tracks_.end(),
      [&](const Logic3RuntimeGreenTrack & track) {
        // 这里只清除短期关联轨迹；已经写入的永久绿/橙点不会因失联删除。
        return track.missing_count >=
        logic3_runtime_green_candidate_release_scans_;
      }),
    logic3_runtime_green_tracks_.end());
  // 历史红点只用于绘图，绝不因绿/橙点后来更新靠近而反向补做接管。
  // 路径接管只能由apply_channel_lidar_obstacle_avoidance在动作触发瞬间决定。
  return route_replan_required;
}
