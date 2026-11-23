#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "rcutils/error_handling.h"
#include "rcutils/logging.h"

void VisualLidarPathFollower::reset_waypoint_shift_state()
{
  task3_route_available_ = false;
  task3_route_active_ = false;
  return_stop_zone_entry_active_ = false;
  task3_start_index_ = waypoints_.size();
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    if (waypoints_[index].task3_start) {
      task3_route_available_ = true;
      task3_start_index_ = index;
      break;
    }
  }
  // 保留航点文件原本的Task3末点。路线外(0,0)保底点不写入waypoints_，
  // 不参与下面的静态偏移、circle_envelope或路径PID缓存。
  original_waypoints_ = waypoints_;
  waypoint_shift_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_missing_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_dynamic_directions_.assign(waypoints_.size(), 0);
  waypoint_shift_world_directions_.assign(waypoints_.size(), Point2D());
  waypoint_shift_source_indices_.assign(waypoints_.size(), waypoints_.size());
  waypoint_shift_obstacle_centers_.assign(waypoints_.size(), Point2D());
  waypoint_shift_forbidden_radii_.assign(waypoints_.size(), 0.0);
  logic3_frozen_base_offsets_.clear();
  logic3_frozen_base_shift_directions_.clear();
  logic3_frozen_base_source_indices_.clear();
  logic3_frozen_base_obstacle_centers_.clear();
  logic3_frozen_base_forbidden_radii_.clear();
  logic3_runtime_green_tracks_.clear();
  if (!channel_lidar_obstacle_active_) {
    channel_lidar_avoid_direction_ = 0;
  }
  processed_waypoint_shift_scan_sequence_ = scan_sequence_;
  rebuild_path_pid_route_cache();
  reset_path_pid_state();
}

bool VisualLidarPathFollower::logic3_ready_snapshot_needs_preplan(
  const Logic3PlanningSnapshot & snapshot) const
{
  if (!snapshot.valid) {
    return false;
  }
  const Logic3PlanningSnapshot * reference_snapshot = nullptr;
  if (logic3_ready_preplan_candidate_pending_ &&
    logic3_ready_preplan_candidate_snapshot_.valid)
  {
    reference_snapshot = &logic3_ready_preplan_candidate_snapshot_;
  } else if (logic3_ready_preplan_valid_ &&
    logic3_ready_preplanned_snapshot_.valid)
  {
    reference_snapshot = &logic3_ready_preplanned_snapshot_;
  }
  if (!reference_snapshot ||
    snapshot.obstacles.size() != reference_snapshot->obstacles.size())
  {
    return true;
  }

  // READY障碍按稳定身份顺序导出。数量相同时，只在任一可靠中心发生
  // 配置量级的实际移动后重算；点云细碎抖动不会重复触发完整路线规划。
  for (std::size_t index = 0; index < snapshot.obstacles.size(); ++index) {
    const auto & current = snapshot.obstacles[index].cluster.center;
    const auto & planned = reference_snapshot->obstacles[index].cluster.center;
    if (std::hypot(current.x - planned.x, current.y - planned.y) + 1.0e-9 >=
      logic3_ready_preplan_min_center_change_)
    {
      return true;
    }
  }
  return false;
}

void VisualLidarPathFollower::preplan_logic3_ready_route(
  const Logic3PlanningSnapshot & snapshot)
{
  if (!logic3_ready_preplan_enabled_ || !snapshot.valid ||
    logic3_planning_pose_anchored_ || logic3_motion_candidate_ ||
    logic3_start_zone_motion_pending_ || logic3_ready_preplanning_active_ ||
    waypoints_.empty() || original_waypoints_.size() != waypoints_.size() ||
    !logic3_ready_snapshot_needs_preplan(snapshot))
  {
    return;
  }

  const auto planning_started_at = std::chrono::steady_clock::now();
  if (logic3_ready_preplan_last_at_valid_ &&
    std::chrono::duration<double>(
      planning_started_at - logic3_ready_preplan_last_at_).count() + 1.0e-9 <
    logic3_ready_preplan_min_interval_)
  {
    // 冷却期间logic3_last_complete_ready_snapshot_仍会继续刷新。后续新
    // 雷达帧到来且满0.50s后，直接对最新快照规划，不排队处理中间版本。
    return;
  }

  // 规划只发生在可信发车前的静止READY窗口。先保存上一次已提交路线，
  // 再借用冻结簇回放算法生成候选；候选完成后恢复旧路线，绝不直接覆盖。
  // 只有后续新里程计再次确认静止才会提交；若先检测到发车则丢弃候选。
  auto saved_green_clusters = std::move(logic3_green_clusters_);
  auto saved_identity_anchors = std::move(logic3_green_identity_anchors_);
  auto saved_dynamic_takeover = std::move(logic3_green_dynamic_takeover_);
  auto saved_clearance_failed = std::move(logic3_green_route_clearance_failed_);
  auto saved_waypoints = std::move(waypoints_);
  auto saved_offsets = std::move(waypoint_shift_offsets_);
  auto saved_pending_offsets = std::move(waypoint_shift_pending_offsets_);
  auto saved_pending_counts = std::move(waypoint_shift_pending_counts_);
  auto saved_missing_counts = std::move(waypoint_shift_missing_counts_);
  auto saved_dynamic_directions = std::move(waypoint_shift_dynamic_directions_);
  auto saved_shift_directions = std::move(waypoint_shift_world_directions_);
  auto saved_source_indices = std::move(waypoint_shift_source_indices_);
  auto saved_obstacle_centers = std::move(waypoint_shift_obstacle_centers_);
  auto saved_forbidden_radii = std::move(waypoint_shift_forbidden_radii_);
  const bool saved_replay = logic3_replay_frozen_clusters_;
  const bool saved_force_full_route = logic3_force_full_route_;
  const bool saved_force_confirm = logic3_force_confirm_;

  logic3_green_clusters_.clear();
  logic3_green_clusters_.reserve(snapshot.obstacles.size());
  for (const auto & obstacle : snapshot.obstacles) {
    logic3_green_clusters_.push_back(obstacle.cluster);
  }
  logic3_green_identity_anchors_.clear();
  logic3_green_dynamic_takeover_.assign(logic3_green_clusters_.size(), false);
  logic3_green_route_clearance_failed_.assign(logic3_green_clusters_.size(), false);

  waypoints_ = original_waypoints_;
  waypoint_shift_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_missing_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_dynamic_directions_.assign(waypoints_.size(), 0);
  waypoint_shift_world_directions_.assign(waypoints_.size(), Point2D());
  waypoint_shift_source_indices_.assign(waypoints_.size(), waypoints_.size());
  waypoint_shift_obstacle_centers_.assign(waypoints_.size(), Point2D());
  waypoint_shift_forbidden_radii_.assign(waypoints_.size(), 0.0);

  logic3_ready_preplanning_active_ = true;
  logic3_replay_frozen_clusters_ = true;
  logic3_force_full_route_ = true;
  logic3_force_confirm_ = true;
  update_waypoint_shifts(RobotPose(), 0);
  logic3_ready_preplan_candidate_waypoints_ = std::move(waypoints_);
  logic3_ready_preplan_candidate_offsets_ = std::move(waypoint_shift_offsets_);
  logic3_ready_preplan_candidate_shift_directions_ =
    std::move(waypoint_shift_world_directions_);
  logic3_ready_preplan_candidate_source_indices_ =
    std::move(waypoint_shift_source_indices_);
  logic3_ready_preplan_candidate_obstacle_centers_ =
    std::move(waypoint_shift_obstacle_centers_);
  logic3_ready_preplan_candidate_forbidden_radii_ =
    std::move(waypoint_shift_forbidden_radii_);
  logic3_ready_preplan_candidate_clearance_failed_ =
    logic3_green_route_clearance_failed_;
  logic3_replay_frozen_clusters_ = saved_replay;
  logic3_force_full_route_ = saved_force_full_route;
  logic3_force_confirm_ = saved_force_confirm;
  logic3_ready_preplanning_active_ = false;

  logic3_green_clusters_ = std::move(saved_green_clusters);
  logic3_green_identity_anchors_ = std::move(saved_identity_anchors);
  logic3_green_dynamic_takeover_ = std::move(saved_dynamic_takeover);
  logic3_green_route_clearance_failed_ = std::move(saved_clearance_failed);
  waypoints_ = std::move(saved_waypoints);
  waypoint_shift_offsets_ = std::move(saved_offsets);
  waypoint_shift_pending_offsets_ = std::move(saved_pending_offsets);
  waypoint_shift_pending_counts_ = std::move(saved_pending_counts);
  waypoint_shift_missing_counts_ = std::move(saved_missing_counts);
  waypoint_shift_dynamic_directions_ = std::move(saved_dynamic_directions);
  waypoint_shift_world_directions_ = std::move(saved_shift_directions);
  waypoint_shift_source_indices_ = std::move(saved_source_indices);
  waypoint_shift_obstacle_centers_ = std::move(saved_obstacle_centers);
  waypoint_shift_forbidden_radii_ = std::move(saved_forbidden_radii);
  rebuild_path_pid_route_cache();
  reset_path_pid_state();

  logic3_ready_preplan_candidate_snapshot_ = snapshot;
  logic3_ready_preplan_candidate_pending_ =
    !logic3_ready_preplan_candidate_waypoints_.empty();
  logic3_ready_preplan_candidate_odom_sequence_ = chassis_odom_sequence_;
  logic3_ready_preplan_last_at_ = planning_started_at;
  logic3_ready_preplan_last_at_valid_ = true;
}

void VisualLidarPathFollower::resolve_logic3_ready_preplan_candidate(bool moving)
{
  if (!logic3_ready_preplan_candidate_pending_) {
    return;
  }

  const auto clear_candidate = [&]() {
      logic3_ready_preplan_candidate_pending_ = false;
      logic3_ready_preplan_candidate_snapshot_ = Logic3PlanningSnapshot();
      logic3_ready_preplan_candidate_waypoints_.clear();
      logic3_ready_preplan_candidate_offsets_.clear();
      logic3_ready_preplan_candidate_shift_directions_.clear();
      logic3_ready_preplan_candidate_source_indices_.clear();
      logic3_ready_preplan_candidate_obstacle_centers_.clear();
      logic3_ready_preplan_candidate_forbidden_radii_.clear();
      logic3_ready_preplan_candidate_clearance_failed_.clear();
    };

  // 规划开始后出现任何可信运动迹象，都把本次候选视为过期。现行单线程
  // executor会在规划函数返回后处理这条里程计；由于候选从未覆盖活动路线，
  // 此时清掉它即可严格继续使用上一次完整路线。
  if (moving || logic3_motion_candidate_ || logic3_start_zone_motion_pending_) {
    clear_candidate();
    return;
  }
  if (chassis_odom_sequence_ <= logic3_ready_preplan_candidate_odom_sequence_) {
    return;
  }

  waypoints_ = std::move(logic3_ready_preplan_candidate_waypoints_);
  waypoint_shift_offsets_ = std::move(logic3_ready_preplan_candidate_offsets_);
  waypoint_shift_pending_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_missing_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_dynamic_directions_.assign(waypoints_.size(), 0);
  waypoint_shift_world_directions_ =
    std::move(logic3_ready_preplan_candidate_shift_directions_);
  waypoint_shift_source_indices_ =
    std::move(logic3_ready_preplan_candidate_source_indices_);
  waypoint_shift_obstacle_centers_ =
    std::move(logic3_ready_preplan_candidate_obstacle_centers_);
  waypoint_shift_forbidden_radii_ =
    std::move(logic3_ready_preplan_candidate_forbidden_radii_);
  logic3_ready_preplanned_route_clearance_failed_ =
    std::move(logic3_ready_preplan_candidate_clearance_failed_);
  logic3_ready_preplanned_snapshot_ = logic3_ready_preplan_candidate_snapshot_;
  logic3_ready_preplan_valid_ = true;
  rebuild_path_pid_route_cache();
  reset_path_pid_state();
  clear_candidate();
}

void VisualLidarPathFollower::rebuild_logic3_route_from_green_clusters(
  const char * reason)
{
  if (!logic3_planning_pose_anchored_ || waypoints_.empty() ||
    original_waypoints_.size() != waypoints_.size() ||
    task_phase_ == TaskPhase::UNKNOWN ||
    task_phase_ == TaskPhase::FINISHED)
  {
    return;
  }
  if (enable_static_map_filter_ && require_map_for_lidar_assist_ &&
    (!latest_map_ || !latest_restriction_map_))
  {
    return;
  }
  const auto planning_started_at = std::chrono::steady_clock::now();

  // 每次都从当前阶段的原始路线重算。未被动态接管的永久点继续贡献静态
  // 偏移；接管点仍留在点库和图片中，但本次发车不再影响静态路线。
  waypoints_ = original_waypoints_;
  waypoint_shift_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_missing_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_dynamic_directions_.assign(waypoints_.size(), 0);
  waypoint_shift_world_directions_.assign(waypoints_.size(), Point2D());
  waypoint_shift_source_indices_.assign(waypoints_.size(), waypoints_.size());
  waypoint_shift_obstacle_centers_.assign(waypoints_.size(), Point2D());
  waypoint_shift_forbidden_radii_.assign(waypoints_.size(), 0.0);

  logic3_replay_frozen_clusters_ = true;
  logic3_force_full_route_ = true;
  logic3_force_confirm_ = true;
  update_waypoint_shifts(RobotPose(), 0);
  logic3_force_confirm_ = false;
  logic3_force_full_route_ = false;
  logic3_replay_frozen_clusters_ = false;

  logic3_frozen_base_offsets_ = waypoint_shift_offsets_;
  logic3_frozen_base_shift_directions_ = waypoint_shift_world_directions_;
  logic3_frozen_base_source_indices_ = waypoint_shift_source_indices_;
  logic3_frozen_base_obstacle_centers_ = waypoint_shift_obstacle_centers_;
  logic3_frozen_base_forbidden_radii_ = waypoint_shift_forbidden_radii_;
  logic3_green_last_planned_centers_.clear();
  logic3_green_last_planned_centers_.reserve(logic3_green_clusters_.size());
  for (const auto & green_cluster : logic3_green_clusters_) {
    logic3_green_last_planned_centers_.push_back(green_cluster.center);
  }
  logic3_green_route_replan_pending_ = false;

  logic3_baseline_original_route_ = original_waypoints_;
  logic3_baseline_route_ = waypoints_;
  logic3_visualization_route_ = waypoints_;
  if (task_phase_ == TaskPhase::TASK1) {
    task1_frozen_route_ = waypoints_;
  } else {
    final_frozen_route_ = waypoints_;
  }
  reset_path_pid_state();
  publish_debug_route();
  const auto planning_finished_at = std::chrono::steady_clock::now();
  logic3_last_route_replan_duration_ms_ = 1000.0 *
    std::chrono::duration<double>(
    planning_finished_at - planning_started_at).count();
  logic3_last_runtime_route_replan_at_ = planning_finished_at;
  logic3_last_runtime_route_replan_at_valid_ = true;
  RCLCPP_WARN(
    get_logger(), "%s：用%zu个永久绿点重新计算%s路线；耗时=%.2fms。",
    reason ? reason : "运行绿点更新", logic3_green_clusters_.size(),
    task_phase_name(task_phase_), logic3_last_route_replan_duration_ms_);
}

void VisualLidarPathFollower::freeze_logic3_route()
{
  const Logic3PlanningSnapshot & snapshot = logic3_motion_pending_snapshot_;
  logic3_green_clusters_.clear();
  logic3_green_identity_anchors_.clear();
  logic3_green_dynamic_takeover_.clear();
  logic3_green_route_clearance_failed_.clear();
  if (snapshot.valid) {
    for (const auto & obstacle : snapshot.obstacles) {
      logic3_green_clusters_.push_back(obstacle.cluster);
      // 可信发车后绿色身份始终围绕这个READY坐标关联；
      // obstacle.cluster.center本身仍允许由运行高分观测修正。
      logic3_green_identity_anchors_.push_back(obstacle.cluster.center);
      logic3_green_dynamic_takeover_.push_back(false);
      logic3_green_route_clearance_failed_.push_back(false);
    }
    if (snapshot.obstacles.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "静态+动态方案可信发车时没有已独立完成状态1和10次平均的绿色簇；"
        "按规则不使用原始帧兜底，将冻结空快照。");
    }
  } else {
    RCLCPP_WARN(
      get_logger(),
      "静态+动态方案发车候选时状态1或READY无效；按规则不使用原始帧兜底，将记录为空。");
  }
  logic3_orange_green_begin_index_ = logic3_green_clusters_.size();
  // 颜色边界就在可信发车冻结这一刻锁定：READY快照保持绿色，之后由
  // match_score高分跨帧逻辑新增的永久点统一画成橙色，与二维码无关。
  logic3_orange_green_boundary_locked_ = true;
  logic3_green_last_planned_centers_.clear();
  logic3_green_last_planned_centers_.reserve(logic3_green_clusters_.size());
  for (const auto & green_cluster : logic3_green_clusters_) {
    logic3_green_last_planned_centers_.push_back(green_cluster.center);
  }
  logic3_green_route_replan_pending_ = false;

  logic3_planning_pose_anchored_ = true;
  logic3_motion_candidate_ = false;
  logic3_start_zone_motion_pending_ = false;
  logic3_runtime_green_tracks_.clear();
  logic3_red_avoidance_records_.clear();
  logic3_next_red_record_id_ = 1;
  logic3_task1_before_red_snapshot_count_ = 0;
  logic3_final_before_red_snapshot_count_ = 0;
  channel_lidar_waiting_for_post_reverse_scan_ = false;
  channel_lidar_post_reverse_scan_sequence_ = 0;
  channel_lidar_post_reverse_time_nanoseconds_ = 0;
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
  task1_lidar_avoid_count_ = 0;
  transition_lidar_avoid_count_ = 0;
  task2_lidar_avoid_count_ = 0;
  task3_lidar_avoid_count_ = 0;
  task1_lidar_last_avoid_direction_ = 0;
  transition_lidar_last_avoid_direction_ = 0;
  task2_lidar_last_avoid_direction_ = 0;
  task3_lidar_last_avoid_direction_ = 0;
  qr_reverse_rearm_after_post_stop_ = false;
  logic3_final_snapshot_published_ = false;
  logic3_final_pose_valid_ = false;
  logic3_actual_route_recording_ = false;
  logic3_actual_route_.clear();
  logic3_session_id_ = make_logic3_session_id();
  logic3_last_pose_correction_at_ = std::chrono::steady_clock::now();
  task1_departure_timer_started_ = true;
  task1_departure_started_at_ = std::chrono::steady_clock::now();
  task1_qr_timeout_logged_ = false;
  task1_endpoint_fallback_logged_ = false;
  task1_endpoint_wait_timer_started_ = false;
  task1_endpoint_wait_started_at_ = std::chrono::steady_clock::time_point{};
  task1_pending_qr_is_fallback_ = false;

  if (logic3_planning_pose_source_ != "live_lidar_location") {
    RobotPose live_map_odom;
    if (lookup_pose(
        frame_id_, logic3_odom_frame_id_,
        rclcpp::Time(0, 0, get_clock()->get_clock_type()), live_map_odom))
    {
      logic3_map_odom_plan_ = live_map_odom;
      logic3_map_odom_plan_valid_ = true;
    }
  }

  RCLCPP_WARN(
    get_logger(),
    "静态障碍记录完毕：冻结%zu个绿色障碍坐标及点云，"
    "发车后新增永久点将画成橙色；开始Task1计时，输出批次=%s。",
    logic3_green_clusters_.size(),
    logic3_session_id_.c_str());

  if (quiet_logs_after_static_freeze_) {
    const rcutils_ret_t result = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_ERROR);
    if (result != RCUTILS_RET_OK) {
      RCLCPP_ERROR(
        get_logger(), "静态冻结后切换终端静默模式失败，继续保留原日志级别。");
      rcutils_reset_error();
    }
  }

  // 可信发车优先直接冻结与本次snapshot完全一致的上一次完整预规划路线。
  // 规划中的候选已经在第一条运动里程计到来时丢弃，绝不在这里等待它。
  if (task_phase_ == TaskPhase::TASK1 && !waypoints_.empty()) {
    const bool reuse_ready_preplan = logic3_ready_preplan_enabled_ &&
      logic3_ready_preplan_valid_ && snapshot.valid &&
      logic3_ready_preplanned_snapshot_.valid &&
      std::abs(
      snapshot.stamp_seconds -
      logic3_ready_preplanned_snapshot_.stamp_seconds) <= 1.0e-9;
    if (reuse_ready_preplan) {
      logic3_green_route_clearance_failed_ =
        logic3_ready_preplanned_route_clearance_failed_;
      logic3_green_route_clearance_failed_.resize(
        logic3_green_clusters_.size(), false);
      commit_logic3_current_route_as_frozen("task1_frozen");
    } else {
      apply_logic3_frozen_clusters_to_current_route("task1_frozen");
    }
  }
}

void VisualLidarPathFollower::commit_logic3_current_route_as_frozen(
  const std::string & snapshot_event)
{
  logic3_frozen_base_offsets_ = waypoint_shift_offsets_;
  logic3_frozen_base_shift_directions_ = waypoint_shift_world_directions_;
  logic3_frozen_base_source_indices_ = waypoint_shift_source_indices_;
  logic3_frozen_base_obstacle_centers_ = waypoint_shift_obstacle_centers_;
  logic3_frozen_base_forbidden_radii_ = waypoint_shift_forbidden_radii_;
  logic3_green_last_planned_centers_.clear();
  logic3_green_last_planned_centers_.reserve(logic3_green_clusters_.size());
  for (const auto & green_cluster : logic3_green_clusters_) {
    logic3_green_last_planned_centers_.push_back(green_cluster.center);
  }
  logic3_green_route_replan_pending_ = false;
  // 静态回放只留下当前永久绿点集合的基础偏移；清空规划确认计数。
  // 运行中新增/修正绿点时会从原始路线和完整绿色集合重新计算。
  std::fill(waypoint_shift_pending_offsets_.begin(), waypoint_shift_pending_offsets_.end(), 0.0);
  std::fill(waypoint_shift_pending_counts_.begin(), waypoint_shift_pending_counts_.end(), 0);
  std::fill(waypoint_shift_missing_counts_.begin(), waypoint_shift_missing_counts_.end(), 0);
  std::fill(
    waypoint_shift_dynamic_directions_.begin(),
    waypoint_shift_dynamic_directions_.end(), 0);
  logic3_baseline_original_route_ = original_waypoints_;
  logic3_baseline_route_ = waypoints_;
  logic3_visualization_route_ = waypoints_;
  logic3_final_snapshot_published_ = false;
  logic3_final_pose_valid_ = false;
  if (snapshot_event == "task1_frozen") {
    task1_original_route_ = logic3_baseline_original_route_;
    task1_frozen_route_ = logic3_baseline_route_;
    task1_dynamic_original_route_.clear();
    task1_dynamic_route_.clear();
  } else if (snapshot_event == "final_frozen") {
    final_original_route_ = logic3_baseline_original_route_;
    final_frozen_route_ = logic3_baseline_route_;
  }
  publish_logic3_snapshot(snapshot_event);
  RCLCPP_WARN(
    get_logger(),
    "路线已规划完毕：已将%zu个冻结绿色障碍按局部路线段独立方向应用到%s路线。",
    logic3_green_clusters_.size(), selected_route_direction_.c_str());
}

void VisualLidarPathFollower::apply_logic3_frozen_clusters_to_current_route(
  const std::string & snapshot_event)
{
  if (!logic3_planning_pose_anchored_) {
    return;
  }
  // 没有可复用预规划版本时，也必须从当前阶段原始路线重新开始，避免
  // READY旧候选或上一次静态偏移残留到本次冻结结果。
  waypoints_ = original_waypoints_;
  waypoint_shift_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_missing_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_dynamic_directions_.assign(waypoints_.size(), 0);
  waypoint_shift_world_directions_.assign(waypoints_.size(), Point2D());
  waypoint_shift_source_indices_.assign(waypoints_.size(), waypoints_.size());
  waypoint_shift_obstacle_centers_.assign(waypoints_.size(), Point2D());
  waypoint_shift_forbidden_radii_.assign(waypoints_.size(), 0.0);
  logic3_green_route_clearance_failed_.assign(logic3_green_clusters_.size(), false);
  logic3_replay_frozen_clusters_ = true;
  logic3_force_full_route_ = true;
  logic3_force_confirm_ = true;
  update_waypoint_shifts(RobotPose(), 0);
  logic3_force_confirm_ = false;
  logic3_force_full_route_ = false;
  logic3_replay_frozen_clusters_ = false;
  commit_logic3_current_route_as_frozen(snapshot_event);
}

// 静态+动态方案 waypoint_shift：每帧新雷达提前检查当前目标之后的固定数量航点。
// 以障碍物最近路线点为中心，前后分别取配置数量的点并横移。
// 静态绿色障碍使路线向其相对位置的反侧绕行。
// 横移量先作为本帧候选，连续确认后才提交；障碍消失或错误观测消失后会释放回原路线。
void VisualLidarPathFollower::update_waypoint_shifts(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  std::size_t target_waypoint_index)
{
  const bool replay_frozen_clusters = logic3_replay_frozen_clusters_;
  const bool runtime_green_score_pending =
    logic3_planning_pose_anchored_ && !replay_frozen_clusters &&
    logic3_runtime_green_update_enabled_ &&
    (!latest_lidar_match_score_valid_ ||
    latest_match_score_scan_sequence_ != scan_sequence_);
  const bool runtime_green_diagnostics_pending =
    logic3_planning_pose_anchored_ && !replay_frozen_clusters &&
    logic3_runtime_green_update_enabled_ &&
    logic3_runtime_green_motion_gate_enabled_ &&
    (!latest_match_diagnostics_valid_ ||
    latest_match_diagnostics_scan_sequence_ != scan_sequence_);
  const bool static_green_score_pending =
    !logic3_planning_pose_anchored_ && !replay_frozen_clusters &&
    (!latest_lidar_match_score_valid_ ||
    latest_match_score_scan_sequence_ != scan_sequence_);
  if ((!latest_scan_ && !replay_frozen_clusters) ||
    (processed_waypoint_shift_scan_sequence_ == scan_sequence_ && !replay_frozen_clusters) ||
    runtime_green_score_pending || runtime_green_diagnostics_pending ||
    static_green_score_pending ||
    original_waypoints_.size() != waypoints_.size() ||
    waypoint_shift_offsets_.size() != waypoints_.size() ||
    waypoint_shift_pending_offsets_.size() != waypoints_.size() ||
    waypoint_shift_pending_counts_.size() != waypoints_.size() ||
    waypoint_shift_missing_counts_.size() != waypoints_.size() ||
    waypoint_shift_dynamic_directions_.size() != waypoints_.size() ||
    waypoint_shift_world_directions_.size() != waypoints_.size() ||
    waypoint_shift_source_indices_.size() != waypoints_.size() ||
    waypoint_shift_obstacle_centers_.size() != waypoints_.size() ||
    waypoint_shift_forbidden_radii_.size() != waypoints_.size() || waypoints_.empty())
  {
    return;
  }
  if (!replay_frozen_clusters) {
    processed_waypoint_shift_scan_sequence_ = scan_sequence_;
  }

  if (enable_static_map_filter_ && require_map_for_lidar_assist_ &&
    (!latest_map_ || !latest_restriction_map_))
  {
    return;
  }

  // 发车后的全图更新允许单点候选进入跨帧轨迹，再由点数对应的连续
  // 雷达帧门槛过滤；READY前静态点库仍沿用独立的静态最小点数参数。
  const int minimum_cluster_points = !logic3_planning_pose_anchored_ ?
    logic3_static_cluster_min_points_ : 1;
  const bool use_logic3_static_restriction_map =
    !logic3_planning_pose_anchored_ && !replay_frozen_clusters;
  // 运行中必须保留旧绿点圈内的点云，才能在高match_score时重新测量其
  // 位置。实时避障的绿色掩码在控制分支单独执行。
  const bool filter_logic3_green_mask =
    logic3_planning_pose_anchored_ && !replay_frozen_clusters &&
    !logic3_runtime_green_update_enabled_;
  RobotPose cluster_observation_pose = robot_pose;
  std::vector<ScanPointCluster> clusters = replay_frozen_clusters ?
    logic3_green_clusters_ : collect_non_static_scan_clusters_map(
    robot_pose, minimum_cluster_points, use_logic3_static_restriction_map,
    filter_logic3_green_mask, &cluster_observation_pose);
  if (replay_frozen_clusters && !logic3_green_dynamic_takeover_.empty()) {
    std::vector<ScanPointCluster> static_offset_clusters;
    static_offset_clusters.reserve(clusters.size());
    for (std::size_t index = 0; index < clusters.size(); ++index) {
      const bool dynamic_takeover =
        index < logic3_green_dynamic_takeover_.size() &&
        logic3_green_dynamic_takeover_[index];
      if (!dynamic_takeover) {
        static_offset_clusters.push_back(clusters[index]);
      }
    }
    clusters = std::move(static_offset_clusters);
  }

  // 远距离时同一障碍物的1~3个回波可能被无效量程或地图过滤切成多个
  // 单点段。静止阶段先按0.20m做传递连通合并：A靠近B、B靠近C时三者
  // 必须属于同一分量，不能受输入顺序或逐次更新质心影响而残留第二个簇。
  // 跨帧ID关联仍使用logic3_static_cluster_match_radius，不使用绿色冻结圆。
  if (!logic3_planning_pose_anchored_ && !replay_frozen_clusters)
  {
    std::vector<bool> consumed(clusters.size(), false);
    std::vector<ScanPointCluster> merged_clusters;
    merged_clusters.reserve(clusters.size());
    for (std::size_t seed = 0; seed < clusters.size(); ++seed) {
      if (consumed[seed]) {
        continue;
      }
      consumed[seed] = true;
      std::vector<std::size_t> component{seed};
      for (std::size_t cursor = 0; cursor < component.size(); ++cursor) {
        const std::size_t current = component[cursor];
        for (std::size_t candidate = 0; candidate < clusters.size(); ++candidate) {
          if (consumed[candidate]) {
            continue;
          }
          if (std::hypot(
              clusters[current].center.x - clusters[candidate].center.x,
              clusters[current].center.y - clusters[candidate].center.y) >
            logic3_static_same_frame_merge_radius_)
          {
            continue;
          }
          consumed[candidate] = true;
          component.push_back(candidate);
        }
      }

      ScanPointCluster merged;
      for (const std::size_t index : component) {
        merged.points.insert(
          merged.points.end(), clusters[index].points.begin(), clusters[index].points.end());
      }
      if (merged.points.empty()) {
        for (const std::size_t index : component) {
          merged.center.x += clusters[index].center.x;
          merged.center.y += clusters[index].center.y;
        }
        const double component_count = static_cast<double>(component.size());
        merged.center.x /= component_count;
        merged.center.y /= component_count;
      } else {
        for (const auto & point : merged.points) {
          merged.center.x += point.x;
          merged.center.y += point.y;
        }
        const double point_count = static_cast<double>(merged.points.size());
        merged.center.x /= point_count;
        merged.center.y /= point_count;
      }
      merged_clusters.push_back(std::move(merged));
    }
    clusters = std::move(merged_clusters);
  }

  if (use_logic3_static_restriction_map) {
    // Task1发车前只收集用户指定map矩形内的绿色候选。该坐标门限
    // 不影响运行红点记录、实时避障或后续Task2/Task3。
    clusters.erase(
      std::remove_if(
        clusters.begin(), clusters.end(), [&](const ScanPointCluster & cluster) {
          const bool in_static_green_region =
            cluster.center.x > task1_static_min_x_ &&
            cluster.center.y > task1_static_min_y_;
          return !in_static_green_region;
        }),
      clusters.end());
  }

  if (logic3_planning_pose_anchored_ && !replay_frozen_clusters)
  {
    if (logic3_runtime_green_update_enabled_) {
      std::vector<ScanPointCluster> green_observations;
      const bool should_measure_motion =
        logic3_runtime_green_motion_gate_enabled_ ||
        logic3_runtime_green_gate_log_rate_hz_ > 0.0;
      double steering_span = std::numeric_limits<double>::infinity();
      bool measured_steering_within_limit = true;
      bool steering_measurement_valid = false;
      double minimum_abs_speed = std::numeric_limits<double>::infinity();
      double maximum_speed_step = std::numeric_limits<double>::infinity();
      bool measured_speed_within_limits = true;
      bool speed_measurement_valid = false;
      if (should_measure_motion) {
        measured_steering_within_limit =
          logic3_runtime_green_steering_is_stable(*latest_scan_, &steering_span);
        steering_measurement_valid = std::isfinite(steering_span);
        measured_speed_within_limits = logic3_runtime_green_speed_is_stable(
          *latest_scan_, &minimum_abs_speed, &maximum_speed_step);
        speed_measurement_valid = std::isfinite(minimum_abs_speed) &&
          std::isfinite(maximum_speed_step);
      }
      const double map_odom_translation_correction_abs =
        std::abs(latest_map_odom_translation_correction_);
      const bool measured_translation_within_limit =
        map_odom_translation_correction_abs <=
        logic3_runtime_green_max_map_odom_translation_correction_;
      const double map_odom_yaw_correction_abs_deg =
        std::abs(latest_map_odom_yaw_correction_) * 180.0 / M_PI;
      const bool measured_yaw_within_limit =
        map_odom_yaw_correction_abs_deg <=
        logic3_runtime_green_max_map_odom_yaw_correction_deg_;
      record_logic3_runtime_green_gate_log_sample(
        steering_span, steering_measurement_valid,
        minimum_abs_speed, maximum_speed_step, speed_measurement_valid,
        map_odom_translation_correction_abs,
        map_odom_yaw_correction_abs_deg,
        measured_steering_within_limit, measured_speed_within_limits,
        measured_translation_within_limit, measured_yaw_within_limit);
      const bool steering_is_stable =
        !logic3_runtime_green_motion_gate_enabled_ ||
        (steering_measurement_valid && measured_steering_within_limit);
      const bool speed_is_stable =
        !logic3_runtime_green_motion_gate_enabled_ ||
        (speed_measurement_valid && measured_speed_within_limits);
      const bool map_odom_translation_is_stable =
        !logic3_runtime_green_motion_gate_enabled_ || measured_translation_within_limit;
      const bool map_odom_yaw_is_stable =
        !logic3_runtime_green_motion_gate_enabled_ || measured_yaw_within_limit;
      const bool motion_is_stable =
        steering_is_stable && speed_is_stable &&
        map_odom_translation_is_stable && map_odom_yaw_is_stable;
      if (latest_lidar_match_score_ >= logic3_runtime_green_min_match_score_ &&
        motion_is_stable)
      {
        // 全局绿点库由同圈匹配分数、发布速度/转向稳定度、map->odom
        // 平移/航向修正和跨帧确认共同决定，不受当前任务阶段、Task1
        // 静态矩形、Task3高度分界或运行红点ROI限制。
        green_observations.reserve(clusters.size());
        for (const auto & cluster : clusters) {
          const double distance_from_robot = std::hypot(
            cluster.center.x - cluster_observation_pose.x,
            cluster.center.y - cluster_observation_pose.y);
          // 近车点云最容易混入车体、基地边缘或运动畸变。可信发车后，
          // 以本雷达帧采样时刻的base_link为圆心统一拦截绿/橙候选：
          // 边界也属于排除区。过滤后短期轨迹会按“观测中断”清零，
          // 因此车辆离开后必须重新完整累计连续可信帧。
          if (logic3_runtime_green_robot_exclusion_radius_ > 0.0 &&
            distance_from_robot <=
            logic3_runtime_green_robot_exclusion_radius_ + 1.0e-9)
          {
            continue;
          }
          green_observations.push_back(cluster);
        }
      } else if (latest_lidar_match_score_ >= logic3_runtime_green_min_match_score_ &&
        logic3_runtime_green_motion_gate_enabled_)
      {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "高分雷达帧不更新绿/橙点：转向跨度=%.3frad(上限%.3f)，"
          "最低速度幅值=%.3fm/s(下限%.3f)，"
          "相邻发布速度跳变=%.3fm/s(上限%.3f)，"
          "map->odom平移修正=%.3fm(上限%.3fm)，"
          "map->odom航向修正=%.2fdeg(上限%.2fdeg)。",
          steering_span, logic3_runtime_green_max_steering_span_,
          minimum_abs_speed, logic3_runtime_green_min_speed_,
          maximum_speed_step, logic3_runtime_green_max_speed_step_,
          map_odom_translation_correction_abs,
          logic3_runtime_green_max_map_odom_translation_correction_,
          map_odom_yaw_correction_abs_deg,
          logic3_runtime_green_max_map_odom_yaw_correction_deg_);
      }
      if (update_logic3_runtime_green_tracks(
          green_observations,
          minimum_abs_speed, maximum_speed_step,
          map_odom_translation_correction_abs,
          map_odom_yaw_correction_abs_deg))
      {
        logic3_green_route_replan_pending_ = true;
      }
      const auto runtime_replan_now = std::chrono::steady_clock::now();
      const bool runtime_replan_interval_elapsed =
        !logic3_last_runtime_route_replan_at_valid_ ||
        std::chrono::duration<double>(
        runtime_replan_now - logic3_last_runtime_route_replan_at_).count() >=
        logic3_runtime_green_replan_min_interval_;
      if (logic3_green_route_replan_pending_ && runtime_replan_interval_elapsed &&
        (task_phase_ != TaskPhase::TRANSITION ||
        tracking_controller_mode_ == "smac_pid"))
      {
        // smac_pid障碍策略在所有正向阶段都及时把确认点写入路线；
        // legacy_lookahead仍保留过渡段只积累、进入Task2后统一应用的旧行为。
        // 冷却期间点库继续更新，pending保持为true；到期后直接使用最新点库，
        // 不会丢失第一次不准的橙点后续修正。
        rebuild_logic3_route_from_green_clusters("高置信度全局绿点更新");
      }
    }

    // 红点不再由这里的全部运行簇生成；只有实时雷达真正覆盖控制命令时，
    // 才由apply_channel_lidar_obstacle_avoidance记录对应命中点。
    return;
  }

  // Task1只让静态矩形内的冻结绿色簇参与静态路径偏移；矩形外的冻结
  // 绿色簇不偏移，并由上面的绿圈放行逻辑交给实时动态雷达。
  if (replay_frozen_clusters &&
    (following_qr_path_ || task_phase_ == TaskPhase::TASK1) &&
    tracking_controller_mode_ != "smac_pid")
  {
    clusters.erase(
      std::remove_if(
        clusters.begin(), clusters.end(),
        [&](const ScanPointCluster & cluster) {
          return !task1_static_avoid_region_contains(cluster.center);
        }),
      clusters.end());
  }

  const bool full_route = logic3_force_full_route_ || replay_frozen_clusters;
  const std::size_t first_index = full_route ? 0 :
    std::min(target_waypoint_index, waypoints_.size() - 1);
  const std::size_t last_index = full_route ? waypoints_.size() : std::min(
    waypoints_.size(), first_index + static_cast<std::size_t>(waypoint_shift_check_count_));
  const std::size_t managed_last_index = full_route ? waypoints_.size() - 1 : std::min(
    waypoints_.size() - 1,
    first_index + static_cast<std::size_t>(waypoint_shift_check_count_) +
    static_cast<std::size_t>(std::max({
      task1_waypoint_shift_points_after_,
      transition_waypoint_shift_points_after_,
      task23_waypoint_shift_points_after_})));
  const double check_radius_squared =
    waypoint_cloud_check_radius_ * waypoint_cloud_check_radius_;

  struct ShiftCandidateSegment
  {
    std::size_t obstacle_waypoint{0};
    std::size_t first{0};
    std::size_t last{0};
    double left_required_offset{0.0};
    double right_required_offset{0.0};
    int preferred_direction{0};
    Point2D left_normal;
    Point2D obstacle_center;
    double forbidden_radius{0.0};
  };
  struct RouteSegmentMatch
  {
    bool valid{false};
    std::size_t segment{0};
    double t{0.0};
    double distance_squared{std::numeric_limits<double>::infinity()};
    Point2D tangent;
    Point2D projection;
  };
  std::vector<ShiftCandidateSegment> candidate_segments;

  auto match_logic3_route_segment = [&](
    const ScanPointCluster & cluster, std::size_t segment_index,
    RouteSegmentMatch & match)
    {
      match = RouteSegmentMatch();
      if (segment_index + 1 >= original_waypoints_.size()) {
        return false;
      }
      const auto & start = original_waypoints_[segment_index];
      const auto & finish = original_waypoints_[segment_index + 1];
      const double segment_x = finish.x - start.x;
      const double segment_y = finish.y - start.y;
      const double length_squared = segment_x * segment_x + segment_y * segment_y;
      if (length_squared < 1e-8) {
        return false;
      }

      auto projection = [&](const Point2D & point) {
          const double raw_t =
            ((point.x - start.x) * segment_x +
            (point.y - start.y) * segment_y) / length_squared;
          return std::max(0.0, std::min(1.0, raw_t));
        };
      const double center_t = projection(cluster.center);
      const double projection_x = start.x + center_t * segment_x;
      const double projection_y = start.y + center_t * segment_y;
      double cloud_distance_squared = std::numeric_limits<double>::infinity();
      auto accumulate_distance = [&](const Point2D & point) {
          const double point_t = projection(point);
          const double point_projection_x = start.x + point_t * segment_x;
          const double point_projection_y = start.y + point_t * segment_y;
          const double dx = point.x - point_projection_x;
          const double dy = point.y - point_projection_y;
          cloud_distance_squared = std::min(cloud_distance_squared, dx * dx + dy * dy);
        };
      if (cluster.points.empty()) {
        accumulate_distance(cluster.center);
      } else {
        for (const auto & point : cluster.points) {
          accumulate_distance(point);
        }
      }

      const double length = std::sqrt(length_squared);
      match.valid = std::isfinite(cloud_distance_squared);
      match.segment = segment_index;
      match.t = center_t;
      match.distance_squared = cloud_distance_squared;
      match.tangent = Point2D{segment_x / length, segment_y / length};
      match.projection = Point2D{projection_x, projection_y};
      return match.valid;
    };

  auto append_shift_candidate = [&](
    const ScanPointCluster & cluster, const RouteSegmentMatch & route_match,
    std::size_t cluster_route_first, std::size_t cluster_route_last,
    int preferred_direction)
    {
      if (!route_match.valid || cluster_route_first >= cluster_route_last) {
        return;
      }
      const Point2D left_normal{-route_match.tangent.y, route_match.tangent.x};
      const Point2D right_normal{-left_normal.x, -left_normal.y};
      double target_clearance = task1_waypoint_shift_target_clearance_;
      if (route_deformation_mode_ == "circle_envelope") {
        target_clearance = circle_envelope_target_clearance_;
      } else {
        switch (task_phase_) {
          case TaskPhase::TRANSITION:
            target_clearance = transition_waypoint_shift_target_clearance_;
            break;
          case TaskPhase::TASK2:
            target_clearance = task2_waypoint_shift_target_clearance_;
            break;
          case TaskPhase::TASK3:
            target_clearance = task3_waypoint_shift_target_clearance_;
            break;
          case TaskPhase::TASK1:
          case TaskPhase::UNKNOWN:
          case TaskPhase::FINISHED:
            break;
        }
      }

      auto required_offset_along = [&](const Point2D & shift_normal) {
          if (route_deformation_mode_ == "circle_envelope") {
            // circle_envelope按点云中心的固定禁入圆直接构造几何包络。
            // target_clearance就是圆半径，不再叠加点云自身拟合半径。
            const double center_along_shift =
              (cluster.center.x - route_match.projection.x) * shift_normal.x +
              (cluster.center.y - route_match.projection.y) * shift_normal.y;
            return std::max(
              0.0, center_along_shift + circle_envelope_target_clearance_);
          }
          double cloud_edge_along_shift = -std::numeric_limits<double>::infinity();
          if (cluster.points.empty()) {
            cloud_edge_along_shift =
              (cluster.center.x - route_match.projection.x) * shift_normal.x +
              (cluster.center.y - route_match.projection.y) * shift_normal.y;
          } else {
            for (const auto & point : cluster.points) {
              const double lateral_distance =
                (point.x - route_match.projection.x) * shift_normal.x +
                (point.y - route_match.projection.y) * shift_normal.y;
              cloud_edge_along_shift = std::max(cloud_edge_along_shift, lateral_distance);
            }
          }
          return std::max(
            0.0, cloud_edge_along_shift + target_clearance);
        };
      const double left_required_offset = required_offset_along(left_normal);
      const double right_required_offset = required_offset_along(right_normal);
      if (left_required_offset <= 1e-9 && right_required_offset <= 1e-9) {
        return;
      }
      double cloud_radius = 0.0;
      for (const auto & point : cluster.points) {
        cloud_radius = std::max(
          cloud_radius,
          std::hypot(point.x - cluster.center.x, point.y - cluster.center.y));
      }
      const double forbidden_radius = route_deformation_mode_ == "circle_envelope" ?
        circle_envelope_target_clearance_ + circle_envelope_numeric_margin_ :
        cloud_radius + target_clearance + circle_envelope_numeric_margin_;

      std::size_t obstacle_waypoint = route_match.t < 0.5 ?
        route_match.segment : route_match.segment + 1;
      obstacle_waypoint = std::max(
        cluster_route_first,
        std::min(obstacle_waypoint, original_waypoints_.size() - 1));
      int configured_points_before = task23_waypoint_shift_points_before_;
      int configured_points_after = task23_waypoint_shift_points_after_;
      if (following_qr_path_ || task_phase_ == TaskPhase::TASK1) {
        configured_points_before = task1_waypoint_shift_points_before_;
        configured_points_after = task1_waypoint_shift_points_after_;
      } else if (channel_entry_waypoint_index_ > 0 &&
        obstacle_waypoint < channel_entry_waypoint_index_)
      {
        configured_points_before = transition_waypoint_shift_points_before_;
        configured_points_after = transition_waypoint_shift_points_after_;
      }
      const std::size_t points_before =
        static_cast<std::size_t>(configured_points_before);
      const std::size_t points_after =
        static_cast<std::size_t>(configured_points_after);
      const std::size_t influence_first = std::max(
        cluster_route_first,
        obstacle_waypoint > points_before ? obstacle_waypoint - points_before : 0);
      const std::size_t influence_last = std::min(
        std::min(original_waypoints_.size() - 1, cluster_route_last - 1),
        obstacle_waypoint + points_after);
      candidate_segments.push_back(
        ShiftCandidateSegment{
          obstacle_waypoint, influence_first, influence_last, left_required_offset,
          right_required_offset, preferred_direction, left_normal,
          cluster.center, forbidden_radius});
    };

  if (!logic3_planning_pose_anchored_ && !replay_frozen_clusters)
  {
    // 第一条发车候选已经保存READY平均快照后，停止接收后续可能包含底盘
    // 震动的静态帧；候选若取消，下一帧会恢复观察并按连续性规则重建。
    if (logic3_motion_candidate_ || logic3_start_zone_motion_pending_) {
      return;
    }
    // 新channel路线会从扫码/倒车后的base_link向正式首点延长，因此此时
    // 不能再拿旧channel路线预筛大厅静态簇。保留原ID、稳定帧和最高分采样，
    // 规划阶段再由新路线的检查半径决定哪些绿色簇真正产生偏移。
    update_logic3_static_cluster_tracks(
      clusters, robot_pose, latest_lidar_match_score_,
      latest_match_score_scan_sequence_);
    const auto ready_obstacles = logic3_ready_static_obstacles();
    if (logic3_static_ready_ && !ready_obstacles.empty()) {
      // 只在完整READY状态下刷新。可信发车一旦出现运动预兆，静态更新会
      // 先暂停，因此这里保存的始终是运动前最后一次完整READY快照。
      logic3_last_complete_ready_snapshot_.valid = true;
      logic3_last_complete_ready_snapshot_.stamp_seconds = now().seconds();
      logic3_last_complete_ready_snapshot_.obstacles = ready_obstacles;
      preplan_logic3_ready_route(logic3_last_complete_ready_snapshot_);
    }
    maybe_log_logic3_static_obstacles(clusters.size(), ready_obstacles);
    // READY期间只提交已经完整算完、并由后一帧静止里程计确认的路线；
    // 尚在计算或尚未确认的候选绝不覆盖当前活动路线。
    return;
  }

  {
    struct Logic3SegmentAssignment
    {
      bool valid{false};
      std::size_t cluster_index{0};
      std::size_t route_first{0};
      std::size_t route_last{0};
      RouteSegmentMatch match;
    };
    std::vector<Logic3SegmentAssignment> assignments(original_waypoints_.size() - 1);

    // 每条路线段只归属检查半径内最近的障碍点云。相同障碍在路线索引上
    // 不连续时会自然形成多个局部命中段，不需要入口/出口高度特判。
    for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
      // 只在二维码倒车完成后的进场路线重生成时，用当时A沿yaw前移配置
      // 距离后的虚拟参考点和原yaw对永久点做一次车前车后分类。
      // 车后点仍可匹配A后保护长度之外及
      // 后续任务路线，但其直接命中及before窗口不得倒灌到A点附近。
      const double relative_x = clusters[cluster_index].center.x -
        channel_entry_rear_static_filter_start_x_;
      const double relative_y = clusters[cluster_index].center.y -
        channel_entry_rear_static_filter_start_y_;
      const bool behind_channel_entry_start =
        channel_entry_rear_static_filter_active_ &&
        relative_x * std::cos(channel_entry_rear_static_filter_start_yaw_) +
        relative_y * std::sin(channel_entry_rear_static_filter_start_yaw_) < 0.0;
      // 分界高度只给“整个障碍簇”选择处理器，不能拿它裁剪路线点。
      // 因此这里只阻止下方障碍匹配返程段；上方障碍一旦匹配成功，后续
      // influence_first..influence_last仍可完整跨过return_obstacle_split_y。
      const bool return_lidar_only = tracking_controller_mode_ != "smac_pid" &&
        clusters[cluster_index].center.y <= return_obstacle_split_y_;
      const std::size_t cluster_route_first = behind_channel_entry_start ?
        std::max(
        first_index,
        std::min(
          channel_entry_rear_static_filter_first_index_,
          original_waypoints_.size() - 1)) : first_index;
      const std::size_t cluster_route_last = last_index;
      const std::size_t segment_first = cluster_route_first > 0 ?
        cluster_route_first - 1 : cluster_route_first;
      const std::size_t segment_last = std::min(
        cluster_route_last, original_waypoints_.size() - 1);
      for (std::size_t segment_index = segment_first;
        segment_index < segment_last; ++segment_index)
      {
        if (return_lidar_only && task3_route_available_ &&
          segment_index >= task3_start_index_)
        {
          continue;
        }
        RouteSegmentMatch match;
        if (!match_logic3_route_segment(clusters[cluster_index], segment_index, match) ||
          match.distance_squared > check_radius_squared)
        {
          continue;
        }
        auto & assignment = assignments[segment_index];
        if (!assignment.valid ||
          match.distance_squared + 1e-12 < assignment.match.distance_squared)
        {
          assignment.valid = true;
          assignment.cluster_index = cluster_index;
          assignment.route_first = cluster_route_first;
          assignment.route_last = cluster_route_last;
          assignment.match = match;
        }
      }
    }

    std::size_t segment_index = 0;
    while (segment_index < assignments.size()) {
      if (!assignments[segment_index].valid) {
        ++segment_index;
        continue;
      }
      const std::size_t run_first = segment_index;
      const std::size_t cluster_index = assignments[segment_index].cluster_index;
      std::size_t run_last = segment_index;
      while (run_last + 1 < assignments.size() &&
        assignments[run_last + 1].valid &&
        assignments[run_last + 1].cluster_index == cluster_index)
      {
        ++run_last;
      }

      const Logic3SegmentAssignment * anchor = &assignments[run_first];
      for (std::size_t index = run_first + 1; index <= run_last; ++index) {
        if (assignments[index].match.distance_squared < anchor->match.distance_squared) {
          anchor = &assignments[index];
        }
      }
      const auto & cluster = clusters[cluster_index];
      const Point2D left_normal{-anchor->match.tangent.y, anchor->match.tangent.x};
      const double obstacle_lateral =
        (cluster.center.x - anchor->match.projection.x) * left_normal.x +
        (cluster.center.y - anchor->match.projection.y) * left_normal.y;
      // +1表示路线左移，-1表示路线右移。每个连续局部命中段只由自己的
      // 最近锚点决定一次方向，随后扩展before/after，避免逐点方向抖动。
      const int preferred_direction = obstacle_lateral >= 0.0 ? -1 : 1;
      append_shift_candidate(
        cluster, anchor->match, anchor->route_first, anchor->route_last,
        preferred_direction);
      if (debug_control_logs_ && !logic3_ready_preplanning_active_) {
        RCLCPP_INFO(
          get_logger(),
          "debug: 静态+动态方案局部路线命中：绿色簇(%.2f, %.2f)，路线段=%zu-%zu，"
          "锚点=%zu，向%s。",
          cluster.center.x, cluster.center.y, run_first + 1, run_last + 1,
          anchor->match.segment + 1, preferred_direction > 0 ? "左" : "右");
      }
      segment_index = run_last + 1;
    }
  }

  // 所有候选都先基于未偏移的original_waypoints_独立确定锚点、方向和距离。
  // circle_envelope只确认这些独立锚点，绝不让前一个障碍物的影响窗口覆盖
  // 后一个障碍物；legacy_offset仍保留原来的分段窗口写入行为。
  std::sort(
    candidate_segments.begin(), candidate_segments.end(),
    [&](const ShiftCandidateSegment & lhs, const ShiftCandidateSegment & rhs) {
      if (lhs.obstacle_waypoint != rhs.obstacle_waypoint)
      {
        return lhs.obstacle_waypoint < rhs.obstacle_waypoint;
      }
      if (lhs.first != rhs.first) {
        return lhs.first < rhs.first;
      }
      return lhs.last < rhs.last;
    });

  std::vector<double> observed_offsets(waypoints_.size(), 0.0);
  std::vector<bool> observed_offsets_valid(waypoints_.size(), false);
  std::vector<Point2D> observed_world_directions(waypoints_.size(), Point2D());
  std::vector<std::size_t> observed_source_indices(
    waypoints_.size(), waypoints_.size());
  std::vector<Point2D> observed_obstacle_centers(waypoints_.size(), Point2D());
  std::vector<double> observed_forbidden_radii(waypoints_.size(), 0.0);
  std::vector<bool> observed_geometry_metadata_valid(waypoints_.size(), false);
  if (route_deformation_mode_ == "circle_envelope") {
    for (const auto & segment : candidate_segments) {
      if (segment.preferred_direction == 0) {
        continue;
      }
      const double required_offset = segment.preferred_direction > 0 ?
        segment.left_required_offset : segment.right_required_offset;
      if (required_offset <= 1.0e-9) {
        continue;
      }

      // 通常每个障碍物都有不同锚点。极少数锚点重合时，只借用其影响窗内
      // 最近的空元数据槽；source仍保存真实投影锚点，不合并障碍物身份。
      std::size_t metadata_index = segment.obstacle_waypoint;
      if (observed_geometry_metadata_valid[metadata_index]) {
        bool found_slot = false;
        const std::size_t maximum_distance = std::max(
          metadata_index - segment.first, segment.last - metadata_index);
        for (std::size_t distance = 1; distance <= maximum_distance; ++distance) {
          if (metadata_index >= segment.first + distance &&
            !observed_geometry_metadata_valid[metadata_index - distance])
          {
            metadata_index -= distance;
            found_slot = true;
            break;
          }
          if (metadata_index + distance <= segment.last &&
            !observed_geometry_metadata_valid[metadata_index + distance])
          {
            metadata_index += distance;
            found_slot = true;
            break;
          }
        }
        if (!found_slot) {
          if (!logic3_ready_preplanning_active_) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "circle_envelope独立锚点槽不足：障碍(%.3f, %.3f)锚点=%zu；"
              "本轮保留已有锚点，不把两个障碍物合并。",
              segment.obstacle_center.x, segment.obstacle_center.y,
              segment.obstacle_waypoint + 1);
          }
          continue;
        }
      }

      observed_offsets[metadata_index] =
        static_cast<double>(segment.preferred_direction) * required_offset;
      observed_offsets_valid[metadata_index] = true;
      observed_world_directions[metadata_index] =
        segment.preferred_direction > 0 ?
        segment.left_normal : Point2D{-segment.left_normal.x, -segment.left_normal.y};
      observed_source_indices[metadata_index] = segment.obstacle_waypoint;
      observed_obstacle_centers[metadata_index] = segment.obstacle_center;
      observed_forbidden_radii[metadata_index] = segment.forbidden_radius;
      observed_geometry_metadata_valid[metadata_index] = true;
      if (debug_control_logs_ && !logic3_ready_preplanning_active_) {
        RCLCPP_INFO(
          get_logger(),
          "debug: circle_envelope独立锚点：障碍(%.2f, %.2f)，锚点=%zu，"
          "元数据槽=%zu，向%s偏移%.3fm。",
          segment.obstacle_center.x, segment.obstacle_center.y,
          segment.obstacle_waypoint + 1, metadata_index + 1,
          segment.preferred_direction > 0 ? "左" : "右", required_offset);
      }
    }
  } else {
    const ShiftCandidateSegment * previous_active_segment = nullptr;
    for (const auto & segment : candidate_segments) {
      if (segment.preferred_direction == 0) {
        continue;
      }
      const double required_offset = segment.preferred_direction > 0 ?
        segment.left_required_offset : segment.right_required_offset;
      if (required_offset <= 1e-9) {
        continue;
      }
      const double candidate =
        static_cast<double>(segment.preferred_direction) * required_offset;
      std::size_t applied_first = segment.first;
      std::size_t overlap_count = 0;
      if (previous_active_segment) {
        const std::size_t overlap_first = std::max(
          previous_active_segment->first, segment.first);
        const std::size_t overlap_last = std::min(
          previous_active_segment->last, segment.last);
        if (overlap_first <= overlap_last) {
          overlap_count = overlap_last - overlap_first + 1;
          // 整数除法将不能二等分的余数交给后一个障碍物。
          applied_first = overlap_first + overlap_count / 2;
        }
      }
      std::size_t overwritten_count = 0;
      for (std::size_t index = applied_first; index <= segment.last; ++index) {
        if (observed_offsets_valid[index]) {
          ++overwritten_count;
        }
        // 候选已按路线进度排序；后一个障碍物从重叠区1/2位置开始取得控制权。
        observed_offsets[index] = candidate;
        observed_offsets_valid[index] = true;
        observed_world_directions[index] = segment.preferred_direction > 0 ?
          segment.left_normal : Point2D{-segment.left_normal.x, -segment.left_normal.y};
        observed_source_indices[index] = segment.obstacle_waypoint;
        observed_obstacle_centers[index] = segment.obstacle_center;
        observed_forbidden_radii[index] = segment.forbidden_radius;
        observed_geometry_metadata_valid[index] = true;
      }
      previous_active_segment = &segment;
      if (debug_control_logs_ && !logic3_ready_preplanning_active_) {
        RCLCPP_INFO(
          get_logger(),
          "debug: 静态+动态方案二分之一覆盖：障碍点=%zu，原窗口=%zu-%zu，实际写入=%zu-%zu，"
          "向%s偏移%.3fm，重叠=%zu个，覆盖前障碍=%zu个。",
          segment.obstacle_waypoint + 1, segment.first + 1, segment.last + 1,
          applied_first + 1, segment.last + 1,
          segment.preferred_direction > 0 ? "左" : "右", required_offset,
          overlap_count, overwritten_count);
      }
    }
  }

  // 延长路线的第0点就是本次规划瞬间的base_link位置A。静态障碍可以
  // 改造其后的连接段，但A必须固定，不能被首个绿色簇横移。
  if (channel_entry_waypoint_index_ > 0 && !observed_offsets_valid.empty()) {
    observed_offsets_valid.front() = false;
    observed_geometry_metadata_valid.front() = false;
    observed_offsets.front() = 0.0;
    observed_obstacle_centers.front() = Point2D();
    observed_forbidden_radii.front() = 0.0;
  }
  bool route_changed = false;
  auto observed_geometry_changed = [&](std::size_t index) {
      if (!observed_geometry_metadata_valid[index]) {
        return false;
      }
      if (waypoint_shift_source_indices_[index] >= waypoints_.size()) {
        return true;
      }
      const std::size_t source_delta = waypoint_shift_source_indices_[index] >
        observed_source_indices[index] ?
        waypoint_shift_source_indices_[index] - observed_source_indices[index] :
        observed_source_indices[index] - waypoint_shift_source_indices_[index];
      const auto & current_direction = waypoint_shift_world_directions_[index];
      const auto & observed_direction = observed_world_directions[index];
      const double direction_alignment =
        current_direction.x * observed_direction.x +
        current_direction.y * observed_direction.y;
      const double center_change = std::hypot(
        waypoint_shift_obstacle_centers_[index].x -
        observed_obstacle_centers[index].x,
        waypoint_shift_obstacle_centers_[index].y -
        observed_obstacle_centers[index].y);
      const double radius_change = std::abs(
        waypoint_shift_forbidden_radii_[index] -
        observed_forbidden_radii[index]);
      // 0.02m航点下允许来源点在2点内轻微抖动；方向变化约2.6度以上才重建。
      const double geometry_change_threshold = std::max(
        0.005, 0.5 * waypoint_shift_min_change_);
      return source_delta > 2 || direction_alignment < 0.999 ||
             center_change >= geometry_change_threshold ||
             radius_change >= geometry_change_threshold;
    };

  std::size_t confirmed_count = 0;
  std::size_t released_count = 0;
  for (std::size_t index = first_index; index <= managed_last_index; ++index) {
    if (observed_offsets_valid[index]) {
      const double candidate = observed_offsets[index];
      waypoint_shift_missing_counts_[index] = 0;

      const bool same_direction =
        waypoint_shift_pending_counts_[index] > 0 &&
        waypoint_shift_pending_offsets_[index] * candidate > 0.0;

      if (!same_direction) {
        waypoint_shift_pending_offsets_[index] = candidate;
        waypoint_shift_pending_counts_[index] = 1;
      } else {
        waypoint_shift_pending_offsets_[index] = candidate;
        ++waypoint_shift_pending_counts_[index];
      }

      const int required_confirm_scans = logic3_force_confirm_ ?
        1 : waypoint_shift_confirm_scans_;
      if (waypoint_shift_pending_counts_[index] >= required_confirm_scans) {
        const double confirmed_offset = waypoint_shift_pending_offsets_[index];
        if (std::abs(confirmed_offset - waypoint_shift_offsets_[index]) >=
          waypoint_shift_min_change_)
        {
          waypoint_shift_offsets_[index] = confirmed_offset;
          if (observed_geometry_metadata_valid[index]) {
            waypoint_shift_world_directions_[index] = observed_world_directions[index];
            waypoint_shift_source_indices_[index] = observed_source_indices[index];
            waypoint_shift_obstacle_centers_[index] = observed_obstacle_centers[index];
            waypoint_shift_forbidden_radii_[index] = observed_forbidden_radii[index];
          }
          route_changed = true;
          ++confirmed_count;
        } else if (std::abs(waypoint_shift_offsets_[index]) > 1e-9 &&
          observed_geometry_changed(index))
        {
          waypoint_shift_world_directions_[index] = observed_world_directions[index];
          waypoint_shift_source_indices_[index] = observed_source_indices[index];
          waypoint_shift_obstacle_centers_[index] = observed_obstacle_centers[index];
          waypoint_shift_forbidden_radii_[index] = observed_forbidden_radii[index];
          route_changed = true;
        }
      }
      continue;
    }

    waypoint_shift_pending_offsets_[index] = 0.0;
    waypoint_shift_pending_counts_[index] = 0;
    ++waypoint_shift_missing_counts_[index];
    const double base_offset = logic3_planning_pose_anchored_ &&
      logic3_frozen_base_offsets_.size() == waypoint_shift_offsets_.size() ?
      logic3_frozen_base_offsets_[index] : 0.0;
    if (waypoint_shift_missing_counts_[index] >= waypoint_shift_release_scans_) {
      waypoint_shift_dynamic_directions_[index] = 0;
      Point2D base_direction;
      std::size_t base_source = waypoints_.size();
      Point2D base_center;
      double base_radius = 0.0;
      if (std::abs(base_offset) > 1e-9 &&
        logic3_frozen_base_shift_directions_.size() == waypoints_.size() &&
        logic3_frozen_base_source_indices_.size() == waypoints_.size() &&
        logic3_frozen_base_obstacle_centers_.size() == waypoints_.size() &&
        logic3_frozen_base_forbidden_radii_.size() == waypoints_.size())
      {
        base_direction = logic3_frozen_base_shift_directions_[index];
        base_source = logic3_frozen_base_source_indices_[index];
        base_center = logic3_frozen_base_obstacle_centers_[index];
        base_radius = logic3_frozen_base_forbidden_radii_[index];
      }
      const bool release_changed =
        std::abs(waypoint_shift_offsets_[index] - base_offset) > 1e-9 ||
        waypoint_shift_source_indices_[index] != base_source ||
        std::hypot(
          waypoint_shift_obstacle_centers_[index].x - base_center.x,
          waypoint_shift_obstacle_centers_[index].y - base_center.y) > 1e-9 ||
        std::abs(waypoint_shift_forbidden_radii_[index] - base_radius) > 1e-9;
      if (release_changed) {
        waypoint_shift_offsets_[index] = base_offset;
        waypoint_shift_world_directions_[index] = base_direction;
        waypoint_shift_source_indices_[index] = base_source;
        waypoint_shift_obstacle_centers_[index] = base_center;
        waypoint_shift_forbidden_radii_[index] = base_radius;
        route_changed = true;
        ++released_count;
      }
    }
  }

  if (!logic3_ready_preplanning_active_ &&
    (confirmed_count > 0 || released_count > 0))
  {
    RCLCPP_WARN(
      get_logger(),
      "waypoint_shift：确认刷新%zu个点，释放%zu个点（确认帧=%d，释放帧=%d）。",
      confirmed_count, released_count, waypoint_shift_confirm_scans_,
      waypoint_shift_release_scans_);
  }

  if (route_changed) {
    rebuild_route_from_confirmed_shifts();
    if (!logic3_ready_preplanning_active_) {
      publish_debug_route();
    }
  }

}
