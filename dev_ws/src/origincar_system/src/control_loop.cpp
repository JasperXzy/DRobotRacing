#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

// ********************************************************************************
// ******************************** 主控制流程 ********************************
// ********************************************************************************
// 主控制循环：拿位姿 -> 更新路线线段进度 -> 算离散前瞻 waypoint 速度 -> 发布。
// 这里保留完整顺序，避免为了几行赋值再跳到小包装函数里。
void VisualLidarPathFollower::control_once()
{
  if (waiting_for_qr_start_) {
    if (!logic3_planning_pose_anchored_ && !waypoints_.empty())
    {
      RobotPose planning_pose;
      if (lookup_robot_pose(planning_pose.x, planning_pose.y, planning_pose.yaw)) {
        if (!logic3_motion_candidate_ && !logic3_start_zone_motion_pending_) {
          logic3_force_full_route_ = true;
          update_waypoint_shifts(planning_pose, 0);
          logic3_force_full_route_ = false;
        }
        publish_debug_visualization(planning_pose);
      }
    }
    set_control_state(ControlState::WAITING_FOR_QR);
    return;
  }
  if (waypoints_.empty()) {
    publish_stop();
    return;
  }
  RobotPose pose;
  if (!lookup_robot_pose(pose.x, pose.y, pose.yaw)) {
    set_control_state(ControlState::WAITING_FOR_POSE);
    if (qr_escape_state_ == QrEscapeState::WAITING_FOR_PRE_REVERSE_SCAN) {
      // 真实二维码交接可能明确跳过反相制动；等待新雷达帧期间必须继续
      // 走同一条直接零速通道，不能被残余实测轮速重新武装反相脉冲。
      publish_translation_stop_without_reverse_brake(qr_reverse_hold_angular_command_);
      return;
    }
    if (qr_escape_state_ == QrEscapeState::PRE_REVERSE_BRAKE ||
      qr_escape_state_ == QrEscapeState::BACKING_ARMED ||
      qr_escape_state_ == QrEscapeState::BACKING_RIGHT ||
      qr_escape_state_ == QrEscapeState::POST_STOP ||
      qr_escape_state_ == QrEscapeState::WAITING_FOR_POST_STOP_SCAN ||
      channel_entry_route_rebuild_pending_)
    {
      // 正在倒车、倒车后静止或等待按新A点重建路线时不能盲目沿用旧指令。
      publish_translation_stop(qr_reverse_hold_angular_command_);
      return;
    }
    // 正向行驶时单次定位查询失败不能插入零速脉冲；本周期不刷新控制指令，
    // 由底盘继续执行上一条有效指令，避免50Hz控制中出现走走停停。
    return;
  }
  record_logic3_actual_route(pose);

  if (task_phase_ == TaskPhase::TASK1 && logic3_planning_pose_anchored_) {
    const auto steady_now = std::chrono::steady_clock::now();
    if (!task1_pending_direction_.empty()) {
      if (pose.x >= task1_transition_min_x_) {
        const std::string pending_direction = task1_pending_direction_;
        const std::string pending_reason = task1_pending_reason_;
        const bool pending_from_qr = task1_pending_from_qr_;
        if (complete_task1_and_select_route(
            pending_direction, pending_reason.c_str(), pending_from_qr))
        {
          // complete_task1_and_select_route已在路线切换前下发对应停车命令；
          // 本控制周期必须立刻结束，不能再计算Task1正向命令。
          return;
        }
        // 门槛已经满足但路线加载/规划暂未成功时，也不能在同一周期恢复
        // Task1正向巡线。真实二维码继续保持直接零速；默认流程保持原出口。
        set_control_state(ControlState::WAITING_FOR_QR);
        if (pending_from_qr) {
          publish_translation_stop_without_reverse_brake(
            qr_reverse_hold_angular_command_);
        } else {
          publish_translation_stop(qr_reverse_hold_angular_command_);
        }
        return;
      }
    } else if (task1_endpoint_waiting_) {
      if (!task1_endpoint_wait_timer_started_) {
        task1_endpoint_wait_timer_started_ = true;
        task1_endpoint_wait_started_at_ = steady_now;
        RCLCPP_INFO(
          get_logger(),
          "Task1末点已直接零速停车（不执行反相制动），开始计时等待二维码%.2f秒。",
          task1_endpoint_qr_wait_seconds_);
      }
      const double wait_seconds =
        std::chrono::duration<double>(
        steady_now - task1_endpoint_wait_started_at_).count();
      if (wait_seconds < task1_endpoint_qr_wait_seconds_) {
        set_control_state(ControlState::WAITING_FOR_QR);
        publish_translation_stop_without_reverse_brake(qr_reverse_hold_angular_command_);
        return;
      }
      if (!task1_endpoint_fallback_logged_) {
        task1_endpoint_fallback_logged_ = true;
        task1_pending_qr_text_ = task1_timeout_qr_text_;
        task1_pending_qr_is_fallback_ = true;
        task1_qr_speech_pending_ = true;
        task1_qr_speech_published_ = false;
        RCLCPP_WARN(
          get_logger(),
          "Task1末点二维码等待已结束：停车等待%.2f秒后仍未收到有效二维码，"
          "默认选择顺时针路线；进入下一阶段后按真实二维码流程播报%s。",
          wait_seconds, task1_timeout_qr_text_.c_str());
      }
      if (complete_task1_and_select_route(
          "clockwise", "Task1终点等待结束仍未识别到有效二维码", false))
      {
        return;
      }
    } else if (task1_departure_timer_started_) {
      const double elapsed_seconds =
        std::chrono::duration<double>(
        steady_now - task1_departure_started_at_).count();
      if (elapsed_seconds >= task1_qr_timeout_seconds_) {
        if (!task1_qr_timeout_logged_) {
          task1_qr_timeout_logged_ = true;
          task1_pending_qr_text_ = task1_timeout_qr_text_;
          task1_pending_qr_is_fallback_ = true;
          task1_qr_speech_pending_ = true;
          task1_qr_speech_published_ = false;
          RCLCPP_WARN(
            get_logger(),
            "Task1二维码识别已超时：可信发车后已等待%.2f秒，"
            "仍未收到有效二维码，默认选择顺时针路线；"
            "进入下一阶段后按真实二维码流程播报%s。",
            elapsed_seconds, task1_timeout_qr_text_.c_str());
        }
        if (complete_task1_and_select_route(
            "clockwise", "Task1可信发车后二维码识别超时", false))
        {
          return;
        }
      }
    }
  }

  if (task3_terminal_homing_active_) {
    control_task3_terminal_homing(pose);
    return;
  }

  if (return_to_start_active_) {
    control_return_to_start(pose);
    return;
  }

  next_control_state_ = ControlState::PATH_FOLLOWING;

  // 正式进入过渡且入口X要求强制倒车时，先完成一次反相停车制动。
  // 此状态位于路线重建和雷达判断之前：制动未结束时禁止继续正向巡线，
  // 也不允许用制动期间缓存的旧雷达帧决定倒车动作。
  if (qr_channel_approach_active_ &&
    qr_escape_state_ == QrEscapeState::PRE_REVERSE_BRAKE)
  {
    set_control_state(next_control_state_);
    publish_translation_stop(qr_reverse_hold_angular_command_);
    if (!stop_brake_sequence_active_) {
      qr_pre_reverse_scan_sequence_ = scan_sequence_;
      qr_pre_reverse_time_nanoseconds_ = now().nanoseconds();
      qr_escape_state_ = QrEscapeState::WAITING_FOR_PRE_REVERSE_SCAN;
      RCLCPP_INFO(
        get_logger(),
        "二维码倒车前反相制动完成：保持零速度，等待制动完成后的新雷达帧。"
      );
    }
    return;
  }

  if (qr_channel_approach_active_ &&
    qr_escape_state_ == QrEscapeState::WAITING_FOR_PRE_REVERSE_SCAN)
  {
    bool fresh_scan_ready =
      latest_scan_ && scan_sequence_ > qr_pre_reverse_scan_sequence_;
    if (fresh_scan_ready) {
      const std::int64_t scan_stamp_nanoseconds = rclcpp::Time(
        latest_scan_->header.stamp, get_clock()->get_clock_type()).nanoseconds();
      fresh_scan_ready = scan_stamp_nanoseconds == 0 ||
        scan_stamp_nanoseconds > qr_pre_reverse_time_nanoseconds_;
    }
    if (!fresh_scan_ready) {
      set_control_state(next_control_state_);
      publish_translation_stop_without_reverse_brake(qr_reverse_hold_angular_command_);
      return;
    }
    // 本周期的pose是停车命令后的最新位姿；PENDING直接使用刚确认的新
    // 雷达帧进入强制倒车状态。A→C→B规划必须等该状态退出后才执行。
    qr_escape_state_ = QrEscapeState::PENDING;
  }

  bool rebuilt_after_completed_reverse = false;
  auto rebuild_pending_channel_entry = [&]() {
      if (!channel_entry_route_rebuild_pending_) {
        return true;
      }
      const bool after_completed_reverse = channel_entry_route_rebuild_after_reverse_;
      const char * reason = after_completed_reverse ?
        "大厅倒车完成" : "二维码路线切换";
      if (rebuild_channel_entry_route(pose, reason)) {
        rebuilt_after_completed_reverse = after_completed_reverse;
        return true;
      }
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s后无法从当前base_link重建waypoint_channel进场路线，当前停车。",
        reason);
      if (after_completed_reverse) {
        publish_translation_stop(qr_reverse_hold_angular_command_);
      } else {
        publish_stop();
      }
      return false;
    };

  // 强制倒车状态必须先于任何A→C→B/circle_envelope规划执行。当前路线
  // 重建待办保留到倒车状态结束；否则一次马上会失效的规划会阻塞负速度。
  if (qr_channel_approach_active_) {
    geometry_msgs::msg::Twist reverse_cmd;
    if (apply_qr_reverse_maneuver(pose, reverse_cmd)) {
      set_control_state(next_control_state_);
      publish_velocity_command(reverse_cmd);
      publish_debug_visualization(pose);
      return;
    }
    // 每轮实际倒车退出后先由POST_STOP持续发布配置时长的零速度；只有
    // 停车结束，apply_qr_reverse_maneuver才会在这里重新置位重规划待办。
    // 随后用停车结束时的最新base_link重建A点；倒车机制可继续保持武装。
    if (!rebuild_pending_channel_entry()) {
      return;
    }
    if (rebuilt_after_completed_reverse) {
      // 停车结束的这一周期先完成重规划并明确下发零速；下一控制周期
      // 才沿新路线前进，避免重规划与正向指令在同一周期交接。
      set_control_state(next_control_state_);
      publish_translation_stop(qr_reverse_hold_angular_command_);
      publish_debug_visualization(pose);
      return;
    }
    update_qr_channel_approach_phase(pose);
  }

  if (!first_waypoint_reached_) {
    const auto & first = waypoints_.front();
    const double first_distance = std::hypot(first.x - pose.x, first.y - pose.y);
    if (first_distance <= waypoint_pass_radius(0)) {
      first_waypoint_reached_ = true;
      if (debug_control_logs_) {
        RCLCPP_INFO(get_logger(), "debug: 已到达起始航点。");
      }
    }
  }

  const bool freeze_image_capture_route_progress =
    image_capture_route_progress_guard_enabled_ &&
    image_capture_corridor_state_ == ImageCaptureCorridorState::STOPPING;
  if (image_capture_route_guard_active_ &&
    image_capture_corridor_state_ != ImageCaptureCorridorState::STOPPING)
  {
    // 拍照完成后的第一个起步控制周期先从停车锚点附近受限重投影；此时
    // find_route_projection()也受同一弧长窗口约束，不能被定位前跳带飞。
    reproject_image_capture_route_on_resume(pose);
  }
  if (first_waypoint_reached_ && !freeze_image_capture_route_progress) {
    update_route_progress(pose);
  }

  // 生成路线里的返程直线仍属于channel。这里只在进入该段后
  // 打开停车标志识别并执行停车判定，不切换追点或避障控制器。
  if (handle_task3_start(pose)) {
    return;
  }

  if (stop_if_route_finished(pose)) {
    return;
  }

  // Task2顶部直道不再追逐独立拍照点。进入当前方向的检测矩形后替换
  // Task2基础线速度；位置和航向合格后只停车一次。停车期间继续运行
  // 巡线和雷达避障，最终只覆盖平移速度。
  const bool image_capture_hold_translation = update_image_capture_corridor(pose);

  const double lookahead_distance = std::min(
    lookahead_max_,
    lookahead_base_ + lookahead_speed_gain_ * std::abs(latest_chassis_linear_speed_));
  auto target = compute_lookahead_target(pose, lookahead_distance);
  if (!target.reverse) {
    if (!logic3_planning_pose_anchored_) {
      logic3_force_full_route_ = true;
    }
    update_waypoint_shifts(pose, target.waypoint_index);
    logic3_force_full_route_ = false;
    target = compute_lookahead_target(pose, lookahead_distance);
  }

  geometry_msgs::msg::Twist cmd;
  bool path_tracking_failed = false;
  if (!target.reverse && tracking_controller_mode_ == "smac_pid") {
    if (!compute_path_pid_tracking_command(pose, cmd)) {
      // smac_pid失败时禁止用另一套控制器掩盖问题。保持路线和节点运行，
      // 但本周期明确输出零速；具体失败原因由对应ERROR日志给出。
      cmd = geometry_msgs::msg::Twist();
      path_tracking_failed = true;
    }
  } else {
    reset_path_pid_state();
    cmd = compute_tracking_command(pose, target);
  }
  const bool image_capture_corridor_speed_active =
    !path_tracking_failed && !target.reverse && cmd.linear.x > 0.0 &&
    image_capture_corridor_speed_ > 0.0 &&
    image_capture_corridor_contains(pose);
  if (image_capture_corridor_speed_active) {
    // 只替换Task2的基础正向速度，不改变巡线或避障计算出的转向量。
    cmd.linear.x = image_capture_corridor_speed_;
  }
  // 两套正向跟线控制器共用同一套转弯降速。检测矩形速度在这里之前
  // 替换基础速度，因此原有转弯减速仍继续生效。
  if (!path_tracking_failed) {
    apply_turn_speed_limit(cmd);
  }
  const double image_capture_corridor_turn_limited_cap =
    image_capture_corridor_speed_active && cmd.linear.x > 0.0 ?
    cmd.linear.x : std::numeric_limits<double>::infinity();
  const bool runtime_lidar_active = logic3_planning_pose_anchored_ &&
    task_phase_ != TaskPhase::FINISHED;
  if (runtime_lidar_active && !target.reverse && !path_tracking_failed) {
    if (apply_channel_lidar_obstacle_avoidance(cmd)) {
      // 雷达覆盖了本周期的路线控制命令，下次回到路线时重新建立PID状态。
      reset_path_pid_state();
    }
  }
  if (std::isfinite(image_capture_corridor_turn_limited_cap) && cmd.linear.x > 0.0) {
    // 雷达接管可以继续减速，但不能把矩形基础速度或转弯后的速度重新抬高。
    cmd.linear.x = std::min(cmd.linear.x, image_capture_corridor_turn_limited_cap);
  }
  // 只在Task1二维码尚未有效解码时，根据当前检测框高度封顶最终线速度。
  // 放在雷达控制之后，确保雷达分支不会把0.20m/s限制重新抬高。
  apply_qr_code_speed_limit(cmd);
  // Task3返程减速是最终正向速度上限，放在实时雷达接管之后，确保靠近
  // 基地时雷达避障也不会把速度重新抬高。零速和倒车不会被它改变。
  apply_task3_return_slowdown(pose, cmd);
  if (image_capture_hold_translation) {
    // 拍照停车只禁止车身平移，保留巡线或雷达避障最终算出的angular.z。
    // 不调用publish_stop()，避免舵机回中和巡线PID状态被拍照逻辑重置。
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
  }
  set_control_state(next_control_state_);
  publish_velocity_command(cmd);
  if (task1_dynamic_snapshot_pending_) {
    task1_dynamic_snapshot_pending_ = false;
    publish_logic3_snapshot("task1_dynamic");
  }
  if (!pending_control_snapshot_event_.empty()) {
    const std::string snapshot_event = pending_control_snapshot_event_;
    pending_control_snapshot_event_.clear();
    publish_logic3_snapshot(snapshot_event);
  }
  // Foxglove Marker构建会重新聚类并创建大量圆弧点；始终排在本周期
  // 底盘控制发布和必要的红点事件锁存之后，不能延迟首条运动指令。
  publish_debug_visualization(pose);
  // 轮速真正离开静止阈值后才恢复普通局部投影；没有轮速反馈时则在
  // 完成一次受限重投影后解除，避免永久锁在停车锚点附近。
  maybe_release_image_capture_route_guard();
}
