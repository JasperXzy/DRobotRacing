#include "origincar_system/visual_lidar_path_follower.hpp"

#include "origincar_system/circle_envelope.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using origincar_system::CircleIntersectionChord;
using origincar_system::circle_intersection_chord;
using origincar_system::normal_line_intersects_chord;
using origincar_system::point_near_chord_segment;

bool VisualLidarPathFollower::original_route_normal_at(
  std::size_t waypoint_index, VisualLidarPathFollower::Point2D & normal) const
{
  return origincar_system::left_normal_at(original_waypoints_, waypoint_index, normal);
}

// 逐点法线偏移在圆弧内侧满足 kappa * d >= 1 时会退化成尖点并反相。
// 这里先生成旧算法候选，再检测离散曲率、沿原路线进度和局部自交；只有
// 检测到不可行的最小连续偏移点段才改用同侧局部曲线；同一障碍窗口内的
// 其余点保留逐点法线偏移。局部曲线只允许改写危险点本身，并且每个原本
// 要求偏移的点都必须保留完整偏移量；若无法同时消除反相/自交并满足该
// 约束，就保留原始直接偏移（包括局部反相），绝不扩大融合或回退原路线。
std::size_t VisualLidarPathFollower::rebuild_curvature_unsafe_shift_segments(
  std::vector<Waypoint> & shifted_route)
{
  const std::size_t route_size = original_waypoints_.size();
  if (!waypoint_shift_enable_local_rebuild_ || route_size < 4 ||
    shifted_route.size() != route_size ||
    waypoint_shifts_.size() != route_size)
  {
    return 0;
  }

  auto normalize_vector = [](Point2D vector, Point2D & unit) {
      const double length = std::hypot(vector.x, vector.y);
      if (length < 1e-9) {
        return false;
      }
      unit = Point2D{vector.x / length, vector.y / length};
      return true;
    };
  auto route_tangent_at = [&](const std::vector<Waypoint> & route,
      std::size_t index, Point2D & tangent) {
      const std::size_t previous = index > 0 ? index - 1 : index;
      const std::size_t next = std::min(index + 1, route.size() - 1);
      return normalize_vector(
        Point2D{route[next].x - route[previous].x,
          route[next].y - route[previous].y}, tangent);
    };
  auto point_distance = [](const Waypoint & lhs, const Waypoint & rhs) {
      return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
    };
  auto signed_area = [](const Waypoint & a, const Waypoint & b, const Waypoint & c) {
      return (b.x - a.x) * (c.y - a.y) -
             (b.y - a.y) * (c.x - a.x);
    };
  auto proper_segments_intersect = [&](const Waypoint & a, const Waypoint & b,
      const Waypoint & c, const Waypoint & d) {
      constexpr double intersection_epsilon = 1e-9;
      const double ab_c = signed_area(a, b, c);
      const double ab_d = signed_area(a, b, d);
      const double cd_a = signed_area(c, d, a);
      const double cd_b = signed_area(c, d, b);
      return ((ab_c > intersection_epsilon && ab_d < -intersection_epsilon) ||
             (ab_c < -intersection_epsilon && ab_d > intersection_epsilon)) &&
             ((cd_a > intersection_epsilon && cd_b < -intersection_epsilon) ||
             (cd_a < -intersection_epsilon && cd_b > intersection_epsilon));
    };
  auto local_geometry_valid = [&](const std::vector<Waypoint> & route,
      std::size_t first, std::size_t last, std::string & invalid_reason) {
      // first/last是本轮真正改写的两个锚点。锚点之外的直接偏移没有被
      // 本轮修改，不能因为相邻障碍物在所有权交界处原本就有偏移跳变，
      // 反过来否决当前这段独立圆弧的合法重建。
      for (std::size_t index = first; index < last; ++index) {
        const double new_length = point_distance(route[index], route[index + 1]);
        const double original_length = point_distance(
          original_waypoints_[index], original_waypoints_[index + 1]);
        if (new_length < 1e-6 ||
          new_length > std::max(0.10, 6.0 * original_length))
        {
          invalid_reason = "相邻点距离异常";
          return false;
        }
      }
      // 锚点本身也要检查接入角，因此转角检查比改写区各向外读取一个
      // 未修改点；这里只读邻点，不把外侧线段当作本轮候选重新验证。
      const std::size_t turn_first = std::max<std::size_t>(1, first);
      const std::size_t turn_last = std::min(last, route_size - 2);
      for (std::size_t index = turn_first; index <= turn_last; ++index) {
        const Point2D incoming{
          route[index].x - route[index - 1].x,
          route[index].y - route[index - 1].y};
        const Point2D outgoing{
          route[index + 1].x - route[index].x,
          route[index + 1].y - route[index].y};
        const double incoming_length = std::hypot(incoming.x, incoming.y);
        const double outgoing_length = std::hypot(outgoing.x, outgoing.y);
        if (incoming_length < 1e-9 || outgoing_length < 1e-9) {
          invalid_reason = "接入处存在零长度线段";
          return false;
        }
        const double turn_cosine =
          (incoming.x * outgoing.x + incoming.y * outgoing.y) /
          (incoming_length * outgoing_length);
        if (turn_cosine < -0.25) {
          invalid_reason = "接入处或曲线内部出现急剧反向";
          return false;
        }
      }
      const std::size_t intersection_first = first > 0 ? first - 1 : first;
      const std::size_t intersection_last = std::min(last + 1, route_size - 1);
      for (std::size_t first_segment = intersection_first;
        first_segment + 1 <= intersection_last; ++first_segment)
      {
        for (std::size_t second_segment = first_segment + 2;
          second_segment + 1 <= intersection_last; ++second_segment)
        {
          if (proper_segments_intersect(
              route[first_segment], route[first_segment + 1],
              route[second_segment], route[second_segment + 1]))
          {
            invalid_reason = "局部曲线自交";
            return false;
          }
        }
      }
      invalid_reason.clear();
      return true;
    };
  auto chord_progress_valid = [&](const std::vector<Waypoint> & route,
      std::size_t first, std::size_t last, const Point2D & start,
      const Point2D & finish, std::string & invalid_reason) {
      if (first >= last) {
        return true;
      }
      const Point2D chord{finish.x - start.x, finish.y - start.y};
      const double chord_length_squared =
        chord.x * chord.x + chord.y * chord.y;
      if (chord_length_squared < 1e-12) {
        invalid_reason = "过渡段两锚点重合";
        return false;
      }
      // 圆角切弦本来就不会逐点平行于原圆弧切线，因此不能再用“同索引
      // 原线段点积”否决合法的切角路线。改为要求Bezier采样沿自己的起终
      // 点弦严格单调前进；这能直接禁止先向前再向后，同时允许独立圆弧
      // 在不侵入相邻障碍窗口的前提下切角接回。
      for (std::size_t index = first; index < last; ++index) {
        const Point2D segment{
          route[index + 1].x - route[index].x,
          route[index + 1].y - route[index].y};
        const double chord_progress =
          (segment.x * chord.x + segment.y * chord.y) /
          chord_length_squared;
        if (chord_progress <= 1e-6) {
          invalid_reason = "Bezier过渡沿自身接回弦发生停滞或反向";
          return false;
        }
      }
      return true;
    };
  auto fill_bezier = [&](std::vector<Waypoint> & route,
      std::size_t first, std::size_t last, const Point2D & start,
      const Point2D & finish, const Point2D & start_tangent,
      const Point2D & finish_tangent, double handle_ratio) {
      if (first >= last) {
        route[first].x = start.x;
        route[first].y = start.y;
        return;
      }
      const double chord_length = std::hypot(
        finish.x - start.x, finish.y - start.y);
      const double handle_length = handle_ratio * chord_length;
      const Point2D control1{
        start.x + handle_length * start_tangent.x,
        start.y + handle_length * start_tangent.y};
      const Point2D control2{
        finish.x - handle_length * finish_tangent.x,
        finish.y - handle_length * finish_tangent.y};
      double total_arc_length = 0.0;
      for (std::size_t index = first; index < last; ++index) {
        total_arc_length += point_distance(
          original_waypoints_[index], original_waypoints_[index + 1]);
      }
      double travelled = 0.0;
      for (std::size_t index = first; index <= last; ++index) {
        const double t = total_arc_length > 1e-9 ?
          travelled / total_arc_length :
          static_cast<double>(index - first) / static_cast<double>(last - first);
        const double one_minus_t = 1.0 - t;
        route[index].x =
          one_minus_t * one_minus_t * one_minus_t * start.x +
          3.0 * one_minus_t * one_minus_t * t * control1.x +
          3.0 * one_minus_t * t * t * control2.x + t * t * t * finish.x;
        route[index].y =
          one_minus_t * one_minus_t * one_minus_t * start.y +
          3.0 * one_minus_t * one_minus_t * t * control1.y +
          3.0 * one_minus_t * t * t * control2.y + t * t * t * finish.y;
        if (index < last) {
          travelled += point_distance(
            original_waypoints_[index], original_waypoints_[index + 1]);
        }
      }
    };
  auto requested_offsets_preserved = [&](const std::vector<Waypoint> & route,
      std::size_t first, std::size_t last, std::string & invalid_reason) {
      for (std::size_t index = first; index <= last; ++index) {
        const double requested_offset = std::abs(waypoint_shifts_[index].offset);
        if (requested_offset < 1e-9) {
          continue;
        }
        Point2D requested_direction;
        if (!normalize_vector(
            waypoint_shifts_[index].world_direction, requested_direction))
        {
          Point2D normal;
          if (!original_route_normal_at(index, normal)) {
            invalid_reason = "无法计算偏移量验收方向";
            return false;
          }
          const double sign = waypoint_shifts_[index].offset > 0.0 ? 1.0 : -1.0;
          requested_direction = Point2D{sign * normal.x, sign * normal.y};
        }
        const Point2D displacement{
          route[index].x - original_waypoints_[index].x,
          route[index].y - original_waypoints_[index].y};
        const double retained_offset =
          displacement.x * requested_direction.x +
          displacement.y * requested_direction.y;
        if (retained_offset + 1e-6 < requested_offset) {
          invalid_reason = "局部曲线削弱了前/后窗口设定的偏移量";
          return false;
        }
      }
      return true;
    };

  std::size_t rebuilt_count = 0;
  // 这些区间已经通过“沿自身接回弦严格前进 + 局部无折返/无自交”检查。
  // 最终守门不能再拿同索引原路线切线否决合法的切弯 Bezier；否则内弯
  // 虽然已经被安全切掉，仍会因为局部切线方向不同而被误判成反相。
  std::vector<std::pair<std::size_t, std::size_t>> chord_validated_spans;
  std::size_t run_first = 0;
  while (run_first < route_size) {
    if (std::abs(waypoint_shifts_[run_first].offset) < 1e-9) {
      ++run_first;
      continue;
    }
    const int offset_sign = waypoint_shifts_[run_first].offset > 0.0 ? 1 : -1;
    const std::size_t source_identity = waypoint_shifts_[run_first].source_index;
    std::size_t run_last = run_first;
    while (run_last + 1 < route_size &&
      waypoint_shifts_[run_last + 1].offset * static_cast<double>(offset_sign) > 1e-9)
    {
      const std::size_t next_source = waypoint_shifts_[run_last + 1].source_index;
      if (source_identity < route_size && next_source < route_size &&
        next_source != source_identity)
      {
        break;
      }
      ++run_last;
    }

    // 风险判定保留逐点粒度：同一障碍物的整个直接偏移窗口中，只有真正
    // 发生曲率退化、前进反相或自交的点才进入后续局部曲线重建。
    const std::size_t run_length = run_last - run_first + 1;
    std::vector<bool> unsafe_points(run_length, false);
    std::vector<double> risk_factors(
      run_length, std::numeric_limits<double>::infinity());
    auto mark_unsafe_point = [&](std::size_t index, double risk_factor) {
        if (index < run_first || index > run_last) {
          return;
        }
        const std::size_t local_index = index - run_first;
        unsafe_points[local_index] = true;
        risk_factors[local_index] = std::min(
          risk_factors[local_index], risk_factor);
      };

    const std::size_t curvature_first = std::max<std::size_t>(1, run_first);
    const std::size_t curvature_last = std::min(run_last, route_size - 2);
    for (std::size_t index = curvature_first; index <= curvature_last; ++index) {
      const Point2D previous_segment{
        original_waypoints_[index].x - original_waypoints_[index - 1].x,
        original_waypoints_[index].y - original_waypoints_[index - 1].y};
      const Point2D next_segment{
        original_waypoints_[index + 1].x - original_waypoints_[index].x,
        original_waypoints_[index + 1].y - original_waypoints_[index].y};
      const double previous_length = std::hypot(previous_segment.x, previous_segment.y);
      const double next_length = std::hypot(next_segment.x, next_segment.y);
      if (previous_length < 1e-9 || next_length < 1e-9) {
        continue;
      }
      const Point2D previous_tangent{
        previous_segment.x / previous_length, previous_segment.y / previous_length};
      const Point2D next_tangent{
        next_segment.x / next_length, next_segment.y / next_length};
      const double turn_angle = std::atan2(
        previous_tangent.x * next_tangent.y -
        previous_tangent.y * next_tangent.x,
        previous_tangent.x * next_tangent.x +
        previous_tangent.y * next_tangent.y);
      const double curvature = turn_angle / (0.5 * (previous_length + next_length));
      const double feasibility =
        1.0 - curvature * waypoint_shifts_[index].offset;
      if (feasibility <= waypoint_shift_rebuild_min_progress_ratio_) {
        mark_unsafe_point(index, feasibility);
      }
    }

    const std::size_t progress_first = run_first > 0 ? run_first - 1 : run_first;
    const std::size_t progress_last = std::min(run_last, route_size - 2);
    for (std::size_t index = progress_first; index <= progress_last; ++index) {
      const Point2D original_segment{
        original_waypoints_[index + 1].x - original_waypoints_[index].x,
        original_waypoints_[index + 1].y - original_waypoints_[index].y};
      const double original_length_squared =
        original_segment.x * original_segment.x +
        original_segment.y * original_segment.y;
      if (original_length_squared < 1e-12) {
        continue;
      }
      const Point2D shifted_segment{
        shifted_route[index + 1].x - shifted_route[index].x,
        shifted_route[index + 1].y - shifted_route[index].y};
      const double progress_ratio =
        (original_segment.x * shifted_segment.x +
        original_segment.y * shifted_segment.y) / original_length_squared;
      if (progress_ratio <= waypoint_shift_rebuild_min_progress_ratio_) {
        // 反相发生在线段上；只标记该线段位于当前偏移窗口内的端点。
        mark_unsafe_point(index, progress_ratio);
        mark_unsafe_point(index + 1, progress_ratio);
      }
    }

    const std::size_t intersection_first = progress_first;
    const std::size_t intersection_last = std::min(run_last + 1, route_size - 1);
    for (std::size_t first_segment = intersection_first;
      first_segment + 1 <= intersection_last; ++first_segment)
    {
      for (std::size_t second_segment = first_segment + 2;
        second_segment + 1 <= intersection_last; ++second_segment)
      {
        if (!proper_segments_intersect(
            shifted_route[first_segment], shifted_route[first_segment + 1],
            shifted_route[second_segment], shifted_route[second_segment + 1]))
        {
          continue;
        }
        // 两条不相邻线段相交时，中间这段构成局部回环；仅标记回环内部点。
        const std::size_t loop_first = std::max(run_first, first_segment + 1);
        const std::size_t loop_last = std::min(run_last, second_segment);
        for (std::size_t index = loop_first; index <= loop_last; ++index) {
          mark_unsafe_point(index, 0.0);
        }
      }
    }

    std::size_t unsafe_first = run_first;
    while (unsafe_first <= run_last) {
      while (unsafe_first <= run_last &&
        !unsafe_points[unsafe_first - run_first])
      {
        ++unsafe_first;
      }
      if (unsafe_first > run_last) {
        break;
      }
      std::size_t unsafe_last = unsafe_first;
      while (unsafe_last < run_last &&
        unsafe_points[unsafe_last + 1 - run_first])
      {
        ++unsafe_last;
      }

      double minimum_risk_factor = std::numeric_limits<double>::infinity();
      std::size_t risk_index = unsafe_first;
      for (std::size_t index = unsafe_first; index <= unsafe_last; ++index) {
        const double risk_factor = risk_factors[index - run_first];
        if (risk_factor < minimum_risk_factor) {
          minimum_risk_factor = risk_factor;
          risk_index = index;
        }
      }

      // B前的轻微圆滑已经是偏移前基准路线的一部分。这一小段若再进入
      // 偏移后曲率兜底，就会破坏1/2—1/2直接偏移分配；仅跳过与B重叠的
      // 危险子段，不再连带跳过或重建同一障碍窗口的其他点。
      const std::size_t repair_context_first =
        unsafe_first > 0 ? unsafe_first - 1 : unsafe_first;
      const std::size_t repair_context_last =
        std::min(unsafe_last + 1, route_size - 1);
      const bool overlaps_pre_shift_entry_smoothing =
        channel_entry_waypoint_index_ > 0 &&
        channel_entry_smoothing_first_index_ < channel_entry_waypoint_index_ &&
        repair_context_first <= channel_entry_waypoint_index_ &&
        repair_context_last >= channel_entry_smoothing_first_index_;
      if (overlaps_pre_shift_entry_smoothing) {
        RCLCPP_INFO(
          get_logger(),
          "waypoint_shift：危险子段点%zu-%zu与B前基准圆滑区点%zu-%zu重叠；"
          "保留该处直接偏移，跳过偏移后局部曲线重构。",
          unsafe_first + 1, unsafe_last + 1,
          channel_entry_smoothing_first_index_ + 1,
          channel_entry_waypoint_index_ + 1);
        unsafe_first = unsafe_last + 1;
        continue;
      }

      // 只处理真正发生反相/自交的危险点。真实障碍来源点若不在危险子段
      // 内，不能为了让曲线经过它而把整段前/后偏移窗口一起拉成大圆弧。
      const std::size_t source_index =
        source_identity >= unsafe_first && source_identity <= unsafe_last ?
        source_identity : risk_index;
      Point2D fixed_direction = waypoint_shifts_[source_index].world_direction;
      if (!normalize_vector(fixed_direction, fixed_direction)) {
        fixed_direction = Point2D();
        for (std::size_t index = unsafe_first; index <= unsafe_last; ++index) {
          fixed_direction.x += waypoint_shifts_[index].world_direction.x;
          fixed_direction.y += waypoint_shifts_[index].world_direction.y;
        }
        if (!normalize_vector(fixed_direction, fixed_direction)) {
          Point2D normal;
          if (!original_route_normal_at(source_index, normal)) {
            unsafe_first = unsafe_last + 1;
            continue;
          }
          fixed_direction = Point2D{
            static_cast<double>(offset_sign) * normal.x,
            static_cast<double>(offset_sign) * normal.y};
        }
      }
      double target_offset = 0.0;
      for (std::size_t index = unsafe_first; index <= unsafe_last; ++index) {
        target_offset = std::max(
          target_offset, std::abs(waypoint_shifts_[index].offset));
      }
      if (target_offset < 1e-9) {
        unsafe_first = unsafe_last + 1;
        continue;
      }

      // 两个接回锚点严格取危险子段的相邻点：锚点不改写，Bezier只覆盖
      // unsafe_first..unsafe_last。若相邻点不足或这一最小局部桥不可行，
      // 就保留原始直接偏移；不再向障碍中心或窗口外扩展几十个路径点。
      const std::size_t repair_core_first = unsafe_first;
      const std::size_t repair_core_last = unsafe_last;
      const std::size_t anchor_search_first =
        repair_core_first > 0 ? repair_core_first - 1 : repair_core_first;
      const std::size_t anchor_search_last =
        std::min(repair_core_last + 1, route_size - 1);
      const std::size_t maximum_context = std::max(
        repair_core_first - anchor_search_first,
        anchor_search_last - repair_core_last);
      std::size_t accepted_anchor_first = unsafe_first;
      std::size_t accepted_anchor_last = unsafe_last;
      double accepted_handle_ratio = 0.0;
      double accepted_clearance = target_offset;
      const char * accepted_bridge_mode = "单段切角桥";
      std::string last_invalid_reason = "没有可用的双侧接回锚点";
      bool valid_rebuild = false;
      const double handle_ratios[] = {
        waypoint_shift_rebuild_bezier_handle_ratio_,
        0.75 * waypoint_shift_rebuild_bezier_handle_ratio_,
        0.50 * waypoint_shift_rebuild_bezier_handle_ratio_,
        0.25 * waypoint_shift_rebuild_bezier_handle_ratio_, 0.0};
      std::vector<Waypoint> accepted_candidate;
      std::size_t previous_anchor_first = route_size;
      std::size_t previous_anchor_last = route_size;
      for (std::size_t context = 1;
        context <= maximum_context && !valid_rebuild; ++context)
      {
        const std::size_t anchor_first =
          repair_core_first > anchor_search_first + context ?
          repair_core_first - context : anchor_search_first;
        const std::size_t anchor_last =
          std::min(anchor_search_last, repair_core_last + context);
        if (anchor_first == previous_anchor_first &&
          anchor_last == previous_anchor_last)
        {
          continue;
        }
        previous_anchor_first = anchor_first;
        previous_anchor_last = anchor_last;
        if (anchor_first >= repair_core_first || anchor_last <= repair_core_last ||
          source_index <= anchor_first || source_index >= anchor_last)
        {
          continue;
        }

        const Point2D start_point{
          shifted_route[anchor_first].x, shifted_route[anchor_first].y};
        const Point2D finish_point{
          shifted_route[anchor_last].x, shifted_route[anchor_last].y};
        double original_span_length = 0.0;
        for (std::size_t index = anchor_first; index < anchor_last; ++index) {
          original_span_length += point_distance(
            original_waypoints_[index], original_waypoints_[index + 1]);
        }
        const double bridge_chord_length = std::hypot(
          finish_point.x - start_point.x, finish_point.y - start_point.y);
        if (original_span_length < 1e-9 ||
          bridge_chord_length / original_span_length <=
          waypoint_shift_rebuild_min_progress_ratio_)
        {
          last_invalid_reason = "Bezier桥弦长过短，会把危险圆弧压缩成近似原地转向";
          continue;
        }
        Point2D start_tangent;
        Point2D finish_tangent;
        Point2D source_tangent;
        if (!route_tangent_at(original_waypoints_, anchor_first, start_tangent) ||
          !route_tangent_at(original_waypoints_, anchor_last, finish_tangent) ||
          !route_tangent_at(original_waypoints_, source_index, source_tangent))
        {
          last_invalid_reason = "无法计算接回切向";
          continue;
        }

        for (const double handle_ratio : handle_ratios) {
          std::vector<Waypoint> rebuilt_candidate = shifted_route;
          fill_bezier(
            rebuilt_candidate, anchor_first, anchor_last,
            start_point, finish_point, start_tangent, finish_tangent,
            handle_ratio);
          const Point2D source_displacement{
            rebuilt_candidate[source_index].x - original_waypoints_[source_index].x,
            rebuilt_candidate[source_index].y - original_waypoints_[source_index].y};
          const double source_clearance =
            source_displacement.x * fixed_direction.x +
            source_displacement.y * fixed_direction.y;
          std::string candidate_invalid_reason;
          const bool single_bridge_geometry_valid =
            chord_progress_valid(
              rebuilt_candidate, anchor_first, anchor_last,
              start_point, finish_point, candidate_invalid_reason) &&
            local_geometry_valid(
              rebuilt_candidate, anchor_first, anchor_last,
              candidate_invalid_reason) &&
            requested_offsets_preserved(
              rebuilt_candidate, repair_core_first, repair_core_last,
              candidate_invalid_reason);
          if (single_bridge_geometry_valid)
          {
            valid_rebuild = true;
            accepted_anchor_first = anchor_first;
            accepted_anchor_last = anchor_last;
            accepted_handle_ratio = handle_ratio;
            accepted_clearance = source_clearance;
            accepted_bridge_mode = "单段切角桥";
            accepted_candidate = std::move(rebuilt_candidate);
            break;
          }
          if (!single_bridge_geometry_valid) {
            last_invalid_reason = candidate_invalid_reason;
          }

          // 外弯的直接等距曲线本身不会发生曲率坍缩；单段切弦反而可能
          // 向内吃掉已经满足的净空。此时保留来源索引处的完整直接偏移点，
          // 只用两段Bezier消除局部折线/自交。内弯若使用这个中点会再次
          // 折返，下面的双弦单调、转角和自交检查会把它拒绝。
          const Point2D shifted_source{
            shifted_route[source_index].x, shifted_route[source_index].y};
          rebuilt_candidate = shifted_route;
          fill_bezier(
            rebuilt_candidate, anchor_first, source_index,
            start_point, shifted_source, start_tangent, source_tangent,
            handle_ratio);
          fill_bezier(
            rebuilt_candidate, source_index, anchor_last,
            shifted_source, finish_point, source_tangent, finish_tangent,
            handle_ratio);
          if (chord_progress_valid(
              rebuilt_candidate, anchor_first, source_index,
              start_point, shifted_source, last_invalid_reason) &&
            chord_progress_valid(
              rebuilt_candidate, source_index, anchor_last,
              shifted_source, finish_point, last_invalid_reason) &&
            local_geometry_valid(
              rebuilt_candidate, anchor_first, anchor_last, last_invalid_reason) &&
            requested_offsets_preserved(
              rebuilt_candidate, repair_core_first, repair_core_last,
              last_invalid_reason))
          {
            valid_rebuild = true;
            accepted_anchor_first = anchor_first;
            accepted_anchor_last = anchor_last;
            accepted_handle_ratio = handle_ratio;
            accepted_clearance = std::abs(waypoint_shifts_[source_index].offset);
            accepted_bridge_mode = "双段外弯保净空桥";
            accepted_candidate = std::move(rebuilt_candidate);
            break;
          }
        }
      }
      // 最小局部桥若失败，原始直接偏移保持不动。允许巡点逻辑和到点容限
      // 消化这一处局部反相/自交，绝不扩大曲线范围或恢复原路线。
      if (!valid_rebuild) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "waypoint_shift：危险子段%zu-%zu在所属直接偏移窗口%zu-%zu内仍无法"
          "生成保持完整净距、严格前进且无自交的独立同侧曲线（最后原因：%s）；"
          "保留前/后窗口的原始完整偏移及该处局部反相/自交，不扩大融合、不回退原路线。",
          unsafe_first + 1, unsafe_last + 1, run_first + 1, run_last + 1,
          last_invalid_reason.c_str());
        unsafe_first = unsafe_last + 1;
        continue;
      }
      shifted_route = std::move(accepted_candidate);
      chord_validated_spans.emplace_back(
        accepted_anchor_first, accepted_anchor_last);
      ++rebuilt_count;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waypoint_shift：检测到圆弧内侧偏移不可行（危险点%zu-%zu，"
        "所属直接偏移窗口%zu-%zu，最小进度/可行因子=%.3f），"
        "仅在点%zu-%zu内用%s独立重建危险圆弧，来源点=%zu，"
        "危险段最大设定偏移/来源点实际偏移=%.3f/%.3fm，"
        "且危险段逐点偏移验收通过；Bezier控制柄比例=%.3f。",
        unsafe_first + 1, unsafe_last + 1, run_first + 1, run_last + 1,
        minimum_risk_factor, accepted_anchor_first + 1, accepted_anchor_last + 1,
        accepted_bridge_mode, source_index + 1, target_offset,
        accepted_clearance, accepted_handle_ratio);
      unsafe_first = unsafe_last + 1;
    }
    run_first = run_last + 1;
  }

  // 最终整条路线只做诊断：上面的最小局部算法已经尝试处理反向、折返和
  // 自交；若仍有问题，按要求保留完整直接偏移和局部几何，不再扩大范围。
  auto route_safety_issue = [&](const std::vector<Waypoint> & route,
      std::vector<std::size_t> & involved_indices, std::string & reason) {
      involved_indices.clear();
      for (std::size_t index = 0; index + 1 < route_size; ++index) {
        const bool belongs_to_validated_chord = std::any_of(
          chord_validated_spans.begin(), chord_validated_spans.end(),
          [&](const auto & span) {
            return index >= span.first && index + 1 <= span.second;
          });
        if (belongs_to_validated_chord) {
          // 该段已经按Bezier自身的起终弦验证为严格前进。切弯后局部切线
          // 本来就可能不同于同索引原圆弧切线，不能重复按原切线判反相。
          continue;
        }
        const Point2D original_segment{
          original_waypoints_[index + 1].x - original_waypoints_[index].x,
          original_waypoints_[index + 1].y - original_waypoints_[index].y};
        const double original_length_squared =
          original_segment.x * original_segment.x +
          original_segment.y * original_segment.y;
        if (original_length_squared < 1e-12) {
          continue;
        }
        const Point2D shifted_segment{
          route[index + 1].x - route[index].x,
          route[index + 1].y - route[index].y};
        const double progress_ratio =
          (shifted_segment.x * original_segment.x +
          shifted_segment.y * original_segment.y) / original_length_squared;
        if (progress_ratio < -0.05) {
          involved_indices = {index, index + 1};
          reason = "出现相对原路线明确反向的线段";
          return true;
        }
      }
      for (std::size_t index = 1; index + 1 < route_size; ++index) {
        const Point2D incoming{
          route[index].x - route[index - 1].x,
          route[index].y - route[index - 1].y};
        const Point2D outgoing{
          route[index + 1].x - route[index].x,
          route[index + 1].y - route[index].y};
        const double incoming_length = std::hypot(incoming.x, incoming.y);
        const double outgoing_length = std::hypot(outgoing.x, outgoing.y);
        if (incoming_length < 1e-9 || outgoing_length < 1e-9) {
          continue;
        }
        const double turn_cosine =
          (incoming.x * outgoing.x + incoming.y * outgoing.y) /
          (incoming_length * outgoing_length);
        if (turn_cosine >= -0.25) {
          continue;
        }
        const Point2D original_incoming{
          original_waypoints_[index].x - original_waypoints_[index - 1].x,
          original_waypoints_[index].y - original_waypoints_[index - 1].y};
        const Point2D original_outgoing{
          original_waypoints_[index + 1].x - original_waypoints_[index].x,
          original_waypoints_[index + 1].y - original_waypoints_[index].y};
        const double original_denominator =
          std::hypot(original_incoming.x, original_incoming.y) *
          std::hypot(original_outgoing.x, original_outgoing.y);
        const double original_turn_cosine = original_denominator > 1e-12 ?
          (original_incoming.x * original_outgoing.x +
          original_incoming.y * original_outgoing.y) / original_denominator : 1.0;
        if (original_turn_cosine >= -0.25) {
          involved_indices = {index - 1, index, index + 1};
          reason = "静态偏移新引入急剧折返";
          return true;
        }
      }
      for (std::size_t first_segment = 0;
        first_segment + 1 < route_size; ++first_segment)
      {
        for (std::size_t second_segment = first_segment + 2;
          second_segment + 1 < route_size; ++second_segment)
        {
          if (!proper_segments_intersect(
              route[first_segment], route[first_segment + 1],
              route[second_segment], route[second_segment + 1]) ||
            proper_segments_intersect(
              original_waypoints_[first_segment],
              original_waypoints_[first_segment + 1],
              original_waypoints_[second_segment],
              original_waypoints_[second_segment + 1]))
          {
            continue;
          }
          involved_indices = {
            first_segment, first_segment + 1,
            second_segment, second_segment + 1};
          reason = "静态偏移新引入路线自交";
          return true;
        }
      }
      reason.clear();
      return false;
    };

  std::vector<std::size_t> unsafe_final_indices;
  std::string final_invalid_reason;
  if (route_safety_issue(
      shifted_route, unsafe_final_indices, final_invalid_reason))
  {
    const auto unsafe_bounds = std::minmax_element(
      unsafe_final_indices.begin(), unsafe_final_indices.end());
    if (!logic3_ready_preplanning_active_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waypoint_shift最终检查仍发现%s（涉及点%zu-%zu）；"
        "局部最小重建已失败或不适用，按要求保留前/后窗口的全部原始偏移"
        "和该处局部几何，不扩大融合、不降低偏移量、不恢复原路线。",
        final_invalid_reason.c_str(),
        *unsafe_bounds.first + 1, *unsafe_bounds.second + 1);
    }
  }
  return rebuilt_count;
}

void VisualLidarPathFollower::rebuild_route_from_confirmed_shifts()
{
  if (original_waypoints_.size() != waypoint_shifts_.size()) {
    return;
  }
  waypoints_ = original_waypoints_;
  if (route_deformation_mode_ == "circle_envelope") {
    apply_circle_envelope_deformation(waypoints_);
  } else {
    for (std::size_t index = 0; index < waypoints_.size(); ++index) {
      if (std::abs(waypoint_shifts_[index].offset) < 1e-9) {
        continue;
      }
      Point2D normal;
      if (!original_route_normal_at(index, normal)) {
        continue;
      }
      waypoints_[index].x += waypoint_shifts_[index].offset * normal.x;
      waypoints_[index].y += waypoint_shifts_[index].offset * normal.y;
    }
    rebuild_curvature_unsafe_shift_segments(waypoints_);
  }

  if (following_qr_path_ || task_phase_ == TaskPhase::TASK1) {
    for (auto & waypoint : waypoints_) {
      waypoint.y = std::max(waypoint.y, task1_waypoint_shift_min_y_);
    }
  }
  if (channel_entry_waypoint_index_ > 0 && !waypoints_.empty()) {
    waypoints_.front() = original_waypoints_.front();
  }
  validate_logic3_recorded_obstacle_route_clearance();
  rebuild_path_pid_route_cache();
  reset_path_pid_state();
}

void VisualLidarPathFollower::validate_logic3_recorded_obstacle_route_clearance()
{
  if (route_deformation_mode_ != "circle_envelope" || waypoints_.size() < 2 ||
    waypoint_shifts_.size() != waypoints_.size())
  {
    return;
  }

  struct RecordedObstacleRouteCandidate
  {
    Point2D center;
    double forbidden_radius{0.0};
    std::size_t first_segment{0};
  };
  auto obstacle_first_segment = [&](const Point2D & center) {
      if (!channel_entry_rear_static_filter_active_) {
        return std::size_t{0};
      }
      const double relative_x =
        center.x - channel_entry_rear_static_filter_start_x_;
      const double relative_y =
        center.y - channel_entry_rear_static_filter_start_y_;
      const double forward_projection =
        relative_x * std::cos(channel_entry_rear_static_filter_start_yaw_) +
        relative_y * std::sin(channel_entry_rear_static_filter_start_yaw_);
      if (forward_projection >= 0.0) {
        return std::size_t{0};
      }
      return std::min(
        channel_entry_rear_static_filter_first_index_,
        waypoints_.size() - 1);
    };
  std::vector<RecordedObstacleRouteCandidate> candidates;
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    if (std::abs(waypoint_shifts_[index].offset) < 1.0e-9 ||
      waypoint_shifts_[index].forbidden_radius <= 1.0e-9)
    {
      continue;
    }
    const auto & center = waypoint_shifts_[index].obstacle_center;
    const std::size_t first_segment = obstacle_first_segment(center);
    auto existing = std::find_if(
      candidates.begin(), candidates.end(),
      [&](const RecordedObstacleRouteCandidate & candidate) {
        return std::hypot(
          candidate.center.x - center.x,
          candidate.center.y - center.y) <= 0.05;
      });
    if (existing == candidates.end()) {
      candidates.push_back(
        RecordedObstacleRouteCandidate{
          center, waypoint_shifts_[index].forbidden_radius, first_segment});
    } else {
      existing->forbidden_radius = std::max(
        existing->forbidden_radius,
        waypoint_shifts_[index].forbidden_radius);
      existing->first_segment = std::max(
        existing->first_segment, first_segment);
    }
  }
  if (logic3_planning_pose_anchored_)
  {
    const double permanent_forbidden_radius =
      circle_envelope_target_clearance_ + circle_envelope_numeric_margin_;
    for (std::size_t obstacle_index = 0;
      obstacle_index < logic3_green_clusters_.size(); ++obstacle_index)
    {
      const bool dynamic_takeover =
        obstacle_index < logic3_green_dynamic_takeover_.size() &&
        logic3_green_dynamic_takeover_[obstacle_index];
      if (dynamic_takeover) {
        continue;
      }
      const auto & center = logic3_green_clusters_[obstacle_index].center;
      const std::size_t first_segment = obstacle_first_segment(center);
      auto existing = std::find_if(
        candidates.begin(), candidates.end(),
        [&](const RecordedObstacleRouteCandidate & candidate) {
          return std::hypot(
            candidate.center.x - center.x,
            candidate.center.y - center.y) <= 0.05;
        });
      if (existing == candidates.end()) {
        candidates.push_back(
          RecordedObstacleRouteCandidate{
            center, permanent_forbidden_radius, first_segment});
      } else {
        existing->forbidden_radius = std::max(
          existing->forbidden_radius, permanent_forbidden_radius);
        existing->first_segment = std::max(
          existing->first_segment, first_segment);
      }
    }
  }

  auto closest_point_on_segment = [](const Point2D & point,
      const Waypoint & start, const Waypoint & finish) {
      const double segment_x = finish.x - start.x;
      const double segment_y = finish.y - start.y;
      const double length_squared =
        segment_x * segment_x + segment_y * segment_y;
      if (length_squared < 1.0e-12) {
        return Point2D{start.x, start.y};
      }
      const double t = std::clamp(
        ((point.x - start.x) * segment_x +
        (point.y - start.y) * segment_y) / length_squared,
        0.0, 1.0);
      return Point2D{
        start.x + t * segment_x,
        start.y + t * segment_y};
    };

  for (std::size_t candidate_index = 0;
    candidate_index < candidates.size(); ++candidate_index)
  {
    const auto & candidate = candidates[candidate_index];
    double route_distance = std::numeric_limits<double>::infinity();
    Point2D closest_route_point;
    for (std::size_t index = candidate.first_segment;
      index + 1 < waypoints_.size(); ++index)
    {
      const Point2D closest = closest_point_on_segment(
        candidate.center, waypoints_[index], waypoints_[index + 1]);
      const double distance = std::hypot(
        closest.x - candidate.center.x,
        closest.y - candidate.center.y);
      if (distance < route_distance) {
        route_distance = distance;
        closest_route_point = closest;
      }
    }
    // 路线净距在apply_circle_envelope_deformation中已做硬验收；未通过
    // target_clearance的ID会解除绿/橙掩码并交给实时雷达。这里保留第二份
    // 诊断日志，不再修改路线或覆盖前面的失败标记。
    const double required_route_distance = std::max(
      0.0,
      candidate.forbidden_radius - circle_envelope_numeric_margin_);
    if (route_distance + 1.0e-6 >= required_route_distance) {
      continue;
    }

    bool intentional_overlap_corridor = false;
    if (route_deformation_mode_ == "circle_envelope") {
      for (std::size_t partner_index = 0;
        partner_index < candidates.size(); ++partner_index)
      {
        if (partner_index == candidate_index) {
          continue;
        }
        const auto & partner = candidates[partner_index];
        CircleIntersectionChord chord;
        if (circle_intersection_chord(
            candidate.center.x, candidate.center.y,
            candidate.forbidden_radius,
            partner.center.x, partner.center.y,
            partner.forbidden_radius,
            chord) &&
          point_near_chord_segment(
            closest_route_point.x, closest_route_point.y, chord,
            std::max(0.02, 0.5 * circle_envelope_numeric_margin_)))
        {
          intentional_overlap_corridor = true;
          break;
        }
      }
    }
    if (intentional_overlap_corridor) {
      continue;
    }
    if (!logic3_ready_preplanning_active_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "circle_envelope路线对已记录障碍(%.3f, %.3f)的最近距离%.3fm小于要求%.3fm；"
        "仅标记路线净距异常，不修改该永久点及当前控制器的障碍处理策略。",
        candidate.center.x, candidate.center.y,
        route_distance, required_route_distance);
    }
  }
}

// 把已确认的点云障碍转换成“固定安全圆 + 圆滑接线”。对每个障碍只做三件事：
// 1. 使用waypoint_shift已经确定的绕行方向，后面不得翻转；
// 2. 原路线被以circle_envelope_target_clearance为半径的圆覆盖时，
//    将整段路线限制在指定侧的圆边界外，不再只锁一个障碍锚点；
// 3. 从未偏移的original_waypoints_同时生成所有接线并整体圆滑。
// 两个安全圆相交时不合并障碍；若相反方向的圆约束已无法同时满足，
// 使用含numeric_margin的两个规划圆交点形成公共弦，并圆滑连接弦的两端。
void VisualLidarPathFollower::apply_circle_envelope_deformation(
  std::vector<Waypoint> & shifted_route)
{
  origincar_system::CircleEnvelopeRequest request{
    {}, {
      circle_envelope_target_clearance_,
      circle_envelope_numeric_margin_,
      circle_envelope_min_transition_length_,
      circle_envelope_max_transition_length_,
      circle_envelope_extension_step_,
      circle_envelope_merge_gap_,
      circle_envelope_max_curvature_,
      circle_envelope_smoothing_iterations_,
      cone_circle_max_radius_,
      waypoint_cloud_check_radius_,
      waypoint_shift_rebuild_min_progress_ratio_},
    std::nullopt, std::nullopt, logic3_planning_pose_anchored_};
  request.obstacles.reserve(logic3_green_clusters_.size());
  for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
    request.obstacles.push_back({
      logic3_green_clusters_[index].center,
      index < logic3_green_dynamic_takeover_.size() && logic3_green_dynamic_takeover_[index]});
  }
  if (channel_entry_rear_static_filter_active_) {
    request.rear_obstacle_limit = origincar_system::RearObstacleLimit{
      {channel_entry_rear_static_filter_start_x_, channel_entry_rear_static_filter_start_y_},
      channel_entry_rear_static_filter_start_yaw_, channel_entry_rear_static_filter_first_index_};
  }
  if (following_qr_path_ || task_phase_ == TaskPhase::TASK1) {
    request.minimum_y = task1_waypoint_shift_min_y_;
  }

  auto plan = origincar_system::plan_circle_envelope(
    original_waypoints_, waypoint_shifts_, request,
    [this](const origincar_system::ClearanceRepair & repair) {
      RCLCPP_WARN(
        get_logger(),
        "circle_envelope二次净距审计第%zu轮：处理%zu个真实侵圆连续段；"
        "已按固定侧加入局部圆约束并统一圆滑重算；累计拒绝%zu次净距未改善候选。",
        repair.round, repair.constraint_segments, repair.rejected_candidates);
    });
  if (!plan) {
    return;
  }
  logic3_green_route_clearance_failed_ =
    std::move(plan->obstacles_requiring_realtime_avoidance);
  const auto & report = plan->report;
  if (!report.shaped) {
    return;
  }
  shifted_route = std::move(plan->route);
  if (logic3_ready_preplanning_active_) {
    return;
  }
  // READY预规划是发车前的纯计算候选：不发布调试图，也不用这些
  // 过程信息刷屏。真正冻结或运行中重规划仍保留原有日志。
  if (report.shared_passes > 0) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope共用通道：识别%zu组正反重复经过、%zu对同位航点；"
      "仅统一障碍世界绕行侧，不再强制平均两次路线坐标。",
      report.shared_passes, report.shared_waypoint_pairs);
  }
  if (report.clearance_failures > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope第一次生成及局部复查后仍有%zu个独立安全圆净距不足；"
      "没有合并障碍物或翻转方向，继续输出当前最优路线。",
      report.clearance_failures);
  }
  if (report.permanent_clearance_failures > 0) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope二次净距审计后仍有%zu个永久障碍不满足target_clearance；"
      "保留当前最外推路线且不停车、不回退原路线；这些ID本轮解除绿/橙掩码，"
      "继续由实时雷达执行偏移。",
      report.permanent_clearance_failures);
  }
  if (report.anchor_conflicts > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope有%zu个非相交圆路线点同时受到相反约束；"
      "障碍物保持独立，未换向。",
      report.anchor_conflicts);
  }
  if (report.progress_violations > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope路线仍有%zu个局部点对原路线进度反向；"
      "局部Bezier桥未通过净距或方向校验，保留橙色基线路线。",
      report.progress_violations);
  }
  if (!report.accepted &&
    report.maximum_curvature > circle_envelope_max_curvature_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope最大离散曲率%.3f仍高于软目标%.3f；"
      "安全圆、独立方向和原路线进度优先，继续使用该路线。",
      report.maximum_curvature, circle_envelope_max_curvature_);
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "circle_envelope生成：第一次约束=%zu，二次新增=%zu，主约束升级=%zu，"
    "区间扩展=%zu，同段强化=%zu(轮次=%zu)，"
    "最终约束=%zu，进度反向局部桥=%zu，尖峰等距回填=%zu/%zu，净距不足=%zu，"
    "非相交冲突=%zu，相交圆公共弦点=%zu，进度反向=%zu，"
    "最大曲率=%.3f。",
    report.primary_constraints, report.secondary_constraints,
    report.promoted_constraints, report.expanded_constraints,
    report.reinforced_constraints, report.repair_rounds,
    report.final_constraints,
    report.local_bridges,
    report.fairing_accepted, report.fairing_rejected,
    report.clearance_failures,
    report.anchor_conflicts, report.overlap_corridor_points,
    report.progress_violations,
    report.maximum_curvature);
}
