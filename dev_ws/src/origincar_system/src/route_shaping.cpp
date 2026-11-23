#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{

struct CircleIntersectionChord
{
  double first_x{0.0};
  double first_y{0.0};
  double second_x{0.0};
  double second_y{0.0};
};

// 计算两个真实规划圆的公共弦端点。半径由调用方直接传入，因此
// circle_envelope模式会自然使用target_clearance + numeric_margin。
// 相离、相切、同心或一圆包含另一圆时没有两个稳定交点，返回false。
bool circle_intersection_chord(
  double first_x, double first_y, double first_radius,
  double second_x, double second_y, double second_radius,
  CircleIntersectionChord & chord)
{
  const double center_x = second_x - first_x;
  const double center_y = second_y - first_y;
  const double center_distance = std::hypot(center_x, center_y);
  if (first_radius <= 1.0e-9 || second_radius <= 1.0e-9 ||
    center_distance <= 1.0e-9 ||
    center_distance + 1.0e-9 >= first_radius + second_radius ||
    center_distance <= std::abs(first_radius - second_radius) + 1.0e-9)
  {
    return false;
  }

  const double along_centers =
    (first_radius * first_radius - second_radius * second_radius +
    center_distance * center_distance) / (2.0 * center_distance);
  const double half_chord_squared =
    first_radius * first_radius - along_centers * along_centers;
  if (half_chord_squared <= 1.0e-12) {
    return false;
  }
  const double half_chord = std::sqrt(half_chord_squared);
  const double unit_x = center_x / center_distance;
  const double unit_y = center_y / center_distance;
  const double chord_center_x = first_x + along_centers * unit_x;
  const double chord_center_y = first_y + along_centers * unit_y;
  const double perpendicular_x = -unit_y;
  const double perpendicular_y = unit_x;
  chord.first_x = chord_center_x + half_chord * perpendicular_x;
  chord.first_y = chord_center_y + half_chord * perpendicular_y;
  chord.second_x = chord_center_x - half_chord * perpendicular_x;
  chord.second_y = chord_center_y - half_chord * perpendicular_y;
  return true;
}

bool normal_line_intersects_chord(
  double base_x, double base_y, double normal_x, double normal_y,
  const CircleIntersectionChord & chord, double & normal_offset,
  double & chord_ratio)
{
  const double chord_x = chord.second_x - chord.first_x;
  const double chord_y = chord.second_y - chord.first_y;
  const double chord_length_squared = chord_x * chord_x + chord_y * chord_y;
  const double denominator = normal_x * chord_y - normal_y * chord_x;
  if (chord_length_squared <= 1.0e-12 || std::abs(denominator) <= 1.0e-9) {
    return false;
  }
  normal_offset =
    ((chord.first_x - base_x) * chord_y -
    (chord.first_y - base_y) * chord_x) / denominator;
  const double intersection_x = base_x + normal_offset * normal_x;
  const double intersection_y = base_y + normal_offset * normal_y;
  chord_ratio =
    ((intersection_x - chord.first_x) * chord_x +
    (intersection_y - chord.first_y) * chord_y) / chord_length_squared;
  return std::isfinite(normal_offset) && std::isfinite(chord_ratio);
}

bool point_near_chord_segment(
  double point_x, double point_y, const CircleIntersectionChord & chord,
  double tolerance)
{
  const double chord_x = chord.second_x - chord.first_x;
  const double chord_y = chord.second_y - chord.first_y;
  const double chord_length_squared = chord_x * chord_x + chord_y * chord_y;
  if (chord_length_squared <= 1.0e-12) {
    return false;
  }
  const double chord_length = std::sqrt(chord_length_squared);
  const double ratio =
    ((point_x - chord.first_x) * chord_x +
    (point_y - chord.first_y) * chord_y) / chord_length_squared;
  const double ratio_tolerance = std::max(0.0, tolerance) / chord_length;
  if (ratio < -ratio_tolerance || ratio > 1.0 + ratio_tolerance) {
    return false;
  }
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  const double closest_x = chord.first_x + clamped_ratio * chord_x;
  const double closest_y = chord.first_y + clamped_ratio * chord_y;
  return std::hypot(point_x - closest_x, point_y - closest_y) <= tolerance;
}

}  // namespace

bool VisualLidarPathFollower::original_route_normal_at(
  std::size_t waypoint_index, VisualLidarPathFollower::Point2D & normal) const
{
  if (original_waypoints_.size() < 2 || waypoint_index >= original_waypoints_.size()) {
    return false;
  }

  const std::size_t previous_index = waypoint_index > 0 ? waypoint_index - 1 : 0;
  const std::size_t next_index = std::min(
    waypoint_index + 1, original_waypoints_.size() - 1);
  const double tangent_x =
    original_waypoints_[next_index].x - original_waypoints_[previous_index].x;
  const double tangent_y =
    original_waypoints_[next_index].y - original_waypoints_[previous_index].y;
  const double tangent_length = std::hypot(tangent_x, tangent_y);
  if (tangent_length < 1e-8) {
    return false;
  }
  normal = Point2D{-tangent_y / tangent_length, tangent_x / tangent_length};
  return true;
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
    waypoint_shift_offsets_.size() != route_size ||
    waypoint_shift_world_directions_.size() != route_size ||
    waypoint_shift_source_indices_.size() != route_size)
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
        const double requested_offset = std::abs(waypoint_shift_offsets_[index]);
        if (requested_offset < 1e-9) {
          continue;
        }
        Point2D requested_direction;
        if (!normalize_vector(
            waypoint_shift_world_directions_[index], requested_direction))
        {
          Point2D normal;
          if (!original_route_normal_at(index, normal)) {
            invalid_reason = "无法计算偏移量验收方向";
            return false;
          }
          const double sign = waypoint_shift_offsets_[index] > 0.0 ? 1.0 : -1.0;
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
    if (std::abs(waypoint_shift_offsets_[run_first]) < 1e-9) {
      ++run_first;
      continue;
    }
    const int offset_sign = waypoint_shift_offsets_[run_first] > 0.0 ? 1 : -1;
    const std::size_t source_identity = waypoint_shift_source_indices_[run_first];
    std::size_t run_last = run_first;
    while (run_last + 1 < route_size &&
      waypoint_shift_offsets_[run_last + 1] * static_cast<double>(offset_sign) > 1e-9)
    {
      const std::size_t next_source = waypoint_shift_source_indices_[run_last + 1];
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
        1.0 - curvature * waypoint_shift_offsets_[index];
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
      Point2D fixed_direction = waypoint_shift_world_directions_[source_index];
      if (!normalize_vector(fixed_direction, fixed_direction)) {
        fixed_direction = Point2D();
        for (std::size_t index = unsafe_first; index <= unsafe_last; ++index) {
          fixed_direction.x += waypoint_shift_world_directions_[index].x;
          fixed_direction.y += waypoint_shift_world_directions_[index].y;
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
          target_offset, std::abs(waypoint_shift_offsets_[index]));
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
            accepted_clearance = std::abs(waypoint_shift_offsets_[source_index]);
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
  if (original_waypoints_.size() != waypoint_shift_offsets_.size()) {
    return;
  }
  waypoints_ = original_waypoints_;
  if (route_deformation_mode_ == "circle_envelope") {
    apply_circle_envelope_deformation(waypoints_);
  } else {
    for (std::size_t index = 0; index < waypoints_.size(); ++index) {
      if (std::abs(waypoint_shift_offsets_[index]) < 1e-9) {
        continue;
      }
      Point2D normal;
      if (!original_route_normal_at(index, normal)) {
        continue;
      }
      waypoints_[index].x += waypoint_shift_offsets_[index] * normal.x;
      waypoints_[index].y += waypoint_shift_offsets_[index] * normal.y;
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
    waypoint_shift_offsets_.size() != waypoints_.size() ||
    waypoint_shift_obstacle_centers_.size() != waypoints_.size() ||
    waypoint_shift_forbidden_radii_.size() != waypoints_.size())
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
    if (std::abs(waypoint_shift_offsets_[index]) < 1.0e-9 ||
      waypoint_shift_forbidden_radii_[index] <= 1.0e-9)
    {
      continue;
    }
    const auto & center = waypoint_shift_obstacle_centers_[index];
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
          center, waypoint_shift_forbidden_radii_[index], first_segment});
    } else {
      existing->forbidden_radius = std::max(
        existing->forbidden_radius,
        waypoint_shift_forbidden_radii_[index]);
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
  const std::size_t route_size = original_waypoints_.size();
  if (route_size < 3 || shifted_route.size() != route_size ||
    waypoint_shift_offsets_.size() != route_size ||
    waypoint_shift_world_directions_.size() != route_size ||
    waypoint_shift_source_indices_.size() != route_size ||
    waypoint_shift_obstacle_centers_.size() != route_size ||
    waypoint_shift_forbidden_radii_.size() != route_size)
  {
    return;
  }

  // 每次静态路线重算都重新验收全部永久点。最终仍不合格的ID会在本函数
  // 末尾单独标记并临时解除雷达掩码，不能沿用上一版路线的失败状态。
  logic3_green_route_clearance_failed_.assign(
    logic3_green_clusters_.size(), false);

  std::vector<double> route_s(route_size, 0.0);
  std::vector<Point2D> left_normals(route_size, Point2D());
  std::vector<bool> normal_valid(route_size, false);
  for (std::size_t index = 1; index < route_size; ++index) {
    route_s[index] = route_s[index - 1] + std::hypot(
      original_waypoints_[index].x - original_waypoints_[index - 1].x,
      original_waypoints_[index].y - original_waypoints_[index - 1].y);
  }
  for (std::size_t index = 0; index < route_size; ++index) {
    normal_valid[index] = original_route_normal_at(index, left_normals[index]);
  }

  // A→C→B已经在original_waypoints_中接到Task2首点B之前。静态偏移必须
  // 把这条组合路线视为一个整体，允许B附近障碍的before窗口自然跨过B，
  // 不能因为过渡段同时启用实时雷达就把A→C→B排除在可偏移范围之外。
  const std::size_t first_deformable_index = 0;
  const std::size_t last_deformable_index = route_size - 1;
  if (first_deformable_index > last_deformable_index) {
    return;
  }

  // 仅二维码倒车完成后的A->C->B重建会启用本过滤：用当时A沿yaw前移
  // 配置距离后的虚拟参考点和原yaw冻结一次车前/车后关系。车后永久点仍可
  // 作用于起始保护长度之后的路线，但circle_envelope接线和二次净距修复
  // 不得越过同一轮保存的最早可偏移索引，避免倒灌到A点附近。
  auto obstacle_first_deformable_index = [&](const Point2D & center) {
      if (!channel_entry_rear_static_filter_active_) {
        return first_deformable_index;
      }
      const double relative_x =
        center.x - channel_entry_rear_static_filter_start_x_;
      const double relative_y =
        center.y - channel_entry_rear_static_filter_start_y_;
      const double forward_projection =
        relative_x * std::cos(channel_entry_rear_static_filter_start_yaw_) +
        relative_y * std::sin(channel_entry_rear_static_filter_start_yaw_);
      if (forward_projection >= 0.0) {
        return first_deformable_index;
      }
      return std::clamp(
        channel_entry_rear_static_filter_first_index_,
        first_deformable_index, last_deformable_index);
    };

  struct CircleConstraint
  {
    std::size_t source{0};
    int direction{0};
    Point2D center;
    double radius{0.0};
    double peak_offset{0.0};
    double transition_extension{0.0};
    // 第一次路线生成完后，若某个此前未覆盖的永久绿/橙点被新路线侵入，
    // 只允许在真正侵圆的连续索引段使用“原路线法线与圆求交”补充边界。
    // 普通约束仍坚持原航点本身必须在圆内，不能退回无限长法线全局约束。
    bool secondary_repair{false};
    std::size_t repair_first{0};
    std::size_t repair_last{0};
    // 单个障碍允许影响的最早路线索引。通常为0；本轮入口A点车后的
    // 永久点从A后的保护长度边界开始，防止圆滑接线越过车前/车后过滤边界。
    std::size_t first_deformable{0};
    // 正反两次经过同一物理通道时，整个圆滑接入区间都必须保持已经选定的
    // 路线相对侧。只锁圆心附近的硬边界会让其他障碍的长接线尾巴在水平
    // 转弯段把符号平均翻转，出现“竖直段右绕、水平段反而向上”的形状。
    bool transition_side_locked{false};
  };
  std::vector<CircleConstraint> constraints;
  for (std::size_t index = 0; index < route_size; ++index) {
    if (std::abs(waypoint_shift_offsets_[index]) < 1.0e-9 ||
      waypoint_shift_forbidden_radii_[index] <= 1.0e-9 ||
      waypoint_shift_source_indices_[index] >= route_size)
    {
      continue;
    }

    const std::size_t source = waypoint_shift_source_indices_[index];
    if (source < first_deformable_index || source > last_deformable_index ||
      !normal_valid[source])
    {
      continue;
    }

    // 优先使用障碍物投影到原路线时保存的世界坐标绕行方向，再换算成
    // source处原路线左法向的正负号。后续任何圆滑和净距修正都不得改号。
    int direction = waypoint_shift_offsets_[index] > 0.0 ? 1 : -1;
    const auto & world_direction = waypoint_shift_world_directions_[index];
    const double world_direction_length =
      std::hypot(world_direction.x, world_direction.y);
    if (world_direction_length > 1.0e-9) {
      const double alignment =
        (world_direction.x * left_normals[source].x +
        world_direction.y * left_normals[source].y) / world_direction_length;
      if (std::abs(alignment) > 0.2) {
        direction = alignment > 0.0 ? 1 : -1;
      }
    }
    const auto & center = waypoint_shift_obstacle_centers_[index];
    const std::size_t constraint_first_deformable =
      obstacle_first_deformable_index(center);
    if (source < constraint_first_deformable) {
      continue;
    }
    auto existing = std::find_if(
      constraints.begin(), constraints.end(),
      [&](const CircleConstraint & candidate) {
        // 这里只去除同一障碍物、同一路线锚点的重复元数据；不同中心或
        // 不同路线经过位置始终保留为独立约束，绝不合并相邻障碍物。
        return candidate.source == source &&
               std::hypot(
          candidate.center.x - center.x,
          candidate.center.y - center.y) <= 0.02;
      });
    if (existing == constraints.end()) {
      constraints.push_back(
        CircleConstraint{
          source, direction, center,
          waypoint_shift_forbidden_radii_[index],
          std::abs(waypoint_shift_offsets_[index]), 0.0,
          false, 0, 0, constraint_first_deformable, false});
    } else {
      existing->radius = std::max(
        existing->radius, waypoint_shift_forbidden_radii_[index]);
      existing->peak_offset = std::max(
        existing->peak_offset, std::abs(waypoint_shift_offsets_[index]));
      existing->first_deformable = std::max(
        existing->first_deformable, constraint_first_deformable);
    }
  }
  if (constraints.empty()) {
    return;
  }
  std::sort(
    constraints.begin(), constraints.end(),
    [](const CircleConstraint & first, const CircleConstraint & second) {
      if (first.source != second.source) {
        return first.source < second.source;
      }
      if (first.center.x != second.center.x) {
        return first.center.x < second.center.x;
      }
      return first.center.y < second.center.y;
    });

  // 同一物理通道可能在路线序列中正向、反向各经过一次。同一个障碍会因此
  // 生成两个source相距很远、但世界坐标几乎重合且法向相反的独立约束。
  // 两次经过仍各自在完整路线s轴上连续画接线，只统一障碍的世界绕行侧、
  // 半径和偏移峰值；不能在偏移完成后强制同步坐标，否则会在同步边界造尖峰。
  // 这里只配对“同中心 + 同位置 + 反向共线”的重复经过；不同障碍、普通
  // 交叉点以及同方向近邻路线都不参与，仍保持原来的独立约束语义。
  struct SharedPassGeometry
  {
    std::size_t first_source{0};
    std::size_t second_source{0};
    Point2D center;
    int first_direction{0};
    int second_direction{0};
    std::vector<std::pair<std::size_t, std::size_t>> waypoint_pairs;
  };
  std::vector<SharedPassGeometry> shared_pass_geometries;
  std::size_t shared_pass_waypoint_pair_count = 0;
  constexpr double shared_center_tolerance = 0.03;
  constexpr double shared_source_tolerance = 0.05;
  constexpr double shared_waypoint_tolerance = 0.015;
  constexpr double minimum_shared_pass_length = 0.20;
  constexpr double opposite_normal_cosine = -0.80;
  for (std::size_t first_index = 0;
    first_index < constraints.size(); ++first_index)
  {
    for (std::size_t second_index = first_index + 1;
      second_index < constraints.size(); ++second_index)
    {
      auto & first = constraints[first_index];
      auto & second = constraints[second_index];
      if (std::hypot(
          first.center.x - second.center.x,
          first.center.y - second.center.y) > shared_center_tolerance)
      {
        continue;
      }
      const auto & first_source = original_waypoints_[first.source];
      const auto & second_source = original_waypoints_[second.source];
      if (std::hypot(
          first_source.x - second_source.x,
          first_source.y - second_source.y) > shared_source_tolerance)
      {
        continue;
      }
      const double normal_alignment =
        left_normals[first.source].x * left_normals[second.source].x +
        left_normals[first.source].y * left_normals[second.source].y;
      if (normal_alignment > opposite_normal_cosine) {
        continue;
      }

      std::vector<std::pair<std::size_t, std::size_t>> shared_waypoint_pairs;
      shared_waypoint_pairs.emplace_back(first.source, second.source);
      for (std::size_t step = 1;
        first.source + step < route_size && second.source >= step; ++step)
      {
        const std::size_t first_waypoint = first.source + step;
        const std::size_t second_waypoint = second.source - step;
        if (std::hypot(
            original_waypoints_[first_waypoint].x -
            original_waypoints_[second_waypoint].x,
            original_waypoints_[first_waypoint].y -
            original_waypoints_[second_waypoint].y) > shared_waypoint_tolerance)
        {
          break;
        }
        shared_waypoint_pairs.emplace_back(first_waypoint, second_waypoint);
      }
      for (std::size_t step = 1;
        first.source >= step && second.source + step < route_size; ++step)
      {
        const std::size_t first_waypoint = first.source - step;
        const std::size_t second_waypoint = second.source + step;
        if (std::hypot(
            original_waypoints_[first_waypoint].x -
            original_waypoints_[second_waypoint].x,
            original_waypoints_[first_waypoint].y -
            original_waypoints_[second_waypoint].y) > shared_waypoint_tolerance)
        {
          break;
        }
        shared_waypoint_pairs.emplace_back(first_waypoint, second_waypoint);
      }
      // 单点交叉或B/C附近很短的相向接线不属于共用通道。先保留最少
      // 3个连续匹配点的离散门槛，下面还会用实际弧长和两段各自已经
      // 判定的世界绕行方向复核，防止入口段把返回段方向覆盖掉。
      if (shared_waypoint_pairs.size() < 3) {
        continue;
      }

      std::sort(
        shared_waypoint_pairs.begin(), shared_waypoint_pairs.end(),
        [](const auto & lhs, const auto & rhs) {
          return lhs.first < rhs.first;
        });
      shared_waypoint_pairs.erase(
        std::unique(
          shared_waypoint_pairs.begin(), shared_waypoint_pairs.end()),
        shared_waypoint_pairs.end());

      const double first_shared_length =
        route_s[shared_waypoint_pairs.back().first] -
        route_s[shared_waypoint_pairs.front().first];
      std::size_t minimum_second_waypoint = route_size - 1;
      std::size_t maximum_second_waypoint = 0;
      for (const auto & pair : shared_waypoint_pairs) {
        minimum_second_waypoint = std::min(
          minimum_second_waypoint, pair.second);
        maximum_second_waypoint = std::max(
          maximum_second_waypoint, pair.second);
      }
      const double second_shared_length =
        route_s[maximum_second_waypoint] - route_s[minimum_second_waypoint];
      if (first_shared_length + 1.0e-9 < minimum_shared_pass_length ||
        second_shared_length + 1.0e-9 < minimum_shared_pass_length)
      {
        continue;
      }

      const Point2D first_world_direction{
        static_cast<double>(first.direction) * left_normals[first.source].x,
        static_cast<double>(first.direction) * left_normals[first.source].y};
      // “同中心、同世界位置、反向共线且共享至少0.20m”的几何已经足以
      // 证明这是同一物理通道。中心几乎压在线上时，两次独立判边可能因
      // 毫米级误差得出相反结果；不能再把这个相反结果当成拒绝配对的理由。
      // 使用路线序列中较早一遍已经确定的世界侧作为本次发车的规范方向，
      // 另一遍换算成自己的局部法向符号。这样既不复制最终坐标，也不会
      // 出现一遍右绕、一遍穿心或左绕。
      const double second_alignment =
        first_world_direction.x * left_normals[second.source].x +
        first_world_direction.y * left_normals[second.source].y;
      second.direction = second_alignment >= 0.0 ? 1 : -1;
      first.transition_side_locked = true;
      second.transition_side_locked = true;
      const double shared_radius = std::max(first.radius, second.radius);
      const double shared_peak = std::max(first.peak_offset, second.peak_offset);
      first.radius = shared_radius;
      second.radius = shared_radius;
      first.peak_offset = shared_peak;
      second.peak_offset = shared_peak;
      const auto duplicate_geometry = std::find_if(
        shared_pass_geometries.begin(), shared_pass_geometries.end(),
        [&](const SharedPassGeometry & geometry) {
          return geometry.first_source == first.source &&
                 geometry.second_source == second.source;
        });
      if (duplicate_geometry == shared_pass_geometries.end()) {
        shared_pass_waypoint_pair_count += shared_waypoint_pairs.size();
        shared_pass_geometries.push_back(
          SharedPassGeometry{
            first.source, second.source, first.center,
            first.direction, second.direction,
            std::move(shared_waypoint_pairs)});
      }
    }
  }

  // 二次净距审计可能在上面的首次配对之后才新增或升级同一永久点的约束。
  // 每轮重算前都用已确认的共享通道世界侧重新换算所有同中心约束，保证
  // 新增约束不会再次按接近零的横向量独立选到相反侧。
  auto apply_shared_pass_side_locks = [&](std::vector<CircleConstraint> & active) {
      for (auto & constraint : active) {
        for (const auto & shared : shared_pass_geometries) {
          if (std::hypot(
              constraint.center.x - shared.center.x,
              constraint.center.y - shared.center.y) > shared_center_tolerance ||
            constraint.source >= route_size || !normal_valid[constraint.source])
          {
            continue;
          }
          const double first_distance = std::abs(
            route_s[constraint.source] - route_s[shared.first_source]);
          const double second_distance = std::abs(
            route_s[constraint.source] - route_s[shared.second_source]);
          // 同一障碍还可能在转弯肩部新增二次约束。肩部法向已经旋转，
          // 与固定世界方向可能正交，不能因此丢掉侧向锁；按完整路线s轴
          // 归到更近的那次经过，并继承该次经过的有符号偏移方向。
          if (std::min(first_distance, second_distance) >
            circle_envelope_max_transition_length_ +
            circle_envelope_extension_step_)
          {
            continue;
          }
          constraint.direction = first_distance <= second_distance ?
            shared.first_direction : shared.second_direction;
          constraint.transition_side_locked = true;
          break;
        }
      }
    };
  apply_shared_pass_side_locks(constraints);

  // 先在每个独立锚点上求自己的最低有符号偏移量。这里读取的几何始终
  // 是original_waypoints_，不会拿前一个障碍物已经偏移后的路线判断后一个。
  for (auto & constraint : constraints) {
    const auto & base = original_waypoints_[constraint.source];
    const auto & normal = left_normals[constraint.source];
    const double qx = base.x - constraint.center.x;
    const double qy = base.y - constraint.center.y;
    const double along_normal = qx * normal.x + qy * normal.y;
    const double perpendicular_squared = std::max(
      0.0, qx * qx + qy * qy - along_normal * along_normal);
    const double discriminant =
      constraint.radius * constraint.radius - perpendicular_squared;
    if (discriminant >= 0.0) {
      const double root = std::sqrt(discriminant);
      const double signed_boundary = -along_normal +
        static_cast<double>(constraint.direction) * root;
      constraint.peak_offset = std::max(
        constraint.peak_offset,
        std::max(0.0, static_cast<double>(constraint.direction) * signed_boundary));
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
      const double ratio = std::clamp(
        ((point.x - start.x) * segment_x +
        (point.y - start.y) * segment_y) / length_squared,
        0.0, 1.0);
      return Point2D{
        start.x + ratio * segment_x,
        start.y + ratio * segment_y};
    };

  auto build_candidate = [&](const std::vector<CircleConstraint> & active_constraints,
      double common_extension, std::vector<Waypoint> & candidate,
      std::size_t & constraint_conflicts,
      std::size_t & overlap_corridor_points,
      std::size_t & local_bridge_count) {
      struct ConstraintProfile
      {
        std::size_t owner{0};
        std::size_t source{0};
        std::size_t first_deformable{0};
        double entry_s{0.0};
        double center_s{0.0};
        double exit_s{0.0};
        double signed_peak{0.0};
      };
      std::vector<ConstraintProfile> profiles;
      profiles.reserve(active_constraints.size());
      // 以原来的逐障碍smoothstep偏移作为默认路线。它的接入距离长、
      // 不会为了贴住每个安全圆而强制经过圆边界，因此在普通弯道和
      // 单障碍场景下通常比连续锚点曲线更自然。后面只对它真正出现
      // 进度反向的小区间做局部Bezier修复。
      std::vector<double> weighted_offsets(route_size, 0.0);
      std::vector<double> weights(route_size, 0.0);
      std::vector<double> lower_bounds(
        route_size, -std::numeric_limits<double>::infinity());
      std::vector<double> upper_bounds(
        route_size, std::numeric_limits<double>::infinity());
      const std::size_t no_constraint = active_constraints.size();
      std::vector<std::size_t> lower_owners(route_size, no_constraint);
      std::vector<std::size_t> upper_owners(route_size, no_constraint);

      for (std::size_t index = 0; index < route_size; ++index) {
        if (!normal_valid[index] || index > last_deformable_index)
        {
          lower_bounds[index] = 0.0;
          upper_bounds[index] = 0.0;
          continue;
        }
        if (following_qr_path_ || task_phase_ == TaskPhase::TASK1) {
          const auto & base = original_waypoints_[index];
          const auto & normal = left_normals[index];
          const double minimum_y_offset = task1_waypoint_shift_min_y_ - base.y;
          if (normal.y > 1.0e-9) {
            lower_bounds[index] = std::max(
              lower_bounds[index], minimum_y_offset / normal.y);
          } else if (normal.y < -1.0e-9) {
            upper_bounds[index] = std::min(
              upper_bounds[index], minimum_y_offset / normal.y);
          } else if (base.y < task1_waypoint_shift_min_y_) {
            lower_bounds[index] = 0.0;
            upper_bounds[index] = 0.0;
          }
        }
      }

      auto circle_boundary_offset = [&](std::size_t index,
          const CircleConstraint & constraint, double & boundary) {
          if (!normal_valid[index]) {
            return false;
          }
          const auto & base = original_waypoints_[index];
          const auto & normal = left_normals[index];
          const double center_x = constraint.center.x - base.x;
          const double center_y = constraint.center.y - base.y;
          const double center_normal =
            center_x * normal.x + center_y * normal.y;
          const double tangent_squared = std::max(
            0.0,
            center_x * center_x + center_y * center_y -
            center_normal * center_normal);
          const double radius_squared =
            constraint.radius * constraint.radius;
          const double center_distance_squared =
            center_x * center_x + center_y * center_y;
          const bool in_secondary_repair_core =
            constraint.secondary_repair &&
            index >= constraint.repair_first &&
            index <= constraint.repair_last;
          // 第一次生成仍只替换原路线真正落入安全圆的连续点段，避免
          // “无限长法线穿圆”让远处弯道受到假约束。唯一例外是二次审计
          // 已经用第一次生成后的有限路线段证实侵圆；此时也只在那一小段
          // 索引内用法线求圆边界，绝不把约束扩散到整条路线。
          if ((!in_secondary_repair_core &&
            center_distance_squared > radius_squared) ||
            tangent_squared > radius_squared)
          {
            return false;
          }
          const double half_chord = std::sqrt(std::max(
            0.0, radius_squared - tangent_squared));
          // d是原路线左法向的偏移量。圆在该法线上占据
          // [center_normal-half_chord, center_normal+half_chord]。
          boundary = constraint.direction > 0 ?
            std::max(0.0, center_normal + half_chord) :
            std::min(0.0, center_normal - half_chord);
          return true;
        };

      auto smoothstep_at = [&](double current_s, double center_s,
          double entry_s, double exit_s) {
          double progress = 1.0;
          if (current_s < center_s) {
            const double length = center_s - entry_s;
            progress = length > 1.0e-9 ?
              (current_s - entry_s) / length : 1.0;
          } else if (current_s > center_s) {
            const double length = exit_s - center_s;
            progress = length > 1.0e-9 ?
              (exit_s - current_s) / length : 1.0;
          }
          progress = std::clamp(progress, 0.0, 1.0);
          return progress * progress * progress *
                 (10.0 + progress * (-15.0 + 6.0 * progress));
        };

      for (std::size_t constraint_index = 0;
        constraint_index < active_constraints.size(); ++constraint_index)
      {
        const auto & constraint = active_constraints[constraint_index];
        const std::size_t constraint_first_deformable = std::clamp(
          constraint.first_deformable,
          first_deformable_index, last_deformable_index);
        auto transition_length_for_peak = [&](double peak) {
            const double curvature_transition = std::sqrt(
              6.0 * std::max(0.001, peak) /
              circle_envelope_max_curvature_);
            return std::clamp(
              std::max(
                circle_envelope_min_transition_length_ +
                0.5 * circle_envelope_merge_gap_, curvature_transition) +
              common_extension + constraint.transition_extension,
              circle_envelope_min_transition_length_,
              circle_envelope_max_transition_length_);
          };

        const double maximum_reasonable_peak =
          waypoint_cloud_check_radius_ + cone_circle_max_radius_ +
          circle_envelope_target_clearance_ + circle_envelope_numeric_margin_;
        double effective_peak = std::min(
          constraint.peak_offset, maximum_reasonable_peak);
        double transition_length = transition_length_for_peak(effective_peak);
        const double center_s = route_s[constraint.source];
        double entry_s = std::max(
          center_s - transition_length,
          route_s[constraint_first_deformable]);
        double exit_s = std::min(
          center_s + transition_length, route_s[last_deformable_index]);

        // 峰值只取安全圆几何上真正需要的最大偏移。不得用
        // boundary/smoothstep反推峰值：接线边缘的smoothstep接近0，
        // 这种除法会把整条路线放大到地图外。圆滑后的局部净距
        // 由下面整段circle boundary约束保证。
        for (std::size_t index = constraint_first_deformable;
          index <= last_deformable_index; ++index)
        {
          if (route_s[index] < entry_s || route_s[index] > exit_s) {
            continue;
          }
          double boundary = 0.0;
          if (!circle_boundary_offset(index, constraint, boundary)) {
            continue;
          }
          effective_peak = std::min(
            maximum_reasonable_peak,
            std::max(effective_peak, std::abs(boundary)));
        }

        transition_length = transition_length_for_peak(effective_peak);
        entry_s = std::max(
          center_s - transition_length,
          route_s[constraint_first_deformable]);
        exit_s = std::min(
          center_s + transition_length, route_s[last_deformable_index]);
        profiles.push_back(
          ConstraintProfile{
            constraint_index, constraint.source, constraint_first_deformable,
            entry_s, center_s, exit_s,
            static_cast<double>(constraint.direction) * effective_peak});
        for (std::size_t index = constraint_first_deformable;
          index <= last_deformable_index; ++index)
        {
          if (!normal_valid[index] ||
            route_s[index] < entry_s || route_s[index] > exit_s)
          {
            continue;
          }
          const double smoothstep = smoothstep_at(
            route_s[index], center_s, entry_s, exit_s);
          const double proposal =
            static_cast<double>(constraint.direction) *
            effective_peak * smoothstep;
          weighted_offsets[index] += smoothstep * proposal;
          weights[index] += smoothstep;

          // 共享通道的正反两遍已选定同一世界绕行侧。锁定整个smoothstep
          // 接入区间的符号，而不只是安全圆核心：以本次顺时针返程为例，
          // 同一个正偏移在向左水平段表示向下，在随后向下竖直段表示向右，
          // 正好形成连续的“水平向下→竖直向右”路线。其他障碍接线只允许
          // 改变偏移大小，不能通过加权平均把它翻到原路线另一侧。
          if (constraint.transition_side_locked) {
            if (constraint.direction > 0 && lower_bounds[index] < 0.0) {
              lower_bounds[index] = 0.0;
              lower_owners[index] = constraint_index;
            } else if (constraint.direction < 0 && upper_bounds[index] > 0.0) {
              upper_bounds[index] = 0.0;
              upper_owners[index] = constraint_index;
            }
          }

          double boundary = 0.0;
          if (!circle_boundary_offset(index, constraint, boundary)) {
            continue;
          }
          if (constraint.direction > 0 && boundary > lower_bounds[index]) {
            lower_bounds[index] = boundary;
            lower_owners[index] = constraint_index;
          } else if (constraint.direction < 0 &&
            boundary < upper_bounds[index])
          {
            upper_bounds[index] = boundary;
            upper_owners[index] = constraint_index;
          }
        }
      }

      std::vector<double> offsets(route_size, 0.0);
      for (std::size_t index = 0; index < route_size; ++index) {
        if (weights[index] > 1.0e-12) {
          offsets[index] = weighted_offsets[index] / weights[index];
        }
      }
      local_bridge_count = 0;

      struct ActiveOverlapChord
      {
        std::size_t lower_owner{0};
        std::size_t upper_owner{0};
        std::size_t core_first{0};
        std::size_t core_last{0};
        CircleIntersectionChord chord;
      };
      std::vector<ActiveOverlapChord> active_overlap_chords;
      std::vector<bool> overlap_corridor(route_size, false);

      // 两个方向相反的规划圆发生冲突时，不再逐航点平均上下边界。
      // 平均值依赖每个航点自己的原路线法线，生成的世界坐标并不共线，
      // 后续一维平滑便会在重叠区形成不规则折弯。这里直接使用实际规划
      // 半径（target_clearance + numeric_margin）计算两个交点，并把局部
      // 冲突段锁到两个交点之间的公共弦上。
      std::size_t conflict_index = 0;
      while (conflict_index < route_size) {
        if (lower_bounds[conflict_index] <= upper_bounds[conflict_index] ||
          lower_owners[conflict_index] == no_constraint ||
          upper_owners[conflict_index] == no_constraint)
        {
          ++conflict_index;
          continue;
        }

        const std::size_t lower_owner = lower_owners[conflict_index];
        const std::size_t upper_owner = upper_owners[conflict_index];
        const auto & lower_circle = active_constraints[lower_owner];
        const auto & upper_circle = active_constraints[upper_owner];
        std::size_t run_last = conflict_index;
        while (run_last + 1 < route_size &&
          lower_bounds[run_last + 1] > upper_bounds[run_last + 1] &&
          lower_owners[run_last + 1] == lower_owner &&
          upper_owners[run_last + 1] == upper_owner)
        {
          ++run_last;
        }

        CircleIntersectionChord chord;
        if (!circle_intersection_chord(
            lower_circle.center.x, lower_circle.center.y, lower_circle.radius,
            upper_circle.center.x, upper_circle.center.y, upper_circle.radius,
            chord))
        {
          // 同心、内含或数值上接近相切时没有稳定的两个交点。保留旧的
          // 中值退化处理，避免无交点几何把路线留在互相矛盾的硬边界中。
          const double center_distance = std::hypot(
            lower_circle.center.x - upper_circle.center.x,
            lower_circle.center.y - upper_circle.center.y);
          if (center_distance + 1.0e-9 <
            lower_circle.radius + upper_circle.radius)
          {
            for (std::size_t index = conflict_index;
              index <= run_last; ++index)
            {
              const double corridor_offset =
                0.5 * (lower_bounds[index] + upper_bounds[index]);
              lower_bounds[index] = corridor_offset;
              upper_bounds[index] = corridor_offset;
              overlap_corridor[index] = true;
            }
          }
          conflict_index = run_last + 1;
          continue;
        }

        auto chord_offset_at = [&](std::size_t index, bool require_segment,
            double & chord_offset) {
            if (index >= route_size || !normal_valid[index]) {
              return false;
            }
            double chord_ratio = 0.0;
            const auto & base = original_waypoints_[index];
            const auto & normal = left_normals[index];
            if (!normal_line_intersects_chord(
                base.x, base.y, normal.x, normal.y,
                chord, chord_offset, chord_ratio))
            {
              return false;
            }
            return !require_segment ||
                   (chord_ratio >= -1.0e-6 && chord_ratio <= 1.0 + 1.0e-6);
          };

        // 先确认冲突段确实能由原路线各点的局部法线投影到公共弦。
        // 若极端几何下法线与弦平行，则只对该段使用中值退化，不把约束
        // 扩散到路线其他位置。
        std::size_t first_chord_index = route_size;
        std::size_t last_chord_index = 0;
        for (std::size_t index = conflict_index; index <= run_last; ++index) {
          double chord_offset = 0.0;
          if (chord_offset_at(index, true, chord_offset)) {
            first_chord_index = std::min(first_chord_index, index);
            last_chord_index = std::max(last_chord_index, index);
          }
        }
        if (first_chord_index == route_size) {
          for (std::size_t index = conflict_index; index <= run_last; ++index) {
            const double corridor_offset =
              0.5 * (lower_bounds[index] + upper_bounds[index]);
            lower_bounds[index] = corridor_offset;
            upper_bounds[index] = corridor_offset;
            overlap_corridor[index] = true;
          }
          conflict_index = run_last + 1;
          continue;
        }

        const std::size_t corridor_first_deformable = std::max(
          lower_circle.first_deformable, upper_circle.first_deformable);
        while (first_chord_index > corridor_first_deformable) {
          double chord_offset = 0.0;
          if (!chord_offset_at(first_chord_index - 1, true, chord_offset)) {
            break;
          }
          --first_chord_index;
        }
        while (last_chord_index < last_deformable_index) {
          double chord_offset = 0.0;
          if (!chord_offset_at(last_chord_index + 1, true, chord_offset)) {
            break;
          }
          ++last_chord_index;
        }

        for (std::size_t index = first_chord_index;
          index <= last_chord_index; ++index)
        {
          double chord_offset = 0.0;
          if (!chord_offset_at(index, true, chord_offset)) {
            continue;
          }
          const bool foreign_lower_bound =
            std::isfinite(lower_bounds[index]) &&
            lower_owners[index] != lower_owner &&
            lower_owners[index] != upper_owner;
          const bool foreign_upper_bound =
            std::isfinite(upper_bounds[index]) &&
            upper_owners[index] != lower_owner &&
            upper_owners[index] != upper_owner;
          if ((foreign_lower_bound &&
            chord_offset + 1.0e-9 < lower_bounds[index]) ||
            (foreign_upper_bound &&
            chord_offset - 1.0e-9 > upper_bounds[index]))
          {
            // Task1最低Y或第三个独立圆仍然是硬约束；两圆公共弦不能
            // 静默覆盖它。保留原边界，让质量检查把真正的多圆无解记录出来。
            continue;
          }
          lower_bounds[index] = chord_offset;
          upper_bounds[index] = chord_offset;
          overlap_corridor[index] = true;
        }
        active_overlap_chords.push_back(
          ActiveOverlapChord{
            lower_owner, upper_owner,
            first_chord_index, last_chord_index, chord});
        conflict_index = run_last + 1;
      }

      // 公共弦核心是精确硬约束；两端则沿公共弦延长线用五次smoothstep
      // 逐渐混回原来的整体偏移场。这样交点之间保持严格共线，接入处的
      // 一、二阶变化同时趋零，随后十轮全局平滑只需消除离散采样误差。
      const double chord_shoulder_length = std::max(
        circle_envelope_merge_gap_,
        0.5 * (circle_envelope_target_clearance_ +
        circle_envelope_numeric_margin_));
      const double maximum_reasonable_chord_offset =
        waypoint_cloud_check_radius_ + cone_circle_max_radius_ +
        circle_envelope_target_clearance_ + circle_envelope_numeric_margin_;
      auto quintic_blend = [](double progress) {
          const double t = std::clamp(progress, 0.0, 1.0);
          return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
      };
      for (const auto & corridor : active_overlap_chords) {
        const std::size_t corridor_first_deformable = std::max(
          active_constraints[corridor.lower_owner].first_deformable,
          active_constraints[corridor.upper_owner].first_deformable);
        auto blend_to_chord = [&](std::size_t index, double blend) {
            if (overlap_corridor[index] || !normal_valid[index]) {
              return;
            }
            double chord_offset = 0.0;
            double chord_ratio = 0.0;
            const auto & base = original_waypoints_[index];
            const auto & normal = left_normals[index];
            if (!normal_line_intersects_chord(
                base.x, base.y, normal.x, normal.y,
                corridor.chord, chord_offset, chord_ratio) ||
              std::abs(chord_offset) > maximum_reasonable_chord_offset)
            {
              return;
            }
            offsets[index] =
              (1.0 - blend) * offsets[index] + blend * chord_offset;
          };

        for (std::size_t index = corridor.core_first;
          index > corridor_first_deformable; )
        {
          --index;
          const double distance_to_core =
            route_s[corridor.core_first] - route_s[index];
          if (distance_to_core > chord_shoulder_length) {
            break;
          }
          blend_to_chord(
            index,
            quintic_blend(1.0 - distance_to_core / chord_shoulder_length));
        }
        for (std::size_t index = corridor.core_last + 1;
          index <= last_deformable_index; ++index)
        {
          const double distance_to_core =
            route_s[index] - route_s[corridor.core_last];
          if (distance_to_core > chord_shoulder_length) {
            break;
          }
          blend_to_chord(
            index,
            quintic_blend(1.0 - distance_to_core / chord_shoulder_length));
        }
      }
      overlap_corridor_points = static_cast<std::size_t>(std::count(
        overlap_corridor.begin(), overlap_corridor.end(), true));

      auto enforce_geometric_bounds = [&]() {
          std::size_t conflicts = 0;
          for (std::size_t index = 0; index < route_size; ++index) {
            if (lower_bounds[index] <= upper_bounds[index]) {
              offsets[index] = std::clamp(
                offsets[index], lower_bounds[index], upper_bounds[index]);
              continue;
            }
            // 非相交圆仍出现相反约束才到这里。不合并、不换向，
            // 保留离整体圆滑结果更近的一侧并详细记录。
            ++conflicts;
            offsets[index] =
              std::abs(offsets[index] - lower_bounds[index]) <=
              std::abs(offsets[index] - upper_bounds[index]) ?
              lower_bounds[index] : upper_bounds[index];
          }
          return conflicts;
        };

      constraint_conflicts = enforce_geometric_bounds();
      for (int iteration = 0;
        iteration < circle_envelope_smoothing_iterations_; ++iteration)
      {
        std::vector<double> smoothed = offsets;
        for (std::size_t index = 1; index + 1 < route_size; ++index) {
          smoothed[index] =
            0.25 * offsets[index - 1] +
            0.50 * offsets[index] +
            0.25 * offsets[index + 1];
        }
        offsets = std::move(smoothed);
        constraint_conflicts = std::max(
          constraint_conflicts, enforce_geometric_bounds());
      }

      candidate = original_waypoints_;
      for (std::size_t index = 0; index < route_size; ++index) {
        if (!normal_valid[index] || std::abs(offsets[index]) < 1.0e-12) {
          continue;
        }
        candidate[index].x += offsets[index] * left_normals[index].x;
        candidate[index].y += offsets[index] * left_normals[index].y;
      }

      // 相交安全圆允许路线沿公共弦附近的窄通道通过。这个例外只服务于
      // 后面的局部折返桥校验：桥上的点必须属于产生冲突的那一对圆，且
      // 仍贴近它们的有限公共弦，不能借“圆相交”放宽到整片区域。
      auto overlap_bridge_allows = [&](const Point2D & point,
          std::size_t constraint_owner) {
          const double corridor_tolerance = std::max(
            0.03, circle_envelope_merge_gap_);
          for (const auto & corridor : active_overlap_chords) {
            if (constraint_owner != corridor.lower_owner &&
              constraint_owner != corridor.upper_owner)
            {
              continue;
            }
            if (point_near_chord_segment(
                point.x, point.y, corridor.chord, corridor_tolerance))
            {
              return true;
            }
          }
          return false;
        };

      // 原路线在C/B一类高曲率接缝附近转向较快时，即使每个点都沿自己的
      // 法线落在安全圆外，也可能因为相邻法线交叉而把航点次序反过来，
      // 表现为小回环或尖峰。这里不重新选障碍方向，也不合并圆：只找最终
      // 橙色基线中真正出现进度反向或近停滞的最小连续段，使用更长的三次Bezier
      // 桥接两端。相交圆公共弦不再成为“禁止修复区”；桥可以沿公共弦的
      // 窄通道通过，但仍必须单向前进且不得自交。普通安全区保留橙线，不再
      // 为了贴住每个圆而重画。桥接结果必须逐圆通过target_clearance净距、
      // 保持各障碍原定绕行侧并消除整段进度反向，否则原样保留当前候选。
      auto segment_has_insufficient_original_progress = [&](
          const std::vector<Waypoint> & route,
          std::size_t segment) {
          if (segment + 1 >= route_size) {
            return false;
          }
          const double original_x =
            original_waypoints_[segment + 1].x - original_waypoints_[segment].x;
          const double original_y =
            original_waypoints_[segment + 1].y - original_waypoints_[segment].y;
          if (original_x * original_x + original_y * original_y < 1.0e-12) {
            return false;
          }
          const double route_x = route[segment + 1].x - route[segment].x;
          const double route_y = route[segment + 1].y - route[segment].y;
          const double progress_ratio =
            (route_x * original_x + route_y * original_y) /
            (original_x * original_x + original_y * original_y);
          // 不能只在已经倒退时才修复。圆偏移经过原路线急弯时，
          // 1-kappa*d可能仍为正但已经接近0，离散路线会表现成几乎
          // 原地转向的尖角。复用legacy局部重建的前进比例门槛，提前
          // 把这种近停滞段交给同一条Bezier桥处理。
          return progress_ratio <= waypoint_shift_rebuild_min_progress_ratio_;
        };
      const double progress_bridge_shoulder = std::min(
        circle_envelope_max_transition_length_,
        // 只包住真正反向段，并在两侧各留约0.40m相切接回。此前0.60m
        // 会把本来已经很顺的下一个障碍圆弧也一起重画。
        std::max(0.30, 0.25 * circle_envelope_min_transition_length_));
      constexpr double bridge_handle_ratio = 0.40;
      std::size_t progress_segment = 0;
      while (progress_segment + 1 < route_size) {
        if (!segment_has_insufficient_original_progress(
            candidate, progress_segment))
        {
          ++progress_segment;
          continue;
        }
        const std::size_t violation_first = progress_segment;
        std::size_t violation_last = progress_segment;
        while (violation_last + 2 < route_size &&
          segment_has_insufficient_original_progress(
            candidate, violation_last + 1))
        {
          ++violation_last;
        }

        const double requested_first_s = std::max(
          route_s.front(), route_s[violation_first] - progress_bridge_shoulder);
        const double requested_last_s = std::min(
          route_s.back(), route_s[violation_last + 1] + progress_bridge_shoulder);
        std::size_t anchor_first = static_cast<std::size_t>(std::distance(
          route_s.begin(), std::lower_bound(
            route_s.begin(), route_s.end(), requested_first_s)));
        std::size_t anchor_last = static_cast<std::size_t>(std::distance(
          route_s.begin(), std::lower_bound(
            route_s.begin(), route_s.end(), requested_last_s)));
        anchor_first = std::clamp<std::size_t>(anchor_first, 1, route_size - 2);
        // Task3停车点本身固定不动，但其前一段也可能产生反向尖峰。
        // 允许Bezier以固定末点为接回锚点；末端切向改从原路线单侧取得。
        anchor_last = std::clamp<std::size_t>(anchor_last, 1, route_size - 1);

        // 正反两次经过共用通道时，它们在完整路线s轴上是两段独立路线，
        // 允许各自修复自己的反向尖峰。锚点可以跨在公共弦两侧；是否仍在
        // 允许的两圆中间窄通道内，由下面逐线段的公共弦校验决定。
        if (anchor_last <= anchor_first + 3 ||
          anchor_first >= violation_first ||
          anchor_last <= violation_last + 1)
        {
          progress_segment = violation_last + 1;
          continue;
        }

        Point2D start_tangent{
          candidate[anchor_first].x - candidate[anchor_first - 1].x,
          candidate[anchor_first].y - candidate[anchor_first - 1].y};
        Point2D finish_tangent;
        if (anchor_last + 1 < route_size) {
          finish_tangent = Point2D{
            candidate[anchor_last + 1].x - candidate[anchor_last].x,
            candidate[anchor_last + 1].y - candidate[anchor_last].y};
        } else {
          finish_tangent = Point2D{
            original_waypoints_[anchor_last].x -
            original_waypoints_[anchor_last - 1].x,
            original_waypoints_[anchor_last].y -
            original_waypoints_[anchor_last - 1].y};
        }
        const double start_tangent_length = std::hypot(
          start_tangent.x, start_tangent.y);
        const double finish_tangent_length = std::hypot(
          finish_tangent.x, finish_tangent.y);
        const double chord_length = std::hypot(
          candidate[anchor_last].x - candidate[anchor_first].x,
          candidate[anchor_last].y - candidate[anchor_first].y);
        if (start_tangent_length < 1.0e-9 ||
          finish_tangent_length < 1.0e-9 || chord_length < 1.0e-6)
        {
          progress_segment = violation_last + 1;
          continue;
        }
        start_tangent.x /= start_tangent_length;
        start_tangent.y /= start_tangent_length;
        finish_tangent.x /= finish_tangent_length;
        finish_tangent.y /= finish_tangent_length;

        std::vector<Waypoint> trial = candidate;
        const Point2D p0{
          candidate[anchor_first].x, candidate[anchor_first].y};
        const Point2D p1{
          p0.x + bridge_handle_ratio * chord_length * start_tangent.x,
          p0.y + bridge_handle_ratio * chord_length * start_tangent.y};
        const Point2D p3{
          candidate[anchor_last].x, candidate[anchor_last].y};
        const Point2D p2{
          p3.x - bridge_handle_ratio * chord_length * finish_tangent.x,
          p3.y - bridge_handle_ratio * chord_length * finish_tangent.y};
        const double bridge_s_length =
          route_s[anchor_last] - route_s[anchor_first];
        for (std::size_t index = anchor_first + 1;
          index < anchor_last; ++index)
        {
          const double t = std::clamp(
            (route_s[index] - route_s[anchor_first]) / bridge_s_length,
            0.0, 1.0);
          const double u = 1.0 - t;
          const double w0 = u * u * u;
          const double w1 = 3.0 * u * u * t;
          const double w2 = 3.0 * u * t * t;
          const double w3 = t * t * t;
          trial[index].x =
            w0 * p0.x + w1 * p1.x + w2 * p2.x + w3 * p3.x;
          trial[index].y =
            w0 * p0.y + w1 * p1.y + w2 * p2.y + w3 * p3.y;
        }

        bool valid_bridge = true;
        for (std::size_t index = anchor_first;
          index < anchor_last; ++index)
        {
          if (segment_has_insufficient_original_progress(trial, index)) {
            valid_bridge = false;
            break;
          }
        }
        // “对原路线保持前进”之外，再直接禁止局部桥自身相交，避免急弯
        // 附近每一小段点积都为正、组合起来却仍形成一个小圈。
        auto signed_area = [](const Waypoint & a, const Waypoint & b,
            const Waypoint & c) {
            return (b.x - a.x) * (c.y - a.y) -
                   (b.y - a.y) * (c.x - a.x);
          };
        auto proper_segments_intersect = [&](const Waypoint & a,
            const Waypoint & b, const Waypoint & c, const Waypoint & d) {
            constexpr double epsilon = 1.0e-9;
            const double ab_c = signed_area(a, b, c);
            const double ab_d = signed_area(a, b, d);
            const double cd_a = signed_area(c, d, a);
            const double cd_b = signed_area(c, d, b);
            return ((ab_c > epsilon && ab_d < -epsilon) ||
                   (ab_c < -epsilon && ab_d > epsilon)) &&
                   ((cd_a > epsilon && cd_b < -epsilon) ||
                   (cd_a < -epsilon && cd_b > epsilon));
          };
        for (std::size_t first_segment = anchor_first;
          valid_bridge && first_segment + 1 < anchor_last; ++first_segment)
        {
          for (std::size_t second_segment = first_segment + 2;
            second_segment + 1 <= anchor_last; ++second_segment)
          {
            if (proper_segments_intersect(
                trial[first_segment], trial[first_segment + 1],
                trial[second_segment], trial[second_segment + 1]))
            {
              valid_bridge = false;
              break;
            }
          }
        }
        for (const auto & profile : profiles) {
          if (!valid_bridge ||
            profile.exit_s < route_s[anchor_first] ||
            profile.entry_s > route_s[anchor_last])
          {
            continue;
          }
          const auto & constraint = active_constraints[profile.owner];
          const double required_radius = std::max(
            0.0, constraint.radius - circle_envelope_numeric_margin_);
          for (std::size_t index = anchor_first;
            index < anchor_last; ++index)
          {
            const Point2D closest = closest_point_on_segment(
              constraint.center, trial[index], trial[index + 1]);
            if (std::hypot(
                closest.x - constraint.center.x,
                closest.y - constraint.center.y) + 1.0e-7 < required_radius)
            {
              if (!overlap_bridge_allows(closest, profile.owner)) {
                valid_bridge = false;
                break;
              }
            }
          }
          if (valid_bridge && profile.source >= anchor_first &&
            profile.source <= anchor_last)
          {
            const Point2D source_point{
              trial[profile.source].x, trial[profile.source].y};
            if (!overlap_bridge_allows(source_point, profile.owner)) {
              const double signed_side =
                static_cast<double>(constraint.direction) *
                ((trial[profile.source].x -
                original_waypoints_[profile.source].x) *
                left_normals[profile.source].x +
                (trial[profile.source].y -
                original_waypoints_[profile.source].y) *
                left_normals[profile.source].y);
              valid_bridge = signed_side >= -1.0e-6;
            }
          }
        }
        if (valid_bridge) {
          candidate.swap(trial);
          ++local_bridge_count;
          progress_segment = anchor_last;
        } else {
          progress_segment = violation_last + 1;
        }
      }

    };

  auto overlap_corridor_allows = [&](const Point2D & point,
      std::size_t constraint_index,
      const std::vector<CircleConstraint> & active_constraints) {
      const auto & constraint = active_constraints[constraint_index];
      for (std::size_t partner_index = 0;
        partner_index < active_constraints.size(); ++partner_index)
      {
        if (partner_index == constraint_index) {
          continue;
        }
        const auto & partner = active_constraints[partner_index];
        if (constraint.direction == partner.direction) {
          continue;
        }
        CircleIntersectionChord chord;
        if (circle_intersection_chord(
            constraint.center.x, constraint.center.y, constraint.radius,
            partner.center.x, partner.center.y, partner.radius,
            chord) &&
          point_near_chord_segment(
            point.x, point.y, chord,
            std::max(0.02, 0.5 * circle_envelope_numeric_margin_)))
        {
          return true;
        }
      }
      return false;
    };

  auto clearance_margins = [&](const std::vector<Waypoint> & candidate,
      const std::vector<CircleConstraint> & active_constraints) {
      std::vector<double> margins(
        active_constraints.size(), std::numeric_limits<double>::infinity());
      for (std::size_t constraint_index = 0;
        constraint_index < active_constraints.size(); ++constraint_index)
      {
        const auto & constraint = active_constraints[constraint_index];
        const double required_radius = std::max(
          0.0, constraint.radius - circle_envelope_numeric_margin_);
        for (std::size_t index = 0; index + 1 < candidate.size(); ++index) {
          const Point2D closest = closest_point_on_segment(
            constraint.center, candidate[index], candidate[index + 1]);
          const double margin = std::hypot(
            closest.x - constraint.center.x,
            closest.y - constraint.center.y) - required_radius;
          if (margin < 0.0 && overlap_corridor_allows(
              closest, constraint_index, active_constraints))
          {
            continue;
          }
          margins[constraint_index] = std::min(
            margins[constraint_index], margin);
        }
      }
      return margins;
    };

  auto discrete_curvature_at = [](const std::vector<Waypoint> & route,
      std::size_t index) {
      if (index == 0 || index + 1 >= route.size()) {
        return 0.0;
      }
      const double ab = std::hypot(
        route[index].x - route[index - 1].x,
        route[index].y - route[index - 1].y);
      const double bc = std::hypot(
        route[index + 1].x - route[index].x,
        route[index + 1].y - route[index].y);
      const double ac = std::hypot(
        route[index + 1].x - route[index - 1].x,
        route[index + 1].y - route[index - 1].y);
      const double denominator = ab * bc * ac;
      if (denominator < 1.0e-9) {
        return 0.0;
      }
      const double cross =
        (route[index].x - route[index - 1].x) *
        (route[index + 1].y - route[index - 1].y) -
        (route[index].y - route[index - 1].y) *
        (route[index + 1].x - route[index - 1].x);
      return std::abs(2.0 * cross / denominator);
    };
  auto maximum_discrete_curvature = [&](const std::vector<Waypoint> & route) {
      double maximum = 0.0;
      for (std::size_t index = 1; index + 1 < route.size(); ++index) {
        maximum = std::max(maximum, discrete_curvature_at(route, index));
      }
      return maximum;
    };

  auto progress_violation_count = [&](const std::vector<Waypoint> & route) {
      std::size_t count = 0;
      for (std::size_t index = 0; index + 1 < route_size; ++index) {
        const double original_x =
          original_waypoints_[index + 1].x - original_waypoints_[index].x;
        const double original_y =
          original_waypoints_[index + 1].y - original_waypoints_[index].y;
        const double original_length_squared =
          original_x * original_x + original_y * original_y;
        if (original_length_squared < 1.0e-12) {
          continue;
        }
        const double candidate_x = route[index + 1].x - route[index].x;
        const double candidate_y = route[index + 1].y - route[index].y;
        const double progress_ratio =
          (candidate_x * original_x + candidate_y * original_y) /
          original_length_squared;
        if (progress_ratio <= waypoint_shift_rebuild_min_progress_ratio_) {
          ++count;
        }
      }
      return count;
    };

  struct CandidateQuality
  {
    std::size_t clearance_failures{std::numeric_limits<std::size_t>::max()};
    std::size_t anchor_conflicts{std::numeric_limits<std::size_t>::max()};
    std::size_t progress_violations{std::numeric_limits<std::size_t>::max()};
    std::size_t overlap_corridor_points{0};
    std::size_t local_bridge_count{0};
    double minimum_clearance_margin{-std::numeric_limits<double>::infinity()};
    double maximum_curvature{std::numeric_limits<double>::infinity()};
  };
  auto quality_better = [](const CandidateQuality & first,
      const CandidateQuality & second) {
      if (first.clearance_failures != second.clearance_failures) {
        return first.clearance_failures < second.clearance_failures;
      }
      if (first.anchor_conflicts != second.anchor_conflicts) {
        return first.anchor_conflicts < second.anchor_conflicts;
      }
      if (first.progress_violations != second.progress_violations) {
        return first.progress_violations < second.progress_violations;
      }
      if (first.clearance_failures == 0 &&
        std::abs(first.maximum_curvature - second.maximum_curvature) > 1.0e-9)
      {
        return first.maximum_curvature < second.maximum_curvature;
      }
      if (std::abs(
          first.minimum_clearance_margin -
          second.minimum_clearance_margin) > 1.0e-9)
      {
        return first.minimum_clearance_margin >
               second.minimum_clearance_margin;
      }
      return first.maximum_curvature < second.maximum_curvature;
    };

  const double extension_range = std::max(
    0.0,
    circle_envelope_max_transition_length_ -
    circle_envelope_min_transition_length_);
  // 旧版会按extension_step把整个长度区间逐格穷举；当前0.80m区间会生成
  // 17条完整路线，并在每轮二次审计中重复同一批全路线圆滑。这里恢复为
  // 有界候选：只保留最短和最长两档，仍覆盖完整配置范围。
  // 净距不足由后面的局部连续段修复处理，不再依靠0.05m密集扫参。
  constexpr int maximum_attempt = 1;
  auto search_best_route = [&](const std::vector<CircleConstraint> & active_constraints,
      std::vector<Waypoint> & selected_route,
      CandidateQuality & selected_quality, bool & selected_accepted) {
      selected_route = original_waypoints_;
      selected_quality = CandidateQuality();
      selected_accepted = false;
      for (int attempt = 0; attempt <= maximum_attempt; ++attempt) {
        const double attempt_ratio = maximum_attempt > 0 ?
          static_cast<double>(attempt) / static_cast<double>(maximum_attempt) : 0.0;
        const double common_extension = extension_range * attempt_ratio;
        std::vector<Waypoint> candidate;
        std::size_t anchor_conflicts = 0;
        std::size_t overlap_corridor_points = 0;
        std::size_t local_bridge_count = 0;
        build_candidate(
          active_constraints, common_extension, candidate, anchor_conflicts,
          overlap_corridor_points, local_bridge_count);
        const std::vector<double> margins = clearance_margins(
          candidate, active_constraints);

        CandidateQuality quality;
        quality.anchor_conflicts = anchor_conflicts;
        quality.progress_violations = progress_violation_count(candidate);
        quality.overlap_corridor_points = overlap_corridor_points;
        quality.local_bridge_count = local_bridge_count;
        quality.maximum_curvature = maximum_discrete_curvature(candidate);
        quality.clearance_failures = 0;
        quality.minimum_clearance_margin =
          std::numeric_limits<double>::infinity();
        for (const double margin : margins) {
          quality.minimum_clearance_margin = std::min(
            quality.minimum_clearance_margin, margin);
          if (margin < -1.0e-7) {
            ++quality.clearance_failures;
          }
        }

        if (quality_better(quality, selected_quality)) {
          selected_quality = quality;
          selected_route = candidate;
        }
        if (quality.clearance_failures == 0 &&
          quality.anchor_conflicts == 0 &&
          quality.progress_violations == 0 &&
          quality.maximum_curvature <= circle_envelope_max_curvature_)
        {
          selected_accepted = true;
          break;
        }
      }
    };

  std::vector<Waypoint> best_route;
  CandidateQuality best_quality;
  bool accepted = false;
  search_best_route(constraints, best_route, best_quality, accepted);

  // 第一次路线只由原路线真正命中的障碍生成。它可能被别的障碍的圆滑尾段
  // 推入一个“原路线本来在圆外、因此此前没有偏移元数据”的永久绿/橙点。
  // 这里审计有限的最终路线段，只把真实侵入circle_envelope_target_clearance
  // 的连续段补成局部约束，然后仍交给同一个批量生成器从原路线统一重算。
  // 不逐航点硬推、不把零偏移点注册成全局约束；用有限轮次处理修复后新出现
  // 的另一个侵圆点。已有同ID主约束若仍然侵圆，必须升级为有限区间修复，
  // 不能把“已有约束”误当成“约束已经生效”。
  const std::size_t primary_constraint_count = constraints.size();
  std::size_t secondary_repair_count = 0;
  std::size_t promoted_primary_repair_count = 0;
  std::size_t expanded_secondary_repair_count = 0;
  std::size_t reinforced_secondary_repair_count = 0;
  std::size_t secondary_repair_round_count = 0;
  std::size_t rejected_secondary_replan_count = 0;
  std::size_t route_fairing_accept_count = 0;
  std::size_t route_fairing_reject_count = 0;
  std::size_t residual_permanent_clearance_failures = 0;
  const double planning_clearance_radius =
    circle_envelope_target_clearance_ + circle_envelope_numeric_margin_;
  // 恢复5518413的有界策略。二次审计只修真正小于target_clearance的
  // 连续段；numeric_margin是生成和候选排序的软余量，不值得为它阻塞
  // 50Hz控制循环十几轮。
  constexpr int maximum_secondary_repair_rounds = 4;
  constexpr double permanent_center_tolerance = 0.03;
  const bool audit_permanent_obstacles =
    logic3_planning_pose_anchored_ && !logic3_green_clusters_.empty();
  if (audit_permanent_obstacles) {
    auto intentional_permanent_overlap_corridor = [&](const Point2D & point,
        std::size_t obstacle_index) {
        const auto & obstacle = logic3_green_clusters_[obstacle_index].center;
        for (std::size_t partner_index = 0;
          partner_index < logic3_green_clusters_.size(); ++partner_index)
        {
          if (partner_index == obstacle_index) {
            continue;
          }
          const auto & partner = logic3_green_clusters_[partner_index].center;
          CircleIntersectionChord chord;
          if (circle_intersection_chord(
              obstacle.x, obstacle.y, planning_clearance_radius,
              partner.x, partner.y, planning_clearance_radius,
              chord) &&
            point_near_chord_segment(
              point.x, point.y, chord,
              std::max(0.02, 0.5 * circle_envelope_numeric_margin_)))
          {
            return true;
          }
        }
        return false;
      };

    const std::size_t last_auditable_segment = std::min(
      last_deformable_index, route_size - 2);
    struct PermanentClearanceQuality
    {
      std::size_t failing_obstacles{0};
      std::size_t violating_segments{0};
      double maximum_penetration{0.0};
      double total_penetration{0.0};
      double minimum_clearance{std::numeric_limits<double>::infinity()};
      std::vector<double> maximum_penetration_by_obstacle;
      std::vector<double> total_penetration_by_obstacle;
    };
    auto evaluate_permanent_clearance = [&](const std::vector<Waypoint> & route,
        double required_clearance) {
        PermanentClearanceQuality quality;
        quality.maximum_penetration_by_obstacle.assign(
          logic3_green_clusters_.size(), 0.0);
        quality.total_penetration_by_obstacle.assign(
          logic3_green_clusters_.size(), 0.0);
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
          const std::size_t obstacle_first_deformable =
            obstacle_first_deformable_index(center);
          bool obstacle_failed = false;
          for (std::size_t segment = obstacle_first_deformable;
            segment <= last_auditable_segment; ++segment)
          {
            const Point2D closest = closest_point_on_segment(
              center, route[segment], route[segment + 1]);
            const double clearance = std::hypot(
              closest.x - center.x, closest.y - center.y);
            if (clearance + 1.0e-7 < required_clearance &&
              intentional_permanent_overlap_corridor(closest, obstacle_index))
            {
              continue;
            }
            quality.minimum_clearance = std::min(
              quality.minimum_clearance, clearance);
            const double penetration = std::max(
              0.0, required_clearance - clearance);
            if (penetration <= 1.0e-7) {
              continue;
            }
            obstacle_failed = true;
            ++quality.violating_segments;
            quality.maximum_penetration = std::max(
              quality.maximum_penetration, penetration);
            quality.total_penetration += penetration;
            quality.maximum_penetration_by_obstacle[obstacle_index] = std::max(
              quality.maximum_penetration_by_obstacle[obstacle_index],
              penetration);
            quality.total_penetration_by_obstacle[obstacle_index] += penetration;
          }
          if (obstacle_failed) {
            ++quality.failing_obstacles;
          }
        }
        return quality;
      };
    auto permanent_quality_better = [](
        const PermanentClearanceQuality & candidate,
        const PermanentClearanceQuality & retained) {
        constexpr double tolerance = 1.0e-7;
        if (candidate.failing_obstacles != retained.failing_obstacles) {
          return candidate.failing_obstacles < retained.failing_obstacles;
        }
        if (std::abs(
            candidate.maximum_penetration -
            retained.maximum_penetration) > tolerance)
        {
          return candidate.maximum_penetration < retained.maximum_penetration;
        }
        if (std::abs(
            candidate.total_penetration - retained.total_penetration) > tolerance)
        {
          return candidate.total_penetration < retained.total_penetration;
        }
        if (candidate.violating_segments != retained.violating_segments) {
          return candidate.violating_segments < retained.violating_segments;
        }
        return candidate.minimum_clearance >
               retained.minimum_clearance + tolerance;
      };
    auto preserves_existing_hard_clearance = [](
        const PermanentClearanceQuality & candidate,
        const PermanentClearanceQuality & retained) {
        constexpr double tolerance = 1.0e-7;
        if (candidate.maximum_penetration_by_obstacle.size() !=
          retained.maximum_penetration_by_obstacle.size())
        {
          return false;
        }
        for (std::size_t obstacle_index = 0;
          obstacle_index < candidate.maximum_penetration_by_obstacle.size();
          ++obstacle_index)
        {
          // 已经达到target_clearance的永久点是不可回退的硬底线。
          // numeric_margin仍可在不同障碍间作为优化余量重新分配。
          if (retained.maximum_penetration_by_obstacle[obstacle_index] <= tolerance &&
            candidate.maximum_penetration_by_obstacle[obstacle_index] > tolerance)
          {
            return false;
          }
        }
        return true;
      };

    for (int repair_round = 0;
      repair_round < maximum_secondary_repair_rounds; ++repair_round)
    {
      // 本轮候选若没有改善，约束也必须一起回退；否则下一轮会在一组已经
      // 被拒绝的强化约束上继续做相同的全路线重算。
      const std::vector<CircleConstraint> retained_constraints = constraints;
      std::size_t added_this_round = 0;
      for (std::size_t obstacle_index = 0;
        obstacle_index < logic3_green_clusters_.size(); ++obstacle_index)
      {
        const bool dynamic_takeover =
          obstacle_index < logic3_green_dynamic_takeover_.size() &&
          logic3_green_dynamic_takeover_[obstacle_index];
        if (dynamic_takeover) {
          // 该ID已经明确切换为本次发车实时雷达接管，不能由二次审计
          // 偷偷恢复静态偏移；它的绿/橙雷达掩码仍按原逻辑保留。
          continue;
        }
        const auto & center = logic3_green_clusters_[obstacle_index].center;
        const std::size_t obstacle_first_deformable =
          obstacle_first_deformable_index(center);
        std::vector<bool> violating_segments(route_size - 1, false);
        for (std::size_t segment = obstacle_first_deformable;
          segment <= last_auditable_segment; ++segment)
        {
          const Point2D closest = closest_point_on_segment(
            center, best_route[segment], best_route[segment + 1]);
          if (std::hypot(
              closest.x - center.x, closest.y - center.y) + 1.0e-7 >=
            circle_envelope_target_clearance_ ||
            intentional_permanent_overlap_corridor(closest, obstacle_index))
          {
            continue;
          }
          violating_segments[segment] = true;
        }

        std::size_t segment = obstacle_first_deformable;
        while (segment <= last_auditable_segment) {
          if (!violating_segments[segment]) {
            ++segment;
            continue;
          }
          const std::size_t run_first_segment = segment;
          std::size_t run_last_segment = segment;
          while (run_last_segment + 1 <= last_auditable_segment &&
            violating_segments[run_last_segment + 1])
          {
            ++run_last_segment;
          }
          const std::size_t repair_first = run_first_segment;
          const std::size_t repair_last = run_last_segment + 1;
          const std::size_t represented_first =
            repair_first > obstacle_first_deformable + 2 ?
            repair_first - 2 : obstacle_first_deformable;
          const std::size_t represented_last = std::min(
            last_deformable_index, repair_last + 2);
          auto represented = std::find_if(
            constraints.begin(), constraints.end(),
            [&](const CircleConstraint & constraint) {
              if (std::hypot(
                  constraint.center.x - center.x,
                  constraint.center.y - center.y) > permanent_center_tolerance)
              {
                return false;
              }
              if (constraint.secondary_repair) {
                return constraint.repair_first <= repair_last &&
                       constraint.repair_last >= repair_first;
              }
              return constraint.source >= represented_first &&
                     constraint.source <= represented_last;
            });
          if (represented != constraints.end()) {
            if (!represented->secondary_repair) {
              // 主约束只会约束“原路线航点自身在圆内”的点。如果安全圆
              // 恰好落在两个航点之间，最终线段仍可能穿圆；二次审计已经
              // 用有限线段证实了这次侵入，必须把该主约束升级为有限区间
              // 修复，而不是因为同ID约束存在就直接跳过。
              represented->secondary_repair = true;
              represented->repair_first = repair_first;
              represented->repair_last = repair_last;
              represented->radius = std::max(
                represented->radius,
                circle_envelope_target_clearance_ +
                circle_envelope_numeric_margin_);
              represented->first_deformable = std::max(
                represented->first_deformable,
                obstacle_first_deformable);
              ++added_this_round;
              ++promoted_primary_repair_count;
            } else if (repair_first < represented->repair_first ||
              repair_last > represented->repair_last)
            {
              // 第一轮重算后侵圆连续段可能沿路线前后移动少量索引；只扩展
              // 已有局部核心，不新建同一ID的全局约束，也不改变其方向。
              represented->repair_first = std::min(
                represented->repair_first, repair_first);
              represented->repair_last = std::max(
                represented->repair_last, repair_last);
              ++added_this_round;
              ++expanded_secondary_repair_count;
            } else {
              // “已有二次约束且索引范围相同”不代表最终路线已经避开。
              // 若重算后同一连续段仍侵圆，继续向两侧扩展一个离散步长，
              // 同时增加接线长度；否则added_this_round会保持0，审计会把
              // 一条仍然穿圆的路线误判成无法再处理并提前结束。
              const double requested_first_s = std::max(
                route_s[obstacle_first_deformable],
                route_s[repair_first] - circle_envelope_extension_step_);
              const double requested_last_s = std::min(
                route_s[last_deformable_index],
                route_s[repair_last] + circle_envelope_extension_step_);
              const std::size_t reinforced_first = std::clamp<std::size_t>(
                static_cast<std::size_t>(std::distance(
                  route_s.begin(), std::lower_bound(
                    route_s.begin(), route_s.end(), requested_first_s))),
                obstacle_first_deformable, last_deformable_index);
              const std::size_t reinforced_last = std::clamp<std::size_t>(
                static_cast<std::size_t>(std::distance(
                  route_s.begin(), std::lower_bound(
                    route_s.begin(), route_s.end(), requested_last_s))),
                obstacle_first_deformable, last_deformable_index);
              const std::size_t previous_repair_first =
                represented->repair_first;
              const std::size_t previous_repair_last =
                represented->repair_last;
              const double previous_extension =
                represented->transition_extension;
              represented->repair_first = std::min(
                represented->repair_first, reinforced_first);
              represented->repair_last = std::max(
                represented->repair_last, reinforced_last);
              represented->transition_extension = std::min(
                circle_envelope_max_transition_length_ -
                circle_envelope_min_transition_length_,
                represented->transition_extension +
                circle_envelope_extension_step_);
              represented->radius = std::max(
                represented->radius,
                circle_envelope_target_clearance_ +
                circle_envelope_numeric_margin_);
              if (represented->repair_first < previous_repair_first ||
                represented->repair_last > previous_repair_last ||
                represented->transition_extension > previous_extension + 1.0e-12)
              {
                ++added_this_round;
                ++reinforced_secondary_repair_count;
              }
            }
            segment = run_last_segment + 1;
            continue;
          }

          std::size_t source_segment = run_first_segment;
          Point2D source_projection;
          double source_projection_ratio = 0.0;
          double minimum_original_distance_squared =
            std::numeric_limits<double>::infinity();
          for (std::size_t original_segment = run_first_segment;
            original_segment <= run_last_segment; ++original_segment)
          {
            const auto & start = original_waypoints_[original_segment];
            const auto & finish = original_waypoints_[original_segment + 1];
            const double segment_x = finish.x - start.x;
            const double segment_y = finish.y - start.y;
            const double length_squared =
              segment_x * segment_x + segment_y * segment_y;
            if (length_squared < 1.0e-12) {
              continue;
            }
            const double ratio = std::clamp(
              ((center.x - start.x) * segment_x +
              (center.y - start.y) * segment_y) / length_squared,
              0.0, 1.0);
            const Point2D projection{
              start.x + ratio * segment_x,
              start.y + ratio * segment_y};
            const double distance_squared =
              (projection.x - center.x) * (projection.x - center.x) +
              (projection.y - center.y) * (projection.y - center.y);
            if (distance_squared < minimum_original_distance_squared) {
              minimum_original_distance_squared = distance_squared;
              source_segment = original_segment;
              source_projection = projection;
              source_projection_ratio = ratio;
            }
          }
          std::size_t source = source_projection_ratio < 0.5 ?
            source_segment : source_segment + 1;
          source = std::clamp(
            source, obstacle_first_deformable, last_deformable_index);
          if (!std::isfinite(minimum_original_distance_squared) ||
            !normal_valid[source])
          {
            segment = run_last_segment + 1;
            continue;
          }

          const auto & source_normal = left_normals[source];
          const double obstacle_lateral =
            (center.x - source_projection.x) * source_normal.x +
            (center.y - source_projection.y) * source_normal.y;
          const int direction = obstacle_lateral >= 0.0 ? -1 : 1;
          const double repair_radius =
            circle_envelope_target_clearance_ +
            circle_envelope_numeric_margin_;
          const auto & source_base = original_waypoints_[source];
          const double center_x = center.x - source_base.x;
          const double center_y = center.y - source_base.y;
          const double center_normal =
            center_x * source_normal.x + center_y * source_normal.y;
          const double tangent_squared = std::max(
            0.0,
            center_x * center_x + center_y * center_y -
            center_normal * center_normal);
          double peak_offset = 0.0;
          if (tangent_squared <= repair_radius * repair_radius) {
            const double half_chord = std::sqrt(std::max(
              0.0, repair_radius * repair_radius - tangent_squared));
            const double signed_boundary = direction > 0 ?
              center_normal + half_chord : center_normal - half_chord;
            peak_offset = std::max(
              0.0, static_cast<double>(direction) * signed_boundary);
          }
          constraints.push_back(
            CircleConstraint{
              source, direction, center, repair_radius, peak_offset, 0.0,
              true, repair_first, repair_last,
              obstacle_first_deformable, false});
          ++added_this_round;
          ++secondary_repair_count;
          segment = run_last_segment + 1;
        }
      }

      if (added_this_round == 0) {
        break;
      }
      ++secondary_repair_round_count;
      std::sort(
        constraints.begin(), constraints.end(),
        [](const CircleConstraint & first, const CircleConstraint & second) {
          if (first.source != second.source) {
            return first.source < second.source;
          }
          if (first.center.x != second.center.x) {
            return first.center.x < second.center.x;
          }
          return first.center.y < second.center.y;
        });
      apply_shared_pass_side_locks(constraints);

      // 新一轮仍从未偏移路线批量生成候选，但必须保持每个已经通过
      // target_clearance的永久点继续合格，并且0.35m硬净距质量严格改善，
      // 才替换上一轮。0.42m规划余量和纯曲率改善都不能维持二次修复，
      // 避免安全已经等价时继续重画整条路线。失败候选直接丢弃，因此
      // 增加修复轮数也不会把已经合格的蓝线退回灰色原路线。
      const std::vector<Waypoint> retained_route = best_route;
      const CandidateQuality retained_quality = best_quality;
      const bool retained_accepted = accepted;
      const PermanentClearanceQuality retained_hard =
        evaluate_permanent_clearance(
        retained_route, circle_envelope_target_clearance_);
      std::vector<Waypoint> trial_route;
      CandidateQuality trial_quality;
      bool trial_accepted = false;
      search_best_route(
        constraints, trial_route, trial_quality, trial_accepted);
      const PermanentClearanceQuality trial_hard =
        evaluate_permanent_clearance(
        trial_route, circle_envelope_target_clearance_);
      const bool hard_clearance_preserved =
        preserves_existing_hard_clearance(trial_hard, retained_hard);
      if (hard_clearance_preserved &&
        permanent_quality_better(trial_hard, retained_hard))
      {
        best_route = std::move(trial_route);
        best_quality = trial_quality;
        accepted = trial_accepted;
      } else {
        constraints = retained_constraints;
        best_route = retained_route;
        best_quality = retained_quality;
        accepted = retained_accepted;
        ++rejected_secondary_replan_count;
      }
      RCLCPP_WARN(
        get_logger(),
        "circle_envelope二次净距审计第%zu轮：处理%zu个真实侵圆连续段；"
        "已按固定侧加入局部圆约束并统一圆滑重算；累计拒绝%zu次净距未改善候选。",
        secondary_repair_round_count, added_this_round,
        rejected_secondary_replan_count);
      // 一次无改善就说明当前固定绕行侧和接线模型无法通过继续强化获得
      // 更好路线。保留上一条有效偏移线，立即结束，禁止再空转十几轮。
      if (rejected_secondary_replan_count > 0) {
        break;
      }
    }

    // 圆边界与原路线法线在急弯处接近相切时，路线虽然仍沿原路线前进，
    // 相邻航点却可能被压得过密，形成离散曲率尖峰。这里把尖峰两侧锚点
    // 之间的坏采样点“逻辑删除”，用与当前航点数一致的等弧长三次Bezier
    // 点回填原索引；这样不破坏障碍来源、停车点等按索引保存的状态。
    // 每条回填候选都独立验收硬净距、0.15前进比例及原定绕行侧，不合格
    // 就封锁该局部并继续检查其他尖峰。因此它只能改善已保留的最优路线，
    // 不能把路线退回灰线，也不能用圆滑换取安全净距。
    const PermanentClearanceQuality fairing_reference_planning =
      evaluate_permanent_clearance(best_route, planning_clearance_radius);
    const PermanentClearanceQuality fairing_reference_hard =
      evaluate_permanent_clearance(
      best_route, circle_envelope_target_clearance_);
    std::vector<double> fairing_reference_side(route_size, 0.0);
    std::vector<double> original_curvature(route_size, 0.0);
    for (std::size_t index = 0; index < route_size; ++index) {
      if (normal_valid[index]) {
        fairing_reference_side[index] =
          (best_route[index].x - original_waypoints_[index].x) *
          left_normals[index].x +
          (best_route[index].y - original_waypoints_[index].y) *
          left_normals[index].y;
      }
      original_curvature[index] = discrete_curvature_at(
        original_waypoints_, index);
    }
    struct CurvatureExcessQuality
    {
      double total_squared_excess{0.0};
      double maximum_excess{0.0};
    };
    auto curvature_excess_quality = [&](const std::vector<Waypoint> & route) {
        CurvatureExcessQuality quality;
        for (std::size_t index = 1; index + 1 < route_size; ++index) {
          // 原路线本身的急弯不是本轮障碍规划造成的，不能为了追逐一个
          // 绝对2.2/m目标而改写整条基准路线；只压制明显高于原曲率的峰。
          const double allowed_curvature = std::max(
            circle_envelope_max_curvature_,
            1.5 * original_curvature[index] + 0.25);
          const double excess = std::max(
            0.0, discrete_curvature_at(route, index) - allowed_curvature);
          quality.total_squared_excess += excess * excess;
          quality.maximum_excess = std::max(quality.maximum_excess, excess);
        }
        return quality;
      };
    auto fairing_side_preserved = [&](const std::vector<Waypoint> & route) {
        constexpr double side_reference_threshold = 0.005;
        for (std::size_t index = 0; index < route_size; ++index) {
          if (!normal_valid[index] ||
            std::abs(fairing_reference_side[index]) <=
            side_reference_threshold)
          {
            continue;
          }
          const double candidate_side =
            (route[index].x - original_waypoints_[index].x) *
            left_normals[index].x +
            (route[index].y - original_waypoints_[index].y) *
            left_normals[index].y;
          if (candidate_side * fairing_reference_side[index] < -1.0e-8) {
            return false;
          }
        }
        for (const auto & constraint : constraints) {
          if (constraint.source >= route_size ||
            !normal_valid[constraint.source])
          {
            continue;
          }
          const double signed_side =
            static_cast<double>(constraint.direction) *
            ((route[constraint.source].x -
            original_waypoints_[constraint.source].x) *
            left_normals[constraint.source].x +
            (route[constraint.source].y -
            original_waypoints_[constraint.source].y) *
            left_normals[constraint.source].y);
          // 相交圆公共弦内，已验收路线在某个圆的局部法线定义下可能
          // 天然呈负值；不能要求回填候选突然跳回该圆的普通外侧。
          // 这种情况只允许保持或改善当前值，绝不允许进一步恶化。
          const double reference_signed_side =
            static_cast<double>(constraint.direction) *
            fairing_reference_side[constraint.source];
          const double minimum_signed_side = reference_signed_side < -1.0e-6 ?
            reference_signed_side - 1.0e-6 : -1.0e-6;
          if (signed_side < minimum_signed_side) {
            return false;
          }
        }
        return true;
      };
    const double fairing_clearance_allowance = std::min(
      0.002, 0.10 * std::max(0.0, circle_envelope_numeric_margin_));
    auto fairing_planning_clearance_preserved = [&] (
        const PermanentClearanceQuality & candidate) {
        if (candidate.maximum_penetration_by_obstacle.size() !=
          fairing_reference_planning.maximum_penetration_by_obstacle.size())
        {
          return false;
        }
        for (std::size_t obstacle_index = 0;
          obstacle_index < candidate.maximum_penetration_by_obstacle.size();
          ++obstacle_index)
        {
          if (candidate.maximum_penetration_by_obstacle[obstacle_index] >
            fairing_reference_planning.
            maximum_penetration_by_obstacle[obstacle_index] +
            fairing_clearance_allowance + 1.0e-7)
          {
            return false;
          }
        }
        return true;
      };

    // 尖峰只做少量局部“删除后等弧回填”。每个候选都要重新检查硬净距、
    // 固定侧和前进性，因此必须限制候选数，不能重新退化成全路线优化器。
    constexpr std::size_t maximum_fairing_hotspots = 4;
    const std::size_t maximum_fairing_iterations = std::min<std::size_t>(
      maximum_fairing_hotspots, route_size);
    const double fairing_bridge_shoulder = std::clamp(
      0.15 * circle_envelope_min_transition_length_, 0.20, 0.30);
    constexpr std::size_t fairing_block_radius = 2;
    auto build_equal_arc_fairing_bridge = [&] (
        const std::vector<Waypoint> & source_route,
        std::size_t hotspot, double shoulder, double handle_ratio,
        std::vector<Waypoint> & trial_route,
        std::size_t & fairing_first, std::size_t & fairing_last) {
        fairing_first = hotspot;
        while (fairing_first > 1 &&
          route_s[hotspot] - route_s[fairing_first] < shoulder)
        {
          --fairing_first;
        }
        fairing_last = hotspot;
        while (fairing_last + 1 < route_size - 1 &&
          route_s[fairing_last] - route_s[hotspot] < shoulder)
        {
          ++fairing_last;
        }
        if (fairing_last <= fairing_first + 3) {
          return false;
        }

        Point2D start_tangent{
          source_route[fairing_first].x -
          source_route[fairing_first - 1].x,
          source_route[fairing_first].y -
          source_route[fairing_first - 1].y};
        Point2D finish_tangent{
          source_route[fairing_last + 1].x -
          source_route[fairing_last].x,
          source_route[fairing_last + 1].y -
          source_route[fairing_last].y};
        const double start_tangent_length = std::hypot(
          start_tangent.x, start_tangent.y);
        const double finish_tangent_length = std::hypot(
          finish_tangent.x, finish_tangent.y);
        const double chord_length = std::hypot(
          source_route[fairing_last].x - source_route[fairing_first].x,
          source_route[fairing_last].y - source_route[fairing_first].y);
        if (start_tangent_length < 1.0e-9 ||
          finish_tangent_length < 1.0e-9 || chord_length < 1.0e-6)
        {
          return false;
        }
        start_tangent.x /= start_tangent_length;
        start_tangent.y /= start_tangent_length;
        finish_tangent.x /= finish_tangent_length;
        finish_tangent.y /= finish_tangent_length;

        const Point2D p0{
          source_route[fairing_first].x, source_route[fairing_first].y};
        const double handle_length = handle_ratio * chord_length;
        const Point2D p1{
          p0.x + handle_length * start_tangent.x,
          p0.y + handle_length * start_tangent.y};
        const Point2D p3{
          source_route[fairing_last].x, source_route[fairing_last].y};
        const Point2D p2{
          p3.x - handle_length * finish_tangent.x,
          p3.y - handle_length * finish_tangent.y};
        const std::size_t sample_count = std::max<std::size_t>(
          64, 8 * (fairing_last - fairing_first));
        std::vector<Point2D> bridge_samples(sample_count + 1);
        std::vector<double> bridge_sample_s(sample_count + 1, 0.0);
        for (std::size_t sample = 0; sample <= sample_count; ++sample) {
          const double t = static_cast<double>(sample) /
            static_cast<double>(sample_count);
          const double u = 1.0 - t;
          const double w0 = u * u * u;
          const double w1 = 3.0 * u * u * t;
          const double w2 = 3.0 * u * t * t;
          const double w3 = t * t * t;
          bridge_samples[sample] = Point2D{
            w0 * p0.x + w1 * p1.x + w2 * p2.x + w3 * p3.x,
            w0 * p0.y + w1 * p1.y + w2 * p2.y + w3 * p3.y};
          if (sample > 0) {
            bridge_sample_s[sample] = bridge_sample_s[sample - 1] +
              std::hypot(
              bridge_samples[sample].x - bridge_samples[sample - 1].x,
              bridge_samples[sample].y - bridge_samples[sample - 1].y);
          }
        }
        if (bridge_sample_s.back() < 1.0e-6) {
          return false;
        }

        trial_route = source_route;
        const double bridge_interval_count = static_cast<double>(
          fairing_last - fairing_first);
        for (std::size_t index = fairing_first + 1;
          index < fairing_last; ++index)
        {
          const double target_s = bridge_sample_s.back() *
            static_cast<double>(index - fairing_first) /
            bridge_interval_count;
          auto upper = std::lower_bound(
            bridge_sample_s.begin(), bridge_sample_s.end(), target_s);
          std::size_t upper_index = static_cast<std::size_t>(std::distance(
            bridge_sample_s.begin(), upper));
          upper_index = std::clamp<std::size_t>(
            upper_index, 1, sample_count);
          const std::size_t lower_index = upper_index - 1;
          const double sample_span =
            bridge_sample_s[upper_index] - bridge_sample_s[lower_index];
          const double sample_ratio = sample_span > 1.0e-12 ?
            std::clamp(
            (target_s - bridge_sample_s[lower_index]) / sample_span,
            0.0, 1.0) : 0.0;
          trial_route[index].x =
            (1.0 - sample_ratio) * bridge_samples[lower_index].x +
            sample_ratio * bridge_samples[upper_index].x;
          trial_route[index].y =
            (1.0 - sample_ratio) * bridge_samples[lower_index].y +
            sample_ratio * bridge_samples[upper_index].y;
        }
        return true;
      };

    // 主偏移继续做16轮一维圆滑；这里只补最多4个残余尖峰，避免重复优化。
    {
      std::vector<bool> fairing_blocked(route_size, false);
      for (std::size_t iteration = 0;
        iteration < maximum_fairing_iterations; ++iteration)
      {
        std::size_t hotspot = route_size;
        double hotspot_excess = 1.0e-6;
        for (std::size_t index = 1; index + 1 < route_size; ++index) {
          if (fairing_blocked[index]) {
            continue;
          }
          const double allowed_curvature = std::max(
            circle_envelope_max_curvature_,
            1.5 * original_curvature[index] + 0.25);
          const double excess =
            discrete_curvature_at(best_route, index) - allowed_curvature;
          if (excess > hotspot_excess) {
            hotspot_excess = excess;
            hotspot = index;
          }
        }
        if (hotspot == route_size) {
          break;
        }

        const std::vector<std::pair<double, double>> bridge_variants{
          {fairing_bridge_shoulder, 0.35}};

        const CurvatureExcessQuality retained_shape =
          curvature_excess_quality(best_route);
        bool selected = false;
        std::vector<Waypoint> selected_route;
        CurvatureExcessQuality selected_shape;
        selected_shape.total_squared_excess =
          std::numeric_limits<double>::infinity();
        selected_shape.maximum_excess =
          std::numeric_limits<double>::infinity();
        for (const auto & variant : bridge_variants) {
          std::vector<Waypoint> trial_route;
          std::size_t fairing_first = 0;
          std::size_t fairing_last = 0;
          if (!build_equal_arc_fairing_bridge(
              best_route, hotspot, variant.first, variant.second,
              trial_route, fairing_first, fairing_last))
          {
            continue;
          }

          const PermanentClearanceQuality trial_hard =
            evaluate_permanent_clearance(
            trial_route, circle_envelope_target_clearance_);
          const PermanentClearanceQuality trial_planning =
            evaluate_permanent_clearance(
            trial_route, planning_clearance_radius);
          const CurvatureExcessQuality trial_shape =
            curvature_excess_quality(trial_route);
          const std::size_t region_first =
            fairing_first > 1 ? fairing_first - 1 : 1;
          const std::size_t region_last = std::min(
            route_size - 2, fairing_last + 1);
          double retained_region_excess = 0.0;
          double trial_region_excess = 0.0;
          for (std::size_t index = region_first;
            index <= region_last; ++index)
          {
            const double allowed_curvature = std::max(
              circle_envelope_max_curvature_,
              1.5 * original_curvature[index] + 0.25);
            retained_region_excess = std::max(
              retained_region_excess,
              discrete_curvature_at(best_route, index) - allowed_curvature);
            trial_region_excess = std::max(
              trial_region_excess,
              discrete_curvature_at(trial_route, index) - allowed_curvature);
          }
          const bool valid_fairing =
            progress_violation_count(trial_route) == 0 &&
            fairing_side_preserved(trial_route) &&
            preserves_existing_hard_clearance(
              trial_hard, fairing_reference_hard) &&
            fairing_planning_clearance_preserved(trial_planning) &&
            trial_region_excess + 1.0e-7 < retained_region_excess &&
            trial_shape.maximum_excess <=
            retained_shape.maximum_excess + 1.0e-7 &&
            trial_shape.total_squared_excess + 1.0e-9 <
            retained_shape.total_squared_excess;
          if (!valid_fairing) {
            continue;
          }
          if (!selected ||
            trial_shape.maximum_excess + 1.0e-7 <
            selected_shape.maximum_excess ||
            (std::abs(
              trial_shape.maximum_excess - selected_shape.maximum_excess) <=
            1.0e-7 &&
            trial_shape.total_squared_excess + 1.0e-9 <
            selected_shape.total_squared_excess))
          {
            selected = true;
            selected_route = std::move(trial_route);
            selected_shape = trial_shape;
          }
        }
        if (selected) {
          best_route = std::move(selected_route);
          ++route_fairing_accept_count;
          continue;
        }

        const std::size_t blocked_first =
          hotspot > fairing_block_radius ? hotspot - fairing_block_radius : 1;
        const std::size_t blocked_last = std::min(
          route_size - 2, hotspot + fairing_block_radius);
        for (std::size_t index = blocked_first;
          index <= blocked_last; ++index)
        {
          fairing_blocked[index] = true;
        }
        ++route_fairing_reject_count;
      }
    }

    // 最后只按真实绿/橙点逐线段验收，不再用“约束已经存在”代替净距。
    // 数值裕量用于生成，验收仍严格采用用户配置的target_clearance。
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
      const std::size_t obstacle_first_deformable =
        obstacle_first_deformable_index(center);
      bool obstacle_failed = false;
      for (std::size_t segment = obstacle_first_deformable;
        segment < route_size - 1; ++segment)
      {
        const Point2D closest = closest_point_on_segment(
          center, best_route[segment], best_route[segment + 1]);
        if (std::hypot(
            closest.x - center.x, closest.y - center.y) + 1.0e-7 <
          circle_envelope_target_clearance_ &&
          !intentional_permanent_overlap_corridor(closest, obstacle_index))
        {
          obstacle_failed = true;
          break;
        }
      }
      if (obstacle_failed) {
        ++residual_permanent_clearance_failures;
        if (obstacle_index < logic3_green_route_clearance_failed_.size()) {
          logic3_green_route_clearance_failed_[obstacle_index] = true;
        }
      }
    }

    // 二次审计可能补入新约束，最终按真实永久点再做一次硬净距统计。
    const std::vector<double> final_margins = clearance_margins(
      best_route, constraints);
    best_quality.clearance_failures = 0;
    best_quality.minimum_clearance_margin =
      std::numeric_limits<double>::infinity();
    for (const double margin : final_margins) {
      best_quality.minimum_clearance_margin = std::min(
        best_quality.minimum_clearance_margin, margin);
      if (margin < -1.0e-7) {
        ++best_quality.clearance_failures;
      }
    }
    best_quality.progress_violations = progress_violation_count(best_route);
    best_quality.maximum_curvature = maximum_discrete_curvature(best_route);
    accepted = residual_permanent_clearance_failures == 0 &&
      best_quality.progress_violations == 0 &&
      best_quality.maximum_curvature <= circle_envelope_max_curvature_;
  }

  shifted_route = std::move(best_route);
  // READY预规划是发车前的纯计算候选：不发布调试图，也不用这些
  // 过程信息刷屏。真正冻结或运行中重规划仍保留原有日志。
  if (!logic3_ready_preplanning_active_ && !shared_pass_geometries.empty()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope共用通道：识别%zu组正反重复经过、%zu对同位航点；"
      "仅统一障碍世界绕行侧，不再强制平均两次路线坐标。",
      shared_pass_geometries.size(), shared_pass_waypoint_pair_count);
  }
  if (!logic3_ready_preplanning_active_ && best_quality.clearance_failures > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope第一次生成及局部复查后仍有%zu个独立安全圆净距不足；"
      "没有合并障碍物或翻转方向，继续输出当前最优路线。",
      best_quality.clearance_failures);
  }
  if (!logic3_ready_preplanning_active_ && residual_permanent_clearance_failures > 0) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope二次净距审计后仍有%zu个永久障碍不满足target_clearance；"
      "保留当前最外推路线且不停车、不回退原路线；这些ID本轮解除绿/橙掩码，"
      "继续由实时雷达执行偏移。",
      residual_permanent_clearance_failures);
  }
  if (!logic3_ready_preplanning_active_ && best_quality.anchor_conflicts > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope有%zu个非相交圆路线点同时受到相反约束；"
      "障碍物保持独立，未换向。",
      best_quality.anchor_conflicts);
  }
  if (!logic3_ready_preplanning_active_ && best_quality.progress_violations > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope路线仍有%zu个局部点对原路线进度反向；"
      "局部Bezier桥未通过净距或方向校验，保留橙色基线路线。",
      best_quality.progress_violations);
  }
  if (!logic3_ready_preplanning_active_ && !accepted &&
    best_quality.maximum_curvature > circle_envelope_max_curvature_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope最大离散曲率%.3f仍高于软目标%.3f；"
      "安全圆、独立方向和原路线进度优先，继续使用该路线。",
      best_quality.maximum_curvature, circle_envelope_max_curvature_);
  }
  if (!logic3_ready_preplanning_active_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope生成：第一次约束=%zu，二次新增=%zu，主约束升级=%zu，"
      "区间扩展=%zu，同段强化=%zu(轮次=%zu)，"
      "最终约束=%zu，进度反向局部桥=%zu，尖峰等距回填=%zu/%zu，净距不足=%zu，"
      "非相交冲突=%zu，相交圆公共弦点=%zu，进度反向=%zu，"
      "最大曲率=%.3f。",
      primary_constraint_count, secondary_repair_count,
      promoted_primary_repair_count, expanded_secondary_repair_count,
      reinforced_secondary_repair_count, secondary_repair_round_count,
      constraints.size(),
      best_quality.local_bridge_count,
      route_fairing_accept_count, route_fairing_reject_count,
      best_quality.clearance_failures,
      best_quality.anchor_conflicts, best_quality.overlap_corridor_points,
      best_quality.progress_violations,
      best_quality.maximum_curvature);
  }
}
