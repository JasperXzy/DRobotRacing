#include "circle_envelope_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace origincar_system::detail
{

CircleEnvelopeCandidate build_circle_envelope_candidate(
  const CircleEnvelopeRouteGeometry & geometry,
  const std::vector<CircleConstraint> & active_constraints,
  const CircleEnvelopeSettings & settings, const std::optional<double> & minimum_y,
  double common_extension)
{
  const auto & original_route = geometry.original_route;
  const auto & route_s = geometry.route_s;
  const auto & left_normals = geometry.left_normals;
  const auto & normal_valid = geometry.normal_valid;
  const std::size_t route_size = original_route.size();
  constexpr std::size_t first_deformable_index = 0;
  const std::size_t last_deformable_index = route_size - 1;

  CircleEnvelopeCandidate result;
  auto & candidate = result.route;
  auto & constraint_conflicts = result.anchor_conflicts;
  auto & overlap_corridor_points = result.overlap_corridor_points;
  auto & local_bridge_count = result.local_bridge_count;

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
    if (minimum_y) {
      const auto & base = original_route[index];
      const auto & normal = left_normals[index];
      const double minimum_y_offset = *minimum_y - base.y;
      if (normal.y > 1.0e-9) {
        lower_bounds[index] = std::max(
          lower_bounds[index], minimum_y_offset / normal.y);
      } else if (normal.y < -1.0e-9) {
        upper_bounds[index] = std::min(
          upper_bounds[index], minimum_y_offset / normal.y);
      } else if (base.y < *minimum_y) {
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
      const auto & base = original_route[index];
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
          settings.max_curvature);
        return std::clamp(
          std::max(
            settings.min_transition_length +
            0.5 * settings.merge_gap, curvature_transition) +
          common_extension + constraint.transition_extension,
          settings.min_transition_length,
          settings.max_transition_length);
      };

    const double maximum_reasonable_peak =
      settings.cloud_check_radius + settings.max_cone_radius +
      settings.target_clearance + settings.numeric_margin;
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
        const auto & base = original_route[index];
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
    settings.merge_gap,
    0.5 * (settings.target_clearance +
    settings.numeric_margin));
  const double maximum_reasonable_chord_offset =
    settings.cloud_check_radius + settings.max_cone_radius +
    settings.target_clearance + settings.numeric_margin;
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
        const auto & base = original_route[index];
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
    iteration < settings.smoothing_iterations; ++iteration)
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

  candidate = original_route;
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
        0.03, settings.merge_gap);
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
        original_route[segment + 1].x - original_route[segment].x;
      const double original_y =
        original_route[segment + 1].y - original_route[segment].y;
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
      return progress_ratio <= settings.min_progress_ratio;
    };
  const double progress_bridge_shoulder = std::min(
    settings.max_transition_length,
    // 只包住真正反向段，并在两侧各留约0.40m相切接回。此前0.60m
    // 会把本来已经很顺的下一个障碍圆弧也一起重画。
    std::max(0.30, 0.25 * settings.min_transition_length));
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
        original_route[anchor_last].x -
        original_route[anchor_last - 1].x,
        original_route[anchor_last].y -
        original_route[anchor_last - 1].y};
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
        0.0, constraint.radius - settings.numeric_margin);
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
            original_route[profile.source].x) *
            left_normals[profile.source].x +
            (trial[profile.source].y -
            original_route[profile.source].y) *
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

  return result;
}

}  // namespace origincar_system::detail
