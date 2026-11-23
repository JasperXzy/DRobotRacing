#include "origincar_system/circle_envelope.hpp"

#include "circle_envelope_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace origincar_system
{

using detail::CircleConstraint;
using detail::closest_point_on_segment;

std::optional<CircleEnvelopePlan> plan_circle_envelope(
  const std::vector<Waypoint> & original_route, const std::vector<WaypointShift> & shifts,
  const CircleEnvelopeRequest & request, const ClearanceRepairObserver & on_repair)
{
  const std::size_t route_size = original_route.size();
  if (route_size < 3 || shifts.size() != route_size) {
    return std::nullopt;
  }

  const auto & settings = request.settings;
  CircleEnvelopePlan plan;
  plan.obstacles_requiring_realtime_avoidance.assign(request.obstacles.size(), false);

  std::vector<double> route_s(route_size, 0.0);
  std::vector<Point2D> left_normals(route_size, Point2D());
  std::vector<bool> normal_valid(route_size, false);
  for (std::size_t index = 1; index < route_size; ++index) {
    route_s[index] = route_s[index - 1] + std::hypot(
      original_route[index].x - original_route[index - 1].x,
      original_route[index].y - original_route[index - 1].y);
  }
  for (std::size_t index = 0; index < route_size; ++index) {
    normal_valid[index] = left_normal_at(original_route, index, left_normals[index]);
  }

  // A→C→B已经在original_waypoints_中接到Task2首点B之前。静态偏移必须
  // 把这条组合路线视为一个整体，允许B附近障碍的before窗口自然跨过B，
  // 不能因为过渡段同时启用实时雷达就把A→C→B排除在可偏移范围之外。
  const std::size_t first_deformable_index = 0;
  const std::size_t last_deformable_index = route_size - 1;

  // 仅二维码倒车完成后的A->C->B重建会启用本过滤：用当时A沿yaw前移
  // 配置距离后的虚拟参考点和原yaw冻结一次车前/车后关系。车后永久点仍可
  // 作用于起始保护长度之后的路线，但circle_envelope接线和二次净距修复
  // 不得越过同一轮保存的最早可偏移索引，避免倒灌到A点附近。
  auto obstacle_first_deformable_index = [&](const Point2D & center) {
      if (!request.rear_obstacle_limit) {
        return first_deformable_index;
      }
      const double relative_x =
        center.x - request.rear_obstacle_limit->origin.x;
      const double relative_y =
        center.y - request.rear_obstacle_limit->origin.y;
      const double forward_projection =
        relative_x * std::cos(request.rear_obstacle_limit->yaw) +
        relative_y * std::sin(request.rear_obstacle_limit->yaw);
      if (forward_projection >= 0.0) {
        return first_deformable_index;
      }
      return std::clamp(
        request.rear_obstacle_limit->first_deformable_index,
        first_deformable_index, last_deformable_index);
    };

  std::vector<CircleConstraint> constraints;
  for (std::size_t index = 0; index < route_size; ++index) {
    if (std::abs(shifts[index].offset) < 1.0e-9 ||
      shifts[index].forbidden_radius <= 1.0e-9 ||
      shifts[index].source_index >= route_size)
    {
      continue;
    }

    const std::size_t source = shifts[index].source_index;
    if (source < first_deformable_index || source > last_deformable_index ||
      !normal_valid[source])
    {
      continue;
    }

    // 优先使用障碍物投影到原路线时保存的世界坐标绕行方向，再换算成
    // source处原路线左法向的正负号。后续任何圆滑和净距修正都不得改号。
    int direction = shifts[index].offset > 0.0 ? 1 : -1;
    const auto & world_direction = shifts[index].world_direction;
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
    const auto & center = shifts[index].obstacle_center;
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
          shifts[index].forbidden_radius,
          std::abs(shifts[index].offset), 0.0,
          false, 0, 0, constraint_first_deformable, false});
    } else {
      existing->radius = std::max(
        existing->radius, shifts[index].forbidden_radius);
      existing->peak_offset = std::max(
        existing->peak_offset, std::abs(shifts[index].offset));
      existing->first_deformable = std::max(
        existing->first_deformable, constraint_first_deformable);
    }
  }
  if (constraints.empty()) {
    plan.route = original_route;
    return plan;
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
      const auto & first_source = original_route[first.source];
      const auto & second_source = original_route[second.source];
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
            original_route[first_waypoint].x -
            original_route[second_waypoint].x,
            original_route[first_waypoint].y -
            original_route[second_waypoint].y) > shared_waypoint_tolerance)
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
            original_route[first_waypoint].x -
            original_route[second_waypoint].x,
            original_route[first_waypoint].y -
            original_route[second_waypoint].y) > shared_waypoint_tolerance)
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
            settings.max_transition_length +
            settings.extension_step)
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
    const auto & base = original_route[constraint.source];
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

  const detail::CircleEnvelopeRouteGeometry geometry{
    original_route, route_s, left_normals, normal_valid};

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
            std::max(0.02, 0.5 * settings.numeric_margin)))
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
          0.0, constraint.radius - settings.numeric_margin);
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
          original_route[index + 1].x - original_route[index].x;
        const double original_y =
          original_route[index + 1].y - original_route[index].y;
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
        if (progress_ratio <= settings.min_progress_ratio) {
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
    settings.max_transition_length -
    settings.min_transition_length);
  // 旧版会按extension_step把整个长度区间逐格穷举；当前0.80m区间会生成
  // 17条完整路线，并在每轮二次审计中重复同一批全路线圆滑。这里恢复为
  // 有界候选：只保留最短和最长两档，仍覆盖完整配置范围。
  // 净距不足由后面的局部连续段修复处理，不再依靠0.05m密集扫参。
  constexpr int maximum_attempt = 1;
  auto search_best_route = [&](const std::vector<CircleConstraint> & active_constraints,
      std::vector<Waypoint> & selected_route,
      CandidateQuality & selected_quality, bool & selected_accepted) {
      selected_route = original_route;
      selected_quality = CandidateQuality();
      selected_accepted = false;
      for (int attempt = 0; attempt <= maximum_attempt; ++attempt) {
        const double attempt_ratio = maximum_attempt > 0 ?
          static_cast<double>(attempt) / static_cast<double>(maximum_attempt) : 0.0;
        const double common_extension = extension_range * attempt_ratio;
        auto candidate = detail::build_circle_envelope_candidate(
          geometry, active_constraints, settings, request.minimum_y, common_extension);
        const std::vector<double> margins = clearance_margins(
          candidate.route, active_constraints);

        CandidateQuality quality;
        quality.anchor_conflicts = candidate.anchor_conflicts;
        quality.progress_violations = progress_violation_count(candidate.route);
        quality.overlap_corridor_points = candidate.overlap_corridor_points;
        quality.local_bridge_count = candidate.local_bridge_count;
        quality.maximum_curvature = maximum_discrete_curvature(candidate.route);
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
          selected_route = std::move(candidate.route);
        }
        if (quality.clearance_failures == 0 &&
          quality.anchor_conflicts == 0 &&
          quality.progress_violations == 0 &&
          quality.maximum_curvature <= settings.max_curvature)
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
  std::size_t rejected_secondary_replan_count = 0;
  std::size_t route_fairing_accept_count = 0;
  std::size_t route_fairing_reject_count = 0;
  std::size_t residual_permanent_clearance_failures = 0;
  const double planning_clearance_radius =
    settings.target_clearance + settings.numeric_margin;
  // 恢复5518413的有界策略。二次审计只修真正小于target_clearance的
  // 连续段；numeric_margin是生成和候选排序的软余量，不值得为它阻塞
  // 50Hz控制循环十几轮。
  constexpr int maximum_secondary_repair_rounds = 4;
  constexpr double permanent_center_tolerance = 0.03;
  const bool audit_permanent_obstacles =
    request.audit_permanent_obstacles && !request.obstacles.empty();
  if (audit_permanent_obstacles) {
    auto intentional_permanent_overlap_corridor = [&](const Point2D & point,
        std::size_t obstacle_index) {
        const auto & obstacle = request.obstacles[obstacle_index].center;
        for (std::size_t partner_index = 0;
          partner_index < request.obstacles.size(); ++partner_index)
        {
          if (partner_index == obstacle_index) {
            continue;
          }
          const auto & partner = request.obstacles[partner_index].center;
          CircleIntersectionChord chord;
          if (circle_intersection_chord(
              obstacle.x, obstacle.y, planning_clearance_radius,
              partner.x, partner.y, planning_clearance_radius,
              chord) &&
            point_near_chord_segment(
              point.x, point.y, chord,
              std::max(0.02, 0.5 * settings.numeric_margin)))
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
          request.obstacles.size(), 0.0);
        quality.total_penetration_by_obstacle.assign(
          request.obstacles.size(), 0.0);
        for (std::size_t obstacle_index = 0;
          obstacle_index < request.obstacles.size(); ++obstacle_index)
        {
          if (request.obstacles[obstacle_index].handled_by_realtime_avoidance) {
            continue;
          }
          const auto & center = request.obstacles[obstacle_index].center;
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
        obstacle_index < request.obstacles.size(); ++obstacle_index)
      {
        if (request.obstacles[obstacle_index].handled_by_realtime_avoidance) {
          // 该ID已经明确切换为本次发车实时雷达接管，不能由二次审计
          // 偷偷恢复静态偏移；它的绿/橙雷达掩码仍按原逻辑保留。
          continue;
        }
        const auto & center = request.obstacles[obstacle_index].center;
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
            settings.target_clearance ||
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
                settings.target_clearance +
                settings.numeric_margin);
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
                route_s[repair_first] - settings.extension_step);
              const double requested_last_s = std::min(
                route_s[last_deformable_index],
                route_s[repair_last] + settings.extension_step);
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
                settings.max_transition_length -
                settings.min_transition_length,
                represented->transition_extension +
                settings.extension_step);
              represented->radius = std::max(
                represented->radius,
                settings.target_clearance +
                settings.numeric_margin);
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
            const auto & start = original_route[original_segment];
            const auto & finish = original_route[original_segment + 1];
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
            settings.target_clearance +
            settings.numeric_margin;
          const auto & source_base = original_route[source];
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
        retained_route, settings.target_clearance);
      std::vector<Waypoint> trial_route;
      CandidateQuality trial_quality;
      bool trial_accepted = false;
      search_best_route(
        constraints, trial_route, trial_quality, trial_accepted);
      const PermanentClearanceQuality trial_hard =
        evaluate_permanent_clearance(
        trial_route, settings.target_clearance);
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
      ++plan.report.repair_rounds;
      if (on_repair) {
        on_repair({plan.report.repair_rounds, added_this_round, rejected_secondary_replan_count});
      }
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
      best_route, settings.target_clearance);
    std::vector<double> fairing_reference_side(route_size, 0.0);
    std::vector<double> original_curvature(route_size, 0.0);
    for (std::size_t index = 0; index < route_size; ++index) {
      if (normal_valid[index]) {
        fairing_reference_side[index] =
          (best_route[index].x - original_route[index].x) *
          left_normals[index].x +
          (best_route[index].y - original_route[index].y) *
          left_normals[index].y;
      }
      original_curvature[index] = discrete_curvature_at(
        original_route, index);
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
            settings.max_curvature,
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
            (route[index].x - original_route[index].x) *
            left_normals[index].x +
            (route[index].y - original_route[index].y) *
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
            original_route[constraint.source].x) *
            left_normals[constraint.source].x +
            (route[constraint.source].y -
            original_route[constraint.source].y) *
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
      0.002, 0.10 * std::max(0.0, settings.numeric_margin));
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
      0.15 * settings.min_transition_length, 0.20, 0.30);
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
            settings.max_curvature,
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
            trial_route, settings.target_clearance);
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
              settings.max_curvature,
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
      obstacle_index < request.obstacles.size(); ++obstacle_index)
    {
      if (request.obstacles[obstacle_index].handled_by_realtime_avoidance) {
        continue;
      }
      const auto & center = request.obstacles[obstacle_index].center;
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
          settings.target_clearance &&
          !intentional_permanent_overlap_corridor(closest, obstacle_index))
        {
          obstacle_failed = true;
          break;
        }
      }
      if (obstacle_failed) {
        ++residual_permanent_clearance_failures;
        plan.obstacles_requiring_realtime_avoidance[obstacle_index] = true;
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
      best_quality.maximum_curvature <= settings.max_curvature;
  }

  plan.route = std::move(best_route);
  plan.report.shaped = true;
  plan.report.accepted = accepted;
  plan.report.shared_passes = shared_pass_geometries.size();
  plan.report.shared_waypoint_pairs = shared_pass_waypoint_pair_count;
  plan.report.primary_constraints = primary_constraint_count;
  plan.report.secondary_constraints = secondary_repair_count;
  plan.report.promoted_constraints = promoted_primary_repair_count;
  plan.report.expanded_constraints = expanded_secondary_repair_count;
  plan.report.reinforced_constraints = reinforced_secondary_repair_count;
  plan.report.final_constraints = constraints.size();
  plan.report.local_bridges = best_quality.local_bridge_count;
  plan.report.fairing_accepted = route_fairing_accept_count;
  plan.report.fairing_rejected = route_fairing_reject_count;
  plan.report.clearance_failures = best_quality.clearance_failures;
  plan.report.permanent_clearance_failures = residual_permanent_clearance_failures;
  plan.report.anchor_conflicts = best_quality.anchor_conflicts;
  plan.report.overlap_corridor_points = best_quality.overlap_corridor_points;
  plan.report.progress_violations = best_quality.progress_violations;
  plan.report.maximum_curvature = best_quality.maximum_curvature;
  return plan;
}

}  // namespace origincar_system
