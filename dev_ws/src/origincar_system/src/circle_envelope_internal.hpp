#ifndef ORIGINCAR_SYSTEM_CIRCLE_ENVELOPE_INTERNAL_HPP_
#define ORIGINCAR_SYSTEM_CIRCLE_ENVELOPE_INTERNAL_HPP_

#include <algorithm>
#include <cstddef>
#include <optional>
#include <vector>

#include "origincar_system/circle_envelope.hpp"

namespace origincar_system::detail
{

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

struct CircleEnvelopeRouteGeometry
{
  const std::vector<Waypoint> & original_route;
  const std::vector<double> & route_s;
  const std::vector<Point2D> & left_normals;
  const std::vector<bool> & normal_valid;
};

struct CircleEnvelopeCandidate
{
  std::vector<Waypoint> route;
  std::size_t anchor_conflicts{0};
  std::size_t overlap_corridor_points{0};
  std::size_t local_bridge_count{0};
};

inline Point2D closest_point_on_segment(
  const Point2D & point, const Waypoint & start, const Waypoint & finish)
{
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
}

CircleEnvelopeCandidate build_circle_envelope_candidate(
  const CircleEnvelopeRouteGeometry & geometry,
  const std::vector<CircleConstraint> & active_constraints,
  const CircleEnvelopeSettings & settings, const std::optional<double> & minimum_y,
  double common_extension);

}  // namespace origincar_system::detail

#endif  // ORIGINCAR_SYSTEM_CIRCLE_ENVELOPE_INTERNAL_HPP_
