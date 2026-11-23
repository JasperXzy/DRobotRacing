#include "origincar_system/route_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace origincar_system
{

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

bool left_normal_at(
  const std::vector<Waypoint> & route, std::size_t index, Point2D & normal)
{
  if (route.size() < 2 || index >= route.size()) {
    return false;
  }

  const std::size_t previous_index = index > 0 ? index - 1 : 0;
  const std::size_t next_index = std::min(
    index + 1, route.size() - 1);
  const double tangent_x =
    route[next_index].x - route[previous_index].x;
  const double tangent_y =
    route[next_index].y - route[previous_index].y;
  const double tangent_length = std::hypot(tangent_x, tangent_y);
  if (tangent_length < 1e-8) {
    return false;
  }
  normal = Point2D{-tangent_y / tangent_length, tangent_x / tangent_length};
  return true;
}

}  // namespace origincar_system
