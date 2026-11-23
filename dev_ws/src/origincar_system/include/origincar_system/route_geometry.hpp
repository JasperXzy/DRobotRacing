#ifndef ORIGINCAR_SYSTEM_ROUTE_GEOMETRY_HPP_
#define ORIGINCAR_SYSTEM_ROUTE_GEOMETRY_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace origincar_system
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Waypoint
{
  std::string name;
  double x{0.0};
  double y{0.0};
  bool reverse{false};
  bool task3_start{false};
  double pass_radius{0.0};
  double reverse_pass_radius{0.0};
};

struct WaypointShift
{
  double offset;
  Point2D world_direction;
  std::size_t source_index;
  Point2D obstacle_center;
  double forbidden_radius;
};

bool left_normal_at(const std::vector<Waypoint> & route, std::size_t index, Point2D & normal);

struct CircleIntersectionChord
{
  double first_x{0.0};
  double first_y{0.0};
  double second_x{0.0};
  double second_y{0.0};
};

bool circle_intersection_chord(
  double first_x, double first_y, double first_radius,
  double second_x, double second_y, double second_radius,
  CircleIntersectionChord & chord);

bool normal_line_intersects_chord(
  double base_x, double base_y, double normal_x, double normal_y,
  const CircleIntersectionChord & chord, double & normal_offset, double & chord_ratio);

bool point_near_chord_segment(
  double point_x, double point_y, const CircleIntersectionChord & chord, double tolerance);

}  // namespace origincar_system

#endif  // ORIGINCAR_SYSTEM_ROUTE_GEOMETRY_HPP_
