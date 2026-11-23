#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>

#include "origincar_system/circle_envelope.hpp"
#include "origincar_system/route_geometry.hpp"

namespace
{

using origincar_system::CircleEnvelopeRequest;
using origincar_system::CircleEnvelopeSettings;
using origincar_system::Point2D;
using origincar_system::Waypoint;
using origincar_system::WaypointShift;
using origincar_system::plan_circle_envelope;

constexpr CircleEnvelopeSettings kSettings{
  0.35, 0.07, 0.90, 1.80, 0.05, 0.10, 2.40, 8, 0.065, 0.25, 0.15};
constexpr std::size_t kObstacleIndex = 100;
constexpr double kRadius = kSettings.target_clearance + kSettings.numeric_margin;
int failures = 0;

void expect(bool passed, const char * description)
{
  if (!passed) {
    std::cerr << description << '\n';
    ++failures;
  }
}

std::vector<Waypoint> straight_route()
{
  std::vector<Waypoint> route(201);
  for (std::size_t index = 0; index < route.size(); ++index) {
    route[index].x = static_cast<double>(index) * 0.02;
  }
  route[kObstacleIndex].name = "obstacle_anchor";
  route[kObstacleIndex].pass_radius = 0.12;
  route.back().task3_start = true;
  return route;
}

double minimum_clearance(const std::vector<Waypoint> & route, Point2D obstacle)
{
  double clearance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1; index < route.size(); ++index) {
    const auto & first = route[index - 1];
    const auto & second = route[index];
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double squared_length = dx * dx + dy * dy;
    double progress = 0.0;
    if (squared_length > 0.0) {
      progress = std::clamp(
        ((obstacle.x - first.x) * dx + (obstacle.y - first.y) * dy) / squared_length,
        0.0, 1.0);
    }
    clearance = std::min(clearance, std::hypot(
      first.x + progress * dx - obstacle.x, first.y + progress * dy - obstacle.y));
  }
  return clearance;
}

void test_circle_clearance_and_metadata()
{
  const auto route = straight_route();
  const Point2D obstacle{route[kObstacleIndex].x, 0.0};
  std::vector<WaypointShift> shifts(route.size(), {0.0, {}, route.size(), {}, 0.0});
  shifts[kObstacleIndex] = {kRadius, {0.0, 1.0}, kObstacleIndex, obstacle, kRadius};
  const CircleEnvelopeRequest request{{{obstacle, false}}, kSettings, {}, {}, true};

  const auto plan = plan_circle_envelope(route, shifts, request);
  expect(plan && plan->report.shaped, "An observed obstacle should produce a detour");
  if (!plan || plan->route.size() != route.size()) {
    expect(false, "Route shaping must preserve route length");
    return;
  }
  expect(plan->route[kObstacleIndex].y >= kSettings.target_clearance, "Keep chosen detour side");
  expect(
    minimum_clearance(plan->route, obstacle) >= kSettings.target_clearance - 1.0e-7,
    "Every output segment should clear the obstacle");
  expect(
    plan->obstacles_requiring_realtime_avoidance == std::vector<bool>{false},
    "A clear obstacle stays under static planning");
  expect(
    plan->route[kObstacleIndex].name == route[kObstacleIndex].name &&
    plan->route[kObstacleIndex].pass_radius == route[kObstacleIndex].pass_radius &&
    plan->route.back().task3_start && route[kObstacleIndex].y == 0.0,
    "Preserve waypoint metadata and original route");
}

void test_rear_limit_and_invalid_input()
{
  const auto route = straight_route();
  const Point2D obstacle{route[kObstacleIndex].x, 0.0};
  std::vector<WaypointShift> shifts(route.size(), {0.0, {}, route.size(), {}, 0.0});
  shifts[kObstacleIndex] = {kRadius, {0.0, 1.0}, kObstacleIndex, obstacle, kRadius};
  const CircleEnvelopeRequest request{
    {{obstacle, false}}, kSettings,
    origincar_system::RearObstacleLimit{{obstacle.x + 0.2, 0.0}, 0.0, kObstacleIndex + 1},
    std::nullopt, true};

  const auto plan = plan_circle_envelope(route, shifts, request);
  expect(plan && !plan->report.shaped, "An excluded rear anchor should leave the route in place");
  expect(
    plan && plan->route.size() == route.size() &&
    plan->route[kObstacleIndex].y == route[kObstacleIndex].y,
    "The original route should be returned when no constraints apply");
  shifts.pop_back();
  expect(!plan_circle_envelope(route, shifts, request), "Reject incomplete waypoint metadata");
}

void test_common_chord()
{
  origincar_system::CircleIntersectionChord chord;
  expect(
    origincar_system::circle_intersection_chord(0.0, 0.0, 1.0, 1.0, 0.0, 1.0, chord),
    "Overlapping safety circles share a chord");
  double offset = 0.0;
  double ratio = 0.0;
  expect(
    origincar_system::normal_line_intersects_chord(0.0, 0.0, 1.0, 0.0, chord, offset, ratio) &&
    std::abs(offset - 0.5) < 1.0e-9 && std::abs(ratio - 0.5) < 1.0e-9,
    "The route normal crosses the center of the shared chord");
  expect(
    !origincar_system::circle_intersection_chord(0.0, 0.0, 1.0, 2.0, 0.0, 1.0, chord),
    "Tangent circles do not form a shared corridor");
}

}  // namespace

int main()
{
  test_circle_clearance_and_metadata();
  test_rear_limit_and_invalid_input();
  test_common_chord();
  return failures == 0 ? 0 : 1;
}
