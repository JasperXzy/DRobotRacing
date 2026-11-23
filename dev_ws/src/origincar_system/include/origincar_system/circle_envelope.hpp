#ifndef ORIGINCAR_SYSTEM_CIRCLE_ENVELOPE_HPP_
#define ORIGINCAR_SYSTEM_CIRCLE_ENVELOPE_HPP_

#include <cstddef>
#include <functional>
#include <optional>
#include <vector>

#include "origincar_system/route_geometry.hpp"

namespace origincar_system
{

struct PermanentObstacle
{
  Point2D center;
  bool handled_by_realtime_avoidance;
};

struct RearObstacleLimit
{
  Point2D origin;
  double yaw;
  std::size_t first_deformable_index;
};

struct CircleEnvelopeSettings
{
  double target_clearance;
  double numeric_margin;
  double min_transition_length;
  double max_transition_length;
  double extension_step;
  double merge_gap;
  double max_curvature;
  int smoothing_iterations;
  double max_cone_radius;
  double cloud_check_radius;
  double min_progress_ratio;
};

struct CircleEnvelopeRequest
{
  std::vector<PermanentObstacle> obstacles;
  CircleEnvelopeSettings settings;
  std::optional<RearObstacleLimit> rear_obstacle_limit;
  std::optional<double> minimum_y;
  bool audit_permanent_obstacles;
};

struct ClearanceRepair
{
  std::size_t round;
  std::size_t constraint_segments;
  std::size_t rejected_candidates;
};

struct CircleEnvelopeReport
{
  bool shaped{false};
  bool accepted{false};
  std::size_t shared_passes{0};
  std::size_t shared_waypoint_pairs{0};
  std::size_t primary_constraints{0};
  std::size_t secondary_constraints{0};
  std::size_t promoted_constraints{0};
  std::size_t expanded_constraints{0};
  std::size_t reinforced_constraints{0};
  std::size_t final_constraints{0};
  std::size_t local_bridges{0};
  std::size_t fairing_accepted{0};
  std::size_t fairing_rejected{0};
  std::size_t clearance_failures{0};
  std::size_t permanent_clearance_failures{0};
  std::size_t anchor_conflicts{0};
  std::size_t overlap_corridor_points{0};
  std::size_t progress_violations{0};
  double maximum_curvature{0.0};
  std::size_t repair_rounds{0};
};

struct CircleEnvelopePlan
{
  std::vector<Waypoint> route;
  std::vector<bool> obstacles_requiring_realtime_avoidance;
  CircleEnvelopeReport report;
};

using ClearanceRepairObserver = std::function<void(const ClearanceRepair &)>;

std::optional<CircleEnvelopePlan> plan_circle_envelope(
  const std::vector<Waypoint> & original_route, const std::vector<WaypointShift> & shifts,
  const CircleEnvelopeRequest & request, const ClearanceRepairObserver & on_repair = {});

}  // namespace origincar_system

#endif  // ORIGINCAR_SYSTEM_CIRCLE_ENVELOPE_HPP_
