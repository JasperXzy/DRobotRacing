#include "origincar_system/waypoint_shift_confirmation.hpp"

#include <algorithm>
#include <cmath>

namespace origincar_system
{
namespace
{

bool observed_geometry_changed(
  const WaypointShift & current, const WaypointShift & observed,
  const WaypointShiftConfirmationSettings & settings)
{
  if (current.source_index >= settings.waypoint_count) {
    return true;
  }
  const std::size_t source_delta = current.source_index > observed.source_index ?
    current.source_index - observed.source_index : observed.source_index - current.source_index;
  const double direction_alignment =
    current.world_direction.x * observed.world_direction.x +
    current.world_direction.y * observed.world_direction.y;
  const double center_change = std::hypot(
    current.obstacle_center.x - observed.obstacle_center.x,
    current.obstacle_center.y - observed.obstacle_center.y);
  const double radius_change = std::abs(current.forbidden_radius - observed.forbidden_radius);
  // 0.02m航点下允许来源点在2点内轻微抖动；方向变化约2.6度以上才重建。
  const double geometry_change_threshold = std::max(0.005, 0.5 * settings.min_change);
  return source_delta > 2 || direction_alignment < 0.999 ||
         center_change >= geometry_change_threshold ||
         radius_change >= geometry_change_threshold;
}

}  // namespace

WaypointShiftUpdate update_waypoint_shift(
  WaypointShift & shift, WaypointShiftConfirmation & confirmation,
  const std::optional<WaypointShift> & observation, const WaypointShift * frozen,
  const WaypointShiftConfirmationSettings & settings)
{
  if (observation) {
    confirmation.missing_count = 0;
    const bool same_direction = confirmation.confirm_count > 0 &&
      confirmation.pending_offset * observation->offset > 0.0;
    confirmation.pending_offset = observation->offset;
    confirmation.confirm_count = same_direction ? confirmation.confirm_count + 1 : 1;

    if (confirmation.confirm_count < settings.confirm_scans) {
      return WaypointShiftUpdate::NONE;
    }
    if (std::abs(observation->offset - shift.offset) >= settings.min_change) {
      shift = *observation;
      return WaypointShiftUpdate::CONFIRMED;
    }
    if (std::abs(shift.offset) > 1e-9 && observed_geometry_changed(shift, *observation, settings)) {
      const double retained_offset = shift.offset;
      shift = *observation;
      shift.offset = retained_offset;
      return WaypointShiftUpdate::GEOMETRY_CHANGED;
    }
    return WaypointShiftUpdate::NONE;
  }

  confirmation.pending_offset = 0.0;
  confirmation.confirm_count = 0;
  ++confirmation.missing_count;
  if (confirmation.missing_count < settings.release_scans) {
    return WaypointShiftUpdate::NONE;
  }

  WaypointShift fallback{0.0, {}, settings.waypoint_count, {}, 0.0};
  if (frozen) {
    fallback.offset = frozen->offset;
    if (std::abs(frozen->offset) > 1e-9) {
      fallback = *frozen;
    }
  }
  const bool release_changed =
    std::abs(shift.offset - fallback.offset) > 1e-9 ||
    shift.source_index != fallback.source_index ||
    std::hypot(
      shift.obstacle_center.x - fallback.obstacle_center.x,
      shift.obstacle_center.y - fallback.obstacle_center.y) > 1e-9 ||
    std::abs(shift.forbidden_radius - fallback.forbidden_radius) > 1e-9;
  if (release_changed) {
    shift = fallback;
    return WaypointShiftUpdate::RELEASED;
  }
  return WaypointShiftUpdate::NONE;
}

}  // namespace origincar_system
