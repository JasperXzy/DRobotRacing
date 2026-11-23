#ifndef ORIGINCAR_SYSTEM_WAYPOINT_SHIFT_CONFIRMATION_HPP_
#define ORIGINCAR_SYSTEM_WAYPOINT_SHIFT_CONFIRMATION_HPP_

#include <cstddef>
#include <optional>

#include "origincar_system/route_geometry.hpp"

namespace origincar_system
{

struct WaypointShiftConfirmation
{
  double pending_offset{0.0};
  int confirm_count{0};
  int missing_count{0};
};

struct WaypointShiftConfirmationSettings
{
  int confirm_scans;
  int release_scans;
  double min_change;
  std::size_t waypoint_count;
};

enum class WaypointShiftUpdate
{
  NONE,
  CONFIRMED,
  GEOMETRY_CHANGED,
  RELEASED
};

WaypointShiftUpdate update_waypoint_shift(
  WaypointShift & shift, WaypointShiftConfirmation & confirmation,
  const std::optional<WaypointShift> & observation, const WaypointShift * frozen,
  const WaypointShiftConfirmationSettings & settings);

}  // namespace origincar_system

#endif  // ORIGINCAR_SYSTEM_WAYPOINT_SHIFT_CONFIRMATION_HPP_
