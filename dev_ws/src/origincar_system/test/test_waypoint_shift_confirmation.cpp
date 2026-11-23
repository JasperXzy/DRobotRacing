#include <iostream>
#include <optional>

#include "origincar_system/waypoint_shift_confirmation.hpp"

namespace
{

using origincar_system::WaypointShift;
using origincar_system::WaypointShiftConfirmation;
using origincar_system::WaypointShiftConfirmationSettings;
using origincar_system::WaypointShiftUpdate;
using origincar_system::update_waypoint_shift;

constexpr WaypointShiftConfirmationSettings kSettings{2, 3, 0.04, 6};
int failures = 0;

void expect(bool passed, const char * description)
{
  if (!passed) {
    std::cerr << description << '\n';
    ++failures;
  }
}

void test_confirmation_and_geometry_update()
{
  WaypointShift active{0.0, {}, kSettings.waypoint_count, {}, 0.0};
  WaypointShiftConfirmation state;
  const WaypointShift left{0.2, {0.0, 1.0}, 1, {2.0, 1.0}, 0.42};
  const WaypointShift right{-0.3, {0.0, -1.0}, 1, {2.0, 1.0}, 0.42};

  expect(update_waypoint_shift(active, state, left, nullptr, kSettings) ==
    WaypointShiftUpdate::NONE, "One frame leaves the active offset unchanged");
  expect(update_waypoint_shift(active, state, right, nullptr, kSettings) ==
    WaypointShiftUpdate::NONE && state.confirm_count == 1,
    "Changing sides restarts the confirmation count");
  expect(update_waypoint_shift(active, state, right, nullptr, kSettings) ==
    WaypointShiftUpdate::CONFIRMED && active.offset == right.offset,
    "Consecutive same-side frames confirm the offset");

  WaypointShift moved = right;
  moved.offset = -0.31;
  moved.obstacle_center.x = 2.1;
  expect(update_waypoint_shift(active, state, moved, nullptr, kSettings) ==
    WaypointShiftUpdate::GEOMETRY_CHANGED &&
    active.offset == right.offset && active.obstacle_center.x == moved.obstacle_center.x,
    "An obstacle location change updates metadata while retaining the confirmed offset");
}

void test_release_to_frozen_route()
{
  const WaypointShift frozen{0.15, {0.0, 1.0}, 1, {2.0, 1.0}, 0.42};
  WaypointShift active{0.3, {0.0, 1.0}, 4, {4.0, 2.0}, 0.45};
  WaypointShiftConfirmation state{0.3, 3, 0};

  for (int scan = 1; scan < kSettings.release_scans; ++scan) {
    expect(update_waypoint_shift(active, state, std::nullopt, &frozen, kSettings) ==
      WaypointShiftUpdate::NONE && active.offset == 0.3 && state.confirm_count == 0,
      "A short loss resets confirmation while retaining the active offset");
  }
  expect(update_waypoint_shift(active, state, std::nullopt, &frozen, kSettings) ==
    WaypointShiftUpdate::RELEASED && active.offset == frozen.offset &&
    active.source_index == frozen.source_index && active.forbidden_radius == frozen.forbidden_radius,
    "The release threshold restores the frozen geometry");
  expect(update_waypoint_shift(active, state, std::nullopt, &frozen, kSettings) ==
    WaypointShiftUpdate::NONE, "A released waypoint stays stable while the obstacle is absent");
  expect(update_waypoint_shift(active, state, std::nullopt, nullptr, kSettings) ==
    WaypointShiftUpdate::RELEASED && active.offset == 0.0 &&
    active.source_index == kSettings.waypoint_count,
    "Without a frozen base, release returns to the original route");
}

}  // namespace

int main()
{
  test_confirmation_and_geometry_update();
  test_release_to_frozen_route();
  return failures == 0 ? 0 : 1;
}
