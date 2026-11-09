// Copyright 2026 diting
// SPDX-License-Identifier: Apache-2.0
#ifndef SENTRY_TUNNEL_BT__EXIT_SPIN_SAFETY_HPP_
#define SENTRY_TUNNEL_BT__EXIT_SPIN_SAFETY_HPP_

#include "pb_omni_pid_pursuit_controller/footprint_collision.hpp"
#include "sentry_tunnel_bt/tunnel_geometry.hpp"
#include <cmath>
#include <limits>

namespace sentry_tunnel_bt {

// All coordinates must already share the costmap frame. Check a rotation disk
// along the untravelled path only: historical tunnel points need not permit
// rotation now that the complete body has left them. Invalid data fail closed.
inline bool remainingSpinPathSafe(const nav2_costmap_2d::Costmap2D &map,
                                  const nav_msgs::msg::Path &route,
                                  double current_x, double current_y,
                                  double radius) {
  if (!std::isfinite(current_x) || !std::isfinite(current_y) ||
      !std::isfinite(radius) || radius <= 0.0 || route.poses.empty() ||
      route.header.frame_id.empty() ||
      !std::isfinite(map.getResolution()) || map.getResolution() <= 0.0 ||
      !std::isfinite(map.getOriginX()) || !std::isfinite(map.getOriginY()) ||
      map.getSizeInCellsX() == 0 || map.getSizeInCellsY() == 0) {
    return false;
  }
  size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < route.poses.size(); ++i) {
    const auto &pose = route.poses[i];
    const auto &p = pose.pose.position;
    const auto &q = pose.pose.orientation;
    if ((!pose.header.frame_id.empty() &&
         pose.header.frame_id != route.header.frame_id) ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w)) {
      return false;
    }
    const double distance = std::hypot(p.x - current_x, p.y - current_y);
    if (!std::isfinite(distance)) {
      return false;
    }
    if (distance < nearest_distance) {
      nearest = i;
      nearest_distance = distance;
    }
  }
  const auto disk = circumscribedRotationFootprint(radius);
  if (pb_omni_pid_pursuit_controller::sweptFootprintCollision(
          map, disk, current_x, current_y, current_x, current_y, 0.0)) {
    return false;
  }
  // Always include the current-to-next connection and exact requested final
  // endpoint, even if the nearest path point is already the final endpoint.
  const size_t first = std::min(nearest + 1, route.poses.size() - 1);
  double x = current_x, y = current_y;
  for (size_t i = first; i < route.poses.size(); ++i) {
    const auto &point = route.poses[i].pose.position;
    if (pb_omni_pid_pursuit_controller::sweptFootprintCollision(
            map, disk, x, y, point.x, point.y, 0.0)) {
      return false;
    }
    x = point.x;
    y = point.y;
  }
  return true;
}

// The rotation envelope must clear the selected tunnel's exit by 5 cm before
// enabling spin; the remaining global route must also admit every orientation.
inline bool exitSpinSafe(const nav2_costmap_2d::Costmap2D &map,
                         const nav_msgs::msg::Path &route,
                         const Tunnel &tunnel, int direction,
                         double current_x, double current_y, double radius) {
  if ((direction != 1 && direction != -1) ||
      !std::isfinite(tunnel.x_min) || !std::isfinite(tunnel.x_max) ||
      !std::isfinite(tunnel.y) || !std::isfinite(tunnel.half_width) ||
      tunnel.x_min >= tunnel.x_max || tunnel.half_width <= 0.0 ||
      !std::isfinite(current_x) || !std::isfinite(current_y) ||
      !std::isfinite(radius) || radius <= 0.0) {
    return false;
  }
  const double boundary = direction > 0 ? tunnel.x_max : tunnel.x_min;
  const double clearance = direction * (current_x - boundary);
  if (!std::isfinite(clearance) || clearance + 1e-9 < radius + 0.05) {
    return false;
  }
  return remainingSpinPathSafe(map, route, current_x, current_y, radius);
}
} // namespace sentry_tunnel_bt
#endif // SENTRY_TUNNEL_BT__EXIT_SPIN_SAFETY_HPP_
