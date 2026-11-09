// Copyright 2026 diting
// SPDX-License-Identifier: Apache-2.0
#ifndef SENTRY_TUNNEL_BT__CONTINUOUS_ROUTE_HPP_
#define SENTRY_TUNNEL_BT__CONTINUOUS_ROUTE_HPP_

#include "sentry_tunnel_bt/tunnel_geometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace sentry_tunnel_bt {
namespace continuous_route_detail {
inline bool finitePose(const geometry_msgs::msg::PoseStamped &pose) {
  const auto &p = pose.pose.position;
  const auto &q = pose.pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w);
}
inline double distance(const geometry_msgs::msg::PoseStamped &a,
                       const geometry_msgs::msg::PoseStamped &b) {
  return std::hypot(std::hypot(a.pose.position.x - b.pose.position.x,
                               a.pose.position.y - b.pose.position.y),
                    a.pose.position.z - b.pose.position.z);
}
inline bool sameOrientation(const geometry_msgs::msg::PoseStamped &a,
                            const geometry_msgs::msg::PoseStamped &b) {
  const auto &p = a.pose.orientation;
  const auto &q = b.pose.orientation;
  // Opposite quaternion signs represent the same requested orientation.
  const double direct = std::hypot(std::hypot(p.x - q.x, p.y - q.y),
                                   std::hypot(p.z - q.z, p.w - q.w));
  const double opposite = std::hypot(std::hypot(p.x + q.x, p.y + q.y),
                                     std::hypot(p.z + q.z, p.w + q.w));
  return std::min(direct, opposite) <= 1e-6;
}
inline bool validPath(const nav_msgs::msg::Path &path) {
  return !path.poses.empty() && !path.header.frame_id.empty() &&
         std::all_of(path.poses.begin(), path.poses.end(),
                     [&](const auto &pose) {
                       return finitePose(pose) &&
                              (pose.header.frame_id.empty() ||
                               pose.header.frame_id == path.header.frame_id);
                     });
}
} // namespace continuous_route_detail

// Eligibility only: observed passage and full-body collision checks are still
// required before consuming any waypoint or commanding the joined route.
inline bool continuousThreePointRoute(
    const std::vector<geometry_msgs::msg::PoseStamped> &goals,
    const Tunnel &tunnel, int direction, double entry_x,
    double lateral_tolerance, double longitudinal_extent) {
  if (goals.size() != 3 || (direction != 1 && direction != -1) ||
      !std::isfinite(tunnel.x_min) || !std::isfinite(tunnel.x_max) ||
      !std::isfinite(tunnel.y) || !std::isfinite(tunnel.half_width) ||
      !(tunnel.x_min < tunnel.x_max) || !(tunnel.half_width > 0.0) ||
      !std::isfinite(entry_x) || !std::isfinite(lateral_tolerance) ||
      !(lateral_tolerance > 0.0) || !std::isfinite(longitudinal_extent) ||
      !(longitudinal_extent > 0.0)) {
    return false;
  }
  const auto &frame = goals.front().header.frame_id;
  if (frame.empty() ||
      !std::all_of(goals.begin(), goals.end(), [&](const auto &pose) {
        return continuous_route_detail::finitePose(pose) &&
               pose.header.frame_id == frame;
      })) {
    return false;
  }
  const double entrance = direction > 0 ? tunnel.x_min : tunnel.x_max;
  const double exit = direction > 0 ? tunnel.x_max : tunnel.x_min;
  const auto &first = goals[0].pose.position;
  const auto &middle = goals[1].pose.position;
  const auto &last = goals[2].pose.position;
  return direction * (entrance - entry_x) > 0.0 &&
         std::hypot(first.x - entry_x, first.y - tunnel.y) <= 0.15 &&
         middle.x > tunnel.x_min && middle.x < tunnel.x_max &&
         std::abs(middle.y - tunnel.y) <= lateral_tolerance &&
         direction * (last.x - exit) >= longitudinal_extent + 0.5;
}

// Continuous-entry preparation deliberately aims slightly upstream while
// lateral error converges. Once heading and lateral proximity permit it, keep
// a bounded forward component instead of braking to a full intermediate stop.
// Eligibility, actual entry tolerances and the rotation-disk sweep remain the
// caller's responsibility. The ordinary stopped approach keeps its forecast.
inline CenteringCommand
continuousApproachCommand(double x, double y, double entry_x, double center_y,
                          double maximum_speed, int direction,
                          double lateral_tolerance,
                          double longitudinal_tolerance, double heading_error) {
  if (!std::isfinite(heading_error)) {
    throw std::invalid_argument("Invalid continuous approach heading");
  }
  // Preserve the originally verified straight corridor until its final
  // 0.60 m; steering toward the upstream point earlier can cut a field corner.
  const bool near_entry = std::hypot(entry_x - x, center_y - y) <= 0.60;
  const double target_x = near_entry ? entry_x - direction * 0.20 : entry_x;
  auto command = shortApproachCommand(x, y, target_x, center_y, maximum_speed,
                                      direction, lateral_tolerance,
                                      longitudinal_tolerance, 0.0, 0.0, 0.0);
  // Once upstream alignment has been passed, do not pull the body back
  // toward that preparation target during the last yaw correction.
  if (near_entry && direction * (x - target_x) > 0.0) {
    command.vx = direction * std::max(0.0, direction * command.vx);
  }
  // The upstream target can already be behind us while yaw is still settling.
  // Removing its backward component must not magnify a tiny, already accepted
  // lateral residual to the effective command floor below.
  if (near_entry && std::abs(y - center_y) <= lateral_tolerance + 1e-9) {
    command.vy = 0.0;
  }
  if (near_entry && std::abs(heading_error) <= kRotationStopTolerance &&
      std::abs(y - center_y) <= 0.04 + 1e-9 &&
      direction * (entry_x - x) > 0.10 + 1e-9) {
    command.vx = direction * std::max(0.25, direction * command.vx);
  }
  const double speed = std::hypot(command.vx, command.vy);
  if (!std::isfinite(speed)) {
    throw std::invalid_argument("Continuous approach speed overflow");
  }
  if (speed < 1e-9) {
    return {};
  }
  // Projection can remove a backward component. Keep the proven effective
  // command floor for the remaining lateral correction rather than stalling.
  const double bounded_speed = std::clamp(speed, std::min(0.12, maximum_speed),
                                          maximum_speed);
  if (std::abs(bounded_speed - speed) > 1e-12) {
    const double scale = bounded_speed / speed;
    command.vx *= scale;
    command.vy *= scale;
  }
  command.stop = false;
  return command;
}

inline double continuousExitDistance(double requested_exit,
                                     double longitudinal_extent,
                                     double max_lookahead) {
  if (!std::isfinite(requested_exit) || !(requested_exit > 0.0) ||
      !std::isfinite(longitudinal_extent) || !(longitudinal_extent > 0.0) ||
      !std::isfinite(max_lookahead) || max_lookahead < 0.0) {
    throw std::invalid_argument("Invalid continuous-passage clearance");
  }
  const double clearance = longitudinal_extent + 0.01 + max_lookahead + 0.15;
  if (!std::isfinite(clearance)) {
    throw std::invalid_argument("Continuous-passage clearance overflow");
  }
  return std::max(requested_exit, clearance);
}

// Join only already validated paths. A discontinuity is rejected rather than
// silently bridged, and the exact requested final pose remains the endpoint.
inline nav_msgs::msg::Path
appendContinuousTail(const nav_msgs::msg::Path &axis,
                     const nav_msgs::msg::Path &tail,
                     const geometry_msgs::msg::PoseStamped &final_goal) {
  using namespace continuous_route_detail;
  if (!validPath(axis) || !validPath(tail) || !finitePose(final_goal) ||
      axis.header.frame_id != tail.header.frame_id ||
      final_goal.header.frame_id != axis.header.frame_id) {
    throw std::invalid_argument("Continuous route has invalid data or frames");
  }
  if (distance(axis.poses.back(), tail.poses.front()) > 0.05) {
    throw std::invalid_argument("Continuous route has an unvalidated seam");
  }
  if (distance(tail.poses.back(), final_goal) > 1e-6 ||
      !sameOrientation(tail.poses.back(), final_goal)) {
    throw std::invalid_argument("Continuous tail changes the requested goal");
  }
  auto result = axis;
  for (const auto &pose : tail.poses) {
    if (distance(result.poses.back(), pose) > 1e-9) {
      result.poses.push_back(pose);
    }
  }
  result.poses.back() = final_goal;
  return result;
}
} // namespace sentry_tunnel_bt
#endif // SENTRY_TUNNEL_BT__CONTINUOUS_ROUTE_HPP_
