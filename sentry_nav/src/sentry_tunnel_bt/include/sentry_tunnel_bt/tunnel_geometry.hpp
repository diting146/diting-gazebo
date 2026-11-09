// Copyright 2026 diting
// SPDX-License-Identifier: Apache-2.0
#ifndef SENTRY_TUNNEL_BT__TUNNEL_GEOMETRY_HPP_
#define SENTRY_TUNNEL_BT__TUNNEL_GEOMETRY_HPP_
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>
namespace sentry_tunnel_bt {
constexpr int64_t kInputFutureToleranceNs = 50000000;
// Keep the newest scan in an epoch without extending freshness for backlog.
// A ROS-clock reset starts a new epoch; future messages from the old epoch must
// not poison its lower timestamp range.
struct ScanStampGate {
  int64_t last_clock_ns{0}, last_stamp_ns{0};
  bool accept(int64_t stamp_ns, int64_t now_ns) {
    if (now_ns < last_clock_ns) {
      last_stamp_ns = 0;
    }
    last_clock_ns = now_ns;
    if (stamp_ns <= 0 || stamp_ns > now_ns + kInputFutureToleranceNs ||
        stamp_ns <= last_stamp_ns) {
      return false;
    }
    last_stamp_ns = stamp_ns;
    return true;
  }
};
struct Tunnel {
  double x_min, x_max, y;
  double half_width{0.4};
};
struct Crossing {
  int tunnel{-1};
  int direction{0};
  double order{0.0};
};
inline double wrap(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}
// Only a complete entrance-to-exit intersection activates the special sequence.
// A detour, merely nearby path, or goal inside does not force a passage.
inline Crossing firstCrossing(const nav_msgs::msg::Path &path,
                              const std::vector<Tunnel> &tunnels,
                              const std::set<int> &visited = {}) {
  Crossing best;
  best.order = std::numeric_limits<double>::infinity();
  for (size_t t = 0; t < tunnels.size(); ++t) {
    if (visited.count(static_cast<int>(t))) {
      continue;
    }
    const auto &region = tunnels[t];
    for (const int direction : {1, -1}) {
      const double entry = direction > 0 ? region.x_min : region.x_max;
      const double exit = direction > 0 ? region.x_max : region.x_min;
      double entered = -1.0;
      for (size_t i = 1; i < path.poses.size(); ++i) {
        const auto &a = path.poses[i - 1].pose.position;
        const auto &b = path.poses[i].pose.position;
        if (!std::isfinite(a.x + a.y + b.x + b.y)) {
          break;
        }
        const double dx = b.x - a.x;
        auto at = [&](double x) {
          if (dx * direction <= 1e-9) {
            return -1.0;
          }
          const double u = (x - a.x) / dx;
          if (u < 0.0 || u > 1.0 ||
              std::abs(a.y + u * (b.y - a.y) - region.y) >= region.half_width) {
            return -1.0;
          }
          return static_cast<double>(i - 1) + u;
        };
        const double in = at(entry);
        if (in >= 0.0 && entered < 0.0) {
          entered = in;
        }
        if (entered >= 0.0) {
          // Reject a path which leaves the tunnel sideways before crossing
          // exit.
          if (a.x > region.x_min && a.x < region.x_max &&
              std::abs(a.y - region.y) >= region.half_width) {
            entered = -1.0;
          }
          const double out = at(exit);
          if (out > entered && entered >= 0.0) {
            if (entered < best.order) {
              best = {static_cast<int>(t), direction, entered};
            }
            break;
          }
        }
      }
    }
  }
  return best;
}
// Ordinary following may not enter even part of a tunnel. Full crossings are
// handled by TunnelPassage; a partial crossing has no safe preparation contract.
inline bool pathTouchesTunnel(const nav_msgs::msg::Path &path,
                              const std::vector<Tunnel> &tunnels) {
  if (path.poses.empty()) {
    return true;
  }
  for (const auto &tunnel : tunnels) {
    if (!std::isfinite(tunnel.x_min + tunnel.x_max + tunnel.y + tunnel.half_width) ||
        tunnel.x_min >= tunnel.x_max || tunnel.half_width <= 0.0) {
      return true;
    }
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto &a = path.poses[i == 0 ? 0 : i - 1].pose.position;
      const auto &b = path.poses[i].pose.position;
      if (!std::isfinite(a.x + a.y + b.x + b.y)) {
        return true;
      }
      double first = 0.0, last = 1.0;
      const auto clips = [&](double start, double delta, double low, double high) {
        if (std::abs(delta) <= 1e-12) {
          return start >= low && start <= high;
        }
        const double u = (low - start) / delta, v = (high - start) / delta;
        first = std::max(first, std::min(u, v));
        last = std::min(last, std::max(u, v));
        return first <= last;
      };
      if (clips(a.x, b.x - a.x, tunnel.x_min, tunnel.x_max) &&
          clips(a.y, b.y - a.y, tunnel.y - tunnel.half_width,
                tunnel.y + tunnel.half_width)) {
        return true;
      }
    }
  }
  return false;
}
// An outside waiting point needs precise lateral alignment, not an exact
// longitudinal stop. Positive outside_error means farther from the entrance.
inline bool waitingPositionAcceptable(double x, double y, double entry_x,
                                      double center_y, int direction,
                                      double lateral_tolerance,
                                      double longitudinal_tolerance) {
  if (!std::isfinite(x + y + entry_x + center_y + lateral_tolerance +
                     longitudinal_tolerance) ||
      (direction != 1 && direction != -1) || lateral_tolerance <= 0.0 ||
      longitudinal_tolerance < lateral_tolerance) {
    throw std::invalid_argument("Invalid waiting-position geometry");
  }
  const double outside_error = direction * (entry_x - x);
  constexpr double epsilon = 1e-9;
  return std::abs(center_y - y) <= lateral_tolerance + epsilon &&
         outside_error >= -lateral_tolerance - epsilon &&
         outside_error <= longitudinal_tolerance + epsilon;
}
// Once the waiting-point slowdown zone has been entered, do not accelerate
// again because localization or the path moves just outside its boundary.
inline bool approachSlowdownRequired(double distance, double slowdown_distance,
                                     bool already_slowed) {
  if (!std::isfinite(distance) || distance < 0.0 ||
      !std::isfinite(slowdown_distance) || slowdown_distance <= 0.0) {
    throw std::invalid_argument("Invalid approach slowdown distance");
  }
  return already_slowed || distance <= slowdown_distance;
}
inline bool validCenteringSettings(double lateral_tolerance,
                                   double longitudinal_tolerance,
                                   double approach_speed,
                                   double centering_speed) {
  return std::isfinite(lateral_tolerance + longitudinal_tolerance +
                       approach_speed + centering_speed) &&
         lateral_tolerance > 0.0 && lateral_tolerance <= 0.025 &&
         longitudinal_tolerance >= lateral_tolerance &&
         longitudinal_tolerance <= 0.30 && approach_speed > 0.0 &&
         centering_speed > 0.0 && centering_speed <= approach_speed &&
         centering_speed <= 0.30;
}
struct CenteringCommand {
  double vx{0.0}, vy{0.0};
  bool stop{true};
};
// A bounded continuous correction with a braking forecast. Acceptance is still
// rechecked by the caller against a fresh pose after an actual full stop.
inline CenteringCommand
centeringCommand(double x, double y, double entry_x, double center_y,
                 int direction, double lateral_tolerance,
                 double longitudinal_tolerance, double maximum_speed,
                 double observed_vx, double observed_vy, double forecast_time) {
  if (!validCenteringSettings(lateral_tolerance, longitudinal_tolerance,
                              maximum_speed, maximum_speed) ||
      !std::isfinite(observed_vx + observed_vy + forecast_time) ||
      forecast_time < 0.0 || forecast_time > 0.5) {
    throw std::invalid_argument("Invalid continuous centering settings");
  }
  const double px = x + observed_vx * forecast_time,
               py = y + observed_vy * forecast_time;
  if (waitingPositionAcceptable(x, y, entry_x, center_y, direction,
                                lateral_tolerance, longitudinal_tolerance) ||
      waitingPositionAcceptable(px, py, entry_x, center_y, direction,
                                lateral_tolerance, longitudinal_tolerance)) {
    return {};
  }
  const double outside_error = direction * (entry_x - x);
  const bool waiting_band = outside_error >= -lateral_tolerance &&
                            outside_error <= longitudinal_tolerance;
  const double dx = waiting_band ? 0.0 : entry_x - px, dy = center_y - py;
  const double distance = std::hypot(dx, dy);
  if (distance < 1e-9) {
    return {};
  }
  // Stop rather than reverse while the measured motion is still carrying the
  // body across the target. A stopped-pose recheck decides any next correction.
  if ((center_y - y) * (center_y - py) <= 0.0 &&
      (center_y - y) * observed_vy > 0.0) {
    return {};
  }
  const double speed = std::min(maximum_speed, std::max(0.12, 2.0 * distance));
  return {speed * dx / distance, speed * dy / distance, false};
}
// Circumscribed polygon contains the entire disk between vertices as well.
inline std::vector<geometry_msgs::msg::Point>
circumscribedRotationFootprint(double radius) {
  if (!std::isfinite(radius) || radius <= 0.0) {
    throw std::invalid_argument("Invalid rotation footprint radius");
  }
  constexpr int sides = 32;
  const double outer = radius / std::cos(M_PI / sides);
  std::vector<geometry_msgs::msg::Point> result;
  for (int i = 0; i < sides; ++i) {
    geometry_msgs::msg::Point p;
    p.x = outer * std::cos(2.0 * M_PI * i / sides);
    p.y = outer * std::sin(2.0 * M_PI * i / sides);
    result.push_back(p);
  }
  return result;
}
// Follow the verified outside corridor until the actual entrance waiting band
// is reached. Forecast only braking; do not steer away from the verified line.
inline CenteringCommand shortApproachCommand(
    double x, double y, double goal_x, double goal_y, double maximum_speed,
    int direction = 1, double lateral_tolerance = 0.015,
    double longitudinal_tolerance = 0.30, double observed_vx = 0.0,
    double observed_vy = 0.0, double forecast_time = 0.0) {
  if (!std::isfinite(x + y + goal_x + goal_y + maximum_speed + observed_vx +
                     observed_vy + forecast_time) ||
      maximum_speed <= 0.0 || maximum_speed > 1.0 ||
      lateral_tolerance <= 0.0 || lateral_tolerance > 0.025 ||
      longitudinal_tolerance > 0.30 || forecast_time < 0.0 ||
      forecast_time > 0.5) {
    throw std::invalid_argument("Invalid short approach command");
  }
  if (waitingPositionAcceptable(x, y, goal_x, goal_y, direction,
                                lateral_tolerance, longitudinal_tolerance)) {
    return {};
  }
  const double dx = goal_x - x, dy = goal_y - y;
  const double distance = std::hypot(dx, dy);
  const double closing_speed = (dx * observed_vx + dy * observed_vy) / distance;
  const double predicted_distance = distance - closing_speed * forecast_time;
  const double px = x + observed_vx * forecast_time,
               py = y + observed_vy * forecast_time;
  if (closing_speed > 0.0 &&
      (predicted_distance <= 0.0 ||
       waitingPositionAcceptable(px, py, goal_x, goal_y, direction,
                                 lateral_tolerance, longitudinal_tolerance))) {
    return {}; // The caller keeps checking the actual pose while braking.
  }
  // Match the proven centering floor: smaller commands may not move the
  // simulated chassis. Forecast braking and the caller's full rotation-disk
  // sweep still guard the approach rather than creeping indefinitely.
  const double speed = std::min(
      maximum_speed, std::max(0.12, 2.0 * std::max(0.0, predicted_distance)));
  return {speed * dx / distance, speed * dy / distance, false};
}
constexpr double kRotationStopTolerance = 0.003490658504; // 0.2 degree
// Use the measured yaw rate and TF age to brake before delayed feedback crosses
// the target. Keep the configured coarse speed, then make low-rate corrections.
inline double preparationAngularCommand(double error, double observed_rate,
                                        double forecast_time,
                                        double maximum_speed) {
  if (!std::isfinite(error + observed_rate + forecast_time + maximum_speed) ||
      forecast_time < 0.0 || forecast_time > 0.4 || maximum_speed <= 0.0 ||
      maximum_speed > 0.6) {
    throw std::invalid_argument("Invalid preparation angular command settings");
  }
  if (std::abs(error) <= kRotationStopTolerance) {
    return 0.0;
  }
  const double predicted_error = error - observed_rate * forecast_time;
  if (error * observed_rate > 0.0 &&
      (std::abs(predicted_error) <= kRotationStopTolerance ||
       error * predicted_error <= 0.0)) {
    return 0.0;
  }
  return std::copysign(std::clamp(1.5 * std::abs(error),
                                  std::min(0.1, maximum_speed), maximum_speed),
                       error);
}
// Consume only an actually observed, ordered waypoint prefix. Never consume the
// final goal. Large gaps are not interpolated as proof of having visited a
// point.
inline size_t
passedGoalPrefix(const std::vector<geometry_msgs::msg::PoseStamped> &goals,
                 const nav_msgs::msg::Path &trace, double radius) {
  if (!std::isfinite(radius) || radius <= 0.0) {
    throw std::invalid_argument("Invalid waypoint pass radius");
  }
  if (goals.size() < 2 || trace.poses.empty()) {
    return 0;
  }
  double cursor = 0.0;
  size_t passed = 0;
  for (; passed + 1 < goals.size(); ++passed) {
    const auto &goal = goals[passed];
    if (goal.header.frame_id != trace.header.frame_id ||
        !std::isfinite(goal.pose.position.x + goal.pose.position.y)) {
      break;
    }
    bool found = false;
    for (size_t i = static_cast<size_t>(cursor); i < trace.poses.size(); ++i) {
      const auto &a = trace.poses[i].pose.position;
      const auto &b =
          trace.poses[std::min(i + 1, trace.poses.size() - 1)].pose.position;
      if (!std::isfinite(a.x + a.y + b.x + b.y)) {
        continue;
      }
      const double dx = b.x - a.x, dy = b.y - a.y, length2 = dx * dx + dy * dy;
      double u = length2 > 1e-12
                     ? std::clamp(((goal.pose.position.x - a.x) * dx +
                                   (goal.pose.position.y - a.y) * dy) /
                                      length2,
                                  0.0, 1.0)
                     : 0.0;
      if (length2 > 0.25) {
        u = 0.0;
      }
      u = std::max(u, std::max(0.0, cursor - static_cast<double>(i)));
      if (length2 > 0.25 && u > 1e-9) {
        continue;
      }
      if (std::hypot(a.x + u * dx - goal.pose.position.x,
                     a.y + u * dy - goal.pose.position.y) <= radius) {
        cursor = static_cast<double>(i) + u;
        found = true;
        break;
      }
    }
    if (!found) {
      break;
    }
  }
  return passed;
}
// Conservative current-body overlap for a failed passage. Keep the failure
// latched until the entire padded footprint is outside the tunnel rectangle;
// otherwise a retry starting inside would skip the entry-to-exit detector.
inline bool
bodyOverlapsTunnel(const Tunnel &tunnel,
                   const std::vector<geometry_msgs::msg::Point> &footprint,
                   double x, double y, double yaw) {
  if (footprint.size() < 3 || !std::isfinite(x + y + yaw)) {
    return true;
  }
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -min_x, min_y = min_x, max_y = -min_x;
  for (const auto &p : footprint) {
    if (!std::isfinite(p.x + p.y)) {
      return true;
    }
    const double px = x + std::cos(yaw) * p.x - std::sin(yaw) * p.y;
    const double py = y + std::sin(yaw) * p.x + std::cos(yaw) * p.y;
    min_x = std::min(min_x, px);
    max_x = std::max(max_x, px);
    min_y = std::min(min_y, py);
    max_y = std::max(max_y, py);
  }
  return max_x >= tunnel.x_min && min_x <= tunnel.x_max &&
         max_y >= tunnel.y - tunnel.half_width &&
         min_y <= tunnel.y + tunnel.half_width;
}
// The filled circumscribed disk contains every footprint orientation. Cells
// intersecting the disk are tested, including their area rather than just
// center.
inline bool rotationCollision(const nav2_costmap_2d::Costmap2D &map, double x,
                              double y, double radius) {
  if (!std::isfinite(x + y + radius) || radius <= 0.0 ||
      map.getResolution() <= 0.0) {
    return true;
  }
  const double r = map.getResolution(), ox = map.getOriginX(),
               oy = map.getOriginY();
  const int x0 = static_cast<int>(std::floor((x - radius - ox) / r));
  const int x1 = static_cast<int>(std::floor((x + radius - ox) / r));
  const int y0 = static_cast<int>(std::floor((y - radius - oy) / r));
  const int y1 = static_cast<int>(std::floor((y + radius - oy) / r));
  if (x0 < 0 || y0 < 0 || x1 >= static_cast<int>(map.getSizeInCellsX()) ||
      y1 >= static_cast<int>(map.getSizeInCellsY())) {
    return true;
  }
  for (int iy = y0; iy <= y1; ++iy) {
    for (int ix = x0; ix <= x1; ++ix) {
      const double dx =
          std::max({ox + ix * r - x, 0.0, x - (ox + (ix + 1) * r)});
      const double dy =
          std::max({oy + iy * r - y, 0.0, y - (oy + (iy + 1) * r)});
      if (dx * dx + dy * dy <= radius * radius && map.getCost(ix, iy) >= 254) {
        return true;
      }
    }
  }
  return false;
}
} // namespace sentry_tunnel_bt
#endif // SENTRY_TUNNEL_BT__TUNNEL_GEOMETRY_HPP_
