// Copyright 2026 diting
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_omni_pid_pursuit_controller/footprint_collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace pb_omni_pid_pursuit_controller
{
namespace
{
using XY = std::pair<double, double>;

double cross(const XY & a, const XY & b, const XY & c)
{
  return (b.first - a.first) * (c.second - a.second) - (b.second - a.second) * (c.first - a.first);
}

std::vector<XY> hull(std::vector<XY> points)
{
  std::sort(points.begin(), points.end());
  points.erase(std::unique(points.begin(), points.end()), points.end());
  if (points.size() < 3) {
    return {};
  }
  std::vector<XY> result(2 * points.size());
  size_t k = 0;
  for (const auto & p : points) {
    while (k >= 2 && cross(result[k - 2], result[k - 1], p) <= 0.0) {
      --k;
    }
    result[k++] = p;
  }
  const auto lower = k + 1;
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (k >= lower && cross(result[k - 2], result[k - 1], *it) <= 0.0) {
      --k;
    }
    result[k++] = *it;
  }
  result.resize(k - 1);
  return result;
}

bool intersectsCell(const std::vector<XY> & polygon, double x, double y, double half)
{
  // Separating-axis test, including the two axes of the occupied grid square.
  for (size_t axis = 0; axis < polygon.size() + 2; ++axis) {
    double nx = 1.0, ny = 0.0;
    if (axis == 1) {
      nx = 0.0;
      ny = 1.0;
    } else if (axis >= 2) {
      const auto & a = polygon[axis - 2];
      const auto & b = polygon[(axis - 1) % polygon.size()];
      nx = -(b.second - a.second);
      ny = b.first - a.first;
    }
    double low = std::numeric_limits<double>::infinity();
    double high = -low;
    for (const auto & p : polygon) {
      const auto projection = nx * p.first + ny * p.second;
      low = std::min(low, projection);
      high = std::max(high, projection);
    }
    const auto center = nx * x + ny * y;
    const auto extent = half * (std::abs(nx) + std::abs(ny));
    if (high < center - extent || low > center + extent) {
      return false;
    }
  }
  return true;
}
}  // namespace

const char * sweepFailureName(SweepFailure reason)
{
  switch (reason) {
    case SweepFailure::NONE:
      return "none";
    case SweepFailure::INVALID_INPUT:
      return "invalid_input";
    case SweepFailure::INVALID_FOOTPRINT:
      return "invalid_footprint";
    case SweepFailure::DEGENERATE_HULL:
      return "degenerate_hull";
    case SweepFailure::OUTSIDE_MAP:
      return "outside_map";
    case SweepFailure::LETHAL_CELL:
      return "lethal_cell";
    case SweepFailure::UNKNOWN_CELL:
      return "unknown_cell";
  }
  return "unknown_reason";
}

bool sweptFootprintCollision(
  const nav2_costmap_2d::Costmap2D & map, const std::vector<geometry_msgs::msg::Point> & footprint,
  double start_x, double start_y, double end_x, double end_y, double yaw)
{
  return sweptFootprintCollision(map, footprint, start_x, start_y, end_x, end_y, yaw, nullptr);
}

bool sweptFootprintCollision(
  const nav2_costmap_2d::Costmap2D & map, const std::vector<geometry_msgs::msg::Point> & footprint,
  double start_x, double start_y, double end_x, double end_y, double yaw,
  SweepCollisionTrace * trace)
{
  if (trace) {
    *trace = SweepCollisionTrace{};
    trace->start_x = start_x;
    trace->start_y = start_y;
    trace->end_x = end_x;
    trace->end_y = end_y;
    trace->yaw = yaw;
  }
  const auto rejected = [&](SweepFailure reason, const std::vector<XY> * polygon = nullptr) {
    if (trace) {
      trace->reason = reason;
      try {
        if (polygon) {
          for (const auto & p : *polygon) {
            geometry_msgs::msg::Point point;
            point.x = p.first;
            point.y = p.second;
            trace->swept_hull.push_back(point);
          }
        }
        trace->complete = true;
      } catch (...) {
        // Observation must never turn the original rejection into acceptance.
        trace->complete = false;
      }
    }
    return true;
  };
  if (
    footprint.size() < 3 || !std::isfinite(start_x) || !std::isfinite(start_y) ||
    !std::isfinite(end_x) || !std::isfinite(end_y) || !std::isfinite(yaw) ||
    !(map.getResolution() > 0.0)) {
    return rejected(SweepFailure::INVALID_INPUT);
  }
  const double c = std::cos(yaw), s = std::sin(yaw);
  std::vector<XY> points;
  for (const auto & p : footprint) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      return rejected(SweepFailure::INVALID_FOOTPRINT);
    }
    const auto x = c * p.x - s * p.y;
    const auto y = s * p.x + c * p.y;
    points.emplace_back(start_x + x, start_y + y);
    points.emplace_back(end_x + x, end_y + y);
  }
  const auto polygon = hull(std::move(points));
  if (polygon.size() < 3) {
    return rejected(SweepFailure::DEGENERATE_HULL, &polygon);
  }
  unsigned int min_x = map.getSizeInCellsX(), min_y = map.getSizeInCellsY();
  unsigned int max_x = 0, max_y = 0;
  for (const auto & p : polygon) {
    unsigned int mx, my;
    if (!map.worldToMap(p.first, p.second, mx, my)) {
      if (trace) {
        trace->outside_vertex.x = p.first;
        trace->outside_vertex.y = p.second;
      }
      return rejected(SweepFailure::OUTSIDE_MAP, &polygon);
    }
    min_x = std::min(min_x, mx);
    min_y = std::min(min_y, my);
    max_x = std::max(max_x, mx);
    max_y = std::max(max_y, my);
  }
  // Include adjacent cells when an edge lies exactly on a grid boundary.
  min_x = min_x > 0 ? min_x - 1 : 0;
  min_y = min_y > 0 ? min_y - 1 : 0;
  max_x = std::min(max_x + 1, map.getSizeInCellsX() - 1);
  max_y = std::min(max_y + 1, map.getSizeInCellsY() - 1);
  for (unsigned int my = min_y; my <= max_y; ++my) {
    for (unsigned int mx = min_x; mx <= max_x; ++mx) {
      if (map.getCost(mx, my) < nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }
      double wx, wy;
      map.mapToWorld(mx, my, wx, wy);
      if (intersectsCell(polygon, wx, wy, map.getResolution() / 2.0)) {
        const auto cost = map.getCost(mx, my);
        if (trace) {
          trace->has_cell = true;
          trace->mx = mx;
          trace->my = my;
          trace->cost = cost;
          trace->cell_x = wx;
          trace->cell_y = wy;
          trace->cell_half = map.getResolution() / 2.0;
        }
        return rejected(
          cost == nav2_costmap_2d::NO_INFORMATION ? SweepFailure::UNKNOWN_CELL
                                                  : SweepFailure::LETHAL_CELL,
          &polygon);
      }
    }
  }
  return false;
}
}  // namespace pb_omni_pid_pursuit_controller
