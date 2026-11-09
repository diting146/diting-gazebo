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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__FOOTPRINT_COLLISION_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__FOOTPRINT_COLLISION_HPP_

#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace pb_omni_pid_pursuit_controller
{
enum class SweepFailure {
  NONE,
  INVALID_INPUT,
  INVALID_FOOTPRINT,
  DEGENERATE_HULL,
  OUTSIDE_MAP,
  LETHAL_CELL,
  UNKNOWN_CELL
};

// Optional evidence from the exact branch that returned true. No map is read a
// second time to infer a different hit. A clear check resets reason to None.
struct SweepCollisionTrace
{
  SweepFailure reason{SweepFailure::NONE};
  bool complete{false};
  double start_x{0.0}, start_y{0.0}, end_x{0.0}, end_y{0.0}, yaw{0.0};
  std::vector<geometry_msgs::msg::Point> swept_hull;
  geometry_msgs::msg::Point outside_vertex{};
  bool has_cell{false};
  unsigned int mx{0}, my{0};
  std::uint8_t cost{0};
  double cell_x{0.0}, cell_y{0.0}, cell_half{0.0};
};

const char * sweepFailureName(SweepFailure reason);

// Check the convex hull swept by a padded body footprint during a straight
// translation at fixed yaw. Unknown cells and any part outside the map fail
// closed. Soft/inflated costs are not physical obstacles: the whole body is
// checked against lethal cells, rather than applying the robot radius twice.
bool sweptFootprintCollision(
  const nav2_costmap_2d::Costmap2D & map, const std::vector<geometry_msgs::msg::Point> & footprint,
  double start_x, double start_y, double end_x, double end_y, double yaw);

// Preserve the seven-argument ABI for other already-built Nav2 plugins.
bool sweptFootprintCollision(
  const nav2_costmap_2d::Costmap2D & map, const std::vector<geometry_msgs::msg::Point> & footprint,
  double start_x, double start_y, double end_x, double end_y, double yaw,
  SweepCollisionTrace * trace);

}  // namespace pb_omni_pid_pursuit_controller
#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__FOOTPRINT_COLLISION_HPP_
