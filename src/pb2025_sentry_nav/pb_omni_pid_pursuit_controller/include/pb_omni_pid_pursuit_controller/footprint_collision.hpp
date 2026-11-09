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

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace pb_omni_pid_pursuit_controller
{
// Check the convex hull swept by a padded body footprint during a straight
// translation at fixed yaw. Unknown cells and any part outside the map fail
// closed. Soft/inflated costs are not physical obstacles: the whole body is
// checked against lethal cells, rather than applying the robot radius twice.
bool sweptFootprintCollision(
  const nav2_costmap_2d::Costmap2D & map, const std::vector<geometry_msgs::msg::Point> & footprint,
  double start_x, double start_y, double end_x, double end_y, double yaw);

}  // namespace pb_omni_pid_pursuit_controller
#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__FOOTPRINT_COLLISION_HPP_
