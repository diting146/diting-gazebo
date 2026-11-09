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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__COLLISION_DIAGNOSTICS_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__COLLISION_DIAGNOSTICS_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pb_omni_pid_pursuit_controller/footprint_collision.hpp"

namespace pb_omni_pid_pursuit_controller
{
// A replan is not recovery. Reset only when a safe command is actually returned.
class CollisionEpisode
{
public:
  bool firstFailure()
  {
    if (rejected_) {
      return false;
    }
    rejected_ = true;
    return true;
  }
  bool rejected() const { return rejected_; }
  void commandAccepted() { rejected_ = false; }

private:
  bool rejected_{false};
};

struct CollisionMapSnapshot
{
  std::string frame;
  double origin_x{0.0}, origin_y{0.0}, resolution{0.0};
  unsigned int size_x{0}, size_y{0};
  bool current{false};
  std::vector<unsigned char> data;

  // Caller holds the same map mutex as the collision decision.
  void capture(
    const nav2_costmap_2d::Costmap2D & map, const std::string & frame_id, bool is_current);
};

struct ControllerCollisionEvent
{
  std::uint64_t event_id{0};
  std::int64_t decision_snapshot_stamp_ns{0}, captured_steady_ns{0};
  std::string plugin, branch, reason, control_frame;
  std::int64_t segment_index{-1};
  bool body_pose_available{false}, projection_available{false}, segment_available{false};
  bool rotation_envelope{false}, polygon_check{true};
  geometry_msgs::msg::PoseStamped input_pose, body_pose, projected_pose;
  geometry_msgs::msg::PoseStamped segment_start_pose, segment_end_pose;
  geometry_msgs::msg::Twist observed_velocity, candidate_command;
  double prediction_time{0.0};
  std::vector<geometry_msgs::msg::Point> padded_footprint, checked_footprint;
  SweepCollisionTrace trace;
  CollisionMapSnapshot map;
};

// JSON uses null for nonfinite diagnostics, never invalid JSON NaN/Infinity.
std::string collisionEventJson(const ControllerCollisionEvent & event);
}  // namespace pb_omni_pid_pursuit_controller
#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__COLLISION_DIAGNOSTICS_HPP_
