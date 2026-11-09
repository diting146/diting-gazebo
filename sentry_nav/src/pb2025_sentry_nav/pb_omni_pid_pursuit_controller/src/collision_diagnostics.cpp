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

#include "pb_omni_pid_pursuit_controller/collision_diagnostics.hpp"

#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

namespace pb_omni_pid_pursuit_controller
{
void CollisionMapSnapshot::capture(
  const nav2_costmap_2d::Costmap2D & map, const std::string & frame_id, bool is_current)
{
  frame = frame_id;
  origin_x = map.getOriginX();
  origin_y = map.getOriginY();
  resolution = map.getResolution();
  size_x = map.getSizeInCellsX();
  size_y = map.getSizeInCellsY();
  current = is_current;
  const auto cells = static_cast<std::size_t>(size_x) * size_y;
  if (cells == 0) {
    data.clear();
    return;
  }
  const auto * source = map.getCharMap();
  data.assign(source, source + cells);
}

namespace
{
void stringValue(std::ostream & out, const std::string & text)
{
  out << '"';
  constexpr char hex[] = "0123456789abcdef";
  for (const unsigned char c : text) {
    if (c == '"' || c == '\\') {
      out << '\\' << c;
    } else if (c < 0x20) {
      out << "\\u00" << hex[c >> 4] << hex[c & 0xf];
    } else {
      out << c;
    }
  }
  out << '"';
}
void number(std::ostream & out, double value)
{
  if (std::isfinite(value)) {
    out << value;
  } else {
    out << "null";
  }
}
void xy(std::ostream & out, double x, double y)
{
  out << '[';
  number(out, x);
  out << ',';
  number(out, y);
  out << ']';
}
void polygon(std::ostream & out, const std::vector<geometry_msgs::msg::Point> & points)
{
  out << '[';
  bool first = true;
  for (const auto & p : points) {
    if (!first) {
      out << ',';
    }
    first = false;
    xy(out, p.x, p.y);
  }
  out << ']';
}
void pose(std::ostream & out, const geometry_msgs::msg::PoseStamped & value)
{
  out << "{\"frame\":";
  stringValue(out, value.header.frame_id);
  out << ",\"stamp_ns\":"
      << static_cast<std::int64_t>(value.header.stamp.sec) * 1000000000LL +
           value.header.stamp.nanosec
      << ",\"position\":[";
  number(out, value.pose.position.x);
  out << ',';
  number(out, value.pose.position.y);
  out << ',';
  number(out, value.pose.position.z);
  out << "],\"orientation_xyzw\":[";
  number(out, value.pose.orientation.x);
  out << ',';
  number(out, value.pose.orientation.y);
  out << ',';
  number(out, value.pose.orientation.z);
  out << ',';
  number(out, value.pose.orientation.w);
  out << "]}";
}
void twist(std::ostream & out, const geometry_msgs::msg::Twist & value)
{
  out << "{\"linear\":[";
  number(out, value.linear.x);
  out << ',';
  number(out, value.linear.y);
  out << ',';
  number(out, value.linear.z);
  out << "],\"angular\":[";
  number(out, value.angular.x);
  out << ',';
  number(out, value.angular.y);
  out << ',';
  number(out, value.angular.z);
  out << "]}";
}
}  // namespace

std::string collisionEventJson(const ControllerCollisionEvent & e)
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17) << std::boolalpha;
  out << "{\"schema\":1,\"event_id\":" << e.event_id << ",\"plugin\":";
  stringValue(out, e.plugin);
  out << ",\"branch\":";
  stringValue(out, e.branch);
  out << ",\"reason\":";
  stringValue(out, e.reason);
  out << ",\"segment_index\":" << e.segment_index
      << ",\"decision_snapshot_stamp_ns\":" << e.decision_snapshot_stamp_ns
      << ",\"captured_steady_ns\":" << e.captured_steady_ns
      << ",\"map_update_stamp_available\":false,\"control_frame\":";
  stringValue(out, e.control_frame);
  out << ",\"input_pose\":";
  pose(out, e.input_pose);
  out << ",\"body_pose\":";
  if (e.body_pose_available) {
    pose(out, e.body_pose);
  } else {
    out << "null";
  }
  out << ",\"projected_pose\":";
  if (e.projection_available) {
    pose(out, e.projected_pose);
  } else {
    out << "null";
  }
  out << ",\"planned_segment\":";
  if (e.segment_available) {
    out << "{\"start\":";
    pose(out, e.segment_start_pose);
    out << ",\"end\":";
    pose(out, e.segment_end_pose);
    out << '}';
  } else {
    out << "null";
  }
  out << ",\"observed_velocity\":";
  twist(out, e.observed_velocity);
  out << ",\"candidate_command\":";
  twist(out, e.candidate_command);
  out << ",\"prediction_time_sec\":";
  number(out, e.prediction_time);
  out << ",\"polygon_check\":" << e.polygon_check
      << ",\"rotation_envelope\":" << e.rotation_envelope << ",\"padded_footprint\":";
  polygon(out, e.padded_footprint);
  out << ",\"checked_footprint\":";
  polygon(out, e.checked_footprint);
  out << ",\"sweep\":{\"reason\":";
  stringValue(out, sweepFailureName(e.trace.reason));
  out << ",\"complete\":" << e.trace.complete << ",\"start\":";
  xy(out, e.trace.start_x, e.trace.start_y);
  out << ",\"end\":";
  xy(out, e.trace.end_x, e.trace.end_y);
  out << ",\"fixed_yaw\":";
  number(out, e.trace.yaw);
  out << ",\"hull\":";
  polygon(out, e.trace.swept_hull);
  out << ",\"outside_vertex\":";
  if (e.trace.reason == SweepFailure::OUTSIDE_MAP) {
    xy(out, e.trace.outside_vertex.x, e.trace.outside_vertex.y);
  } else {
    out << "null";
  }
  out << "},\"cell\":";
  if (e.trace.has_cell) {
    out << "{\"mx\":" << e.trace.mx << ",\"my\":" << e.trace.my
        << ",\"index\":" << static_cast<std::size_t>(e.trace.my) * e.map.size_x + e.trace.mx
        << ",\"cost\":" << static_cast<unsigned int>(e.trace.cost) << ",\"center\":";
    xy(out, e.trace.cell_x, e.trace.cell_y);
    out << ",\"bounds_xyxy\":[";
    number(out, e.trace.cell_x - e.trace.cell_half);
    out << ',';
    number(out, e.trace.cell_y - e.trace.cell_half);
    out << ',';
    number(out, e.trace.cell_x + e.trace.cell_half);
    out << ',';
    number(out, e.trace.cell_y + e.trace.cell_half);
    out << "]}";
  } else {
    out << "null";
  }
  out << ",\"costmap\":{\"frame\":";
  stringValue(out, e.map.frame);
  out << ",\"origin\":";
  xy(out, e.map.origin_x, e.map.origin_y);
  out << ",\"resolution\":";
  number(out, e.map.resolution);
  out << ",\"size\":[" << e.map.size_x << ',' << e.map.size_y << "],\"current\":" << e.map.current
      << ",\"encoding\":\"uint8_row_major\",\"data\":[";
  for (std::size_t i = 0; i < e.map.data.size(); ++i) {
    if (i) {
      out << ',';
    }
    out << static_cast<unsigned int>(e.map.data[i]);
  }
  out << "]}}";
  return out.str();
}
}  // namespace pb_omni_pid_pursuit_controller
