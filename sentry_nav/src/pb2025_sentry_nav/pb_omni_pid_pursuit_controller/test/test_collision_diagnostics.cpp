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

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pb_omni_pid_pursuit_controller/collision_diagnostics.hpp"

namespace pb_omni_pid_pursuit_controller
{
namespace
{
std::vector<geometry_msgs::msg::Point> rectangle(double half = 0.2)
{
  std::vector<geometry_msgs::msg::Point> points(4);
  points[0].x = points[3].x = -half;
  points[1].x = points[2].x = half;
  points[0].y = points[1].y = -half;
  points[2].y = points[3].y = half;
  return points;
}

TEST(CollisionTrace, RecordsExactFirstRowMajorCellAndUnchangedSweepGeometry)
{
  nav2_costmap_2d::Costmap2D map(20, 20, 0.1, -1.0, -1.0, 0);
  map.setCost(10, 10, 254);
  map.setCost(9, 9, 254);
  SweepCollisionTrace trace;
  ASSERT_TRUE(sweptFootprintCollision(map, rectangle(), 0, 0, 0.1, 0, 0, &trace));
  EXPECT_EQ(trace.reason, SweepFailure::LETHAL_CELL);
  ASSERT_TRUE(trace.has_cell);
  EXPECT_TRUE(trace.complete);
  EXPECT_EQ(trace.mx, 9u);
  EXPECT_EQ(trace.my, 9u);
  EXPECT_EQ(trace.cost, 254u);
  EXPECT_NEAR(trace.cell_x, -0.05, 1e-12);
  EXPECT_NEAR(trace.cell_y, -0.05, 1e-12);
  EXPECT_NEAR(trace.cell_half, 0.05, 1e-12);
  EXPECT_DOUBLE_EQ(trace.start_x, 0.0);
  EXPECT_DOUBLE_EQ(trace.end_x, 0.1);
  EXPECT_DOUBLE_EQ(trace.yaw, 0.0);
  ASSERT_EQ(trace.swept_hull.size(), 4u);
  EXPECT_TRUE(sweptFootprintCollision(map, rectangle(), 0, 0, 0.1, 0, 0));
}

TEST(CollisionTrace, DistinguishesUnknownAndDoesNotTreatInflationAsPhysicalObstacle)
{
  nav2_costmap_2d::Costmap2D map(20, 20, 0.1, -1.0, -1.0, 253);
  map.setCost(10, 10, 255);
  SweepCollisionTrace trace;
  ASSERT_TRUE(sweptFootprintCollision(map, rectangle(), 0, 0, 0, 0, 0, &trace));
  EXPECT_EQ(trace.reason, SweepFailure::UNKNOWN_CELL);
  EXPECT_EQ(trace.cost, 255u);
  map.setCost(10, 10, 253);
  EXPECT_FALSE(sweptFootprintCollision(map, rectangle(), 0, 0, 0, 0, 0, &trace));
  EXPECT_EQ(trace.reason, SweepFailure::NONE);
  EXPECT_FALSE(trace.has_cell);
  EXPECT_TRUE(trace.swept_hull.empty());
}

TEST(CollisionTrace, DistinguishesBoundsAndInvalidInputWithoutInventingObstacleCell)
{
  nav2_costmap_2d::Costmap2D map(20, 20, 0.1, -1.0, -1.0, 0);
  SweepCollisionTrace trace;
  ASSERT_TRUE(sweptFootprintCollision(map, rectangle(), 0.9, 0, 0.9, 0, 0, &trace));
  EXPECT_EQ(trace.reason, SweepFailure::OUTSIDE_MAP);
  EXPECT_FALSE(trace.has_cell);
  EXPECT_GE(trace.outside_vertex.x, 1.0);
  ASSERT_TRUE(sweptFootprintCollision(
    map, rectangle(), 0, 0, 0, 0, std::numeric_limits<double>::quiet_NaN(), &trace));
  EXPECT_EQ(trace.reason, SweepFailure::INVALID_INPUT);
  EXPECT_FALSE(trace.has_cell);
  auto bad_body = rectangle();
  bad_body[0].x = std::numeric_limits<double>::infinity();
  ASSERT_TRUE(sweptFootprintCollision(map, bad_body, 0, 0, 0, 0, 0, &trace));
  EXPECT_EQ(trace.reason, SweepFailure::INVALID_FOOTPRINT);
  std::vector<geometry_msgs::msg::Point> degenerate(4);
  ASSERT_TRUE(sweptFootprintCollision(map, degenerate, 0, 0, 0, 0, 0, &trace));
  EXPECT_EQ(trace.reason, SweepFailure::DEGENERATE_HULL);
}

TEST(CollisionTrace, OptionalEvidenceNeverChangesTheExistingBooleanDecision)
{
  for (unsigned char cost : {0, 252, 253, 254, 255}) {
    nav2_costmap_2d::Costmap2D map(40, 40, 0.05, -1.0, -1.0, 0);
    map.setCost(20, 20, cost);
    for (double yaw : {0.0, 0.3, M_PI / 2}) {
      for (double start : {-0.8, 0.0, 0.9}) {
        for (double end : {-0.8, 0.0, 0.9}) {
          SweepCollisionTrace trace;
          const auto old_api = sweptFootprintCollision(map, rectangle(), start, 0, end, 0, yaw);
          const auto traced =
            sweptFootprintCollision(map, rectangle(), start, 0, end, 0, yaw, &trace);
          EXPECT_EQ(old_api, traced);
          EXPECT_EQ(traced, trace.reason != SweepFailure::NONE);
        }
      }
    }
  }
}

TEST(CollisionSnapshot, OwnsExactLockedMapBytesAndCanReplayAfterTheLiveMapChanges)
{
  nav2_costmap_2d::Costmap2D live(20, 20, 0.1, -1.0, -1.0, 0);
  live.setCost(10, 10, 254);
  CollisionMapSnapshot snapshot;
  SweepCollisionTrace original;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*live.getMutex());
    ASSERT_TRUE(sweptFootprintCollision(live, rectangle(), 0, 0, 0.2, 0, 0, &original));
    snapshot.capture(live, "sentry_robot1/odom", true);
  }
  live.setCost(10, 10, 0);
  ASSERT_FALSE(sweptFootprintCollision(live, rectangle(), 0, 0, 0.2, 0, 0));
  ASSERT_EQ(snapshot.data.size(), 400u);
  EXPECT_EQ(snapshot.data[210], 254u);
  nav2_costmap_2d::Costmap2D replay(
    snapshot.size_x, snapshot.size_y, snapshot.resolution, snapshot.origin_x, snapshot.origin_y, 0);
  std::copy(snapshot.data.begin(), snapshot.data.end(), replay.getCharMap());
  SweepCollisionTrace repeated;
  ASSERT_TRUE(sweptFootprintCollision(
    replay, rectangle(), original.start_x, original.start_y, original.end_x, original.end_y,
    original.yaw, &repeated));
  EXPECT_EQ(repeated.reason, original.reason);
  EXPECT_EQ(repeated.mx, original.mx);
  EXPECT_EQ(repeated.my, original.my);
  EXPECT_EQ(repeated.cost, original.cost);
}

TEST(CollisionEpisode, RepeatedRejectionPublishesOnceUntilASafeCommandIsAccepted)
{
  CollisionEpisode episode;
  EXPECT_TRUE(episode.firstFailure());
  for (int tick = 0; tick < 200; ++tick) {
    EXPECT_FALSE(episode.firstFailure());
  }
  EXPECT_TRUE(episode.rejected());
  episode.commandAccepted();
  EXPECT_FALSE(episode.rejected());
  EXPECT_TRUE(episode.firstFailure());
}

TEST(CollisionSerialization, PreservesRawCostsCellBoundsFramesAndExplicitTimestampMeaning)
{
  nav2_costmap_2d::Costmap2D map(2, 2, 0.1, -0.1, -0.1, 0);
  map.setCost(1, 0, 254);
  map.setCost(0, 1, 255);
  map.setCost(1, 1, 1);
  ControllerCollisionEvent event;
  event.event_id = 3;
  event.plugin = "FollowPathTunnel";
  event.branch = "planned";
  event.reason = "lethal_cell";
  event.segment_index = 12;
  event.decision_snapshot_stamp_ns = 15000000000LL;
  event.map.capture(map, "sentry_robot1/odom", true);
  ASSERT_TRUE(
    sweptFootprintCollision(map, rectangle(0.005), 0.05, -0.05, 0.05, -0.05, 0, &event.trace));
  const auto json = collisionEventJson(event);
  EXPECT_NE(json.find("\"branch\":\"planned\""), std::string::npos);
  EXPECT_NE(json.find("\"segment_index\":12"), std::string::npos);
  EXPECT_NE(json.find("\"decision_snapshot_stamp_ns\":15000000000"), std::string::npos);
  EXPECT_NE(json.find("\"map_update_stamp_available\":false"), std::string::npos);
  EXPECT_NE(json.find("\"index\":1,\"cost\":254"), std::string::npos);
  EXPECT_NE(json.find("\"frame\":\"sentry_robot1/odom\""), std::string::npos);
  EXPECT_NE(
    json.find("\"encoding\":\"uint8_row_major\",\"data\":[0,254,255,1]"), std::string::npos);
  EXPECT_NE(json.find("\"bounds_xyxy\":"), std::string::npos);
}

TEST(CollisionSerialization, PreservesCommandProjectionAndBothPlannedSegmentStamps)
{
  ControllerCollisionEvent event;
  event.branch = "planned";
  event.projection_available = true;
  event.projected_pose.header.frame_id = "odom";
  event.projected_pose.header.stamp.sec = 10;
  event.projected_pose.pose.position.x = 0.7;
  event.segment_available = true;
  event.segment_start_pose.header.frame_id = "odom";
  event.segment_start_pose.header.stamp.sec = 11;
  event.segment_start_pose.pose.position.x = 1.2;
  event.segment_end_pose.header.frame_id = "odom";
  event.segment_end_pose.header.stamp.sec = 12;
  event.segment_end_pose.pose.position.x = 1.4;
  const auto json = collisionEventJson(event);
  EXPECT_NE(
    json.find("\"projected_pose\":{\"frame\":\"odom\",\"stamp_ns\":10000000000"),
    std::string::npos);
  EXPECT_NE(
    json.find("\"planned_segment\":{\"start\":{\"frame\":\"odom\",\"stamp_ns\":11000000000"),
    std::string::npos);
  EXPECT_NE(json.find("\"end\":{\"frame\":\"odom\",\"stamp_ns\":12000000000"), std::string::npos);
}

TEST(CollisionSerialization, InvalidDiagnosticsRemainValidJsonTokensAndEscapeStrings)
{
  ControllerCollisionEvent event;
  event.plugin = "quoted\"\\\nplugin";
  event.prediction_time = std::numeric_limits<double>::quiet_NaN();
  event.trace.yaw = std::numeric_limits<double>::infinity();
  const auto json = collisionEventJson(event);
  EXPECT_NE(json.find("quoted\\\"\\\\\\u000aplugin"), std::string::npos);
  EXPECT_NE(json.find("\"prediction_time_sec\":null"), std::string::npos);
  EXPECT_NE(json.find("\"fixed_yaw\":null"), std::string::npos);
  EXPECT_EQ(json.find("NaN"), std::string::npos);
  EXPECT_EQ(json.find("Infinity"), std::string::npos);
}
}  // namespace
}  // namespace pb_omni_pid_pursuit_controller
