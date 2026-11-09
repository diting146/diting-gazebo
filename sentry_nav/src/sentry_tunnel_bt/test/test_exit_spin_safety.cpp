// Copyright 2026 diting
// SPDX-License-Identifier: Apache-2.0
#include "sentry_tunnel_bt/exit_spin_safety.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <utility>

namespace sentry_tunnel_bt {
namespace {
nav_msgs::msg::Path
path(std::initializer_list<std::pair<double, double>> points) {
  nav_msgs::msg::Path result;
  result.header.frame_id = "map";
  for (const auto &point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = result.header;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.orientation.w = 1.0;
    result.poses.push_back(pose);
  }
  return result;
}
nav2_costmap_2d::Costmap2D freeMap() {
  return nav2_costmap_2d::Costmap2D(400, 400, 0.05, -10.0, -10.0, 0);
}
void mark(nav2_costmap_2d::Costmap2D &map, double x, double y,
          unsigned char cost = 254) {
  unsigned int mx, my;
  ASSERT_TRUE(map.worldToMap(x, y, mx, my));
  map.setCost(mx, my, cost);
}
const Tunnel kTunnel{-3.5, -1.5, 0.0, 0.4};
nav_msgs::msg::Path forwardPath() {
  return path({{-4.0, 0.0}, {-3.0, 0.0}, {-2.0, 0.0}, {-1.0, 0.0},
               {0.0, 1.0}, {2.0, 1.0}});
}
TEST(ExitSpinSafety, RequiresCircumscribedRadiusPlusFiveCentimetresClearance) {
  auto map = freeMap();
  EXPECT_FALSE(exitSpinSafe(map, forwardPath(), kTunnel, 1, -1.051, 0.0, 0.4));
  EXPECT_TRUE(exitSpinSafe(map, forwardPath(), kTunnel, 1, -1.05, 0.0, 0.4));
  EXPECT_TRUE(exitSpinSafe(map, forwardPath(), kTunnel, 1, -1.0, 0.0, 0.4));
  const auto reverse = path({{-1.0, 0.0}, {-2.0, 0.0}, {-3.8, 0.0},
                             {-4.2, 0.0}, {-5.0, 1.0}});
  EXPECT_FALSE(exitSpinSafe(map, reverse, kTunnel, -1, -3.949, 0.0, 0.4));
  EXPECT_TRUE(exitSpinSafe(map, reverse, kTunnel, -1, -3.95, 0.0, 0.4));
}
TEST(ExitSpinSafety, HistoricalNarrowTunnelDoesNotBlockRemainingRoute) {
  auto map = freeMap();
  for (double x = -3.5; x <= -1.5; x += 0.05) {
    mark(map, x, 0.375);
    mark(map, x, -0.375);
  }
  const auto disk = circumscribedRotationFootprint(0.4);
  ASSERT_TRUE(pb_omni_pid_pursuit_controller::sweptFootprintCollision(
      map, disk, -3.0, 0.0, -2.0, 0.0, 0.0));
  EXPECT_TRUE(exitSpinSafe(map, forwardPath(), kTunnel, 1, -1.0, 0.0, 0.4));
}
TEST(ExitSpinSafety, RejectsCurrentRotationEnvelopeCollision) {
  auto map = freeMap();
  mark(map, -1.35, 0.0);
  EXPECT_FALSE(exitSpinSafe(map, forwardPath(), kTunnel, 1, -1.0, 0.0, 0.4));
}
TEST(ExitSpinSafety, RejectsFutureWallEvenWhenCurrentEnvelopeIsClear) {
  auto map = freeMap();
  mark(map, 1.0, 1.0);
  ASSERT_FALSE(rotationCollision(map, -1.0, 0.0, 0.4));
  EXPECT_FALSE(remainingSpinPathSafe(map, forwardPath(), -1.0, 0.0, 0.4));
}
TEST(ExitSpinSafety, SweepsCurrentToNextPathPointConnection) {
  auto map = freeMap();
  mark(map, -0.5, 0.5);
  ASSERT_FALSE(rotationCollision(map, -1.0, 0.0, 0.4));
  ASSERT_FALSE(rotationCollision(map, 0.0, 1.0, 0.4));
  EXPECT_FALSE(remainingSpinPathSafe(map, forwardPath(), -1.0, 0.0, 0.4));
}
TEST(ExitSpinSafety, SweepsBetweenRemainingPathPoints) {
  auto map = freeMap();
  mark(map, 1.0, 1.0);
  ASSERT_FALSE(rotationCollision(map, 0.0, 1.0, 0.4));
  ASSERT_FALSE(rotationCollision(map, 2.0, 1.0, 0.4));
  EXPECT_FALSE(remainingSpinPathSafe(map, forwardPath(), -1.0, 0.0, 0.4));
}
TEST(ExitSpinSafety, PreservesExactLastGoalForNearestFinalOrSinglePosePath) {
  auto map = freeMap();
  const auto route = path({{0.0, 0.0}, {2.017, 1.013}});
  mark(map, 2.017, 1.013);
  EXPECT_FALSE(remainingSpinPathSafe(map, route, 1.5, 1.013, 0.2));
  EXPECT_FALSE(remainingSpinPathSafe(map, path({{2.017, 1.013}}), 1.5, 1.013, 0.2));
  EXPECT_TRUE(remainingSpinPathSafe(freeMap(), path({{2.017, 1.013}}),
                                   1.5, 1.013, 0.2));
}
TEST(ExitSpinSafety, RejectsUnknownCellsAndOutOfMapSweeps) {
  auto map = freeMap();
  mark(map, 1.0, 1.0, 255);
  EXPECT_FALSE(remainingSpinPathSafe(map, forwardPath(), -1.0, 0.0, 0.4));
  EXPECT_FALSE(remainingSpinPathSafe(freeMap(), path({{9.0, 0.0}, {9.8, 0.0}}),
                                    9.0, 0.0, 0.4));
  EXPECT_FALSE(remainingSpinPathSafe(freeMap(), path({{9.0, 0.0}, {9.8, 0.0}}),
                                    9.8, 0.0, 0.4));
}
TEST(ExitSpinSafety, AllowsSoftInflationButNotPhysicalObstacles) {
  auto map = freeMap();
  mark(map, 1.0, 1.0, 253);
  EXPECT_TRUE(remainingSpinPathSafe(map, forwardPath(), -1.0, 0.0, 0.4));
}
TEST(ExitSpinSafety, RejectsInvalidFramesGeometryAndNonfiniteHistoricalPoses) {
  auto map = freeMap();
  auto route = forwardPath();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(exitSpinSafe(map, route, kTunnel, 0, -1.0, 0.0, 0.4));
  EXPECT_FALSE(exitSpinSafe(map, route, Tunnel{-1.5, -3.5, 0.0}, 1, -1.0, 0.0, 0.4));
  EXPECT_FALSE(exitSpinSafe(map, route, kTunnel, 1, nan, 0.0, 0.4));
  EXPECT_FALSE(remainingSpinPathSafe(map, route, -1.0, 0.0, nan));
  EXPECT_FALSE(remainingSpinPathSafe(map, route, -1.0, 0.0, 0.0));
  EXPECT_FALSE(remainingSpinPathSafe(map, {}, -1.0, 0.0, 0.4));
  route.poses.front().pose.position.x = nan;
  EXPECT_FALSE(remainingSpinPathSafe(map, route, -1.0, 0.0, 0.4));
  route = forwardPath();
  route.poses.front().pose.orientation.w = nan;
  EXPECT_FALSE(remainingSpinPathSafe(map, route, -1.0, 0.0, 0.4));
  route = forwardPath();
  route.poses.back().header.frame_id = "odom";
  EXPECT_FALSE(remainingSpinPathSafe(map, route, -1.0, 0.0, 0.4));
  route = forwardPath();
  route.header.frame_id.clear();
  EXPECT_FALSE(remainingSpinPathSafe(map, route, -1.0, 0.0, 0.4));
}
} // namespace
} // namespace sentry_tunnel_bt
