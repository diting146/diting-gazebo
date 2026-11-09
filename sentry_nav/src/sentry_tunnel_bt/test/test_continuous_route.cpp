// Copyright 2026 diting
// SPDX-License-Identifier: Apache-2.0
#include "sentry_tunnel_bt/continuous_route.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <utility>

namespace sentry_tunnel_bt {
namespace {
geometry_msgs::msg::PoseStamped pose(double x, double y) {
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.w = 1.0;
  return result;
}
nav_msgs::msg::Path
path(std::initializer_list<std::pair<double, double>> points) {
  nav_msgs::msg::Path result;
  result.header.frame_id = "map";
  for (const auto &point : points) {
    result.poses.push_back(pose(point.first, point.second));
  }
  return result;
}
const Tunnel kTunnel{-3.5, -1.5, 3.475};
std::vector<geometry_msgs::msg::PoseStamped> forwardGoals() {
  return {pose(-4.0, 3.475), pose(-2.5, 3.475), pose(0.0, 0.0)};
}
TEST(ContinuousRoute, AcceptsForwardAndReverseThreePointRoutes) {
  EXPECT_TRUE(continuousThreePointRoute(forwardGoals(), kTunnel, 1, -4.0, 0.015,
                                        0.288));
  const auto reverse = std::vector<geometry_msgs::msg::PoseStamped>{
      pose(-1.0, 3.475), pose(-2.5, 3.475), pose(-5.0, 2.0)};
  EXPECT_TRUE(
      continuousThreePointRoute(reverse, kTunnel, -1, -1.0, 0.015, 0.288));
}
TEST(ContinuousRoute, RejectsExtraOrMissingWaypoints) {
  auto goals = forwardGoals();
  goals.insert(goals.begin() + 2, pose(-2.0, 2.0));
  EXPECT_FALSE(
      continuousThreePointRoute(goals, kTunnel, 1, -4.0, 0.015, 0.288));
  goals = forwardGoals();
  goals.erase(goals.begin());
  EXPECT_FALSE(
      continuousThreePointRoute(goals, kTunnel, 1, -4.0, 0.015, 0.288));
}
TEST(ContinuousRoute, DoesNotSkipOffAxisOrOutOfTunnelIntermediatePoint) {
  auto goals = forwardGoals();
  goals[1].pose.position.y += 0.02;
  EXPECT_FALSE(
      continuousThreePointRoute(goals, kTunnel, 1, -4.0, 0.015, 0.288));
  goals = forwardGoals();
  goals[1].pose.position.x = -4.5;
  EXPECT_FALSE(
      continuousThreePointRoute(goals, kTunnel, 1, -4.0, 0.015, 0.288));
  goals[1].pose.position.x = -1.0;
  EXPECT_FALSE(
      continuousThreePointRoute(goals, kTunnel, 1, -4.0, 0.015, 0.288));
}
TEST(ContinuousRoute, RequiresFirstGoalNearTheActualEntryTarget) {
  auto goals = forwardGoals();
  goals[0].pose.position.x -= 0.16;
  EXPECT_FALSE(
      continuousThreePointRoute(goals, kTunnel, 1, -4.0, 0.015, 0.288));
  EXPECT_FALSE(continuousThreePointRoute(forwardGoals(), kTunnel, 1, -3.4,
                                         0.015, 0.288));
}
TEST(ContinuousRoute, RejectsFinalGoalInsideOrBeforeFullExitClearance) {
  auto goals = forwardGoals();
  goals[2] = pose(-2.0, 3.475);
  EXPECT_FALSE(
      continuousThreePointRoute(goals, kTunnel, 1, -4.0, 0.015, 0.288));
  goals[2] = pose(-0.72, 3.475);
  EXPECT_FALSE(
      continuousThreePointRoute(goals, kTunnel, 1, -4.0, 0.015, 0.288));
  goals[2] = pose(-0.70, 3.475);
  EXPECT_TRUE(continuousThreePointRoute(goals, kTunnel, 1, -4.0, 0.015, 0.288));
}
TEST(ContinuousRoute, RejectsWrongDirectionFramesAndNonfiniteData) {
  EXPECT_FALSE(continuousThreePointRoute(forwardGoals(), kTunnel, 0, -4.0,
                                         0.015, 0.288));
  EXPECT_FALSE(continuousThreePointRoute(forwardGoals(), kTunnel, -1, -4.0,
                                         0.015, 0.288));
  auto goals = forwardGoals();
  goals[1].header.frame_id = "odom";
  EXPECT_FALSE(
      continuousThreePointRoute(goals, kTunnel, 1, -4.0, 0.015, 0.288));
  goals = forwardGoals();
  goals[2].pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
      continuousThreePointRoute(goals, kTunnel, 1, -4.0, 0.015, 0.288));
  EXPECT_FALSE(continuousThreePointRoute(forwardGoals(), kTunnel, 1, -4.0,
                                         -0.01, 0.288));
}
TEST(ContinuousRoute, ExtendsAxisForBodyLookaheadAndMargin) {
  EXPECT_NEAR(continuousExitDistance(0.6, 0.288, 0.4), 0.848, 1e-12);
  EXPECT_DOUBLE_EQ(continuousExitDistance(1.2, 0.288, 0.4), 1.2);
  EXPECT_THROW(continuousExitDistance(0.6, 0.288, -0.1), std::invalid_argument);
  EXPECT_THROW(continuousExitDistance(std::numeric_limits<double>::quiet_NaN(),
                                      0.288, 0.4),
               std::invalid_argument);
}
TEST(ContinuousRoute,
     AppendsTailWithoutDuplicateSeamAndPreservesExactFinalPose) {
  const auto axis = path({{-4.0, 3.475}, {-1.0, 3.475}, {-0.65, 3.475}});
  auto tail = path({{-0.65, 3.475}, {-0.5, 2.0}, {0.0, 0.0}});
  auto final_goal = pose(0.0, 0.0);
  final_goal.header.stamp.sec = 123;
  final_goal.pose.orientation.z = std::sin(0.3);
  final_goal.pose.orientation.w = std::cos(0.3);
  tail.poses.back() = final_goal;
  const auto joined = appendContinuousTail(axis, tail, final_goal);
  ASSERT_EQ(joined.poses.size(), 5u);
  EXPECT_EQ(joined.header, axis.header);
  EXPECT_EQ(joined.poses.front(), axis.poses.front());
  EXPECT_EQ(joined.poses[2], axis.poses.back());
  EXPECT_EQ(joined.poses[3], tail.poses[1]);
  EXPECT_EQ(joined.poses.back(), final_goal);
}
TEST(ContinuousRoute, PreservesSmallDistinctConnectionSegment) {
  const auto axis = path({{-4.0, 3.475}, {-0.65, 3.475}});
  const auto tail = path({{-0.61, 3.475}, {0.0, 0.0}});
  const auto joined = appendContinuousTail(axis, tail, pose(0.0, 0.0));
  ASSERT_EQ(joined.poses.size(), 4u);
  EXPECT_EQ(joined.poses[1], axis.poses.back());
  EXPECT_EQ(joined.poses[2], tail.poses.front());
}
TEST(ContinuousRoute, RejectsUnvalidatedGapAndChangedEndpoint) {
  const auto axis = path({{-4.0, 3.475}, {-0.65, 3.475}});
  EXPECT_THROW(appendContinuousTail(axis, path({{-0.59, 3.475}, {0.0, 0.0}}),
                                    pose(0.0, 0.0)),
               std::invalid_argument);
  EXPECT_THROW(appendContinuousTail(axis, path({{-0.65, 3.475}, {0.1, 0.0}}),
                                    pose(0.0, 0.0)),
               std::invalid_argument);
  auto tail = path({{-0.65, 3.475}, {0.0, 0.0}});
  tail.poses.back().pose.orientation.z = 0.5;
  EXPECT_THROW(appendContinuousTail(axis, tail, pose(0.0, 0.0)),
               std::invalid_argument);
}
TEST(ContinuousRoute, RejectsEmptyFramesAndNonfinitePathPoses) {
  const auto axis = path({{-4.0, 3.475}, {-0.65, 3.475}});
  auto tail = path({{-0.65, 3.475}, {0.0, 0.0}});
  EXPECT_THROW(
      appendContinuousTail(nav_msgs::msg::Path{}, tail, pose(0.0, 0.0)),
      std::invalid_argument);
  EXPECT_THROW(
      appendContinuousTail(axis, nav_msgs::msg::Path{}, pose(0.0, 0.0)),
      std::invalid_argument);
  tail.header.frame_id = "odom";
  EXPECT_THROW(appendContinuousTail(axis, tail, pose(0.0, 0.0)),
               std::invalid_argument);
  tail = path({{-0.65, 3.475}, {0.0, 0.0}});
  tail.poses[0].pose.position.x = std::numeric_limits<double>::infinity();
  EXPECT_THROW(appendContinuousTail(axis, tail, pose(0.0, 0.0)),
               std::invalid_argument);
}
TEST(ContinuousRoute, AcceptsEquivalentQuaternionSignAndKeepsRequestedSign) {
  const auto axis = path({{-4.0, 3.475}, {-0.65, 3.475}});
  auto tail = path({{-0.65, 3.475}, {0.0, 0.0}});
  tail.poses.back().pose.orientation.w = -1.0;
  const auto final_goal = pose(0.0, 0.0);
  EXPECT_EQ(appendContinuousTail(axis, tail, final_goal).poses.back(),
            final_goal);
}
TEST(ContinuousApproach, MovesForwardWhileClosingRemainingLateralError) {
  const auto command = continuousApproachCommand(-4.2, 3.51, -4.0, 3.475, 0.45,
                                                 1, 0.015, 0.30, 0.0);
  EXPECT_FALSE(command.stop);
  EXPECT_NEAR(command.vx, 0.25, 1e-12);
  EXPECT_LT(command.vy, 0.0);
  EXPECT_LE(std::hypot(command.vx, command.vy), 0.45);
}
TEST(ContinuousApproach, MirrorsForwardMotionForReversePassage) {
  const auto command = continuousApproachCommand(-0.8, 3.51, -1.0, 3.475, 0.45,
                                                 -1, 0.015, 0.30, 0.0);
  EXPECT_FALSE(command.stop);
  EXPECT_NEAR(command.vx, -0.25, 1e-12);
  EXPECT_LT(command.vy, 0.0);
  EXPECT_LE(std::hypot(command.vx, command.vy), 0.45);
}
TEST(ContinuousApproach, KeepsCombinedVectorWithinConfiguredSpeed) {
  for (const double maximum : {0.20, 0.45, 1.0}) {
    const auto command = continuousApproachCommand(
        -4.8, 3.51, -4.0, 3.475, maximum, 1, 0.015, 0.30, 0.0);
    EXPECT_FALSE(command.stop);
    EXPECT_GT(command.vx, 0.0);
    EXPECT_LT(command.vy, 0.0);
    EXPECT_LE(std::hypot(command.vx, command.vy), maximum + 1e-12);
  }
}
TEST(ContinuousApproach, RequiresAlignedHeadingBeforeForwardBias) {
  for (const double error : {0.004, -0.004}) {
    const auto command = continuousApproachCommand(-4.2, 3.51, -4.0, 3.475,
                                                   0.45, 1, 0.015, 0.30, error);
    EXPECT_NEAR(command.vx, 0.0, 1e-12);
    EXPECT_LT(command.vy, 0.0);
  }
  const auto boundary = continuousApproachCommand(
      -4.2, 3.51, -4.0, 3.475, 0.45, 1, 0.015, 0.30, -kRotationStopTolerance);
  EXPECT_GT(boundary.vx, 0.0);
}
TEST(ContinuousApproach, RequiresLateralProximityAndLongitudinalRoom) {
  const auto wide = continuousApproachCommand(-4.2, 3.516, -4.0, 3.475, 0.45, 1,
                                              0.015, 0.30, 0.0);
  EXPECT_NEAR(wide.vx, 0.0, 1e-12);
  EXPECT_LT(wide.vy, 0.0);
  const auto no_room = continuousApproachCommand(-4.1, 3.51, -4.0, 3.475, 0.45,
                                                 1, 0.015, 0.30, 0.0);
  EXPECT_LE(no_room.vx, 0.0);
  const auto reverse_no_room = continuousApproachCommand(
      -0.9, 3.51, -1.0, 3.475, 0.45, -1, 0.015, 0.30, 0.0);
  EXPECT_GE(reverse_no_room.vx, 0.0);
}
TEST(ContinuousApproach, DoesNotTreatUpstreamArrivalAsEntryCompletion) {
  const auto command = continuousApproachCommand(-4.4, 3.475, -4.0, 3.475, 0.45,
                                                 1, 0.015, 0.30, 0.0);
  EXPECT_FALSE(
      waitingPositionAcceptable(-4.4, 3.475, -4.0, 3.475, 1, 0.015, 0.30));
  EXPECT_FALSE(command.stop);
  EXPECT_GT(command.vx, 0.0);
}
TEST(ContinuousApproach, DoesNotReverseAfterPassingUpstreamAlignmentTarget) {
  for (const int direction : {1, -1}) {
    const double entry = direction > 0 ? -4.0 : -1.0;
    const double x = entry - direction * 0.15;
    for (const double heading : {0.0, 0.01}) {
      const auto command = continuousApproachCommand(
          x, 3.50, entry, 3.475, 0.45, direction, 0.015, 0.30, heading);
      EXPECT_GE(direction * command.vx, 0.0);
      EXPECT_LT(command.vy, 0.0);
      if (heading == 0.0) {
        EXPECT_GE(direction * command.vx, 0.25);
      }
    }
  }
}
TEST(ContinuousApproach, KeepsEffectiveLateralCorrectionWhenForwardRoomIsUsed) {
  for (const int direction : {1, -1}) {
    const double entry = direction > 0 ? -4.0 : -1.0;
    const auto command = continuousApproachCommand(
        entry - direction * 0.04, 3.445, entry, 3.475,
        0.45, direction, 0.015, 0.30, 0.0);
    EXPECT_NEAR(command.vx, 0.0, 1e-12);
    EXPECT_NEAR(command.vy, 0.12, 1e-12);
    EXPECT_FALSE(command.stop);
  }
}
TEST(ContinuousApproach, DoesNotAmplifyAcceptedLateralResidualAfterProjection) {
  for (const int direction : {1, -1}) {
    const double entry = direction > 0 ? -4.0 : -1.0;
    for (const double residual : {-0.015, -0.001, 0.0, 0.001, 0.015}) {
      const auto command = continuousApproachCommand(
          entry - direction * 0.069, 3.475 + residual, entry, 3.475,
          0.45, direction, 0.015, 0.30, 0.01);
      EXPECT_TRUE(command.stop);
      EXPECT_DOUBLE_EQ(command.vx, 0.0);
      EXPECT_DOUBLE_EQ(command.vy, 0.0);
    }
  }
}
TEST(ContinuousApproach, KeepsForwardMotionWithoutCorrectingAcceptedLateralAxis) {
  for (const int direction : {1, -1}) {
    const double entry = direction > 0 ? -4.0 : -1.0;
    const auto command = continuousApproachCommand(
        entry - direction * 0.2, 3.476, entry, 3.475,
        0.45, direction, 0.015, 0.30, 0.0);
    EXPECT_FALSE(command.stop);
    EXPECT_GE(direction * command.vx, 0.25);
    EXPECT_DOUBLE_EQ(command.vy, 0.0);
  }
}
TEST(ContinuousApproach, RejectsInvalidHeadingDirectionAndSpeed) {
  EXPECT_THROW(
      continuousApproachCommand(-4.2, 3.51, -4.0, 3.475, 0.45, 1, 0.015, 0.30,
                                std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
  EXPECT_THROW(continuousApproachCommand(-4.2, 3.51, -4.0, 3.475, 0.45, 0,
                                         0.015, 0.30, 0.0),
               std::invalid_argument);
  EXPECT_THROW(continuousApproachCommand(-4.2, 3.51, -4.0, 3.475, 1.01, 1,
                                         0.015, 0.30, 0.0),
               std::invalid_argument);
}
TEST(ContinuousApproach, KeepsOriginalVerifiedDirectionBeforeNearEntryZone) {
  for (const int direction : {1, -1}) {
    const double entry = direction > 0 ? -4.0 : -1.0;
    const double x = entry - direction * 0.8;
    const double y = 3.51;
    const auto original = shortApproachCommand(
        x, y, entry, 3.475, 0.45, direction, 0.015, 0.30, 0.0, 0.0, 0.0);
    const auto continuous = continuousApproachCommand(
        x, y, entry, 3.475, 0.45, direction, 0.015, 0.30, 0.0);
    EXPECT_FALSE(continuous.stop);
    EXPECT_NEAR(continuous.vx, original.vx, 1e-12);
    EXPECT_NEAR(continuous.vy, original.vy, 1e-12);
    EXPECT_NEAR(std::hypot(continuous.vx, continuous.vy), 0.45, 1e-12);
  }
}
} // namespace
} // namespace sentry_tunnel_bt
