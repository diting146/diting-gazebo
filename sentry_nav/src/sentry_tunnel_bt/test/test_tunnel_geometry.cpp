// Copyright 2026 diting
// SPDX-License-Identifier: Apache-2.0
#include "sentry_tunnel_bt/tunnel_geometry.hpp"
#include <gtest/gtest.h>
namespace {
nav_msgs::msg::Path
path(std::initializer_list<std::pair<double, double>> points) {
  nav_msgs::msg::Path result;
  for (const auto &p : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = p.first;
    pose.pose.position.y = p.second;
    result.poses.push_back(pose);
  }
  return result;
}
const std::vector<sentry_tunnel_bt::Tunnel> tunnels{{-3.5, -1.5, 3.475},
                                                    {1.5, 3.5, -3.475}};
} // namespace
TEST(TunnelRoute, ActualCrossingAndDirection) {
  const auto east = sentry_tunnel_bt::firstCrossing(
      path({{-4, 3.475}, {-1, 3.475}}), tunnels);
  EXPECT_EQ(east.tunnel, 0);
  EXPECT_EQ(east.direction, 1);
  const auto west = sentry_tunnel_bt::firstCrossing(
      path({{-1, 3.475}, {-4, 3.475}}), tunnels);
  EXPECT_EQ(west.tunnel, 0);
  EXPECT_EQ(west.direction, -1);
}
TEST(TunnelRoute, NearbyDetourAndInsideGoalDoNotForcePassage) {
  EXPECT_EQ(
      sentry_tunnel_bt::firstCrossing(path({{-4, 2.0}, {-1, 2.0}}), tunnels)
          .tunnel,
      -1);
  EXPECT_EQ(sentry_tunnel_bt::firstCrossing(path({{-4, 3.475}, {-2.5, 3.475}}),
                                            tunnels)
                .tunnel,
            -1);
  EXPECT_EQ(sentry_tunnel_bt::firstCrossing(
                path({{-4, 3.475}, {-3, 4.5}, {-1, 3.475}}), tunnels)
                .tunnel,
            -1);
}
TEST(TunnelRoute, PathOrderAndVisitedBound) {
  const auto route = path({{-4, 3.475}, {-1, 3.475}, {1, -3.475}, {4, -3.475}});
  EXPECT_EQ(sentry_tunnel_bt::firstCrossing(route, tunnels).tunnel, 0);
  EXPECT_EQ(sentry_tunnel_bt::firstCrossing(route, tunnels, {0}).tunnel, 1);
  EXPECT_EQ(sentry_tunnel_bt::firstCrossing(route, tunnels, {0, 1}).tunnel, -1);
}
TEST(TunnelRoute, TouchingGuardIncludesSparseSegmentsBoundaryAndPartialRoutes) {
  using sentry_tunnel_bt::pathTouchesTunnel;
  EXPECT_TRUE(pathTouchesTunnel(path({{-4, 3.475}, {-1, 3.475}}), tunnels));
  EXPECT_TRUE(pathTouchesTunnel(path({{-4, 3.475}, {-2.5, 3.475}}), tunnels));
  EXPECT_TRUE(pathTouchesTunnel(path({{-2.5, 3.475}, {-4, 3.475}}), tunnels));
  EXPECT_TRUE(pathTouchesTunnel(path({{-2.5, 3.075}, {-2.5, 3.875}}), tunnels));
  EXPECT_TRUE(pathTouchesTunnel(path({{-3.5, 3.875}}), tunnels));
  EXPECT_FALSE(pathTouchesTunnel(path({{-4, 2}, {-1, 2}}), tunnels));
  EXPECT_FALSE(pathTouchesTunnel(path({{-1, 3.475}, {0, 0}}), tunnels));
  EXPECT_TRUE(pathTouchesTunnel(path({{NAN, 0}}), tunnels));
}
TEST(RotationDisk, TestsFilledAreaUnknownBoundaryAndOutOfMap) {
  nav2_costmap_2d::Costmap2D map(100, 100, .05, 0, 0, 0);
  EXPECT_FALSE(sentry_tunnel_bt::rotationCollision(map, 2.5, 2.5, .4));
  map.setCost(50, 50, 254);
  EXPECT_TRUE(sentry_tunnel_bt::rotationCollision(map, 2.5, 2.5, .4));
  map.setCost(50, 50, 0);
  map.setCost(57, 50, 255);
  EXPECT_TRUE(sentry_tunnel_bt::rotationCollision(map, 2.5, 2.5, .4));
  map.setCost(57, 50, 253);
  EXPECT_FALSE(sentry_tunnel_bt::rotationCollision(map, 2.5, 2.5, .4));
  EXPECT_TRUE(sentry_tunnel_bt::rotationCollision(map, .2, 2.5, .4));
}

TEST(CenteringWaitingPoint, AcceptsFartherOutsideInBothDirections) {
  for (const int direction : {1, -1}) {
    const double entry = direction > 0 ? -4.0 : -1.0;
    const double outside = entry - direction * 0.08;
    EXPECT_TRUE(sentry_tunnel_bt::waitingPositionAcceptable(
        outside, 3.475, entry, 3.475, direction, .015, .15));
    EXPECT_FALSE(sentry_tunnel_bt::waitingPositionAcceptable(
        entry + direction * 0.02, 3.475, entry, 3.475, direction, .015, .15));
    EXPECT_FALSE(sentry_tunnel_bt::waitingPositionAcceptable(
        entry - direction * 0.16, 3.475, entry, 3.475, direction, .015, .15));
  }
}
TEST(CenteringWaitingPoint, RechecksSettledPoseAfterCoasting) {
  EXPECT_TRUE(sentry_tunnel_bt::waitingPositionAcceptable(-4.08, 3.467, -4.0,
                                                          3.475, 1, .015, .15));
  EXPECT_FALSE(sentry_tunnel_bt::waitingPositionAcceptable(
      -4.08, 3.504, -4.0, 3.475, 1, .015, .15));
}
TEST(CenteringFeedback, CorrectsLateralOnlyAndBrakesForPredictedCoast) {
  for (const int direction : {1, -1}) {
    const double entry = direction > 0 ? -4.0 : -1.0;
    const auto moving = sentry_tunnel_bt::centeringCommand(
        entry - direction * .20, 3.40, entry, 3.475, direction, .015, .30, .15,
        0, 0, .25);
    EXPECT_FALSE(moving.stop);
    EXPECT_DOUBLE_EQ(moving.vx, 0.0);
    EXPECT_LE(std::hypot(moving.vx, moving.vy), .150000001);
    EXPECT_GT(moving.vy, 0.0);
    const auto braking = sentry_tunnel_bt::centeringCommand(
        entry - direction * .20, 3.44, entry, 3.475, direction, .015, .30, .15,
        0, .12, .25);
    EXPECT_TRUE(braking.stop); // stopped pose still must be checked by the BT
    EXPECT_DOUBLE_EQ(braking.vy, 0.0);
  }
}
TEST(CenteringFeedback,
     StopsBeforeReversingMovingBodyAndRejectsInvalidSettings) {
  EXPECT_TRUE(sentry_tunnel_bt::centeringCommand(-4.08, 3.45, -4, 3.475, 1,
                                                 .015, .30, .15, 0, .15, .4)
                  .stop);
  using sentry_tunnel_bt::validCenteringSettings;
  EXPECT_TRUE(validCenteringSettings(.015, .30, .3, .15));
  EXPECT_FALSE(validCenteringSettings(.03, .30, .3, .15));
  EXPECT_FALSE(validCenteringSettings(.015, .31, .3, .15));
  EXPECT_FALSE(validCenteringSettings(.015, .30, .1, .15));
  EXPECT_FALSE(validCenteringSettings(.015, .30, .3, 0.0));
  EXPECT_FALSE(validCenteringSettings(
      .015, .30, std::numeric_limits<double>::infinity(), .15));
  EXPECT_THROW(sentry_tunnel_bt::centeringCommand(-4.08, 3.475, -4, 3.475, 0,
                                                  .015, .30, .15, 0, 0, .25),
               std::invalid_argument);
}
TEST(ThroughGoals,
     ConsumesObservedPrefixInBothDirectionsAndPreservesFinalGoal) {
  for (const int direction : {1, -1}) {
    auto goals = path({{-4.0, 3.475}, {-2.5, 3.475}, {0, 0}}).poses;
    for (auto &goal : goals) {
      goal.header.frame_id = "map";
    }
    nav_msgs::msg::Path observed;
    observed.header.frame_id = "map";
    const double begin = direction > 0 ? -4.2 : -0.8;
    for (int i = 0; i <= 34; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = begin + direction * i * .1;
      pose.pose.position.y = 3.475;
      observed.poses.push_back(pose);
    }
    if (direction < 0) {
      std::swap(goals[0], goals[1]);
    }
    EXPECT_EQ(sentry_tunnel_bt::passedGoalPrefix(goals, observed, .15), 2u);
    goals.back().pose.position.x =
        -2.4; // even a final goal on the trace is retained
    goals.back().pose.position.y = 3.475;
    EXPECT_EQ(sentry_tunnel_bt::passedGoalPrefix(goals, observed, .15), 2u);
  }
}
TEST(ThroughGoals,
     DoesNotDeleteUnvisitedOrOutOfOrderPointsOrInterpolateLargeGap) {
  auto goals = path({{-4, 3.475}, {-2.5, 3.475}, {0, 0}}).poses;
  for (auto &goal : goals) {
    goal.header.frame_id = "map";
  }
  auto observed = path({{-4.1, 3.475}, {-4.0, 3.475}, {-3.9, 3.475}});
  observed.header.frame_id = "map";
  EXPECT_EQ(sentry_tunnel_bt::passedGoalPrefix(goals, observed, .15), 1u);
  observed = path({{-4.1, 3.475}, {-1.0, 3.475}});
  observed.header.frame_id = "map";
  EXPECT_EQ(sentry_tunnel_bt::passedGoalPrefix(goals, observed, .15), 1u);
  std::swap(goals[0], goals[1]);
  EXPECT_EQ(sentry_tunnel_bt::passedGoalPrefix(goals, observed, .15), 0u);
}

TEST(FailedPassage, BodyMustBeFullyOutsideBeforeRetryCanSkipTunnel) {
  const sentry_tunnel_bt::Tunnel tunnel{-3.5, -1.5, 3.475};
  std::vector<geometry_msgs::msg::Point> footprint;
  for (const auto &xy : {std::pair<double, double>{-.293, -.272},
                         {.293, -.272},
                         {.293, .272},
                         {-.293, .272}}) {
    geometry_msgs::msg::Point p;
    p.x = xy.first;
    p.y = xy.second;
    footprint.push_back(p);
  }
  EXPECT_TRUE(
      sentry_tunnel_bt::bodyOverlapsTunnel(tunnel, footprint, -2.5, 3.475, 0));
  EXPECT_TRUE(
      sentry_tunnel_bt::bodyOverlapsTunnel(tunnel, footprint, -1.3, 3.475, 0));
  EXPECT_FALSE(
      sentry_tunnel_bt::bodyOverlapsTunnel(tunnel, footprint, -1.1, 3.475, 0));
  EXPECT_TRUE(sentry_tunnel_bt::bodyOverlapsTunnel(tunnel, footprint, -3.7,
                                                   3.475, M_PI));
  EXPECT_FALSE(sentry_tunnel_bt::bodyOverlapsTunnel(tunnel, footprint, -3.9,
                                                    3.475, M_PI));
  EXPECT_FALSE(
      sentry_tunnel_bt::bodyOverlapsTunnel(tunnel, footprint, -2.5, 4.2, 0));
  EXPECT_TRUE(sentry_tunnel_bt::bodyOverlapsTunnel(tunnel, {}, -1.0, 3.475, 0));
}

TEST(PreparationRotation, KeepsCoarseSpeedAndUsesSmallerFinalCorrections) {
  using sentry_tunnel_bt::preparationAngularCommand;
  EXPECT_DOUBLE_EQ(preparationAngularCommand(M_PI / 2, 0, .15, .6), .6);
  EXPECT_DOUBLE_EQ(preparationAngularCommand(-M_PI / 2, 0, .15, .6), -.6);
  EXPECT_DOUBLE_EQ(preparationAngularCommand(.01, 0, .15, .6), .1);
  EXPECT_DOUBLE_EQ(preparationAngularCommand(-.01, 0, .15, .6), -.1);
  EXPECT_DOUBLE_EQ(preparationAngularCommand(.003, 0, .15, .6), 0.0);
}
TEST(PreparationRotation, BrakesForDelayedYawAndDoesNotBrakeWhenMovingAway) {
  using sentry_tunnel_bt::preparationAngularCommand;
  EXPECT_DOUBLE_EQ(preparationAngularCommand(.01, .08, .1, .6), 0.0);
  EXPECT_DOUBLE_EQ(preparationAngularCommand(-.01, -.08, .1, .6), 0.0);
  EXPECT_GT(preparationAngularCommand(.01, -.08, .1, .6), 0.0);
  EXPECT_LT(preparationAngularCommand(-.01, .08, .1, .6), 0.0);
  EXPECT_THROW(preparationAngularCommand(.1, 0, .5, .6), std::invalid_argument);
  EXPECT_THROW(preparationAngularCommand(
                   std::numeric_limits<double>::quiet_NaN(), 0, .1, .6),
               std::invalid_argument);
}

TEST(ApproachSlowdown, ReleasesFarRoadThenLatchesAtWaitingPointBoundary) {
  using sentry_tunnel_bt::approachSlowdownRequired;
  EXPECT_FALSE(approachSlowdownRequired(4.0, 1.5, false));
  EXPECT_FALSE(approachSlowdownRequired(1.5001, 1.5, false));
  EXPECT_TRUE(approachSlowdownRequired(1.5, 1.5, false));
  EXPECT_TRUE(approachSlowdownRequired(.8, 1.5, false));
  EXPECT_TRUE(approachSlowdownRequired(1.7, 1.5, true));
  EXPECT_FALSE(approachSlowdownRequired(4.0, 1.5, false)); // new passage reset
}
TEST(ApproachSlowdown, RejectsInvalidGeometryAndThreshold) {
  using sentry_tunnel_bt::approachSlowdownRequired;
  EXPECT_THROW(approachSlowdownRequired(-.1, 1.5, false),
               std::invalid_argument);
  EXPECT_THROW(approachSlowdownRequired(1.0, 0.0, false),
               std::invalid_argument);
  EXPECT_THROW(approachSlowdownRequired(1.0, -1.0, false),
               std::invalid_argument);
  EXPECT_THROW(approachSlowdownRequired(
                   std::numeric_limits<double>::quiet_NaN(), 1.5, false),
               std::invalid_argument);
  EXPECT_THROW(approachSlowdownRequired(
                   1.0, std::numeric_limits<double>::infinity(), false),
               std::invalid_argument);
}

TEST(RollingAlignment, RotationPolygonContainsFullCircle) {
  const double radius = .4;
  const auto disk = sentry_tunnel_bt::circumscribedRotationFootprint(radius);
  ASSERT_EQ(disk.size(), 32u);
  for (size_t i = 0; i < disk.size(); ++i) {
    const auto &a = disk[i], &b = disk[(i + 1) % disk.size()];
    const double distance =
        std::abs(a.x * b.y - a.y * b.x) / std::hypot(b.x - a.x, b.y - a.y);
    EXPECT_GE(distance + 1e-12, radius);
  }
  EXPECT_THROW(sentry_tunnel_bt::circumscribedRotationFootprint(0),
               std::invalid_argument);
}
TEST(RollingAlignment, BoundsTranslationAndPreservesStoppedValidation) {
  const auto move =
      sentry_tunnel_bt::shortApproachCommand(-4.7, 3.2, -4, 3.475, .45);
  EXPECT_FALSE(move.stop);
  EXPECT_NEAR(std::hypot(move.vx, move.vy), .45, 1e-12);
  EXPECT_GT(move.vx, 0.0);
  EXPECT_GT(move.vy, 0.0);
  EXPECT_TRUE(
      sentry_tunnel_bt::shortApproachCommand(-4.1, 3.475, -4, 3.475, .45).stop);
  EXPECT_THROW(sentry_tunnel_bt::shortApproachCommand(-4.7, 3.2, -4, 3.475, 0),
               std::invalid_argument);
  EXPECT_THROW(
      sentry_tunnel_bt::shortApproachCommand(-4.7, 3.2, -4, 3.475, 1.1),
      std::invalid_argument);
}

TEST(RollingAlignment, FinishesLateralCorrectionInsideOldFifteenCentimeterStop) {
  for (int direction : {1, -1}) {
    const double entry = direction > 0 ? -4.0 : -1.0;
    const auto cmd = sentry_tunnel_bt::shortApproachCommand(
        entry - direction * .095, 3.378, entry, 3.475, .45, direction);
    EXPECT_FALSE(cmd.stop);
    EXPECT_GT(direction * cmd.vx, 0.0);
    EXPECT_GT(cmd.vy, 0.0);
    EXPECT_LE(std::hypot(cmd.vx, cmd.vy), .45);
    EXPECT_LE(.5 * std::hypot(cmd.vx, cmd.vy), std::hypot(.095, .097));
  }
}
TEST(RollingAlignment, UsesExistingWaitingBandWithoutExtraLongitudinalPrecision) {
  for (int direction : {1, -1}) {
    const double entry = direction > 0 ? -4.0 : -1.0;
    EXPECT_TRUE(sentry_tunnel_bt::shortApproachCommand(
        entry - direction * .20, 3.47, entry, 3.475, .45, direction).stop);
    EXPECT_FALSE(sentry_tunnel_bt::shortApproachCommand(
        entry - direction * .20, 3.44, entry, 3.475, .45, direction).stop);
  }
}
TEST(RollingAlignment, ForecastBrakesButDoesNotRelaxActualPositionAcceptance) {
  const auto cmd = sentry_tunnel_bt::shortApproachCommand(
      -4.1, 3.44, -4.0, 3.475, .45, 1, .015, .30, .2, .12, .25);
  EXPECT_TRUE(cmd.stop);
  EXPECT_FALSE(sentry_tunnel_bt::waitingPositionAcceptable(
      -4.1, 3.44, -4.0, 3.475, 1, .015, .30));
  EXPECT_FALSE(sentry_tunnel_bt::shortApproachCommand(
      -4.1, 3.44, -4.0, 3.475, .45, 1, .015, .30, 0, 0, .25).stop);
}
TEST(RollingAlignment, KeepsEffectiveMotionFloorForResidualLateralError) {
  const auto cmd = sentry_tunnel_bt::shortApproachCommand(
      -4.0, 3.44, -4.0, 3.475, .45, 1, .015, .30, 0, 0, .25);
  EXPECT_FALSE(cmd.stop);
  EXPECT_DOUBLE_EQ(cmd.vx, 0.0);
  EXPECT_NEAR(cmd.vy, .12, 1e-12);
  EXPECT_TRUE(sentry_tunnel_bt::shortApproachCommand(
      -4.0, 3.465, -4.0, 3.475, .45, 1, .015, .30, 0, 0, .25).stop);
}
TEST(RollingAlignment, RejectsInvalidForecastDirectionAndTolerance) {
  EXPECT_THROW(sentry_tunnel_bt::shortApproachCommand(
      -4.1, 3.44, -4, 3.475, .45, 0), std::invalid_argument);
  EXPECT_THROW(sentry_tunnel_bt::shortApproachCommand(
      -4.1, 3.44, -4, 3.475, .45, 1, .03), std::invalid_argument);
  EXPECT_THROW(sentry_tunnel_bt::shortApproachCommand(
      -4.1, 3.44, -4, 3.475, .45, 1, .015, .30, 0, 0, .6),
      std::invalid_argument);
}

TEST(ScanStampOrder, RejectsReorderedAndEqualScansWithoutMovingAcceptedStamp) {
  sentry_tunnel_bt::ScanStampGate gate;
  EXPECT_TRUE(gate.accept(1000000000, 1050000000));
  EXPECT_FALSE(gate.accept(900000000, 1060000000));
  EXPECT_FALSE(gate.accept(1000000000, 1070000000));
  EXPECT_EQ(gate.last_stamp_ns, 1000000000);
  EXPECT_TRUE(gate.accept(1100000000, 1110000000));
}
TEST(ScanStampOrder, ClockResetAcceptsNewEpochAndRejectsQueuedFutureEpoch) {
  sentry_tunnel_bt::ScanStampGate gate;
  EXPECT_TRUE(gate.accept(10000000000, 10010000000));
  EXPECT_FALSE(gate.accept(10000000000, 100000000));
  EXPECT_EQ(gate.last_stamp_ns, 0);
  EXPECT_TRUE(gate.accept(90000000, 110000000));
  EXPECT_FALSE(gate.accept(10000000000, 120000000));
  EXPECT_TRUE(gate.accept(130000000, 140000000));
}
TEST(ScanStampOrder, PreservesExistingFiftyMillisecondFutureTolerance) {
  sentry_tunnel_bt::ScanStampGate gate;
  EXPECT_FALSE(gate.accept(0, 100000000));
  EXPECT_FALSE(gate.accept(150000001, 100000000));
  EXPECT_TRUE(gate.accept(150000000, 100000000));
}
