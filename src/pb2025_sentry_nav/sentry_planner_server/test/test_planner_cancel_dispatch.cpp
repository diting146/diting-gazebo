// Copyright 2026 Diting navigation contributors
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include "sentry_planner_server/planner_server.hpp"
#include "tdt_yastar_planner/cancelable_planner.hpp"

namespace
{
using tdt_yastar_planner::CancelChecker;
using tdt_yastar_planner::PlannerCancelled;

class StandardPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    ++legacy_calls;
    if (after_legacy) {after_legacy();}
    nav_msgs::msg::Path path;
    path.poses = {start, goal};
    return path;
  }
  int legacy_calls{0};
  std::function<void()> after_legacy;
};

class CooperativePlanner : public StandardPlanner, public tdt_yastar_planner::CancelablePlanner
{
public:
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, const CancelChecker & checker) override
  {
    ++cooperative_calls;
    // Stand in for successive bounded YAstar expansions.
    for (int step = 0; step < 100; ++step) {
      if (checker && checker()) {throw PlannerCancelled();}
    }
    nav_msgs::msg::Path path;
    path.poses = {start, goal};
    return path;
  }
  int cooperative_calls{0};
};

class Harness : public sentry_planner_server::PlannerServer
{
public:
  void install(const nav2_core::GlobalPlanner::Ptr & planner)
  {
    planners_["GridBased"] = planner;
    planner_ids_concat_ = "GridBased ";
  }
};

class PlannerDispatch : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
  geometry_msgs::msg::PoseStamped start, goal;
};

TEST_F(PlannerDispatch, CancellationReachesCooperativePlugin)
{
  Harness server;
  auto plugin = std::make_shared<CooperativePlanner>();
  server.install(plugin);
  int checks = 0;
  EXPECT_THROW(server.getPlan(start, goal, "GridBased", [&]() {return ++checks >= 5;}),
    PlannerCancelled);
  EXPECT_EQ(plugin->cooperative_calls, 1);
  EXPECT_EQ(plugin->legacy_calls, 0);
}

TEST_F(PlannerDispatch, EmptyIdStillUsesCooperativePlugin)
{
  Harness server;
  auto plugin = std::make_shared<CooperativePlanner>();
  server.install(plugin);
  int checks = 0;
  EXPECT_THROW(server.getPlan(start, goal, "", [&]() {return ++checks >= 5;}),
    PlannerCancelled);
  EXPECT_EQ(plugin->cooperative_calls, 1);
  EXPECT_EQ(plugin->legacy_calls, 0);
}

TEST_F(PlannerDispatch, StandardPluginResultIsRejectedAfterCancellation)
{
  Harness server;
  auto plugin = std::make_shared<StandardPlanner>();
  server.install(plugin);
  bool canceled = false;
  plugin->after_legacy = [&]() {canceled = true;};
  EXPECT_THROW(server.getPlan(start, goal, "GridBased", [&]() {return canceled;}),
    PlannerCancelled);
  EXPECT_EQ(plugin->legacy_calls, 1);
}

TEST_F(PlannerDispatch, AlreadyCanceledDoesNotInvokePlugin)
{
  Harness server;
  auto plugin = std::make_shared<CooperativePlanner>();
  server.install(plugin);
  EXPECT_THROW(server.getPlan(start, goal, "GridBased", []() {return true;}),
    PlannerCancelled);
  EXPECT_EQ(plugin->cooperative_calls, 0);
  EXPECT_EQ(plugin->legacy_calls, 0);
}

TEST_F(PlannerDispatch, EmptyCheckerPreservesStandardPluginCompatibility)
{
  Harness server;
  auto plugin = std::make_shared<StandardPlanner>();
  server.install(plugin);
  EXPECT_EQ(server.getPlan(start, goal, "GridBased").poses.size(), 2u);
  EXPECT_EQ(plugin->legacy_calls, 1);
}
}  // namespace
