// Copyright 2026 diting
// SPDX-License-Identifier: Apache-2.0
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <string>
#include <utility>

namespace {
nav_msgs::msg::Path
path(std::initializer_list<std::pair<double, double>> points) {
  nav_msgs::msg::Path result;
  result.header.frame_id = "map";
  for (const auto &point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    result.poses.push_back(pose);
  }
  return result;
}
struct FollowState {
  int started{0}, running{0}, halted{0};
};
class FollowProbe : public BT::StatefulActionNode {
public:
  FollowProbe(const std::string &name, const BT::NodeConfiguration &config,
              std::shared_ptr<FollowState> state)
      : BT::StatefulActionNode(name, config), state_(std::move(state)) {}
  static BT::PortsList providedPorts() { return {}; }

private:
  BT::NodeStatus onStart() override {
    ++state_->started;
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    ++state_->running;
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override { ++state_->halted; }
  std::shared_ptr<FollowState> state_;
};
class TunnelRouteGuardTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Exercise the registered production condition, never a test copy of it.
    factory_.registerFromPlugin(TUNNEL_BT_PLUGIN_FILE);
  }
  BT::Tree condition() {
    return factory_.createTreeFromText(R"(
      <root main_tree_to_execute="Main"><BehaviorTree ID="Main">
        <TunnelRouteGuard path="{path}" tunnels="{tunnels}"/>
      </BehaviorTree></root>)",
                                       BT::Blackboard::create());
  }
  BT::BehaviorTreeFactory factory_;
  const std::string tunnels_{"-3.5;-1.5;3.475;1.5;3.5;-3.475"};
};
TEST_F(TunnelRouteGuardTest, ChangedPlanHaltsActiveOrdinaryFollower) {
  factory_.registerFromPlugin(NAV2_PIPELINE_PLUGIN_FILE);
  factory_.registerSimpleAction(
      "PlanProbe", [](BT::TreeNode &) { return BT::NodeStatus::SUCCESS; });
  const auto follow = std::make_shared<FollowState>();
  factory_.registerBuilder<FollowProbe>(
      "FollowProbe",
      [follow](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<FollowProbe>(name, config, follow);
      });
  auto tree = factory_.createTreeFromText(R"(
    <root main_tree_to_execute="Main"><BehaviorTree ID="Main">
      <PipelineSequence><PlanProbe/>
        <TunnelRouteGuard path="{path}" tunnels="{tunnels}"/>
        <FollowProbe/>
      </PipelineSequence>
    </BehaviorTree></root>)",
                                          BT::Blackboard::create());
  tree.rootBlackboard()->set("tunnels", tunnels_);
  tree.rootBlackboard()->set("path", path({{-4.0, 2.0}, {-1.0, 2.0}}));
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(follow->started, 1);
  // Replanning selects a complete crossing while ordinary following is active.
  tree.rootBlackboard()->set("path", path({{-4.0, 3.475}, {-1.0, 3.475}}));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(follow->halted, 1);
  EXPECT_EQ(follow->running, 0);
  EXPECT_EQ(follow->started, 1);
}
TEST_F(TunnelRouteGuardTest,
       CompletedAndNearbyRoutesPassButEitherDirectionCrossingFails) {
  auto tree = condition();
  tree.rootBlackboard()->set("tunnels", tunnels_);
  for (const auto &route :
       {path({{-1.0, 3.475}, {0.0, 0.0}}), path({{-4.0, 2.0}, {-1.0, 2.0}})}) {
    tree.rootBlackboard()->set("path", route);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  }
  for (const auto &route : {path({{-4.0, 3.475}, {-1.0, 3.475}}),
                            path({{-1.0, 3.475}, {-4.0, 3.475}})}) {
    tree.rootBlackboard()->set("path", route);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  }
}
TEST_F(TunnelRouteGuardTest, UsesConfiguredRegionAndRejectsInvalidInputs) {
  auto tree = condition();
  tree.rootBlackboard()->set("tunnels", std::string("10;12;2"));
  tree.rootBlackboard()->set("path", path({{9, 2}, {13, 2}}));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  tree.rootBlackboard()->set("tunnels", tunnels_);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  tree.rootBlackboard()->set("tunnels", std::string("-3.5;-1.5"));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  tree.rootBlackboard()->set("tunnels", tunnels_);
  tree.rootBlackboard()->set("path", nav_msgs::msg::Path{});
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  tree.rootBlackboard()->set(
      "path", path({{std::numeric_limits<double>::quiet_NaN(), 0}, {0, 0}}));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
}
TEST_F(TunnelRouteGuardTest, PartialAndInsideRoutesCannotStartOrdinaryFollower) {
  auto tree = condition();
  tree.rootBlackboard()->set("tunnels", tunnels_);
  for (const auto &route : {path({{-4.0, 3.475}, {-2.5, 3.475}}),
                            path({{-2.5, 3.475}, {-4.0, 3.475}}),
                            path({{-2.5, 3.475}, {-1.0, 3.475}}),
                            path({{-2.5, 4.5}, {-2.5, 3.475}}),
                            path({{-2.5, 3.475}})}) {
    tree.rootBlackboard()->set("path", route);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  }
}
TEST_F(TunnelRouteGuardTest, ConfiguredHalfWidthIsUsedAndInvalidWidthFailsClosed) {
  auto tree = factory_.createTreeFromText(R"(
    <root main_tree_to_execute="Main"><BehaviorTree ID="Main">
      <TunnelRouteGuard path="{path}" tunnels="{tunnels}" tunnel_half_width="{width}"/>
    </BehaviorTree></root>)", BT::Blackboard::create());
  tree.rootBlackboard()->set("tunnels", tunnels_);
  tree.rootBlackboard()->set("path", path({{-4.0, 3.775}, {-1.0, 3.775}}));
  tree.rootBlackboard()->set("width", 0.4);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  tree.rootBlackboard()->set("width", 0.2);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  for (double width : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::infinity()}) {
    tree.rootBlackboard()->set("width", width);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  }
}
TEST_F(TunnelRouteGuardTest,
       ThroughFinalGoalReadsLastRemainingPoseWithoutMutatingList) {
  auto tree = factory_.createTreeFromText(R"(
    <root main_tree_to_execute="Main"><BehaviorTree ID="Main">
      <TunnelFinalGoal goals="{goals}" goal="{final_goal}"/>
    </BehaviorTree></root>)",
                                          BT::Blackboard::create());
  auto goals = path({{-4, 3.475}, {-2.5, 3.475}, {0, 0}}).poses;
  tree.rootBlackboard()->set("goals", goals);
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(tree.rootBlackboard()
                       ->get<geometry_msgs::msg::PoseStamped>("final_goal")
                       .pose.position.x,
                   0);
  EXPECT_EQ(tree.rootBlackboard()
                ->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals")
                .size(),
            3u);
  goals.erase(goals.begin(), goals.end() - 1);
  tree.rootBlackboard()->set("goals", goals);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(tree.rootBlackboard()
                       ->get<geometry_msgs::msg::PoseStamped>("final_goal")
                       .pose.position.y,
                   0);
  tree.rootBlackboard()->set("goals",
                             std::vector<geometry_msgs::msg::PoseStamped>{});
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
}
TEST_F(TunnelRouteGuardTest, CompletedContinuousTaskDoesNotStartAnotherFollower) {
  const auto follow = std::make_shared<FollowState>();
  factory_.registerBuilder<FollowProbe>(
      "FollowProbe", [follow](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<FollowProbe>(name, config, follow);
      });
  auto tree = factory_.createTreeFromText(R"(
    <root main_tree_to_execute="Main"><BehaviorTree ID="Main">
      <Fallback><TunnelTaskCompleted completed="{done}"/><FollowProbe/></Fallback>
    </BehaviorTree></root>)", BT::Blackboard::create());
  tree.rootBlackboard()->set("done", true);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(follow->started, 0);
  tree.rootBlackboard()->set("done", false);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(follow->started, 1);
  tree.haltTree();
  EXPECT_EQ(follow->halted, 1);
}
} // namespace
