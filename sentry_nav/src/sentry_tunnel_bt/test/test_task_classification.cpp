// Copyright 2026 diting
// SPDX-License-Identifier: Apache-2.0
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include <chrono>
#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <unistd.h>

namespace {
using namespace std::chrono_literals;
class TaskClassificationTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    const auto ns = "/classification_" + std::to_string(getpid()) + "_" +
                    std::to_string(sequence_++);
    node_ = std::make_shared<rclcpp::Node>("bt", ns);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    for (const auto &name : {"local_costmap", "global_costmap"}) {
      auto node = std::make_shared<rclcpp::Node>(name, ns + "/" + name);
      node->declare_parameter("footprint", std::string(
          "[[-0.288,-0.267],[0.288,-0.267],[0.288,0.267],[-0.288,0.267]]"));
      node->declare_parameter("footprint_padding", 0.005);
      executor_.add_node(node);
      maps_.push_back(node);
    }
    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_controller", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr) {
          ++commands_;
        });
    spin_sub_ = node_->create_subscription<example_interfaces::msg::Float32>(
        "cmd_spin", 10, [this](example_interfaces::msg::Float32::ConstSharedPtr) {
          ++spins_;
        });
    limit_sub_ = node_->create_subscription<nav2_msgs::msg::SpeedLimit>(
        "speed_limit", 10, [this](nav2_msgs::msg::SpeedLimit::ConstSharedPtr) {
          ++limits_;
        });
    state_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "tunnel_passage/state", rclcpp::QoS(20).transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr state) { states_ += state->data + "\n"; });
    executor_.add_node(node_);
    factory_.registerFromPlugin(TUNNEL_BT_PLUGIN_FILE);
    factory_.registerSimpleAction("FollowerProbe", [this](BT::TreeNode &) {
      ++followers_;
      return BT::NodeStatus::SUCCESS;
    });
  }
  void TearDown() override {
    if (tree_) {
      tree_->haltTree();
      tree_.reset();
    }
    executor_.cancel();
    maps_.clear();
    node_.reset();
  }
  void create(double start_x, double start_y, double goal_x, double goal_y) {
    nav_msgs::msg::Path route;
    route.header.frame_id = "map";
    for (const auto &p : {std::make_pair(start_x, start_y),
                           std::make_pair(goal_x, goal_y)}) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = route.header;
      pose.pose.position.x = p.first;
      pose.pose.position.y = p.second;
      pose.pose.orientation.w = 1.0;
      route.poses.push_back(pose);
    }
    auto board = BT::Blackboard::create();
    board->set("node", node_);
    board->set("tf_buffer", tf_);
    board->set("path", route);
    board->set("goal", route.poses.back());
    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(R"(
      <root main_tree_to_execute="Main"><BehaviorTree ID="Main"><Sequence>
        <TunnelPassage path="{path}" goal="{goal}" body_frame="robot"
          use_sim_time="false"/>
        <FollowerProbe/>
      </Sequence></BehaviorTree></root>)", board));
    pump(150ms);
  }
  void pose(double x, double y, double age = 0.0) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "robot";
    transform.header.stamp = node_->now() - rclcpp::Duration::from_seconds(age);
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_->setTransform(transform, "test"));
  }
  void pump(std::chrono::milliseconds duration) {
    const auto until = std::chrono::steady_clock::now() + duration;
    do {
      executor_.spin_some(1ms);
      std::this_thread::sleep_for(5ms);
    } while (std::chrono::steady_clock::now() < until);
  }
  BT::NodeStatus run(double x, double y, double age = 0.0, bool publish_tf = true) {
    auto status = BT::NodeStatus::RUNNING;
    const auto until = std::chrono::steady_clock::now() + 4s;
    do {
      if (publish_tf) pose(x, y, age);
      executor_.spin_some(1ms);
      status = tree_->tickRoot();
      std::this_thread::sleep_for(10ms);
    } while (status == BT::NodeStatus::RUNNING &&
             std::chrono::steady_clock::now() < until);
    pump(50ms);
    return status;
  }
  inline static unsigned int sequence_{0};
  rclcpp::Node::SharedPtr node_;
  std::vector<rclcpp::Node::SharedPtr> maps_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr spin_sub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr limit_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  int commands_{0}, spins_{0}, limits_{0}, followers_{0};
  std::string states_;
};
TEST_F(TaskClassificationTest, OutsideOrdinaryRouteDoesNotChangeMotionCommands) {
  create(-4, 2, -1, 2);
  ASSERT_EQ(run(-4, 2), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(followers_, 1);
  EXPECT_EQ(commands_, 0);
  EXPECT_EQ(spins_, 0);
  EXPECT_EQ(limits_, 0);
}
TEST_F(TaskClassificationTest, InsideReturnIsStoppedBeforeOrdinaryFollower) {
  create(-2.5, 3.475, -4.6, 3.475);
  ASSERT_EQ(run(-2.5, 3.475), BT::NodeStatus::FAILURE);
  EXPECT_EQ(followers_, 0);
  EXPECT_GT(commands_, 0);
  EXPECT_EQ(limits_, 0);
  EXPECT_NE(states_.find("requires_manual_exit"), std::string::npos);
}
TEST_F(TaskClassificationTest, BodyTailOverlapAlsoRejectsOutsideCenter) {
  create(-1.3, 3.475, 0, 0);
  ASSERT_EQ(run(-1.3, 3.475), BT::NodeStatus::FAILURE);
  EXPECT_EQ(followers_, 0);
  EXPECT_NE(states_.find("requires_manual_exit"), std::string::npos);
}
TEST_F(TaskClassificationTest, FullCrossingPathCannotOverrideCurrentBodyOverlap) {
  create(-4, 3.475, -1, 3.475);
  EXPECT_EQ(run(-2.5, 3.475), BT::NodeStatus::FAILURE);
  EXPECT_EQ(followers_, 0);
  EXPECT_EQ(limits_, 0);
}
TEST_F(TaskClassificationTest, RepeatedTaskRereadsFootprintParameters) {
  create(-4, 2, -1, 2);
  ASSERT_EQ(run(-4, 2), BT::NodeStatus::SUCCESS);
  tree_->haltTree();
  for (auto &map : maps_) {
    map->set_parameter(rclcpp::Parameter("footprint_padding", 1.5));
  }
  // The current center stays outside; only the freshly enlarged body overlaps.
  EXPECT_EQ(run(-4, 2), BT::NodeStatus::FAILURE);
  EXPECT_EQ(followers_, 1);
  EXPECT_NE(states_.find("requires_manual_exit"), std::string::npos);
}
TEST_F(TaskClassificationTest, OutsidePartialRouteHasExplicitFailure) {
  create(-4, 3.475, -2.5, 3.475);
  ASSERT_EQ(run(-4, 3.475), BT::NodeStatus::FAILURE);
  EXPECT_EQ(followers_, 0);
  EXPECT_NE(states_.find("partial_tunnel_route_requires_complete_passage"), std::string::npos);
}
TEST_F(TaskClassificationTest, CompleteOutsideCrossingKeepsOriginalPreparation) {
  create(-4, 3.475, -1, 3.475);
  for (int i = 0; i < 30; ++i) {
    pose(-4, 3.475);
    executor_.spin_some(1ms);
    EXPECT_EQ(tree_->tickRoot(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(10ms);
  }
  pump(50ms);
  EXPECT_EQ(followers_, 0);
  EXPECT_GT(limits_, 0);
  EXPECT_NE(states_.find("waiting_for_fresh_inputs"), std::string::npos);
}
TEST_F(TaskClassificationTest, MissingTransformTimesOutWithoutDispatch) {
  create(-4, 2, -1, 2);
  EXPECT_EQ(run(-4, 2, 0.0, false), BT::NodeStatus::FAILURE);
  EXPECT_EQ(followers_, 0);
  EXPECT_NE(states_.find("classification_unavailable"), std::string::npos);
}
TEST_F(TaskClassificationTest, StaleTransformTimesOutWithoutDispatch) {
  create(-4, 2, -1, 2);
  EXPECT_EQ(run(-4, 2, 2.0), BT::NodeStatus::FAILURE);
  EXPECT_EQ(followers_, 0);
}
TEST_F(TaskClassificationTest, MissingFootprintTimesOutWithoutDispatch) {
  for (auto &map : maps_) executor_.remove_node(map);
  maps_.clear();
  create(-4, 2, -1, 2);
  EXPECT_EQ(run(-4, 2), BT::NodeStatus::FAILURE);
  EXPECT_EQ(followers_, 0);
}
} // namespace
