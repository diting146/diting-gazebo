// Copyright 2026 Diting navigation contributors
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "sentry_planner_server/planner_server.hpp"

using namespace std::chrono_literals;
namespace
{
using ToPose = nav2_msgs::action::ComputePathToPose;
using ThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
using tdt_yastar_planner::CancelChecker;

class SlowPlanner : public nav2_core::GlobalPlanner, public tdt_yastar_planner::CancelablePlanner
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override {return createPlan(start, goal, {});}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, const CancelChecker & checker) override
  {
    ++calls;
    auto deadline = std::chrono::steady_clock::now() + 600ms;
    while (std::chrono::steady_clock::now() < deadline) {
      if (checker && checker()) {throw tdt_yastar_planner::PlannerCancelled();}
      std::this_thread::sleep_for(2ms);
    }
    nav_msgs::msg::Path path;
    path.header = goal.header;
    path.poses = {start, goal};
    return path;
  }
  std::atomic<int> calls{0};
};

class NonCurrentLayer : public nav2_costmap_2d::Layer
{
public:
  NonCurrentLayer() {enabled_ = true; current_ = false;}
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(double, double, double, double *, double *, double *, double *) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override {}
};

class CallbackHarness : public sentry_planner_server::PlannerServer
{
public:
  explicit CallbackHarness(const rclcpp::NodeOptions & options) : PlannerServer(options) {}
  bool prepare(const std::shared_ptr<SlowPlanner> & planner, bool wait_for_map = false)
  {
    // Configure a real empty costmap. No layers, map input, TF, Gazebo or robot
    // motion are needed because every request supplies use_start=true in map.
    costmap_ros_->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    if (costmap_ros_->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      return false;
    }
    costmap_ = costmap_ros_->getCostmap();
    tf_ = costmap_ros_->getTfBuffer();
    if (wait_for_map) {
      costmap_ros_->getLayeredCostmap()->addPlugin(std::make_shared<NonCurrentLayer>());
    }
    planners_["GridBased"] = planner;
    planner_ids_concat_ = "GridBased ";
    max_planner_duration_ = 0.0;
    plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);
    plan_publisher_->on_activate();
    action_server_pose_ = std::make_unique<ActionServerToPose>(shared_from_this(),
      "compute_path_to_pose", [this]() {++callbacks_started; computePlan();}, []() {}, 500ms, true);
    action_server_poses_ = std::make_unique<ActionServerThroughPoses>(shared_from_this(),
      "compute_path_through_poses", [this]() {++callbacks_started; computePlanThroughPoses();}, []() {}, 500ms, true);
    action_server_pose_->activate();
    action_server_poses_->activate();
    return true;
  }
  std::atomic<int> callbacks_started{0};
  ~CallbackHarness()
  {
    if (action_server_pose_) {action_server_pose_->deactivate(); action_server_pose_.reset();}
    if (action_server_poses_) {action_server_poses_->deactivate(); action_server_poses_.reset();}
    plan_publisher_.reset();
    planners_.clear();
    if (costmap_ros_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      costmap_ros_->cleanup();
    }
  }
};

std::atomic<int> next_namespace{0};
class RunningCallbackServer
{
public:
  explicit RunningCallbackServer(bool noncurrent = false)
  {
    ns = "/planner_callback_" + std::to_string(++next_namespace);
    auto options = rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=" + ns});
    server = std::make_shared<CallbackHarness>(options);
    planner = std::make_shared<SlowPlanner>();
    ready = server->prepare(planner, noncurrent);
    client_node = std::make_shared<rclcpp::Node>("test_client", ns);
    executor.add_node(client_node);
    spin = std::thread([this]() {executor.spin();});
  }
  ~RunningCallbackServer()
  {
    server.reset();
    executor.cancel();
    if (spin.joinable()) {spin.join();}
  }
  std::string ns;
  bool ready{false};
  std::shared_ptr<CallbackHarness> server;
  std::shared_ptr<SlowPlanner> planner;
  std::shared_ptr<rclcpp::Node> client_node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spin;
};

template<typename Predicate>
bool eventually(Predicate predicate)
{
  auto deadline = std::chrono::steady_clock::now() + 2500ms;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {return true;}
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}
geometry_msgs::msg::PoseStamped pose(double x)
{
  geometry_msgs::msg::PoseStamped value;
  value.header.frame_id = "map";
  value.pose.orientation.w = 1.0;
  value.pose.position.x = x;
  return value;
}
ToPose::Goal makeGoal(ToPose *)
{
  ToPose::Goal goal;
  goal.use_start = true;
  goal.start = pose(0);
  goal.goal = pose(1);
  goal.planner_id = "GridBased";
  return goal;
}
ThroughPoses::Goal makeGoal(ThroughPoses *)
{
  ThroughPoses::Goal goal;
  goal.use_start = true;
  goal.start = pose(0);
  goal.goals = {pose(1), pose(2)};
  goal.planner_id = "GridBased";
  return goal;
}
std::string actionName(ToPose *) {return "compute_path_to_pose";}
std::string actionName(ThroughPoses *) {return "compute_path_through_poses";}

class PlannerCallback : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

template<typename Action>
void cancelActualCallback(bool noncurrent = false)
{
  RunningCallbackServer running(noncurrent);
  ASSERT_TRUE(running.ready);
  auto client = rclcpp_action::create_client<Action>(running.client_node,
    running.ns + "/" + actionName(static_cast<Action *>(nullptr)));
  ASSERT_TRUE(client->wait_for_action_server(2s));
  auto accepted = client->async_send_goal(makeGoal(static_cast<Action *>(nullptr)));
  ASSERT_EQ(accepted.wait_for(2s), std::future_status::ready);
  auto handle = accepted.get();
  ASSERT_NE(handle, nullptr);
  auto result = client->async_get_result(handle);
  if (!noncurrent) {
    ASSERT_TRUE(eventually([&]() {return running.planner->calls > 0;}));
  } else {
    ASSERT_TRUE(eventually([&]() {return running.server->callbacks_started > 0;}));
    std::this_thread::sleep_for(30ms);
    ASSERT_EQ(result.wait_for(0ms), std::future_status::timeout);
  }
  auto cancel = client->async_cancel_goal(handle);
  ASSERT_EQ(cancel.wait_for(2s), std::future_status::ready);
  ASSERT_FALSE(cancel.get()->goals_canceling.empty());
  ASSERT_EQ(result.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  if (noncurrent) {EXPECT_EQ(running.planner->calls, 0);}
}

TEST_F(PlannerCallback, ToPoseSearchCancellation) {cancelActualCallback<ToPose>();}
TEST_F(PlannerCallback, ThroughPosesSearchCancellation) {cancelActualCallback<ThroughPoses>();}
TEST_F(PlannerCallback, ToPoseCostmapWaitCancellation) {cancelActualCallback<ToPose>(true);}
TEST_F(PlannerCallback, ThroughPosesCostmapWaitCancellation) {cancelActualCallback<ThroughPoses>(true);}

TEST_F(PlannerCallback, CancellationWhileOtherActionOwnsPlanningLock)
{
  RunningCallbackServer running;
  ASSERT_TRUE(running.ready);
  auto first_client = rclcpp_action::create_client<ToPose>(running.client_node,
    running.ns + "/compute_path_to_pose");
  auto waiting_client = rclcpp_action::create_client<ThroughPoses>(running.client_node,
    running.ns + "/compute_path_through_poses");
  ASSERT_TRUE(first_client->wait_for_action_server(2s));
  ASSERT_TRUE(waiting_client->wait_for_action_server(2s));
  auto first_accepted = first_client->async_send_goal(makeGoal(static_cast<ToPose *>(nullptr)));
  ASSERT_EQ(first_accepted.wait_for(2s), std::future_status::ready);
  auto first = first_accepted.get();
  ASSERT_NE(first, nullptr);
  auto first_result = first_client->async_get_result(first);
  ASSERT_TRUE(eventually([&]() {return running.planner->calls == 1;}));
  auto waiting_accepted = waiting_client->async_send_goal(makeGoal(static_cast<ThroughPoses *>(nullptr)));
  ASSERT_EQ(waiting_accepted.wait_for(2s), std::future_status::ready);
  auto waiting = waiting_accepted.get();
  ASSERT_NE(waiting, nullptr);
  auto waiting_result = waiting_client->async_get_result(waiting);
  auto cancel = waiting_client->async_cancel_goal(waiting);
  ASSERT_EQ(cancel.wait_for(2s), std::future_status::ready);
  ASSERT_FALSE(cancel.get()->goals_canceling.empty());
  ASSERT_EQ(waiting_result.wait_for(250ms), std::future_status::ready);
  EXPECT_EQ(waiting_result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(first_result.wait_for(0ms), std::future_status::timeout);
  ASSERT_EQ(first_result.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(first_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(running.planner->calls, 1);
}
}  // namespace
