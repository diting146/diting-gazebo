// Copyright 2026 Diting navigation contributors
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "sentry_planner_server/goal_aware_action_server.hpp"

using namespace std::chrono_literals;
using ToPose = nav2_msgs::action::ComputePathToPose;
using ThroughPoses = nav2_msgs::action::ComputePathThroughPoses;

namespace
{
template<typename Predicate>
bool until(Predicate predicate, std::chrono::milliseconds timeout = 2500ms)
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    if (predicate()) {return true;}
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}
ToPose::Goal goalFor(ToPose *, int id)
{
  ToPose::Goal goal;
  goal.goal.pose.position.x = id;
  return goal;
}
ThroughPoses::Goal goalFor(ThroughPoses *, int id)
{
  ThroughPoses::Goal goal;
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = id;
  goal.goals.push_back(pose);
  return goal;
}
int goalId(const ToPose::Goal & goal) {return static_cast<int>(goal.goal.pose.position.x);}
int goalId(const ThroughPoses::Goal & goal) {return static_cast<int>(goal.goals.front().pose.position.x);}

std::atomic<int> test_serial{0};

// Real action requests and cancel services: no mocked status flags or goal handles.
template<typename Action>
class Driver
{
public:
  using Server = sentry_planner_server::GoalAwareSimpleActionServer<Action>;
  using Client = rclcpp_action::Client<Action>;
  using ClientHandle = rclcpp_action::ClientGoalHandle<Action>;
  using ResultFuture = std::shared_future<typename ClientHandle::WrappedResult>;

  Driver()
  {
    const auto suffix = std::to_string(++test_serial);
    server_node = std::make_shared<rclcpp::Node>("goal_server_" + suffix);
    client_node = std::make_shared<rclcpp::Node>("goal_client_" + suffix);
    const auto name = "/cancel_test_" + suffix;
    server = std::make_unique<Server>(server_node, name, [this]() {execute();},
      []() {}, 500ms, true, rcl_action_server_get_default_options(), [this]() {
        if (pause_cancel_transition) {
          cancel_callback_reached = true;
          while (!release_cancel_transition && !stop) {std::this_thread::sleep_for(1ms);}
        }
      });
    server->activate();
    client = rclcpp_action::create_client<Action>(client_node, name);
    executor.add_node(client_node);
    spin = std::thread([this]() {executor.spin();});
  }
  ~Driver()
  {
    release_final = true;
    release_cancel_transition = true;
    stop = true;
    server->deactivate();
    executor.cancel();
    if (spin.joinable()) {spin.join();}
    server.reset();
  }

  std::shared_ptr<ClientHandle> send(int id)
  {
    if (!client->wait_for_action_server(2s)) {return nullptr;}
    auto future = client->async_send_goal(goalFor(static_cast<Action *>(nullptr), id));
    if (future.wait_for(2s) != std::future_status::ready) {return nullptr;}
    return future.get();
  }
  auto cancelAsync(const std::shared_ptr<ClientHandle> & handle)
  {return client->async_cancel_goal(handle);}
  bool cancel(const std::shared_ptr<ClientHandle> & handle)
  {
    auto future = client->async_cancel_goal(handle);
    return future.wait_for(2s) == std::future_status::ready &&
           !future.get()->goals_canceling.empty();
  }
  bool started(int id)
  {
    return until([this, id]() {
      std::lock_guard<std::mutex> lock(events_mutex);
      for (int value : starts) {if (value == id) {return true;}}
      return false;
    });
  }
  int startCount(int id)
  {
    std::lock_guard<std::mutex> lock(events_mutex);
    return static_cast<int>(std::count(starts.begin(), starts.end(), id));
  }
  ResultFuture result(const std::shared_ptr<ClientHandle> & handle)
  {
    return client->async_get_result(handle);
  }

  std::atomic<bool> block_final{false}, final_reached{false}, release_final{false};
  std::atomic<bool> pause_cancel_transition{false}, cancel_callback_reached{false};
  std::atomic<bool> release_cancel_transition{false};
  std::atomic<bool> stop{false};
  std::atomic<int> errors{0};

private:
  void execute()
  {
    auto handle = server->currentGoalForExecution();
    if (!handle) {return;}
    const int id = goalId(*handle->get_goal());
    {
      std::lock_guard<std::mutex> lock(events_mutex);
      starts.push_back(id);
    }
    auto output = std::make_shared<typename Action::Result>();
    const auto canceled = [&]() {
      return stop || !server->is_server_active() || !handle->is_active() || handle->is_canceling();
    };
    // The first goal stays active long enough to create a real pending goal.
    const auto deadline = std::chrono::steady_clock::now() + (id == 1 ? 600ms : 100ms);
    while (std::chrono::steady_clock::now() < deadline) {
      if (canceled()) {server->terminateGoal(handle, output); return;}
      std::this_thread::sleep_for(2ms);
    }
    if (block_final) {
      final_reached = true;
      while (!release_final && !stop) {std::this_thread::sleep_for(2ms);}
    }
    try {
      server->succeedGoal(handle, output);
    } catch (const std::exception &) {
      ++errors;
      server->terminateGoal(handle, output);
    }
  }

  std::shared_ptr<rclcpp::Node> server_node, client_node;
  std::unique_ptr<Server> server;
  typename Client::SharedPtr client;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spin;
  std::mutex events_mutex;
  std::vector<int> starts;
};

class GoalCancel : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

template<typename Action>
void currentCancelKeepsPending()
{
  Driver<Action> driver;
  auto first = driver.send(1);
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(driver.started(1));
  auto second = driver.send(2);
  ASSERT_NE(second, nullptr);
  auto first_result = driver.result(first);
  auto second_result = driver.result(second);
  const auto requested_at = std::chrono::steady_clock::now();
  ASSERT_TRUE(driver.cancel(first));
  ASSERT_EQ(first_result.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(first_result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_LT(std::chrono::steady_clock::now() - requested_at, 500ms);
  ASSERT_EQ(second_result.wait_for(2500ms), std::future_status::ready);
  EXPECT_EQ(second_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(driver.startCount(2), 1);
  EXPECT_EQ(driver.errors, 0);
}

template<typename Action>
void pendingCancelKeepsCurrent()
{
  Driver<Action> driver;
  auto first = driver.send(1);
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(driver.started(1));
  auto second = driver.send(2);
  ASSERT_NE(second, nullptr);
  auto first_result = driver.result(first);
  auto second_result = driver.result(second);
  ASSERT_TRUE(driver.cancel(second));
  ASSERT_EQ(first_result.wait_for(2500ms), std::future_status::ready);
  EXPECT_EQ(first_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_EQ(second_result.wait_for(2500ms), std::future_status::ready);
  EXPECT_EQ(second_result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(driver.errors, 0);
}

template<typename Action>
void cancelAtSuccessBoundary()
{
  Driver<Action> driver;
  driver.block_final = true;
  auto first = driver.send(2);
  ASSERT_NE(first, nullptr);
  auto result = driver.result(first);
  ASSERT_TRUE(until([&]() {return driver.final_reached.load();}));
  ASSERT_TRUE(driver.cancel(first));
  driver.release_final = true;
  ASSERT_EQ(result.wait_for(2500ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(driver.errors, 0);
}

template<typename Action>
void acceptedCancelCannotBeOvertakenBySuccess()
{
  Driver<Action> driver;
  driver.block_final = true;
  driver.pause_cancel_transition = true;
  auto first = driver.send(2);
  ASSERT_NE(first, nullptr);
  auto result = driver.result(first);
  ASSERT_TRUE(until([&]() {return driver.final_reached.load();}));
  auto cancel = driver.cancelAsync(first);
  ASSERT_TRUE(until([&]() {return driver.cancel_callback_reached.load();}));
  // Deliberately pause exactly after ACCEPT but before rclcpp's _cancel_goal().
  // The worker is now allowed to call succeedGoal concurrently.
  driver.release_final = true;
  EXPECT_EQ(result.wait_for(100ms), std::future_status::timeout);
  driver.release_cancel_transition = true;
  ASSERT_EQ(cancel.wait_for(2500ms), std::future_status::ready);
  EXPECT_FALSE(cancel.get()->goals_canceling.empty());
  ASSERT_EQ(result.wait_for(2500ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(driver.errors, 0);
}

TEST_F(GoalCancel, ToPoseCancelCurrentKeepsPending) {currentCancelKeepsPending<ToPose>();}
TEST_F(GoalCancel, ThroughPosesCancelCurrentKeepsPending) {currentCancelKeepsPending<ThroughPoses>();}
TEST_F(GoalCancel, ToPoseCancelPendingKeepsCurrent) {pendingCancelKeepsCurrent<ToPose>();}
TEST_F(GoalCancel, ThroughPosesCancelPendingKeepsCurrent) {pendingCancelKeepsCurrent<ThroughPoses>();}
TEST_F(GoalCancel, ToPoseSuccessBoundaryHonorsCancel) {cancelAtSuccessBoundary<ToPose>();}
TEST_F(GoalCancel, ThroughPosesSuccessBoundaryHonorsCancel) {cancelAtSuccessBoundary<ThroughPoses>();}
TEST_F(GoalCancel, ToPoseAcceptedCancelCannotBeOvertaken)
{acceptedCancelCannotBeOvertakenBySuccess<ToPose>();}
TEST_F(GoalCancel, ThroughPosesAcceptedCancelCannotBeOvertaken)
{acceptedCancelCannotBeOvertakenBySuccess<ThroughPoses>();}
}  // namespace
