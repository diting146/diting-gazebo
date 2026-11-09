// Copyright 2026 diting
// SPDX-License-Identifier: Apache-2.0
#include "action_context.hpp"
#include <atomic>
#include <chrono>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <vector>

namespace sentry_tunnel_bt {
namespace {
template <typename Predicate>
bool waitUntil(Predicate predicate, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
}
// Independent real ROS action server. Its deliberately delayed goal response
// simulates acceptance arriving after the BT's bounded halt wait has expired.
class ActionCancelTest : public ::testing::Test {
protected:
  using ServerHandle = rclcpp_action::ServerGoalHandle<Compute>;
  void SetUp() override {
    ros_context_ = std::make_shared<rclcpp::Context>();
    ros_context_->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(ros_context_);
    options.use_global_arguments(false);
    const auto id =
        std::to_string(getpid()) + "_" + std::to_string(sequence_++);
    server_node_ =
        std::make_shared<rclcpp::Node>("tunnel_cancel_server_" + id, options);
    topic_ = "/tunnel_cancel_test_" + id;
    server_group_ = server_node_->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    server_ = rclcpp_action::create_server<Compute>(
        server_node_, topic_ + "/compute",
        [](const rclcpp_action::GoalUUID &,
           std::shared_ptr<const Compute::Goal> goal) {
          if (goal->goal.pose.position.x == 1.0) {
            std::this_thread::sleep_for(2350ms);
          }
          if (goal->goal.pose.position.x == 5.0) {
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<ServerHandle> goal) {
          if (goal->get_goal()->goal.pose.position.x == 2.0) {
            ++other_cancels_;
          } else {
            ++owned_cancels_;
          }
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<ServerHandle> goal) {
          std::lock_guard<std::mutex> lock(workers_mutex_);
          workers_.emplace_back([this, goal]() {
            while (!stop_workers_.load()) {
              auto result = std::make_shared<Compute::Result>();
              if (goal->is_canceling()) {
                goal->canceled(result);
                return;
              }
              if (goal->get_goal()->goal.pose.position.x == 4.0 ||
                  (goal->get_goal()->goal.pose.position.x == 2.0 &&
                   release_other_.load())) {
                goal->succeed(result);
                return;
              }
              std::this_thread::sleep_for(10ms);
            }
            if (goal->is_active()) {
              try {
                goal->abort(std::make_shared<Compute::Result>());
              } catch (const std::exception &) {
              }
            }
          });
        },
        rcl_action_server_get_default_options(), server_group_);
    zero_sub_ = server_node_->create_subscription<geometry_msgs::msg::Twist>(
        topic_ + "/cmd", 10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr command) {
          if (command->linear.x == 0.0 && command->linear.y == 0.0 &&
              command->angular.z == 0.0) {
            ++zero_commands_;
          }
        });
    spin_sub_ = server_node_->create_subscription<example_interfaces::msg::Float32>(
        topic_ + "/spin", 10,
        [this](example_interfaces::msg::Float32::ConstSharedPtr command) {
          if (command->data == 0.0f) { ++zero_spin_commands_; }
        });
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = ros_context_;
    server_executor_ =
        std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
            executor_options, 4);
    server_executor_->add_node(server_node_);
    server_thread_ = std::thread([this]() { server_executor_->spin(); });
  }
  std::shared_ptr<ActionContext> context(const std::string &name) {
    rclcpp::NodeOptions options;
    options.context(ros_context_);
    options.use_global_arguments(false);
    auto node = std::make_shared<rclcpp::Node>(name, options);
    auto result = std::make_shared<ActionContext>(node);
    result->group = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    result->executor.add_callback_group(result->group,
                                        node->get_node_base_interface());
    result->planner.client = rclcpp_action::create_client<Compute>(
        node, topic_ + "/compute", result->group);
    result->stop_pub =
        node->create_publisher<geometry_msgs::msg::Twist>(topic_ + "/cmd", 1);
    result->spin_pub = node->create_publisher<example_interfaces::msg::Float32>(
        topic_ + "/spin", 1);
    result->limit_pub = node->create_publisher<nav2_msgs::msg::SpeedLimit>(
        topic_ + "/limit", 1);
    threads_.push_back(result->start());
    EXPECT_TRUE(result->planner.client->wait_for_action_server(3s));
    return result;
  }
  static Compute::Goal goal(double marker) {
    Compute::Goal result;
    result.goal.pose.position.x = marker;
    return result;
  }
  void TearDown() override {
    stop_workers_.store(true);
    ros_context_->shutdown("test complete");
    server_executor_->cancel();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    for (auto &thread : threads_) {
      if (thread.joinable()) {
        thread.join();
      }
    }
    std::lock_guard<std::mutex> lock(workers_mutex_);
    for (auto &worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }
  static int sequence_;
  rclcpp::Context::SharedPtr ros_context_;
  rclcpp::Node::SharedPtr server_node_;
  rclcpp::CallbackGroup::SharedPtr server_group_;
  rclcpp_action::Server<Compute>::SharedPtr server_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr zero_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr spin_sub_;
  std::atomic<int> zero_spin_commands_{0};
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> server_executor_;
  std::thread server_thread_;
  std::vector<std::thread> threads_, workers_;
  std::mutex workers_mutex_;
  std::atomic<bool> stop_workers_{false}, release_other_{false};
  std::atomic<int> owned_cancels_{0}, other_cancels_{0}, zero_commands_{0};
  std::string topic_;
};
int ActionCancelTest::sequence_ = 0;

TEST_F(ActionCancelTest, PendingRequestCannotBeOverwritten) {
  auto owner = context("tunnel_pending_client");
  owner->planner.send(goal(3));
  const auto first_token = owner->planner.token;
  EXPECT_THROW(owner->planner.send(goal(2)), std::runtime_error);
  EXPECT_EQ(owner->planner.token, first_token);
  owner->planner.cancel();
  ASSERT_TRUE(waitUntil([&]() { return first_token->terminal.load(); }, 4s));
  EXPECT_EQ(owner->planner.poll(), -1);
  EXPECT_FALSE(owner->planner.active);
  EXPECT_EQ(owned_cancels_.load(), 1);
  owner->release.store(true);
}

TEST_F(ActionCancelTest,
       LateAcceptanceAfterOwnerDestructionIsPreciselyCanceled) {
  auto other = context("tunnel_other_client");
  other->planner.send(goal(2));
  ASSERT_TRUE(waitUntil(
      [&]() {
        return other->planner.accepted.wait_for(0s) ==
               std::future_status::ready;
      },
      3s));
  EXPECT_EQ(other->planner.poll(), 0);
  auto owner = context("tunnel_delayed_client");
  owner->planner.send(goal(1));
  const auto canceled_token = owner->planner.token;
  const auto canceled_result = owner->planner.result;
  std::this_thread::sleep_for(50ms);
  owner->planner.cancel();
  owner->watchCanceled();
  // Wait out the same bounded halt interval while the response is still
  // pending.
  std::this_thread::sleep_for(2s);
  EXPECT_EQ(owner->planner.accepted.wait_for(0s), std::future_status::timeout);
  std::weak_ptr<ActionContext> weak_owner = owner;
  owner->release.store(true);
  // Production destruction follows this exact shared-context worker path.
  threads_.back().detach();
  owner.reset();
  EXPECT_FALSE(weak_owner.expired());
  ASSERT_TRUE(waitUntil([&]() { return canceled_token->terminal.load(); }, 4s));
  EXPECT_TRUE(canceled_token->sent.load());
  // A CANCELED result is the authoritative terminal outcome. The separate
  // cancel-service acknowledgement may still be queued when the cleanup
  // context observes this result and safely releases its executor.
  ASSERT_EQ(canceled_result.wait_for(0s), std::future_status::ready);
  EXPECT_EQ(canceled_result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(owned_cancels_.load(), 1);
  EXPECT_EQ(other_cancels_.load(), 0);
  EXPECT_GT(zero_commands_.load(), 5);
  EXPECT_GT(zero_spin_commands_.load(), 5);
  EXPECT_TRUE(waitUntil([&]() { return weak_owner.expired(); }, 2s));
  EXPECT_FALSE(other->planner.token->requested.load());
  release_other_.store(true);
  ASSERT_TRUE(
      waitUntil([&]() { return other->planner.token->terminal.load(); }, 3s));
  EXPECT_EQ(other->planner.poll(), 1);
  other->release.store(true);
}

TEST_F(ActionCancelTest, FastSuccessBeforeFirstPollRetainsCompleteResult) {
  auto owner = context("tunnel_fast_success_client");
  owner->planner.send(goal(4));
  ASSERT_TRUE(
      waitUntil([&]() { return owner->planner.token->terminal.load(); }, 3s));
  EXPECT_EQ(owner->planner.handle, nullptr);
  EXPECT_EQ(owner->planner.result.get().code,
            rclcpp_action::ResultCode::SUCCEEDED);
  int status = 0;
  EXPECT_NO_THROW(status = owner->planner.poll());
  EXPECT_EQ(status, 1);
  EXPECT_FALSE(owner->planner.active);
  owner->release.store(true);
}

TEST_F(ActionCancelTest, LateCancelResultBeforeFirstPollRemainsReadable) {
  auto owner = context("tunnel_fast_cancel_client");
  owner->planner.send(goal(1));
  owner->planner.cancel();
  ASSERT_TRUE(
      waitUntil([&]() { return owner->planner.token->terminal.load(); }, 4s));
  EXPECT_EQ(owner->planner.handle, nullptr);
  EXPECT_EQ(owner->planner.result.get().code,
            rclcpp_action::ResultCode::CANCELED);
  int status = 0;
  EXPECT_NO_THROW(status = owner->planner.poll());
  EXPECT_EQ(status, -1);
  EXPECT_FALSE(owner->planner.active);
  owner->release.store(true);
}

TEST_F(ActionCancelTest, NewGenerationDoesNotInheritPriorCancellation) {
  auto owner = context("tunnel_generation_client");
  owner->planner.send(goal(3));
  const auto first = owner->planner.token;
  owner->planner.cancel();
  ASSERT_TRUE(waitUntil([&]() { return first->terminal.load(); }, 4s));
  ASSERT_EQ(owner->planner.poll(), -1);
  owner->planner.send(goal(2));
  EXPECT_NE(owner->planner.token, first);
  EXPECT_FALSE(owner->planner.token->requested.load());
  release_other_.store(true);
  ASSERT_TRUE(
      waitUntil([&]() { return owner->planner.token->terminal.load(); }, 3s));
  EXPECT_EQ(owner->planner.poll(), 1);
  EXPECT_EQ(other_cancels_.load(), 0);
  owner->release.store(true);
}

TEST_F(ActionCancelTest, PendingGoalCannotBePreempted) {
  auto owner = context("tunnel_pending_preempt_client");
  owner->planner.send(goal(1));
  const auto original = owner->planner.token;
  EXPECT_FALSE(owner->planner.preempt(goal(3)));
  EXPECT_EQ(owner->planner.token, original);
  EXPECT_TRUE(owner->planner.retired.empty());
  owner->planner.cancel();
  ASSERT_TRUE(waitUntil([&]() { return !owner->planner.busy(); }, 4s));
  EXPECT_EQ(owned_cancels_.load(), 1);
  owner->release.store(true);
}

TEST_F(ActionCancelTest, PreemptDoesNotCancelAndRetainsOlderLiveGeneration) {
  auto owner = context("tunnel_preempt_live_client");
  owner->planner.send(goal(3));
  ASSERT_TRUE(waitUntil([&]() {
    return owner->planner.poll() == 0 && owner->planner.handle != nullptr;
  }, 3s));
  const auto original = owner->planner.token;
  const auto original_handle = owner->planner.handle;
  const auto original_result = owner->planner.result;
  ASSERT_TRUE(owner->planner.preempt(goal(4)));
  ASSERT_EQ(owner->planner.retired.size(), 1u);
  EXPECT_EQ(owner->planner.retired.front().token, original);
  EXPECT_EQ(owner->planner.retired.front().handle, original_handle);
  EXPECT_FALSE(original->requested.load());
  ASSERT_TRUE(waitUntil([&]() { return owner->planner.token->terminal.load(); }, 3s));
  EXPECT_EQ(owner->planner.poll(), 1);
  EXPECT_FALSE(owner->planner.active);
  EXPECT_TRUE(owner->planner.busy());
  EXPECT_EQ(owned_cancels_.load(), 0);
  EXPECT_THROW(owner->planner.send(goal(2)), std::runtime_error);
  owner->planner.cancel();
  ASSERT_TRUE(waitUntil([&]() { return !owner->planner.busy(); }, 3s));
  EXPECT_TRUE(original->requested.load());
  ASSERT_EQ(original_result.wait_for(0s), std::future_status::ready);
  EXPECT_EQ(original_result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_TRUE(owner->planner.retired.empty());
  EXPECT_EQ(owned_cancels_.load(), 1);
  owner->release.store(true);
}

TEST_F(ActionCancelTest, RejectedReplacementStillKeepsPriorGoalCancelable) {
  auto owner = context("tunnel_rejected_preempt_client");
  owner->planner.send(goal(3));
  ASSERT_TRUE(waitUntil([&]() {
    return owner->planner.poll() == 0 && owner->planner.handle != nullptr;
  }, 3s));
  const auto original = owner->planner.token;
  ASSERT_TRUE(owner->planner.preempt(goal(5)));
  ASSERT_TRUE(waitUntil([&]() { return owner->planner.token->terminal.load(); }, 3s));
  EXPECT_EQ(owner->planner.poll(), -1);
  EXPECT_TRUE(owner->planner.busy());
  EXPECT_FALSE(original->requested.load());
  EXPECT_EQ(owned_cancels_.load(), 0);
  owner->planner.cancel();
  ASSERT_TRUE(waitUntil([&]() { return !owner->planner.busy(); }, 3s));
  EXPECT_TRUE(original->terminal.load());
  EXPECT_EQ(owned_cancels_.load(), 1);
  owner->release.store(true);
}

TEST_F(ActionCancelTest, CompletedCurrentGoalIsNotConsumedByPreemptAttempt) {
  auto owner = context("tunnel_terminal_preempt_client");
  owner->planner.send(goal(4));
  const auto completed = owner->planner.token;
  ASSERT_TRUE(waitUntil([&]() { return completed->terminal.load(); }, 3s));
  EXPECT_TRUE(owner->planner.active);
  EXPECT_FALSE(owner->planner.busy());
  EXPECT_TRUE(owner->planner.active);
  EXPECT_FALSE(owner->planner.preempt(goal(3)));
  EXPECT_EQ(owner->planner.token, completed);
  EXPECT_TRUE(owner->planner.active);
  EXPECT_TRUE(owner->planner.retired.empty());
  EXPECT_EQ(owner->planner.poll(), 1);
  EXPECT_FALSE(owner->planner.busy());
  EXPECT_EQ(owned_cancels_.load(), 0);
  owner->release.store(true);
}

TEST_F(ActionCancelTest, MultiplePreemptionsRetainEachGenerationOnce) {
  auto owner = context("tunnel_multiple_preempt_client");
  owner->planner.send(goal(3));
  ASSERT_TRUE(waitUntil([&]() {
    return owner->planner.poll() == 0 && owner->planner.handle != nullptr;
  }, 3s));
  const auto first = owner->planner.token;
  ASSERT_TRUE(owner->planner.preempt(goal(6)));
  ASSERT_TRUE(waitUntil([&]() {
    return owner->planner.poll() == 0 && owner->planner.handle != nullptr;
  }, 3s));
  const auto second = owner->planner.token;
  ASSERT_TRUE(owner->planner.preempt(goal(7)));
  const auto third = owner->planner.token;
  ASSERT_EQ(owner->planner.retired.size(), 2u);
  EXPECT_EQ(owner->planner.retired[0].token, first);
  EXPECT_EQ(owner->planner.retired[1].token, second);
  EXPECT_EQ(owner->planner.pendingTokens().size(), 3u);
  EXPECT_FALSE(first->requested.load());
  EXPECT_FALSE(second->requested.load());
  owner->planner.cancel();
  ASSERT_TRUE(waitUntil([&]() { return !owner->planner.busy(); }, 4s));
  EXPECT_TRUE(first->terminal.load());
  EXPECT_TRUE(second->terminal.load());
  EXPECT_TRUE(third->terminal.load());
  EXPECT_TRUE(owner->planner.retired.empty());
  EXPECT_EQ(owned_cancels_.load(), 3);
  owner->release.store(true);
}

TEST_F(ActionCancelTest, CleanupRetainsRetiredGoalAndLateAcceptedReplacement) {
  auto owner = context("tunnel_retired_cleanup_client");
  owner->planner.send(goal(3));
  ASSERT_TRUE(waitUntil([&]() {
    return owner->planner.poll() == 0 && owner->planner.handle != nullptr;
  }, 3s));
  const auto original = owner->planner.token;
  const auto original_result = owner->planner.result;
  ASSERT_TRUE(owner->planner.preempt(goal(1)));
  const auto replacement = owner->planner.token;
  const auto replacement_result = owner->planner.result;
  owner->planner.cancel();
  owner->watchCanceled();
  std::weak_ptr<ActionContext> weak_owner = owner;
  owner->release.store(true);
  threads_.back().detach();
  owner.reset();
  ASSERT_TRUE(waitUntil([&]() {
    return original->terminal.load() && replacement->terminal.load();
  }, 5s));
  EXPECT_TRUE(original->requested.load());
  EXPECT_TRUE(replacement->requested.load());
  ASSERT_EQ(original_result.wait_for(0s), std::future_status::ready);
  ASSERT_EQ(replacement_result.wait_for(0s), std::future_status::ready);
  EXPECT_EQ(original_result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(replacement_result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(owned_cancels_.load(), 2);
  EXPECT_EQ(other_cancels_.load(), 0);
  EXPECT_TRUE(waitUntil([&]() { return weak_owner.expired(); }, 2s));
}
} // namespace
} // namespace sentry_tunnel_bt
