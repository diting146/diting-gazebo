// Copyright 2026 diting
// SPDX-License-Identifier: Apache-2.0
#ifndef SENTRY_TUNNEL_BT__ACTION_CONTEXT_HPP_
#define SENTRY_TUNNEL_BT__ACTION_CONTEXT_HPP_
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
namespace sentry_tunnel_bt {
using Compute = nav2_msgs::action::ComputePathToPose;
using Follow = nav2_msgs::action::FollowPath;
using namespace std::chrono_literals;
struct CancellationToken {
  std::atomic<bool> requested{false}, sent{false}, terminal{false};
  std::atomic<int> response{-1};
};

template <typename Action> struct Request {
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  typename rclcpp_action::Client<Action>::SharedPtr client;
  std::shared_future<typename Handle::SharedPtr> accepted;
  std::shared_future<typename Handle::WrappedResult> result;
  typename Handle::SharedPtr handle;
  std::shared_ptr<CancellationToken> token;
  bool active{false};
  // Preemption replaces the current command without canceling it. Retain each
  // older generation until the server reports a real terminal outcome; it may
  // still own motion while the new command is awaiting server processing.
  // This deliberately stores no Request, so retirement cannot recursively
  // duplicate an already retired generation list.
  struct RetiredRequest {
    std::shared_future<typename Handle::SharedPtr> accepted;
    std::shared_future<typename Handle::WrappedResult> result;
    typename Handle::SharedPtr handle;
    std::shared_ptr<CancellationToken> token;
  };
  std::vector<RetiredRequest> retired;
  static void
  cancelHandle(const typename rclcpp_action::Client<Action>::SharedPtr &client,
               const typename Handle::SharedPtr &handle,
               const std::shared_ptr<CancellationToken> &state) {
    if (!state->requested.load() || state->terminal.load() ||
        state->sent.exchange(true)) {
      return;
    }
    try {
      client->async_cancel_goal(
          handle,
          [state](
              typename rclcpp_action::Client<Action>::CancelResponse::SharedPtr
                  reply) {
            state->response.store(reply->return_code);
            // ERROR_NONE only acknowledges CANCELING, never terminal
            // completion.
          });
    } catch (const std::exception &) {
      state->response.store(-2);
    }
  }
  void send(const typename Action::Goal &goal) {
    if (busy()) {
      throw std::runtime_error("Cannot overwrite an active tunnel action");
    }
    begin(goal);
  }
  bool preempt(const typename Action::Goal &goal) {
    pollRetired();
    // Do not poll the current result here: a goal can finish between the
    // caller's poll and this update. Preserve active/result so the next normal
    // poll still reports that terminal outcome instead of a false failure.
    if (!active || !token || token->requested.load() || token->terminal.load()) {
      return false;
    }
    if (!handle && accepted.valid() &&
        accepted.wait_for(0s) == std::future_status::ready) {
      handle = accepted.get();
    }
    if (!handle || token->requested.load() || token->terminal.load() ||
        (result.valid() && result.wait_for(0s) == std::future_status::ready)) {
      return false;
    }
    retired.push_back({accepted, result, handle, token});
    active = false;
    // begin() never cancels the retired goal. Nav2's ordinary pending-goal
    // dispatch performs the switch in its existing control loop.
    begin(goal);
    return true;
  }
  bool busy() {
    // Ownership queries must not consume a terminal result that the state
    // machine has not read yet. The callback token is the terminal authority.
    pollRetired();
    return (active && token && !token->terminal.load()) || !retired.empty();
  }
  std::vector<std::shared_ptr<CancellationToken>> pendingTokens() const {
    std::vector<std::shared_ptr<CancellationToken>> result_tokens;
    if (active && token && !token->terminal.load()) {
      result_tokens.push_back(token);
    }
    for (const auto &previous : retired) {
      if (previous.token && !previous.token->terminal.load()) {
        result_tokens.push_back(previous.token);
      }
    }
    return result_tokens;
  }
private:
  void begin(const typename Action::Goal &goal) {
    token = std::make_shared<CancellationToken>();
    const auto state = token;
    std::weak_ptr<rclcpp_action::Client<Action>> weak_client = client;
    auto completion =
        std::make_shared<std::promise<typename Handle::WrappedResult>>();
    result = completion->get_future().share();
    typename rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback =
        [state, weak_client](typename Handle::SharedPtr accepted_handle) {
          if (!accepted_handle) {
            state->terminal.store(true);
            return;
          }
          if (auto client = weak_client.lock()) {
            cancelHandle(client, accepted_handle, state);
          }
        };
    options.result_callback =
        [state, completion](const typename Handle::WrappedResult &completed) {
          // Humble erases terminal UUIDs from Client::goal_handles_. Preserve
          // the complete result here; a later BT tick must not ask the client
          // again.
          completion->set_value(completed);
          state->terminal.store(true);
        };
    handle.reset();
    active = true;
    try {
      accepted = client->async_send_goal(goal, options);
    } catch (...) {
      active = false;
      state->terminal.store(true);
      throw;
    }
  }
public:
  // 0 pending, 1 success, -1 rejected/aborted/canceled. Only the BT tick thread
  // accesses these members; action callbacks access the per-request atomics
  // only.
  int poll() {
    pollRetired();
    if (!active) {
      return -1;
    }
    if (!handle && accepted.valid() &&
        accepted.wait_for(0s) == std::future_status::ready) {
      handle = accepted.get();
      if (!handle) {
        active = false;
        return -1;
      }
    }
    if (result.valid() && result.wait_for(0s) == std::future_status::ready) {
      active = false;
      return result.get().code == rclcpp_action::ResultCode::SUCCEEDED ? 1 : -1;
    }
    return 0;
  }
  void cancel() {
    if (token) {
      token->requested.store(true);
    }
    poll();
    if (active && handle) {
      cancelHandle(client, handle, token);
    }
    for (auto &previous : retired) {
      previous.token->requested.store(true);
      if (previous.handle) {
        cancelHandle(client, previous.handle, previous.token);
      }
      // A goal response arriving after this call runs cancelHandle from its
      // own response callback, using this generation's independent token.
    }
    pollRetired();
  }
  void pollRetired() {
    for (auto &previous : retired) {
      if (!previous.handle && previous.accepted.valid() &&
          previous.accepted.wait_for(0s) == std::future_status::ready) {
        previous.handle = previous.accepted.get();
      }
      if (previous.handle) {
        cancelHandle(client, previous.handle, previous.token);
      }
    }
    retired.erase(
        std::remove_if(retired.begin(), retired.end(), [](const auto &previous) {
          return previous.token->terminal.load();
        }),
        retired.end());
  }
};

// The cleanup context never captures a TunnelPassage pointer. If a goal server
// accepts after BT halt/destruction, its response callback still precisely
// cancels that request. Keep publishing zero until its real result is terminal.
struct ActionContext : public std::enable_shared_from_this<ActionContext> {
  explicit ActionContext(rclcpp::Node::SharedPtr owner)
      : node(std::move(owner)), executor(executorOptions(node)) {}
  static rclcpp::ExecutorOptions
  executorOptions(const rclcpp::Node::SharedPtr &node) {
    rclcpp::ExecutorOptions options;
    options.context = node->get_node_base_interface()->get_context();
    return options;
  }
  std::thread start() {
    return std::thread([context = shared_from_this()]() {
      while (
          rclcpp::ok(context->node->get_node_base_interface()->get_context())) {
        try {
          if (!context->executor_failed.load()) {
            context->executor.spin_some(1ms);
          }
          const bool cleared = context->cleanupTick();
          if (context->release.load() && cleared) {
            break;
          }
        } catch (const std::exception &error) {
          context->executor_failed.store(true);
          RCLCPP_ERROR(context->node->get_logger(),
                       "Tunnel action executor failure: %s", error.what());
        }
        std::this_thread::sleep_for(10ms);
      }
    });
  }
  rclcpp::Node::SharedPtr node;
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::executors::SingleThreadedExecutor executor;
  Request<Compute> planner;
  Request<Follow> follower;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_pub;
  rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr spin_pub;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr limit_pub;
  std::mutex cleanup_mutex;
  std::vector<std::shared_ptr<CancellationToken>> watched;
  std::atomic<bool> release{false}, executor_failed{false};
  void watchCanceled() {
    std::lock_guard<std::mutex> lock(cleanup_mutex);
    watched.clear();
    const auto planner_tokens = planner.pendingTokens();
    const auto follower_tokens = follower.pendingTokens();
    watched.insert(watched.end(), planner_tokens.begin(), planner_tokens.end());
    watched.insert(watched.end(), follower_tokens.begin(), follower_tokens.end());
  }
  void disarmCleanup() {
    std::lock_guard<std::mutex> lock(cleanup_mutex);
    watched.clear();
  }
  bool cleanupTick() {
    std::lock_guard<std::mutex> lock(cleanup_mutex);
    if (watched.empty()) {
      return true;
    }
    if (spin_pub) {
      spin_pub->publish(example_interfaces::msg::Float32());
    }
    stop_pub->publish(geometry_msgs::msg::Twist());
    const bool done =
        std::all_of(watched.begin(), watched.end(),
                    [](const auto &state) { return state->terminal.load(); });
    if (done) {
      nav2_msgs::msg::SpeedLimit limit;
      limit.header.stamp = node->now();
      limit.percentage = false;
      limit.speed_limit = 0.0;
      limit_pub->publish(limit);
      watched.clear();
    }
    return done;
  }
};

} // namespace sentry_tunnel_bt
#endif // SENTRY_TUNNEL_BT__ACTION_CONTEXT_HPP_
